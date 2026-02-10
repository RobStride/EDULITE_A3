#!/usr/bin/env python3
"""
主从遥操作模式测试脚本（含7号电机夹爪）
测试 Menu 键切换主从模式功能（独立于 MoveIt）

功能：
- 订阅 joy 话题监听 Menu 键
- Menu 键按下：两臂回零 -> can0 开启重力补偿(含7号电机) -> can1 跟随(含7号电机)
- 再按 Menu 键：关闭重力补偿 -> 两臂回零 -> 退出

7号电机（夹爪）通过直接 CAN 通信控制，不走 ros2_control
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import SetBool
from builtin_interfaces.msg import Duration
import socket
import struct
import fcntl
import os


class MasterSlaveTestNode(Node):
    def __init__(self):
        super().__init__('master_slave_test')
        
        # 参数
        self.go_zero_duration = 3.0
        self.joint_names = [f"L{i}_joint" for i in range(1, 7)]
        self.gripper_motor_id = 7
        
        # EL05 电机参数范围（7号电机）
        self.P_MIN, self.P_MAX = -12.57, 12.57
        self.V_MIN, self.V_MAX = -50.0, 50.0
        self.KP_MIN, self.KP_MAX = 0.0, 500.0
        self.KD_MIN, self.KD_MAX = 0.0, 5.0
        self.T_MIN, self.T_MAX = -6.0, 6.0
        
        # 状态
        self.master_slave_mode = False
        self.entering_master_slave = False
        self.last_menu_button = 0
        self.motor7_position = 0.0  # can0 motor7 当前位置
        self.motor7_valid = False
        
        # 主臂 (arm1/can0) ROS2 接口
        self.master_joint_state = None
        self.master_joint_sub = self.create_subscription(
            JointState, '/joint_states',
            self.master_joint_state_cb, 10)
        self.master_trajectory_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.master_zero_torque_client = self.create_client(
            SetBool, '/set_zero_torque_mode')
        
        # 从臂 (arm2/can1) ROS2 接口
        self.slave_joint_state = None
        self.slave_joint_sub = self.create_subscription(
            JointState, '/arm2/joint_states',
            self.slave_joint_state_cb, 10)
        self.slave_trajectory_pub = self.create_publisher(
            JointTrajectory, '/arm2/arm2_arm_controller/joint_trajectory', 10)
        
        # 手柄输入
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        
        # CAN sockets for motor 7
        self.can0_socket = None
        self.can1_socket = None
        try:
            self.can0_socket = self._create_can_socket('can0')
            self.get_logger().info('CAN0 socket 已创建（motor 7 主臂）')
        except Exception as e:
            self.get_logger().error(f'CAN0 socket 创建失败: {e}')
        
        try:
            self.can1_socket = self._create_can_socket('can1')
            self.get_logger().info('CAN1 socket 已创建（motor 7 从臂）')
        except Exception as e:
            self.get_logger().error(f'CAN1 socket 创建失败: {e}')
        
        # 使能两侧 motor 7
        if self.can0_socket:
            self._enable_motor(self.can0_socket, self.gripper_motor_id)
            self.get_logger().info('CAN0 motor 7 已使能')
        if self.can1_socket:
            self._enable_motor(self.can1_socket, self.gripper_motor_id)
            self.get_logger().info('CAN1 motor 7 已使能')
        
        # 50Hz 跟随定时器
        self.timer = self.create_timer(0.02, self.update)
        
        self.get_logger().info('=== 主从遥操作测试节点已启动（含7号电机） ===')
        self.get_logger().info('按 Menu 键 (buttons[7]) 切换主从模式')
        self.get_logger().info('  开启: 两臂回零 -> can0 重力补偿(含L7) -> can1 跟随(含L7)')
        self.get_logger().info('  关闭: 关闭重力补偿 -> 两臂回零')
    
    def _create_can_socket(self, interface):
        """创建 CAN RAW socket 并设置为非阻塞"""
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        s.bind((interface,))
        # 设置非阻塞模式，用于读取反馈
        s.setblocking(False)
        return s
    
    def _float_to_uint16(self, x, x_min, x_max):
        """浮点数转 uint16（MIT 模式编码）"""
        x = max(x_min, min(x_max, x))
        return int((x - x_min) * 65535.0 / (x_max - x_min))
    
    def _uint16_to_float(self, x_int, x_min, x_max):
        """uint16 转浮点数（MIT 模式解码）"""
        return x_min + (x_max - x_min) * x_int / 65535.0
    
    def _enable_motor(self, can_socket, motor_id):
        """使能电机（通信类型3）"""
        host_can_id = 253
        comm_type = 3
        can_id = (comm_type << 24) | (host_can_id << 8) | motor_id
        can_id |= 0x80000000  # 扩展帧
        data = bytes(8)
        frame = struct.pack('=IB3x8s', can_id, 8, data)
        can_socket.send(frame)
    
    def _send_mit_command(self, can_socket, motor_id, position, velocity, kp, kd, torque):
        """发送 MIT 运控模式命令（通信类型1）"""
        pos_raw = self._float_to_uint16(position, self.P_MIN, self.P_MAX)
        vel_raw = self._float_to_uint16(velocity, self.V_MIN, self.V_MAX)
        kp_raw = self._float_to_uint16(kp, self.KP_MIN, self.KP_MAX)
        kd_raw = self._float_to_uint16(kd, self.KD_MIN, self.KD_MAX)
        torque_raw = self._float_to_uint16(torque, self.T_MIN, self.T_MAX)
        
        comm_type = 1
        can_id = (comm_type << 24) | (torque_raw << 8) | motor_id
        can_id |= 0x80000000  # 扩展帧
        
        data = struct.pack('>HHHH', pos_raw, vel_raw, kp_raw, kd_raw)
        frame = struct.pack('=IB3x8s', can_id, 8, data)
        can_socket.send(frame)
    
    def _read_motor7_feedback(self):
        """从 CAN0 非阻塞读取 motor 7 反馈，更新 self.motor7_position"""
        if self.can0_socket is None:
            return
        
        # 读取所有可用的 CAN 帧（非阻塞）
        for _ in range(50):  # 最多读 50 帧避免阻塞
            try:
                frame_data = self.can0_socket.recv(16)
                can_id_raw, dlc = struct.unpack_from('=IB', frame_data, 0)
                data = frame_data[8:16]
                
                can_id = can_id_raw & 0x1FFFFFFF  # 去掉扩展帧标志
                comm_type = (can_id >> 24) & 0x3F
                motor_id = (can_id >> 8) & 0xFF
                
                if comm_type == 2 and motor_id == self.gripper_motor_id:
                    # 解析位置
                    pos_raw = (data[0] << 8) | data[1]
                    self.motor7_position = self._uint16_to_float(
                        pos_raw, self.P_MIN, self.P_MAX)
                    self.motor7_valid = True
            except BlockingIOError:
                break  # 没有更多数据
            except Exception:
                break
    
    def master_joint_state_cb(self, msg):
        self.master_joint_state = msg
    
    def slave_joint_state_cb(self, msg):
        self.slave_joint_state = msg
    
    def joy_cb(self, msg):
        if len(msg.buttons) > 7:
            menu = msg.buttons[7]
            if menu == 1 and self.last_menu_button == 0:
                self.toggle_master_slave()
            self.last_menu_button = menu
    
    def update(self):
        """50Hz 定时回调"""
        if not self.master_slave_mode:
            return
        
        # 读取 motor 7 反馈
        self._read_motor7_feedback()
        
        # L1-L6 跟随
        if self.master_joint_state is not None:
            positions = self._extract_positions(self.master_joint_state)
            if positions is not None:
                traj = JointTrajectory()
                traj.joint_names = self.joint_names
                pt = JointTrajectoryPoint()
                pt.positions = positions
                pt.velocities = [0.0] * 6
                pt.time_from_start = Duration(sec=0, nanosec=20_000_000)
                traj.points = [pt]
                self.slave_trajectory_pub.publish(traj)
        
        # Motor 7 零力矩保持（can0）+ 跟随（can1）
        if self.can0_socket:
            # can0 motor 7: 纯零力矩（Kp=0, Kd=0.3 阻尼, torque=0）
            self._send_mit_command(
                self.can0_socket, self.gripper_motor_id,
                0.0, 0.0, 0.0, 0.3, 0.0)
        
        if self.can1_socket and self.motor7_valid:
            # can1 motor 7: 位置跟随（Kp=30, Kd=1.0）
            self._send_mit_command(
                self.can1_socket, self.gripper_motor_id,
                self.motor7_position, 0.0, 30.0, 1.0, 0.0)
    
    def toggle_master_slave(self):
        if self.entering_master_slave:
            self.get_logger().warn('正在切换中，请稍候')
            return
        
        if not self.master_slave_mode:
            self._enable_master_slave()
        else:
            self._disable_master_slave()
    
    def _enable_master_slave(self):
        self.get_logger().info('>>> 开启主从模式（含 motor 7） <<<')
        self.entering_master_slave = True
        
        if self.master_joint_state is None or self.slave_joint_state is None:
            self.get_logger().error('主臂或从臂关节状态不可用')
            self.entering_master_slave = False
            return
        
        # 两臂回零 (L1-L6)
        self._move_both_to_zero()
        
        # 延迟后开启重力补偿
        self._enable_timer = self.create_timer(
            self.go_zero_duration + 0.5, self._on_zero_done_enable)
    
    def _on_zero_done_enable(self):
        self._enable_timer.cancel()
        self.destroy_timer(self._enable_timer)
        
        # 开启主臂 L1-L6 重力补偿
        self._set_zero_torque(True)
        
        # Motor 7 使能（两侧）
        if self.can0_socket:
            self._enable_motor(self.can0_socket, self.gripper_motor_id)
        if self.can1_socket:
            self._enable_motor(self.can1_socket, self.gripper_motor_id)
        
        self.motor7_valid = False
        self.master_slave_mode = True
        self.entering_master_slave = False
        self.get_logger().info('>>> 主从模式已开启 <<<')
        self.get_logger().info('  can0 L1-L6 + L7: 零力矩 - 可手动拖动')
        self.get_logger().info('  can1 L1-L6 + L7: 实时跟随')
    
    def _disable_master_slave(self):
        self.get_logger().info('>>> 关闭主从模式 <<<')
        self.master_slave_mode = False
        self.entering_master_slave = True
        
        # 停止 motor 7 跟随（发送零力矩到 can1 motor 7）
        if self.can1_socket:
            self._send_mit_command(
                self.can1_socket, self.gripper_motor_id,
                0.0, 0.0, 0.0, 0.0, 0.0)
        
        # 关闭 L1-L6 重力补偿
        self._set_zero_torque(False)
        
        # 延迟后回零
        self._pre_disable_timer = self.create_timer(
            0.5, self._on_gravity_off_go_zero)
    
    def _on_gravity_off_go_zero(self):
        self._pre_disable_timer.cancel()
        self.destroy_timer(self._pre_disable_timer)
        
        self._move_both_to_zero()
        
        self._disable_timer = self.create_timer(
            self.go_zero_duration + 0.5, self._on_zero_done_disable)
    
    def _on_zero_done_disable(self):
        self._disable_timer.cancel()
        self.destroy_timer(self._disable_timer)
        self.entering_master_slave = False
        self.get_logger().info('>>> 主从模式已关闭 <<<')
    
    def _set_zero_torque(self, enable):
        if not self.master_zero_torque_client.service_is_ready():
            self.get_logger().warn('主臂零力矩服务不可用')
            return
        req = SetBool.Request()
        req.data = enable
        future = self.master_zero_torque_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'主臂 L1-L6 重力补偿已{"开启" if enable else "关闭"}'))
    
    def _move_both_to_zero(self):
        dur = self.go_zero_duration
        
        if self.master_joint_state is not None:
            pos = self._extract_positions(self.master_joint_state)
            if pos:
                traj = self._gen_zero_traj(pos, dur)
                self.master_trajectory_pub.publish(traj)
                self.get_logger().info(f'主臂 L1-L6 回零轨迹已发送 ({dur}s)')
        
        if self.slave_joint_state is not None:
            pos = self._extract_positions(self.slave_joint_state)
            if pos:
                traj = self._gen_zero_traj(pos, dur)
                self.slave_trajectory_pub.publish(traj)
                self.get_logger().info(f'从臂 L1-L6 回零轨迹已发送 ({dur}s)')
    
    def _extract_positions(self, js):
        positions = []
        for name in self.joint_names:
            if name in js.name:
                idx = js.name.index(name)
                positions.append(js.position[idx])
            else:
                return None
        return positions
    
    def _gen_zero_traj(self, current, duration=3.0, n=10):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        for i in range(1, n + 1):
            alpha = i / n
            pt = JointTrajectoryPoint()
            pt.positions = [c * (1.0 - alpha) for c in current]
            pt.velocities = [0.0] * 6
            t = duration * alpha
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            traj.points.append(pt)
        return traj


def main():
    rclpy.init()
    node = MasterSlaveTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 关闭 CAN sockets
        if node.can0_socket:
            node.can0_socket.close()
        if node.can1_socket:
            node.can1_socket.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
