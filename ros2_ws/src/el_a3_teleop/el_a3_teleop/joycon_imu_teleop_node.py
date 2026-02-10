#!/usr/bin/env python3
"""
Joy-Con IMU 遥控节点

使用 Nintendo Switch Joy-Con 手柄的 IMU 数据控制机械臂末端移动

控制映射：
- Pitch (前后倾斜): X 方向平移
- Roll (左右倾斜): Y 方向平移
- Yaw (水平旋转): Z 轴旋转
- ZL/ZR: Z 方向平移
- A/B/X/Y: 功能按键
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy, Imu, JointState
from geometry_msgs.msg import PoseStamped, Pose, Vector3
from std_msgs.msg import String, Float32
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

from .imu_processor import IMUProcessor, CalibrationState


def quaternion_from_euler(roll, pitch, yaw):
    """将欧拉角转换为四元数 (x, y, z, w)"""
    rot = R.from_euler('xyz', [roll, pitch, yaw])
    quat = rot.as_quat()  # 返回 (x, y, z, w)
    return quat


def euler_from_quaternion(quaternion):
    """将四元数 (x, y, z, w) 转换为欧拉角"""
    rot = R.from_quat(quaternion)
    euler = rot.as_euler('xyz')
    return euler[0], euler[1], euler[2]  # roll, pitch, yaw


class JoyConIMUTeleopNode(Node):
    """Joy-Con IMU 遥控节点"""
    
    def __init__(self):
        super().__init__('joycon_imu_teleop_node')
        
        # ==================== 声明参数 ====================
        # IMU 参数
        self.declare_parameter('sample_rate', 60.0)
        self.declare_parameter('calibration_samples', 200)
        self.declare_parameter('calibration_timeout', 5.0)
        
        # 滤波参数
        self.declare_parameter('filter_type', 'madgwick')
        self.declare_parameter('madgwick_beta', 0.1)
        self.declare_parameter('lowpass_alpha_gyro', 0.8)
        self.declare_parameter('lowpass_alpha_accel', 0.5)
        
        # 控制参数
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('translation_scale', 0.002)
        self.declare_parameter('rotation_scale', 0.02)
        self.declare_parameter('z_scale', 0.003)
        self.declare_parameter('deadzone', 0.05)  # rad, 约 3°
        
        # 运动学参数
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('end_effector_frame', 'end_effector')
        
        # 调试参数
        self.declare_parameter('debug_mode', False)
        
        # ==================== 获取参数 ====================
        self.sample_rate = self.get_parameter('sample_rate').value
        self.calibration_samples = self.get_parameter('calibration_samples').value
        self.calibration_timeout = self.get_parameter('calibration_timeout').value
        self.filter_type = self.get_parameter('filter_type').value
        self.madgwick_beta = self.get_parameter('madgwick_beta').value
        self.lowpass_alpha_gyro = self.get_parameter('lowpass_alpha_gyro').value
        self.lowpass_alpha_accel = self.get_parameter('lowpass_alpha_accel').value
        self.update_rate = self.get_parameter('update_rate').value
        self.translation_scale = self.get_parameter('translation_scale').value
        self.rotation_scale = self.get_parameter('rotation_scale').value
        self.z_scale = self.get_parameter('z_scale').value
        self.deadzone = self.get_parameter('deadzone').value
        self.planning_group = self.get_parameter('planning_group').value
        self.base_frame = self.get_parameter('base_frame').value
        self.end_effector_frame = self.get_parameter('end_effector_frame').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # ==================== 速度档位 ====================
        self.speed_levels = [
            {'name': '精细', 'translation': 0.0005, 'rotation': 0.005, 'z': 0.0008},
            {'name': '标准', 'translation': 0.002, 'rotation': 0.02, 'z': 0.003},
            {'name': '快速', 'translation': 0.005, 'rotation': 0.05, 'z': 0.008},
        ]
        self.current_speed_level = 1  # 默认标准
        self._apply_speed_level()
        
        # ==================== IMU 处理器 ====================
        self.imu_processor = IMUProcessor(
            sample_rate=self.sample_rate,
            calibration_samples=self.calibration_samples,
            filter_type=self.filter_type,
            madgwick_beta=self.madgwick_beta,
            lowpass_alpha_gyro=self.lowpass_alpha_gyro,
            lowpass_alpha_accel=self.lowpass_alpha_accel
        )
        
        # ==================== 状态变量 ====================
        self.current_joy = None
        self.current_imu = None
        self.target_pose = None
        self.current_joint_state = None
        self.last_ik_joint_positions = None
        self.pending_ik_request = False
        self.pose_initialized = False
        self.is_going_home = False
        self.control_enabled = False  # 标定完成后才启用控制
        
        # 按键状态追踪
        self.last_buttons = {}
        
        # 关节名称
        self.joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']
        
        # ==================== 订阅器 ====================
        # Joy-Con 按键
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Joy-Con IMU (通过 joy 节点或自定义驱动发布)
        # 尝试订阅标准 IMU topic
        self.imu_sub = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10
        )
        
        # 关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # ==================== TF2 ====================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # ==================== 发布器 ====================
        # 目标位姿（用于可视化）
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            '/target_pose',
            10
        )
        
        # 关节轨迹
        trajectory_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            trajectory_qos
        )
        
        # 标定状态
        self.calibration_status_pub = self.create_publisher(
            String,
            '/joycon/calibration_status',
            10
        )
        
        # IMU 姿态（调试用）
        self.imu_euler_pub = self.create_publisher(
            Vector3,
            '/joycon/imu_euler',
            10
        )
        
        # ==================== 服务客户端 ====================
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )
        
        # ==================== Action 客户端 ====================
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )
        
        # ==================== 等待服务 ====================
        self.get_logger().info('等待 IK 服务...')
        self.ik_client.wait_for_service()
        self.get_logger().info('等待 MoveGroup 服务...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('服务已连接')
        
        # ==================== 等待关节状态 ====================
        self.get_logger().info('等待关节状态...')
        self._wait_for_joint_states()
        
        # ==================== 启动时回 home ====================
        self.get_logger().info('正在移动到 home 位置...')
        self._startup_go_home()
        
        # ==================== 创建控制定时器 ====================
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_callback)
        
        # ==================== 打印启动信息 ====================
        self._print_startup_info()
        
        # ==================== 自动开始标定 ====================
        self.get_logger().info('='*50)
        self.get_logger().info('请将 Joy-Con 放置在平稳表面上，准备开始 IMU 标定...')
        self.get_logger().info('3 秒后开始标定，请保持设备静止')
        self.get_logger().info('='*50)
        
        # 延迟开始标定
        self.calibration_timer = self.create_timer(3.0, self._start_calibration_once)
    
    def _start_calibration_once(self):
        """一次性启动标定"""
        self.calibration_timer.cancel()
        self.start_calibration()
    
    def _apply_speed_level(self):
        """应用当前速度档位"""
        level = self.speed_levels[self.current_speed_level]
        self.translation_scale = level['translation']
        self.rotation_scale = level['rotation']
        self.z_scale = level['z']
    
    def _print_startup_info(self):
        """打印启动信息"""
        self.get_logger().info('='*50)
        self.get_logger().info('Joy-Con IMU 遥控节点已启动')
        self.get_logger().info('='*50)
        self.get_logger().info(f'控制组: {self.planning_group}')
        self.get_logger().info(f'末端执行器: {self.end_effector_frame}')
        self.get_logger().info(f'更新频率: {self.update_rate} Hz')
        self.get_logger().info(f'滤波器类型: {self.filter_type}')
        self.get_logger().info('')
        self.get_logger().info('=== 控制说明 ===')
        self.get_logger().info('Pitch (前后倾斜): X 方向平移')
        self.get_logger().info('Roll (左右倾斜): Y 方向平移')
        self.get_logger().info('Yaw (水平旋转): 末端旋转')
        self.get_logger().info('ZL/ZR: Z 方向上下移动')
        self.get_logger().info('')
        self.get_logger().info('=== 按键功能 ===')
        self.get_logger().info('A: 切换速度档位')
        self.get_logger().info('B: 回到 home 位置')
        self.get_logger().info('X: 重新标定 IMU')
        self.get_logger().info('Y: 设置当前姿态为零点')
        self.get_logger().info('-: 减号键也可重新标定')
        self.get_logger().info('')
        self._log_speed_level()
    
    def _log_speed_level(self):
        """打印当前速度档位"""
        level = self.speed_levels[self.current_speed_level]
        self.get_logger().info(
            f'当前速度档位: {self.current_speed_level + 1}/{len(self.speed_levels)} '
            f'[{level["name"]}]'
        )
    
    def _wait_for_joint_states(self, timeout_sec=10.0):
        """等待关节状态"""
        start_time = time.time()
        while self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout_sec:
                self.get_logger().warn('等待关节状态超时')
                return False
        return True
    
    def _startup_go_home(self):
        """启动时移动到 home 位置"""
        self.is_going_home = True
        
        # 等待 MoveGroup 准备就绪
        time.sleep(2.0)
        
        # home 位置
        joint_values = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.15
        goal_msg.request.max_acceleration_scaling_factor = 0.15
        
        constraints = Constraints()
        for name, value in zip(self.joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints = [constraints]
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
        
        goal_handle = send_goal_future.result()
        if goal_handle and goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
            self.get_logger().info('✓ 已移动到 home 位置')
        else:
            self.get_logger().warn('移动到 home 失败')
        
        self.is_going_home = False
        self.pose_initialized = False
    
    # ==================== 回调函数 ====================
    
    def joy_callback(self, msg: Joy):
        """Joy-Con 按键回调"""
        self.current_joy = msg
        
        # 处理按键事件
        self._process_buttons(msg)
    
    def imu_callback(self, msg: Imu):
        """IMU 数据回调"""
        self.current_imu = msg
        
        # 提取 IMU 数据
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # 处理 IMU 数据
        roll, pitch, yaw = self.imu_processor.process(accel, gyro)
        
        # 发布欧拉角（调试用）
        if self.debug_mode:
            euler_msg = Vector3()
            euler_msg.x = roll
            euler_msg.y = pitch
            euler_msg.z = yaw
            self.imu_euler_pub.publish(euler_msg)
        
        # 检查标定状态
        if self.imu_processor.calibration_state == CalibrationState.IN_PROGRESS:
            progress = self.imu_processor.calibration_progress
            if int(progress * 100) % 20 == 0:  # 每 20% 打印一次
                self.get_logger().info(f'标定进度: {progress*100:.0f}%')
        
        elif self.imu_processor.calibration_state == CalibrationState.COMPLETED:
            if not self.control_enabled:
                self.control_enabled = True
                self.get_logger().info('='*50)
                self.get_logger().info('✓ IMU 标定完成！控制已启用')
                self.get_logger().info('移动 Joy-Con 来控制机械臂')
                self.get_logger().info('='*50)
                
                # 设置当前姿态为零点
                self.imu_processor.set_zero_reference()
                
                # 发布状态
                status_msg = String()
                status_msg.data = "calibrated"
                self.calibration_status_pub.publish(status_msg)
        
        elif self.imu_processor.calibration_state == CalibrationState.FAILED:
            self.get_logger().warn('标定失败，请重新标定')
            status_msg = String()
            status_msg.data = "failed"
            self.calibration_status_pub.publish(status_msg)
    
    def joint_state_callback(self, msg: JointState):
        """关节状态回调"""
        self.current_joint_state = msg
    
    def _process_buttons(self, msg: Joy):
        """处理按键事件"""
        if len(msg.buttons) < 4:
            return
        
        # Joy-Con 按键映射 (可能需要根据实际情况调整)
        # 通常: A=0, B=1, X=2, Y=3, L=4, R=5, ZL=6, ZR=7, -=8, +=9, Home=10
        
        buttons = {
            'a': msg.buttons[0] if len(msg.buttons) > 0 else 0,
            'b': msg.buttons[1] if len(msg.buttons) > 1 else 0,
            'x': msg.buttons[2] if len(msg.buttons) > 2 else 0,
            'y': msg.buttons[3] if len(msg.buttons) > 3 else 0,
            'minus': msg.buttons[8] if len(msg.buttons) > 8 else 0,
            'plus': msg.buttons[9] if len(msg.buttons) > 9 else 0,
        }
        
        # 检测按下事件（边沿触发）
        for key, value in buttons.items():
            last_value = self.last_buttons.get(key, 0)
            if value == 1 and last_value == 0:
                self._on_button_pressed(key)
        
        self.last_buttons = buttons
    
    def _on_button_pressed(self, button: str):
        """按键按下事件处理"""
        if button == 'a':
            # A: 切换速度档位
            self.current_speed_level = (self.current_speed_level + 1) % len(self.speed_levels)
            self._apply_speed_level()
            self._log_speed_level()
        
        elif button == 'b':
            # B: 回到 home
            self.go_home()
        
        elif button == 'x' or button == 'minus':
            # X 或 -: 重新标定
            self.start_calibration()
        
        elif button == 'y':
            # Y: 设置当前姿态为零点
            self.imu_processor.set_zero_reference()
            self.get_logger().info('已设置当前姿态为零点参考')
    
    def start_calibration(self):
        """开始 IMU 标定"""
        self.control_enabled = False
        self.imu_processor.start_calibration()
        self.get_logger().info('开始 IMU 标定，请保持 Joy-Con 静止...')
        
        status_msg = String()
        status_msg.data = "calibrating"
        self.calibration_status_pub.publish(status_msg)
    
    def go_home(self):
        """回到 home 位置"""
        if self.is_going_home:
            self.get_logger().warn('正在执行 home 动作')
            return
        
        self.is_going_home = True
        self.get_logger().info('正在回到 home 位置...')
        
        joint_values = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        
        constraints = Constraints()
        for name, value in zip(self.joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints = [constraints]
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._home_response_callback)
    
    def _home_response_callback(self, future):
        """home 响应回调"""
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn('Home 请求被拒绝')
            self.is_going_home = False
            return
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._home_result_callback)
    
    def _home_result_callback(self, future):
        """home 结果回调"""
        self.is_going_home = False
        self.pose_initialized = False
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info('✓ 已回到 home 位置')
        else:
            self.get_logger().warn(f'Home 失败: {result.error_code.val}')
    
    # ==================== 控制逻辑 ====================
    
    def update_callback(self):
        """主控制循环"""
        # 检查是否可以控制
        if not self.control_enabled or self.is_going_home:
            return
        
        if self.current_imu is None:
            return
        
        # 获取相对姿态变化
        delta_roll, delta_pitch, delta_yaw = self.imu_processor.get_relative_attitude()
        
        # 应用死区
        delta_roll = self._apply_deadzone(delta_roll)
        delta_pitch = self._apply_deadzone(delta_pitch)
        delta_yaw = self._apply_deadzone(delta_yaw)
        
        # 获取 ZL/ZR 输入（用于 Z 轴控制）
        dz = 0.0
        if self.current_joy and len(self.current_joy.axes) >= 6:
            # 假设 ZL = axes[2], ZR = axes[5]（可能需要调整）
            zl = self._apply_trigger(self.current_joy.axes[2] if len(self.current_joy.axes) > 2 else 1.0)
            zr = self._apply_trigger(self.current_joy.axes[5] if len(self.current_joy.axes) > 5 else 1.0)
            dz = (zr - zl) * self.z_scale
        
        # 将姿态变化映射到笛卡尔增量
        # Pitch (前后倾斜) -> X
        # Roll (左右倾斜) -> Y
        dx = delta_pitch * self.translation_scale
        dy = delta_roll * self.translation_scale
        
        # Yaw -> 末端旋转
        dyaw = delta_yaw * self.rotation_scale
        
        # 检查是否有有效输入
        threshold = 0.0001
        has_input = (abs(dx) > threshold or abs(dy) > threshold or 
                     abs(dz) > threshold or abs(dyaw) > threshold)
        
        if not has_input:
            # 无输入时保持当前位置
            if self.last_ik_joint_positions is not None:
                self._send_joint_positions(self.last_ik_joint_positions)
            return
        
        # 同步当前位姿
        if not self.pose_initialized:
            if not self._sync_current_pose():
                return
        
        # 更新目标位姿
        self.target_pose.pose.position.x += dx
        self.target_pose.pose.position.y += dy
        self.target_pose.pose.position.z += dz
        
        # 更新姿态
        q = self.target_pose.pose.orientation
        current_roll, current_pitch, current_yaw = euler_from_quaternion([
            q.x, q.y, q.z, q.w
        ])
        
        new_yaw = current_yaw + dyaw
        new_q = quaternion_from_euler(current_roll, current_pitch, new_yaw)
        self.target_pose.pose.orientation.x = new_q[0]
        self.target_pose.pose.orientation.y = new_q[1]
        self.target_pose.pose.orientation.z = new_q[2]
        self.target_pose.pose.orientation.w = new_q[3]
        
        # 发布目标位姿
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose_pub.publish(self.target_pose)
        
        # 发送 IK 目标
        self._send_ik_goal()
    
    def _apply_deadzone(self, value: float) -> float:
        """应用死区"""
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone)
    
    def _apply_trigger(self, trigger_value: float) -> float:
        """处理扳机输入（1.0 未按 -> -1.0 完全按下）"""
        normalized = (1.0 - trigger_value) / 2.0
        trigger_deadzone = 0.1
        if normalized < trigger_deadzone:
            return 0.0
        return (normalized - trigger_deadzone) / (1.0 - trigger_deadzone)
    
    def _sync_current_pose(self) -> bool:
        """从 TF 同步当前位姿"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.end_effector_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            self.target_pose = PoseStamped()
            self.target_pose.header.frame_id = self.base_frame
            self.target_pose.pose.position.x = transform.transform.translation.x
            self.target_pose.pose.position.y = transform.transform.translation.y
            self.target_pose.pose.position.z = transform.transform.translation.z
            self.target_pose.pose.orientation = transform.transform.rotation
            
            if not self.pose_initialized:
                self.get_logger().info(
                    f'位姿同步: x={self.target_pose.pose.position.x:.3f}, '
                    f'y={self.target_pose.pose.position.y:.3f}, '
                    f'z={self.target_pose.pose.position.z:.3f}'
                )
                if self.current_joint_state:
                    self.last_ik_joint_positions = list(self.current_joint_state.position[:6])
                self.pose_initialized = True
            
            return True
        except Exception as e:
            self.get_logger().debug(f'TF 查询失败: {e}')
            return False
    
    def _send_ik_goal(self):
        """发送 IK 目标"""
        if self.pending_ik_request:
            return
        
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.planning_group
        request.ik_request.robot_state.is_diff = False
        
        # 使用上次 IK 解作为种子
        if self.last_ik_joint_positions is not None:
            seed_state = JointState()
            seed_state.name = self.joint_names
            seed_state.position = self.last_ik_joint_positions
            request.ik_request.robot_state.joint_state = seed_state
        elif self.current_joint_state:
            request.ik_request.robot_state.joint_state = self.current_joint_state
        
        request.ik_request.pose_stamped.header.frame_id = self.base_frame
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = self.target_pose.pose
        
        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = 10_000_000  # 10ms
        
        self.pending_ik_request = True
        future = self.ik_client.call_async(request)
        future.add_done_callback(self._ik_callback)
    
    def _ik_callback(self, future):
        """IK 回调"""
        self.pending_ik_request = False
        
        try:
            response = future.result()
            
            if response.error_code.val != 1:  # SUCCESS = 1
                return
            
            solution = response.solution.joint_state
            ik_positions = []
            
            for joint_name in self.joint_names:
                if joint_name in solution.name:
                    idx = solution.name.index(joint_name)
                    ik_positions.append(solution.position[idx])
                else:
                    return
            
            # 检查变化是否足够大
            if self.last_ik_joint_positions is not None:
                max_diff = max(abs(ik_positions[i] - self.last_ik_joint_positions[i]) 
                              for i in range(len(ik_positions)))
                if max_diff < 0.0001:
                    return
            
            self.last_ik_joint_positions = ik_positions
            self._send_joint_positions(ik_positions)
            
        except Exception as e:
            self.get_logger().error(f'IK 回调异常: {e}')
    
    def _send_joint_positions(self, positions):
        """发送关节位置"""
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.velocities = [0.0] * 6
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 20_000_000  # 20ms
        
        trajectory.points = [point]
        self.joint_trajectory_pub.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)
    node = JoyConIMUTeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
