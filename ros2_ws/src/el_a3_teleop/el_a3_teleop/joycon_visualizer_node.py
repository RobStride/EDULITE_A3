#!/usr/bin/env python3
"""
Joy-Con IMU 可视化 ROS2 节点

发布 TF 和 Marker，可在 RViz 中查看 Joy-Con 姿态

Topics:
- /joycon/pose: 姿态 PoseStamped
- /joycon/imu_euler: 欧拉角 Vector3
- /joycon/marker: 可视化 Marker
- TF: world -> joycon

启动:
    ros2 run el_a3_teleop joycon_visualizer

然后打开 RViz，添加 TF 和 Marker 显示
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Vector3, TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import String, ColorRGBA
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading
import time

# 尝试导入 evdev
try:
    import evdev
    from evdev import ecodes
    EVDEV_AVAILABLE = True
except ImportError:
    EVDEV_AVAILABLE = False


class MadgwickFilter:
    """Madgwick AHRS 姿态估计"""
    
    def __init__(self, sample_rate=60.0, beta=0.1):
        self.dt = 1.0 / sample_rate
        self.beta = beta
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
    
    def update(self, accel, gyro):
        q = self.q.copy()
        accel_norm = np.linalg.norm(accel)
        if accel_norm > 0:
            a = accel / accel_norm
        else:
            a = accel
        
        q0, q1, q2, q3 = q
        gx, gy, gz = gyro
        
        qDot = 0.5 * np.array([
            -q1*gx - q2*gy - q3*gz,
             q0*gx + q2*gz - q3*gy,
             q0*gy - q1*gz + q3*gx,
             q0*gz + q1*gy - q2*gx
        ])
        
        if accel_norm > 0:
            f = np.array([
                2*(q1*q3 - q0*q2) - a[0],
                2*(q0*q1 + q2*q3) - a[1],
                2*(0.5 - q1*q1 - q2*q2) - a[2]
            ])
            J = np.array([
                [-2*q2,  2*q3, -2*q0,  2*q1],
                [ 2*q1,  2*q0,  2*q3,  2*q2],
                [    0, -4*q1, -4*q2,     0]
            ])
            step = J.T @ f
            step_norm = np.linalg.norm(step)
            if step_norm > 0:
                step = step / step_norm
            qDot = qDot - self.beta * step
        
        q = q + qDot * self.dt
        q = q / np.linalg.norm(q)
        self.q = q
        
        return self.get_euler()
    
    def get_euler(self):
        w, x, y, z = self.q
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(np.clip(sinp, -1, 1))
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def get_quaternion(self):
        """返回 [x, y, z, w] 格式"""
        return [self.q[1], self.q[2], self.q[3], self.q[0]]
    
    def reset(self):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])


class JoyConVisualizerNode(Node):
    """Joy-Con 可视化节点"""
    
    def __init__(self):
        super().__init__('joycon_visualizer_node')
        
        # 参数
        self.declare_parameter('sample_rate', 60.0)
        self.declare_parameter('use_mock', False)
        self.sample_rate = self.get_parameter('sample_rate').value
        self.use_mock = self.get_parameter('use_mock').value
        
        # Madgwick 滤波器
        self.filter = MadgwickFilter(sample_rate=self.sample_rate)
        
        # 标定状态
        self.is_calibrated = False
        self.calibrating = False
        self.calibration_samples = []
        self.gyro_bias = np.zeros(3)
        
        # 零点参考
        self.ref_q = None
        
        # IMU 数据
        self.accel = np.zeros(3)
        self.gyro = np.zeros(3)
        self.data_lock = threading.Lock()
        
        # TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 发布器
        self.pose_pub = self.create_publisher(PoseStamped, '/joycon/pose', 10)
        self.euler_pub = self.create_publisher(Vector3, '/joycon/imu_euler', 10)
        self.marker_pub = self.create_publisher(Marker, '/joycon/marker', 10)
        self.status_pub = self.create_publisher(String, '/joycon/status', 10)
        self.imu_pub = self.create_publisher(Imu, '/joycon/imu_raw', 10)
        
        # 启动 IMU 读取线程
        if not self.use_mock and EVDEV_AVAILABLE:
            self._start_imu_reader()
        else:
            self.get_logger().warn('使用模拟 IMU 数据')
            self._start_mock_reader()
        
        # 创建定时器
        self.timer = self.create_timer(1.0 / self.sample_rate, self.update_callback)
        
        # 打印信息
        self.get_logger().info('='*50)
        self.get_logger().info('Joy-Con 可视化节点已启动')
        self.get_logger().info('='*50)
        self.get_logger().info('发布话题:')
        self.get_logger().info('  /joycon/pose - PoseStamped')
        self.get_logger().info('  /joycon/imu_euler - Vector3')
        self.get_logger().info('  /joycon/marker - Marker')
        self.get_logger().info('  TF: world -> joycon')
        self.get_logger().info('')
        self.get_logger().info('在 RViz 中添加 TF 和 Marker 显示')
        self.get_logger().info('='*50)
        
        # 自动开始标定
        self.get_logger().info('3 秒后开始自动标定，请保持 Joy-Con 静止...')
        self.calibration_timer = self.create_timer(3.0, self._start_calibration_once)
    
    def _start_calibration_once(self):
        self.calibration_timer.cancel()
        self.start_calibration()
    
    def _start_imu_reader(self):
        """启动 evdev IMU 读取"""
        self.imu_device = None
        self.imu_running = True
        
        # 查找设备
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            name_lower = device.name.lower()
            if 'joy-con' in name_lower or 'joycon' in name_lower or 'nintendo' in name_lower:
                self.get_logger().info(f'找到 Joy-Con: {device.name}')
                self.imu_device = device
                break
        
        if self.imu_device:
            self.imu_thread = threading.Thread(target=self._read_imu_loop, daemon=True)
            self.imu_thread.start()
        else:
            self.get_logger().warn('未找到 Joy-Con，使用模拟数据')
            self._start_mock_reader()
    
    def _read_imu_loop(self):
        """读取 evdev 事件"""
        accel_scale = 9.81 / 4096.0
        gyro_scale = 0.0174533 / 131.0
        
        try:
            for event in self.imu_device.read_loop():
                if not self.imu_running:
                    break
                if event.type == ecodes.EV_ABS:
                    with self.data_lock:
                        if event.code == ecodes.ABS_X:
                            self.accel[0] = event.value * accel_scale
                        elif event.code == ecodes.ABS_Y:
                            self.accel[1] = event.value * accel_scale
                        elif event.code == ecodes.ABS_Z:
                            self.accel[2] = event.value * accel_scale
                        elif event.code == ecodes.ABS_RX:
                            self.gyro[0] = event.value * gyro_scale
                        elif event.code == ecodes.ABS_RY:
                            self.gyro[1] = event.value * gyro_scale
                        elif event.code == ecodes.ABS_RZ:
                            self.gyro[2] = event.value * gyro_scale
        except Exception as e:
            self.get_logger().error(f'IMU 读取错误: {e}')
    
    def _start_mock_reader(self):
        """启动模拟数据生成"""
        self.mock_t = 0
        self.imu_running = True
        self.mock_thread = threading.Thread(target=self._mock_loop, daemon=True)
        self.mock_thread.start()
    
    def _mock_loop(self):
        """生成模拟数据"""
        while self.imu_running:
            with self.data_lock:
                self.accel[0] = 0.3 * np.sin(self.mock_t * 0.5)
                self.accel[1] = 0.3 * np.cos(self.mock_t * 0.3)
                self.accel[2] = 9.81 + 0.1 * np.sin(self.mock_t * 0.2)
                
                self.gyro[0] = 0.05 * np.sin(self.mock_t * 0.8)
                self.gyro[1] = 0.05 * np.cos(self.mock_t * 0.6)
                self.gyro[2] = 0.02 * np.sin(self.mock_t * 0.4)
            
            self.mock_t += 1.0 / 60.0
            time.sleep(1.0 / 60.0)
    
    def start_calibration(self):
        """开始标定"""
        self.calibrating = True
        self.calibration_samples = []
        self.is_calibrated = False
        self.get_logger().info('开始标定，请保持 Joy-Con 静止...')
    
    def reset_reference(self):
        """重置零点参考"""
        self.ref_q = self.filter.q.copy()
        self.get_logger().info('零点已重置')
    
    def update_callback(self):
        """更新回调"""
        # 获取 IMU 数据
        with self.data_lock:
            accel = self.accel.copy()
            gyro = self.gyro.copy()
        
        # 发布原始 IMU
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'joycon'
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        self.imu_pub.publish(imu_msg)
        
        # 标定处理
        if self.calibrating:
            self.calibration_samples.append(gyro.copy())
            if len(self.calibration_samples) >= 100:
                self.gyro_bias = np.mean(self.calibration_samples, axis=0)
                self.is_calibrated = True
                self.calibrating = False
                self.filter.reset()
                self.get_logger().info(f'✓ 标定完成! 陀螺仪零偏: [{self.gyro_bias[0]:.4f}, {self.gyro_bias[1]:.4f}, {self.gyro_bias[2]:.4f}]')
                
                # 设置零点参考
                self.reset_reference()
        
        # 应用标定
        if self.is_calibrated:
            gyro = gyro - self.gyro_bias
        
        # 更新姿态
        roll, pitch, yaw = self.filter.update(accel, gyro)
        q = self.filter.get_quaternion()  # [x, y, z, w]
        
        # 发布欧拉角
        euler_msg = Vector3()
        euler_msg.x = roll
        euler_msg.y = pitch
        euler_msg.z = yaw
        self.euler_pub.publish(euler_msg)
        
        # 发布 PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        self.pose_pub.publish(pose_msg)
        
        # 发布 TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'joycon'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        
        # 发布 Marker (Joy-Con 形状)
        self._publish_marker(q)
        
        # 发布状态
        status_msg = String()
        status_msg.data = f"Roll:{np.degrees(roll):.1f} Pitch:{np.degrees(pitch):.1f} Yaw:{np.degrees(yaw):.1f}"
        self.status_pub.publish(status_msg)
    
    def _publish_marker(self, q):
        """发布可视化 Marker"""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'world'
        marker.ns = 'joycon'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # 位置
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        
        # 姿态
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        
        # 尺寸 (Joy-Con 形状)
        marker.scale.x = 0.1   # 长
        marker.scale.y = 0.035  # 宽
        marker.scale.z = 0.015  # 高
        
        # 颜色 (红色 = Joy-Con R)
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.9
        
        self.marker_pub.publish(marker)
        
        # 发布坐标轴 Markers
        for i, (axis, color) in enumerate([
            ([0.15, 0, 0], [1, 0, 0]),  # X - 红
            ([0, 0.15, 0], [0, 1, 0]),  # Y - 绿
            ([0, 0, 0.15], [0, 0, 1]),  # Z - 蓝
        ]):
            arrow = Marker()
            arrow.header = marker.header
            arrow.ns = 'joycon_axes'
            arrow.id = i + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            
            arrow.pose.orientation.x = q[0]
            arrow.pose.orientation.y = q[1]
            arrow.pose.orientation.z = q[2]
            arrow.pose.orientation.w = q[3]
            
            arrow.scale.x = 0.08  # 长度
            arrow.scale.y = 0.01  # 宽度
            arrow.scale.z = 0.01
            
            # 旋转后的轴方向
            rot = R.from_quat(q)
            axis_dir = rot.apply(axis)
            
            arrow.points = []
            from geometry_msgs.msg import Point
            p1 = Point()
            p1.x = 0.0
            p1.y = 0.0
            p1.z = 0.0
            p2 = Point()
            p2.x = axis_dir[0]
            p2.y = axis_dir[1]
            p2.z = axis_dir[2]
            arrow.points = [p1, p2]
            
            arrow.color.r = float(color[0])
            arrow.color.g = float(color[1])
            arrow.color.b = float(color[2])
            arrow.color.a = 1.0
            
            self.marker_pub.publish(arrow)
    
    def destroy_node(self):
        self.imu_running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyConVisualizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
