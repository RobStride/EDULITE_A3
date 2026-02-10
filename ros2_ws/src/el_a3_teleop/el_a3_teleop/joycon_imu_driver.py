#!/usr/bin/env python3
"""
Joy-Con IMU 驱动节点

从 Linux evdev 读取 Joy-Con 的 IMU 数据，发布为 sensor_msgs/Imu

Joy-Con 通过 hid-nintendo 内核模块提供 IMU 数据，
可以通过 evdev 接口读取加速度计和陀螺仪数据。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import struct
import os
import glob
import threading
import time

# 尝试导入 evdev
try:
    import evdev
    from evdev import ecodes
    EVDEV_AVAILABLE = True
except ImportError:
    EVDEV_AVAILABLE = False


class JoyConIMUDriver(Node):
    """Joy-Con IMU 驱动节点"""
    
    def __init__(self):
        super().__init__('joycon_imu_driver')
        
        # 声明参数
        self.declare_parameter('device_path', '')
        self.declare_parameter('publish_rate', 60.0)
        self.declare_parameter('accel_scale', 9.81 / 4096.0)  # ±8g, 16-bit
        self.declare_parameter('gyro_scale', 0.0174533 / 131.0)  # ±2000°/s to rad/s
        
        # 获取参数
        self.device_path = self.get_parameter('device_path').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.accel_scale = self.get_parameter('accel_scale').value
        self.gyro_scale = self.get_parameter('gyro_scale').value
        
        # IMU 数据
        self.accel = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.data_lock = threading.Lock()
        
        # 发布器
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        
        # 检查 evdev 是否可用
        if not EVDEV_AVAILABLE:
            self.get_logger().error('evdev 库未安装，请运行: pip3 install evdev')
            self.get_logger().info('将使用模拟 IMU 数据（用于测试）')
            self._use_mock_data = True
        else:
            self._use_mock_data = False
            # 查找 Joy-Con 设备
            self.device = self._find_joycon_device()
        
        # 启动读取线程
        if not self._use_mock_data and self.device:
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
        
        # 创建发布定时器
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_imu)
        
        self.get_logger().info('Joy-Con IMU 驱动已启动')
        if self._use_mock_data:
            self.get_logger().warn('使用模拟 IMU 数据模式')
    
    def _find_joycon_device(self):
        """查找 Joy-Con 设备"""
        if self.device_path:
            try:
                device = evdev.InputDevice(self.device_path)
                self.get_logger().info(f'使用指定设备: {device.name}')
                return device
            except Exception as e:
                self.get_logger().error(f'无法打开设备 {self.device_path}: {e}')
        
        # 自动查找 Joy-Con
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        
        for device in devices:
            name_lower = device.name.lower()
            # 查找 Joy-Con 或 Pro Controller
            if 'joy-con' in name_lower or 'joycon' in name_lower or 'nintendo' in name_lower:
                # 检查是否有加速度计
                caps = device.capabilities()
                if ecodes.EV_ABS in caps:
                    abs_caps = caps[ecodes.EV_ABS]
                    abs_codes = [c[0] if isinstance(c, tuple) else c for c in abs_caps]
                    # Joy-Con IMU 使用 ABS_X, ABS_Y, ABS_Z 等
                    if ecodes.ABS_X in abs_codes:
                        self.get_logger().info(f'找到 Joy-Con 设备: {device.name} ({device.path})')
                        return device
        
        self.get_logger().warn('未找到 Joy-Con 设备，将使用模拟数据')
        self._use_mock_data = True
        return None
    
    def _read_loop(self):
        """读取 evdev 事件循环"""
        try:
            for event in self.device.read_loop():
                if event.type == ecodes.EV_ABS:
                    with self.data_lock:
                        # 加速度计数据
                        if event.code == ecodes.ABS_X:
                            self.accel[0] = event.value * self.accel_scale
                        elif event.code == ecodes.ABS_Y:
                            self.accel[1] = event.value * self.accel_scale
                        elif event.code == ecodes.ABS_Z:
                            self.accel[2] = event.value * self.accel_scale
                        # 陀螺仪数据
                        elif event.code == ecodes.ABS_RX:
                            self.gyro[0] = event.value * self.gyro_scale
                        elif event.code == ecodes.ABS_RY:
                            self.gyro[1] = event.value * self.gyro_scale
                        elif event.code == ecodes.ABS_RZ:
                            self.gyro[2] = event.value * self.gyro_scale
        except Exception as e:
            self.get_logger().error(f'读取设备失败: {e}')
            self._use_mock_data = True
    
    def _publish_imu(self):
        """发布 IMU 消息"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'joycon_imu'
        
        if self._use_mock_data:
            # 模拟静止状态的 IMU 数据
            # 加速度计：静止时指向重力方向
            msg.linear_acceleration.x = 0.0
            msg.linear_acceleration.y = 0.0
            msg.linear_acceleration.z = 9.81
            
            # 陀螺仪：静止时为零
            msg.angular_velocity.x = 0.0
            msg.angular_velocity.y = 0.0
            msg.angular_velocity.z = 0.0
        else:
            with self.data_lock:
                msg.linear_acceleration.x = self.accel[0]
                msg.linear_acceleration.y = self.accel[1]
                msg.linear_acceleration.z = self.accel[2]
                
                msg.angular_velocity.x = self.gyro[0]
                msg.angular_velocity.y = self.gyro[1]
                msg.angular_velocity.z = self.gyro[2]
        
        # 协方差设为未知
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = 0.01
        msg.linear_acceleration_covariance[0] = 0.1
        
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyConIMUDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
