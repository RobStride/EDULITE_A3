#!/usr/bin/env python3
"""
Xbox 手柄实时 Servo 控制节点
使用 MoveIt Servo 进行实时笛卡尔空间控制

控制映射：
- 左摇杆：X/Y 平移
- LT/RT：Z 平移
- 右摇杆：Yaw/Pitch 旋转
- LB/RB：Roll 旋转
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
import math


class XboxServoTeleopNode(Node):
    def __init__(self):
        super().__init__('xbox_servo_teleop_node')
        
        # Declare parameters
        self.declare_parameter('linear_scale', 0.1)  # m/s
        self.declare_parameter('angular_scale', 0.5)  # rad/s
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('deadzone', 0.1)  # Joystick deadzone
        
        # Get parameters
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.base_frame = self.get_parameter('base_frame').value
        self.deadzone = self.get_parameter('deadzone').value
        
        # Subscribe to controller input
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Publish Twist commands to MoveIt Servo
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )
        
        self.get_logger().info('Xbox 手柄 Servo 控制节点已启动')
        self.get_logger().info(f'线速度缩放：{self.linear_scale} m/s')
        self.get_logger().info(f'角速度缩放：{self.angular_scale} rad/s')
        
    def apply_deadzone(self, value):
        """应用摇杆死区"""
        if abs(value) < self.deadzone:
            return 0.0
        # Linearly map values outside the deadzone
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        
    def joy_callback(self, msg):
        """Controller input callback"""
        # Xbox controller mapping:
        # Left stick X: axes[0]
        # Left stick Y: axes[1]
        # LT: axes[2] (range 1.0 to -1.0, pressed is -1.0)
        # RT: axes[5] (range 1.0 to -1.0, pressed is -1.0)
        # Right stick X: axes[3]
        # Right stick Y: axes[4]
        # LB: buttons[4]
        # RB: buttons[5]
        
        # Ensure indices are valid
        if len(msg.axes) < 6 or len(msg.buttons) < 6:
            return
            
        # Create Twist message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.base_frame
        
        # Translation control (linear velocity)
        twist_msg.twist.linear.x = self.apply_deadzone(msg.axes[0]) * self.linear_scale  # Left stick X -> X
        twist_msg.twist.linear.y = self.apply_deadzone(msg.axes[1]) * self.linear_scale  # Left stick Y -> Y
        
        # LT/RT control Z
        lt = (1.0 - msg.axes[2]) / 2.0  # Convert to 0-1 range
        rt = (1.0 - msg.axes[5]) / 2.0  # Convert to 0-1 range
        twist_msg.twist.linear.z = (rt - lt) * self.linear_scale  # RT up, LT down
        
        # Rotation control (angular velocity)
        twist_msg.twist.angular.z = self.apply_deadzone(msg.axes[3]) * self.angular_scale  # Right stick X -> Yaw
        twist_msg.twist.angular.y = self.apply_deadzone(msg.axes[4]) * self.angular_scale  # Right stick Y -> Pitch
        
        # LB/RB control Roll
        twist_msg.twist.angular.x = (msg.buttons[5] - msg.buttons[4]) * self.angular_scale  # RB positive, LB negative
        
        # Publish Twist command
        self.twist_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = XboxServoTeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()





