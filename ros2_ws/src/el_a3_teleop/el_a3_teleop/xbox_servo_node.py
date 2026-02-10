#!/usr/bin/env python3
"""
Xbox 手柄实时控制节点（基于 MoveIt Servo 增量控制）
使用雅可比矩阵方式，尽量避免 IK 解跳变问题

控制映射：
- 左摇杆：X/Y 平移
- LT/RT：Z 平移
- 右摇杆：Yaw/Roll 旋转
- LB/RB：Pitch 旋转
- A 键：切换速度档位（5 档）
- B 键：回到 home 位置
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import TwistStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from std_srvs.srv import Trigger
import math


class XboxServoNode(Node):
    """使用 MoveIt Servo 的 Xbox 遥操作节点"""

    def __init__(self):
        super().__init__('xbox_servo_node')
        
        # Declare parameters
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('linear_scale', 0.15)      # Linear velocity m/s
        self.declare_parameter('angular_scale', 0.5)      # Angular velocity rad/s
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'end_effector')
        self.declare_parameter('debug_input', False)
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.deadzone = self.get_parameter('deadzone').value
        self.planning_group = self.get_parameter('planning_group').value
        self.base_frame = self.get_parameter('base_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.debug_input = self.get_parameter('debug_input').value
        
        # Speed level settings (5 levels)
        self.speed_levels = [0.2, 0.4, 0.6, 0.8, 1.0]  # Speed multiplier
        self.current_speed_level = 1  # Default level 2 (40%)
        
        # State variables
        self.current_joy = None
        self.last_a_button = 0
        self.last_b_button = 0
        self.is_going_home = False
        self.servo_enabled = False
        self.current_joint_state = None
        self.joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']
        
        # Smoothing filter state
        self.smoothed_vx = 0.0
        self.smoothed_vy = 0.0
        self.smoothed_vz = 0.0
        self.smoothed_wx = 0.0
        self.smoothed_wy = 0.0
        self.smoothed_wz = 0.0
        self.smoothing_factor = 0.3  # EMA filter coefficient
        
        # QoS configuration
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to controller input
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, reliable_qos)
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Publish Twist commands to MoveIt Servo
        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', reliable_qos)
        
        # MoveGroup Action client (for home motion)
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Servo start/stop service clients
        self.servo_start_client = self.create_client(
            Trigger, '/servo_node/start_servo')
        self.servo_stop_client = self.create_client(
            Trigger, '/servo_node/stop_servo')
        
        # Timer - update loop
        self.update_timer = self.create_timer(
            1.0 / self.update_rate, self.update_callback)
        
        self.get_logger().info('Xbox Servo 节点已启动')
        self.get_logger().info(f'  - 更新频率：{self.update_rate} Hz')
        self.get_logger().info(f'  - 线速度范围：±{self.linear_scale} m/s')
        self.get_logger().info(f'  - 角速度范围：±{self.angular_scale} rad/s')
        self.get_logger().info(f'  - 速度档位：{self.current_speed_level + 1}/{len(self.speed_levels)}')
        
        # Wait for joint states
        self.wait_for_joint_states()
        
        # Start Servo
        self.start_servo()
        
        # Move to home position on startup
        self.startup_go_home()
    
    def wait_for_joint_states(self):
        """Wait for joint states to be available"""
        self.get_logger().info('等待关节状态（读取电机位置）...')
        
        timeout_sec = 10.0
        start_time = self.get_clock().now()
        
        while self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().warn('等待关节状态超时！继续启动流程...')
                return False
        
        self.get_logger().info('已读取当前电机位置')
        return True
    
    def start_servo(self):
        """Start MoveIt Servo"""
        if not self.servo_start_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Servo 启动服务不可用，改用直接发布模式')
            self.servo_enabled = True  # Assume direct publish is possible
            return
        
        request = Trigger.Request()
        future = self.servo_start_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().success:
            self.servo_enabled = True
            self.get_logger().info('MoveIt Servo 已启动')
        else:
            self.get_logger().warn('MoveIt Servo 启动失败，改用直接发布模式')
            self.servo_enabled = True
    
    def stop_servo(self):
        """Stop MoveIt Servo"""
        if not self.servo_stop_client.wait_for_service(timeout_sec=2.0):
            return
        
        request = Trigger.Request()
        future = self.servo_stop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        self.servo_enabled = False
        self.get_logger().info('MoveIt Servo 已停止')
    
    def startup_go_home(self):
        """Move to home position on startup"""
        import time
        
        self.is_going_home = True
        self.get_logger().info('正在规划运动到 home 位置...')
        
        # 创建MoveGroup请求
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2
        
        # Set target joint positions (home position)
        joint_values = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]
        
        constraints = Constraints()
        for name, value in zip(self.joint_names, joint_values):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = value
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        goal_msg.request.goal_constraints = [constraints]
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3
        
        # Try sending home request
        max_attempts = 3
        for attempt in range(1, max_attempts + 1):
            if not self.move_group_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().warn(f'MoveGroup 服务未就绪（第 {attempt}/{max_attempts} 次）')
                time.sleep(2.0)
                continue
            
            self.get_logger().info(f'发送 home 运动请求...（第 {attempt}/{max_attempts} 次）')
            send_goal_future = self.move_group_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
            goal_handle = send_goal_future.result()
            
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warn('home 运动请求被拒绝，等待后重试...')
                time.sleep(2.0)
                continue
            
            self.get_logger().info('home 运动请求已接受，正在低速执行...')
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
            
            result = result_future.result()
            self.is_going_home = False
            
            try:
                if result and result.result and result.result.error_code.val == result.result.error_code.SUCCESS:
                    self.get_logger().info('✓ 已低速移动到 home 位置！')
                    return True
            except Exception as e:
                self.get_logger().warn(f'解析 home 运动结果失败：{e}')
            
            time.sleep(2.0)
        
        self.get_logger().error('多次尝试后 home 运动仍失败')
        self.is_going_home = False
        return False
    
    def joy_callback(self, msg):
        """Controller input callback"""
        self.current_joy = msg
    
    def joint_state_callback(self, msg):
        """Joint state callback"""
        self.current_joint_state = msg
    
    def apply_deadzone(self, value):
        """Apply deadzone"""
        if abs(value) < self.deadzone:
            return 0.0
        # Remap values outside deadzone starting from 0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def apply_trigger_deadzone(self, value):
        """Trigger deadzone processing (LT/RT)"""
        # Original range 1.0 to -1.0, pressed is -1.0
        normalized = (1.0 - value) / 2.0  # Convert to 0-1
        if normalized < self.deadzone:
            return 0.0
        return (normalized - self.deadzone) / (1.0 - self.deadzone)
    
    def switch_speed_level(self):
        """Switch speed level"""
        self.current_speed_level = (self.current_speed_level + 1) % len(self.speed_levels)
        level_percent = int(self.speed_levels[self.current_speed_level] * 100)
        self.get_logger().info(f'速度档位：{self.current_speed_level + 1}/{len(self.speed_levels)}（{level_percent}%）')
    
    def update_callback(self):
        """Periodic update callback"""
        if self.current_joy is None or self.is_going_home:
            return
        
        joy = self.current_joy
        
        if len(joy.axes) < 6 or len(joy.buttons) < 6:
            return
        
        # A button - Switch speed level
        a_button = joy.buttons[0]
        if a_button == 1 and self.last_a_button == 0:
            self.switch_speed_level()
        self.last_a_button = a_button
        
        # B button - Return to home
        b_button = joy.buttons[1]
        if b_button == 1 and self.last_b_button == 0:
            self.go_home()
        self.last_b_button = b_button
        
        # Get speed multiplier
        speed_mult = self.speed_levels[self.current_speed_level]
        
        # Parse controller input - calculate velocity directly
        # Translation control - Left stick XY swapped and inverted
        raw_vx = -self.apply_deadzone(joy.axes[1]) * self.linear_scale * speed_mult
        raw_vy = -self.apply_deadzone(joy.axes[0]) * self.linear_scale * speed_mult
        
        # LT/RT control Z
        lt = self.apply_trigger_deadzone(joy.axes[2])
        rt = self.apply_trigger_deadzone(joy.axes[5])
        raw_vz = (rt - lt) * self.linear_scale * speed_mult
        
        # Rotation control - Right stick
        raw_wz = self.apply_deadzone(joy.axes[3]) * self.angular_scale * speed_mult  # Yaw
        raw_wx = self.apply_deadzone(joy.axes[4]) * self.angular_scale * speed_mult  # Roll (right stick Y)
        
        # LB/RB control Pitch
        raw_wy = (joy.buttons[5] - joy.buttons[4]) * self.angular_scale * speed_mult
        
        # Apply smoothing filter
        alpha = self.smoothing_factor
        self.smoothed_vx = alpha * raw_vx + (1 - alpha) * self.smoothed_vx
        self.smoothed_vy = alpha * raw_vy + (1 - alpha) * self.smoothed_vy
        self.smoothed_vz = alpha * raw_vz + (1 - alpha) * self.smoothed_vz
        self.smoothed_wx = alpha * raw_wx + (1 - alpha) * self.smoothed_wx
        self.smoothed_wy = alpha * raw_wy + (1 - alpha) * self.smoothed_wy
        self.smoothed_wz = alpha * raw_wz + (1 - alpha) * self.smoothed_wz
        
        # Use smoothed values
        vx = self.smoothed_vx
        vy = self.smoothed_vy
        vz = self.smoothed_vz
        wx = self.smoothed_wx
        wy = self.smoothed_wy
        wz = self.smoothed_wz
        
        # Check if there is valid input
        velocity_threshold = 0.001
        has_input = (abs(vx) > velocity_threshold or abs(vy) > velocity_threshold or 
                     abs(vz) > velocity_threshold or abs(wx) > velocity_threshold or 
                     abs(wy) > velocity_threshold or abs(wz) > velocity_threshold)
        
        # Debug output
        if self.debug_input and has_input:
            self.get_logger().info(
                f'Twist: vx={vx:.3f} vy={vy:.3f} vz={vz:.3f} '
                f'wx={wx:.3f} wy={wy:.3f} wz={wz:.3f}'
            )
        
        # Publish Twist command
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.ee_frame  # End effector frame
        
        twist_msg.twist.linear.x = vx
        twist_msg.twist.linear.y = vy
        twist_msg.twist.linear.z = vz
        twist_msg.twist.angular.x = wx
        twist_msg.twist.angular.y = wy
        twist_msg.twist.angular.z = wz
        
        # Only publish when there is input, avoid unnecessary motion
        if has_input:
            self.twist_pub.publish(twist_msg)
    
    def go_home(self):
        """Return to home position"""
        self.get_logger().info('正在回到 home 位置...')
        # Pause Servo
        self.stop_servo()
        # Execute home motion
        self.startup_go_home()
        # Restart Servo
        self.start_servo()


def main(args=None):
    rclpy.init(args=args)
    node = XboxServoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_servo()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



