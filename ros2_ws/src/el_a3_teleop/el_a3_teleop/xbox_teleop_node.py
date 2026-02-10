#!/usr/bin/env python3
"""
Xbox 手柄实时控制节点
使用笛卡尔空间控制机械臂末端执行器

控制映射：
- 左摇杆：X/Y 平移
- LT/RT：Z 方向平移
- 右摇杆：Yaw/Roll 旋转
- LB/RB：Pitch 旋转
- A 键：切换速度档位（5 档）
- B 键：回到 home 位置
- Y 键：切换重力补偿模式（手动示教/拖动）
- Menu 键：切换主从遥操作（can0 主臂可拖动 / can1 从臂实时跟随）
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy
import socket
import struct
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    PlanningOptions,
    RobotState,
    JointConstraint,
)
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from std_srvs.srv import SetBool
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
import copy

# Use scipy instead of tf_transformations
from scipy.spatial.transform import Rotation as R

def quaternion_from_euler(roll, pitch, yaw):
    """将欧拉角转换为四元数 (x, y, z, w)"""
    rot = R.from_euler('xyz', [roll, pitch, yaw])
    quat = rot.as_quat()  # Returns (x, y, z, w)
    return quat

def euler_from_quaternion(quaternion):
    """将四元数 (x, y, z, w) 转换为欧拉角"""
    rot = R.from_quat(quaternion)
    euler = rot.as_euler('xyz')
    return euler[0], euler[1], euler[2]  # roll, pitch, yaw


class XboxTeleopNode(Node):
    def __init__(self):
        super().__init__('xbox_teleop_node')
        
        # Declare parameters
        self.declare_parameter('update_rate', 50.0)  # Hz - 50Hz for better smoothness
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('end_effector_frame', 'end_effector')
        self.declare_parameter('deadzone', 0.15)  # Joystick deadzone
        self.declare_parameter('debug_input', False)  # Debug mode
        self.declare_parameter('use_fast_ik_mode', True)  # Use fast IK mode for 50Hz control
        
        # Cartesian velocity parameters (actual physical units)
        self.declare_parameter('max_linear_velocity', 0.15)   # m/s max linear velocity
        self.declare_parameter('max_angular_velocity', 1.5)   # rad/s max angular velocity
        
        # Joint output smoothing parameters
        self.declare_parameter('joint_smoothing_alpha', 0.15)  # Joint smoothing coefficient
        self.declare_parameter('max_joint_velocity', 1.5)      # rad/s max single joint velocity
        self.declare_parameter('max_joint_acceleration', 5.0)  # rad/s² max single joint acceleration
        
        # Input smoothing parameters
        self.declare_parameter('input_smoothing_factor', 0.5)  # Input smoothing coefficient
        
        # Singularity protection parameters
        self.declare_parameter('max_ik_jump_threshold', 0.5)   # rad max allowed single joint jump
        self.declare_parameter('singularity_warning_count', 5) # Consecutive rejection warning threshold
        
        # Collision detection parameters
        self.declare_parameter('enable_collision_check', True)  # Enable collision detection after initialization
        
        # Master-slave teleoperation parameters (Menu button triggered)
        self.declare_parameter('master_slave_enabled', True)
        self.declare_parameter('master_joint_state_topic', '/arm1/joint_states')
        self.declare_parameter('master_trajectory_topic', '/arm1/arm1_arm_controller/joint_trajectory')
        self.declare_parameter('master_zero_torque_service', '/el_a3/set_zero_torque_mode')
        self.declare_parameter('master_joint_prefix', 'arm1_')
        self.declare_parameter('slave_joint_state_topic', '/arm2/joint_states')
        self.declare_parameter('slave_trajectory_topic', '/arm2/arm2_arm_controller/joint_trajectory')
        self.declare_parameter('slave_joint_prefix', 'arm2_')
        self.declare_parameter('go_zero_duration', 3.0)
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.planning_group = self.get_parameter('planning_group').value
        self.base_frame = self.get_parameter('base_frame').value
        self.end_effector_frame = self.get_parameter('end_effector_frame').value
        self.deadzone = self.get_parameter('deadzone').value
        self.debug_input = self.get_parameter('debug_input').value
        self.use_fast_ik_mode = self.get_parameter('use_fast_ik_mode').value
        
        # Cartesian velocity parameters
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        
        # Joint smoothing parameters
        self.joint_smoothing_alpha = self.get_parameter('joint_smoothing_alpha').value
        self.max_joint_velocity = self.get_parameter('max_joint_velocity').value
        self.max_joint_acceleration = self.get_parameter('max_joint_acceleration').value
        self.last_joint_velocities = None  # Previous frame joint velocities (for acceleration limiting)
        
        # Input smoothing parameters
        self.input_smoothing_factor = self.get_parameter('input_smoothing_factor').value
        
        # Singularity protection parameters
        self.max_ik_jump_threshold = self.get_parameter('max_ik_jump_threshold').value
        self.singularity_warning_count = self.get_parameter('singularity_warning_count').value
        self.consecutive_ik_rejects = 0  # Consecutive rejection counter
        self.ik_seed_just_initialized = False  # IK seed just initialized flag, skip first frame jump detection
        
        # Collision detection parameters
        self.enable_collision_check = self.get_parameter('enable_collision_check').value
        self.collision_check_active = False  # Disabled during initialization, enabled after completion
        
        # Master-slave teleoperation parameters
        self.master_slave_enabled = self.get_parameter('master_slave_enabled').value
        master_joint_state_topic = self.get_parameter('master_joint_state_topic').value
        master_trajectory_topic = self.get_parameter('master_trajectory_topic').value
        master_zero_torque_service = self.get_parameter('master_zero_torque_service').value
        master_prefix = self.get_parameter('master_joint_prefix').value
        slave_joint_state_topic = self.get_parameter('slave_joint_state_topic').value
        slave_trajectory_topic = self.get_parameter('slave_trajectory_topic').value
        slave_prefix = self.get_parameter('slave_joint_prefix').value
        self.go_zero_duration = self.get_parameter('go_zero_duration').value
        
        # Calculate time step
        self.dt = 1.0 / self.update_rate
        
        # Debug counter - avoid log spam
        self.debug_counter = 0
        
        # Speed level settings (5 levels) - using speed scale factor
        # Actual speed = max_velocity * factor
        self.speed_levels = [
            {'name': '极慢', 'factor': 0.1},   # 10% 最大速度
            {'name': '慢', 'factor': 0.25},    # 25% 最大速度
            {'name': '中', 'factor': 0.5},     # 50% 最大速度
            {'name': '快', 'factor': 0.75},    # 75% 最大速度
            {'name': '最大', 'factor': 1.0},   # 100% 最大速度
        ]
        self.current_speed_level = 2  # Default medium speed (0-indexed)
        self.speed_factor = self.speed_levels[self.current_speed_level]['factor']
        
        # Button state tracking (for debouncing)
        self.last_a_button = 0
        self.last_b_button = 0
        self.last_x_button = 0
        self.last_y_button = 0
        self.is_going_home = False  # Whether currently returning to home/zero
        self.home_start_time = None  # Home return start time
        self.home_timeout = 15.0  # Home return timeout (seconds)
        
        # Zero-torque (gravity compensation) mode
        self.zero_torque_mode = False
        self.zero_torque_client = self.create_client(
            SetBool, '/el_a3/set_zero_torque_mode')
        
        # Master-slave teleoperation mode state
        self.master_slave_mode = False
        self.entering_master_slave = False  # Switching in progress (go-to-zero process)
        self.last_menu_button = 0
        
        # Slave arm ROS2 interface
        # Use same BEST_EFFORT QoS as main arm trajectory publisher to match arm_controller subscriber
        slave_trajectory_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.slave_joint_state = None
        self.slave_joint_sub = self.create_subscription(
            JointState, slave_joint_state_topic,
            self.slave_joint_state_callback, 10)
        self.slave_trajectory_pub = self.create_publisher(
            JointTrajectory, slave_trajectory_topic, slave_trajectory_qos)
        
        # Master arm namespace interface (for master arm communication in multi_arm mode)
        self.master_joint_state_ms = None  # Master-slave mode dedicated
        self.master_joint_sub_ms = self.create_subscription(
            JointState, master_joint_state_topic,
            self.master_joint_state_ms_callback, 10)
        self.master_trajectory_pub_ms = self.create_publisher(
            JointTrajectory, master_trajectory_topic, slave_trajectory_qos)
        self.master_zero_torque_client_ms = self.create_client(
            SetBool, master_zero_torque_service)
        
        # Master-slave mode joint names (no prefix, matches URDF; arms distinguished by topic namespace)
        self.slave_joint_names = [f"L{i}_joint" for i in range(1, 7)]
        self.master_joint_names_ms = [f"L{i}_joint" for i in range(1, 7)]
        
        # D-pad controls gripper motor (ID=7)
        self.last_dpad_up = 0
        self.last_dpad_down = 0
        self.gripper_torque = 0.0  # Current gripper torque
        self.gripper_torque_step = 1.0  # Torque increment per button press (Nm)
        self.gripper_motor_id = 7
        
        # Initialize CAN socket for direct gripper command sending
        self.can_socket = None
        self.gripper_enabled = False
        try:
            self.can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.can_socket.bind(('can0',))
            self.get_logger().info('CAN socket 初始化成功，已启用夹爪控制')
            # Enable gripper motor
            self.enable_gripper_motor()
        except Exception as e:
            self.get_logger().warn(f'CAN socket 初始化失败，已禁用夹爪控制：{e}')
        
        # Motor 7 (gripper) parameter ranges for MIT mode
        self.P_MIN, self.P_MAX = -12.57, 12.57
        self.V_MIN, self.V_MAX = -50.0, 50.0
        self.KP_MIN, self.KP_MAX = 0.0, 500.0
        self.KD_MIN, self.KD_MAX = 0.0, 5.0
        self.T_MIN, self.T_MAX = -12.0, 12.0
        
        # Motor 7 master-slave state
        self.motor7_position = 0.0
        self.motor7_valid = False
        
        # CAN-based slave arm direct control parameters (all 7 motors)
        self.slave_follow_kp = 80.0  # MIT mode Kp for position follow
        self.slave_follow_kd = 2.0   # MIT mode Kd for velocity damping
        # Motor direction mapping: joint_pos * direction = motor_pos (from URDF ros2_control config)
        # Motors 1-6 (L1-L6 joints), Motor 7 (gripper, no direction inversion needed)
        self.motor_directions = [-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0]  # 7 motors
        self._slave_can_going_zero = False
        self._slave_zero_start_positions = [0.0] * 7  # Motors 1-7 (in motor coordinate frame)
        self._slave_zero_frame = 0
        self._slave_zero_total_frames = 0
        
        # Set can0 socket to non-blocking for Motor 7 feedback reading
        if self.can_socket is not None:
            self.can_socket.setblocking(False)
        
        # CAN1 socket for Motor 7 slave follow (only if master-slave enabled)
        self.can1_socket = None
        if self.master_slave_enabled:
            try:
                self.can1_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                self.can1_socket.bind(('can1',))
                self.can1_socket.setblocking(False)
                self._enable_motor_on_socket(self.can1_socket, self.gripper_motor_id)
                self.get_logger().info('CAN1 socket 初始化成功（从臂 Motor 7 跟随）')
            except Exception as e:
                self.get_logger().warn(f'CAN1 socket 初始化失败，从臂 Motor 7 跟随不可用：{e}')
        
        # Input smoothing filter state (exponential moving average)
        self.smoothed_vx = 0.0  # Smoothed X velocity
        self.smoothed_vy = 0.0  # Smoothed Y velocity
        self.smoothed_vz = 0.0  # Smoothed Z velocity
        self.smoothed_vroll = 0.0  # Smoothed Roll angular velocity
        self.smoothed_vpitch = 0.0  # Smoothed Pitch angular velocity
        self.smoothed_vyaw = 0.0  # Smoothed Yaw angular velocity
        
        # Joint output smoothing state
        self.smoothed_joint_positions = None  # Smoothed joint positions
        
        # Accumulated deltas (for accumulating input during motion)
        self.pending_dx = 0.0
        self.pending_dy = 0.0
        self.pending_dz = 0.0
        self.pending_droll = 0.0
        self.pending_dpitch = 0.0
        self.pending_dyaw = 0.0
        
        # Subscribe to controller input
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Target pose publisher
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            '/target_pose',
            10
        )
        
        # Cartesian path service client
        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )
        
        # IK service client - for fast control
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )
        
        # Direct joint trajectory publishing - for fast control
        # Use BEST_EFFORT QoS to match arm_controller subscription settings
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
        
        # Joint trajectory Action client - for direct trajectory sending
        self.follow_joint_trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        # Subscribe to current joint states
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Joint names
        self.joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']
        
        # [Debug] IK solution publisher - for Foxglove tracking
        self.ik_solution_pub = self.create_publisher(
            JointState,
            '/debug/ik_solution',
            10
        )
        
        # [Debug] Raw IK solution publisher (before processing) - for comparison
        self.ik_raw_solution_pub = self.create_publisher(
            JointState,
            '/debug/ik_raw_solution',
            10
        )
        
        # IK request state tracking (for fast IK mode)
        self.pending_ik_request = False  # Whether there is a pending IK request
        self.last_ik_joint_positions = None  # Cache of last IK joint positions
        
        # Execute trajectory Action client
        self.execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'
        )
        
        # MoveGroup Action client (for returning to home)
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )
        
        # State variables
        self.current_joy = None
        self.target_pose = None
        self.is_moving = False
        self.last_update_time = self.get_clock().now()
        self.pose_initialized = False  # Whether pose is properly initialized
        self.last_sent_pose = None  # Last sent pose, for change detection
        self.min_move_threshold = 0.00005  # Minimum move threshold (meters)
        self.move_start_time = None  # Motion start time, for timeout detection
        self.move_timeout = 2.0  # Motion timeout (seconds)
        
        # Wait for services
        if self.use_fast_ik_mode:
            self.get_logger().info('等待 IK 服务（快速模式）...')
            self.ik_client.wait_for_service()
            self.get_logger().info('IK 服务已连接：使用 50Hz 快速 IK 控制模式')
        else:
            self.get_logger().info('等待笛卡尔路径服务...')
            self.cartesian_path_client.wait_for_service()
        self.get_logger().info('等待轨迹执行服务...')
        self.execute_trajectory_client.wait_for_server()
        self.get_logger().info('等待 MoveGroup 服务...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('服务已连接')
        
        # [Auto home on startup]
        # 1. Wait for joint states to be available (ensure motor positions are read)
        self.get_logger().info('等待关节状态（读取电机位置）...')
        self.wait_for_joint_states()
        
        # 2. Print current joint positions
        if self.current_joint_state:
            self.get_logger().info('已读取当前电机位置：')
            for i, name in enumerate(self.current_joint_state.name):
                if i < len(self.current_joint_state.position):
                    self.get_logger().info(f'  {name}: {self.current_joint_state.position[i]:.4f} rad')
        
        # 3. Automatically move to zero position
        self.get_logger().info('正在规划运动到零位...')
        self.startup_go_zero()
        
        # Create timer for periodic target pose update
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_callback)
        
        self.get_logger().info('Xbox 手柄控制节点已启动')
        self.get_logger().info(f'规划组：{self.planning_group}')
        self.get_logger().info(f'末端执行器：{self.end_effector_frame}')
        self.get_logger().info(f'更新频率：{self.update_rate} Hz（dt={self.dt*1000:.1f}ms）')
        self.get_logger().info(f'最大线速度：{self.max_linear_velocity} m/s')
        self.get_logger().info(f'最大角速度：{self.max_angular_velocity} rad/s')
        self.get_logger().info(f'关节平滑：alpha={self.joint_smoothing_alpha}，max_vel={self.max_joint_velocity} rad/s')
        if self.use_fast_ik_mode:
            self.get_logger().info('控制模式：快速 IK 模式（50Hz 笛卡尔控制）')
        else:
            self.get_logger().info('控制模式：传统路径规划模式')
        self.get_logger().info('=== 控制说明 ===')
        self.get_logger().info('左摇杆：XY 平移 | LT/RT：Z 平移')
        self.get_logger().info('右摇杆：Yaw/Roll | LB/RB：Pitch')
        self.get_logger().info('A：切换速度档 | B：回 home | X：回零位')
        self.get_logger().info('Y：切换重力补偿模式（手动示教/拖动）')
        self.get_logger().info('Menu：切换主从遥操作（can0 主 / can1 从）')
        self.log_speed_level()
        
    def sync_current_pose(self):
        """Sync current end effector pose from TF as target pose starting point
        
        Each call gets the latest actual pose, ensuring incremental motion
        is based on the correct starting point.
        
        Returns:
            bool: True on success, False on failure
        """
        try:
            # Try to get TF transform (with shorter timeout)
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.end_effector_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)  # Shorter timeout for faster response
            )
            
            # Update target pose to current actual pose every time
            self.target_pose = PoseStamped()
            self.target_pose.header.frame_id = self.base_frame
            self.target_pose.pose.position.x = transform.transform.translation.x
            self.target_pose.pose.position.y = transform.transform.translation.y
            self.target_pose.pose.position.z = transform.transform.translation.z
            self.target_pose.pose.orientation = transform.transform.rotation
            
            if not self.pose_initialized:
                self.get_logger().info(f'Initial pose sync: x={self.target_pose.pose.position.x:.4f}, '
                                     f'y={self.target_pose.pose.position.y:.4f}, '
                                     f'z={self.target_pose.pose.position.z:.4f}')
                # [Key] If IK seed is preset (e.g., from home/zero callbacks), keep it.
                # Otherwise initialize from current joint state.
                if self.last_ik_joint_positions is None:
                    # Only initialize from joint_states when IK seed is unset
                    if self.current_joint_state:
                        seed_positions = []
                        for joint_name in self.joint_names:
                            if joint_name in self.current_joint_state.name:
                                idx = self.current_joint_state.name.index(joint_name)
                                seed_positions.append(self.current_joint_state.position[idx])
                            else:
                                seed_positions.append(0.0)  # Default
                        self.last_ik_joint_positions = seed_positions
                        self.smoothed_joint_positions = list(seed_positions)
                        self.last_joint_velocities = [0.0] * len(seed_positions)
                        self.get_logger().info(f'IK 种子已从 joint_states 初始化：{[f"{p:.3f}" for p in self.last_ik_joint_positions]}')
                        self.ik_seed_just_initialized = True
                        self.consecutive_ik_rejects = 0
                else:
                    # IK seed already preset (from home/zero callbacks); only sync Cartesian pose
                    self.get_logger().info(f'使用预设 IK 种子：{[f"{p:.3f}" for p in self.last_ik_joint_positions]}')
                self.pose_initialized = True
            
            return True
            
        except Exception as e:
            self.get_logger().debug(f'TF 查询失败：{str(e)}')
            return False
            
    def joint_state_callback(self, msg):
        """Joint state callback"""
        self.current_joint_state = msg
    
    def slave_joint_state_callback(self, msg):
        """Slave joint state callback (for master-slave mode)"""
        self.slave_joint_state = msg
    
    def master_joint_state_ms_callback(self, msg):
        """Master namespace joint state callback (for master-slave mode)"""
        self.master_joint_state_ms = msg
    
    def wait_for_joint_states(self, timeout_sec=10.0):
        """Wait for joint states (ensure motor positions are read)"""
        import time
        start_time = time.time()
        while self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout_sec:
                self.get_logger().warn('等待关节状态超时！将使用默认位置')
                return False
        return True
    
    def startup_go_zero(self):
        """Move to zero position on startup (sync), then enable collision checking"""
        import time
        
        self.is_going_home = True
        self.home_start_time = self.get_clock().now()
        
        # Wait for MoveGroup service to be fully ready (including planning scene)
        self.get_logger().info('等待 MoveGroup 服务完全就绪...')
        time.sleep(3.0)  # Wait for MoveGroup initialization
        
        # Retry mechanism
        max_retries = 3
        for retry in range(max_retries):
            # Create MoveGroup request
            goal_msg = MoveGroup.Goal()
            goal_msg.request.group_name = self.planning_group
            goal_msg.request.num_planning_attempts = 10
            goal_msg.request.allowed_planning_time = 10.0
            goal_msg.request.max_velocity_scaling_factor = 0.15  # Slower motion on startup for safety
            goal_msg.request.max_acceleration_scaling_factor = 0.15
            
            # Set target joint positions (zero position)
            # Zero position: all joints to zero
            joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']
            joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # All joints to zero
            
            constraints = Constraints()
            for name, value in zip(joint_names, joint_values):
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
            
            # Send request and wait for result
            self.get_logger().info(f'发送零位运动请求...（第 {retry + 1}/{max_retries} 次）')
            send_goal_future = self.move_group_client.send_goal_async(goal_msg)
            
            # Wait for goal acceptance
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
            goal_handle = send_goal_future.result()
            
            if goal_handle and goal_handle.accepted:
                self.get_logger().info('零位请求已接受，正在低速执行...')
                
                # Wait for execution to complete
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
                
                result = result_future.result()
                self.is_going_home = False
                self.home_start_time = None
                
                # Check result
                try:
                    if result and result.result and result.result.error_code.val == result.result.error_code.SUCCESS:
                        self.get_logger().info('✓ 已低速移动到零位！')
                        # [Improvement] Use zero position as starting point to avoid TF drift
                        zero_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                        self.last_ik_joint_positions = zero_joint_positions
                        self.smoothed_joint_positions = list(zero_joint_positions)
                        self.last_joint_velocities = [0.0] * 6
                        self.ik_seed_just_initialized = True
                        self.consecutive_ik_rejects = 0
                        self.pose_initialized = False  # Still need Cartesian pose from TF
                        self.get_logger().info('IK 种子已设置为零位')
                        # Enable collision checking after successful motion
                        if self.enable_collision_check:
                            self.collision_check_active = True
                            self.get_logger().info('✓ 已启用碰撞检测')
                        return True
                    else:
                        error_code = 'unknown'
                        if result and result.result and hasattr(result.result, 'error_code'):
                            error_code = result.result.error_code.val
                        self.get_logger().warn(f'零位执行失败：error_code={error_code}')
                        # Reset state on failure
                        self.pose_initialized = False
                        self.smoothed_joint_positions = None
                        self.last_ik_joint_positions = None
                except Exception as e:
                    self.get_logger().warn(f'解析零位结果失败：{e}')
                    self.pose_initialized = False
                    self.smoothed_joint_positions = None
                    self.last_ik_joint_positions = None
            else:
                self.get_logger().warn('零位请求被拒绝，正在重试...')
                time.sleep(2.0)  # Wait before retry
        
        self.get_logger().error('多次尝试后零位仍失败。请手动控制机械臂。')
        self.is_going_home = False
        self.home_start_time = None
        self.pose_initialized = False
        self.smoothed_joint_positions = None
        self.last_ik_joint_positions = None
        return False
    
    def joy_callback(self, msg):
        """Controller input callback"""
        self.current_joy = msg
        
        # Debug mode: periodically print raw input
        if self.debug_input:
            self.debug_counter += 1
            if self.debug_counter >= 40:  # Print every 2s (assumes 20Hz)
                self.debug_counter = 0
                if len(msg.axes) >= 6:
                    self.get_logger().info(
                        f'摇杆原始值：LX={msg.axes[0]:.3f} LY={msg.axes[1]:.3f} '
                        f'LT={msg.axes[2]:.3f} RX={msg.axes[3]:.3f} '
                        f'RY={msg.axes[4]:.3f} RT={msg.axes[5]:.3f}'
                    )
        
    def apply_deadzone(self, value):
        """Apply stick deadzone"""
        if abs(value) < self.deadzone:
            return 0.0
        # Linearly remap values outside deadzone
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def apply_trigger_deadzone(self, trigger_value):
        """Apply trigger deadzone - for LT/RT"""
        # Trigger range is 1.0 (not pressed) to -1.0 (fully pressed)
        # Convert to 0.0 (not pressed) to 1.0 (fully pressed)
        normalized = (1.0 - trigger_value) / 2.0
        
        # Apply deadzone (triggers need a slightly larger deadzone)
        trigger_deadzone = self.deadzone * 1.5  # Larger deadzone for triggers
        if normalized < trigger_deadzone:
            return 0.0
        # Linear mapping
        return (normalized - trigger_deadzone) / (1.0 - trigger_deadzone)
    
    def log_speed_level(self):
        """Log current speed level"""
        level = self.speed_levels[self.current_speed_level]
        actual_linear = self.max_linear_velocity * self.speed_factor
        actual_angular = self.max_angular_velocity * self.speed_factor
        self.get_logger().info(
            f'当前速度档位：{self.current_speed_level + 1}/5 [{level["name"]}] '
            f'(线速度={actual_linear*1000:.0f}mm/s，角速度={actual_angular:.2f}rad/s)'
        )
    
    def switch_speed_level(self):
        """Switch speed level"""
        self.current_speed_level = (self.current_speed_level + 1) % len(self.speed_levels)
        level = self.speed_levels[self.current_speed_level]
        self.speed_factor = level['factor']
        self.log_speed_level()
    
    def go_home(self):
        """Return to home position"""
        if self.is_moving or self.is_going_home:
            self.get_logger().warn('当前正在执行其他动作，请稍后再试')
            return
        
        self.is_going_home = True
        self.home_start_time = self.get_clock().now()
        self.get_logger().info('正在回到 home 位置...')
        
        # Create MoveGroup request
        goal_msg = MoveGroup.Goal()
        
        # Configure motion request
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        
        # Set target to named position "home"
        goal_msg.request.goal_constraints = []
        
        # Use joint constraints to define home position
        # Home position: L1=0, L2=45°, L3=-45°, L4=0, L5=0, L6=0
        joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']
        joint_values = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]  # Radians
        
        constraints = Constraints()
        for name, value in zip(joint_names, joint_values):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = value
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        goal_msg.request.goal_constraints = [constraints]
        
        # Set planning options
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3
        
        # Send request
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.home_response_callback)
    
    def home_response_callback(self, future):
        """Home response callback"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('home 请求被拒绝')
            self.is_going_home = False
            self.home_start_time = None
            return
        
        self.get_logger().info('home 请求已接受，正在执行...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.home_result_callback)
    
    def home_result_callback(self, future):
        """Home result callback"""
        result = future.result().result
        self.is_going_home = False
        self.home_start_time = None
        
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info('已回到 home 位置！')
            # [Improvement] Use home as start to avoid TF drift
            home_joint_positions = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]
            self.last_ik_joint_positions = home_joint_positions
            self.smoothed_joint_positions = list(home_joint_positions)
            self.last_joint_velocities = [0.0] * 6
            self.ik_seed_just_initialized = True
            self.consecutive_ik_rejects = 0
            # Still let sync_current_pose() get Cartesian pose (from TF)
            self.pose_initialized = False
            self.get_logger().info(f'IK 种子已设置为 home：{[f"{p:.3f}" for p in home_joint_positions]}')
        else:
            self.get_logger().warn(f'home 运动失败：error_code={result.error_code.val}')
            self.pose_initialized = False
            self.smoothed_joint_positions = None
            self.last_ik_joint_positions = None
    
    def go_zero(self):
        """Return to zero position (all joints zero)"""
        if self.is_moving or self.is_going_home:
            self.get_logger().warn('当前正在执行其他动作，请稍后再试')
            return
        
        self.is_going_home = True
        self.home_start_time = self.get_clock().now()
        self.get_logger().info('正在回到零位...')
        
        # Create MoveGroup request
        goal_msg = MoveGroup.Goal()
        
        # Configure motion request
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        
        # Set target to zero position
        # Zero position: all joints are 0
        joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']
        joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Radians
        
        constraints = Constraints()
        for name, value in zip(joint_names, joint_values):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = value
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        goal_msg.request.goal_constraints = [constraints]
        
        # Set planning options
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3
        
        # Send request
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.zero_response_callback)
    
    def zero_response_callback(self, future):
        """Zero response callback"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('零位请求被拒绝')
            self.is_going_home = False
            self.home_start_time = None
            return
        
        self.get_logger().info('零位请求已接受，正在执行...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.zero_result_callback)
    
    def zero_result_callback(self, future):
        """Zero result callback"""
        result = future.result().result
        self.is_going_home = False
        self.home_start_time = None
        
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info('已回到零位！')
            # [Improvement] Use zero as start to avoid TF drift
            zero_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.last_ik_joint_positions = zero_joint_positions
            self.smoothed_joint_positions = list(zero_joint_positions)
            self.last_joint_velocities = [0.0] * 6
            self.ik_seed_just_initialized = True
            self.consecutive_ik_rejects = 0
            # Still let sync_current_pose() get Cartesian pose (from TF)
            self.pose_initialized = False
            self.get_logger().info(f'IK 种子已设置为零位：{[f"{p:.3f}" for p in zero_joint_positions]}')
        else:
            self.get_logger().warn(f'零位运动失败：error_code={result.error_code.val}')
            self.pose_initialized = False
            self.smoothed_joint_positions = None
            self.last_ik_joint_positions = None
    
    def toggle_zero_torque_mode(self):
        """Toggle gravity compensation (zero torque) mode"""
        if not self.zero_torque_client.service_is_ready():
            self.get_logger().warn('零力矩/重力补偿服务不可用')
            return

        new_state = not self.zero_torque_mode

        if not new_state:
            # Before disabling: send current position to arm_controller to prevent jump
            self._sync_controller_to_current_position()

        # Call service
        req = SetBool.Request()
        req.data = new_state
        future = self.zero_torque_client.call_async(req)
        future.add_done_callback(
            lambda f: self._zero_torque_response(f, new_state))

    def _sync_controller_to_current_position(self):
        """Send current joint positions to arm_controller to avoid mode-switch jumps"""
        if self.current_joint_state is None:
            return
        positions = []
        for name in self.joint_names:
            if name in self.current_joint_state.name:
                idx = self.current_joint_state.name.index(name)
                positions.append(self.current_joint_state.position[idx])
            else:
                positions.append(0.0)
        # Send a short trajectory to update arm_controller state immediately
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=0, nanosec=50_000_000)  # 50ms
        msg.points = [point]
        self.joint_trajectory_pub.publish(msg)

    def _zero_torque_response(self, future, new_state):
        """Zero torque service callback"""
        try:
            result = future.result()
            if result.success:
                self.zero_torque_mode = new_state
                if new_state:
                    self.get_logger().info('>>> 已开启重力补偿：机械臂可手动拖动 <<<')
                else:
                    # Restore control: resync pose and IK seed
                    self.pose_initialized = False  # Trigger pose resync
                    self.last_ik_joint_positions = None  # Re-init IK seed from joint_states
                    self.get_logger().info('>>> 已关闭重力补偿：恢复控制器控制 <<<')
            else:
                self.get_logger().error(f'零力矩模式切换失败：{result.message}')
        except Exception as e:
            self.get_logger().error(f'零力矩服务调用异常：{e}')
        
    # ============ Master-slave teleoperation mode ============

    def toggle_master_slave_mode(self):
        """Toggle master-slave teleoperation mode (Menu button)"""
        if self.entering_master_slave:
            self.get_logger().warn('正在切换主从模式，请稍候')
            return
        
        if not self.master_slave_mode:
            # === Enable master-slave mode ===
            self.get_logger().info('>>> 正在启用主从遥操作模式 <<<')
            self.entering_master_slave = True
            
            # Check master joint states availability (slave uses direct CAN, no ROS topic needed)
            if self.master_joint_state_ms is None:
                self.get_logger().error('主臂关节状态不可用，无法启用主从模式')
                self.get_logger().error('请确认主臂 ros2_control 已启动')
                self.entering_master_slave = False
                return
            
            # Send zero trajectory to both arms
            self._move_both_to_zero()
            
            # Use timer to wait for zero motion then proceed
            delay = self.go_zero_duration + 0.5  # Extra 0.5s to ensure arrival
            self._ms_enable_timer = self.create_timer(
                delay, self._on_go_zero_done_enable)
        else:
            # === Disable master-slave mode ===
            self.get_logger().info('>>> 正在关闭主从遥操作模式 <<<')
            self.master_slave_mode = False
            self.entering_master_slave = True
            
            # Note: can1 motors 1-7 keep holding last position during go-zero phase
            # They will be released (zero torque) in _on_go_zero_done_disable()
            
            # First disable master gravity compensation (restore trajectory control)
            self._set_master_zero_torque(False)
            
            # Wait for gravity compensation to disable before zeroing
            self._ms_disable_pre_timer = self.create_timer(
                0.5, self._on_gravity_off_then_go_zero)

    def _on_gravity_off_then_go_zero(self):
        """Send zero trajectory after gravity compensation is off"""
        # Destroy this one-shot timer
        self._ms_disable_pre_timer.cancel()
        self.destroy_timer(self._ms_disable_pre_timer)
        
        # Re-read joint states and send zero trajectory
        self._move_both_to_zero()
        
        # Use timer to wait for zero motion
        delay = self.go_zero_duration + 0.5
        self._ms_disable_timer = self.create_timer(
            delay, self._on_go_zero_done_disable)

    def _on_go_zero_done_enable(self):
        """Enable master gravity compensation after zeroing (enable flow)"""
        # Destroy this one-shot timer
        self._ms_enable_timer.cancel()
        self.destroy_timer(self._ms_enable_timer)
        
        # Enable master gravity compensation (L1-L6)
        self._set_master_zero_torque(True)
        
        # Enable can0 Motor 7 for zero torque mode (draggable)
        if self.can_socket is not None:
            self._enable_motor_on_socket(self.can_socket, self.gripper_motor_id)
            self.get_logger().info('can0 Motor 7 已使能（零力矩模式）')
        
        # Re-ensure all can1 motors enabled (already enabled in _move_both_to_zero)
        if self.can1_socket is not None:
            for motor_id in range(1, 8):
                self._enable_motor_on_socket(self.can1_socket, motor_id)
            self.get_logger().info('can1 Motors 1-7 已确认使能（CAN 跟随模式）')
        
        self.motor7_valid = False
        self.master_slave_mode = True
        self.entering_master_slave = False
        self.get_logger().info('>>> 主从遥操作已启用（全 CAN 直接通信） <<<')
        self.get_logger().info('  can0（主臂）：L1-L6 重力补偿 + L7 零力矩可拖动')
        self.get_logger().info('  can1（从臂）：L1-L7 直接 CAN MIT 跟随')
        self.get_logger().info('  注意：请勿同时运行 arm2 的 ros2_control_node')
        self.get_logger().info('  再按一次 Menu 退出')

    def _on_go_zero_done_disable(self):
        """Restore normal control after zeroing (disable flow)"""
        # Destroy this one-shot timer
        self._ms_disable_timer.cancel()
        self.destroy_timer(self._ms_disable_timer)
        
        # Release all can1 motors (send zero torque)
        if self.can1_socket is not None:
            for motor_id in range(1, 8):
                self._send_mit_command(
                    self.can1_socket, motor_id,
                    0.0, 0.0, 0.0, 0.0, 0.0)
            self.get_logger().info('can1 Motors 1-7 已释放（零力矩）')
        
        self.entering_master_slave = False
        
        # Sync controller to zero, restore normal Xbox control
        self._sync_controller_to_current_position()
        self.pose_initialized = False
        self.last_ik_joint_positions = None
        self.smoothed_joint_positions = None
        self.get_logger().info('>>> 主从遥操作已关闭：恢复 Xbox 控制 <<<')

    def _set_master_zero_torque(self, enable):
        """Set master gravity compensation (namespaced service for master-slave)"""
        if not self.master_zero_torque_client_ms.service_is_ready():
            self.get_logger().warn(f'主臂零力矩服务不可用：{self.master_zero_torque_client_ms.srv_name}')
            return
        
        req = SetBool.Request()
        req.data = enable
        future = self.master_zero_torque_client_ms.call_async(req)
        future.add_done_callback(
            lambda f: self._master_zero_torque_response(f, enable))

    def _master_zero_torque_response(self, future, enable):
        """Master gravity compensation service callback"""
        try:
            result = future.result()
            if result.success:
                state_str = '已开启' if enable else '已关闭'
                self.get_logger().info(f'主臂重力补偿{state_str}')
            else:
                self.get_logger().error(f'主臂重力补偿切换失败：{result.message}')
        except Exception as e:
            self.get_logger().error(f'主臂重力补偿服务异常：{e}')

    def _move_both_to_zero(self):
        """Send slow zero trajectories to both arms"""
        duration = self.go_zero_duration
        
        # Master arm zeroing - use main joint_trajectory_pub (known working QoS)
        if self.current_joint_state is not None:
            master_positions = self._extract_joint_positions(
                self.current_joint_state, self.joint_names)
            if master_positions is not None:
                traj = self._generate_zero_trajectory(
                    master_positions, self.joint_names, duration)
                self.joint_trajectory_pub.publish(traj)
                self.get_logger().info(f'主臂（can0）已发送回零轨迹（{duration}s）')
            else:
                self.get_logger().warn('无法读取主臂关节位置')
        
        # Slave arm zeroing via direct CAN (all 7 motors)
        if self.can1_socket is not None:
            # Enable all motors 1-7 on can1 first
            for motor_id in range(1, 8):
                self._enable_motor_on_socket(self.can1_socket, motor_id)
            self.get_logger().info('can1 Motors 1-7 已使能（准备回零）')
            
            # Get L1-L6 start positions: prefer slave joint state, fallback to master
            slave_positions = None
            if self.slave_joint_state is not None:
                slave_positions = self._extract_joint_positions(
                    self.slave_joint_state, self.slave_joint_names)
            if slave_positions is None and self.master_joint_state_ms is not None:
                slave_positions = self._extract_joint_positions(
                    self.master_joint_state_ms, self.master_joint_names_ms)
                self.get_logger().info('从臂关节状态不可用，使用主臂位置作为近似起始')
            if slave_positions is None:
                slave_positions = [0.0] * 6
                self.get_logger().info('使用零位作为从臂起始位置')
            
            # Get Motor 7 start position from can0 feedback (already in motor frame)
            motor7_start = self.motor7_position if self.motor7_valid else 0.0
            
            # Convert L1-L6 from joint coordinate → motor coordinate frame
            motor_start_positions = [
                slave_positions[i] * self.motor_directions[i] for i in range(6)
            ]
            # All 7 motors start positions (in motor coordinate frame)
            self._slave_zero_start_positions = motor_start_positions + [motor7_start]
            self._slave_zero_frame = 0
            self._slave_zero_total_frames = int(duration * self.update_rate)
            self._slave_can_going_zero = True
            pos_str = ', '.join([f'{p:.3f}' for p in self._slave_zero_start_positions])
            self.get_logger().info(
                f'从臂（can1）CAN 直接回零（{duration}s），起始=[{pos_str}]')
        else:
            self.get_logger().warn('can1 socket 不可用，无法控制从臂回零')

    def _extract_joint_positions(self, joint_state, joint_names):
        """Extract joint positions by name from JointState message"""
        positions = []
        for name in joint_names:
            if name in joint_state.name:
                idx = joint_state.name.index(name)
                positions.append(joint_state.position[idx])
            else:
                return None  # Missing joint data
        return positions

    def _generate_zero_trajectory(self, current_positions, joint_names, duration=3.0, num_points=30):
        """Generate a smooth S-curve trajectory from current position to zero
        
        Uses cosine interpolation (S-curve) for smooth acceleration/deceleration.
        Start and end velocities are zero; intermediate velocities computed from derivatives.
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        n_joints = len(joint_names)
        
        for i in range(1, num_points + 1):
            t_frac = i / num_points
            # S-curve: alpha = 0.5 * (1 - cos(pi * t_frac))
            # Starts slow, accelerates, then decelerates to stop
            alpha = 0.5 * (1.0 - math.cos(math.pi * t_frac))
            
            point = JointTrajectoryPoint()
            point.positions = [c * (1.0 - alpha) for c in current_positions]
            
            # Compute velocity from S-curve derivative:
            # d(alpha)/dt = 0.5 * pi * sin(pi * t_frac) / duration
            if i < num_points:
                d_alpha = 0.5 * math.pi * math.sin(math.pi * t_frac) / duration
                point.velocities = [-c * d_alpha for c in current_positions]
            else:
                point.velocities = [0.0] * n_joints  # Final point: zero velocity
            
            t = duration * t_frac
            point.time_from_start = Duration(
                sec=int(t), nanosec=int((t % 1) * 1e9))
            trajectory.points.append(point)
        
        return trajectory

    def _master_slave_follow(self):
        """Master-slave follow: read master and send to slave via direct CAN (50Hz)
        All 7 motors use CAN MIT commands: can0 read → can1 write
        """
        if self.master_joint_state_ms is None:
            return
        
        # === L1-L6: Direct CAN MIT commands to can1 ===
        # Convert joint coordinate → motor coordinate: motor_pos = joint_pos * direction
        master_positions = self._extract_joint_positions(
            self.master_joint_state_ms, self.master_joint_names_ms)
        if master_positions is not None and self.can1_socket is not None:
            for i, pos in enumerate(master_positions):
                motor_id = i + 1  # Motor IDs 1-6
                motor_pos = pos * self.motor_directions[i]  # Joint → Motor frame
                self._send_mit_command(
                    self.can1_socket, motor_id,
                    motor_pos, 0.0, self.slave_follow_kp, self.slave_follow_kd, 0.0)
        
        # Periodic debug logging (every 2 seconds at 50Hz = every 100 frames)
        if not hasattr(self, '_ms_follow_count'):
            self._ms_follow_count = 0
        self._ms_follow_count += 1
        if self._ms_follow_count % 100 == 1:
            if master_positions is not None:
                pos_str = ', '.join([f'{p:.3f}' for p in master_positions])
                m7_str = f'{self.motor7_position:.3f}' if self.motor7_valid else 'N/A'
                self.get_logger().info(
                    f'[主从跟随-CAN] L1-L6=[{pos_str}] L7={m7_str} '
                    f'Kp={self.slave_follow_kp} Kd={self.slave_follow_kd}')
            else:
                self.get_logger().warn(
                    f'[主从跟随-CAN] master_positions=None | '
                    f'joint_names={self.master_joint_names_ms}')
        
        # === Motor 7 (gripper): can0 zero torque + can1 position follow ===
        # Read Motor 7 feedback from can0
        self._read_motor7_feedback()
        
        # can0 Motor 7: pure zero torque (Kp=0, Kd=0, torque=0) - draggable
        if self.can_socket is not None:
            self._send_mit_command(
                self.can_socket, self.gripper_motor_id,
                0.0, 0.0, 0.0, 0.0, 0.0)
        
        # can1 Motor 7: position follow
        if self.can1_socket is not None and self.motor7_valid:
            self._send_mit_command(
                self.can1_socket, self.gripper_motor_id,
                self.motor7_position, 0.0,
                self.slave_follow_kp, self.slave_follow_kd, 0.0)

    def _slave_can_go_zero_tick(self):
        """CAN-based slave arm go-zero: send one S-curve interpolated frame for all 7 motors"""
        if self.can1_socket is None or not self._slave_can_going_zero:
            return
        
        self._slave_zero_frame += 1
        if self._slave_zero_frame > self._slave_zero_total_frames:
            self._slave_can_going_zero = False
            self.get_logger().info('从臂（can1）CAN 回零完成')
            return
        
        # S-curve interpolation (same formula as _generate_zero_trajectory)
        t_frac = self._slave_zero_frame / self._slave_zero_total_frames
        alpha = 0.5 * (1.0 - math.cos(math.pi * t_frac))
        
        for i, start_pos in enumerate(self._slave_zero_start_positions):
            motor_id = i + 1  # Motors 1-7
            target_pos = start_pos * (1.0 - alpha)  # Interpolate to zero
            self._send_mit_command(
                self.can1_socket, motor_id,
                target_pos, 0.0, self.slave_follow_kp, self.slave_follow_kd, 0.0)

    def _enable_motor_on_socket(self, can_socket, motor_id):
        """Enable motor on specified CAN socket (comm type 3)"""
        host_can_id = 253  # 0xFD
        comm_type = 3
        can_id = (comm_type << 24) | (host_can_id << 8) | motor_id
        can_id |= 0x80000000  # Extended frame flag
        data = bytes(8)
        frame = struct.pack('=IB3x8s', can_id, 8, data)
        can_socket.send(frame)

    def _send_mit_command(self, can_socket, motor_id, position, velocity, kp, kd, torque):
        """Send MIT control mode command to motor via specified CAN socket"""
        try:
            pos_raw = self._float_to_uint16(position, self.P_MIN, self.P_MAX)
            vel_raw = self._float_to_uint16(velocity, self.V_MIN, self.V_MAX)
            kp_raw = self._float_to_uint16(kp, self.KP_MIN, self.KP_MAX)
            kd_raw = self._float_to_uint16(kd, self.KD_MIN, self.KD_MAX)
            torque_raw = self._float_to_uint16(torque, self.T_MIN, self.T_MAX)
            
            comm_type = 1
            can_id = (comm_type << 24) | (torque_raw << 8) | motor_id
            can_id |= 0x80000000  # Extended frame flag
            
            data = struct.pack('>HHHH', pos_raw, vel_raw, kp_raw, kd_raw)
            frame = struct.pack('=IB3x8s', can_id, 8, data)
            can_socket.send(frame)
        except Exception as e:
            self.get_logger().debug(f'MIT command 发送失败：{e}')

    def _float_to_uint16(self, x, x_min, x_max):
        """Float to uint16 conversion (MIT mode encoding)"""
        x = max(x_min, min(x_max, x))
        return int((x - x_min) * 65535.0 / (x_max - x_min))

    def _uint16_to_float(self, x_int, x_min, x_max):
        """uint16 to float conversion (MIT mode decoding)"""
        return x_min + (x_max - x_min) * x_int / 65535.0

    def _read_motor7_feedback(self):
        """Read Motor 7 feedback from can0 (non-blocking), update self.motor7_position"""
        if self.can_socket is None:
            return
        
        for _ in range(50):  # Read up to 50 frames to drain buffer
            try:
                frame_data = self.can_socket.recv(16)
                can_id_raw, dlc = struct.unpack_from('=IB', frame_data, 0)
                data = frame_data[8:16]
                
                can_id = can_id_raw & 0x1FFFFFFF  # Remove extended frame flag
                comm_type = (can_id >> 24) & 0x3F
                motor_id = (can_id >> 8) & 0xFF
                
                if comm_type == 2 and motor_id == self.gripper_motor_id:
                    pos_raw = (data[0] << 8) | data[1]
                    self.motor7_position = self._uint16_to_float(
                        pos_raw, self.P_MIN, self.P_MAX)
                    self.motor7_valid = True
            except BlockingIOError:
                break
            except Exception:
                break

    def update_callback(self):
        """Periodic update callback"""
        if self.current_joy is None:
            return
        
        joy = self.current_joy
        
        # Ensure indices are valid
        if len(joy.axes) < 6 or len(joy.buttons) < 6:
            return
        
        # A button: switch speed level (edge-triggered)
        a_button = joy.buttons[0]
        if a_button == 1 and self.last_a_button == 0:
            self.switch_speed_level()
        self.last_a_button = a_button
        
        # B button: go home (edge-triggered)
        b_button = joy.buttons[1]
        if b_button == 1 and self.last_b_button == 0:
            self.go_home()
        self.last_b_button = b_button
        
        # X button: go zero (edge-triggered)
        x_button = joy.buttons[2]
        if x_button == 1 and self.last_x_button == 0:
            self.go_zero()
        self.last_x_button = x_button
        
        # Y button: toggle gravity compensation (edge-triggered)
        y_button = joy.buttons[3]
        if y_button == 1 and self.last_y_button == 0:
            self.toggle_zero_torque_mode()
        self.last_y_button = y_button
        
        # Menu button: toggle master-slave teleop (edge-triggered)
        if len(joy.buttons) > 7 and self.master_slave_enabled:
            menu_button = joy.buttons[7]
            if menu_button == 1 and self.last_menu_button == 0:
                self.toggle_master_slave_mode()
            self.last_menu_button = menu_button
        
        # D-pad: control gripper motor (ID=7) torque (skip in master-slave mode)
        # Xbox D-pad: axes[6]=left/right, axes[7]=up/down (up=1, down=-1)
        if len(joy.axes) >= 8 and self.can_socket is not None and not self.master_slave_mode:
            dpad_vertical = joy.axes[7]  # Up=1, down=-1
            
            # D-pad up: increase torque
            dpad_up = 1 if dpad_vertical > 0.5 else 0
            if dpad_up == 1 and self.last_dpad_up == 0:
                self.gripper_torque = self.gripper_torque_step
                self.send_gripper_torque(self.gripper_torque)
                self.get_logger().info(f'夹爪力矩：+{self.gripper_torque:.2f} Nm')
            elif dpad_up == 0 and self.last_dpad_up == 1:
                # Send zero torque when released
                self.gripper_torque = 0.0
                self.send_gripper_torque(0.0)
            self.last_dpad_up = dpad_up
            
            # D-pad down: decrease torque
            dpad_down = 1 if dpad_vertical < -0.5 else 0
            if dpad_down == 1 and self.last_dpad_down == 0:
                self.gripper_torque = -self.gripper_torque_step
                self.send_gripper_torque(self.gripper_torque)
                self.get_logger().info(f'夹爪力矩：{self.gripper_torque:.2f} Nm')
            elif dpad_down == 0 and self.last_dpad_down == 1:
                # Send zero torque when released
                self.gripper_torque = 0.0
                self.send_gripper_torque(0.0)
            self.last_dpad_down = dpad_down
            
        # CAN-based slave go-zero tick (runs during entering_master_slave phase)
        if self._slave_can_going_zero:
            self._slave_can_go_zero_tick()
        elif self.entering_master_slave and self.can1_socket is not None:
            # Go-zero complete but still waiting for timer: hold can1 motors at zero
            for motor_id in range(1, 8):
                self._send_mit_command(
                    self.can1_socket, motor_id,
                    0.0, 0.0, self.slave_follow_kp, self.slave_follow_kd, 0.0)
        
        # In master-slave mode: follow master, skip normal controller input
        if self.master_slave_mode:
            self._master_slave_follow()
            return
        if self.entering_master_slave:
            return
        
        # Skip motion control in zero-torque mode (only when NOT in master-slave mode)
        if self.zero_torque_mode:
            return
        
        # If returning home/zero, skip normal motion control (with timeout protection)
        if self.is_going_home:
            # Timeout check to prevent permanent block
            if self.home_start_time is not None:
                elapsed = (self.get_clock().now() - self.home_start_time).nanoseconds / 1e9
                if elapsed > self.home_timeout:
                    self.get_logger().warn(f'home/零位超时（{elapsed:.1f}s），正在重置状态并恢复控制')
                    self.is_going_home = False
                    self.home_start_time = None
                    self.pose_initialized = False  # Resync pose
                else:
                    return
            else:
                return
            
        # Parse controller input -> Cartesian velocity mapping
        # Xbox mapping:
        # Left stick X: axes[0], Left stick Y: axes[1]
        # LT: axes[2], RT: axes[5] (range 1.0 to -1.0, pressed = -1.0)
        # Right stick X: axes[3], Right stick Y: axes[4]
        # LB: buttons[4], RB: buttons[5]
        # A: buttons[0], B: buttons[1]
        
        # ============ Cartesian velocity mapping ============
        # Stick input → velocity (m/s, rad/s) → position delta
        
        # Compute actual max speed at current level
        current_max_linear = self.max_linear_velocity * self.speed_factor
        current_max_angular = self.max_angular_velocity * self.speed_factor
        
        # Translation - swap and invert left stick XY
        raw_vx = -self.apply_deadzone(joy.axes[1]) * current_max_linear  # 左摇杆Y -> X速度 (m/s)
        raw_vy = -self.apply_deadzone(joy.axes[0]) * current_max_linear  # 左摇杆X -> Y速度 (m/s)
        
        # LT/RT control Z velocity
        lt = self.apply_trigger_deadzone(joy.axes[2])  # 转换为0-1范围并应用死区
        rt = self.apply_trigger_deadzone(joy.axes[5])  # 转换为0-1范围并应用死区
        raw_vz = (rt - lt) * current_max_linear  # RT up, LT down (m/s)
        
        # Rotation velocities
        raw_vyaw = self.apply_deadzone(joy.axes[3]) * current_max_angular   # 右摇杆X -> Yaw速度 (rad/s)
        raw_vroll = self.apply_deadzone(joy.axes[4]) * current_max_angular  # 右摇杆Y -> Roll速度 (rad/s)
        
        # LB/RB control pitch velocity
        raw_vpitch = (joy.buttons[5] - joy.buttons[4]) * current_max_angular  # RB positive, LB negative (rad/s)
        
        # Apply input smoothing filter (exponential moving average)
        alpha = self.input_smoothing_factor
        self.smoothed_vx = alpha * raw_vx + (1 - alpha) * self.smoothed_vx
        self.smoothed_vy = alpha * raw_vy + (1 - alpha) * self.smoothed_vy
        self.smoothed_vz = alpha * raw_vz + (1 - alpha) * self.smoothed_vz
        self.smoothed_vroll = alpha * raw_vroll + (1 - alpha) * self.smoothed_vroll
        self.smoothed_vpitch = alpha * raw_vpitch + (1 - alpha) * self.smoothed_vpitch
        self.smoothed_vyaw = alpha * raw_vyaw + (1 - alpha) * self.smoothed_vyaw
        
        # Velocity × dt = position delta
        dx = self.smoothed_vx * self.dt
        dy = self.smoothed_vy * self.dt
        dz = self.smoothed_vz * self.dt
        droll = self.smoothed_vroll * self.dt
        dpitch = self.smoothed_vpitch * self.dt
        dyaw = self.smoothed_vyaw * self.dt
        
        # Check for input based on velocity thresholds
        velocity_threshold = 0.001  # Below 1mm/s treated as no input
        angular_threshold = 0.01    # Below 0.01 rad/s treated as no input
        translation_threshold = velocity_threshold * self.dt
        rotation_threshold = angular_threshold * self.dt
        
        has_translation = abs(dx) > translation_threshold or abs(dy) > translation_threshold or abs(dz) > translation_threshold
        has_rotation = abs(dyaw) > rotation_threshold or abs(dpitch) > rotation_threshold or abs(droll) > rotation_threshold
        
        if not has_translation and not has_rotation:
            # [Change] Keep computing IK even without input, to keep debug topic updated
            # and observe IK stability at rest.
            if self.use_fast_ik_mode and self.pose_initialized:
                self.send_cartesian_goal()  # Keep IK computing current target pose
            return  # No valid input, skip target update
        
        # Accumulate deltas
        self.pending_dx += dx
        self.pending_dy += dy
        self.pending_dz += dz
        self.pending_droll += droll
        self.pending_dpitch += dpitch
        self.pending_dyaw += dyaw
        
        # [Key change] Sync pose only once, then accumulate deltas onto target pose
        # This avoids Z drift from tracking errors (e.g., sagging under gravity).
        if not self.pose_initialized:
            if not self.sync_current_pose():
                self.get_logger().warn('等待 TF 树就绪...')
                return  # TF not ready, skip this update
        
        # Use accumulated deltas
        dx = self.pending_dx
        dy = self.pending_dy
        dz = self.pending_dz
        droll = self.pending_droll
        dpitch = self.pending_dpitch
        dyaw = self.pending_dyaw
        
        # Clear accumulated deltas
        self.pending_dx = 0.0
        self.pending_dy = 0.0
        self.pending_dz = 0.0
        self.pending_droll = 0.0
        self.pending_dpitch = 0.0
        self.pending_dyaw = 0.0
            
        # Save last target to detect jumps
        if hasattr(self, 'last_target_x'):
            position_jump = math.sqrt(
                (self.target_pose.pose.position.x - self.last_target_x) ** 2 +
                (self.target_pose.pose.position.y - self.last_target_y) ** 2 +
                (self.target_pose.pose.position.z - self.last_target_z) ** 2
            )
            # Detect position jumps (warn if > 50mm)
            if position_jump > 0.05:
                self.get_logger().warn(f'检测到目标位姿跳变！距离={position_jump*1000:.1f}mm')
        
        # Debug: print actual inputs (only when there is input)
        if self.debug_input:
            self.get_logger().info(
                f'输入：dx={dx:.6f} dy={dy:.6f} dz={dz:.6f} '
                f'droll={droll:.6f} dpitch={dpitch:.6f} dyaw={dyaw:.6f}'
            )
            
        # Update target pose
        # Translation
        self.target_pose.pose.position.x += dx
        self.target_pose.pose.position.y += dy
        self.target_pose.pose.position.z += dz
        
        # Rotation - convert current quaternion to Euler
        q = self.target_pose.pose.orientation
        current_roll, current_pitch, current_yaw = euler_from_quaternion([
            q.x, q.y, q.z, q.w
        ])
        
        # Update Euler angles
        new_roll = current_roll + droll
        new_pitch = current_pitch + dpitch
        new_yaw = current_yaw + dyaw
        
        # Convert back to quaternion
        new_q = quaternion_from_euler(new_roll, new_pitch, new_yaw)
        self.target_pose.pose.orientation.x = new_q[0]
        self.target_pose.pose.orientation.y = new_q[1]
        self.target_pose.pose.orientation.z = new_q[2]
        self.target_pose.pose.orientation.w = new_q[3]
        
        # Save current target for next jump detection
        self.last_target_x = self.target_pose.pose.position.x
        self.last_target_y = self.target_pose.pose.position.y
        self.last_target_z = self.target_pose.pose.position.z
        
        # Publish target pose for visualization
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose_pub.publish(self.target_pose)
        
        # Send motion command
        self.send_cartesian_goal()
        
    def pose_changed_significantly(self):
        """Check if target pose changed significantly"""
        if self.last_sent_pose is None:
            return True
        
        # Compute position change
        dx = abs(self.target_pose.pose.position.x - self.last_sent_pose.pose.position.x)
        dy = abs(self.target_pose.pose.position.y - self.last_sent_pose.pose.position.y)
        dz = abs(self.target_pose.pose.position.z - self.last_sent_pose.pose.position.z)
        
        # If position change exceeds threshold, treat as significant change
        if dx > self.min_move_threshold or dy > self.min_move_threshold or dz > self.min_move_threshold:
            return True
        
        # Check orientation change (simplified: quaternion w component)
        dw = abs(self.target_pose.pose.orientation.w - self.last_sent_pose.pose.orientation.w)
        if dw > 0.01:  # About 1 degree
            return True
        
        return False
    
    def send_cartesian_goal(self):
        """Send Cartesian goal - use fast IK or path planning based on mode"""
        if self.use_fast_ik_mode:
            self.send_ik_goal()
        else:
            self.send_cartesian_path_goal()
    
    def send_ik_goal(self):
        """Compute joints via IK service and publish trajectory - 50Hz fast mode"""
        # Allow overriding old requests, but avoid queue buildup
        if self.pending_ik_request:
            return  # Previous IK request still processing
        
        # Debug log
        self.get_logger().debug('正在发送 IK 请求...')
        
        # Build IK request
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.planning_group
        request.ik_request.robot_state.is_diff = False  # Use full state, not diff
        
        # [Key fix] Use last successful IK solution as seed to keep continuity
        # This avoids pose jumps from gravity sag.
        if self.last_ik_joint_positions is not None:
            # Use last IK solution as seed
            seed_state = JointState()
            seed_state.name = self.joint_names
            seed_state.position = self.last_ik_joint_positions
            request.ik_request.robot_state.joint_state = seed_state
        elif self.current_joint_state:
            # First time: use current joint state
            request.ik_request.robot_state.joint_state = self.current_joint_state
        
        # Set target end-effector pose
        request.ik_request.pose_stamped.header.frame_id = self.base_frame
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = self.target_pose.pose
        
        # Set timeout (short to keep high rate)
        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = 10_000_000  # 10ms timeout
        
        # Set collision checking
        request.ik_request.avoid_collisions = self.collision_check_active
        
        # Call IK service asynchronously
        self.pending_ik_request = True
        future = self.ik_client.call_async(request)
        future.add_done_callback(self.ik_callback)
    
    def ik_callback(self, future):
        """IK completion callback - publish joint trajectory"""
        self.pending_ik_request = False
        
        # Skip if in master-slave mode or transitioning (avoid overriding zero trajectory)
        if self.master_slave_mode or self.entering_master_slave:
            return
        
        try:
            response = future.result()
            
            # Check IK success (SUCCESS = 1)
            if response.error_code.val != 1:
                # IK failed, target may be unreachable
                self.get_logger().debug(f'IK 失败：error_code={response.error_code.val}')
                return
            
            # Extract joint positions
            solution = response.solution.joint_state
            ik_positions = []
            
            for joint_name in self.joint_names:
                if joint_name in solution.name:
                    idx = solution.name.index(joint_name)
                    ik_positions.append(solution.position[idx])
                else:
                    return  # Missing joint data
            
            # [Debug] Publish raw IK solution (before any processing)
            self.publish_ik_raw_debug(ik_positions)
            
            # Check if IK solution change is too small or too large
            if self.last_ik_joint_positions is not None:
                max_diff = max(abs(ik_positions[i] - self.last_ik_joint_positions[i]) for i in range(len(ik_positions)))
                
                # Change too small, skip
                if max_diff < 0.0001:
                    return
                
                # [Singularity protection] Too large change -> reject IK solution
                # If IK seed just initialized, skip first-frame check to set baseline
                if max_diff > self.max_ik_jump_threshold:
                    if self.ik_seed_just_initialized:
                        # First frame: accept IK solution, clear flag
                        self.get_logger().info(f'IK 种子初始化后的首帧：接受 IK 解（diff={max_diff:.3f}rad）')
                        self.ik_seed_just_initialized = False
                    else:
                        self.consecutive_ik_rejects += 1
                        if self.consecutive_ik_rejects >= self.singularity_warning_count:
                            self.get_logger().warn(
                                f'疑似奇异区：IK 跳变={max_diff:.3f}rad，已保护 {self.consecutive_ik_rejects} 帧'
                            )
                        # [Auto recovery] Re-sync IK seed after 50 consecutive rejects
                        if self.consecutive_ik_rejects >= 50:
                            self.get_logger().warn('IK 解已连续拒绝 50+ 帧，自动重新同步位姿...')
                            self.pose_initialized = False
                            self.smoothed_joint_positions = None
                            self.last_ik_joint_positions = None
                            self.consecutive_ik_rejects = 0
                        # Reject this solution and keep last joint positions
                        return
                else:
                    # IK solution ok: reset counters and flags
                    if self.consecutive_ik_rejects > 0:
                        self.consecutive_ik_rejects = 0
                    if self.ik_seed_just_initialized:
                        self.ik_seed_just_initialized = False
            
            # [Key] Apply joint output smoothing
            # Avoid mechanical shocks from IK jumps
            target_positions = self.smooth_joint_positions(ik_positions)
            self.last_ik_joint_positions = target_positions
            
            # [Debug] Publish IK solution to /debug/ik_solution
            self.publish_ik_debug(target_positions)
            
            # Build and publish joint trajectory
            trajectory = JointTrajectory()
            trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory.joint_names = self.joint_names
            
            # Single-point trajectory - short time lets hardware smooth
            point = JointTrajectoryPoint()
            point.positions = target_positions
            point.velocities = [0.0] * 6  # Target velocity is 0
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 20_000_000  # 20ms (50Hz)
            
            trajectory.points = [point]
            
            # Publish trajectory directly to controller
            self.joint_trajectory_pub.publish(trajectory)
            
            # Count IK successes
            if not hasattr(self, 'ik_success_count'):
                self.ik_success_count = 0
            self.ik_success_count += 1
            if self.ik_success_count % 50 == 0:  # Log every 50
                self.get_logger().info(f'IK 控制：已发送 {self.ik_success_count} 条轨迹指令')
            
            # Update last sent pose (for change detection)
            self.last_sent_pose = copy.deepcopy(self.target_pose)
            
        except Exception as e:
            self.get_logger().error(f'IK 回调异常：{e}')
    
    def smooth_joint_positions(self, target_positions):
        """Smooth IK joint positions (critical damping + deadzone).
        
        Uses critical damping + position deadzone + asymmetric decel to avoid
        joint jumps and idle oscillations.
        
        Args:
            target_positions: Target joint positions from IK
            
        Returns:
            Smoothed joint positions
        """
        # Initialize on first call
        if self.smoothed_joint_positions is None:
            self.smoothed_joint_positions = list(target_positions)
            self.last_joint_velocities = [0.0] * len(target_positions)
            return target_positions
        
        # Ensure velocity array initialized
        if self.last_joint_velocities is None:
            self.last_joint_velocities = [0.0] * len(target_positions)
        
        smoothed = []
        new_velocities = []
        alpha = self.joint_smoothing_alpha
        max_delta_v = self.max_joint_acceleration * self.dt  # Max velocity change per cycle
        
        # [New] Position deadzone (below this is treated as reached)
        position_deadzone = 0.0005  # 0.5 mrad ≈ 0.03 deg
        
        for i in range(len(target_positions)):
            pos_error = target_positions[i] - self.smoothed_joint_positions[i]
            current_velocity = self.last_joint_velocities[i]
            
            # [Improvement 1] Position deadzone: zero when error and velocity are tiny
            if abs(pos_error) < position_deadzone and abs(current_velocity) < 0.01:
                smoothed.append(self.smoothed_joint_positions[i])
                new_velocities.append(0.0)
                continue
            
            # [Improvement 2] Critical damping: desired_velocity considers current velocity
            # PD-like: v_desired = Kp * pos_error - Kd * current_velocity
            # Here Kp is alpha/dt, Kd is damping coefficient.
            damping = 0.7  # Damping coefficient, ~critical damping
            
            # Desired velocity = position error term - damping term
            desired_velocity = alpha * pos_error / self.dt - damping * current_velocity
            
            # [Improvement 3] Asymmetric accel limit: allow 2x accel when decelerating
            velocity_change = desired_velocity - current_velocity
            is_decelerating = (current_velocity * desired_velocity < 0) or \
                              (abs(desired_velocity) < abs(current_velocity))
            
            if is_decelerating:
                effective_max_delta_v = max_delta_v * 2.0  # Double accel when decelerating
            else:
                effective_max_delta_v = max_delta_v
            
            if abs(velocity_change) > effective_max_delta_v:
                desired_velocity = current_velocity + effective_max_delta_v * (1 if velocity_change > 0 else -1)
            
            # Velocity limit
            if abs(desired_velocity) > self.max_joint_velocity:
                desired_velocity = self.max_joint_velocity * (1 if desired_velocity > 0 else -1)
            
            # Compute final position
            final_pos = self.smoothed_joint_positions[i] + desired_velocity * self.dt
            
            smoothed.append(final_pos)
            new_velocities.append(desired_velocity)
        
        self.smoothed_joint_positions = smoothed
        self.last_joint_velocities = new_velocities
        return smoothed
    
    def send_joint_positions(self, positions):
        """Send joint positions to controller directly (no IK)"""
        # Build and publish joint trajectory
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names
        
        # Single-point trajectory - short execution time
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.velocities = [0.0] * 6
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 20_000_000  # 20ms
        
        trajectory.points = [point]
        
        # Publish trajectory
        self.joint_trajectory_pub.publish(trajectory)
    
    def publish_ik_debug(self, positions):
        """Publish IK debug info to /debug/ik_solution (continuous, even without input)"""
        ik_debug_msg = JointState()
        ik_debug_msg.header.stamp = self.get_clock().now().to_msg()
        
        # [Key change] Use joint_states order if known to align indices in Foxglove
        if self.current_joint_state is not None:
            ik_debug_msg.name = self.current_joint_state.name
            
            # Build name-to-position map
            pos_map = {name: pos for name, pos in zip(self.joint_names, positions)}
            
            # Fill positions following joint_states order
            ordered_positions = []
            for name in ik_debug_msg.name:
                if name in pos_map:
                    ordered_positions.append(pos_map[name])
                else:
                    ordered_positions.append(0.0)  # Fallback
            ik_debug_msg.position = ordered_positions
        else:
            # Fallback: use default order
            ik_debug_msg.name = self.joint_names
            ik_debug_msg.position = list(positions)
            
        self.ik_solution_pub.publish(ik_debug_msg)
    
    def publish_ik_raw_debug(self, positions):
        """Publish raw IK solution (pre-processing) to /debug/ik_raw_solution for comparison"""
        ik_raw_msg = JointState()
        ik_raw_msg.header.stamp = self.get_clock().now().to_msg()
        
        if self.current_joint_state is not None:
            ik_raw_msg.name = self.current_joint_state.name
            pos_map = {name: pos for name, pos in zip(self.joint_names, positions)}
            ordered_positions = []
            for name in ik_raw_msg.name:
                if name in pos_map:
                    ordered_positions.append(pos_map[name])
                else:
                    ordered_positions.append(0.0)
            ik_raw_msg.position = ordered_positions
        else:
            ik_raw_msg.name = self.joint_names
            ik_raw_msg.position = list(positions)
            
        self.ik_raw_solution_pub.publish(ik_raw_msg)
    
    def send_cartesian_path_goal(self):
        """Plan Cartesian path with MoveIt (traditional mode) for RViz update and precision"""
        # Check if moving (with timeout)
        if self.is_moving:
            # Timeout check to avoid permanent block
            if self.move_start_time is not None:
                elapsed = (self.get_clock().now() - self.move_start_time).nanoseconds / 1e9
                if elapsed > self.move_timeout:
                    self.get_logger().warn(f'运动超时（{elapsed:.1f}s），正在重置状态')
                    self.is_moving = False
                else:
                    return
            else:
                return
        
        # Mark as moving
        self.is_moving = True
        self.move_start_time = self.get_clock().now()
        
        # Create Cartesian path request
        request = GetCartesianPath.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = self.base_frame
        request.group_name = self.planning_group
        request.link_name = self.end_effector_frame
        
        # Set waypoints - target only
        waypoint = Pose()
        waypoint.position = self.target_pose.pose.position
        waypoint.orientation = self.target_pose.pose.orientation
        request.waypoints = [waypoint]
        
        # Set parameters
        request.max_step = 0.01  # 10mm step
        request.jump_threshold = 0.0  # Disable jump detection
        request.avoid_collisions = self.collision_check_active  # Enable collision checking by state
        
        # Call service asynchronously
        future = self.cartesian_path_client.call_async(request)
        future.add_done_callback(self.cartesian_path_callback)
        
    def cartesian_path_callback(self, future):
        """Cartesian path planning callback"""
        try:
            response = future.result()
            
            if response.fraction < 0.9:
                self.is_moving = False
                return
            
            # Get planned trajectory
            trajectory = response.solution
            
            # Speed up trajectory execution
            self.scale_trajectory_speed(trajectory, 5.0)
            
            # Execute trajectory
            goal_msg = ExecuteTrajectory.Goal()
            goal_msg.trajectory = trajectory
            
            send_goal_future = self.execute_trajectory_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.execute_response_callback)
            
        except Exception as e:
            self.is_moving = False
    
    def scale_trajectory_speed(self, trajectory, speed_scale):
        """Scale trajectory speed - speed_scale > 1 faster, < 1 slower"""
        if speed_scale <= 0:
            return
        
        for point in trajectory.joint_trajectory.points:
            # Compute new time (speed_scale > 1 shorter time, < 1 longer)
            total_nsec = point.time_from_start.sec * 1e9 + point.time_from_start.nanosec
            new_total_nsec = total_nsec / speed_scale
            point.time_from_start.sec = int(new_total_nsec // 1e9)
            point.time_from_start.nanosec = int(new_total_nsec % 1e9)
            
            # Scale velocities (speed_scale > 1 higher)
            point.velocities = [v * speed_scale for v in point.velocities]
            
            # Scale accelerations (speed_scale > 1 higher)
            if point.accelerations:
                point.accelerations = [a * speed_scale * speed_scale for a in point.accelerations]
    
    def execute_response_callback(self, future):
        """Execute trajectory response callback"""
        try:
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().warn('轨迹执行请求被拒绝')
                self.is_moving = False
                return
                
            # Wait for execution result
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.execute_result_callback)
        except Exception as e:
            self.get_logger().error(f'执行响应回调异常：{e}')
            self.is_moving = False
        
    def execute_result_callback(self, future):
        """Execute result callback"""
        try:
            result = future.result().result
            self.is_moving = False
            
            if result.error_code.val == result.error_code.SUCCESS:
                # Update last sent pose
                self.last_sent_pose = copy.deepcopy(self.target_pose)
            else:
                self.get_logger().warn(f'运动失败：error_code={result.error_code.val}')
        except Exception as e:
            self.get_logger().error(f'执行结果回调异常：{e}')
            self.is_moving = False


    def enable_gripper_motor(self):
        """Enable gripper motor (ID=7)"""
        if self.can_socket is None:
            return False
        
        try:
            motor_id = self.gripper_motor_id  # 7
            host_can_id = 253  # 0xFD - host CAN ID
            comm_type = 3  # Communication type 3: enable motor
            
            # Build 29-bit extended CAN ID: (comm_type << 24) | (host_can_id << 8) | motor_id
            can_id = (comm_type << 24) | (host_can_id << 8) | motor_id
            can_id |= 0x80000000  # Set extended frame flag
            
            # Build 8-byte data (all zeros)
            data = bytes(8)
            
            # Build CAN frame
            frame = struct.pack('=IB3x8s', can_id, 8, data)
            
            # Send
            self.can_socket.send(frame)
            self.gripper_enabled = True
            self.get_logger().info(f'夹爪电机已使能（ID={motor_id}）')
            return True
            
        except Exception as e:
            self.get_logger().error(f'使能夹爪电机失败：{e}')
            return False

    def send_gripper_torque(self, torque):
        """
        Send torque command to gripper motor (ID=7) on can0
        position=0, velocity=0, kp=0, kd=0, torque=specified value
        """
        if self.can_socket is None:
            return False
        try:
            self._send_mit_command(
                self.can_socket, self.gripper_motor_id,
                0.0, 0.0, 0.0, 0.0, torque)
            return True
        except Exception as e:
            self.get_logger().error(f'发送夹爪指令失败：{e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = XboxTeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

