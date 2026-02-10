#!/usr/bin/env python3
"""
Joy-Con IMU Teleoperation Launch File

Launch modes:
  Simulation mode:
    ros2 launch el_a3_teleop joycon_imu_teleop.launch.py

  Real hardware:
    ros2 launch el_a3_teleop joycon_imu_teleop.launch.py use_mock_hardware:=false can_interface:=can0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Get package paths
    teleop_pkg = FindPackageShare('el_a3_teleop')
    moveit_pkg = FindPackageShare('el_a3_moveit_config')
    
    # Declare launch arguments
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='true',
        description='Use mock hardware for simulation'
    )
    
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name'
    )
    
    joy_device_arg = DeclareLaunchArgument(
        'joy_device',
        default_value='/dev/input/js0',
        description='Joy-Con device path'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    )
    
    # Get arguments
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    can_interface = LaunchConfiguration('can_interface')
    joy_device = LaunchConfiguration('joy_device')
    debug = LaunchConfiguration('debug')
    
    # Config file path
    teleop_config = PathJoinSubstitution([
        teleop_pkg, 'config', 'joycon_imu_teleop.yaml'
    ])
    
    # ==================== Simulation mode ====================
    # Start MoveIt demo (includes mock hardware)
    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([moveit_pkg, 'launch', 'demo.launch.py'])
        ]),
        condition=IfCondition(use_mock_hardware)
    )
    
    # ==================== Real hardware mode ====================
    # Start real hardware MoveIt
    moveit_real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([moveit_pkg, 'launch', 'robot.launch.py'])
        ]),
        launch_arguments={
            'can_interface': can_interface
        }.items(),
        condition=UnlessCondition(use_mock_hardware)
    )
    
    # ==================== Joy node ====================
    # Publish controller button data
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_name': '',
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        remappings=[
            ('joy', 'joy'),
        ],
    )
    
    # ==================== Joy-Con IMU driver node ====================
    # Read Joy-Con IMU data and publish as sensor_msgs/Imu
    joycon_imu_driver_node = Node(
        package='el_a3_teleop',
        executable='joycon_imu_driver',
        name='joycon_imu_driver',
        parameters=[{
            'device_path': joy_device,
            'publish_rate': 60.0,
        }],
        output='screen',
    )
    
    # ==================== IMU teleoperation node ====================
    joycon_imu_teleop_node = Node(
        package='el_a3_teleop',
        executable='joycon_imu_teleop',
        name='joycon_imu_teleop_node',
        parameters=[teleop_config, {'debug_mode': debug}],
        output='screen',
    )
    
    return LaunchDescription([
        # Argument declarations
        use_mock_hardware_arg,
        can_interface_arg,
        joy_device_arg,
        debug_arg,
        
        # MoveIt launch
        moveit_demo_launch,
        moveit_real_launch,
        
        # Joy-Con nodes
        joy_node,
        joycon_imu_driver_node,
        
        # Teleoperation node
        joycon_imu_teleop_node,
    ])
