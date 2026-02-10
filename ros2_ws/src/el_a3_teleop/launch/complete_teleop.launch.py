#!/usr/bin/env python3
"""
Complete Xbox Controller Launch File
Integrated launch for robot, MoveIt, Servo and controller

Usage:
  # Simulation mode
  ros2 launch el_a3_teleop complete_teleop.launch.py use_mock_hardware:=true
  
  # Real hardware mode
  ros2 launch el_a3_teleop complete_teleop.launch.py use_mock_hardware:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare launch arguments
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='Use mock hardware for simulation'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    # Include MoveIt launch file
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('el_a3_moveit_config'),
                'launch',
                'robot.launch.py'
            ])
        ]),
        launch_arguments={
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items()
    )
    
    # MoveIt Servo node
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('el_a3_teleop'),
                'config',
                'moveit_servo_config.yaml'
            ])
        ],
        output='screen'
    )
    
    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': LaunchConfiguration('device'),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )
    
    # Xbox Servo Teleop node
    xbox_servo_teleop_node = Node(
        package='el_a3_teleop',
        executable='xbox_servo_teleop_node',
        name='xbox_servo_teleop_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('el_a3_teleop'),
                'config',
                'xbox_servo_teleop.yaml'
            ])
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_mock_hardware_arg,
        use_rviz_arg,
        device_arg,
        moveit_launch,
        servo_node,
        joy_node,
        xbox_servo_teleop_node,
    ])





