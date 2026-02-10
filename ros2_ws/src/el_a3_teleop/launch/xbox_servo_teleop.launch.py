#!/usr/bin/env python3
"""
Xbox Controller Servo Control Launch File
Launches joy node and xbox_servo_teleop node
This version uses MoveIt Servo for real-time control
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('el_a3_teleop'),
            'config',
            'xbox_servo_teleop.yaml'
        ]),
        description='Path to config file'
    )
    
    # Joy node - reads controller input
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
    
    # Xbox Servo Teleop node - processes controller input and publishes Twist commands
    xbox_servo_teleop_node = Node(
        package='el_a3_teleop',
        executable='xbox_servo_teleop_node',
        name='xbox_servo_teleop_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )
    
    return LaunchDescription([
        device_arg,
        config_file_arg,
        joy_node,
        xbox_servo_teleop_node,
    ])





