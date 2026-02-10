#!/usr/bin/env python3
"""
Xbox Controller Launch File
Launches joy node and xbox_teleop node
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
            'xbox_teleop.yaml'
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
    
    # Xbox Teleop node - processes controller input and controls robot arm
    xbox_teleop_node = Node(
        package='el_a3_teleop',
        executable='xbox_teleop_node',
        name='xbox_teleop_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )
    
    return LaunchDescription([
        device_arg,
        config_file_arg,
        joy_node,
        xbox_teleop_node,
    ])





