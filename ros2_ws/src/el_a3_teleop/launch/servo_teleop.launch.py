#!/usr/bin/env python3
"""
Xbox Teleoperation Launch File using MoveIt Servo
Servo uses Jacobian matrix for incremental control, avoiding IK solution jumping issues
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    can_interface = LaunchConfiguration('can_interface')
    
    # Get package paths
    el_a3_description_share = get_package_share_directory('el_a3_description')
    el_a3_moveit_config_share = get_package_share_directory('el_a3_moveit_config')
    el_a3_teleop_share = get_package_share_directory('el_a3_teleop')
    
    # Config file path
    servo_config = os.path.join(el_a3_moveit_config_share, 'config', 'servo_config.yaml')
    
    # Argument declarations
    declare_mock_hardware = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='Use mock hardware'
    )
    
    declare_can_interface = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name'
    )
    
    # Include base launch file (el_a3_control.launch.py)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(el_a3_description_share, 'launch', 'el_a3_control.launch.py')
        ),
        launch_arguments={
            'use_mock_hardware': use_mock_hardware,
            'can_interface': can_interface,
        }.items(),
    )
    
    # Include MoveIt launch file
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(el_a3_moveit_config_share, 'launch', 'moveit.launch.py')
        ),
    )
    
    # MoveIt Servo node
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        output='screen',
        parameters=[
            servo_config,
            {'use_sim_time': False},
        ],
    )
    
    # Xbox Servo teleoperation node - delayed start to ensure Servo is ready
    xbox_servo_node = TimerAction(
        period=8.0,  # Delay 8 seconds before starting
        actions=[
            Node(
                package='el_a3_teleop',
                executable='xbox_servo_node',
                name='xbox_servo_node',
                output='screen',
                parameters=[{
                    'update_rate': 50.0,
                    'linear_scale': 0.15,
                    'angular_scale': 0.5,
                    'deadzone': 0.15,
                    'debug_input': False,
                }],
            )
        ]
    )
    
    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
    )
    
    return LaunchDescription([
        declare_mock_hardware,
        declare_can_interface,
        bringup_launch,
        moveit_launch,
        joy_node,
        # Servo node delayed start
        TimerAction(
            period=5.0,
            actions=[servo_node]
        ),
        xbox_servo_node,
    ])

