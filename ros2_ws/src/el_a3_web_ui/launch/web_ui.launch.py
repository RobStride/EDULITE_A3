"""
EL-A3 Web UI Launch File

Starts the web server for robot control and monitoring.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='Host address to bind the web server'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='5000',
        description='Port number for the web server'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    )
    
    # Get arguments
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    debug = LaunchConfiguration('debug')
    
    # Web server node
    # Since the web server is a Python script with Flask,
    # we use ExecuteProcess to run it
    web_server = ExecuteProcess(
        cmd=[
            'python3', '-m', 'el_a3_web_ui.web_server',
            '--host', host,
            '--port', port
        ],
        name='el_a3_web_ui',
        output='screen'
    )
    
    return LaunchDescription([
        host_arg,
        port_arg,
        debug_arg,
        web_server
    ])
