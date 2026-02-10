"""
EL-A3 Master-Slave Teleoperation Launch File

Supports multiple launch modes:
1. Config file mode (recommended): supports complex one-to-many and many-to-many mappings
2. Command line argument mode: simple one-to-one or one-to-many mappings
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Master-slave mapping config file path (YAML). If specified, overrides command line arguments",
        ),
        DeclareLaunchArgument(
            "master_ns",
            default_value="",
            description="Master arm namespace (e.g. arm1)",
        ),
        DeclareLaunchArgument(
            "slave_ns",
            default_value="",
            description="Slave arm namespace (single slave arm, e.g. arm2)",
        ),
        DeclareLaunchArgument(
            "slave_ns_list",
            default_value="",
            description="Slave arm namespace list (one-to-many, e.g. \"['arm2','arm3']\" or \"arm2,arm3\")",
        ),
        DeclareLaunchArgument(
            "rate",
            default_value="50.0",
            description="Control frequency (Hz)",
        ),
    ]
    
    # Master-slave teleoperation node
    master_slave_node = Node(
        package="el_a3_teleop",
        executable="master_slave_node",
        name="master_slave_node",
        output="both",
        parameters=[{
            "config_file": LaunchConfiguration("config_file"),
            "master_ns": LaunchConfiguration("master_ns"),
            "slave_ns": LaunchConfiguration("slave_ns"),
            "slave_ns_list": LaunchConfiguration("slave_ns_list"),
            "rate": LaunchConfiguration("rate"),
        }],
    )
    
    return LaunchDescription(declared_arguments + [master_slave_node])
