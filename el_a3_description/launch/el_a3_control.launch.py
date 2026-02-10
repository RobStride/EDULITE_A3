"""
EL-A3 Robot Arm Control Launch File

Launches ros2_control controllers and robot arm hardware interface
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Use mock hardware for simulation",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="CAN interface name (e.g., can0)",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "host_can_id",
            default_value="253",
            description="Host CAN ID (default 0xFD = 253)",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2",
        )
    )

    # Get arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    can_interface = LaunchConfiguration("can_interface")
    host_can_id = LaunchConfiguration("host_can_id")
    use_rviz = LaunchConfiguration("use_rviz")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("el_a3_description"), "urdf", "el_a3.urdf.xacro"]
            ),
            " use_mock_hardware:=",
            use_mock_hardware,
            " can_interface:=",
            can_interface,
            " host_can_id:=",
            host_can_id,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Controller config
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("el_a3_description"), "config", "el_a3_controllers.yaml"]
    )

    # RViz config
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("el_a3_description"), "config", "el_a3_view.rviz"]
    )

    # Nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    # Spawn controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", 
                   "--controller-manager-timeout", "60"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
    )

    # Delay arm_controller spawner after joint_state_broadcaster is active
    delay_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)

