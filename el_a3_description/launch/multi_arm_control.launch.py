"""
EL-A3 Multi-Arm Control Launch File

Supports launching multiple robot arms simultaneously, each using an independent CAN interface and namespace
"""

import os
import yaml
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(file_path):
    """Load YAML config file"""
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def generate_arm_nodes(context, *args, **kwargs):
    """Dynamically generate robot arm nodes based on configuration"""
    
    # Get arguments
    config_file = LaunchConfiguration('config_file').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    
    # If no config file specified, use default path
    if not config_file:
        config_file = os.path.join(
            FindPackageShare('el_a3_description').perform(context),
            'config', 'multi_arm_config.yaml'
        )
    
    # Load configuration
    config = load_yaml(config_file)
    arms_config = config.get('arms', {})
    global_config = config.get('global', {})
    
    use_mock = global_config.get('use_mock_hardware', False)
    
    nodes = []
    
    # Create nodes for each enabled robot arm
    for arm_name, arm_cfg in arms_config.items():
        if not arm_cfg.get('enabled', False):
            continue
            
        prefix = arm_cfg.get('prefix', f'{arm_name}_')
        can_interface = arm_cfg.get('can_interface', 'can0')
        host_can_id = arm_cfg.get('host_can_id', 253)
        
        # Inertia parameters config file path (each arm can be configured independently)
        default_inertia_path = '/home/wy/RS/A3/el_a3_description/config/inertia_params.yaml'
        inertia_config_path = arm_cfg.get('inertia_config_path', default_inertia_path)
        
        # Generate URDF with prefix
        robot_description_content = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("el_a3_description"), "urdf", "el_a3.urdf.xacro"
            ]),
            f" prefix:={prefix}",
            f" use_mock_hardware:={'true' if use_mock else 'false'}",
            f" can_interface:={can_interface}",
            f" host_can_id:={host_can_id}",
            f" inertia_config_path:={inertia_config_path}",
        ])
        
        robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
        
        # Generate controller config file
        controller_params_file = generate_controller_params(prefix, arm_name, global_config.get('update_rate', 200))
        
        # Create node group (with namespace)
        arm_group = GroupAction([
            PushRosNamespace(arm_name),
            
            # Controller Manager
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, controller_params_file],
                output="both",
                remappings=[
                    ("~/robot_description", f"/{arm_name}/robot_description"),
                ],
            ),
            
            # Robot State Publisher
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description],
            ),
            
            # Joint State Broadcaster Spawner
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    f"{prefix}joint_state_broadcaster",
                    "--controller-manager", f"/{arm_name}/controller_manager",
                    "--controller-manager-timeout", "60"
                ],
            ),
            
            # Arm Controller Spawner
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    f"{prefix}arm_controller",
                    "--controller-manager", f"/{arm_name}/controller_manager",
                    "--controller-manager-timeout", "60"
                ],
            ),
        ])
        
        nodes.append(arm_group)
    
    # RViz (optional, only start one instance)
    if use_rviz.lower() == 'true':
        rviz_config_file = PathJoinSubstitution([
            FindPackageShare("el_a3_description"), "config", "el_a3_view.rviz"
        ])
        
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        )
        nodes.append(rviz_node)
    
    return nodes


def generate_controller_params(prefix: str, arm_name: str, update_rate: int) -> str:
    """Generate controller parameter config file (7 joints including gripper L7)
    
    Returns temporary YAML file path
    """
    
    joints = [f"L{i}_joint" for i in range(1, 7)]  # L1-L6 (no prefix, matches URDF; L7 gripper not in URDF)
    
    # Controller names (with prefix)
    jsb_name = f"{prefix}joint_state_broadcaster"
    ctrl_name = f"{prefix}arm_controller"
    
    # Parameter structure using full namespace path
    params = {
        f"/{arm_name}/controller_manager": {
            "ros__parameters": {
                "update_rate": update_rate,
                jsb_name: {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"
                },
                ctrl_name: {
                    "type": "joint_trajectory_controller/JointTrajectoryController"
                }
            }
        },
        f"/{arm_name}/{ctrl_name}": {
            "ros__parameters": {
                "joints": joints,
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
                "open_loop_control": True,
                "allow_nonzero_velocity_at_trajectory_end": True,
                "interpolation_method": "splines",
                "state_publish_rate": 200.0,
                "action_monitor_rate": 50.0,
                "constraints": {
                    "stopped_velocity_tolerance": 0.1,
                    "goal_time": 0.0,
                    **{joint: {"goal": 0.03} for joint in joints}
                }
            }
        }
    }
    
    # Save as temporary YAML file
    fd, path = tempfile.mkstemp(prefix=f'{arm_name}_controllers_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.dump(params, f, default_flow_style=False)
    
    return path


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Multi-arm configuration file path (YAML)",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to start RViz",
        ),
    ]
    
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=generate_arm_nodes)]
    )
