#!/usr/bin/env python3
"""
EL-A3 主从遥操作节点

支持灵活的主从映射配置：一对一 / 一对多 / 多对多。
主臂可启用零力矩模式进行手动示教，从臂实时跟随。
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from std_srvs.srv import SetBool
from builtin_interfaces.msg import Duration

import yaml
import numpy as np
from typing import List, Dict, Optional
from dataclasses import dataclass, field


@dataclass
class MappingConfig:
    """主从映射配置"""
    master: str
    slaves: List[str]
    scale: float = 1.0
    mirror: bool = False
    joint_offset: List[float] = field(default_factory=lambda: [0.0] * 7)  # 支持 7 关节（含夹爪）
    enabled: bool = True
    include_gripper: bool = True  # 是否包含夹爪（L7）
    joint_count: int = 7  # 关节数量（6 或 7）


class MasterSlaveMapping:
    """单个主从映射（一个主臂，对应多个从臂）"""
    
    def __init__(self, node: Node, config: MappingConfig, safety_config: dict, slave_config: dict):
        self.node = node
        self.config = config
        self.safety = safety_config
        self.slave_cfg = slave_config
        
        self.master_state: Optional[JointState] = None
        self.last_target: Optional[List[float]] = None
        self.last_update_time: Optional[float] = None
        self.state_updated: bool = False  # 是否有新状态可用的标志
        
        # 主臂关节名称（用于按顺序提取位置）
        master_prefix = f"{config.master}_"
        num_joints = config.joint_count if config.include_gripper else 6
        self.master_joint_names = [f"{master_prefix}L{i}_joint" for i in range(1, num_joints + 1)]
        
        # 回调组
        self.cb_group = ReentrantCallbackGroup()
        
        # 订阅主臂状态
        self.master_sub = node.create_subscription(
            JointState,
            f'/{config.master}/joint_states',
            self.master_callback,
            10,
            callback_group=self.cb_group
        )
        
        # 为每个从臂创建发布器/客户端
        self.slave_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        self.slave_clients: Dict[str, ActionClient] = {}
        
        for slave_ns in config.slaves:
            if slave_config.get('use_streaming', True):
                # 流式控制：直接发布轨迹指令
                pub = node.create_publisher(
                    JointTrajectory,
                    f'/{slave_ns}/{slave_ns}_arm_controller/joint_trajectory',
                    10
                )
                self.slave_pubs[slave_ns] = pub
            else:
                # Action 控制
                client = ActionClient(
                    node,
                    FollowJointTrajectory,
                    f'/{slave_ns}/{slave_ns}_arm_controller/follow_joint_trajectory',
                    callback_group=self.cb_group
                )
                self.slave_clients[slave_ns] = client
        
        node.get_logger().info(f'已创建映射：{config.master} -> {config.slaves}')
    
    def master_callback(self, msg: JointState):
        """主臂关节状态回调"""
        self.master_state = msg
        self.state_updated = True
    
    def update(self):
        """更新并发送从臂目标位置"""
        if self.master_state is None:
            return
        
        # 仅处理新状态，避免重复发送
        if not self.state_updated:
            return
        self.state_updated = False
        
        # 按关节名顺序提取主臂关节位置
        master_positions = []
        for joint_name in self.master_joint_names:
            if joint_name in self.master_state.name:
                idx = self.master_state.name.index(joint_name)
                master_positions.append(self.master_state.position[idx])
            else:
                # 关节缺失：跳过本次更新
                return
        
        # 根据配置决定关节数量
        num_joints = self.config.joint_count if self.config.include_gripper else 6
        
        if len(master_positions) < num_joints:
            return
        
        # 计算目标位置
        target_positions = self.compute_target(master_positions[:num_joints])
        
        # 安全限速（使用真实时间间隔）
        import time
        current_time = time.time()
        limited_positions = self.safety_check(target_positions, current_time)
        self.last_update_time = current_time
        
        self.last_target = limited_positions
        
        # 发送到所有从臂
        for slave_ns in self.config.slaves:
            self.send_to_slave(slave_ns, limited_positions)
    
    def compute_target(self, master_positions: List[float]) -> List[float]:
        """计算从臂目标位置（支持 6 或 7 关节）"""
        num_joints = len(master_positions)
        target = list(master_positions)
        
        # 比例缩放
        target = [p * self.config.scale for p in target]
        
        # 镜像映射（翻转 L1/L4/L6）
        if self.config.mirror:
            target[0] = -target[0]  # L1
            target[3] = -target[3]  # L4
            target[5] = -target[5]  # L6
            # 夹爪（L7）不做镜像，保持不变
        
        # 叠加偏置
        offset = self.config.joint_offset[:num_joints]
        target = [p + o for p, o in zip(target, offset)]
        
        return target
    
    def safety_check(self, target: List[float], current_time: float = None) -> List[float]:
        """安全检查与速度限制：返回限速后的目标位置"""
        if self.last_target is None:
            return target
        
        # 使用真实时间间隔
        if current_time and self.last_update_time:
            dt = current_time - self.last_update_time
            dt = max(0.005, min(dt, 0.1))  # Clamp to 5ms ~ 100ms
        else:
            dt = 0.02  # Default 50Hz
        
        # 检查速度限制并进行限幅
        max_vel = self.safety.get('max_velocity', 10.0)  # Default speed limit raised to 10 rad/s
        max_delta = max_vel * dt
        
        limited_target = []
        for i, (curr, prev) in enumerate(zip(target, self.last_target)):
            delta = curr - prev
            if abs(delta) > max_delta:
                # 限制变化量，但允许逐步跟踪
                limited_delta = max_delta if delta > 0 else -max_delta
                limited_target.append(prev + limited_delta)
            else:
                limited_target.append(curr)
        
        return limited_target
    
    def send_to_slave(self, slave_ns: str, target_positions: List[float]):
        """Send command to slave arm (supports 6 or 7 joints)"""
        num_joints = len(target_positions)
        
        # 生成关节名（基于关节数量）
        prefix = f"{slave_ns}_"
        joint_names = [f"{prefix}L{i}_joint" for i in range(1, num_joints + 1)]
        
        # 构造轨迹消息
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = list(target_positions)
        point.velocities = [0.0] * num_joints
        
        # 设置执行时间（缩短以提高响应速度）
        duration_sec = self.slave_cfg.get('trajectory_duration', 0.02)  # Default 20ms
        point.time_from_start = Duration(
            sec=0,
            nanosec=int(duration_sec * 1e9)
        )
        
        trajectory.points = [point]
        
        if slave_ns in self.slave_pubs:
            self.slave_pubs[slave_ns].publish(trajectory)


class MasterSlaveNode(Node):
    """主从遥操作节点"""
    
    def __init__(self):
        super().__init__('master_slave_node')
        
        # Declare parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('master_ns', '')
        self.declare_parameter('slave_ns', '')
        self.declare_parameter('slave_ns_list', '')
        self.declare_parameter('rate', 50.0)
        
        # Get parameters
        config_file = self.get_parameter('config_file').value
        master_ns = self.get_parameter('master_ns').value
        slave_ns = self.get_parameter('slave_ns').value
        slave_ns_list = self.get_parameter('slave_ns_list').value
        rate = self.get_parameter('rate').value
        
        # Load configuration
        self.config = self.load_config(config_file, master_ns, slave_ns, slave_ns_list)
        
        # Update rate
        self.rate = self.config.get('rate', rate)
        
        # Get safety and slave arm configuration
        self.safety_config = self.config.get('safety', {})
        self.slave_config = self.config.get('slave', {})
        self.master_config = self.config.get('master', {})
        
        # Create mappings
        self.mappings: List[MasterSlaveMapping] = []
        
        for mapping_cfg in self.config.get('mappings', []):
            if mapping_cfg.get('enabled', True):
                include_gripper = mapping_cfg.get('include_gripper', True)
                joint_count = mapping_cfg.get('joint_count', 7 if include_gripper else 6)
                default_offset = [0.0] * joint_count
                
                cfg = MappingConfig(
                    master=mapping_cfg['master'],
                    slaves=mapping_cfg['slaves'],
                    scale=mapping_cfg.get('scale', 1.0),
                    mirror=mapping_cfg.get('mirror', False),
                    joint_offset=mapping_cfg.get('joint_offset', default_offset),
                    enabled=mapping_cfg.get('enabled', True),
                    include_gripper=include_gripper,
                    joint_count=joint_count
                )
                mapping = MasterSlaveMapping(
                    self, cfg, self.safety_config, self.slave_config
                )
                self.mappings.append(mapping)
        
        # Enable master arm zero-torque mode
        if self.master_config.get('use_zero_torque_mode', True):
            self.enable_master_zero_torque()
        
        # Control timer
        self.timer = self.create_timer(1.0 / self.rate, self.control_loop)
        
        self.get_logger().info(
            f'主从遥操作节点已启动：rate={self.rate}Hz，映射数量={len(self.mappings)}'
        )
    
    def load_config(self, config_file: str, master_ns: str, 
                    slave_ns: str, slave_ns_list: str) -> dict:
        """加载配置"""
        config = {}
        
        # Load from file
        if config_file:
            try:
                with open(config_file, 'r') as f:
                    config = yaml.safe_load(f)
                self.get_logger().info(f'已加载配置文件：{config_file}')
            except Exception as e:
                self.get_logger().error(f'加载配置文件失败：{e}')
        
        # Override from command line parameters
        if master_ns and (slave_ns or slave_ns_list):
            # Parse slave arm list
            slaves = []
            if slave_ns_list:
                # Parse list format "['arm2','arm3']" or "arm2,arm3"
                slave_ns_list = slave_ns_list.strip("[]'\"")
                slaves = [s.strip().strip("'\"") for s in slave_ns_list.split(',')]
            elif slave_ns:
                slaves = [slave_ns]
            
            # Create single mapping
            config['mappings'] = [{
                'master': master_ns,
                'slaves': slaves,
                'scale': 1.0,
                'mirror': False,
                'enabled': True
            }]
            
            self.get_logger().info(f'使用命令行参数：{master_ns} -> {slaves}')
        
        return config
    
    def enable_master_zero_torque(self):
        """启用主臂零力矩模式（支持纯零力矩模式）"""
        use_pure_zero_torque = self.master_config.get('pure_zero_torque', True)
        
        for mapping in self.mappings:
            master_ns = mapping.config.master
            
            # Select service based on configuration
            if use_pure_zero_torque:
                # Pure zero-torque mode: Kp=0, Kd=0, torque=0 (for master-slave teleoperation)
                service_name = f'/{master_ns}/set_pure_zero_torque_mode'
                mode_name = 'pure zero-torque'
            else:
                # Zero-torque mode with gravity compensation
                service_name = f'/{master_ns}/set_zero_torque_mode'
                mode_name = 'gravity-compensated zero-torque'
            
            client = self.create_client(SetBool, service_name)
            
            if client.wait_for_service(timeout_sec=2.0):
                request = SetBool.Request()
                request.data = True
                future = client.call_async(request)
                self.get_logger().info(f'已请求启用 {master_ns} 的 {mode_name} 模式')
            else:
                self.get_logger().warn(f'服务不可用：{service_name}')
    
    def control_loop(self):
        """主控制循环"""
        for mapping in self.mappings:
            mapping.update()


def main(args=None):
    rclpy.init(args=args)
    
    node = MasterSlaveNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
