#!/usr/bin/env python3
"""
EL-A3 Web UI 的 ROS2 桥接节点。
负责在 ROS2 与 WebSocket 之间进行通信。
"""

import threading
import time
import subprocess
import signal
import os
from typing import Callable, Dict, List, Optional
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import SetBool
from std_msgs.msg import String
from builtin_interfaces.msg import Duration


class ROS2Bridge(Node):
    """Web 界面的 ROS2 桥接节点。"""

    # Joint names in order
    JOINT_NAMES = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']
    
    # Joint limits (from URDF)
    JOINT_LIMITS = {
        'L1_joint': {'lower': -2.79253, 'upper': 2.79253},
        'L2_joint': {'lower': -3.14159, 'upper': 3.14159},
        'L3_joint': {'lower': -3.14159, 'upper': 3.14159},
        'L4_joint': {'lower': -3.14159, 'upper': 3.14159},
        'L5_joint': {'lower': -3.14159, 'upper': 3.14159},
        'L6_joint': {'lower': -3.14159, 'upper': 3.14159},
    }

    def __init__(self):
        super().__init__('el_a3_web_bridge')
        
        # State storage
        self._joint_states: Dict[str, Dict] = {}
        self._gravity_torques: Dict[str, float] = {}
        self._system_status = 'disconnected'
        self._controller_status = 'unknown'
        self._last_update_time = 0.0
        
        # Motor enable state
        self._motors_enabled = False
        
        # Teleop process management
        self._teleop_process: Optional[subprocess.Popen] = None
        self._teleop_status = 'stopped'
        
        # CAN interface configuration
        self._current_can_interface = 'can0'
        
        # Callbacks for WebSocket updates
        self._state_callback: Optional[Callable] = None
        self._log_callback: Optional[Callable] = None
        
        # Lock for thread safety
        self._lock = threading.Lock()
        
        # Initialize joint states
        for name in self.JOINT_NAMES:
            self._joint_states[name] = {
                'position': 0.0,
                'velocity': 0.0,
                'effort': 0.0
            }
            self._gravity_torques[name] = 0.0
        
        # Motor temperatures (°C)
        self._motor_temperatures: Dict[str, float] = {}
        for name in self.JOINT_NAMES:
            self._motor_temperatures[name] = 0.0
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        
        self.gravity_torque_sub = self.create_subscription(
            JointState,
            '/debug/gravity_torque',
            self._gravity_torque_callback,
            10
        )
        
        # Motor temperature subscriber
        self.temperature_sub = self.create_subscription(
            JointState,
            '/debug/motor_temperature',
            self._temperature_callback,
            10
        )
        
        # Publisher for trajectory commands
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        
        # Service clients
        self.zero_torque_client = self.create_client(
            SetBool,
            '/el_a3/set_zero_torque_mode'
        )
        
        # Status timer (check connection every 2 seconds)
        self.status_timer = self.create_timer(2.0, self._check_status)
        
        # Teleop process check timer
        self.teleop_check_timer = self.create_timer(1.0, self._check_teleop_process)
        
        self.get_logger().info('ROS2 Bridge initialized')

    def set_state_callback(self, callback: Callable):
        """设置状态更新回调。"""
        self._state_callback = callback

    def set_log_callback(self, callback: Callable):
        """设置日志回调。"""
        self._log_callback = callback

    def _log(self, message: str, level: str = 'info'):
        """记录日志并转发给回调。"""
        if level == 'info':
            self.get_logger().info(message)
        elif level == 'warn':
            self.get_logger().warn(message)
        elif level == 'error':
            self.get_logger().error(message)
        
        if self._log_callback:
            self._log_callback({
                'timestamp': time.time(),
                'level': level,
                'message': message
            })

    def _joint_state_callback(self, msg: JointState):
        """处理关节状态消息。"""
        with self._lock:
            self._last_update_time = time.time()
            self._system_status = 'connected'
            
            for i, name in enumerate(msg.name):
                if name in self._joint_states:
                    if i < len(msg.position):
                        self._joint_states[name]['position'] = msg.position[i]
                    if i < len(msg.velocity):
                        self._joint_states[name]['velocity'] = msg.velocity[i]
                    if i < len(msg.effort):
                        self._joint_states[name]['effort'] = msg.effort[i]
        
        # Notify WebSocket
        if self._state_callback:
            self._state_callback(self.get_state())

    def _gravity_torque_callback(self, msg: JointState):
        """处理重力补偿力矩消息。"""
        with self._lock:
            for i, name in enumerate(msg.name):
                if name in self._gravity_torques and i < len(msg.effort):
                    self._gravity_torques[name] = msg.effort[i]

    def _temperature_callback(self, msg: JointState):
        """处理电机温度消息。
        
        注意：温度通过 JointState 的 effort 字段发布。
        """
        with self._lock:
            for i, name in enumerate(msg.name):
                if name in self._motor_temperatures and i < len(msg.effort):
                    self._motor_temperatures[name] = msg.effort[i]

    def _check_status(self):
        """检查系统连接状态。"""
        with self._lock:
            if time.time() - self._last_update_time > 3.0:
                if self._system_status != 'disconnected':
                    self._system_status = 'disconnected'
                    self._log('与机器人连接已断开', 'warn')
            else:
                if self._system_status != 'connected':
                    self._system_status = 'connected'
                    self._log('已连接到机器人', 'info')

    def get_state(self) -> Dict:
        """获取当前机器人状态。"""
        with self._lock:
            joints = []
            for name in self.JOINT_NAMES:
                state = self._joint_states[name]
                joints.append({
                    'name': name,
                    'position': state['position'],
                    'position_deg': math.degrees(state['position']),
                    'velocity': state['velocity'],
                    'effort': state['effort'],
                    'gravity_torque': self._gravity_torques.get(name, 0.0),
                    'temperature': self._motor_temperatures.get(name, 0.0),
                    'limits': self.JOINT_LIMITS[name]
                })
            
            return {
                'timestamp': time.time(),
                'status': self._system_status,
                'controller': self._controller_status,
                'motors_enabled': self._motors_enabled,
                'teleop_status': self.get_teleop_status(),
                'can_interface': self._current_can_interface,
                'joints': joints
            }

    def send_joint_command(self, positions: List[float], duration: float = 2.0) -> bool:
        """发送关节位置指令。
        
        Args:
            positions: 6 个关节角（弧度）
            duration: 到达目标所需时间（秒）
        
        Returns:
            指令发送成功返回 True
        """
        if len(positions) != 6:
            self._log(f'关节数量非法：{len(positions)}，期望 6', 'error')
            return False
        
        # Clamp positions to limits
        clamped_positions = []
        for i, (name, pos) in enumerate(zip(self.JOINT_NAMES, positions)):
            limits = self.JOINT_LIMITS[name]
            clamped = max(limits['lower'], min(limits['upper'], pos))
            if clamped != pos:
                self._log(f'{name} 超出限位：{math.degrees(pos):.1f}° -> {math.degrees(clamped):.1f}°（已夹紧）', 'warn')
            clamped_positions.append(clamped)
        
        # Create trajectory message
        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES
        
        point = JointTrajectoryPoint()
        point.positions = clamped_positions
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        
        traj.points.append(point)
        
        self.trajectory_pub.publish(traj)
        self._log(f'已发送轨迹指令（时长：{duration:.1f}s）', 'info')
        
        return True

    def send_single_joint_command(self, joint_index: int, position: float, duration: float = 1.0) -> bool:
        """单关节控制：其他关节保持当前角度。
        
        Args:
            joint_index: Index of joint (0-5)
            position: Target position in radians
            duration: Time to reach the position
        
        Returns:
            指令发送成功返回 True
        """
        if not 0 <= joint_index < 6:
            self._log(f'关节索引非法：{joint_index}', 'error')
            return False
        
        with self._lock:
            positions = [self._joint_states[name]['position'] for name in self.JOINT_NAMES]
        
        positions[joint_index] = position
        return self.send_joint_command(positions, duration)

    def go_home(self, duration: float = 3.0) -> bool:
        """回到 home（全零位）。"""
        self._log('正在回到 home（全零位）', 'info')
        return self.send_joint_command([0.0] * 6, duration)

    def _call_service_cli(self, service_name: str, data: bool) -> bool:
        """使用 ros2 CLI 调用 SetBool 服务，避免线程问题。"""
        try:
            cmd = [
                'ros2', 'service', 'call',
                service_name,
                'std_srvs/srv/SetBool',
                f'{{data: {str(data).lower()}}}'
            ]
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=5.0
            )
            # Check if service call succeeded
            if 'success=True' in result.stdout or 'success: true' in result.stdout.lower():
                return True
            elif result.returncode == 0:
                # Service was called but check response
                return 'success' in result.stdout.lower()
            return False
        except subprocess.TimeoutExpired:
            self._log(f'服务调用超时：{service_name}', 'error')
            return False
        except Exception as e:
            self._log(f'服务调用异常：{e}', 'error')
            return False

    def set_zero_torque_mode(self, enable: bool) -> bool:
        """开启/关闭零力矩模式。
        
        Args:
            enable: True 表示开启零力矩模式
        
        Returns:
            成功返回 True
        """
        success = self._call_service_cli('/el_a3/set_zero_torque_mode', enable)
        
        if success:
            self._log(f'零力矩模式：{"已开启" if enable else "已关闭"}', 'info')
        else:
            self._log('设置零力矩模式失败', 'error')
        
        return success

    def emergency_stop(self):
        """急停：发送当前位置保持指令。"""
        self._log('急停（EMERGENCY STOP）', 'warn')
        with self._lock:
            positions = [self._joint_states[name]['position'] for name in self.JOINT_NAMES]
        
        # Send immediate command to hold current position
        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1 seconds
        
        traj.points.append(point)
        self.trajectory_pub.publish(traj)

    # ==================== Motor Enable/Disable ====================
    # Note: Uses zero_torque_mode service as backend via command line
    # Enable motors = Disable zero torque mode (normal position control)
    # Disable motors = Enable zero torque mode (free movement)
    
    def enable_motors(self) -> bool:
        """使能所有电机（关闭零力矩模式）。
        
        Returns:
            True if successful
        """
        success = self._call_service_cli('/el_a3/set_zero_torque_mode', False)
        
        if success:
            self._motors_enabled = True
            self._log('电机已使能（位置控制生效）', 'info')
        else:
            self._log('电机使能失败', 'error')
        
        return success

    def disable_motors(self) -> bool:
        """失能所有电机（开启零力矩模式以便自由拖动）。
        
        Returns:
            True if successful
        """
        success = self._call_service_cli('/el_a3/set_zero_torque_mode', True)
        
        if success:
            self._motors_enabled = False
            self._log('电机已失能（零力矩模式）', 'info')
        else:
            self._log('电机失能失败', 'error')
        
        return success

    def get_motors_enabled(self) -> bool:
        """获取电机使能状态。"""
        return self._motors_enabled

    # ==================== Teleop Control ====================
    
    def _check_teleop_process(self):
        """周期性检查 teleop 进程状态。"""
        if self._teleop_process is not None:
            poll = self._teleop_process.poll()
            if poll is not None:
                # Process has terminated
                self._teleop_status = 'stopped'
                self._teleop_process = None
                self._log('Teleop 进程已退出', 'info')

    def start_teleop(self) -> bool:
        """启动 Xbox 遥操作节点。
        
        Returns:
            启动成功返回 True
        """
        if self._teleop_process is not None:
            poll = self._teleop_process.poll()
            if poll is None:
                self._log('Teleop 已在运行', 'warn')
                return True
        
        try:
            # Start teleop node using ros2 run
            cmd = [
                'ros2', 'run', 'el_a3_teleop', 'xbox_teleop_node',
                '--ros-args',
                '-p', 'use_fast_ik_mode:=true'
            ]
            
            self._teleop_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group
            )
            
            self._teleop_status = 'running'
            self._log('Teleop 已启动', 'info')
            return True
            
        except Exception as e:
            self._log(f'启动 Teleop 失败：{e}', 'error')
            self._teleop_status = 'error'
            return False

    def stop_teleop(self) -> bool:
        """停止 Xbox 遥操作节点。
        
        Returns:
            停止成功返回 True
        """
        if self._teleop_process is None:
            self._teleop_status = 'stopped'
            return True
        
        try:
            # Send SIGTERM to process group
            os.killpg(os.getpgid(self._teleop_process.pid), signal.SIGTERM)
            
            # Wait for process to terminate
            try:
                self._teleop_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                # Force kill if not responding
                os.killpg(os.getpgid(self._teleop_process.pid), signal.SIGKILL)
                self._teleop_process.wait()
            
            self._teleop_process = None
            self._teleop_status = 'stopped'
            self._log('Teleop 已停止', 'info')
            return True
            
        except Exception as e:
            self._log(f'停止 Teleop 失败：{e}', 'error')
            self._teleop_process = None
            self._teleop_status = 'error'
            return False

    def get_teleop_status(self) -> str:
        """获取 teleop 节点状态。
        
        Returns:
            状态字符串：'running' / 'stopped' / 'error'
        """
        if self._teleop_process is not None:
            poll = self._teleop_process.poll()
            if poll is None:
                return 'running'
        return self._teleop_status

    # ==================== CAN Interface Configuration ====================
    
    def get_can_interface(self) -> str:
        """获取当前 CAN 接口名。"""
        return self._current_can_interface

    def set_can_interface(self, interface: str) -> bool:
        """设置 CAN 接口。
        
        注意：该配置需要重启硬件驱动才能生效。
        
        Args:
            interface: CAN 接口名（例如 'can0' / 'can1'）
        
        Returns:
            配置保存成功返回 True（实际切换需要重启）
        """
        # Validate interface name
        if not interface.startswith('can'):
            self._log(f'CAN 接口名非法：{interface}', 'error')
            return False
        
        self._current_can_interface = interface
        self._log(f'CAN 接口已设置为 {interface}（需重启生效）', 'info')
        return True

    @staticmethod
    def get_available_can_interfaces() -> List[str]:
        """获取系统中可用的 CAN 接口列表。
        
        Returns:
            CAN 接口名列表
        """
        interfaces = []
        try:
            result = subprocess.run(
                ['ip', 'link', 'show', 'type', 'can'],
                capture_output=True,
                text=True,
                timeout=5.0
            )
            
            # Parse output: lines like "3: can0: <NOARP,UP,LOWER_UP>..."
            for line in result.stdout.split('\n'):
                if ':' in line and 'can' in line:
                    parts = line.split(':')
                    if len(parts) >= 2:
                        iface = parts[1].strip()
                        if iface.startswith('can'):
                            interfaces.append(iface)
        except Exception:
            pass
        
        # If no interfaces found, return default
        if not interfaces:
            interfaces = ['can0']
        
        return interfaces


class ROS2BridgeThread:
    """ROS2Bridge 的线程封装。"""
    
    def __init__(self):
        self.node: Optional[ROS2Bridge] = None
        self._thread: Optional[threading.Thread] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._running = False

    def start(self):
        """在后台线程启动 ROS2 bridge。"""
        if self._running:
            return
        
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        
        # Wait for node to be created
        timeout = 5.0
        start_time = time.time()
        while self.node is None and time.time() - start_time < timeout:
            time.sleep(0.1)
        
        if self.node is None:
            raise RuntimeError('启动 ROS2 bridge 失败')

    def _run(self):
        """运行 ROS2 节点。"""
        try:
            rclpy.init()
            self.node = ROS2Bridge()
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self.node)
            
            while self._running:
                self._executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            print(f'ROS2 Bridge 异常：{e}')
        finally:
            if self.node:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def stop(self):
        """停止 ROS2 bridge。"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=5.0)
        self.node = None


# Singleton instance
_bridge_thread: Optional[ROS2BridgeThread] = None


def get_bridge() -> ROS2Bridge:
    """获取或创建 ROS2 bridge 单例。"""
    global _bridge_thread
    
    if _bridge_thread is None:
        _bridge_thread = ROS2BridgeThread()
        _bridge_thread.start()
    
    return _bridge_thread.node


def shutdown_bridge():
    """关闭 ROS2 bridge。"""
    global _bridge_thread
    
    if _bridge_thread:
        _bridge_thread.stop()
        _bridge_thread = None
