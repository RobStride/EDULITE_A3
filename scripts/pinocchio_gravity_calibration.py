#!/usr/bin/env python3
"""
Pinocchio 重力补偿标定程序

使用 Pinocchio 动力学库计算完整的重力补偿，并通过实际测量数据
标定惯性参数缩放因子，以提高重力补偿精度。

功能：
1. 加载 URDF 模型到 Pinocchio
2. 在多个关节位置组合下收集力矩数据
3. 比较 Pinocchio 预测值与实际测量值
4. 通过最小二乘法拟合惯性参数缩放因子
5. 输出标定结果

使用方法：
    ros2 run el_a3_description pinocchio_gravity_calibration.py [options]
    
    选项：
        --urdf PATH     URDF文件路径
        --quick         快速模式（减少测试点）
        --verify        仅验证模式（不标定）
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from builtin_interfaces.msg import Duration

import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional
import time
import argparse
import os

# Pinocchio imports
try:
    import pinocchio as pin
    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False
    print("警告: Pinocchio 未安装，部分功能不可用")


@dataclass
class CalibrationConfig:
    """标定配置"""
    # URDF 路径
    urdf_path: str = ""
    
    # 关节名称
    joint_names: List[str] = field(default_factory=lambda: [
        'L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint'
    ])
    
    # 安全 Home 位置
    home_position: List[float] = field(default_factory=lambda: [0.0, 0.785, -0.785, 0.0, 0.0, 0.0])
    
    # 测试点配置
    num_test_configs: int = 20        # 测试配置数量
    samples_per_config: int = 30      # 每个配置采样次数
    settle_time: float = 2.0          # 稳定等待时间
    sample_interval: float = 0.02     # 采样间隔
    motion_duration: float = 6.0      # 运动时间
    
    # 关节测试范围（相对于 home 位置）
    test_range: float = 0.8           # ±0.8 rad
    
    # 安全边界
    zero_margin: float = 0.3          # 避开零点 ±0.3 rad
    max_angle: float = 1.5            # 最大测试角度 ±1.5 rad


@dataclass
class CalibrationResult:
    """单个关节的标定结果"""
    joint_name: str
    mass_scale: float = 1.0
    com_offset: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rmse: float = 0.0
    r_squared: float = 0.0
    num_samples: int = 0


class PinocchioGravityCalibrator(Node):
    """Pinocchio 重力补偿标定器"""
    
    def __init__(self, config: CalibrationConfig):
        super().__init__('pinocchio_gravity_calibrator')
        
        self.config = config
        self.joint_names = config.joint_names
        
        # 当前状态
        self.current_positions = [0.0] * 6
        self.current_efforts = [0.0] * 6
        self.state_received = False
        
        # Pinocchio 模型
        self.pin_model = None
        self.pin_data = None
        
        # 标定数据
        self.calibration_data: Dict[str, List[Tuple[np.ndarray, np.ndarray]]] = {}
        
        # ROS2 接口
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        
        self.zero_torque_client = self.create_client(
            SetBool, '/el_a3/set_zero_torque_mode')
        
        # 初始化 Pinocchio
        if PINOCCHIO_AVAILABLE and config.urdf_path:
            self.init_pinocchio(config.urdf_path)
    
    def init_pinocchio(self, urdf_path: str) -> bool:
        """初始化 Pinocchio 模型"""
        try:
            # 构建模型
            self.pin_model = pin.buildModelFromUrdf(urdf_path)
            self.pin_data = pin.Data(self.pin_model)
            
            self.get_logger().info(f'Pinocchio 模型加载成功:')
            self.get_logger().info(f'  模型名称: {self.pin_model.name}')
            self.get_logger().info(f'  关节数: {self.pin_model.njoints}')
            self.get_logger().info(f'  自由度: {self.pin_model.nv}')
            
            # 打印关节信息
            for i in range(1, self.pin_model.njoints):
                self.get_logger().info(f'  关节 {i}: {self.pin_model.names[i]}')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Pinocchio 模型加载失败: {e}')
            return False
    
    def joint_state_callback(self, msg: JointState):
        """关节状态回调"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
                if idx < len(msg.effort):
                    self.current_efforts[i] = msg.effort[idx]
        self.state_received = True
    
    def compute_pinocchio_gravity(self, q: np.ndarray) -> np.ndarray:
        """使用 Pinocchio 计算重力力矩"""
        if self.pin_model is None:
            return np.zeros(len(q))
        
        # 确保 q 是正确的维度
        q_pin = np.zeros(self.pin_model.nq)
        for i in range(min(len(q), self.pin_model.nq)):
            q_pin[i] = q[i]
        
        # 零速度和零加速度
        v = np.zeros(self.pin_model.nv)
        a = np.zeros(self.pin_model.nv)
        
        # RNEA 计算逆动力学（v=0, a=0 时得到重力力矩）
        tau = pin.rnea(self.pin_model, self.pin_data, q_pin, v, a)
        
        return tau[:len(q)]
    
    def move_to_position(self, target_positions: List[float], duration: float = None) -> bool:
        """移动到目标位置"""
        if duration is None:
            duration = self.config.motion_duration
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server 不可用')
            return False
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        
        goal_msg.trajectory.points = [point]
        
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('运动目标被拒绝')
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 10)
        
        return True
    
    def enable_zero_torque(self, enable: bool) -> bool:
        """启用/禁用零力矩模式"""
        if not self.zero_torque_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('零力矩服务不可用')
            return False
        
        request = SetBool.Request()
        request.data = enable
        
        future = self.zero_torque_client.call_async(request)
        
        start = time.time()
        while not future.done() and (time.time() - start) < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if future.done():
            result = future.result()
            return result.success if result else False
        
        return False
    
    def collect_samples(self, num_samples: int = None) -> Tuple[np.ndarray, np.ndarray]:
        """在当前位置收集力矩样本"""
        if num_samples is None:
            num_samples = self.config.samples_per_config
        
        positions = []
        efforts = []
        
        for _ in range(num_samples):
            rclpy.spin_once(self, timeout_sec=self.config.sample_interval)
            positions.append(self.current_positions.copy())
            efforts.append(self.current_efforts.copy())
            time.sleep(self.config.sample_interval)
        
        return np.array(positions), np.array(efforts)
    
    def generate_test_configurations(self) -> List[List[float]]:
        """生成测试配置点"""
        configs = []
        home = np.array(self.config.home_position)
        
        # 策略：在多个关节组合下测试
        # 1. 首先添加 home 位置
        configs.append(home.tolist())
        
        # 2. 单关节变化（主要关节 L2, L3）
        for joint_idx in [1, 2]:  # L2, L3
            for offset in np.linspace(-self.config.test_range, self.config.test_range, 5):
                config = home.copy()
                new_angle = home[joint_idx] + offset
                
                # 检查安全边界
                if abs(new_angle) < self.config.zero_margin:
                    continue
                if abs(new_angle) > self.config.max_angle:
                    continue
                
                config[joint_idx] = new_angle
                configs.append(config.tolist())
        
        # 3. L2-L3 组合变化
        for l2_offset in np.linspace(-0.5, 0.5, 3):
            for l3_offset in np.linspace(-0.5, 0.5, 3):
                config = home.copy()
                config[1] = home[1] + l2_offset
                config[2] = home[2] + l3_offset
                
                # 检查安全边界
                if abs(config[1]) < self.config.zero_margin or abs(config[1]) > self.config.max_angle:
                    continue
                if abs(config[2]) < self.config.zero_margin or abs(config[2]) > self.config.max_angle:
                    continue
                
                configs.append(config.tolist())
        
        # 去重并限制数量
        unique_configs = []
        for cfg in configs:
            is_duplicate = False
            for uc in unique_configs:
                if np.allclose(cfg, uc, atol=0.05):
                    is_duplicate = True
                    break
            if not is_duplicate:
                unique_configs.append(cfg)
        
        return unique_configs[:self.config.num_test_configs]
    
    def run_calibration(self) -> Dict[str, CalibrationResult]:
        """运行完整的标定流程"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('  Pinocchio 重力补偿标定程序')
        self.get_logger().info('=' * 60)
        
        if self.pin_model is None:
            self.get_logger().error('Pinocchio 模型未初始化')
            return {}
        
        # 等待关节状态
        self.get_logger().info('等待关节状态...')
        start = time.time()
        while not self.state_received and (time.time() - start) < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.state_received:
            self.get_logger().error('无法获取关节状态')
            return {}
        
        self.get_logger().info(f'当前位置: {[f"{p:.3f}" for p in self.current_positions]}')
        
        # 生成测试配置
        test_configs = self.generate_test_configurations()
        self.get_logger().info(f'生成 {len(test_configs)} 个测试配置')
        
        # 收集数据
        all_positions = []
        all_measured_torques = []
        all_predicted_torques = []
        
        for idx, config in enumerate(test_configs):
            self.get_logger().info(f'\n测试配置 {idx+1}/{len(test_configs)}:')
            self.get_logger().info(f'  目标: {[f"{p:.3f}" for p in config]}')
            
            # 移动到目标位置
            if not self.move_to_position(config):
                self.get_logger().warn('  移动失败，跳过')
                continue
            
            # 等待稳定
            time.sleep(self.config.settle_time)
            
            # 收集样本
            positions, efforts = self.collect_samples()
            
            # 计算平均值
            avg_positions = positions.mean(axis=0)
            avg_efforts = efforts.mean(axis=0)
            
            # 计算 Pinocchio 预测的重力力矩
            predicted_gravity = self.compute_pinocchio_gravity(avg_positions)
            
            self.get_logger().info(f'  实际位置: {[f"{p:.3f}" for p in avg_positions]}')
            self.get_logger().info(f'  测量力矩: {[f"{t:.4f}" for t in avg_efforts]}')
            self.get_logger().info(f'  预测力矩: {[f"{t:.4f}" for t in predicted_gravity]}')
            
            all_positions.append(avg_positions)
            all_measured_torques.append(avg_efforts)
            all_predicted_torques.append(predicted_gravity)
        
        # 回到 home 位置
        self.get_logger().info('\n回到 Home 位置...')
        self.move_to_position(self.config.home_position)
        
        # 分析数据并计算缩放因子
        results = self.analyze_calibration_data(
            np.array(all_positions),
            np.array(all_measured_torques),
            np.array(all_predicted_torques)
        )
        
        return results
    
    def analyze_calibration_data(
        self,
        positions: np.ndarray,
        measured: np.ndarray,
        predicted: np.ndarray
    ) -> Dict[str, CalibrationResult]:
        """分析标定数据，计算缩放因子"""
        
        results = {}
        
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('  标定结果分析')
        self.get_logger().info('=' * 60)
        
        for i, joint_name in enumerate(self.joint_names):
            result = CalibrationResult(joint_name=joint_name)
            
            # 提取该关节的数据
            m = measured[:, i]
            p = predicted[:, i]
            
            # 计算最优缩放因子（最小二乘）
            # measured ≈ scale * predicted
            # 避免除以零
            if np.abs(p).sum() > 0.01:
                # 使用最小二乘法
                scale = np.dot(m, p) / np.dot(p, p)
                scale = np.clip(scale, 0.5, 2.0)  # 限制范围
            else:
                scale = 1.0
            
            result.mass_scale = scale
            
            # 计算误差
            scaled_predicted = p * scale
            errors = m - scaled_predicted
            result.rmse = np.sqrt(np.mean(errors ** 2))
            
            # 计算 R²
            ss_res = np.sum(errors ** 2)
            ss_tot = np.sum((m - np.mean(m)) ** 2)
            if ss_tot > 1e-6:
                result.r_squared = 1 - ss_res / ss_tot
            else:
                result.r_squared = 0.0
            
            result.num_samples = len(m)
            
            results[joint_name] = result
            
            self.get_logger().info(f'\n{joint_name}:')
            self.get_logger().info(f'  质量缩放因子: {result.mass_scale:.4f}')
            self.get_logger().info(f'  RMSE: {result.rmse:.4f} Nm')
            self.get_logger().info(f'  R²: {result.r_squared:.4f}')
            self.get_logger().info(f'  样本数: {result.num_samples}')
        
        # 输出 XACRO 配置参数
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('  XACRO 配置参数（复制到 el_a3_ros2_control.xacro）')
        self.get_logger().info('=' * 60)
        
        for i, joint_name in enumerate(self.joint_names):
            result = results[joint_name]
            print(f'<param name="inertia_scale_L{i+1}_mass">{result.mass_scale:.4f}</param>')
        
        return results
    
    def verify_gravity_compensation(self) -> None:
        """验证重力补偿效果"""
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('  验证重力补偿效果')
        self.get_logger().info('=' * 60)
        
        if self.pin_model is None:
            self.get_logger().error('Pinocchio 模型未初始化')
            return
        
        # 在当前位置计算重力
        q = np.array(self.current_positions)
        gravity = self.compute_pinocchio_gravity(q)
        
        self.get_logger().info(f'当前位置: {[f"{p:.3f}" for p in q]}')
        self.get_logger().info(f'预测重力力矩: {[f"{t:.4f}" for t in gravity]}')
        self.get_logger().info(f'实际测量力矩: {[f"{t:.4f}" for t in self.current_efforts]}')
        
        # 计算误差
        errors = np.array(self.current_efforts) - gravity
        self.get_logger().info(f'误差: {[f"{e:.4f}" for e in errors]}')


def main():
    parser = argparse.ArgumentParser(description='Pinocchio 重力补偿标定')
    parser.add_argument('--urdf', type=str, 
                        default='/home/wy/RS/A3/install/el_a3_description/share/el_a3_description/urdf/el_a3.urdf',
                        help='URDF 文件路径')
    parser.add_argument('--quick', action='store_true', help='快速模式')
    parser.add_argument('--verify', action='store_true', help='仅验证模式')
    
    args = parser.parse_args()
    
    if not PINOCCHIO_AVAILABLE:
        print("错误: 需要安装 Pinocchio 库")
        print("  sudo apt install ros-humble-pinocchio")
        return
    
    rclpy.init()
    
    # 配置
    config = CalibrationConfig(urdf_path=args.urdf)
    
    if args.quick:
        config.num_test_configs = 10
        config.samples_per_config = 20
        config.settle_time = 1.5
        config.motion_duration = 5.0
    
    # 创建标定器
    calibrator = PinocchioGravityCalibrator(config)
    
    # 等待初始化
    time.sleep(1.0)
    
    try:
        if args.verify:
            # 仅验证模式
            start = time.time()
            while not calibrator.state_received and (time.time() - start) < 10:
                rclpy.spin_once(calibrator, timeout_sec=0.1)
            
            calibrator.verify_gravity_compensation()
        else:
            # 完整标定
            results = calibrator.run_calibration()
            
            if results:
                print("\n标定完成！")
                print("请将上述 XACRO 参数复制到配置文件中。")
            else:
                print("\n标定失败。")
    
    except KeyboardInterrupt:
        print("\n标定被中断")
    
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
