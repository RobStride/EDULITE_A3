#!/usr/bin/env python3
"""
EL-A3 高精度重力补偿标定程序

功能:
1. 以 home 点 (零位) 为基础
2. 在关节限位范围内的多个点位进行力矩采样
3. 使用最小二乘法拟合重力补偿参数
4. 验证标定结果并输出 XACRO 参数

使用方法:
1. 启动机器人控制器: ros2 launch el_a3_description el_a3_control.launch.py
2. 运行标定: python3 gravity_calibration.py

作者: EL-A3 Project
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
import time
import json
import os
from datetime import datetime
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple


@dataclass
class JointCalibrationData:
    """单个关节的标定数据"""
    joint_name: str
    joint_idx: int
    angles: List[float] = field(default_factory=list)
    torques: List[float] = field(default_factory=list)
    torque_stds: List[float] = field(default_factory=list)  # 力矩标准差（用于质量评估）
    
    # 拟合结果
    sin_coeff: float = 0.0
    cos_coeff: float = 0.0
    offset: float = 0.0
    
    # 拟合质量
    rmse: float = 0.0
    max_error: float = 0.0
    r_squared: float = 0.0


@dataclass 
class CalibrationConfig:
    """标定配置"""
    # 采样参数
    num_angles: int = 7                 # 每个关节采样角度数 (快速测试用7个)
    samples_per_angle: int = 50         # 每个角度采样次数
    settle_time: float = 1.5            # 移动后稳定等待时间 (秒)
    sample_interval: float = 0.02       # 采样间隔 (秒)
    
    # 关节范围边界余量
    margin_ratio: float = 0.15          # 避开极限位置 15%
    
    # 运动参数
    motion_duration: float = 4.0        # 单次运动时间 (秒)
    
    # 异常值剔除
    outlier_threshold: float = 2.5      # 异常值阈值 (标准差倍数)
    
    # 验证参数  
    validation_points: int = 5          # 验证测试点数
    max_drift_threshold: float = 0.1    # 最大允许漂移 (rad)
    
    # 标定顺序（从末端到基座，减少耦合影响）
    joint_order: List[int] = field(default_factory=lambda: [5, 4, 3, 2, 1, 0])
    
    # Home 点位置（所有关节都远离零点，避免限位错误）
    # L1=0.5(29°), L2=0.785(45°), L3=-0.785(-45°), L4=0.5(29°), L5=0.5(29°), L6=0.5(29°)
    safe_home_position: List[float] = field(default_factory=lambda: [0.5, 0.785, -0.785, 0.5, 0.5, 0.5])


class GravityCalibrationNode(Node):
    """重力补偿标定节点"""
    
    # 关节限位 (从 URDF 获取)
    JOINT_LIMITS = {
        'L1_joint': (-2.79253, 2.79253),
        'L2_joint': (-3.14159, 3.14159),
        'L3_joint': (-3.14159, 3.14159),
        'L4_joint': (-3.14159, 3.14159),
        'L5_joint': (-3.14159, 3.14159),
        'L6_joint': (-3.14159, 3.14159),
    }
    
    def __init__(self, config: CalibrationConfig = None):
        super().__init__('gravity_calibration')
        
        self.config = config or CalibrationConfig()
        
        self.joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 
                           'L4_joint', 'L5_joint', 'L6_joint']
        
        # 当前状态
        self.current_positions = [0.0] * 6
        self.current_velocities = [0.0] * 6
        self.current_efforts = [0.0] * 6
        self.state_received = False
        
        # 标定数据
        self.calibration_data: Dict[str, JointCalibrationData] = {}
        for i, name in enumerate(self.joint_names):
            self.calibration_data[name] = JointCalibrationData(
                joint_name=name, joint_idx=i
            )
        
        # 订阅关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Action client 用于移动关节
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        # 零力矩模式服务客户端
        self.zero_torque_client = self.create_client(
            SetBool,
            '/el_a3/set_zero_torque_mode'
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('  EL-A3 重力补偿标定程序')
        self.get_logger().info('=' * 60)
        
    def joint_state_callback(self, msg: JointState):
        """关节状态回调"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
                if len(msg.velocity) > idx:
                    self.current_velocities[i] = msg.velocity[idx]
                if len(msg.effort) > idx:
                    self.current_efforts[i] = msg.effort[idx]
        self.state_received = True
    
    def wait_for_state(self, timeout: float = 5.0) -> bool:
        """等待获取关节状态"""
        start = time.time()
        while not self.state_received and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.state_received
    
    def spin_and_collect_samples(self, duration: float) -> Tuple[List[float], List[float]]:
        """
        在指定时间内收集力矩样本
        返回: (positions, efforts) 列表
        """
        positions = []
        efforts = []
        
        start = time.time()
        while (time.time() - start) < duration:
            rclpy.spin_once(self, timeout_sec=self.config.sample_interval)
            positions.append(self.current_positions.copy())
            efforts.append(self.current_efforts.copy())
        
        return positions, efforts
    
    def move_to_position(self, target_positions: List[float], 
                         duration: float = None) -> bool:
        """
        移动到目标位置
        """
        if duration is None:
            duration = self.config.motion_duration
            
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server 不可用!')
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
            self.get_logger().error('目标被拒绝!')
            return False
        
        # 等待执行完成
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 10)
        
        return True
    
    def collect_torque_at_position(self, joint_idx: int, 
                                   target_angle: float,
                                   other_positions: List[float]) -> Tuple[float, float, float]:
        """
        在指定位置采集力矩数据
        
        返回: (mean_torque, std_torque, actual_angle)
        """
        # 构建目标位置
        target_positions = other_positions.copy()
        target_positions[joint_idx] = target_angle
        
        # 移动到目标位置
        self.get_logger().info(f'    移动到角度: {target_angle:.3f} rad ({np.degrees(target_angle):.1f}°)')
        
        if not self.move_to_position(target_positions):
            return None, None, None
        
        # 等待稳定
        time.sleep(self.config.settle_time)
        
        # 采集样本
        sample_duration = self.config.samples_per_angle * self.config.sample_interval
        positions, efforts = self.spin_and_collect_samples(sample_duration)
        
        if len(efforts) < 10:
            self.get_logger().warn('采样数量不足!')
            return None, None, None
        
        # 提取目标关节的数据
        joint_efforts = [e[joint_idx] for e in efforts]
        joint_positions = [p[joint_idx] for p in positions]
        
        # 异常值剔除
        efforts_array = np.array(joint_efforts)
        mean_effort = np.mean(efforts_array)
        std_effort = np.std(efforts_array)
        
        if std_effort > 0:
            inlier_mask = np.abs(efforts_array - mean_effort) < self.config.outlier_threshold * std_effort
            filtered_efforts = efforts_array[inlier_mask]
        else:
            filtered_efforts = efforts_array
        
        if len(filtered_efforts) < 5:
            filtered_efforts = efforts_array
        
        final_mean = np.mean(filtered_efforts)
        final_std = np.std(filtered_efforts)
        actual_angle = np.mean(joint_positions)
        
        self.get_logger().info(
            f'      力矩: {final_mean:.4f} ± {final_std:.4f} Nm, '
            f'实际角度: {actual_angle:.3f} rad'
        )
        
        return final_mean, final_std, actual_angle
    
    def generate_test_angles(self, joint_idx: int) -> List[float]:
        """
        生成测试角度序列
        完全避开 0 点附近，且不超过安全范围（防止限位使电机进入错误状态）
        """
        joint_name = self.joint_names[joint_idx]
        
        # 获取该关节的 home 位置
        home_angle = self.config.safe_home_position[joint_idx]
        
        # 零点安全距离（±0.3 rad，约 ±17°）
        zero_margin = 0.3
        
        # 安全测试范围限制（不超过 ±1.8 rad，约 ±103°，避免接近物理限位）
        max_test_angle = 1.8
        
        # 定义测试范围：完全在零点一侧，且限制最大角度
        if home_angle >= 0:
            # 在正角度范围测试
            test_lower = max(zero_margin, home_angle - 0.8)
            test_upper = min(max_test_angle, home_angle + 1.0)
        else:
            # 在负角度范围测试
            test_lower = max(-max_test_angle, home_angle - 1.0)
            test_upper = min(-zero_margin, home_angle + 0.8)
        
        # 确保范围有效
        if test_lower >= test_upper:
            # 如果范围无效，使用 home 点附近的小范围
            test_lower = home_angle - 0.5
            test_upper = home_angle + 0.5
        
        # 生成均匀分布的角度
        angles = np.linspace(test_lower, test_upper, self.config.num_angles)
        
        self.get_logger().info(f'  关节 {joint_name} 测试范围: [{test_lower:.2f}, {test_upper:.2f}] rad (安全范围)')
        
        return list(angles)
    
    def calibrate_single_joint(self, joint_idx: int, 
                               fixed_positions: List[float] = None) -> JointCalibrationData:
        """
        标定单个关节
        
        Args:
            joint_idx: 关节索引 (0-5)
            fixed_positions: 其他关节固定位置（默认使用安全 home 点）
        """
        joint_name = self.joint_names[joint_idx]
        data = self.calibration_data[joint_name]
        
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'  开始标定关节: {joint_name} (索引: {joint_idx})')
        self.get_logger().info(f'{"="*60}')
        
        # 其他关节位置（默认使用安全 home 点，避免碰撞）
        if fixed_positions is None:
            fixed_positions = self.config.safe_home_position.copy()
        
        # 生成测试角度
        test_angles = self.generate_test_angles(joint_idx)
        self.get_logger().info(f'  测试角度数: {len(test_angles)}')
        self.get_logger().info(f'  角度范围: [{test_angles[0]:.2f}, {test_angles[-1]:.2f}] rad')
        
        # 首先回到 home 点
        self.get_logger().info(f'\n  [1/3] 回到 Home 点...')
        self.move_to_position(fixed_positions)
        time.sleep(1.0)
        
        # 采集数据
        self.get_logger().info(f'\n  [2/3] 开始采集力矩数据...')
        data.angles = []
        data.torques = []
        data.torque_stds = []
        
        for i, angle in enumerate(test_angles):
            self.get_logger().info(f'\n  采样点 {i+1}/{len(test_angles)}:')
            
            torque, torque_std, actual_angle = self.collect_torque_at_position(
                joint_idx, angle, fixed_positions
            )
            
            if torque is not None:
                data.angles.append(actual_angle)
                data.torques.append(torque)
                data.torque_stds.append(torque_std)
        
        # 拟合参数
        self.get_logger().info(f'\n  [3/3] 拟合重力补偿参数...')
        self.fit_gravity_params(data)
        
        # 回到 home 点
        self.move_to_position(fixed_positions)
        
        return data
    
    def fit_gravity_params(self, data: JointCalibrationData):
        """
        使用最小二乘法拟合重力补偿参数
        
        模型: τ = a*sin(θ) + b*cos(θ) + c
        """
        if len(data.angles) < 3:
            self.get_logger().error('数据点不足，无法拟合!')
            return
        
        angles = np.array(data.angles)
        torques = np.array(data.torques)
        
        # 构建设计矩阵 A = [sin(θ), cos(θ), 1]
        A = np.column_stack([
            np.sin(angles),
            np.cos(angles),
            np.ones_like(angles)
        ])
        
        # 最小二乘求解
        params, residuals, rank, s = np.linalg.lstsq(A, torques, rcond=None)
        
        data.sin_coeff = params[0]
        data.cos_coeff = params[1]
        data.offset = params[2]
        
        # 计算拟合质量
        predicted = A @ params
        errors = torques - predicted
        
        data.rmse = np.sqrt(np.mean(errors ** 2))
        data.max_error = np.max(np.abs(errors))
        
        # R² 决定系数
        ss_res = np.sum(errors ** 2)
        ss_tot = np.sum((torques - np.mean(torques)) ** 2)
        data.r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0
        
        self.get_logger().info(f'  拟合结果:')
        self.get_logger().info(f'    sin_coeff = {data.sin_coeff:.4f}')
        self.get_logger().info(f'    cos_coeff = {data.cos_coeff:.4f}')
        self.get_logger().info(f'    offset    = {data.offset:.4f}')
        self.get_logger().info(f'  拟合质量:')
        self.get_logger().info(f'    RMSE      = {data.rmse:.4f} Nm')
        self.get_logger().info(f'    Max Error = {data.max_error:.4f} Nm')
        self.get_logger().info(f'    R²        = {data.r_squared:.4f}')
    
    def run_full_calibration(self, joints_to_calibrate: List[int] = None):
        """
        运行完整标定流程
        
        Args:
            joints_to_calibrate: 要标定的关节索引列表，默认按配置顺序
        """
        if joints_to_calibrate is None:
            joints_to_calibrate = self.config.joint_order
        
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('  开始完整重力补偿标定')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  标定顺序: {[self.joint_names[i] for i in joints_to_calibrate]}')
        self.get_logger().info(f'  每关节采样点数: {self.config.num_angles}')
        self.get_logger().info(f'  每点采样次数: {self.config.samples_per_angle}')
        
        # 安全 Home 点（避免碰撞）
        safe_home = self.config.safe_home_position.copy()
        self.get_logger().info(f'  安全 Home 点: {[f"{p:.2f}" for p in safe_home]}')
        
        # 等待连接
        if not self.wait_for_state():
            self.get_logger().error('无法获取关节状态，请检查控制器!')
            return
        
        # 等待 Action Server
        self.get_logger().info('\n等待 Action Server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action Server 不可用!')
            return
        
        self.get_logger().info('连接成功!\n')
        
        # 首先回到安全 home 点（避免碰撞）
        self.get_logger().info('首先回到安全 Home 点（避免碰撞）...')
        self.move_to_position(safe_home)
        time.sleep(2.0)
        
        # 逐关节标定
        for joint_idx in joints_to_calibrate:
            self.calibrate_single_joint(joint_idx, safe_home)
            time.sleep(1.0)
        
        # 最终回到安全 home 点
        self.get_logger().info('\n标定完成，回到安全 Home 点...')
        self.move_to_position(safe_home)
        
        # 打印结果汇总
        self.print_calibration_summary()
        
        # 保存结果
        self.save_results()
    
    def print_calibration_summary(self):
        """打印标定结果汇总"""
        print('\n' + '=' * 80)
        print('  EL-A3 重力补偿标定结果汇总')
        print('=' * 80)
        print(f'\n{"关节":<10} {"sin_coeff":>12} {"cos_coeff":>12} {"offset":>12} {"RMSE":>10} {"R²":>8}')
        print('-' * 80)
        
        for name in self.joint_names:
            data = self.calibration_data[name]
            if len(data.angles) > 0:
                print(f'{name:<10} {data.sin_coeff:>12.4f} {data.cos_coeff:>12.4f} '
                      f'{data.offset:>12.4f} {data.rmse:>10.4f} {data.r_squared:>8.4f}')
            else:
                print(f'{name:<10} {"未标定":^50}')
        
        print('-' * 80)
        
        # 生成 XACRO 参数
        print('\n生成的 XACRO 参数 (复制到 el_a3_ros2_control.xacro):')
        print('-' * 80)
        
        for name in self.joint_names:
            data = self.calibration_data[name]
            short_name = name.replace('_joint', '')
            if len(data.angles) > 0:
                print(f'          <param name="gravity_comp_{short_name}_sin">{data.sin_coeff:.4f}</param>')
                print(f'          <param name="gravity_comp_{short_name}_cos">{data.cos_coeff:.4f}</param>')
                print(f'          <param name="gravity_comp_{short_name}_offset">{data.offset:.4f}</param>')
                print()
        
        print('=' * 80)
    
    def save_results(self, filename: str = None):
        """保存标定结果到文件"""
        if filename is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'gravity_calibration_{timestamp}.json'
        
        # 准备保存数据
        results = {
            'timestamp': datetime.now().isoformat(),
            'config': {
                'num_angles': self.config.num_angles,
                'samples_per_angle': self.config.samples_per_angle,
                'settle_time': self.config.settle_time,
            },
            'joints': {}
        }
        
        for name in self.joint_names:
            data = self.calibration_data[name]
            results['joints'][name] = {
                'sin_coeff': data.sin_coeff,
                'cos_coeff': data.cos_coeff,
                'offset': data.offset,
                'rmse': data.rmse,
                'max_error': data.max_error,
                'r_squared': data.r_squared,
                'raw_data': {
                    'angles': data.angles,
                    'torques': data.torques,
                    'torque_stds': data.torque_stds,
                }
            }
        
        # 保存到 scripts 目录
        script_dir = os.path.dirname(os.path.abspath(__file__))
        filepath = os.path.join(script_dir, filename)
        
        with open(filepath, 'w') as f:
            json.dump(results, f, indent=2)
        
        self.get_logger().info(f'\n标定结果已保存到: {filepath}')


def interactive_menu(node: GravityCalibrationNode):
    """交互式菜单"""
    while True:
        print('\n' + '=' * 50)
        print('  EL-A3 重力补偿标定程序')
        print('=' * 50)
        print('  [1] 全自动标定 (所有关节)')
        print('  [2] 标定单个关节')
        print('  [3] 快速标定 (仅 L2, L3, L5)')
        print('  [4] 查看当前关节状态')
        print('  [5] 移动到安全 Home 位置')
        print('  [Q] 退出')
        print('=' * 50)
        
        choice = input('请选择: ').strip().upper()
        
        if choice == '1':
            print('\n开始全自动标定，这将需要几分钟时间...')
            confirm = input('确认开始? (y/n): ').strip().lower()
            if confirm == 'y':
                node.run_full_calibration()
        
        elif choice == '2':
            print('\n选择要标定的关节:')
            for i, name in enumerate(node.joint_names):
                print(f'  [{i}] {name}')
            try:
                joint_idx = int(input('关节索引 (0-5): ').strip())
                if 0 <= joint_idx <= 5:
                    node.calibrate_single_joint(joint_idx)
                    node.print_calibration_summary()
                else:
                    print('无效的关节索引!')
            except ValueError:
                print('请输入有效的数字!')
        
        elif choice == '3':
            print('\n快速标定模式: 仅标定 L2, L3, L5 (主要受重力影响的关节)')
            confirm = input('确认开始? (y/n): ').strip().lower()
            if confirm == 'y':
                node.run_full_calibration(joints_to_calibrate=[1, 2, 4])  # L2, L3, L5
        
        elif choice == '4':
            if node.wait_for_state(timeout=2.0):
                print('\n当前关节状态:')
                print(f'{"关节":<12} {"位置(rad)":>12} {"位置(°)":>10} {"力矩(Nm)":>12}')
                print('-' * 50)
                for i, name in enumerate(node.joint_names):
                    pos = node.current_positions[i]
                    eff = node.current_efforts[i]
                    print(f'{name:<12} {pos:>12.4f} {np.degrees(pos):>10.2f} {eff:>12.4f}')
            else:
                print('无法获取关节状态!')
        
        elif choice == '5':
            print(f'\n移动到安全 Home 位置: {node.config.safe_home_position}')
            node.move_to_position(node.config.safe_home_position)
            print('完成!')
        
        elif choice == 'Q':
            print('\n退出标定程序')
            break
        
        else:
            print('无效选择，请重试!')


def main():
    rclpy.init()
    
    # 创建配置 (快速测试用较少采样点)
    config = CalibrationConfig(
        num_angles=7,               # 7个采样角度（快速测试）
        samples_per_angle=50,       # 每个角度50个样本
        settle_time=1.5,            # 1.5秒稳定时间
        motion_duration=8.0,        # 8秒运动时间（大幅度运动需要更长时间）
        margin_ratio=0.15,          # 15%边界余量
    )
    
    node = GravityCalibrationNode(config)
    
    try:
        # 等待初始连接
        print('\n正在连接...')
        if not node.wait_for_state(timeout=10.0):
            print('错误: 无法获取关节状态，请确保控制器已启动!')
            print('启动命令: ros2 launch el_a3_description el_a3_control.launch.py')
            return
        
        print('连接成功!')
        
        # 检查命令行参数
        import sys
        if len(sys.argv) > 1:
            if sys.argv[1] == '--auto':
                # 自动模式
                node.run_full_calibration()
            elif sys.argv[1] == '--quick':
                # 快速模式
                node.run_full_calibration(joints_to_calibrate=[1, 2, 4])
            elif sys.argv[1] == '--joint':
                # 单关节模式
                if len(sys.argv) > 2:
                    joint_idx = int(sys.argv[2])
                    node.calibrate_single_joint(joint_idx)
                    node.print_calibration_summary()
        else:
            # 交互模式
            interactive_menu(node)
    
    except KeyboardInterrupt:
        print('\n\n用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
