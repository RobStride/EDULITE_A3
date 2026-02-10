#!/usr/bin/env python3
"""
EL-A3 arm inertia parameter calibration

Use the Pinocchio dynamics library to measure torques across multiple joint
configurations and fit link inertia parameters (mass, COM) for accurate gravity
compensation.

Calibration flow:
1. Generate multi-joint test configurations
2. Move to each configuration and collect steady-state torques
3. Fit inertia parameters using scipy.optimize
4. Validate fit quality and save configuration

Usage:
    python3 inertia_calibration.py [--quick] [--output PATH]
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

import numpy as np
from scipy.optimize import minimize
from dataclasses import dataclass, field
from typing import List, Dict, Tuple
import time
import argparse
import yaml
from datetime import datetime

try:
    import pinocchio as pin
    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False
    print("Error: Pinocchio library is required")
    print("  sudo apt install ros-humble-pinocchio")


@dataclass
class CalibrationConfig:
    """Calibration config"""
    urdf_path: str = "/home/wy/RS/A3/el_a3_description/urdf/el_a3.urdf"
    output_path: str = "/home/wy/RS/A3/el_a3_description/config/inertia_params.yaml"
    
    joint_names: List[str] = field(default_factory=lambda: [
        'L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint'
    ])
    
    # Reference position (avoid zero)
    home_position: List[float] = field(default_factory=lambda: [0.0, 0.785, -0.785, 0.5, 0.5, 0.0])
    
    # Sampling parameters
    samples_per_config: int = 40
    settle_time: float = 1.5
    sample_interval: float = 0.02
    motion_duration: float = 5.0


class InertiaCalibrator(Node):
    """Inertia parameter calibrator"""
    
    def __init__(self, config: CalibrationConfig):
        super().__init__('inertia_calibrator')
        
        self.config = config
        self.joint_names = config.joint_names
        
        # Current state
        self.current_positions = [0.0] * 6
        self.current_efforts = [0.0] * 6
        self.state_received = False
        
        # Pinocchio model
        self.model = None
        self.data = None
        
        # Calibration data
        self.calibration_data: List[Tuple[np.ndarray, np.ndarray]] = []
        
        # ROS2 interfaces
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        
        # Initialize Pinocchio
        if PINOCCHIO_AVAILABLE:
            self.init_pinocchio()
    
    def init_pinocchio(self):
        """Initialize Pinocchio model"""
        try:
            self.model = pin.buildModelFromUrdf(self.config.urdf_path)
            self.data = pin.Data(self.model)
            self.get_logger().info(f'Pinocchio model loaded: {self.model.njoints} joints')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to load Pinocchio: {e}')
            return False
    
    def joint_state_callback(self, msg: JointState):
        """Joint state callback"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
                if idx < len(msg.effort):
                    self.current_efforts[i] = msg.effort[idx]
        self.state_received = True
    
    def move_to_position(self, target: List[float], duration: float = None) -> bool:
        """Move to target position"""
        if duration is None:
            duration = self.config.motion_duration
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            return False
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = target
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        goal_msg.trajectory.points = [point]
        
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 10)
        
        return True
    
    def collect_samples(self) -> Tuple[np.ndarray, np.ndarray]:
        """Collect samples at current position"""
        positions = []
        efforts = []
        
        for _ in range(self.config.samples_per_config):
            rclpy.spin_once(self, timeout_sec=self.config.sample_interval)
            positions.append(self.current_positions.copy())
            efforts.append(self.current_efforts.copy())
            time.sleep(self.config.sample_interval)
        
        return np.mean(positions, axis=0), np.mean(efforts, axis=0)
    
    def check_collision_with_base(self, config: List[float]) -> bool:
        """Check if configuration may collide with base.
        
        When L2 is large (end effector down) and L3 is near 0 (forearm level),
        the end effector may collide with the base.
        
        Returns:
            True if collision is likely and should be skipped
        """
        l2 = config[1]  # L2 joint angle
        l3 = config[2]  # L3 joint angle
        
        # Heuristic rules (based on empirical collision tests):
        # Point 39: L2=1.37, L3=-0.41 collided
        # Point 49: L2=1.20, L3=-0.35 collided
        # Point 53: L2=1.43, L3=-0.55 collided
        
        # Rule 1: L2 >= 1.2 and L3 > -0.4 -> collision
        if l2 >= 1.2 and l3 > -0.4:
            return True
        
        # Rule 2: L2 > 1.35 and L3 > -0.6 -> collision
        if l2 > 1.35 and l3 > -0.6:
            return True
        
        # Rule 3: region with L2 >= 1.2 and L3 >= -0.55
        if l2 >= 1.2 and l3 >= -0.55 and (l2 + l3 * 0.5) > 0.9:
            return True
        
        return False
    
    def generate_test_configurations(self, mode: str = 'full') -> List[List[float]]:
        """Generate test configurations.
        
        Args:
            mode: 'quick' (~20), 'full' (~46), 'high' (~80), 'ultra' (~120)
        """
        configs = []
        home = np.array(self.config.home_position)
        
        if mode == 'quick':
            # Quick mode: ~20 test points
            l2_range = np.linspace(0.4, 1.4, 6)
            l3_range = np.linspace(-1.2, -0.4, 5)
            l4_range = np.array([0.3, 0.7, 1.0])
            l5_range = np.array([0.3, 0.6])
            grid_step = 2
        elif mode == 'high':
            # High precision mode: ~80 test points
            l2_range = np.linspace(0.25, 1.55, 12)
            l3_range = np.linspace(-1.35, -0.25, 12)
            l4_range = np.linspace(0.25, 1.25, 7)
            l5_range = np.linspace(0.25, 1.05, 5)
            grid_step = 2
        elif mode == 'ultra':
            # Ultra precision mode: ~120 test points
            l2_range = np.linspace(0.2, 1.6, 15)
            l3_range = np.linspace(-1.4, -0.2, 15)
            l4_range = np.linspace(0.2, 1.3, 8)
            l5_range = np.linspace(0.2, 1.1, 6)
            grid_step = 2
        else:
            # Full mode: ~46 test points
            l2_range = np.linspace(0.3, 1.5, 10)
            l3_range = np.linspace(-1.3, -0.3, 10)
            l4_range = np.linspace(0.3, 1.2, 6)
            l5_range = np.linspace(0.3, 1.0, 5)
            grid_step = 2
        
        # Stage 1: vary L2 only
        self.get_logger().info('Generating L2 test points...')
        for l2 in l2_range:
            cfg = home.copy()
            cfg[1] = l2
            configs.append(cfg.tolist())
        
        # Stage 2: vary L3 only
        self.get_logger().info('Generating L3 test points...')
        for l3 in l3_range:
            cfg = home.copy()
            cfg[2] = l3
            configs.append(cfg.tolist())
        
        # Stage 3: L2+L3 grid combinations
        self.get_logger().info('Generating L2+L3 grid combinations...')
        l2_grid = l2_range[::grid_step]
        l3_grid = l3_range[::grid_step]
        for l2 in l2_grid:
            for l3 in l3_grid:
                cfg = home.copy()
                cfg[1] = l2
                cfg[2] = l3
                configs.append(cfg.tolist())
        
        # Stage 4: L4+L5 combinations
        self.get_logger().info('Generating L4+L5 combination points...')
        for l4 in l4_range:
            for l5 in l5_range:
                cfg = home.copy()
                cfg[3] = l4
                cfg[4] = l5
                configs.append(cfg.tolist())
        
        # De-duplicate
        unique_configs = []
        for cfg in configs:
            is_dup = False
            for uc in unique_configs:
                if np.allclose(cfg, uc, atol=0.05):
                    is_dup = True
                    break
            if not is_dup:
                unique_configs.append(cfg)
        
        # Filter configs that may collide with base
        safe_configs = []
        skipped = 0
        for cfg in unique_configs:
            if self.check_collision_with_base(cfg):
                skipped += 1
                self.get_logger().info(f'  Skipping collision config: L2={cfg[1]:.2f}, L3={cfg[2]:.2f}')
            else:
                safe_configs.append(cfg)
        
        if skipped > 0:
            self.get_logger().info(f'Filtered {skipped} potentially colliding configs')
        
        self.get_logger().info(f'Total safe test points: {len(safe_configs)}')
        return safe_configs
    
    def generate_wrist_configurations(self) -> List[List[float]]:
        """Generate wrist joint (L4/L5/L6) test configurations.
        
        L2/L3 fixed at home position; only vary L4/L5/L6.
        """
        configs = []
        home = np.array(self.config.home_position)
        
        # L4/L5/L6 test ranges (finer)
        l4_range = np.linspace(0.0, 1.5, 12)    # 12 points
        l5_range = np.linspace(0.0, 1.2, 10)    # 10 points
        l6_range = np.linspace(-0.8, 0.8, 8)    # 8 points
        
        # Stage 1: vary L4 only
        self.get_logger().info('Generating L4 test points...')
        for l4 in l4_range:
            cfg = home.copy()
            cfg[3] = l4
            configs.append(cfg.tolist())
        
        # Stage 2: vary L5 only
        self.get_logger().info('Generating L5 test points...')
        for l5 in l5_range:
            cfg = home.copy()
            cfg[4] = l5
            configs.append(cfg.tolist())
        
        # Stage 3: vary L6 only
        self.get_logger().info('Generating L6 test points...')
        for l6 in l6_range:
            cfg = home.copy()
            cfg[5] = l6
            configs.append(cfg.tolist())
        
        # Stage 4: L4+L5 grid combinations
        self.get_logger().info('Generating L4+L5 combination points...')
        l4_grid = l4_range[::2]  # 6 points
        l5_grid = l5_range[::2]  # 5 points
        for l4 in l4_grid:
            for l5 in l5_grid:
                cfg = home.copy()
                cfg[3] = l4
                cfg[4] = l5
                configs.append(cfg.tolist())
        
        # Stage 5: L4+L5+L6 combos (representative points)
        self.get_logger().info('Generating L4+L5+L6 combination points...')
        l4_combo = [0.3, 0.8, 1.2]
        l5_combo = [0.3, 0.7]
        l6_combo = [-0.5, 0.0, 0.5]
        for l4 in l4_combo:
            for l5 in l5_combo:
                for l6 in l6_combo:
                    cfg = home.copy()
                    cfg[3] = l4
                    cfg[4] = l5
                    cfg[5] = l6
                    configs.append(cfg.tolist())
        
        # De-duplicate
        unique_configs = []
        for cfg in configs:
            is_dup = False
            for uc in unique_configs:
                if np.allclose(cfg, uc, atol=0.05):
                    is_dup = True
                    break
            if not is_dup:
                unique_configs.append(cfg)
        
        self.get_logger().info(f'Total wrist test points: {len(unique_configs)}')
        return unique_configs
    
    def compute_gravity_with_params(self, q: np.ndarray, params: np.ndarray) -> np.ndarray:
        """Compute gravity torque with given inertia parameters"""
        if self.model is None:
            return np.zeros(6)
        
        # Temporarily modify model inertias
        # params format: [L2_mass, L2_com_x, L3_mass, L3_com_x, L3_com_y,
        #                L4_mass, L4_com_x, L4_com_y, L5_mass, L5_com_z, L6_mass, L6_com_z]
        
        # Save originals
        original_inertias = []
        for i in range(1, min(7, self.model.nbodies)):
            original_inertias.append({
                'mass': self.model.inertias[i].mass,
                'lever': self.model.inertias[i].lever.copy()
            })
        
        # Apply new parameters
        try:
            # L2 (index 2 in model)
            if len(params) > 0:
                self.model.inertias[2].mass = params[0]
            if len(params) > 1:
                self.model.inertias[2].lever[0] = params[1]
            
            # L3 (index 3 in model)
            if len(params) > 2:
                self.model.inertias[3].mass = params[2]
            if len(params) > 3:
                self.model.inertias[3].lever[0] = params[3]
            if len(params) > 4:
                self.model.inertias[3].lever[1] = params[4]
            
            # L4 (index 4 in model)
            if len(params) > 5:
                self.model.inertias[4].mass = params[5]
            if len(params) > 6:
                self.model.inertias[4].lever[0] = params[6]
            if len(params) > 7:
                self.model.inertias[4].lever[1] = params[7]
            
            # L5 (index 5 in model)
            if len(params) > 8:
                self.model.inertias[5].mass = params[8]
            if len(params) > 9:
                self.model.inertias[5].lever[2] = params[9]
            
            # L6 (index 6 in model)
            if len(params) > 10:
                self.model.inertias[6].mass = params[10]
            if len(params) > 11:
                self.model.inertias[6].lever[2] = params[11]
            
            # Recreate data
            self.data = pin.Data(self.model)
            
            # Compute gravity torque
            q_pin = np.array(q[:self.model.nq])
            v = np.zeros(self.model.nv)
            a = np.zeros(self.model.nv)
            tau = pin.rnea(self.model, self.data, q_pin, v, a)
            
        finally:
            # Restore originals
            for i, orig in enumerate(original_inertias):
                self.model.inertias[i + 1].mass = orig['mass']
                self.model.inertias[i + 1].lever = orig['lever']
            self.data = pin.Data(self.model)
        
        return tau[:6]
    
    def objective_function(self, params: np.ndarray) -> float:
        """Objective: MSE between measured and predicted torques"""
        total_error = 0.0
        
        for positions, measured_efforts in self.calibration_data:
            predicted = self.compute_gravity_with_params(positions, params)
            
            # Only compute error for L2-L6 (L1 rotates around Z, no gravity effect)
            for i in range(1, 6):
                error = measured_efforts[i] - predicted[i]
                total_error += error ** 2
        
        return total_error
    
    def wrist_objective_function(self, wrist_params: np.ndarray, fixed_l2l3_params: np.ndarray) -> float:
        """Wrist calibration objective: only optimize L4/L5/L6.
        
        Args:
            wrist_params: [L4_mass, L4_com_x, L4_com_y, L5_mass, L5_com_z, L6_mass, L6_com_z] (7 params)
            fixed_l2l3_params: [L2_mass, L2_com_x, L3_mass, L3_com_x, L3_com_y] (5 params)
        """
        # Combine full params
        full_params = np.concatenate([fixed_l2l3_params, wrist_params])
        
        total_error = 0.0
        for positions, measured_efforts in self.calibration_data:
            predicted = self.compute_gravity_with_params(positions, full_params)
            
            # Only compute error for L4-L6 (indices 3, 4, 5)
            for i in range(3, 6):
                error = measured_efforts[i] - predicted[i]
                total_error += error ** 2
        
        return total_error
    
    def run_calibration(self, mode: str = 'full') -> Dict:
        """Run full calibration flow.
        
        Args:
            mode: 'quick', 'full', 'high', 'ultra'
        """
        mode_names = {'quick': 'quick', 'full': 'full', 'high': 'high', 'ultra': 'ultra'}
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  EL-A3 inertia calibration ({mode_names.get(mode, mode)} mode)')
        self.get_logger().info('=' * 60)
        
        if self.model is None:
            self.get_logger().error('Pinocchio model not initialized')
            return {}
        
        # Wait for joint states
        self.get_logger().info('Waiting for joint states...')
        start = time.time()
        while not self.state_received and (time.time() - start) < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.state_received:
            self.get_logger().error('Failed to get joint states')
            return {}
        
        # Generate test configurations
        test_configs = self.generate_test_configurations(mode)
        
        # Collect data
        self.get_logger().info(f'\nStarting data collection ({len(test_configs)} test points)...')
        self.calibration_data.clear()
        
        for idx, config in enumerate(test_configs):
            self.get_logger().info(f'\n[{idx+1}/{len(test_configs)}] Target: L2={config[1]:.2f}, L3={config[2]:.2f}, L4={config[3]:.2f}, L5={config[4]:.2f}')
            
            if not self.move_to_position(config):
                self.get_logger().warn('  Move failed, skipping')
                continue
            
            time.sleep(self.config.settle_time)
            
            positions, efforts = self.collect_samples()
            self.calibration_data.append((positions, efforts))
            
            self.get_logger().info(f'  Actual: L2={positions[1]:.3f}, L3={positions[2]:.3f}')
            self.get_logger().info(f'  Torque: L2={efforts[1]:.4f}, L3={efforts[2]:.4f}, L4={efforts[3]:.4f}')
        
        # Return to home
        self.get_logger().info('\nReturning to Home position...')
        self.move_to_position(self.config.home_position)
        
        if len(self.calibration_data) < 10:
            self.get_logger().error('Too few data points, cannot calibrate')
            return {}
        
        # Optimize fit
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('  Optimizing inertia parameters')
        self.get_logger().info('=' * 60)
        
        # Initial values (from URDF defaults)
        initial_params = np.array([
            0.8348, 0.095,      # L2: mass, com_x
            0.1976, -0.056, 0.049,  # L3: mass, com_x, com_y
            0.4606, -0.024, 0.031,  # L4: mass, com_x, com_y
            0.0180, 0.018,      # L5: mass, com_z
            0.5313, 0.070       # L6: mass, com_z
        ])
        
        # Parameter bounds
        bounds = [
            (0.1, 2.0), (-0.2, 0.2),     # L2
            (0.05, 0.5), (-0.15, 0.0), (0.0, 0.1),  # L3
            (0.1, 1.0), (-0.1, 0.0), (0.0, 0.1),   # L4
            (0.005, 0.1), (0.0, 0.05),   # L5
            (0.1, 1.0), (0.0, 0.15)      # L6
        ]
        
        self.get_logger().info('Initial parameters:')
        self.get_logger().info(f'  L2: mass={initial_params[0]:.4f}, com_x={initial_params[1]:.4f}')
        self.get_logger().info(f'  L3: mass={initial_params[2]:.4f}, com_x={initial_params[3]:.4f}, com_y={initial_params[4]:.4f}')
        self.get_logger().info(f'  L4: mass={initial_params[5]:.4f}, com_x={initial_params[6]:.4f}, com_y={initial_params[7]:.4f}')
        self.get_logger().info(f'  L5: mass={initial_params[8]:.4f}, com_z={initial_params[9]:.4f}')
        self.get_logger().info(f'  L6: mass={initial_params[10]:.4f}, com_z={initial_params[11]:.4f}')
        
        initial_error = self.objective_function(initial_params)
        self.get_logger().info(f'\nInitial error: {initial_error:.6f}')
        
        # Optimize
        self.get_logger().info('Starting optimization...')
        result = minimize(
            self.objective_function,
            initial_params,
            method='L-BFGS-B',
            bounds=bounds,
            options={'maxiter': 500, 'disp': False}
        )
        
        optimized_params = result.x
        final_error = result.fun
        
        self.get_logger().info(f'Optimization done! Final error: {final_error:.6f} (reduced {(1 - final_error/initial_error)*100:.1f}%)')
        
        # Compute RMSE and R²
        n_samples = len(self.calibration_data) * 5  # L2-L6
        rmse = np.sqrt(final_error / n_samples)
        
        # Compute SS_tot
        all_measured = []
        for _, efforts in self.calibration_data:
            all_measured.extend(efforts[1:6])
        mean_measured = np.mean(all_measured)
        ss_tot = sum((m - mean_measured) ** 2 for m in all_measured)
        r_squared = 1 - final_error / ss_tot if ss_tot > 0 else 0
        
        self.get_logger().info(f'RMSE: {rmse:.4f} Nm')
        self.get_logger().info(f'R²: {r_squared:.4f}')
        
        # Assemble results
        results = {
            'L2': {'mass': float(optimized_params[0]), 'com': [float(optimized_params[1]), 0.0, 0.0]},
            'L3': {'mass': float(optimized_params[2]), 'com': [float(optimized_params[3]), float(optimized_params[4]), 0.003]},
            'L4': {'mass': float(optimized_params[5]), 'com': [float(optimized_params[6]), float(optimized_params[7]), 0.0]},
            'L5': {'mass': float(optimized_params[8]), 'com': [0.004, 0.0, float(optimized_params[9])]},
            'L6': {'mass': float(optimized_params[10]), 'com': [0.0, -0.001, float(optimized_params[11])]},
            'calibration_info': {
                'date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'num_samples': len(self.calibration_data),
                'rmse': float(rmse),
                'r_squared': float(r_squared)
            }
        }
        
        self.get_logger().info('\nOptimized parameters:')
        for joint in ['L2', 'L3', 'L4', 'L5', 'L6']:
            p = results[joint]
            self.get_logger().info(f'  {joint}: mass={p["mass"]:.4f}, com={p["com"]}')
        
        return results
    
    def run_wrist_calibration(self) -> Dict:
        """Run wrist (L4/L5/L6) calibration.
        
        Keep calibrated L2/L3 and only optimize L4/L5/L6.
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('  EL-A3 wrist calibration (L4/L5/L6)')
        self.get_logger().info('=' * 60)
        
        if self.model is None:
            self.get_logger().error('Pinocchio model not initialized')
            return {}
        
        # Load L2/L3 parameters from existing config
        existing_params = self.load_existing_params()
        if existing_params is None:
            self.get_logger().warn('Failed to read existing params, using URDF defaults')
            fixed_l2l3_params = np.array([0.8348, 0.095, 0.1976, -0.056, 0.049])
        else:
            fixed_l2l3_params = np.array([
                existing_params['L2']['mass'], existing_params['L2']['com'][0],
                existing_params['L3']['mass'], existing_params['L3']['com'][0], existing_params['L3']['com'][1]
            ])
            self.get_logger().info('Using calibrated L2/L3 parameters:')
            self.get_logger().info(f'  L2: mass={fixed_l2l3_params[0]:.4f}, com_x={fixed_l2l3_params[1]:.4f}')
            self.get_logger().info(f'  L3: mass={fixed_l2l3_params[2]:.4f}, com_x={fixed_l2l3_params[3]:.4f}, com_y={fixed_l2l3_params[4]:.4f}')
        
        # Wait for joint states
        self.get_logger().info('Waiting for joint states...')
        start = time.time()
        while not self.state_received and (time.time() - start) < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.state_received:
            self.get_logger().error('Failed to get joint states')
            return {}
        
        # Generate wrist test configurations
        test_configs = self.generate_wrist_configurations()
        
        # Collect data
        self.get_logger().info(f'\nStarting wrist data collection ({len(test_configs)} test points)...')
        self.calibration_data.clear()
        
        for idx, config in enumerate(test_configs):
            self.get_logger().info(f'\n[{idx+1}/{len(test_configs)}] Target: L4={config[3]:.2f}, L5={config[4]:.2f}, L6={config[5]:.2f}')
            
            if not self.move_to_position(config):
                self.get_logger().warn('  Move failed, skipping')
                continue
            
            time.sleep(self.config.settle_time)
            
            positions, efforts = self.collect_samples()
            self.calibration_data.append((positions, efforts))
            
            self.get_logger().info(f'  Actual: L4={positions[3]:.3f}, L5={positions[4]:.3f}, L6={positions[5]:.3f}')
            self.get_logger().info(f'  Torque: L4={efforts[3]:.4f}, L5={efforts[4]:.4f}, L6={efforts[5]:.4f}')
        
        # Return to home
        self.get_logger().info('\nReturning to Home position...')
        self.move_to_position(self.config.home_position)
        
        if len(self.calibration_data) < 10:
            self.get_logger().error('Too few data points, cannot calibrate')
            return {}
        
        # Optimize fit
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('  Optimizing wrist inertia parameters (L4/L5/L6)')
        self.get_logger().info('=' * 60)
        
        # Wrist parameter initial values
        initial_wrist_params = np.array([
            0.4606, -0.024, 0.031,  # L4: mass, com_x, com_y
            0.0180, 0.018,          # L5: mass, com_z
            0.5313, 0.070           # L6: mass, com_z
        ])
        
        # Parameter bounds
        wrist_bounds = [
            (0.1, 1.5), (-0.15, 0.05), (0.0, 0.1),   # L4
            (0.005, 0.2), (0.0, 0.08),                # L5
            (0.1, 1.5), (0.0, 0.2)                    # L6
        ]
        
        self.get_logger().info('Initial wrist parameters:')
        self.get_logger().info(f'  L4: mass={initial_wrist_params[0]:.4f}, com_x={initial_wrist_params[1]:.4f}, com_y={initial_wrist_params[2]:.4f}')
        self.get_logger().info(f'  L5: mass={initial_wrist_params[3]:.4f}, com_z={initial_wrist_params[4]:.4f}')
        self.get_logger().info(f'  L6: mass={initial_wrist_params[5]:.4f}, com_z={initial_wrist_params[6]:.4f}')
        
        initial_error = self.wrist_objective_function(initial_wrist_params, fixed_l2l3_params)
        self.get_logger().info(f'\nInitial error: {initial_error:.6f}')
        
        # Optimize
        self.get_logger().info('Starting optimization...')
        result = minimize(
            lambda x: self.wrist_objective_function(x, fixed_l2l3_params),
            initial_wrist_params,
            method='L-BFGS-B',
            bounds=wrist_bounds,
            options={'maxiter': 500, 'disp': False}
        )
        
        optimized_wrist = result.x
        final_error = result.fun
        
        self.get_logger().info(f'Optimization done! Final error: {final_error:.6f} (reduced {(1 - final_error/initial_error)*100:.1f}%)')
        
        # Compute RMSE and R²
        n_samples = len(self.calibration_data) * 3  # L4-L6
        rmse = np.sqrt(final_error / n_samples)
        
        # Compute SS_tot
        all_measured = []
        for _, efforts in self.calibration_data:
            all_measured.extend(efforts[3:6])
        mean_measured = np.mean(all_measured)
        ss_tot = sum((m - mean_measured) ** 2 for m in all_measured)
        r_squared = 1 - final_error / ss_tot if ss_tot > 0 else 0
        
        self.get_logger().info(f'RMSE: {rmse:.4f} Nm')
        self.get_logger().info(f'R²: {r_squared:.4f}')
        
        # Assemble results (keep L2/L3, update L4/L5/L6)
        results = {
            'L2': {'mass': float(fixed_l2l3_params[0]), 'com': [float(fixed_l2l3_params[1]), 0.0, 0.0]},
            'L3': {'mass': float(fixed_l2l3_params[2]), 'com': [float(fixed_l2l3_params[3]), float(fixed_l2l3_params[4]), 0.003]},
            'L4': {'mass': float(optimized_wrist[0]), 'com': [float(optimized_wrist[1]), float(optimized_wrist[2]), 0.0]},
            'L5': {'mass': float(optimized_wrist[3]), 'com': [0.004, 0.0, float(optimized_wrist[4])]},
            'L6': {'mass': float(optimized_wrist[5]), 'com': [0.0, -0.001, float(optimized_wrist[6])]},
            'calibration_info': {
                'date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'num_samples': len(self.calibration_data),
                'rmse': float(rmse),
                'r_squared': float(r_squared)
            }
        }
        
        self.get_logger().info('\nOptimized wrist parameters:')
        for joint in ['L4', 'L5', 'L6']:
            p = results[joint]
            self.get_logger().info(f'  {joint}: mass={p["mass"]:.4f}, com={p["com"]}')
        
        return results
    
    def load_existing_params(self) -> Dict:
        """Load existing inertia parameters from config file"""
        try:
            config_path = self.config.output_path
            with open(config_path, 'r') as f:
                lines = f.readlines()
            
            params = {'L2': {}, 'L3': {}, 'L4': {}, 'L5': {}, 'L6': {}}
            current_joint = None
            
            for line in lines:
                line = line.strip()
                if line.startswith('L') and ':' in line and 'mass' not in line and 'com' not in line:
                    for j in ['L2', 'L3', 'L4', 'L5', 'L6']:
                        if line.startswith(j + ':'):
                            current_joint = j
                            break
                elif current_joint and 'mass:' in line:
                    mass_str = line.split('mass:')[1].strip()
                    params[current_joint]['mass'] = float(mass_str)
                elif current_joint and 'com:' in line:
                    com_str = line.split('com:')[1].strip()
                    com_str = com_str.strip('[]')
                    params[current_joint]['com'] = [float(x.strip()) for x in com_str.split(',')]
            
            # Validate parameter completeness
            for j in ['L2', 'L3']:
                if 'mass' not in params[j] or 'com' not in params[j]:
                    return None
            
            return params
        except Exception as e:
            self.get_logger().warn(f'Failed to read config file: {e}')
            return None
    
    def save_results(self, results: Dict, output_path: str):
        """Save calibration results to YAML file"""
        yaml_content = f"""# EL-A3 arm inertia parameters
# For Pinocchio gravity compensation
# These parameters are fitted by the calibration program and override URDF defaults
#
# Calibration date: {results['calibration_info']['date']}
# Calibration script: scripts/inertia_calibration.py
# RMSE: {results['calibration_info']['rmse']:.4f} Nm
# R²: {results['calibration_info']['r_squared']:.4f}

# Whether to enable calibrated inertia parameters
use_calibrated_params: true

# Inertia parameters for each link
inertia_params:
  # L2 - upper arm
  L2:
    mass: {results['L2']['mass']:.4f}
    com: {results['L2']['com']}
  
  # L3 - forearm
  L3:
    mass: {results['L3']['mass']:.4f}
    com: {results['L3']['com']}
  
  # L4 - wrist roll
  L4:
    mass: {results['L4']['mass']:.4f}
    com: {results['L4']['com']}
  
  # L5 - wrist pitch
  L5:
    mass: {results['L5']['mass']:.4f}
    com: {results['L5']['com']}
  
  # L6 - end-effector yaw
  L6:
    mass: {results['L6']['mass']:.4f}
    com: {results['L6']['com']}

# Calibration statistics
calibration_info:
  date: "{results['calibration_info']['date']}"
  num_samples: {results['calibration_info']['num_samples']}
  rmse: {results['calibration_info']['rmse']:.4f}
  r_squared: {results['calibration_info']['r_squared']:.4f}
"""
        
        with open(output_path, 'w') as f:
            f.write(yaml_content)
        
        self.get_logger().info(f'\nResults saved to: {output_path}')


def main():
    parser = argparse.ArgumentParser(description='EL-A3 inertia calibration')
    parser.add_argument('--quick', action='store_true', help='Quick mode (~20 points)')
    parser.add_argument('--high', action='store_true', help='High precision (~80 points)')
    parser.add_argument('--ultra', action='store_true', help='Ultra precision (~120 points)')
    parser.add_argument('--wrist', action='store_true', help='Wrist mode (only L4/L5/L6)')
    parser.add_argument('--samples', type=int, default=40, help='Samples per point (default 40)')
    parser.add_argument('--output', type=str, 
                        default='/home/wy/RS/A3/el_a3_description/config/inertia_params.yaml',
                        help='Output config file path')
    parser.add_argument('--urdf', type=str,
                        default='/home/wy/RS/A3/el_a3_description/urdf/el_a3.urdf',
                        help='URDF file path')
    
    args = parser.parse_args()
    
    # Determine mode
    if args.ultra:
        mode = 'ultra'
    elif args.high:
        mode = 'high'
    elif args.quick:
        mode = 'quick'
    else:
        mode = 'full'
    
    if not PINOCCHIO_AVAILABLE:
        return
    
    rclpy.init()
    
    config = CalibrationConfig(
        urdf_path=args.urdf,
        output_path=args.output,
        samples_per_config=args.samples
    )
    
    calibrator = InertiaCalibrator(config)
    
    try:
        if args.wrist:
            # Wrist calibration mode
            results = calibrator.run_wrist_calibration()
            mode_desc = "Wrist (L4/L5/L6)"
        else:
            # Full calibration mode
            results = calibrator.run_calibration(mode=mode)
            mode_desc = "Full"
        
        if results:
            calibrator.save_results(results, args.output)
            print("\n" + "=" * 60)
            print(f"  {mode_desc} calibration completed!")
            print("  Please restart the controller to apply new parameters")
            print("=" * 60)
        else:
            print("\nCalibration failed")
    
    except KeyboardInterrupt:
        print("\nCalibration interrupted")
    
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
