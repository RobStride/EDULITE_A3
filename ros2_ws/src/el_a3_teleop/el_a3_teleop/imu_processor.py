#!/usr/bin/env python3
"""
IMU 数据处理库
包含：零飘标定、滤波器、Madgwick 姿态估计

用于 Switch Joy-Con IMU 数据处理
"""

import numpy as np
from scipy.spatial.transform import Rotation
from typing import Tuple, Optional
from enum import Enum
import time


class CalibrationState(Enum):
    """标定状态"""
    NOT_STARTED = 0
    IN_PROGRESS = 1
    COMPLETED = 2
    FAILED = 3


class IMUCalibrator:
    """
    IMU 零飘标定器
    
    通过采集静止状态下的 IMU 数据，计算陀螺仪零偏和加速度计偏差
    """
    
    def __init__(self, calibration_samples: int = 200, timeout: float = 5.0):
        """
        初始化标定器
        
        Args:
            calibration_samples: 标定所需采样数
            timeout: 标定超时时间(秒)
        """
        self.calibration_samples = calibration_samples
        self.timeout = timeout
        
        # 标定结果
        self.gyro_bias = np.zeros(3)   # 陀螺仪零偏 [x, y, z]
        self.accel_bias = np.zeros(3)  # 加速度计偏差 [x, y, z]
        
        # 标定状态
        self.state = CalibrationState.NOT_STARTED
        self.samples_collected = 0
        self.start_time: Optional[float] = None
        
        # 采样缓冲
        self._accel_buffer = []
        self._gyro_buffer = []
        
        # 静止检测阈值
        self.gyro_variance_threshold = 0.01  # rad/s
        self.accel_variance_threshold = 0.5  # m/s²
    
    def reset(self):
        """重置标定器"""
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)
        self.state = CalibrationState.NOT_STARTED
        self.samples_collected = 0
        self.start_time = None
        self._accel_buffer = []
        self._gyro_buffer = []
    
    def start_calibration(self):
        """开始标定"""
        self.reset()
        self.state = CalibrationState.IN_PROGRESS
        self.start_time = time.time()
    
    def add_sample(self, accel: np.ndarray, gyro: np.ndarray) -> CalibrationState:
        """
        添加标定样本
        
        Args:
            accel: 加速度计数据 [ax, ay, az] (m/s²)
            gyro: 陀螺仪数据 [gx, gy, gz] (rad/s)
        
        Returns:
            当前标定状态
        """
        if self.state != CalibrationState.IN_PROGRESS:
            return self.state
        
        # 检查超时
        if time.time() - self.start_time > self.timeout:
            self.state = CalibrationState.FAILED
            return self.state
        
        # 添加样本
        self._accel_buffer.append(accel.copy())
        self._gyro_buffer.append(gyro.copy())
        self.samples_collected += 1
        
        # 检查是否收集够样本
        if self.samples_collected >= self.calibration_samples:
            self._compute_calibration()
        
        return self.state
    
    def _compute_calibration(self):
        """计算标定参数"""
        accels = np.array(self._accel_buffer)
        gyros = np.array(self._gyro_buffer)
        
        # 检查设备是否静止（通过方差）
        gyro_var = np.var(gyros, axis=0)
        accel_var = np.var(accels, axis=0)
        
        if np.any(gyro_var > self.gyro_variance_threshold):
            # 设备可能在移动，标定失败
            self.state = CalibrationState.FAILED
            return
        
        # 计算陀螺仪零偏 = 静止时平均值
        self.gyro_bias = np.mean(gyros, axis=0)
        
        # 计算加速度计偏差
        # 静止时加速度应为重力方向 [0, 0, g] 或其他方向
        # 这里假设 Z 轴向上，静止时 az ≈ 9.81
        mean_accel = np.mean(accels, axis=0)
        
        # 计算重力矢量的大小
        g_measured = np.linalg.norm(mean_accel)
        g_expected = 9.81
        
        # 如果测量值合理（重力方向检测）
        if 8.0 < g_measured < 12.0:
            # 假设 Z 轴朝上时的期望值
            # 实际应用中可能需要根据 Joy-Con 的持握方式调整
            expected_accel = mean_accel / g_measured * g_expected
            self.accel_bias = mean_accel - expected_accel
            self.state = CalibrationState.COMPLETED
        else:
            self.state = CalibrationState.FAILED
    
    def apply(self, accel: np.ndarray, gyro: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        应用标定补偿
        
        Args:
            accel: 原始加速度计数据
            gyro: 原始陀螺仪数据
        
        Returns:
            补偿后的 (accel, gyro)
        """
        if self.state != CalibrationState.COMPLETED:
            return accel, gyro
        
        return accel - self.accel_bias, gyro - self.gyro_bias
    
    @property
    def is_calibrated(self) -> bool:
        """是否已完成标定"""
        return self.state == CalibrationState.COMPLETED
    
    @property
    def progress(self) -> float:
        """标定进度 (0.0 - 1.0)"""
        if self.state != CalibrationState.IN_PROGRESS:
            return 1.0 if self.is_calibrated else 0.0
        return min(1.0, self.samples_collected / self.calibration_samples)


class LowPassFilter:
    """
    一阶低通滤波器 (指数移动平均)
    
    y[n] = α * x[n] + (1 - α) * y[n-1]
    """
    
    def __init__(self, alpha: float = 0.5, initial_value: Optional[np.ndarray] = None):
        """
        初始化滤波器
        
        Args:
            alpha: 滤波系数 (0-1)，越大越接近原始值
            initial_value: 初始值
        """
        self.alpha = np.clip(alpha, 0.0, 1.0)
        self.value = initial_value
        self.initialized = initial_value is not None
    
    def update(self, x: np.ndarray) -> np.ndarray:
        """
        更新滤波器
        
        Args:
            x: 新的输入值
        
        Returns:
            滤波后的值
        """
        if not self.initialized:
            self.value = x.copy()
            self.initialized = True
            return self.value
        
        self.value = self.alpha * x + (1 - self.alpha) * self.value
        return self.value
    
    def reset(self, value: Optional[np.ndarray] = None):
        """重置滤波器"""
        self.value = value
        self.initialized = value is not None


class ComplementaryFilter:
    """
    互补滤波器
    
    融合加速度计和陀螺仪数据估计姿态
    """
    
    def __init__(self, alpha: float = 0.98, sample_rate: float = 60.0):
        """
        初始化互补滤波器
        
        Args:
            alpha: 陀螺仪权重 (0-1)，越大越信任陀螺仪
            sample_rate: 采样频率 (Hz)
        """
        self.alpha = alpha
        self.dt = 1.0 / sample_rate
        
        # 当前姿态 (欧拉角)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.initialized = False
    
    def update(self, accel: np.ndarray, gyro: np.ndarray) -> Tuple[float, float, float]:
        """
        更新姿态估计
        
        Args:
            accel: 加速度计数据 [ax, ay, az] (m/s²)
            gyro: 陀螺仪数据 [gx, gy, gz] (rad/s)
        
        Returns:
            (roll, pitch, yaw) 欧拉角 (rad)
        """
        # 从加速度计计算 roll 和 pitch
        accel_roll = np.arctan2(accel[1], accel[2])
        accel_pitch = np.arctan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2))
        
        if not self.initialized:
            self.roll = accel_roll
            self.pitch = accel_pitch
            self.yaw = 0.0
            self.initialized = True
            return self.roll, self.pitch, self.yaw
        
        # 陀螺仪积分
        gyro_roll = self.roll + gyro[0] * self.dt
        gyro_pitch = self.pitch + gyro[1] * self.dt
        gyro_yaw = self.yaw + gyro[2] * self.dt
        
        # 互补滤波融合
        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch
        self.yaw = gyro_yaw  # Yaw 只能靠陀螺仪积分（无磁力计）
        
        return self.roll, self.pitch, self.yaw
    
    def reset(self):
        """重置滤波器"""
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.initialized = False


class MadgwickFilter:
    """
    Madgwick AHRS 姿态估计算法
    
    使用梯度下降融合加速度计和陀螺仪数据
    """
    
    def __init__(self, sample_rate: float = 60.0, beta: float = 0.1):
        """
        初始化 Madgwick 滤波器
        
        Args:
            sample_rate: 采样频率 (Hz)
            beta: 算法增益，越大收敛越快但噪声越大
        """
        self.sample_rate = sample_rate
        self.dt = 1.0 / sample_rate
        self.beta = beta
        
        # 四元数 [w, x, y, z]
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        
        # 参考姿态（用于计算相对变化）
        self.q_ref = None
        self.use_relative = False
    
    def update(self, accel: np.ndarray, gyro: np.ndarray) -> Tuple[float, float, float]:
        """
        更新姿态估计
        
        Args:
            accel: 加速度计数据 [ax, ay, az] (m/s²)
            gyro: 陀螺仪数据 [gx, gy, gz] (rad/s)
        
        Returns:
            (roll, pitch, yaw) 欧拉角 (rad)
        """
        q = self.q.copy()
        
        # 归一化加速度
        accel_norm = np.linalg.norm(accel)
        if accel_norm > 0:
            a = accel / accel_norm
        else:
            a = accel
        
        # 四元数分量
        q0, q1, q2, q3 = q
        
        # 陀螺仪四元数导数 (角速度 -> 四元数变化率)
        gx, gy, gz = gyro
        qDot = 0.5 * np.array([
            -q1*gx - q2*gy - q3*gz,
             q0*gx + q2*gz - q3*gy,
             q0*gy - q1*gz + q3*gx,
             q0*gz + q1*gy - q2*gx
        ])
        
        # 梯度下降校正（如果加速度有效）
        if accel_norm > 0:
            # 目标函数：预测的重力方向与测量的加速度之差
            # f = [2(q1q3 - q0q2) - ax,
            #      2(q0q1 + q2q3) - ay,
            #      2(0.5 - q1² - q2²) - az]
            f = np.array([
                2*(q1*q3 - q0*q2) - a[0],
                2*(q0*q1 + q2*q3) - a[1],
                2*(0.5 - q1*q1 - q2*q2) - a[2]
            ])
            
            # 雅可比矩阵
            J = np.array([
                [-2*q2,  2*q3, -2*q0,  2*q1],
                [ 2*q1,  2*q0,  2*q3,  2*q2],
                [    0, -4*q1, -4*q2,     0]
            ])
            
            # 梯度
            step = J.T @ f
            step_norm = np.linalg.norm(step)
            if step_norm > 0:
                step = step / step_norm
            
            # 应用校正
            qDot = qDot - self.beta * step
        
        # 积分更新四元数
        q = q + qDot * self.dt
        
        # 归一化
        q = q / np.linalg.norm(q)
        self.q = q
        
        # 转换为欧拉角
        return self.quaternion_to_euler(q)
    
    def quaternion_to_euler(self, q: np.ndarray) -> Tuple[float, float, float]:
        """
        四元数转欧拉角
        
        Args:
            q: 四元数 [w, x, y, z]
        
        Returns:
            (roll, pitch, yaw) 欧拉角 (rad)
        """
        w, x, y, z = q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def set_reference(self):
        """设置当前姿态为参考姿态"""
        self.q_ref = self.q.copy()
        self.use_relative = True
    
    def get_relative_euler(self) -> Tuple[float, float, float]:
        """
        获取相对于参考姿态的欧拉角变化
        
        Returns:
            (delta_roll, delta_pitch, delta_yaw)
        """
        if self.q_ref is None:
            return self.quaternion_to_euler(self.q)
        
        # 计算相对四元数: q_rel = q_ref^(-1) * q
        q_ref_inv = np.array([self.q_ref[0], -self.q_ref[1], -self.q_ref[2], -self.q_ref[3]])
        q_rel = self._quaternion_multiply(q_ref_inv, self.q)
        
        return self.quaternion_to_euler(q_rel)
    
    def _quaternion_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """四元数乘法"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
    
    def reset(self):
        """重置滤波器"""
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.q_ref = None
        self.use_relative = False


class IMUProcessor:
    """
    完整的 IMU 数据处理器
    
    集成标定、滤波和姿态估计
    """
    
    def __init__(
        self,
        sample_rate: float = 60.0,
        calibration_samples: int = 200,
        filter_type: str = "madgwick",
        madgwick_beta: float = 0.1,
        lowpass_alpha_gyro: float = 0.8,
        lowpass_alpha_accel: float = 0.5
    ):
        """
        初始化 IMU 处理器
        
        Args:
            sample_rate: 采样频率
            calibration_samples: 标定采样数
            filter_type: 滤波器类型 ("madgwick" / "complementary")
            madgwick_beta: Madgwick 算法增益
            lowpass_alpha_gyro: 陀螺仪低通滤波系数
            lowpass_alpha_accel: 加速度计低通滤波系数
        """
        self.sample_rate = sample_rate
        
        # 标定器
        self.calibrator = IMUCalibrator(calibration_samples=calibration_samples)
        
        # 低通滤波器
        self.gyro_filter = LowPassFilter(alpha=lowpass_alpha_gyro)
        self.accel_filter = LowPassFilter(alpha=lowpass_alpha_accel)
        
        # 姿态估计器
        self.filter_type = filter_type
        if filter_type == "madgwick":
            self.attitude_filter = MadgwickFilter(sample_rate=sample_rate, beta=madgwick_beta)
        else:
            self.attitude_filter = ComplementaryFilter(sample_rate=sample_rate)
        
        # 输出状态
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # 初始姿态（用于计算增量）
        self.initial_roll = 0.0
        self.initial_pitch = 0.0
        self.initial_yaw = 0.0
        self.initial_set = False
    
    def start_calibration(self):
        """开始标定"""
        self.calibrator.start_calibration()
    
    def process(self, accel: np.ndarray, gyro: np.ndarray) -> Tuple[float, float, float]:
        """
        处理 IMU 数据
        
        Args:
            accel: 加速度计数据 [ax, ay, az] (m/s²)
            gyro: 陀螺仪数据 [gx, gy, gz] (rad/s)
        
        Returns:
            (roll, pitch, yaw) 欧拉角 (rad)
        """
        # 如果正在标定，添加样本
        if self.calibrator.state == CalibrationState.IN_PROGRESS:
            self.calibrator.add_sample(accel, gyro)
            return 0.0, 0.0, 0.0
        
        # 应用标定补偿
        accel_cal, gyro_cal = self.calibrator.apply(accel, gyro)
        
        # 低通滤波
        accel_filtered = self.accel_filter.update(accel_cal)
        gyro_filtered = self.gyro_filter.update(gyro_cal)
        
        # 姿态估计
        self.roll, self.pitch, self.yaw = self.attitude_filter.update(
            accel_filtered, gyro_filtered
        )
        
        return self.roll, self.pitch, self.yaw
    
    def set_zero_reference(self):
        """设置当前姿态为零点参考"""
        self.initial_roll = self.roll
        self.initial_pitch = self.pitch
        self.initial_yaw = self.yaw
        self.initial_set = True
        
        # 如果使用 Madgwick，也设置参考四元数
        if isinstance(self.attitude_filter, MadgwickFilter):
            self.attitude_filter.set_reference()
    
    def get_relative_attitude(self) -> Tuple[float, float, float]:
        """
        获取相对于零点参考的姿态变化
        
        Returns:
            (delta_roll, delta_pitch, delta_yaw)
        """
        if isinstance(self.attitude_filter, MadgwickFilter) and self.attitude_filter.use_relative:
            return self.attitude_filter.get_relative_euler()
        
        if not self.initial_set:
            return self.roll, self.pitch, self.yaw
        
        return (
            self.roll - self.initial_roll,
            self.pitch - self.initial_pitch,
            self.yaw - self.initial_yaw
        )
    
    def reset(self):
        """重置处理器"""
        self.calibrator.reset()
        self.gyro_filter.reset()
        self.accel_filter.reset()
        self.attitude_filter.reset()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.initial_set = False
    
    @property
    def is_calibrated(self) -> bool:
        """是否已标定"""
        return self.calibrator.is_calibrated
    
    @property
    def calibration_progress(self) -> float:
        """标定进度"""
        return self.calibrator.progress
    
    @property
    def calibration_state(self) -> CalibrationState:
        """标定状态"""
        return self.calibrator.state
