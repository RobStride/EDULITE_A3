#!/usr/bin/env python3
"""
Joy-Con IMU 姿态可视化测试程序

在 3D 界面中实时展示 Joy-Con 手柄的姿态变化

功能：
- 实时显示 Joy-Con 的 Roll, Pitch, Yaw 姿态
- 3D 可视化手柄方向
- 显示加速度和角速度原始数据
- 支持标定和零点重置

使用方法：
    python3 joycon_visualizer.py

按键：
    C - 开始/重新标定
    R - 重置零点
    Q - 退出
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import threading
import time
import sys
import os

# 添加路径以导入 IMU 处理模块
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 
                                '../ros2_ws/src/el_a3_teleop/el_a3_teleop'))

try:
    from imu_processor import IMUProcessor, CalibrationState, MadgwickFilter
except ImportError:
    print("无法导入 imu_processor 模块，使用内置实现")
    
    class MadgwickFilter:
        """简化的 Madgwick 滤波器"""
        def __init__(self, sample_rate=60.0, beta=0.1):
            self.dt = 1.0 / sample_rate
            self.beta = beta
            self.q = np.array([1.0, 0.0, 0.0, 0.0])
        
        def update(self, accel, gyro):
            q = self.q.copy()
            accel_norm = np.linalg.norm(accel)
            if accel_norm > 0:
                a = accel / accel_norm
            else:
                a = accel
            
            q0, q1, q2, q3 = q
            gx, gy, gz = gyro
            
            qDot = 0.5 * np.array([
                -q1*gx - q2*gy - q3*gz,
                 q0*gx + q2*gz - q3*gy,
                 q0*gy - q1*gz + q3*gx,
                 q0*gz + q1*gy - q2*gx
            ])
            
            if accel_norm > 0:
                f = np.array([
                    2*(q1*q3 - q0*q2) - a[0],
                    2*(q0*q1 + q2*q3) - a[1],
                    2*(0.5 - q1*q1 - q2*q2) - a[2]
                ])
                J = np.array([
                    [-2*q2,  2*q3, -2*q0,  2*q1],
                    [ 2*q1,  2*q0,  2*q3,  2*q2],
                    [    0, -4*q1, -4*q2,     0]
                ])
                step = J.T @ f
                step_norm = np.linalg.norm(step)
                if step_norm > 0:
                    step = step / step_norm
                qDot = qDot - self.beta * step
            
            q = q + qDot * self.dt
            q = q / np.linalg.norm(q)
            self.q = q
            
            return self.quaternion_to_euler(q)
        
        def quaternion_to_euler(self, q):
            w, x, y, z = q
            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = np.arctan2(sinr_cosp, cosr_cosp)
            
            sinp = 2 * (w * y - z * x)
            if abs(sinp) >= 1:
                pitch = np.sign(sinp) * np.pi / 2
            else:
                pitch = np.arcsin(sinp)
            
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)
            
            return roll, pitch, yaw
        
        def reset(self):
            self.q = np.array([1.0, 0.0, 0.0, 0.0])

# 尝试导入 evdev
try:
    import evdev
    from evdev import ecodes
    EVDEV_AVAILABLE = True
except ImportError:
    EVDEV_AVAILABLE = False
    print("警告: evdev 未安装，将使用模拟数据")
    print("安装: pip3 install evdev")


class JoyConIMUReader:
    """Joy-Con IMU 数据读取器"""
    
    def __init__(self):
        self.accel = np.zeros(3)
        self.gyro = np.zeros(3)
        self.accel_scale = 9.81 / 4096.0
        self.gyro_scale = 0.0174533 / 131.0
        self.running = False
        self.device = None
        self.use_mock = not EVDEV_AVAILABLE
        self.lock = threading.Lock()
        
        if EVDEV_AVAILABLE:
            self._find_device()
    
    def _find_device(self):
        """查找 Joy-Con 设备"""
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        
        for device in devices:
            name_lower = device.name.lower()
            if 'joy-con' in name_lower or 'joycon' in name_lower or 'nintendo' in name_lower:
                caps = device.capabilities()
                if ecodes.EV_ABS in caps:
                    print(f"找到 Joy-Con: {device.name} ({device.path})")
                    self.device = device
                    return
        
        print("未找到 Joy-Con 设备，使用模拟数据")
        self.use_mock = True
    
    def start(self):
        """启动读取线程"""
        self.running = True
        if not self.use_mock and self.device:
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
        else:
            self.thread = threading.Thread(target=self._mock_loop, daemon=True)
            self.thread.start()
    
    def stop(self):
        """停止读取"""
        self.running = False
    
    def _read_loop(self):
        """读取 evdev 事件"""
        try:
            for event in self.device.read_loop():
                if not self.running:
                    break
                if event.type == ecodes.EV_ABS:
                    with self.lock:
                        if event.code == ecodes.ABS_X:
                            self.accel[0] = event.value * self.accel_scale
                        elif event.code == ecodes.ABS_Y:
                            self.accel[1] = event.value * self.accel_scale
                        elif event.code == ecodes.ABS_Z:
                            self.accel[2] = event.value * self.accel_scale
                        elif event.code == ecodes.ABS_RX:
                            self.gyro[0] = event.value * self.gyro_scale
                        elif event.code == ecodes.ABS_RY:
                            self.gyro[1] = event.value * self.gyro_scale
                        elif event.code == ecodes.ABS_RZ:
                            self.gyro[2] = event.value * self.gyro_scale
        except Exception as e:
            print(f"读取错误: {e}")
    
    def _mock_loop(self):
        """模拟数据（用于测试）"""
        t = 0
        while self.running:
            with self.lock:
                # 模拟缓慢变化的姿态
                self.accel[0] = 0.5 * np.sin(t * 0.5)
                self.accel[1] = 0.5 * np.cos(t * 0.3)
                self.accel[2] = 9.81 + 0.2 * np.sin(t * 0.2)
                
                self.gyro[0] = 0.1 * np.sin(t * 0.8)
                self.gyro[1] = 0.1 * np.cos(t * 0.6)
                self.gyro[2] = 0.05 * np.sin(t * 0.4)
            
            t += 1.0 / 60.0
            time.sleep(1.0 / 60.0)
    
    def get_data(self):
        """获取当前 IMU 数据"""
        with self.lock:
            return self.accel.copy(), self.gyro.copy()


class JoyConVisualizer:
    """Joy-Con 3D 可视化器"""
    
    def __init__(self):
        # IMU 读取器
        self.imu_reader = JoyConIMUReader()
        
        # Madgwick 滤波器
        self.filter = MadgwickFilter(sample_rate=60.0, beta=0.1)
        
        # 姿态数据
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # 参考姿态（用于相对显示）
        self.ref_roll = 0.0
        self.ref_pitch = 0.0
        self.ref_yaw = 0.0
        self.use_reference = False
        
        # 标定状态
        self.calibrating = False
        self.calibration_samples = []
        self.gyro_bias = np.zeros(3)
        self.is_calibrated = False
        
        # 历史数据（用于绘图）
        self.history_len = 200
        self.roll_history = np.zeros(self.history_len)
        self.pitch_history = np.zeros(self.history_len)
        self.yaw_history = np.zeros(self.history_len)
        self.time_history = np.linspace(-self.history_len/60, 0, self.history_len)
        
        # 创建图形
        self._setup_figure()
        
        # 启动 IMU 读取
        self.imu_reader.start()
    
    def _setup_figure(self):
        """设置图形界面"""
        self.fig = plt.figure(figsize=(14, 8))
        self.fig.suptitle('Joy-Con IMU 姿态可视化', fontsize=14, fontweight='bold')
        
        # 3D 视图
        self.ax_3d = self.fig.add_subplot(121, projection='3d')
        self.ax_3d.set_xlim([-1.5, 1.5])
        self.ax_3d.set_ylim([-1.5, 1.5])
        self.ax_3d.set_zlim([-1.5, 1.5])
        self.ax_3d.set_xlabel('X')
        self.ax_3d.set_ylabel('Y')
        self.ax_3d.set_zlabel('Z')
        self.ax_3d.set_title('3D 姿态')
        
        # 欧拉角历史曲线
        self.ax_euler = self.fig.add_subplot(322)
        self.ax_euler.set_xlim([-self.history_len/60, 0])
        self.ax_euler.set_ylim([-180, 180])
        self.ax_euler.set_xlabel('时间 (s)')
        self.ax_euler.set_ylabel('角度 (°)')
        self.ax_euler.set_title('欧拉角历史')
        self.ax_euler.grid(True, alpha=0.3)
        
        self.line_roll, = self.ax_euler.plot([], [], 'r-', label='Roll', linewidth=1.5)
        self.line_pitch, = self.ax_euler.plot([], [], 'g-', label='Pitch', linewidth=1.5)
        self.line_yaw, = self.ax_euler.plot([], [], 'b-', label='Yaw', linewidth=1.5)
        self.ax_euler.legend(loc='upper right')
        
        # 状态显示
        self.ax_status = self.fig.add_subplot(324)
        self.ax_status.axis('off')
        self.status_text = self.ax_status.text(0.1, 0.9, '', transform=self.ax_status.transAxes,
                                                fontsize=11, verticalalignment='top',
                                                fontfamily='monospace')
        
        # 帮助信息
        self.ax_help = self.fig.add_subplot(326)
        self.ax_help.axis('off')
        help_text = """按键说明:
        
C - 开始/重新标定 (保持静止)
R - 重置零点参考
Q - 退出

移动 Joy-Con 查看姿态变化
"""
        self.ax_help.text(0.1, 0.9, help_text, transform=self.ax_help.transAxes,
                          fontsize=10, verticalalignment='top',
                          fontfamily='monospace',
                          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # 按键事件
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        
        plt.tight_layout()
    
    def _on_key(self, event):
        """按键事件处理"""
        if event.key == 'c':
            self.start_calibration()
        elif event.key == 'r':
            self.reset_reference()
        elif event.key == 'q':
            self.imu_reader.stop()
            plt.close()
    
    def start_calibration(self):
        """开始标定"""
        print("开始标定，请保持 Joy-Con 静止...")
        self.calibrating = True
        self.calibration_samples = []
        self.is_calibrated = False
    
    def reset_reference(self):
        """重置零点参考"""
        self.ref_roll = self.roll
        self.ref_pitch = self.pitch
        self.ref_yaw = self.yaw
        self.use_reference = True
        print(f"零点已重置: Roll={np.degrees(self.ref_roll):.1f}°, "
              f"Pitch={np.degrees(self.ref_pitch):.1f}°, "
              f"Yaw={np.degrees(self.ref_yaw):.1f}°")
    
    def _draw_joycon(self, roll, pitch, yaw):
        """绘制 Joy-Con 3D 模型"""
        self.ax_3d.clear()
        self.ax_3d.set_xlim([-1.5, 1.5])
        self.ax_3d.set_ylim([-1.5, 1.5])
        self.ax_3d.set_zlim([-1.5, 1.5])
        self.ax_3d.set_xlabel('X')
        self.ax_3d.set_ylabel('Y')
        self.ax_3d.set_zlabel('Z')
        
        # Joy-Con 尺寸（简化为长方体）
        # 长 x 宽 x 高
        L, W, H = 1.0, 0.3, 0.15
        
        # 定义顶点（本地坐标）
        vertices = np.array([
            [-L/2, -W/2, -H/2],
            [ L/2, -W/2, -H/2],
            [ L/2,  W/2, -H/2],
            [-L/2,  W/2, -H/2],
            [-L/2, -W/2,  H/2],
            [ L/2, -W/2,  H/2],
            [ L/2,  W/2,  H/2],
            [-L/2,  W/2,  H/2],
        ])
        
        # 旋转矩阵
        from scipy.spatial.transform import Rotation as R
        rot = R.from_euler('xyz', [roll, pitch, yaw])
        rotated_vertices = rot.apply(vertices)
        
        # 定义面
        faces = [
            [rotated_vertices[0], rotated_vertices[1], rotated_vertices[5], rotated_vertices[4]],  # 底
            [rotated_vertices[2], rotated_vertices[3], rotated_vertices[7], rotated_vertices[6]],  # 顶
            [rotated_vertices[0], rotated_vertices[3], rotated_vertices[7], rotated_vertices[4]],  # 左
            [rotated_vertices[1], rotated_vertices[2], rotated_vertices[6], rotated_vertices[5]],  # 右
            [rotated_vertices[0], rotated_vertices[1], rotated_vertices[2], rotated_vertices[3]],  # 后
            [rotated_vertices[4], rotated_vertices[5], rotated_vertices[6], rotated_vertices[7]],  # 前
        ]
        
        # 颜色
        colors = ['#FF4444', '#44FF44', '#4444FF', '#FFFF44', '#FF44FF', '#44FFFF']
        
        for i, face in enumerate(faces):
            poly = Poly3DCollection([face], alpha=0.8)
            poly.set_facecolor(colors[i])
            poly.set_edgecolor('black')
            self.ax_3d.add_collection3d(poly)
        
        # 绘制坐标轴
        axis_len = 1.2
        origin = np.array([0, 0, 0])
        
        # X 轴（红色）
        x_axis = rot.apply([axis_len, 0, 0])
        self.ax_3d.quiver(*origin, *x_axis, color='red', arrow_length_ratio=0.1, linewidth=2)
        
        # Y 轴（绿色）
        y_axis = rot.apply([0, axis_len, 0])
        self.ax_3d.quiver(*origin, *y_axis, color='green', arrow_length_ratio=0.1, linewidth=2)
        
        # Z 轴（蓝色）
        z_axis = rot.apply([0, 0, axis_len])
        self.ax_3d.quiver(*origin, *z_axis, color='blue', arrow_length_ratio=0.1, linewidth=2)
        
        self.ax_3d.set_title(f'3D 姿态\n'
                             f'Roll: {np.degrees(roll):.1f}° | '
                             f'Pitch: {np.degrees(pitch):.1f}° | '
                             f'Yaw: {np.degrees(yaw):.1f}°')
    
    def update(self, frame):
        """动画更新函数"""
        # 获取 IMU 数据
        accel, gyro = self.imu_reader.get_data()
        
        # 标定处理
        if self.calibrating:
            self.calibration_samples.append(gyro.copy())
            if len(self.calibration_samples) >= 100:
                # 计算陀螺仪零偏
                self.gyro_bias = np.mean(self.calibration_samples, axis=0)
                self.is_calibrated = True
                self.calibrating = False
                self.filter.reset()
                print(f"标定完成! 陀螺仪零偏: {self.gyro_bias}")
        
        # 应用标定
        if self.is_calibrated:
            gyro = gyro - self.gyro_bias
        
        # 更新姿态
        self.roll, self.pitch, self.yaw = self.filter.update(accel, gyro)
        
        # 相对姿态
        if self.use_reference:
            display_roll = self.roll - self.ref_roll
            display_pitch = self.pitch - self.ref_pitch
            display_yaw = self.yaw - self.ref_yaw
        else:
            display_roll = self.roll
            display_pitch = self.pitch
            display_yaw = self.yaw
        
        # 更新历史
        self.roll_history = np.roll(self.roll_history, -1)
        self.pitch_history = np.roll(self.pitch_history, -1)
        self.yaw_history = np.roll(self.yaw_history, -1)
        self.roll_history[-1] = np.degrees(display_roll)
        self.pitch_history[-1] = np.degrees(display_pitch)
        self.yaw_history[-1] = np.degrees(display_yaw)
        
        # 更新 3D 视图
        self._draw_joycon(display_roll, display_pitch, display_yaw)
        
        # 更新欧拉角曲线
        self.line_roll.set_data(self.time_history, self.roll_history)
        self.line_pitch.set_data(self.time_history, self.pitch_history)
        self.line_yaw.set_data(self.time_history, self.yaw_history)
        
        # 更新状态文本
        status = f"""IMU 数据:
加速度: [{accel[0]:7.3f}, {accel[1]:7.3f}, {accel[2]:7.3f}] m/s²
角速度: [{gyro[0]:7.4f}, {gyro[1]:7.4f}, {gyro[2]:7.4f}] rad/s

姿态 (相对):
  Roll:  {np.degrees(display_roll):7.2f}°
  Pitch: {np.degrees(display_pitch):7.2f}°
  Yaw:   {np.degrees(display_yaw):7.2f}°

状态:
  标定: {'✓ 已完成' if self.is_calibrated else ('进行中...' if self.calibrating else '未标定')}
  零点: {'✓ 已设置' if self.use_reference else '未设置'}
  数据源: {'Joy-Con' if not self.imu_reader.use_mock else '模拟数据'}
"""
        self.status_text.set_text(status)
        
        return [self.line_roll, self.line_pitch, self.line_yaw, self.status_text]
    
    def run(self):
        """运行可视化"""
        print("Joy-Con IMU 可视化已启动")
        print("按 C 开始标定 | 按 R 重置零点 | 按 Q 退出")
        
        ani = FuncAnimation(self.fig, self.update, interval=1000/60, blit=False)
        plt.show()
        
        self.imu_reader.stop()


def main():
    print("="*50)
    print("  Joy-Con IMU 姿态可视化测试程序")
    print("="*50)
    print()
    
    visualizer = JoyConVisualizer()
    visualizer.run()


if __name__ == '__main__':
    main()
