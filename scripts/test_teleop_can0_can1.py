#!/usr/bin/env python3
"""
EL-A3 遥操作测试程序

通过 can0 控制主臂（零力矩模式），can1 跟随主臂位置

使用方法：
    python3 test_teleop_can0_can1.py
"""

import socket
import struct
import time
import threading
import signal
import sys
import math
import yaml
import os
from dataclasses import dataclass
from typing import Dict, Optional
from enum import IntEnum


class MotorType(IntEnum):
    RS00 = 0  # 1-3号关节
    EL05 = 1  # 4-6号关节


@dataclass
class MotorParams:
    p_min: float = -12.57
    p_max: float = 12.57
    v_min: float = -50.0
    v_max: float = 50.0
    t_min: float = -12.0
    t_max: float = 12.0
    kp_min: float = 0.0
    kp_max: float = 500.0
    kd_min: float = 0.0
    kd_max: float = 5.0


MOTOR_PARAMS = {
    MotorType.RS00: MotorParams(v_min=-33.0, v_max=33.0, t_min=-14.0, t_max=14.0),
    MotorType.EL05: MotorParams(v_min=-50.0, v_max=50.0, t_min=-6.0, t_max=6.0),
}

MOTOR_TYPE_MAP = {1: MotorType.RS00, 2: MotorType.RS00, 3: MotorType.RS00,
                  4: MotorType.EL05, 5: MotorType.EL05, 6: MotorType.EL05,
                  7: MotorType.EL05}  # 7号电机：夹爪

# 关节方向（与 el_a3_ros2_control.xacro 配置一致）
# 用于将关节坐标系转换为电机坐标系
JOINT_DIRECTIONS = {
    1: -1.0,  # L1
    2: 1.0,   # L2
    3: -1.0,  # L3
    4: 1.0,   # L4
    5: -1.0,  # L5
    6: 1.0,   # L6
    7: 1.0,   # L7 夹爪
}


@dataclass
class MotorFeedback:
    motor_id: int = 0
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    temperature: float = 0.0
    is_valid: bool = False


class CANInterface:
    """简化的 CAN 接口"""
    
    def __init__(self, interface: str, host_can_id: int = 0xFD):
        self.interface = interface
        self.host_can_id = host_can_id
        self.socket = None
        self.feedbacks: Dict[int, MotorFeedback] = {}
        self.running = False
        self.recv_thread = None
        self._lock = threading.Lock()
    
    def connect(self) -> bool:
        try:
            self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.socket.bind((self.interface,))
            self.socket.settimeout(0.1)
            print(f"[{self.interface}] 已连接")
            return True
        except Exception as e:
            print(f"[{self.interface}] 连接失败: {e}")
            return False
    
    def disconnect(self):
        self.stop_recv()
        if self.socket:
            self.socket.close()
            self.socket = None
    
    def start_recv(self):
        if self.running:
            return
        self.running = True
        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.recv_thread.start()
    
    def stop_recv(self):
        self.running = False
        if self.recv_thread:
            self.recv_thread.join(timeout=1.0)
    
    def _recv_loop(self):
        while self.running:
            try:
                frame = self.socket.recv(16)
                if len(frame) == 16:
                    self._parse(frame)
            except socket.timeout:
                continue
            except:
                pass
    
    def _parse(self, frame: bytes):
        can_id, _ = struct.unpack('=IB3x', frame[:8])
        data = frame[8:16]
        
        if not (can_id & 0x80000000):
            return
        
        can_id &= 0x1FFFFFFF
        comm_type = (can_id >> 24) & 0x1F
        motor_id = (can_id >> 8) & 0xFF
        
        if comm_type == 2 and 1 <= motor_id <= 7:  # 包含7号夹爪电机
            motor_type = MOTOR_TYPE_MAP.get(motor_id, MotorType.RS00)
            params = MOTOR_PARAMS[motor_type]
            
            pos_raw, vel_raw, torque_raw, temp_raw = struct.unpack('>HHHH', data)
            
            fb = MotorFeedback()
            fb.motor_id = motor_id
            fb.position = pos_raw * (params.p_max - params.p_min) / 65535.0 + params.p_min
            fb.velocity = vel_raw * (params.v_max - params.v_min) / 65535.0 + params.v_min
            fb.torque = torque_raw * (params.t_max - params.t_min) / 65535.0 + params.t_min
            fb.temperature = temp_raw / 10.0
            fb.is_valid = True
            
            with self._lock:
                self.feedbacks[motor_id] = fb
    
    def get_feedback(self, motor_id: int) -> Optional[MotorFeedback]:
        with self._lock:
            return self.feedbacks.get(motor_id)
    
    def _send(self, can_id: int, data: bytes) -> bool:
        if not self.socket:
            return False
        try:
            frame = struct.pack('=IB3x8s', can_id, 8, data.ljust(8, b'\x00'))
            self.socket.send(frame)
            return True
        except:
            return False
    
    def _make_can_id(self, comm_type: int, data2: int, target_id: int) -> int:
        return ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (target_id & 0xFF) | 0x80000000
    
    @staticmethod
    def _to_uint16(x: float, x_min: float, x_max: float) -> int:
        x = max(x_min, min(x_max, x))
        return int((x - x_min) * 65535.0 / (x_max - x_min))
    
    def enable_motor(self, motor_id: int) -> bool:
        can_id = self._make_can_id(3, self.host_can_id, motor_id)
        return self._send(can_id, bytes(8))
    
    def disable_motor(self, motor_id: int, clear_fault: bool = False) -> bool:
        can_id = self._make_can_id(4, self.host_can_id, motor_id)
        return self._send(can_id, bytes([1 if clear_fault else 0]) + bytes(7))
    
    def set_run_mode(self, motor_id: int, mode: int = 0) -> bool:
        can_id = self._make_can_id(18, self.host_can_id, motor_id)
        data = struct.pack('<HBx', 0x7005, 0) + struct.pack('<f', float(mode))
        return self._send(can_id, data)
    
    def send_motion(self, motor_id: int, pos: float, vel: float, kp: float, kd: float, torque: float = 0.0):
        motor_type = MOTOR_TYPE_MAP.get(motor_id, MotorType.RS00)
        params = MOTOR_PARAMS[motor_type]
        
        pos_raw = self._to_uint16(pos, params.p_min, params.p_max)
        vel_raw = self._to_uint16(vel, -50.0, 50.0)
        kp_raw = self._to_uint16(kp, params.kp_min, params.kp_max)
        kd_raw = self._to_uint16(kd, params.kd_min, params.kd_max)
        torque_raw = self._to_uint16(torque, params.t_min, params.t_max)
        
        can_id = self._make_can_id(1, torque_raw, motor_id)
        data = struct.pack('>HHHH', pos_raw, vel_raw, kp_raw, kd_raw)
        self._send(can_id, data)


class TeleopTest:
    """遥操作测试：can0->can2, can1->can3"""
    
    def __init__(self):
        # 两对主从臂
        self.master1 = CANInterface('can0')  # 主臂1
        self.slave1 = CANInterface('can2')   # 从臂1
        self.master2 = CANInterface('can1')  # 主臂2
        self.slave2 = CANInterface('can3')   # 从臂2
        
        self.motor_ids = [1, 2, 3, 4, 5, 6, 7]  # 包含夹爪电机
        self.running = False
        
        # 控制参数
        self.master_kd = 0.0      # 主臂阻尼（完全无力矩）
        self.slave_kp = 50.0      # 从臂 Kp
        self.slave_kd = 1.0       # 从臂 Kd
        self.rate = 200.0         # 控制频率 Hz
        
        # 加载标定参数文件进行多关节综合重力补偿
        self.inertia_params = self._load_inertia_params()
        
        # 连杆长度（从 URDF 获取，单位：米）
        self.link_lengths = {
            2: 0.19,   # L2 连杆长度
            3: 0.185,  # L3 连杆长度
            4: 0.05,   # L4 连杆长度
            5: 0.038,  # L5 连杆长度
            6: 0.074,  # L6 连杆长度（到末端）
        }
        
        # 重力加速度
        self.g = 9.81
        
        self.loop_count = 0
        self.last_print = time.time()
    
    def _load_inertia_params(self):
        """加载标定后的惯性参数"""
        config_path = os.path.join(
            os.path.dirname(__file__), 
            '../el_a3_description/config/inertia_params.yaml'
        )
        
        params = {}
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            if config.get('use_calibrated_params', False):
                inertia = config.get('inertia_params', {})
                for link_name, data in inertia.items():
                    # 提取连杆编号 (L2 -> 2)
                    link_id = int(link_name[1])
                    params[link_id] = {
                        'mass': data.get('mass', 0.0),
                        'com': data.get('com', [0.0, 0.0, 0.0])
                    }
                print(f"[INFO] 已加载标定惯性参数: {list(params.keys())}")
            else:
                print("[WARN] 标定参数未启用，使用默认值")
        except Exception as e:
            print(f"[WARN] 加载惯性参数失败: {e}，使用默认值")
        
        # 默认参数（如果加载失败）
        if not params:
            params = {
                2: {'mass': 0.83, 'com': [0.125, 0.0, 0.0]},
                3: {'mass': 0.23, 'com': [-0.148, 0.0, 0.0]},
                4: {'mass': 0.36, 'com': [-0.15, 0.0, 0.0]},
                5: {'mass': 0.08, 'com': [0.0, 0.0, 0.08]},
                6: {'mass': 0.10, 'com': [0.0, 0.0, 0.0]},
            }
        
        return params
    
    def compute_gravity_compensation(self, positions: dict) -> dict:
        """
        计算多关节级联重力补偿
        
        基于标定的惯性参数，考虑各连杆对关节力矩的贡献
        
        Args:
            positions: {motor_id: position} 字典，位置为弧度
            
        Returns:
            {motor_id: torque} 字典，重力补偿力矩
        """
        torques = {i: 0.0 for i in range(1, 8)}
        
        # 获取各关节位置
        q2 = positions.get(2, 0.0)  # L2 俯仰角
        q3 = positions.get(3, 0.0)  # L3 俯仰角
        q5 = positions.get(5, 0.0)  # L5 俯仰角
        
        # L1: 绕Z轴旋转，无重力力矩
        torques[1] = 0.0
        
        # 计算后续连杆总质量
        m2 = self.inertia_params.get(2, {}).get('mass', 0.0)
        m3 = self.inertia_params.get(3, {}).get('mass', 0.0)
        m4 = self.inertia_params.get(4, {}).get('mass', 0.0)
        m5 = self.inertia_params.get(5, {}).get('mass', 0.0)
        m6 = self.inertia_params.get(6, {}).get('mass', 0.0)
        
        # 质心位置
        com2_x = self.inertia_params.get(2, {}).get('com', [0.125, 0, 0])[0]
        com3_x = abs(self.inertia_params.get(3, {}).get('com', [-0.148, 0, 0])[0])
        com4_x = abs(self.inertia_params.get(4, {}).get('com', [-0.15, 0, 0])[0])
        
        L2 = self.link_lengths.get(2, 0.19)
        L3 = self.link_lengths.get(3, 0.185)
        
        # L2 关节力矩: 承受 L2-L6 所有连杆重力
        # τ2 = g × [m2×com2×sin(q2) + (m3+m4+m5+m6)×L2×sin(q2) + m3×com3×sin(q2+q3) + ...]
        tau2 = self.g * (
            m2 * com2_x * math.sin(q2) +
            (m3 + m4 + m5 + m6) * L2 * math.sin(q2) +
            m3 * com3_x * math.sin(q2 + q3) +
            (m4 + m5 + m6) * L3 * math.sin(q2 + q3)
        )
        torques[2] = tau2
        
        # L3 关节力矩: 承受 L3-L6 连杆重力
        # τ3 = g × [m3×com3×sin(q2+q3) + (m4+m5+m6)×L3×sin(q2+q3)]
        tau3 = self.g * (
            m3 * com3_x * math.sin(q2 + q3) +
            (m4 + m5 + m6) * L3 * math.sin(q2 + q3)
        )
        torques[3] = tau3
        
        # L4: 腕部Roll，主要是水平旋转，重力影响小
        # 简化：与 L5 的俯仰角相关
        tau4 = self.g * (m5 + m6) * com4_x * math.sin(q5)
        torques[4] = tau4
        
        # L5: 腕部Pitch
        com5_z = abs(self.inertia_params.get(5, {}).get('com', [0, 0, 0.08])[2])
        tau5 = self.g * (m5 * com5_z + m6 * 0.04) * math.sin(q5)
        torques[5] = tau5
        
        # L6: 末端Yaw，绕Z轴旋转，无重力力矩
        torques[6] = 0.0
        
        # L7: 夹爪，无重力补偿
        torques[7] = 0.0
        
        return torques
    
    def connect(self) -> bool:
        # 连接四个CAN接口
        if not self.master1.connect():
            return False
        if not self.slave1.connect():
            self.master1.disconnect()
            return False
        if not self.master2.connect():
            self.master1.disconnect()
            self.slave1.disconnect()
            return False
        if not self.slave2.connect():
            self.master1.disconnect()
            self.slave1.disconnect()
            self.master2.disconnect()
            return False
        return True
    
    def disconnect(self):
        self.master1.disconnect()
        self.slave1.disconnect()
        self.master2.disconnect()
        self.slave2.disconnect()
    
    def enable_motors(self):
        print("[INFO] 使能电机...")
        
        interfaces = [
            ('can0->can2', self.master1, self.slave1),
            ('can1->can3', self.master2, self.slave2),
        ]
        
        # 第一步：清除故障并设置运控模式
        print("[DEBUG] 步骤1: 清除故障并设置运控模式")
        for name, master, slave in interfaces:
            print(f"  [{name}]")
            for motor_id in self.motor_ids:
                master.disable_motor(motor_id, clear_fault=True)
                master.set_run_mode(motor_id, 0)
                slave.disable_motor(motor_id, clear_fault=True)
                slave.set_run_mode(motor_id, 0)
                time.sleep(0.02)
        
        time.sleep(0.1)
        
        # 第二步：使能电机
        print("[DEBUG] 步骤2: 使能电机")
        for name, master, slave in interfaces:
            print(f"  [{name}]")
            for motor_id in self.motor_ids:
                ok_m = master.enable_motor(motor_id)
                ok_s = slave.enable_motor(motor_id)
                print(f"    M{motor_id}: master={ok_m} slave={ok_s}")
                time.sleep(0.02)
        
        # 等待反馈更新
        time.sleep(0.3)
        
        # 第三步：检查从臂反馈
        print("[DEBUG] 步骤3: 检查从臂反馈")
        for name, master, slave in interfaces:
            print(f"  [{name}]")
            for motor_id in self.motor_ids:
                fb_s = slave.get_feedback(motor_id)
                if fb_s and fb_s.is_valid:
                    print(f"    M{motor_id}: pos={fb_s.position:.3f} (有效)")
                else:
                    print(f"    M{motor_id}: 无反馈!")
        
        # 第四步：读取主臂当前位置，发送初始命令
        print("[DEBUG] 步骤4: 发送初始命令")
        for name, master, slave in interfaces:
            print(f"  [{name}]")
            for motor_id in self.motor_ids:
                fb = master.get_feedback(motor_id)
                pos = fb.position if fb and fb.is_valid else 0.0
                
                # 主臂：完全无力矩（Kp=0, Kd=0, torque=0）
                master.send_motion(motor_id, pos, 0.0, 0.0, 0.0, 0.0)
                
                # 从臂：跟随主臂位置 (Kp=50, Kd=1)
                slave.send_motion(motor_id, pos, 0.0, self.slave_kp, self.slave_kd, 0.0)
                print(f"    M{motor_id}: pos={pos:.3f} -> slave Kp={self.slave_kp}")
                time.sleep(0.01)
        
        print("[INFO] 电机已使能")
    
    def disable_motors(self):
        print("[INFO] 停止电机...")
        for motor_id in self.motor_ids:
            self.master1.disable_motor(motor_id)
            self.slave1.disable_motor(motor_id)
            self.master2.disable_motor(motor_id)
            self.slave2.disable_motor(motor_id)
            time.sleep(0.01)
    
    def run(self):
        self.running = True
        dt = 1.0 / self.rate
        
        print(f"\n[INFO] 遥操作启动")
        print(f"  can0 -> can2 (主臂1 -> 从臂1)")
        print(f"  can1 -> can3 (主臂2 -> 从臂2)")
        print(f"  从臂: Kp={self.slave_kp}, Kd={self.slave_kd}")
        print(f"  控制频率: {self.rate} Hz")
        print("[INFO] 按 Ctrl+C 停止\n")
        
        pairs = [
            (self.master1, self.slave1),
            (self.master2, self.slave2),
        ]
        
        while self.running:
            t0 = time.time()
            
            # 处理两对主从臂
            for master, slave in pairs:
                # 读取主臂位置并发送命令
                for motor_id in self.motor_ids:
                    fb = master.get_feedback(motor_id)
                    pos = fb.position if fb and fb.is_valid else 0.0
                    
                    # 主臂：完全无力矩
                    master.send_motion(motor_id, pos, 0.0, 0.0, 0.0, 0.0)
                    
                    # 从臂：跟随主臂位置
                    slave.send_motion(motor_id, pos, 0.0, self.slave_kp, self.slave_kd, 0.0)
            
            # 打印状态（每秒一次）
            self.loop_count += 1
            now = time.time()
            if now - self.last_print >= 1.0:
                actual_freq = self.loop_count / (now - self.last_print)
                self.last_print = now
                self.loop_count = 0
                
                # 显示两对主从臂的反馈
                for i, (master, slave) in enumerate(pairs):
                    m_info = []
                    s_info = []
                    for mid in self.motor_ids:
                        fb_m = master.get_feedback(mid)
                        fb_s = slave.get_feedback(mid)
                        m_info.append(f"{fb_m.position:.2f}" if fb_m and fb_m.is_valid else "N/A")
                        s_info.append(f"{fb_s.position:.2f}" if fb_s and fb_s.is_valid else "N/A")
                    
                    pair_name = "can0->can2" if i == 0 else "can1->can3"
                    print(f"[{actual_freq:.0f}Hz] {pair_name} 主:{' '.join(m_info)} 从:{' '.join(s_info)}")
            
            # 4. 控制频率
            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
    
    def stop(self):
        self.running = False


def main():
    print("=" * 50)
    print("  EL-A3 遥操作测试")
    print("  can0 -> can2, can1 -> can3")
    print("=" * 50)
    
    teleop = TeleopTest()
    
    def signal_handler(sig, frame):
        print("\n[INFO] 停止中...")
        teleop.stop()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        if not teleop.connect():
            print("[ERROR] 连接失败")
            return 1
        
        # 启动四个CAN接口的接收线程
        teleop.master1.start_recv()
        teleop.slave1.start_recv()
        teleop.master2.start_recv()
        teleop.slave2.start_recv()
        
        time.sleep(0.5)  # 等待反馈
        
        teleop.enable_motors()
        time.sleep(0.5)
        
        teleop.run()
        
    except Exception as e:
        print(f"[ERROR] {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        teleop.disable_motors()
        teleop.master1.stop_recv()
        teleop.slave1.stop_recv()
        teleop.master2.stop_recv()
        teleop.slave2.stop_recv()
        teleop.disconnect()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
