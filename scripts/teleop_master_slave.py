#!/usr/bin/env python3
"""
EL-A3 主从遥操作脚本

功能：
- can0: 主臂，运行零力矩模式（可手动拖动）
- can1-4: 从臂，跟随主臂位置（Kp=150, Kd=1）

使用方法：
    python3 teleop_master_slave.py [--slaves can1,can2,can3,can4]
"""

import socket
import struct
import time
import threading
import argparse
import signal
import sys
from dataclasses import dataclass
from typing import List, Dict, Optional
from enum import IntEnum


class MotorType(IntEnum):
    """电机类型"""
    RS00 = 0  # 1-3号关节: 力矩±14Nm, 速度±33rad/s
    EL05 = 1  # 4-6号关节: 力矩±6Nm, 速度±50rad/s


@dataclass
class MotorParams:
    """电机参数范围"""
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


# RS00 和 EL05 电机参数
MOTOR_PARAMS = {
    MotorType.RS00: MotorParams(v_min=-33.0, v_max=33.0, t_min=-14.0, t_max=14.0),
    MotorType.EL05: MotorParams(v_min=-50.0, v_max=50.0, t_min=-6.0, t_max=6.0),
}

# 电机ID到类型的映射（1-3是RS00，4-6是EL05）
MOTOR_TYPE_MAP = {
    1: MotorType.RS00,
    2: MotorType.RS00,
    3: MotorType.RS00,
    4: MotorType.EL05,
    5: MotorType.EL05,
    6: MotorType.EL05,
}

# 关节方向（与 xacro 配置一致）
JOINT_DIRECTIONS = {
    1: -1.0,  # L1
    2: 1.0,   # L2
    3: -1.0,  # L3
    4: 1.0,   # L4
    5: -1.0,  # L5
    6: 1.0,   # L6
}


@dataclass
class MotorFeedback:
    """电机反馈数据"""
    motor_id: int = 0
    position: float = 0.0   # rad
    velocity: float = 0.0   # rad/s
    torque: float = 0.0     # Nm
    temperature: float = 0.0  # °C
    mode_state: int = 0
    fault_code: int = 0
    is_valid: bool = False


class RobstrideCanInterface:
    """Robstride 电机 CAN 通信接口"""
    
    # 通信类型定义
    COMM_TYPE_MOTION_CONTROL = 1   # 运控模式
    COMM_TYPE_FEEDBACK = 2         # 电机反馈
    COMM_TYPE_ENABLE = 3           # 使能电机
    COMM_TYPE_DISABLE = 4          # 停止电机
    COMM_TYPE_SET_ZERO = 6         # 设置零位
    COMM_TYPE_WRITE_PARAM = 18     # 写入参数
    
    # 参数索引
    PARAM_RUN_MODE = 0x7005
    PARAM_LOC_REF = 0x7016
    PARAM_LIMIT_SPD = 0x7017
    
    def __init__(self, interface: str, host_can_id: int = 0xFD):
        """
        初始化 CAN 接口
        
        Args:
            interface: CAN 接口名称（如 "can0"）
            host_can_id: 主机 CAN ID（默认 0xFD）
        """
        self.interface = interface
        self.host_can_id = host_can_id
        self.socket = None
        self.feedbacks: Dict[int, MotorFeedback] = {}
        self.receive_running = False
        self.receive_thread = None
        self._lock = threading.Lock()
        
    def connect(self) -> bool:
        """连接 CAN 接口"""
        try:
            self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.socket.bind((self.interface,))
            self.socket.settimeout(0.1)
            print(f"[{self.interface}] CAN 接口已连接")
            return True
        except Exception as e:
            print(f"[{self.interface}] CAN 连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开 CAN 接口"""
        self.stop_receive_thread()
        if self.socket:
            self.socket.close()
            self.socket = None
            print(f"[{self.interface}] CAN 接口已断开")
    
    def start_receive_thread(self):
        """启动接收线程"""
        if self.receive_running:
            return
        self.receive_running = True
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()
    
    def stop_receive_thread(self):
        """停止接收线程"""
        self.receive_running = False
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)
            self.receive_thread = None
    
    def _receive_loop(self):
        """接收循环"""
        while self.receive_running:
            try:
                frame = self.socket.recv(16)
                if len(frame) == 16:
                    self._parse_frame(frame)
            except socket.timeout:
                continue
            except Exception as e:
                if self.receive_running:
                    print(f"[{self.interface}] 接收错误: {e}")
    
    def _parse_frame(self, frame: bytes):
        """解析 CAN 帧"""
        can_id, dlc = struct.unpack('=IB3x', frame[:8])
        data = frame[8:16]
        
        # 检查扩展帧标志
        if not (can_id & 0x80000000):
            return
        
        can_id &= 0x1FFFFFFF  # 去掉扩展帧标志
        
        # 解析通信类型和电机ID
        comm_type = (can_id >> 24) & 0x1F
        motor_id = (can_id >> 8) & 0xFF
        
        if comm_type == self.COMM_TYPE_FEEDBACK and 1 <= motor_id <= 6:
            self._parse_feedback(can_id, data, motor_id)
    
    def _parse_feedback(self, can_id: int, data: bytes, motor_id: int):
        """解析电机反馈"""
        motor_type = MOTOR_TYPE_MAP.get(motor_id, MotorType.RS00)
        params = MOTOR_PARAMS[motor_type]
        
        # 解析数据（大端序）
        pos_raw, vel_raw, torque_raw, temp_raw = struct.unpack('>HHHH', data)
        
        fb = MotorFeedback()
        fb.motor_id = motor_id
        fb.position = self._uint16_to_float(pos_raw, params.p_min, params.p_max)
        fb.velocity = self._uint16_to_float(vel_raw, params.v_min, params.v_max)
        fb.torque = self._uint16_to_float(torque_raw, params.t_min, params.t_max)
        fb.temperature = temp_raw / 10.0
        fb.mode_state = (can_id >> 22) & 0x03
        fb.fault_code = (can_id >> 16) & 0x3F
        fb.is_valid = True
        
        with self._lock:
            self.feedbacks[motor_id] = fb
    
    def get_feedback(self, motor_id: int) -> Optional[MotorFeedback]:
        """获取电机反馈"""
        with self._lock:
            return self.feedbacks.get(motor_id)
    
    def get_all_feedbacks(self) -> Dict[int, MotorFeedback]:
        """获取所有电机反馈"""
        with self._lock:
            return dict(self.feedbacks)
    
    def _build_extended_can_id(self, comm_type: int, data_area2: int, target_id: int) -> int:
        """构建29位扩展 CAN ID"""
        can_id = ((comm_type & 0x1F) << 24) | ((data_area2 & 0xFFFF) << 8) | (target_id & 0xFF)
        return can_id | 0x80000000  # 设置扩展帧标志
    
    def _send_frame(self, can_id: int, data: bytes) -> bool:
        """发送 CAN 帧"""
        if not self.socket:
            return False
        try:
            frame = struct.pack('=IB3x8s', can_id, 8, data.ljust(8, b'\x00'))
            self.socket.send(frame)
            return True
        except Exception as e:
            print(f"[{self.interface}] 发送失败: {e}")
            return False
    
    @staticmethod
    def _float_to_uint16(x: float, x_min: float, x_max: float) -> int:
        """浮点数转 uint16"""
        x = max(x_min, min(x_max, x))
        return int((x - x_min) * 65535.0 / (x_max - x_min))
    
    @staticmethod
    def _uint16_to_float(x_int: int, x_min: float, x_max: float) -> float:
        """uint16 转浮点数"""
        return x_int * (x_max - x_min) / 65535.0 + x_min
    
    def enable_motor(self, motor_id: int) -> bool:
        """使能电机"""
        can_id = self._build_extended_can_id(self.COMM_TYPE_ENABLE, self.host_can_id, motor_id)
        return self._send_frame(can_id, bytes(8))
    
    def disable_motor(self, motor_id: int, clear_fault: bool = False) -> bool:
        """停止电机"""
        can_id = self._build_extended_can_id(self.COMM_TYPE_DISABLE, self.host_can_id, motor_id)
        data = bytes([1 if clear_fault else 0]) + bytes(7)
        return self._send_frame(can_id, data)
    
    def write_parameter(self, motor_id: int, param_index: int, value: float) -> bool:
        """写入参数"""
        can_id = self._build_extended_can_id(self.COMM_TYPE_WRITE_PARAM, self.host_can_id, motor_id)
        
        # 构建数据（小端序）
        value_bytes = struct.pack('<f', value)
        data = struct.pack('<HBx', param_index, 0) + value_bytes
        
        return self._send_frame(can_id, data)
    
    def set_run_mode(self, motor_id: int, mode: int = 0) -> bool:
        """设置运行模式（0=运控模式）"""
        return self.write_parameter(motor_id, self.PARAM_RUN_MODE, float(mode))
    
    def send_motion_control(self, motor_id: int, position: float, velocity: float,
                           kp: float, kd: float, torque: float = 0.0) -> bool:
        """
        发送运控模式命令
        
        Args:
            motor_id: 电机ID (1-6)
            position: 目标位置 (rad)
            velocity: 目标速度 (rad/s)
            kp: 位置增益
            kd: 速度增益
            torque: 前馈力矩 (Nm)
        """
        motor_type = MOTOR_TYPE_MAP.get(motor_id, MotorType.RS00)
        params = MOTOR_PARAMS[motor_type]
        
        # 转换为原始值
        pos_raw = self._float_to_uint16(position, params.p_min, params.p_max)
        vel_raw = self._float_to_uint16(velocity, -50.0, 50.0)  # 运控模式统一速度范围
        kp_raw = self._float_to_uint16(kp, params.kp_min, params.kp_max)
        kd_raw = self._float_to_uint16(kd, params.kd_min, params.kd_max)
        torque_raw = self._float_to_uint16(torque, params.t_min, params.t_max)
        
        # 构建 CAN ID（torque 在数据区2）
        can_id = self._build_extended_can_id(self.COMM_TYPE_MOTION_CONTROL, torque_raw, motor_id)
        
        # 构建数据（大端序）
        data = struct.pack('>HHHH', pos_raw, vel_raw, kp_raw, kd_raw)
        
        return self._send_frame(can_id, data)


class TeleopMasterSlave:
    """主从遥操作控制器"""
    
    def __init__(self, master_interface: str = 'can0',
                 slave_interfaces: List[str] = None,
                 master_kd: float = 1.0,
                 slave_kp: float = 150.0,
                 slave_kd: float = 1.0,
                 update_rate: float = 200.0):
        """
        初始化遥操作控制器
        
        Args:
            master_interface: 主臂 CAN 接口
            slave_interfaces: 从臂 CAN 接口列表
            master_kd: 主臂阻尼系数
            slave_kp: 从臂位置增益
            slave_kd: 从臂速度增益
            update_rate: 控制频率 (Hz)
        """
        self.master_interface = master_interface
        self.slave_interfaces = slave_interfaces or ['can1', 'can2', 'can3', 'can4']
        self.master_kd = master_kd
        self.slave_kp = slave_kp
        self.slave_kd = slave_kd
        self.update_rate = update_rate
        self.dt = 1.0 / update_rate
        
        self.master_can: Optional[RobstrideCanInterface] = None
        self.slave_cans: Dict[str, RobstrideCanInterface] = {}
        
        self.running = False
        self.motor_ids = [1, 2, 3, 4, 5, 6]
        
        # 统计信息
        self.loop_count = 0
        self.last_print_time = time.time()
        
    def connect(self) -> bool:
        """连接所有 CAN 接口"""
        # 连接主臂
        self.master_can = RobstrideCanInterface(self.master_interface)
        if not self.master_can.connect():
            return False
        
        # 连接从臂
        for interface in self.slave_interfaces:
            can = RobstrideCanInterface(interface)
            if can.connect():
                self.slave_cans[interface] = can
            else:
                print(f"[警告] 从臂 {interface} 连接失败，跳过")
        
        if not self.slave_cans:
            print("[错误] 没有可用的从臂连接")
            return False
        
        print(f"[信息] 主臂: {self.master_interface}, 从臂: {list(self.slave_cans.keys())}")
        return True
    
    def disconnect(self):
        """断开所有连接"""
        if self.master_can:
            self.master_can.disconnect()
        for can in self.slave_cans.values():
            can.disconnect()
        self.slave_cans.clear()
    
    def enable_all_motors(self):
        """使能所有电机"""
        print("[信息] 使能所有电机...")
        
        # 使能主臂电机
        for motor_id in self.motor_ids:
            # 先清除故障
            self.master_can.disable_motor(motor_id, clear_fault=True)
            time.sleep(0.03)
            
            # 设置运控模式
            self.master_can.set_run_mode(motor_id, 0)
            time.sleep(0.03)
            
            # 使能电机
            self.master_can.enable_motor(motor_id)
            time.sleep(0.03)
        
        # 使能从臂电机
        for interface, can in self.slave_cans.items():
            for motor_id in self.motor_ids:
                can.disable_motor(motor_id, clear_fault=True)
                time.sleep(0.03)
                can.set_run_mode(motor_id, 0)
                time.sleep(0.03)
                can.enable_motor(motor_id)
                time.sleep(0.03)
        
        print("[信息] 所有电机已使能")
    
    def disable_all_motors(self):
        """停止所有电机"""
        print("[信息] 停止所有电机...")
        
        if self.master_can:
            for motor_id in self.motor_ids:
                self.master_can.disable_motor(motor_id)
                time.sleep(0.01)
        
        for can in self.slave_cans.values():
            for motor_id in self.motor_ids:
                can.disable_motor(motor_id)
                time.sleep(0.01)
        
        print("[信息] 所有电机已停止")
    
    def start_receive_threads(self):
        """启动所有接收线程"""
        self.master_can.start_receive_thread()
        for can in self.slave_cans.values():
            can.start_receive_thread()
    
    def stop_receive_threads(self):
        """停止所有接收线程"""
        if self.master_can:
            self.master_can.stop_receive_thread()
        for can in self.slave_cans.values():
            can.stop_receive_thread()
    
    def run(self):
        """运行主循环"""
        self.running = True
        print(f"\n[信息] 遥操作启动 (主臂Kd={self.master_kd}, 从臂Kp={self.slave_kp}, Kd={self.slave_kd})")
        print("[信息] 按 Ctrl+C 停止\n")
        
        try:
            while self.running:
                loop_start = time.time()
                
                # 1. 发送主臂零力矩命令并读取位置
                master_positions = self._control_master()
                
                # 2. 向从臂发送位置命令
                if master_positions:
                    self._control_slaves(master_positions)
                
                # 3. 打印状态
                self._print_status(master_positions)
                
                # 4. 控制循环频率
                elapsed = time.time() - loop_start
                sleep_time = self.dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                self.loop_count += 1
                
        except KeyboardInterrupt:
            print("\n[信息] 收到停止信号")
        finally:
            self.running = False
    
    def _control_master(self) -> Dict[int, float]:
        """控制主臂（零力矩模式）并读取位置"""
        positions = {}
        
        for motor_id in self.motor_ids:
            # 发送零力矩命令（Kp=0，让机械臂可以自由移动）
            # 使用当前位置作为目标（无关紧要因为Kp=0）
            fb = self.master_can.get_feedback(motor_id)
            current_pos = fb.position if fb and fb.is_valid else 0.0
            
            self.master_can.send_motion_control(
                motor_id=motor_id,
                position=current_pos,  # 位置无关紧要
                velocity=0.0,
                kp=0.0,                # 零位置增益
                kd=self.master_kd,     # 阻尼
                torque=0.0             # 可选：添加重力补偿
            )
            
            # 读取位置
            if fb and fb.is_valid:
                positions[motor_id] = fb.position
        
        return positions
    
    def _control_slaves(self, master_positions: Dict[int, float]):
        """控制从臂（位置跟随）"""
        for interface, can in self.slave_cans.items():
            for motor_id in self.motor_ids:
                if motor_id not in master_positions:
                    continue
                
                # 目标位置 = 主臂位置
                target_pos = master_positions[motor_id]
                
                # 发送位置命令
                can.send_motion_control(
                    motor_id=motor_id,
                    position=target_pos,
                    velocity=0.0,
                    kp=self.slave_kp,    # Kp=150
                    kd=self.slave_kd,    # Kd=1
                    torque=0.0
                )
    
    def _print_status(self, master_positions: Dict[int, float]):
        """打印状态信息"""
        now = time.time()
        if now - self.last_print_time < 1.0:
            return
        
        self.last_print_time = now
        
        if master_positions:
            pos_str = ", ".join([f"J{i}:{master_positions.get(i, 0):.2f}" for i in self.motor_ids])
            print(f"[主臂位置] {pos_str} | 循环: {self.loop_count}")
        else:
            print(f"[警告] 未收到主臂反馈 | 循环: {self.loop_count}")


def signal_handler(sig, frame):
    """信号处理"""
    print("\n[信息] 正在停止...")
    sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description='EL-A3 主从遥操作')
    parser.add_argument('--master', type=str, default='can0',
                        help='主臂 CAN 接口 (默认: can0)')
    parser.add_argument('--slaves', type=str, default='can1,can2,can3,can4',
                        help='从臂 CAN 接口列表，逗号分隔 (默认: can1,can2,can3,can4)')
    parser.add_argument('--master-kd', type=float, default=1.0,
                        help='主臂阻尼系数 (默认: 1.0)')
    parser.add_argument('--slave-kp', type=float, default=150.0,
                        help='从臂位置增益 (默认: 150.0)')
    parser.add_argument('--slave-kd', type=float, default=1.0,
                        help='从臂速度增益 (默认: 1.0)')
    parser.add_argument('--rate', type=float, default=200.0,
                        help='控制频率 Hz (默认: 200.0)')
    
    args = parser.parse_args()
    
    # 解析从臂接口
    slave_interfaces = [s.strip() for s in args.slaves.split(',') if s.strip()]
    
    print("=" * 60)
    print("  EL-A3 主从遥操作")
    print("=" * 60)
    print(f"  主臂: {args.master} (零力矩模式, Kd={args.master_kd})")
    print(f"  从臂: {slave_interfaces} (位置跟随, Kp={args.slave_kp}, Kd={args.slave_kd})")
    print(f"  控制频率: {args.rate} Hz")
    print("=" * 60)
    
    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    
    # 创建控制器
    teleop = TeleopMasterSlave(
        master_interface=args.master,
        slave_interfaces=slave_interfaces,
        master_kd=args.master_kd,
        slave_kp=args.slave_kp,
        slave_kd=args.slave_kd,
        update_rate=args.rate
    )
    
    try:
        # 连接
        if not teleop.connect():
            print("[错误] 连接失败")
            return 1
        
        # 启动接收线程
        teleop.start_receive_threads()
        
        # 等待接收反馈
        print("[信息] 等待电机反馈...")
        time.sleep(0.5)
        
        # 使能电机
        teleop.enable_all_motors()
        
        # 等待稳定
        time.sleep(0.5)
        
        # 运行主循环
        teleop.run()
        
    except Exception as e:
        print(f"[错误] {e}")
        import traceback
        traceback.print_exc()
        return 1
        
    finally:
        # 停止电机
        teleop.disable_all_motors()
        
        # 停止接收线程
        teleop.stop_receive_threads()
        
        # 断开连接
        teleop.disconnect()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
