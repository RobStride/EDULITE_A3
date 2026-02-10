#!/usr/bin/env python3
"""
EL-A3 teleoperation test program 2

Use master arm on can1 to control slave arms on can0, can2, can3, can4

Usage:
    python3 test_teleop_can1_master.py
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
from typing import Dict, Optional, List
from enum import IntEnum


class MotorType(IntEnum):
    RS00 = 0  # Joints 1-3
    EL05 = 1  # Joints 4-6


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
                  7: MotorType.EL05}  # Motor 7: gripper


@dataclass
class MotorFeedback:
    motor_id: int = 0
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    temperature: float = 0.0
    is_valid: bool = False


class CANInterface:
    """Simplified CAN interface"""
    
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
            print(f"[{self.interface}] Connected")
            return True
        except Exception as e:
            print(f"[{self.interface}] Connection failed: {e}")
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
        
        if comm_type == 2 and 1 <= motor_id <= 7:
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
    """Teleoperation test: can1 master -> can0, can2, can3, can4 slaves"""
    
    def __init__(self):
        # Master arm: can1
        self.master = CANInterface('can1')
        
        # Slave arms: can0, can2, can3, can4
        self.slaves: List[CANInterface] = [
            CANInterface('can0'),
            CANInterface('can2'),
            CANInterface('can3'),
            CANInterface('can4'),
        ]
        self.slave_names = ['can0', 'can2', 'can3', 'can4']
        
        self.motor_ids = [1, 2, 3, 4, 5, 6, 7]  # Includes gripper motor
        self.running = False
        
        # Control parameters
        self.master_kd = 0.0      # Master damping (zero torque)
        self.slave_kp = 50.0      # Slave Kp
        self.slave_kd = 2.0       # Slave Kd
        self.rate = 200.0         # Control frequency (Hz)
        
        self.loop_count = 0
        self.last_print = time.time()
    
    def connect(self) -> bool:
        # Connect master arm
        if not self.master.connect():
            return False
        
        # Connect all slave arms
        connected_slaves = []
        for i, slave in enumerate(self.slaves):
            if slave.connect():
                connected_slaves.append(slave)
            else:
                # Disconnect already-connected interfaces
                self.master.disconnect()
                for s in connected_slaves:
                    s.disconnect()
                return False
        
        return True
    
    def disconnect(self):
        self.master.disconnect()
        for slave in self.slaves:
            slave.disconnect()
    
    def enable_motors(self):
        print("[INFO] Enabling motors...")
        
        # Step 1: clear faults and set motion control mode
        print("[DEBUG] Step 1: clear faults and set motion control mode")
        print("  [master: can1]")
        for motor_id in self.motor_ids:
            self.master.disable_motor(motor_id, clear_fault=True)
            self.master.set_run_mode(motor_id, 0)
            time.sleep(0.02)
        
        for name, slave in zip(self.slave_names, self.slaves):
            print(f"  [slave: {name}]")
            for motor_id in self.motor_ids:
                slave.disable_motor(motor_id, clear_fault=True)
                slave.set_run_mode(motor_id, 0)
                time.sleep(0.02)
        
        time.sleep(0.1)
        
        # Step 2: enable motors
        print("[DEBUG] Step 2: enable motors")
        print("  [master: can1]")
        for motor_id in self.motor_ids:
            ok = self.master.enable_motor(motor_id)
            print(f"    M{motor_id}: {ok}")
            time.sleep(0.02)
        
        for name, slave in zip(self.slave_names, self.slaves):
            print(f"  [slave: {name}]")
            for motor_id in self.motor_ids:
                ok = slave.enable_motor(motor_id)
                print(f"    M{motor_id}: {ok}")
                time.sleep(0.02)
        
        # Wait for feedback update
        time.sleep(0.3)
        
        # Step 3: check slave feedback
        print("[DEBUG] Step 3: check slave feedback")
        for name, slave in zip(self.slave_names, self.slaves):
            print(f"  [{name}]")
            for motor_id in self.motor_ids:
                fb = slave.get_feedback(motor_id)
                if fb and fb.is_valid:
                    print(f"    M{motor_id}: pos={fb.position:.3f} (valid)")
                else:
                    print(f"    M{motor_id}: no feedback!")
        
        # Step 4: send initial commands
        print("[DEBUG] Step 4: send initial commands")
        for motor_id in self.motor_ids:
            fb = self.master.get_feedback(motor_id)
            pos = fb.position if fb and fb.is_valid else 0.0
            
            # Master: zero torque (Kp=0, Kd=0, torque=0)
            self.master.send_motion(motor_id, pos, 0.0, 0.0, 0.0, 0.0)
            
            # All slaves: follow master position
            for slave in self.slaves:
                slave.send_motion(motor_id, pos, 0.0, self.slave_kp, self.slave_kd, 0.0)
            
            print(f"  M{motor_id}: pos={pos:.3f} -> slaves Kp={self.slave_kp}")
            time.sleep(0.01)
        
        print("[INFO] Motors enabled")
    
    def disable_motors(self):
        print("[INFO] Stopping motors...")
        for motor_id in self.motor_ids:
            self.master.disable_motor(motor_id)
            for slave in self.slaves:
                slave.disable_motor(motor_id)
            time.sleep(0.01)
    
    def run(self):
        self.running = True
        dt = 1.0 / self.rate
        
        print(f"\n[INFO] Teleoperation started")
        print(f"  Master: can1")
        print(f"  Slaves: can0, can2, can3, can4")
        print(f"  Slave gains: Kp={self.slave_kp}, Kd={self.slave_kd}")
        print(f"  Control frequency: {self.rate} Hz")
        print("[INFO] Press Ctrl+C to stop\n")
        
        while self.running:
            t0 = time.time()
            
            # Read master position and send commands
            for motor_id in self.motor_ids:
                fb = self.master.get_feedback(motor_id)
                pos = fb.position if fb and fb.is_valid else 0.0
                
                # Master: zero torque
                self.master.send_motion(motor_id, pos, 0.0, 0.0, 0.0, 0.0)
                
                # All slaves: follow master position
                for slave in self.slaves:
                    slave.send_motion(motor_id, pos, 0.0, self.slave_kp, self.slave_kd, 0.0)
            
            # Print status once per second
            self.loop_count += 1
            now = time.time()
            if now - self.last_print >= 1.0:
                actual_freq = self.loop_count / (now - self.last_print)
                self.last_print = now
                self.loop_count = 0
                
                # Show master positions
                m_info = []
                for mid in self.motor_ids:
                    fb_m = self.master.get_feedback(mid)
                    m_info.append(f"{fb_m.position:.2f}" if fb_m and fb_m.is_valid else "N/A")
                
                print(f"[{actual_freq:.0f}Hz] Master(can1): {' '.join(m_info)}")
                
                # Show each slave's positions
                for name, slave in zip(self.slave_names, self.slaves):
                    s_info = []
                    for mid in self.motor_ids:
                        fb_s = slave.get_feedback(mid)
                        s_info.append(f"{fb_s.position:.2f}" if fb_s and fb_s.is_valid else "N/A")
                    print(f"        Slave({name}): {' '.join(s_info)}")
            
            # Control frequency
            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
    
    def stop(self):
        self.running = False


def main():
    print("=" * 50)
    print("  EL-A3 Teleoperation Test 2")
    print("  Master: can1")
    print("  Slaves: can0, can2, can3, can4")
    print("=" * 50)
    
    teleop = TeleopTest()
    
    def signal_handler(sig, frame):
        print("\n[INFO] Stopping...")
        teleop.stop()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        if not teleop.connect():
            print("[ERROR] Connection failed")
            return 1
        
        # Start receive threads for master and slaves
        teleop.master.start_recv()
        for slave in teleop.slaves:
            slave.start_recv()
        
        time.sleep(0.5)  # Wait for feedback
        
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
        teleop.master.stop_recv()
        for slave in teleop.slaves:
            slave.stop_recv()
        teleop.disconnect()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
