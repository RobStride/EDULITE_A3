#!/usr/bin/env python3
"""
Standalone zeroing test script.
Repeatedly tests zeroing error and monitors joint positions.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
import sys

class ZeroTest(Node):
    def __init__(self):
        super().__init__('zero_test')
        
        self.joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']
        self.current_positions = [0.0] * 6
        self.position_received = False
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info('Zero test script started, waiting for joint states...')
    
    def joint_state_callback(self, msg):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
        self.position_received = True
    
    def wait_for_position(self, timeout=10.0):
        start = time.time()
        while not self.position_received and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.position_received
    
    def get_errors(self, target=None):
        """Compute error between current and target positions"""
        if target is None:
            target = [0.0] * 6
        
        # Update positions
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
        
        errors = []
        for i in range(6):
            errors.append(self.current_positions[i] - target[i])
        return errors
    
    def print_status(self, label=""):
        """Print current status"""
        errors = self.get_errors()
        
        print(f"\n{'='*60}")
        if label:
            print(f"  {label}")
            print(f"{'='*60}")
        
        print(f"{'Joint':<8} {'Pos(rad)':<12} {'Pos(°)':<10} {'Err(rad)':<12} {'Err(°)':<10}")
        print(f"{'-'*60}")
        
        for i, name in enumerate(['L1', 'L2', 'L3', 'L4', 'L5', 'L6']):
            pos = self.current_positions[i]
            pos_deg = pos * 180.0 / 3.14159
            err = errors[i]
            err_deg = err * 180.0 / 3.14159
            print(f"{name:<8} {pos:>10.4f}  {pos_deg:>8.2f}°  {err:>10.4f}  {err_deg:>8.2f}°")
        
        max_err = max(abs(e) for e in errors)
        avg_err = sum(abs(e) for e in errors) / 6
        print(f"{'-'*60}")
        print(f"Max error: {max_err:.4f} rad ({max_err*180/3.14159:.2f}°)")
        print(f"Avg error: {avg_err:.4f} rad ({avg_err*180/3.14159:.2f}°)")
        print(f"{'='*60}")
        
        return max_err, avg_err
    
    def monitor_loop(self, interval=1.0, count=None):
        """Continuously monitor positions"""
        i = 0
        try:
            while count is None or i < count:
                self.print_status(f"Monitor #{i+1}")
                time.sleep(interval)
                i += 1
        except KeyboardInterrupt:
            print("\nMonitoring stopped")


def main():
    rclpy.init()
    
    tester = ZeroTest()
    
    # Wait for positions
    if not tester.wait_for_position():
        tester.get_logger().error('Failed to get joint positions. Check if controller is running.')
        rclpy.shutdown()
        return
    
    print("\n" + "="*60)
    print("  Zeroing error monitor")
    print("  Press Ctrl+C to stop")
    print("="*60)
    
    # Continuous monitoring
    tester.monitor_loop(interval=2.0)
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


















