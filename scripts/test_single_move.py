#!/usr/bin/env python3
"""测试单次运动"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time

class TestMove(Node):
    def __init__(self):
        super().__init__('test_move')
        
        self.joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 
                           'L4_joint', 'L5_joint', 'L6_joint']
        
        self.current_positions = [0.0] * 6
        self.state_received = False
        
        # 订阅
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Action client
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        
    def joint_state_callback(self, msg):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
        self.state_received = True
    
    def move_to(self, positions, duration=3.0):
        """移动到指定位置"""
        print(f"目标: {positions}")
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            print("Action server 不可用!")
            return False
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=int(duration), nanosec=0)
        
        goal_msg.trajectory.points = [point]
        
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            print("目标被拒绝!")
            return False
        
        print("执行中...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 5)
        
        # 更新位置
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        print(f"完成后位置: {[f'{p:.3f}' for p in self.current_positions]}")
        return True

def main():
    rclpy.init()
    node = TestMove()
    
    # 等待状态
    print("等待关节状态...")
    while not node.state_received:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    print(f"当前位置: {[f'{p:.3f}' for p in node.current_positions]}")
    
    # 测试移动到 home 位置
    home = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]
    print("\n测试: 移动到 home 位置")
    node.move_to(home)
    
    # 测试移动 L2 到 -1.0
    test_pos = [0.0, -1.0, -0.785, 0.0, 0.0, 0.0]
    print("\n测试: 移动 L2 到 -1.0 rad")
    node.move_to(test_pos)
    
    # 回到 home
    print("\n测试: 回到 home")
    node.move_to(home)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
