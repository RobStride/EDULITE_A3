#!/usr/bin/env python3
"""
EL-A3机械臂简单运动测试
顺序移动各关节，验证仿真显示更新
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class SimpleMotionTest(Node):
    def __init__(self):
        super().__init__('simple_motion_test')
        
        self.joint_names = [
            'L1_joint', 'L2_joint', 'L3_joint',
            'L4_joint', 'L5_joint', 'L6_joint'
        ]
        
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('等待 arm_controller action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server 不可用!')
            return
        
        self.get_logger().info('===== 开始简单运动测试 =====')
        self.run_test()
    
    def send_trajectory(self, positions, duration_sec=3):
        """发送轨迹命令"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=duration_sec, nanosec=0)
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f'发送目标: {[f"{p:.2f}" for p in positions]}')
        
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝!')
            return False
        
        self.get_logger().info('执行中...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 5)
        
        return True
    
    def run_test(self):
        """运行测试序列"""
        test_positions = [
            # 测试1: 归零
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "归零位置"),
            # 测试2: 移动L1关节
            ([0.5, 0.0, 0.0, 0.0, 0.0, 0.0], "L1 +0.5rad"),
            # 测试3: 移动L2关节
            ([0.5, 0.5, 0.0, 0.0, 0.0, 0.0], "L2 +0.5rad"),
            # 测试4: 移动L3关节
            ([0.5, 0.5, -0.5, 0.0, 0.0, 0.0], "L3 -0.5rad"),
            # 测试5: 移动末端关节
            ([0.5, 0.5, -0.5, 0.3, 0.3, 0.3], "L4,L5,L6 +0.3rad"),
            # 测试6: 回到零位
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "回到零位"),
        ]
        
        for positions, description in test_positions:
            self.get_logger().info(f'\n>>> 测试: {description}')
            if self.send_trajectory(positions, duration_sec=3):
                self.get_logger().info(f'✅ {description} 完成')
            else:
                self.get_logger().error(f'❌ {description} 失败')
            time.sleep(1)  # 等待一秒观察
        
        self.get_logger().info('\n===== 运动测试完成 =====')
        self.get_logger().info('请查看RViz中机械臂是否跟随移动！')


def main():
    rclpy.init()
    node = SimpleMotionTest()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

