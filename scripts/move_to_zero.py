#!/usr/bin/env python3
"""
将EL-A3机械臂所有关节移动到零位
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class MoveToZero(Node):
    def __init__(self):
        super().__init__('move_to_zero')
        
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
        
        self.get_logger().info('连接成功，发送归零命令...')
        self.send_goal()
    
    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        # 目标点：所有关节到0位置
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start = Duration(sec=5, nanosec=0)  # 5秒内到达
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info('发送目标: 所有关节 -> 0.0 rad')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝!')
            return
        
        self.get_logger().info('目标已接受，正在执行...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        positions = feedback.actual.positions
        if len(positions) >= 6:
            self.get_logger().info(
                f'当前位置: L1={positions[0]:.2f}, L2={positions[1]:.2f}, '
                f'L3={positions[2]:.2f}, L4={positions[3]:.2f}, '
                f'L5={positions[4]:.2f}, L6={positions[5]:.2f}'
            )
    
    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info('✅ 归零完成!')
        else:
            self.get_logger().error(f'执行失败, error_code: {result.error_code}')
        
        # 完成后退出
        rclpy.shutdown()


def main():
    rclpy.init()
    node = MoveToZero()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

