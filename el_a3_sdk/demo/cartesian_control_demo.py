#!/usr/bin/env python3
"""
笛卡尔位姿控制示例

演示使用 SDK 的 EndPoseCtrl 和 MoveL 功能。
需要 Pinocchio: pip install pin
"""

import os
import time
import math
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from el_a3_sdk import ELA3Interface, ArmEndPose

# 零位（各关节 0 rad）
HOME_POSITIONS = [0.0] * 6

# 笛卡尔控制工作姿态：远离 J2/J3 限位和奇异构型，IK 才有收敛空间
READY_POSITIONS = [math.radians(d) for d in [0, 45, -60, 0, 15, 0]]


def main():
    arm = ELA3Interface(can_name="can1", default_kp=80.0, default_kd=4.0)
    arm.ConnectPort()
    arm.EnableArm()
    time.sleep(0.5)

    # 上电回零：从当前（可能下垂的）姿态平滑回到零位
    print("--- 回零 ---")
    arm.MoveJ(HOME_POSITIONS, duration=4.0)
    time.sleep(0.5)

    # 零位贴着 J2/J3 限位，不适合直接做笛卡尔运动，先抬到工作姿态
    print("--- 移动到工作姿态 ---")
    arm.MoveJ(READY_POSITIONS, duration=3.0)
    time.sleep(0.5)

    # 获取当前末端位姿
    pose = arm.GetArmEndPoseMsgs()
    print(f"当前末端位姿: x={pose.x:.4f}, y={pose.y:.4f}, z={pose.z:.4f}")
    print(f"              rx={pose.rx:.4f}, ry={pose.ry:.4f}, rz={pose.rz:.4f}")

    # EndPoseCtrl: 移动到指定笛卡尔位姿
    print("\n--- EndPoseCtrl 测试 ---")
    target_x = pose.x + 0.05
    target_z = pose.z + 0.03
    print(f"目标: x={target_x:.4f}, z={target_z:.4f}")
    success = arm.EndPoseCtrl(target_x, pose.y, target_z, pose.rx, pose.ry, pose.rz, duration=3.0)
    print(f"EndPoseCtrl 结果: {'成功' if success else '失败'}")
    time.sleep(1.0)

    # MoveL: 直线运动回到原始位姿
    print("\n--- MoveL 测试 ---")
    print(f"直线运动回到: x={pose.x:.4f}, z={pose.z:.4f}")
    success = arm.MoveL(pose, duration=3.0)
    print(f"MoveL 结果: {'成功' if success else '失败'}")
    time.sleep(1.0)

    # 获取 Jacobian
    print("\n--- Jacobian ---")
    J = arm.GetJacobian()
    print(f"Jacobian shape: {J.shape}")
    print(f"Jacobian:\n{J}")

    # 回零后再断使能，避免从高处失能瘫落
    print("\n--- 回零 ---")
    arm.MoveJ(HOME_POSITIONS, duration=4.0)
    time.sleep(0.5)

    arm.DisableArm()
    arm.DisconnectPort()
    print("\n完成")


if __name__ == "__main__":
    main()
