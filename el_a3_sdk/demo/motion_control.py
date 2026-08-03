#!/usr/bin/env python3
"""
示例: 关节运动控制

机械臂先回零位，然后执行一组预定义关节运动。

使用前确保:
  1. CAN 接口已激活
  2. 机械臂已上电
  3. 机械臂周围无障碍物
"""

import time
import math
from el_a3_sdk import ELA3Interface, LogLevel


def deg2rad(d: float) -> float:
    return d * math.pi / 180.0


def main():
    arm = ELA3Interface(can_name="can1", logger_level=LogLevel.INFO)

    if not arm.ConnectPort():
        print("连接失败")
        return

    arm.EnableArm()
    time.sleep(0.5)

    # 设置 PD 增益
    arm.SetPositionPD(kp=60.0, kd=3.5)

    waypoints = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [deg2rad(30), deg2rad(45), deg2rad(-30), 0.0, 0.0, 0.0],
        [deg2rad(-30), deg2rad(60), deg2rad(-45), deg2rad(20), 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]

    print("\n=== 关节运动控制 ===\n")

    try:
        for idx, wp in enumerate(waypoints):
            print(f"  -> 路径点 {idx}: {[f'{w*180/math.pi:.1f}°' for w in wp]}")
            arm.JointCtrlList(wp)

            # 等待到达（简单的定时等待）
            time.sleep(2.0)

            joints = arm.GetArmJointMsgs()
            current_deg = [j * 180.0 / math.pi for j in joints.to_list()]
            print(f"     当前位置: {[f'{d:.1f}°' for d in current_deg]}")

        print("\n运动完成")
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        arm.DisableArm()
        arm.DisconnectPort()


if __name__ == "__main__":
    main()
