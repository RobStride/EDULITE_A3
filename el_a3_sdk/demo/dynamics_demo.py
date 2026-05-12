#!/usr/bin/env python3
"""
动力学计算示例

演示 ELA3Kinematics 的 FK/IK/Jacobian/重力补偿/RNEA/ABA/质量矩阵/科氏力。
需要 Pinocchio：pip install pin
"""

import os
import sys
import numpy as np
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from el_a3_sdk.kinematics import ELA3Kinematics
from el_a3_sdk.data_types import ArmEndPose


def main():
    kin = ELA3Kinematics()
    print(f"模型: nq={kin.nq}, nv={kin.nv}")

    q_home = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]
    print(f"\n关节角度 q_home = {q_home}")

    # FK
    pose = kin.forward_kinematics(q_home)
    print(f"\n=== 正运动学 (FK) ===")
    print(f"末端位姿: x={pose.x:.4f}, y={pose.y:.4f}, z={pose.z:.4f}")
    print(f"          rx={pose.rx:.4f}, ry={pose.ry:.4f}, rz={pose.rz:.4f}")

    # IK
    print(f"\n=== 逆运动学 (IK) ===")
    q_sol = kin.inverse_kinematics(pose, q_init=[0.0]*6)
    if q_sol:
        print(f"IK 解: {[f'{v:.4f}' for v in q_sol]}")
        err = sum((a - b)**2 for a, b in zip(q_home, q_sol))**0.5
        print(f"误差范数: {err:.6f} rad")
    else:
        print("IK 求解失败")

    # Jacobian
    print(f"\n=== Jacobian ===")
    J = kin.compute_jacobian(q_home)
    print(f"形状: {J.shape}")
    print(f"J =\n{np.array2string(J, precision=4, suppress_small=True)}")

    # 重力补偿
    print(f"\n=== 重力补偿 (RNEA, v=0, a=0) ===")
    tau_g = kin.compute_gravity(q_home)
    print(f"重力力矩: {[f'{t:.4f}' for t in tau_g]} Nm")

    # 质量矩阵
    print(f"\n=== 质量矩阵 (CRBA) ===")
    M = kin.mass_matrix(q_home)
    print(f"形状: {M.shape}")
    print(f"对角线: {[f'{M[i,i]:.4f}' for i in range(M.shape[0])]}")

    # 科氏力矩阵
    print(f"\n=== 科氏力矩阵 ===")
    v = [0.1, 0.2, -0.1, 0.0, 0.0, 0.0, 0.0]
    C = kin.coriolis_matrix(q_home, v)
    print(f"C(q,v) 形状: {C.shape}")
    print(f"C*v = {[f'{x:.4f}' for x in (C @ np.array(v))]}")

    # 逆动力学 (RNEA)
    print(f"\n=== 逆动力学 (RNEA) ===")
    a = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    tau = kin.inverse_dynamics(q_home, v, a)
    print(f"tau = M*a + C*v + g = {[f'{t:.4f}' for t in tau]} Nm")

    # 正动力学 (ABA)
    print(f"\n=== 正动力学 (ABA) ===")
    a_result = kin.forward_dynamics(q_home, [0.0]*7, tau_g)
    print(f"给定 tau=gravity: a = {[f'{x:.6f}' for x in a_result]} rad/s²")
    print("(应接近零，因为重力恰好被力矩平衡)")

    print("\n完成")


if __name__ == "__main__":
    main()
