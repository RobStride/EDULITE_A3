"""
EL-A3 运动学与动力学模块

基于 Pinocchio 库，独立于 ROS，CAN 和 ROS 模式共用。

功能：
  - FK (正运动学): 关节角 -> 末端位姿
  - IK (逆运动学): 末端位姿 -> 关节角（数值迭代法）
  - Jacobian: 6xN 雅可比矩阵
  - 重力补偿: RNEA(q, 0, 0)
  - 逆动力学 (RNEA): (q, v, a) -> tau
  - 正动力学 (ABA): (q, v, tau) -> a
  - 质量矩阵: CRBA(q)
  - 科氏力矩阵: C(q, v)
"""

import os
import logging
from typing import Optional, List, Tuple, Dict
from pathlib import Path

import numpy as np

try:
    import pinocchio as pin
    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False

from el_a3_sdk.data_types import ArmEndPose
from el_a3_sdk.protocol import DEFAULT_JOINT_DIRECTIONS, DEFAULT_JOINT_LIMITS

logger = logging.getLogger("el_a3_sdk.kinematics")

_DEFAULT_URDF = str(Path(__file__).resolve().parent.parent / "resources" / "urdf" / "el_a3.urdf")

JOINT_NAMES = ["L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint"]


class ELA3Kinematics:
    """
    EL-A3 Pinocchio 运动学/动力学封装

    用法::

        from el_a3_sdk.kinematics import ELA3Kinematics

        kin = ELA3Kinematics()
        pose = kin.forward_kinematics([0.0]*6)
        q = kin.inverse_kinematics(pose)
        tau_g = kin.compute_gravity([0.0]*6)
    """

    NUM_JOINTS = 6

    def __init__(
        self,
        urdf_path: Optional[str] = None,
        ee_frame_name: str = "end_effector",
        inertia_config_path: Optional[str] = None,
        joint_directions: Optional[Dict[int, float]] = None,
    ):
        if not PINOCCHIO_AVAILABLE:
            raise ImportError(
                "pinocchio is required for ELA3Kinematics. "
                "Install: sudo apt install ros-humble-pinocchio  or  pip install pin"
            )

        self._urdf_path = urdf_path or _DEFAULT_URDF
        if not os.path.isfile(self._urdf_path):
            raise FileNotFoundError(f"URDF not found: {self._urdf_path}")

        self._ee_frame_name = ee_frame_name
        self._joint_directions = joint_directions or dict(DEFAULT_JOINT_DIRECTIONS)

        self._model: pin.Model = pin.buildModelFromUrdf(self._urdf_path)
        grav = self._model.gravity
        grav.linear = np.array([0.0, 0.0, -9.81])
        self._model.gravity = grav
        self._data: pin.Data = pin.Data(self._model)

        self._ee_frame_id = self._model.getFrameId(ee_frame_name)
        if self._ee_frame_id >= self._model.nframes:
            available = [self._model.frames[i].name for i in range(self._model.nframes)]
            raise ValueError(
                f"Frame '{ee_frame_name}' not found. Available: {available}"
            )

        if inertia_config_path:
            self._load_calibrated_inertia(inertia_config_path)

        logger.info(
            "Pinocchio model loaded: nq=%d, nv=%d, nframes=%d, ee_frame='%s' (id=%d)",
            self._model.nq, self._model.nv, self._model.nframes,
            ee_frame_name, self._ee_frame_id,
        )

    @property
    def model(self) -> "pin.Model":
        return self._model

    @property
    def data(self) -> "pin.Data":
        return self._data

    @property
    def nq(self) -> int:
        return self._model.nq

    @property
    def nv(self) -> int:
        return self._model.nv

    # ================================================================
    # 正运动学
    # ================================================================

    def forward_kinematics(self, q: List[float]) -> ArmEndPose:
        """
        正运动学: 关节角度 -> 末端位姿

        Args:
            q: 关节角度列表 (rad), 长度 6

        Returns:
            ArmEndPose (x, y, z, rx, ry, rz) — 位置(m) + 欧拉角(rad, XYZ 内旋)
        """
        q_pin = self._to_pinocchio_q(q)

        pin.forwardKinematics(self._model, self._data, q_pin)
        pin.updateFramePlacements(self._model, self._data)

        oMf = self._data.oMf[self._ee_frame_id]
        pos = oMf.translation
        rpy = pin.rpy.matrixToRpy(oMf.rotation)

        return ArmEndPose(
            x=float(pos[0]), y=float(pos[1]), z=float(pos[2]),
            rx=float(rpy[0]), ry=float(rpy[1]), rz=float(rpy[2]),
        )

    def forward_kinematics_se3(self, q: List[float]) -> "pin.SE3":
        """正运动学: 返回 SE3 对象（用于内部计算）"""
        q_pin = self._to_pinocchio_q(q)
        pin.forwardKinematics(self._model, self._data, q_pin)
        pin.updateFramePlacements(self._model, self._data)
        return self._data.oMf[self._ee_frame_id].copy()

    # ================================================================
    # 逆运动学
    # ================================================================

    def inverse_kinematics(
        self,
        target_pose: ArmEndPose,
        q_init: Optional[List[float]] = None,
        max_iter: int = 200,
        eps: float = 1e-4,
        dt: float = 0.1,
        damping: float = 1e-6,
        joint_limit_margin: float = 0.05,
    ) -> Optional[List[float]]:
        """
        数值逆运动学: 末端位姿 -> 关节角度 (DLS 阻尼最小二乘)

        Args:
            target_pose: 目标末端位姿
            q_init: 初始关节角度（None 则用零位）
            max_iter: 最大迭代次数
            eps: 收敛精度 (位置 m / 角度 rad)
            dt: 步长
            damping: 阻尼因子（防止奇异）
            joint_limit_margin: 关节限位余量 (rad)

        Returns:
            收敛的关节角度列表 (rad)，失败返回 None
        """
        target_se3 = self._pose_to_se3(target_pose)

        q = self._to_pinocchio_q(q_init) if q_init else pin.neutral(self._model)

        for i in range(max_iter):
            pin.forwardKinematics(self._model, self._data, q)
            pin.updateFramePlacements(self._model, self._data)

            oMf = self._data.oMf[self._ee_frame_id]
            err_se3 = oMf.actInv(target_se3)
            err = pin.log6(err_se3).vector

            if np.linalg.norm(err) < eps:
                return self._from_pinocchio_q(q)

            J = pin.computeFrameJacobian(
                self._model, self._data, q, self._ee_frame_id,
                pin.LOCAL,
            )

            JtJ = J.T @ J + damping * np.eye(self._model.nv)
            dq = np.linalg.solve(JtJ, J.T @ err)
            q = pin.integrate(self._model, q, dq * dt)

            self._clamp_to_limits(q, joint_limit_margin)

        logger.warning("IK did not converge after %d iterations (err=%.4f)", max_iter, np.linalg.norm(err))
        return None

    def ik_step(
        self,
        target_pose: ArmEndPose,
        q_current: List[float],
        damping: float = 5e-3,
        max_step: float = 0.5,
        max_iter: int = 3,
        converge_eps: float = 1e-5,
    ) -> Tuple[Optional[List[float]], float]:
        """
        迭代 Jacobian IK: SE3 误差 → 自适应 DLS → 关节增量（实时控制循环专用）

        执行最多 max_iter 次 FK + Jacobian 迭代，误差收敛则提前退出。
        正常运动区域 1 步即收敛，仅在近奇异区做额外迭代消除跟踪残差。
        通过 SE3 对数映射计算精确位姿误差，不受欧拉角奇异影响。
        内置 SVD 自适应阻尼，奇异区自动增大阻尼防止关节抖动。

        Args:
            target_pose: 目标末端位姿
            q_current: 当前关节角度 (rad)
            damping: DLS 基础阻尼因子
            max_step: 单步最大关节增量 (rad)
            max_iter: 最大迭代次数（默认 3，正常区域 1 步收敛提前退出）
            converge_eps: 收敛阈值 (rad/m)

        Returns:
            (q_new, err_norm) — 新关节角度和任务空间误差范数；
            IK 误差极小时返回 (q_current, err_norm)
        """
        q_pin = self._to_pinocchio_q(q_current)
        target_se3 = self._pose_to_se3(target_pose)
        err_norm = 0.0

        for _iteration in range(max_iter):
            pin.forwardKinematics(self._model, self._data, q_pin)
            pin.updateFramePlacements(self._model, self._data)

            oMf = self._data.oMf[self._ee_frame_id]
            err_se3 = oMf.actInv(target_se3)
            err = pin.log6(err_se3).vector
            err_norm = float(np.linalg.norm(err))

            if err_norm < converge_eps:
                break

            J = pin.computeFrameJacobian(
                self._model, self._data, q_pin, self._ee_frame_id,
                pin.LOCAL,
            )

            sigma = np.linalg.svd(J, compute_uv=False)
            s_min, s_max = float(sigma[-1]), float(sigma[0])

            lam = damping
            s_thresh = 0.035
            if s_min < s_thresh:
                r = (s_min / s_thresh) ** 2
                lam = max(lam, damping + 0.08 * (1.0 - r))
            cond = s_max / max(s_min, 1e-12)
            if cond > 80.0:
                rc = min((cond - 80.0) / 80.0, 1.0)
                lam = max(lam, damping + 0.02 * rc)

            JtJ = J.T @ J + lam * np.eye(self._model.nv)
            rhs = J.T @ err

            q5_idx = 4
            q5_val = float(q_pin[q5_idx])
            q5_safe = 0.1
            if abs(q5_val) < q5_safe and s_min < s_thresh:
                grad = np.zeros(self._model.nv)
                sign = 1.0 if q5_val >= 0 else -1.0
                grad[q5_idx] = 0.5 * (q5_safe * sign - q5_val)
                rhs += lam * grad

            dq = np.linalg.solve(JtJ, rhs)

            dq_norm = float(np.linalg.norm(dq))
            if dq_norm > max_step:
                dq = dq * (max_step / dq_norm)

            q_pin = pin.integrate(self._model, q_pin, dq)
            self._clamp_to_limits(q_pin)

        return self._from_pinocchio_q(q_pin), err_norm

    # ================================================================
    # Jacobian
    # ================================================================

    def compute_jacobian(self, q: List[float]) -> np.ndarray:
        """
        计算末端帧的 6xN Jacobian 矩阵 (世界对齐坐标系)

        Returns:
            (6, nv) numpy 数组, 前 3 行为线速度, 后 3 行为角速度
        """
        q_pin = self._to_pinocchio_q(q)
        pin.forwardKinematics(self._model, self._data, q_pin)
        pin.updateFramePlacements(self._model, self._data)
        J = pin.computeFrameJacobian(
            self._model, self._data, q_pin, self._ee_frame_id,
            pin.LOCAL_WORLD_ALIGNED,
        )
        return J.copy()

    # ================================================================
    # 重力补偿
    # ================================================================

    def compute_gravity(self, q: List[float]) -> List[float]:
        """
        计算重力补偿力矩: RNEA(q, v=0, a=0)

        Args:
            q: 关节角度 (rad)

        Returns:
            各关节重力补偿力矩 (Nm), 长度 nv
        """
        q_pin = self._to_pinocchio_q(q)
        v = np.zeros(self._model.nv)
        a = np.zeros(self._model.nv)
        tau = pin.rnea(self._model, self._data, q_pin, v, a)
        return [float(tau[i]) for i in range(self._model.nv)]

    # ================================================================
    # 逆动力学 (RNEA)
    # ================================================================

    def inverse_dynamics(
        self, q: List[float], v: List[float], a: List[float],
    ) -> List[float]:
        """
        逆动力学 (RNEA): 给定 q, v, a -> 关节力矩 tau

        tau = M(q)*a + C(q,v)*v + g(q)
        """
        q_pin = self._to_pinocchio_q(q)
        v_pin = self._to_pinocchio_v(v)
        a_pin = self._to_pinocchio_v(a)
        tau = pin.rnea(self._model, self._data, q_pin, v_pin, a_pin)
        return [float(tau[i]) for i in range(self._model.nv)]

    # ================================================================
    # 正动力学 (ABA)
    # ================================================================

    def forward_dynamics(
        self, q: List[float], v: List[float], tau: List[float],
    ) -> List[float]:
        """
        正动力学 (ABA): 给定 q, v, tau -> 关节加速度 a

        a = M(q)^{-1} * (tau - C(q,v)*v - g(q))
        """
        q_pin = self._to_pinocchio_q(q)
        v_pin = self._to_pinocchio_v(v)
        tau_pin = self._to_pinocchio_v(tau)
        a = pin.aba(self._model, self._data, q_pin, v_pin, tau_pin)
        return [float(a[i]) for i in range(self._model.nv)]

    # ================================================================
    # 质量矩阵
    # ================================================================

    def mass_matrix(self, q: List[float]) -> np.ndarray:
        """
        关节空间惯性矩阵 M(q) (CRBA)

        Returns:
            (nv, nv) 对称正定矩阵
        """
        q_pin = self._to_pinocchio_q(q)
        M = pin.crba(self._model, self._data, q_pin)
        M = np.triu(M) + np.triu(M, 1).T
        return M.copy()

    # ================================================================
    # 科氏力 / 离心力矩阵
    # ================================================================

    def coriolis_matrix(self, q: List[float], v: List[float]) -> np.ndarray:
        """
        科氏力 + 离心力矩阵 C(q, v)

        满足: tau_coriolis = C(q, v) * v

        Returns:
            (nv, nv) numpy 数组
        """
        q_pin = self._to_pinocchio_q(q)
        v_pin = self._to_pinocchio_v(v)
        C = pin.computeCoriolisMatrix(self._model, self._data, q_pin, v_pin)
        return C.copy()

    # ================================================================
    # 辅助方法
    # ================================================================

    def _to_pinocchio_q(self, q: List[float]) -> np.ndarray:
        """将用户 6 维关节角度转为 Pinocchio 配置向量"""
        q_arr = np.array(q[:self._model.nq], dtype=float)
        if len(q_arr) < self._model.nq:
            q_arr = np.concatenate([q_arr, np.zeros(self._model.nq - len(q_arr))])
        return q_arr
    
    def _to_pinocchio_v(self,v:List[float]) -> np.ndarray:
        v_arr = np.array(v[:self._model.nv],dtype=float)
        if len(v_arr)< self.model.nv:
            v_arr = np.concatenate([v_arr,np.zeros(self.model.nq - len(v_arr))])
        return v_arr

    def _from_pinocchio_q(self, q_pin: np.ndarray) -> List[float]:
        """将 Pinocchio 配置向量转为用户列表"""
        return [float(q_pin[i]) for i in range(min(self.NUM_JOINTS, len(q_pin)))]

    def _pose_to_se3(self, pose: ArmEndPose) -> "pin.SE3":
        """ArmEndPose -> pinocchio SE3"""
        R = pin.rpy.rpyToMatrix(pose.rx, pose.ry, pose.rz)
        t = np.array([pose.x, pose.y, pose.z])
        return pin.SE3(R, t)

    def _clamp_to_limits(self, q: np.ndarray, margin: float = 0.0):
        """将关节角度夹紧到限位范围内"""
        for i in range(min(self.NUM_JOINTS, len(q))):
            mid = i + 1
            limits = DEFAULT_JOINT_LIMITS.get(mid)
            if limits:
                lo, hi = limits[0] + margin, limits[1] - margin
                q[i] = np.clip(q[i], lo, hi)

    def _load_calibrated_inertia(self, config_path: str):
        """从 YAML 加载标定惯量参数并应用到 Pinocchio 模型"""
        try:
            import yaml
        except ImportError:
            logger.warning("PyYAML not available, skipping inertia calibration loading")
            return

        if not os.path.isfile(config_path):
            logger.warning("Inertia config not found: %s", config_path)
            return

        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)

        if not cfg or not cfg.get("use_calibrated_params", False):
            logger.info("Calibrated inertia params disabled in config")
            return

        params = cfg.get("inertia_params", {})
        link_map = {"L2": 1, "L3": 2, "L4": 3, "L5": 4, "L6": 5}

        for link_name, body_idx_offset in link_map.items():
            if link_name not in params:
                continue
            p = params[link_name]
            body_idx = body_idx_offset + 1
            if body_idx >= self._model.nbodies:
                continue

            mass = p.get("mass", None)
            com = p.get("com", None)
            if mass is not None:
                self._model.inertias[body_idx].mass = mass
            if com is not None and len(com) == 3:
                self._model.inertias[body_idx].lever = np.array(com)

        self._data = pin.Data(self._model)
        logger.info("Calibrated inertia params applied from %s", config_path)
