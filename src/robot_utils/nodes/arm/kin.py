"""Arm kinematics and dynamics for the arm node (pinocchio, through rebot_ctl). No bus, no threads.

One ``ArmKinematics`` per node. It owns the pinocchio model with a ``tcp`` frame added on
``end_link`` (the TCP offset), with the joint limits narrowed to what the hardware accepts (URDF
and the config's ``q_min_deg`` / ``q_max_deg``, as ``RebotArm`` enforces them), so IK never
returns a setpoint the driver would refuse. Every method is safe to call from any thread
(pinocchio ``Data`` is per thread).

Conventions: poses and twists in the arm base frame (URDF ``base_link``); joint vectors are the 6
arm joints; the fingers exist in the model only and stay at 0.
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Optional

import numpy as np
import pinocchio as pin

from reBotArm_control_py.dynamics import compute_generalized_gravity
from reBotArm_control_py.kinematics import get_arm_joint_count, get_end_effector_frame_id, load_robot_model
from reBotArm_control_py.kinematics.inverse_kinematics import IKParams, solve_ik

N = 6
DEFAULT_TCP_OFFSET = (-0.15, 0.0, 0.0)   # m in end_link: the Quest teleop's TCP
PINV_DAMPING = 0.01                       # damped pseudo-inverse for the twist feedforward
IK_POS_TOL = 0.002                        # m: an IK solve is accepted within this position error
IK_ROT_TOL = math.radians(10.0)           # rad: and this orientation error (see ArmKinematics.__init__)
EXACT_ROT = math.radians(1.0)             # rad: within this (and IK_POS_TOL) the solve is "exact"
LIMIT_NEAR = math.radians(0.5)            # rad: a joint this close to a limit is reported as at it


@dataclass
class IKOut:
    ok: bool
    q: np.ndarray
    pos_err_mm: float
    rot_err_deg: float
    within_limits: bool
    iterations: int
    reason: Optional[str] = None
    exact: bool = False
    nearest: Optional[dict] = None            # pose_public(FK(q)): where q puts the TCP (reachable as is)
    at_limit: list = None                     # joint names at (or past) a limit

    def public(self) -> dict:
        return {"ok": self.ok, "exact": self.exact, "q": [round(float(v), 6) for v in self.q],
                "pos_err_mm": round(self.pos_err_mm, 3), "rot_err_deg": round(self.rot_err_deg, 3),
                "within_limits": self.within_limits, "joints_at_limit": self.at_limit or [],
                "iterations": self.iterations, "reason": self.reason, "nearest": self.nearest}


def se3(pos, quat_xyzw=None, rpy=None) -> pin.SE3:
    """A pose from a position and a quaternion [x, y, z, w] or roll/pitch/yaw (rad, fixed X-Y-Z)."""
    p = np.asarray(pos, dtype=float).reshape(3)
    o = quat_xyzw if quat_xyzw is not None else rpy
    if not (np.all(np.isfinite(p)) and (o is None or np.all(np.isfinite(np.asarray(o, dtype=float))))):
        raise ValueError("a pose must be finite numbers")
    if quat_xyzw is not None:
        x, y, z, w = (float(v) for v in quat_xyzw)
        n = math.sqrt(x * x + y * y + z * z + w * w)
        if not n > 1e-9:
            raise ValueError("quaternion must not be zero")
        R = pin.Quaternion(w / n, x / n, y / n, z / n).toRotationMatrix()
    elif rpy is not None:
        R = pin.rpy.rpyToMatrix(*(float(v) for v in rpy))
    else:
        raise ValueError("a pose needs `quat` [x, y, z, w] or `rpy` [roll, pitch, yaw]")
    return pin.SE3(R, p)


def pose_public(T: pin.SE3) -> dict:
    q = pin.Quaternion(T.rotation)
    return {"pos": [round(float(v), 6) for v in T.translation],
            "quat": [round(float(v), 6) for v in (q.x, q.y, q.z, q.w)],
            "rpy": [round(float(v), 6) for v in pin.rpy.matrixToRpy(T.rotation)]}


def pose_error(a: pin.SE3, b: pin.SE3) -> tuple[float, float]:
    """(position error m, rotation error rad) between two poses."""
    dp = float(np.linalg.norm(a.translation - b.translation))
    dr = float(np.linalg.norm(pin.log3(a.rotation.T @ b.rotation)))
    return dp, dr


class ArmKinematics:
    def __init__(self, extra_limits: Optional[dict] = None, tcp_offset=DEFAULT_TCP_OFFSET):
        """``extra_limits``: {joint name: (lo, hi) rad} narrowing the URDF limits (the config's
        q_min_deg / q_max_deg; RebotArm.setpoint_limits gives URDF and config combined)."""
        self.model = load_robot_model()
        self._ee = get_end_effector_frame_id(self.model)
        self.joint_names = [self.model.names[j] for j in range(1, N + 1)]
        for i, name in enumerate(self.joint_names):
            lo, hi = (extra_limits or {}).get(name, (-math.inf, math.inf))
            jid = self.model.getJointId(name)
            q = self.model.joints[jid].idx_q
            self.model.lowerPositionLimit[q] = max(self.model.lowerPositionLimit[q], lo)
            self.model.upperPositionLimit[q] = min(self.model.upperPositionLimit[q], hi)
        ee = self.model.frames[self._ee]
        self._tcp_placement = pin.SE3(np.eye(3), np.asarray(tcp_offset, dtype=float))
        self.tcp = self.model.addFrame(pin.Frame("tcp", ee.parentJoint, self._ee, ee.placement * self._tcp_placement,
                                                 pin.FrameType.OP_FRAME))
        assert get_arm_joint_count(self.model, self.tcp) == N
        self.lo = np.array(self.model.lowerPositionLimit[:N])
        self.hi = np.array(self.model.upperPositionLimit[:N])
        self._local = threading.local()
        self._lock = threading.Lock()
        # rebot_ctl's end-pose controller IK (max_iter 200, tolerance 1e-4, step 0.5, damping 1e-6)
        # with the teleop's rotation weight 0.02: at a wrist limit, keep the TCP position rather
        # than use other joints for the orientation
        self.ik_params = IKParams(max_iter=200, tolerance=1e-4, step_size=0.5, damping=1e-6, rot_weight=0.02)
        # Acceptance. With rot_weight 0.02 the solver keeps the TCP position and lets the orientation
        # give at the edge of the workspace (an exact pose there may not exist: re-solving with full
        # rotation weight trades the angle into cm of position). So a solve that holds the position
        # but is a few degrees off in orientation is the right nearest answer: accept it up to
        # IK_ROT_TOL, report the angle, and say whether it was exact (within EXACT_ROT).
        self.ik_pos_tol = IK_POS_TOL
        self.ik_rot_tol = IK_ROT_TOL

    # -- TCP offset ------------------------------------------------------------------------------

    @property
    def tcp_offset(self) -> list[float]:
        return [float(v) for v in self._tcp_placement.translation]

    def set_tcp_offset(self, offset) -> None:
        off = np.asarray(offset, dtype=float).reshape(3)
        if not np.all(np.isfinite(off)) or np.linalg.norm(off) > 0.5:
            raise ValueError("tcp offset must be 3 finite numbers within 0.5 m of end_link")
        with self._lock:
            self._tcp_placement = pin.SE3(np.eye(3), off)
            ee = self.model.frames[self._ee]
            self.model.frames[self.tcp].placement = ee.placement * self._tcp_placement
            self._local = threading.local()   # every thread rebuilds its Data

    # -- internals -----------------------------------------------------------------------------

    def _data(self) -> pin.Data:
        d = getattr(self._local, "data", None)
        if d is None:
            d = self._local.data = self.model.createData()
        return d

    def _full(self, q) -> np.ndarray:
        full = np.zeros(self.model.nq)
        full[:N] = np.asarray(q, dtype=float)[:N]
        return full

    # -- queries ---------------------------------------------------------------------------------

    def within_limits(self, q, margin: float = 1e-6) -> bool:
        q = np.asarray(q, dtype=float)[:N]
        return bool(np.all(q >= self.lo - margin) and np.all(q <= self.hi + margin))

    def fk(self, q) -> pin.SE3:
        d = self._data()
        pin.framesForwardKinematics(self.model, d, self._full(q))
        return pin.SE3(d.oMf[self.tcp])

    def frames(self, q) -> list[pin.SE3]:
        """Placements of the 6 joint frames, end_link and tcp (for drawing the arm)."""
        d = self._data()
        pin.framesForwardKinematics(self.model, d, self._full(q))
        out = [pin.SE3(d.oMi[j]) for j in range(1, N + 1)]
        return out + [pin.SE3(d.oMf[self._ee]), pin.SE3(d.oMf[self.tcp])]

    def jacobian(self, q, frame: str = "base") -> np.ndarray:
        """6×6 TCP Jacobian [v; ω]: ``base`` = LOCAL_WORLD_ALIGNED (TCP velocity in base axes),
        ``tcp`` = LOCAL (in the TCP's own axes)."""
        ref = {"base": pin.LOCAL_WORLD_ALIGNED, "tcp": pin.LOCAL}[frame]
        d = self._data()
        J = pin.computeFrameJacobian(self.model, d, self._full(q), self.tcp, ref)
        return np.array(J[:, :N])

    @staticmethod
    def manipulability(J: np.ndarray) -> dict:
        """Yoshikawa w = sqrt(det(J Jᵀ)) (= product of singular values), its translational part,
        the singular values and the translational ellipsoid's axes."""
        s = np.linalg.svd(J, compute_uv=False)
        U, sl, _ = np.linalg.svd(J[:3], full_matrices=False)
        return {"w": float(np.prod(s)), "w_lin": float(np.prod(sl)), "sigma_min": float(s[-1]),
                "sigma": [float(v) for v in s], "lin_sigma": [float(v) for v in sl],
                "lin_axes": [[float(v) for v in U[:, i]] for i in range(3)]}

    def gravity(self, q) -> np.ndarray:
        return np.array(compute_generalized_gravity(self.model, self._full(q), self._data())[:N])

    def twist_to_qd(self, q, twist, frame: str = "base") -> np.ndarray:
        """Joint velocities for a TCP twist [vx, vy, vz, wx, wy, wz] (damped pseudo-inverse)."""
        J = self.jacobian(q, frame)
        JJt = J @ J.T + (PINV_DAMPING ** 2) * np.eye(6)
        return J.T @ np.linalg.solve(JJt, np.asarray(twist, dtype=float))

    def joints_at_limit(self, q) -> list[str]:
        q = np.asarray(q, float)[:N]
        return [n for n, v, lo, hi in zip(self.joint_names, q, self.lo, self.hi) if v <= lo + LIMIT_NEAR or v >= hi - LIMIT_NEAR]

    def ik(self, target: pin.SE3, seed, max_iter: Optional[int] = None, rot_tol: Optional[float] = None) -> IKOut:
        """Full-pose IK from ``seed``. ``ok`` if within ``ik_pos_tol`` / ``ik_rot_tol`` and the joint
        limits (``exact`` if also within EXACT_ROT). Always: ``q`` is the closest found, ``nearest`` the
        pose it reaches (a target that is reachable as is: retry with it), and when not ok, ``reason``
        says which way it missed and what to try. ``rot_tol`` (rad) overrides ``ik_rot_tol`` for this call."""
        p = self.ik_params
        params = p if max_iter is None else IKParams(max_iter=max_iter, tolerance=p.tolerance, step_size=p.step_size,
                                                     damping=p.damping, rot_weight=p.rot_weight)
        seed = np.clip(np.asarray(seed, dtype=float)[:N], self.lo, self.hi)
        r = solve_ik(self.model, self._data(), self.tcp, target, seed, params, controlled_joints=N)
        q = np.asarray(r.q, dtype=float)[:N]
        T = self.fk(q)
        dp, dr = pose_error(T, target)
        lim = self.within_limits(q)
        at = self.joints_at_limit(q)
        rtol = self.ik_rot_tol if rot_tol is None else float(rot_tol)
        tol = f"{self.ik_pos_tol * 1e3:g} mm / {math.degrees(rtol):g}°"
        reason = None
        if not lim:
            reason = (f"the closest solution is outside the joint limits ({', '.join(at) or 'a joint'}); "
                      f"`nearest` is where it would put the TCP")
        elif dp > self.ik_pos_tol:
            reason = (f"out of reach: the closest solution is {dp * 1e3:.1f} mm / {math.degrees(dr):.1f}° away"
                      + (f", with {', '.join(at)} at a limit" if at else "")
                      + f" (accepts {tol}). `nearest` is reachable as is: retry with it, a pose nearer the arm, "
                        "or a different orientation (e.g. pitch), or move the base")
        elif dr > rtol:
            reason = (f"the position is reachable but the orientation only to within {math.degrees(dr):.1f}° "
                      f"(accepts {tol})" + (f"; {', '.join(at)} at a limit" if at else "")
                      + ". `nearest` keeps the position with the reachable orientation: retry with it, or "
                        "relax the orientation (e.g. pitch)")
        ok = reason is None
        return IKOut(ok, q, dp * 1e3, math.degrees(dr), lim, int(r.iterations), reason,
                     exact=ok and dr <= EXACT_ROT, nearest=pose_public(T), at_limit=at)
