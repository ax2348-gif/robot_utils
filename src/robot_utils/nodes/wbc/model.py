"""The WBC's kinematic model: the description node's robot with a planar base (pinocchio).

``GET /model/urdf?base=planar`` (or the same URDF assembled locally, or a file) gives ``odom -> base_x
(prismatic x) -> base_y (prismatic y) -> base_yaw (continuous) -> base_link -> ...``. Every joint that is
neither a base joint nor an end effector's arm joint (the fingers) is locked at 0 in a reduced model, so the
tangent space is ``[x, y, yaw, <arm joints>...]`` (nv = 3 + 6 for one arm). Each end effector gets a ``<name>_tcp``
frame on its link at the TCP offset.

World = the planar root ``odom`` = the estimator's world frame (z = 0 at the floor, like ``base_link``).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import pinocchio as pin

BASE_JOINTS = ("base_x", "base_y", "base_yaw")
LIMIT_MARGIN = math.radians(2.0)   # rad kept from every arm joint limit


@dataclass(frozen=True)
class EESpec:
    """One end effector: its arm's joints (in order, as the arm node numbers them), the link the TCP hangs
    from, the TCP offset in that link, the arm's mount link (arm-only manipulability, facing)."""
    name: str = "arm"
    link: str = "arm_end_link"
    tcp: tuple = (-0.15, 0.0, 0.0)          # m in `link`: = arm.kin.DEFAULT_TCP_OFFSET (the Quest teleop's TCP)
    joints: tuple = tuple(f"arm_joint{i}" for i in range(1, 7))
    mount: str = "arm_base_link"
    arm_limits: Optional[dict] = field(default=None, compare=False)   # {joint: (lo, hi)} narrowing the URDF's


def arm_config_limits() -> Optional[dict]:
    """The arm's own joint limits (rebot_ctl's hardware config, as the arm node enforces them), keyed by the
    model's joint names; None if rebot_ctl is not importable."""
    try:
        from reBotArm_control_py.actuator.rebotarm import joint_limits, load_cfg
        from ..arm.config import DEFAULT_PROFILE, PROFILES
        lim = joint_limits(load_cfg(PROFILES[DEFAULT_PROFILE]))
    except Exception:  # noqa: BLE001 - optional narrowing
        return None
    return {f"arm_{k}": v for k, v in lim.items() if k.startswith("joint")}


def fetch_urdf(source: str) -> tuple[str, str]:
    """(URDF text, where it came from): ``local`` assembles the shipped description here; a path to a
    ``.urdf`` file reads it; anything else is a description node (instance name or URL)."""
    if source == "local":
        import tempfile
        from ..description.model import Options
        from ..description.node import DescriptionNode
        urdf, v = DescriptionNode(tempfile.mkdtemp(prefix="wbc-desc-"), None).model_urdf(Options(base="planar"))
        return urdf, f"local description (model {v})"
    if source.endswith(".urdf"):
        with open(source, encoding="utf-8") as f:
            return f.read(), f"file {source}"
    from ..common.client import NodeClient
    with NodeClient(source, timeout=10.0) as c:
        r = c.http.get("/model/urdf", params={"base": "planar"})
        if r.status_code != 200:
            raise RuntimeError(f"{source}: GET /model/urdf: {r.status_code} {r.text[:200]}")
        return r.text, f"description node {source} (model {r.headers.get('x-model-version')})"


class WbcModel:
    def __init__(self, urdf: str, ees: tuple = (EESpec(),), margin: float = LIMIT_MARGIN):
        full = pin.buildModelFromXML(urdf)
        for j in BASE_JOINTS:
            if not full.existJointName(j):
                raise ValueError(f"the model has no {j} joint: fetch it with base=planar")
        keep = set(BASE_JOINTS) | {j for e in ees for j in e.joints}
        for e in ees:
            for j in e.joints:
                if not full.existJointName(j):
                    raise ValueError(f"end effector {e.name!r}: no joint {j!r} in the model")
            if not full.existFrame(e.link):
                raise ValueError(f"end effector {e.name!r}: no link {e.link!r} in the model")
        lock = [full.getJointId(full.names[i]) for i in range(1, full.njoints) if full.names[i] not in keep]
        self.model = pin.buildReducedModel(full, lock, pin.neutral(full)) if lock else full
        m = self.model
        self.data = m.createData()
        self.ees = {e.name: e for e in ees}
        self.nv, self.nq = m.nv, m.nq
        jx, jy, jw = (m.joints[m.getJointId(n)] for n in BASE_JOINTS)
        self.base_v = np.array([jx.idx_v, jy.idx_v, jw.idx_v])
        self.base_q = (jx.idx_q, jy.idx_q, jw.idx_q)           # yaw: cos, sin at idx_q, idx_q + 1
        self.arm_v: dict[str, np.ndarray] = {}
        self.arm_q: dict[str, np.ndarray] = {}
        self.tcp: dict[str, int] = {}
        self.mount: dict[str, int] = {}
        self.shoulder: dict[str, int] = {}                     # joint id of the arm's first joint (its axis point)
        for e in ees:
            js = [m.joints[m.getJointId(j)] for j in e.joints]
            self.arm_v[e.name] = np.array([j.idx_v for j in js])
            self.arm_q[e.name] = np.array([j.idx_q for j in js])
            self.shoulder[e.name] = m.getJointId(e.joints[0])
            lim = e.arm_limits or {}
            for name in e.joints:
                jq = m.joints[m.getJointId(name)].idx_q
                lo, hi = lim.get(name, (-math.inf, math.inf))
                m.lowerPositionLimit[jq] = max(m.lowerPositionLimit[jq], lo) + margin
                m.upperPositionLimit[jq] = min(m.upperPositionLimit[jq], hi) - margin
            parent = m.frames[m.getFrameId(e.link)]
            place = parent.placement * pin.SE3(np.eye(3), np.asarray(e.tcp, float))
            self.tcp[e.name] = m.addFrame(pin.Frame(f"{e.name}_tcp", parent.parentJoint, m.getFrameId(e.link),
                                                    place, pin.FrameType.OP_FRAME))
            self.mount[e.name] = m.getFrameId(e.mount) if m.existFrame(e.mount) else 0
        self.data = m.createData()
        # the arm mount's x axis in base_link: "facing" a point turns the base so it lies along it
        q0 = self.config((0.0, 0.0, 0.0), {k: np.zeros(len(v)) for k, v in self.arm_q.items()})
        pin.framesForwardKinematics(m, self.data, q0)
        self.mount_yaw = {k: math.atan2(self.data.oMf[f].rotation[1, 0], self.data.oMf[f].rotation[0, 0])
                          for k, f in self.mount.items()}
        self.mount_xy = {k: np.array(self.data.oMf[f].translation[:2]) for k, f in self.mount.items()}
        self.reach = {k: self._nominal_reach(k) for k in self.ees}

    def _nominal_reach(self, ee: str, n: int = 2000) -> float:
        """Shoulder (the first joint's axis point) to TCP, the largest over random configurations within the
        joint limits (deterministic seed): the arm's nominal reach."""
        lo, hi = self.arm_limits(ee)
        rng = np.random.default_rng(0)
        best = 0.0
        for _ in range(n):
            q = self.config((0.0, 0.0, 0.0), {ee: lo + rng.random(len(lo)) * (hi - lo)})
            self.update(q)
            best = max(best, float(np.linalg.norm(self.data.oMf[self.tcp[ee]].translation
                                                  - self.data.oMi[self.shoulder[ee]].translation)))
        return best

    # -- configuration ---------------------------------------------------------------------------------

    def arm_limits(self, ee: str) -> tuple[np.ndarray, np.ndarray]:
        iq = self.arm_q[ee]
        return np.array(self.model.lowerPositionLimit[iq]), np.array(self.model.upperPositionLimit[iq])

    def config(self, base, arms: dict) -> np.ndarray:
        """q from the base pose (x, y, yaw) and the arms' joint vectors."""
        q = pin.neutral(self.model)
        ix, iy, iw = self.base_q
        q[ix], q[iy] = base[0], base[1]
        q[iw], q[iw + 1] = math.cos(base[2]), math.sin(base[2])
        for k, v in arms.items():
            q[self.arm_q[k]] = v
        return q

    def set_tcp(self, ee: str, offset) -> None:
        e = self.ees[ee]
        m = self.model
        parent = m.frames[m.getFrameId(e.link)]
        m.frames[self.tcp[ee]].placement = parent.placement * pin.SE3(np.eye(3), np.asarray(offset, float))
        self.ees[ee] = EESpec(e.name, e.link, tuple(float(v) for v in offset), e.joints, e.mount, e.arm_limits)

    # -- kinematics (one Data: call from one thread) ----------------------------------------------------

    def update(self, q: np.ndarray) -> None:
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

    def fk(self, q: np.ndarray, ee: str) -> pin.SE3:
        self.update(q)
        return pin.SE3(self.data.oMf[self.tcp[ee]])

    def jacobian(self, q: np.ndarray, ee: str) -> np.ndarray:
        """6 x nv TCP Jacobian [v; w] in the world axes at the TCP (LOCAL_WORLD_ALIGNED); call after update()."""
        pin.computeJointJacobians(self.model, self.data, q)
        return pin.getFrameJacobian(self.model, self.data, self.tcp[ee], pin.LOCAL_WORLD_ALIGNED)

    def shoulder_point(self, q: np.ndarray, ee: str) -> tuple[np.ndarray, np.ndarray]:
        """(position, 3 x nv position Jacobian, world axes) of the arm's shoulder; call after jacobian() /
        computeJointJacobians at q. Only the base columns are non-zero."""
        j = self.shoulder[ee]
        J = pin.getJointJacobian(self.model, self.data, j, pin.LOCAL_WORLD_ALIGNED)[:3]
        return np.array(self.data.oMi[j].translation), np.array(J)

    def arm_manipulability(self, q: np.ndarray, ee: str) -> float:
        """Yoshikawa sqrt(det(J J^T)) of the arm alone (its joints' columns; the base excluded)."""
        self.update(q)
        J = self.jacobian(q, ee)[:, self.arm_v[ee]]
        return float(math.sqrt(max(np.linalg.det(J @ J.T), 0.0)))

    def manipulability_gradient(self, q: np.ndarray, ee: str, h: float = 1e-5) -> tuple[float, np.ndarray]:
        """(w, dw/dq_arm) by forward differences (7 arm Jacobians)."""
        iq = self.arm_q[ee]
        w0 = self.arm_manipulability(q, ee)
        g = np.zeros(len(iq))
        for i, k in enumerate(iq):
            qp = q.copy()
            qp[k] += h
            g[i] = (self.arm_manipulability(qp, ee) - w0) / h
        return w0, g
