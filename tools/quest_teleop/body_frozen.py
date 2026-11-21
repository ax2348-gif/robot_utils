"""The body-frozen scheme (quest_env DEMO.md, "Experimental walking scheme: body-frozen"), robot side, and the
whole-body command derived from one headset frame. Pure: no sockets, the clock is passed in.

Per engagement of the arm's hand:

- **Operator frame**, captured from the raw head pose of the engaging frame: origin = head position, X = the
  gravity-aligned head yaw, Z = up (``frames.OperatorFrame``). The controller anchor and every later controller
  pose are expressed in it: ``dp = p - p0``, ``dR = R R0^T``.
- **Robot frame**, captured from the WBC stream at the same moment: the upright platform frame (base x, y and
  yaw, world) and the **measured** EE pose (``arm_meas_*``, world) expressed in it.
- **Target** = that frozen robot frame applied to ``(anchor.p + dp, dR anchor.R)``, in the world frame. At
  engagement it is exactly the measured EE pose. Platform motion does not move it; operator travel does. The frame's
  origin cancels (``target.p = ee.p + R_F dp``), so the platform frame and a shoulder frame with the same yaw give
  the same target.

Engagement rules: an engagement starts only after this consumer has seen the hand released (valid, not engaged)
since the last discontinuity (start, reconnect, session or revision change, gap, invalid frame, estimator reset,
missing robot state at capture). A reconnect mid-engagement therefore waits for a release and a new engagement;
an existing nonzero delta is never applied to a newly captured anchor.

Released (or never engaged): the arm holds where it was in the **base** frame (the last target, or the measured EE
at start), so a moving platform carries it (DEMO.md: release holds arm joints).

Base: the left stick (the intent's ``base_velocity``: forward/left m/s, yaw rad/s, 18 % dead zone applied natively)
-> mode ``commanded`` with a ttl while it is deflected. Otherwise, while engaged, mode ``hint`` with a ``heading``
target from the head's gravity-aligned yaw (relative to the engagement: base_yaw0 + (head_yaw - head_yaw0)) and,
optionally, a soft ``pose`` target from the head's displacement in the engagement's operator frame. Released and no
stick: ``locked``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from . import frames as F

NATIVE_MAX_SPEED = 0.45   # native full-scale stick translation (m/s)
NATIVE_MAX_OMEGA = 0.8    # native full-scale stick yaw (rad/s)


@dataclass
class RobotState:
    """The newest WBC stream row the client uses (world frame)."""
    t_recv: float                 # our clock
    ee_p: np.ndarray              # measured EE position
    ee_r: np.ndarray              # measured EE orientation
    base_x: float
    base_y: float
    base_yaw: float
    est_reset: float = 0.0

    @classmethod
    def from_row(cls, row: dict, t_recv: float, ee: str = "arm") -> Optional["RobotState"]:
        try:
            p = np.array([row[f"{ee}_meas_{k}"] for k in ("x", "y", "z")], float)
            q = np.array([row[f"{ee}_meas_q{k}"] for k in ("x", "y", "z", "w")], float)
            bx, by, byaw = (float(row[k]) for k in ("base_x", "base_y", "base_yaw"))
        except KeyError:
            return None
        reset = float(row.get("est_reset", 0.0))
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(q)) and np.linalg.norm(q) > 0.5
                and all(math.isfinite(v) for v in (bx, by, byaw))):
            return None
        return cls(t_recv, p, F.matrix_from_quat(q), bx, by, byaw, reset)

    @property
    def base(self) -> F.PlanarFrame:
        return F.PlanarFrame(self.base_x, self.base_y, self.base_yaw)


@dataclass
class Engagement:
    id: int
    operator: F.OperatorFrame
    hand_anchor: tuple            # controller pose in the operator frame at engagement
    robot: F.PlanarFrame          # frozen platform frame
    ee_anchor: tuple              # measured EE in the frozen platform frame
    est_reset: float
    head_yaw0: float
    head_p0: np.ndarray           # LOCAL
    target: Optional[tuple] = None   # newest world target
    hint_base: Optional[F.PlanarFrame] = None   # platform frame the head hints refer to (re-referenced after the stick)


def body_frozen_target(eng: Engagement, hand_p_local, hand_r_local):
    """World target for the current controller pose (DEMO.md body-frozen formula)."""
    current = eng.operator.pose(hand_p_local, hand_r_local)
    dp, dr = F.delta(eng.hand_anchor, current)
    return eng.robot.to_world(*F.apply_delta(eng.ee_anchor, dp, dr))


def capture(eng_id: int, head_p, head_r, hand_p, hand_r, robot: RobotState) -> Optional[Engagement]:
    op = F.OperatorFrame.from_head(head_p, head_r)
    if op is None:
        return None
    frame = robot.base
    return Engagement(eng_id, op, op.pose(hand_p, hand_r), frame, frame.to_local(robot.ee_p, robot.ee_r),
                      robot.est_reset, op.yaw, np.asarray(head_p, float).copy())


@dataclass
class Config:
    hand: str = "right"
    ee: str = "arm"
    robot_stale_s: float = 0.3        # WBC stream row older than this: no new engagement
    heading_hint: bool = True
    pose_hint: bool = False
    heading_weight: float = 0.5
    heading_tol: float = 0.05
    pose_weight: float = 0.5
    pose_tol: float = 0.10
    max_speed: float = 0.3            # stick full scale (m/s), scaled from native 0.45
    max_omega: float = 0.8            # stick full scale (rad/s)
    velocity_ttl: float = 0.3
    gripper: bool = True


def head_pose(frame: dict):
    head = frame.get("head") or {}
    if not (head.get("position_tracked") and head.get("orientation_tracked")):
        return None
    return F.pose_from_matrix16(head.get("pose"))


def hand_pose(frame: dict, hand: str):
    h = (frame.get("hands") or {}).get(hand) or {}
    if not h.get("active"):
        return None
    return F.pose_from_matrix16(h.get("pose"))


@dataclass
class Status:
    state: str = "waiting"        # waiting | released | needs_release | engaged | stale
    reason: str = ""
    engagements: int = 0
    refused: dict = field(default_factory=dict)


class TeleopCore:
    """Turns headset frames (already gated) and WBC rows into the next datagram body.

    ``on_frame(frame, verdict, robot)`` updates the engagement state for every accepted frame, in order.
    ``command(now, input_fresh, robot)`` returns the ``ees``/``base`` body of the next target datagram, the string
    ``"alive"`` when input is fine but no robot state has arrived yet (nothing to hold), or None: stop sending.
    """

    def __init__(self, cfg: Config = Config()):
        self.cfg = cfg
        self.eng: Optional[Engagement] = None
        self.needs_release = True
        self.hold_base: Optional[tuple] = None    # (p, R) in the base frame
        self.last_target: Optional[tuple] = None  # world
        self.frame: Optional[dict] = None         # newest valid frame
        self.valid = False
        self.stick_active = False
        self.status = Status()

    # -- engagement ------------------------------------------------------------------------------

    def drop(self, reason: str) -> None:
        """End the engagement (if any) and require a release before the next one."""
        self._release_to_hold(None)
        self.needs_release = True
        if reason:
            self.status.reason = reason

    def _release_to_hold(self, robot: Optional[RobotState]) -> None:
        if self.eng is not None and self.eng.target is not None:
            base = robot.base if robot is not None else None
            if base is not None:
                self.hold_base = base.to_local(*self.eng.target)
            else:
                self.hold_base = None      # re-derived from the next robot row
        self.eng = None

    def on_frame(self, frame: dict, verdict, robot: Optional[RobotState], now: float) -> None:
        if not verdict.accept:
            return
        if verdict.discontinuity:
            self.drop(verdict.reason)
        self.valid = verdict.valid
        if not verdict.valid:
            self.frame = None
            return
        self.frame = frame
        ih = frame["control_intent"]["hands"].get(self.cfg.hand) or {}
        if not ih.get("valid"):
            self.drop("hand invalid")
            self.valid = False
            return
        if self.eng is not None and robot is not None and robot.est_reset != self.eng.est_reset:
            self.drop("estimator reset")
            return
        if not ih.get("engaged"):
            if self.eng is not None:
                self._release_to_hold(robot)
            self.needs_release = False
            self.status.state = "released"
            return
        eid = ih.get("engagement_id")
        if self.eng is not None:
            if eid != self.eng.id:            # a release we did not see: treat as a discontinuity
                self.drop("engagement changed")
                return
            hp = hand_pose(frame, self.cfg.hand)
            if hp is None:
                self.drop("hand pose missing")
                return
            self.eng.target = body_frozen_target(self.eng, *hp)
            self.status.state = "engaged"
            return
        if self.needs_release:
            self.status.state = "needs_release"
            return
        # a fresh engagement, seen from its start: capture both anchors now
        why = None
        head, hp = head_pose(frame), hand_pose(frame, self.cfg.hand)
        if robot is None or now - robot.t_recv > self.cfg.robot_stale_s:
            why = "no fresh WBC state"
        elif head is None:
            why = "head not tracked"
        elif hp is None:
            why = "hand pose missing"
        if why is None:
            eng = capture(eid, head[0], head[1], hp[0], hp[1], robot)
            if eng is None:
                why = "head heading undefined"
        if why is not None:
            self.status.refused[why] = self.status.refused.get(why, 0) + 1
            self.drop(f"engagement refused: {why}")
            self.status.state = "needs_release"
            return
        eng.target = (robot.ee_p.copy(), robot.ee_r.copy())
        self.eng = eng
        self.status.engagements += 1
        self.status.state = "engaged"
        self.status.reason = ""

    def stale(self, reason: str = "stale") -> None:
        """Input went stale (no frame for the freshness timeout): drop everything."""
        self.drop(reason)
        self.valid = False
        self.frame = None
        self.status.state = "stale"

    # -- command ---------------------------------------------------------------------------------

    def command(self, now: float, input_fresh: bool, robot: Optional[RobotState]):
        if not input_fresh:
            if self.frame is not None or self.eng is not None:
                self.stale()
            return None
        if not self.valid or self.frame is None:
            return None
        cfg = self.cfg
        ee: dict
        if self.eng is not None and self.eng.target is not None:
            p, r = self.eng.target
            ee = {"frame": "world", "pos": _l(p), "quat": _l(F.quat_from_matrix(r))}
            self.last_target = (p, r)
        else:
            if self.hold_base is None:
                if robot is None:
                    return "alive"
                self.hold_base = robot.base.to_local(robot.ee_p, robot.ee_r)
            p, r = self.hold_base
            ee = {"frame": "base", "pos": _l(p), "quat": _l(F.quat_from_matrix(r))}
        intent = self.frame["control_intent"]
        ih = intent["hands"].get(cfg.hand) or {}
        if cfg.gripper and ih.get("valid") and _finite(ih.get("gripper")):
            ee["gripper"] = min(1.0, max(0.0, float(ih["gripper"])))
        return {"ees": {cfg.ee: ee}, "base": self._base(intent, robot)}

    def _base(self, intent: dict, robot: Optional[RobotState]) -> dict:
        cfg = self.cfg
        v = intent.get("base_velocity") or [0, 0, 0]
        v = [float(x) if _finite(x) else 0.0 for x in (list(v) + [0, 0, 0])[:3]]
        if any(abs(x) > 1e-9 for x in v):
            self.stick_active = True
            s = cfg.max_speed / NATIVE_MAX_SPEED
            w = cfg.max_omega / NATIVE_MAX_OMEGA
            return {"mode": "commanded", "velocity": {"vx": v[0] * s, "vy": v[1] * s, "omega": v[2] * w,
                                                      "frame": "base", "ttl": cfg.velocity_ttl}}
        if self.stick_active:
            self.stick_active = False
            # the stick moved the platform on purpose: re-reference the head hints so they do not undo it
            if self.eng is not None and robot is not None:
                head = head_pose(self.frame)
                yaw = F.head_yaw(head[1]) if head is not None else None
                if yaw is not None:
                    self.eng.head_yaw0, self.eng.head_p0 = yaw, head[0].copy()
                    self.eng.hint_base = robot.base
        if self.eng is None or not (cfg.heading_hint or cfg.pose_hint):
            return {"mode": "locked"} if self.eng is None else {"mode": "free"}
        target = {}
        head = head_pose(self.frame)
        ref = self.eng.hint_base or self.eng.robot
        if head is not None:
            yaw = F.head_yaw(head[1])
            if cfg.heading_hint and yaw is not None:
                target["heading"] = {"yaw": F.wrap(ref.yaw + F.wrap(yaw - self.eng.head_yaw0)), "frame": "world",
                                     "weight": cfg.heading_weight, "tol": cfg.heading_tol}
            if cfg.pose_hint:
                d = F.rot_z(-self.eng.head_yaw0) @ F.C @ (head[0] - self.eng.head_p0)
                xy = ref.p[:2] + ref.r[:2, :2] @ d[:2]
                target["pose"] = {"x": float(xy[0]), "y": float(xy[1]), "frame": "world",
                                  "weight": cfg.pose_weight, "tol": cfg.pose_tol}
        return {"mode": "hint", "target": target} if target else {"mode": "free"}


def _finite(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _l(a) -> list:
    return [float(x) for x in a]
