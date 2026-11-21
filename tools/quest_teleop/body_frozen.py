"""The body-frozen scheme (quest_env DEMO.md, "Experimental walking scheme: body-frozen"), robot side, and the
whole-body command derived from one headset frame. Pure: no sockets, the clock is passed in.

The rule above all: **the robot never moves when the clutch is released and re-engaged**, however the operator
moved in between. Hence the anchors are the WBC's *commanded* state, not the measured one, and every stick or head
contribution restarts at zero on each engagement.

Per engagement of the arm's hand:

- **Operator frame**, captured from the raw head pose of the engaging frame: origin = head position, X = the
  gravity-aligned head yaw, Z = up (``frames.OperatorFrame``). The controller anchor and every later controller
  pose are expressed in it: ``dp = p - p0``, ``dR = R R0^T``.
- **Robot frame**, captured from the WBC stream at the same moment: the upright platform frame (base x, y and
  yaw, world) and the **commanded** EE pose (``arm_cmd_*``, world) expressed in it. DEMO.md says measured because a
  remote consumer normally lacks the command; we have it, and anchoring to the measured pose would move the arm by
  its sag/backlash error at engagement. (Measured ``arm_meas_*`` only if the row's command is not finite.)
- **Target** = that frozen robot frame applied to ``(anchor.p + dp, dR anchor.R)``, in the world frame. At
  engagement it is exactly the current command. Platform motion does not move it; operator travel does. The frame's
  origin cancels (``target.p = ee.p + R_F dp``), so the platform frame and a shoulder frame with the same yaw give
  the same target.

Engagement rules: an engagement starts only after this consumer has seen the hand released (valid, not engaged)
since the last discontinuity (start, reconnect, session or revision change, gap, invalid frame, estimator reset,
missing robot state at capture). A reconnect mid-engagement therefore waits for a release and a new engagement;
an existing nonzero delta is never applied to a newly captured anchor.

Released (or never engaged): ``"ees": {"arm": {"hold": true}}``: the WBC holds the arm's commanded joints, so the
arm rides with the base when the sticks drive it (DEMO.md: release holds arm joints). Not a ``frame: "base"`` pose
target: the WBC converts those to the world once, at their timestamp, which would pin the EE in the room.

Base (the sticks are the intent's ``base_velocity``: left stick forward/left m/s, right stick X yaw rad/s, 18 %
dead zone and full scale 0.45 m/s / 0.8 rad/s applied natively):

- **Released** (the arm's hand not engaged): no hint; only the sticks move the base: mode ``commanded`` with
  ``velocity`` in the base frame and a short ttl; sticks idle: ``locked`` (the base decelerates to a stop and holds).
- **Engaged**: mode ``hint`` with a ``pose`` and a ``heading`` target = the base pose at engagement (from the stream)
  + the head's motion since engagement (displacement and gravity-aligned yaw change, mapped through the frozen frames
  as body-frozen does for the arm) + the stick **offset** ``(x, y, yaw)`` (world). The offset starts at zero on every
  engagement and integrates the sticks: x/y along the robot base frame at integration time (stick forward = robot
  forward), yaw from the right stick. Standing still and driving with the sticks, walking with the robot mirroring,
  or both. Whatever the sticks achieved is in the base pose the next engagement captures.

The gripper follows the trigger whenever the hand is valid, independent of the clutch.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from . import frames as F



@dataclass
class RobotState:
    """The newest WBC stream row the client uses (world frame)."""
    t_recv: float                 # our clock
    ee_p: np.ndarray              # the anchor EE position: commanded (measured if the command is not finite)
    ee_r: np.ndarray              # the anchor EE orientation
    base_x: float
    base_y: float
    base_yaw: float
    est_reset: float = 0.0
    ee_source: str = "cmd"        # "cmd" | "meas"

    @staticmethod
    def _pose(row: dict, prefix: str):
        try:
            p = np.array([row[f"{prefix}_{k}"] for k in ("x", "y", "z")], float)
            q = np.array([row[f"{prefix}_q{k}"] for k in ("x", "y", "z", "w")], float)
        except KeyError:
            return None
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(q)) and np.linalg.norm(q) > 0.5):
            return None
        return p, F.matrix_from_quat(q)

    @classmethod
    def from_row(cls, row: dict, t_recv: float, ee: str = "arm") -> Optional["RobotState"]:
        source = "cmd"
        pose = cls._pose(row, f"{ee}_cmd")
        if pose is None:
            source, pose = "meas", cls._pose(row, f"{ee}_meas")
        try:
            bx, by, byaw = (float(row[k]) for k in ("base_x", "base_y", "base_yaw"))
        except KeyError:
            return None
        if pose is None or not all(math.isfinite(v) for v in (bx, by, byaw)):
            return None
        return cls(t_recv, pose[0], pose[1], bx, by, byaw, float(row.get("est_reset", 0.0)), source)

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
    head_d: np.ndarray = field(default_factory=lambda: np.zeros(3))    # last head displacement (engagement frame)
    head_dyaw: float = 0.0                                             # last head yaw change


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
    head_walk: bool = True            # head displacement and yaw drive the base hint while engaged
    heading_weight: float = 0.5
    heading_tol: float = 0.05
    pose_weight: float = 0.5
    pose_tol: float = 0.10
    velocity_ttl: float = 0.2         # released: commanded stick velocity lapses after this
    max_dt: float = 0.05              # offset integration: longest step between two frames (s)
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
    or None: stop sending.
    """

    def __init__(self, cfg: Config = Config()):
        self.cfg = cfg
        self.eng: Optional[Engagement] = None
        self.needs_release = True
        self.last_target: Optional[tuple] = None  # world
        self.frame: Optional[dict] = None         # newest valid frame
        self.valid = False
        self.offset = np.zeros(3)                 # stick offset (x, y, yaw), world; zero at every engagement
        self.hint: Optional[dict] = None          # the newest base hint target sent (status)
        self._last_ns: Optional[int] = None
        self.status = Status()

    # -- engagement ------------------------------------------------------------------------------

    def drop(self, reason: str) -> None:
        """End the engagement (if any) and require a release before the next one."""
        self._last_ns = None
        self._release_to_hold(None)
        self.needs_release = True
        if reason:
            self.status.reason = reason

    def _release_to_hold(self, robot: Optional[RobotState]) -> None:
        """End the engagement: the arm holds (``hold: true``) from the next datagram on."""
        self.eng = None

    def on_frame(self, frame: dict, verdict, robot: Optional[RobotState], now: float) -> None:
        if not verdict.accept:
            return
        self._engagement(frame, verdict, robot, now)
        if self.valid and self.frame is not None:
            self._sticks(frame, robot)

    def _sticks(self, frame: dict, robot: Optional[RobotState]) -> None:
        ns = frame.get("sample_time_ns")
        dt = 0.0
        if isinstance(ns, int) and self._last_ns is not None:
            dt = min(max(0.0, (ns - self._last_ns) * 1e-9), self.cfg.max_dt)
        self._last_ns = ns if isinstance(ns, int) else None
        if self.eng is None or dt <= 0:
            return
        v = stick_velocity(frame["control_intent"])
        yaw = robot.base_yaw if robot is not None else self.eng.robot.yaw
        self.offset[:2] += F.rot_z(yaw)[:2, :2] @ np.array(v[:2]) * dt
        self.offset[2] += v[2] * dt

    def _engagement(self, frame: dict, verdict, robot: Optional[RobotState], now: float) -> None:
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
        self.offset = np.zeros(3)
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
            reset = self.eng.est_reset            # the estimator epoch the anchor was captured in
        else:
            ee = {"hold": True}
            reset = robot.est_reset if robot is not None else None
        intent = self.frame["control_intent"]
        ih = intent["hands"].get(cfg.hand) or {}
        if cfg.gripper and ih.get("valid") and _finite(ih.get("gripper")):
            ee["gripper"] = min(1.0, max(0.0, float(ih["gripper"])))
        out = {"ees": {cfg.ee: ee}, "base": self._base(intent)}
        if reset is not None:
            out["est_reset"] = int(reset) if float(reset).is_integer() else reset
        return out

    def _base(self, intent: dict) -> dict:
        cfg = self.cfg
        if self.eng is None:
            self.hint = None
            v = stick_velocity(intent)
            if any(abs(x) > 1e-9 for x in v):
                return {"mode": "commanded", "velocity": {"vx": v[0], "vy": v[1], "omega": v[2], "frame": "base",
                                                          "ttl": cfg.velocity_ttl}}
            return {"mode": "locked"}
        eng = self.eng
        if cfg.head_walk:
            head = head_pose(self.frame)
            yaw = F.head_yaw(head[1]) if head is not None else None
            if yaw is not None:         # head lost: keep its last contribution
                eng.head_d = F.rot_z(-eng.head_yaw0) @ F.C @ (head[0] - eng.head_p0)
                eng.head_dyaw = F.wrap(yaw - eng.head_yaw0)
        off = self.offset
        xy = eng.robot.p[:2] + eng.robot.r[:2, :2] @ eng.head_d[:2] + off[:2]
        hyaw = F.wrap(eng.robot.yaw + eng.head_dyaw + off[2])
        target = {"pose": {"x": float(xy[0]), "y": float(xy[1]), "frame": "world",
                           "weight": cfg.pose_weight, "tol": cfg.pose_tol},
                  "heading": {"yaw": hyaw, "frame": "world", "weight": cfg.heading_weight, "tol": cfg.heading_tol}}
        self.hint = target
        return {"mode": "hint", "target": target}


def stick_velocity(intent: dict) -> list:
    """The intent's ``base_velocity`` (forward, left m/s; yaw rad/s), non-finite parts as 0."""
    v = intent.get("base_velocity") or [0, 0, 0]
    return [float(x) if _finite(x) else 0.0 for x in (list(v) + [0, 0, 0])[:3]]


def _finite(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _l(a) -> list:
    return [float(x) for x in a]
