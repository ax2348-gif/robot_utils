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
  the same target. That is the formula with head walking off.

**Two links (head walking on).** The operator chain is world → head (worn on the neck: a torso proxy; position and
gravity-aligned yaw) → hand, and the robot chain mirrors it: world → base reference → EE.

- **Link 1**, base reference = the base at engagement + the head's motion since engagement (+ the stick offset):
  the base hint below.
- **Link 2**, the EE in the base reference = the EE at engagement (``B0⁻¹ ∘ command``) + the hand's delta **in the
  current head frame** since engagement (``ee_rel``). The head frame takes from the head only what the base can
  follow: horizontal position and gravity-aligned yaw. Its height stays at the head's height at engagement, so a
  crouch lowers the hand and the arm with it.
- **Target** = link 1 ∘ link 2.

The invariant: while the hand keeps its pose relative to the head, the EE target keeps its pose relative to the base
reference, for any walking or turning. Vertical motion is not absorbed: the base cannot follow it. The frozen-frame formula breaks it on turns: the hand sweeps an arc about the
head, the base turns about its own origin, and the EE drifts sideways by about ``|dyaw| * |ee_in_base - hand_in_head|``
in the plane. Clutch invariance is unchanged: both links' deltas start at zero, so engaging gives exactly the
command. A head lost mid-engagement freezes both links (link 2 reads the hand in the last tracked head frame). When
the head comes back, both re-anchor where they are, as at an engagement, so the robot does not move.

Engagement rules: an engagement starts only after this consumer has seen the hand released (valid, not engaged)
since the last discontinuity (start, reconnect, session or revision change, gap, invalid frame, estimator reset,
missing robot state at capture). A reconnect mid-engagement therefore waits for a release and a new engagement;
an existing nonzero delta is never applied to a newly captured anchor.

Released (or never engaged): the arm rides with the platform (DEMO.md: release holds arm joints). The WBC takes
world targets only, so the client computes the riding target: at release it stores the **commanded EE pose at
release** relative to the base (``arm_cmd_*`` and the base pose of the same stream row; at start, the current
command), and every frame sends ``world target = current base pose ∘ that relative pose`` with a twist feedforward =
the base's rigid-body velocity at the EE (from the stream's commanded base twist ``base_vx, base_vy, base_w``, read
as base-frame components), which cancels the ~20 ms lag. Not the last target: the WBC may have been limiting the arm
short of it.

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

Velocity feedforward while engaged (so the robot keeps up with a walking operator instead of trailing by v / k):
the EE target carries ``twist`` = the target's own velocity (the hand's motion mapped through the frozen frames,
linear and angular) and the base hint carries the hint target's velocity (``vx``, ``vy``, ``omega``: the head's
motion mapped as ``head_d`` is, plus the stick offset's rate, exact). Both are least-squares slopes over the frames
of the last ``ff_window`` (60 ms) on ``sample_time_ns``: exact for a steady motion, about ``ff_window / 2`` (30 ms)
behind a change of speed; clamped; zero while the head is lost (and its history restarts). Every engagement starts
them at zero: only frames of this engagement count, and they ramp in with the window's fill (re-engaging never moves
the robot). Released: unchanged (the riding target's twist is the base's own).

The gripper follows the trigger whenever the hand is valid, independent of the clutch.

Armed latch (``arming.py``; the client sends targets only while armed): ``on_arm`` / ``on_disarm`` drop the
engagement (the grip needs its release first, as always) and the riding pose (``hold_base``: captured again from the
current command, never a stale one); after arming the sticks give no base velocity until both are seen neutral. The
gripper (the owner's rebot_ctl behaviour): no gripper command before arming (after Start, after any disarm or disable:
the gripper stays where it is); from the arming frame on, ``1 - trigger`` in every target (a released trigger opens).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from . import frames as F
from .arming import sticks_neutral

UP_LOCAL = F.C.T @ np.array([0.0, 0.0, 1.0])     # the up axis in OpenXR LOCAL (+Y)



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
    base_twist: tuple = (0.0, 0.0, 0.0)   # commanded base twist (vx, vy in the base frame, w)

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
        tw = tuple(float(row.get(k, 0.0)) for k in ("base_vx", "base_vy", "base_w"))
        tw = tw if all(math.isfinite(v) for v in tw) else (0.0, 0.0, 0.0)
        return cls(t_recv, pose[0], pose[1], bx, by, byaw, float(row.get("est_reset", 0.0)), source, tw)

    def twist_at(self, p_world) -> list:
        """The base's rigid-body velocity at a world point: ``[vx, vy, vz, wx, wy, wz]``, world frame."""
        vx, vy, w = self.base_twist
        v = F.rot_z(self.base_yaw)[:2, :2] @ [vx, vy]
        rx, ry = p_world[0] - self.base_x, p_world[1] - self.base_y
        return [float(v[0] - w * ry), float(v[1] + w * rx), 0.0, 0.0, 0.0, float(w)]

    @property
    def base(self) -> F.PlanarFrame:
        return F.PlanarFrame(self.base_x, self.base_y, self.base_yaw)


class VelocityFit:
    """The velocity of a sampled signal: the least-squares slope over the samples of the last ``window`` seconds,
    scaled by how full the window is (0 with fewer than 3 samples), clamped to ``max_norm`` per block of 3."""

    def __init__(self, window: float = 0.06, max_norm: Sequence = (1.0,)):
        self.window = window
        self.max_norm = tuple(max_norm)
        self.t: list = []
        self.x: list = []

    def reset(self) -> None:
        self.t, self.x = [], []

    def push(self, t: float, x) -> None:
        if self.t and t <= self.t[-1]:
            return
        self.t.append(float(t))
        self.x.append(np.asarray(x, float))
        while self.t and self.t[0] < t - self.window:
            self.t.pop(0)
            self.x.pop(0)

    def velocity(self, dim: int) -> np.ndarray:
        if len(self.t) < 3:
            return np.zeros(dim)
        t = np.asarray(self.t)
        x = np.asarray(self.x)
        tc = t - t.mean()
        span = t[-1] - t[0]
        if span <= 1e-6:
            return np.zeros(dim)
        v = (tc @ (x - x.mean(axis=0))) / float(tc @ tc)
        v = v * min(1.0, span / (0.6 * self.window))          # ramps in as the window fills
        for k, cap in enumerate(self.max_norm):
            blk = v[3 * k:3 * k + 3]
            n = float(np.linalg.norm(blk))
            if n > cap:
                blk *= cap / n
        return v


def rotvec(r) -> np.ndarray:
    """The rotation vector (axis * angle) of a rotation matrix."""
    r = np.asarray(r, float)
    a = F.rotation_angle(r)
    w = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    if a < 1e-9:
        return 0.5 * w
    if a > math.pi - 1e-6:                                    # 180 degrees: not reached between frames
        return np.zeros(3)
    return w * (a / (2.0 * math.sin(a)))


@dataclass
class Engagement:
    id: int
    operator: F.OperatorFrame
    hand_anchor: tuple            # link 2 anchor: the controller pose in the head frame (operator frame at engagement)
    robot: F.PlanarFrame          # link 1 anchor: the platform frame at engagement (moved by a head re-anchor)
    ee_anchor: tuple              # link 2 anchor: the commanded EE in the link-1 anchor frame
    est_reset: float
    head_yaw0: float
    head_p0: np.ndarray           # LOCAL
    target: Optional[tuple] = None   # newest world target: base reference (with the stick offset) ∘ ee_rel
    target_raw: Optional[tuple] = None   # the same without the stick offset
    ee_rel: Optional[tuple] = None   # link 2: the EE target in the base reference frame
    head_last: Optional[tuple] = None   # the newest tracked head (LOCAL position, yaw): link 2's frame
    rebase_hand: bool = False        # the head was re-anchored: re-anchor link 2 on the next hand pose
    head_d: np.ndarray = field(default_factory=lambda: np.zeros(3))    # last head displacement (engagement frame)
    head_dyaw: float = 0.0                                             # last head yaw change
    head_yaw_unwrapped: float = 0.0                                    # continuous (the velocity fit)
    head_tracked: bool = True                                          # the newest frame had the head
    target_r0: Optional[np.ndarray] = None                             # the first target orientation (fit origin)
    hand_fit: Optional["VelocityFit"] = None                           # the target's velocity
    head_fit: Optional["VelocityFit"] = None                           # the head's (hint) velocity, world
    captured_now: bool = False                                         # captured on the newest frame


def body_frozen_target(eng: Engagement, hand_p_local, hand_r_local):
    """World target for the current controller pose by the DEMO.md body-frozen formula: the hand's delta in the
    operator frame frozen at engagement, applied in the frozen platform frame. ``TeleopCore`` uses it only through
    the same formula with link 1 frozen (``head_walk`` off); with head walking it composes the two links."""
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
    ride_lead: float = 0.01           # s: released, the riding target is placed on the base pose predicted this far
                                      # past the stream row's arrival (plus its age), so it does not lag the base
    feedforward: bool = True          # engaged: the EE twist and the hint velocity (see the module docstring)
    ff_window: float = 0.06           # s, the velocity fit's window (adds about half of it as lag)
    ff_max_speed: float = 1.0         # m/s, EE feedforward clamp
    ff_max_omega: float = 3.0         # rad/s, EE feedforward clamp
    hint_max_speed: float = 0.6       # m/s, the hint velocity clamp (the WBC refuses more than its max_speed)
    hint_max_omega: float = 0.8       # rad/s (the WBC's max_omega)


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
        self.last_target: Optional[tuple] = None  # world
        self.frame: Optional[dict] = None         # newest valid frame
        self.valid = False
        self.hold_base: Optional[tuple] = None    # released: the command at release relative to the base
        self.last_robot: Optional[RobotState] = None
        self.offset = np.zeros(3)                 # stick offset (x, y, yaw), world; zero at every engagement
        self.hint: Optional[dict] = None          # the newest base hint target sent (status)
        self._last_ns: Optional[int] = None
        self._tracked = False
        self.status = Status()
        # the armed latch (README "Arming"): the sticks drive only once seen neutral after arming; the gripper is
        # commanded only while armed. A core that is never armed (tests, tools) starts live.
        self.sticks_live = True
        self.gripper_live = True

    # -- the armed latch -------------------------------------------------------------------------

    def on_arm(self, frame: Optional[dict] = None) -> None:
        """Armed by a Y press (after the arming frame went through ``on_frame``): the grip needs its release first
        (as always), the sticks must be seen neutral; the gripper follows the trigger from this frame on."""
        self.drop("armed")
        self.hold_base = None                     # the riding target is captured from the current command
        self.sticks_live = False
        self.gripper_live = True

    def on_disarm(self, reason: str = "disarmed") -> None:
        """Disarmed: nothing carries over to the next arm (engagement, riding pose, sticks); no gripper command."""
        self.drop(reason)
        self.hold_base = None
        self.sticks_live = False
        self.gripper_live = False

    def _gates(self, frame: dict) -> None:
        """After arming: the sticks seen neutral (both, inside the native dead zone)."""
        if not self.sticks_live and sticks_neutral(frame):
            self.sticks_live = True

    def stick_v(self, intent: dict) -> list:
        """The sticks' base velocity, zero until they were seen neutral after arming."""
        return stick_velocity(intent) if self.sticks_live else [0.0, 0.0, 0.0]

    # -- engagement ------------------------------------------------------------------------------

    def drop(self, reason: str) -> None:
        """End the engagement (if any) and require a release before the next one."""
        self._last_ns = None
        self._release_to_hold(None)
        self.needs_release = True
        if reason:
            self.status.reason = reason

    def _release_to_hold(self, robot: Optional[RobotState]) -> None:
        """End the engagement: hold the commanded EE at this moment, in the base frame (no robot row yet: the next
        row's)."""
        robot = robot or self.last_robot
        if self.eng is not None:
            self.hold_base = robot.base.to_local(robot.ee_p, robot.ee_r) if robot is not None else None
        self.eng = None

    def on_frame(self, frame: dict, verdict, robot: Optional[RobotState], now: float) -> None:
        if not verdict.accept:
            return
        if robot is not None:
            self.last_robot = robot
        self._tracked = False
        self._engagement(frame, verdict, robot, now)
        if self.valid and self.frame is not None:
            self._gates(frame)
            self._sticks(frame, robot)
        if self._tracked and self.eng is not None:
            # after the sticks: the EE target and the base hint use the same (this frame's) offset
            self._track_offset(frame)

    def _sticks(self, frame: dict, robot: Optional[RobotState]) -> None:
        ns = frame.get("sample_time_ns")
        dt = 0.0
        if isinstance(ns, int) and self._last_ns is not None:
            dt = min(max(0.0, (ns - self._last_ns) * 1e-9), self.cfg.max_dt)
        self._last_ns = ns if isinstance(ns, int) else None
        if self.eng is None or dt <= 0 or self.eng.captured_now:
            return                       # the capture frame: the offset stays zero (engaging never moves the robot)
        v = self.stick_v(frame["control_intent"])
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
            self.eng.captured_now = False
            self._track(frame)                   # link 1 (and a head re-anchor) before link 2 reads the head
            self.eng.ee_rel = self._link2(*hp)
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
        eng.target_raw = (robot.ee_p.copy(), robot.ee_r.copy())
        eng.target = eng.target_raw
        eng.ee_rel = eng.ee_anchor
        eng.head_last = (eng.head_p0.copy(), eng.head_yaw0)
        eng.hand_fit = VelocityFit(self.cfg.ff_window, (self.cfg.ff_max_speed, self.cfg.ff_max_omega))
        eng.head_fit = VelocityFit(self.cfg.ff_window, (self.cfg.hint_max_speed, self.cfg.hint_max_omega))
        self.offset = np.zeros(3)
        eng.captured_now = True
        self.eng = eng
        self._track(frame)
        self.status.engagements += 1
        self.status.state = "engaged"
        self.status.reason = ""

    def _track(self, frame: dict) -> None:
        """Engaged: this frame's head contribution; the offset and the feedforward samples follow the sticks
        (``_track_offset``, after ``_sticks`` in ``on_frame``)."""
        eng = self.eng
        if self.cfg.head_walk:
            head = head_pose(frame)
            yaw = F.head_yaw(head[1]) if head is not None else None
            if yaw is not None:
                if not eng.head_tracked:            # reacquired: re-anchor so neither link jumps
                    self._reanchor_head(head[0], yaw)
                eng.head_d = F.rot_z(-eng.head_yaw0) @ F.C @ (head[0] - eng.head_p0)
                dyaw = F.wrap(yaw - eng.head_yaw0)
                eng.head_yaw_unwrapped += F.wrap(dyaw - eng.head_dyaw)
                eng.head_dyaw = dyaw
                eng.head_last = (np.asarray(head[0], float).copy(), yaw)
            elif eng.head_tracked:                  # head lost: keep its last contribution, no velocity
                eng.head_fit.reset()
            eng.head_tracked = yaw is not None
        self._tracked = True

    def _reanchor_head(self, head_p_local, yaw: float) -> None:
        """The head came back after a loss, possibly elsewhere. Both links restart from where they are, as at an
        engagement: link 1's anchor becomes the current base reference (without the stick offset) and the head's
        new pose; link 2 re-anchors on the next hand pose (``rebase_hand``) at the current ``ee_rel``. The robot
        does not move; only motion from here on counts."""
        eng = self.eng
        x, y, byaw = self._hint_pose(np.zeros(3))
        eng.robot = F.PlanarFrame(x, y, byaw, float(eng.robot.p[2]))
        eng.head_p0 = np.asarray(head_p_local, float).copy()
        eng.head_yaw0 = float(yaw)
        eng.head_d = np.zeros(3)
        eng.head_dyaw = 0.0
        eng.head_last = (eng.head_p0.copy(), eng.head_yaw0)
        eng.rebase_hand = True
        eng.head_fit.reset()

    def _link2(self, hand_p_local, hand_r_local) -> tuple:
        """Link 2: the EE target in the base reference frame, from the hand's motion relative to the head.

        With head walking the hand is read in the current head frame (newest tracked head: position, gravity-aligned
        yaw), so a rigid operator motion (head and hand together) leaves it unchanged; link 1 carries the motion to
        the base. Without head walking link 1 is frozen and so is the frame (the operator frame at engagement): the
        DEMO.md body-frozen formula."""
        eng = self.eng
        if self.cfg.head_walk and eng.head_last is not None:
            # only what the base can follow moves the frame (x, y, yaw): the height stays at the head's height at
            # engagement (or re-anchor), so a crouch or a rise moves the hand, and so the arm, up and down
            p, yaw = eng.head_last
            p = np.asarray(p, float) + UP_LOCAL * float(UP_LOCAL @ (eng.head_p0 - np.asarray(p, float)))
            frame = F.OperatorFrame(p, yaw)
        else:
            frame = eng.operator
        hand = frame.pose(hand_p_local, hand_r_local)
        if eng.rebase_hand:
            eng.hand_anchor = hand
            eng.ee_anchor = eng.ee_rel
            eng.rebase_hand = False
        return F.apply_delta(eng.ee_anchor, *F.delta(eng.hand_anchor, hand))

    def _compose(self, offset) -> tuple:
        """World target = the base reference (link 1, with ``offset``) ∘ ``ee_rel`` (link 2)."""
        x, y, yaw = self._hint_pose(offset)
        return F.PlanarFrame(x, y, yaw, float(self.eng.robot.p[2])).to_world(*self.eng.ee_rel)

    def _track_offset(self, frame: dict) -> None:
        """The target with this frame's stick offset (the base hint's too) and the feedforward fits' samples."""
        eng = self.eng
        if not eng.captured_now:                   # the capture frame keeps the exact command (no jump)
            eng.target_raw = self._compose(np.zeros(3))
            eng.target = self._compose(self.offset)
        ns = frame.get("sample_time_ns")
        if not isinstance(ns, int):
            return
        t = ns * 1e-9
        p, r = eng.target
        eng.hand_fit.push(t, np.concatenate([p, rotvec(r @ eng.target_r0.T) if eng.target_r0 is not None else
                                             np.zeros(3)]))
        if eng.target_r0 is None:
            eng.target_r0 = r.copy()
        if self.cfg.head_walk and eng.head_tracked:
            xy = eng.robot.r[:2, :2] @ eng.head_d[:2]
            eng.head_fit.push(t, [xy[0], xy[1], eng.head_yaw_unwrapped])

    def _hint_pose(self, offset) -> tuple:
        """The base hint (x, y, yaw) world: the base at engagement + the head's motion + ``offset``."""
        eng = self.eng
        xy = eng.robot.p[:2] + eng.robot.r[:2, :2] @ eng.head_d[:2] + offset[:2]
        return float(xy[0]), float(xy[1]), F.wrap(eng.robot.yaw + eng.head_dyaw + offset[2])

    def feedforward(self) -> tuple:
        """(EE twist [vx, vy, vz, wx, wy, wz] world, hint velocity (vx, vy, omega) world) of the engagement: the
        fits, plus the sticks' rate for the hint; zeros when off or not engaged."""
        eng = self.eng
        if eng is None or not self.cfg.feedforward or eng.hand_fit is None:
            return np.zeros(6), np.zeros(3)
        tw = eng.hand_fit.velocity(6)
        hv = eng.head_fit.velocity(3) if self.cfg.head_walk and eng.head_tracked else np.zeros(3)
        sv = self.stick_v(self.frame["control_intent"]) if self.frame is not None else [0.0, 0.0, 0.0]
        yaw = self.last_robot.base_yaw if self.last_robot is not None else eng.robot.yaw
        hv = hv + np.concatenate([F.rot_z(yaw)[:2, :2] @ np.array(sv[:2]), [sv[2]]])
        n = float(np.hypot(hv[0], hv[1]))
        if n > self.cfg.hint_max_speed:
            hv[:2] *= self.cfg.hint_max_speed / n
        hv[2] = float(np.clip(hv[2], -self.cfg.hint_max_omega, self.cfg.hint_max_omega))
        return tw, hv

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
            if cfg.feedforward:
                ee["twist"] = _l(self.feedforward()[0])
            self.last_target = (p, r)
            reset = self.eng.est_reset            # the estimator epoch the anchor was captured in
        else:
            if robot is None:
                return "alive"
            if self.hold_base is None:
                self.hold_base = robot.base.to_local(robot.ee_p, robot.ee_r)
            # rides with the platform: on the base pose predicted to when the WBC applies it (the stream row's age
            # plus ride_lead, by the commanded twist), so the target does not trail a moving base
            dt = max(0.0, now - robot.t_recv) + cfg.ride_lead
            vx, vy, w = robot.base_twist
            c, s_ = math.cos(robot.base_yaw), math.sin(robot.base_yaw)
            base = F.PlanarFrame(robot.base_x + (c * vx - s_ * vy) * dt, robot.base_y + (s_ * vx + c * vy) * dt,
                                 robot.base_yaw + w * dt)
            p, r = base.to_world(*self.hold_base)
            ee = {"frame": "world", "pos": _l(p), "quat": _l(F.quat_from_matrix(r)), "twist": robot.twist_at(p)}
            reset = robot.est_reset
        intent = self.frame["control_intent"]
        ih = intent["hands"].get(cfg.hand) or {}
        if cfg.gripper and self.gripper_live and ih.get("valid") and _finite(ih.get("gripper")):
            # the headset's gripper intent follows the trigger (0 released .. 1 squeezed); the WBC's is the opening
            # (0 closed .. 1 open): squeezing closes, as in the rebot_ctl teleop
            ee["gripper"] = 1.0 - min(1.0, max(0.0, float(ih["gripper"])))
        out = {"ees": {cfg.ee: ee}, "base": self._base(intent)}
        if reset is not None:
            out["est_reset"] = int(reset) if float(reset).is_integer() else reset
        return out

    def _base(self, intent: dict) -> dict:
        cfg = self.cfg
        if self.eng is None:
            self.hint = None
            v = self.stick_v(intent)
            if any(abs(x) > 1e-9 for x in v):
                return {"mode": "commanded", "velocity": {"vx": v[0], "vy": v[1], "omega": v[2], "frame": "base",
                                                          "ttl": cfg.velocity_ttl}}
            return {"mode": "locked"}
        hx, hy, hyaw = self._hint_pose(self.offset)
        target = {"pose": {"x": hx, "y": hy, "frame": "world",
                           "weight": cfg.pose_weight, "tol": cfg.pose_tol},
                  "heading": {"yaw": hyaw, "frame": "world", "weight": cfg.heading_weight, "tol": cfg.heading_tol}}
        if cfg.feedforward:
            hv = self.feedforward()[1]
            target["pose"].update(vx=float(hv[0]), vy=float(hv[1]))
            target["heading"]["omega"] = float(hv[2])
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
