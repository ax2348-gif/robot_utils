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
dead zone and full scale 0.45 m/s / 0.8 rad/s applied natively; ``stick_velocity`` rescales that full scale to
``Config.stick_max_speed`` / ``stick_max_omega``, the left stick's translation clamped to a circle):

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
from .arming import button, sticks_neutral

UP_LOCAL = F.C.T @ np.array([0.0, 0.0, 1.0])     # the up axis in OpenXR LOCAL (+Y)

# The native stick scaling (quest_env native/teleop_core.h, Teleop::update: ``.45 * stick(..)`` / ``.8 * stick(..)``,
# 18 % dead zone, not rescaled). MAGIC NUMBERS duplicated from the headset: they must match it until the scaling moves
# to one place (a joint refactor with quest_env: send the raw normalised sticks and scale here only).
NATIVE_STICK_SPEED = 0.45    # m/s, the native base_velocity forward / left at full deflection
NATIVE_STICK_OMEGA = 0.8     # rad/s, the native base_velocity yaw at full deflection
# Bounds on the configurable top speed: the swerve's hard limits (docs/swerve_node.md; the WBC caps lie within them)
TOP_SPEED_LIMIT = 2.0        # m/s
TOP_OMEGA_LIMIT = 4.0        # rad/s


def _pl_compose(a, b) -> np.ndarray:
    """Planar poses (x, y, yaw): ``a ∘ b``."""
    c, s_ = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0] + c * b[0] - s_ * b[1], a[1] + s_ * b[0] + c * b[1], F.wrap(a[2] + b[2])])


def _pl_inverse(a) -> np.ndarray:
    c, s_ = math.cos(a[2]), math.sin(a[2])
    return np.array([-(c * a[0] + s_ * a[1]), s_ * a[0] - c * a[1], F.wrap(-a[2])])


def operator_planar(head_p_local, yaw: float) -> np.ndarray:
    """The operator's planar pose in the headset's room (operator FLU axes of LOCAL): position, gravity-aligned
    yaw."""
    p = F.C @ np.asarray(head_p_local, float)
    return np.array([p[0], p[1], float(yaw)])



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
    mode: str = "third"              # "third" | "first": where the stick terms enter (Config.mode at capture)
    correction: np.ndarray = field(default_factory=lambda: np.zeros(3))   # first person: anchor correction C
    hand_delta: Optional[tuple] = None   # link 2's hand delta (dp, dR), body frame
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
    headless: bool = True             # third person, registered: the left stick moves the robot along the
                                      # operator's facing (mapped by the registration), not the robot's forward
    mode: str = "third"               # third: the sticks move the robot, directions stay room-aligned (the
                                      # grip registers the headset to the robot); first: the sticks shift the
                                      # anchor (your forward is the robot's forward). Read at each engagement.
    ride_lead: float = 0.01           # s: released, the riding target is placed on the base pose predicted this far
                                      # past the stream row's arrival (plus its age), so it does not lag the base
    feedforward: bool = True          # engaged: the EE twist and the hint velocity (see the module docstring)
    ff_window: float = 0.06           # s, the velocity fit's window (adds about half of it as lag)
    ff_max_speed: float = 1.0         # m/s, EE feedforward clamp
    ff_max_omega: float = 3.0         # rad/s, EE feedforward clamp
    # the top speed (``set_top_speed``: both pairs together; the WBC clamps further to its own caps)
    stick_max_speed: float = 0.8      # m/s, the left stick at full deflection (any direction: clamped to a circle)
    stick_max_omega: float = 0.9      # rad/s, the right stick at full deflection
    hint_max_speed: float = 0.8       # m/s, the hint velocity clamp (head walking + sticks)
    hint_max_omega: float = 0.9       # rad/s


def set_top_speed(cfg: Config, max_speed: Optional[float] = None, max_omega: Optional[float] = None) -> Config:
    """The teleop's top speed, in place: the sticks' full deflection and the hint velocity clamp together
    (``max_speed`` m/s, ``max_omega`` rad/s; None keeps that one). Bounded by the swerve's hard limits
    (``TOP_*_LIMIT``); ValueError otherwise, nothing changed. Applies from the next frame (the head's velocity fit
    takes its clamp at the next engagement)."""
    for v, lim, what in ((max_speed, TOP_SPEED_LIMIT, "max_speed (m/s)"), (max_omega, TOP_OMEGA_LIMIT, "max_omega (rad/s)")):
        if v is not None and not (_finite(v) and 0 < v <= lim):
            raise ValueError(f"{what} must be in (0, {lim}]")
    if max_speed is not None:
        cfg.stick_max_speed = cfg.hint_max_speed = float(max_speed)
    if max_omega is not None:
        cfg.stick_max_omega = cfg.hint_max_omega = float(max_omega)
    return cfg


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
    registered: bool = False      # third person: the headset's room is registered to the robot's world
    registration: str = ""        # how / why (first enable, click, clutch; or why it was dropped)


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
        self.reg: Optional[np.ndarray] = None    # third person: A = robot ∘ operator⁻¹ (planar), the registration
        self.reg_reset: Optional[float] = None   # the estimator epoch it was made in
        self._click = True                       # the right stick click, last frame (True: wait for a release)
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
        if self.reg is None and frame is not None:
            self._register(frame, "first enable")  # the default zero: the operator faces where the robot faces

    # -- the registration (third person) ---------------------------------------------------------

    def _register(self, frame: dict, why: str) -> bool:
        """Register the headset's room to the robot's world from this frame: the operator faces where the robot
        faces (``A = robot ∘ operator⁻¹``). Released only (engaged: ``_register_in_place``)."""
        head, robot = head_pose(frame), self.last_robot
        yaw = F.head_yaw(head[1]) if head is not None else None
        if yaw is None or robot is None:
            return False
        o = operator_planar(head[0], yaw)
        self.reg = _pl_compose(np.array([robot.base_x, robot.base_y, robot.base_yaw]), _pl_inverse(o))
        self.reg_reset = robot.est_reset
        self.status.registered, self.status.registration = True, why
        return True

    def _unregister(self, why: str) -> None:
        if self.reg is not None:
            self.reg, self.reg_reset = None, None
            self.status.registered, self.status.registration = False, f"dropped: {why}"

    def _register_in_place(self, frame: dict) -> bool:
        """Engaged, third person: register to how the operator faces now without moving the robot. The current
        base reference becomes the registered pose of the operator's current pose (``A = Ref ∘ O⁻¹``, sigma = s = 0)
        and both links re-anchor where they are."""
        eng = self.eng
        head, hp = head_pose(frame), hand_pose(frame, self.cfg.hand)
        yaw = F.head_yaw(head[1]) if head is not None else None
        if yaw is None or hp is None or eng.target is None:
            return False
        ref = np.array(self._hint_pose())
        self.reg = _pl_compose(ref, _pl_inverse(operator_planar(head[0], yaw)))
        self.reg_reset = eng.est_reset
        eng.robot = F.PlanarFrame(ref[0], ref[1], ref[2], float(eng.robot.p[2]))
        eng.head_p0 = np.asarray(head[0], float).copy()
        eng.head_yaw0 = float(yaw)
        eng.head_d, eng.head_dyaw = np.zeros(3), 0.0
        eng.head_last = (eng.head_p0.copy(), eng.head_yaw0)
        eng.operator = F.OperatorFrame(eng.head_p0, eng.head_yaw0)
        self.offset = np.zeros(3)
        eng.ee_anchor = eng.robot.to_local(*eng.target)
        eng.ee_rel = eng.ee_anchor
        eng.hand_anchor = eng.operator.pose(*hp)
        eng.hand_delta = F.delta(eng.hand_anchor, eng.hand_anchor)
        eng.rebase_hand = False
        eng.head_fit.reset()
        self.status.registered, self.status.registration = True, "right stick click (engaged)"
        return True

    def _clicks(self, frame: dict) -> None:
        """The right stick click (a press seen released first) registers, third person: released at once,
        engaged in place."""
        down = button(frame, "right", "stick")
        pressed, self._click = down and not self._click, down
        if not pressed or self.cfg.mode == "first":
            return
        if self.eng is not None:
            if self.eng.mode == "third":
                self._register_in_place(frame)
        else:
            self._register(frame, "right stick click")

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
        return (stick_velocity(intent, self.cfg.stick_max_speed, self.cfg.stick_max_omega) if self.sticks_live
                else [0.0, 0.0, 0.0])

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
            self._clicks(frame)
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
        # stick forward = the robot's forward (headless: the operator's facing, through the registration)
        d_xy = F.rot_z(self._stick_yaw(frame, yaw))[:2, :2] @ np.array(v[:2]) * dt
        d_yaw = v[2] * dt
        if self.eng.mode == "first":
            # an increment in the robot's own frame, Base -> Base·Δ, folded into the anchor correction
            # C <- (Base Δ Base⁻¹) C: a turn about the base reference's position, then the translation
            x, y, _ = self._hint_pose()
            c = self.eng.correction
            pivot = np.array([x, y])
            rz = F.rot_z(d_yaw)[:2, :2]
            c[:2] = rz @ (c[:2] - pivot) + pivot + d_xy
            c[2] += d_yaw
        else:
            # third person: the translation in the world (left, T(s)), the yaw on the robot only (right, R(sigma))
            self.offset[:2] += d_xy
            self.offset[2] += d_yaw

    def _engagement(self, frame: dict, verdict, robot: Optional[RobotState], now: float) -> None:
        if verdict.discontinuity:
            self.drop(verdict.reason)
            # only events that change the headset's frame drop the registration (a new XR session, an operator
            # frame realign); gaps and reconnects do not: re-arming after one must not re-register silently
            if {"session", "revision"} & set((verdict.reason or "").split(",")):
                self._unregister(verdict.reason)
            self._click = True
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
        if self.reg is not None and robot is not None and robot.est_reset != self.reg_reset:
            self._unregister("estimator reset")
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
        eng.mode = "first" if self.cfg.mode == "first" else "third"
        eng.head_last = (eng.head_p0.copy(), eng.head_yaw0)
        eng.hand_fit = VelocityFit(self.cfg.ff_window, (self.cfg.ff_max_speed, self.cfg.ff_max_omega))
        eng.head_fit = VelocityFit(self.cfg.ff_window, (self.cfg.hint_max_speed, self.cfg.hint_max_omega))
        self.offset = np.zeros(3)
        if eng.mode == "third":
            if self.reg is None:
                self._register(frame, "first clutch (no registration yet)")
            if self.reg is not None:
                # keep the registration: link 1's anchor is the registered pose of the operator now (A ∘ O_k);
                # the robot's actual pose differs by a translation s (world) and a heading offset sigma, which go
                # into the stick slots, so this clutch moves nothing and the directions stay registered
                ref = _pl_compose(self.reg, operator_planar(eng.head_p0, eng.head_yaw0))
                eng.robot = F.PlanarFrame(ref[0], ref[1], ref[2], float(eng.robot.p[2]))
                self.offset = np.array([robot.base_x - ref[0], robot.base_y - ref[1],
                                        F.wrap(robot.base_yaw - ref[2])])
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
        x, y, byaw = self._head_pose()
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
        sigma = self._sigma()
        if eng.rebase_hand:
            # the new anchor is the current link-2 pose with the third-person stick yaw taken back out (it is
            # applied to the anchor, inside the hand delta: R(sigma) anchor ⊕ 0 = the current pose)
            eng.hand_anchor = hand
            eng.ee_anchor = _rot_pose(eng.ee_rel, -sigma)
            eng.rebase_hand = False
        eng.hand_delta = F.delta(eng.hand_anchor, hand)
        return F.apply_delta(_rot_pose(eng.ee_anchor, sigma), *eng.hand_delta)

    def _stick_yaw(self, frame: Optional[dict], robot_yaw: float) -> float:
        """The world direction of the left stick's forward: the robot's heading, or with ``headless`` (third
        person, registered, head tracked) the operator's facing mapped by the registration (``A ∘ O``'s yaw)."""
        mode = self.eng.mode if self.eng is not None else self.cfg.mode
        if self.cfg.headless and mode == "third" and self.reg is not None and frame is not None:
            head = head_pose(frame)
            yaw = F.head_yaw(head[1]) if head is not None else None
            if yaw is not None:
                return F.wrap(float(self.reg[2]) + yaw)
        return robot_yaw

    def _sigma(self) -> float:
        """The stick yaw applied inside the hand delta: the third-person stick yaw (it turns the robot, not the
        direction mappings); 0 in first person, whose stick yaw lives in the anchor correction."""
        return float(self.offset[2]) if self.eng is not None and self.eng.mode == "third" else 0.0

    def _compose(self, with_sticks: bool = True) -> tuple:
        """World target = the base reference without the stick terms (``A ∘ O(t)``: base at engagement + the
        head's motion) ∘ ``ee_rel``, then the stick terms on the left: third person ``T(s)``, first person the
        anchor correction ``C``. (The third-person stick yaw is already inside ``ee_rel``.)"""
        eng = self.eng
        if eng.hand_delta is not None:
            # link 2 with this frame's stick yaw (the sticks are integrated after the hand is read)
            eng.ee_rel = F.apply_delta(_rot_pose(eng.ee_anchor, self._sigma()), *eng.hand_delta)
        x, y, yaw = self._head_pose()
        p, r = F.PlanarFrame(x, y, yaw, float(eng.robot.p[2])).to_world(*eng.ee_rel)
        if not with_sticks:
            return p, r
        if eng.mode == "third":
            return p + np.array([self.offset[0], self.offset[1], 0.0]), r
        c = eng.correction
        rz = F.rot_z(c[2])
        return rz @ p + np.array([c[0], c[1], 0.0]), rz @ r

    def _track_offset(self, frame: dict) -> None:
        """The target with this frame's stick terms (the base hint's too) and the feedforward fits' samples."""
        eng = self.eng
        if not eng.captured_now:                   # the capture frame keeps the exact command (no jump)
            eng.target_raw = self._compose(with_sticks=False)
            eng.target = self._compose()
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
            # in the operator frame of the engagement; mapped to the world at use (``_map_rot``): in first person
            # the mapping turns with the stick yaw
            eng.head_fit.push(t, [eng.head_d[0], eng.head_d[1], eng.head_yaw_unwrapped])

    def _head_pose(self) -> tuple:
        """The base reference without the stick terms, ``A ∘ O(t)``: the base at engagement (link-1 anchor) + the
        head's motion, (x, y, yaw) world."""
        eng = self.eng
        xy = eng.robot.p[:2] + eng.robot.r[:2, :2] @ eng.head_d[:2]
        return float(xy[0]), float(xy[1]), F.wrap(eng.robot.yaw + eng.head_dyaw)

    def _map_rot(self) -> np.ndarray:
        """The rotation (2x2) taking operator-frame motion to world motion: the engagement's, and in first person
        also the anchor correction's yaw."""
        eng = self.eng
        r = eng.robot.r[:2, :2]
        return F.rot_z(eng.correction[2])[:2, :2] @ r if eng.mode == "first" else r

    def _hint_pose(self, offset=None) -> tuple:
        """The base hint (x, y, yaw) world: ``A ∘ O(t)`` with the stick terms. Third person ``T(s) · A·O(t) ·
        R(sigma)`` (``offset``: the stick translation s, world, and yaw sigma; default the current ones); first
        person ``C · A·O(t)`` with the anchor correction ``C``."""
        eng = self.eng
        x, y, yaw = self._head_pose()
        if eng.mode == "first":
            c = eng.correction
            xy = F.rot_z(c[2])[:2, :2] @ [x, y] + c[:2]
            return float(xy[0]), float(xy[1]), F.wrap(yaw + c[2])
        o = self.offset if offset is None else offset
        return float(x + o[0]), float(y + o[1]), F.wrap(yaw + o[2])

    def feedforward(self) -> tuple:
        """(EE twist [vx, vy, vz, wx, wy, wz] world, hint velocity (vx, vy, omega) world) of the engagement: the
        fits, plus the sticks' rate for the hint; zeros when off or not engaged."""
        eng = self.eng
        if eng is None or not self.cfg.feedforward or eng.hand_fit is None:
            return np.zeros(6), np.zeros(3)
        tw = eng.hand_fit.velocity(6)
        hv = eng.head_fit.velocity(3) if self.cfg.head_walk and eng.head_tracked else np.zeros(3)
        hv[:2] = self._map_rot() @ hv[:2]
        sv = self.stick_v(self.frame["control_intent"]) if self.frame is not None else [0.0, 0.0, 0.0]
        yaw = self.last_robot.base_yaw if self.last_robot is not None else eng.robot.yaw
        yaw = self._stick_yaw(self.frame, yaw)
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
            if self.cfg.headless and self.last_robot is not None:
                # headless: the stick's direction in the world, expressed in the base frame for the command
                ryaw = self.last_robot.base_yaw
                vxy = F.rot_z(ryaw - self._stick_yaw(self.frame, ryaw))[:2, :2].T @ np.array(v[:2])
                v = [float(vxy[0]), float(vxy[1]), v[2]]
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


def _rot_pose(pose, angle: float) -> tuple:
    """A pose turned about Z through the origin of its frame."""
    if not angle:
        return pose
    rz = F.rot_z(angle)
    return rz @ np.asarray(pose[0], float), rz @ np.asarray(pose[1], float)


def stick_velocity(intent: dict, max_speed: float = NATIVE_STICK_SPEED, max_omega: float = NATIVE_STICK_OMEGA) -> list:
    """The intent's ``base_velocity`` (forward, left m/s; yaw rad/s), non-finite parts as 0, rescaled from the native
    full scale (``NATIVE_STICK_*``) to ``max_speed`` / ``max_omega``; the translation's norm is clamped to
    ``max_speed`` (the native per-axis scaling makes a square: a full diagonal would be sqrt 2 faster)."""
    v = intent.get("base_velocity") or [0, 0, 0]
    vx, vy, w = (float(x) if _finite(x) else 0.0 for x in (list(v) + [0, 0, 0])[:3])
    vx, vy, w = vx * max_speed / NATIVE_STICK_SPEED, vy * max_speed / NATIVE_STICK_SPEED, w * max_omega / NATIVE_STICK_OMEGA
    n = math.hypot(vx, vy)
    if n > max_speed:
        vx, vy = vx * max_speed / n, vy * max_speed / n
    return [vx, vy, max(-max_omega, min(max_omega, w))]


def _finite(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _l(a) -> list:
    return [float(x) for x in a]
