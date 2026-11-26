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
  forward), yaw from the right stick, held to the commanded base by the stick tether (below). Standing still and
  driving with the sticks, walking with the robot mirroring, or both. Whatever the sticks achieved is in the base
  pose the next engagement captures.

**The stick tether** (``Config.tether``, on; read at each engagement). Integrated raw, the offset moves the base
reference, and the EE target composed on it, at full stick speed the instant the stick moves, while the WBC
accelerates the base at its ``max_accel`` / ``max_alpha``: the reference leads the base by up to v²/2a (0.32 m at
0.8 m/s and 1 m/s², 0.27 rad at 0.9 rad/s and 1.5 rad/s²), the arm reaches to cover it, and swings back when the stick
is released and the base brakes past the reference. Tethered, each frame after the stick's increment:

- **The commanded base** ``B``: the newest stream row's base pose (the one ``RobotState.base`` and the riding target
  use) advanced by its commanded twist over the row's age plus ``ride_lead`` (``_commanded_base``). Missing or older
  than ``robot_stale_s``: the offset **freezes** (the stick's increment is dropped).
- **The lead** ``L = H - B`` (the base reference ``H = _hint_pose()`` minus ``B``; x, y world, yaw), split into
  walking's share ``W`` and the stick's share ``S = L - W``. ``W`` is the lead when the stick starts (from idle: the
  lead so far is walking's); from then on it grows with the reference's own motion with the stick held (the head's,
  mapped: ``H`` before this frame's increment minus ``H`` after the last step), and the base's progress is credited
  to it **first**: the component of the base's motion along ``W``, clamped to ``[0, |W|]`` (translation and yaw
  separately). Why: the inputs drive one base and their leads cannot be split by measurement; walking is the
  operator's intent (a walking lead is never dragged back, and the base consumes it by moving towards it), the
  stick's share is what the base has not done of the stick's motion beyond that. A rule that kept ``W`` fixed while
  the offset follows the base would make the base chase a lead it can never close.
- **Per channel** (translation, yaw), from the stick (outside the native dead zone) and the base's commanded twist
  against the walking hint velocity (``tether_settle_v`` / ``_w``): **deflected** (active): ``|S|`` at most
  ``tether_pos`` (0.05 m, norm) / ``tether_yaw`` (0.1 rad): the reference waits for the base, never more than that
  ahead; **idle while the base still moves from the stick** (braking; the first idle frame always): bound zero: the
  offset follows the base, so the robot stops where its braking puts it, no snap-back; **idle and settled**: the
  offset freezes, the channel idle (``S`` = 0; ``W`` restarts with the next stick). Gripped while the base still moves
  (driven released): both moving channels start braking.
- **The projection** moves only the stick channel: third person ``s`` / ``sigma``, first person ``C`` (a world
  translation; a turn about the reference's position, as a stick increment). ``eng.head_d`` / ``head_dyaw`` are never
  touched.
- **The base hint** keeps the raw stick velocity in ``vx, vy, omega`` (``feedforward``): never clipped by the base's
  state (modules perpendicular to the stick would otherwise give 0 forever), so steering, the WBC's start charge and
  its acceleration limit act as before. Its pose and heading lead the base by at most the bound.
- **Link 2's frame takes the commanded base's yaw for the stick's yaw** (``arm_yaw`` = the stick's yaw share after
  the projection): a translation lead only offsets the EE (≤ 5 cm; the hand's deltas still map exactly), but a yaw
  lead would rotate link 2, the hand's deltas and the whole arm pose, about the base. Third person: ``sigma_arm =
  sigma - arm_yaw`` replaces ``sigma`` in link 2 (``_sigma``); first person: ``C_arm = Rot_P(-arm_yaw) · C``, ``C``
  turned back about the reference's position ``P`` (``_compose``). Either way the EE's frame has yaw ``B.yaw +
  W.yaw``: the commanded base's, plus walking's lead, which keeps leading as designed. Capture: ``arm_yaw`` = 0 and
  ``S`` = 0, so engaging moves nothing. A right-stick re-registration in place adds ``S`` to ``W`` (the reference
  does not move).
- **The EE feedforward** is the target's own fitted velocity, so the tethered offset's rate (the base's), not the
  raw stick.

Two hands: each core tethers its own offset to the same commanded base; the base hint comes from the owner. Both
references stay within their bound of the base, so a handover moves the hint by at most the two cores' leads
(``test_quest_stick_tether.py``: 4 mm; untethered, each core's reference integrated the stick from its own engagement
and the handover jumped 0.2 m). ``tether`` off: exactly the raw integral (the datagrams before the tether).

**The hint's source** (``Config.hint_source``, read at each engagement like ``mode``): ``head`` (the headset, raw: the
behaviour described above), ``chest`` or ``hips`` (Meta body tracking's joints, ``hint_source.py``). A body source
drives link 1 *and* link 2's frame with the same structure: its horizontal displacement and yaw change since the
engagement, displacement read in the engagement's operator frame (the head's facing at the grip), both low-passed
with a deadband. When it is not usable (no body, low confidence, the joint untracked) the head takes over, smoothed
the same way, and the body takes back over once usable again for ``hint_return_s``. Each switch (and each loss and
return) starts a new segment where the previous one ended, so the base reference and the EE target never jump: only
motion from then on counts, from the new source. Neither height nor where the headset is worn (neck or head) enters.

Velocity feedforward while engaged (so the robot keeps up with a walking operator instead of trailing by v / k):
the EE target carries ``twist`` = the target's own velocity (the hand's motion mapped through the frozen frames,
linear and angular) and the base hint carries the hint target's velocity (``vx``, ``vy``, ``omega``: the head's
motion mapped as ``head_d`` is, plus the raw stick velocity, exact; tethered, the target's own velocity carries the
tethered offset's rate, not the raw stick). Both are least-squares slopes over the frames
of the last ``ff_window`` (60 ms) on ``sample_time_ns``: exact for a steady motion, about ``ff_window / 2`` (30 ms)
behind a change of speed; clamped; zero while the head is lost (and its history restarts). Every engagement starts
them at zero: only frames of this engagement count, and they ramp in with the window's fill (re-engaging never moves
the robot). Released: unchanged (the riding target's twist is the base's own).

The gripper follows the trigger whenever the hand is valid, independent of the clutch.

Two hands, two arms (``Config.hands``, e.g. ``{"left": "arm_left", "right": "arm_right"}``; ``MultiCore``): one core
per hand, each exactly the above for its own arm: its grip clutches its arm, its trigger is its gripper, released its
arm rides with the platform. The base: with no hand engaged, the sticks (as above); with one hand engaged, that hand's
engagement (as above); with both engaged, the hand that engaged **first** keeps the base until it releases, then the
other's engagement takes over. Its base reference tracked the same head and stick motion since its own engagement,
so it continues from where the base is, up to the base's tracking error at that engagement. A hand that is not valid
(controller lost) only rides; the other keeps working.

Mirror pairing (two hands, ``Config.mirror``; ``MultiCore``): which arm each hand drives. **Straight** = ``Config.hands``
(left → the left arm); **mirror** = swapped (the operator's left → the robot's right arm, and back), as in a mirror
when the robot faces the operator. Decided once, when the first hand engages from all released (no core engaged),
before that frame's capture, and **locked until both hands are released**: a second hand engaging, or one hand
releasing while the other stays engaged, never re-decides; releasing both and engaging again does. ``Config.mirror``:

- ``off`` (default): always straight, no decision and no event (exactly the behaviour before mirror existed);
- ``straight`` / ``mirror``: forced, but each decision is still reported (with the heading difference);
- ``auto``: mirror iff the operator's heading and the robot base's heading are more than 90° apart (the robot faces
  the operator). The operator's heading is the gravity-aligned yaw of the head (the scheme's torso proxy,
  ``MultiCore.operator_yaw``) mapped into the robot's world by the third-person registration (``A``; the yaw of
  ``A ∘ O``); not registered (it registers at this clutch: the operator faces where the robot faces) or first person
  (the operator's forward is the robot's by construction): the difference is 0, straight.

Only the pairing changes: each hand's EE target is the body-frozen mapping above, unchanged, applied to its paired
arm, anchored on that arm's own commanded state at engagement (the capture runs after the swap), so a swap never
jumps an arm; the hand's trigger is its paired arm's gripper; a paired arm with no stream row refuses the
engagement as before. A swap while released carries each arm's riding pose with the arm (``hold_base`` and the
newest row are per arm), so the released arms do not move either.

The interface for consumers (the elbow-swivel hint): ``MultiCore.pairing`` ({hand: ee} in force),
``MultiCore.mirrored``, ``MultiCore.pairing_info()`` (also in the node's ``/state`` ``teleop.pairing``), and per core
``TeleopCore.mirrored`` (constant for the whole engagement). The elbow swivel needs no pairing sign: it is sent
with the per-side "positive = out" convention (``swivel.SIDE_SIGN``, the WBC's), so the operator's elbow out drives
the paired robot arm's elbow out in either pairing (mirrored: its mirror image).

The elbow-swivel hint (``Config.elbow_hint``, default off; ``swivel.py``): per engaged hand, the operator's swivel
change since the engagement, from body tracking (positive = out), sent as the paired EE's ``swivel`` object
(``TeleopCore.swivel_out``; ``swivel_info`` for the page). Computed whenever there is a body, sent only when on at
the engagement's capture and still on. Off: the datagrams are exactly those without it.

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

from . import body as B
from . import frames as F
from . import swivel as SW
from .arming import button, sticks_neutral
from .hint_source import SOURCES as HINT_SOURCES, HintFilterConfig, HintSources

UP_LOCAL = F.C.T @ np.array([0.0, 0.0, 1.0])     # the up axis in OpenXR LOCAL (+Y)

# The native stick scaling (quest_env native/teleop_core.h, Teleop::update: ``.45 * stick(..)`` / ``.8 * stick(..)``,
# 18 % dead zone, not rescaled). MAGIC NUMBERS duplicated from the headset: they must match it until the scaling moves
# to one place (a joint refactor with quest_env: send the raw normalised sticks and scale here only).
NATIVE_STICK_SPEED = 0.45    # m/s, the native base_velocity forward / left at full deflection
NATIVE_STICK_OMEGA = 0.8     # rad/s, the native base_velocity yaw at full deflection
# Bounds on the configurable top speed: the swerve's hard limits (docs/swerve_node.md; the WBC caps lie within them)
TOP_SPEED_LIMIT = 2.0        # m/s
TOP_OMEGA_LIMIT = 4.0        # rad/s
# Mirror pairing (the module docstring): Config.mirror's values, and auto's threshold on the heading difference
MIRROR_MODES = ("off", "auto", "straight", "mirror")
MIRROR_FACING = math.pi / 2


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
    swivel: Optional[float] = None        # the WBC's ``<ee>_swivel`` (the robot arm's swivel), when it publishes it
    swivel_w: Optional[float] = None      # the WBC's ``<ee>_swivel_w`` (the swivel hint in force)

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
        sw = [_opt_float(row.get(f"{ee}_swivel{k}")) for k in ("", "_w")]
        return cls(t_recv, pose[0], pose[1], bx, by, byaw, float(row.get("est_reset", 0.0)), source, tw, *sw)

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
    swivel_seq: Optional[int] = None                                   # the swivel object's seq (swivel.py)
    swivel_on: bool = False                                            # Config.elbow_hint at the capture
    head_fit: Optional["VelocityFit"] = None                           # the head's (hint) velocity, world
    captured_now: bool = False                                         # captured on the newest frame
    hint_source: str = "head"                                          # Config.hint_source at capture
    hint_in_force: Optional[str] = None                                # body sources: the one driving now
    seg_src: Optional[str] = None                                      # body sources: the current segment's
    seg_p0: Optional[np.ndarray] = None                                # its anchor position (LOCAL, smoothed)
    seg_yaw0: float = 0.0                                              # its anchor yaw (unwrapped, smoothed)
    seg_acc: np.ndarray = field(default_factory=lambda: np.zeros(3))   # the total (dx, dy, dyaw) at its start
    body_ok_since: Optional[float] = None                              # the selected body source usable since
    # the stick tether (module docstring "The stick tether"): per channel idle | active | braking
    tether_on: bool = False                                            # Config.tether at the capture
    tether: dict = field(default_factory=lambda: {"xy": "idle", "yaw": "idle"})
    tether_w: np.ndarray = field(default_factory=lambda: np.zeros(3))  # walking's share of the lead (world x, y; yaw)
    tether_s: np.ndarray = field(default_factory=lambda: np.zeros(3))  # the stick's share after the projection
    tether_h: Optional[np.ndarray] = None                              # the reference, last tether step (x, y, yaw)
    tether_b: Optional[np.ndarray] = None                              # the commanded base, last tether step
    arm_yaw: float = 0.0                                               # the stick yaw share kept out of link 2


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
    hands: Optional[dict] = None      # two hands: {hand: ee} (MultiCore; ``hand`` / ``ee`` then name the primary one)
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
    # engaged: the stick velocity that moves the reference (the offset, its feedforward and the base hint's rate) is
    # ramped at the base's own limits, so the EE target never leads a base that cannot keep up (a step of 0.8 m/s
    # against max_accel 1 m/s^2 put the reference 0.32 m ahead: the arms reached for it, 2026-11-26). With
    # ``stick_shape`` ``open_wbc`` sets the rates from the WBC's max_accel / max_alpha. 0 = that part off (the raw stick)
    stick_shape: bool = True          # open_wbc: take stick_accel / stick_alpha from the WBC's caps
    stick_accel: float = 0.0          # m/s^2, the shaped stick translation's rate of change (0: unshaped)
    stick_alpha: float = 0.0          # rad/s^2, the shaped stick yaw's rate of change (0: unshaped)
    # headroom so the base can catch up (the owner, 2026-11-26): with the ramp at the base's own accel limit and the
    # stick's full scale at its speed cap, a base that starts late (latency, steering, the start charge) could never
    # close the gap, and the arm trailed it until the stick was released
    stick_accel_frac: float = 0.75    # open_wbc: stick_accel / stick_alpha = this x the WBC's max_accel / max_alpha
    stick_speed_frac: float = 1.0     # engaged: the stick's full scale = this x stick_max_speed / stick_max_omega (the
                                      # WBC's caps stay at those); open_wbc sets 0.85 with the ramp
    mirror: str = "off"               # two hands: the hand-to-arm pairing, MIRROR_MODES (``MultiCore.set_mirror``)
    # the base hint's source (module docstring; hint_source.py): head (the default, raw) | chest | hips
    hint_source: str = "head"
    hint_tau: float = 0.15            # s, body sources (and the head as their fallback): low-pass time constant
    hint_deadband_m: float = 0.02     # m, then a deadband in the plane
    hint_deadband_rad: float = 0.03   # rad, and on the yaw
    hint_min_confidence: float = 0.5  # the body's confidence below this: fall back to the head
    hint_require_tracked: bool = True  # the joint must be tracked (not only valid)
    hint_return_s: float = 0.3        # s, the body usable this long before it takes back over from the head
    # the elbow-swivel hint (swivel.py): send each engaged EE's ``swivel`` object (the operator's swivel change)
    elbow_hint: bool = False
    # the stick tether (module docstring "The stick tether"): engaged, the stick offset leads the WBC's commanded
    # base by at most this (the stick's share of the lead; zero while the base brakes from the stick), and link 2
    # takes the commanded base's yaw for the stick's yaw. Off: the offset is the raw stick integral (before 2026-11-26).
    # Read at each engagement.
    tether: bool = False  # off by default (the owner, 2026-11-26: "not usable"; the stick ramp instead)
    tether_pos: float = 0.05          # m, the stick's translation lead (norm) while deflected
    tether_yaw: float = 0.1           # rad, the stick's heading lead while deflected (the base hint only)
    tether_settle_v: float = 0.02     # m/s: stick idle, the base's commanded velocity within this of the walking
    tether_settle_w: float = 0.03     # rad/s  hint's: settled (the braking ends, the offset freezes)


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
    hint_in_force: Optional[str] = None   # engaged: the source driving the base hint now (None: lost / released)
    hint_confidence: Optional[float] = None   # the body's confidence (the newest frame), None without a body
    hint_switches: int = 0        # fallbacks and returns (body sources)
    hint_reason: str = ""         # why the selected source is not in force
    tether: Optional[dict] = None  # engaged, tether on: per channel state and the stick's share of the lead


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
        self.v_shaped = np.zeros(3)               # engaged: the stick velocity ramped at stick_accel / stick_alpha
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
        # the base hint's source candidates (head, chest, hips), raw and smoothed, every frame (logged by on_hint)
        self.sources = HintSources(HintFilterConfig(cfg.hint_tau, cfg.hint_deadband_m, cfg.hint_deadband_rad,
                                                    cfg.hint_min_confidence, cfg.hint_require_tracked))
        self.on_hint = None                       # (record dict) per valid frame: the candidates side by side
        # the elbow-swivel hint's input (swivel.py): this hand's operator arm, the swivel change while engaged
        self.arm: Optional[SW.ArmSample] = None   # the newest valid frame's (None: no valid frame)
        self.swivel = SW.SwivelTracker()
        self.seqs = SW.SeqCounter()               # MultiCore shares one between its hands
        self.swivel_sent = False                  # the last command carried this EE's swivel object

    # -- the mirror pairing (set by MultiCore; the module docstring "Mirror pairing") --------------

    @property
    def mirrored(self) -> bool:
        """This hand drives the opposite-side arm (mirror pairing); constant while engaged."""
        return bool(getattr(self.cfg, "mirrored", False))

    @property
    def swivel_sign(self) -> float:
        """The operator's arm's side sign (swivel.SIDE_SIGN): the right-handed swivel × this is positive out."""
        return SW.SIDE_SIGN[self.cfg.hand]

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

    def on_resume(self, frame: Optional[dict] = None) -> None:
        """Resumed by a re-grip (``arming.py``, the hold tier): unlike ``on_arm`` the engagement this frame just
        captured is kept (it is anchored to the commanded state, so the robot does not move); a released hand's
        riding pose is captured afresh from the current command; the sticks need neutral again; the gripper follows
        the trigger from this frame on."""
        if self.eng is None:
            self.hold_base = None
        self.sticks_live = False
        self.gripper_live = True

    @property
    def captured_now(self) -> bool:
        """An engagement was captured on the newest frame."""
        return bool(self.eng is not None and getattr(self.eng, "captured_now", False))

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
        # tethered: the reference does not move, so the stick's share joins the anchored lead and link 2's frame is
        # the reference's (the target, composed with the arm's yaw, becomes the new anchor below)
        eng.tether_w = eng.tether_w + eng.tether_s
        eng.tether_s, eng.arm_yaw = np.zeros(3), 0.0
        self.reg = _pl_compose(ref, _pl_inverse(operator_planar(head[0], yaw)))
        self.reg_reset = eng.est_reset
        eng.robot = F.PlanarFrame(ref[0], ref[1], ref[2], float(eng.robot.p[2]))
        eng.head_p0 = np.asarray(head[0], float).copy()
        eng.head_yaw0 = float(yaw)
        eng.head_d, eng.head_dyaw = np.zeros(3), 0.0
        eng.head_last = (eng.head_p0.copy(), eng.head_yaw0)
        eng.operator = F.OperatorFrame(eng.head_p0, eng.head_yaw0)
        self.offset = np.zeros(3)
        self.v_shaped = np.zeros(3)
        eng.ee_anchor = eng.robot.to_local(*eng.target)
        eng.ee_rel = eng.ee_anchor
        eng.hand_anchor = eng.operator.pose(*hp)
        eng.hand_delta = F.delta(eng.hand_anchor, eng.hand_anchor)
        eng.rebase_hand = False
        eng.head_fit.reset()
        if eng.hint_source != "head":             # a body source: a new segment from zero
            eng.seg_src, eng.seg_acc, eng.head_yaw_unwrapped = None, np.zeros(3), 0.0
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

    def _shape(self, v, dt: float) -> list:
        """Engaged: the stick velocity ramped toward ``v`` at ``stick_accel`` (the translation, as a vector) and
        ``stick_alpha`` (the yaw); 0 = that part unshaped. Never clipped by the robot's state: only the rate of change
        is limited, so a stick held always reaches its full velocity."""
        cfg, s, v = self.cfg, self.v_shaped, np.asarray(v, float)
        d = v[:2] - s[:2]
        n, cap = float(np.linalg.norm(d)), cfg.stick_accel * dt
        s[:2] = v[:2] if cfg.stick_accel <= 0 or n <= cap else s[:2] + d * (cap / n)
        cap = cfg.stick_alpha * dt
        s[2] = v[2] if cfg.stick_alpha <= 0 else s[2] + min(max(v[2] - s[2], -cap), cap)
        return [float(x) for x in s]

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

    def on_frame(self, frame: dict, verdict, robot: Optional[RobotState], now: float,
                 body: Optional["B.Body"] = None) -> None:
        if not verdict.accept:
            return
        if robot is not None:
            self.last_robot = robot
        self._tracked = False
        cands = self.sources.update(frame, body) if verdict.valid else None
        if cands is not None:
            sel = cands.get(self.cfg.hint_source) if self.cfg.hint_source != "head" else None
            self.status.hint_confidence = (cands["chest"].confidence if cands["chest"].confidence is not None
                                           else cands["hips"].confidence)
            self.status.hint_reason = sel.reason if sel is not None and not sel.ok else ""
        self._engagement(frame, verdict, robot, now)
        if self.valid and self.frame is not None:
            self._gates(frame)
            self._sticks(frame, robot, now)
            self._clicks(frame)
        if self._tracked and self.eng is not None:
            # after the sticks: the EE target and the base hint use the same (this frame's) offset
            self._track_offset(frame)
        self._track_swivel(verdict, body, now)
        eng = self.eng
        self.status.tether = None if eng is None or not eng.tether_on else {
            "xy": eng.tether["xy"], "yaw": eng.tether["yaw"], "lead_m": round(float(np.hypot(*eng.tether_s[:2])), 4),
            "lead_rad": round(float(eng.tether_s[2]), 4)}
        self.status.hint_in_force = None if eng is None else (
            ("head" if eng.head_tracked else None) if eng.hint_source == "head" else eng.hint_in_force)
        if cands is not None and self.on_hint is not None:
            self.on_hint({"t_rx": now, "session_id": frame.get("session_id"), "frame_seq": frame.get("frame_seq"),
                          "sample_time_ns": frame.get("sample_time_ns"), "selected": self.cfg.hint_source,
                          "engaged_source": eng.hint_source if eng is not None else None,
                          "in_force": self.status.hint_in_force,
                          "candidates": {n: c.record() for n, c in cands.items()}})

    def _track_swivel(self, verdict, body: Optional["B.Body"], now: float) -> None:
        """The elbow-swivel input (swivel.py): this hand's arm from the body block, every valid frame; while
        engaged, the swivel change since the engagement (a new ``seq`` at each capture). Computed whether or not the
        hint is sent (the page shows it before it is switched on)."""
        self.arm = SW.arm_sample(body, self.cfg.hand) if verdict.valid else None
        eng = self.eng
        if eng is None:
            self.swivel.stop()
            return
        if eng.swivel_seq is None:
            eng.swivel_seq = self.seqs.next(now)
            eng.swivel_on = bool(self.cfg.elbow_hint)
            self.swivel.start(eng.swivel_seq)
        if self.arm is not None:
            self.swivel.update(self.arm)

    def swivel_out(self) -> Optional[dict]:
        """The datagram's ``swivel`` object for this hand's paired EE (swivel.py), or None: not engaged, no valid
        arm in the newest frame, or no elbow direction yet since the engagement. Whether it is sent is
        ``Config.elbow_hint``."""
        a, sw = self.arm, self.swivel
        if self.eng is None or not sw.active or sw.seq != self.eng.swivel_seq or a is None or not a.valid \
                or sw.ref is None:
            return None
        return {"seq": sw.seq, "dpsi": round(self.swivel_sign * sw.dpsi, 5) + 0.0, "elbow": round(a.elbow_angle, 4),
                "conf": round(a.conf, 3)}

    def swivel_info(self) -> dict:
        """The page's and ``/state``'s view of this hand's elbow-swivel hint (the paired EE's)."""
        a, sw, out = self.arm, self.swivel, self.swivel_out()
        r = self.last_robot
        info = {"hand": self.cfg.hand, "enabled": bool(self.cfg.elbow_hint), "engaged": self.eng is not None,
                "pending": bool(self.cfg.elbow_hint and self.eng is not None and not self.eng.swivel_on),
                "seq": sw.seq, "straight": sw.straight if sw.active else None,
                "dpsi": out["dpsi"] if out else (round(self.swivel_sign * sw.dpsi, 5) if sw.ref is not None else None),
                "sending": bool(self.swivel_sent), "robot_swivel": r.swivel if r is not None else None,
                "robot_swivel_w": r.swivel_w if r is not None else None}
        info.update(a.summary() if a is not None else {"valid": False, "reason": "no valid frame", "elbow": None,
                                                       "conf": None, "tracked": None})
        return info

    def _sticks(self, frame: dict, robot: Optional[RobotState], now: float) -> None:
        ns = frame.get("sample_time_ns")
        dt = 0.0
        if isinstance(ns, int) and self._last_ns is not None:
            dt = min(max(0.0, (ns - self._last_ns) * 1e-9), self.cfg.max_dt)
        self._last_ns = ns if isinstance(ns, int) else None
        if self.eng is None or dt <= 0 or self.eng.captured_now:
            return                       # the capture frame: the offset stays zero (engaging never moves the robot)
        v = [x * self.cfg.stick_speed_frac for x in self.stick_v(frame["control_intent"])]
        if not self.eng.tether_on:           # the ramp (``_shape``) is the untethered alternative; tethered, the raw
            v = self._shape(v, dt)           # stick goes to the base and the tether bounds the reference instead
        base = self._commanded_base(now) if self.eng.tether_on else None
        if self.eng.tether_on and base is None:
            return                       # tethered, no fresh robot state: the offset freezes (the increment is lost)
        h0 = np.array(self._hint_pose()) if base is not None else None   # before this frame's stick increment
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
        if base is not None:
            self._tether(v, base, h0)

    # -- the stick tether (module docstring "The stick tether") ------------------------------------

    def _commanded_base(self, now: float) -> Optional[np.ndarray]:
        """The WBC's commanded base (x, y, yaw, world) predicted to when the WBC applies this frame's command: the
        stream row's pose advanced by its commanded twist over the row's age plus ``ride_lead`` (as the released
        riding target). None when there is no row or it is older than ``robot_stale_s``."""
        r = self.last_robot
        if r is None or now - r.t_recv > self.cfg.robot_stale_s:
            return None
        dt = max(0.0, now - r.t_recv) + self.cfg.ride_lead
        vx, vy, w = r.base_twist
        c, s_ = math.cos(r.base_yaw), math.sin(r.base_yaw)
        return np.array([r.base_x + (c * vx - s_ * vy) * dt, r.base_y + (s_ * vx + c * vy) * dt,
                         F.wrap(r.base_yaw + w * dt)])

    @staticmethod
    def _diff(a, b) -> np.ndarray:
        """Planar poses ``a - b``: (x, y) world, yaw wrapped."""
        return np.array([a[0] - b[0], a[1] - b[1], F.wrap(a[2] - b[2])])

    def _walk_velocity(self) -> np.ndarray:
        """The walking part of the hint velocity (world vx, vy, omega): the head's (or body source's) fit, mapped;
        zero when not walking or the head is lost. Feedforward on or off."""
        eng = self.eng
        if not (self.cfg.head_walk and eng.head_tracked) or eng.head_fit is None:
            return np.zeros(3)
        hv = eng.head_fit.velocity(3)
        hv[:2] = self._map_rot() @ hv[:2]
        return hv

    def _base_moving(self, walk) -> tuple:
        """(translation, yaw): the commanded base moves other than the walking hint asks (the stick's motion)."""
        r = self.last_robot
        vx, vy, w = r.base_twist
        v = F.rot_z(r.base_yaw)[:2, :2] @ np.array([vx, vy])
        return (float(np.hypot(v[0] - walk[0], v[1] - walk[1])) > self.cfg.tether_settle_v,
                abs(w - walk[2]) > self.cfg.tether_settle_w)

    def _shift(self, d_xy) -> None:
        """Move the base reference by a world translation through the stick channel only."""
        if self.eng.mode == "first":
            self.eng.correction[:2] += d_xy
        else:
            self.offset[:2] += d_xy

    def _turn(self, d_yaw: float) -> None:
        """Turn the base reference about its own position through the stick channel only."""
        if self.eng.mode == "first":
            c = self.eng.correction
            x, y, _ = self._hint_pose()
            pivot = np.array([x, y])
            c[:2] = F.rot_z(d_yaw)[:2, :2] @ (c[:2] - pivot) + pivot
            c[2] += d_yaw
        else:
            self.offset[2] += d_yaw

    def _tether(self, v, base, h0) -> None:
        """After this frame's stick increment: per channel (translation, yaw) its state; the lead split into walking's
        and the stick's shares (walking's first credited with the base's progress); the stick's share projected into
        the bound (deflected: ``tether_pos`` / ``tether_yaw``; braking: zero; idle: nothing) through the stick channel
        only; link 2 keeps the stick's yaw share out (``arm_yaw``). ``h0``: the reference before the increment."""
        eng, cfg = self.eng, self.cfg
        moving = self._base_moving(self._walk_velocity())
        tracked = eng.tether_h is not None and eng.tether_b is not None
        w = eng.tether_w
        if tracked:
            # walking's share: + the reference's own motion since the last step (the head's, the stick held), - the
            # base's progress towards it (only up to the share: never past it, never away from it)
            w += self._diff(h0, eng.tether_h)
            w[2] = F.wrap(w[2])
            db = self._diff(base, eng.tether_b)
            for ix in ([0, 1], [2]):
                n = float(np.linalg.norm(w[ix]))
                if n > 1e-12:
                    u = w[ix] / n
                    w[ix] -= u * min(n, max(0.0, float(db[ix] @ u)))
        bound = {}
        for k, (ch, ix) in enumerate((("xy", [0, 1]), ("yaw", [2]))):
            deflected = (v[0] != 0 or v[1] != 0) if ch == "xy" else v[2] != 0
            prev = eng.tether[ch]
            new = "active" if deflected else "braking" if prev == "active" or (prev == "braking" and moving[k]) \
                else "idle"
            if (prev == "idle" or not tracked) and new == "active":   # the stick starts: the lead so far is walking's
                w[ix] = self._diff(h0, base)[ix]
            if new == "active":
                bound[ch] = cfg.tether_pos if ch == "xy" else cfg.tether_yaw
            elif prev != "idle":                              # braking, and the frame it settles: onto the base
                bound[ch] = 0.0
            eng.tether[ch] = new
        share = self._diff(self._hint_pose(), base) - w
        share[2] = F.wrap(share[2])
        if "xy" in bound:
            n, r = float(np.hypot(share[0], share[1])), bound["xy"]
            if n > r:
                corr = share[:2] * (r / n - 1.0)
                self._shift(corr)
                share[:2] += corr
        if "yaw" in bound:
            r = bound["yaw"]
            y = min(r, max(-r, float(share[2])))
            if y != share[2]:
                self._turn(y - share[2])
            share[2] = y
            eng.arm_yaw = y
        for ch, ix in (("xy", [0, 1]), ("yaw", [2])):
            if eng.tether[ch] == "idle":
                share[ix] = 0.0                               # idle: no stick share (the offset is frozen)
        if eng.tether["xy"] == eng.tether["yaw"] == "idle":
            eng.tether_h = eng.tether_b = None                # not tracked while idle: restarts with the stick
        else:
            eng.tether_h, eng.tether_b = np.array(self._hint_pose()), np.asarray(base, float).copy()
        eng.tether_s = share

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
        eng.hint_source = self.cfg.hint_source if self.cfg.hint_source in HINT_SOURCES else "head"
        eng.head_last = (eng.head_p0.copy(), eng.head_yaw0)
        eng.hand_fit = VelocityFit(self.cfg.ff_window, (self.cfg.ff_max_speed, self.cfg.ff_max_omega))
        eng.head_fit = VelocityFit(self.cfg.ff_window, (self.cfg.hint_max_speed, self.cfg.hint_max_omega))
        self.offset = np.zeros(3)
        self.v_shaped = np.zeros(3)
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
        eng.tether_on = bool(self.cfg.tether)
        if eng.tether_on:
            # the base still moving from the stick (driven released): braking from the start, the offset follows it
            vb = F.rot_z(robot.base_yaw)[:2, :2] @ np.array(robot.base_twist[:2])
            if float(np.hypot(*vb)) > self.cfg.tether_settle_v:
                eng.tether["xy"] = "braking"
            if abs(robot.base_twist[2]) > self.cfg.tether_settle_w:
                eng.tether["yaw"] = "braking"
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
        if self.cfg.head_walk and eng.hint_source != "head":
            self._track_body(frame)
        elif self.cfg.head_walk:
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

    def _track_body(self, frame: dict) -> None:
        """Link 1 (and link 2's frame) from a body source, or the head as its fallback (module docstring): the
        total displacement and yaw since the engagement, continued across source switches and losses by segments."""
        eng, cfg = self.eng, self.cfg
        cands = self.sources.latest
        want = cands.get(eng.hint_source)
        ns = frame.get("sample_time_ns")
        t = ns * 1e-9 if isinstance(ns, int) else None
        body_ok = want is not None and want.ok
        if not body_ok:
            eng.body_ok_since = None
        elif eng.body_ok_since is None:
            eng.body_ok_since = t
        back = eng.hint_in_force in (None, eng.hint_source) or t is None or \
            (eng.body_ok_since is not None and t - eng.body_ok_since >= cfg.hint_return_s)
        src = eng.hint_source if body_ok and back else "head" if cands["head"].ok else \
            eng.hint_source if body_ok else None
        if src is None:                           # nothing usable: hold the contribution so far, no velocity
            if eng.head_tracked:
                eng.head_fit.reset()
            eng.head_tracked, eng.seg_src = False, None
            return
        smp = cands[src]
        p, yaw = np.asarray(smp.f_p, float), float(smp.f_yaw)
        if eng.seg_src != src:                    # a new segment from where the total is (no jump)
            eng.seg_acc = np.array([eng.head_d[0], eng.head_d[1], eng.head_yaw_unwrapped])
            eng.seg_src, eng.seg_p0, eng.seg_yaw0 = src, p.copy(), yaw
            if eng.hint_in_force is not None and src != eng.hint_in_force:
                self.status.hint_switches += 1
            eng.hint_in_force = src
        d = F.rot_z(-eng.head_yaw0) @ F.C @ (p - eng.seg_p0)
        eng.head_d = np.array([eng.seg_acc[0] + d[0], eng.seg_acc[1] + d[1], 0.0])
        eng.head_yaw_unwrapped = float(eng.seg_acc[2] + yaw - eng.seg_yaw0)
        eng.head_dyaw = F.wrap(eng.head_yaw_unwrapped)
        eng.head_last = (eng.head_p0 + F.C.T @ F.rot_z(eng.head_yaw0) @ eng.head_d,
                         eng.head_yaw0 + eng.head_yaw_unwrapped)
        eng.head_tracked = True

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
        if self.eng is None or self.eng.mode != "third":
            return 0.0
        # tethered: the stick's yaw share is kept out (link 2's frame has the commanded base's yaw)
        return float(self.offset[2]) - self.eng.arm_yaw if self.eng.arm_yaw else float(self.offset[2])

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
        if eng.arm_yaw:
            # tethered: C turned back by the stick's yaw share about the base reference, so link 2's frame has the
            # commanded base's yaw: x -> R(-a) (C x - P) + P
            a = eng.arm_yaw
            hx, hy, _ = self._hint_pose()
            pv = np.array([hx, hy, 0.0])
            ra = F.rot_z(-a)
            rz = F.rot_z(c[2] - a)
            return rz @ p + ra @ (np.array([c[0], c[1], 0.0]) - pv) + pv, rz @ r
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
        if self.frame is None:
            sv = [0.0, 0.0, 0.0]
        elif eng.tether_on:                  # tethered: the raw stick to the base (never shaped or clipped)
            sv = self.stick_v(self.frame["control_intent"])
        else:                                # untethered: the ramped stick (``_shape``)
            sv = self.v_shaped
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
        self.arm = None
        self.swivel.stop()
        self.status.state = "stale"

    # -- command ---------------------------------------------------------------------------------

    def command(self, now: float, input_fresh: bool, robot: Optional[RobotState]):
        if not input_fresh:
            if self.frame is not None or self.eng is not None:
                self.stale()
            self.swivel_sent = False
            return None
        if not self.valid or self.frame is None:
            self.swivel_sent = False
            return None
        cfg = self.cfg
        ee: dict
        if self.eng is not None and self.eng.target is not None:
            p, r = self.eng.target
            ee = {"frame": "world", "pos": _l(p), "quat": _l(F.quat_from_matrix(r))}
            if cfg.feedforward:
                ee["twist"] = _l(self.feedforward()[0])
            # switched on: from the next engagement (the WBC anchors the robot's swivel at a new seq); off: at once
            sw = self.swivel_out() if cfg.elbow_hint and self.eng.swivel_on else None
            if sw is not None:
                ee["swivel"] = sw
            self.swivel_sent = sw is not None
            self.last_target = (p, r)
            reset = self.eng.est_reset            # the estimator epoch the anchor was captured in
        else:
            self.swivel_sent = False
            if robot is None:
                return "alive"
            ee = self.riding(now, robot)
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

    def riding(self, now: float, robot: RobotState) -> dict:
        """Released: the EE target that rides with the platform (the command at release, relative to the base), on
        the base pose predicted to when the WBC applies it (the stream row's age plus ride_lead, by the commanded
        twist), so the target does not trail a moving base."""
        if self.hold_base is None:
            self.hold_base = robot.base.to_local(robot.ee_p, robot.ee_r)
        dt = max(0.0, now - robot.t_recv) + self.cfg.ride_lead
        vx, vy, w = robot.base_twist
        c, s_ = math.cos(robot.base_yaw), math.sin(robot.base_yaw)
        base = F.PlanarFrame(robot.base_x + (c * vx - s_ * vy) * dt, robot.base_y + (s_ * vx + c * vy) * dt,
                             robot.base_yaw + w * dt)
        p, r = base.to_world(*self.hold_base)
        return {"frame": "world", "pos": _l(p), "quat": _l(F.quat_from_matrix(r)), "twist": robot.twist_at(p)}

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


class HandConfig:
    """One hand's view of the shared ``Config``: its own ``hand``, ``ee`` (the paired arm) and ``mirrored``; every
    other key read (and written) live on the shared one, so the page's mode / headless / top speed apply to both
    hands."""

    OWN = ("hand", "ee", "mirrored")

    def __init__(self, base: Config, hand: str, ee: str):
        object.__setattr__(self, "_base", base)
        object.__setattr__(self, "hand", hand)
        object.__setattr__(self, "ee", ee)
        object.__setattr__(self, "mirrored", False)

    def __getattr__(self, k):
        return getattr(self._base, k)

    def __setattr__(self, k, v):
        if k in self.OWN:
            object.__setattr__(self, k, v)
        else:
            setattr(self._base, k, v)


class MultiCore:
    """Two hands, two arms (the module docstring): one ``TeleopCore`` per hand of ``cfg.hands``, the base from the
    first-engaged hand. The same interface as ``TeleopCore``; ``on_frame`` / ``command`` take the robot per EE
    (``{ee: RobotState}``)."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.cores = {h: TeleopCore(HandConfig(cfg, h, ee)) for h, ee in (cfg.hands or {}).items()}
        if len(self.cores) < 1:
            raise ValueError("MultiCore needs Config.hands")
        seqs = SW.SeqCounter()                    # one swivel seq sequence for both hands (swivel.py)
        for c in self.cores.values():
            c.seqs = seqs
        self.owner: Optional[str] = None          # the hand whose engagement drives the base
        self._order: dict = {}                    # hand -> when its engagement started (engagement order)
        self._n = 0
        if getattr(cfg, "mirror", "off") not in MIRROR_MODES:
            raise ValueError(f"mirror must be one of {MIRROR_MODES}, not {cfg.mirror!r}")
        self.straight = {h: c.cfg.ee for h, c in self.cores.items()}   # Config.hands: the straight pairing
        self.mirrored = False                     # the pairing in force (locked while any hand is engaged)
        self.decision: Optional[dict] = None      # the newest locked decision (the ``pairing`` event's detail)
        self.events: list = []                    # ("pairing", detail) for the client's on_event; drained by it
        self._apply_forced()

    @property
    def ees(self) -> list:
        return [c.cfg.ee for c in self.cores.values()]

    def _primary(self) -> TeleopCore:
        return self.cores.get(self.cfg.hand) or next(iter(self.cores.values()))

    def _lead(self) -> TeleopCore:
        """The core the base follows: the owner, else the primary."""
        return self.cores[self.owner] if self.owner is not None else self._primary()

    # -- the TeleopCore interface -------------------------------------------------------------------------

    @property
    def eng(self) -> Optional[Engagement]:
        return self._lead().eng

    @property
    def hint(self):
        return self._lead().hint

    @property
    def offset(self):
        return self._lead().offset

    @property
    def frame(self):
        return next((c.frame for c in self.cores.values() if c.frame is not None), None)

    @property
    def engaged(self) -> list:
        return [h for h, c in self.cores.items() if c.eng is not None]

    @property
    def status(self) -> Status:
        cs = list(self.cores.values())
        st = Status()
        states = [c.status.state for c in cs]
        st.state = next((x for x in ("engaged", "released", "needs_release", "stale", "waiting") if x in states),
                        states[0])
        st.reason = "; ".join(f"{h}: {c.status.reason}" for h, c in self.cores.items() if c.status.reason)
        st.engagements = sum(c.status.engagements for c in cs)
        for c in cs:
            for k, v in c.status.refused.items():
                st.refused[k] = st.refused.get(k, 0) + v
        st.registered = any(c.status.registered for c in cs)
        st.registration = self._primary().status.registration
        lead = self._lead().status                # the base hint: the core the base follows
        st.hint_in_force, st.hint_confidence, st.hint_reason = lead.hint_in_force, lead.hint_confidence, lead.hint_reason
        st.hint_switches = sum(c.status.hint_switches for c in cs)
        st.tether = lead.tether
        return st

    @property
    def on_hint(self):
        return self._primary().on_hint

    @on_hint.setter
    def on_hint(self, fn) -> None:
        """The hint candidates' log: from the primary core only (every core sees the same frame and body)."""
        self._primary().on_hint = fn

    def swivel_info(self) -> dict:
        """Per EE (the pairing in force): its hand's elbow-swivel input and hint (``TeleopCore.swivel_info``)."""
        return {c.cfg.ee: c.swivel_info() for c in self.cores.values()}

    def hands_status(self) -> dict:
        """Per hand: its EE, state, engagements, whether it drives the base."""
        return {h: {"ee": c.cfg.ee, "state": c.status.state, "engagements": c.status.engagements,
                    "base": h == self.owner, "mirrored": c.mirrored} for h, c in self.cores.items()}

    # -- the mirror pairing (the module docstring) -------------------------------------------------------

    @property
    def pairing(self) -> dict:
        """{hand: ee} in force."""
        return {h: c.cfg.ee for h, c in self.cores.items()}

    def can_mirror(self) -> bool:
        return set(self.straight) == {"left", "right"}

    def _mirror_map(self) -> dict:
        s = self.straight
        return {"left": s["right"], "right": s["left"]} if self.can_mirror() else dict(s)

    def pairing_info(self) -> dict:
        """The pairing in force, for ``/state`` and consumers: the mode, straight / mirror, {hand: ee}, locked (a
        hand engaged: it holds until both are released), per arm the mirror flag and the elbow-swivel sign, and the
        newest decision."""
        by_ee = {c.cfg.ee: c for c in self.cores.values()}
        return {"mode": self.cfg.mirror, "pairing": "mirror" if self.mirrored else "straight",
                "hands": self.pairing, "locked": bool(self.engaged),
                "mirror": {e: c.mirrored for e, c in by_ee.items()},
                "decision": self.decision}

    def _set_pairing(self, mirrored: bool) -> None:
        """Swap which arm each hand drives (released only). The per-arm state (the riding pose ``hold_base``, the
        newest row, the last target) moves with its arm, so a released arm keeps riding exactly where it was."""
        mirrored = bool(mirrored) and self.can_mirror()
        if mirrored == self.mirrored:
            return
        target = self._mirror_map() if mirrored else self.straight
        per_arm = {c.cfg.ee: (c.hold_base, c.last_robot, c.last_target) for c in self.cores.values()}
        for h, c in self.cores.items():
            c.cfg.ee = target[h]
            c.cfg.mirrored = mirrored
            c.hold_base, c.last_robot, c.last_target = per_arm[target[h]]
        self.mirrored = mirrored

    def _apply_forced(self) -> None:
        """off / straight / mirror take effect at once (released); auto waits for the next engagement."""
        if self.cfg.mirror != "auto":
            self._set_pairing(self.cfg.mirror == "mirror")

    def set_mirror(self, mode: str) -> dict:
        """The pairing mode (``MIRROR_MODES``). Only while no hand is engaged (ValueError otherwise, nothing
        changed): a pairing never changes under an engaged hand."""
        if mode not in MIRROR_MODES:
            raise ValueError(f"mirror must be one of {', '.join(MIRROR_MODES)}, not {mode!r}")
        if mode != "off" and not self.can_mirror():
            raise ValueError("mirror pairing needs a left and a right hand")
        if self.engaged:
            raise ValueError(f"a hand is engaged ({', '.join(self.engaged)}): release both to change the pairing")
        self.cfg.mirror = mode
        self._apply_forced()
        return self.pairing_info()

    def operator_yaw(self, frame: dict) -> Optional[float]:
        """The operator's gravity-aligned heading in the headset's room (LOCAL): the head's (the scheme's torso
        proxy); None when not tracked or undefined."""
        head = head_pose(frame)
        return F.head_yaw(head[1]) if head is not None else None

    def heading_diff(self, frame: dict, robot, hand: str) -> Optional[float]:
        """The operator's heading minus the robot base's (rad, wrapped), both in the robot's world: the head's yaw
        through ``hand``'s registration. 0 when there is none (it is made at this clutch, facing where the robot
        faces) or in first person; None without a head heading or a stream row."""
        rows = [r for r in (_robot_for(robot, e) for e in self.straight.values()) if r is not None]
        yaw = self.operator_yaw(frame)
        if not rows or yaw is None:
            return None
        c = self.cores[hand]
        if self.cfg.mode == "first" or c.reg is None or c.reg_reset != rows[0].est_reset:
            return 0.0
        return F.wrap(float(c.reg[2]) + yaw - rows[0].base_yaw)

    def _engaging(self, frame: dict, verdict) -> list:
        """The hands whose engagement starts on this frame (as ``TeleopCore._engagement`` will see it), when no hand
        is engaged: the moment the pairing is decided."""
        if self.engaged or not (verdict.accept and verdict.valid) or verdict.discontinuity:
            return []
        hands = ((frame.get("control_intent") or {}).get("hands")) or {}
        return [h for h, c in self.cores.items()
                if not c.needs_release and (hands.get(h) or {}).get("valid") and (hands.get(h) or {}).get("engaged")]

    def _decide(self, frame: dict, robot, hands: list) -> Optional[dict]:
        """Before the cores capture: pair by the mode (and, auto, the heading difference). Returns the decision."""
        mode = self.cfg.mirror
        if mode == "off" or not self.can_mirror():
            return None
        diff = self.heading_diff(frame, robot, hands[0])
        if mode == "auto":
            m = diff is not None and abs(diff) > MIRROR_FACING
        else:
            m = mode == "mirror"
        self._set_pairing(m)
        return {"pairing": "mirror" if m else "straight", "mode": mode,
                "heading_diff_deg": None if diff is None else round(math.degrees(diff), 1),
                "hand": hands[0], "hands": self.pairing}

    def on_arm(self, frame: Optional[dict] = None) -> None:
        for c in self.cores.values():
            c.on_arm(frame)
        self.owner, self._order = None, {}

    def on_disarm(self, reason: str = "disarmed") -> None:
        for c in self.cores.values():
            c.on_disarm(reason)
        self.owner, self._order = None, {}

    def on_resume(self, frame: Optional[dict] = None) -> None:
        for c in self.cores.values():
            c.on_resume(frame)

    @property
    def captured_now(self) -> bool:
        return any(c.captured_now for c in self.cores.values())

    def drop(self, reason: str) -> None:
        for c in self.cores.values():
            c.drop(reason)
        self.owner, self._order = None, {}

    def stale(self, reason: str = "stale") -> None:
        for c in self.cores.values():
            c.stale(reason)
        self.owner, self._order = None, {}

    def on_frame(self, frame: dict, verdict, robot, now: float, body: Optional["B.Body"] = None) -> None:
        engaging = self._engaging(frame, verdict)
        before = self.mirrored
        decision = self._decide(frame, robot, engaging) if engaging else None
        for h, c in self.cores.items():
            c.on_frame(frame, verdict, _robot_for(robot, c.cfg.ee), now, body)
        self._owner()
        if decision is not None:
            if self.engaged:                       # locked until both hands are released
                self.decision = decision
                self.events.append(("pairing", decision))
            else:                                  # refused: nothing started, the previous pairing stays
                self._set_pairing(before)

    def _owner(self) -> None:
        engaged = self.engaged
        for h in list(self._order):
            if h not in engaged:
                del self._order[h]
        for h in engaged:
            if h not in self._order:
                self._n += 1
                self._order[h] = self._n
        if self.owner not in engaged:
            self.owner = min(engaged, key=lambda h: (self._order[h], h != self.cfg.hand)) if engaged else None

    def command(self, now: float, input_fresh: bool, robot):
        outs = {h: c.command(now, input_fresh, _robot_for(robot, c.cfg.ee)) for h, c in self.cores.items()}
        if not input_fresh:
            return None
        ees: dict = {}
        lead = self.owner if self.owner is not None else None
        body = None
        for h, o in outs.items():
            c = self.cores[h]
            if isinstance(o, dict):
                ees.update(o["ees"])
                if body is None or h == lead or (lead is None and c is self._primary()):
                    body = o
            else:                                  # this hand invalid / no command: its arm rides
                r = _robot_for(robot, c.cfg.ee)
                if r is not None:
                    ees[c.cfg.ee] = c.riding(now, r)
        if body is None:                           # no hand gives a command
            return None if all(o is None for o in outs.values()) and not ees else "alive"
        out = {"ees": ees, "base": body["base"]}
        if "est_reset" in body:
            out["est_reset"] = body["est_reset"]
        return out


def _robot_for(robot, ee: str):
    return robot.get(ee) if isinstance(robot, dict) else robot


def make_core(cfg: Config):
    """``MultiCore`` with ``cfg.hands`` (two hands), else ``TeleopCore``."""
    return MultiCore(cfg) if cfg.hands else TeleopCore(cfg)


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


def _opt_float(x) -> Optional[float]:
    return float(x) if _finite(x) else None


def _finite(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _l(a) -> list:
    return [float(x) for x in a]
