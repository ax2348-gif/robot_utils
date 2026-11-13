"""Response models of the arm HTTP API (server.py declares them as ``response_model``).

The handlers return plain dicts (node.py, kin.py, config.py); these models describe them and pass them
through unchanged: fields are declared in the order the dicts are built, keys that are sometimes absent
are optional and the routes use ``response_model_exclude_unset``. Frames: the arm frame is the arm's
URDF ``base_link`` (the underside of its mount plate on joint 1's axis, axes parallel to the robot base;
GET /geometry `frames.arm_in_base` gives its pose in the robot base frame); the robot base frame is the
description node's ``base``. Positions in m, angles in rad unless a name says otherwise (`_deg`, `_mm`).
Time stamps are server time.monotonic() s (the clock of GET /time `mono`).
"""
from __future__ import annotations

from typing import Literal, Optional

from pydantic import BaseModel, Field, create_model

from ..common.realtime import RealtimeOut
from ..common.server import EStopState, Event, LockOut
from .node import FIELD_DOC, FIELDS, PHASES, SOURCES, STATES, UNITS

ArmState = Literal[tuple(STATES)]
Source = Literal[tuple(SOURCES)]
Phase = Literal[tuple(PHASES)]
STATE_DOC = ("connecting (no bus yet) | uninitialized (connected, never enabled) | initializing (the first enable's "
             "gripper check) | off (motors off: damping) | disabled (motors on, commands refused) | floating (enabled, "
             "gravity compensated, not servoing) | servoing (holding or following a command) | fault (see `error`; "
             "POST /reset).")

# -- poses and IK --------------------------------------------------------------------------------


class PoseOut(BaseModel):
    """A pose: position and orientation, both as a quaternion and as roll / pitch / yaw. The frame is
    given where the pose appears."""
    pos: list[float] = Field(description="[x, y, z] m.")
    quat: list[float] = Field(description="Orientation as a unit quaternion [x, y, z, w].")
    rpy: list[float] = Field(description="The same orientation as [roll, pitch, yaw] rad, fixed X-Y-Z.")


class IKResult(BaseModel):
    """An IK check (nothing moves)."""
    ok: bool = Field(description="A solution within the tolerances (2 mm; the orientation within `max_rot_deg`, "
                                 "default 10°) and the joint limits was found.")
    exact: bool = Field(description="ok, and the orientation also within 1°.")
    q: list[float] = Field(description="Joint angles of the solution (6, rad); when not ok, the closest found.")
    pos_err_mm: float = Field(description="Distance from the requested TCP position to where `q` puts the TCP (mm).")
    rot_err_deg: float = Field(description="Angle between the requested TCP orientation and `q`'s (degrees).")
    within_limits: bool = Field(description="`q` is within every joint's limits.")
    joints_at_limit: list[str] = Field(description="Joints of `q` at (within 0.5° of) or past a limit, e.g. "
                                                   "`joint2`; empty if none.")
    iterations: int = Field(description="Solver iterations used.")
    reason: Optional[str] = Field(description="Why it is not ok, and how to adjust the request; null when ok.")
    nearest: Optional[PoseOut] = Field(description="The TCP pose `q` reaches (arm frame): reachable as is, so a "
                                                   "request can be retried with it.")


class RenderView(BaseModel):
    """The view a picture was drawn with: the request's `render` over the defaults."""
    azimuth_deg: float = Field(description="Direction the view looks from, about +z: 0 = from +x, 90 = from +y.")
    elevation_deg: float = Field(description="0 = level, 90 = from straight above, negative = from below.")
    zoom: float = Field(description="1 = the drawing fits the image; > 1 magnifies about its centre.")
    width: int = Field(description="Image width, px.")
    height: int = Field(description="Image height, px.")


class IKOut(IKResult):
    """POST /ik. `image` and `view` only when the request had `render`."""
    image: Optional[str] = Field(None, description="The query drawn as a PNG, as a data URL "
                                                   "(`data:image/png;base64,...`). Only with `render`.")
    view: Optional[RenderView] = Field(None, description="The view `image` was drawn with. Only with `render`.")


# -- kinematics ----------------------------------------------------------------------------------


class KinematicsOut(BaseModel):
    """Kinematics at one joint configuration (computation only)."""
    q: list[float] = Field(description="The joint angles (6, rad).")
    within_limits: bool = Field(description="`q` is within every joint's limits.")
    tcp: PoseOut = Field(description="TCP pose at `q`, arm frame.")
    frame: Literal["arm", "tcp"] = Field(description="Axes of the Jacobian's rows: the arm frame's (`arm`) or the "
                                                     "TCP's own (`tcp`). A request for `base` answers `arm`.")
    jacobian: list[list[float]] = Field(description="6x6 TCP Jacobian: row r is [vx, vy, vz, wx, wy, wz][r] (m/s, "
                                                    "rad/s of the TCP, in `frame`'s axes), column c is joint c+1 "
                                                    "(per rad/s).")
    w: float = Field(description="Manipulability sqrt(det(J Jᵀ)) of the whole Jacobian: 0 at a singularity.")
    w_lin: float = Field(description="The same for its translational rows only.")
    sigma_min: float = Field(description="Smallest singular value of J: how near a singularity (0 = at one).")
    sigma: list[float] = Field(description="All 6 singular values of J, largest first.")
    lin_sigma: list[float] = Field(description="The 3 singular values of the translational rows, largest first: "
                                               "the semi-axes of the TCP velocity ellipsoid (m/s per rad/s).")
    lin_axes: list[list[float]] = Field(description="The ellipsoid's 3 axis directions (unit vectors in `frame`'s "
                                                    "axes), in the order of `lin_sigma`.")


class KinematicsNow(KinematicsOut):
    """GET /kinematics."""
    which: Literal["measured", "command"] = Field(description="`measured`: at the joint readings; `command`: at the "
                                                              "joint command (only while servoing).")


class KinematicsBatch(BaseModel):
    """POST /kinematics."""
    results: list[KinematicsOut] = Field(description="One per requested configuration, in order.")


class LinksOut(BaseModel):
    """GET /links."""
    frames: list[PoseOut] = Field(description="Pose of each frame named in `names`, arm frame, at the measured "
                                              "joint angles.")
    names: list[str] = Field(description="joint1..joint6 (each joint's frame), end_link (the gripper frame: at the "
                                         "fingertips, midway between the jaws) and tcp.")


# -- geometry ------------------------------------------------------------------------------------


class PointBoth(BaseModel):
    """A point in both frames (m)."""
    arm: list[float] = Field(description="[x, y, z] m, arm frame.")
    base: list[float] = Field(description="[x, y, z] m, robot base frame.")


class VectorBoth(BaseModel):
    """A unit direction in both frames."""
    arm: list[float] = Field(description="[x, y, z], arm frame axes.")
    base: list[float] = Field(description="[x, y, z], robot base frame axes.")


class PoseBoth(BaseModel):
    """A pose in both frames."""
    arm: PoseOut = Field(description="In the arm frame.")
    base: PoseOut = Field(description="In the robot base frame.")


class ArmInBase(BaseModel):
    matrix: list[list[float]] = Field(description="4x4 homogeneous transform T: p_base = T p_arm.")
    xyz: list[float] = Field(description="The arm frame's origin in the robot base frame (m).")
    source: str = Field(description="Where the transform came from: the description node's `arm` frame, or the "
                                    "shipped default when that node is unreachable (the text says which).")


class GeomFrames(BaseModel):
    arm_in_base: ArmInBase = Field(description="Pose of the arm frame in the robot base frame.")
    note: str = Field(description="How the two frames relate (fixed text).")


class GeomTcp(PoseBoth):
    """The TCP: the point motion commands target."""
    offset_in_end_link: list[float] = Field(description="Where the TCP sits in end_link (m; default [-0.15, 0, 0]: "
                                                        "0.15 m behind the fingertips).")
    note: str = Field(description="Fixed text.")


class Fingertips(BaseModel):
    left: PointBoth = Field(description="The left jaw's tip on its inner (gripping) face, mid height.")
    right: PointBoth = Field(description="The right jaw's tip, likewise.")
    center: PoseBoth = Field(description="end_link: midway between the tips; x = approach direction, y = jaw "
                                         "travel (left jaw at +y), z = the jaws' height.")
    note: str = Field(description="Fixed text.")


class GeomAxes(BaseModel):
    approach: VectorBoth = Field(description="Out of the gripper, between the jaws (end_link x).")
    jaw_travel: VectorBoth = Field(description="Along the jaws' travel, toward the left jaw (end_link y).")
    up: VectorBoth = Field(description="Across the jaws' height (end_link z).")


class Jaw(BaseModel):
    tip: PointBoth = Field(description="Tip of the jaw on its inner face, mid height.")
    inner_face_center: PointBoth = Field(description="Centre of the inner (gripping) face.")
    inner_face_root: PointBoth = Field(description="Back end of the inner face, where the jaw carriage begins.")
    outer_face_tip: PointBoth = Field(description="Tip of the jaw on its outer face.")
    inward: VectorBoth = Field(description="From this jaw's inner face toward the other jaw.")


class Jaws(BaseModel):
    left: Jaw = Field(description="The jaw at +y of end_link.")
    right: Jaw = Field(description="The jaw at -y of end_link.")


class JawShape(BaseModel):
    """Approximate, from the meshes."""
    grasp_depth: float = Field(description="m: how far an object can sit between the jaws, from the tips back.")
    length: float = Field(description="m: length of a jaw's inner face, tip to root.")
    thickness: float = Field(description="m: thickness of a jaw outward from its inner face.")
    height_root: float = Field(description="m: the jaw's height at its root.")
    height_tip: float = Field(description="m: the jaw's height at its tip.")
    note: str = Field(description="Fixed text.")


class Bounds(BaseModel):
    min: list[float] = Field(description="[x, y, z] m, robot base frame.")
    max: list[float] = Field(description="[x, y, z] m, robot base frame.")


class PastFootprint(BaseModel):
    """m the arm reaches past each side of the robot's square frame (negative: it stays inside by that much)."""
    front: float = Field(description="Past the +x side (m).")
    rear: float = Field(description="Past the -x side (m).")
    left: float = Field(description="Past the +y side (m).")
    right: float = Field(description="Past the -y side (m).")


class Extent(BaseModel):
    arm: Bounds = Field(description="Axis-aligned bounds of every box of the arm, gripper included.")
    gripper: Bounds = Field(description="Axis-aligned bounds of the gripper's boxes (rail, motor block, jaws).")
    past_footprint_m: PastFootprint
    note: str = Field(description="Fixed text (names the frame's side length).")


class Box(BaseModel):
    """One part of the arm as an oriented box (approximate, from the meshes)."""
    name: str = Field(description="The part, e.g. `upper arm`, `left jaw`.")
    link: str = Field(description="What it moves with: `base` (fixed), `link1`..`link5`, `end` (the gripper "
                                  "body) or `finger` (a jaw).")
    center: PointBoth = Field(description="Centre of the box.")
    size_m: list[float] = Field(description="Edge lengths along its three axes (m).")
    axes_base: list[list[float]] = Field(description="The box's three axes (unit vectors, robot base frame), in "
                                                     "the order of `size_m`.")


class GeometryOut(BaseModel):
    """GET / POST /geometry. Every point in both the arm frame and the robot base frame."""
    which: Literal["measured", "q", "ik"] = Field(description="The configuration used: the joint readings, the "
                                                              "request's `q`, or the IK solution of its `pose`.")
    q: list[float] = Field(description="Joint angles used (6, rad).")
    within_limits: bool = Field(description="`q` is within every joint's limits.")
    ik: Optional[IKResult] = Field(description="The IK check of the request's `pose`; null otherwise.")
    gap_mm: float = Field(description="Jaw gap the jaws are placed at (mm).")
    gap_source: str = Field(description="`given` (the request's gap_mm), `measured`, or `unknown (no reading): 0`.")
    frames: GeomFrames
    tcp: GeomTcp
    fingertips: Fingertips
    axes: GeomAxes = Field(description="The gripper's axes.")
    jaws: Jaws
    jaw_shape_m: JawShape
    extent_base: Extent = Field(description="How far the arm reaches, robot base frame.")
    boxes: list[Box] = Field(description="Every part of the arm as an oriented box.")


# -- state ---------------------------------------------------------------------------------------


def _tick_docs() -> dict:
    """GET /fields `doc` spread over the columns it names (j<i> = every joint; `a / b` several columns;
    `a..b` a range of columns)."""
    out = {}

    def resolve(tok, first):
        for cand in (tok, "j<i>_" + tok if first.startswith("j<i>_") else None, first.rsplit("_", 1)[0] + "_" + tok):
            if cand and any(f == cand.replace("<i>", "1") for f in FIELDS):
                return cand
        return None

    for key, doc in FIELD_DOC.items():
        parts = [p.strip() for p in key.split(" / ")]
        names = []
        for p in parts:
            if ".." in p:
                a, b = p.split("..")
                a, b = resolve(a, parts[0]), (resolve(a.rsplit("_", 1)[0] + "_" + b, a) if "_" in a else None) or resolve(b, a)
                if a and b:
                    names += FIELDS[FIELDS.index(a):FIELDS.index(b) + 1]
            elif (n := resolve(p, parts[0])) is not None:
                names.append(n)
        for n in names:
            for i in range(1, 7):
                out.setdefault(n.replace("<i>", str(i)), doc)
    return out


_TICK_EXTRA = {
    "t_tick": "server monotonic time of the tick", "tick": "ticks since the node started",
    "enabled": "1 while enabled", "motors_on": "1 while the motors are on (off = damping)",
    "estop": "1 while the e-stop is latched", "lease_id": "id of the lease in force (0: none)",
    "seq": "`seq` of the track command in force (0: none)", "overruns": "ticks more than 15 ms after the previous one, since the node started",
    "tick_dt": "time since the previous tick", "motion_id": "id of the current or last planned motion",
    "progress": "of that motion: 0..1 over its planned duration", "g_qd": "gripper motor velocity",
    "g_tau": "gripper motor torque reading", "g_tau_est": "gripper torque the MIT law applies",
    "g_force_max": "squeeze force limit in force", "g_limiting": "1 while the force limit holds the squeeze back",
    "g_status": "gripper motor's Damiao status code", **{f"j{i}_t_mos": "motor driver (MOS) temperature" for i in range(1, 7)},
}
_UNIT_WORDS = {"enum": "index (see doc)", "bool": "1 or 0", "count": "count", "id": "id", "": "unitless", "C": "°C"}


def _tick_model():
    docs = _tick_docs()
    fields = {}
    for n, u in zip(FIELDS, UNITS):
        doc = docs.get(n) or _TICK_EXTRA.get(n, "")
        desc = f"{_UNIT_WORDS.get(u, u)}" + (f": {doc}" if doc else "")
        fields[n] = (Optional[float], Field(description=desc + ". null when not available (NaN)."))
    return create_model("TickRow", __doc__="The latest control tick: one stream row, keyed by the column names "
                                           "of GET /fields (same units). Flags are 1 / 0.", **fields)


TickRow = _tick_model()


class FieldsOut(BaseModel):
    """GET /fields: the columns of the binary stream rows."""
    fields: list[str] = Field(description="Column names, in row order.")
    units: list[str] = Field(description="Unit of each column, same order: s, m, rad, rad/s, Nm, N, C (°C), or "
                                         "`bool` (1 / 0), `enum` (an index: see `doc`), `count`, `id`, `` (unitless).")
    doc: dict[str, str] = Field(description="Meanings of the columns, keyed by a column name or a pattern "
                                            "(`j<i>` = joint i, `a / b` = several columns, `a..b` = a range of "
                                            "columns). The same for both channels.")


class GripperOut(BaseModel):
    """The gripper."""
    mode: Literal["holding", "free", "damping"] = Field(description="`holding`: servoing to `goal_mm`; `free`: motor "
                                                                    "on, no gains (after enable, until a gripper "
                                                                    "command); `damping`: motor off (disabled, off).")
    engaged: bool = Field(description="Holding (closed loop on).")
    gap_mm: Optional[float] = Field(description="Measured jaw gap (mm; 0 = closed); null before the first reading.")
    sent_mm: Optional[float] = Field(description="Gap sent to the motor this tick (mm; the force limit may hold it "
                                                 "back from the goal); null unless holding.")
    goal_mm: Optional[float] = Field(description="Gap goal of the last gripper command (mm); null if none yet.")
    max_force_n: float = Field(description="Squeeze force limit in force (N).")
    force_n: Optional[float] = Field(description="Squeeze force applied, estimated (N); null unless holding.")
    demand_n: Optional[float] = Field(description="Squeeze force the position loop asks for before the limit (N); "
                                                  "above max_force_n while limiting; null unless holding.")
    limiting: bool = Field(description="The force limit is holding the squeeze back.")
    gap_max_mm: float = Field(description="Fully open gap (mm).")
    default_max_force_n: float = Field(description="Force limit a command without `max_force_n` gets (N).")


class JointLimits(BaseModel):
    lo: Optional[list[float]] = Field(description="Lower limit per joint (6, rad); null before the model is loaded.")
    hi: Optional[list[float]] = Field(description="Upper limit per joint (6, rad); null before the model is loaded.")


class JointsOut(BaseModel):
    """GET /joints: the latest reading."""
    names: list[str] = Field(description="joint1..joint6, the order of the 6-entry lists.")
    q: list[float] = Field(description="Output encoder angles (rad): the arm's measured pose.")
    q_mot: list[float] = Field(description="Motor-side feedback angles (rad): what each motor's own loop sees.")
    qd: list[float] = Field(description="Motor feedback velocities (rad/s).")
    tau: list[float] = Field(description="Motor feedback torques (Nm).")
    q_cmd: Optional[list[float]] = Field(description="Joint position command (rad); null unless servoing.")
    status: list[float] = Field(description="Damiao status code per motor, 7 entries: joint1..6 then the gripper "
                                            "(1 enabled, 0 disabled, 8 and above a fault).")
    t_mos: list[float] = Field(description="Motor driver (MOS) temperature per motor (°C), 7 entries as `status`.")
    t_rotor: list[float] = Field(description="Motor rotor temperature per motor (°C), 7 entries as `status`.")
    limits: JointLimits = Field(description="Joint limits (rad).")
    t: float = Field(description="When the reading was taken (server monotonic s).")


class TrackingOut(BaseModel):
    """GET /tracking: from the latest control tick. Null where not available (not servoing)."""
    servo: bool = Field(description="The arm is servoing.")
    e_mot: list[Optional[float]] = Field(description="Per joint: command minus motor-side angle (rad), the error "
                                                     "each motor's loop sees; null unless servoing.")
    tau_est: list[Optional[float]] = Field(description="Per joint: torque the motor's control law applies, "
                                                       "estimated (Nm).")
    tau_g: list[Optional[float]] = Field(description="Per joint: gravity feedforward torque (Nm); null while the "
                                                     "motors are off.")
    err_pos_m: Optional[float] = Field(description="Measured TCP vs commanded TCP, position (m).")
    err_rot_rad: Optional[float] = Field(description="Measured TCP vs commanded TCP, orientation (rad).")
    goal_err_pos_m: Optional[float] = Field(description="Commanded TCP vs the goal of the motion in force, "
                                                        "position (m).")
    goal_err_rot_rad: Optional[float] = Field(description="The same, orientation (rad).")
    limited: bool = Field(description="The joint speed limit held the command back this tick.")
    t: float = Field(description="Time of the tick (server monotonic s).")


class GripperGains(BaseModel):
    kp: float = Field(description="Gripper position gain (Nm/rad).")
    kd: float = Field(description="Gripper damping gain (Nm·s/rad).")


class GainsOut(BaseModel):
    """A gain profile (GET /gains, POST /profile)."""
    name: Literal["soft", "safe"] = Field(description="`soft` (production) or `safe` (debug).")
    file: str = Field(description="The rebot_ctl config file it comes from.")
    kp: list[float] = Field(description="Position gain per joint (6, Nm/rad).")
    kd: list[float] = Field(description="Damping gain per joint (6, Nm·s/rad).")
    gripper: GripperGains
    tau_scale: list[float] = Field(description="Gravity feedforward scale per joint (6; 1 = the model's torque).")


class SpeedLimits(BaseModel):
    joint_vel: list[float] = Field(description="Joint speed cap per joint (6, rad/s).")
    joint_acc: list[float] = Field(description="Joint acceleration cap per joint (6, rad/s²; limits only increases "
                                               "of commanded speed).")
    lin_vel: float = Field(description="TCP speed for servo moves (m/s).")
    lin_acc: float = Field(description="TCP acceleration for servo moves (m/s²).")
    ang_vel: float = Field(description="TCP angular speed for servo moves (rad/s).")
    ang_acc: float = Field(description="TCP angular acceleration for servo moves (rad/s²).")
    home_vel: float = Field(description="Joint speed of joint-space moves: home, named poses, stow (rad/s).")


class LimitsOut(BaseModel):
    """GET / POST /limits."""
    hard: SpeedLimits = Field(description="Fixed caps (config).")
    soft: SpeedLimits = Field(description="Caps in force: the lease holder may lower them within `hard`; reset "
                                          "when the lease ends.")


class TcpOut(BaseModel):
    """GET / POST /tcp."""
    offset: Optional[list[float]] = Field(description="TCP offset in end_link (m, [x, y, z]); null before the arm "
                                                      "has connected.")


class RequestedGoal(BaseModel):
    """A servo goal that was not exactly reachable: what was asked for (the move goes to `goal`)."""
    pose: PoseOut = Field(description="The requested TCP pose, arm frame.")
    pos_err_mm: float = Field(description="Its distance from the goal used (mm).")
    rot_err_deg: float = Field(description="Its angle from the goal used (degrees).")


class MotionInfo(BaseModel):
    """Details of a planned motion, by kind. Servo: distance_m, angle_rad, clik_unconverged, requested;
    chunk: skipped, n, t0, dt, blend_s; joint-space moves (pose, home, stow): none."""
    distance_m: Optional[float] = Field(None, description="Servo: TCP travel (m).")
    angle_rad: Optional[float] = Field(None, description="Servo: TCP rotation (rad).")
    clik_unconverged: Optional[int] = Field(None, description="Servo: path samples where the path tracker's IK did "
                                                              "not converge (absent for a move of zero length).")
    requested: Optional[RequestedGoal] = Field(None, description="Servo: present when the goal was accepted within "
                                                                 "the IK tolerance rather than exactly.")
    skipped: Optional[int] = Field(None, description="Chunk: poses skipped because they were already due.")
    n: Optional[int] = Field(None, description="Chunk: poses sent.")
    t0: Optional[float] = Field(None, description="Chunk: when poses[0] was due (server monotonic s).")
    dt: Optional[float] = Field(None, description="Chunk: s between poses.")
    blend_s: Optional[float] = Field(None, description="Chunk: blend into it (s).")


class MotionOut(BaseModel):
    """A planned motion (servo, named pose, home, stow, chunk)."""
    id: int = Field(description="Motion id (increasing).")
    kind: Literal["servo", "pose", "home", "stow", "chunk"] = Field(description="`pose`: a named pose (POST /servo "
                                                                                "with `name`).")
    name: Optional[str] = Field(description="Named pose (pose, home, stow); null otherwise.")
    phase: Phase = Field(description="executing → settling → reached | not_reached (chunk: done); or aborted "
                                     "(obstruction, cancel, disable, e-stop, lease end) | preempted (by a newer "
                                     "command) | jammed (static but still pushing: the arm holds where it is; GET /jam).")
    reason: Optional[str] = Field(description="Why it ended as it did (aborted, preempted, not_reached, jammed); null "
                                              "otherwise.")
    progress: float = Field(description="0..1 over the planned duration.")
    duration: float = Field(description="Planned duration (s).")
    t_start: float = Field(description="Planned start (server monotonic s).")
    t_end_planned: float = Field(description="Planned end (server monotonic s).")
    goal: Optional[PoseOut] = Field(description="Final TCP pose, arm frame.")
    info: MotionInfo


# -- lease / control / motion replies ------------------------------------------------------------


class LeaseHolder(BaseModel):
    """GET /lease (null when nobody holds it)."""
    client: str = Field(description="Name the holder gave.")
    lease_id: int = Field(description="Lease id (increasing).")
    expires_in: float = Field(description="s until it lapses without a renew or command.")


class LeaseAcquired(BaseModel):
    token: str = Field(description="Send as `X-Lease-Token` with every controlling request.")
    client: str = Field(description="Name the holder gave (`anonymous` if none).")
    lease_id: int = Field(description="Lease id (increasing).")
    heartbeat: float = Field(description="s the lease lasts without a renew or command; lapsing disables.")


class LeaseRenewed(BaseModel):
    lease_id: int = Field(description="Lease id.")
    expires_in: float = Field(description="s until it lapses without a renew or command.")


class LeaseReleased(BaseModel):
    released: bool = Field(description="Always true.")


class EnableOut(BaseModel):
    enabled: bool = Field(description="Always true.")
    state: ArmState = Field(description=STATE_DOC)
    initialized: bool = Field(description="The gripper check has run since connecting.")


class DisableOut(BaseModel):
    enabled: bool = Field(description="Always false.")
    state: ArmState = Field(description=STATE_DOC)


class StateWord(BaseModel):
    state: ArmState = Field(description=STATE_DOC)


class StowOut(BaseModel):
    stowed: bool = Field(description="True: already within 3° of home on every joint, the motors are off now. "
                                     "False: homing first (`motion`); the motors go off when it is reached.")
    motion: Optional[MotionOut] = Field(description="The homing motion; null when stowed at once.")


class TrackOut(BaseModel):
    """POST /track."""
    ok: bool = Field(description="The pose was taken as the goal. False: the previous goal stays (`reason`).")
    seq: int = Field(description="The request's `seq`, echoed.")
    ik: Optional[IKResult] = Field(description="The IK solve of the pose; null when the request was refused as "
                                               "stale (older seq).")
    limited: bool = Field(description="The joint speed limit is clipping the command this tick (it lags the goal).")
    twist_scaled: Optional[bool] = Field(None, description="The twist was scaled down to the soft TCP speed limits. "
                                                           "Absent when the request was stale.")
    reason: Optional[str] = Field(description="Why not ok (IK failure, a jump over 20°, stale seq); null when ok.")


class CancelOut(BaseModel):
    cancelled: bool = Field(description="A planned motion was in force and ended (aborted); false if there was "
                                        "none (a track, if any, still ended).")


class ResetOut(StateWord):
    pass


# -- GET /state ----------------------------------------------------------------------------------


class LeaseState(BaseModel):
    held: bool = Field(description="Someone holds the lease.")
    holder: Optional[str] = Field(description="Name the holder gave; null if none.")
    lease_id: Optional[int] = Field(description="Lease id; null if none.")
    expires_in: Optional[float] = Field(description="s until it lapses; null if none.")


class TrackState(BaseModel):
    """The track command in force."""
    seq: int = Field(description="Its `seq`.")
    age: float = Field(description="s since it was sent.")
    goal: PoseOut = Field(description="Its TCP pose, arm frame.")
    twist: Optional[list[float]] = Field(description="Its twist [vx, vy, vz, wx, wy, wz] (m/s, rad/s) after "
                                                     "scaling; null if none.")
    twist_frame: Literal["arm", "tcp"] = Field(description="Axes of the twist.")


class JamConfigOut(BaseModel):
    """Jam detection: a STATIC arm still pushing (the aftermath of a contact, not the instant of it)."""
    bands: list[float] = Field(description="Nm per joint: the residual |measured torque - gravity model| above which a "
                                           "joint counts as pushing.")
    hold_s: float = Field(description="s a residual must stay above its band before it fires.")
    release: float = Field(description="It clears once every residual is below release x its band (hysteresis).")
    static_qd: float = Field(description="rad/s: the arm counts as static while every joint is slower than this (and "
                                         "no motion is executing).")
    settle_s: float = Field(description="s static before judging (after a move ends or stops).")
    response: Literal["hold", "report", "off"] = Field(
        description="hold: the command is pulled to where the arm is (it stops pushing; a planned move ends `jammed`); "
                    "report: an event only; off: not checked. While tracking it only reports.")
    refire: bool = Field(description="Fire again after clearing (e.g. the base keeps pushing the arm in); false = once "
                                     "per command.")


class JamEvent(BaseModel):
    t: float = Field(description="Server time.monotonic() (s) when it fired.")
    wall: float = Field(description="Unix epoch s.")
    joint: int = Field(description="The joint (1-6) furthest over its band.")
    residual_nm: float = Field(description="Its residual, Nm (sign = direction of the push).")
    band_nm: float = Field(description="Its band, Nm.")
    held_s: float = Field(description="s it had been over its band.")
    response: Literal["hold", "report"] = Field(description="What was done.")
    source: str = Field(description="Arm source then (hold, servo, pose, track, ...).")
    residuals_nm: list[float] = Field(description="Every joint's residual, Nm.")
    count: int = Field(description="Jams since the node started.")


class JamState(BaseModel):
    config: JamConfigOut
    active: Optional[JamEvent] = Field(description="The jam in force (until every residual is well under its band); "
                                                   "null if none.")
    count: int = Field(description="Jams since the node started.")
    recent: list[JamEvent] = Field(description="The last 20 jams, oldest first.")


class ArmStateOut(BaseModel):
    """GET /state (and every /events message). Non-finite numbers are null."""
    state: ArmState = Field(description=STATE_DOC)
    error: Optional[str] = Field(description="The fault or connection error; null if none.")
    sim: bool = Field(description="A simulated arm.")
    initialized: bool = Field(description="The gripper check has run since connecting.")
    enabled: bool = Field(description="Enabled (commands accepted while the motors are on).")
    motors_on: bool = Field(description="Off = damping.")
    servo: bool = Field(description="The arm is servoing (closed loop).")
    gripper_engaged: bool = Field(description="The gripper is holding (closed loop).")
    latched_motor_faults: list[str] = Field(description="Motor faults seen while the motors were off, e.g. "
                                                        "`joint3: over-current`; the next enable clears them.")
    chocked: bool = Field(description="A chock lock refuses enable.")
    locks: list[LockOut] = Field(description="The chocks in force.")
    estop: EStopState
    lease: LeaseState
    source: Source = Field(description="Arm motion source in force (`none` = not servoing; `hold` = holding the "
                                       "last command).")
    track: Optional[TrackState] = Field(description="The track command in force; null unless tracking.")
    motion: Optional[MotionOut] = Field(description="The planned motion in force, or else the last one; null if "
                                                    "none yet.")
    jam: JamState = Field(description="Jam detection (GET /jam).")
    gripper: GripperOut
    profile: str = Field(description="Gain profile in force.")
    profiles: list[str] = Field(description="Gain profiles available.")
    tcp_offset: Optional[list[float]] = Field(description="TCP offset in end_link (m); null before connecting.")
    named_poses: dict[str, list[float]] = Field(description="Named poses (POST /servo `name`): name → 6 joint angles "
                                                            "in DEGREES.")
    stow_near_deg: float = Field(description="Stow turns the motors off at once if every joint is this close to "
                                             "home (degrees).")
    limits: LimitsOut
    twist_timeout: float = Field(description="s a track twist stays in force.")
    lease_heartbeat: float = Field(description="s the lease lasts without a renew or command.")
    loop_hz: Optional[float] = Field(description="Control loop rate over the last ~100 ticks (Hz); null until "
                                                 "there are enough.")
    overruns: int = Field(description="Ticks more than 15 ms after the previous one (the loop runs at 100 Hz), since "
                                      "the node started.")
    late_ticks: int = Field(description="Ticks more than 80 ms after the previous one (each also logged as an event; the "
                                        "motors' own watchdog is 100 ms).")
    read_misses: int = Field(description="Ticks with missed motor replies since the node started.")
    read_misses_by_motor: dict[str, int] = Field(description="Missed replies per motor name (`joint1`..`gripper`).")
    realtime: RealtimeOut = Field(description="How the control loop's thread is scheduled. Without real time, other "
                                              "work on the computer can hold the loop past the motors' 100 ms watchdog.")
    clients: int = Field(description="Open stream / event subscriptions.")
    events: list[Event] = Field(description="The last 20 events, oldest first.")
    latest: Optional[TickRow] = Field(description="The latest control tick; null before the first.")
    server_mono: float = Field(description="Server monotonic s when this was built.")

