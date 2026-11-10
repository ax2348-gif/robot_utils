"""Response models of the agent_nav HTTP API (server.py declares them as ``response_model``).

The handlers return plain dicts (node.py, geometry.py); these models describe them and pass them
through unchanged: fields are declared in the order the dicts are built, keys that are sometimes
absent are optional and the routes use ``response_model_exclude_unset``. Frames: base (floor level
at the wheel centre; x forward, y left, z up, m), world (the estimator's frame), image (colour
pixels, u right, v down). Angles in rad, CCW +.
"""
from __future__ import annotations

from typing import Any, Literal, Optional, Union

from pydantic import BaseModel, ConfigDict, Field

from ..common.server import Event, LockOut

XYTheta = list[float]   # [x m, y m, theta rad]
Pct = list[float]       # [5th, 50th, 95th] percentile


# -- session / profile ---------------------------------------------------------------------------

class Caps(BaseModel):
    """Motion caps (a safety profile, or the caps in force after an agent lowered some)."""
    max_speed: float = Field(description="m/s, translation.")
    max_omega: float = Field(description="rad/s, yaw rate.")
    max_accel: float = Field(description="m/s², translation.")
    max_alpha: float = Field(description="rad/s², yaw.")
    max_jerk: float = Field(description="m/s³, translation.")
    max_ang_jerk: float = Field(description="rad/s³, yaw.")
    max_distance: float = Field(description="m: the farthest a single goal may be from the current pose.")
    max_rotation: float = Field(description="rad: the largest turn of a single goal.")


class Session(BaseModel):
    """The live session."""
    id: str = Field(description="Session id: its start time YYYYmmdd-HHMMSS (server local time); the name of its "
                                "stored transcript.")
    agent: str = Field(description="Name of the caller that opened it (`agent` if none was given).")
    avatar: str = Field(description="Emoji / short text or a data:image URI (🤖 if none was given).")
    started: float = Field(description="Unix epoch s.")
    profile: str = Field(description="Safety profile when it was opened.")


class StoredSession(BaseModel):
    """A stored session (its session.json). Rows of GET /sessions also carry `bytes` (int: size of its
    directory), `turns` (int: dialogue turns in its log) and `live` (bool: it is the open session), after
    the keys below; any other key of the stored file is passed through."""
    id: str = Field(description="Session id (YYYYmmdd-HHMMSS, server local time).")
    agent: str
    avatar: str
    started: float = Field(description="Unix epoch s.")
    profile: Optional[str] = Field(None, description="Safety profile when it was opened.")
    ended: Optional[float] = Field(None, description="Unix epoch s; absent while live (or if the node stopped "
                                                     "without closing it). Stored with `turns`.")
    model_config = ConfigDict(extra="allow")


class OpenSessionOut(BaseModel):
    session: Session
    profile: str = Field(description="Safety profile in force (the human chooses it).")
    caps: Caps = Field(description="The profile's caps (an agent may lower them at POST /enable).")
    guide: str = Field(description="Relative path of the guide to read (`guide.md`).")
    robot: str = Field(description="Where to read the robot's description (GET robot).")
    camera_summary: Optional[str] = Field(None, description="One line: camera height, pitch, field of view, nearest "
                                                            "visible floor. Absent if the robot could not be described.")


class Closed(BaseModel):
    closed: Union[str, bool] = Field(description="Id of the session closed, or false if none was open.")


class EnableOut(BaseModel):
    enabled: bool = Field(description="Always true.")
    profile: str = Field(description="Safety profile in force.")
    caps: Caps = Field(description="Caps in force: the profile's, lowered by the request.")
    adopted: Optional[str] = Field(None, description="Present when pose_servo was already enabled (idle) by someone "
                                                     "else through agent_nav and was re-enabled with these caps.")
    pose_servo: Optional[dict[str, Any]] = Field(description="pose_servo's config in force (its POST /enable reply's "
                                                             "`config`).")


class DisableOut(BaseModel):
    enabled: bool = Field(description="Always false.")
    latched: bool = Field(description="The human operator holds it (disabled or e-stopped on the page): the agent "
                                      "cannot enable until they release it.")


class Released(BaseModel):
    released: bool = Field(description="A hold was in force (and is now released).")


class ProfileOut(BaseModel):
    profile: str
    caps: Caps = Field(description="The profile's caps (apply at the next enable).")


class ReflexOut(BaseModel):
    reflex: bool = Field(description="The stop reflex is on.")


# -- robot ----------------------------------------------------------------------------------------

class Frames(BaseModel):
    """What each frame is (one sentence each)."""
    base: str
    world: str
    image: str
    color_optical: str
    snapshot: str = Field(alias="snapshot:<id>")


class Wheel(BaseModel):
    name: str = Field(description="Module name (FL, FR, RR, RL).")
    x: float = Field(description="m, base frame.")
    y: float = Field(description="m, base frame.")


class Body(BaseModel):
    footprint_m: list[float] = Field(description="[length, width] of the square frame, m.")
    shape: str
    drive: str
    wheels: list[Wheel] = Field(description="Wheel module positions (description node, else built-in constants).")
    turn_in_place_sweep_radius_m: float = Field(description="m: radius the corners sweep when turning in place.")
    height_note: str


class CameraCalibrationSource(BaseModel):
    source: Optional[dict[str, Any]] = Field(description="The description node's `from` of the camera pose (nominal, "
                                                         "ground / yaw calibration ids).")
    serial: Optional[str] = Field(description="Camera serial number.")


class CameraInfo(BaseModel):
    """The camera in the base frame. Without a calibrated camera pose only `calibrated` (and `calibration`)."""
    calibrated: bool = Field(description="3d_perception reports a camera pose in the base (`base_to_color`).")
    position_base_m: Optional[list[float]] = Field(None, description="[x, y, z] m, base frame.")
    height_m: Optional[float] = Field(None, description="m above the floor.")
    optical_axis_base: Optional[list[float]] = Field(None, description="Unit viewing direction, base frame.")
    pitch_down_deg: Optional[float] = Field(None, description="Degrees below horizontal.")
    yaw_deg: Optional[float] = Field(None, description="Degrees, CCW from the base x axis.")
    roll_deg: Optional[float] = Field(None, description="Degrees.")
    hfov_deg: Optional[float] = Field(None, description="Horizontal field of view, degrees.")
    vfov_deg: Optional[float] = Field(None, description="Vertical field of view, degrees.")
    image_size: Optional[list[int]] = Field(None, description="[width, height] colour pixels.")
    nearest_visible_floor_m: Optional[float] = Field(None, description="m ahead of the base centre where the image's "
                                                                       "bottom centre meets the floor; null if it does not.")
    nearest_visible_floor_width_m: Optional[float] = Field(None, description="m: width of the floor seen along the "
                                                                             "image's bottom row; null if unknown.")
    image_centre_hits_floor_at_m: Optional[float] = Field(None, description="m ahead of the base centre; null if the "
                                                                            "optical axis does not meet the floor.")
    top_of_image: Optional[str] = Field(None, description="Where the image's top centre ray goes.")
    blind_zone: Optional[str] = Field(None, description="What is never visible.")
    base_to_color: Optional[list[list[float]]] = Field(None, description="4x4: colour optical frame -> base frame "
                                                                          "(p_base = T p_color).")
    calibration: Optional[CameraCalibrationSource] = Field(None, description="Present when the description node "
                                                                              "describes the camera.")


class RobotLimits(BaseModel):
    profile: str = Field(description="Safety profile in force.")
    caps: Caps = Field(description="Caps in force (the enabled caps, else the profile's).")
    wheel_speed_max_m_s: float = Field(description="m/s, each wheel (pose_servo planner).")
    steer_rate_max_rad_s: float = Field(description="rad/s, each wheel's steering (pose_servo planner).")


class RobotOut(BaseModel):
    frames: Frames
    body: Body
    camera: CameraInfo
    limits: RobotLimits
    sensing_notes: list[str]
    diagram: Optional[str] = Field(description="Relative URL of a side + top view PNG (images/...); null without a "
                                               "calibrated camera.")
    warnings: Optional[list[str]] = Field(None, description="Present when the description node was unavailable.")


# -- perception ------------------------------------------------------------------------------------

class LookOut(BaseModel):
    snapshot: str = Field(description="Snapshot id (s1, s2, ...; the last 30 are kept).")
    image: str = Field(description="Relative URL of the colour JPEG (with a pixel grid if `grid`).")
    bev: str = Field(description="Relative URL of a bird's-eye PNG of the depth (base frame, 0.5 m grid).")
    image_size: list[int] = Field(description="[width, height] colour pixels: the pixels selections are given in.")
    depth_size: list[int] = Field(description="[width, height] of the registered depth grid.")
    pose_world: Optional[XYTheta] = Field(description="Base pose [x m, y m, theta rad] in the world at capture; null "
                                                      "without an estimator pose.")
    mode: str = Field(description="3d_perception's motion compensation of the frame (full | base | rotation | none "
                                   "| no_depth).")
    depth_fraction: float = Field(description="Fraction (0..1) of depth pixels with a depth.")
    moving: bool = Field(description="pose_servo was moving or braking at capture.")
    warnings: list[str]
    grid_image: str = Field(description="Relative URL of the snapshot with a labelled pixel grid.")


class RegionBase(BaseModel):
    x: Pct = Field(description="Percentiles [5, 50, 95] of x, m.")
    y: Pct = Field(description="Percentiles [5, 50, 95] of y, m.")
    z: Pct = Field(description="Percentiles [5, 50, 95] of z (height), m.")
    centroid: list[float] = Field(description="Median [x, y, z], m.")
    nearest_xy: Optional[list[float]] = Field(None, description="[x, y, z] m of the point nearest the base centre "
                                                                "(horizontally). Base frame only.")


class RegionOut(BaseModel):
    """POST /query/region: 3D extent of the selected pixels with depth."""
    query: str = Field(description="Query id (q1, q2, ...), for POST /plan/approach.")
    kind: Literal["region"]
    snapshot: str = Field(description="Snapshot queried.")
    pixels: int = Field(description="Selected depth-grid pixels.")
    with_depth: int = Field(description="Of those, with a depth.")
    depth_fraction: float = Field(description="with_depth / pixels.")
    range_m: Optional[Pct] = Field(None, description="Percentiles [5, 50, 95] of the distance from the camera, m.")
    range_spread_m: Optional[float] = Field(None, description="95th - 5th percentile of the range, m (> 0.4: MIXED "
                                                              "warning).")
    warnings: Optional[list[str]] = Field(None, description="MIXED: the selection likely includes background.")
    touches_image_edge: Optional[bool] = Field(None, description="The selection reaches the image border.")
    base: Optional[RegionBase] = Field(None, description="Base frame at capture (with a calibrated camera pose).")
    world: Optional[RegionBase] = Field(None, description="World frame (with a pose at capture).")
    warning: Optional[str] = Field(None, description="No depth in the selection, or no calibrated camera pose.")
    color_optical_centroid: Optional[list[float]] = Field(None, description="Median [x, y, z] m in the colour optical "
                                                                            "frame (without a calibrated camera pose).")
    overlay: str = Field(description="Relative URL of the snapshot with the selection drawn.")
    bev: str = Field(description="Relative URL of a bird's-eye PNG with the selection outlined.")


class PlaneBase(BaseModel):
    centre: list[float] = Field(description="[x, y, z] m: middle of the fitted extent.")
    normal: list[float] = Field(description="Unit normal, toward the camera.")
    facing_heading: float = Field(description="rad: heading of the normal's horizontal part (where the surface faces).")


class PlaneOut(BaseModel):
    """POST /query/plane: a robust plane through the selected points."""
    query: str = Field(description="Query id (q1, q2, ...), for POST /plan/approach.")
    kind: Literal["plane"]
    snapshot: str = Field(description="Snapshot queried.")
    points: int = Field(description="Selected points with depth.")
    inliers: int = Field(description="Points within 2 cm of the plane.")
    inlier_fraction: float
    rms_m: float = Field(description="m: RMS distance of the inliers from the plane.")
    orientation: str = Field(description="vertical | horizontal | inclined.")
    tilt_from_vertical_deg: float = Field(description="Degrees: 0 = a vertical surface, 90 = horizontal.")
    base: PlaneBase = Field(description="Base frame at capture.")
    width_m: float = Field(description="m: extent along the plane's horizontal (or first) axis (1-99 %).")
    height_m: float = Field(description="m: extent along the other in-plane axis (1-99 %).")
    touches_image_edge: bool = Field(description="The inliers reach the image border: the surface may extend beyond.")
    corners_base: list[list[float]] = Field(description="The extent's 4 corners [x, y, z] m, base frame.")
    warning: Optional[str] = Field(None, description="Image edge reached and / or MIXED (other surfaces selected).")
    world: Optional[PlaneBase] = Field(None, description="World frame (with a pose at capture).")
    overlay: str = Field(description="Relative URL of the snapshot with the selection and the plane drawn.")
    bev: str = Field(description="Relative URL of a bird's-eye PNG with the plane.")


class LineBase(BaseModel):
    centre: list[float] = Field(description="[x, y, z] m: middle of the segment.")
    direction: Optional[list[float]] = Field(None, description="Unit direction (from the first point / left to right "
                                                               "in the image). Base frame only.")
    heading: float = Field(description="rad: heading of the direction's horizontal part.")
    ends: list[list[float]] = Field(description="The segment's ends [[x, y, z], [x, y, z]] m (1-99 % extent).")


class LineOut(BaseModel):
    """POST /query/line: a robust 3D line through points (or a thin selection)."""
    query: str = Field(description="Query id (q1, q2, ...).")
    kind: Literal["line"]
    snapshot: str = Field(description="Snapshot queried.")
    points: int = Field(description="Points used (pixel points with depth, or selected points).")
    inliers: int = Field(description="Points within 2 cm of the line.")
    inlier_fraction: float
    rms_m: float = Field(description="m: RMS distance of the inliers from the line.")
    length_m: float = Field(description="m: segment length.")
    tilt_from_horizontal_deg: float = Field(description="Degrees.")
    base: LineBase = Field(description="Base frame at capture.")
    per_point_base: Optional[list[Optional[list[float]]]] = Field(
        None, description="With `points`: each point's [x, y, z] m (median of a 4 px disk), null without depth.")
    world: Optional[LineBase] = Field(None, description="World frame (with a pose at capture).")
    warning: Optional[str] = Field(None, description="MIXED, or the points are not collinear.")
    overlay: str = Field(description="Relative URL of the snapshot with the points / selection and the line drawn.")
    bev: str = Field(description="Relative URL of a bird's-eye PNG with the line.")


# -- planning / motion ------------------------------------------------------------------------------

class Resolved(BaseModel):
    """A goal resolved against the current estimator pose."""
    world: XYTheta = Field(description="Goal [x m, y m, theta rad] in the world (theta: the literal turn target).")
    local: XYTheta = Field(description="The same goal relative to the current pose (x forward, y left).")
    from_world: XYTheta = Field(description="Current pose [x, y, theta] in the world.")
    distance: float = Field(description="m to travel.")


class Moved(BaseModel):
    m: float = Field(description="m travelled since the snapshot.")
    rad: float = Field(description="rad turned since the snapshot.")


class PathCheck(BaseModel):
    """Loud, advisory check of the straight move against recent snapshots. Never refuses anything."""
    checked: bool = Field(description="False: no snapshot or no calibrated camera / pose (see `warnings`).")
    length_m: Optional[float] = Field(None, description="m: straight-line length.")
    turn_rad: Optional[float] = Field(None, description="rad: goal heading minus current heading.")
    footprint_m: Optional[float] = Field(None, description="m: side of the square footprint swept.")
    margin_m: Optional[float] = Field(None, description="m added around the footprint.")
    obstacle_points_in_path: Optional[int] = Field(None, description="Points 0.04-1.6 m above the floor inside the "
                                                                     "swept footprint + margin.")
    min_clearance_m: Optional[float] = Field(None, description="m from the bare footprint to the nearest obstacle "
                                                               "point; null if none was seen.")
    verified_fraction: Optional[float] = Field(None, description="Fraction (0..1) of the swept area seen; null for a "
                                                                 "turn in place.")
    snapshots_used: Optional[int] = Field(None, description="Snapshots whose points were used.")
    warnings: list[str] = Field(description="OBSTACLE / tight / UNVERIFIED / STALE / UNCHECKED, for the agent to "
                                            "weigh.")
    nearest_in_path_world: Optional[list[list[float]]] = Field(None, description="Up to 5 obstacle points [x, y] m "
                                                                                 "(world) in the path, nearest first.")
    note: Optional[str] = Field(None, description="For a turn in place: what the check cannot see.")
    against_snapshot: Optional[str] = Field(None, description="The snapshot checked against.")
    also_used: Optional[list[str]] = Field(None, description="Recent snapshots also used (same world frame).")
    snapshot_age_s: Optional[float] = Field(None, description="s since that snapshot's capture.")
    moved_since_snapshot: Optional[Moved] = Field(None, description="Motion since that snapshot (with poses).")


class PlanCheckOut(BaseModel):
    goal: Resolved
    check: PathCheck
    bev: Optional[str] = Field(description="Relative URL of a bird's-eye PNG of the check (green clear, red blocked, "
                                           "grey unseen); null if unchecked.")


class Proposal(BaseModel):
    """A goal to POST /goal as is."""
    frame: str = Field(description="`snapshot:<id>`: the base frame at the queried snapshot.")
    x: float = Field(description="m.")
    y: float = Field(description="m.")
    theta: float = Field(description="rad.")


class ApproachOut(BaseModel):
    proposal: Proposal
    goal: Resolved
    check: PathCheck
    warnings: list[str] = Field(description="The check's warnings, after the source query's warning if any.")
    bev: Optional[str] = Field(description="Relative URL of a bird's-eye PNG of the check; null if unchecked.")
    note: str = Field(description="A proposal only: nothing moves until POST /goal.")


class GoalOut(BaseModel):
    goal_id: int = Field(description="pose_servo's goal id.")
    status: str = Field(description="Always `moving`.")
    goal: Resolved
    check: PathCheck
    warnings: list[str] = Field(description="The check's warnings (the goal is executed regardless).")
    bev: Optional[str] = Field(description="Relative URL of a bird's-eye PNG of the check; null if unchecked.")
    wait: str = Field(description="Relative URL to long-poll for the result.")


class GoalResult(BaseModel):
    """How the goal ended."""
    result: str = Field(description="reached | stopped | stalled | aborted.")
    reason: Optional[str] = Field(description="Why it did not reach (pose_servo's reason); null if reached.")
    pose_world: Optional[list[Optional[float]]] = Field(None, description="Pose [x, y, theta] at the end; null without "
                                                                          "pose_servo telemetry. Absent if aborted "
                                                                          "because pose_servo was disabled.")
    err_pos: Optional[float] = Field(None, description="m from the goal at the end.")
    duration_s: Optional[float] = Field(None, description="s from issue to the end.")


class Progress(BaseModel):
    """pose_servo's latest tick (null values without telemetry)."""
    err_pos: Optional[float] = Field(description="m to the goal.")
    err_theta: Optional[float] = Field(description="rad to the goal heading.")
    vx: Optional[float] = Field(description="m/s, robot frame.")
    vy: Optional[float] = Field(description="m/s, robot frame.")
    omega: Optional[float] = Field(description="rad/s.")


class GoalState(BaseModel):
    goal_id: int
    done: bool
    result: Optional[GoalResult] = Field(description="Null until it ends.")
    goal: Resolved = Field(description="The goal as resolved when it was issued.")
    issued_by: str = Field(description="agent | human.")
    age_s: float = Field(description="s since it was issued.")
    progress: Optional[Progress] = Field(None, description="While not done.")


class ServoStop(BaseModel):
    """pose_servo's POST /stop reply, passed through."""
    status: str = Field(description="pose_servo status after the request (braking | idle | ...).")
    model_config = ConfigDict(extra="allow")


class ServoEStop(BaseModel):
    """pose_servo's POST /estop reply, passed through (pose_servo disables itself either way)."""
    forwarded: bool = Field(description="The e-stop reached the swerve (it latches there).")
    error: Optional[str] = Field(None, description="Why forwarding failed.")
    model_config = ConfigDict(extra="allow")


# -- memory -----------------------------------------------------------------------------------------

class Note(BaseModel):
    t: float = Field(description="Unix epoch s.")
    by: str = Field(description="agent | human.")
    text: str


class Place(BaseModel):
    x: float = Field(description="m, world.")
    y: float = Field(description="m, world.")
    theta: Optional[float] = Field(description="rad, world; null = any heading.")
    reset_id: Optional[float] = Field(description="Estimator reset id it belongs to (invalid after a reset).")
    t: float = Field(description="Unix epoch s.")


class Memory(BaseModel):
    """This session's memory (cleared by a new session)."""
    notes: list[Note]
    places: dict[str, Place] = Field(description="Named world poses (goal frame `place:<name>`).")


class NoteOut(BaseModel):
    notes: int = Field(description="Notes now stored.")


class PlaceOut(BaseModel):
    """The place stored (its name, then the same fields as in /memory)."""
    place: str = Field(description="The name.")
    x: float = Field(description="m, world.")
    y: float = Field(description="m, world.")
    theta: Optional[float] = Field(description="rad, world; null = any heading.")
    reset_id: Optional[float] = Field(description="Estimator reset id it belongs to (invalid after a reset).")
    t: float = Field(description="Unix epoch s.")


# -- log / state --------------------------------------------------------------------------------------

class Turn(BaseModel):
    """One dialogue turn: a call (or a robot event) and the reply."""
    n: int = Field(description="Turn number (since the node started).")
    session: Optional[str] = Field(description="Session id; null outside a session.")
    t: float = Field(description="Server time.monotonic() (s).")
    wall: float = Field(description="Unix epoch s.")
    actor: str = Field(description="agent | human | robot.")
    kind: str = Field(description="The call (look, query_plane, goal, ...) or `event` (robot).")
    request: dict[str, Any] = Field(description="The call's arguments.")
    reply: dict[str, Any] = Field(description="The reply (the endpoint's JSON), or {error, status} if refused, or "
                                              "{message, ...} for a robot event.")
    comment: str = Field(description="The caller's comment.")
    images: list[str] = Field(description="Image names (GET images/<name> while kept).")
    level: str = Field("info", description="info | warn.")
    thumbs: Optional[list[str]] = Field(None, description="Stored thumbnails (GET sessions/<id>/thumbs/<name>); absent "
                                                          "outside a session.")


class LogOut(BaseModel):
    turns: list[Turn] = Field(description="Turns after `since` (up to `limit`, oldest first).")
    last: int = Field(description="Number of the latest turn (pass as `since`).")
    session: Optional[Session]


class StoredSessionOut(BaseModel):
    session: StoredSession
    turns: list[Turn] = Field(description="All its turns (log.jsonl).")


class Deleted(BaseModel):
    deleted: str = Field(description="Id of the session deleted.")


class NavEStop(BaseModel):
    """The swerve's e-stop, as pose_servo last saw it."""
    latched: bool
    source: Optional[str] = Field(description="`swerve` if latched, else null.")
    reason: None = Field(description="Always null (see the swerve's /state).")


class Latch(BaseModel):
    why: str = Field(description="disabled on the page | e-stop from the page.")
    t: float = Field(description="Unix epoch s.")


class PoseServoBrief(BaseModel):
    """From pose_servo's /state."""
    status: Optional[str] = Field(description="idle | moving | braking | holding | disabled.")
    enabled: Optional[bool]
    holder: Optional[str] = Field(description="Who enabled it (`agent_nav: <name>` through this node).")
    disabled_reason: Optional[str]
    goal: Optional[dict[str, Any]] = Field(description="pose_servo's last goal record (goal_id, resolved pose, motion, "
                                                       "result, reason, ...).")


class ReflexTick(BaseModel):
    t: float = Field(description="Unix epoch s.")
    v: float = Field(description="m/s: translation speed.")
    lookahead_m: float = Field(description="m: braking distance + latency + margin checked ahead.")
    points: int = Field(description="Obstacle points found in that zone.")


class Reflex(BaseModel):
    """The stop reflex (it can only stop)."""
    enabled: bool = Field(description="Turned on (human).")
    armed: bool = Field(description="Armed by a goal; disarmed after it stopped one.")
    last: Union[ReflexTick, str, None] = Field(description="The last check, or `error: ...`; null before the first.")
    blind: Optional[str] = Field(description="Set while moving outside the camera's view.")
    stops: int = Field(description="Stops so far.")


class GoalBrief(BaseModel):
    done: bool
    result: Optional[str] = Field(description="reached | stopped | stalled | aborted; null until done.")


class UpstreamStatus(BaseModel):
    """A pose stream input (estimator Upstream.status())."""
    url: str = Field(description="Stream URL.")
    connected: bool
    error: Optional[str]
    rate_hz: Optional[float] = Field(description="Rows per s (null if stalled > 1 s or too few).")
    last_age: Optional[float] = Field(description="s since the last record.")
    samples: int = Field(description="Rows received.")
    clock_offset: float = Field(description="s: upstream monotonic - local monotonic.")
    clock_uncertainty: Optional[float] = Field(description="s.")


class Inputs(BaseModel):
    estimator: Optional[UpstreamStatus] = Field(description="Null if no estimator is configured.")


class NavState(BaseModel):
    chocked: bool = Field(description="A chock lock covers swerve / pose_servo / agent_nav.")
    covered: bool = Field(description="A cover lock covers the camera.")
    locks: list[LockOut] = Field(description="Those locks.")
    estop: NavEStop
    session: Optional[Session]
    profile: str = Field(description="Safety profile in force (for the next enable).")
    profiles: dict[str, Caps] = Field(description="All profiles (cautious, normal).")
    enabled_by: Optional[str] = Field(description="agent | human; null when disabled.")
    caps: Optional[Caps] = Field(description="Caps of the last enable; null before.")
    latch: Optional[Latch] = Field(description="The human's hold; null if none.")
    pose_world: Optional[XYTheta] = Field(description="Current base pose; null without a fresh estimator pose.")
    reset_id: Optional[float] = Field(description="The estimator's reset id (its world frame).")
    pose_servo: Optional[PoseServoBrief] = Field(description="Null while pose_servo is unreachable.")
    reflex: Reflex
    snapshots: list[str] = Field(description="Latest snapshot ids (up to 10), oldest first.")
    goals: dict[int, GoalBrief] = Field(description="Latest goals of the session (up to 10), by goal id.")
    memory: Memory
    turns: int = Field(description="Number of the latest turn.")
    inputs: Inputs
    events: list[Event] = Field(description="Latest node events (up to 20), oldest first.")


class HelpOut(BaseModel):
    help: str = Field(description="The chat commands.")


# POST /chat: the reply of the endpoint the command maps to, or the help.
ChatOut = Union[LookOut, RegionOut, PlaneOut, LineOut, PlanCheckOut, ApproachOut, GoalOut, RobotOut, EnableOut,
                DisableOut, Released, NoteOut, PlaceOut, ServoEStop, ServoStop, HelpOut]


EVENTS_DOC = (
    "Unnamed Server-Sent Events, on every new turn and every 1 s; each `data:` line is a small JSON state: "
    "`turns` (latest turn number), `session` (as in /state), `enabled_by`, `latch`, `pose_world` "
    "([x, y, theta], nulls without a pose), `servo` (pose_servo status), `reflex` (the reflex's `blind` "
    "note), `latest` (pose_servo's latest tick), and the lock fields `chocked`, `covered`, `locks`, `estop` "
    "of /state.")
