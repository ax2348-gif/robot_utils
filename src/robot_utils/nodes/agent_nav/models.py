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
    agent: str = Field(description="The agent's name.")
    avatar: str = Field(description="Its avatar (emoji, short text or data URI).")
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
    profile: str = Field(description="The profile now in force: cautious | normal.")
    caps: Caps = Field(description="The profile's caps (apply at the next enable).")


class ReflexOut(BaseModel):
    reflex: bool = Field(description="The stop reflex is on.")


# -- robot ----------------------------------------------------------------------------------------

class Frames(BaseModel):
    """What each frame is (one sentence each)."""
    base: str = Field(description="The robot's own frame (goals' `local`).")
    world: str = Field(description="The estimator's frame (goals' `global`).")
    image: str = Field(description="Colour-image pixels: where selections are given.")
    color_optical: str = Field(description="The colour camera's frame.")
    snapshot: str = Field(alias="snapshot:<id>", description="The base frame when a snapshot was taken.")


class Wheel(BaseModel):
    name: str = Field(description="Module name (FL, FR, RR, RL).")
    x: float = Field(description="m, base frame.")
    y: float = Field(description="m, base frame.")


class Body(BaseModel):
    footprint_m: list[float] = Field(description="[length, width] of the square frame, m.")
    shape: str = Field(description="The footprint's shape and where it is centred.")
    drive: str = Field(description="How the base moves (swerve: any direction while turning).")
    wheels: list[Wheel] = Field(description="Wheel module positions (description node, else built-in constants).")
    turn_in_place_sweep_radius_m: float = Field(description="m: radius the corners sweep when turning in place.")
    height_note: str = Field(description="Heights of the base, mast and arm, in words.")


class CalibrationFrom(BaseModel):
    """How the description node composed the camera pose (its GET /description `cameras.<name>.from`)."""
    nominal: bool = Field(description="True: x and y (and anything not calibrated) come from the measured nominal pose.")
    ground: Optional[str] = Field(None, description="Id of the ground calibration used (roll, pitch, height).")
    yaw: Optional[str] = Field(None, description="Id of the yaw calibration used (heading).")
    residual_deg: Optional[list[float]] = Field(None, description="deg: how far each calibration vector disagrees with "
                                                                  "the fit (ground normal first, then each yaw vector).")


class CameraCalibrationSource(BaseModel):
    source: Optional[CalibrationFrom] = Field(description="How the camera pose was composed; null if unknown.")
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
    sensing_notes: list[str] = Field(description="What the robot can't sense, in words.")
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
    warnings: list[str] = Field(description="Read these: no camera calibration or pose at capture, the robot moving, "
                                            "a session camera correction in use.")
    grid_image: str = Field(description="Relative URL of the snapshot with a labelled pixel grid.")
    grid: Optional["GridInfo"] = Field(None, description="With grid values: the grid and the sampled values.")


class GridSample(BaseModel):
    u: int = Field(description="Colour pixel column (full image).")
    v: int = Field(description="Colour pixel row (full image).")
    base: Optional[list[float]] = Field(description="[x, y, z] m in the base frame at this grid intersection (median of "
                                                    "the depth around it); null where there is no depth.")


class GridInfo(BaseModel):
    box: list[int] = Field(description="[u0, v0, u1, v1]: the part of the image shown (colour pixels).")
    magnification: float = Field(description="Magnification of the shown part (1 = actual size).")
    step: int = Field(description="Grid spacing, colour pixels.")
    values: Optional[str] = Field(description="Which value is written at each grid intersection (x | y | z | range | "
                                              "xz | xyz), or null for lines only.")
    samples: Optional[list[GridSample]] = Field(None, description="Every grid intersection's point (with values).")
    unit: Optional[str] = Field(None, description="Unit and frame of the drawn values (with values).")


class ZoomOut(BaseModel):
    zoom: str = Field(description="Zoom id (z1, z2, ...).")
    snapshot: str = Field(description="The snapshot shown (no new capture).")
    image: str = Field(description="Relative URL of the JPEG.")
    box: list[int] = Field(description="[u0, v0, u1, v1]: the part of the image shown (colour pixels).")
    magnification: float = Field(description="Magnification of the shown part (1 = actual size).")
    step: int = Field(description="Grid spacing, colour pixels; the labels are full-image pixel coordinates.")
    values: Optional[str] = Field(description="The value written at each grid intersection, or null for lines only.")
    samples: Optional[list[GridSample]] = Field(None, description="Every grid intersection's point (with values).")
    unit: Optional[str] = Field(None, description="Unit and frame of the drawn values (with values).")
    note: str = Field(description="How to read the image.")


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
    kind: Literal["region"] = Field(description="The query kind.")
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
    kind: Literal["plane"] = Field(description="The query kind.")
    snapshot: str = Field(description="Snapshot queried.")
    points: int = Field(description="Selected points with depth.")
    inliers: int = Field(description="Points within 2 cm of the plane.")
    inlier_fraction: float = Field(description="inliers / points (low: the selection mixes surfaces).")
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
    kind: Literal["line"] = Field(description="The query kind.")
    snapshot: str = Field(description="Snapshot queried.")
    points: int = Field(description="Points used (pixel points with depth, or selected points).")
    inliers: int = Field(description="Points within 2 cm of the line.")
    inlier_fraction: float = Field(description="inliers / points (low: a mixed or curved selection).")
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
    """Loud, advisory check of the straight move against the snapshot (plus any named in `also_use`). Never refuses
    anything."""
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
    also_used: Optional[list[str]] = Field(None, description="The other snapshots combined (those named in `also_use`).")
    obstacles_by_snapshot: Optional[dict[str, int]] = Field(
        None, description="With obstacle points: how many each snapshot contributed (by snapshot id).")
    snapshot_age_s: Optional[float] = Field(None, description="s since that snapshot's capture.")
    moved_since_snapshot: Optional[Moved] = Field(None, description="Motion since that snapshot (with poses).")
    obstacle_masks: Optional[dict[str, str]] = Field(
        None, description="With obstacle points: per snapshot, a relative URL of its image with the counted pixels "
                          "tinted red. Look at it: they may be the robot itself or something no longer there.")


class Placement(BaseModel):
    xyz: list[float] = Field(description="m: the frame's origin in base (for a correction: the shift, base axes).")
    rpy_deg: list[float] = Field(description="degrees, fixed axes X, Y, Z (for a correction: the turn about the "
                                             "frame's origin, base axes).")


class PlacementPair(BaseModel):
    camera: Optional[Placement] = Field(None, description="The colour camera's optical frame in base; null if unknown "
                                                          "(see `warnings`).")
    arm: Optional[Placement] = Field(None, description="The arm frame in base; null if the arm node is unreachable.")


class GeometryFrames(BaseModel):
    camera: str = Field(description="Which frame `camera` is.")
    arm: str = Field(description="Which frame `arm` is.")


class CorrectionLimits(BaseModel):
    xyz_m: float = Field(description="m: the largest correction per axis.")
    rpy_deg: float = Field(description="degrees: the largest turn per axis.")


class SessionGeometryOut(BaseModel):
    """Where the camera and the arm mount are on the robot: as described, the session's corrections, and the
    result this session uses for display and perception. Motion never uses the corrections."""
    frames: GeometryFrames
    described: PlacementPair = Field(description="From the description node (via 3d_perception and the arm node).")
    corrections: PlacementPair = Field(description="This session's corrections (zero: none).")
    effective: PlacementPair = Field(description="Described + corrections.")
    limits: CorrectionLimits
    applies_to: str = Field(description="What uses the corrections.")
    never_applies_to: str = Field(description="What never does.")
    warnings: Optional[list[str]] = Field(None, description="A placement that could not be read (camera or arm).")


class ArmInBase(BaseModel):
    matrix: list[list[float]] = Field(description="4x4: the arm frame in base, as described (p_base = T p_arm).")
    xyz: list[float] = Field(description="m: its origin in base.")
    source: str = Field(description="Where it came from (the description node, or the shipped default).")
    effective: list[list[float]] = Field(description="4x4: what was drawn (with the session's arm correction).")
    session_correction: Optional[Placement] = Field(description="The session's arm-mount correction; null if none.")


class ArmDrawn(BaseModel):
    which: Literal["measured", "q", "ik"] = Field(description="measured: the arm as it is now; q: the given joint "
                                                              "angles; ik: the IK solution of the given pose.")
    q: list[float] = Field(description="Joint angles drawn, rad.")
    gap_mm: float = Field(description="Gripper gap drawn.")
    ik: Optional[dict[str, Any]] = Field(description="With a pose: the arm node's IK result (ok, exact, q, pos_err_mm, "
                                                     "rot_err_deg, nearest, reason; as its POST /ik). Null otherwise.")
    arm_in_base: ArmInBase


class PastFootprint(BaseModel):
    """m beyond each side of the square frame (negative: inside by that much)."""
    front: float = Field(description="m beyond the front face.")
    rear: float = Field(description="m beyond the rear face.")
    left: float = Field(description="m beyond the left face.")
    right: float = Field(description="m beyond the right face.")


class TipPoint(BaseModel):
    base: list[float] = Field(description="[x, y, z] m in the base frame at the goal pose.")
    snapshot_base: list[float] = Field(description="[x, y, z] m in the base frame of the snapshot (where the robot was).")
    world: list[float] = Field(description="[x, y, z] m in the world frame.")


class Fingertips(BaseModel):
    left: TipPoint = Field(description="The left jaw's tip (on its gripping face, mid height).")
    right: TipPoint = Field(description="The right jaw's tip.")
    center: TipPoint = Field(description="Midway between them (the arm's end_link).")


class BoxHits(BaseModel):
    box: str = Field(description="The arm part (a box of the arm node's GET /geometry `boxes`).")
    points: int = Field(description="Depth points of the snapshot inside it.")


class RobotView(BaseModel):
    """The whole robot at the goal, imagined: the base, and the arm (as it is now, or at `arm`), drawn into the
    snapshot. Nothing moves."""
    warnings: list[str] = Field(description="Read these: no arm node, an unreachable arm pose (drawn at the nearest), "
                                            "depth points inside the arm, parts out of view, session corrections.")
    goal_in_snapshot_base: Optional[list[float]] = Field(None, description="[x, y, theta] of the goal in the snapshot's "
                                                                          "base frame.")
    arm: Optional[ArmDrawn] = Field(None, description="The arm configuration drawn (null without the arm node).")
    fingertips: Optional[Fingertips] = Field(None, description="Where the fingertips would be.")
    past_footprint_m: Optional[PastFootprint] = Field(None, description="How far the arm reaches past each side of the "
                                                                        "frame at that configuration (negative: inside).")
    depth_points_in_arm: Optional[list[BoxHits]] = Field(None, description="Depth points of the snapshot inside each "
                                                         "arm part at the goal: an obstacle in the way, or the arm itself "
                                                         "where it is now.")
    boxes_in_view: Optional[int] = Field(None, description="Robot parts (boxes) with any corner in the image.")
    boxes_partly_out_of_view: Optional[int] = Field(None, description="Parts with some corner outside the image.")
    view: Optional[str] = Field(None, description="Relative URL of the snapshot with the robot drawn in (cyan base, "
                                                  "orange arm, magenta L/R fingertips).")


class PlanCheckOut(BaseModel):
    goal: Resolved
    check: PathCheck
    bev: Optional[str] = Field(description="Relative URL of a bird's-eye PNG of the check (green clear, red blocked, "
                                           "grey unseen); null if unchecked.")
    robot: Optional[RobotView] = Field(None, description="With `view` (default): the robot drawn at the goal.")


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
    reflex_margin_m: Optional[float] = Field(None, description="The stop reflex's margin for this goal (m).")


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
    goal_id: int = Field(description="pose_servo's goal id.")
    done: bool = Field(description="The goal has ended (`result` says how).")
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
    text: str = Field(description="The note.")


class Place(BaseModel):
    x: float = Field(description="m, world.")
    y: float = Field(description="m, world.")
    theta: Optional[float] = Field(description="rad, world; null = any heading.")
    reset_id: Optional[float] = Field(description="Estimator reset id it belongs to (invalid after a reset).")
    t: float = Field(description="Unix epoch s.")


class Memory(BaseModel):
    """This session's memory (cleared by a new session)."""
    notes: list[Note] = Field(description="Notes, oldest first.")
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
    session: Optional[Session] = Field(description="The open session; null if none.")


class StoredSessionOut(BaseModel):
    session: StoredSession
    turns: list[Turn] = Field(description="All its turns (log.jsonl).")


class Deleted(BaseModel):
    deleted: str = Field(description="Id of the session deleted.")


class NavEStop(BaseModel):
    """The swerve's e-stop, as pose_servo last saw it."""
    latched: bool = Field(description="An e-stop is latched: nothing moves until a human clears it.")
    source: Optional[str] = Field(description="`swerve` if latched, else null.")
    reason: None = Field(description="Always null (see the swerve's /state).")


class Latch(BaseModel):
    why: str = Field(description="disabled on the page | e-stop from the page.")
    t: float = Field(description="Unix epoch s.")


class PoseServoBrief(BaseModel):
    """From pose_servo's /state."""
    status: Optional[str] = Field(description="idle | moving | braking | holding | disabled.")
    enabled: Optional[bool] = Field(description="pose_servo holds the swerve lease and accepts goals.")
    holder: Optional[str] = Field(description="Who enabled it (`agent_nav: <name>` through this node).")
    disabled_reason: Optional[str] = Field(description="Why it was last disabled (null while enabled).")
    goal: Optional[dict[str, Any]] = Field(description="pose_servo's last goal record (goal_id, resolved pose, motion, "
                                                       "result, reason, ...).")


class ReflexTick(BaseModel):
    t: float = Field(description="Unix epoch s.")
    v: float = Field(description="m/s: translation speed.")
    lookahead_m: float = Field(description="m checked ahead of the bumper: the stopping distance (one check interval "
                                           "+ latency at v, then jerk-limited braking) + the margin.")
    margin_m: Optional[float] = Field(None, description="m: the margin in force (the goal's reflex_margin_m).")
    tick_s: Optional[float] = Field(None, description="s: measured interval between checks.")
    points: int = Field(description="Obstacle points found in that zone.")


class Reflex(BaseModel):
    """The stop reflex (it can only stop)."""
    enabled: bool = Field(description="Turned on (human).")
    armed: bool = Field(description="Armed by a goal; disarmed after it stopped one.")
    last: Union[ReflexTick, str, None] = Field(description="The last check, or `error: ...`; null before the first.")
    blind: Optional[str] = Field(description="Set while moving outside the camera's view.")
    stops: int = Field(description="Stops so far.")
    margin: float = Field(description="m: the margin of the current goal (default 0.12).")
    tick_errors: int = Field(description="Checks that failed (no frame, ...): the reflex did not look that time.")


class GoalBrief(BaseModel):
    done: bool = Field(description="The goal has ended.")
    result: Optional[str] = Field(description="reached | stopped | stalled | aborted; null until done.")


class UpstreamStatus(BaseModel):
    """A pose stream input (estimator Upstream.status())."""
    url: str = Field(description="Stream URL.")
    connected: bool = Field(description="The stream is open.")
    error: Optional[str] = Field(description="The last connection error; null if none.")
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
    session: Optional[Session] = Field(description="The open session; null if none.")
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
ChatOut = Union[LookOut, ZoomOut, RegionOut, PlaneOut, LineOut, PlanCheckOut, ApproachOut, GoalOut, RobotOut,
                SessionGeometryOut, EnableOut, DisableOut, Released, NoteOut, PlaceOut, ServoEStop, ServoStop, HelpOut]


EVENTS_DOC = (
    "Unnamed Server-Sent Events, on every new turn and every 1 s; each `data:` line is a small JSON state: "
    "`turns` (latest turn number), `session` (as in /state), `enabled_by`, `latch`, `pose_world` "
    "([x, y, theta], nulls without a pose), `servo` (pose_servo status), `reflex` (the reflex's `blind` "
    "note), `latest` (pose_servo's latest tick), and the lock fields `chocked`, `covered`, `locks`, `estop` "
    "of /state.")
