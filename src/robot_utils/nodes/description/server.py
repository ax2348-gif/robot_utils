"""HTTP API of the description node (FastAPI). Conventions: docs/nodes.md; guide: docs/description_node.md."""
from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from typing import Annotated, Any, Literal, Optional, Union

from fastapi import FastAPI, HTTPException, Query, Request
from fastapi.responses import FileResponse, HTMLResponse, Response, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field, RootModel, WithJsonSchema

from ..common.server import (EStopCleared, EStopState, EStopTriggered, Event, LockOut, add_base_routes,
                             add_page_routes, node_meta, setup_app, sse)
from . import __version__
from .node import DescriptionNode
from .routines import LIMITS, GroundParams, YawParams
from .store import TYPES

TAGS = [
    {"name": "description", "description": "Frames and transforms of the robot."},
    {"name": "calibration", "description": "Calibration routines (jobs), their records and the active ones."},
    {"name": "control", "description": "E-stop and abort (the yaw routine drives the base)."},
    {"name": "state", "description": "Status, clock sync, events."},
]


class NominalIn(BaseModel):
    xyz: list[float] = Field(min_length=3, max_length=3, description="m, position of depth_optical in base (measure x, y)")
    rpy: list[float] = Field(min_length=3, max_length=3, description="rad, fixed-axis X, Y, Z (URDF)")


class GroundIn(BaseModel):
    camera: str = Field("camera", description="Camera name in the robot description (a key of GET /description "
                                              "`cameras`).")
    polygons: Optional[list[list[list[float]]]] = Field(
        None, description="Floor regions on the depth image: polygons of [x, y] in normalised image coords "
                          "(0..1, origin top left). Omit for automatic (largest near-horizontal plane, using gravity).")
    frames: int = Field(10, ge=1, le=60, description="Depth frames to take the per-pixel median of.")
    max_tilt_deg: float = Field(30.0, gt=0, le=60, description="deg; automatic mode only: the plane must be within "
                                                                "this of horizontal (camera IMU gravity).")
    tol: float = Field(0.01, gt=0, le=0.1, description="m, inlier distance.")
    activate: bool = Field(True, description="Activate the new record if it passes the quality gate.")
    model_config = ConfigDict(json_schema_extra={"examples": [
        {"polygons": [[[0.1, 0.7], [0.9, 0.7], [0.9, 0.98], [0.1, 0.98]]]}, {}]})


class YawIn(BaseModel):
    camera: str = Field("camera", description="Camera name in the robot description (a key of GET /description "
                                              "`cameras`).")
    vectors: list[list[float]] = Field([[1.0, 0.0], [0.0, 1.0]], description="Base-frame directions [vx, vy] (normalised).")
    amplitude: float = Field(0.08, gt=0, le=LIMITS["amplitude_max"], description="m; the base swings 0..2A along each vector.")
    frequency: float = Field(1.0, ge=LIMITS["frequency_min"], le=LIMITS["frequency_max"], description="Hz")
    cycles: int = Field(5, ge=LIMITS["cycles_min"], le=LIMITS["cycles_max"], description="Sine periods per vector.")
    accel_lowpass_hz: Optional[float] = Field(None, gt=0, le=50, description="Accelerometer low-pass before the "
                                              "line fit (default 2 × frequency; removes the base's vibration).")
    confirm: bool = Field(False, description="Must be true: this drives the robot.")
    activate: bool = Field(True, description="Activate the new record if it passes the quality gate.")
    model_config = ConfigDict(json_schema_extra={"examples": [{"confirm": True},
                                                              {"vectors": [[1, 0], [0, 1], [1, 1]], "amplitude": 0.06,
                                                               "frequency": 1.2, "cycles": 6, "confirm": True}]})


class ResolveIn(BaseModel):
    accel_lowpass_hz: Optional[float] = Field(None, gt=0, le=50, description="Default 2 × the run's frequency.")
    activate: bool = Field(True, description="Activate the new record if it passes the quality gate.")


class ActivateIn(BaseModel):
    serial: str = Field(description="Camera serial the record is filed under.")
    id: Optional[str] = Field(None, description="Calibration id; null = none active.")


class EStopIn(BaseModel):
    reason: str = Field("", description="Why (free text); `e-stop` if empty.")
    source: str = Field("", description="Who (free text); the client IP if empty.")


# -- responses ---------------------------------------------------------------------------------
# Calibration records (meta.json) are read back from disk, possibly written by an older version:
# their models keep every key optional (absent stays absent) and allow extra keys.

# An int or a float, passed through as is (values from the YAML description or settings files).
Number = Annotated[Union[int, float], WithJsonSchema({"type": "number"})]


class Pose(BaseModel):
    """A rigid transform (frames.describe_T): the pose of a child frame b in a frame a,
    p_a = R p_b + t (homogeneous: p_a = matrix · p_b)."""
    xyz: list[float] = Field(description="m, translation t (1 µm resolution).")
    rpy: list[float] = Field(description="rad, R as fixed-axis X, Y, Z (URDF: R = Rz(yaw) Ry(pitch) Rx(roll)).")
    rpy_deg: list[float] = Field(description="The same in degrees.")
    rotation: list[list[float]] = Field(description="R, 3×3 row-major.")
    matrix: list[list[float]] = Field(description="4×4 homogeneous transform [[R, t], [0, 1]], row-major.")


class PoseSource(BaseModel):
    """How a camera's depth_optical pose was composed (node.compose_camera). Keys are absent when unused."""
    nominal: bool = Field(description="Always true: x and y always come from the nominal pose, and anything not "
                                      "calibrated.")
    ground: Optional[str] = Field(None, description="Id of the active ground calibration used (roll, pitch, height).")
    yaw: Optional[str] = Field(None, description="Id of the active yaw calibration used (yaw).")
    residual_deg: Optional[list[float]] = Field(None, description="deg, Wahba residual of each calibration vector pair "
                                                                  "(ground normal first, then each yaw vector); "
                                                                  "present with any calibration.")


class Frame(BaseModel):
    """One frame and its pose in its parent: p_parent = R p_child + t. Robot frames come from the
    description file (`rpy`/`xyz`, or `rotation`/`translation`; free-form, e.g. `doc`);
    `<camera>/depth_optical` has xyz, rpy and `source` (how it was composed); the camera's internal
    frames (`<camera>/color_optical`, `<camera>/imu`) have rotation, translation and `source`
    `camera node (factory)`. Keys a frame does not have are absent."""
    parent: Optional[str] = Field(None, description="Parent frame name; null for the root (`base`).")
    xyz: Optional[list[Number]] = Field(None, description="m, t.")
    rpy: Optional[list[Number]] = Field(None, description="rad, R as fixed-axis X, Y, Z (URDF).")
    rotation: Optional[list[list[Number]]] = Field(None, description="R, 3×3 row-major.")
    translation: Optional[list[Number]] = Field(None, description="m, t.")
    source: Optional[Union[PoseSource, str]] = Field(None, description="Where the pose comes from: how a camera's "
                                                                       "depth_optical was composed, or `camera node "
                                                                       "(factory)`.")
    doc: Optional[str] = Field(None, description="What the frame is (description file).")
    model_config = ConfigDict(extra="allow")


class SwerveModule(BaseModel):
    name: str = Field(description="FL | FR | RR | RL.")
    x: float = Field(description="m, wheel contact point forward of base.")
    y: float = Field(description="m, wheel contact point left of base.")


class RobotGeometry(BaseModel):
    """The swerve base's geometry (swerve.constants), for drawing the robot."""
    frame_size: float = Field(description="m, side of the square chassis frame (17 in).")
    frame_z: list[float] = Field(description="m, bottom and top faces of the 2x1 frame tubes above the floor "
                                             "(2.75 in, 4.75 in).")
    top_rail_z: list[float] = Field(description="m, bottom and top faces of the 1x1 top rails (7.75 in, 8.75 in).")
    top_z: float = Field(description="m, the robot's top surface (top rails, 8.75 in), on which the arm's mount "
                                     "plate sits.")
    modules: list[SwerveModule] = Field(description="The four swerve modules (wheels), FL, FR, RR, RL order "
                                                    "as in swerve.constants.")


class NominalPose(BaseModel):
    """A camera's measured/nominal pose of depth_optical in base (description file, or POST /cameras/{name}/nominal)."""
    xyz: list[Number] = Field(description="m.")
    rpy: list[Number] = Field(description="rad, fixed-axis X, Y, Z (URDF).")


class Distortion(BaseModel):
    """Lens distortion coefficients as the camera SDK reports them (unitless; OpenCV's rational model, as
    agent_nav projects with it: radial factor (1 + k1 r² + k2 r⁴ + k3 r⁶) / (1 + k4 r² + k5 r⁴ + k6 r⁶) on the
    normalised image point, plus the tangential p1, p2). Keys the SDK does not give are absent."""
    k1: Optional[Number] = Field(None, description="Radial numerator, r² term (k1).")
    k2: Optional[Number] = Field(None, description="Radial numerator, r⁴ term (k2).")
    k3: Optional[Number] = Field(None, description="Radial numerator, r⁶ term (k3).")
    k4: Optional[Number] = Field(None, description="Radial denominator, r² term (k4).")
    k5: Optional[Number] = Field(None, description="Radial denominator, r⁴ term (k5).")
    k6: Optional[Number] = Field(None, description="Radial denominator, r⁶ term (k6).")
    p1: Optional[Number] = Field(None, description="Tangential coefficient p1.")
    p2: Optional[Number] = Field(None, description="Tangential coefficient p2.")
    model_config = ConfigDict(extra="allow")


class Intrinsics(BaseModel):
    """Pinhole intrinsics of one active profile, in pixels of that profile's width × height (factory values,
    the camera node's GET /description)."""
    fx: Number = Field(description="px, focal length along x.")
    fy: Number = Field(description="px, focal length along y.")
    cx: Number = Field(description="px, principal point x from the left edge.")
    cy: Number = Field(description="px, principal point y from the top edge.")
    width: int = Field(description="px, image width of the profile.")
    height: int = Field(description="px, image height of the profile.")
    distortion: Optional[Distortion] = Field(None, description="Lens distortion; absent when the SDK gives none.")
    model_config = ConfigDict(extra="allow")


class CameraIntrinsicsOut(BaseModel):
    """Intrinsics of the camera's active profiles; a stream that is off is absent."""
    depth: Optional[Intrinsics] = Field(None, description="The depth profile (unaligned depth pixels).")
    color: Optional[Intrinsics] = Field(None, description="The color profile (also aligned depth pixels, when "
                                                          "`aligned_depth`).")
    model_config = ConfigDict(extra="allow")


class CameraOut(BaseModel):
    node: str = Field(description="Where its camera node is: the configured `url`, else the node instance name.")
    serial: Optional[str] = Field(description="Device serial (from the camera node, else the last one a calibration "
                                              "saw); null if never seen.")
    nominal: NominalPose
    pose_in_base: Pose = Field(description="depth_optical in base: the nominal pose refined by the active "
                                           "calibrations.")
    from_: PoseSource = Field(alias="from", description="How pose_in_base was composed.")
    reachable: bool = Field(description="The last poll of the camera node (every 5 s) succeeded.")
    error: Optional[str] = Field(description="The last poll's error; null if fine.")
    intrinsics: Optional[CameraIntrinsicsOut] = Field(
        description="Pinhole intrinsics of the active profiles, as in the camera node's GET /description; null "
                    "before the camera node was reached.")


class Conventions(BaseModel):
    base: str = Field(description="Where base is and its axes.")
    optical: str = Field(description="Axes of the optical frames.")
    transform: str = Field(description="What a frame's pose means and the rpy convention.")


class DescriptionOut(BaseModel):
    frames: dict[str, Frame] = Field(description="Every frame by name, each with its pose in its parent.")
    frames_in_base: dict[str, list[list[float]]] = Field(
        description="4×4 pose of each frame in base (p_base = T p_frame), row-major; frames not under base are left "
                    "out.")
    robot: RobotGeometry
    cameras: dict[str, CameraOut] = Field(description="Each camera of the description by name.")
    conventions: Conventions


class Quality(BaseModel):
    """The quality gate (node.QUALITY). A run is always saved; activate=true makes it active only if ok."""
    ok: bool = Field(description="Passed the gate (no problems).")
    problems: list[str] = Field(description="Why not ok (empty if ok).")


class GroundFit(BaseModel):
    """Floor plane fit in depth_optical (solvers.fit_ground_plane)."""
    normal: list[float] = Field(description="Unit floor normal, depth_optical frame, pointing from the floor towards "
                                            "the camera.")
    height: float = Field(description="m, camera (depth_optical origin) height above the floor plane.")
    rms: Optional[float] = Field(None, description="m, rms distance of the inliers to the plane.")
    inliers: Optional[int] = Field(None, description="Points within `tol` of the plane.")
    n_points: Optional[int] = Field(None, description="Valid depth points considered.")
    angle_to_gravity_deg: Optional[float] = Field(None, description="deg between the normal and gravity (camera "
                                                                    "IMU); absent without an IMU reading.")


class AngleCheck(BaseModel):
    expected_deg: float = Field(description="deg between the two base-frame vectors.")
    measured_deg: float = Field(description="deg between their measured directions (depth_optical).")
    error_deg: float = Field(description="measured - expected, deg.")


_CHECKS_DOC = ("`angle_<i>_<j>` (i < j, vector indices): the angle between vectors i and j, expected vs measured; "
               "`floor_normal_vs_ground_deg` (deg, a number): the yaw run's floor normal vs the active ground "
               "calibration's, if both exist.")


class YawChecks(RootModel[dict[str, Union[AngleCheck, float]]]):
    """Consistency checks of a yaw run, by name: `angle_<i>_<j>` (i < j, vector indices) -> AngleCheck,
    one per pair of vectors; `floor_normal_vs_ground_deg` -> deg (a number) between the run's floor
    normal and the active ground calibration's, present only if both exist."""

_V = {
    "vector": "Unit base-frame direction [vx, vy] the base translated along.",
    "direction_depth": "Unit motion direction measured in depth_optical (the IMU line fit rotated into depth).",
    "linearity": "sqrt(λ2/λ1) of the low-passed accelerometer samples: 0 = a perfect line.",
    "linearity_unfiltered": "The same without the low-pass.",
    "sign_corr": "Normalised correlation of the line coordinate with the sign of the measured base acceleration "
                 "(fixes the direction's sign); null with too few labelled samples.",
    "amplitude": "m/s², rms of the low-passed accelerometer along the line.",
    "vibration_rms": "m/s², rms of what the low-pass removed (the base's vibration).",
    "yaw_rate_max_dps": "deg/s, peak low-passed yaw rate (about gravity) in the window.",
    "n": "IMU samples in the fitted window (the run minus half a period at each end).",
    "t_start": "s, start of the sinusoid (this machine's time.monotonic()).",
    "t_end": "s, end of the sinusoid (this machine's time.monotonic()).",
    "centroid": "m/s², mean accelerometer vector in the window (IMU frame: gravity reaction + bias).",
    "direction": "Unit line direction in the IMU frame.",
    "gyro_rms_dps": "deg/s, rms of the raw gyro in the window.",
}


class YawVectorResult(BaseModel):
    """One vector's fit, as a job result reports it."""
    vector: list[float] = Field(description=_V["vector"])
    direction_depth: list[float] = Field(description=_V["direction_depth"])
    linearity: float = Field(description=_V["linearity"])
    linearity_unfiltered: Optional[float] = Field(description=_V["linearity_unfiltered"])
    sign_corr: Optional[float] = Field(description=_V["sign_corr"])
    amplitude: float = Field(description=_V["amplitude"])
    vibration_rms: Optional[float] = Field(description=_V["vibration_rms"])
    yaw_rate_max_dps: Optional[float] = Field(description=_V["yaw_rate_max_dps"])
    n: int = Field(description=_V["n"])


class YawVectorFit(BaseModel):
    """One vector's fit as stored in a yaw record (keys added later are absent in older records)."""
    vector: list[float] = Field(description=_V["vector"])
    t_start: Optional[float] = Field(None, description=_V["t_start"])
    t_end: Optional[float] = Field(None, description=_V["t_end"])
    n: Optional[int] = Field(None, description=_V["n"])
    centroid: Optional[list[float]] = Field(None, description=_V["centroid"])
    amplitude: Optional[float] = Field(None, description=_V["amplitude"])
    linearity: Optional[float] = Field(None, description=_V["linearity"])
    direction: Optional[list[float]] = Field(None, description=_V["direction"])
    sign_corr: Optional[float] = Field(None, description=_V["sign_corr"])
    direction_depth: list[float] = Field(description=_V["direction_depth"])
    linearity_unfiltered: Optional[float] = Field(None, description=_V["linearity_unfiltered"])
    vibration_rms: Optional[float] = Field(None, description=_V["vibration_rms"])
    yaw_rate_max_dps: Optional[float] = Field(None, description=_V["yaw_rate_max_dps"])
    gyro_rms_dps: Optional[float] = Field(None, description=_V["gyro_rms_dps"])
    model_config = ConfigDict(extra="allow")


class GroundParamsOut(BaseModel):
    """Ground run parameters (as POST /calibrate/ground; `camera` only in a job's params)."""
    camera: Optional[str] = Field(None, description="Camera name.")
    polygons: Optional[list[list[list[float]]]] = Field(description="Floor polygons, [x, y] normalised image coords; "
                                                                     "null = automatic.")
    frames: int = Field(description="Depth frames median-combined.")
    max_tilt_deg: float = Field(description="deg, automatic mode: plane within this of horizontal.")
    tol: float = Field(description="m, inlier distance.")


class YawParamsOut(BaseModel):
    """Yaw run parameters (routines.YawParams; `camera` only in a job's params). A job has the
    vectors as requested, a record has them normalised."""
    camera: Optional[str] = Field(None, description="Camera name.")
    vectors: list[list[float]] = Field(description="Base-frame directions [vx, vy].")
    amplitude: float = Field(description="m; the base swings 0..2A along each vector.")
    frequency: float = Field(description="Hz.")
    cycles: int = Field(description="Sine periods per vector.")
    align_speed: float = Field(description="m/s, the nudge along the vector that aligns the wheels first.")
    align_s: float = Field(description="s, duration of that nudge.")
    settle_s: float = Field(description="s stopped before and after each run.")
    confirm: bool = Field(description="Always true (the routine refuses to drive without it).")
    accel_lowpass_hz: Optional[float] = Field(None, description="Hz, accelerometer low-pass before the line fit; "
                                                                "null = 2 × frequency.")
    model_config = ConfigDict(extra="allow")


class _ResultCore(BaseModel):
    id: str = Field(description="Id of the saved calibration record (`YYYYmmdd-HHMMSS`, `-n` if taken).")
    serial: str = Field(description="Camera serial the record is filed under.")
    summary: str = Field(description="One line for people.")
    activated: bool = Field(description="Made active: asked to (activate) and it passed the quality gate.")
    quality: Quality


class GroundResult(_ResultCore):
    """A finished ground job's result."""
    result: GroundFit = Field(description="The plane fit.")
    angle_to_yaw_floor_normal_deg: Optional[float] = Field(
        None, description="deg between this normal and the active yaw calibration's floor normal; absent if none.")


class YawResult(_ResultCore):
    """A finished yaw job's result (also POST .../resolve)."""
    checks: YawChecks = Field(description=_CHECKS_DOC)
    accel_lowpass_hz: Optional[float] = Field(description="Hz, the accelerometer low-pass used.")
    vectors: list[YawVectorResult] = Field(description="One fit per vector, in the order of `params.vectors`.")


class _JobCore(BaseModel):
    id: int = Field(description="Job id (counts up from 1 per node start; the latest 20 are kept).")
    kind: str = Field(description="ground | yaw.")
    state: str = Field(description="running | done | failed | aborted.")
    params: Any
    progress: list[str] = Field(description="Latest progress messages (up to 20), oldest first.")
    result: Any
    error: Optional[str] = Field(description="Why it failed or was aborted; null otherwise.")
    raw: Optional[str] = Field(description="Path (from the node root) of an aborted run's recording "
                                           "(`jobs/{id}/raw.npz`); null if none.")
    elapsed: float = Field(description="s since it started, until it ended (10 ms resolution).")


class GroundJob(_JobCore):
    kind: Literal["ground"] = Field(description="`ground`.")
    params: GroundParamsOut
    result: Optional[GroundResult] = Field(description="Set once done; null otherwise.")


class YawJob(_JobCore):
    kind: Literal["yaw"] = Field(description="`yaw`.")
    params: YawParamsOut
    result: Optional[YawResult] = Field(description="Set once done; null otherwise.")


JobInfo = Annotated[Union[GroundJob, YawJob], Field(discriminator="kind")]


class Job(RootModel[JobInfo]):
    """A calibration job: a GroundJob or a YawJob, by `kind`."""


class NoJob(BaseModel):
    detail: str = Field(description="`no job running`.")


class CalibrationSummary(BaseModel):
    id: str = Field(description="Calibration id (`YYYYmmdd-HHMMSS`, `-n` if taken).")
    type: str = Field(description="ground | yaw.")
    serial: str = Field(description="Camera serial.")
    active: bool = Field(description="The active one of its type for this camera.")
    t_saved: Optional[float] = Field(description="Unix epoch s when saved; null if the record lacks it.")
    summary: Optional[str] = Field(description="One line for people; null if the record lacks it.")
    quality: Optional[Quality] = Field(description="The quality gate's verdict; null in records older than the "
                                                   "gate.")
    has_raw: bool = Field(description="Raw data is stored (GET .../raw.npz).")


class CameraDevice(BaseModel):
    """The camera device (camera node)."""
    name: Optional[str] = Field(None, description="Device name reported by the SDK.")
    serial: Optional[str] = Field(None, description="Device serial.")
    firmware: Optional[str] = Field(None, description="Firmware version.")
    model_config = ConfigDict(extra="allow")


class CameraConventions(BaseModel):
    """The camera node's frame conventions."""
    optical: Optional[str] = Field(None, description="Axes of every camera frame: `x right, y down, z out of the "
                                                     "lens`.")
    units: Optional[str] = Field(None, description="Length unit of the translations: `m`.")
    transform: Optional[str] = Field(None, description="`p_parent = rotation @ p_child + translation`.")
    model_config = ConfigDict(extra="allow")


class CameraFrame(BaseModel):
    """One frame of the camera's own tree. The root (`depth_optical`) has only `parent` (null)."""
    parent: Optional[str] = Field(None, description="Parent frame name (camera-local, e.g. `depth_optical`); null "
                                                    "for the root.")
    rotation: Optional[list[list[Number]]] = Field(None, description="R of this frame in its parent, 3×3 row-major; "
                                                                     "absent on the root.")
    translation: Optional[list[Number]] = Field(None, description="m, t of this frame in its parent; absent on the "
                                                                  "root.")
    model_config = ConfigDict(extra="allow")


class CameraFrames(RootModel[dict[str, CameraFrame]]):
    """Camera-local frames by name: `depth_optical` (root), and `color_optical`, `imu` when the camera's
    calibration has them."""


class CameraSnapshot(BaseModel):
    """The camera node's GET /description at the time of the run (keys as that node returned them)."""
    kind: Optional[str] = Field(None, description="Camera node kind, `orbbec`.")
    root: Optional[str] = Field(None, description="Root frame of the camera's tree, `depth_optical`.")
    device: Optional[CameraDevice] = Field(None, description="The connected camera; null if none was open.")
    conventions: Optional[CameraConventions] = Field(None, description="The camera node's frame conventions.")
    frames: Optional[CameraFrames] = Field(None, description="Camera-local frames by name, each with its pose in "
                                                             "its parent.")
    intrinsics: Optional[CameraIntrinsicsOut] = Field(None, description="Pinhole intrinsics of the active "
                                                                        "profiles.")
    aligned_depth: Optional[bool] = Field(None, description="Depth was aligned to color (depth pixels use the "
                                                            "color intrinsics).")
    settings_version: Optional[int] = Field(None, description="The camera node's settings version the values "
                                                              "belong to; null before its first pipeline start.")
    model_config = ConfigDict(extra="allow")


class YawLimits(BaseModel):
    """The yaw routine's safety limits at the time of the run (routines.LIMITS). Keys added later are absent in
    older records."""
    amplitude_max: Optional[Number] = Field(None, description="m, largest amplitude A.")
    frequency_min: Optional[Number] = Field(None, description="Hz, lowest frequency.")
    frequency_max: Optional[Number] = Field(None, description="Hz, highest frequency.")
    peak_speed_max: Optional[Number] = Field(None, description="m/s, largest peak speed A·2πf.")
    cycles_min: Optional[Number] = Field(None, description="Fewest cycles per vector.")
    cycles_max: Optional[Number] = Field(None, description="Most cycles per vector.")
    vectors_max: Optional[Number] = Field(None, description="Most vectors per run.")
    travel_max: Optional[Number] = Field(None, description="m, largest total travel of the run.")
    yaw_rate_abort_dps: Optional[Number] = Field(None, description="deg/s; aborts when the yaw rate, averaged "
                                                                   "over yaw_rate_window_s, exceeds this.")
    yaw_rate_window_s: Optional[Number] = Field(None, description="s, averaging window of the yaw rate check.")
    heading_abort_deg: Optional[Number] = Field(None, description="deg; aborts when the integrated heading drift "
                                                                  "exceeds this.")
    stream_stall_s: Optional[Number] = Field(None, description="s; aborts when the camera or swerve stream is "
                                                               "silent this long.")
    model_config = ConfigDict(extra="allow")


_REC = {
    "camera": "The camera node's GET /description at the time of the run.",
    "version": "Description node version that made it.",
    "summary": "One line for people.",
    "camera_name": "Camera name in the robot description.",
    "id": "Calibration id (`YYYYmmdd-HHMMSS`, `-n` if taken).",
    "serial": "Camera serial.",
    "t_saved": "Unix epoch s when saved.",
    "quality": "The quality gate's verdict; absent in records older than the gate.",
}


class GroundRecord(BaseModel):
    """A ground calibration record (meta.json; key order as stored). Keys a record lacks (older versions)
    are absent; unknown keys are passed through after the known ones."""
    params: Optional[GroundParamsOut] = Field(None, description="The run's parameters.")
    result: Optional[GroundFit] = Field(None, description="The plane fit.")
    gravity_up_depth: Optional[list[float]] = Field(None, description="Mean accelerometer vector (m/s², points up "
                                                                      "when still) rotated into depth_optical; "
                                                                      "null without an IMU reading.")
    camera: Optional[CameraSnapshot] = Field(None, description=_REC["camera"])
    frame_seqs: Optional[list[int]] = Field(None, description="Sequence numbers of the depth frames used.")
    version: Optional[str] = Field(None, description=_REC["version"])
    summary: Optional[str] = Field(None, description=_REC["summary"])
    quality: Optional[Quality] = Field(None, description=_REC["quality"])
    camera_name: Optional[str] = Field(None, description=_REC["camera_name"])
    id: str = Field(description=_REC["id"])
    type: Literal["ground"] = Field(description="`ground`.")
    serial: str = Field(description=_REC["serial"])
    t_saved: float = Field(description=_REC["t_saved"])
    model_config = ConfigDict(extra="allow")


class YawRecord(BaseModel):
    """A yaw calibration record (meta.json; key order as stored). Keys a record lacks (older versions)
    are absent; unknown keys are passed through after the known ones."""
    params: Optional[YawParamsOut] = Field(None, description="The run's parameters (vectors normalised).")
    vectors: Optional[list[YawVectorFit]] = Field(None, description="One fit per vector, in the order of "
                                                                    "`params.vectors`.")
    checks: Optional[YawChecks] = Field(None, description=_CHECKS_DOC)
    floor_normal_depth: Optional[list[float]] = Field(None, description="Unit floor normal (pointing up) in "
                                                                        "depth_optical from the two most orthogonal "
                                                                        "motion lines; null with one vector.")
    gravity_up_imu: Optional[list[float]] = Field(None, description="Unit gravity reaction (up) in the IMU frame, "
                                                                    "from the still window before the run.")
    camera: Optional[CameraSnapshot] = Field(None, description=_REC["camera"])
    limits: Optional[YawLimits] = Field(None, description="The routine's safety limits at the time "
                                                          "(routines.LIMITS).")
    version: Optional[str] = Field(None, description=_REC["version"])
    accel_lowpass_hz: Optional[float] = Field(None, description="Hz, the accelerometer low-pass used.")
    summary: Optional[str] = Field(None, description=_REC["summary"])
    resolved_from: Optional[str] = Field(None, description="Id of the record this was re-solved from (POST .../resolve).")
    quality: Optional[Quality] = Field(None, description=_REC["quality"])
    camera_name: Optional[str] = Field(None, description=_REC["camera_name"])
    id: str = Field(description=_REC["id"])
    type: Literal["yaw"] = Field(description="`yaw`.")
    serial: str = Field(description=_REC["serial"])
    t_saved: float = Field(description=_REC["t_saved"])
    model_config = ConfigDict(extra="allow")


class CalibrationRecord(RootModel[Annotated[Union[GroundRecord, YawRecord], Field(discriminator="type")]]):
    """One calibration record (meta.json): a GroundRecord or a YawRecord, by `type`."""


_ACTIVE_DOC = "Active calibrations: camera serial -> {type -> calibration id}."


class ActiveMap(RootModel[dict[str, dict[Literal["ground", "yaw"], str]]]):
    """Active calibrations (active.json): camera serial -> {type (`ground` | `yaw`) -> calibration id}. A type
    with no active record is absent; a camera with none may map to {}."""


class CameraLink(BaseModel):
    reachable: bool = Field(description="The last poll of its camera node (every 5 s) succeeded.")
    error: Optional[str] = Field(description="The last poll's error; null if fine.")
    serial: Optional[str] = Field(description="Device serial from the last good poll; null if none.")


class NoLease(BaseModel):
    """This node has no lease of its own (the yaw routine takes the swerve node's)."""
    held: bool = Field(description="Always false.")
    holder: Optional[str] = Field(description="Always null.")


class DescriptionState(BaseModel):
    state: str = Field(description="ready | calibrating (a job is running).")
    error: Optional[str] = Field(description="Always null.")
    job: Optional[JobInfo] = Field(description="The latest job (running or finished); null before the first.")
    estop: EStopState
    lease: NoLease
    cameras: dict[str, CameraLink] = Field(description="Camera name -> its node's link (empty until the first poll).")
    active: ActiveMap = Field(description=_ACTIVE_DOC)
    events: list[Event] = Field(description="Latest node events (up to 50), oldest first.")
    chocked: bool = Field(description="A chock on swerve or description is in force: the yaw routine must not drive.")
    covered: bool = Field(description="A cover on the camera is in force.")
    locks: list[LockOut] = Field(description="The chocks and covers in force.")



def create_app(node: DescriptionNode, start: bool = True) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app):
        node.hub.attach(asyncio.get_running_loop())
        if start:
            node.start()
        yield
        node.close()

    app = FastAPI(
        title="Robot description node",
        description=(
            "The robot's frames (base, cameras and their internal frames) and their calibration.\n\n"
            "**Frames**: `GET /description` (all frames, each with its parent; camera poses composed from the "
            "nominal pose and the active calibrations), `GET /transform?from=&to=`.\n\n"
            "**Calibration** (jobs; one at a time): `POST /calibrate/ground` (depth plane fit: roll, pitch, height; "
            "never moves), `POST /calibrate/yaw` (**drives the base** back and forth along given vectors: yaw; needs "
            "`confirm: true`, the swerve lease free; aborts on yaw rate/heading drift). Progress in `GET /jobs/{id}` "
            "and `/state`. Records: `GET /calibrations`, `POST /calibrations/{type}/activate`.\n\n"
            "`POST /estop` (anyone) aborts a running job and e-stops the swerve node too."),
        version=__version__, lifespan=lifespan, openapi_tags=TAGS)
    setup_app(app)
    add_base_routes(app, lambda: {"node": "description", "state": node.state()["state"]})
    add_page_routes(app, node_meta("description", "Robot description", __version__, "description", actuated=True,
                                   moves="swerve base (yaw calibration only)"))

    def bad(e: Exception, code: int = 400):
        return HTTPException(code, str(e))

    @app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def index():
        from .page import PAGE
        return PAGE

    @app.get("/state", response_model=DescriptionState, response_model_exclude_unset=True, tags=["state"],
             summary="Status, running job, e-stop, cameras, active calibrations")
    async def get_state():
        return node.state()

    @app.get("/events", tags=["state"], summary="Server-Sent Events: /state on every change and every 1 s",
             response_class=StreamingResponse,
             responses={200: {"content": {"text/event-stream": {}},
                              "description": "Unnamed events; each `data:` line is the GET /state JSON "
                                             "(DescriptionState)."}})
    async def events():
        return StreamingResponse(sse(node.hub, node.state), media_type="text/event-stream", headers={"Cache-Control": "no-store"})

    # -- description -----------------------------------------------------------------------------

    @app.get("/description", response_model=DescriptionOut, response_model_exclude_unset=True, tags=["description"],
             summary="All frames, cameras and conventions")
    async def description():
        return await asyncio.to_thread(node.description)

    @app.get("/transform", response_model=Pose, tags=["description"],
             summary="Pose of frame `to` in frame `from` (p_from = T p_to)")
    async def transform(frm: str = Query("base", alias="from"), to: str = Query("camera/depth_optical")):
        try:
            return node.transform(frm, to)
        except (KeyError, ValueError) as e:
            raise bad(e)

    @app.post("/cameras/{name}/nominal", response_model=CameraOut, response_model_exclude_unset=True,
              tags=["description"], summary="Set a camera's measured/nominal pose in base")
    async def set_nominal(name: str, body: NominalIn):
        """x and y (and anything not calibrated) come from here: measure them. Saved on the robot."""
        try:
            return node.set_nominal(name, body.xyz, body.rpy)
        except KeyError as e:
            raise bad(e, 404)

    @app.get("/cameras/{name}/depth.jpg", tags=["description"], response_class=Response,
             summary="The camera's current depth image (for drawing ground polygons)",
             responses={200: {"content": {"image/jpeg": {}},
                              "description": "The camera node's GET /snapshot/depth?format=jpg, passed through; "
                                             "header `X-Frame-Header` carries its frame header."}})
    async def depth_jpg(name: str):
        from ..common.client import NodeClient
        from .node import target
        cams = node.store.description().get("cameras") or {}
        if name not in cams:
            raise bad(KeyError(name), 404)

        def fetch():
            with NodeClient(target(cams[name]), timeout=5.0) as c:
                return c.http.get("/snapshot/depth", params={"format": "jpg"})
        r = await asyncio.to_thread(fetch)
        if r.status_code >= 400:
            raise HTTPException(503, f"camera: {r.text[:200]}")
        return Response(r.content, media_type="image/jpeg", headers={"Cache-Control": "no-store",
                                                                       "X-Frame-Header": r.headers.get("X-Frame-Header", "")})

    # -- calibration -----------------------------------------------------------------------------

    @app.post("/calibrate/ground", response_model=GroundJob, response_model_exclude_unset=True, tags=["calibration"],
              summary="Ground-plane calibration (never moves): starts a job")
    async def calibrate_ground(body: GroundIn = GroundIn()):
        try:
            return node.start_ground(body.camera, GroundParams(body.polygons, body.frames, body.max_tilt_deg, body.tol),
                                     body.activate)
        except (KeyError, RuntimeError, ValueError) as e:
            raise bad(e, 409 if isinstance(e, RuntimeError) else 400)

    @app.post("/calibrate/yaw", response_model=YawJob, response_model_exclude_unset=True, tags=["calibration"],
              summary="Camera-yaw calibration: DRIVES THE BASE; starts a job")
    async def calibrate_yaw(body: YawIn):
        """Translates the base back and forth along each vector (omega always 0) while recording the
        camera IMU; aborts on yaw rate > 15°/s (0.1 s mean) or heading drift > 5° (the robot may be tethered), lost
        streams, or `POST /estop` / `POST /jobs/abort`. Limits: amplitude ≤ 0.15 m, peak speed
        A·2πf ≤ 0.8 m/s, total travel ≤ 12 m."""
        try:
            return node.start_yaw(body.camera, YawParams(body.vectors, body.amplitude, body.frequency, body.cycles,
                                                         confirm=body.confirm, accel_lowpass_hz=body.accel_lowpass_hz),
                                  body.activate)
        except (KeyError, RuntimeError, ValueError) as e:
            raise bad(e, 409 if isinstance(e, RuntimeError) else 400)

    @app.get("/jobs", response_model=list[JobInfo], response_model_exclude_unset=True, tags=["calibration"],
             summary="Recent jobs")
    async def jobs():
        return [j.info() for j in node.jobs.values()]

    @app.get("/jobs/{jid}", response_model=Job, response_model_exclude_unset=True, tags=["calibration"],
             summary="One job: state, progress, result")
    async def job(jid: int):
        j = node.jobs.get(jid)
        if j is None:
            raise bad(KeyError(f"no job {jid}"), 404)
        return j.info()

    @app.get("/jobs/{jid}/raw.npz", tags=["calibration"], summary="What an aborted job recorded (numpy .npz)",
             response_class=FileResponse,
             responses={200: {"content": {"application/octet-stream": {}},
                              "description": "A numpy .npz archive (`numpy.load`)."}})
    async def job_raw(jid: int):
        """IMU (``imu_*``) and swerve tick (``swerve_*``) rows, phase marks (``marks``, ``marks_t``, local
        monotonic s), the monitor's gravity/bias reference and limits. Also kept in the data directory."""
        j = node.jobs.get(jid)
        if j is None or j.raw_path is None or not j.raw_path.exists():
            raise bad(KeyError(f"no recording for job {jid}"), 404)
        return FileResponse(j.raw_path, media_type="application/octet-stream", filename=j.raw_path.name)

    @app.post("/jobs/abort", response_model=Union[JobInfo, NoJob], response_model_exclude_unset=True, tags=["control"],
              summary="Abort the running job (stops and releases the base)")
    async def abort():
        return node.abort("abort requested") or {"detail": "no job running"}

    @app.get("/calibrations", response_model=list[CalibrationSummary], tags=["calibration"],
             summary="Calibration records (newest first), active flagged")
    async def calibrations(type: Optional[str] = None, serial: Optional[str] = None):
        return node.store.list(type, serial)

    @app.get("/calibrations/{kind}/{serial}/{cid}", response_model=CalibrationRecord, response_model_exclude_unset=True,
             tags=["calibration"], summary="One calibration record (meta.json)")
    async def calibration(kind: str, serial: str, cid: str):
        try:
            return node.store.get(kind, serial, cid)
        except KeyError as e:
            raise bad(e, 404)

    @app.get("/calibrations/{kind}/{serial}/{cid}/raw.npz", tags=["calibration"], summary="Raw data of a record (numpy .npz)",
             response_class=FileResponse,
             responses={200: {"content": {"application/octet-stream": {}},
                              "description": "A numpy .npz archive (`numpy.load`): ground `depth_mm` (uint16) and "
                                             "`mask`; yaw `imu_*`, `swerve_*` columns and `windows`."}})
    async def calibration_raw(kind: str, serial: str, cid: str):
        p = node.store.raw_path(kind, serial, cid)
        if not p.exists():
            raise bad(KeyError("no raw data"), 404)
        return FileResponse(p, media_type="application/octet-stream", filename=f"{kind}-{cid}.npz")

    @app.post("/calibrations/yaw/{serial}/{cid}/resolve", response_model=YawResult, response_model_exclude_unset=True,
              tags=["calibration"],
              summary="Solve a stored yaw run again from its raw data (nothing moves); saves a new record")
    async def resolve_yaw(serial: str, cid: str, body: ResolveIn = ResolveIn()):
        try:
            return await asyncio.to_thread(node.resolve_yaw, serial, cid, body.accel_lowpass_hz, body.activate)
        except KeyError as e:
            raise bad(e, 404)

    @app.post("/calibrations/{kind}/activate", response_model=ActiveMap, tags=["calibration"],
              response_description=_ACTIVE_DOC, summary="Choose the active calibration of a type for a camera")
    async def activate(kind: str, body: ActivateIn):
        if kind not in TYPES:
            raise bad(ValueError(f"type must be one of {TYPES}"))
        try:
            node.activate(kind, body.serial, body.id)
        except KeyError as e:
            raise bad(e, 404)
        return node.store.active()

    # -- e-stop ----------------------------------------------------------------------------------

    @app.post("/estop", response_model=EStopTriggered, tags=["control"],
              summary="E-stop: aborts any job and e-stops the swerve node (anyone)")
    async def estop(request: Request, body: Optional[EStopIn] = None):
        body = body or EStopIn()
        src = body.source or (request.client.host if request.client else "")
        info = await asyncio.to_thread(node.estop.trigger, body.reason, src)
        return {"latched": True, "reason": info.reason, "source": info.source}

    @app.post("/estop/clear", response_model=EStopCleared, tags=["control"],
              summary="Clear this node's e-stop (the swerve node's is separate)")
    async def estop_clear():
        node.estop.clear()
        node.events.add("e-stop cleared")
        node.hub.state_changed()
        return {"latched": False}

    return app
