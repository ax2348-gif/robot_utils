"""Pydantic models of the API and the pure settings logic (validated against the device catalog).

Nothing here touches the SDK, so it is unit-testable without a camera.
"""
from __future__ import annotations

from typing import Literal, Optional, Union

from pydantic import BaseModel, ConfigDict, Field, model_validator

StreamName = Literal["color", "depth", "ir_left", "ir_right"]
STREAM_NAMES: tuple[str, ...] = ("color", "depth", "ir_left", "ir_right")
STREAM_LABELS = {"color": "Color", "depth": "Depth", "ir_left": "Left IR", "ir_right": "Right IR"}
STEREO = ("depth", "ir_left", "ir_right")    # share one resolution/fps on the device
# Changing any of these restarts the video pipeline.
PIPELINE_KEYS = ("streams", "color_profile", "stereo_mode", "align", "frame_sync")
IMU_RESTART_KEYS = ("enabled", "sample_rate", "accel_range", "gyro_range")   # filter changes apply live
# The colour profile the node starts with (the device default if the device lacks it). Not the
# Gemini's own default (1280x720@10): at that size a full-resolution look in agent_interface held
# the arm's control loop long enough for its readings to go stale and the arm to fault (2026-11-13).
DEFAULT_COLOR_PROFILE = "640x480@15 MJPG"


class SettingsError(ValueError):
    """A settings change that is invalid for the connected device (HTTP 400)."""


class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid")


# -- settings ----------------------------------------------------------------------------------

class DisplaySettings(_Strict):
    """How depth/IR are rendered for the *monitoring* (``mode=jpeg``) encoding. Raw delivery is
    unaffected. Applies to the next frame, no pipeline restart."""
    colormap: str = Field("turbo", description="Colormap name, one of `GET /catalog` → `colormaps`.")
    invert: bool = Field(False, description="Reverse the colormap (near = high end).")
    range_mode: Literal["auto", "manual"] = Field(
        "auto", description="`auto`: map the pct_lo..pct_hi percentiles of valid depth; `manual`: min_mm..max_mm.")
    pct_lo: float = Field(2.0, ge=0, le=100)
    pct_hi: float = Field(98.0, ge=0, le=100)
    min_mm: float = Field(200.0, ge=0)
    max_mm: float = Field(3000.0, ge=0)
    invalid_color: str = Field("#000000", pattern=r"^#[0-9a-fA-F]{6}$",
                               description="Color of pixels without depth (#rrggbb).")
    jpeg_quality: int = Field(80, ge=30, le=95, description="Quality of server-encoded JPEGs (depth/IR). "
                                                            "Color MJPG is passed through and is unaffected.")

    @model_validator(mode="after")
    def _ranges(self):
        if not self.pct_lo < self.pct_hi:
            raise ValueError("need pct_lo < pct_hi")
        if not self.min_mm < self.max_mm:
            raise ValueError("need min_mm < max_mm")
        return self


class ImuFilterSettings(_Strict):
    """Attitude filter (Mahony + gyro-bias tracking) run on every raw sample inside the node."""
    output_rate_hz: float = Field(100.0, ge=10, le=500, description="Records per second on `/imu/stream`.")
    kp: float = Field(0.5, ge=0, le=20, description="Gravity correction gain, 1/s (tilt time constant ~1/kp s).")
    ki: float = Field(0.02, ge=0, le=2, description="Gyro-bias tracking gain through the gravity error, 1/s².")
    accel_gate: float = Field(0.5, gt=0, le=5, description="Use the accelerometer only when its magnitude is "
                                                           "within this many m/s² of the calibrated gravity.")
    zaru: bool = Field(True, description="Zero-rate update: re-estimate all three gyro biases (incl. yaw) "
                                         "while the IMU is still.")
    zaru_tau_s: float = Field(5.0, ge=0.5, le=600, description="Time constant of the zero-rate update.")
    calib_s: float = Field(1.0, ge=0.2, le=30, description="Still time used by a calibration.")


class ImuSettings(_Strict):
    """Accelerometer + gyroscope. Rates/ranges come from `GET /catalog` → `imu`."""
    enabled: bool = Field(True, description="Run the IMU (its own SDK pipeline; independent of video).")
    sample_rate: Optional[str] = Field(None, description="Raw sample rate the node runs the IMU at, e.g. "
                                       "`1kHz`; null = the catalog's default (1 kHz when offered). The filtered "
                                       "stream rate is `filter.output_rate_hz`.", examples=["1kHz"])
    accel_range: Optional[str] = Field(None, description="Accelerometer full scale, e.g. `4g`.")
    gyro_range: Optional[str] = Field(None, description="Gyroscope full scale, e.g. `1000dps`.")
    filter: ImuFilterSettings = Field(default_factory=ImuFilterSettings,
                                        description="Attitude filter; changes apply at once (no restart).")


class Settings(_Strict):
    """The node's complete, global configuration (one pipeline shared by all clients)."""
    streams: dict[StreamName, bool] = Field(
        default_factory=lambda: {"color": True, "depth": True, "ir_left": False, "ir_right": False},
        description="Which video streams the camera runs.")
    color_profile: Optional[str] = Field(DEFAULT_COLOR_PROFILE, description="Key from `GET /catalog` → `color`, "
                                                                            "e.g. `640x480@15 MJPG` (the node's default). "
                                                                            "null or a profile the device lacks = the device "
                                                                            "default.")
    stereo_mode: Optional[str] = Field(None, description="Depth/IR mode from `GET /catalog` → `stereo`, "
                                                         "e.g. `848x480@10` (depth and both IRs share it).")
    align: bool = Field(False, description="Align depth to color (SDK software D2C). Aligned depth has the "
                                           "color resolution; implies frame_sync.")
    frame_sync: bool = Field(False, description="SDK frame sync: deliver color and depth of the same instant "
                                                "together. Off = each frame as soon as it arrives (lower latency).")
    display: DisplaySettings = Field(default_factory=DisplaySettings)
    imu: ImuSettings = Field(default_factory=ImuSettings)

    model_config = ConfigDict(extra="forbid", json_schema_extra={"examples": [{
        "streams": {"color": True, "depth": True, "ir_left": False, "ir_right": False},
        "color_profile": "640x480@15 MJPG", "stereo_mode": "848x480@10", "align": False,
        "frame_sync": False, "display": DisplaySettings().model_dump(), "imu": ImuSettings().model_dump()}]})


class DisplayPatch(_Strict):
    colormap: Optional[str] = None
    invert: Optional[bool] = None
    range_mode: Optional[Literal["auto", "manual"]] = None
    pct_lo: Optional[float] = None
    pct_hi: Optional[float] = None
    min_mm: Optional[float] = None
    max_mm: Optional[float] = None
    invalid_color: Optional[str] = None
    jpeg_quality: Optional[int] = None


class ImuPatch(_Strict):
    enabled: Optional[bool] = None
    sample_rate: Optional[str] = None
    accel_range: Optional[str] = None
    gyro_range: Optional[str] = None
    filter: Optional["ImuFilterPatch"] = None


class ImuFilterPatch(_Strict):
    output_rate_hz: Optional[float] = None
    kp: Optional[float] = None
    ki: Optional[float] = None
    accel_gate: Optional[float] = None
    zaru: Optional[bool] = None
    zaru_tau_s: Optional[float] = None
    calib_s: Optional[float] = None


ImuPatch.model_rebuild()


class ImuCalibrateRequest(_Strict):
    seconds: Optional[float] = Field(None, ge=0.2, le=30, description="Still time to use; null = filter.calib_s.")


class SettingsPatch(_Strict):
    """Partial update: only the given fields change. Unknown keys are rejected."""
    streams: Optional[dict[StreamName, bool]] = Field(None, description="Only the listed streams change.")
    color_profile: Optional[str] = None
    stereo_mode: Optional[str] = None
    align: Optional[bool] = None
    frame_sync: Optional[bool] = None
    display: Optional[DisplayPatch] = None
    imu: Optional[ImuPatch] = None

    model_config = ConfigDict(extra="forbid", json_schema_extra={"examples": [
        {"streams": {"color": True, "depth": True}, "stereo_mode": "640x400@30", "color_profile": "640x480@30 MJPG"},
        {"display": {"colormap": "jet", "range_mode": "manual", "min_mm": 300, "max_mm": 2000}},
        {"streams": {"ir_left": True, "ir_right": True}},
        {"imu": {"enabled": True, "sample_rate": "1kHz"}},
        {"imu": {"filter": {"kp": 0.3, "accel_gate": 0.3}}},
    ]})


# -- catalog -----------------------------------------------------------------------------------

class ProfileOption(BaseModel):
    key: str = Field(description="Value to use in settings, e.g. `1280x720@10 MJPG` or `848x480@10`.")
    width: int
    height: int
    fps: int
    format: Optional[str] = None
    default: bool = False


class ImuCatalog(BaseModel):
    available: bool = False
    sample_rates: list[str] = []
    default_rate: Optional[str] = None
    accel_ranges: list[str] = []
    gyro_ranges: list[str] = []
    extrinsics: dict = Field({}, description="From the device calibration: `accel_to_color`, `accel_to_depth`, "
                                             "`gyro_to_accel`, each {rotation: 3x3 row-major, translation_mm}. "
                                             "Maps a point in the first frame into the second.")
    intrinsics: dict = Field({}, description="Factory IMU intrinsics as reported by the device (`accel`, `gyro`: "
                                             "bias, noise_density, random_walk, scale_misalignment, ...). The node "
                                             "does not apply them.")


class Catalog(BaseModel):
    """What the connected device offers (empty while no camera is connected)."""
    available: list[StreamName] = Field([], description="Video streams the device has.")
    color: list[ProfileOption] = Field([], description="Color profiles (MJPG first-class; uncompressed "
                                                       "formats deliver nothing on USB 2).")
    stereo: list[ProfileOption] = Field([], description="Depth/IR modes common to depth, left IR and right IR.")
    imu: ImuCatalog = Field(default_factory=ImuCatalog)
    colormaps: dict[str, list[str]] = Field({}, description="Colormap name -> a few #rrggbb stops (for a legend).")
    catalog_id: int = Field(0, description="Changes when the device is (re)opened.")


def usb2_link(connection: str) -> bool:
    """The device reports a USB 2 connection (e.g. `USB2.1`): limited bandwidth, MJPG colour only."""
    return connection.replace(" ", "").upper().startswith("USB2")


def fix_settings(s: Settings, cat: Catalog) -> Settings:
    """Make settings valid for the connected device: unknown profiles become the device default,
    unavailable streams are switched off."""
    d = s.model_dump()
    for n in STREAM_NAMES:
        d["streams"][n] = bool(d["streams"].get(n)) and n in cat.available
    color = {p.key for p in cat.color}
    if d["color_profile"] not in color:
        d["color_profile"] = next((p.key for p in cat.color if p.default), cat.color[0].key if cat.color else None)
    stereo = {p.key for p in cat.stereo}
    if d["stereo_mode"] not in stereo:
        d["stereo_mode"] = next((p.key for p in cat.stereo if p.default), cat.stereo[0].key if cat.stereo else None)
    imu = d["imu"]
    if cat.imu.available:
        if imu["sample_rate"] not in cat.imu.sample_rates:
            imu["sample_rate"] = cat.imu.default_rate or (cat.imu.sample_rates[0] if cat.imu.sample_rates else None)
        if imu["accel_range"] not in cat.imu.accel_ranges:
            imu["accel_range"] = cat.imu.accel_ranges[0] if cat.imu.accel_ranges else None
        if imu["gyro_range"] not in cat.imu.gyro_ranges:
            imu["gyro_range"] = cat.imu.gyro_ranges[0] if cat.imu.gyro_ranges else None
    return Settings.model_validate(d)


def apply_patch(cur: Settings, patch: SettingsPatch, cat: Catalog) -> tuple[Settings, bool, bool]:
    """Validate ``patch`` against the catalog and merge it into ``cur``.

    Returns (new settings, video pipeline must restart, IMU must restart).
    Raises SettingsError with a human-readable message.
    """
    d = cur.model_dump()
    p = patch.model_dump(exclude_unset=True)
    connected = bool(cat.available)
    for n, on in (p.pop("streams", None) or {}).items():
        if on and n not in cat.available:
            raise SettingsError(f"stream {n!r} is not available" + ("" if connected else " (camera not connected)"))
        d["streams"][n] = bool(on)
    if "color_profile" in p:
        v = p.pop("color_profile")
        if v not in {o.key for o in cat.color}:
            raise SettingsError(f"unknown color profile {v!r}" + ("" if connected else " (camera not connected)")
                                + "; see GET /catalog")
        d["color_profile"] = v
    if "stereo_mode" in p:
        v = p.pop("stereo_mode")
        if v not in {o.key for o in cat.stereo}:
            raise SettingsError(f"unknown depth/IR mode {v!r}" + ("" if connected else " (camera not connected)")
                                + "; see GET /catalog")
        d["stereo_mode"] = v
    for k in ("align", "frame_sync"):
        if k in p:
            d[k] = bool(p.pop(k))
    disp = p.pop("display", None) or {}
    if "colormap" in disp and cat.colormaps and disp["colormap"] not in cat.colormaps:
        raise SettingsError(f"unknown colormap {disp['colormap']!r}; one of {sorted(cat.colormaps)}")
    d["display"].update({k: v for k, v in disp.items() if v is not None})
    imu = p.pop("imu", None) or {}
    imu = {k: v for k, v in imu.items() if v is not None}
    if imu.get("enabled") and not cat.imu.available:
        raise SettingsError("the device has no IMU" + ("" if connected else " (camera not connected)"))
    for k, opts in (("sample_rate", cat.imu.sample_rates), ("accel_range", cat.imu.accel_ranges),
                    ("gyro_range", cat.imu.gyro_ranges)):
        if k in imu and imu[k] not in opts:
            raise SettingsError(f"unknown IMU {k} {imu[k]!r}; one of {opts}")
    filt = imu.pop("filter", None) or {}
    d["imu"].update(imu)
    d["imu"]["filter"].update({k: v for k, v in filt.items() if v is not None})
    assert not p, p
    try:
        new = Settings.model_validate(d)
    except Exception as e:   # pydantic ValidationError -> one line
        errs = getattr(e, "errors", None)
        msg = "; ".join(f"{'.'.join(map(str, x['loc']))}: {x['msg']}" for x in errs()) if errs else str(e)
        raise SettingsError(msg) from None
    restart = any(getattr(new, k) != getattr(cur, k) for k in PIPELINE_KEYS)
    restart_imu = any(getattr(new.imu, k) != getattr(cur.imu, k) for k in IMU_RESTART_KEYS)
    return new, restart, restart_imu


def describe(s: Settings) -> str:
    parts = []
    if s.streams.get("color"):
        parts.append(f"color {s.color_profile}")
    stereo = [STREAM_LABELS[n].lower() for n in STEREO if s.streams.get(n)]
    if stereo:
        parts.append(f"{'+'.join(stereo)} {s.stereo_mode}")
    if s.align and s.streams.get("depth") and s.streams.get("color"):
        parts.append("depth aligned to color")
    elif s.frame_sync:
        parts.append("frame sync")
    return ", ".join(parts) or "no video streams"


# -- responses ---------------------------------------------------------------------------------

class TimeResponse(BaseModel):
    """Server clocks, read in one go (for NTP-style sync)."""
    mono: float = Field(description="Server time.monotonic() (s). The reference clock of every timestamp.")
    wall: float = Field(description="Server time.time() (Unix epoch s).")
    epoch_minus_mono: float = Field(description="wall - mono at this instant.")
    model_config = ConfigDict(json_schema_extra={"examples": [
        {"mono": 18234.512345, "wall": 1794019145.300510, "epoch_minus_mono": 1794000910.788165}]})


class Event(BaseModel):
    t: float = Field(description="Server monotonic s.")
    wall: float
    msg: str
    error: bool = False


class DeviceInfo(BaseModel):
    name: str = ""
    serial: str = ""
    firmware: str = ""
    connection: str = ""
    global_timestamp: bool = Field(False, description="SDK maps device timestamps to host time (t_sensor).")


class StreamStatus(BaseModel):
    label: str
    available: bool
    enabled: bool
    fps: float = Field(description="Frames/s published over the last ~3 s.")
    width: int = 0
    height: int = 0
    format: str = ""
    seq: int = Field(0, description="Last published sequence number.")
    age_ms: Optional[float] = Field(None, description="Now minus capture time of the last frame.")


class ImuStatus(BaseModel):
    available: bool = False
    enabled: bool = False
    running: bool = False
    rate_hz: float = Field(0.0, description="Achieved sample rate over the last ~3 s.")
    samples: int = 0
    gaps: int = Field(0, description="Samples missing according to the device frame index.")
    last: Optional[dict] = Field(None, description="Last sample: accel m/s^2, gyro rad/s, temperature C.")
    error: Optional[str] = None
    filter: Optional[dict] = Field(None, description="Attitude filter: q (IMU->world, w x y z), rpy_deg, "
                                                     "gyro_bias rad/s, calibrated/still/initializing/"
                                                     "calibrating, calibration (bias, g_ref, source).")


class CpuStatus(BaseModel):
    process_pct: float = Field(description="This process, % of one core, last ~2 s.")
    load1: float
    cores: int


class State(BaseModel):
    state: str = Field(description="starting | streaming | covered | idle | restarting | reconnecting | no device | stopped")
    covered: bool = Field(False, description="A cover lock is in force: no images are captured; every stream "
                          "serves procedural frames (headers carry covered: true). The IMU keeps running.")
    locks: list[dict] = Field([], description="The cover locks in force (common/locks.py).")
    error: Optional[str] = None
    events: list[Event] = []
    device: Optional[DeviceInfo] = None
    settings: Settings
    settings_version: int
    catalog_id: int
    streams: dict[str, StreamStatus]
    imu: ImuStatus
    clients: int = Field(description="Open streaming responses.")
    cpu: Optional[CpuStatus] = None
    server_mono: float
    server_wall: float


class SettingsResult(BaseModel):
    ok: bool = True
    settings: Settings
    settings_version: int
    restarted: bool = Field(description="The video pipeline was restarted.")
    t_applied: float = Field(description="Server monotonic s when the change took effect "
                                         "(frames captured after this use the new settings).")


class PropertyInfo(BaseModel):
    group: str
    id: str = Field(description="OBPropertyID name, e.g. OB_PROP_COLOR_EXPOSURE_INT.")
    label: str
    type: Literal["bool", "int", "float", "error"]
    value: Union[bool, int, float, None] = None
    min: Optional[float] = None
    max: Optional[float] = None
    step: Optional[float] = None
    default: Optional[float] = None
    error: Optional[str] = None


class PropertySet(_Strict):
    id: str = Field(description="OBPropertyID name from `GET /properties`.")
    value: Union[bool, int, float]
    model_config = ConfigDict(extra="forbid", json_schema_extra={"examples": [
        {"id": "OB_PROP_COLOR_AUTO_EXPOSURE_BOOL", "value": False},
        {"id": "OB_PROP_COLOR_EXPOSURE_INT", "value": 100}]})


class ErrorResponse(BaseModel):
    detail: str


class CameraIntrinsics(BaseModel):
    """Pinhole intrinsics of one active profile, in pixels of that profile's width x height (factory values)."""
    fx: float = Field(description="Focal length along x, px.")
    fy: float = Field(description="Focal length along y, px.")
    cx: float = Field(description="Principal point x, px from the left edge.")
    cy: float = Field(description="Principal point y, px from the top edge.")
    width: int = Field(description="Image width of the profile, px.")
    height: int = Field(description="Image height of the profile, px.")
    distortion: Optional[dict[str, float]] = Field(None, description="Lens distortion coefficients as the SDK "
                                                   "reports them (k1..k6 radial, p1, p2 tangential; unitless). "
                                                   "Absent when the SDK gives none.")


class Extrinsic(BaseModel):
    """Rigid transform from frame A into frame B: ``p_B = rotation @ p_A + translation_mm``."""
    rotation: list[list[float]] = Field(description="3x3 rotation, row-major.")
    translation_mm: list[float] = Field(description="[x, y, z] translation, mm.")


class CalibrationImu(BaseModel):
    accel_to_depth: Optional[Extrinsic] = Field(description="Maps an IMU (accelerometer) frame point into the "
                                                            "depth frame; null if the device reports none.")


class CameraCalibration(BaseModel):
    """``GET /calibration``: factory calibration of the profiles in use, recomputed when the pipeline (re)starts.
    Frames: x right, y down, z out of the lens. Empty (`{}`) until the first pipeline start; keys a profile
    does not provide are absent."""
    depth: Optional[CameraIntrinsics] = Field(None, description="Intrinsics of the active depth/IR profile; "
                                              "absent if none.")
    color: Optional[CameraIntrinsics] = Field(None, description="Intrinsics of the active color profile; "
                                              "absent if none.")
    depth_to_color: Optional[Extrinsic] = Field(None, description="Maps a depth-frame point into the color frame; "
                                                "absent unless both depth and color profiles are known.")
    aligned_depth: Optional[bool] = Field(None, description="Depth is aligned to color (depth pixels use the "
                                          "color intrinsics).")
    imu: Optional[CalibrationImu] = None
    settings_version: Optional[int] = Field(None, description="`settings_version` (GET /state) the values "
                                            "belong to.")


class DescriptionDevice(BaseModel):
    name: str = Field(description="Device name reported by the SDK.")
    serial: str
    firmware: str


class DescriptionConventions(BaseModel):
    optical: str = Field(description="Axis convention of every frame: `x right, y down, z out of the lens`.")
    units: str = Field(description="Length unit of the translations: `m`.")
    transform: str = Field(description="How a frame's pose is applied: `p_parent = rotation @ p_child + translation`.")


class DescriptionFrame(BaseModel):
    """One frame of the tree. The root has only `parent` (null)."""
    parent: Optional[str] = Field(description="Parent frame name; null for the root.")
    rotation: Optional[list[list[float]]] = Field(None, description="3x3 row-major rotation of this frame in "
                                                  "its parent (1e-9 resolution); absent on the root.")
    translation: Optional[list[float]] = Field(None, description="[x, y, z] position of this frame's origin in "
                                               "its parent, m (1 µm resolution); absent on the root.")


class CameraDescription(BaseModel):
    """``GET /description``: the camera's frame tree (metres, optical convention) and intrinsics."""
    kind: str = Field(description="Always `orbbec`.")
    root: str = Field(description="Root frame, always `depth_optical` (the frame depth pixels unproject into).")
    device: Optional[DescriptionDevice] = Field(description="The connected camera; null before one was opened.")
    conventions: DescriptionConventions
    frames: dict[str, DescriptionFrame] = Field(description="By name: `depth_optical` (root), `color_optical` "
                                                "and `imu` when the calibration has them.")
    intrinsics: dict[str, CameraIntrinsics] = Field(description="`depth` / `color` pinhole intrinsics of the "
                                                    "active profiles, as GET /calibration.")
    aligned_depth: bool = Field(description="Depth is aligned to color (use the color intrinsics for depth).")
    settings_version: Optional[int] = Field(description="`settings_version` the values belong to; null before "
                                            "the first pipeline start.")


class DepthAt(BaseModel):
    """``GET /depth_at``. Only `mm` (null) when there is no depth frame yet."""
    u: Optional[int] = Field(None, description="Pixel column in the latest depth frame (absent without a frame).")
    v: Optional[int] = Field(None, description="Pixel row in the latest depth frame (absent without a frame).")
    seq: Optional[int] = Field(None, description="Sequence number of that depth frame (absent without a frame).")
    mm: Optional[float] = Field(description="Depth at (u, v), mm (raw x depth_scale, 0.1 mm resolution); null if "
                                            "the pixel has no depth or there is no depth frame.")
    median5: Optional[float] = Field(None, description="Median of the valid depths in the 5x5 window around "
                                     "(u, v), mm (0.1 mm resolution); null if none is valid.")


class ImuFieldsOut(BaseModel):
    """``GET /imu/fields``: the column description shared with the other nodes' ``GET /fields``."""
    kind: Literal["state", "raw"] = Field(description="`state`: filter records; `raw`: raw samples.")
    fields: list[str] = Field(description="Column names, in payload order.")
    units: list[str] = Field(description="Unit of each column (s, us, m/s^2, rad/s, rad, m/s, degC; count, bits, "
                                         "fraction, 1 = unitless).")
    doc: dict[str, str] = Field(description="Meaning of each column.")


class ImuLatest(BaseModel):
    """``GET /imu/latest``: the last n IMU rows as JSON (oldest first)."""
    kind: Literal["state", "raw"] = Field(description="`state`: filter records; `raw`: raw samples.")
    fields: list[str] = Field(description="Column names, in row order (STATE_FIELDS or IMU_FIELDS).")
    units: dict[str, str] = Field(description="Short unit map, as in the /imu/stream headers. Keys are column "
                                              "names or, for `state`, patterns (`q*`, `da*`, `a*`, ...).")
    samples: list[list[Optional[float]]] = Field(description="Rows of len(fields) values; NaN sent as null.")


class ImuCalibrationOut(BaseModel):
    """``POST /imu/calibrate``: the gyro bias / gravity calibration now in use (saved per camera serial)."""
    gyro_bias: list[float] = Field(description="[x, y, z] gyro bias, rad/s, IMU frame (mean of the still window).")
    g_ref: float = Field(description="Gravity magnitude the accelerometer reads, m/s^2 (mean |a| of the window).")
    gyro_std: list[float] = Field(description="[x, y, z] gyro standard deviation over the window, rad/s.")
    accel_std: float = Field(description="Standard deviation of |a| over the window, m/s^2.")
    temperature: Optional[float] = Field(description="Mean IMU temperature over the window, degC; null if not "
                                                   "reported (and on the default calibration).")
    source: str = Field(description="default | stored | startup | request.")


class ImuResetOut(BaseModel):
    ok: bool = Field(description="Always true (the reset happens on the next IMU batch).")
