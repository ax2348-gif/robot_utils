"""HTTP API of the 3d_perception node (FastAPI). Conventions: docs/nodes.md; guide: docs/perception3d_node.md."""
from __future__ import annotations

import asyncio
import json
import struct
import time
from contextlib import asynccontextmanager
from typing import Annotated, Any, Literal, Optional, Union

from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import HTMLResponse, Response, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field, WithJsonSchema

from ..common.protocol import pack_record
from ..common.server import PING_S, Event, add_base_routes, add_page_routes, node_meta, setup_app, sse
from . import __version__
from .node import Fused, PerceptionNode

KIND = "3d_perception"

TAGS = [
    {"name": "state", "description": "Status, inputs, timing, events, calibration."},
    {"name": "frames", "description": "Fused RGB-D frames: colour + depth registered into the colour image."},
    {"name": "query", "description": "Depth and 3D point at a colour pixel (for click-to-servo tools)."},
    {"name": "control", "description": "Registration settings."},
]

FRAME_DOC = (
    "Binary record (docs/nodes.md framing): `[u32 BE header length][JSON header][payload]`, header "
    "`type: \"fused\"`. Payload = `color` bytes (the camera's MJPG, untouched) followed by `depth` "
    "(uint16 little-endian, mm, **z in the colour camera frame** at `t_color`, 0 = none, on the grid "
    "`depth.width` x `depth.height` = colour x `scale`); `header.color.offset/len` and "
    "`header.depth.offset/len` locate them. Header: `seq`, `t_color`, `t_depth` (Pi monotonic s, "
    "capture times), `dt_color_depth`, `mode` (full | base | rotation | none | no_depth), `motion` "
    "(4x4, depth frame at t_depth -> depth frame at t_color, applied), `rotation_deg`, `translation_mm`, "
    "`compute_ms`, `cpu_ms`, `latency_ms` (published - t_color), `dropped` (frames skipped for this client).")


class SettingsIn(BaseModel):
    scale: Optional[float] = Field(None, ge=0.1, le=1.0, description="Output grid = colour resolution x scale.")
    stride: Optional[int] = Field(None, ge=1, le=8, description="Use every stride-th depth pixel.")
    splat: Optional[Literal[1, 2]] = Field(None, description="2 = write 2x2 blocks (fills gaps at high `scale`).")
    compensation: Optional[Literal["auto", "rotation", "none"]] = Field(
        None, description="auto: best available (full / base / rotation / none); rotation: IMU only; none: off.")
    wait_s: Optional[float] = Field(None, ge=0, le=0.5, description="Max wait for IMU/pose coverage of a colour frame.")
    max_depth_age_s: Optional[float] = Field(None, gt=0, le=5)
    idle_s: Optional[float] = Field(None, ge=0, description="Stop registering after this long without consumers (0 = never).")
    allow_nominal: Optional[bool] = Field(None, description="Use an uncalibrated (nominal) camera pose for translation.")
    model_config = ConfigDict(extra="forbid", json_schema_extra={"examples": [
        {"scale": 0.5, "stride": 1}, {"compensation": "none"}, {"scale": 1.0, "splat": 2}]})


# -- responses ---------------------------------------------------------------------------------

# A number passed through as given (an int stays an int on the wire, e.g. Params(idle_s=0)).
Num = Annotated[Union[int, float], WithJsonSchema({"type": "number"})]


class ParamsOut(BaseModel):
    """The registration settings in force (``Params``)."""
    scale: Num = Field(description="Output grid = colour resolution x scale.")
    stride: int = Field(description="Every stride-th depth pixel is used.")
    splat: int = Field(description="1, or 2 = 2x2 blocks.")
    compensation: str = Field(description="auto | rotation | none.")
    wait_s: Num = Field(description="s: max wait for IMU / pose coverage of a colour frame.")
    max_depth_age_s: Num = Field(description="s: older depth is not used (the frame is published without depth).")
    idle_s: Num = Field(description="s without consumers before registering stops (0 = never).")
    allow_nominal: bool = Field(description="An uncalibrated (nominal) camera pose is used for translation.")


class Distortion(BaseModel):
    """OpenCV rational model (Brown-Conrady when k4..k6 = 0)."""
    k1: float
    k2: float
    k3: float
    k4: float
    k5: float
    k6: float
    p1: float
    p2: float


class IntrinsicsOut(BaseModel):
    fx: float = Field(description="Focal length, px.")
    fy: float = Field(description="Focal length, px.")
    cx: float = Field(description="Principal point, px.")
    cy: float = Field(description="Principal point, px.")
    width: int = Field(description="px.")
    height: int = Field(description="px.")
    distortion: Distortion


class BaseCalibration(BaseModel):
    from_: dict[str, Any] = Field(alias="from", description="The description node's `from` of the camera pose "
                                                            "(nominal; ground / yaw calibration ids).")
    calibrated: bool = Field(description="A ground or yaw calibration is applied (not only the nominal pose).")


class CalibrationOut(BaseModel):
    """Only `ready` (false) until the first frame was registered."""
    ready: bool
    color: Optional[IntrinsicsOut] = Field(None, description="Colour camera, full resolution (/point takes these "
                                                             "pixels).")
    depth_grid: Optional[IntrinsicsOut] = Field(None, description="The registered depth image (colour x `scale`, "
                                                                  "same distortion).")
    scale: Optional[float] = Field(None, description="Output grid = colour resolution x scale.")
    stride: Optional[int] = Field(None, description="Every stride-th depth pixel is used.")
    splat: Optional[int] = Field(None, description="1, or 2 = 2x2 blocks.")
    depth: Optional[IntrinsicsOut] = Field(None, description="The depth camera itself.")
    depth_to_color: Optional[list[list[float]]] = Field(None, description="4x4, m: p_color = T p_depth (factory).")
    imu_to_depth: Optional[list[list[float]]] = Field(None, description="3x3 rotation: v_depth = R v_imu.")
    frames: Optional[str] = Field(None, description="The axes of the optical frames.")
    base_to_color: Optional[list[list[float]]] = Field(None, description="4x4, m: p_base = T p_color (the colour "
                                                                         "camera pose in the base); null without a "
                                                                         "description.")
    base_calibration: Optional[BaseCalibration] = Field(None, description="Null without a description.")


class WindowMm(BaseModel):
    min: int = Field(description="mm.")
    median: int = Field(description="mm.")
    max: int = Field(description="mm.")


class PointOut(BaseModel):
    u: float = Field(description="Colour pixel column queried (full resolution).")
    v: float = Field(description="Colour pixel row queried.")
    u_out: int = Field(description="Its column on the depth grid.")
    v_out: int = Field(description="Its row on the depth grid.")
    depth_mm: Optional[int] = Field(description="z in the colour camera frame, mm; null without depth here.")
    source: Optional[str] = Field(description="pixel | window_median | outside; null = no depth here.")
    n_valid: int = Field(description="Depth pixels with a value within `radius`.")
    point: Optional[list[float]] = Field(description="[x, y, z] m, colour camera frame at t_color (the depth along "
                                                     "the pixel's undistorted ray); null without depth.")
    ray: list[float] = Field(description="[x, y, 1]: the pixel's undistorted ray (colour optical frame).")
    window_mm: Optional[WindowMm] = Field(None, description="Valid depths within `radius`; absent if none (or outside).")
    seq: int = Field(description="Frame sequence number.")
    t_color: Optional[float] = Field(description="Colour capture time (server monotonic s).")
    mode: str = Field(description="Motion compensation of the frame: full | base | rotation | none | no_depth.")
    frame: str = Field(description="Always `color_optical`.")
    age_s: float = Field(description="s since the frame was published.")
    point_base: Optional[list[float]] = Field(description="[x, y, z] m, base frame at t_color; null without depth or "
                                                          "without a calibrated camera pose.")


class Pctl(BaseModel):
    median: Optional[float] = Field(description="Over the frames of the last 5 s; null if none.")
    p95: Optional[float] = Field(description="95th percentile, same window.")


class Timing(BaseModel):
    """Per published frame, over the last 5 s."""
    compute_ms: Pctl = Field(description="ms: wall time of the registration.")
    cpu_ms: Pctl = Field(description="ms: worker thread CPU time.")
    latency_ms: Pctl = Field(description="ms: published - colour capture.")
    wait_ms: Pctl = Field(description="ms: received -> registration start.")
    dt_color_depth_ms: Pctl = Field(description="ms: colour capture - depth capture.")
    valid: Optional[float] = Field(description="Median fraction (0..1) of output pixels with depth.")


class LatestFrame(BaseModel):
    """Header fields of the latest fused frame."""
    seq: int
    t_color: Optional[float] = Field(description="Colour capture time (server monotonic s).")
    t_depth: Optional[float] = Field(description="Depth capture time (s); null if published without depth.")
    dt_color_depth: Optional[float] = Field(description="s: t_color - t_depth.")
    mode: str = Field(description="full | base | rotation | none | no_depth.")
    rotation_deg: float = Field(description="Degrees of the motion compensation applied.")
    translation_mm: float = Field(description="mm of the motion compensation applied.")
    compute_ms: float = Field(description="ms: wall time of the registration.")
    latency_ms: Optional[float] = Field(description="ms: published - t_color.")


class TrackSource(BaseModel):
    track_end: Optional[float] = Field(description="Time of the newest sample (server monotonic s); null if none.")
    samples: int = Field(description="Samples held.")


class OkSource(BaseModel):
    ok: bool
    error: Optional[str]


class DescriptionSource(BaseModel):
    ok: bool = Field(description="A camera pose in base was read from the description node.")
    error: Optional[str]
    calibrated: Optional[bool] = Field(description="Ground or yaw calibrated; null without a description.")
    from_: Optional[dict[str, Any]] = Field(alias="from", description="The description's `from`; null without one.")


class Sources(BaseModel):
    imu: TrackSource = Field(description="The camera IMU attitude track (rotation compensation).")
    pose: TrackSource = Field(description="The estimator base pose track (translation compensation).")
    camera_calibration: OkSource = Field(description="The camera node's /calibration.")
    description: DescriptionSource


class InputStatus(BaseModel):
    """An upstream binary stream."""
    target: str = Field(description="Node instance name or URL.")
    path: str = Field(description="Stream path on it.")
    connected: bool
    error: Optional[str]
    paused: bool = Field(description="Disconnected on purpose (idle: no consumers).")
    rate_hz: Optional[float] = Field(description="Records per s (null if stalled > 1 s or too few).")
    last_age: Optional[float] = Field(description="s since the last record.")
    count: int = Field(description="Records received.")
    clock_offset: float = Field(description="s: upstream monotonic - local monotonic.")
    clock_uncertainty: Optional[float] = Field(description="s.")


class PerceptionState(BaseModel):
    state: str = Field(description="running | waiting | idle (no consumers for `idle_s`).")
    error: Optional[str] = Field(description="Worker error, or the inputs' errors while not running.")
    mode: Optional[str] = Field(description="Compensation of the latest frame while running; else null.")
    covered: Optional[bool] = Field(description="The camera is covered (procedural frames) while running; else null.")
    modes: dict[str, int] = Field(description="Frames per compensation mode over the last 5 s.")
    rate_hz: Optional[float] = Field(description="Fused frames per s over the last 5 s.")
    color_rate_hz: Optional[float] = Field(description="Colour frames received per s.")
    seq: int = Field(description="Fused frames published.")
    skipped: int = Field(description="Colour frames skipped.")
    clients: int = Field(description="Open /stream subscribers.")
    timing: Timing
    latest: Optional[LatestFrame] = Field(description="Null before the first frame.")
    params: ParamsOut
    sources: Sources
    inputs: dict[str, InputStatus] = Field(description="frames, imu (camera), pose (estimator), when configured.")
    events: list[Event] = Field(description="Node events, oldest first.")


EVENTS_DOC = "Unnamed events; each `data:` line is the GET /state JSON (PerceptionState)."


def create_app(node: PerceptionNode, start: bool = True) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app):
        node.attach(asyncio.get_running_loop())
        if start:
            node.start()
        yield
        node.close()

    app = FastAPI(
        title="3D perception node",
        description=(
            "**Depth registered into the colour image, motion-compensated.** For every colour frame the newest "
            "depth frame (no SDK frame sync: depth arrives ~1 period late) is reprojected into that colour "
            "image's pixel grid **at the colour frame's capture time**: p_c(t_c) = E · M · p_d(t_d) with E the "
            "factory depth->colour extrinsic and M the camera motion between t_d and t_c (rotation from the "
            "camera's IMU attitude, translation from the estimator's base pose and the description's camera "
            "pose; whatever is available, reported per frame as `mode`). Nearest depth wins per pixel.\n\n"
            "Frames: `GET /stream` (every fused frame, latest-wins), `GET /frame` (one), `GET /color.jpg`, "
            "`GET /depth.png`. Point query for click tools: `GET /point?u=&v=` (colour pixel -> depth and 3D "
            "point in the colour camera frame, and in the base frame when the camera is calibrated).\n\n"
            "All times are the Pi's monotonic clock (s). Frames: optical x right, y down, z out of the lens; m.\n\n"
            + FRAME_DOC),
        version=__version__, lifespan=lifespan, openapi_tags=TAGS)
    setup_app(app)
    add_base_routes(app, lambda: {"node": KIND, "state": node.state()["state"]})
    add_page_routes(app, node_meta(KIND, "3D perception", __version__, KIND, actuated=False,
                                   api={"stream": "stream", "frame": "frame", "point": "point",
                                        "calibration": "calibration"}))

    @app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def index():
        from .page import PAGE
        return PAGE

    @app.get("/state", response_model=PerceptionState, tags=["state"], summary="Status, mode, timing (compute / CPU / latency), inputs, events")
    async def get_state():
        return node.state()

    @app.get("/events", tags=["state"], summary="Server-Sent Events: /state on every change and every 1 s",
             response_class=StreamingResponse,
             responses={200: {"content": {"text/event-stream": {}}, "description": EVENTS_DOC}})
    async def events():
        return StreamingResponse(sse(node, node.state), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-store"})

    @app.get("/calibration", response_model=CalibrationOut, response_model_exclude_unset=True, tags=["state"],
             summary="Intrinsics of the output (colour camera and the depth grid)")
    async def calibration():
        """`color`: colour camera intrinsics (full resolution, with distortion; `/point` takes pixels of
        this). `depth_grid`: intrinsics of the registered depth image (colour scaled by `scale`; same
        distortion). `depth_to_color`, `imu_to_depth` (rotation), `base_to_color` (colour camera pose in
        the base, 4x4, null without a description): p_parent = T p_child. `ready` false until the first
        frame was registered."""
        node.demand()
        return node.calibration()

    @app.get("/settings", response_model=ParamsOut, tags=["control"], summary="Current registration settings")
    async def get_settings():
        return node.params.to_json()

    @app.post("/settings", response_model=ParamsOut, tags=["control"], summary="Change registration settings (partial)")
    async def post_settings(body: SettingsIn):
        """Applies to the next frame (the registrar is rebuilt when the grid changes)."""
        return node.set_params(**body.model_dump(exclude_none=True)).to_json()

    async def fresh(max_age: float, timeout: float) -> Fused:
        """The latest fused frame no older than max_age (waits for one up to timeout)."""
        node.demand()
        deadline = time.monotonic() + timeout
        while True:
            fut = node.slot.changed()
            f = node.slot.get()
            if f is not None and time.monotonic() - f.t_published <= max_age:
                return f
            left = deadline - time.monotonic()
            if left <= 0:
                if f is not None:
                    return f
                st = node.state()
                raise HTTPException(503, f"no fused frame yet ({st['state']}: {st['error'] or 'waiting for inputs'})")
            await asyncio.wait({fut}, timeout=left)

    stream_headers = {"Cache-Control": "no-store", "X-Accel-Buffering": "no"}

    def record(f: Fused, color: bool, depth: bool, dropped: int) -> list:
        h = dict(f.header)
        hc, hd = dict(h["color"]), dict(h["depth"])
        parts, off = [], 0
        for on, sub, data in ((color, hc, lambda: f.color), (depth, hd, lambda: memoryview(f.depth.astype("<u2", copy=False)).cast("B"))):
            if on:
                b = data()
                sub["offset"], sub["len"] = off, memoryview(b).nbytes
                off += sub["len"]
                parts.append(b)
            else:
                sub["offset"], sub["len"] = None, 0
        h.update(color=hc, depth=hd, dropped=dropped, t_send=time.monotonic(), len=off)
        hb = json.dumps(h, separators=(",", ":"), default=float).encode()
        return [struct.pack(">I", len(hb)) + hb, *parts]

    @app.get("/stream", tags=["frames"], response_class=StreamingResponse,
             summary="Every fused frame (latest-wins) as binary records",
             responses={200: {"content": {"application/octet-stream": {}}, "description": FRAME_DOC}})
    async def stream(color: bool = Query(True, description="Include the colour JPEG."),
                     depth: bool = Query(True, description="Include the registered depth (uint16 mm).")):
        async def gen():
            node.clients += 1
            node.demand()
            last = 0
            try:
                while True:
                    fut = node.slot.changed()
                    f = node.slot.get()
                    if f is not None and f.header["seq"] > last:
                        dropped = f.header["seq"] - last - 1 if last else 0
                        last = f.header["seq"]
                        for part in record(f, color, depth, dropped):
                            yield part
                        continue
                    done, _ = await asyncio.wait({fut}, timeout=PING_S)
                    if not done:
                        yield pack_record({"type": "ping", "t_send": time.monotonic()})
            finally:
                node.clients -= 1
                node.demand()
        return StreamingResponse(gen(), media_type="application/octet-stream", headers=stream_headers)

    @app.get("/frame", tags=["frames"], response_class=Response, summary="The latest fused frame as one binary record",
             responses={200: {"content": {"application/octet-stream": {}}, "description": FRAME_DOC}})
    async def frame(color: bool = True, depth: bool = True,
                    max_age: float = Query(0.5, gt=0, description="Wait for a frame newer than this (s)."),
                    timeout: float = Query(3.0, gt=0, le=30)):
        f = await fresh(max_age, timeout)
        return Response(b"".join(bytes(p) for p in record(f, color, depth, 0)), media_type="application/octet-stream",
                        headers={"Cache-Control": "no-store"})

    @app.get("/color.jpg", tags=["frames"], response_class=Response, summary="The colour image of the latest fused frame",
             responses={200: {"content": {"image/jpeg": {}}}})
    async def color_jpg(max_age: float = Query(0.5, gt=0), timeout: float = Query(3.0, gt=0, le=30)):
        f = await fresh(max_age, timeout)
        if f.header["color"]["format"] not in ("MJPG", "JPEG"):
            raise HTTPException(409, f"colour format is {f.header['color']['format']}, not JPEG")
        return Response(f.color, media_type="image/jpeg", headers={"Cache-Control": "no-store",
                                                                   "X-Frame-Seq": str(f.header["seq"])})

    @app.get("/depth.png", tags=["frames"], response_class=Response, summary="The registered depth of the latest frame (16-bit PNG, mm)",
             responses={200: {"content": {"image/png": {}}}})
    async def depth_png(max_age: float = Query(0.5, gt=0), timeout: float = Query(3.0, gt=0, le=30)):
        """16-bit grey PNG on the `depth_grid` (see `/calibration`); value = z in mm in the colour camera
        frame, 0 = none."""
        f = await fresh(max_age, timeout)
        return Response(await asyncio.to_thread(png16, f.depth), media_type="image/png",
                        headers={"Cache-Control": "no-store", "X-Frame-Seq": str(f.header["seq"])})

    @app.get("/point", response_model=PointOut, response_model_exclude_unset=True, tags=["query"], summary="Depth and 3D point at a colour pixel (latest frame)")
    async def point(u: float = Query(..., description="Colour pixel column (full colour resolution; or a "
                                                       "fraction of the width with norm=true)."),
                    v: float = Query(..., description="Colour pixel row."),
                    norm: bool = Query(False, description="u, v are fractions (0..1) of the width / height."),
                    radius: int = Query(2, ge=0, le=20, description="Search radius (depth-grid pixels) when the "
                                                                    "pixel itself has no depth: median of the valid ones."),
                    max_age: float = Query(0.5, gt=0), timeout: float = Query(3.0, gt=0, le=30)):
        """Returns `depth_mm` (z in the colour camera frame), `point` [x, y, z] m in the colour camera frame
        at `t_color` (the depth along the pixel's undistorted ray), `point_base` (the same point in the base
        frame at t_color, when the description has a calibrated camera pose; else null), `source`
        (pixel | window_median | outside | null = no depth here), `window_mm` (min / median / max around
        it), and the frame's `seq`, `t_color`, `mode`, `age_s`."""
        f = await fresh(max_age, timeout)
        if norm:
            u, v = u * f.color_k.width - 0.5, v * f.color_k.height - 0.5
        return node.point(f, u, v, radius)

    return app


def png16(img) -> bytes:
    try:
        import cv2
        ok, buf = cv2.imencode(".png", img)
        if ok:
            return buf.tobytes()
    except ImportError:
        pass
    import io

    from PIL import Image
    b = io.BytesIO()
    Image.fromarray(img.astype("<u2"), "I;16").save(b, "PNG")
    return b.getvalue()
