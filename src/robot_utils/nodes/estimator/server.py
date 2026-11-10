"""HTTP API of the estimator node (FastAPI). Conventions: docs/nodes.md; guide: docs/estimator_node.md."""
from __future__ import annotations

import asyncio
import math
from contextlib import asynccontextmanager
from typing import Optional

from fastapi import FastAPI, Query
from fastapi.responses import HTMLResponse, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field, create_model

from ..common.server import Event, add_base_routes, add_page_routes, node_meta, rows_stream, setup_app, sse
from ..swerve.constants import MODULES
from . import __version__
from .fusion import FIELD_DOC, FIELDS, UNITS
from .node import IMU_WAIT_S, EstimatorNode

TAGS = [
    {"name": "state", "description": "Status, inputs, current pose, clock sync, events."},
    {"name": "control", "description": "Set the pose."},
    {"name": "stream", "description": "Every fused pose (100 Hz, at the swerve ticks' times) as binary float64 rows."},
]


class PoseIn(BaseModel):
    x: float = Field(0.0, description="m")
    y: float = Field(0.0, description="m")
    theta: float = Field(0.0, description="rad, CCW +")
    model_config = ConfigDict(json_schema_extra={"examples": [{}, {"x": 1.0, "y": -0.5, "theta": 1.5708}]})


# -- responses ---------------------------------------------------------------------------------

def _column(name: str, unit: str):
    return Optional[float], Field(description=f"{unit}; {FIELD_DOC[name]} (null = no value).")


# One field per /stream column (FIELDS), flat like the rows.
EstimatorPose = create_model("EstimatorPose", __doc__="The latest fused pose row, as in /stream (NaN sent as null).",
                             **{n: _column(n, u) for n, u in zip(FIELDS, UNITS)})


class UpstreamStatus(BaseModel):
    """One input stream (estimator.node ``Upstream.status``)."""
    url: str = Field(description="Upstream base (instance name or URL) + stream path, e.g. `swerve/stream`.")
    connected: bool = Field(description="The stream is open (it reconnects every 1 s after an error).")
    error: Optional[str] = Field(description="Why the last connection failed or ended; null while connected.")
    rate_hz: Optional[float] = Field(description="Rows per second received over the last ~200 rows (0.1 Hz "
                                                 "resolution); null if too few or none for 1 s.")
    last_age: Optional[float] = Field(description="s since the last rows arrived (1 ms resolution); null if none yet.")
    samples: int = Field(description="Rows received since the node started.")
    clock_offset: float = Field(description="s, upstream clock minus this host's monotonic clock (NTP-style, "
                                            "re-synced every 30 s; ~0 on the same machine; 1 µs resolution).")
    clock_uncertainty: Optional[float] = Field(description="s, ± bound of clock_offset (half-width of the shortest "
                                                           "round trips' intersected bounds, at most half the best "
                                                           "round trip; 1 µs resolution); null before the first sync.")


class EstimatorState(BaseModel):
    state: str = Field(description="running (poses published within the last 1 s) | waiting.")
    error: Optional[str] = Field(description="The inputs' errors (`<input>: <error>`, joined by `; `) while "
                                             "waiting; null when running or no input has an error.")
    mode: str = Field(description="waiting for inputs | fused (wheel odometry + IMU heading) | odometry only (no "
                                  "IMU) | heading only (IMU; no swerve data).")
    pose: Optional[EstimatorPose] = Field(description="The latest pose; null before the first.")
    rate_hz: Optional[float] = Field(description="Poses published per second over the last ~200 (0.1 Hz "
                                                 "resolution); null while not running or too few.")
    imu_coverage: Optional[float] = Field(description="Share (0..1) of the last 500 odometry ticks (5 s) whose "
                                                      "heading came from the IMU (imu_ok); null before the first tick.")
    imu_still: bool = Field(description="The camera IMU filter's latest record flags the camera as still.")
    inputs: dict[str, UpstreamStatus] = Field(description="By input: `swerve` (wheel odometry) and `imu` (camera "
                                                          "IMU), each only if configured.")
    events: list[Event] = Field(description="Latest node events (up to 50), oldest first.")


class FieldsOut(BaseModel):
    fields: list[str] = Field(description="Column names of the /stream rows, in order.")
    units: list[str] = Field(description="Unit of each column (s, m, rad, m/s, rad/s; bool = 0/1; enum, count).")
    doc: dict[str, str] = Field(description="Meaning of each column, by name.")


class PoseOut(BaseModel):
    """The pose set (as requested)."""
    x: float = Field(description="m, world frame.")
    y: float = Field(description="m, world frame.")
    theta: float = Field(description="rad, CCW +.")
    theta_deg: float = Field(description="theta in degrees.")


STREAM_DOC = (
    "Records `[u32 big-endian header length N][N bytes UTF-8 JSON header][header.len bytes payload]` "
    "(docs/nodes.md; parse with `common.protocol` or `RU.RecordReader`). Every header has `type` and `len`. "
    "First a `schema` record (no payload): `schema` (id 1), `fields` and `units` (as GET /fields), "
    "`dtype` `\"<f8\"`, `kind` `\"pose\"`, `node` `\"estimator\"`, `rate_hz` 100, `t_send`. Then `rows` records: "
    "payload a little-endian float64 array of shape (n, len(fields)); header `n`, `schema`, `dropped` (rows lost "
    "because this client fell behind), `kind`, `t_send`. `ping` records (header `t_send` only, no payload) after "
    "~1 s without data. `t_send`: server monotonic s.")


def create_app(node: EstimatorNode, start: bool = True) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app):
        node.hub.attach(asyncio.get_running_loop())
        if start:
            node.start()
        yield
        node.close()

    app = FastAPI(
        title="Estimator node",
        description=(
            "Planar pose of the base (x, y, heading) from **swerve wheel odometry** (translation) and the "
            "**camera IMU** attitude filter (heading), at 100 Hz.\n\n"
            "The swerve odometry is the master: every pose is computed at a swerve tick's sample time; "
            "the IMU heading is interpolated to it (a tick waits up to "
            f"{IMU_WAIT_S * 1000:.0f} ms for the IMU, else the wheels' heading is used and `imu_ok` = 0). "
            "World frame = the pose at the last reset; x forward, y left, theta CCW.\n\n"
            "`GET /stream` (binary records, docs/nodes.md) carries every pose; `GET /state` the latest. "
            "No trajectory is kept here: clients (like the page) accumulate the stream."),
        version=__version__, lifespan=lifespan, openapi_tags=TAGS)
    setup_app(app)
    add_base_routes(app, lambda: {"node": "estimator", "state": node.state()["state"]})
    half_len = max(abs(m.x) for m in MODULES)
    half_wid = max(abs(m.y) for m in MODULES)
    add_page_routes(app, node_meta("estimator", "State estimator", __version__, "estimator", actuated=False,
                                   rate_hz=100,
                                   footprint={"half_length": half_len, "half_width": half_wid}))

    @app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def index():
        from .page import PAGE
        return PAGE

    @app.get("/state", response_model=EstimatorState, tags=["state"], summary="Status, inputs, mode and the latest pose")
    async def get_state():
        return node.state()

    @app.get("/events", tags=["state"], summary="Server-Sent Events: /state on every change and every 1 s",
             response_class=StreamingResponse,
             responses={200: {"content": {"text/event-stream": {}},
                              "description": "Unnamed events; each `data:` line is the GET /state JSON (EstimatorState)."}})
    async def events():
        return StreamingResponse(sse(node.hub, node.state), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-store"})

    @app.get("/pose", response_model=Optional[EstimatorPose], tags=["state"], summary="The latest pose only (one row as JSON)")
    async def pose():
        st = node.state()
        return st["pose"]

    @app.get("/fields", response_model=FieldsOut, tags=["stream"], summary="Columns of the pose rows")
    async def fields():
        return {"fields": FIELDS, "units": UNITS, "doc": FIELD_DOC}

    @app.get("/stream", tags=["stream"], response_class=StreamingResponse,
             summary="Every fused pose as binary float64 rows (schema record first)",
             responses={200: {"content": {"application/octet-stream": {}}, "description": STREAM_DOC}})
    async def stream(batch_ms: float = Query(0.0, ge=0, le=1000, description="Collect rows this long per record (0 = every pose)."),
                     max_hz: float = Query(0.0, ge=0, le=100, description="At most this many rows per second, the latest each time (0 = all; e.g. 10 for an overview)."),
                     max_backlog_s: float = Query(20.0, gt=0, le=600, description="Drop the oldest rows beyond this backlog.")):
        gen = rows_stream(node.hub, FIELDS, UNITS, channel="pose", kind="pose", batch_ms=batch_ms,
                          max_rows=int(max_backlog_s * 100), schema_extra={"node": "estimator", "rate_hz": 100}, max_hz=max_hz)
        return StreamingResponse(gen, media_type="application/octet-stream", headers={"Cache-Control": "no-store"})

    @app.post("/reset", response_model=PoseOut, tags=["control"], summary="Set the pose (default: the origin)")
    async def reset(body: PoseIn = PoseIn()):
        """Both the fused and the odometry-only pose jump to this pose; the distance restarts at 0.
        Applied before the next sample. Anyone may call it (the estimator moves nothing)."""
        node.reset(body.x, body.y, body.theta)
        return {"x": body.x, "y": body.y, "theta": body.theta, "theta_deg": math.degrees(body.theta)}

    return app
