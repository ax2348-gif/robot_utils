"""HTTP API of the estimator node (FastAPI). Conventions: docs/nodes.md; guide: docs/estimator_node.md."""
from __future__ import annotations

import asyncio
import math
from contextlib import asynccontextmanager

from fastapi import FastAPI, Query
from fastapi.responses import HTMLResponse, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field

from ..common.server import add_base_routes, add_page_routes, node_meta, rows_stream, setup_app, sse
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

    @app.get("/state", tags=["state"], summary="Status, inputs, mode and the latest pose")
    async def get_state():
        return node.state()

    @app.get("/events", tags=["state"], summary="Server-Sent Events: /state on every change and every 1 s",
             response_class=StreamingResponse)
    async def events():
        return StreamingResponse(sse(node.hub, node.state), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-store"})

    @app.get("/pose", tags=["state"], summary="The latest pose only (one row as JSON)")
    async def pose():
        st = node.state()
        return st["pose"]

    @app.get("/fields", tags=["stream"], summary="Columns of the pose rows")
    async def fields():
        return {"fields": FIELDS, "units": UNITS, "doc": FIELD_DOC}

    @app.get("/stream", tags=["stream"], response_class=StreamingResponse,
             summary="Every fused pose as binary float64 rows (schema record first)")
    async def stream(batch_ms: float = Query(0.0, ge=0, le=1000, description="Collect rows this long per record (0 = every pose)."),
                     max_hz: float = Query(0.0, ge=0, le=100, description="At most this many rows per second, the latest each time (0 = all; e.g. 10 for an overview)."),
                     max_backlog_s: float = Query(20.0, gt=0, le=600, description="Drop the oldest rows beyond this backlog.")):
        gen = rows_stream(node.hub, FIELDS, UNITS, channel="pose", kind="pose", batch_ms=batch_ms,
                          max_rows=int(max_backlog_s * 100), schema_extra={"node": "estimator", "rate_hz": 100}, max_hz=max_hz)
        return StreamingResponse(gen, media_type="application/octet-stream", headers={"Cache-Control": "no-store"})

    @app.post("/reset", tags=["control"], summary="Set the pose (default: the origin)")
    async def reset(body: PoseIn = PoseIn()):
        """Both the fused and the odometry-only pose jump to this pose; the distance restarts at 0.
        Applied before the next sample. Anyone may call it (the estimator moves nothing)."""
        node.reset(body.x, body.y, body.theta)
        return {"x": body.x, "y": body.y, "theta": body.theta, "theta_deg": math.degrees(body.theta)}

    return app
