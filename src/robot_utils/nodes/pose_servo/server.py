"""HTTP API of the pose servo node (FastAPI). Conventions: docs/nodes.md; guide: docs/pose_servo_node.md."""
from __future__ import annotations

import asyncio
import math
from contextlib import asynccontextmanager
from typing import Literal, Optional

from fastapi import FastAPI, Query, Request
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field

from ..common.server import add_base_routes, add_page_routes, node_meta, rows_stream, setup_app, sse
from . import __version__
from .node import FIELD_DOC, FIELDS, PERIOD, UNITS, PoseServoNode

TAGS = [
    {"name": "state", "description": "Status, goal, config, inputs, events."},
    {"name": "control", "description": "Enable / disable (= the swerve lease), config, goals, stop, e-stop."},
    {"name": "stream", "description": f"Every control tick ({1 / PERIOD:.0f} Hz) as binary float64 rows."},
]


class ConfigIn(BaseModel):
    """Any subset; the rest keeps its current value (defaults at enable)."""
    max_speed: Optional[float] = Field(None, description="m/s, |(vx, vy)| of every twist (default 0.3; <= 2.0)")
    max_omega: Optional[float] = Field(None, description="rad/s (default 0.8; <= 4.0)")
    max_accel: Optional[float] = Field(None, description="m/s², |twist rate| linear (default 0.5)")
    max_alpha: Optional[float] = Field(None, description="rad/s² (default 1.5)")
    max_jerk: Optional[float] = Field(None, description="m/s³ (default 2.0)")
    max_ang_jerk: Optional[float] = Field(None, description="rad/s³ (default 6.0)")
    max_distance: Optional[float] = Field(None, description="m: goals farther from the current pose are rejected (default 3.0)")
    max_rotation: Optional[float] = Field(None, description="rad: goals turning more are rejected (default π)")
    require_imu: Optional[bool] = Field(None, description="abort when the estimator's heading is not IMU-fused (default true)")
    mode: Optional[Literal["synchronized", "independent"]] = Field(None, description="default synchronized")
    pos_tol: Optional[float] = Field(None, description="m, acceptance region (default 0.02)")
    heading_tol: Optional[float] = Field(None, description="rad, acceptance region (default 0.035)")
    on_reach: Optional[Literal["terminate", "hold"]] = Field(None, description="default terminate")
    hold_band: Optional[float] = Field(None, description="an axis done inside its tolerance moves again beyond hold_band × tolerance (default 2)")
    min_wheel_speed: Optional[float] = Field(None, description="m/s: while the base does not move, the command is raised up to this wheel speed (static friction; default 0.08, 0 = off)")
    stall_s: Optional[float] = Field(None, description="s without progress before a goal ends as 'stalled' (default 3, 0 = never)")
    model_config = ConfigDict(extra="forbid", json_schema_extra={"examples": [
        {}, {"max_speed": 0.15, "max_accel": 0.2, "max_distance": 1.0}]})

    def changes(self) -> dict:
        return {k: v for k, v in self.model_dump().items() if v is not None}


class EnableIn(BaseModel):
    client: str = Field("", description="Who is controlling (shown in /state and as the swerve lease holder).")
    config: ConfigIn = Field(default_factory=ConfigIn)
    model_config = ConfigDict(json_schema_extra={"examples": [
        {"client": "alice"}, {"client": "alice", "config": {"max_speed": 0.15, "max_accel": 0.25}}]})


class GoalIn(BaseModel):
    frame: Literal["local", "global"] = Field("local", description="local: relative to the current pose (x forward, "
                                              "y left, theta = turn, taken literally: 2π = a full turn); global: the "
                                              "estimator's world frame (theta: shortest turn)")
    x: float = Field(0.0, description="m")
    y: float = Field(0.0, description="m")
    theta: float = Field(0.0, description="rad, CCW +")
    mode: Optional[Literal["synchronized", "independent"]] = None
    pos_tol: Optional[float] = None
    heading_tol: Optional[float] = None
    on_reach: Optional[Literal["terminate", "hold"]] = None
    hold_band: Optional[float] = None
    min_wheel_speed: Optional[float] = None
    stall_s: Optional[float] = None
    speed: Optional[float] = Field(None, description="m/s, <= max_speed")
    omega: Optional[float] = Field(None, description="rad/s, <= max_omega")
    accel: Optional[float] = Field(None, description="m/s², <= max_accel")
    alpha: Optional[float] = Field(None, description="rad/s², <= max_alpha")
    jerk: Optional[float] = Field(None, description="m/s³, <= max_jerk")
    ang_jerk: Optional[float] = Field(None, description="rad/s³, <= max_ang_jerk")
    model_config = ConfigDict(extra="forbid", json_schema_extra={"examples": [
        {"x": 0.5}, {"x": 0.3, "y": 0.2, "theta": 1.5708, "speed": 0.15},
        {"frame": "global", "x": 0, "y": 0, "theta": 0, "mode": "independent", "on_reach": "hold"}]})


class EStopIn(BaseModel):
    reason: str = ""
    source: str = Field("", description="Who pressed it (free text).")


def create_app(node: PoseServoNode, start: bool = True) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app):
        node.hub.attach(asyncio.get_running_loop())
        if start:
            node.start()
        yield
        await asyncio.to_thread(node.close)

    app = FastAPI(
        title="Pose servo node",
        description=(
            "Drives the swerve base to a pose (x, y, heading) with smooth, jerk-limited motion, using the "
            "estimator node's pose.\n\n"
            "`POST /enable` takes the swerve lease and enables the swerve; `POST /disable` stops at once and "
            "releases it. `POST /goal` (local by default) starts or replaces a motion, continuing from the "
            "current velocity. Every twist sent stays within the caps of the config and the hard limits "
            "(each wheel ≤ 3 m/s, steering ≤ 4π rad/s). The motion is aborted (braked within the caps) when "
            "the estimator stream is stale, resets, jumps or loses its IMU heading.\n\n"
            "`GET /stream` carries every control tick; `GET /state` the latest."),
        version=__version__, lifespan=lifespan, openapi_tags=TAGS)
    setup_app(app)

    @app.exception_handler(ValueError)
    async def _value_error(request: Request, exc: ValueError):   # rejected goal / config: 422
        return JSONResponse(status_code=422, content={"detail": str(exc)})

    add_base_routes(app, lambda: {"node": "pose_servo", "state": node.state()["status"]})
    add_page_routes(app, node_meta("pose_servo", "Pose servo", __version__, "pose_servo", actuated=True,
                                   rate_hz=round(1 / PERIOD)))

    @app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def index():
        from .page import PAGE
        return PAGE

    @app.get("/state", tags=["state"], summary="Status, goal and how it ended, config, inputs, latest tick")
    async def get_state():
        return node.state()

    @app.get("/events", tags=["state"], summary="Server-Sent Events: /state on every change and every 1 s",
             response_class=StreamingResponse)
    async def events():
        return StreamingResponse(sse(node.hub, node.state), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-store"})

    @app.get("/fields", tags=["stream"], summary="Columns of the tick rows")
    async def fields():
        return {"fields": FIELDS, "units": UNITS, "doc": FIELD_DOC}

    @app.get("/stream", tags=["stream"], response_class=StreamingResponse,
             summary="Every control tick as binary float64 rows (schema record first)")
    async def stream(batch_ms: float = Query(0.0, ge=0, le=1000, description="Collect rows this long per record (0 = every tick)."),
                     max_backlog_s: float = Query(20.0, gt=0, le=600, description="Drop the oldest rows beyond this backlog.")):
        gen = rows_stream(node.hub, FIELDS, UNITS, channel="servo", kind="servo", batch_ms=batch_ms,
                          max_rows=int(max_backlog_s / PERIOD), schema_extra={"node": "pose_servo", "rate_hz": 1 / PERIOD})
        return StreamingResponse(gen, media_type="application/octet-stream", headers={"Cache-Control": "no-store"})

    @app.post("/enable", tags=["control"], summary="Take the swerve lease, set its limits to the caps, enable it")
    async def enable(body: EnableIn = EnableIn()):
        """409 if already enabled or the swerve lease is held by someone else (or its e-stop is latched).
        The config (defaults + `config`) lasts until disable."""
        return await asyncio.to_thread(node.enable, body.client, body.config.changes())

    @app.post("/disable", tags=["control"], summary="Stop at once (no braking), disable the swerve, release its lease")
    async def disable():
        return node.disable("requested")

    @app.get("/config", tags=["control"], summary="The config in force and the hard limits")
    async def get_config():
        return node.get_config()

    @app.post("/config", tags=["control"], summary="Change the config (enabled and idle only)")
    async def set_config(body: ConfigIn):
        return await asyncio.to_thread(node.set_config, body.changes())

    @app.post("/goal", tags=["control"], summary="Go to a pose (local by default); replaces any motion in progress")
    async def goal(body: GoalIn):
        """422 if the goal is beyond `max_distance` / `max_rotation` or an option exceeds its cap;
        409 if not enabled; 503 if the estimator pose is not usable. Returns the goal resolved to the
        world frame. Its progress and result (reached / stopped / aborted + reason) are in `/state.goal`."""
        opts = body.model_dump(exclude={"frame", "x", "y", "theta"})
        return node.set_goal(body.frame, body.x, body.y, body.theta, opts)

    @app.post("/stop", tags=["control"], summary="Brake to a stop within the caps; stay enabled")
    async def stop():
        return node.stop()

    @app.post("/estop", tags=["control"], summary="E-stop: forwarded to the swerve (latches, stops at once); disables")
    async def estop(request: Request, body: Optional[EStopIn] = None):
        body = body or EStopIn()
        src = body.source or (request.client.host if request.client else "")
        return await asyncio.to_thread(node.estop, body.reason, src)

    return app
