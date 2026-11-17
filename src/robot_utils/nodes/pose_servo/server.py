"""HTTP API of the pose servo node (FastAPI). Conventions: docs/nodes.md; guide: docs/pose_servo_node.md."""
from __future__ import annotations

import asyncio
import math
from contextlib import asynccontextmanager
from typing import Literal, Optional

from fastapi import FastAPI, Query, Request
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field, create_model

from ..common.server import (Event, LockOut, add_base_routes, add_page_routes, node_meta, rows_stream, setup_app,
                             sse)
from ..common.realtime import RealtimeOut
from ..estimator.server import UpstreamStatus
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


# -- responses ---------------------------------------------------------------------------------

class ServoConfig(BaseModel):
    """The config in force (node.Config): the defaults while disabled."""
    max_speed: float = Field(description="m/s, cap on |(vx, vy)| of every twist.")
    max_omega: float = Field(description="rad/s, cap on the yaw rate.")
    max_accel: float = Field(description="m/s², cap on the linear twist rate.")
    max_alpha: float = Field(description="rad/s², cap on the angular twist rate.")
    max_jerk: float = Field(description="m/s³, linear jerk cap.")
    max_ang_jerk: float = Field(description="rad/s³, angular jerk cap.")
    max_distance: float = Field(description="m: goals farther from the current pose are rejected.")
    max_rotation: float = Field(description="rad: goals turning more are rejected.")
    require_imu: bool = Field(description="Abort when the estimator's heading is not IMU-fused.")
    mode: str = Field(description="Default motion mode: synchronized | independent.")
    pos_tol: float = Field(description="m, default position acceptance region.")
    heading_tol: float = Field(description="rad, default heading acceptance region.")
    on_reach: str = Field(description="Default on reaching the goal: terminate | hold.")
    hold_band: float = Field(description="An axis done inside its tolerance moves again beyond hold_band × tolerance.")
    min_wheel_speed: float = Field(description="m/s, static-friction boost limit (0 = off).")
    stall_s: float = Field(description="s without progress before a goal ends as `stalled` (0 = never).")


class HardLimits(BaseModel):
    """Fixed limits of the servo and the swerve node."""
    steer_rate: float = Field(description="rad/s, each wheel's steering rate (direction of its velocity).")
    wheel_speed: float = Field(description="m/s, each wheel.")
    steer_min_speed: float = Field(description="m/s: slower wheels have no defined direction (not rate-limited).")
    wheel_radius_from_centre: float = Field(description="m, base centre to the farthest wheel.")
    swerve_max_speed: float = Field(description="m/s, the swerve node's hard translation limit.")
    swerve_max_omega: float = Field(description="rad/s, the swerve node's hard yaw-rate limit.")
    period: float = Field(description="s, control period.")


class ConfigOut(BaseModel):
    enabled: bool = Field(description="The servo holds the swerve lease and drives.")
    config: ServoConfig
    hard: HardLimits


class Disabled(BaseModel):
    enabled: bool = Field(description="Always false.")
    was_enabled: bool = Field(description="It was enabled before this request.")


class StopOut(BaseModel):
    status: str = Field(description="Servo status after the stop: braking (still moving) | idle.")


class AlignIn(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, json_schema_extra={"examples": [
        {"twist": [1.0, 0.0, 0.0]}, {"twist": [0.6, -0.8, 0.0]}]})
    twist: list[float] = Field(min_length=3, max_length=3,
                               description="Unit twist [vx, vy, omega] (only its direction matters): every wheel is set the "
                                           "way it would roll for it, the drive motors are off (coasting), so the base "
                                           "rolls freely along that twist if pushed (e.g. by the arm meeting something).")


class AlignState(BaseModel):
    """Defensive alignment: wheels set for a unit twist with the drives coasting, held while idle."""
    twist: Optional[list[float]] = Field(description="The unit twist in force, normalised; null when off.")
    aligned: bool = Field(description="Every wheel is within 3 degrees of its setting.")
    steer_err_deg: Optional[float] = Field(description="The worst wheel's steering error, degrees; null when off or "
                                                       "not yet read.")
    since_s: Optional[float] = Field(description="s since this alignment was set; null when off.")


class EStopOut(BaseModel):
    """The node disables itself whether or not the forward worked."""
    forwarded: bool = Field(description="The swerve node accepted the e-stop (it latches there).")
    error: Optional[str] = Field(None, description="Why the forward failed; absent if it worked or no swerve "
                                                   "node is configured.")


class XYTheta(BaseModel):
    x: float = Field(description="m.")
    y: float = Field(description="m.")
    theta: float = Field(description="rad, CCW +.")


class MotionOut(BaseModel):
    """How the goal is approached (planner.Motion): the config defaults with the goal's overrides."""
    speed: float = Field(description="m/s, max |(vx, vy)|.")
    omega: float = Field(description="rad/s, max yaw rate.")
    accel: float = Field(description="m/s².")
    alpha: float = Field(description="rad/s².")
    jerk: float = Field(description="m/s³.")
    ang_jerk: float = Field(description="rad/s³.")
    mode: str = Field(description="synchronized | independent.")
    pos_tol: float = Field(description="m, position acceptance region.")
    heading_tol: float = Field(description="rad, heading acceptance region.")
    on_reach: str = Field(description="terminate | hold.")
    hold_band: float = Field(description="An axis done inside its tolerance moves again beyond hold_band × tolerance.")
    min_wheel_speed: float = Field(description="m/s, static-friction boost limit (0 = off).")
    stall_s: float = Field(description="s without progress before the goal ends as `stalled` (0 = never).")


class GoalOut(BaseModel):
    """A goal resolved to the estimator's world frame, and how it ended."""
    goal_id: int = Field(description="Increments with every accepted goal.")
    frame: str = Field(description="local | global, as requested.")
    request: XYTheta = Field(description="x, y, theta as requested (in `frame`).")
    x: float = Field(description="m, goal in the world frame.")
    y: float = Field(description="m, goal in the world frame.")
    theta: float = Field(description="rad, goal heading in the world frame, continuous (turns included).")
    motion: MotionOut
    from_: XYTheta = Field(alias="from", description="The predicted pose (world frame) when the goal was set.")
    distance: float = Field(description="m, from `from` to the goal.")
    rotation: float = Field(description="rad, theta - from.theta (signed).")
    result: Optional[str] = Field(description="reached | stopped | aborted | stalled; null while in progress.")
    reason: Optional[str] = Field(description="Why it ended (empty when reached); null while in progress.")
    t: float = Field(description="Server time.monotonic() (s) when the goal was set.")
    model_config = ConfigDict(populate_by_name=True)


class EstimatorHealth(BaseModel):
    health: str = Field(description="ok, or why the pose cannot be used (no pose yet, stale stream, no wheel "
                                    "odometry, heading not IMU-fused with require_imu).")
    reset_id: Optional[float] = Field(description="The estimator's reset_id (count) of the latest pose; null before "
                                                  "the first.")
    pose_jumps: int = Field(description="Pose jumps seen (> 0.15 m or 0.3 rad between consecutive poses).")
    age: Optional[float] = Field(description="s since the latest pose's sample time (0.1 ms resolution); null "
                                             "before the first.")


class SwerveView(BaseModel):
    """From the latest swerve telemetry row (0/1 flags as floats)."""
    t: float = Field(description="s, its t_tick on this host's monotonic clock.")
    enabled: float = Field(description="1 if the swerve is enabled.")
    estop: float = Field(description="1 if its e-stop is latched.")
    lease_id: float = Field(description="Its lease in force (0 = none).")
    steer_err: Optional[float] = Field(None, description="rad: the worst wheel's steering error while a steer-only "
                                                         "(alignment) command is in force; null otherwise.")


_TICK_DOC = {
    "t": "control tick time (this host's monotonic clock; the Pi's when run there)",
    "status": "0 disabled, 1 idle, 2 moving, 3 holding, 4 braking",
    "goal_id": "goal of the planner this tick (0 = none)",
    "x": "pose used this tick: the estimator's latest, extrapolated to t with the commanded twist",
    "y": "pose used this tick (as x)",
    "theta": "heading used this tick (as x), continuous",
    "goal_x": "goal in the estimator's world frame",
    "goal_y": "goal in the estimator's world frame",
    "goal_theta": "goal heading in the estimator's world frame, continuous",
    "err_pos": "distance to the goal",
    "err_theta": "goal_theta - theta",
    "vx": "commanded body twist this tick, forward",
    "vy": "commanded body twist this tick, left",
    "omega": "commanded body twist this tick, yaw rate (CCW +)",
    "ax": "rate of vx: (twist - previous twist) / period",
    "ay": "rate of vy",
    "alpha": "rate of omega",
    "vdes_x": "planner's desired body twist before jerk-limited tracking, forward",
    "vdes_y": "planner's desired body twist, left",
    "wdes": "planner's desired yaw rate",
    "scale_lin": "synchronized mode time scale of the linear axis (1 = full limits)",
    "scale_rot": "synchronized mode time scale of the rotation axis (1 = full limits)",
    "ttg_lin": "predicted time to the position acceptance region",
    "ttg_rot": "predicted time to the heading acceptance region",
    "lin_done": "1 if the linear axis is latched inside its acceptance region",
    "rot_done": "1 if the rotation axis is latched inside its acceptance region",
    "kept": "share of the planned step the limiter kept (1 = no intervention)",
    "boost": "static-friction wheel-speed floor in force (rises while the base does not move)",
    "wheel_max": "fastest wheel speed of the commanded twist",
    "steer_rate": "fastest wheel steering rate of this step",
    "est_age": "age of the estimator pose used",
    "sent": "1 if the twist was accepted by the swerve node",
    "violations": "steps that broke a constraint (expected 0 always)",
}


def _column(name: str, unit: str):
    return Optional[float], Field(description=f"{unit or 'ratio'}; {_TICK_DOC[name]} (null = no value).")


# One field per /stream column (FIELDS), flat like the rows.
ServoTick = create_model("ServoTick", __doc__="The last control tick's row, as in /stream (NaN sent as null).",
                         **{n: _column(n, u) for n, u in zip(FIELDS, UNITS)})


class ServoState(BaseModel):
    chocked: bool = Field(description="A chock lock covers this node or the swerve (the swerve then refuses to "
                                      "enable, so POST /enable fails).")
    covered: bool = Field(description="Always false (no cover locks apply to this node).")
    locks: list[LockOut] = Field(description="The chocks in force.")
    status: str = Field(description="disabled | idle | moving | holding | braking.")
    enabled: bool = Field(description="The servo holds the swerve lease and drives (POST /enable).")
    holder: Optional[str] = Field(description="`client` of POST /enable (`anonymous` if empty); null if disabled.")
    align: AlignState = Field(description="Defensive alignment (POST /align): ended by a goal, stop or disable.")
    disabled_reason: Optional[str] = Field(description="Why it was last disabled; null if never or enabled since.")
    swerve_lease_id: Optional[int] = Field(description="The swerve lease it holds; null if disabled.")
    goal: Optional[GoalOut] = Field(description="The last goal (kept after it ends, with its result); null if none yet.")
    config: ServoConfig
    estimator: EstimatorHealth
    swerve: Optional[SwerveView] = Field(description="The latest swerve telemetry; null before the first.")
    inputs: dict[str, UpstreamStatus] = Field(description="By input: `estimator` (pose stream) and `swerve` "
                                                          "(telemetry stream), each only if configured.")
    violations: int = Field(description="Control steps that broke a constraint (expected 0 always).")
    loop_hz: Optional[float] = Field(description="Control tick rate over the last ~100 ticks (Hz, 0.1 resolution); "
                                                 "null while there are too few.")
    realtime: RealtimeOut = Field(description="How the control loop's thread is scheduled.")
    latest: Optional[ServoTick] = Field(description="The last tick's row; null before the first tick.")
    events: list[Event] = Field(description="Latest node events (up to 30), oldest first.")


class FieldsOut(BaseModel):
    fields: list[str] = Field(description="Column names of the /stream rows, in order.")
    units: list[str] = Field(description="Unit of each column (s, m, rad, m/s, rad/s, m/s^2, rad/s^2; bool = 0/1; "
                                         "enum, id, count; empty = a ratio).")
    doc: dict[str, str] = Field(description="Meaning of the less obvious columns (one entry may cover a group, "
                                            "e.g. `x` for x, y, theta).")


STREAM_DOC = (
    "Records `[u32 big-endian header length N][N bytes UTF-8 JSON header][header.len bytes payload]` "
    "(docs/nodes.md; parse with `common.protocol` or `RU.RecordReader`). Every header has `type` and `len`. "
    "First a `schema` record (no payload): `schema` (id 1), `fields` and `units` (as GET /fields), "
    f"`dtype` `\"<f8\"`, `kind` `\"servo\"`, `node` `\"pose_servo\"`, `rate_hz` {1 / PERIOD:g}, `t_send`. Then `rows` "
    "records: payload a little-endian float64 array of shape (n, len(fields)); header `n`, `schema`, `dropped` (rows "
    "lost because this client fell behind), `kind`, `t_send`. `ping` records (header `t_send` only, no payload) after "
    "~1 s without data. `t_send`: server monotonic s.")


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

    @app.get("/state", response_model=ServoState, tags=["state"], summary="Status, goal and how it ended, config, inputs, latest tick")
    async def get_state():
        return node.state()

    @app.get("/events", tags=["state"], summary="Server-Sent Events: /state on every change and every 1 s",
             response_class=StreamingResponse,
             responses={200: {"content": {"text/event-stream": {}},
                              "description": "Unnamed events; each `data:` line is the GET /state JSON (ServoState)."}})
    async def events():
        return StreamingResponse(sse(node.hub, node.state), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-store"})

    @app.get("/fields", response_model=FieldsOut, tags=["stream"], summary="Columns of the tick rows")
    async def fields():
        return {"fields": FIELDS, "units": UNITS, "doc": FIELD_DOC}

    @app.get("/stream", tags=["stream"], response_class=StreamingResponse,
             summary="Every control tick as binary float64 rows (schema record first)",
             responses={200: {"content": {"application/octet-stream": {}}, "description": STREAM_DOC}})
    async def stream(batch_ms: float = Query(0.0, ge=0, le=1000, description="Collect rows this long per record (0 = every tick)."),
                     max_backlog_s: float = Query(20.0, gt=0, le=600, description="Drop the oldest rows beyond this backlog.")):
        gen = rows_stream(node.hub, FIELDS, UNITS, channel="servo", kind="servo", batch_ms=batch_ms,
                          max_rows=int(max_backlog_s / PERIOD), schema_extra={"node": "pose_servo", "rate_hz": 1 / PERIOD})
        return StreamingResponse(gen, media_type="application/octet-stream", headers={"Cache-Control": "no-store"})

    @app.post("/enable", response_model=ConfigOut, tags=["control"], summary="Take the swerve lease, set its limits to the caps, enable it")
    async def enable(body: EnableIn = EnableIn()):
        """409 if already enabled or the swerve lease is held by someone else (or its e-stop is latched).
        The config (defaults + `config`) lasts until disable."""
        return await asyncio.to_thread(node.enable, body.client, body.config.changes())

    @app.post("/disable", response_model=Disabled, tags=["control"], summary="Stop at once (no braking), disable the swerve, release its lease")
    async def disable():
        return node.disable("requested")

    @app.get("/config", response_model=ConfigOut, tags=["control"], summary="The config in force and the hard limits")
    async def get_config():
        return node.get_config()

    @app.post("/config", response_model=ConfigOut, tags=["control"], summary="Change the config (enabled and idle only)")
    async def set_config(body: ConfigIn):
        return await asyncio.to_thread(node.set_config, body.changes())

    @app.post("/goal", response_model=GoalOut, tags=["control"], summary="Go to a pose (local by default); replaces any motion in progress")
    async def goal(body: GoalIn):
        """422 if the goal is beyond `max_distance` / `max_rotation` or an option exceeds its cap;
        409 if not enabled; 503 if the estimator pose is not usable. Returns the goal resolved to the
        world frame. Its progress and result (reached / stopped / aborted + reason) are in `/state.goal`."""
        opts = body.model_dump(exclude={"frame", "x", "y", "theta"})
        return node.set_goal(body.frame, body.x, body.y, body.theta, opts)

    @app.post("/align", response_model=AlignState, tags=["control"],
              summary="While idle: hold the wheels set for a unit twist with the drives coasting (defensive alignment)",
              description="409 if not enabled or a goal is running. Holds until the next goal, POST /stop, "
                          "DELETE /align or disable. `aligned` turns true once every wheel is within 3 degrees.")
    async def align(body: AlignIn):
        return node.set_align(body.twist)

    @app.delete("/align", response_model=AlignState, tags=["control"], summary="End the defensive alignment")
    async def align_off():
        return node.clear_align("DELETE /align")

    @app.post("/stop", response_model=StopOut, tags=["control"], summary="Brake to a stop within the caps; stay enabled")
    async def stop():
        return node.stop()

    @app.post("/estop", response_model=EStopOut, response_model_exclude_unset=True, tags=["control"], summary="E-stop: forwarded to the swerve (latches, stops at once); disables")
    async def estop(request: Request, body: Optional[EStopIn] = None):
        body = body or EStopIn()
        src = body.source or (request.client.host if request.client else "")
        return await asyncio.to_thread(node.estop, body.reason, src)

    return app
