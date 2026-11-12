"""HTTP API of the arm node (FastAPI). Design: docs/arm.md; conventions: docs/nodes.md.

Controlling requests carry the lease token (``X-Lease-Token``). Handlers only touch the node's
locked state; the bus belongs to the control thread. Blocking work (enable's gripper check,
planning) runs in a worker thread. Poses are in the arm frame (the arm's URDF ``base_link``; ``GET
/geometry`` gives its pose in the robot base frame): a position [x, y, z] m plus either ``quat``
[x, y, z, w] or ``rpy`` [roll, pitch, yaw] rad.
"""
from __future__ import annotations

import asyncio
import time
from contextlib import asynccontextmanager
from typing import Literal, Optional

from fastapi import FastAPI, Header, Query, Request
from fastapi.responses import HTMLResponse, Response, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field, model_validator

from ..common.lease import RequestError
from ..common.server import EStopCleared, EStopTriggered, LeaseToken, add_base_routes, add_page_routes, node_meta, rows_stream, setup_app, sse
from . import config as C
from .kin import se3
from . import models as M
from .render import VIEW_DEFAULTS, VIEW_DOC
from .node import FIELD_DOC, FIELDS, JAC_FIELDS, JAC_UNITS, LEASE_HEARTBEAT, RATE_HZ, TWIST_TIMEOUT, UNITS, ArmNode

TAGS = [
    {"name": "state", "description": "Status, clock sync, events."},
    {"name": "lease", "description": "One controller at a time: acquire a lease, send its token as `X-Lease-Token`."},
    {"name": "control", "description": "Enable (runs the gripper check the first time), disable, e-stop, reset."},
    {"name": "motion", "description": "Track (streamed, low level), servo / named poses / home (planned), chunks "
                                      "(time-indexed poses), gripper. One source at a time; a new one preempts."},
    {"name": "kinematics", "description": "IK checks, TCP pose, Jacobian and manipulability; lookups at any joint "
                                          "configuration (computation only, nothing moves, no lease)."},
    {"name": "settings", "description": "Gain profile, TCP offset, speed limits."},
    {"name": "stream", "description": "Every control tick (100 Hz) as binary float64 rows."},
]


class Pose(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False, json_schema_extra={"examples": [{"pos": [0.25, 0.0, 0.2], "rpy": [0.0, 1.57, 0.0]}]})
    pos: list[float] = Field(min_length=3, max_length=3, description="m, arm frame (the arm's URDF base_link; its pose in the "
                                                                  "robot base frame: GET /geometry `frames.arm_in_base`).")
    quat: Optional[list[float]] = Field(None, min_length=4, max_length=4, description="[x, y, z, w].")
    rpy: Optional[list[float]] = Field(None, min_length=3, max_length=3, description="[roll, pitch, yaw] rad, fixed X-Y-Z.")

    @model_validator(mode="after")
    def _one_orientation(self):
        if (self.quat is None) == (self.rpy is None):
            raise ValueError("give exactly one of quat or rpy")
        return self

    def se3(self):
        return se3(self.pos, self.quat, self.rpy)


class GripperIn(BaseModel):
    gap_mm: Optional[float] = Field(None, ge=0, le=round(C.GAP_MAX * 1000, 1), allow_inf_nan=False,
                                    description="Jaw gap goal (0 = closed).")
    max_force_n: Optional[float] = Field(None, ge=0, le=C.HARD_MAX_FORCE, allow_inf_nan=False,
                                         description=f"Squeeze force limit (default {C.DEFAULT_MAX_FORCE:g} N when omitted).")


class TrackIn(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)
    pose: Pose
    twist: Optional[list[float]] = Field(None, min_length=6, max_length=6,
                                         description=f"TCP velocity [vx, vy, vz, wx, wy, wz] (m/s, rad/s): joint velocity "
                                                     f"feedforward for {TWIST_TIMEOUT} s.")
    twist_frame: Literal["arm", "tcp", "base"] = Field("arm", description="Axes of the twist: the arm frame's or the TCP's own "
                                                                       "(`base` is the old name of `arm`, still accepted).")
    gripper: Optional[GripperIn] = Field(None, description="Also command the gripper (as POST /gripper).")
    seq: int = Field(0, description="Echoed in the stream (`seq`); debugging only.")


class ServoIn(BaseModel):
    pose: Optional[Pose] = Field(None, description="TCP goal pose, arm frame (a Cartesian move); or give `name`.")
    name: Optional[str] = Field(None, description="A named pose (joint-space move) instead of `pose`.")
    lin_vel: Optional[float] = Field(None, gt=0, allow_inf_nan=False, description="m/s (default: the soft limit).")
    lin_acc: Optional[float] = Field(None, gt=0, allow_inf_nan=False, description="m/s².")
    ang_vel: Optional[float] = Field(None, gt=0, allow_inf_nan=False, description="rad/s.")
    ang_acc: Optional[float] = Field(None, gt=0, allow_inf_nan=False, description="rad/s².")
    duration: Optional[float] = Field(None, gt=0, le=60, allow_inf_nan=False,
                                      description="Exact duration (s). Too short for the limits: nothing is planned and the "
                                                  "response's `retry` gives the shortest that works.")
    max_rot_deg: Optional[float] = Field(None, ge=0, le=30, allow_inf_nan=False,
                                         description="Accept an IK solution whose orientation is within this many degrees of the "
                                                     "request (default 10; the position must be within 2 mm). 1 asks for an exact pose.")

    @model_validator(mode="after")
    def _one_target(self):
        if (self.pose is None) == (self.name is None):
            raise ValueError("give exactly one of pose or name")
        return self


class ChunkIn(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)
    poses: list[Pose] = Field(min_length=1, max_length=1000, description="TCP poses, arm frame, `dt` apart from `t0`.")
    dt: float = Field(gt=0, le=1.0, allow_inf_nan=False, description="s between poses.")
    t0: Optional[float] = Field(None, allow_inf_nan=False,
                                description="Server monotonic s when poses[0] is due (GET /time); default now. "
                                            "Samples already past are skipped.")
    gaps_mm: Optional[list[float]] = Field(None, description="Gripper gap per pose (mm).")
    blend_s: float = Field(0.1, ge=0, le=1.0, description="Join the chunk this long from now.")


class IKIn(BaseModel):
    pose: Pose
    seed: Literal["command", "measured"] = Field("command", description="Where IK starts: `command` = the joint "
                                                       "command while servoing (else the measured angles); `measured` = the "
                                                       "measured angles. Only a starting point: the answer is the solution.")
    max_rot_deg: Optional[float] = Field(None, ge=0, le=30, allow_inf_nan=False,
                                         description="Accept an IK solution whose orientation is within this many degrees of the "
                                                     "request (default 10; the position must be within 2 mm). 1 asks for an exact pose.")


class RenderIn(BaseModel):
    """View of the agent picture (render.py). Every field is optional."""
    model_config = ConfigDict(allow_inf_nan=False, json_schema_extra={
        "description": VIEW_DOC, "examples": [{}, {"azimuth_deg": 0, "elevation_deg": 0}, {"azimuth_deg": 90, "elevation_deg": 90, "zoom": 1.5}]})
    azimuth_deg: Optional[float] = Field(None, ge=-360, le=360, description=f"Look from this direction about +z: 0 = from +x toward the base, 90 = from +y (default {VIEW_DEFAULTS['azimuth_deg']:g}).")
    elevation_deg: Optional[float] = Field(None, ge=-90, le=90, description=f"0 = level, 90 = from straight above (default {VIEW_DEFAULTS['elevation_deg']:g}).")
    zoom: Optional[float] = Field(None, ge=0.25, le=8, description="1 = the arm, target and nearest pose fill the image; >1 magnifies about their centre.")
    width: Optional[int] = Field(None, ge=160, le=1600, description=f"px (default {VIEW_DEFAULTS['width']}).")
    height: Optional[int] = Field(None, ge=120, le=1200, description=f"px (default {VIEW_DEFAULTS['height']}).")


class IKRenderIn(IKIn):
    render: Optional[RenderIn] = Field(None, description="Also return `image`: a PNG (data URL) of the query for a vision model: "
                                                          "the measured arm, the IK solution (or nearest reachable configuration) as a "
                                                          "ghost arm, the target and nearest TCP frames, joints at a limit, labelled base "
                                                          "axes and a scale. " + VIEW_DOC)


class KinIn(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)
    q: list[list[float]] = Field(min_length=1, max_length=200,
                                 description="Joint configurations (6 rad each). Inputs to a computation only.")
    frame: Literal["arm", "tcp", "base"] = Field("arm", description="Jacobian axes: the arm frame's (`arm`; `base` is its old "
                                                                    "name) or the TCP's own (`tcp`).")


class GeometryIn(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)
    q: Optional[list[float]] = Field(None, min_length=6, max_length=6, description="Joint angles (rad): the geometry there.")
    pose: Optional[Pose] = Field(None, description="A TCP pose (arm frame): the geometry at its IK solution (or the nearest "
                                                   "reachable configuration; `ik` says which). Nothing moves.")
    seed: Literal["command", "measured"] = Field("measured", description="IK start for `pose`, as for POST /ik.")
    max_rot_deg: Optional[float] = Field(None, ge=0, le=30, description="As for POST /ik.")
    gap_mm: Optional[float] = Field(None, ge=0, le=round(C.GAP_MAX * 1000, 1), description="Gripper gap to place the jaws at "
                                                                                          "(default: the measured gap).")

    @model_validator(mode="after")
    def _at_most_one(self):
        if self.q is not None and self.pose is not None:
            raise ValueError("give q or pose, not both")
        return self


class ProfileIn(BaseModel):
    name: Literal["soft", "safe"] = Field(description="Gain profile: `soft` (production) or `safe` (debug).")


class TcpIn(BaseModel):
    offset: list[float] = Field(min_length=3, max_length=3, description="m in end_link.")


class LimitsIn(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)
    joint_vel: Optional[list[float]] = Field(None, min_length=6, max_length=6, description="rad/s per joint.")
    joint_acc: Optional[list[float]] = Field(None, min_length=6, max_length=6,
                                             description="rad/s² per joint: limits only increases of commanded speed.")
    lin_vel: Optional[float] = Field(None, description="m/s, TCP speed of servo moves.")
    lin_acc: Optional[float] = Field(None, description="m/s², TCP acceleration of servo moves.")
    ang_vel: Optional[float] = Field(None, description="rad/s, TCP angular speed of servo moves.")
    ang_acc: Optional[float] = Field(None, description="rad/s², TCP angular acceleration of servo moves.")
    home_vel: Optional[float] = Field(None, description="rad/s, joint-space moves.")


class LeaseIn(BaseModel):
    client: str = Field("anonymous", description="Your name, shown to others as the holder.")


class EStopIn(BaseModel):
    reason: str = Field("", description="Why (free text; `e-stop` if empty).")
    source: str = Field("", description="Who pressed it (free text; default: the caller's IP).")


STREAM_DOC = ("Binary records as every node's /stream (docs/nodes.md): a `schema` record then float64 `rows`. "
              "`channel=tick` (default): one row per control tick, columns as GET /fields. `channel=jacobian`: "
              "t_tick and the 36 entries of the TCP Jacobian (row-major, base-frame axes) at the measured pose.")


class JamIn(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, json_schema_extra={"examples": [
        {"response": "report"}, {"bands": [1.0, 4.0, 4.0, 1.5, 0.8, 0.6], "hold_s": 1.0}, {"response": "off"}]})
    bands: Optional[list[float]] = Field(None, min_length=6, max_length=6, description="Nm per joint (all > 0).")
    hold_s: Optional[float] = Field(None, gt=0, description="s over its band before it fires.")
    release: Optional[float] = Field(None, gt=0, lt=1, description="Clears below release x band.")
    static_qd: Optional[float] = Field(None, gt=0, description="rad/s: every joint slower than this = static.")
    settle_s: Optional[float] = Field(None, ge=0, description="s static before judging.")
    response: Optional[Literal["hold", "report", "off"]] = Field(None, description="hold | report | off.")
    refire: Optional[bool] = Field(None, description="Fire again after clearing (false = once per command).")


def create_app(node: ArmNode, start: bool = True, sim: bool = False) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app):
        node.hub.attach(asyncio.get_running_loop())
        name = getattr(app.state, "node_meta", {}).get("name")
        if name:
            node.lock_names.add(name)
        if start:
            node.start()
        yield
        await asyncio.to_thread(node.close)

    app = FastAPI(
        title="Arm node",
        description=(
            "Gravity-compensated Cartesian control of the reBot Arm B601-DM and its 100 Hz telemetry.\n\n"
            "States: off (motors off, damping) → enable → floating (gravity compensated, no servoing; the "
            "first arm command engages servoing from the measured pose) ⇄ servoing; `POST /float` back to "
            "floating; `POST /disable` (or a lapsed lease, a chock): arm floating, gripper in damping, commands "
            "refused; `POST /stow`: safe homing then off; `POST /estop`: off, latched.\n\n"
            "`POST /lease` → token; `POST /enable` (the first enable closes the gripper to check it); then "
            "`POST /track` (stream poses; the optional twist feedforward lasts "
            f"{TWIST_TIMEOUT} s, the pose holds), `POST /servo` (planned move, reports reached), `POST /chunk`, "
            "`POST /home`, `POST /gripper`. "
            f"The lease lapses after {LEASE_HEARTBEAT} s without a renew or command, which disables. "
            "No joint commands exist by design; joint positions are readings, named poses and lookup inputs only."),
        version="0.1.0", lifespan=lifespan, openapi_tags=TAGS)
    setup_app(app)

    @app.exception_handler(ValueError)
    async def _value_error(request: Request, exc: ValueError):
        from fastapi.responses import JSONResponse
        return JSONResponse(status_code=422, content={"detail": str(exc)})

    add_base_routes(app, lambda: {"node": "arm", "state": node.state})
    add_page_routes(app, node_meta("arm", "Arm", app.version, "arm", actuated=True, rate_hz=RATE_HZ,
                                   fake=sim))

    @app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def index():
        from .page import PAGE
        return PAGE

    # -- state ------------------------------------------------------------------------------------

    @app.get("/state", response_model=M.ArmStateOut, response_model_exclude_unset=True, tags=["state"], summary="Status, lease, motion, gripper, settings, latest telemetry")
    async def get_state():
        return node.state_dict()

    @app.get("/events", tags=["state"], response_class=StreamingResponse,
             summary="Server-Sent Events: /state on every change and every 1 s",
             responses={200: {"content": {"text/event-stream": {}},
                              "description": "Unnamed events; each `data:` line is the GET /state JSON (ArmStateOut)."}})
    async def events():
        return StreamingResponse(sse(node.hub, node.state_dict), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-store"})

    @app.get("/fields", response_model=M.FieldsOut, response_model_exclude_unset=True, tags=["stream"], summary="Columns of the stream rows")
    async def fields(channel: Literal["tick", "jacobian"] = "tick"):
        if channel == "jacobian":
            return {"fields": JAC_FIELDS, "units": JAC_UNITS, "doc": FIELD_DOC}
        return {"fields": FIELDS, "units": UNITS, "doc": FIELD_DOC}

    @app.get("/stream", tags=["stream"], response_class=StreamingResponse, summary="Telemetry rows",
             responses={200: {"content": {"application/octet-stream": {}}, "description": STREAM_DOC}})
    async def stream(channel: Literal["tick", "jacobian"] = "tick",
                     batch_ms: float = Query(0.0, ge=0, le=1000),
                     max_hz: float = Query(0.0, ge=0, le=100, description="At most this many rows per second (0 = all)."),
                     max_backlog_s: float = Query(20.0, gt=0, le=600)):
        f, u = (JAC_FIELDS, JAC_UNITS) if channel == "jacobian" else (FIELDS, UNITS)
        gen = rows_stream(node.hub, f, u, channel=channel, kind=channel, batch_ms=batch_ms,
                          max_rows=int(max_backlog_s * RATE_HZ),
                          schema_extra={"node": "arm", "rate_hz": RATE_HZ}, max_hz=max_hz)
        return StreamingResponse(gen, media_type="application/octet-stream", headers={"Cache-Control": "no-store"})

    @app.get("/joints", response_model=M.JointsOut, response_model_exclude_unset=True, tags=["state"], summary="Joint readings (output encoders, motor side, temperatures, limits)")
    async def joints():
        return node.joints()

    @app.get("/tracking", response_model=M.TrackingOut, response_model_exclude_unset=True, tags=["state"], summary="Tracking errors of the last tick")
    async def tracking():
        return node.tracking()

    JAM_DOC = ("Jam detection: an arm that is static (no motion executing, every joint slow, after a short settle) but "
               "still pushing: a joint's residual |measured torque - gravity model| above its band for `hold_s`. It "
               "detects the aftermath of a contact (pressed against something, or pushed by the base), not the instant "
               "of it. Response `hold` pulls the command to where the arm is (a planned move ends `jammed`); while "
               "tracking it only reports. Fires again after clearing unless `refire` is false.")

    @app.get("/jam", response_model=M.JamState, tags=["settings"], summary="Jam detection: settings, the jam in force, "
             "recent jams", description=JAM_DOC)
    async def get_jam():
        return node.jam_state()

    @app.post("/jam", response_model=M.JamState, tags=["settings"],
              summary="(human) Change jam detection: bands, timing, response (hold | report | off)",
              description=JAM_DOC + " Changes need the header `X-Operator: human`: the person in charge decides, "
                                    "not the lease holder or an agent.")
    async def set_jam(body: JamIn, x_operator: Optional[str] = Header(None)):
        if (x_operator or "").lower() != "human":
            raise RequestError(403, "jam detection is the human operator's to change (X-Operator: human)")
        return node.set_jam(body.model_dump(exclude_none=True))

    @app.get("/gains", response_model=M.GainsOut, response_model_exclude_unset=True, tags=["settings"], summary="The gain profile in force")
    async def gains():
        return node.profile.public()

    @app.get("/gripper", response_model=M.GripperOut, response_model_exclude_unset=True, tags=["motion"], summary="Gripper gap, force and limit")
    async def get_gripper():
        return node.gripper_state()

    # -- lease and control ---------------------------------------------------------------------------

    @app.get("/lease", response_model=Optional[M.LeaseHolder], response_model_exclude_unset=True, tags=["lease"], summary="Who holds the lease (no token)")
    async def get_lease():
        info = node.lease.info
        return None if info is None else {"client": info.client, "lease_id": info.lease_id,
                                          "expires_in": info.expires - time.monotonic()}

    @app.post("/lease", response_model=M.LeaseAcquired, response_model_exclude_unset=True, tags=["lease"], summary="Acquire the lease (409 if held)")
    async def acquire(body: LeaseIn):
        token, info = node.acquire(body.client)
        return {"token": token, "client": info.client, "lease_id": info.lease_id, "heartbeat": node.lease.heartbeat}

    @app.post("/lease/renew", response_model=M.LeaseRenewed, response_model_exclude_unset=True, tags=["lease"], summary="Keep the lease (any accepted command also renews it)")
    async def renew(token: LeaseToken = None):
        info = node.renew(token)
        return {"lease_id": info.lease_id, "expires_in": info.expires - time.monotonic()}

    @app.delete("/lease", response_model=M.LeaseReleased, response_model_exclude_unset=True, tags=["lease"], summary="Release the lease (disables)")
    async def release(token: LeaseToken = None):
        node.release(token)
        return {"released": True}

    @app.post("/enable", response_model=M.EnableOut, response_model_exclude_unset=True, tags=["control"], summary="Enable: the arm floats until a command engages it (holder; first time: gripper check)")
    async def enable(token: LeaseToken = None):
        """Motors on, gravity compensated. The arm floats (kp = kd = 0) until the first closed-loop arm
        command (track, servo, chunk, home, stow) engages servoing from the measured pose; the gripper
        is free until a gripper command makes it hold (disable puts it into damping; enable frees it again)."""
        return await asyncio.to_thread(node.enable, token)

    @app.post("/disable", response_model=M.DisableOut, response_model_exclude_unset=True, tags=["control"], summary="Disable: the arm floats, the gripper goes into damping, commands refused (holder)")
    async def disable(token: LeaseToken = None):
        return node.disable(token)

    @app.post("/float", response_model=M.StateWord, response_model_exclude_unset=True, tags=["control"], summary="Float the arm: stays enabled, servoing off, gravity on (holder); the gripper is untouched")
    async def float_arm(token: LeaseToken = None):
        return node.float_arm(token)

    @app.post("/stow", response_model=M.StowOut, response_model_exclude_unset=True, tags=["control"], summary="Safe homing, then motors off (damping) at home; at once if already within 3° of home (holder)")
    async def stow(token: LeaseToken = None):
        return await asyncio.to_thread(node.stow, token)

    @app.post("/estop", response_model=EStopTriggered, response_model_exclude_unset=True, tags=["control"], summary="E-stop: anyone; all motors off (damping); latches until the holder clears it")
    async def estop(request: Request, body: Optional[EStopIn] = None):
        body = body or EStopIn()
        src = body.source or (request.client.host if request.client else "")
        info = node.trigger_estop(body.reason, src)
        return {"latched": True, "reason": info.reason, "source": info.source}

    @app.post("/estop/clear", response_model=EStopCleared, response_model_exclude_unset=True, tags=["control"], summary="Clear the e-stop (holder; stays disabled)")
    async def clear_estop(token: LeaseToken = None):
        node.clear_estop(token)
        return {"latched": False}

    @app.post("/reset", response_model=M.ResetOut, response_model_exclude_unset=True, tags=["control"], summary="Reconnect after a fault (holder; ends uninitialized)")
    async def reset(token: LeaseToken = None):
        node.reset(token)
        return {"state": node.state}

    # -- motion -----------------------------------------------------------------------------------------

    @app.post("/track", response_model=M.TrackOut, response_model_exclude_unset=True, tags=["motion"], summary="Stream a TCP pose (+ twist feedforward, gripper); 200 with ok=false if IK fails")
    async def track(body: TrackIn, token: LeaseToken = None):
        """Low level and rapid: IK from the previous goal; on failure the previous goal stays and
        `ok` is false (with the residuals). The pose holds until replaced; the twist feeds forward
        for 0.25 s. There is no "reached": watch the streamed errors."""
        g = body.gripper.model_dump() if body.gripper else None
        return await asyncio.to_thread(node.track_cmd, token, body.pose.se3(), body.twist, body.twist_frame,
                                       body.seq, g)

    @app.post("/servo", response_model=M.MotionOut, response_model_exclude_unset=True, tags=["motion"], summary="Planned move to a pose or a named pose (422 if it can't be planned: nothing moves; `code`, `retry`, `nearest` say how to adjust)")
    async def servo(body: ServoIn, token: LeaseToken = None):
        return await asyncio.to_thread(node.servo_cmd, token, body.pose.se3() if body.pose else None, body.name,
                                       body.lin_vel, body.lin_acc, body.ang_vel, body.ang_acc, body.duration,
                                       body.max_rot_deg)

    @app.post("/home", response_model=M.MotionOut, response_model_exclude_unset=True, tags=["motion"], summary="Slow joint-space move to the home pose (through home_via)")
    async def home(token: LeaseToken = None):
        return await asyncio.to_thread(node.home, token)

    @app.post("/chunk", response_model=M.MotionOut, response_model_exclude_unset=True, tags=["motion"], summary="Time-indexed TCP poses (an action chunk); 422 if it can't be planned (nothing moves; `code`, `retry` say why)")
    async def chunk(body: ChunkIn, token: LeaseToken = None):
        return await asyncio.to_thread(node.chunk, token, [p.se3() for p in body.poses], body.dt, body.t0,
                                       body.gaps_mm, body.blend_s)

    @app.post("/cancel", response_model=M.CancelOut, response_model_exclude_unset=True, tags=["motion"], summary="End the planned motion or track: hold the current command")
    async def cancel(token: LeaseToken = None):
        return node.cancel(token)

    @app.post("/gripper", response_model=M.GripperOut, response_model_exclude_unset=True, tags=["motion"], summary="Gripper gap (mm) with a force limit: makes it hold")
    async def gripper(body: GripperIn, token: LeaseToken = None):
        return node.gripper_cmd(token, body.gap_mm, body.max_force_n)

    # -- kinematics ---------------------------------------------------------------------------------

    @app.post("/ik", response_model=M.IKOut, response_model_exclude_unset=True, tags=["kinematics"], summary="IK check only (nothing moves, no lease): always returns `nearest`, the closest reachable pose",
              description="Answers `{ok, exact, q, pos_err_mm, rot_err_deg, within_limits, joints_at_limit, iterations, reason, "
                          "nearest}`. With `render` (an object, `{}` for the defaults) it also returns `image`, a PNG data URL "
                          "showing the query, and `view`, the view used: for agents (vision models) choosing how to move. "
                          "Ask for several views to understand depth (e.g. azimuth 0 and 90 at elevation 0, or elevation 90 "
                          "from above). " + VIEW_DOC)
    async def ik(body: IKRenderIn):
        return await asyncio.to_thread(node.ik_check, body.pose.se3(), body.seed, body.max_rot_deg,
                                       None if body.render is None else body.render.model_dump())

    @app.post("/ik/render", tags=["kinematics"], summary="The IK check as a PNG image only (for agents; nothing moves, no lease)",
              response_class=Response, responses={200: {"content": {"image/png": {}}}},
              description="The same query and picture as `POST /ik` with `render`, answered as `image/png`; the IK result "
                          "is in the headers `X-IK-Ok`, `X-IK-Exact`, `X-IK-Pos-Err-Mm`, `X-IK-Rot-Err-Deg`. " + VIEW_DOC)
    async def ik_render(body: IKRenderIn):
        out, png = await asyncio.to_thread(node.ik_image, body.pose.se3(), body.seed, body.max_rot_deg,
                                           (body.render or RenderIn()).model_dump())
        return Response(png, media_type="image/png", headers={
            "X-IK-Ok": str(out["ok"]).lower(), "X-IK-Exact": str(out["exact"]).lower(),
            "X-IK-Pos-Err-Mm": f"{out['pos_err_mm']:.3f}", "X-IK-Rot-Err-Deg": f"{out['rot_err_deg']:.3f}"})

    @app.get("/kinematics", response_model=M.KinematicsNow, response_model_exclude_unset=True, tags=["kinematics"], summary="TCP pose, Jacobian and manipulability now")
    async def kin_now(frame: Literal["arm", "tcp", "base"] = "arm", which: Literal["measured", "command"] = "measured"):
        return node.kinematics_now(frame, which)

    @app.post("/kinematics", response_model=M.KinematicsBatch, response_model_exclude_unset=True, tags=["kinematics"], summary="The same at given joint configurations (planning lookups)")
    async def kin_lookup(body: KinIn):
        return {"results": await asyncio.to_thread(node.kinematics_batch, body.q, body.frame)}

    GEOM_DOC = ("Read only: nothing moves, no lease. Every point in both the arm frame and the robot base frame, with the "
                "transform between them (`frames.arm_in_base`, from the description node). `fingertips`: each jaw's tip "
                "(left, right) and the centre between them (end_link). `jaws`: per jaw its tip, inner (gripping) face "
                "centre and root, and outer face; `jaw_shape_m.grasp_depth`: how far an object can sit between the jaws. "
                "`tcp`: the point motion commands target, near the wrist, at `tcp.offset_in_end_link` from the fingertip "
                "centre; to put the fingertips somewhere, command the TCP pose that places them there. `extent_base`: the "
                "arm's bounds in the base frame and how far it reaches past the frame. `boxes`: every part as an oriented "
                "box (approximate, from the meshes).")

    @app.get("/geometry", response_model=M.GeometryOut, response_model_exclude_unset=True, tags=["kinematics"], summary="Fingertips, jaws and the arm's extent as numbers, now (both frames)",
             description=GEOM_DOC)
    async def geometry_now():
        return await asyncio.to_thread(node.geometry)

    @app.post("/geometry", response_model=M.GeometryOut, response_model_exclude_unset=True, tags=["kinematics"], summary="The same at given joint angles or at a TCP pose's IK solution",
              description=GEOM_DOC)
    async def geometry_at(body: GeometryIn):
        return await asyncio.to_thread(node.geometry, body.q, None if body.pose is None else body.pose.se3(), body.seed,
                                       body.max_rot_deg, body.gap_mm)

    @app.get("/links", response_model=M.LinksOut, response_model_exclude_unset=True, tags=["kinematics"], summary="Joint frames, end_link and tcp at the measured pose (drawing)")
    async def links():
        return {"frames": node.links(), "names": [f"joint{i}" for i in range(1, 7)] + ["end_link", "tcp"]}

    # -- settings ---------------------------------------------------------------------------------------

    @app.post("/profile", response_model=M.GainsOut, response_model_exclude_unset=True, tags=["settings"], summary="Switch the gain profile (holder; only while disabled)")
    async def profile(body: ProfileIn, token: LeaseToken = None):
        return node.set_profile(token, body.name)

    @app.get("/tcp", response_model=M.TcpOut, response_model_exclude_unset=True, tags=["settings"], summary="TCP offset in end_link (m)")
    async def get_tcp():
        return {"offset": None if node.kin is None else node.kin.tcp_offset}

    @app.post("/tcp", response_model=M.TcpOut, response_model_exclude_unset=True, tags=["settings"], summary="Set the TCP offset (holder; only while disabled)")
    async def set_tcp(body: TcpIn, token: LeaseToken = None):
        return node.set_tcp(token, body.offset)

    @app.get("/limits", response_model=M.LimitsOut, response_model_exclude_unset=True, tags=["settings"], summary="Hard and soft speed limits")
    async def get_limits():
        return node.limits()

    @app.post("/limits", response_model=M.LimitsOut, response_model_exclude_unset=True, tags=["settings"], summary="Soft limits within the hard ones (holder; reset when the lease ends)")
    async def set_limits(body: LimitsIn, token: LeaseToken = None):
        return node.set_limits(token, **body.model_dump())

    return app
