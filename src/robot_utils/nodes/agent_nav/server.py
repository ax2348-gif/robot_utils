"""HTTP API of agent_nav (FastAPI). Guide for agents: GET /guide.md (docs/agent_nav.md)."""
from __future__ import annotations

import asyncio
import os
import time
from contextlib import asynccontextmanager
from importlib import resources
from typing import Literal, Optional

from fastapi import FastAPI, Header, Query, Request
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, PlainTextResponse, Response, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field, model_validator

from ..common.hub import RowHub
from ..common.lease import RequestError
from ..common.server import add_base_routes, add_page_routes, node_meta, setup_app, sse
from . import __version__
from . import chat as chatmod
from . import models as M
from .node import AgentNav

Operator = Header(None, alias="X-Operator", description="'human' for the page / a person; omit as an agent.")

TAGS = [
    {"name": "guide", "description": "Read GET /guide.md first."},
    {"name": "session", "description": "Who is driving; enable / disable (= pose_servo + the swerve lease)."},
    {"name": "perception", "description": "READ-ONLY: snapshots, selections, region / plane queries, overlays, BEV."},
    {"name": "planning", "description": "READ-ONLY: path checks and proposals. Nothing moves."},
    {"name": "motion", "description": "The only calls that move the base: explicit goals, plus stop / e-stop."},
    {"name": "memory", "description": "Per-session notes and named places."},
    {"name": "log", "description": "The dialogue transcript (API boundary), live and per session."},
]

class Shape(BaseModel):
    """A selection of colour-image pixels (u right, v down, origin top left): exactly one of the four."""
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, json_schema_extra={"examples": [
        {"point": [320, 240]}, {"disk": [320, 240, 8]}, {"box": [200, 10, 350, 80]},
        {"polygon": [[190, 10], [350, 10], [345, 75], [195, 75]]}]})
    point: Optional[list[float]] = Field(None, min_length=2, max_length=2, description="[u, v]: one pixel.")
    disk: Optional[list[float]] = Field(None, min_length=3, max_length=3, description="[u, v, r]: pixels within r of (u, v).")
    box: Optional[list[float]] = Field(None, min_length=4, max_length=4, description="[u0, v0, u1, v1]: a screen-aligned "
                                                                                     "rectangle (corners).")
    polygon: Optional[list[list[float]]] = Field(None, min_length=3, description="[[u, v], ...]: the pixels inside the "
                                                                                 "outline (3 or more corners, in order).")

    @model_validator(mode="after")
    def _exactly_one(self):
        if sum(v is not None for v in (self.point, self.disk, self.box, self.polygon)) != 1:
            raise ValueError("a selection has exactly one of point, disk, box, polygon")
        return self


def shape_dict(s: Optional[Shape]) -> Optional[dict]:
    return None if s is None else s.model_dump(exclude_none=True)


class Commented(BaseModel):
    comment: str = Field("", description="Optional note for the transcript (why you are doing this).")
    model_config = ConfigDict(extra="forbid")


class SessionIn(Commented):
    agent: str = Field(..., description="Your name, shown in the dialogue.")
    avatar: str = Field("", description="An emoji / short text (<= 16 chars) or a data:image/... URI (<= 64 KB).")
    model_config = ConfigDict(extra="forbid", json_schema_extra={"examples": [{"agent": "Claude", "avatar": "🦉"}]})


class CapsIn(BaseModel):
    """Caps below the profile's for this session (each <= the profile's value; `GET /state` shows the profile)."""
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    max_speed: Optional[float] = Field(None, gt=0, description="m/s.")
    max_omega: Optional[float] = Field(None, gt=0, description="rad/s, turn rate.")
    max_accel: Optional[float] = Field(None, gt=0, description="m/s².")
    max_alpha: Optional[float] = Field(None, gt=0, description="rad/s², angular acceleration.")
    max_jerk: Optional[float] = Field(None, gt=0, description="m/s³.")
    max_ang_jerk: Optional[float] = Field(None, gt=0, description="rad/s³.")
    max_distance: Optional[float] = Field(None, gt=0, description="m: the farthest a single goal may be.")
    max_rotation: Optional[float] = Field(None, gt=0, description="rad: the largest turn a single goal may make.")


class EnableIn(Commented):
    caps: Optional[CapsIn] = Field(None, description="Lower any cap of the profile for this session (never raise).")


class GridIn(BaseModel):
    model_config = ConfigDict(extra="forbid")
    step: Optional[int] = Field(None, ge=2, le=400, description="Grid spacing in colour pixels (look: 100; zoom: chosen "
                                                                "to fit the box).")
    values: Optional[Literal["x", "y", "z", "range", "xz", "xyz"]] = Field(
        None, description="Write the base-frame value of the depth at each grid intersection (m): z = height, x = ahead, "
                          "y = left, range = horizontal distance, xz = ahead and height (with the pixel, a full 3D point), "
                          "xyz = all three. Also returned as numbers (`grid.samples`).")


class LookIn(Commented):
    scale: Optional[float] = Field(None, description="Depth grid = colour x scale; default the node's (0.5); 1.0 = full "
                                   "resolution (slower, briefly applies to every viewer).")
    grid: bool | GridIn = Field(False, description="Draw a labelled pixel grid on the returned image: true (100 px), or "
                                                   "{step, values} to set the spacing and write the depth's base-frame value "
                                                   "(e.g. `z`: a height grid) at each intersection, also returned as `grid.samples`.")
    model_config = ConfigDict(extra="forbid", json_schema_extra={"examples": [
        {"grid": True}, {"grid": {"step": 50, "values": "xz"}, "comment": "distance and height at each grid point"}, {"scale": 1.0, "settle": True}]})
    settle: bool = Field(False, description="Wait (<= 8 s) until the base is not moving before capturing.")


class PlacementIn(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    xyz: list[float] = Field([0.0, 0.0, 0.0], min_length=3, max_length=3,
                             description="m: move the frame's origin by this, in base axes (|each| <= 0.05).")
    rpy_deg: list[float] = Field([0.0, 0.0, 0.0], min_length=3, max_length=3,
                                 description="degrees: turn the frame about its origin, fixed base axes X, Y, Z (|each| <= 3).")


class SessionGeometryIn(Commented):
    camera: Optional[PlacementIn] = Field(None, description="Correction of the colour camera's pose (omitted: unchanged; "
                                                            "{}: none).")
    arm: Optional[PlacementIn] = Field(None, description="Correction of the arm mount (the arm frame) (omitted: unchanged).")
    model_config = ConfigDict(extra="forbid", json_schema_extra={"examples": [
        {"camera": {"xyz": [0.01, 0.0, 0.0]}, "comment": "the drawn fingertips sit 1 cm behind the real ones"},
        {"camera": {}, "arm": {}, "comment": "back to the description"}]})


class ZoomIn(Commented):
    snapshot: Optional[str] = Field(None, description="Snapshot id (default: the latest). Nothing is captured.")
    box: Optional[list[float]] = Field(None, min_length=4, max_length=4,
                                       description="[u0, v0, u1, v1] colour pixels to enlarge (default: the whole image).")
    step: Optional[int] = Field(None, ge=2, le=400, description="Grid spacing, colour pixels (default: about 8 lines).")
    values: Optional[Literal["x", "y", "z", "range", "xz", "xyz"]] = Field(None, description="As for look's grid.values.")
    model_config = ConfigDict(extra="forbid", json_schema_extra={"examples": [
        {"box": [380, 260, 500, 350], "values": "xz"}, {"snapshot": "s4", "box": [300, 200, 460, 320], "step": 10}]})


class QueryIn(Commented):
    snapshot: Optional[str] = Field(None, description="Snapshot id (default: the latest).")
    select: Shape = Field(..., description='{"point":[u,v]} | {"disk":[u,v,r]} | {"box":[u0,v0,u1,v1]} | '
                          '{"polygon":[[u,v],...]} in colour-image pixels.')
    exclude: list[Shape] = Field(default_factory=list, description="Shapes removed from the selection.")
    model_config = ConfigDict(extra="forbid", json_schema_extra={"examples": [
        {"select": {"box": [200, 10, 350, 80]}},
        {"select": {"polygon": [[190, 10], [350, 10], [345, 75], [195, 75]]}, "exclude": [{"disk": [260, 30, 12]}]}]})


class PoseIn(Commented):
    frame: str = Field("local", description="local (default: x forward, y left; theta literal) | global (theta: "
                       "shortest turn) | snapshot:<id> (the base at that capture) | place:<name>")
    x: float = Field(0.0, description="m, forward in `frame`.")
    y: float = Field(0.0, description="m, left in `frame`.")
    theta: float = Field(0.0, description="rad, heading in `frame`, counter-clockwise positive.")


class ArmPose(BaseModel):
    """A TCP pose in the arm frame, as the arm node's POST /ik takes it."""
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    pos: list[float] = Field(min_length=3, max_length=3, description="m, arm frame (the arm's base_link; axes as the base).")
    rpy: Optional[list[float]] = Field(None, min_length=3, max_length=3, description="[roll, pitch, yaw] rad, fixed X-Y-Z.")
    quat: Optional[list[float]] = Field(None, min_length=4, max_length=4, description="[x, y, z, w] (instead of rpy).")


class ArmAt(BaseModel):
    """An arm configuration to imagine (nothing moves): a TCP pose in the arm frame (IK, as the arm node's POST
    /ik), or joint angles; and the gripper gap."""
    model_config = ConfigDict(extra="forbid", json_schema_extra={"examples": [
        {"pose": {"pos": [0.3, 0.0, 0.45], "rpy": [0, 0, 0]}, "gap_mm": 40}]})
    pose: Optional[ArmPose] = Field(None, description="TCP pose in the arm frame (the arm node's GET /geometry says where "
                                                      "its TCP and fingertips are).")
    q: Optional[list[float]] = Field(None, min_length=6, max_length=6, description="Joint angles, rad.")
    gap_mm: Optional[float] = Field(None, ge=0, le=60, description="Gripper gap (default: as measured).")
    max_rot_deg: Optional[float] = Field(None, ge=0, le=30, description="As for the arm node's POST /ik.")


class CheckIn(PoseIn):
    snapshot: Optional[str] = Field(None, description="Check against this snapshot (default: the latest).")
    also_use: Optional[list[str]] = Field(
        None, description="Other snapshots of this session to combine with `snapshot` (coverage and obstacles), in the "
                          "same world frame, e.g. to cover the blind zone from an earlier position. Default: none, only "
                          "`snapshot`. An older view may show the arm or things where they no longer are.")


class PlanCheckIn(CheckIn):
    arm: Optional[ArmAt] = Field(None, description="Imagine the arm at this configuration (default: as it is now).")
    view: bool = Field(True, description="Draw the whole robot (base and arm) at the goal into the snapshot (`robot`).")


class GoalIn(CheckIn):
    mode: Optional[Literal["synchronized", "independent"]] = Field(
        None, description="synchronized (default): translation and rotation start and finish together; independent: "
                          "each runs at its own pace.")
    pos_tol: Optional[float] = Field(None, gt=0, description="m: the goal is reached within this distance (default 0.02).")
    heading_tol: Optional[float] = Field(None, gt=0, description="rad: and within this heading (default 0.035).")
    on_reach: Optional[Literal["terminate", "hold"]] = Field(
        None, description="terminate (default): the goal ends when reached; hold: it keeps correcting until replaced or "
                          "stopped.")
    hold_band: Optional[float] = Field(None, gt=0, description="While holding: an axis inside its tolerance moves again "
                                                               "beyond hold_band x tolerance (default 2).")
    speed: Optional[float] = Field(None, gt=0, description="m/s, at most this goal (<= the cap).")
    omega: Optional[float] = Field(None, gt=0, description="rad/s, turn rate (<= the cap).")
    accel: Optional[float] = Field(None, gt=0, description="m/s² (<= the cap).")
    alpha: Optional[float] = Field(None, gt=0, description="rad/s², angular acceleration (<= the cap).")
    jerk: Optional[float] = Field(None, gt=0, description="m/s³ (<= the cap).")
    ang_jerk: Optional[float] = Field(None, gt=0, description="rad/s³ (<= the cap).")
    min_wheel_speed: Optional[float] = Field(None, ge=0, description="m/s: while the base doesn't move, the command is "
                                                                     "raised up to this wheel speed to overcome static "
                                                                     "friction (default 0.08; 0 = off).")
    stall_s: Optional[float] = Field(None, ge=0, description="s without progress before the goal ends as `stalled` "
                                                             "(default 3; 0 = never).")
    reflex_margin_m: Optional[float] = Field(
        None, gt=0, le=1.0, allow_inf_nan=False,
        description="The gap (m) the stop reflex leaves in front of what it sees, for this goal (default 0.12). "
                    "Braking and latency are added as always; a small margin lets the robot end close to a surface "
                    "(e.g. flush against a wall) and still stop before touching it.")
    model_config = ConfigDict(extra="forbid", json_schema_extra={"examples": [
        {"x": 0.5, "comment": "half a metre forward, toward the gap"},
        {"frame": "snapshot:s3", "x": 1.4, "y": 0.2, "theta": -0.5}]})


class LineIn(Commented):
    snapshot: Optional[str] = Field(None, description="Snapshot id (default: the latest).")
    select: Optional[Shape] = Field(None, description="A selection along the line (a thin polygon on an edge), or ...")
    points: Optional[list[list[float]]] = Field(None, description="... >= 2 colour pixels [[u, v], ...] on the line "
                                                "(each read as the median depth of a 4 px disk). Preferred for edges.")
    exclude: list[Shape] = Field(default_factory=list, description="Shapes removed from the selection.")
    model_config = ConfigDict(extra="forbid", json_schema_extra={"examples": [
        {"points": [[425, 240], [475, 272], [540, 312], [600, 348]], "comment": "table edge beside the robot"}]})


class ApproachIn(Commented):
    query: str = Field(..., description="A plane (or region) query id.")
    standoff: float = Field(1.0, gt=0, le=5, description="m from the surface (plane) / point (region).")
    face: bool = Field(True, description="Turn the proposed goal to face the surface (plane) / point (region).")


class StopIn(Commented):
    reason: str = Field("", description="Why (shown to the human with the latched e-stop).")


class NoteIn(Commented):
    text: str = Field(..., description="The note, kept with this session's memory (`GET /memory`).")


class PlaceIn(PoseIn):
    name: str = Field(..., description="The place's name (use it later as frame `place:<name>`).")
    theta: Optional[float] = Field(None, description="rad, heading to remember (null: the place has no heading).")


class ChatIn(BaseModel):
    text: str = Field(..., description="One command line, as typed on the page (`help` lists them).")


def create_app(node: AgentNav, start: bool = True) -> FastAPI:
    hub = RowHub({"turns": 1})
    node.on_change = hub.state_changed

    @asynccontextmanager
    async def lifespan(app):
        hub.attach(asyncio.get_running_loop())
        if start:
            node.start()
        yield
        node.close()

    app = FastAPI(
        title="agent_nav",
        description=(
            "Navigation for agents. **Read `GET /guide.md` first.**\n\n"
            "Perception and planning calls are read-only and return data, proposals and overlay images. "
            "Only `POST /goal` (and stop / e-stop) moves the base, and only when you call it. Start goals and "
            "wait for them asynchronously (`GET /goals/{id}/wait`). Every call is logged as a dialogue turn; "
            "add a `comment` to explain yourself."),
        version=__version__, lifespan=lifespan, openapi_tags=TAGS)
    setup_app(app)

    @app.exception_handler(ValueError)
    async def _value_error(request: Request, exc: ValueError):
        return JSONResponse(status_code=422, content={"detail": str(exc)})

    add_base_routes(app, lambda: {"node": "agent_nav", "session": (node.session or {}).get("agent")})
    add_page_routes(app, node_meta("agent_nav", "Agent navigation", __version__, "agent_nav", actuated=True,
                                   guide="guide.md"))

    def actor(op: Optional[str]) -> str:
        return "human" if (op or "").lower() == "human" else "agent"

    async def run(fn, *a, **k):
        """Run a node call in a worker thread; a refused call is logged as a dialogue turn too."""
        try:
            return await asyncio.to_thread(fn, *a, **k)
        except (RequestError, ValueError) as e:
            who = a[0] if a and a[0] in ("agent", "human") else "agent"
            req = {"args": [x for x in a[1:] if isinstance(x, (str, int, float, bool, dict, list, type(None)))]}
            node.turn(who, fn.__name__, req, {"error": str(e), "status": getattr(e, "status", 422)},
                      k.get("comment", ""), level="warn")
            raise

    # -- guide / page -------------------------------------------------------------------------------

    @app.get("/guide.md", tags=["guide"], response_class=Response,
             summary="How to drive with agent_nav (read first)",
             responses={200: {"content": {"text/markdown": {}}, "description": "The guide (Markdown)."}})
    async def guide():
        return PlainTextResponse(GUIDE, media_type="text/markdown; charset=utf-8")

    @app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def index():
        from .page import PAGE
        return PAGE

    # -- state / log --------------------------------------------------------------------------------

    @app.get("/state", response_model=M.NavState, tags=["session"], summary="Session, ownership, profile, pose, pose_servo, reflex, memory")
    async def get_state():
        return node.state()

    @app.get("/events", tags=["log"], response_class=StreamingResponse,
             summary="SSE: a small state on every new turn",
             responses={200: {"content": {"text/event-stream": {}}, "description": M.EVENTS_DOC}})
    async def events():
        def small():
            return {"turns": node.n_turn, "session": node.session, "enabled_by": node.enabled_by, "latch": node.latch,
                    "pose_world": (node.pose_now() or [None] * 3)[:3],
                    "servo": (node._ps_status or {}).get("status"), "reflex": node.reflex.get("blind"),
                    "latest": (node._ps_status or {}).get("latest"), **node.safety()}
        return StreamingResponse(sse(hub, small), media_type="text/event-stream", headers={"Cache-Control": "no-store"})

    @app.get("/log", response_model=M.LogOut, response_model_exclude_unset=True, tags=["log"], summary="Dialogue turns after `since` (live session, in memory)")
    async def get_log(since: int = 0, limit: int = Query(200, le=1000)):
        turns = [t for t in list(node.turns) if t["n"] > since][-limit:]
        return {"turns": turns, "last": node.n_turn, "session": node.session}

    @app.get("/sessions", response_model=list[M.StoredSession], response_model_exclude_unset=True, tags=["log"], summary="Stored session transcripts (newest first)")
    async def sessions():
        return await run(node.list_sessions)

    @app.get("/sessions/{sid}", response_model=M.StoredSessionOut, response_model_exclude_unset=True, tags=["log"], summary="A stored session: metadata + all its turns")
    async def session_read(sid: str):
        return await run(node.read_session, sid)

    @app.get("/sessions/{sid}/export.zip", tags=["log"], response_class=Response,
             summary="Download a session (log.jsonl, thumbnails, metadata)",
             responses={200: {"content": {"application/zip": {}},
                              "description": "agent_nav_<id>/: session.json, log.jsonl, thumbs/*.jpg."}})
    async def session_export(sid: str):
        data = await run(node.export_session, sid)
        return Response(data, media_type="application/zip",
                        headers={"Content-Disposition": f'attachment; filename="agent_nav_{sid}.zip"'})

    @app.delete("/sessions/{sid}", response_model=M.Deleted, tags=["log"], summary="(human) Delete a stored session (not the live one)")
    async def session_delete(sid: str, x_operator: Optional[str] = Operator):
        if actor(x_operator) != "human":
            raise RequestError(403, "only the human operator deletes sessions")
        return await run(node.delete_session, sid)

    @app.get("/sessions/{sid}/log.jsonl", tags=["log"], response_class=FileResponse,
             summary="A session's transcript (JSON lines)",
             responses={200: {"content": {"application/x-ndjson": {}},
                              "description": "One turn per line (the Turn of GET /log)."}})
    async def session_log(sid: str):
        p = _safe_path(node.data_dir, sid, "log.jsonl")
        return FileResponse(p, media_type="application/x-ndjson")

    @app.get("/sessions/{sid}/thumbs/{fn}", tags=["log"], response_class=FileResponse,
             summary="A stored thumbnail",
             responses={200: {"content": {"image/jpeg": {}}, "description": "JPEG, 320 px wide."}})
    async def session_thumb(sid: str, fn: str):
        return FileResponse(_safe_path(node.data_dir, sid, "thumbs", fn), media_type="image/jpeg")

    @app.get("/images/{name}", tags=["perception"], response_class=Response,
             summary="A rendered image (snapshot, overlay, BEV)",
             responses={200: {"content": {"image/jpeg": {}, "image/png": {}},
                              "description": "JPEG (snapshots, overlays) or PNG (BEV, diagrams), by the name's extension."}})
    async def image(name: str):
        img = node.images.get(name)
        if img is None:
            raise RequestError(404, f"image {name!r} unknown or expired (the last few hundred are kept)")
        return Response(img[0], media_type=img[1], headers={"Cache-Control": "no-store"})

    @app.get("/images/{sid}/grid.jpg", tags=["perception"], response_class=Response,
             summary="A snapshot with a labelled pixel grid",
             responses={200: {"content": {"image/jpeg": {}}}})
    async def grid(sid: str):
        return Response(await run(node.grid_image, sid), media_type="image/jpeg")

    # -- session --------------------------------------------------------------------------------------

    @app.post("/session", response_model=M.OpenSessionOut, response_model_exclude_unset=True, tags=["session"], summary="Open a session: your name and avatar")
    async def open_session(body: SessionIn, x_operator: Optional[str] = Operator):
        return await run(node.open_session, body.agent, body.avatar, body.comment, actor(x_operator))

    @app.delete("/session", response_model=M.Closed, tags=["session"], summary="Close the session (disables)")
    async def close_session(x_operator: Optional[str] = Operator):
        return await run(node.close_session, "closed", actor(x_operator))

    @app.post("/enable", response_model=M.EnableOut, response_model_exclude_unset=True, tags=["session"], summary="Enable motion (pose_servo + swerve lease) with the profile's caps")
    async def enable(body: EnableIn = EnableIn(), x_operator: Optional[str] = Operator):
        """423 while the human operator holds it (disabled or e-stopped on the page)."""
        return await run(node.enable, actor(x_operator), None if body.caps is None else body.caps.model_dump(exclude_none=True),
                         body.comment)

    @app.post("/disable", response_model=M.DisableOut, tags=["session"], summary="Disable at once (no braking); releases the swerve")
    async def disable(body: Commented = Commented(), x_operator: Optional[str] = Operator):
        return await run(node.disable, actor(x_operator), body.comment)

    @app.post("/release", response_model=M.Released, tags=["session"], summary="(human) Release the hold so the agent may enable again")
    async def release(body: Commented = Commented(), x_operator: Optional[str] = Operator):
        if actor(x_operator) != "human":
            raise RequestError(403, "only the human operator releases the hold")
        return await run(node.release, body.comment)

    @app.post("/profile/{name}", response_model=M.ProfileOut, tags=["session"], summary="(human) Choose the safety profile (applies at the next enable)")
    async def profile(name: str, x_operator: Optional[str] = Operator):
        if actor(x_operator) != "human":
            raise RequestError(403, "only the human operator chooses the profile")
        return await run(node.set_profile, name)

    @app.post("/reflex", response_model=M.ReflexOut, tags=["session"], summary="(human) Turn the stop reflex on / off")
    async def reflex(enabled: bool, x_operator: Optional[str] = Operator):
        if actor(x_operator) != "human":
            raise RequestError(403, "only the human operator changes the reflex")
        node.reflex["enabled"] = enabled
        node.turn("human", "reflex", {"enabled": enabled}, {"reflex": enabled})
        return {"reflex": enabled}

    # -- robot ------------------------------------------------------------------------------------------

    @app.get("/robot", response_model=M.RobotOut, response_model_exclude_unset=True, tags=["perception"], summary="The robot: body, camera pose / view / blind zone, frames, limits, diagram")
    async def robot(comment: str = "", x_operator: Optional[str] = Operator):
        """Read-only. `diagram` is a side + top view of the base and what the camera sees."""
        return await run(node.describe_robot, actor(x_operator), comment)

    # -- perception -------------------------------------------------------------------------------------

    @app.post("/look", response_model=M.LookOut, response_model_exclude_unset=True, tags=["perception"], summary="Capture a snapshot (colour + registered depth + pose)")
    async def look(body: LookIn = LookIn(), x_operator: Optional[str] = Operator):
        grid = body.grid.model_dump() if isinstance(body.grid, GridIn) else body.grid
        return await run(node.look, actor(x_operator), body.scale, grid, body.settle, body.comment)

    GEO_DOC = ("Placements of the camera and the arm mount on the robot, as described and as this session corrects them. "
               "Corrections are for display and perception only (look, zoom values, queries, the robot drawn by "
               "plan/check), last for the session, and are logged; the stop reflex, goals and arm commands never use "
               "them. Use them when what you see disagrees with where the robot is drawn (e.g. plan/check with no "
               "move draws the arm where it is now: its drawn fingertips should sit on the real ones).")

    @app.get("/session/geometry", response_model=M.SessionGeometryOut, response_model_exclude_unset=True,
             tags=["perception"], summary="Camera and arm-mount placements: described, corrected, effective", description=GEO_DOC)
    async def session_geometry():
        return await run(node.session_geometry)

    @app.post("/session/geometry", response_model=M.SessionGeometryOut, response_model_exclude_unset=True,
              tags=["perception"], summary="Correct the camera or arm-mount placement for this session (display and "
                                           "perception only)", description=GEO_DOC)
    async def set_session_geometry(body: SessionGeometryIn, x_operator: Optional[str] = Operator):
        return await run(node.set_session_geometry, actor(x_operator),
                         None if body.camera is None else body.camera.model_dump(),
                         None if body.arm is None else body.arm.model_dump(), body.comment)

    @app.post("/zoom", response_model=M.ZoomOut, response_model_exclude_unset=True, tags=["perception"],
              summary="Enlarge part of a snapshot, with a grid in full-image pixels and optional depth values (no capture)")
    async def zoom(body: ZoomIn, x_operator: Optional[str] = Operator):
        return await run(node.zoom, actor(x_operator), body.snapshot, body.box, body.step, body.values, body.comment)

    @app.post("/query/region", response_model=M.RegionOut, response_model_exclude_unset=True, tags=["perception"], summary="3D extent of a selection (base / world frames)")
    async def q_region(body: QueryIn, x_operator: Optional[str] = Operator):
        return await run(node.query, actor(x_operator), "region", body.snapshot, shape_dict(body.select), [shape_dict(e) for e in body.exclude], body.comment)

    @app.post("/query/plane", response_model=M.PlaneOut, response_model_exclude_unset=True, tags=["perception"], summary="Fit a plane to a selection (centre, extent, normal)")
    async def q_plane(body: QueryIn, x_operator: Optional[str] = Operator):
        return await run(node.query, actor(x_operator), "plane", body.snapshot, shape_dict(body.select), [shape_dict(e) for e in body.exclude], body.comment)

    @app.post("/query/line", response_model=M.LineOut, response_model_exclude_unset=True, tags=["perception"], summary="Fit a 3D line (edge, row of legs): direction, heading, length")
    async def q_line(body: LineIn, x_operator: Optional[str] = Operator):
        """Use `points` on the edge itself (preferred) or a thin `select`ion. Gives the heading to align with."""
        return await run(node.query, actor(x_operator), "line", body.snapshot, shape_dict(body.select), [shape_dict(e) for e in body.exclude], body.comment,
                         points=body.points)

    # -- planning -----------------------------------------------------------------------------------------

    @app.post("/plan/check", response_model=M.PlanCheckOut, response_model_exclude_unset=True, tags=["planning"], summary="Check a straight move against a snapshot (loud, advisory)")
    async def plan_check(body: PlanCheckIn, x_operator: Optional[str] = Operator):
        return await run(node.plan_check, actor(x_operator), body.frame, body.x, body.y, body.theta, body.snapshot, body.comment,
                         None if body.arm is None else body.arm.model_dump(exclude_none=True), body.view, body.also_use)

    @app.post("/plan/approach", response_model=M.ApproachOut, response_model_exclude_unset=True, tags=["planning"], summary="Propose a goal in front of a queried surface (moves nothing)")
    async def plan_approach(body: ApproachIn, x_operator: Optional[str] = Operator):
        return await run(node.plan_approach, actor(x_operator), body.query, body.standoff, body.face, body.comment)

    # -- motion -----------------------------------------------------------------------------------------------

    @app.post("/goal", response_model=M.GoalOut, response_model_exclude_unset=True, tags=["motion"], summary="Move: start (or replace) a goal; returns at once")
    async def goal(body: GoalIn, x_operator: Optional[str] = Operator):
        """The path is checked against the latest (or given) snapshot and the result reported loudly in
        `warnings`; the goal is executed as issued. Wait with `GET /goals/{id}/wait` in the background."""
        opts = body.model_dump(exclude={"frame", "x", "y", "theta", "snapshot", "comment", "also_use"})
        return await run(node.goal, actor(x_operator), body.frame, body.x, body.y, body.theta, opts, body.snapshot, body.comment,
                         body.also_use)

    @app.get("/goals/{gid}", response_model=M.GoalState, response_model_exclude_unset=True, tags=["motion"], summary="A goal's state")
    async def goal_state(gid: int):
        return node.goal_state(gid)

    @app.get("/goals/{gid}/wait", response_model=M.GoalState, response_model_exclude_unset=True, tags=["motion"], summary="Long-poll until the goal ends (or timeout)")
    async def goal_wait(gid: int, timeout: float = Query(60.0, gt=0, le=600)):
        node.goal_state(gid)
        rec = node.goals[gid]
        end = time.monotonic() + timeout
        while not rec.done.is_set() and time.monotonic() < end:
            await asyncio.sleep(0.1)
        return node.goal_state(gid)

    @app.post("/stop", response_model=M.ServoStop, tags=["motion"], summary="Brake to a stop (stays enabled)")
    async def stop(body: Commented = Commented(), x_operator: Optional[str] = Operator):
        return await run(node.stop, actor(x_operator), body.comment)

    @app.post("/estop", response_model=M.ServoEStop, response_model_exclude_unset=True, tags=["motion"], summary="E-stop (anyone): forwarded to the swerve; latches")
    async def estop(body: StopIn = StopIn(), x_operator: Optional[str] = Operator):
        return await run(node.estop, actor(x_operator), body.reason, body.comment)

    # -- memory -----------------------------------------------------------------------------------------------

    @app.get("/memory", response_model=M.Memory, tags=["memory"], summary="This session's notes and places")
    async def memory():
        return node.memory

    @app.post("/memory/note", response_model=M.NoteOut, tags=["memory"], summary="Add a note")
    async def note(body: NoteIn, x_operator: Optional[str] = Operator):
        return await run(node.note, actor(x_operator), body.text, body.comment)

    @app.post("/memory/place", response_model=M.PlaceOut, tags=["memory"], summary="Name a pose (default: here)")
    async def place(body: PlaceIn, x_operator: Optional[str] = Operator):
        return await run(node.place, actor(x_operator), body.name, body.frame, body.x, body.y, body.theta, body.comment)

    # -- chat (humans) -------------------------------------------------------------------------------------------

    @app.post("/chat", response_model=M.ChatOut, response_model_exclude_unset=True, tags=["log"], summary="(human) One command line, e.g. 'look grid' or 'goal 0.5 0 90d'")
    async def chat(body: ChatIn):
        """Replies as the endpoint the command maps to (look: POST /look, plane: POST /query/plane, goal:
        POST /goal, ...), or `{help}` for `help`."""
        text = body.text.strip()
        try:
            method, kw = chatmod.parse(text)
        except (ValueError, IndexError) as e:
            node.turn("human", "chat", {"text": text}, {"error": str(e) or "bad arguments", "help": chatmod.HELP}, level="warn")
            raise RequestError(422, str(e) or "bad arguments")
        if method == "help":
            node.turn("human", "chat", {"text": text}, {"help": chatmod.HELP})
            return {"help": chatmod.HELP}
        fn = getattr(node, method)
        args = dict(kw)
        if method == "release":
            return await run(fn, comment=f"chat: {text}")
        return await run(fn, "human", comment=f"chat: {text}", **args)

    return app


def _safe_path(root: str, *parts: str) -> str:
    p = os.path.realpath(os.path.join(root, *parts))
    if not p.startswith(os.path.realpath(root) + os.sep) or not os.path.exists(p):
        raise RequestError(404, "not found")
    return p


try:
    GUIDE = resources.files("robot_utils.nodes.agent_nav").joinpath("guide.md").read_text()
except Exception:   # source checkout without package data
    GUIDE = "# agent_nav\n\nguide.md missing from the package"
