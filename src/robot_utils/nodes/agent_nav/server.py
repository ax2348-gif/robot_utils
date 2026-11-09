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
from pydantic import BaseModel, ConfigDict, Field

from ..common.hub import RowHub
from ..common.lease import RequestError
from ..common.server import add_base_routes, add_page_routes, node_meta, setup_app, sse
from . import __version__
from . import chat as chatmod
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

Shape = dict  # {"point":[u,v]} | {"disk":[u,v,r]} | {"box":[u0,v0,u1,v1]} | {"polygon":[[u,v],...]}


class Commented(BaseModel):
    comment: str = Field("", description="Optional note for the transcript (why you are doing this).")
    model_config = ConfigDict(extra="forbid")


class SessionIn(Commented):
    agent: str = Field(..., description="Your name, shown in the dialogue.")
    avatar: str = Field("", description="An emoji / short text (<= 16 chars) or a data:image/... URI (<= 64 KB).")
    model_config = ConfigDict(extra="forbid", json_schema_extra={"examples": [{"agent": "Claude", "avatar": "🦉"}]})


class EnableIn(Commented):
    caps: dict = Field(default_factory=dict, description="Lower any cap of the profile (max_speed, max_omega, "
                       "max_accel, max_alpha, max_jerk, max_ang_jerk, max_distance, max_rotation).")


class LookIn(Commented):
    scale: Optional[float] = Field(None, description="Depth grid = colour (1280x720) x scale; default the node's "
                                   "(0.5 = 640x360); 1.0 = full resolution (slower, briefly applies to every viewer).")
    grid: bool = Field(False, description="Draw a labelled pixel grid on the returned image.")
    settle: bool = Field(False, description="Wait (<= 8 s) until the base is not moving before capturing.")


class QueryIn(Commented):
    snapshot: Optional[str] = Field(None, description="Snapshot id (default: the latest).")
    select: Shape = Field(..., description='{"point":[u,v]} | {"disk":[u,v,r]} | {"box":[u0,v0,u1,v1]} | '
                          '{"polygon":[[u,v],...]} in colour-image pixels.')
    exclude: list[Shape] = Field(default_factory=list, description="Shapes removed from the selection.")
    model_config = ConfigDict(extra="forbid", json_schema_extra={"examples": [
        {"select": {"box": [400, 20, 700, 160]}},
        {"select": {"polygon": [[380, 10], [700, 10], [690, 150], [390, 150]]}, "exclude": [{"disk": [520, 60, 25]}]}]})


class PoseIn(Commented):
    frame: str = Field("local", description="local (default: x forward, y left; theta literal) | global (theta: "
                       "shortest turn) | snapshot:<id> (the base at that capture) | place:<name>")
    x: float = 0.0
    y: float = 0.0
    theta: float = Field(0.0, description="rad, CCW +")


class CheckIn(PoseIn):
    snapshot: Optional[str] = Field(None, description="Check against this snapshot (default: the latest).")


class GoalIn(CheckIn):
    mode: Optional[Literal["synchronized", "independent"]] = None
    pos_tol: Optional[float] = None
    heading_tol: Optional[float] = None
    on_reach: Optional[Literal["terminate", "hold"]] = None
    hold_band: Optional[float] = None
    speed: Optional[float] = None
    omega: Optional[float] = None
    accel: Optional[float] = None
    alpha: Optional[float] = None
    jerk: Optional[float] = None
    ang_jerk: Optional[float] = None
    min_wheel_speed: Optional[float] = None
    stall_s: Optional[float] = None
    model_config = ConfigDict(extra="forbid", json_schema_extra={"examples": [
        {"x": 0.5, "comment": "half a metre forward, toward the gap"},
        {"frame": "snapshot:s3", "x": 1.4, "y": 0.2, "theta": -0.5}]})


class ApproachIn(Commented):
    query: str = Field(..., description="A plane (or region) query id.")
    standoff: float = Field(1.0, gt=0, le=5, description="m from the surface (plane) / point (region).")
    face: bool = True


class StopIn(Commented):
    reason: str = ""


class NoteIn(Commented):
    text: str


class PlaceIn(PoseIn):
    name: str
    theta: Optional[float] = None


class ChatIn(BaseModel):
    text: str


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

    @app.get("/guide.md", tags=["guide"], response_class=PlainTextResponse, summary="How to drive with agent_nav (read first)")
    async def guide():
        return PlainTextResponse(GUIDE, media_type="text/markdown; charset=utf-8")

    @app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def index():
        from .page import PAGE
        return PAGE

    # -- state / log --------------------------------------------------------------------------------

    @app.get("/state", tags=["session"], summary="Session, ownership, profile, pose, pose_servo, reflex, memory")
    async def get_state():
        return node.state()

    @app.get("/events", tags=["log"], response_class=StreamingResponse, summary="SSE: a small state on every new turn")
    async def events():
        def small():
            return {"turns": node.n_turn, "session": node.session, "enabled_by": node.enabled_by, "latch": node.latch,
                    "pose_world": (node.pose_now() or [None] * 3)[:3],
                    "servo": (node._ps_status or {}).get("status"), "reflex": node.reflex.get("blind"),
                    "latest": (node._ps_status or {}).get("latest")}
        return StreamingResponse(sse(hub, small), media_type="text/event-stream", headers={"Cache-Control": "no-store"})

    @app.get("/log", tags=["log"], summary="Dialogue turns after `since` (live session, in memory)")
    async def get_log(since: int = 0, limit: int = Query(200, le=1000)):
        turns = [t for t in list(node.turns) if t["n"] > since][-limit:]
        return {"turns": turns, "last": node.n_turn, "session": node.session}

    @app.get("/sessions", tags=["log"], summary="Stored session transcripts (newest first)")
    async def sessions():
        return await run(node.list_sessions)

    @app.get("/sessions/{sid}", tags=["log"], summary="A stored session: metadata + all its turns")
    async def session_read(sid: str):
        return await run(node.read_session, sid)

    @app.get("/sessions/{sid}/export.zip", tags=["log"], summary="Download a session (log.jsonl, thumbnails, metadata)")
    async def session_export(sid: str):
        data = await run(node.export_session, sid)
        return Response(data, media_type="application/zip",
                        headers={"Content-Disposition": f'attachment; filename="agent_nav_{sid}.zip"'})

    @app.delete("/sessions/{sid}", tags=["log"], summary="(human) Delete a stored session (not the live one)")
    async def session_delete(sid: str, x_operator: Optional[str] = Operator):
        if actor(x_operator) != "human":
            raise RequestError(403, "only the human operator deletes sessions")
        return await run(node.delete_session, sid)

    @app.get("/sessions/{sid}/log.jsonl", tags=["log"], summary="A session's transcript (JSON lines)")
    async def session_log(sid: str):
        p = _safe_path(node.data_dir, sid, "log.jsonl")
        return FileResponse(p, media_type="application/x-ndjson")

    @app.get("/sessions/{sid}/thumbs/{fn}", tags=["log"], summary="A stored thumbnail")
    async def session_thumb(sid: str, fn: str):
        return FileResponse(_safe_path(node.data_dir, sid, "thumbs", fn), media_type="image/jpeg")

    @app.get("/images/{name}", tags=["perception"], summary="A rendered image (snapshot, overlay, BEV)")
    async def image(name: str):
        img = node.images.get(name)
        if img is None:
            raise RequestError(404, f"image {name!r} unknown or expired (the last few hundred are kept)")
        return Response(img[0], media_type=img[1], headers={"Cache-Control": "no-store"})

    @app.get("/images/{sid}/grid.jpg", tags=["perception"], summary="A snapshot with a labelled pixel grid")
    async def grid(sid: str):
        return Response(await run(node.grid_image, sid), media_type="image/jpeg")

    # -- session --------------------------------------------------------------------------------------

    @app.post("/session", tags=["session"], summary="Open a session: your name and avatar")
    async def open_session(body: SessionIn, x_operator: Optional[str] = Operator):
        return await run(node.open_session, body.agent, body.avatar, body.comment, actor(x_operator))

    @app.delete("/session", tags=["session"], summary="Close the session (disables)")
    async def close_session(x_operator: Optional[str] = Operator):
        return await run(node.close_session, "closed", actor(x_operator))

    @app.post("/enable", tags=["session"], summary="Enable motion (pose_servo + swerve lease) with the profile's caps")
    async def enable(body: EnableIn = EnableIn(), x_operator: Optional[str] = Operator):
        """423 while the human operator holds it (disabled or e-stopped on the page)."""
        return await run(node.enable, actor(x_operator), body.caps, body.comment)

    @app.post("/disable", tags=["session"], summary="Disable at once (no braking); releases the swerve")
    async def disable(body: Commented = Commented(), x_operator: Optional[str] = Operator):
        return await run(node.disable, actor(x_operator), body.comment)

    @app.post("/release", tags=["session"], summary="(human) Release the hold so the agent may enable again")
    async def release(body: Commented = Commented(), x_operator: Optional[str] = Operator):
        if actor(x_operator) != "human":
            raise RequestError(403, "only the human operator releases the hold")
        return await run(node.release, body.comment)

    @app.post("/profile/{name}", tags=["session"], summary="(human) Choose the safety profile (applies at the next enable)")
    async def profile(name: str, x_operator: Optional[str] = Operator):
        if actor(x_operator) != "human":
            raise RequestError(403, "only the human operator chooses the profile")
        return await run(node.set_profile, name)

    @app.post("/reflex", tags=["session"], summary="(human) Turn the stop reflex on / off")
    async def reflex(enabled: bool, x_operator: Optional[str] = Operator):
        if actor(x_operator) != "human":
            raise RequestError(403, "only the human operator changes the reflex")
        node.reflex["enabled"] = enabled
        node.turn("human", "reflex", {"enabled": enabled}, {"reflex": enabled})
        return {"reflex": enabled}

    # -- perception -------------------------------------------------------------------------------------

    @app.post("/look", tags=["perception"], summary="Capture a snapshot (colour + registered depth + pose)")
    async def look(body: LookIn = LookIn(), x_operator: Optional[str] = Operator):
        return await run(node.look, actor(x_operator), body.scale, body.grid, body.settle, body.comment)

    @app.post("/query/region", tags=["perception"], summary="3D extent of a selection (base / world frames)")
    async def q_region(body: QueryIn, x_operator: Optional[str] = Operator):
        return await run(node.query, actor(x_operator), "region", body.snapshot, body.select, body.exclude, body.comment)

    @app.post("/query/plane", tags=["perception"], summary="Fit a plane to a selection (centre, extent, normal)")
    async def q_plane(body: QueryIn, x_operator: Optional[str] = Operator):
        return await run(node.query, actor(x_operator), "plane", body.snapshot, body.select, body.exclude, body.comment)

    # -- planning -----------------------------------------------------------------------------------------

    @app.post("/plan/check", tags=["planning"], summary="Check a straight move against a snapshot (loud, advisory)")
    async def plan_check(body: CheckIn, x_operator: Optional[str] = Operator):
        return await run(node.plan_check, actor(x_operator), body.frame, body.x, body.y, body.theta, body.snapshot, body.comment)

    @app.post("/plan/approach", tags=["planning"], summary="Propose a goal in front of a queried surface (moves nothing)")
    async def plan_approach(body: ApproachIn, x_operator: Optional[str] = Operator):
        return await run(node.plan_approach, actor(x_operator), body.query, body.standoff, body.face, body.comment)

    # -- motion -----------------------------------------------------------------------------------------------

    @app.post("/goal", tags=["motion"], summary="Move: start (or replace) a goal; returns at once")
    async def goal(body: GoalIn, x_operator: Optional[str] = Operator):
        """The path is checked against the latest (or given) snapshot and the result reported loudly in
        `warnings`; the goal is executed as issued. Wait with `GET /goals/{id}/wait` in the background."""
        opts = body.model_dump(exclude={"frame", "x", "y", "theta", "snapshot", "comment"})
        return await run(node.goal, actor(x_operator), body.frame, body.x, body.y, body.theta, opts, body.snapshot, body.comment)

    @app.get("/goals/{gid}", tags=["motion"], summary="A goal's state")
    async def goal_state(gid: int):
        return node.goal_state(gid)

    @app.get("/goals/{gid}/wait", tags=["motion"], summary="Long-poll until the goal ends (or timeout)")
    async def goal_wait(gid: int, timeout: float = Query(60.0, gt=0, le=600)):
        node.goal_state(gid)
        rec = node.goals[gid]
        end = time.monotonic() + timeout
        while not rec.done.is_set() and time.monotonic() < end:
            await asyncio.sleep(0.1)
        return node.goal_state(gid)

    @app.post("/stop", tags=["motion"], summary="Brake to a stop (stays enabled)")
    async def stop(body: Commented = Commented(), x_operator: Optional[str] = Operator):
        return await run(node.stop, actor(x_operator), body.comment)

    @app.post("/estop", tags=["motion"], summary="E-stop (anyone): forwarded to the swerve; latches")
    async def estop(body: StopIn = StopIn(), x_operator: Optional[str] = Operator):
        return await run(node.estop, actor(x_operator), body.reason, body.comment)

    # -- memory -----------------------------------------------------------------------------------------------

    @app.get("/memory", tags=["memory"], summary="This session's notes and places")
    async def memory():
        return node.memory

    @app.post("/memory/note", tags=["memory"], summary="Add a note")
    async def note(body: NoteIn, x_operator: Optional[str] = Operator):
        return await run(node.note, actor(x_operator), body.text, body.comment)

    @app.post("/memory/place", tags=["memory"], summary="Name a pose (default: here)")
    async def place(body: PlaceIn, x_operator: Optional[str] = Operator):
        return await run(node.place, actor(x_operator), body.name, body.frame, body.x, body.y, body.theta, body.comment)

    # -- chat (humans) -------------------------------------------------------------------------------------------

    @app.post("/chat", tags=["log"], summary="(human) One command line, e.g. 'look grid' or 'goal 0.5 0 90d'")
    async def chat(body: ChatIn):
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
