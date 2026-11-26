"""HTTP API of the Quest teleop node (FastAPI). Conventions: docs/nodes.md; guide: docs/quest_teleop_node.md."""
from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from typing import Any, Optional

from fastapi import FastAPI, HTTPException, Query, Request
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from pydantic import BaseModel, Field

from ..common.server import add_base_routes, add_page_routes, node_meta, rows_stream, setup_app, sse
from . import __version__
from .node import FIELD_DOC, FIELDS, ROW_PERIOD, UNITS, QuestTeleopNode
from .oplog import MAX_RECORDS, oplog_stream

TAGS = [
    {"name": "state", "description": "Teleop client state, headset link, the WBC echo, headset commands, events."},
    {"name": "control", "description": "Start / stop teleop (the WBC lease), e-stop, the headset (quest up / down)."},
    {"name": "stream", "description": f"Rows every {ROW_PERIOD * 1e3:.0f} ms (binary float64); the operator stream."},
    {"name": "episodes", "description": "The task, the event log (docs/plans/data_collection_v1.md, the contract)."},
    {"name": "haptics", "description": "Controller vibration cues sent to the headset (docs/quest_teleop_node.md "
                                       "\"Haptics\")."},
]


class TaskIn(BaseModel):
    text: str = Field("", description="The task of the next episodes (in each episode_start's detail).")


class TopSpeedIn(BaseModel):
    max_speed: Optional[float] = Field(None, description="m/s: the left stick's full deflection, the hint clamp and "
                                                         "(while running) the WBC's max_speed; omitted: unchanged")
    max_omega: Optional[float] = Field(None, description="rad/s: likewise for yaw (the WBC's max_omega)")


class ResumeIn(BaseModel):
    enabled: Optional[bool] = Field(None, description="the tiered recovery on (a transient holds; a re-grip resumes) "
                                                       "or off (every disarm needs Y); omitted: unchanged")
    grace_s: Optional[float] = Field(None, description="s: a transient cause present longer than this escalates to a "
                                                       "disarm (needs Y)")
    hold_max_s: Optional[float] = Field(None, description="s: a hold not resumed within this escalates")
    classes: Optional[dict] = Field(None, description="{class: bool}: which transient causes hold (input_stale, "
                                                      "input_late, input_gap, input_invalid, left_inactive, wbc_stale, "
                                                      "wbc_disarm, no_command)")


class HapticsCueIn(BaseModel):
    enabled: Optional[bool] = None
    amplitude: Optional[float] = Field(None, ge=0.0, le=1.0)


class HapticsIn(BaseModel):
    enabled: Optional[bool] = Field(None, description="The master switch (every cue)")
    cues: dict[str, HapticsCueIn] = Field(default_factory=dict,
                                          description="Per cue (GET /haptics `config.cues`): on / off, amplitude 0..1")
    limit_kinds: Optional[list[str]] = Field(None, description="The WBC limits the limit cue buzzes for (default "
                                                                "reach, manipulability, joint_limit)")


class HapticsTestIn(BaseModel):
    cue: str = Field("armed", description="Any cue, played even when off (a level cue: a 1 s pulse at its amplitude)")
    hand: str = Field("both", pattern="^(left|right|both)$")


class EStopIn(BaseModel):
    reason: str = ""
    source: str = Field("", description="Who pressed it (free text).")


def create_app(node: QuestTeleopNode, start: bool = True) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app):
        node.hub.attach(asyncio.get_running_loop())
        node.oplog.attach(asyncio.get_running_loop())
        if start:
            node.start()
        yield
        await asyncio.to_thread(node.close)

    app = FastAPI(
        title="Quest teleop node",
        description=(
            "Whole-body teleop from the Quest headset through the local WBC node (the client of tools/quest_teleop, "
            "run on this machine; docs/quest_teleop_node.md).\n\n"
            "`POST /start` takes the WBC lease, sets the session hints, enables the WBC and opens a datagram "
            "session on its unix socket: nothing moves until Y (left controller) arms; X disarms and disables. "
            "Armed, every batch of headset frames becomes one target datagram. "
            "`POST /stop` disables the WBC and releases the lease. `POST /estop` e-stops the arm, the swerve and the "
            "WBC directly, in parallel, and stops teleop. `POST /quest/up` and `/quest/down` run quest_ctl's up / down "
            "on the headset over ssh, only when asked. Episodes: A (right controller) starts one, a B tap stops "
            "it (the sim stack: A resets the sim first); the node emits events (`GET /eventlog`, its only files) "
            "and the operator stream (`GET /oplog/stream`: headset frames, datagrams sent, events); recording is "
            "the client-side script's job (`python -m robot_utils.collect run`), which a sim stack's node runs "
            "itself from Start to Stop (`/state` `recording`)."),
        version=__version__, lifespan=lifespan, openapi_tags=TAGS)
    setup_app(app)

    add_base_routes(app, lambda: {"node": "quest_teleop", "state": node.client_state()})
    add_page_routes(app, node_meta(node.name, "Quest teleop", __version__, "quest_teleop", actuated=True,
                                   rate_hz=round(1 / ROW_PERIOD)))

    @app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def index():
        from .page import PAGE
        return PAGE

    @app.get("/state", tags=["state"], summary="Client state, headset link, the WBC echo, headset command output")
    async def get_state() -> Any:
        return node.state()

    @app.get("/events", tags=["state"], response_class=StreamingResponse,
             summary="Server-Sent Events: /state on every change and every 1 s")
    async def events():
        return StreamingResponse(sse(node.hub, node.state), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-store"})

    @app.get("/fields", tags=["stream"], summary="Columns of the rows")
    async def fields():
        return {"fields": FIELDS, "units": UNITS, "doc": FIELD_DOC}

    @app.get("/stream", tags=["stream"], response_class=StreamingResponse,
             summary="Rows (schema record first): headset link, hands, client state, the WBC echo, targets")
    async def stream(batch_ms: float = Query(0.0, ge=0, le=1000),
                     max_backlog_s: float = Query(20.0, gt=0, le=600)):
        gen = rows_stream(node.hub, FIELDS, UNITS, channel="tick", kind="tick", batch_ms=batch_ms,
                          max_rows=int(max_backlog_s / ROW_PERIOD),
                          schema_extra={"node": "quest_teleop", "rate_hz": 1 / ROW_PERIOD})
        return StreamingResponse(gen, media_type="application/octet-stream", headers={"Cache-Control": "no-store"})

    @app.get("/oplog/stream", tags=["stream"], response_class=StreamingResponse,
             summary="The operator stream: records `headset` {t_rx, frame}, `sent` {t_send, datagram}, `event` {the "
                     "event line, flat}, in order; a client further behind than `max_records` loses the oldest "
                     "(the next record's `dropped`)")
    async def oplog(max_records: int = Query(MAX_RECORDS, ge=10, le=100000)):
        return StreamingResponse(oplog_stream(node.oplog, max_records), media_type="application/octet-stream",
                                 headers={"Cache-Control": "no-store"})

    @app.post("/task", tags=["episodes"], summary="The task of the next episodes (in /state and each episode_start)")
    async def task(body: TaskIn):
        return node.set_task(body.text)

    @app.get("/eventlog", tags=["episodes"], response_class=StreamingResponse,
             summary="Event lines (JSON lines) after `since_seq`, from the log files, at most `limit`: repeat from "
                     "the last seq until fewer come back")
    async def eventlog(since_seq: int = Query(0, ge=0), limit: int = Query(1000, ge=1, le=100000)):
        lines = await asyncio.to_thread(lambda: list(node.eventlog.since(since_seq, limit)))
        return StreamingResponse(iter(lines), media_type="application/x-ndjson", headers={"Cache-Control": "no-store"})

    @app.get("/eventlog/files", tags=["episodes"], summary="The log files: [{name, bytes, first_seq, last_seq}]")
    async def eventlog_files():
        return await asyncio.to_thread(node.eventlog.files)

    @app.get("/haptics", tags=["haptics"], response_class=JSONResponse,
             summary="The haptics state (teleop.quest_haptics.v1). `stream=1`: the headset's stream of it, one JSON "
                     "line per change and a keepalive line every 0.2 s (busy) / 1 s (idle)",
             responses={200: {"content": {"application/json": {}, "application/x-ndjson": {}}}})
    async def haptics(stream: bool = Query(False, description="NDJSON lines until the client goes (the headset)")):
        h = node.haptics
        if stream:
            return StreamingResponse(h.stream_lines(), media_type="application/x-ndjson",
                                     headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"})
        return JSONResponse(h.message(), headers={"Cache-Control": "no-store"})

    @app.get("/haptics/info", tags=["haptics"],
             summary="The cues' config, the levels, the headset's ack and round trip, the stream readers, counters")
    async def haptics_info():
        return node.haptics.info()

    @app.post("/haptics/config", tags=["haptics"], summary="The master switch, cues on / off and their amplitudes, "
                                                            "the limit kinds")
    async def haptics_config(body: HapticsIn):
        try:
            return node.haptics.configure(body.enabled, {k: v.model_dump(exclude_none=True) for k, v in body.cues.items()},
                                          body.limit_kinds)
        except ValueError as e:
            raise HTTPException(422, str(e))

    @app.post("/haptics/test", tags=["haptics"], summary="Play one cue now on a hand or both (even when off)")
    async def haptics_test(body: HapticsTestIn):
        from .haptics import CUES, HANDS
        if body.cue not in CUES:
            raise HTTPException(422, f"unknown cue {body.cue!r} (cues: {', '.join(CUES)})")
        made = node.haptics.cue(body.cue, HANDS if body.hand == "both" else (body.hand,), test=True, source="test")
        return {"pulses": [p["id"] for p in made], "headset": node.haptics.headset}

    @app.post("/start", tags=["control"], summary="Start teleop: WBC lease, hints, enable, a datagram session "
                                                   "(409 with the WBC's reason if it refuses)")
    async def start_teleop():
        return await asyncio.to_thread(node.start_teleop)

    @app.post("/mode/{mode}", tags=["control"], summary="Third or first person (`third` | `first`): where the sticks "
                                                         "enter the frame chain; applies from the next grip")
    async def set_mode(mode: str):
        try:
            return node.set_mode(mode)
        except ValueError as e:
            raise HTTPException(400, str(e))

    @app.post("/mirror/{mode}", tags=["control"],
              summary="Two hands: the hand-to-arm pairing (`off` | `auto` | `straight` | `mirror`), decided when the "
                      "first hand engages and locked until both release; 409 while a hand is engaged")
    async def set_mirror(mode: str):
        try:
            return await asyncio.to_thread(node.set_mirror, mode)
        except ValueError as e:
            raise HTTPException(400, str(e))

    @app.post("/sim/auto_pause/{state}", tags=["control"],
              summary="Sim only: pause sim time while neither clutched nor driving (`on` | `off`)")
    async def sim_auto_pause(state: str):
        if node.aids is None:
            raise HTTPException(409, "not a sim stack (--sim none)")
        if state not in ("on", "off"):
            raise HTTPException(422, "on or off")
        return await asyncio.to_thread(node.aids.set_auto_pause, state == "on")

    @app.post("/sim/ghost/{state}", tags=["control"],
              summary="Sim only: the end effector's goal drawn as a gripper ghost, coloured by the error (`on` | `off`)")
    async def sim_ghost(state: str):
        if node.aids is None:
            raise HTTPException(409, "not a sim stack (--sim none)")
        if state not in ("on", "off"):
            raise HTTPException(422, "on or off")
        return await asyncio.to_thread(node.aids.set_ghost, state == "on")

    @app.post("/hint_source/{source}", tags=["control"],
              summary="What moves the base hint while engaged (`head` | `chest` | `hips`; body tracking, smoothed, "
                      "falling back to the headset); applies from the next grip")
    async def set_hint_source(source: str):
        try:
            return node.set_hint_source(source)
        except ValueError as e:
            raise HTTPException(400, str(e))

    @app.post("/elbow_hint/{state}", tags=["control"],
              summary="Send each engaged arm's elbow-swivel input to the WBC (`on` | `off`; body tracking): on "
                      "applies from each hand's next grip, off at once")
    async def set_elbow_hint(state: str):
        if state not in ("on", "off"):
            raise HTTPException(400, "state must be 'on' or 'off'")
        return node.set_elbow_hint(state == "on")

    @app.post("/tether/{state}", tags=["control"],
              summary="Stick tether (`on` | `off`): engaged, the stick offset leads the WBC's commanded base by at "
                      "most 5 cm / 0.1 rad and the arm keeps the commanded base's heading; from the next grip")
    async def set_tether(state: str):
        if state not in ("on", "off"):
            raise HTTPException(400, "state must be 'on' or 'off'")
        return node.set_tether(state == "on")

    @app.post("/headless/{state}", tags=["control"], summary="Headless left stick (`on` | `off`, third person, "
                                                              "registered): the stick moves along the operator's facing")
    async def set_headless(state: str):
        if state not in ("on", "off"):
            raise HTTPException(400, "state must be 'on' or 'off'")
        return node.set_headless(state == "on")

    @app.post("/top_speed", tags=["control"], summary="The top speed: the sticks' full deflection, the hint clamp and, "
                                                       "while running, the WBC caps this lease asserts (restored at stop)")
    async def set_top_speed(body: TopSpeedIn):
        try:
            return await asyncio.to_thread(node.set_top_speed, body.max_speed, body.max_omega)
        except ValueError as e:
            raise HTTPException(400, str(e))

    @app.post("/resume", tags=["control"], summary="The tiered recovery: transient dropouts hold and a re-grip "
                                                  "resumes; the grace window and the per-cause classes")
    async def set_resume(body: ResumeIn):
        try:
            return node.set_resume(body.enabled, body.grace_s, body.hold_max_s, body.classes)
        except ValueError as e:
            raise HTTPException(400, str(e))

    @app.post("/stop", tags=["control"], summary="Stop teleop: stop sending, disable the WBC, release its lease")
    async def stop_teleop():
        return await asyncio.to_thread(node.stop_teleop, "requested")

    @app.post("/estop", tags=["control"], summary="E-stop: the arm, the swerve and the WBC directly, in parallel (each "
                                                   "latches and coasts); stops teleop. `latched` = any latched; "
                                                   "`targets` per node")
    async def estop(request: Request, body: Optional[EStopIn] = None):
        body = body or EStopIn()
        src = body.source or (request.client.host if request.client else "")
        return await asyncio.to_thread(node.estop, body.reason, src)

    @app.post("/quest/up", tags=["control"], summary="quest_ctl up on the headset (ssh): update if behind, telemetry "
                                                      "headless with the scheme, wait for the stream; 409 if busy")
    async def quest_up():
        return node.quest_command("up")

    @app.post("/quest/down", tags=["control"], summary="quest_ctl down on the headset (ssh): quest stop; 409 if busy")
    async def quest_down():
        return node.quest_command("down")

    return app
