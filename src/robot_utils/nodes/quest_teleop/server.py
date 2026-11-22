"""HTTP API of the Quest teleop node (FastAPI). Conventions: docs/nodes.md; guide: docs/quest_teleop_node.md."""
from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from typing import Any, Optional

from fastapi import FastAPI, Query, Request
from fastapi.responses import HTMLResponse, StreamingResponse
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
]


class TaskIn(BaseModel):
    text: str = Field("", description="The task of the next episodes (in each episode_start's detail).")


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
            "the client-side script's job."),
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

    @app.post("/start", tags=["control"], summary="Start teleop: WBC lease, hints, enable, a datagram session "
                                                   "(409 with the WBC's reason if it refuses)")
    async def start_teleop():
        return await asyncio.to_thread(node.start_teleop)

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
