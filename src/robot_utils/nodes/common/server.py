"""FastAPI building blocks shared by the nodes (see docs/nodes.md for the conventions).

- ``setup_app(app)``: CORS open to every origin (a dashboard on another port talks to several
  nodes), and ``RequestError`` / ``LeaseError`` -> its HTTP status with ``{"detail": ...}``.
- ``add_base_routes(app, health)``: ``GET /time`` (NTP-style clock sync) and ``GET /health``.
- ``EventLog``: a short log of node events for ``/state``.
- ``rows_stream(...)``: the async generator behind every-sample ``GET /stream`` endpoints
  (schema record first, then rows records; pings when idle).
- ``sse(...)``: Server-Sent Events of a JSON state on every change and every second.
- ``LeaseToken``: the ``X-Lease-Token`` header as a FastAPI dependency parameter.
- Response models of what every node serves (``Health``, ``NodeMeta``, ``TimeResponse``, ``Event``,
  ``EStopState`` / ``EStopTriggered`` / ``EStopCleared``, ``LockOut``), for nodes to reuse.
"""
from __future__ import annotations

import asyncio
import json
import math
import time
from collections import deque
from typing import Annotated, Any, Callable, Optional, Sequence

from fastapi import FastAPI, Header, Request
from fastapi.encoders import jsonable_encoder
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ConfigDict, Field

from .hub import RowHub
from .lease import RequestError
from .protocol import pack_record, rows_record, schema_record

PING_S = 1.0   # keep-alive interval of idle streams

LeaseToken = Annotated[Optional[str], Header(alias="X-Lease-Token",
                                              description="Token from POST /lease (controlling requests).")]


class TimeResponse(BaseModel):
    mono: float = Field(description="Server time.monotonic() (s). The reference clock of every timestamp.")
    wall: float = Field(description="Server time.time() (Unix epoch s).")
    epoch_minus_mono: float = Field(description="wall - mono at this instant.")
    model_config = ConfigDict(json_schema_extra={"examples": [
        {"mono": 18234.512345, "wall": 1794019145.300510, "epoch_minus_mono": 1794000910.788165}]})


class Event(BaseModel):
    """One ``EventLog`` entry."""
    t: float = Field(description="Server time.monotonic() (s) when it happened (the clock of /time `mono`).")
    wall: float = Field(description="Server time.time() (Unix epoch s) when it happened.")
    msg: str
    level: str = Field("info", description="info | warn | error (a few older entries say `warning`: treat it as warn).")


class Health(BaseModel):
    """``GET /health``. Open-ended: each node adds its own keys (agent_nav: `session`)."""
    ok: bool = Field(description="Always true: the node answers.")
    node: Optional[str] = Field(None, description="Node kind, e.g. `swerve`.")
    state: Optional[str] = Field(None, description="The node's status word (the `state` of its /state).")
    model_config = ConfigDict(extra="allow", json_schema_extra={"examples": [
        {"ok": True, "node": "swerve", "state": "running"}]})


class MetaEndpoints(BaseModel):
    """Paths relative to the node root; null if the node lacks it."""
    health: str
    state: str
    events: str
    estop: Optional[str] = Field(description="`estop` on actuated nodes, else null.")


class NodeMeta(BaseModel):
    """``GET /meta`` (built by ``node_meta``). Open-ended: nodes add their own keys (e.g. `rate_hz`,
    the telemetry rate in Hz; `fake`, a simulated backend)."""
    name: str = Field(description="Instance name (the proxy path /nodes/<name>/); the kind unless renamed at startup.")
    title: str = Field(description="Human-readable title.")
    version: str
    kind: str = Field(description="Node type, e.g. `swerve`.")
    has_page: bool = Field(description="Serves a browser page at its root.")
    page: Optional[str] = Field(description="`./` if it has a page, else null.")
    actuated: bool = Field(description="Moves hardware (has a lease and an e-stop).")
    endpoints: MetaEndpoints
    model_config = ConfigDict(extra="allow")


class EStopState(BaseModel):
    """The e-stop as reported in an actuated node's /state."""
    latched: bool
    reason: Optional[str] = Field(description="Why it was pressed; null if not latched.")
    source: Optional[str] = Field(description="Who pressed it (free text, or the client IP); null if not latched.")
    t: Optional[float] = Field(description="Server time.monotonic() (s) when it latched; null if not latched.")


class EStopTriggered(BaseModel):
    """``POST /estop``. A second press keeps the first reason and source."""
    latched: bool = Field(description="Always true.")
    reason: str = Field(description="Reason of the press in force (`e-stop` if none was given).")
    source: str = Field(description="Source of the press in force (`unknown` if none).")


class EStopCleared(BaseModel):
    latched: bool = Field(description="Always false.")


class LockOut(BaseModel):
    """A lock in force (common/locks.py ``Lock.public()``)."""
    id: str
    kind: str = Field(description="chock | cover.")
    target: str = Field(description="Node kind, instance name, or `all`.")
    by: str = Field(description="Who placed it.")
    reason: str
    placed: float = Field(description="Unix epoch s (wall clock) when placed.")
    source: str = Field(description="How it was placed: `ssh` or `web <client>`.")
    age_s: float = Field(description="Seconds since placed (wall clock, 0.1 s resolution).")


class EventLog:
    def __init__(self, n: int = 50, on_add: Optional[Callable[[], None]] = None):
        self._events: deque = deque(maxlen=n)
        self.on_add = on_add

    def add(self, msg: str, level: str = "info") -> None:
        self._events.append({"t": time.monotonic(), "wall": time.time(), "msg": msg, "level": level})
        if self.on_add is not None:
            self.on_add()

    def list(self) -> list[dict]:
        return list(self._events)


def setup_app(app: FastAPI) -> None:
    app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"],
                       expose_headers=["*"])

    @app.exception_handler(RequestError)
    async def _request_error(request: Request, exc: RequestError):
        return JSONResponse(status_code=exc.status, content={"detail": str(exc), **_json_safe(jsonable_encoder(exc.extra))})

    @app.exception_handler(RequestValidationError)
    async def _validation_error(request: Request, exc: RequestValidationError):
        # FastAPI's default 422 echoes the offending input, and a non-finite one (JSON 1e400 ->
        # inf) can't be encoded, turning the 422 into a 500: send those as strings instead.
        return JSONResponse(status_code=422, content={"detail": _json_safe(jsonable_encoder(exc.errors()))})


def _json_safe(v):
    if isinstance(v, float) and not math.isfinite(v):
        return str(v)
    if isinstance(v, dict):
        return {k: _json_safe(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_json_safe(x) for x in v]
    return v


def add_base_routes(app: FastAPI, health: Callable[[], dict]) -> None:
    @app.get("/time", response_model=TimeResponse, tags=["state"], summary="Server clocks for NTP-style sync")
    async def get_time():
        """Read `t0` on your clock, call this, read `t1`. Then `offset = mono - (t0+t1)/2` maps your
        clock to the server's (`t_server = t_client + offset`) within ±(t1-t0)/2. Repeat ~16 times
        over a kept-alive connection and keep the smallest round trips (`common.client.NodeClient.sync`)."""
        m = time.monotonic()
        w = time.time()
        return {"mono": m, "wall": w, "epoch_minus_mono": w - m}

    @app.get("/health", response_model=Health, response_model_exclude_unset=True, tags=["state"],
             summary="Liveness")
    async def get_health():
        return {"ok": True, **health()}


async def rows_stream(hub: RowHub, fields: Sequence[str], units: Sequence[str], *,
                      channel: Optional[str] = None, kind: Optional[str] = None,
                      batch_ms: float = 0.0, max_rows: int = 2000, schema_id: int = 1,
                      schema_extra: Optional[dict] = None, max_hz: float = 0.0):
    """Every row published on a hub channel, as a schema record and then rows records.

    ``batch_ms`` > 0 coalesces each client's rows for that long before sending (fewer, larger
    records; per client). ``kind`` (e.g. "tick") goes into every schema/rows header. A client
    more than ``max_rows`` behind loses the oldest rows; the next record's ``dropped`` says how
    many. ``max_hz`` > 0 thins the stream for overviews: at most that many rows per second, each the
    latest row (the skipped ones are not counted as ``dropped``).
    """
    sub = hub.subscribe(max_rows, channel)
    k = {"kind": kind} if kind else {}
    try:
        yield schema_record(schema_id, fields, units, t_send=time.monotonic(), **k, **(schema_extra or {}))
        last_send = time.monotonic()
        while True:
            try:
                await asyncio.wait_for(sub.event.wait(), PING_S)
            except asyncio.TimeoutError:
                yield pack_record({"type": "ping", "t_send": time.monotonic()})
                last_send = time.monotonic()
                continue
            gap = max(batch_ms / 1000.0, 1.0 / max_hz if max_hz > 0 else 0.0)
            if gap > 0:
                wait = gap - (time.monotonic() - last_send)
                if wait > 0:
                    await asyncio.sleep(wait)
            rows, dropped = sub.take()
            if rows is None:
                continue
            if max_hz > 0:
                rows = rows[-1:]
            yield rows_record(rows, schema_id, dropped=dropped, t_send=time.monotonic(), **k)
            last_send = time.monotonic()
    finally:
        hub.unsubscribe(sub)


async def sse(hub: RowHub, state: Callable[[], Any]):
    """``text/event-stream`` chunks: the JSON state on every change and at least every second."""
    while True:
        fut = hub.state_future()
        yield f"data: {json.dumps(state(), default=str)}\n\n"
        try:
            await asyncio.wait_for(asyncio.shield(fut), 1.0)
        except asyncio.TimeoutError:
            pass


def node_meta(name: str, title: str, version: str, kind: str, *, actuated: bool,
              has_page: bool = True, **extra) -> dict:
    """``GET /meta`` body (the frontend's contract). Paths are relative to the node root."""
    return {"name": name, "title": title, "version": version, "kind": kind,
            "has_page": has_page, "page": "./" if has_page else None, "actuated": actuated,
            "endpoints": {"health": "health", "state": "state", "events": "events",
                          "estop": "estop" if actuated else None},
            **extra}


FONT_TYPES = {".woff2": "font/woff2", ".txt": "text/plain"}


def font_response(name: str):
    """A bundled font file (``nodes/common/fonts/``): served next to common/ui.css by every node
    and by the landing page, so pages get the same typeface offline."""
    from fastapi.responses import Response
    from importlib import resources
    from pathlib import PurePosixPath
    if "/" in name or name.startswith(".") or PurePosixPath(name).suffix not in FONT_TYPES:
        return Response(status_code=404)
    f = resources.files("robot_utils.nodes.common").joinpath("fonts", name)
    if not f.is_file():
        return Response(status_code=404)
    return Response(f.read_bytes(), media_type=FONT_TYPES[PurePosixPath(name).suffix],
                    headers={"Cache-Control": "public, max-age=604800"})


def add_viz_routes(app: FastAPI) -> None:
    """The shared visualisation components at ``common/viz.js`` (viz_assets.py; ``RU.viz``)."""
    from fastapi.responses import Response

    from .viz_assets import VIZ_JS

    @app.get("/common/viz.js", include_in_schema=False)
    async def viz_js():
        return Response(VIZ_JS, media_type="text/javascript", headers={"Cache-Control": "no-cache"})


def add_page_routes(app: FastAPI, meta: dict) -> None:
    """``GET /meta`` plus the shared browser toolkit at ``common/ui.js`` and ``common/ui.css``."""
    from fastapi.responses import Response

    from .ui_assets import UI_CSS, UI_JS

    app.state.node_meta = meta   # serve() sets meta["name"] to the instance name
    add_viz_routes(app)

    @app.get("/meta", response_model=NodeMeta, tags=["state"], summary="What this node is (for dashboards)")
    async def get_meta():
        return meta

    @app.get("/common/ui.js", include_in_schema=False)
    async def ui_js():
        return Response(UI_JS, media_type="text/javascript", headers={"Cache-Control": "no-cache"})

    @app.get("/common/fonts/{name}", include_in_schema=False)
    async def ui_font(name: str):
        return font_response(name)

    @app.get("/common/ui.css", include_in_schema=False)
    async def ui_css():
        return Response(UI_CSS, media_type="text/css", headers={"Cache-Control": "no-cache"})
