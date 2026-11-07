"""FastAPI building blocks shared by the nodes (see docs/nodes.md for the conventions).

- ``setup_app(app)``: CORS open to every origin (a dashboard on another port talks to several
  nodes), and ``RequestError`` / ``LeaseError`` -> its HTTP status with ``{"detail": ...}``.
- ``add_base_routes(app, health)``: ``GET /time`` (NTP-style clock sync) and ``GET /health``.
- ``EventLog``: a short log of node events for ``/state``.
- ``rows_stream(...)``: the async generator behind every-sample ``GET /stream`` endpoints
  (schema record first, then rows records; pings when idle).
- ``sse(...)``: Server-Sent Events of a JSON state on every change and every second.
- ``LeaseToken``: the ``X-Lease-Token`` header as a FastAPI dependency parameter.
"""
from __future__ import annotations

import asyncio
import json
import time
from collections import deque
from typing import Annotated, Any, Callable, Optional, Sequence

from fastapi import FastAPI, Header, Request
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
    t: float = Field(description="Server monotonic s.")
    wall: float
    msg: str
    level: str = "info"


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
        return JSONResponse(status_code=exc.status, content={"detail": str(exc)})


def add_base_routes(app: FastAPI, health: Callable[[], dict]) -> None:
    @app.get("/time", response_model=TimeResponse, tags=["state"], summary="Server clocks for NTP-style sync")
    async def get_time():
        """Read `t0` on your clock, call this, read `t1`. Then `offset = mono - (t0+t1)/2` maps your
        clock to the server's (`t_server = t_client + offset`) within ±(t1-t0)/2. Repeat ~16 times
        over a kept-alive connection and keep the smallest round trips (`common.client.NodeClient.sync`)."""
        m = time.monotonic()
        w = time.time()
        return {"mono": m, "wall": w, "epoch_minus_mono": w - m}

    @app.get("/health", tags=["state"], summary="Liveness")
    async def get_health():
        return {"ok": True, **health()}


async def rows_stream(hub: RowHub, fields: Sequence[str], units: Sequence[str], *,
                      batch_ms: float = 0.0, max_rows: int = 2000, schema_id: int = 1,
                      schema_extra: Optional[dict] = None):
    """Every published row, as a schema record and then rows records (bytes chunks).

    ``batch_ms`` > 0 collects rows for that long before sending (fewer, larger records).
    A client more than ``max_rows`` behind loses the oldest rows; the next record's ``dropped``
    says how many.
    """
    sub = hub.subscribe(max_rows)
    try:
        yield schema_record(schema_id, fields, units, t_send=time.monotonic(), **(schema_extra or {}))
        last_send = time.monotonic()
        while True:
            try:
                await asyncio.wait_for(sub.event.wait(), PING_S)
            except asyncio.TimeoutError:
                yield pack_record({"type": "ping", "t_send": time.monotonic()})
                last_send = time.monotonic()
                continue
            if batch_ms > 0:
                wait = batch_ms / 1000.0 - (time.monotonic() - last_send)
                if wait > 0:
                    await asyncio.sleep(wait)
            rows, dropped = sub.take()
            if rows is None:
                continue
            yield rows_record(rows, schema_id, dropped=dropped, t_send=time.monotonic())
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


def add_page_routes(app: FastAPI, meta: dict) -> None:
    """``GET /meta`` plus the shared browser toolkit at ``common/ui.js`` and ``common/ui.css``."""
    from fastapi.responses import Response

    from .ui_assets import UI_CSS, UI_JS

    @app.get("/meta", tags=["state"], summary="What this node is (for dashboards)")
    async def get_meta():
        return meta

    @app.get("/common/ui.js", include_in_schema=False)
    async def ui_js():
        return Response(UI_JS, media_type="text/javascript", headers={"Cache-Control": "no-cache"})

    @app.get("/common/ui.css", include_in_schema=False)
    async def ui_css():
        return Response(UI_CSS, media_type="text/css", headers={"Cache-Control": "no-cache"})
