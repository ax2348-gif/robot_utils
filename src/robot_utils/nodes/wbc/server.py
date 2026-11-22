"""HTTP API of the WBC node (FastAPI). Conventions: docs/nodes.md; guide: docs/wbc.md."""
from __future__ import annotations

import asyncio
import time
from contextlib import asynccontextmanager
from typing import Any, Optional

from fastapi import FastAPI, Query, Request
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field

from ..common.server import (EStopCleared, LeaseToken, add_base_routes, add_page_routes, node_meta, rows_stream,
                             setup_app, sse)
from . import __version__
from .node import WbcNode

TAGS = [
    {"name": "state", "description": "Status, leases, session, inputs, latest tick, events."},
    {"name": "lease", "description": "The WBC's own client lease (common/lease: token in X-Lease-Token)."},
    {"name": "control", "description": "Enable (the arm and swerve leases together), disable, e-stop, hints, "
                                       "the datagram session."},
    {"name": "stream", "description": "Every control tick (100 Hz) as binary float64 rows."},
]


class LeaseIn(BaseModel):
    client: str = Field("", description="Who is controlling (shown in /state and as the arm / swerve lease holder).")


class EStopIn(BaseModel):
    reason: str = ""
    source: str = Field("", description="Who pressed it (free text).")


class HintsIn(BaseModel):
    """The hint structure (docs/wbc.md "Hints"): `base` and `ees`; absent keys take the defaults."""
    base: Optional[dict] = None
    ees: Optional[dict] = None
    model_config = ConfigDict(extra="forbid", json_schema_extra={"examples": [
        {"base": {"mode": "locked"}},
        {"base": {"mode": "hint", "target": {"heading": {"yaw": 1.57, "weight": 0.5, "tol": 0.05}}},
         "ees": {"arm": {"weights": {"position": 1.0, "orientation": 0.3}}}}]})


class SessionOut(BaseModel):
    session: int = Field(description="Put it in every target / chunk / alive datagram.")
    udp_port: Optional[int] = Field(description="UDP port on the node's host (send there directly, not through "
                                                "the proxy); null if UDP is off.")
    unix_socket: Optional[str] = Field(description="Unix datagram socket path on the robot; null if off.")


def create_app(node: WbcNode, start: bool = True) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app):
        if node.hub is not None:
            node.hub.attach(asyncio.get_running_loop())
        if start:
            await asyncio.to_thread(node.start)
            node.hub.attach(asyncio.get_running_loop())
        yield
        await asyncio.to_thread(node.close)

    app = FastAPI(
        title="Whole-body control node",
        description=(
            "Moves the arm and the base together to follow end-effector targets in the world (estimator) frame "
            "(docs/wbc.md).\n\n"
            "`POST /lease`, then `POST /enable` (takes the arm and swerve leases together, opens the arm's joint "
            "channel), `POST /session` (-> session id, UDP port, unix socket) and send target / chunk / alive "
            "datagrams (JSON). `POST /hints` says how (base free / locked / hint / commanded, weights, posture). "
            "Stale input holds: the feedforward fades after ff_timeout, the base stops and the arm holds after "
            "hold_timeout. `GET /stream` carries every tick, including the commanded and measured EE poses."),
        version=__version__, lifespan=lifespan, openapi_tags=TAGS)
    setup_app(app)

    @app.exception_handler(ValueError)
    async def _value_error(request: Request, exc: ValueError):
        return JSONResponse(status_code=422, content={"detail": str(exc)})

    add_base_routes(app, lambda: {"node": "wbc", "state": node.state})
    add_page_routes(app, node_meta(node.name, "Whole-body control", __version__, "wbc", actuated=True,
                                   rate_hz=round(node.cfg.rate_hz)))

    @app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def index():
        from .page import PAGE
        return PAGE

    @app.get("/state", tags=["state"], summary="Status, fault, limits in force, leases, session, inputs, latest tick")
    async def get_state() -> Any:
        return node.state_dict()

    @app.get("/events", tags=["state"], response_class=StreamingResponse,
             summary="Server-Sent Events: /state on every change and every 1 s")
    async def events():
        return StreamingResponse(sse(node.hub, node.state_dict), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-store"})

    @app.get("/fields", tags=["stream"], summary="Columns of the tick rows (append-only)")
    async def fields():
        return {"fields": node.fields, "units": node.units, "doc": node.field_doc}

    @app.get("/stream", tags=["stream"], response_class=StreamingResponse,
             summary="Every control tick as binary float64 rows (schema record first)")
    async def stream(batch_ms: float = Query(0.0, ge=0, le=1000), max_hz: float = Query(0.0, ge=0, le=1000),
                     max_backlog_s: float = Query(20.0, gt=0, le=600)):
        gen = rows_stream(node.hub, node.fields, node.units, channel="tick", kind="tick", batch_ms=batch_ms,
                          max_hz=max_hz, max_rows=int(max_backlog_s * node.cfg.rate_hz),
                          schema_extra={"node": "wbc", "rate_hz": node.cfg.rate_hz})
        return StreamingResponse(gen, media_type="application/octet-stream", headers={"Cache-Control": "no-store"})

    # -- lease ---------------------------------------------------------------------------------------------

    @app.get("/lease", tags=["lease"], summary="Who holds the lease (no token)")
    async def get_lease():
        info = node.lease.info
        return None if info is None else {"client": info.client, "lease_id": info.lease_id,
                                          "expires_in": info.expires - node.clock()}

    @app.post("/lease", tags=["lease"], summary="Acquire the lease (409 if held)")
    async def acquire(body: LeaseIn = LeaseIn()):
        token, info = node.acquire(body.client)
        return {"token": token, "client": info.client, "lease_id": info.lease_id, "heartbeat": node.lease.heartbeat}

    @app.post("/lease/renew", tags=["lease"], summary="Keep the lease (every accepted datagram also renews it)")
    async def renew(token: LeaseToken = None):
        info = node.renew(token)
        return {"lease_id": info.lease_id, "expires_in": info.expires - node.clock()}

    @app.delete("/lease", tags=["lease"], summary="Release the lease (the WBC holds; POST /disable releases the robot)")
    async def release(token: LeaseToken = None):
        node.release(token)
        return {"released": True}

    # -- control -------------------------------------------------------------------------------------------

    @app.post("/enable", tags=["control"], summary="Take the arm and swerve leases together, enable, open the arm's "
                                                    "joint channel (holder); refuses with the reason if any part fails")
    async def enable(token: LeaseToken = None):
        return await asyncio.to_thread(node.enable, token)

    @app.post("/disable", tags=["control"], summary="Disable: the base stops, the arm holds then is released; "
                                                     "clears a fault (holder)")
    async def disable(token: LeaseToken = None):
        return await asyncio.to_thread(node.disable, "requested", token, True)

    @app.post("/estop", tags=["control"], summary="E-stop: anyone; forwarded to the arm and the swerve (both latch); "
                                                   "disables")
    async def estop(request: Request, body: Optional[EStopIn] = None):
        body = body or EStopIn()
        src = body.source or (request.client.host if request.client else "")
        return await asyncio.to_thread(node.trigger_estop, body.reason, src)

    @app.post("/estop/clear", response_model=EStopCleared, tags=["control"], summary="Clear the WBC's e-stop latch "
              "(holder; the arm's and the swerve's are cleared on those nodes)")
    async def clear_estop(token: LeaseToken = None):
        node.clear_estop(token)
        return {"latched": False}

    @app.get("/hints", tags=["control"], summary="The session hints, and the hints in force (with the latest "
                                                  "per-command override)")
    async def get_hints():
        return node.get_hints()

    @app.post("/hints", tags=["control"], summary="Replace the session hints (holder; 422 beyond the caps)")
    async def set_hints(body: HintsIn, token: LeaseToken = None):
        return node.set_hints(token, body.model_dump(exclude_none=True))

    @app.post("/session", response_model=SessionOut, tags=["control"],
              summary="Open the datagram session (holder; replaces the previous one)")
    async def session(body: Optional[dict] = None, token: LeaseToken = None):
        return node.open_session(token)

    @app.get("/fence", tags=["control"], summary="The base fence (not built yet)")
    async def fence():
        return JSONResponse(status_code=501, content={"detail": "the base fence is not built yet (wbc_v1.md)"})

    @app.get("/config", tags=["state"], summary="The config in force, the defaults and the hard limits checked against")
    async def config():
        return node.get_config()

    @app.post("/config", tags=["control"], summary="Change config keys at runtime (no lease; structural keys only while "
                                                    "disabled; 422 with the reason when invalid)")
    async def set_config(body: dict):
        try:
            return await asyncio.to_thread(node.set_config, body)
        except ValueError as e:
            return JSONResponse(status_code=422, content={"detail": str(e)})

    return app
