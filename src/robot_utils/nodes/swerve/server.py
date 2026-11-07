"""HTTP API of the swerve node (FastAPI). Conventions: docs/nodes.md; guide: docs/swerve_node.md.

Every controlling request carries the lease token in the ``X-Lease-Token`` header. Handlers
only touch the node's small, lock-protected state (CommandSlot, Lease, EStop, flags); the CAN
bus is owned by the control thread. Blocking work (starting / rebuilding the drive) runs in a
worker thread.
"""
from __future__ import annotations

import asyncio
import time
from contextlib import asynccontextmanager
from typing import Optional

from fastapi import FastAPI, Query, Request
from fastapi.responses import HTMLResponse, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field

from ..common.server import LeaseToken, add_base_routes, rows_stream, setup_app, sse
from .constants import COMMAND_TIMEOUT
from .node import FIELD_DOC, FIELDS, HARD_MAX_OMEGA, HARD_MAX_SPEED, LEASE_HEARTBEAT, UNITS, SwerveNode

TAGS = [
    {"name": "state", "description": "Status, clock sync, events."},
    {"name": "lease", "description": "One controller at a time. Acquire a lease, then send its token "
                                     "as `X-Lease-Token` with every controlling request."},
    {"name": "control", "description": "Enable, command, limits, e-stop, reset."},
    {"name": "stream", "description": "Every control tick (100 Hz) as binary float64 rows."},
]


class LeaseIn(BaseModel):
    client: str = Field("anonymous", description="Who is controlling (shown to others in /state).")
    model_config = ConfigDict(json_schema_extra={"examples": [{"client": "alice-laptop"}]})


class LeaseOut(BaseModel):
    token: str = Field(description="Send as the `X-Lease-Token` header.")
    client: str
    lease_id: int
    heartbeat: float = Field(description="Seconds without a renew (or accepted command) before the lease lapses.")


class CommandIn(BaseModel):
    twist: list[float] = Field(min_length=3, max_length=3,
                               description="[vx m/s forward, vy m/s left, omega rad/s CCW], robot frame.")
    seq: int = Field(0, description="Optional, for debugging only: echoed in the stream (`cmd_seq`). "
                                    "Never used for ordering or rejection; leave it 0 if you don't need it.")
    model_config = ConfigDict(json_schema_extra={"examples": [{"twist": [0.2, 0.0, 0.0]},
                                                              {"twist": [0.0, 0.0, 0.5], "seq": 17}]})


class LimitsIn(BaseModel):
    max_speed: Optional[float] = Field(None, ge=0, le=HARD_MAX_SPEED, description="m/s, translation norm.")
    max_omega: Optional[float] = Field(None, ge=0, le=HARD_MAX_OMEGA, description="rad/s.")
    model_config = ConfigDict(json_schema_extra={"examples": [{"max_speed": 0.5, "max_omega": 1.0}]})


class EStopIn(BaseModel):
    reason: str = ""
    source: str = Field("", description="Who pressed it (free text).")


def create_app(node: SwerveNode, start: bool = True) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app):
        node.hub.attach(asyncio.get_running_loop())
        if start:
            await asyncio.to_thread(node.start)
        yield
        await asyncio.to_thread(node.close)

    app = FastAPI(
        title="Swerve node",
        description=(
            "Twist control of the MAXSwerve base and its 100 Hz telemetry.\n\n"
            "Control: `POST /lease` → token; `POST /enable`; `POST /command` at least every "
            f"{COMMAND_TIMEOUT} s while moving (the command times out); `DELETE /lease` when done. "
            f"The lease lapses after {LEASE_HEARTBEAT} s without a renew or command. "
            f"Hard limits {HARD_MAX_SPEED} m/s and {HARD_MAX_OMEGA} rad/s; soft limits per lease. "
            "`POST /estop` works for anyone and latches until the lease holder clears it.\n\n"
            "Data: `GET /stream` (binary records, docs/nodes.md), `GET /fields` for the columns."),
        version="0.1.0", lifespan=lifespan, openapi_tags=TAGS)
    setup_app(app)
    add_base_routes(app, lambda: {"node": "swerve", "state": node.status})

    @app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def index():
        from .page import PAGE
        return PAGE

    @app.get("/state", tags=["state"], summary="Status, lease, limits, command, latest telemetry")
    async def get_state():
        return node.state()

    @app.get("/events", tags=["state"], summary="Server-Sent Events: /state on every change and every 1 s",
             response_class=StreamingResponse)
    async def events():
        return StreamingResponse(sse(node.hub, node.state), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-store"})

    @app.get("/fields", tags=["stream"], summary="Columns of the telemetry rows")
    async def fields():
        return {"fields": FIELDS, "units": UNITS, "doc": FIELD_DOC}

    @app.get("/stream", tags=["stream"], response_class=StreamingResponse,
             summary="Every control tick as binary float64 rows (schema record first)")
    async def stream(batch_ms: float = Query(0.0, ge=0, le=1000, description="Collect rows this long per record (0 = every tick)."),
                     max_backlog_s: float = Query(20.0, gt=0, le=600, description="Drop the oldest rows beyond this backlog.")):
        gen = rows_stream(node.hub, FIELDS, UNITS, batch_ms=batch_ms, max_rows=int(max_backlog_s * 100),
                          schema_extra={"node": "swerve", "rate_hz": 100})
        return StreamingResponse(gen, media_type="application/octet-stream", headers={"Cache-Control": "no-store"})

    @app.get("/lease", tags=["lease"], summary="Who holds the lease (no token)")
    async def get_lease():
        info = node.lease.info
        return None if info is None else {"client": info.client, "lease_id": info.lease_id,
                                          "expires_in": info.expires - time.monotonic()}

    @app.post("/lease", tags=["lease"], response_model=LeaseOut, summary="Acquire the lease (409 if held)")
    async def acquire(body: LeaseIn):
        token, info = node.acquire(body.client)
        return {"token": token, "client": info.client, "lease_id": info.lease_id, "heartbeat": node.lease.heartbeat}

    @app.post("/lease/renew", tags=["lease"], summary="Keep the lease (any accepted command also renews it)")
    async def renew(token: LeaseToken = None):
        info = node.renew(token)
        return {"lease_id": info.lease_id, "expires_in": info.expires - time.monotonic()}

    @app.delete("/lease", tags=["lease"], summary="Release the lease (stops and disables)")
    async def release(token: LeaseToken = None):
        node.release(token)
        return {"released": True}

    @app.post("/enable", tags=["control"], summary="Allow commands to move the base (holder)")
    async def enable(token: LeaseToken = None):
        node.enable(token)
        return {"enabled": True}

    @app.post("/disable", tags=["control"], summary="Stop and refuse commands (holder)")
    async def disable(token: LeaseToken = None):
        node.disable(token)
        return {"enabled": False}

    @app.post("/command", tags=["control"], summary="Set the twist (holds for 0.25 s; resend to keep moving)")
    async def command(body: CommandIn, token: LeaseToken = None):
        """The slow path: one command per request. It writes the same latest-command slot a future
        streaming fast path will write; the control tick applies whatever was written last."""
        return node.command(token, body.twist, body.seq, source="slow")

    @app.get("/limits", tags=["control"], summary="Hard and soft limits")
    async def get_limits():
        return node.limits()

    @app.post("/limits", tags=["control"], summary="Set soft limits within the hard limits (holder; reset when the lease ends)")
    async def set_limits(body: LimitsIn, token: LeaseToken = None):
        return node.set_limits(token, body.max_speed, body.max_omega)

    @app.post("/estop", tags=["control"], summary="E-stop: anyone; latches until the holder clears it")
    async def estop(request: Request, body: Optional[EStopIn] = None):
        body = body or EStopIn()
        src = body.source or (request.client.host if request.client else "")
        info = node.trigger_estop(body.reason, src)
        return {"latched": True, "reason": info.reason, "source": info.source}

    @app.post("/estop/clear", tags=["control"], summary="Clear the e-stop (holder; stays disabled)")
    async def clear_estop(token: LeaseToken = None):
        node.clear_estop(token)
        return {"latched": False}

    @app.post("/reset", tags=["control"], summary="Rebuild the drive after a loop error (holder; ends disabled)")
    async def reset(token: LeaseToken = None):
        node.lease.check(token)
        await asyncio.to_thread(node.reset, token)
        return {"state": node.status, "error": node.drive_error}

    return app
