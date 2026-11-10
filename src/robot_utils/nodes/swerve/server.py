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
from pydantic import BaseModel, ConfigDict, Field, create_model

from ..common.server import (EStopCleared, EStopState, EStopTriggered, Event, LeaseToken, LockOut, add_base_routes,
                             add_page_routes, node_meta, rows_stream, setup_app, sse)
from .constants import COMMAND_TIMEOUT, MODULES
from .node import (AUTO_INTERVAL_S, AUTO_STILL_S, FIELD_DOC, FIELDS, HARD_MAX_OMEGA, HARD_MAX_SPEED, LEASE_HEARTBEAT,
                   REBASE_STILL_S, UNITS, SwerveNode)

TAGS = [
    {"name": "state", "description": "Status, clock sync, events."},
    {"name": "lease", "description": "One controller at a time. Acquire a lease, then send its token "
                                     "as `X-Lease-Token` with every controlling request."},
    {"name": "control", "description": "Enable, command, limits, e-stop, reset."},
    {"name": "stream", "description": "Every control tick (100 Hz) as binary float64 rows."},
    {"name": "encoders", "description": "Drive-encoder rebase: drive_pos back to 0 while stationary (float32 "
                                        "precision). Moves nothing; anyone may call."},
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


class AutoRebaseIn(BaseModel):
    enabled: Optional[bool] = Field(None, description="Rebase automatically when due and stationary.")
    interval_s: Optional[float] = Field(None, ge=1.0, allow_inf_nan=False, description="Due when the last rebase is older than this (s).")
    still_s: Optional[float] = Field(None, ge=REBASE_STILL_S, le=60.0,
                                     description="Stationary for this long first (s).")
    model_config = ConfigDict(json_schema_extra={"examples": [{"enabled": False}, {"interval_s": 300, "still_s": 2.0}]})


class EStopIn(BaseModel):
    reason: str = ""
    source: str = Field("", description="Who pressed it (free text).")


# -- responses ---------------------------------------------------------------------------------

class SpeedLimits(BaseModel):
    max_speed: float = Field(description="m/s, translation norm (of vx, vy).")
    max_omega: float = Field(description="rad/s.")


class Limits(BaseModel):
    hard: SpeedLimits = Field(description="Fixed.")
    soft: SpeedLimits = Field(description="Set by the lease holder within the hard limits; back to the hard "
                                          "limits when the lease ends.")


class LeaseState(BaseModel):
    held: bool
    holder: Optional[str] = Field(description="`client` of the holder; null if not held.")
    lease_id: Optional[int] = Field(description="Increments with every acquire; null if not held.")
    expires_in: Optional[float] = Field(description="s until it lapses without a renew or command (1 ms "
                                                    "resolution); null if not held.")


class LeaseHolder(BaseModel):
    client: str
    lease_id: int
    expires_in: float = Field(description="s until it lapses without a renew or command (may be slightly "
                                          "negative until the next control tick ends it).")


class LeaseRenewed(BaseModel):
    lease_id: int
    expires_in: float = Field(description="s until it lapses without a renew or command (the heartbeat).")


class LeaseReleased(BaseModel):
    released: bool = Field(description="Always true.")


class Enabled(BaseModel):
    enabled: bool = Field(description="The node's enable flag after the request.")


class CommandState(BaseModel):
    """The command in the latest-command slot (it stops moving the base COMMAND_TIMEOUT s after it was written)."""
    twist: list[float] = Field(description="[vx m/s forward, vy m/s left, omega rad/s CCW], robot frame, as "
                                           "requested (before the soft-limit clamp).")
    seq: int = Field(description="Echo of the request's `seq`.")
    source: str = Field(description="slow (POST /command) | fast.")
    age: float = Field(description="s since it was written (server monotonic, 1 ms resolution).")


class CommandOut(BaseModel):
    seq: int = Field(description="Echo of the request's `seq`.")
    applied: list[float] = Field(description="[vx m/s, vy m/s, omega rad/s] after the soft-limit clamp: what the "
                                             "control tick sends while the command is fresh.")
    clamped: bool = Field(description="The soft limits changed the twist.")
    soft_limits: SpeedLimits = Field(description="The soft limits applied.")


class AutoRebase(BaseModel):
    """Opportunistic rebase policy (runtime only; back to the defaults when the node restarts)."""
    enabled: bool = Field(description="On by default.")
    interval_s: float = Field(description=f"Rebase when the last one is older than this (default {AUTO_INTERVAL_S:g} s; "
                                          "also once at node start).")
    still_s: float = Field(description=f"... and the base has been stationary this long (default {AUTO_STILL_S:g} s).")


class Encoders(BaseModel):
    """Drive-encoder rebase state. Stationary = every |drive_vel| < 0.01 m/s, fresh readings,
    and no non-zero command in force."""
    epoch: int = Field(description="Confirmed rebases since the node started (= `enc_epoch` in /stream).")
    last_rebase_t: Optional[float] = Field(description="Server monotonic s of the last confirmed rebase; null if none.")
    last_rebase_age: Optional[float] = Field(description="s since then (1 ms resolution); null if none.")
    rebasing: bool = Field(description="A rebase was sent and is not confirmed yet.")
    last_error: Optional[str] = Field(description="Why the last rebase failed; null after a success.")
    auto: AutoRebase
    max_abs_drive_pos: Optional[float] = Field(description="m, largest |drive_pos| of the four drives now (float32 "
                                                           "resolution degrades as it grows); null without readings.")
    stationary_s: Optional[float] = Field(description="s the base has been stationary; null while it is not.")
    motion: Optional[str] = Field(description="Why the base is not stationary; null when it is.")


class ResetOut(BaseModel):
    state: str = Field(description="Drive status after the rebuild: running | no drive (it failed) | error.")
    error: Optional[str] = Field(description="Why the drive failed to start; null if it started.")


class FieldsOut(BaseModel):
    fields: list[str] = Field(description="Column names of the /stream rows, in order.")
    units: list[str] = Field(description="Unit of each column (s, m/s, rad, rad/s, m, duty, A; bool = 0/1; "
                                         "enum, id, count).")
    doc: dict[str, str] = Field(description="Meaning of the less obvious columns (keys may be patterns: "
                                            "`<module>_*`, `req_*`).")


_TELEMETRY_DOC = {
    "t_tick": "server time.monotonic() at the start of the control tick",
    "tick": "control ticks since the node started",
    "motors_enabled": "1 if the tick sent setpoints and an enabling heartbeat",
    "enabled": "the node's enable flag (POST /enable)",
    "estop": "1 if the e-stop is latched",
    "lease_id": "id of the lease in force (0 = none)",
    "cmd_seq": "`seq` of the command in the slot (0 = none)",
    "cmd_source": "writer of the command in the slot: 0 none, 1 slow (POST /command), 2 fast",
    "cmd_age": "t_tick minus the time the command was written; null if none",
    "req_vx": "requested forward speed, robot frame; null if no command",
    "req_vy": "requested left speed, robot frame; null if no command",
    "req_omega": "requested yaw rate (CCW +); null if no command",
    "cmd_vx": "forward speed sent this tick (after the soft-limit clamp); null when stopped",
    "cmd_vy": "left speed sent this tick (after the soft-limit clamp); null when stopped",
    "cmd_omega": "yaw rate sent this tick (after the soft-limit clamp); null when stopped",
    "soft_max_speed": "soft translation limit in force",
    "soft_max_omega": "soft yaw-rate limit in force",
    "loop_ok": "0 once the control loop stopped on an error",
    "enc_epoch": "confirmed drive-encoder rebases since the node started; drive_pos restarts near 0 when it changes",
    "enc_rebasing": "1 from sending a rebase until it is confirmed or fails (drive_pos may jump: don't integrate it)",
    "rebase_age": "t_tick minus the last confirmed rebase; null if none yet",
    "steer_angle": "wheel angle from the absolute encoder (0 = forward, CCW +), STATUS_5",
    "steer_vel": "steering rate from the absolute encoder's velocity, STATUS_5",
    "t_steer": "server monotonic arrival time of the steer SPARK's STATUS_5",
    "drive_vel": "wheel speed, STATUS_2",
    "drive_pos": "wheel travel since the SPARK booted or the last rebase (float32 on the wire), STATUS_2",
    "t_drive": "server monotonic arrival time of the drive SPARK's STATUS_2",
    "steer_sp": "wheel angle setpoint sent this tick",
    "drive_sp": "wheel speed setpoint sent this tick",
    "steer_out": "steer motor applied output (-1..1), STATUS_0",
    "steer_current": "steer motor current, STATUS_0",
    "t_steer_s0": "server monotonic arrival time of the steer SPARK's STATUS_0",
    "drive_out": "drive motor applied output (-1..1), STATUS_0",
    "drive_current": "drive motor current, STATUS_0",
    "t_drive_s0": "server monotonic arrival time of the drive SPARK's STATUS_0",
}


def _column(name: str, unit: str):
    module, _, rest = name.partition("_")
    doc = (f"{module} module: {_TELEMETRY_DOC[rest]}" if module in {m.name for m in MODULES}
           else _TELEMETRY_DOC[name])
    return Optional[float], Field(description=f"{unit}; {doc} (null = no value).")


# One field per /stream column (FIELDS), flat like the rows; module columns are prefixed FL_, FR_, RR_, RL_.
Telemetry = create_model("Telemetry", __doc__="The last control tick's row, as in /stream (NaN sent as null).",
                         **{n: _column(n, u) for n, u in zip(FIELDS, UNITS)})


class SwerveState(BaseModel):
    state: str = Field(description="Drive status: starting | running | error | no drive.")
    error: Optional[str] = Field(description="Why the drive failed to start or the control loop stopped; null if fine.")
    interface: str = Field(description="CAN interface, e.g. `can0`.")
    enabled: bool = Field(description="Commands may move the base (POST /enable).")
    chocked: bool = Field(description="A chock lock covers this node: stopped, and enabling is refused (423).")
    locks: list[LockOut] = Field(description="The chocks in force.")
    estop: EStopState
    lease: LeaseState
    limits: Limits
    command: Optional[CommandState] = Field(description="The latest command, even if timed out; null if none since "
                                                        "the last enable / disable / lease end / e-stop.")
    command_timeout: float = Field(description="s a command keeps moving the base.")
    lease_heartbeat: float = Field(description="s without a renew or command before the lease lapses.")
    loop_hz: Optional[float] = Field(description="Control tick rate measured over the last ~100 ticks (Hz); null "
                                                 "while there are too few.")
    clients: int = Field(description="Open /stream subscribers.")
    encoders: Encoders
    events: list[Event] = Field(description="Latest node events (up to 20), oldest first.")
    latest: Optional[Telemetry] = Field(description="The last tick's telemetry; null before the first tick.")
    server_mono: float = Field(description="Server time.monotonic() (s) when this state was built.")


STREAM_DOC = (
    "Records `[u32 big-endian header length N][N bytes UTF-8 JSON header][header.len bytes payload]` "
    "(docs/nodes.md; parse with `common.protocol` or `RU.RecordReader`). Every header has `type` and `len`. "
    "First a `schema` record (no payload): `schema` (id 1), `fields` and `units` (as GET /fields), "
    "`dtype` `\"<f8\"`, `kind` `\"tick\"`, `node` `\"swerve\"`, `rate_hz` 100, `t_send`. Then `rows` records: "
    "payload a little-endian float64 array of shape (n, len(fields)); header `n`, `schema`, `dropped` (rows lost "
    "because this client fell behind), `kind`, `t_send`. `ping` records (header `t_send` only, no payload) after "
    "~1 s without data. `t_send`: server monotonic s.")


def create_app(node: SwerveNode, start: bool = True) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app):
        node.hub.attach(asyncio.get_running_loop())
        name = getattr(app.state, "node_meta", {}).get("name")
        if name:
            node.lock_names.add(name)            # chocks may name this instance as well as "swerve"
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
    add_page_routes(app, node_meta("swerve", "Swerve base", app.version, "swerve", actuated=True,
                                   rate_hz=100))

    @app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def index():
        from .page import PAGE
        return PAGE

    @app.get("/state", response_model=SwerveState, tags=["state"], summary="Status, lease, limits, command, latest telemetry")
    async def get_state():
        return node.state()

    @app.get("/events", tags=["state"], summary="Server-Sent Events: /state on every change and every 1 s",
             response_class=StreamingResponse,
             responses={200: {"content": {"text/event-stream": {}},
                              "description": "Unnamed events; each `data:` line is the GET /state JSON (SwerveState)."}})
    async def events():
        return StreamingResponse(sse(node.hub, node.state), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-store"})

    @app.get("/fields", response_model=FieldsOut, tags=["stream"], summary="Columns of the telemetry rows")
    async def fields():
        return {"fields": FIELDS, "units": UNITS, "doc": FIELD_DOC}

    @app.get("/stream", tags=["stream"], response_class=StreamingResponse,
             summary="Every control tick as binary float64 rows (schema record first)",
             responses={200: {"content": {"application/octet-stream": {}}, "description": STREAM_DOC}})
    async def stream(batch_ms: float = Query(0.0, ge=0, le=1000, description="Collect rows this long per record (0 = every tick)."),
                     max_hz: float = Query(0.0, ge=0, le=100, description="At most this many rows per second, the latest each time (0 = all; e.g. 10 for an overview)."),
                     max_backlog_s: float = Query(20.0, gt=0, le=600, description="Drop the oldest rows beyond this backlog.")):
        gen = rows_stream(node.hub, FIELDS, UNITS, channel="tick", kind="tick", batch_ms=batch_ms,
                          max_rows=int(max_backlog_s * 100),
                          schema_extra={"node": "swerve", "rate_hz": 100}, max_hz=max_hz)
        return StreamingResponse(gen, media_type="application/octet-stream", headers={"Cache-Control": "no-store"})

    @app.get("/lease", response_model=Optional[LeaseHolder], tags=["lease"], summary="Who holds the lease (no token)")
    async def get_lease():
        info = node.lease.info
        return None if info is None else {"client": info.client, "lease_id": info.lease_id,
                                          "expires_in": info.expires - time.monotonic()}

    @app.post("/lease", tags=["lease"], response_model=LeaseOut, summary="Acquire the lease (409 if held)")
    async def acquire(body: LeaseIn):
        token, info = node.acquire(body.client)
        return {"token": token, "client": info.client, "lease_id": info.lease_id, "heartbeat": node.lease.heartbeat}

    @app.post("/lease/renew", response_model=LeaseRenewed, tags=["lease"], summary="Keep the lease (any accepted command also renews it)")
    async def renew(token: LeaseToken = None):
        info = node.renew(token)
        return {"lease_id": info.lease_id, "expires_in": info.expires - time.monotonic()}

    @app.delete("/lease", response_model=LeaseReleased, tags=["lease"], summary="Release the lease (stops and disables)")
    async def release(token: LeaseToken = None):
        node.release(token)
        return {"released": True}

    @app.post("/enable", response_model=Enabled, tags=["control"], summary="Allow commands to move the base (holder)")
    async def enable(token: LeaseToken = None):
        node.enable(token)
        return {"enabled": True}

    @app.post("/disable", response_model=Enabled, tags=["control"], summary="Stop and refuse commands (holder)")
    async def disable(token: LeaseToken = None):
        node.disable(token)
        return {"enabled": False}

    @app.post("/command", response_model=CommandOut, tags=["control"], summary="Set the twist (holds for 0.25 s; resend to keep moving)")
    async def command(body: CommandIn, token: LeaseToken = None):
        """The slow path: one command per request. It writes the same latest-command slot a future
        streaming fast path will write; the control tick applies whatever was written last."""
        return node.command(token, body.twist, body.seq, source="slow")

    @app.get("/limits", response_model=Limits, tags=["control"], summary="Hard and soft limits")
    async def get_limits():
        return node.limits()

    @app.post("/limits", response_model=Limits, tags=["control"], summary="Set soft limits within the hard limits (holder; reset when the lease ends)")
    async def set_limits(body: LimitsIn, token: LeaseToken = None):
        return node.set_limits(token, body.max_speed, body.max_omega)

    @app.post("/estop", response_model=EStopTriggered, tags=["control"], summary="E-stop: anyone; latches until the holder clears it")
    async def estop(request: Request, body: Optional[EStopIn] = None):
        body = body or EStopIn()
        src = body.source or (request.client.host if request.client else "")
        info = node.trigger_estop(body.reason, src)
        return {"latched": True, "reason": info.reason, "source": info.source}

    @app.post("/estop/clear", response_model=EStopCleared, tags=["control"], summary="Clear the e-stop (holder; stays disabled)")
    async def clear_estop(token: LeaseToken = None):
        node.clear_estop(token)
        return {"latched": False}

    @app.get("/encoders", response_model=Encoders, tags=["encoders"], summary="Drive-encoder rebase state")
    async def get_encoders():
        return node.encoders()

    @app.post("/encoders/rebase", response_model=Encoders, tags=["encoders"],
              summary="Set the four drive encoders to 0 now (anyone; 409 unless stationary)")
    async def rebase_encoders():
        """Needs the base stationary for 0.5 s (409 with the reason otherwise). Waits for
        fresh STATUS_2 frames to confirm |drive_pos| ~ 0 on all four drives, then `epoch` increments (503 if
        they don't confirm within 0.5 s). Moves nothing, so any client may call it; a lease holder's command
        that arrives during the ~20-30 ms in flight is held at zero until it confirms."""
        return await asyncio.to_thread(node.rebase)

    @app.get("/encoders/auto", response_model=AutoRebase, tags=["encoders"], summary="Automatic rebase policy")
    async def get_auto_rebase():
        return node.encoders()["auto"]

    @app.post("/encoders/auto", response_model=AutoRebase, tags=["encoders"],
              summary="Change the automatic rebase policy (anyone; runtime only)")
    async def set_auto_rebase(body: AutoRebaseIn):
        return node.set_auto_rebase(body.enabled, body.interval_s, body.still_s)

    @app.post("/reset", response_model=ResetOut, tags=["control"], summary="Rebuild the drive after a loop error (holder; ends disabled)")
    async def reset(token: LeaseToken = None):
        node.lease.check(token)
        await asyncio.to_thread(node.reset, token)
        return {"state": node.status, "error": node.drive_error}

    return app
