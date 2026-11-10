"""HTTP API of the power node (FastAPI). Conventions: docs/nodes.md; guide: docs/power.md."""
from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from typing import Annotated, Callable, Optional, Union

from fastapi import FastAPI, Query, Request
from fastapi.responses import HTMLResponse, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field, WithJsonSchema

from ..common.server import Event, add_base_routes, add_page_routes, node_meta, setup_app, sse
from . import __version__
from .node import PowerNode
from .node import HISTORY_S
from .shutdown import CONFIRM, DELAY_DEFAULT_S, DELAY_MAX_S, DELAY_MIN_S
from .sources import HISTORY_FIELDS, STALE_S

TAGS = [
    {"name": "state", "description": "Sources (UPS, main), low-voltage status, the pending shutdown, events."},
    {"name": "shutdown", "description": "Remote shutdown of the Pi: confirm string, countdown, cancel."},
    {"name": "config", "description": "Low-voltage auto-shutdown settings."},
]


class ShutdownIn(BaseModel):
    confirm: Optional[str] = Field(None, description=f'Must be exactly "{CONFIRM}".')
    reason: str = Field("", description="Why (required; shown to everyone and logged).")
    by: str = Field("", description="Your name (logged next to the identity the robot resolves).")
    delay_s: float = Field(DELAY_DEFAULT_S, description=f"Countdown, {DELAY_MIN_S:.0f}..{DELAY_MAX_S:.0f} s.")
    model_config = ConfigDict(json_schema_extra={"examples": [
        {"confirm": CONFIRM, "reason": "end of day", "by": "avery", "delay_s": 30}]})


class CancelIn(BaseModel):
    by: str = ""
    reason: str = ""


class LowVoltageIn(BaseModel):
    enabled: bool
    volts: float = Field(description="Pack threshold, V (11-16; 13.2 = 3.3 V per cell).")
    hold_s: Optional[float] = Field(None, description="Seconds the pack must stay low (5-120).")
    confirm: bool = Field(False, description="Needed to enable while the pack is already below the threshold.")
    by: str = ""


# -- responses ---------------------------------------------------------------------------------

# An int or a float, passed through as is (values from a settings file, or mixed-type rows).
Number = Annotated[Union[int, float], WithJsonSchema({"type": "number"})]


class UpsInput(BaseModel):
    present: bool = Field(description="USB-C input power present (VBUS).")
    v: float = Field(description="V, input voltage.")
    a: float = Field(description="A, input current.")
    w: float = Field(description="W, input power.")


class UpsLinks(BaseModel):
    fuel_gauge: bool = Field(description="The HAT's MCU talks to the BQ4050 fuel gauge.")
    charger: bool = Field(description="The HAT's MCU talks to the IP2368 charger.")


class UpsControl(BaseModel):
    """Registers 0x40-0x43 as read (never written)."""
    auto_power_on: bool = Field(description="Power the Pi back on when input returns (0x40 bit 0).")
    watchdog: bool = Field(description="Watchdog enabled (0x40 bit 1).")
    address: Optional[int] = Field(description="I2C address the HAT answers on (0x41).")
    wd_timeout_s: Optional[int] = Field(description="s, watchdog timeout (0x42).")
    wd_delay_s: Optional[int] = Field(description="s, watchdog start-up delay (0x43).")


class PowerSource(BaseModel):
    """One power source (sources.py). Every source has the envelope (id .. age_s); a tether adds
    `note`; the UPS adds `model`, then, once it has a good reading, the common battery fields
    (percent .. remaining_mah; null when unknown) and the UPS-only ones. Keys a source does not
    have are absent, not null."""
    id: str = Field(description="Stable source id: ups | main | (later) aux.")
    type: str = Field(description="ups | tether | (later) battery.")
    role: str = Field(description="What it powers: pi (keeps the Pi up) | main (the robot) | aux.")
    telemetry: bool = Field(description="False when there is nothing to read (a bench tether).")
    ok: Optional[bool] = Field(description="The last read succeeded and is fresh; null without telemetry.")
    stale: bool = Field(description=f"The last good reading is older than {STALE_S:.0f} s.")
    error: Optional[str] = Field(description="The last read error; null if none.")
    age_s: Optional[float] = Field(description="s since the last good reading (0.1 s resolution); null if none.")
    note: Optional[str] = Field(None, description="Tether only: what it is.")
    model: Optional[str] = Field(None, description="UPS only: the hardware.")
    percent: Optional[int] = Field(None, description="%, state of charge (fuel gauge).")
    pack_v: Optional[float] = Field(None, description="V, pack voltage.")
    cells_v: Optional[list[float]] = Field(None, description="V, each cell.")
    current_a: Optional[float] = Field(None, description="A, pack current (+ = charging).")
    power_w: Optional[float] = Field(None, description="W, pack power (+ = charging).")
    charge_state: Optional[str] = Field(None, description="charging | full | discharging | idle.")
    charging: Optional[bool] = Field(None, description="The charger is charging.")
    fast_charge: Optional[bool] = Field(None, description="Fast charge active.")
    input_present: Optional[bool] = Field(None, description="Input power present.")
    runtime_min: Optional[int] = Field(None, description="min to empty while discharging; null otherwise.")
    runtime_source: Optional[str] = Field(None, description="gauge (the fuel gauge's time to empty) | estimate "
                                                            "(remaining mAh / current); null without a runtime.")
    time_to_full_min: Optional[int] = Field(None, description="min to full while charging; null otherwise.")
    remaining_mah: Optional[int] = Field(None, description="mAh, remaining capacity.")
    input: Optional[UpsInput] = Field(None, description="UPS only: USB-C input.")
    charger_state: Optional[str] = Field(None, description="UPS only: IP2368 state (standby | trickle charge | "
                                                           "constant current | constant voltage | waiting | full | "
                                                           "timeout | undefined).")
    links: Optional[UpsLinks] = Field(None, description="UPS only: the MCU's I2C links.")
    firmware: Optional[str] = Field(None, description="UPS only: HAT firmware version, e.g. `2.0`.")
    power_off_pending: Optional[bool] = Field(None, description="UPS only: the power-off trigger is armed (0x01 reads "
                                                                "0x55): power is cut ~30 s after it was written.")
    control: Optional[UpsControl] = Field(None, description="UPS only: power-on and watchdog registers.")


class LowVoltageOut(BaseModel):
    """Low-voltage auto-shutdown settings and status (lowvolt.py)."""
    enabled: bool = Field(description="Shut down when the pack stays low while not charging.")
    volts: Number = Field(description="V, pack threshold.")
    hold_s: Number = Field(description="s the pack must stay below the threshold.")
    status: str = Field(description="off | armed | no data: not acting | pack low: shutdown in N s unless it "
                                    "recovers | triggered: shutdown scheduled | shutdown cancelled; re-arms in N s "
                                    "if still low.")
    low: bool = Field(description="Enabled, and the last reading is below the threshold and not charging.")
    pack_v: Optional[float] = Field(description="V, the last pack reading the check saw; null if none (no fresh, "
                                                "valid reading).")
    range_v: list[float] = Field(description="V, [min, max] allowed for `volts`.")
    range_hold_s: list[float] = Field(description="s, [min, max] allowed for `hold_s`.")
    first_run: bool = Field(description="No settings saved yet (off by default until the cutover, docs/power.md).")
    source: str = Field(description="The source checked (`ups`).")


class EStopResult(BaseModel):
    """One node's result of the shutdown's e-stop fan-out (frontend.app.estop_node)."""
    name: str = Field(description="Node instance name (`*` if the fan-out itself failed).")
    url: Optional[str] = Field(None, description="The e-stop URL posted to; null if none was.")
    status: str = Field(description="latched | skipped (not actuated) | failed | unreachable.")
    detail: Optional[str] = Field(None, description="Why, if not latched.")


class DelayBounds(BaseModel):
    default: float = Field(description="s.")
    min: float = Field(description="s.")
    max: float = Field(description="s.")


class LastShutdown(BaseModel):
    """The last shutdown that ended."""
    phase: Optional[str] = Field(description="cancelled | done | failed.")
    by: Optional[str] = Field(description="The name the requester typed.")
    client: Optional[str] = Field(description="The identity the node resolved for the requester.")
    source: Optional[str] = Field(description="web | local | low_voltage.")
    reason: Optional[str]
    requested_at: Optional[float] = Field(description="Unix epoch s.")
    at: Optional[float] = Field(description="Unix epoch s the power-off was due.")
    cancelled_by: Optional[str] = Field(description="Typed name of who cancelled it; null if not cancelled.")
    cancelled_client: Optional[str] = Field(description="Resolved identity of who cancelled it; null if not cancelled.")
    cancelled_at: Optional[float] = Field(description="Unix epoch s; null if not cancelled.")
    error: Optional[str] = Field(description="Why the power-off failed; null otherwise.")


class ShutdownState(BaseModel):
    """The shutdown: one flat shape whether or not one is pending. Every field except `pending`,
    `phase`, `confirm`, `delay_bounds_s` and `last` is null when idle."""
    pending: bool = Field(description="A shutdown is scheduled or under way.")
    phase: str = Field(description="idle | countdown | executing (e-stopping) | powering_off.")
    by: Optional[str] = Field(description="The name the requester typed (`?` if none).")
    client: Optional[str] = Field(description="The identity the node resolved (Tailscale identity, IP, or "
                                              "`local (unix socket)`; `power node` for the low-voltage monitor).")
    source: Optional[str] = Field(description="web | local | low_voltage.")
    reason: Optional[str]
    delay_s: Optional[float] = Field(description="s, the countdown requested.")
    requested_at: Optional[float] = Field(description="Unix epoch s.")
    at: Optional[float] = Field(description="Unix epoch s the power-off is due.")
    estop: Optional[list[EStopResult]] = Field(description="Per-node e-stop results once the countdown ended; "
                                                           "null before.")
    error: Optional[str] = Field(description="Why the power-off failed.")
    in_s: Optional[float] = Field(description="s until the deadline (0.1 s resolution; 0 once past).")
    confirm: str = Field(description="The confirm string POST /shutdown needs.")
    delay_bounds_s: DelayBounds = Field(description="Countdown default and bounds.")
    last: Optional[LastShutdown] = Field(description="The last shutdown that ended; null if none since the node "
                                                     "started.")


class AuditRecord(BaseModel):
    """One audit-log entry (shutdown.jsonl). Open-ended: the other keys depend on `event` (by, client,
    source, reason, status, detail, delay_s, results, error, before, after, ...)."""
    wall: float = Field(description="Unix epoch s (1 ms resolution).")
    time: str = Field(description="Local time, `%Y-%m-%d %H:%M:%S %z`.")
    event: str = Field(description="requested | refused | cancelled | cancel_refused | executing | estopped | "
                                   "powered_off | failed | low_voltage.")
    model_config = ConfigDict(extra="allow")


class ShutdownWithLog(ShutdownState):
    log: list[AuditRecord] = Field(description="The last `n` audit-log entries, oldest first.")


class AuditLogOut(BaseModel):
    log: list[AuditRecord] = Field(description="The last `n` audit-log entries, oldest first.")


class HistoryOut(BaseModel):
    source: str = Field(description="The source id.")
    fields: list[str] = Field(description=f"Column names of `rows`: {', '.join(HISTORY_FIELDS)}.")
    interval_s: float = Field(description="s between rows (while the source has fresh readings).")
    now: float = Field(description="Server time.monotonic() (s) when this was built (the clock of the `t` column).")
    units: list[str] = Field(description="Unit of each column.")
    rows: list[list[Number]] = Field(description=f"Oldest first, one per {HISTORY_S:.0f} s within `seconds`: "
                                                 "t (server monotonic s), wall (Unix epoch s), percent, pack_v (V), "
                                                 "current_a (A, + = charging), input_present (0/1), charging (0/1).")


class PowerState(BaseModel):
    state: str = Field(description="Short status for the landing tile, e.g. `UPS 59 % · charging`, `SHUTDOWN in "
                                   "22 s`, `UPS stale`, `UPS unavailable`, `no UPS`.")
    error: Optional[str] = Field(description="The UPS read error; null if fine.")
    fake: bool = Field(description="Simulated UPS (--fake): never powers anything off.")
    t: float = Field(description="Server time.monotonic() (s) when this state was built.")
    sources: list[PowerSource]
    low_voltage: LowVoltageOut
    shutdown: ShutdownState
    events: list[Event] = Field(description="Latest node events (up to 15), oldest first.")


def default_identify() -> Callable[[str], str]:
    """IP -> label, with the frontend's Tailscale identities (``tailscale whois``, cached)."""
    ids = None

    def identify(ip: str) -> str:
        nonlocal ids
        try:
            if ids is None:
                from ...frontend.observe import Identities
                ids = Identities()
            return ids.get(ip)["label"]
        except Exception:  # noqa: BLE001 - identity is best effort
            return ip
    return identify


def create_app(node: PowerNode, start: bool = True, identify: Optional[Callable[[str], str]] = None) -> FastAPI:
    identify = identify or default_identify()

    @asynccontextmanager
    async def lifespan(app):
        node.hub.attach(asyncio.get_running_loop())
        if start:
            node.start()
        yield
        node.close()

    app = FastAPI(
        title="Power node",
        description=(
            "Power for the robot. Today: the Pi's **UPS HAT (E)** (battery, input, charger) and the main "
            "supply, a bench **tether** with no telemetry. Later: main and auxiliary batteries, as more "
            "entries in `sources`.\n\n"
            "**Remote shutdown** (`POST /shutdown`) needs the exact confirm string `\"shutdown\"` and a reason, "
            f"counts down ({DELAY_MIN_S:.0f}-{DELAY_MAX_S:.0f} s, default {DELAY_DEFAULT_S:.0f}), can be cancelled "
            "(`POST /shutdown/cancel`), then e-stops every actuated node and powers the Pi off. The robot is "
            "then unreachable until input power returns or someone power-cycles it. Every request is logged "
            "with the requester's identity (docs/power.md)."),
        version=__version__, lifespan=lifespan, openapi_tags=TAGS)
    setup_app(app)
    add_base_routes(app, lambda: {"node": "power", "state": node.state()["state"]})
    add_page_routes(app, node_meta("power", "Power", __version__, "power", actuated=False, fake=node.fake))

    async def who(request: Request) -> tuple[str, str, Optional[str]]:
        """(client label, source, rate-limit key). Through nginx (or the landing page's proxy): the
        LAST X-Forwarded-For hop, the address the proxy itself saw (earlier hops come from the client
        and can be forged), resolved to a Tailscale identity. Straight on the unix socket (the SSH
        CLI, scripts on the robot): local."""
        ip = request.headers.get("x-forwarded-for", "").split(",")[-1].strip()
        if ip:
            return await asyncio.to_thread(identify, ip), "web", ip
        host = request.client.host if request.client else ""
        if host:                                   # the debug TCP port (--port)
            return await asyncio.to_thread(identify, host), "web", host
        return "local (unix socket)", "local", None

    @app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def index():
        from .page import PAGE
        return PAGE

    @app.get("/state", response_model=PowerState, response_model_exclude_unset=True, tags=["state"],
             summary="Sources, low-voltage status, pending shutdown, recent events")
    async def get_state():
        return node.state()

    @app.get("/events", tags=["state"], summary="Server-Sent Events: /state on every change and every 1 s",
             response_class=StreamingResponse,
             responses={200: {"content": {"text/event-stream": {}},
                              "description": "Unnamed events; each `data:` line is the GET /state JSON (PowerState)."}})
    async def events():
        return StreamingResponse(sse(node.hub, node.state), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-store"})

    @app.get("/history", response_model=HistoryOut, tags=["state"],
             summary="Recent readings of a source (one row per 30 s, up to 24 h)")
    async def history(source: str = Query("ups"), seconds: float = Query(86400, gt=0, le=86400)):
        return node.history_rows(source, seconds)

    @app.get("/shutdown", response_model=ShutdownWithLog, response_model_exclude_unset=True, tags=["shutdown"],
             summary="The pending shutdown (if any) and the audit log")
    async def get_shutdown(n: int = Query(30, ge=1, le=100)):
        return {**node.shutdown.public(), "log": node.audit.list(n)}

    @app.post("/shutdown", response_model=ShutdownState, response_model_exclude_unset=True, tags=["shutdown"],
              status_code=202,
              summary='Schedule a shutdown of the Pi (needs confirm: "shutdown")')
    async def post_shutdown(request: Request, body: ShutdownIn = ShutdownIn()):
        """Refused (and logged) without the exact confirm string (400), without a reason (422), with a
        countdown out of range (422), while another shutdown is pending (409), or too often from one
        client (429). At the deadline: e-stop every actuated node, arm the UPS power-off, `systemctl
        poweroff`."""
        client, source, key = await who(request)
        return node.shutdown.request(confirm=body.confirm, reason=body.reason, by=body.by, client=client,
                                     source=source, delay_s=body.delay_s, rate_key=key)

    @app.post("/shutdown/cancel", response_model=ShutdownState, response_model_exclude_unset=True, tags=["shutdown"],
              summary="Cancel the pending shutdown (during the countdown)")
    async def cancel_shutdown(request: Request, body: CancelIn = CancelIn()):
        client, source, _ = await who(request)
        return node.shutdown.cancel(by=body.by, client=client, source=source, reason=body.reason)

    @app.get("/low_voltage", response_model=LowVoltageOut, tags=["config"],
             summary="Low-voltage auto-shutdown settings and status")
    async def get_low_voltage():
        return node.low.public(node.clock())

    @app.post("/low_voltage", response_model=LowVoltageOut, tags=["config"],
              summary="Change the low-voltage auto-shutdown settings")
    async def post_low_voltage(request: Request, body: LowVoltageIn):
        """Off by default until the old ups-dashboard service is retired (docs/power.md, Cutover).
        Enabling while the pack is already below the threshold and not charging needs `confirm: true`."""
        client, _, _ = await who(request)
        return node.set_low_voltage(body.enabled, body.volts, body.hold_s, body.confirm,
                                    (body.by or "?").strip()[:80], client)

    @app.get("/log", response_model=AuditLogOut, response_model_exclude_unset=True, tags=["state"],
             summary="Audit log: shutdown requests, cancellations, outcomes, settings")
    async def get_log(n: int = Query(50, ge=1, le=100)):
        return {"log": node.audit.list(n)}

    return app
