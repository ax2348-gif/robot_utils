"""HTTP API of the power node (FastAPI). Conventions: docs/nodes.md; guide: docs/power.md."""
from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from typing import Callable, Optional

from fastapi import FastAPI, Query, Request
from fastapi.responses import HTMLResponse, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field

from ..common.server import add_base_routes, add_page_routes, node_meta, setup_app, sse
from . import __version__
from .node import PowerNode
from .shutdown import CONFIRM, DELAY_DEFAULT_S, DELAY_MAX_S, DELAY_MIN_S

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

    @app.get("/state", tags=["state"], summary="Sources, low-voltage status, pending shutdown, recent events")
    async def get_state():
        return node.state()

    @app.get("/events", tags=["state"], summary="Server-Sent Events: /state on every change and every 1 s",
             response_class=StreamingResponse)
    async def events():
        return StreamingResponse(sse(node.hub, node.state), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-store"})

    @app.get("/history", tags=["state"], summary="Recent readings of a source (one row per 30 s, up to 24 h)")
    async def history(source: str = Query("ups"), seconds: float = Query(86400, gt=0, le=86400)):
        return node.history_rows(source, seconds)

    @app.get("/shutdown", tags=["shutdown"], summary="The pending shutdown (if any) and the audit log")
    async def get_shutdown(n: int = Query(30, ge=1, le=100)):
        return {**node.shutdown.public(), "log": node.audit.list(n)}

    @app.post("/shutdown", tags=["shutdown"], status_code=202,
              summary='Schedule a shutdown of the Pi (needs confirm: "shutdown")')
    async def post_shutdown(request: Request, body: ShutdownIn = ShutdownIn()):
        """Refused (and logged) without the exact confirm string (400), without a reason (422), with a
        countdown out of range (422), while another shutdown is pending (409), or too often from one
        client (429). At the deadline: e-stop every actuated node, arm the UPS power-off, `systemctl
        poweroff`."""
        client, source, key = await who(request)
        return node.shutdown.request(confirm=body.confirm, reason=body.reason, by=body.by, client=client,
                                     source=source, delay_s=body.delay_s, rate_key=key)

    @app.post("/shutdown/cancel", tags=["shutdown"], summary="Cancel the pending shutdown (during the countdown)")
    async def cancel_shutdown(request: Request, body: CancelIn = CancelIn()):
        client, source, _ = await who(request)
        return node.shutdown.cancel(by=body.by, client=client, source=source, reason=body.reason)

    @app.get("/low_voltage", tags=["config"], summary="Low-voltage auto-shutdown settings and status")
    async def get_low_voltage():
        return node.low.public(node.clock())

    @app.post("/low_voltage", tags=["config"], summary="Change the low-voltage auto-shutdown settings")
    async def post_low_voltage(request: Request, body: LowVoltageIn):
        """Off by default until the old ups-dashboard service is retired (docs/power.md, Cutover).
        Enabling while the pack is already below the threshold and not charging needs `confirm: true`."""
        client, _, _ = await who(request)
        return node.set_low_voltage(body.enabled, body.volts, body.hold_s, body.confirm,
                                    (body.by or "?").strip()[:80], client)

    @app.get("/log", tags=["state"], summary="Audit log: shutdown requests, cancellations, outcomes, settings")
    async def get_log(n: int = Query(50, ge=1, le=100)):
        return {"log": node.audit.list(n)}

    return app
