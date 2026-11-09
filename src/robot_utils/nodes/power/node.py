"""Power node core: polls the sources, keeps a history, runs low-voltage and shutdown logic.

One owner thread (``step`` every ``TICK_S``) is the only code that reads the hardware: it polls
every source each ``POLL_S``, runs the low-voltage check, appends a history row per source with
telemetry every ``HISTORY_S``, and fires the shutdown at its deadline. No web code here
(server.py serves it).
"""
from __future__ import annotations

import asyncio
import logging
import threading
import time
from collections import deque
from pathlib import Path
from typing import Callable, Optional

from ..common.hub import RowHub
from ..common.lease import RequestError
from ..common.server import EventLog
from .lowvolt import LowVoltage
from .lowvolt import default_path as default_settings_path
from .shutdown import CONFIRM, DELAY_MIN_S, AuditLog, ShutdownController, default_state_dir, refuse_power_off
from .sources import HISTORY_FIELDS, Source, UpsSource

log = logging.getLogger("power")

TICK_S = 0.25
POLL_S = 1.0
HISTORY_S = 30.0
HISTORY_LEN = int(24 * 3600 / HISTORY_S)      # 24 h


def estop_robot(config: Optional[str] = None, exclude: tuple = ()) -> list[dict]:
    """E-stop every actuated node, like the landing page's E-STOP ALL (frontend.app.estop_all):
    the nodes in nodes.yaml plus any other node socket on this machine. Blocking; one result per node."""
    from ...frontend.app import NodeLinks, estop_all
    from ...frontend.config import NodeEntry, load_config
    from ..common.serve import NAME, socket_dir
    try:
        nodes = load_config(config)
    except Exception as e:  # noqa: BLE001 - a broken registry must not stop the e-stop of what we can find
        log.error("e-stop: cannot read the node registry (%s); using the sockets only", e)
        nodes = []
    names = {n.name for n in nodes}
    for sock in sorted(socket_dir().glob("*.sock")):
        if sock.stem not in names and sock.stem != "frontend" and NAME.match(sock.stem):
            nodes.append(NodeEntry(name=sock.stem))
    nodes = [n for n in nodes if n.name not in exclude]

    async def run():
        links = NodeLinks()
        try:
            return await estop_all(links, nodes)
        finally:
            await links.aclose()
    return asyncio.run(run())


def system_power_off(ups) -> Callable[[], None]:
    """The real power-off (robot only, never --fake): arm the HAT (0x01 <- 0x55: power is cut in
    ~30 s, irreversible), then ``systemctl poweroff`` (allowed for this user by polkit, no sudo)."""
    import subprocess

    def power_off() -> None:
        armed = None
        if ups is not None:
            try:
                ups.arm_power_off()
                armed = True
            except OSError as e:
                armed = False
                log.error("could not arm the UPS power-off (%s); shutting the OS down anyway", e)
        done = subprocess.run(["systemctl", "poweroff"], capture_output=True, text=True, timeout=30)
        if done.returncode != 0:
            raise RuntimeError(("the HAT will cut power in ~30 s, but " if armed else "") +
                               "systemctl poweroff failed: " + (done.stderr.strip() or f"exit {done.returncode}"))
    return power_off


class PowerNode:
    def __init__(self, sources: list[Source], *, estop: Optional[Callable[[], list]] = None,
                 power_off: Callable[[], None] = refuse_power_off, clock: Callable[[], float] = time.monotonic,
                 settings_path: Optional[Path] = None, audit_path: Optional[Path] = None,
                 spawn=None, fake: bool = False, name: str = "power", config: Optional[str] = None):
        self.sources = sources
        self.clock, self.fake, self.name = clock, fake, name
        self.hub = RowHub(1)
        self.events = EventLog(50, on_add=self.hub.state_changed)
        self.audit = AuditLog(audit_path)
        self.shutdown = ShutdownController(
            estop or (lambda: estop_robot(config, exclude=(self.name,))), power_off,
            audit=self.audit, clock=clock, spawn=spawn, on_change=self.hub.state_changed,
            on_event=lambda msg, level: self.events.add(msg, level))
        self.low = LowVoltage(settings_path, self._low_trigger)
        self.history = {s.name: deque(maxlen=HISTORY_LEN) for s in sources if s.telemetry}
        self._lock = threading.Lock()
        self._last_poll = self._last_hist = -1e18
        self._prev_err: dict[str, Optional[str]] = {}
        self._was_low = False
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        if self.low.first_run:
            self.events.add("low-voltage shutdown is off (first run): the old ups-dashboard service still "
                            "handles it until the cutover (docs/power.md)")

    @classmethod
    def default_paths(cls) -> dict:
        return {"settings_path": default_settings_path(), "audit_path": default_state_dir() / "shutdown.jsonl"}

    @property
    def ups(self) -> Optional[UpsSource]:
        return next((s for s in self.sources if isinstance(s, UpsSource)), None)

    # -- owner thread --------------------------------------------------------------------------

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="power", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self.step(self.clock())
            except Exception:  # noqa: BLE001
                log.exception("power step failed")
            self._stop.wait(TICK_S)

    def step(self, now: float) -> None:
        if now - self._last_poll >= POLL_S:
            self._last_poll = now
            for s in self.sources:
                s.poll(now)
                err = getattr(s, "error", None)
                if err != self._prev_err.get(s.name, None):
                    if err:
                        self.events.add(f"{s.name}: {err}", "error")
                    elif s.name in self._prev_err:
                        self.events.add(f"{s.name}: reading again")
                    self._prev_err[s.name] = err
            ups = self.ups
            bat = ups.battery() if ups is not None and ups.fresh(now) else None
            self.low.check(now, bat, self.shutdown.pending is not None)
            low = self.low.low_since is not None and self.low.triggered_at is None
            if low and not self._was_low:
                st = self.low.settings
                self.events.add(f"pack {bat['pack_mv'] / 1000:.2f} V below {st['volts']:.2f} V and not charging: "
                                f"low-voltage shutdown in {st['hold_s']:.0f} s unless it recovers", "warn")
            self._was_low = low
            if now - self._last_hist >= HISTORY_S:
                self._last_hist = now
                wall = time.time()
                with self._lock:
                    for s in self.sources:
                        row = s.history_row(now, wall)
                        if row is not None and s.name in self.history:
                            self.history[s.name].append(row)
            self.hub.state_changed()
        self.shutdown.tick(now)

    def _low_trigger(self, pack_mv: int) -> None:
        st = self.low.settings
        try:
            self.shutdown.request(confirm=CONFIRM, by="low-voltage monitor", client="power node", source="low_voltage",
                                  reason=f"low voltage: pack {pack_mv / 1000:.2f} V below {st['volts']:.2f} V for "
                                         f"{st['hold_s']:.0f} s while not charging", delay_s=DELAY_MIN_S)
        except RequestError as e:
            log.warning("low-voltage shutdown not scheduled: %s", e)

    # -- web side ------------------------------------------------------------------------------

    def set_low_voltage(self, enabled: bool, volts: float, hold_s: Optional[float], confirm: bool,
                        by: str, client: str) -> dict:
        now = self.clock()
        ups = self.ups
        bat = ups.battery() if ups is not None and ups.fresh(now) else None
        before = dict(self.low.settings)
        out = self.low.update(enabled, volts, hold_s, confirm, bat)
        after = self.low.settings
        msg = (f"low-voltage shutdown {'ON' if after['enabled'] else 'off'} at {after['volts']:.2f} V, "
               f"hold {after['hold_s']:.0f} s, by {by} [{client}]")
        log.warning(msg)
        self.audit.write("low_voltage", by=by, client=client, before=before, after=after)
        self.events.add(msg, "warn" if after["enabled"] != before["enabled"] else "info")
        return out

    def backfill(self, source: str, rows: list[list]) -> None:
        with self._lock:
            self.history[source].extend(rows)

    def history_rows(self, source: str, seconds: float) -> dict:
        if source not in self.history:
            raise RequestError(404, f"no history for {source!r}; sources with telemetry: {sorted(self.history)}")
        now = self.clock()
        with self._lock:
            rows = [r for r in self.history[source] if now - r[0] <= seconds]
        return {"source": source, "fields": HISTORY_FIELDS, "interval_s": HISTORY_S, "now": now,
                "units": ["s (monotonic)", "s (epoch)", "%", "V", "A (+ charging)", "0/1", "0/1"], "rows": rows}

    def state(self) -> dict:
        now = self.clock()
        sources = [s.public(now) for s in self.sources]
        sd = self.shutdown.public(now)
        ups = next((s for s in sources if s["type"] == "ups"), None)
        if sd["pending"]:
            st = f"SHUTDOWN in {sd['in_s']:.0f} s" if sd["phase"] == "countdown" else f"shutting down: {sd['phase']}"
        elif ups is None:
            st = "no UPS"
        elif ups["ok"]:
            st = f"UPS {ups['percent']} % · {ups['charge_state']}"
        elif ups["stale"]:
            st = "UPS stale"
        else:
            st = "UPS unavailable"
        return {"state": st, "error": ups["error"] if ups else None, "fake": self.fake, "t": now,
                "sources": sources, "low_voltage": self.low.public(now), "shutdown": sd,
                "events": self.events.list()[-15:]}
