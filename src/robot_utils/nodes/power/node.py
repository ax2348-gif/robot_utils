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
from .cpu import CpuGovernor
from .lowvolt import LowVoltage
from .lowvolt import default_path as default_settings_path
from .powerlog import CELL_LOW_V, SPREAD_V, DiskLog, Watch
from .powerloss import KEEP_S as LOSS_KEEP_S
from .powerloss import PowerLoss, sample_of, sample_of_log
from .shutdown import CONFIRM, DELAY_MIN_S, AuditLog, ShutdownController, default_state_dir, refuse_power_off
from .sources import DRAIN_A, HISTORY_FIELDS, HISTORY_UNITS, Source, UpsSource, flow

log = logging.getLogger("power")

TICK_S = 0.25
POLL_S = 1.0
HISTORY_S = 30.0
HISTORY_LEN = int(24 * 3600 / HISTORY_S)      # 24 h
ESTIMATE_S = 5.0                               # power-loss estimate refresh
CONDITIONS_IN_STATE = 10                       # the latest Watch spans in /state
# what the page marks as anomaly spans on the history charts (the same thresholds as powerlog.Watch, no hold)
HISTORY_LIMITS = {"drain_a": DRAIN_A, "cell_low_v": CELL_LOW_V, "spread_v": SPREAD_V}


def _num(r: dict, k: str) -> Optional[float]:
    try:
        return float(r[k])
    except (KeyError, TypeError, ValueError):
        return None


def _dur_min(m: float) -> str:
    if m < 1:
        return "< 1 min"
    m = round(m)
    return f"{m} min" if m < 60 else f"{m // 60} h {m % 60:02d} min"


def log_history_row(t: float, r: dict) -> list:
    """One HISTORY_FIELDS row from a disk-log CSV row (``DiskLog.recent``). Input power is input_v x input_a
    (the log has no input W column); a value the row lacks is None."""
    iv, ia, present = _num(r, "input_v"), _num(r, "input_a"), _num(r, "input_present")
    input_w = iv * ia if iv is not None and ia is not None else 0.0 if present == 0 else None
    return [t, float(r["wall"]), int(r["percent"]), float(r["pack_v"]), float(r["current_a"]),
            float(r["input_present"]), float(r["charging"]),
            *flow(input_w, _num(r, "power_w"), [_num(r, f"cell{i}_v") for i in range(1, 5)])]


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
                 spawn=None, fake: bool = False, name: str = "power", config: Optional[str] = None,
                 log_dir: Optional[Path] = None, cpu: Optional[CpuGovernor] = None):
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
        self.disklog = DiskLog(log_dir) if log_dir is not None else None     # every UPS poll, on disk
        self.cpu = cpu                                  # the CPU governor toggle (cpu.py); None: not offered
        self._cpu_state: Optional[dict] = None
        self._cpu_t = -1e18
        self.watch = Watch(self._note)
        self.loss = PowerLoss()                        # time until the Pi loses power (powerloss.py)
        self.power_loss = self.loss.estimate(time.time())
        self._last_est = -1e18
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
        return {"settings_path": default_settings_path(), "audit_path": default_state_dir() / "shutdown.jsonl",
                "log_dir": default_state_dir() / "log"}

    @property
    def ups(self) -> Optional[UpsSource]:
        return next((s for s in self.sources if isinstance(s, UpsSource)), None)

    # -- owner thread --------------------------------------------------------------------------

    def start(self) -> None:
        self.apply_cpu_saved()
        self._thread = threading.Thread(target=self._run, name="power", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        if self.disklog is not None:
            self.disklog.close()

    def _note(self, msg: str, level: str = "info") -> None:
        """An event for the page that also goes to the journal (the page's events are in memory only)."""
        log.log({"error": logging.ERROR, "warn": logging.WARNING}.get(level, logging.INFO), msg)
        self.events.add(msg, level)

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
                        self._note(f"{s.name}: {err}", "error")
                    elif s.name in self._prev_err:
                        self._note(f"{s.name}: reading again")
                    self._prev_err[s.name] = err
            ups = self.ups
            if ups is not None and ups.ups is not None:            # a HAT to read (not: no I2C at all)
                d = ups.data if ups.t_ok == now else None        # this poll's reading, or None if it failed
                self.watch.update(now, d)
                wall = time.time()
                if d is not None:
                    self.loss.add(sample_of(wall, d))
                if self.disklog is not None:
                    self.disklog.write(wall, d, ups.error)
                if now - self._last_est >= ESTIMATE_S:
                    self._last_est = now
                    self.power_loss = self.loss.estimate(wall, self.low.settings, DELAY_MIN_S)
            bat = ups.battery() if ups is not None and ups.fresh(now) else None
            self.low.check(now, bat, self.shutdown.pending is not None)
            low = self.low.low_since is not None and self.low.triggered_at is None
            if low and not self._was_low:
                st = self.low.settings
                self._note(f"low voltage ({self.low.reason}): low-voltage shutdown in {st['hold_s']:.0f} s "
                           "unless it recovers", "warn")
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
                                  reason=f"low voltage: {self.low.reason} for {st['hold_s']:.0f} s",
                                  delay_s=DELAY_MIN_S)
        except RequestError as e:
            log.warning("low-voltage shutdown not scheduled: %s", e)

    # -- web side ------------------------------------------------------------------------------

    def set_low_voltage(self, enabled: bool, volts: float, hold_s: Optional[float], confirm: bool,
                        by: str, client: str, cell_volts: Optional[float] = None) -> dict:
        now = self.clock()
        ups = self.ups
        bat = ups.battery() if ups is not None and ups.fresh(now) else None
        before = dict(self.low.settings)
        out = self.low.update(enabled, volts, hold_s, confirm, bat, cell_volts)
        after = self.low.settings
        msg = (f"low-voltage shutdown {'ON' if after['enabled'] else 'off'} at pack {after['volts']:.2f} V / "
               f"cell {after['cell_volts']:.2f} V, hold {after['hold_s']:.0f} s, by {by} [{client}]")
        log.warning(msg)
        self.audit.write("low_voltage", by=by, client=client, before=before, after=after)
        self.events.add(msg, "warn" if after["enabled"] != before["enabled"] else "info")
        return out

    # -- CPU governor --------------------------------------------------------------------------------

    def apply_cpu_saved(self) -> None:
        """At startup: re-apply the saved governor (the kernel boots on its default); journal + page event."""
        if self.cpu is None:
            return
        try:
            msg = self.cpu.apply_saved()
        except Exception as e:  # noqa: BLE001 - a governor problem must not stop the power node
            msg = f"CPU governor: re-applying the saved choice failed: {e}"
        if msg:
            self._note(msg, "warn" if "could not" in msg or "failed" in msg else "info")

    def cpu_state(self, max_age_s: float = 1.0) -> Optional[dict]:
        """``cpu.read()``, at most once per ``max_age_s`` (sysfs reads; /state is built every second per client)."""
        if self.cpu is None:
            return None
        now = self.clock()
        if self._cpu_state is None or now - self._cpu_t >= max_age_s:
            self._cpu_state, self._cpu_t = self.cpu.read(), now
        return self._cpu_state

    def set_cpu_governor(self, governor: str, by: str, client: str) -> dict:
        if self.cpu is None:
            raise RequestError(404, "this power node has no CPU governor control")
        before = self.cpu.read().get("governor")
        try:
            out = self.cpu.set(governor)
        except RequestError as e:
            self.audit.write("cpu_governor", by=by, client=client, before=before, after=governor, status="failed",
                             error=str(e))
            self._note(f"CPU governor {before} -> {governor} by {by} [{client}] failed: {e}", "error")
            raise
        self._cpu_state, self._cpu_t = out, self.clock()
        self.audit.write("cpu_governor", by=by, client=client, before=before, after=out["governor"])
        self._note(f"CPU governor {before} -> {out['governor']} by {by} [{client}]")
        return out

    def restore_history(self) -> int:
        """Refill the UPS history (last 24 h, one row per HISTORY_S) from the disk log, e.g. after a reboot."""
        if self.disklog is None or "ups" not in self.history:
            return 0
        now, wall = self.clock(), time.time()
        rows = [log_history_row(now - (wall - float(r["wall"])), r)
                for r in self.disklog.recent(wall, HISTORY_LEN * HISTORY_S, HISTORY_S)]
        self.backfill("ups", rows)
        self.loss.extend(sample_of_log(r) for r in self.disklog.recent(wall, LOSS_KEEP_S))   # 1 Hz, for the estimate
        self.power_loss = self.loss.estimate(wall, self.low.settings, DELAY_MIN_S)
        return len(rows)

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
                "units": HISTORY_UNITS, "limits": HISTORY_LIMITS, "rows": rows}

    def power_loss_now(self, sd: dict) -> dict:
        """The cached estimate (``ESTIMATE_S``), or the pending shutdown itself (it comes first)."""
        loss = dict(self.power_loss)
        if sd["pending"]:
            in_s = sd["in_s"] or 0.0
            loss.update(status="shutdown_pending", event="shutdown", clean=True, in_min=round(in_s / 60, 1),
                        range_min=None, at=sd["at"], blocked_reason=None, confidence="high",
                        note=f"{sd['source']} shutdown by {sd['by']}: {sd['reason']}",
                        text=f"shutdown in {in_s:.0f} s ({sd['phase']})" if sd["phase"] == "countdown"
                        else f"shutting down: {sd['phase']}")
        return loss

    def state(self) -> dict:
        now = self.clock()
        sources = [s.public(now) for s in self.sources]
        sd = self.shutdown.public(now)
        ups = next((s for s in sources if s["type"] == "ups"), None)
        loss = self.power_loss_now(sd)
        if ups is not None and ups["telemetry"] and self.ups is not None and self.ups.ups is not None:
            ups["power_loss"] = loss
        if sd["pending"]:
            st = f"SHUTDOWN in {sd['in_s']:.0f} s" if sd["phase"] == "countdown" else f"shutting down: {sd['phase']}"
        elif ups is None:
            st = "no UPS"
        elif ups["ok"]:
            # the charger bit says "charging" while the pack drains on input: say what the pack does instead
            drain = self.watch.active.get("drain", False)
            st = f"UPS {ups['percent']} % · {'draining on input' if drain else ups['charge_state']}"
            if loss["status"] == "draining":         # the tile's one line: when the Pi goes down, and how
                st += f" · {'auto-shutdown' if loss['clean'] else 'power cut'} in ≈ {_dur_min(loss['in_min'])}"
        elif ups["stale"]:
            st = "UPS stale"
        else:
            st = "UPS unavailable"
        return {"state": st, "error": ups["error"] if ups else None, "fake": self.fake, "t": now,
                "sources": sources, "low_voltage": self.low.public(now), "shutdown": sd,
                "events": self.events.list()[-15:], "conditions": self.watch.spans()[-CONDITIONS_IN_STATE:],
                **({"cpu": self.cpu_state()} if self.cpu is not None else {})}
