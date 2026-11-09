"""Remote shutdown of the Pi: confirm string, countdown, cancel, e-stop first, then power off.

Losing the Pi loses the robot for everyone until someone power-cycles it, so a shutdown is
deliberate (docs/power.md, "Shutdown"):

1. ``request`` needs the exact confirm string ``"shutdown"`` and a reason; it schedules the
   shutdown ``delay_s`` (10-300 s, default 30) ahead. A second request while one is pending is
   refused (409); web clients are rate-limited (429).
2. ``cancel`` stops it at any time during the countdown.
3. At the deadline (``tick``) the executor runs, off the caller's thread:
   ``estop()`` (every actuated node, like the landing page's E-STOP ALL), then ``power_off()``.
   Both are injected: the node passes the real ones only on the robot without ``--fake``; the
   default ``refuse_power_off`` raises, so tests and fakes can never power anything off.

Every request (accepted or refused), cancellation and outcome goes to the Python log (the
journal), the node's event log, and an append-only JSON-lines audit file that survives the
shutdown (``~/.local/state/robot_utils/power/shutdown.jsonl``).
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time
from collections import deque
from pathlib import Path
from typing import Callable, Optional

from ..common.lease import RequestError

log = logging.getLogger("power.shutdown")

CONFIRM = "shutdown"
DELAY_DEFAULT_S = 30.0
DELAY_MIN_S = 10.0
DELAY_MAX_S = 300.0
MAX_REASON = 200
MAX_BY = 80
WEB_MIN_INTERVAL_S = 60.0       # per web client, between accepted requests
WEB_MAX_PER_HOUR = 5            # per web client


def default_state_dir() -> Path:
    base = os.environ.get("XDG_STATE_HOME", os.path.expanduser("~/.local/state"))
    return Path(base) / "robot_utils" / "power"


def refuse_power_off() -> None:
    """The default power-off: refuses. Only the node's __main__ (robot, no --fake) passes the real one."""
    raise RuntimeError("power-off is not wired in this process (test or --fake)")


class AuditLog:
    """Append-only JSON lines (fsynced), plus the last entries in memory."""

    def __init__(self, path: Optional[Path], keep: int = 100):
        self.path = Path(path) if path else None
        self.recent: deque = deque(maxlen=keep)
        self._lock = threading.Lock()
        if self.path and self.path.exists():
            try:
                for line in self.path.read_text().splitlines()[-keep:]:
                    self.recent.append(json.loads(line))
            except (OSError, ValueError):
                pass

    def write(self, event: str, **fields) -> dict:
        rec = {"wall": round(time.time(), 3), "time": time.strftime("%Y-%m-%d %H:%M:%S %z"), "event": event, **fields}
        with self._lock:
            self.recent.append(rec)
            if self.path:
                try:
                    self.path.parent.mkdir(parents=True, exist_ok=True)
                    with open(self.path, "a") as f:
                        f.write(json.dumps(rec, default=str) + "\n")
                        f.flush()
                        os.fsync(f.fileno())
                except OSError as e:
                    log.error("audit log %s: %s", self.path, e)
        return rec

    def list(self, n: int = 50) -> list[dict]:
        with self._lock:
            return list(self.recent)[-n:]


class RateLimit:
    """Per-client: at least ``min_interval`` s between accepted requests, at most ``per_hour``."""

    def __init__(self, min_interval: float = WEB_MIN_INTERVAL_S, per_hour: int = WEB_MAX_PER_HOUR):
        self.min_interval, self.per_hour = min_interval, per_hour
        self._log: dict[str, list[float]] = {}

    def check(self, client: str, now: float) -> None:
        times = [t for t in self._log.get(client, []) if now - t < 3600]
        self._log[client] = times
        if times and now - times[-1] < self.min_interval:
            raise RequestError(429, f"one shutdown request per {self.min_interval:.0f} s per client; "
                                    f"try again in {self.min_interval - (now - times[-1]):.0f} s")
        if len(times) >= self.per_hour:
            raise RequestError(429, f"{self.per_hour} shutdown requests per client per hour")

    def note(self, client: str, now: float) -> None:
        self._log.setdefault(client, []).append(now)


class ShutdownController:
    """The pending shutdown. Thread-safe; ``tick(now)`` runs from the node's owner thread.

    ``estop() -> list[dict]`` and ``power_off() -> None`` are injected; ``spawn(fn)`` runs the
    executor (a daemon thread by default; tests run it inline). ``clock`` is monotonic seconds.
    """

    def __init__(self, estop: Callable[[], list], power_off: Callable[[], None] = refuse_power_off, *,
                 audit: Optional[AuditLog] = None, clock: Callable[[], float] = time.monotonic,
                 spawn: Optional[Callable[[Callable[[], None]], None]] = None,
                 on_change: Optional[Callable[[], None]] = None,
                 on_event: Optional[Callable[[str, str], None]] = None,
                 rate: Optional[RateLimit] = None):
        self.estop, self.power_off = estop, power_off
        self.audit = audit or AuditLog(None)
        self.clock = clock
        self.spawn = spawn or (lambda fn: threading.Thread(target=fn, name="power-off", daemon=True).start())
        self.on_change = on_change or (lambda: None)
        self.on_event = on_event or (lambda msg, level: None)
        self.rate = rate or RateLimit()
        self._lock = threading.Lock()
        self.pending: Optional[dict] = None     # the scheduled / executing shutdown
        self.last: Optional[dict] = None        # the last one that ended (cancelled / done / failed)

    # -- helpers -----------------------------------------------------------------------------

    def _note(self, event: str, msg: str, level: str = "info", **fields) -> None:
        getattr(log, "warning" if level != "info" else "info")(msg)
        self.audit.write(event, **fields)
        self.on_event(msg, level)
        self.on_change()

    # -- API ---------------------------------------------------------------------------------

    def request(self, *, confirm, reason: str, by: str, client: str, source: str = "web",
                delay_s: float = DELAY_DEFAULT_S, rate_key: Optional[str] = None) -> dict:
        """Schedule a shutdown. Raises RequestError 400 / 422 / 409 / 429 (each refusal is logged)."""
        by = (by or "").strip()[:MAX_BY] or "?"
        reason = (reason or "").strip()[:MAX_REASON]
        who = dict(by=by, client=client, source=source, reason=reason)

        def refuse(status: int, why: str):
            self._note("refused", f"shutdown request refused ({why}): by {by} [{client}]", "warn", status=status,
                       detail=why, **who)
            raise RequestError(status, why)

        if confirm != CONFIRM:
            refuse(400, f'confirm must be exactly "{CONFIRM}"')
        if not reason:
            refuse(422, "a reason is required")
        try:
            delay_s = float(delay_s)
        except (TypeError, ValueError):
            refuse(422, "delay_s must be a number")
        if not DELAY_MIN_S <= delay_s <= DELAY_MAX_S:
            refuse(422, f"delay_s must be {DELAY_MIN_S:.0f}..{DELAY_MAX_S:.0f} s")
        now, wall = self.clock(), time.time()
        with self._lock:
            if self.pending is not None:
                p = self.pending
                busy = (f"a shutdown is already {'under way' if p['phase'] != 'countdown' else 'pending'} "
                        f"(by {p['by']}: {p['reason']})")
            else:
                busy = None
        if busy:
            refuse(409, busy)
        if rate_key is not None:
            try:
                self.rate.check(rate_key, wall)
            except RequestError as e:
                refuse(e.status, str(e))
        with self._lock:
            if self.pending is not None:            # raced another request
                busy = "a shutdown is already pending"
            else:
                if rate_key is not None:
                    self.rate.note(rate_key, wall)
                self.pending = {"phase": "countdown", **who, "delay_s": delay_s,
                                "requested_t": now, "requested_at": wall,
                                "deadline_t": now + delay_s, "at": wall + delay_s,
                                "estop": None, "error": None}
        if busy:
            refuse(409, busy)
        self._note("requested", f"SHUTDOWN scheduled in {delay_s:.0f} s by {by} [{client}, {source}]: {reason}",
                   "warn", delay_s=delay_s, **who)
        return self.public()

    def cancel(self, *, by: str, client: str, source: str = "web", reason: str = "") -> dict:
        by = (by or "").strip()[:MAX_BY] or "?"
        with self._lock:
            p = self.pending
            if p is None:
                err = (409, "no shutdown is pending")
            elif p["phase"] != "countdown":
                err = (409, f"too late: the shutdown is already {p['phase']}")
            else:
                err = None
                p.update(phase="cancelled", cancelled_by=by, cancelled_client=client, cancelled_at=time.time())
                self.last, self.pending = p, None
        if err:
            self._note("cancel_refused", f"shutdown cancel refused ({err[1]}): by {by} [{client}]", "warn",
                       by=by, client=client, source=source, detail=err[1])
            raise RequestError(*err)
        self._note("cancelled", f"shutdown cancelled by {by} [{client}, {source}]" + (f": {reason}" if reason else "")
                   + f" (requested by {p['by']}: {p['reason']})", "info", by=by, client=client, source=source,
                   reason=reason, requested_by=p["by"], requested_reason=p["reason"])
        return self.public()

    def tick(self, now: Optional[float] = None) -> None:
        """At the deadline, start the executor (once)."""
        now = self.clock() if now is None else now
        with self._lock:
            p = self.pending
            if p is None or p["phase"] != "countdown" or now < p["deadline_t"]:
                return
            p["phase"] = "executing"
        self._note("executing", f"shutdown countdown over: e-stopping the robot, then powering off "
                                f"(by {p['by']}: {p['reason']})", "warn", by=p["by"], client=p["client"],
                   reason=p["reason"])
        self.spawn(lambda: self._execute(p))

    def _execute(self, p: dict) -> None:
        try:
            results = list(self.estop() or [])
        except Exception as e:  # noqa: BLE001 - power still goes off; the failure is recorded
            results = [{"name": "*", "status": "failed", "detail": f"{type(e).__name__}: {e}"}]
        p["estop"] = results
        bad = [r for r in results if r.get("status") not in ("latched", "skipped")]
        summary = ", ".join(f"{r.get('name')}: {r.get('status')}" for r in results) or "no nodes"
        self._note("estopped", f"shutdown e-stop: {summary}", "warn" if bad else "info", results=results)
        with self._lock:
            p["phase"] = "powering_off"
        self.on_change()
        try:
            self.power_off()
        except Exception as e:  # noqa: BLE001
            with self._lock:
                p.update(phase="failed", error=f"{type(e).__name__}: {e}")
                self.last, self.pending = p, None
            self._note("failed", f"power-off failed: {type(e).__name__}: {e}", "error", error=p["error"])
            return
        with self._lock:
            p["phase"] = "done"
            self.last, self.pending = p, None
        self._note("powered_off", "power-off issued (HAT armed, systemctl poweroff)", "warn")

    def public(self, now: Optional[float] = None) -> dict:
        now = self.clock() if now is None else now
        with self._lock:
            p = dict(self.pending) if self.pending else None
            last = dict(self.last) if self.last else None
        # one flat shape whether or not anything is pending (null fields when idle); times are epoch s
        keys = ("by", "client", "source", "reason", "delay_s", "requested_at", "at", "estop", "error")
        out = {"pending": p is not None, "phase": p["phase"] if p else "idle",
               **{k: (p[k] if p else None) for k in keys},
               "in_s": round(max(0.0, p["deadline_t"] - now), 1) if p else None,
               "confirm": CONFIRM, "delay_bounds_s": {"default": DELAY_DEFAULT_S, "min": DELAY_MIN_S, "max": DELAY_MAX_S}}
        out["last"] = None if last is None else {k: last.get(k) for k in (
            "phase", "by", "client", "source", "reason", "requested_at", "at",
            "cancelled_by", "cancelled_client", "cancelled_at", "error")}
        return out
