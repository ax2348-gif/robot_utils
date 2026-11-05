"""Keep slow side work (visualisation, status printing, logging) out of the control loop.

Steps should only read, compute and command. Anything that can block (a MeshCat or
network round trip, a terminal write, a disk flush) goes through a ``Monitor``: the step
``publish``es an immutable snapshot, which never blocks, and a worker thread hands the
newest snapshot to ``fn`` at most ``rate_hz`` times a second. Snapshots published faster
than that are simply superseded. A slow or failing ``fn`` cannot delay or stop the robot:
exceptions are logged and counted, and after ``max_errors`` in a row the monitor gives up.

    def start(self, ctx):
        self.monitor = Monitor(self.render, rate_hz=30).start()
    def active_step(self, ctx, inputs):
        ...
        self.monitor.publish(Snapshot(q=self.q.copy(), target=target))
    def stop(self, ctx, result):
        self.monitor.close()
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

log = logging.getLogger(__name__)

_NOTHING = object()


@dataclass(frozen=True)
class MonitorStats:
    published: int
    handled: int
    errors: int
    max_handle_s: float
    failed: bool


class Monitor:
    def __init__(self, fn: Callable[[Any], None], rate_hz: float = 30.0, *, name: str = "monitor",
                 max_errors: Optional[int] = 10):
        if not rate_hz > 0:
            raise ValueError("rate_hz must be positive")
        self.fn = fn
        self.period_s = 1.0 / rate_hz
        self.name = name
        self.max_errors = max_errors
        self._cond = threading.Condition()
        self._pending: Any = _NOTHING
        self._closed = False
        self._thread: Optional[threading.Thread] = None
        self._published = self._handled = self._errors = 0
        self._consecutive = 0
        self._max_handle_s = 0.0
        self._failed = False

    def start(self) -> "Monitor":
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name=self.name, daemon=True)
            self._thread.start()
        return self

    def publish(self, snapshot: Any) -> None:
        """Hand over the newest state; never blocks on ``fn``. Publish copies, not live arrays."""
        with self._cond:
            self._pending = snapshot
            self._published += 1
            self._cond.notify()

    def close(self, timeout_s: float = 1.0) -> None:
        """Stop the worker; waits at most ``timeout_s`` for a ``fn`` call in progress."""
        with self._cond:
            self._closed = True
            self._cond.notify()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout_s)

    def __enter__(self) -> "Monitor":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.close()

    @property
    def stats(self) -> MonitorStats:
        with self._cond:
            return MonitorStats(self._published, self._handled, self._errors, self._max_handle_s,
                                self._failed)

    def _run(self) -> None:
        next_t = time.monotonic()
        while True:
            with self._cond:
                while self._pending is _NOTHING and not self._closed:
                    self._cond.wait()
                if self._closed:
                    return
            # rate limit, then take whatever is newest by then
            delay = next_t - time.monotonic()
            if delay > 0:
                with self._cond:
                    self._cond.wait_for(lambda: self._closed, delay)
                    if self._closed:
                        return
            with self._cond:
                snapshot, self._pending = self._pending, _NOTHING
            t0 = time.monotonic()
            next_t = t0 + self.period_s
            try:
                self.fn(snapshot)
                self._consecutive = 0
            except Exception:
                self._consecutive += 1
                with self._cond:
                    self._errors += 1
                log.exception("%s failed", self.name)
                if self.max_errors is not None and self._consecutive >= self.max_errors:
                    log.error("%s: %d failures in a row; disabled", self.name, self._consecutive)
                    with self._cond:
                        self._failed = True
                    return
            dt = time.monotonic() - t0
            with self._cond:
                self._handled += 1
                self._max_handle_s = max(self._max_handle_s, dt)
