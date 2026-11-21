"""Where a control tick's time goes, tick by tick (``--tick-phases``; ``GET /debug/tick_phases``).

A late tick is the previous tick's work running past the period, or the loop's thread waking late
from its sleep. To tell the possible causes apart, each tick records:

  wake          how late the thread woke after its sleep (s past the scheduled tick start). The
                thread is SCHED_FIFO, so the timer itself is ~0.1 ms late at most: more is the wait
                for the interpreter lock (another Python thread holding it) or for a CPU.
  read          ``bus.read()``, split (hardware bus) into
  read_collect    draining the bus and reading the cached replies (no frames written), and
  read_request    writing the next tick's requests (xout and feedback refreshes)
  control       the computation in the tick (everything that is not read, send or publish)
  send          ``bus.send()``: the MIT frames
  publish       the telemetry row (kinematics) handed to the stream
  work          the whole tick, from waking to the end of publish
  cpu           the control thread's own CPU time in the tick (``time.thread_time``): work - cpu is
                the time it was blocked (frames waiting for the adapter, the interpreter lock,
                motorbridge's mutex) or waiting for a CPU

Recording costs a few clock reads per tick; the statistics are computed only when asked for (in the
request thread, from a copy).
"""
from __future__ import annotations

import time
from typing import Optional

import numpy as np

PHASES = ("wake", "read", "read_collect", "read_request", "control", "send", "publish", "work", "cpu")
_COL = {n: i for i, n in enumerate(PHASES)}
WORST = 10        # slowest ticks listed in full


class TickPhases:
    """Ring buffer of per-tick phase durations (s). Only the control thread writes; ``stats`` may be
    called from any thread (it copies the buffer; a row being written meanwhile may be torn)."""

    def __init__(self, capacity: int = 6000, clock=time.perf_counter, cpu_clock=time.thread_time):
        self.rows = np.full((capacity, len(PHASES)), np.nan)
        self.ticks = np.zeros(capacity)          # the tick's start (clock), for the worst-tick list
        self.n = 0                               # ticks recorded so far (the ring holds the last capacity)
        self._clock, self._cpu_clock = clock, cpu_clock
        self._row: Optional[np.ndarray] = None
        self._t0 = self._mark = self._cpu0 = 0.0

    def begin(self, wake_late: float) -> None:
        self._row = np.zeros(len(PHASES))
        self._row[_COL["wake"]] = max(wake_late, 0.0)
        self._t0 = self._mark = self._clock()
        self._cpu0 = self._cpu_clock()

    def mark(self, phase: str) -> None:
        """Charge the time since the last mark (or ``begin``) to ``phase`` (accumulates)."""
        if self._row is None:
            return
        now = self._clock()
        self._row[_COL[phase]] += now - self._mark
        self._mark = now

    def set(self, phase: str, seconds: float) -> None:
        if self._row is not None:
            self._row[_COL[phase]] = seconds

    def end(self) -> None:
        row = self._row
        if row is None:
            return
        self._row = None
        now = self._clock()
        row[_COL["control"]] += now - self._mark          # anything not charged to a phase
        row[_COL["work"]] = now - self._t0
        row[_COL["cpu"]] = self._cpu_clock() - self._cpu0
        i = self.n % len(self.rows)
        self.rows[i] = row
        self.ticks[i] = self._t0
        self.n += 1

    def abandon(self) -> None:
        """The tick ended early (a fault, an enable step): not recorded."""
        self._row = None

    def stats(self) -> dict:
        k = min(self.n, len(self.rows))
        rows, ticks = self.rows[:k].copy(), self.ticks[:k].copy()
        out = {"ticks": int(k), "recorded": int(self.n), "phases": {}, "worst": []}
        if k == 0:
            return out
        ms = rows * 1e3
        for name in PHASES:
            v = ms[:, _COL[name]]
            v = v[np.isfinite(v)]
            if len(v):
                p50, p90, p99 = np.percentile(v, [50, 90, 99])
                out["phases"][name] = {"p50": round(float(p50), 3), "p90": round(float(p90), 3),
                                       "p99": round(float(p99), 3), "max": round(float(v.max()), 3)}
        # a tick overruns the period when its work plus its own late wake exceed it
        late = ms[:, _COL["wake"]] + ms[:, _COL["work"]]
        order = np.argsort(late)[::-1][:WORST]
        t_last = float(ticks.max())
        for j in order:
            row = {name: (None if not np.isfinite(ms[j, c]) else round(float(ms[j, c]), 3)) for name, c in _COL.items()}
            out["worst"].append({"age_s": round(t_last - float(ticks[j]), 3), **row})
        out["over_10ms"] = int(np.sum(late > 10.0))
        out["over_15ms"] = int(np.sum(late > 15.0))
        return out
