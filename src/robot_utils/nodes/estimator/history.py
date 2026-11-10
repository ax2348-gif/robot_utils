"""Time-indexed pose history: a dense ring buffer of every published pose row (no I/O).

The fusion thread appends every row it publishes; API handlers query it. One lock covers both,
so a reply is a consistent snapshot: every pose in it comes from the same ``reset_id`` (the
world frame), and a reset (``clear``) can't land halfway through a query.

Storage per sample: t, x, y, theta (continuous), vx, vy, omega as float64 plus one byte of flags
(imu_ok, odom_ok): 57 bytes. At 100 Hz with 5 % headroom on the rate, 4 h is 1.51 M samples,
about 86 MB (allocated up front; the OS maps pages as they are first written).
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from .fusion import FIELDS

RATE_HZ = 100.0
HEADROOM = 1.05            # capacity = window * rate * this (the output runs at ~100.0-100.5 Hz)
FUTURE_TOL = 0.1           # s: a time this little past the latest sample gets the latest sample
COLUMNS = ("t", "x", "y", "theta", "vx", "vy", "omega")
_IDX = [FIELDS.index(c) for c in COLUMNS]
_IMU, _ODOM = FIELDS.index("imu_ok"), FIELDS.index("odom_ok")
_RESET = FIELDS.index("reset_id")


class HistoryError(Exception):
    """A time the history can't answer: ``status`` 404 (before the window / the last reset, or
    no samples yet) or 409 (newer than the latest sample by more than FUTURE_TOL)."""

    def __init__(self, status: int, msg: str, available: dict):
        super().__init__(msg)
        self.status, self.available = status, available


@dataclass
class Snapshot:
    """Poses for some query times, all from one reset_id."""
    reset_id: int
    poses: list


class PoseHistory:
    def __init__(self, window_s: float = 4 * 3600.0, rate_hz: float = RATE_HZ):
        if not window_s > 0:
            raise ValueError("window_s must be positive")
        self.window_s = float(window_s)
        self.capacity = max(16, int(math.ceil(window_s * rate_hz * HEADROOM)))
        self._v = np.zeros((len(COLUMNS), self.capacity), dtype=np.float64)   # rows: COLUMNS
        self._flags = np.zeros(self.capacity, dtype=np.uint8)                 # bit 0 imu_ok, bit 1 odom_ok
        self._lock = threading.Lock()
        self._head = 0          # physical index of the oldest sample
        self._n = 0
        self._evicted = False   # samples since the last clear were overwritten (the ring is full)
        self.reset_id = 0
        self.out_of_order = 0   # rows not newer than the latest (not stored)

    @property
    def memory_bytes(self) -> int:
        return int(self._v.nbytes + self._flags.nbytes)

    # -- writer (the fusion thread) -------------------------------------------------------------

    def clear(self, reset_id: int) -> None:
        """A reset: a new world frame, an empty history."""
        with self._lock:
            self._head = self._n = 0
            self._evicted = False
            self.reset_id = int(reset_id)

    def append(self, rows: np.ndarray) -> None:
        """Rows in the pose FIELDS layout, in time order (a row not newer than the latest is dropped)."""
        rows = np.asarray(rows, dtype=np.float64).reshape(-1, len(FIELDS))
        with self._lock:
            last = self._t_at(self._n - 1) if self._n else -math.inf
            for r in rows:      # usually one row per call (100 Hz)
                if int(r[_RESET]) != self.reset_id:
                    continue            # a row from another frame (can't happen with clear() in order)
                t = r[0]
                if not t > last:
                    self.out_of_order += 1
                    continue
                i = (self._head + self._n) % self.capacity
                self._v[:, i] = r[_IDX]
                self._flags[i] = int(r[_IMU] == 1.0) | int(r[_ODOM] == 1.0) << 1
                if self._n < self.capacity:
                    self._n += 1
                else:
                    self._head = (self._head + 1) % self.capacity
                    self._evicted = True
                last = t

    # -- readers (any thread) -------------------------------------------------------------------

    def _t_at(self, k: int) -> float:
        return self._v[0, (self._head + k) % self.capacity]

    def _span(self) -> tuple[Optional[float], Optional[float]]:
        """(t_start, t_end) answerable now (under the lock): the stored samples, at most window_s."""
        if not self._n:
            return None, None
        t_end = self._t_at(self._n - 1)
        return max(self._t_at(0), t_end - self.window_s), t_end

    def available(self) -> dict:
        with self._lock:
            return self._available()

    def _available(self) -> dict:
        t0, t1 = self._span()
        return {"t_start": t0, "t_end": t1, "reset_id": self.reset_id}

    def info(self) -> dict:
        with self._lock:
            t0, t1 = self._span()
            return {"reset_id": self.reset_id, "t_start": t0, "t_end": t1, "n": self._n,
                    "capacity": self.capacity, "window_s": self.window_s, "memory_bytes": self.memory_bytes,
                    "out_of_order": self.out_of_order}

    def _search(self, t: float) -> int:
        """Logical index of the first sample with time >= t (under the lock)."""
        cap, h, n = self.capacity, self._head, self._n
        ts = self._v[0]
        if h + n <= cap:
            return int(np.searchsorted(ts[h:h + n], t))
        first = ts[h:]                                   # oldest part, then the wrapped part
        if t <= first[-1]:
            return int(np.searchsorted(first, t))
        return len(first) + int(np.searchsorted(ts[:h + n - cap], t))

    def _sample(self, k: int) -> tuple[np.ndarray, int]:
        i = (self._head + k) % self.capacity
        return self._v[:, i], int(self._flags[i])

    def _pose(self, t: float) -> dict:
        t0, t1 = self._span()
        if t1 is None:
            raise HistoryError(404, "no poses since the last reset yet", self._available())
        if t > t1:
            if t - t1 > FUTURE_TOL:
                raise HistoryError(409, f"not yet: t={t:.3f} is {t - t1:.3f} s after the latest pose "
                                        f"(t_end={t1:.3f})", self._available())
            v, f = self._sample(self._n - 1)
            src = "latest"
        elif t < t0:
            why = (f"older than the {self.window_s:g} s window" if self._evicted or t < t1 - self.window_s
                   else f"before the first pose since the last reset (reset_id {self.reset_id})")
            raise HistoryError(404, f"t={t:.3f} is {why}; available {t0:.3f}..{t1:.3f}", self._available())
        else:
            k = self._search(t)
            v, f = self._sample(k)
            if v[0] == t:
                src = "exact"
            else:
                v0, f0 = self._sample(k - 1)
                a = (t - v0[0]) / (v[0] - v0[0])
                v, f = v0 + (v - v0) * a, f0 & f
                src = "interpolated"
        out = {"t_query": t, **{c: float(x) for c, x in zip(COLUMNS, v)}, "source": src,
               "imu_ok": bool(f & 1), "odom_ok": bool(f & 2)}
        if src == "interpolated":
            out["t"] = t
        return out

    def poses_at(self, ts: Sequence[Optional[float]], now: float) -> Snapshot:
        """Poses at times ``ts`` (None = now), one consistent snapshot; raises HistoryError."""
        with self._lock:
            return Snapshot(self.reset_id, [self._pose(now if t is None else float(t)) for t in ts])


def relative(a: dict, b: dict) -> dict:
    """SE(2) pose b expressed in the robot frame of pose a: (dx forward, dy left, dtheta)."""
    c, s = math.cos(a["theta"]), math.sin(a["theta"])
    wx, wy = b["x"] - a["x"], b["y"] - a["y"]
    dth = b["theta"] - a["theta"]
    return {"dx": c * wx + s * wy, "dy": -s * wx + c * wy,
            "dtheta": math.atan2(math.sin(dth), math.cos(dth)), "dtheta_total": dth}
