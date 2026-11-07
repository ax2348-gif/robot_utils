"""Thread -> asyncio handoff.

The hardware-owner threads call ``publish_frames`` / ``publish_imu`` / ``state_changed``; those
store data under a short lock and schedule a wake-up on the event loop with
``loop.call_soon_threadsafe``. Async consumers never poll: they take the current "changed"
future, look at the data, and await the future (with a timeout for keep-alive pings). Taking the
future *before* looking at the data means a publish can never be missed.

Video is latest-frame-wins (only the newest frame per stream is kept). IMU is every-sample: each
subscriber has its own bounded queue of batches (oldest dropped and counted if a client falls
more than ``max_backlog_s`` behind).
"""
from __future__ import annotations

import asyncio
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np


@dataclass(eq=False)
class Frame:
    """One published video frame. Immutable once published (shared by all clients)."""
    stream: str
    fmt: str               # camera pixel format: MJPG, Y16, Y8, ...
    w: int
    h: int
    data: Any              # bytes (MJPG) or numpy array (Y16 depth: HxW uint16; IR/other: flat uint8)
    t_capture: float       # server monotonic s: SDK host receipt (system timestamp)
    t_sensor: Optional[float]   # server monotonic s: device timestamp mapped to host (global ts)
    t_device_us: int       # raw device timestamp (camera clock, us)
    index: int             # SDK frame index
    t_owner: float = 0.0   # server monotonic s: owner thread got the frameset from the SDK
    depth_scale: float = 1.0
    aligned: bool = False
    seq: int = 0           # per stream, set at publish
    frameset: int = 0      # frames from one SDK frameset share this
    t_publish: float = 0.0

    def header(self) -> dict:
        h = {"type": "frame", "stream": self.stream, "seq": self.seq, "frameset": self.frameset,
             "index": self.index, "t_capture": self.t_capture, "t_sensor": self.t_sensor,
             "t_device_us": self.t_device_us, "t_owner": self.t_owner, "t_publish": self.t_publish,
             "width": self.w, "height": self.h, "source_format": self.fmt, "aligned": self.aligned}
        if self.stream == "depth":
            h["depth_scale"] = self.depth_scale
        return h


class ImuSubscriber:
    def __init__(self, max_samples: int):
        self.batches: deque = deque()
        self.n = 0
        self.dropped = 0
        self.max_samples = max_samples
        self.event = asyncio.Event()

    def take(self) -> tuple[Optional[np.ndarray], int]:
        """All queued samples as one array, and the number dropped since the last take."""
        if not self.batches:
            return None, 0
        rows = np.concatenate(list(self.batches)) if len(self.batches) > 1 else self.batches[0]
        self.batches.clear()
        self.n = 0
        dropped, self.dropped = self.dropped, 0
        return rows, dropped


class Hub:
    def __init__(self, imu_history: int = 5000):
        self._lock = threading.Lock()
        self._latest: dict[str, Frame] = {}
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self._frame_fut: Optional[asyncio.Future] = None
        self._state_fut: Optional[asyncio.Future] = None
        # IMU channels: "raw" = every 1 kHz sample (protocol.IMU_FIELDS), "state" = filter output
        # records (imu_filter.STATE_FIELDS)
        self._imu_subs: dict[str, set[ImuSubscriber]] = {"raw": set(), "state": set()}
        self.imu_recent: deque = deque(maxlen=imu_history)   # raw rows, for GET /imu/latest
        self.imu_state_recent: deque = deque(maxlen=imu_history // 10)   # state rows
        self.clients = 0

    def attach(self, loop: asyncio.AbstractEventLoop) -> None:
        self.loop = loop

    def _call(self, fn, *args) -> None:
        loop = self.loop
        if loop is None or loop.is_closed():
            return
        try:
            loop.call_soon_threadsafe(fn, *args)
        except RuntimeError:   # loop closed between the check and the call (shutdown)
            pass

    # -- owner-thread side -------------------------------------------------------------------

    def publish_frames(self, frames: list[Frame]) -> None:
        with self._lock:
            for f in frames:
                self._latest[f.stream] = f
        self._call(self._wake_frames)

    def drop_frames(self, keep: tuple = ()) -> None:
        """Forget latest frames (pipeline restarted: shapes/streams may change)."""
        with self._lock:
            for n in list(self._latest):
                if n not in keep:
                    del self._latest[n]
        self._call(self._wake_frames)

    def publish_imu(self, rows: np.ndarray, channel: str = "raw") -> None:
        with self._lock:
            (self.imu_recent if channel == "raw" else self.imu_state_recent).extend(rows)
        self._call(self._dispatch_imu, rows, channel)

    def state_changed(self) -> None:
        self._call(self._wake_state)

    # -- event-loop side ---------------------------------------------------------------------

    def latest(self, names=None) -> dict[str, Frame]:
        with self._lock:
            if names is None:
                return dict(self._latest)
            return {n: self._latest[n] for n in names if n in self._latest}

    def recent_imu(self, n: int, channel: str = "raw") -> list:
        with self._lock:
            d = self.imu_recent if channel == "raw" else self.imu_state_recent
            k = len(d)
            return [d[i] for i in range(max(0, k - n), k)]

    def frames_changed(self) -> asyncio.Future:
        """Future resolved at the next publish. Take it *before* reading latest()."""
        if self._frame_fut is None or self._frame_fut.done():
            self._frame_fut = asyncio.get_running_loop().create_future()
        return self._frame_fut

    def state_future(self) -> asyncio.Future:
        if self._state_fut is None or self._state_fut.done():
            self._state_fut = asyncio.get_running_loop().create_future()
        return self._state_fut

    def _wake_frames(self) -> None:
        f, self._frame_fut = self._frame_fut, None
        if f is not None and not f.done():
            f.set_result(None)

    def _wake_state(self) -> None:
        f, self._state_fut = self._state_fut, None
        if f is not None and not f.done():
            f.set_result(None)

    def subscribe_imu(self, max_samples: int, channel: str = "state") -> ImuSubscriber:
        sub = ImuSubscriber(max_samples)
        sub.channel = channel
        self._imu_subs[channel].add(sub)
        return sub

    def unsubscribe_imu(self, sub: ImuSubscriber) -> None:
        self._imu_subs[sub.channel].discard(sub)

    def _dispatch_imu(self, rows: np.ndarray, channel: str = "raw") -> None:
        for sub in self._imu_subs[channel]:
            sub.batches.append(rows)
            sub.n += len(rows)
            while sub.n > sub.max_samples and len(sub.batches) > 1:
                old = sub.batches.popleft()
                sub.n -= len(old)
                sub.dropped += len(old)
            sub.event.set()
