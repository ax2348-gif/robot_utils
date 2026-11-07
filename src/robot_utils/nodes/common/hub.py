"""Thread -> asyncio handoff for every-sample data (rows) and state changes.

A hardware-owner thread calls ``publish_rows`` (or ``state_changed``); the hub stores the data
under a short lock and wakes the event loop with ``loop.call_soon_threadsafe``. Async consumers
never poll: each streaming client has a ``RowSubscriber`` whose bounded queue receives every
published row (the oldest rows are dropped, and counted, if a client falls more than
``max_rows`` behind), and state consumers await ``state_future()``.

The camera node's video uses a latest-frame-wins variant of the same idea (nodes/orbbec/hub.py).
"""
from __future__ import annotations

import asyncio
import threading
from collections import deque
from typing import Optional

import numpy as np


class RowSubscriber:
    def __init__(self, max_rows: int):
        self.batches: deque = deque()
        self.n = 0
        self.dropped = 0
        self.max_rows = max_rows
        self.event = asyncio.Event()

    def take(self) -> tuple[Optional[np.ndarray], int]:
        """All queued rows as one array, and the number dropped since the last take."""
        self.event.clear()
        if not self.batches:
            return None, 0
        rows = np.concatenate(list(self.batches)) if len(self.batches) > 1 else self.batches[0]
        self.batches.clear()
        self.n = 0
        dropped, self.dropped = self.dropped, 0
        return rows, dropped


class RowHub:
    def __init__(self, columns: int, recent: int = 1000):
        self.columns = columns
        self._lock = threading.Lock()
        self._recent: deque = deque(maxlen=recent)
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self._subs: set[RowSubscriber] = set()
        self._state_fut: Optional[asyncio.Future] = None

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

    # -- owner-thread side ---------------------------------------------------------------------

    def publish_rows(self, rows: np.ndarray) -> None:
        rows = np.atleast_2d(np.asarray(rows, np.float64))
        if rows.shape[1] != self.columns:
            raise ValueError(f"rows have {rows.shape[1]} columns, expected {self.columns}")
        with self._lock:
            self._recent.extend(rows)
        self._call(self._dispatch, rows)

    def state_changed(self) -> None:
        self._call(self._wake_state)

    # -- event-loop side -----------------------------------------------------------------------

    def recent(self, n: int) -> np.ndarray:
        """The last n published rows (for snapshots and /state), shape (k, columns)."""
        with self._lock:
            k = len(self._recent)
            rows = [self._recent[i] for i in range(max(0, k - n), k)]
        return np.array(rows).reshape(-1, self.columns)

    def subscribe(self, max_rows: int) -> RowSubscriber:
        sub = RowSubscriber(max_rows)
        self._subs.add(sub)
        return sub

    def unsubscribe(self, sub: RowSubscriber) -> None:
        self._subs.discard(sub)

    @property
    def subscribers(self) -> int:
        return len(self._subs)

    def state_future(self) -> asyncio.Future:
        """Future resolved at the next state change. Take it *before* reading the state."""
        if self._state_fut is None or self._state_fut.done():
            self._state_fut = asyncio.get_running_loop().create_future()
        return self._state_fut

    def _dispatch(self, rows: np.ndarray) -> None:
        for sub in self._subs:
            sub.batches.append(rows)
            sub.n += len(rows)
            while sub.n > sub.max_rows and len(sub.batches) > 1:
                old = sub.batches.popleft()
                sub.n -= len(old)
                sub.dropped += len(old)
            sub.event.set()

    def _wake_state(self) -> None:
        f, self._state_fut = self._state_fut, None
        if f is not None and not f.done():
            f.set_result(None)
