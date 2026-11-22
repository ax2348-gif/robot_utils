"""The operator stream (``GET /oplog/stream``; the contract: docs/plans/data_collection_v1.md, "The node <-> script
contract"): everything the node did, in order of occurrence, as node-protocol records (docs/nodes.md, "Streams";
JSON header, no payload):

- ``{"type": "headset", "t_rx": <monotonic s>, "frame": {...}}``: each headset frame as received;
- ``{"type": "sent", "t_send": <monotonic s>, "datagram": {...}}``: each datagram sent to the WBC, exactly;
- ``{"type": "event", <the event line: seq, t, wall, kind, episode, detail>}``.

Publishers (the teleop loop, the event log) never block: each subscriber has a bounded queue; one that falls more
than ``max_records`` behind loses the oldest records and the next record's header carries ``dropped`` (how many),
as the rows streams do. ``ping`` records keep an idle stream alive.
"""
from __future__ import annotations

import asyncio
import threading
import time
from collections import deque
from typing import Optional

from ..common.protocol import pack_record
from ..common.server import PING_S

MAX_RECORDS = 5000        # per subscriber (~70 s of headset frames at 72 Hz)


class OpSubscriber:
    def __init__(self, max_records: int):
        self.q: deque = deque()
        self.max = max_records
        self.dropped = 0
        self.dropped_total = 0
        self.event = asyncio.Event()
        self.wake_pending = False


class OpLog:
    def __init__(self):
        self._lock = threading.Lock()
        self._subs: set[OpSubscriber] = set()
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.counts = {"headset": 0, "sent": 0, "event": 0}

    def attach(self, loop: asyncio.AbstractEventLoop) -> None:
        self.loop = loop

    @property
    def subscribers(self) -> int:
        return len(self._subs)

    # -- publishers (any thread) -----------------------------------------------------------------------------

    def headset(self, t_rx: float, frame: dict) -> None:
        self.publish({"type": "headset", "t_rx": t_rx, "frame": frame})

    def sent(self, t_send: float, datagram: dict) -> None:
        self.publish({"type": "sent", "t_send": t_send, "datagram": datagram})

    def event(self, line: dict) -> None:
        self.publish({"type": "event", **line})

    def publish(self, header: dict) -> None:
        self.counts[header["type"]] = self.counts.get(header["type"], 0) + 1
        if not self._subs:
            return
        wake = []
        with self._lock:
            for s in self._subs:
                s.q.append(header)
                if len(s.q) > s.max:
                    s.q.popleft()
                    s.dropped += 1
                    s.dropped_total += 1
                if not s.wake_pending:
                    s.wake_pending = True
                    wake.append(s)
        loop = self.loop
        if wake and loop is not None and not loop.is_closed():
            for s in wake:
                try:
                    loop.call_soon_threadsafe(s.event.set)
                except RuntimeError:   # the loop closed meanwhile (shutdown)
                    pass

    # -- subscribers (the event loop) ------------------------------------------------------------------------

    def subscribe(self, max_records: int = MAX_RECORDS) -> OpSubscriber:
        s = OpSubscriber(max_records)
        with self._lock:
            self._subs.add(s)
        return s

    def unsubscribe(self, s: OpSubscriber) -> None:
        with self._lock:
            self._subs.discard(s)

    def take(self, s: OpSubscriber) -> tuple[list, int]:
        with self._lock:
            s.event.clear()
            s.wake_pending = False
            items = list(s.q)
            s.q.clear()
            dropped, s.dropped = s.dropped, 0
        return items, dropped


def encode(items: list, dropped: int) -> bytes:
    """Records for ``items``; the first one carries ``dropped`` when records were lost before it."""
    out = []
    for i, h in enumerate(items):
        if i == 0 and dropped:
            h = {**h, "dropped": dropped}
        out.append(pack_record(h))
    return b"".join(out)


async def oplog_stream(oplog: OpLog, max_records: int = MAX_RECORDS):
    sub = oplog.subscribe(max_records)
    try:
        yield pack_record({"type": "ping", "t_send": time.monotonic()})
        while True:
            try:
                await asyncio.wait_for(sub.event.wait(), PING_S)
            except asyncio.TimeoutError:
                yield pack_record({"type": "ping", "t_send": time.monotonic()})
                continue
            items, dropped = oplog.take(sub)
            if items:
                yield encode(items, dropped)
    finally:
        oplog.unsubscribe(sub)
