"""Latching emergency stop for actuated nodes. Pure Python, thread safe.

Anyone may trigger it (a read-only viewer must be able to stop a runaway robot); it stays
latched until the lease holder clears it, and clearing it does not re-enable anything: the
holder must enable again. The node's control loop checks ``active`` every tick and also gets
``on_trigger`` called at once, so the hardware stops without waiting for the next tick.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional


@dataclass(frozen=True)
class EStopInfo:
    reason: str
    source: str
    t: float


class EStop:
    def __init__(self, on_trigger: Optional[Callable[[EStopInfo], None]] = None,
                 clock: Callable[[], float] = time.monotonic):
        self.on_trigger = on_trigger
        self._clock = clock
        self._lock = threading.Lock()
        self._info: Optional[EStopInfo] = None

    @property
    def active(self) -> bool:
        return self._info is not None

    @property
    def info(self) -> Optional[EStopInfo]:
        return self._info

    def trigger(self, reason: str = "", source: str = "") -> EStopInfo:
        """Latch the e-stop (a second trigger keeps the first reason)."""
        with self._lock:
            first = self._info is None
            if first:
                self._info = EStopInfo(reason or "e-stop", source or "unknown", self._clock())
            info = self._info
        if first and self.on_trigger is not None:
            self.on_trigger(info)
        return info

    def clear(self) -> None:
        """Unlatch. Callers check the lease first (only the holder may clear)."""
        with self._lock:
            self._info = None
