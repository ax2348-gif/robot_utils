"""Single-controller lease for actuated nodes (swerve, later the arm). Pure Python.

One client at a time holds the lease and may command the hardware; everyone else is read-only.

- ``acquire(client)`` returns a token (409-style ``LeaseError`` if someone else holds it).
- The holder presents the token with every controlling request. ``renew(token)`` (or any
  accepted command, which calls it) pushes the expiry ``heartbeat`` seconds into the future.
- ``release(token)`` ends it; so does letting it lapse. ``expire_if_due()`` must be called
  periodically by the node (the swerve node calls it every control tick, on the control thread,
  so a lapse stops the hardware even if the web server is stuck).
- ``on_end(reason)`` runs once whenever a lease ends (released / lapsed / replaced), under no
  lock: nodes use it to stop and disable the hardware and to reset per-lease settings such as
  soft limits.

Thread safe; the clock is injectable for tests.
"""
from __future__ import annotations

import secrets
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

DEFAULT_HEARTBEAT = 1.0   # s


class RequestError(Exception):
    """A request the node does not carry out; ``status`` is the HTTP status to answer with
    (``common.server.setup_app`` maps it to ``{"detail": message, **extra}``: ``detail`` stays a
    human-readable string, ``extra`` adds machine-readable fields beside it)."""

    def __init__(self, status: int, msg: str, extra: Optional[dict] = None):
        super().__init__(msg)
        self.status = status
        self.extra = extra or {}


class LeaseError(RequestError):
    """The request needs the lease and does not have it: 401 (no token), 403 (wrong or expired
    token), 409 (someone else holds the lease)."""


@dataclass(frozen=True)
class LeaseInfo:
    client: str
    acquired: float
    expires: float
    lease_id: int


class Lease:
    def __init__(self, heartbeat: float = DEFAULT_HEARTBEAT,
                 on_end: Optional[Callable[[str], None]] = None,
                 clock: Callable[[], float] = time.monotonic):
        self.heartbeat = float(heartbeat)
        self.on_end = on_end
        self._clock = clock
        self._lock = threading.Lock()
        self._token: Optional[str] = None
        self._info: Optional[LeaseInfo] = None
        self._next_id = 1

    # -- queries -------------------------------------------------------------------------------

    @property
    def info(self) -> Optional[LeaseInfo]:
        with self._lock:
            return self._info

    @property
    def held(self) -> bool:
        return self.info is not None

    def check(self, token: Optional[str]) -> LeaseInfo:
        """Raise LeaseError unless ``token`` is the current, unexpired lease."""
        with self._lock:
            if not token:
                raise LeaseError(401, "this request needs the lease token (X-Lease-Token header)")
            if self._info is None:
                raise LeaseError(403, "no lease is held (acquire one first; it may have lapsed)")
            if not secrets.compare_digest(token, self._token):
                raise LeaseError(403, "wrong lease token (the lease is held by "
                                      f"{self._info.client!r}, or yours lapsed)")
            if self._clock() > self._info.expires:
                raise LeaseError(403, "lease expired")
            return self._info

    # -- changes -------------------------------------------------------------------------------

    def acquire(self, client: str) -> tuple[str, LeaseInfo]:
        self.expire_if_due()
        with self._lock:
            if self._info is not None:
                raise LeaseError(409, f"the lease is held by {self._info.client!r}")
            now = self._clock()
            self._token = secrets.token_urlsafe(16)
            self._info = LeaseInfo(client or "anonymous", now, now + self.heartbeat, self._next_id)
            self._next_id += 1
            return self._token, self._info

    def renew(self, token: Optional[str]) -> LeaseInfo:
        self.check(token)
        with self._lock:
            info = self._info
            self._info = LeaseInfo(info.client, info.acquired, self._clock() + self.heartbeat,
                                   info.lease_id)
            return self._info

    def release(self, token: Optional[str]) -> None:
        self.check(token)
        self._end("released")

    def expire_if_due(self) -> bool:
        """End the lease if its heartbeat lapsed. Returns True if it ended now."""
        with self._lock:
            due = self._info is not None and self._clock() > self._info.expires
        if due:
            self._end("lapsed")
        return due

    def _end(self, reason: str) -> None:
        with self._lock:
            if self._info is None:
                return
            self._token, self._info = None, None
        if self.on_end is not None:
            self.on_end(reason)
