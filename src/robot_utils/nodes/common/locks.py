"""Lockout / tagout for the robot: persistent locks that nodes enforce.

A lock is a small JSON file in ``~/.local/state/robot_utils/locks/`` (``ROBOT_LOCKS_DIR`` overrides
it): what it covers, who placed it, why and when. Files survive restarts and reboots, like a wheel
chock left in place. Kinds:

- ``chock``: actuated nodes (target ``swerve``, ``arm``, an instance name, or ``all``). While any
  chock covers a node, it stops at once, refuses to enable (HTTP 423), and reports the lock.
  Everything else (telemetry, API, page) keeps working.
- ``cover``: cameras (target ``camera``, an instance name, or ``all``). While covered, the camera
  captures no images; every stream serves procedural frames flagged ``covered``. The IMU keeps
  running.

Several people can each place their own lock; a target is free only when every lock on it is
removed. Placing is open to anyone through the landing page (rate-limited, de-duplicated, capped).
Removing is an admin action on the robot over SSH:

    python -m robot_utils.admin.locks status | chock <target> | cover <target> | remove <id> | remove --all
"""
from __future__ import annotations

import json
import os
import secrets
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Optional

from .lease import RequestError

KINDS = ("chock", "cover")
CHECK_EVERY_S = 0.2           # nodes re-check the directory at most this often
WEB_MAX_ACTIVE = 5            # web-placed locks active at once (all clients)
WEB_MIN_INTERVAL_S = 30.0     # per client between web placements
WEB_MAX_PER_DAY = 10          # per client
MAX_REASON = 200
MAX_BY = 80


def default_dir() -> Path:
    env = os.environ.get("ROBOT_LOCKS_DIR")
    if env:
        return Path(env)
    base = os.environ.get("XDG_STATE_HOME", os.path.expanduser("~/.local/state"))
    return Path(base) / "robot_utils" / "locks"


@dataclass(frozen=True)
class Lock:
    id: str
    kind: str
    target: str
    by: str
    reason: str
    placed: float          # wall clock
    source: str            # "ssh" | "web <client>"

    def covers(self, kind: str, names: Iterable[str]) -> bool:
        return self.kind == kind and (self.target == "all" or self.target in set(names))

    def public(self) -> dict:
        d = asdict(self)
        d["age_s"] = round(time.time() - self.placed, 1)
        return d


class LockStore:
    """Reads (cheaply, cached by the directory's mtime) and writes the lock files."""

    def __init__(self, path: Optional[Path] = None, clock=time.monotonic):
        self.path = Path(path) if path else default_dir()
        self.clock = clock
        self._lock = threading.Lock()
        self._cache: list[Lock] = []
        self._sig = None
        self._checked = -1e9
        self._web_log: dict[str, list[float]] = {}     # client -> placement times (wall)

    # -- reading -------------------------------------------------------------------------------

    def active(self, force: bool = False) -> list[Lock]:
        now = self.clock()
        with self._lock:
            if not force and now - self._checked < CHECK_EVERY_S:
                return list(self._cache)
            self._checked = now
            try:
                st = self.path.stat()
                sig = (st.st_mtime_ns, st.st_ino)
            except FileNotFoundError:
                self._cache, self._sig = [], None
                return []
            if sig == self._sig and not force:
                return list(self._cache)
            locks = []
            for f in sorted(self.path.glob("*.json")):
                try:
                    d = json.loads(f.read_text())
                    if d.get("kind") in KINDS and d.get("id") == f.stem:
                        locks.append(Lock(d["id"], d["kind"], d["target"], d.get("by", "?"), d.get("reason", ""),
                                          float(d.get("placed", 0)), d.get("source", "?")))
                except (OSError, ValueError, KeyError, TypeError):
                    continue          # a half-written / foreign file never unlocks anything: it is ignored
            self._cache, self._sig = locks, sig
            return list(locks)

    def matching(self, kind: str, names: Iterable[str]) -> list[Lock]:
        names = list(names)
        return [l for l in self.active() if l.covers(kind, names)]

    # -- writing -------------------------------------------------------------------------------

    def place(self, kind: str, target: str, by: str, reason: str, source: str = "ssh",
              now: Optional[float] = None) -> tuple[Lock, bool]:
        """Place a lock. Returns (lock, created). For web placements: an identical active lock
        (same kind and target) is returned instead of a new one, and rate limits apply (429)."""
        if kind not in KINDS:
            raise RequestError(422, f"kind must be one of {KINDS}")
        target, by, reason = target.strip(), (by or "?").strip()[:MAX_BY], (reason or "").strip()[:MAX_REASON]
        if not target or "/" in target or len(target) > 64:
            raise RequestError(422, "invalid target")
        now = time.time() if now is None else now
        web = source.startswith("web")
        if web:
            existing = [l for l in self.active(force=True) if l.kind == kind and l.target == target]
            if existing:
                return existing[0], False
            web_active = [l for l in self.active() if l.source.startswith("web")]
            if len(web_active) >= WEB_MAX_ACTIVE:
                raise RequestError(429, f"{len(web_active)} web-placed locks are already active (max {WEB_MAX_ACTIVE}); "
                                        "an admin must remove some over SSH")
            client = source[4:] or "?"
            with self._lock:
                times = [t for t in self._web_log.get(client, []) if now - t < 86400]
                if times and now - times[-1] < WEB_MIN_INTERVAL_S:
                    raise RequestError(429, f"one lock per {WEB_MIN_INTERVAL_S:.0f} s per client; try again in "
                                            f"{WEB_MIN_INTERVAL_S - (now - times[-1]):.0f} s")
                if len(times) >= WEB_MAX_PER_DAY:
                    raise RequestError(429, f"{WEB_MAX_PER_DAY} web lock placements per client per day")
                times.append(now)
                self._web_log[client] = times
        lock = Lock(f"{kind}-{time.strftime('%Y%m%d-%H%M%S', time.localtime(now))}-{secrets.token_hex(2)}",
                    kind, target, by, reason, now, source)
        self._write(lock)
        return lock, True

    def _write(self, lock: Lock) -> None:
        self.path.mkdir(parents=True, exist_ok=True)
        tmp = self.path / f".{lock.id}.tmp"
        with open(tmp, "w") as f:
            json.dump(asdict(lock), f)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.path / f"{lock.id}.json")
        self.active(force=True)

    def remove(self, lock_id: str) -> Lock:
        if not lock_id or "/" in lock_id or lock_id.startswith("."):
            raise RequestError(404, "no such lock")
        f = self.path / f"{lock_id}.json"
        found = [l for l in self.active(force=True) if l.id == lock_id]
        if not found or not f.exists():
            raise RequestError(404, f"no lock {lock_id!r}")
        f.unlink()
        self.active(force=True)
        return found[0]

    def remove_all(self, kind: Optional[str] = None) -> list[Lock]:
        out = []
        for l in self.active(force=True):
            if kind is None or l.kind == kind:
                out.append(self.remove(l.id))
        return out


def lock_state(store: LockStore, chock: Iterable[str] = (), cover: Iterable[str] = ()) -> dict:
    """For a node's /state: the chocks / covers that apply to it (pages show them; common/ui.js)."""
    ch = store.matching("chock", chock) if chock else []
    cv = store.matching("cover", cover) if cover else []
    return {"chocked": bool(ch), "covered": bool(cv), "locks": [l.public() for l in ch + cv]}


def describe(locks: list[Lock]) -> str:
    return "; ".join(f"{l.kind}ed by {l.by}" + (f": {l.reason}" if l.reason else "") + f" ({l.id})" for l in locks)
