"""The teleop node's event log (docs/quest_teleop_node.md, "Event log"; the contract: docs/plans/data_collection_v1.md,
"The node <-> script contract").

Every event is one line ``{"seq", "t", "wall", "kind", "episode", "detail"}``: ``seq`` +1 per event for the node's
life, continued across restarts (read back from the newest file at start); ``t`` this machine's monotonic clock,
``wall`` unix time. Events only, never frames. One append-only JSONL file per (local) day in ``data_dir``
(``events-YYYYMMDD.jsonl``), each line flushed and fsynced as it is written, the oldest files deleted when the
directory passes ``cap_bytes``.

``emit`` is cheap and never blocks (the teleop loop calls it): it numbers the line, hands it to the listeners (the
operator stream) in seq order, and queues it for the writer thread. A full queue or a failed write is counted
(``dropped``, ``write_errors``, logged), never silent; ``close`` writes everything queued before it returns.
Without a ``data_dir`` the log is kept in memory only (the last ``MEMORY_LINES``; tests, library use).
"""
from __future__ import annotations

import json
import logging
import os
import queue
import re
import threading
import time
from collections import deque
from typing import Callable, Iterator, Optional

log = logging.getLogger("quest_teleop.eventlog")

DATA_DIR = os.path.expanduser("~/.local/share/robot_utils/quest_teleop")
CAP_BYTES = 200 * 1024 * 1024
QUEUE_SIZE = 10000
MEMORY_LINES = 100000
FILE_RE = re.compile(r"^events-(\d{8})\.jsonl$")
EPISODE_RE = re.compile(r"^(\d{8})-(\d{4,})$")
KINDS = ("button", "armed", "disarmed", "enabled", "disabled", "episode_start", "episode_stop", "episode_abort",
         "estop", "fault", "hold", "stale", "teleop_start", "teleop_stop", "sim_reset", "sim_pause", "sim_resume",
         "pairing", "held", "resumed", "resume_refused", "escalated", "y_refused")


def day_of(wall: float) -> str:
    return time.strftime("%Y%m%d", time.localtime(wall))


def _parse(raw: bytes) -> Optional[dict]:
    try:
        d = json.loads(raw)
    except ValueError:
        return None
    return d if isinstance(d, dict) and isinstance(d.get("seq"), int) else None


def _tail_lines(path: str, chunk: int = 65536) -> Iterator[bytes]:
    """The file's lines, last first (reads backwards in chunks)."""
    with open(path, "rb") as f:
        f.seek(0, os.SEEK_END)
        pos = f.tell()
        rest = b""
        while pos > 0:
            n = min(chunk, pos)
            pos -= n
            f.seek(pos)
            buf = f.read(n) + rest
            lines = buf.split(b"\n")
            rest = lines[0]
            for ln in reversed(lines[1:]):
                if ln.strip():
                    yield ln
        if rest.strip():
            yield rest


class TeleopEventLog:
    def __init__(self, data_dir: Optional[str] = None, cap_bytes: int = CAP_BYTES, *,
                 clock: Callable[[], float] = time.monotonic, wall: Callable[[], float] = time.time,
                 queue_size: int = QUEUE_SIZE):
        self.dir = data_dir
        self.cap = int(cap_bytes)
        self.clock, self.wall = clock, wall
        self.listeners: list[Callable[[dict], None]] = []
        self._lock = threading.Lock()          # numbering + listeners: seq order = listener order
        self._written = threading.Condition()
        self.seq = 0                           # the newest seq given out
        self.written_seq = 0                   # the newest seq the writer has finished with (on disk or failed)
        self.last: Optional[dict] = None
        self.dropped = 0                       # lines that never reached the disk (queue full, write failed)
        self.write_errors = 0
        self.last_error: Optional[str] = None
        self.pruned = 0                        # files deleted by the size cap
        self.episode_day: Optional[str] = None
        self.episode_n = 0                     # the newest episode number of ``episode_day``
        self.memory: deque = deque(maxlen=MEMORY_LINES)
        self._q: "queue.Queue[Optional[dict]]" = queue.Queue(queue_size)
        self._f = None
        self._f_day: Optional[str] = None
        self._bytes = 0
        self._info_cache: dict = {}
        self._thread: Optional[threading.Thread] = None
        self.closed = False
        if self.dir:
            os.makedirs(self.dir, exist_ok=True)
            self._recover()
            self._thread = threading.Thread(target=self._run, name="quest-eventlog", daemon=True)
            self._thread.start()

    # -- files -----------------------------------------------------------------------------------------------

    def _names(self) -> list[str]:
        try:
            return sorted(n for n in os.listdir(self.dir) if FILE_RE.match(n))
        except OSError:
            return []

    def _path(self, name: str) -> str:
        return os.path.join(self.dir, name)

    def _recover(self) -> None:
        """seq from the newest file's last line (older files if it has none); the episode counter of the newest
        file's day from its newest episode id."""
        names = self._names()
        self._bytes = sum(self._size(n) for n in names)
        for name in reversed(names):
            for ln in _tail_lines(self._path(name)):
                d = _parse(ln)
                if d is not None:
                    self.seq = self.written_seq = d["seq"]
                    self.last = d
                    break
            if self.seq:
                break
        if names:
            for ln in _tail_lines(self._path(names[-1])):
                d = _parse(ln)
                m = EPISODE_RE.match(str(d.get("episode") or "")) if d else None
                if m:
                    self.episode_day, self.episode_n = m.group(1), int(m.group(2))
                    break
        if self.seq:
            log.info("event log %s: continuing at seq %d", self.dir, self.seq + 1)

    def _size(self, name: str) -> int:
        try:
            return os.path.getsize(self._path(name))
        except OSError:
            return 0

    def file_info(self, name: str) -> dict:
        """``{name, bytes, first_seq, last_seq}`` (cached per size and mtime)."""
        p = self._path(name)
        try:
            st = os.stat(p)
        except OSError:
            return {"name": name, "bytes": 0, "first_seq": None, "last_seq": None}
        key = (st.st_size, st.st_mtime_ns)
        c = self._info_cache.get(name)
        if c is not None and c[0] == key:
            return c[1]
        first = last = None
        with open(p, "rb") as f:
            for ln in f:
                d = _parse(ln)
                if d is not None:
                    first = d["seq"]
                    break
        for ln in _tail_lines(p):
            d = _parse(ln)
            if d is not None:
                last = d["seq"]
                break
        info = {"name": name, "bytes": st.st_size, "first_seq": first, "last_seq": last}
        self._info_cache[name] = (key, info)
        return info

    def files(self) -> list[dict]:
        if not self.dir:
            return []
        self.flush(1.0)
        return [self.file_info(n) for n in self._names()]

    # -- events ----------------------------------------------------------------------------------------------

    def next_episode(self) -> str:
        """A new episode id ``YYYYMMDD-NNNN``: this node's counter per (local) day, continued across restarts."""
        with self._lock:
            day = day_of(self.wall())
            if day != self.episode_day:
                self.episode_day, self.episode_n = day, 0
            self.episode_n += 1
            return f"{day}-{self.episode_n:04d}"

    def emit(self, kind: str, detail: Optional[dict] = None, episode: Optional[str] = None) -> dict:
        if kind not in KINDS:
            raise ValueError(f"unknown event kind {kind!r}")
        with self._lock:
            self.seq += 1
            line = {"seq": self.seq, "t": self.clock(), "wall": self.wall(), "kind": kind, "episode": episode,
                    "detail": dict(detail or {})}
            self.last = line
            if not self.dir:
                self.memory.append(line)
                self.written_seq = self.seq
            elif self.closed:
                self._lost(line, "the event log is closed")
            else:
                try:
                    self._q.put_nowait(line)
                except queue.Full:
                    self._lost(line, "the writer queue is full")
            for fn in self.listeners:
                try:
                    fn(line)
                except Exception:  # noqa: BLE001 (a listener never stops the log)
                    log.exception("event listener")
        return line

    def _lost(self, line: dict, why: str) -> None:
        self.dropped += 1
        self.last_error = f"seq {line['seq']} not written: {why}"
        log.error("event log: %s (%d lost so far)", self.last_error, self.dropped)

    # -- the writer ------------------------------------------------------------------------------------------

    def _run(self) -> None:
        while True:
            line = self._q.get()
            if line is None:
                break
            self._write(line)
            with self._written:
                self.written_seq = line["seq"]
                self._written.notify_all()
        if self._f is not None:
            try:
                self._f.close()
            except OSError:
                pass
            self._f = None

    def _open(self, day: str) -> None:
        if self._f is not None:
            try:
                self._f.close()
            except OSError:
                pass
            self._f = None
        p = self._path(f"events-{day}.jsonl")
        f = open(p, "ab")
        if f.tell() > 0:                       # a line cut by a crash: start ours on a new line
            with open(p, "rb") as r:
                r.seek(-1, os.SEEK_END)
                if r.read(1) != b"\n":
                    f.write(b"\n")
                    self._bytes += 1
        self._f, self._f_day = f, day

    def _write(self, line: dict) -> None:
        data = (json.dumps(line, separators=(",", ":"), default=str) + "\n").encode()
        day = day_of(line["wall"])
        for attempt in range(3):
            try:
                if self._f is None or self._f_day != day:
                    self._open(day)
                self._f.write(data)
                self._f.flush()
                os.fsync(self._f.fileno())
                self._bytes += len(data)
                break
            except OSError as e:
                self.write_errors += 1
                self.last_error = f"{type(e).__name__}: {e}"[:200]
                log.error("event log write (attempt %d): %s", attempt + 1, self.last_error)
                try:
                    if self._f is not None:
                        self._f.close()
                except OSError:
                    pass
                self._f = None
                time.sleep(0.1)
        else:
            self._lost(line, self.last_error or "write failed")
            return
        if self._bytes > self.cap:
            self._prune()

    def _prune(self) -> None:
        """Delete the oldest files (never the one being written) until the directory is under the cap."""
        names = self._names()
        self._bytes = sum(self._size(n) for n in names)
        current = f"events-{self._f_day}.jsonl"
        for n in names:
            if self._bytes <= self.cap:
                break
            if n == current:
                continue
            size = self._size(n)
            try:
                os.remove(self._path(n))
            except OSError as e:
                log.error("event log prune %s: %s", n, e)
                continue
            self._bytes -= size
            self.pruned += 1
            self._info_cache.pop(n, None)
            log.info("event log: pruned %s (%d bytes; cap %d)", n, size, self.cap)

    def flush(self, timeout: float = 2.0) -> bool:
        """Wait until every line emitted so far is written (or failed); False on timeout."""
        target = self.seq
        deadline = time.monotonic() + timeout
        with self._written:
            while self.written_seq < target and not (self.dir and self._thread is not None
                                                      and not self._thread.is_alive()):
                left = deadline - time.monotonic()
                if left <= 0:
                    return False
                self._written.wait(left)
        return True

    def close(self) -> None:
        """Write everything queued, then stop the writer."""
        with self._lock:
            if self.closed:
                return
            self.closed = True
        if self._thread is not None:
            self._q.put(None)          # blocks only if the queue is full: the writer is draining it
            self._thread.join(10)

    # -- reading ---------------------------------------------------------------------------------------------

    def since(self, since_seq: int = 0, limit: Optional[int] = None) -> Iterator[bytes]:
        """The lines after ``since_seq`` (JSON, newline-terminated), oldest first, across the files (the memory
        without a data dir)."""
        n = 0
        if not self.dir:
            for d in list(self.memory):
                if d["seq"] > since_seq:
                    yield (json.dumps(d, separators=(",", ":"), default=str) + "\n").encode()
                    n += 1
                    if limit is not None and n >= limit:
                        return
            return
        self.flush(2.0)
        for name in self._names():
            info = self.file_info(name)
            if info["last_seq"] is not None and info["last_seq"] <= since_seq:
                continue
            try:
                f = open(self._path(name), "rb")
            except OSError:
                continue                       # pruned meanwhile
            with f:
                for ln in f:
                    d = _parse(ln)
                    if d is None or d["seq"] <= since_seq:
                        continue
                    yield ln if ln.endswith(b"\n") else ln + b"\n"
                    n += 1
                    if limit is not None and n >= limit:
                        return

    def info(self) -> dict:
        return {"dir": self.dir, "seq": self.seq, "written_seq": self.written_seq, "queued": self._q.qsize(),
                "dropped": self.dropped, "write_errors": self.write_errors, "last_error": self.last_error,
                "bytes": self._bytes if self.dir else None, "cap_bytes": self.cap if self.dir else None,
                "pruned": self.pruned}
