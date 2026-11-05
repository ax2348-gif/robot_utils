"""Background TCP reader for the Quest app's JSON-lines telemetry.

``QuestClient`` keeps one connection open (reconnecting with backoff), parses every line on
its own thread and keeps only the newest usable frame. Consumers poll ``latest()`` from
their control loop; nothing here blocks or calls back into user code.

What it guarantees about ``latest()``:

- it is a fully parsed ``QuestFrame`` (see ``messages``) or ``None``;
- within one connection and provider session, ``frame_seq`` only increases (late or repeated
  frames are dropped and counted);
- ``received_at`` is ``time.monotonic()`` on this machine at arrival. The client never
  decides freshness; compare ``received_at`` with your own clock (``age()``), because a
  dead connection leaves the last frame in place.
"""
from __future__ import annotations

import json
import logging
import socket
import threading
import time
from dataclasses import dataclass, replace
from typing import Callable, Optional

from .messages import ProtocolError, QuestFrame, parse_frame

log = logging.getLogger(__name__)

DEFAULT_PORT = 9391          # the Quest app's controller stream, on all interfaces
MAX_LINE_BYTES = 1 << 20


@dataclass(frozen=True)
class ClientStats:
    connected: bool = False
    connects: int = 0
    frames: int = 0               # accepted
    out_of_order: int = 0         # dropped: frame_seq not newer than the last accepted
    bad_lines: int = 0            # dropped: not JSON, wrong schema, oversize line
    last_error: Optional[str] = None


class QuestClient:
    """Read Quest telemetry from ``host:port`` on a daemon thread.

    Use as a context manager, or call ``start()`` and ``close()``. ``clock`` exists for tests.
    """

    def __init__(self, host: str, port: int = DEFAULT_PORT, *, connect_timeout_s: float = 2.0,
                 read_timeout_s: float = 0.5, backoff_s: tuple[float, float] = (0.2, 2.0),
                 clock: Callable[[], float] = time.monotonic):
        self.address = (host, port)
        self.connect_timeout_s = connect_timeout_s
        self.read_timeout_s = read_timeout_s
        self.backoff_s = backoff_s
        self._clock = clock
        self._lock = threading.Lock()
        self._new_frame = threading.Condition(self._lock)
        self._latest: Optional[QuestFrame] = None
        self._stats = ClientStats()
        self._closed = threading.Event()
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> "QuestClient":
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name=f"quest-client-{self.address[0]}",
                                            daemon=True)
            self._thread.start()
        return self

    def close(self, timeout_s: float = 2.0) -> None:
        self._closed.set()
        with self._lock:
            sock = self._sock
            self._new_frame.notify_all()
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout_s)

    def __enter__(self) -> "QuestClient":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.close()

    # -- consumer API ------------------------------------------------------

    def latest(self) -> Optional[QuestFrame]:
        """The newest accepted frame, however old (check ``age()``), or None if none yet."""
        with self._lock:
            return self._latest

    def age(self, frame: Optional[QuestFrame] = None) -> float:
        """Seconds since ``frame`` (default: the latest) arrived; inf if there is none."""
        frame = frame if frame is not None else self.latest()
        return float("inf") if frame is None else self._clock() - frame.received_at

    def wait_for_frame(self, timeout_s: float, newer_than: int = -1) -> Optional[QuestFrame]:
        """Block until a frame with ``frame_seq > newer_than`` (any session) arrives."""
        deadline = time.monotonic() + timeout_s
        with self._lock:
            while not self._closed.is_set():
                f = self._latest
                if f is not None and f.frame_seq > newer_than:
                    return f
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._new_frame.wait(remaining)
        return None

    @property
    def stats(self) -> ClientStats:
        with self._lock:
            return self._stats

    # -- reader thread -----------------------------------------------------

    def _bump(self, **changes) -> None:
        # caller holds the lock
        self._stats = replace(self._stats, **changes)

    def _run(self) -> None:
        delay = self.backoff_s[0]
        while not self._closed.is_set():
            try:
                sock = socket.create_connection(self.address, timeout=self.connect_timeout_s)
            except OSError as e:
                with self._lock:
                    self._bump(last_error=f"connect: {e}")
                self._closed.wait(delay)
                delay = min(delay * 2, self.backoff_s[1])
                continue
            delay = self.backoff_s[0]
            with self._lock:
                if self._closed.is_set():
                    sock.close()
                    return
                self._sock = sock
                self._bump(connected=True, connects=self._stats.connects + 1)
            log.info("connected to Quest at %s:%d", *self.address)
            try:
                self._read(sock)
            except OSError as e:
                if not self._closed.is_set():
                    with self._lock:
                        self._bump(last_error=f"read: {e}")
            finally:
                with self._lock:
                    self._sock = None
                    self._bump(connected=False)
                sock.close()
            if not self._closed.is_set():
                log.warning("Quest connection lost (%s); reconnecting", self._stats.last_error)
                self._closed.wait(delay)

    def _read(self, sock: socket.socket) -> None:
        sock.settimeout(self.read_timeout_s)
        buf = bytearray()
        last: Optional[tuple[str, int]] = None   # (session_id, frame_seq) in this connection
        while not self._closed.is_set():
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                # A TCP connection can stay "open" through a dead peer; the consumer's
                # freshness check covers that, so just keep waiting.
                continue
            if not chunk:
                with self._lock:
                    self._bump(last_error="connection closed by peer")
                return
            buf += chunk
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                line = bytes(buf[:nl])
                del buf[:nl + 1]
                last = self._accept(line, last)
            if len(buf) > MAX_LINE_BYTES:
                buf.clear()
                with self._lock:
                    self._bump(bad_lines=self._stats.bad_lines + 1, last_error="oversize line")
                # resynchronise at the next newline
                rest = self._skip_to_newline(sock)
                if rest is None:
                    return
                buf += rest

    def _skip_to_newline(self, sock: socket.socket) -> Optional[bytes]:
        while not self._closed.is_set():
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                return None
            nl = chunk.find(b"\n")
            if nl >= 0:
                return chunk[nl + 1:]
        return None

    def _accept(self, line: bytes, last: Optional[tuple[str, int]]) -> Optional[tuple[str, int]]:
        if not line.strip():
            return last
        now = self._clock()
        try:
            frame = parse_frame(json.loads(line), now)
        except (ValueError, ProtocolError) as e:  # JSONDecodeError and UnicodeDecodeError are ValueErrors
            with self._lock:
                self._bump(bad_lines=self._stats.bad_lines + 1, last_error=f"bad line: {e}")
            return last
        if last is not None and frame.session_id == last[0] and frame.frame_seq <= last[1]:
            with self._lock:
                self._bump(out_of_order=self._stats.out_of_order + 1)
            return last
        with self._lock:
            self._latest = frame
            self._bump(frames=self._stats.frames + 1)
            self._new_frame.notify_all()
        return frame.session_id, frame.frame_seq
