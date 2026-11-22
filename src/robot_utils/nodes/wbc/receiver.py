"""The WBC's external fast path: target datagrams (JSON v0) on UDP and a Unix datagram socket
(docs/plans/wbc_v1.md "Interfaces for the first build", 3).

One thread, ``selectors`` over both sockets. Per datagram: parse, check the session and ``seq`` (older or equal
dropped), then hand it to the node's inbox; the control tick validates and applies it. Never raises; every drop
is counted by reason (``counts``)."""
from __future__ import annotations

import json
import os
import selectors
import socket
import threading
from collections import deque
from typing import Callable, Optional

MAX_DATAGRAM = 65507
TYPES = ("target", "chunk", "alive")


class Receiver:
    def __init__(self, udp_host: Optional[str], udp_port: int, unix_path: Optional[str],
                 deliver: Callable[[dict, str], None]):
        """``deliver(msg, source)`` is called for every datagram that passed the session and seq checks."""
        self.deliver = deliver
        self.lock = threading.Lock()
        self.session: Optional[int] = None
        self.seq: Optional[int] = None
        self.counts: dict = {"accepted": 0, "dropped": {}}
        self.last_source: Optional[str] = None
        self.sel = selectors.DefaultSelector()
        self.udp: Optional[socket.socket] = None
        self.unix: Optional[socket.socket] = None
        self.unix_path = unix_path
        if udp_host is not None:
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp.bind((udp_host, udp_port))
            self.udp.setblocking(False)
            self.sel.register(self.udp, selectors.EVENT_READ)
        if unix_path:
            if os.path.exists(unix_path):
                os.unlink(unix_path)
            os.makedirs(os.path.dirname(unix_path) or ".", exist_ok=True)
            self.unix = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            self.unix.bind(unix_path)
            os.chmod(unix_path, 0o600)
            self.unix.setblocking(False)
            self.sel.register(self.unix, selectors.EVENT_READ)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def udp_port(self) -> Optional[int]:
        return None if self.udp is None else self.udp.getsockname()[1]

    def open_session(self, session: int) -> None:
        with self.lock:
            self.session, self.seq = session, None

    def close_session(self) -> None:
        with self.lock:
            self.session, self.seq = None, None

    def drop(self, reason: str) -> None:
        with self.lock:
            d = self.counts["dropped"]
            d[reason] = d.get(reason, 0) + 1

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="wbc-rx", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(1.0)
        for s in (self.udp, self.unix):
            if s is not None:
                s.close()
        if self.unix_path:
            try:
                os.unlink(self.unix_path)
            except OSError:
                pass

    def _run(self) -> None:
        while not self._stop.is_set():
            for key, _ in self.sel.select(0.2):
                sock = key.fileobj
                while True:
                    try:
                        data, addr = sock.recvfrom(MAX_DATAGRAM)
                    except (BlockingIOError, InterruptedError):
                        break
                    except OSError:
                        break
                    src = "udp" if sock is self.udp else "unix"
                    self.handle(data, f"{src}:{addr}" if addr else src)

    def handle(self, data: bytes, source: str = "test") -> Optional[str]:
        """Check one datagram; returns the drop reason or None (delivered)."""
        try:
            msg = json.loads(data)
        except (ValueError, UnicodeDecodeError):
            self.drop("not_json")
            return "not_json"
        if not isinstance(msg, dict) or msg.get("type") not in TYPES:
            self.drop("bad_type")
            return "bad_type"
        sess, seq = msg.get("session"), msg.get("seq")
        if not isinstance(seq, int) or isinstance(seq, bool) or not isinstance(sess, int):
            self.drop("bad_header")
            return "bad_header"
        with self.lock:
            if self.session is None or sess != self.session:
                why = "unknown_session"
            elif self.seq is not None and seq <= self.seq:
                why = "old_seq"
            else:
                why = None
                self.seq = seq
                self.counts["accepted"] += 1
                self.last_source = source
            if why:
                d = self.counts["dropped"]
                d[why] = d.get(why, 0) + 1
                return why
        self.deliver(msg, source)
        return None


class Inbox:
    """Receiver -> control tick hand-over (the tick drains it; bounded: the oldest go first)."""

    def __init__(self, n: int = 256):
        self._q: deque = deque(maxlen=n)
        self._lock = threading.Lock()

    def put(self, item) -> None:
        with self._lock:
            self._q.append(item)

    def drain(self) -> list:
        with self._lock:
            out = list(self._q)
            self._q.clear()
        return out
