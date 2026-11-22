"""Transport of the arm's INTERNAL joint channel (docs/arm.md "Internal joint channel"): one Unix datagram socket
``<runtime dir>/robot/<name>.joint.sock``, owner-only (0600), and one receiver thread. Robot-local by construction:
no UDP, no HTTP route, nothing behind nginx.

The receiver waits in ``selectors`` (the socket and a wake-up pair for ``close``), drains every queued datagram
and hands each to ``handler(data, addr)`` (``JointStream.stream_datagram``: microseconds of work, no event per
datagram); a reply it returns goes back to ``addr``. ``send`` is non-blocking and used by the control thread for
the state packets: a full or vanished client socket costs a counted error, never a wait.
"""
from __future__ import annotations

import logging
import selectors
import socket
import threading
from typing import Callable, Optional

from ..common.fastpath import bind_dgram, unlink_socket

log = logging.getLogger(__name__)

RECV_MAX = 65536


class JointChannel:
    def __init__(self, path: str, handler: Callable[[bytes, str], Optional[bytes]]):
        self.path = str(path)
        self.handler = handler
        self.sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._wake_r = self._wake_w = None
        self._stop = threading.Event()
        self.received = 0
        self.recv_errors = 0
        self.handler_errors = 0
        self.send_errors = 0

    def start(self, thread: bool = True) -> None:
        """``thread`` False: no receiver thread; the owner calls ``poll`` when ``fileno`` is readable (the arm's
        control process: its one thread waits on this socket with its command socket)."""
        self.sock = bind_dgram(self.path)
        self._buf = bytearray(RECV_MAX)
        self._view = memoryview(self._buf)
        if not thread:
            return
        self._wake_r, self._wake_w = socket.socketpair()
        self._wake_r.setblocking(False)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="arm-joint-channel", daemon=True)
        self._thread.start()

    def fileno(self) -> int:
        return self.sock.fileno()

    def poll(self) -> None:
        """Every queued datagram to the handler, now (no thread; never blocks)."""
        if self.sock is not None:
            self._drain(self._buf, self._view)

    def close(self) -> None:
        self._stop.set()
        if self._wake_w is not None:
            try:
                self._wake_w.send(b"x")
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        for s in (self.sock, self._wake_r, self._wake_w):
            if s is not None:
                s.close()
        self.sock = self._wake_r = self._wake_w = None
        unlink_socket(self.path)

    def send(self, data: bytes, addr) -> bool:
        sock = self.sock
        if sock is None or not addr:
            return False
        try:
            sock.sendto(data, addr)
            return True
        except OSError:            # full queue (EAGAIN / ENOBUFS), client gone (ENOENT / ECONNREFUSED), ...
            self.send_errors += 1
            return False

    def _run(self) -> None:
        sel = selectors.DefaultSelector()
        sel.register(self.sock, selectors.EVENT_READ)
        sel.register(self._wake_r, selectors.EVENT_READ)
        buf = bytearray(RECV_MAX)
        view = memoryview(buf)
        try:
            while not self._stop.is_set():
                for key, _ in sel.select():
                    if key.fileobj is self._wake_r:
                        return
                    self._drain(buf, view)
        except Exception:  # noqa: BLE001 - the socket closed under us, or worse: say so, never spin
            if not self._stop.is_set():
                log.exception("joint channel receiver stopped")
        finally:
            sel.close()

    def _drain(self, buf: bytearray, view: memoryview) -> None:
        sock = self.sock
        while True:
            try:
                n, addr = sock.recvfrom_into(buf)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                self.recv_errors += 1
                return
            self.received += 1
            try:
                reply = self.handler(bytes(view[:n]), addr or None)
            except Exception:  # noqa: BLE001 - one bad datagram must not stop the channel
                self.handler_errors += 1
                log.exception("joint channel datagram")
                continue
            if reply is not None and addr:
                self.send(reply, addr)
