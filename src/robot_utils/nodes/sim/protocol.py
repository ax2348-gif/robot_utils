"""The sim world's backend IPC: request / reply messages over Unix stream sockets (docs/sim.md, "Protocol").

One socket per node backend, under ``<runtime dir>/robot/sim/<sim name>/``: ``arm.sock``, ``swerve.sock``,
``camera-<camera>.sock``. A message is

    u32 big-endian N | N bytes of UTF-8 JSON (the header) | header["len"] bytes of binary payload

``len`` is absent or 0 when there is no payload. A client sends ``{"op": <name>, ...arguments}`` and
reads exactly one reply: ``{"ok": true, ...results}`` or ``{"ok": false, "error": <text>}``. Requests
on one connection are answered in order; a client may keep its connection for its whole life (the
world notices a closed connection, e.g. to switch the arm's motors off like a lost CAN bus). Numbers
are JSON numbers, vectors JSON lists; only images go in the payload.
"""
from __future__ import annotations

import json
import logging
import os
import socket
import struct
import threading
from pathlib import Path
from typing import Callable, Optional

from ..common.serve import check_name, socket_dir

log = logging.getLogger("sim.ipc")

MAX_HEADER = 1 << 20
MAX_PAYLOAD = 64 << 20


class IpcError(RuntimeError):
    """The world answered ``ok: false``, or the connection failed."""


def backend_dir(sim_name: str) -> Path:
    return socket_dir() / "sim" / check_name(sim_name)


def backend_socket(sim_name: str, backend: str) -> Path:
    """``<runtime dir>/robot/sim/<sim name>/<backend>.sock``."""
    return backend_dir(sim_name) / f"{check_name(backend)}.sock"


def _read_exact(f, n: int) -> bytes:
    buf = f.read(n)
    if buf is None or len(buf) < n:
        raise ConnectionError("connection closed")
    return buf


def send_msg(sock: socket.socket, header: dict, payload: bytes = b"") -> None:
    h = dict(header)
    if payload:
        h["len"] = len(payload)
    raw = json.dumps(h, separators=(",", ":"), allow_nan=True).encode()
    sock.sendall(struct.pack(">I", len(raw)) + raw + (payload or b""))


def recv_msg(f) -> tuple[dict, bytes]:
    """One message from a socket's binary file (``sock.makefile("rb")``)."""
    n = struct.unpack(">I", _read_exact(f, 4))[0]
    if n > MAX_HEADER:
        raise ConnectionError(f"header too large ({n} bytes)")
    header = json.loads(_read_exact(f, n))
    k = int(header.get("len") or 0)
    if k > MAX_PAYLOAD:
        raise ConnectionError(f"payload too large ({k} bytes)")
    return header, (_read_exact(f, k) if k else b"")


Handler = Callable[[dict, bytes, "Connection"], tuple]


class Connection:
    """One client connection on the server side (``state``: the handler's per-connection data)."""

    def __init__(self, sock: socket.socket):
        self.sock = sock
        self.state: dict = {}


class IpcServer:
    """Serves one backend socket: a thread per connection calls ``handler(header, payload, conn)`` for
    each request and sends what it returns, ``(reply dict)`` or ``(reply dict, payload bytes)``; an
    exception becomes ``{"ok": false, "error": ...}``. ``on_close(conn)`` runs when a client goes."""

    def __init__(self, path: Path, handler: Handler, on_close: Optional[Callable[[Connection], None]] = None):
        self.path = Path(path)
        self.handler = handler
        self.on_close = on_close
        self.connections: set = set()
        self.requests = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        if self.path.exists() or self.path.is_symlink():
            probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                probe.connect(str(self.path))
                probe.close()
                raise RuntimeError(f"already running: something is listening on {self.path}")
            except OSError:
                self.path.unlink()          # stale
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.bind(str(self.path))
        os.chmod(self.path, 0o660)
        self.sock.listen(16)
        self._thread = threading.Thread(target=self._accept, name=f"ipc-{self.path.stem}", daemon=True)
        self._thread.start()

    def _accept(self) -> None:
        while not self._stop.is_set():
            try:
                s, _ = self.sock.accept()
            except OSError:
                return
            conn = Connection(s)
            with self._lock:
                self.connections.add(conn)
            threading.Thread(target=self._serve, args=(conn,), name=f"ipc-{self.path.stem}-conn", daemon=True).start()

    def _serve(self, conn: Connection) -> None:
        f = conn.sock.makefile("rb")
        try:
            while not self._stop.is_set():
                try:
                    header, payload = recv_msg(f)
                except (ConnectionError, OSError, ValueError):
                    return
                self.requests += 1
                try:
                    out = self.handler(header, payload, conn)
                    reply, data = (out if isinstance(out, tuple) else (out, b""))
                    reply = {"ok": True, **reply}
                except Exception as e:  # noqa: BLE001 - reported to the client
                    if not isinstance(e, (ValueError, KeyError, RuntimeError)):
                        log.exception("%s: %s failed", self.path.name, header.get("op"))
                    reply, data = {"ok": False, "error": f"{type(e).__name__}: {e}"}, b""
                try:
                    send_msg(conn.sock, reply, data)
                except OSError:
                    return
        finally:
            with self._lock:
                self.connections.discard(conn)
            try:
                conn.sock.close()
            except OSError:
                pass
            if self.on_close is not None:
                try:
                    self.on_close(conn)
                except Exception:  # noqa: BLE001
                    log.exception("%s: on_close", self.path.name)

    @property
    def clients(self) -> int:
        with self._lock:
            return len(self.connections)

    def close(self) -> None:
        self._stop.set()
        try:
            self.sock.close()
        except OSError:
            pass
        with self._lock:
            conns = list(self.connections)
        for c in conns:
            try:
                c.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        try:
            self.path.unlink()
        except OSError:
            pass


class IpcClient:
    """A backend's connection to the world (thread-safe: one request at a time). Reconnects on the
    next call after a failure; ``connect_timeout`` bounds waiting for a world that is not up yet."""

    def __init__(self, path, timeout: float = 5.0):
        self.path = str(path)
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._f = None
        self._lock = threading.Lock()

    def _connect(self) -> None:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        try:
            s.connect(self.path)
        except OSError as e:
            s.close()
            raise IpcError(f"sim world not reachable at {self.path}: {e}") from None
        self._sock, self._f = s, s.makefile("rb")

    def call(self, op: str, payload: bytes = b"", timeout: Optional[float] = None, **args) -> tuple[dict, bytes]:
        with self._lock:
            if self._sock is None:
                self._connect()
            try:
                if timeout is not None:
                    self._sock.settimeout(timeout)
                send_msg(self._sock, {"op": op, **args}, payload)
                reply, data = recv_msg(self._f)
                if timeout is not None:
                    self._sock.settimeout(self.timeout)
            except (OSError, ConnectionError, ValueError) as e:
                self._drop()
                raise IpcError(f"sim world {op}: {type(e).__name__}: {e}") from None
        if not reply.get("ok"):
            raise IpcError(reply.get("error") or f"{op} failed")
        return reply, data

    def _drop(self) -> None:
        for x in (self._f, self._sock):
            try:
                if x is not None:
                    x.close()
            except OSError:
                pass
        self._sock = self._f = None

    def close(self) -> None:
        with self._lock:
            self._drop()
