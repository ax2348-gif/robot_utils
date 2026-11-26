"""The WBC node, as this client uses it (wbc_v1.md, "Interfaces for the first build", items 2-4).

- HTTP (``robot_utils.nodes.common.client.NodeClient``): ``POST /lease`` (token in ``X-Lease-Token``),
  ``POST /lease/renew`` (a background renewer), ``DELETE /lease``; ``POST /enable`` / ``/disable``;
  ``POST /hints`` (session defaults); ``POST /session`` -> ``{session, udp_port, unix_socket}``.
- Target datagrams (JSON v0) over UDP to the node's host (or ``udp_host``) at ``udp_port``: ``target`` with
  ``session``, ``seq`` (strictly increasing per session) and ``t: null`` (now); ``alive`` keep-alives.
- The stream (``GET /stream``, rows): a reader thread keeps the newest row as a dict and its arrival time.
"""
from __future__ import annotations

import json
import socket
import threading
import time
from typing import Callable, Optional
from urllib.parse import urlparse

from robot_utils.nodes.common.client import NodeClient, NodeError   # noqa: F401 (re-exported)


class WbcStream:
    """Newest WBC stream row (``{field: value}``) and when it arrived on our clock; reconnects on its own."""

    def __init__(self, base_url: str, *, clock: Callable[[], float] = time.monotonic, path: str = "/stream"):
        self.base_url = base_url
        self.path = path
        self.clock = clock
        self.row: Optional[dict] = None
        self.t_recv: Optional[float] = None
        self.rows = 0
        self.connects = 0
        self.last_error: Optional[str] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._client: Optional[NodeClient] = None
        self._thread = threading.Thread(target=self._run, name="wbc-stream", daemon=True)

    def start(self) -> "WbcStream":
        self._thread.start()
        return self

    def latest(self) -> tuple[Optional[dict], Optional[float]]:
        with self._lock:
            return self.row, self.t_recv

    def close(self) -> None:
        self._stop.set()
        c = self._client
        if c is not None:
            try:
                c.close()
            except Exception:  # noqa: BLE001
                pass
        self._thread.join(2)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._client = NodeClient(self.base_url)
                self.connects += 1
                for _, rows in self._client.rows(self.path):
                    if self._stop.is_set():
                        return
                    t = self.clock()
                    last = rows[-1]
                    row = {name: float(last[name]) for name in rows.dtype.names}
                    with self._lock:
                        self.row, self.t_recv = row, t
                        self.rows += len(rows)
            except Exception as e:  # noqa: BLE001 (any transport error: reconnect)
                self.last_error = f"{type(e).__name__}: {e}"[:200]
            finally:
                c, self._client = self._client, None
                if c is not None:
                    try:
                        c.close()
                    except Exception:  # noqa: BLE001
                        pass
            self._stop.wait(0.5)


class WbcClient:
    """Control plane over HTTP plus the datagram sender."""

    def __init__(self, base_url: str, *, udp_host: Optional[str] = None, client_name: str = "quest_teleop",
                 timeout: float = 3.0):
        self.base_url = base_url.rstrip("/")
        self.http = NodeClient(self.base_url, timeout=timeout)
        self.client_name = client_name
        self.udp_host = udp_host or urlparse(self.base_url).hostname or "127.0.0.1"
        self.token: Optional[str] = None
        self.heartbeat = 1.0
        self.session: Optional[int] = None
        self.udp_port: Optional[int] = None
        self.seq = 0
        self.arm_seq_floor = 0             # POST /session's arm_seq: presses are numbered above it
        self.sent = {"target": 0, "alive": 0}
        self.renew_errors = 0
        self.caps: dict = {"asserted": {}, "previous": {}}   # the WBC caps this lease set, and what they replaced
        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._addr: Optional[tuple] = None
        self._stop_renew = threading.Event()
        self._renewer: Optional[threading.Thread] = None

    # -- HTTP -------------------------------------------------------------------------------------

    def acquire(self) -> dict:
        out = self.http.post("/lease", {"client": self.client_name})
        self.token = out["token"]
        self.heartbeat = float(out.get("heartbeat") or 1.0)
        self.http.headers["X-Lease-Token"] = self.token
        self._stop_renew.clear()
        self._renewer = threading.Thread(target=self._renew_loop, name="wbc-lease", daemon=True)
        self._renewer.start()
        return out

    def _renew_loop(self) -> None:
        period = max(0.05, self.heartbeat / 3)
        while not self._stop_renew.wait(period):
            try:
                self.http.post("/lease/renew")
            except Exception:  # noqa: BLE001 (counted; the node holds/stops when the lease lapses)
                self.renew_errors += 1

    def enable(self) -> dict:
        return self.http.post("/enable")

    def disarm_reason(self) -> Optional[str]:
        """The WBC's ``/state`` ``disarm_reason`` (None while armed): the teleop latch's hold classifies its own
        disarms by it."""
        return (self.http.get("/state") or {}).get("disarm_reason")

    def disable(self) -> Optional[dict]:
        return self.http.post("/disable")

    def get_config(self) -> dict:
        """The WBC's config in force (``GET /config``'s ``config``)."""
        return self.http.get("/config")["config"]

    def set_config(self, changes: dict) -> dict:
        """``POST /config`` (no lease needed; the base caps only while the base is still)."""
        return self.http.post("/config", changes)

    def set_hints(self, hints: dict) -> Optional[dict]:
        return self.http.post("/hints", hints)

    def open_session(self) -> dict:
        out = self.http.post("/session", {})
        self.session = int(out["session"])
        self.udp_port = int(out["udp_port"])
        self._addr = (self.udp_host, self.udp_port)
        self.seq = 0
        self.arm_seq_floor = int(out.get("arm_seq") or 0)
        return out

    def release(self) -> None:
        self._stop_renew.set()
        if self._renewer is not None:
            self._renewer.join(2)
        if self.token is not None:
            try:
                self.http.delete("/lease")
            finally:
                self.token = None
                self.http.headers.pop("X-Lease-Token", None)

    def close(self) -> None:
        self._stop_renew.set()
        self.http.close()
        self._udp.close()

    # -- datagrams --------------------------------------------------------------------------------

    def _send(self, msg: dict) -> dict:
        if self._addr is None:
            raise RuntimeError("no WBC session: open_session() first")
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        msg = {"type": msg.pop("type"), "session": self.session, "seq": self.seq, **msg}
        self._udp.sendto(json.dumps(msg, separators=(",", ":")).encode(), self._addr)
        self.sent[msg["type"]] = self.sent.get(msg["type"], 0) + 1
        return msg

    def send_target(self, body: dict, armed: bool = False, arm_seq: int = 0) -> dict:
        """``body``: ``{"ees": {...}, "base": {...}}`` (the reactive target; ``t: null`` = now). ``armed`` /
        ``arm_seq``: the armed latch's state, in every datagram (default: disarmed)."""
        return self._send({"type": "target", "t": None, **body, "armed": bool(armed), "arm_seq": int(arm_seq)})

    def send_alive(self, armed: bool = False, arm_seq: int = 0) -> dict:
        """A keep-alive (never arms anything) with the latch's state."""
        return self._send({"type": "alive", "armed": bool(armed), "arm_seq": int(arm_seq)})
