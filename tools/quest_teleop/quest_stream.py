"""Reading the headset's ``teleop.quest_state.v1`` stream and the remote-consumer rules (quest_env DEMO.md).

``QuestReader`` connects to the headset (TCP, JSON lines, port 9391), reconnects on its own and queues every
frame with its arrival time on *our* monotonic clock and a connection number (a reconnect is a discontinuity).

``QuestGate`` applies the rules a remote robot consumer must apply to each frame, in order:

- schema ``teleop.quest_state.v1`` with a ``control_intent`` (else the frame is invalid);
- a new connection, a new ``session_id`` (provider session; the intent's own ``session_id`` too) or a new
  ``operator_frame_revision``: a discontinuity (engagements and anchors are obsolete; ``frame_seq`` tracking
  restarts, the native app resets it per session);
- ``frame_seq`` must increase within a connection and session; an older or repeated frame is dropped;
- an arrival gap > ``stale_s`` since the previous frame: a discontinuity (native does the same for its own gaps);
- focus: ``input_focused`` and ``action_sync_result == 0``, else the frame is invalid (and invalid input ends
  engagements, like native).

Freshness itself (no frame for ``stale_s`` on our clock) is checked by the caller at each control step with
``QuestGate.fresh(now)``.
"""
from __future__ import annotations

import json
import queue
import socket
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

SCHEMA = "teleop.quest_state.v1"
INTENT_SCHEMA = "teleop.operator_intent.v1"
STALE_S = 0.2           # DEMO.md: recommended freshness timeout on the consumer's clock
DEFAULT_PORT = 9391


@dataclass
class Frame:
    data: dict
    arrived: float       # our monotonic clock
    conn: int            # connection number (increments on every (re)connect)


class QuestReader:
    """Background TCP reader. ``get(timeout)`` returns the next Frame (or None); ``drain()`` all queued ones."""

    def __init__(self, host: str, port: int = DEFAULT_PORT, *, clock: Callable[[], float] = time.monotonic,
                 reconnect_s: float = 0.5, silence_s: float = 1.0, maxsize: int = 1000):
        self.address = (host, port)
        self.clock = clock
        self.reconnect_s = reconnect_s
        self.silence_s = silence_s     # a silent connection is dropped and reopened after this long
        self.frames: "queue.Queue[Frame]" = queue.Queue(maxsize)
        self.conn = 0
        self.connected = False
        self.counts = {"frames": 0, "bad_lines": 0, "connects": 0, "overflow": 0}
        self.last_error: Optional[str] = None
        self._stop = threading.Event()
        self._sock: Optional[socket.socket] = None
        self._thread = threading.Thread(target=self._run, name="quest-reader", daemon=True)

    def start(self) -> "QuestReader":
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        s = self._sock
        if s is not None:
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self._thread.join(2)

    def get(self, timeout: Optional[float] = None) -> Optional[Frame]:
        try:
            return self.frames.get(timeout=timeout)
        except queue.Empty:
            return None

    def drain(self) -> list[Frame]:
        out = []
        while True:
            try:
                out.append(self.frames.get_nowait())
            except queue.Empty:
                return out

    def _put(self, f: Frame) -> None:
        try:
            self.frames.put_nowait(f)
        except queue.Full:            # nobody is consuming: keep the newest
            self.counts["overflow"] += 1
            try:
                self.frames.get_nowait()
            except queue.Empty:
                pass
            self.frames.put_nowait(f)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                with socket.create_connection(self.address, timeout=2) as sock:
                    self._sock = sock
                    sock.settimeout(self.silence_s)
                    self.conn += 1
                    self.counts["connects"] += 1
                    self.connected = True
                    for line in sock.makefile("rb"):
                        if self._stop.is_set():
                            break
                        t = self.clock()
                        try:
                            data = json.loads(line)
                        except ValueError:
                            self.counts["bad_lines"] += 1
                            continue
                        if not isinstance(data, dict):
                            self.counts["bad_lines"] += 1
                            continue
                        self.counts["frames"] += 1
                        self._put(Frame(data, t, self.conn))
            except OSError as e:
                self.last_error = f"{type(e).__name__}: {e}"
            finally:
                self.connected = False
                self._sock = None
            self._stop.wait(self.reconnect_s)


@dataclass
class Verdict:
    accept: bool              # False: drop the frame entirely (out of order); nothing else applies
    valid: bool = False       # focused, well-formed: commands may be derived from it
    discontinuity: bool = False   # engagements and anchors must be dropped (release required)
    reason: str = ""


class QuestGate:
    """The remote-consumer rules, frame by frame (pure; the clock is the frames' arrival time)."""

    def __init__(self, stale_s: float = STALE_S):
        self.stale_s = stale_s
        self.conn: Optional[int] = None
        self.session = None
        self.intent_session = None
        self.revision = None
        self.seq: Optional[int] = None
        self.last_arrival: Optional[float] = None
        self.last_valid: Optional[float] = None    # arrival time of the newest valid frame
        self.counts = {"dropped_order": 0, "invalid": 0, "discontinuities": 0}

    def fresh(self, now: float) -> bool:
        """A valid frame arrived within ``stale_s`` (our clock)."""
        return self.last_valid is not None and now - self.last_valid <= self.stale_s

    def check(self, f: Frame) -> Verdict:
        d = f.data
        reasons = []
        if self.conn != f.conn:
            if self.conn is not None:
                reasons.append("reconnect")
            self.conn, self.seq = f.conn, None
        if d.get("schema") != SCHEMA:
            return self._invalid(f, "schema", reasons)
        session = d.get("session_id")
        intent = d.get("control_intent")
        if not isinstance(intent, dict) or intent.get("schema") != INTENT_SCHEMA:
            return self._invalid(f, "no control_intent", reasons)
        if session != self.session or intent.get("session_id") != self.intent_session:
            if self.session is not None:
                reasons.append("session")
            self.session, self.intent_session, self.seq = session, intent.get("session_id"), None
        seq = d.get("frame_seq")
        if not isinstance(seq, int):
            return self._invalid(f, "no frame_seq", reasons)
        if self.seq is not None and seq <= self.seq:
            self.counts["dropped_order"] += 1
            return Verdict(False, reason="out of order")
        self.seq = seq
        revision = intent.get("operator_frame_revision")
        if revision != self.revision:
            if self.revision is not None:
                reasons.append("revision")
            self.revision = revision
        if self.last_arrival is not None and f.arrived - self.last_arrival > self.stale_s:
            reasons.append("gap")
        self.last_arrival = f.arrived
        if not (d.get("input_focused") is True and d.get("action_sync_result", 0) == 0):
            return self._invalid(f, "unfocused", reasons)
        if not isinstance(intent.get("hands"), dict) or not isinstance(d.get("hands"), dict):
            return self._invalid(f, "no hands", reasons)
        self.last_valid = f.arrived
        if reasons:
            self.counts["discontinuities"] += 1
        return Verdict(True, True, bool(reasons), ",".join(reasons))

    def _invalid(self, f: Frame, why: str, reasons: list) -> Verdict:
        self.counts["invalid"] += 1
        self.last_arrival = f.arrived
        return Verdict(True, False, True, ",".join(reasons + [why]))
