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
``QuestGate.fresh(now)``. ``fresh`` is also false while the stream is **late or slow** (headset latency detection):
per valid frame the latency estimate is its arrival (our monotonic clock) minus its ``sample_time_ns`` (the headset's
clock), relative to the running minimum of that difference over the session (the clock offset cancels); a frame
later than ``max_latency`` (150 ms) over that baseline, or fewer than ``min_rate`` (30 Hz) valid frames over the last
``rate_window`` (0.5 s), is bad. Input is fresh only once a FULL rate window has been judged after each (re)connect
(no lenient start-up) and after ``recover_s`` (0.5 s) of good frames, at start and after any bad frame (recovery
never re-arms anything: the client needs a new Y press). A frame without ``sample_time_ns`` is bad.

Limits of the baseline (a later item: a windowed minimum or a real clock-offset estimate): the running minimum of
arrival − sample time since the connect cannot see a latency that is already there, constant, from the connect, and
it drifts with the two clocks' relative drift. For the first tests the headset runs on the direct LAN.
"""
from __future__ import annotations

import json
import queue
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional

SCHEMA = "teleop.quest_state.v1"
INTENT_SCHEMA = "teleop.operator_intent.v1"
STALE_S = 0.2           # DEMO.md: recommended freshness timeout on the consumer's clock
MAX_LATENCY_S = 0.15    # a frame this much later than the session's best (arrival - sample time): stale
MIN_RATE_HZ = 30.0      # fewer valid frames than this over RATE_WINDOW_S: stale
RATE_WINDOW_S = 0.5
RECOVER_S = 0.5         # good frames needed (continuously) before late / slow input counts as fresh again
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
    """The remote-consumer rules, frame by frame (pure; the clock is the frames' arrival time), and the headset
    latency / rate detection (module docstring)."""

    def __init__(self, stale_s: float = STALE_S, max_latency: float = MAX_LATENCY_S, min_rate: float = MIN_RATE_HZ,
                 rate_window: float = RATE_WINDOW_S, recover_s: float = RECOVER_S):
        self.stale_s = stale_s
        self.max_latency, self.min_rate = max_latency, min_rate
        self.rate_window, self.recover_s = rate_window, recover_s
        self.conn: Optional[int] = None
        self.session = None
        self.intent_session = None
        self.revision = None
        self.seq: Optional[int] = None
        self.last_arrival: Optional[float] = None
        self.last_valid: Optional[float] = None    # arrival time of the newest valid frame
        self.counts = {"dropped_order": 0, "invalid": 0, "discontinuities": 0, "late": 0, "slow": 0}
        # latency / rate
        self.offset_min: Optional[float] = None    # min over the session of (arrival - sample time)
        self.latency: Optional[float] = None       # the newest valid frame's, over that baseline (s)
        self.arrivals: deque = deque()             # valid arrivals within rate_window
        self.t_first: Optional[float] = None       # the first valid arrival of the session
        self.good_since: Optional[float] = None    # good frames continuously since (None: bad now)
        self.timing_reason = "no frames yet"

    def _reset_timing(self) -> None:
        self.offset_min = self.latency = self.t_first = self.good_since = None
        self.arrivals.clear()

    def rate(self, now: float) -> float:
        """Valid frames per second over the last ``rate_window`` (``rate_window`` 0: the rate check is off: inf)."""
        if self.rate_window <= 0:
            return float("inf")
        while self.arrivals and self.arrivals[0] <= now - self.rate_window:
            self.arrivals.popleft()
        return sum(1 for t in self.arrivals if t <= now) / self.rate_window

    def _rate_due(self, now: float) -> bool:
        """The rate is judged once a full window has passed since the session's (or connection's) first frame."""
        return self.t_first is not None and now - self.t_first >= max(self.rate_window, 0.0)

    def timing_ok(self, now: float) -> bool:
        """Not late or slow: a full rate window judged since the (re)connect, good frames for ``recover_s`` and the
        rate still at least ``min_rate`` now."""
        if not self._rate_due(now):
            self.timing_reason = "headset rate not judged yet (a full window after the connect)"
            return False
        if self.good_since is None or now - self.good_since < self.recover_s:
            return False
        if self.rate(now) < self.min_rate:
            self.timing_reason = f"headset rate {self.rate(now):.0f} Hz < {self.min_rate:g} Hz"
            return False
        return True

    def fresh(self, now: float) -> bool:
        """A valid frame arrived within ``stale_s`` (our clock), and the stream is neither late nor slow."""
        return self.last_valid is not None and now - self.last_valid <= self.stale_s and self.timing_ok(now)

    def why_stale(self, now: float) -> str:
        if self.last_valid is None:
            return "no valid headset frame yet"
        if now - self.last_valid > self.stale_s:
            return f"headset input stale ({(now - self.last_valid) * 1e3:.0f} ms)"
        if not self._rate_due(now):
            return "headset rate not judged yet (a full window after the connect)"
        if self.good_since is not None and now - self.good_since < self.recover_s and self.timing_reason == "ok":
            return "headset input recovering"
        return self.timing_reason

    def _timing(self, f: Frame) -> None:
        """A valid frame: its latency over the session's baseline and the rate; good or bad."""
        ns = f.data.get("sample_time_ns")
        if self.t_first is None:
            self.t_first = f.arrived
        self.arrivals.append(f.arrived)
        why = None
        if not isinstance(ns, int) or isinstance(ns, bool):
            self.latency = None
            why = "no sample_time_ns"
        else:
            off = f.arrived - ns * 1e-9
            self.offset_min = off if self.offset_min is None else min(self.offset_min, off)
            self.latency = off - self.offset_min
            if self.latency > self.max_latency:
                self.counts["late"] += 1
                why = f"headset latency {self.latency * 1e3:.0f} ms > {self.max_latency * 1e3:.0f} ms"
        if why is None and self._rate_due(f.arrived) and self.rate(f.arrived) < self.min_rate:
            self.counts["slow"] += 1
            why = f"headset rate {self.rate(f.arrived):.0f} Hz < {self.min_rate:g} Hz"
        if why is not None:
            self.good_since, self.timing_reason = None, why
        else:
            if self.good_since is None:
                self.good_since = f.arrived          # fresh only after recover_s of good frames (and a full window)
            self.timing_reason = "ok"

    def check(self, f: Frame) -> Verdict:
        d = f.data
        reasons = []
        if self.conn != f.conn:
            if self.conn is not None:
                reasons.append("reconnect")
            self.conn, self.seq = f.conn, None
            self._reset_timing()
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
            self._reset_timing()
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
        self._timing(f)
        self.last_valid = f.arrived
        if reasons:
            self.counts["discontinuities"] += 1
        return Verdict(True, True, bool(reasons), ",".join(reasons))

    def _invalid(self, f: Frame, why: str, reasons: list) -> Verdict:
        self.counts["invalid"] += 1
        self.last_arrival = f.arrived
        return Verdict(True, False, True, ",".join(reasons + [why]))
