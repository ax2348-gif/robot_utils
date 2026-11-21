"""The ``stream`` source: joint-space goals from the arm's internal joint channel (docs/arm.md "Internal joint
channel"; design docs/plans/fast_path_v1.md "Local only" and "Constraints from the code review").

INTERNAL, robot-local: a Unix datagram socket owned by the robot user (``joint_channel.py`` carries the bytes;
``common/fastpath.py`` the packets). Not part of the public API: the public fast path is one level up, at the WBC.

A ``JointStream`` mixin of ``ArmNode``: the session (hello with the lease token, bye), the receiver's per-datagram
work (validate, gate, write the goal slot), the control thread's source (goal, feedforward, staleness ramp) and the
state packet sent every control-loop iteration. Rules:

- **Session:** ``hello`` with the arm lease token opens it (the reply is ``welcome`` or ``refused``); one at a time,
  a new hello replaces it. Bye, lease end, e-stop, ``POST /disable`` and node shutdown close it (the node sends
  ``{"type": "bye", "reason"}`` unless the client said bye). A fault, a chock or a stow leave it open: the client keeps
  getting state packets (``state`` fault / disabled / off) and its goals are dropped until the arm is enabled again.
- **Every datagram** of the session (right address, newer ``seq``) renews the lease (``Lease.renew_holder``); ``seq``
  is per session, strictly increasing over keep-alives and goals alike (older or equal: dropped, counted).
- **Goals are checked in the receiver** (finite, within the joint limits + LIMIT_TOL, gap within 0..GAP_MAX, the arm
  inside its limits to engage, every joint within ``max_lead`` of the measured pose) and **gated at write time** under
  ``_lock``: enabled, motors on, no e-stop. A refused goal changes nothing; drops are counted by reason, and events
  are rate-limited (one per reason per session, then at most every NOTE_EVERY s), never one per datagram.
- **Preemption:** an accepted goal replaces track (newest wins); HTTP track replaces the stream. A planned motion
  (servo / pose / home / stow / chunk) is never preempted by the stream: it **pauses** the session (``paused``, the
  STATE_PAUSED flag), goals are dropped while it executes or settles, and the first goal after it ended resumes.
  ``POST /cancel`` and ``/float`` drop the stream's goal like track's (the next goal engages again).
- **A goal is (q, qd) from the tick that first uses it:** until the next goal the goal moves on at qd (capped at
  joint_vel), so a client one tick late does not stop the arm dead; with qd = 0 the goal is q, literally.
- **Staleness:** no fresh goal for ``stale_ms`` (keep-alives do not refresh a goal): the command ramps to rest at
  the soft joint acceleration from its current velocity (no instant brake) and holds there; the feedforward decays
  at the same rate. The next fresh goal resumes from that velocity (acceleration limited as always).
- **Feedforward ``qd``** changes at most by ``joint_acc`` × tick per tick (and is capped at ``joint_vel`` as all
  references are). Every goal still goes through ``_command_step`` (speed, acceleration, joint limits).
- **Jam:** report-only, like track; ``_cmd_count`` grows once per stream engagement, not per datagram.
"""
from __future__ import annotations

import math
import secrets
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from ..common import fastpath as F
from ..common.lease import LeaseError
from . import config as C
from .config import N
from .telemetry import SOURCES, STATES

STALE_MS = 30.0                      # ms without a fresh goal before the ramp to a hold (3 ticks)
MAX_LEAD = math.radians(20.0)        # rad a goal may lead the measured pose, any joint (track's TRACK_MAX_JUMP)
LIMIT_TOL = math.radians(0.5)        # rad a goal may lie outside the joint limits (it is clipped to them anyway)
NOTE_EVERY = 5.0                     # s between events of one drop reason (after the first of a session)
DROP_REASONS = ("session", "addr", "old", "mode", "not_enabled", "paused", "nonfinite", "limits", "gap",
                "no_reading", "engage", "lead")
BAD_PACKET = ("size", "magic", "version", "json", "unbound", "control")


@dataclass
class StreamConfig:
    stale_ms: float = STALE_MS
    max_lead: float = MAX_LEAD
    limit_tol: float = LIMIT_TOL

    def public(self) -> dict:
        return {"stale_ms": self.stale_ms, "max_lead_deg": round(math.degrees(self.max_lead), 3),
                "limit_tol_deg": round(math.degrees(self.limit_tol), 3)}


@dataclass
class StreamGoal:
    """The goal slot (one: the newest accepted goal wins)."""
    q: np.ndarray
    qd: np.ndarray
    t: float                                  # arrival, node clock
    seq: int
    t_basis: float                            # the client's t_basis (a state packet's t_tick), NaN: none
    t_apply: Optional[float] = None           # the tick that first used it (q is the goal then)
    ramp_v: Optional[np.ndarray] = None       # stale: the command's velocity on its ramp to rest


@dataclass
class StreamSession:
    id: int
    lease_id: int
    client: str
    addr: str
    opened: float
    seq: int = 0                              # newest accepted seq (keep-alives and goals)
    t_last: Optional[float] = None            # newest accepted datagram
    paused: bool = False
    counts: dict = field(default_factory=lambda: {"goals": 0, "keepalives": 0, "stale": 0, "sent": 0,
                                                  "send_errors": 0})
    dropped: dict = field(default_factory=dict)
    noted: dict = field(default_factory=dict)  # reason -> [time of the last event, drops since]

    def public(self, now: float, stale: bool) -> dict:
        return {"id": self.id, "client": self.client, "lease_id": self.lease_id, "address": self.addr,
                "age": round(now - self.opened, 3), "seq": self.seq,
                "last_age": None if self.t_last is None else round(now - self.t_last, 4),
                "paused": self.paused, "stale": stale, "counts": dict(self.counts),
                "dropped": dict(self.dropped)}


def scale_to_cap(v: np.ndarray, cap) -> np.ndarray:
    """``v`` scaled down as a whole so no component exceeds ``cap`` (node.scale_to)."""
    k = float(np.max(np.abs(v) / cap)) if np.any(v) else 0.0
    return v / k if k > 1.0 else v


class JointStream:
    """Mixin of ArmNode (uses its _lock, lease, estop, reading, kin, soft, events, motion, track, dt, ...)."""

    def _init_stream(self, socket_path, cfg: Optional[StreamConfig]) -> None:
        self.joint_socket = None if socket_path is None else str(socket_path)
        self.stream_cfg = cfg or StreamConfig()
        self.joint_channel = None                   # joint_channel.JointChannel once started
        self.stream: Optional[StreamGoal] = None    # the goal slot (an arm source, like track)
        self.stream_session: Optional[StreamSession] = None
        self.stream_bad: dict = {}                  # datagrams without a session: reason -> count
        self.nonfinite_goals = 0                    # _command_step's backstop (any source)
        self._steps = 0                             # control-loop iterations (the state packet's tick)

    # -- transport ----------------------------------------------------------------------------------------

    def start_joint_channel(self) -> None:
        if self.joint_socket is None or self.joint_channel is not None:
            return
        from .joint_channel import JointChannel
        self.joint_channel = JointChannel(self.joint_socket, self.stream_datagram)
        self.joint_channel.start()

    def stop_joint_channel(self) -> None:
        self.close_stream_session("node shutting down")
        ch, self.joint_channel = self.joint_channel, None
        if ch is not None:
            ch.close()

    def _stream_send(self, data: bytes, addr) -> bool:
        ch = self.joint_channel
        return ch is not None and ch.send(data, addr)

    # -- receiver thread ----------------------------------------------------------------------------------

    def stream_datagram(self, data, addr) -> Optional[bytes]:
        """One datagram from the socket (receiver thread). Returns the reply to send to ``addr`` (control
        messages), or None. Never raises for a bad datagram: it is counted."""
        if F.is_control(data):
            try:
                msg = F.decode_control(data)
            except F.PacketError as e:
                return self._bad(e.reason)
            kind = msg["type"]
            if kind == "hello":
                return self._stream_hello(msg, addr)
            if kind == "bye":
                s = self.stream_session
                if s is not None and addr and addr == s.addr:
                    self.close_stream_session("bye from the client", notify=False)
                return None
            return self._bad("control")
        try:
            c = F.unpack_command(data)
        except F.PacketError as e:
            return self._bad(e.reason)
        self._stream_command(c, addr, self.clock())
        return None

    def _bad(self, reason: str) -> None:
        self.stream_bad[reason] = self.stream_bad.get(reason, 0) + 1
        return None

    def _stream_hello(self, msg: dict, addr) -> Optional[bytes]:
        if not addr:
            return self._bad("unbound")             # nowhere to send replies or state: the client must bind
        client = str(msg.get("client") or "client")[:64]
        try:
            info = self.lease.check(msg.get("token"))
        except LeaseError as e:
            self.events.add(f"joint channel: hello from {client!r} refused: {e}", "warn")
            return F.encode_control(F.refused(str(e)))
        if self.estop.active:
            return F.encode_control(F.refused("e-stop is latched"))
        if self.kin is None:
            return F.encode_control(F.refused(f"arm not connected yet ({self.state})"))
        now = self.clock()
        with self._lock:
            old = self.stream_session
            sid = secrets.randbits(32) or 1
            while old is not None and sid == old.id:
                sid = secrets.randbits(32) or 1
            self.stream_session = StreamSession(sid, info.lease_id, client, addr, now)
            if old is not None and self.stream is not None:
                self.stream.t = -math.inf           # the old session's goal: ramp to a hold until the new one sends
        if old is not None and old.addr != addr:
            self._stream_send(F.encode_control(F.bye("replaced by a new session")), old.addr)
        self.events.add(f"joint channel: session {sid} opened by {client!r} (lease {info.lease_id})"
                        + (f", replacing {old.id}" if old is not None else ""))
        kin, s = self.kin, self.soft
        limits = {"lo": kin.lo.tolist(), "hi": kin.hi.tolist(), "joint_vel": s.joint_vel.tolist(),
                  "joint_acc": s.joint_acc.tolist(), "max_lead": self.stream_cfg.max_lead,
                  "limit_tol": self.stream_cfg.limit_tol, "gap_max": C.GAP_MAX}
        return F.encode_control(F.welcome(sid, self.stream_cfg.stale_ms, limits, rate_hz=round(1.0 / self.dt, 3)))

    def close_stream_session(self, reason: str, notify: bool = True) -> None:
        """End the session (any thread). Its goal, if still in force, goes stale at once: the arm ramps to a hold."""
        with self._lock:
            s, self.stream_session = self.stream_session, None
            if s is None:
                return
            if self.stream is not None:
                self.stream.t = -math.inf
        if notify:
            self._stream_send(F.encode_control(F.bye(reason)), s.addr)
        self.events.add(f"joint channel: session {s.id} closed ({reason}); {s.counts['goals']} goals, "
                        f"{sum(s.dropped.values())} dropped")

    def _drop(self, s: StreamSession, reason: str, now: float, why: Optional[str] = None) -> None:
        """Count a dropped datagram; an event the first time per session and reason, then at most every NOTE_EVERY s."""
        s.dropped[reason] = s.dropped.get(reason, 0) + 1
        n = s.noted.get(reason)
        if n is None or now - n[0] >= NOTE_EVERY:
            more = 0 if n is None else n[1]
            s.noted[reason] = [now, 0]
            self.events.add(f"joint channel: goal dropped ({reason}{': ' + why if why else ''})"
                            + (f"; {more} more since the last report" if more else ""), "warn")
        else:
            n[1] += 1

    def _stream_command(self, c: F.Command, addr, now: float) -> None:
        renewed = True
        with self._lock:
            s = self.stream_session
            if s is None or c.session != s.id:
                self._bad("session")
                return
            if addr != s.addr:
                s.dropped["addr"] = s.dropped.get("addr", 0) + 1
                return
            if c.seq <= s.seq:
                s.dropped["old"] = s.dropped.get("old", 0) + 1
                return
            if self.lease.renew_holder(s.lease_id) is None:
                renewed = False
            else:
                s.seq, s.t_last = c.seq, now
                self._stream_accept(s, c, now)
        if not renewed:
            self.close_stream_session("the lease ended")

    def _stream_accept(self, s: StreamSession, c: F.Command, now: float) -> None:
        """Under _lock: a datagram of the session, in order. Keep-alive, or a goal to check, gate and write."""
        if c.mode == F.MODE_KEEPALIVE:
            s.counts["keepalives"] += 1
            return
        if c.mode != F.MODE_GOAL:
            return self._drop(s, "mode", now, f"unknown mode {c.mode}")
        if not (self.enabled and self.motors_on and not self.estop.active):
            return self._drop(s, "not_enabled", now, f"the arm is {self.state}")
        m = self.motion
        if m is not None and m.phase in ("executing", "settling"):
            s.paused = True
            return self._drop(s, "paused", now, f"{m.traj.kind} {m.traj.id} in force; the first goal after it "
                                                "ends resumes")
        q, qd, gap = np.array(c.q, float), np.array(c.qd, float), float(c.gap)
        if not (np.all(np.isfinite(q)) and np.all(np.isfinite(qd)) and not math.isinf(gap)):
            return self._drop(s, "nonfinite", now)
        kin, cfg = self.kin, self.stream_cfg
        out = np.maximum(kin.lo - q, q - kin.hi)
        if np.any(out > cfg.limit_tol):
            j = int(np.argmax(out))
            return self._drop(s, "limits", now, f"joint{j + 1} goal {math.degrees(q[j]):+.1f}° outside "
                                                f"{math.degrees(kin.lo[j]):+.1f}..{math.degrees(kin.hi[j]):+.1f}°")
        if not math.isnan(gap) and not (0.0 <= gap <= C.GAP_MAX + 1e-6):
            return self._drop(s, "gap", now, f"gap {gap * 1e3:.1f} mm outside 0..{C.GAP_MAX * 1e3:.1f}")
        r = self.reading
        if r is None:
            return self._drop(s, "no_reading", now)
        if not self.servo:
            why = self._limit_problem(r.q_mot)
            if why:
                return self._drop(s, "engage", now, why)
        lead = np.abs(q - r.q)
        if np.any(lead > cfg.max_lead):
            j = int(np.argmax(lead))
            return self._drop(s, "lead", now, f"joint{j + 1} goal {math.degrees(lead[j]):.1f}° from the measured "
                                              f"pose (max {math.degrees(cfg.max_lead):.0f}°)")
        if self.stream is None:
            self._cmd_count += 1                    # one command per engagement (jam refire), not per datagram
            if self.track is not None:
                self.events.add("track replaced by the joint stream")
        self.track = None
        self.hold_q = None
        s.paused = False
        self.stream = StreamGoal(q, qd, now, c.seq, float(c.t_basis))
        if not math.isnan(gap):
            self.gripper.goal = gap
            if not self.g_servo:
                self._g_engage = True
        s.counts["goals"] += 1

    # -- control thread -----------------------------------------------------------------------------------

    def _stream_target(self, now: float):
        """(goal, MIT velocity reference) of the stream slot; servoing, under _lock."""
        g, s = self.stream, self.stream_session
        acc = self.soft.joint_acc
        dv = acc * self.dt
        if now - g.t <= self.stream_cfg.stale_ms * 1e-3:
            g.ramp_v = None
            if g.t_apply is None:
                g.t_apply = now
            # (q, qd) is a constant-velocity goal from the tick that first used it: until the next goal (or
            # stale_ms) it moves on with qd instead of stopping dead at q
            qd_want = scale_to_cap(g.qd, self.soft.joint_vel)
            q_goal = g.q + qd_want * (now - g.t_apply)
        else:
            if g.ramp_v is None:                    # just went stale: ramp down from the command's own velocity
                g.ramp_v = self.v_cmd.copy()
                if s is not None:
                    s.counts["stale"] += 1
            else:
                g.ramp_v = np.sign(g.ramp_v) * np.maximum(np.abs(g.ramp_v) - dv, 0.0)
            q_goal, qd_want = self.q_cmd + g.ramp_v * self.dt, np.zeros(N)
        prev = self.qd_ff if np.all(np.isfinite(self.qd_ff)) else np.zeros(N)
        return q_goal, prev + np.clip(qd_want - prev, -dv, dv)

    def _stream_out(self, now: float, r, read_age: float) -> None:
        """The RUS1 state packet to the session's client, right after the read (control thread; also on fault and
        connecting iterations). Non-blocking; failures are counted, never raised."""
        s = self.stream_session
        if s is None or self.joint_channel is None:
            return
        try:
            nan6 = [math.nan] * N
            with self._lock:
                servo = self.servo
                q_cmd = self.q_cmd.tolist() if (servo and self.q_cmd is not None) else nan6
                g = self.stream
                flags = ((F.STATE_SERVOING if servo else 0) | (F.STATE_STALE if (g is not None and g.ramp_v is not None) else 0)
                         | (F.STATE_JAM if self.jam_active is not None else 0)
                         | (F.STATE_LIMITED if (self._limited and servo) else 0)
                         | (F.STATE_PAUSED if s.paused else 0))
                seq = g.seq if g is not None else 0
                src = self._source_name()
            if r is None:
                q = qd = tau = nan6
                gap = math.nan
            else:
                q, qd, tau, gap = r.q.tolist(), r.qd.tolist(), r.tau.tolist(), C.rad_to_gap(r.g_q)
            pkt = F.pack_state(F.State(flags, s.id, self._steps & 0xFFFFFFFF, now, seq & 0xFFFFFFFF,
                                       SOURCES.index(src), STATES.index(self.state), q, qd, tau, q_cmd, gap,
                                       read_age))
        except Exception:  # noqa: BLE001 - never into the tick
            s.counts["send_errors"] += 1
            return
        if self._stream_send(pkt, s.addr):
            s.counts["sent"] += 1
        else:
            s.counts["send_errors"] += 1

    def _stream_row(self, now: float) -> tuple:
        """The row's stream_seq, stream_age, stream_lat (NaN without a stream goal)."""
        g = self.stream
        if g is None:
            return math.nan, math.nan, math.nan
        return float(g.seq), now - g.t, now - g.t_basis

    # -- reads ----------------------------------------------------------------------------------------------

    def stream_state(self) -> dict:
        now = self.clock()
        with self._lock:
            s, g = self.stream_session, self.stream
            sess = None if s is None else s.public(now, stale=g is not None and g.ramp_v is not None)
            goal = None if g is None else {"seq": g.seq, "age": round(now - g.t, 4) if math.isfinite(g.t) else None,
                                           "ramping": g.ramp_v is not None}
        return {"socket": self.joint_socket, "listening": self.joint_channel is not None,
                "config": self.stream_cfg.public(), "session": sess, "goal": goal,
                "bad_datagrams": dict(self.stream_bad), "nonfinite_goals": self.nonfinite_goals}
