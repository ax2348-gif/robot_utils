"""The WBC's session on the arm's internal joint channel (docs/arm.md "Internal joint channel").

Packets and control messages are ``nodes/common/fastpath.py``'s. This module keeps the WBC's side of the session:
bind its own socket, hello (with the arm lease token) -> welcome (``session``, ``stale_ms``, ``rate_hz``,
``limits``: lo, hi, joint_vel, joint_acc, max_lead, limit_tol, gap_max), one goal per tick (non-blocking, errors
counted), the newest state each tick, and the node's ``bye`` (replaced, lease end, e-stop, disable, shutdown)
recorded as ``closed_reason`` (the WBC treats it as losing the arm).

What the arm does with goals (the WBC relies on it): a goal (q, qd) runs on at qd until the next one; keep-alives
do not refresh it; after ``stale_ms`` the command ramps to rest. Goals farther than ``max_lead`` from the measured
pose are dropped. A planned motion (HTTP home / stow / servo / pose / chunk) pauses the session (the PAUSED flag):
goals are dropped until it ends; the WBC then resyncs from the state packets before sending again.
"""
from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

from ..common import fastpath as F

N = F.NJ
# the arm node's telemetry enums (nodes/arm/telemetry.py STATES / SOURCES), by index
ARM_STATES = ["connecting", "uninitialized", "initializing", "off", "disabled", "floating", "servoing", "fault"]
ARM_SOURCES = ["none", "hold", "track", "servo", "pose", "home", "stow", "chunk", "stream"]


def joint_socket_path(arm: str = "arm") -> Path:
    """``<runtime>/robot/<arm instance>.joint.sock``."""
    from ..common.serve import check_name, socket_dir
    return socket_dir() / f"{check_name(arm)}.joint.sock"


@dataclass
class ArmState:
    """A RUS1 state packet (fastpath.State) with arrays and its arrival time."""
    flags: int
    session: int
    tick: int
    t_tick: float
    seq: int
    source: int
    state: int
    q: np.ndarray
    qd: np.ndarray
    tau: np.ndarray
    q_cmd: np.ndarray
    gap: float
    read_age: float
    t_rx: float = math.nan            # this host's (the WBC clock's) time of arrival

    @classmethod
    def of(cls, s: F.State, t_rx: float) -> "ArmState":
        return cls(s.flags, s.session, s.tick, s.t_tick, s.seq, s.source, s.state, np.array(s.q, float),
                   np.array(s.qd, float), np.array(s.tau, float), np.array(s.q_cmd, float), s.gap, s.read_age, t_rx)

    @property
    def state_name(self) -> str:
        return ARM_STATES[self.state] if 0 <= self.state < len(ARM_STATES) else f"state {self.state}"

    @property
    def servoing(self) -> bool:
        return bool(self.flags & F.STATE_SERVOING)

    @property
    def paused(self) -> bool:
        return bool(self.flags & F.STATE_PAUSED)

    @property
    def stale(self) -> bool:
        return bool(self.flags & F.STATE_STALE)


class ChannelError(RuntimeError):
    pass


class ArmChannel:
    """One session. Not thread safe: one owner (the WBC control thread), except ``open`` (the enable request, before
    the control thread uses it)."""

    def __init__(self, path, *, client: str = "wbc", local_dir: Optional[str] = None):
        self.path = str(path)
        self.client = client
        d = local_dir or os.path.dirname(self.path)
        self.local = os.path.join(d, f"{client}-{os.getpid()}-{id(self) & 0xFFFF:04x}.arm.sock")
        self.sock = None
        self.session: Optional[int] = None
        self.stale_ms: Optional[float] = None
        self.rate_hz: Optional[float] = None
        self.limits: dict = {}
        self.n: Optional[int] = None                  # the arm's joints (its state packets': 6 for a reBot, 7 ...)
        self.seq = 0
        self.closed_reason: Optional[str] = None     # the node's bye (or refusal) ended the session
        self.counts = {"sent": 0, "keepalives": 0, "send_errors": 0, "states": 0, "bad": 0, "other_session": 0}
        self.last_error: Optional[str] = None

    def open(self, token: str, timeout: float = 1.0) -> dict:
        """hello -> welcome; ChannelError with the reason otherwise. Blocking (up to ``timeout``)."""
        self.begin_open(token)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            m = self.poll_open()
            if m is not None:
                return m
            time.sleep(0.001)
        self.close(send_bye=False)
        raise ChannelError(f"no welcome from the arm joint channel within {timeout:g} s")

    def begin_open(self, token: str) -> None:
        """Bind and send the hello; ``poll_open`` then reads the answer (the control process never blocks on it)."""
        if not os.path.exists(self.path):
            raise ChannelError(f"no arm joint channel at {self.path} (an arm node without the joint channel?)")
        self.sock = F.bind_dgram(self.local)
        try:
            self.sock.sendto(F.encode_control(F.hello(token, self.client)), self.path)
        except OSError as e:
            self.close(send_bye=False)
            raise ChannelError(f"hello: {e}") from e

    def poll_open(self) -> Optional[dict]:
        """The welcome (the session is open), None (nothing yet); ChannelError if refused."""
        while True:
            try:
                data = self.sock.recv(65536)
            except (BlockingIOError, InterruptedError):
                return None
            if not F.is_control(data):
                continue                              # a state packet of an older session
            try:
                m = F.decode_control(data)
            except F.PacketError:
                continue
            if m["type"] == "welcome":
                self.session = int(m["session"])
                self.stale_ms = m.get("stale_ms")
                self.rate_hz = m.get("rate_hz")
                self.limits = m.get("limits") or {}
                self.seq = 0
                self.closed_reason = None
                return m
            if m["type"] == "refused":
                self.close(send_bye=False)
                raise ChannelError(f"refused: {m.get('reason')}")

    def close(self, send_bye: bool = True) -> None:
        if self.sock is not None:
            if send_bye and self.session is not None and self.closed_reason is None:
                try:
                    self.sock.sendto(F.encode_control(F.bye()), self.path)
                except OSError:
                    pass
            self.sock.close()
            self.sock = None
        self.session = None
        F.unlink_socket(self.local)

    @property
    def is_open(self) -> bool:
        return self.sock is not None and self.session is not None and self.closed_reason is None

    def send(self, q, qd, gap: float = math.nan, t_basis: float = math.nan, keepalive: bool = False) -> bool:
        if not self.is_open:
            return False
        self.seq += 1
        mode = F.MODE_KEEPALIVE if keepalive else F.MODE_GOAL
        try:
            if keepalive:
                n = self.n or F.NJ                       # the arm's joints (its state packets'): a keep-alive
                pkt = F.pack_command(self.session, self.seq, (math.nan,) * n, (0.0,) * n, mode=mode)
            else:
                pkt = F.pack_command(self.session, self.seq, np.asarray(q, float).tolist(),
                                     np.asarray(qd, float).tolist(), float(gap), mode, t_basis=float(t_basis))
            self.sock.sendto(pkt, self.path)
            self.counts["keepalives" if keepalive else "sent"] += 1
            return True
        except OSError as e:          # buffer full, peer gone: counted, never raised into the tick
            self.counts["send_errors"] += 1
            self.last_error = f"{type(e).__name__}: {e}"
            return False

    def poll(self, now: float) -> Optional[ArmState]:
        """Read everything pending; the newest state of this session (None if none arrived). A ``bye`` from the
        node sets ``closed_reason``."""
        latest = None
        if self.sock is None:
            return None
        while True:
            try:
                data = self.sock.recv(65536)
            except (BlockingIOError, InterruptedError):
                break
            except OSError as e:
                self.last_error = str(e)
                break
            try:
                if F.is_control(data):
                    m = F.decode_control(data)
                    if m["type"] in ("bye", "refused"):
                        self.closed_reason = str(m.get("reason") or m["type"])
                    continue
                st = F.unpack_state(data)
            except F.PacketError:
                self.counts["bad"] += 1
                continue
            if st.session != self.session:
                self.counts["other_session"] += 1
                continue
            self.counts["states"] += 1
            self.n = len(st.q)
            latest = st
        return None if latest is None else ArmState.of(latest, now)
