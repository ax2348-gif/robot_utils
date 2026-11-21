"""Client of the arm's internal joint channel (docs/plans/fast_path_v1.md "Local only", "Protocol (v1)").

A Unix datagram socket ``<runtime>/robot/<arm>.joint.sock`` (owner-only). Control datagrams are small JSON
objects; commands (``RUC1``, 88 bytes) and state (``RUS1``, 136 bytes) are fixed little-endian structs:

- ``{"type": "hello", "token": <arm lease token>, "client": "wbc"}`` → ``{"type": "welcome", "session": u32,
  "stale_ms": 30, "limits": {...}}`` or ``{"type": "refused", "reason": ...}``; ``{"type": "bye"}`` closes.
- command ``<4sBBHIIdd6f6fff``: magic ``RUC1``, version 1, mode (0 keep-alive, 1 joint goal), flags, session, seq,
  t_send, t_basis (the ``t_tick`` of the state the command was computed from, or NaN), q[6], qd[6], gap (NaN =
  unchanged), reserved.
- state ``<4sBBHIIdIBB2x6f6f6f6fff``: magic ``RUS1``, version, flags, session, tick, t_tick, seq in force,
  source, state, q[6], qd[6], tau[6], q_cmd[6], gap, read_age; sent every arm tick while the session is open.

The shared module (``nodes/common/fastpath.py``) is being written on another branch; this is the WBC's own copy
written against the documented layout, to be replaced by the shared one at the merge (the struct formats and the
tests stay). The state's ``flags`` bits (servoing 1, stale 2, jam 4, limited 8) are this module's assumption.
"""
from __future__ import annotations

import json
import math
import os
import socket
import struct
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

CMD_FMT = "<4sBBHIIdd6f6fff"
STATE_FMT = "<4sBBHIIdIBB2x6f6f6f6fff"
CMD_SIZE = struct.calcsize(CMD_FMT)       # 88
STATE_SIZE = struct.calcsize(STATE_FMT)   # 136
CMD_MAGIC, STATE_MAGIC, VERSION = b"RUC1", b"RUS1", 1
MODE_KEEPALIVE, MODE_GOAL = 0, 1
FLAG_SERVOING, FLAG_STALE, FLAG_JAM, FLAG_LIMITED = 1, 2, 4, 8
# the arm node's telemetry enums (nodes/arm/telemetry.py STATES / SOURCES), by index
ARM_STATES = ["connecting", "uninitialized", "initializing", "off", "disabled", "floating", "servoing", "fault"]
ARM_SOURCES = ["none", "hold", "track", "servo", "pose", "home", "stow", "chunk", "stream"]
N = 6

assert CMD_SIZE == 88 and STATE_SIZE == 136


def joint_socket_path(arm: str = "arm") -> Path:
    """``<runtime>/robot/<arm instance>.joint.sock``."""
    from ..common.serve import check_name, socket_dir
    return socket_dir() / f"{check_name(arm)}.joint.sock"


def pack_command(session: int, seq: int, q, qd, gap: float = math.nan, *, mode: int = MODE_GOAL,
                 t_send: Optional[float] = None, t_basis: float = math.nan) -> bytes:
    q = np.zeros(N) if q is None else np.asarray(q, float)
    qd = np.zeros(N) if qd is None else np.asarray(qd, float)
    return struct.pack(CMD_FMT, CMD_MAGIC, VERSION, int(mode), 0, int(session) & 0xFFFFFFFF, int(seq) & 0xFFFFFFFF,
                       time.monotonic() if t_send is None else float(t_send), float(t_basis),
                       *q.tolist(), *qd.tolist(), float(gap), 0.0)


@dataclass
class Command:
    mode: int
    flags: int
    session: int
    seq: int
    t_send: float
    t_basis: float
    q: np.ndarray
    qd: np.ndarray
    gap: float


def unpack_command(data: bytes) -> Optional[Command]:
    if len(data) != CMD_SIZE or data[:4] != CMD_MAGIC:
        return None
    v = struct.unpack(CMD_FMT, data)
    if v[1] != VERSION:
        return None
    return Command(v[2], v[3], v[4], v[5], v[6], v[7], np.array(v[8:14]), np.array(v[14:20]), v[20])


@dataclass
class ArmState:
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
    t_rx: float = math.nan           # this host's monotonic time of arrival

    @property
    def state_name(self) -> str:
        return ARM_STATES[self.state] if 0 <= self.state < len(ARM_STATES) else f"state {self.state}"

    @property
    def servoing(self) -> bool:
        return bool(self.flags & FLAG_SERVOING)


def pack_state(session: int, tick: int, t_tick: float, seq: int, source: int, state: int, q, qd, tau, q_cmd,
               gap: float = math.nan, read_age: float = 0.0, flags: int = 0) -> bytes:
    """For test peers (the arm node sends these)."""
    a = [np.asarray(x if x is not None else np.full(N, math.nan), float).tolist() for x in (q, qd, tau, q_cmd)]
    return struct.pack(STATE_FMT, STATE_MAGIC, VERSION, int(flags), 0, int(session), int(tick), float(t_tick),
                       int(seq), int(source), int(state), *a[0], *a[1], *a[2], *a[3], float(gap), float(read_age))


def unpack_state(data: bytes) -> Optional[ArmState]:
    if len(data) != STATE_SIZE or data[:4] != STATE_MAGIC:
        return None
    v = struct.unpack(STATE_FMT, data)
    if v[1] != VERSION:
        return None
    return ArmState(v[2], v[4], v[5], v[6], v[7], v[8], v[9], np.array(v[10:16]), np.array(v[16:22]),
                    np.array(v[22:28]), np.array(v[28:34]), v[34], v[35])


class ChannelError(RuntimeError):
    pass


class ArmChannel:
    """One session on the arm's joint channel. Not thread safe: one owner (the WBC control thread) except
    ``open`` (the enable request, before the control thread uses it)."""

    def __init__(self, path, *, client: str = "wbc", local_dir: Optional[str] = None):
        self.path = str(path)
        self.client = client
        d = local_dir or os.path.dirname(self.path) or tempfile.gettempdir()
        self.local = os.path.join(d, f"{client}-{os.getpid()}-{id(self) & 0xFFFF:04x}.arm.sock")
        self.sock: Optional[socket.socket] = None
        self.session: Optional[int] = None
        self.stale_ms: Optional[float] = None
        self.limits: dict = {}
        self.seq = 0
        self.closed_reason: Optional[str] = None     # a control message ended the session
        self.counts = {"sent": 0, "send_errors": 0, "states": 0, "bad": 0, "other_session": 0}
        self.last_error: Optional[str] = None

    # -- lifecycle -------------------------------------------------------------------------------------

    def _bind(self) -> None:
        if os.path.exists(self.local):
            os.unlink(self.local)
        s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        s.bind(self.local)
        os.chmod(self.local, 0o600)
        s.setblocking(False)
        self.sock = s

    def open(self, token: str, timeout: float = 1.0) -> dict:
        """hello → welcome; raises ChannelError with the reason otherwise."""
        if not os.path.exists(self.path):
            raise ChannelError(f"no arm joint channel at {self.path} (an arm node without the fast path?)")
        self._bind()
        msg = json.dumps({"type": "hello", "token": token, "client": self.client}).encode()
        try:
            self.sock.sendto(msg, self.path)
        except OSError as e:
            self.close(send_bye=False)
            raise ChannelError(f"hello: {e}") from e
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                data = self.sock.recv(4096)
            except BlockingIOError:
                time.sleep(0.001)
                continue
            if data[:1] != b"{":
                continue                              # a state packet of an older session: ignore
            try:
                m = json.loads(data)
            except ValueError:
                continue
            if m.get("type") == "welcome":
                self.session = int(m["session"])
                self.stale_ms = m.get("stale_ms")
                self.limits = m.get("limits") or {}
                self.seq = 0
                self.closed_reason = None
                return m
            if m.get("type") == "refused":
                self.close(send_bye=False)
                raise ChannelError(f"refused: {m.get('reason')}")
        self.close(send_bye=False)
        raise ChannelError(f"no welcome from the arm joint channel within {timeout:g} s")

    def close(self, send_bye: bool = True) -> None:
        if self.sock is not None:
            if send_bye and self.session is not None:
                try:
                    self.sock.sendto(json.dumps({"type": "bye", "session": self.session}).encode(), self.path)
                except OSError:
                    pass
            self.sock.close()
            self.sock = None
        self.session = None
        try:
            os.unlink(self.local)
        except OSError:
            pass

    @property
    def is_open(self) -> bool:
        return self.sock is not None and self.session is not None and self.closed_reason is None

    # -- data ------------------------------------------------------------------------------------------

    def send(self, q, qd, gap: float = math.nan, t_basis: float = math.nan, keepalive: bool = False) -> bool:
        if not self.is_open:
            return False
        self.seq += 1
        pkt = pack_command(self.session, self.seq, q, qd, gap, mode=MODE_KEEPALIVE if keepalive else MODE_GOAL,
                           t_basis=t_basis)
        try:
            self.sock.sendto(pkt, self.path)
            self.counts["sent"] += 1
            return True
        except OSError as e:          # buffer full, peer gone: counted, never raised into the tick
            self.counts["send_errors"] += 1
            self.last_error = f"{type(e).__name__}: {e}"
            return False

    def poll(self) -> Optional[ArmState]:
        """Read everything pending; return the newest state of this session (None if none arrived)."""
        latest = None
        if self.sock is None:
            return None
        while True:
            try:
                data = self.sock.recv(4096)
            except (BlockingIOError, InterruptedError):
                break
            except OSError as e:
                self.last_error = str(e)
                break
            if data[:1] == b"{":
                try:
                    m = json.loads(data)
                except ValueError:
                    self.counts["bad"] += 1
                    continue
                if m.get("type") in ("closed", "bye", "refused"):
                    self.closed_reason = str(m.get("reason") or m.get("type"))
                continue
            st = unpack_state(data)
            if st is None:
                self.counts["bad"] += 1
                continue
            if st.session != self.session:
                self.counts["other_session"] += 1
                continue
            st.t_rx = time.monotonic()
            self.counts["states"] += 1
            latest = st
        return latest
