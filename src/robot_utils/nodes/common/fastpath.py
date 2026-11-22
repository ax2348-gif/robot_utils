"""Fast-path datagrams: the joint channel's binary packets and JSON control messages. Pure Python (``struct``,
``socket``, ``json``); no arm code, so other nodes (the WBC) reuse it for their own channels.

Internal, robot-local interface (docs/arm.md "Internal joint channel"; design docs/plans/fast_path_v1.md "Local
only", contract docs/plans/wbc_v1.md "Interfaces for the first build" 1). A Unix datagram socket, owner-only; no
UDP, no HTTP. Three kinds of datagram travel on it, told apart by their first bytes:

- ``RUC1`` command, client -> node, 88 bytes, little-endian ``<4sBBHIIdd6f6fff``: magic, version (1), mode
  (0 keep-alive, 1 joint goal), flags (reserved 0), session, seq, t_send, t_basis, q (6), qd (6), gap, reserved.
- ``RUS1`` state, node -> client, 136 bytes, ``<4sBBHIIdIBB2x6f6f6f6fff``: magic, version, a reserved u8 (0; the
  command's ``mode`` position), flags (u16, STATE_*), session, tick, t_tick, seq in force, source, state, q, qd, tau,
  q_cmd (6 each), gap, read_age.
- control: a JSON object (first byte ``{``) with ``type``: ``hello`` / ``bye`` (client -> node), ``welcome`` /
  ``refused`` / ``bye`` (node -> client).

``unpack_*`` raise ``PacketError`` with a short ``reason`` (``size``, ``magic``, ``version``, ``json``) that a
receiver counts; nothing here logs.
"""
from __future__ import annotations

import errno
import json
import math
import os
import socket
import stat
import struct
import time
from pathlib import Path
from typing import NamedTuple, Optional, Sequence

VERSION = 1
CMD_MAGIC = b"RUC1"
STATE_MAGIC = b"RUS1"
CMD = struct.Struct("<4sBBHIIdd6f6fff")
STATE = struct.Struct("<4sBBHIIdIBB2x6f6f6f6fff")
CMD_SIZE = CMD.size          # 88
STATE_SIZE = STATE.size      # 136
NJ = 6                       # joints per packet

MODE_KEEPALIVE = 0           # renews (the lease, the session), no goal change
MODE_GOAL = 1                # joint goal (q, qd feedforward, gap)

# RUS1 flags (bits of the u16)
STATE_SERVOING = 1 << 0      # the arm is servoing (closed loop)
STATE_STALE = 1 << 1         # the stream's newest goal is older than stale_ms: ramping to a hold / holding
STATE_JAM = 1 << 2           # a jam is in force
STATE_LIMITED = 1 << 3       # the joint speed limit held the command back this tick
STATE_PAUSED = 1 << 4        # a planned motion (HTTP) paused the session: goals are dropped until it ends

MAX_CONTROL = 4096           # bytes a control message may have

assert CMD_SIZE == 88 and STATE_SIZE == 136


class PacketError(ValueError):
    def __init__(self, reason: str, msg: str = ""):
        super().__init__(msg or reason)
        self.reason = reason


class Command(NamedTuple):
    mode: int
    flags: int
    session: int
    seq: int
    t_send: float
    t_basis: float
    q: tuple
    qd: tuple
    gap: float
    reserved: float


class State(NamedTuple):
    flags: int
    session: int
    tick: int
    t_tick: float
    seq: int
    source: int
    state: int
    q: Sequence[float]
    qd: Sequence[float]
    tau: Sequence[float]
    q_cmd: Sequence[float]
    gap: float
    read_age: float


# -- binary packets --------------------------------------------------------------------------------------

def pack_command(session: int, seq: int, q: Sequence[float] = (math.nan,) * NJ,
                 qd: Sequence[float] = (0.0,) * NJ, gap: float = math.nan, mode: int = MODE_GOAL,
                 t_send: Optional[float] = None, t_basis: float = math.nan, flags: int = 0,
                 reserved: float = 0.0) -> bytes:
    """An RUC1 command. ``t_send`` defaults to the sender's monotonic clock; ``t_basis`` is the ``t_tick`` of the
    state packet the goal was computed from (NaN: none)."""
    return CMD.pack(CMD_MAGIC, VERSION, mode, flags, session, seq,
                    time.monotonic() if t_send is None else t_send, t_basis, *q, *qd, gap, reserved)


def unpack_command(buf) -> Command:
    if len(buf) != CMD_SIZE:
        raise PacketError("size", f"a command is {CMD_SIZE} bytes, got {len(buf)}")
    v = CMD.unpack(buf)
    if v[0] != CMD_MAGIC:
        raise PacketError("magic", f"bad magic {v[0]!r}")
    if v[1] != VERSION:
        raise PacketError("version", f"unknown version {v[1]}")
    return Command(v[2], v[3], v[4], v[5], v[6], v[7], v[8:14], v[14:20], v[20], v[21])


def pack_state(s: State) -> bytes:
    return STATE.pack(STATE_MAGIC, VERSION, 0, s.flags, s.session, s.tick, s.t_tick, s.seq, s.source, s.state,
                      *s.q, *s.qd, *s.tau, *s.q_cmd, s.gap, s.read_age)


def unpack_state(buf) -> State:
    if len(buf) != STATE_SIZE:
        raise PacketError("size", f"a state packet is {STATE_SIZE} bytes, got {len(buf)}")
    v = STATE.unpack(buf)
    if v[0] != STATE_MAGIC:
        raise PacketError("magic", f"bad magic {v[0]!r}")
    if v[1] != VERSION:
        raise PacketError("version", f"unknown version {v[1]}")
    return State(v[3], v[4], v[5], v[6], v[7], v[8], v[9], v[10:16], v[16:22], v[22:28], v[28:34], v[34], v[35])


# -- control messages --------------------------------------------------------------------------------------

def is_control(buf) -> bool:
    return len(buf) > 0 and buf[0:1] == b"{"


def encode_control(msg: dict) -> bytes:
    """A control message (``type`` required); NaN / inf are not JSON and are refused."""
    if not isinstance(msg.get("type"), str):
        raise ValueError("a control message needs a string 'type'")
    return json.dumps(msg, allow_nan=False, separators=(",", ":")).encode()


def decode_control(buf) -> dict:
    if len(buf) > MAX_CONTROL:
        raise PacketError("size", f"a control message is at most {MAX_CONTROL} bytes")
    try:
        msg = json.loads(bytes(buf))
    except (ValueError, UnicodeDecodeError) as e:
        raise PacketError("json", f"not JSON: {e}") from None
    if not isinstance(msg, dict) or not isinstance(msg.get("type"), str):
        raise PacketError("json", "a control message is a JSON object with a string 'type'")
    return msg


def hello(token: str, client: str) -> dict:
    return {"type": "hello", "token": token, "client": client}


def welcome(session: int, stale_ms: float, limits: dict, **extra) -> dict:
    return {"type": "welcome", "session": session, "stale_ms": stale_ms, "limits": limits, **extra}


def refused(reason: str) -> dict:
    return {"type": "refused", "reason": reason}


def bye(reason: Optional[str] = None) -> dict:
    return {"type": "bye"} if reason is None else {"type": "bye", "reason": reason}


# -- sockets ----------------------------------------------------------------------------------------------

def _dgram_alive(path: Path) -> bool:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        s.connect(str(path))
        return True
    except OSError:
        return False
    finally:
        s.close()


def bind_dgram(path) -> socket.socket:
    """A Unix datagram socket bound at ``path``, owner-only (0600), non-blocking. Its directory is created 0700 (the
    runtime dir already is), which also covers the moment between bind and chmod. (Binding under a temporary name
    and renaming would not do: a peer's ``recvfrom`` reports the name the socket was bound to.) A stale socket file
    is replaced, a live one refused, anything else at ``path`` is left alone (RuntimeError)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    if path.exists() or path.is_symlink():
        if not stat.S_ISSOCK(path.lstat().st_mode):
            raise RuntimeError(f"{path} exists and is not a socket; not touching it")
        if _dgram_alive(path):
            raise RuntimeError(f"already running: something is bound to {path}")
        path.unlink()
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        sock.bind(str(path))
        os.chmod(path, 0o600)
    except BaseException:
        sock.close()
        raise
    sock.setblocking(False)
    return sock


def unlink_socket(path) -> None:
    try:
        p = Path(path)
        if p.is_socket():
            p.unlink()
    except OSError:
        pass


class Client:
    """A small blocking client of a joint channel (tests, tools, a reference for real clients): binds its own
    socket (the node sends state packets there), says hello, sends commands, receives state packets."""

    def __init__(self, server_path, client_path):
        self.server = str(server_path)
        self.path = Path(client_path)
        self.sock = bind_dgram(self.path)
        self.sock.setblocking(True)
        self.session: Optional[int] = None
        self.seq = 0
        self.welcome: Optional[dict] = None
        self.controls: list = []          # control messages received while waiting for state

    def _send(self, data: bytes) -> None:
        self.sock.sendto(data, self.server)

    def control(self, msg: dict, timeout: float = 2.0) -> dict:
        """Send a control message and wait for the node's reply (state packets meanwhile are skipped)."""
        self._send(encode_control(msg))
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                raise TimeoutError(f"no reply to {msg.get('type')}")
            self.sock.settimeout(left)
            data = self.sock.recv(65536)
            if is_control(data):
                return decode_control(data)

    def hello(self, token: str, client: str = "client", timeout: float = 2.0) -> dict:
        r = self.control(hello(token, client), timeout)
        if r["type"] == "welcome":
            self.session, self.seq, self.welcome = r["session"], 0, r
        return r

    def send(self, q=(math.nan,) * NJ, qd=(0.0,) * NJ, gap: float = math.nan, mode: int = MODE_GOAL,
             t_basis: float = math.nan, seq: Optional[int] = None, session: Optional[int] = None) -> int:
        if seq is None:
            self.seq += 1
            seq = self.seq
        self._send(pack_command(self.session if session is None else session, seq, q, qd, gap, mode,
                                t_basis=t_basis))
        return seq

    def keepalive(self) -> int:
        return self.send(mode=MODE_KEEPALIVE)

    def recv(self, timeout: Optional[float] = 1.0):
        """The next datagram from the node: a State, or a control dict (also kept in ``controls``). None on timeout."""
        self.sock.settimeout(timeout)
        try:
            data = self.sock.recv(65536)
        except (socket.timeout, BlockingIOError):
            return None
        if is_control(data):
            m = decode_control(data)
            self.controls.append(m)
            return m
        return unpack_state(data)

    def latest_state(self, timeout: float = 1.0) -> Optional[State]:
        """The newest state packet queued (drains the queue; waits up to ``timeout`` for one if none)."""
        st = None
        self.sock.setblocking(False)
        try:
            while True:
                try:
                    data = self.sock.recv(65536)
                except BlockingIOError:
                    break
                if is_control(data):
                    self.controls.append(decode_control(data))
                else:
                    st = unpack_state(data)
        finally:
            self.sock.setblocking(True)
        if st is None:
            r = self.recv(timeout)
            while isinstance(r, dict):
                r = self.recv(timeout)
            st = r
        return st

    def bye(self) -> None:
        try:
            self._send(encode_control(bye()))
        except OSError as e:
            if e.errno not in (errno.ECONNREFUSED, errno.ENOENT):
                raise
        self.session = None

    def close(self) -> None:
        self.sock.close()
        unlink_socket(self.path)
