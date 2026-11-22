"""Between the WBC's two processes (docs/wbc.md "Processes"): one shared-memory file and two Unix datagram sockets.

Shared memory (an ``mmap`` of a file in a private 0700 directory on tmpfs: ``$XDG_RUNTIME_DIR`` on the robot):

- ``Beats``: float64 cells (the heartbeats, the datagram count that renews the client lease); one writer per cell.
- ``Slot``: the latest value of a picklable object behind a seqlock (+ a CRC32 of the bytes), one writer. A reader
  never waits: a write in progress (or a torn read) returns the previous value. A Python store has no memory
  barrier; the CRC catches a reordered (torn) read on a weakly ordered CPU (the Pi's aarch64) as well.
- ``Ring``: fixed-width float64 rows, one producer, one consumer, drop-oldest (counted); each row is tagged with its
  index, so a row overwritten while it was copied is dropped, never mixed.

Commands (``Port``): pickled messages on Unix datagram sockets (atomic, non-blocking on the control side; a full
buffer drops the message and the caller sees ``False``). Both sockets live in the private directory.

Not ``multiprocessing.shared_memory``: in Python 3.12 its resource tracker unlinks or warns about a block attached by a
spawned child; a file of our own has no such machinery.
"""
from __future__ import annotations

import mmap
import os
import pickle
import socket
import tempfile
import zlib
from typing import Optional

import numpy as np

ALIGN = 64

# Beats cells
SRV_HB = 0        # the server process's heartbeat (time.monotonic, written every few ms)
CTL_HB = 1        # the control process's heartbeat (every tick)
RX_COUNT = 2      # datagrams of the session in force (each renews the client lease: the server renews it)
CTL_PID = 3
N_BEATS = 16

# the estimator and swerve rows the server forwards to the control process (the fields the control tick uses)
EST_COLS = ("t", "x", "y", "theta", "reset_id", "imu_ok", "odom_ok")
SW_COLS = ("t", "enabled", "estop", "lease_id")


def runtime_dir() -> str:
    """A private (0700) directory for the shared memory and the command sockets: on tmpfs (``$XDG_RUNTIME_DIR``,
    ``/dev/shm``) where there is one; ``/tmp`` otherwise (macOS: short, for the 104-byte socket path limit)."""
    for d in (os.environ.get("XDG_RUNTIME_DIR"), "/dev/shm", "/tmp"):
        if d and os.path.isdir(d) and os.access(d, os.W_OK):
            return tempfile.mkdtemp(prefix="wbc-", dir=d)
    return tempfile.mkdtemp(prefix="wbc-")


class Beats:
    def __init__(self, buf, off: int, n: int = N_BEATS):
        self.a = np.ndarray(n, np.float64, buf, off)

    def __getitem__(self, i: int) -> float:
        return float(self.a[i])

    def __setitem__(self, i: int, v: float) -> None:
        self.a[i] = v


class Slot:
    """The latest value of an object (pickled), behind a seqlock. One writer."""
    HDR = 24          # seq, length, crc (uint64 each)

    def __init__(self, buf, off: int, size: int):
        self.hdr = np.ndarray(3, np.uint64, buf, off)
        self.data = np.ndarray(size, np.uint8, buf, off + self.HDR)
        self.size = size
        self._seq: Optional[int] = None
        self._last = None
        self.torn = 0

    def write(self, obj) -> None:
        b = pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL)
        n = len(b)
        if n > self.size:
            raise ValueError(f"slot too small: {n} > {self.size} bytes")
        s = int(self.hdr[0])
        self.hdr[0] = s + 1                     # odd: a write in progress
        self.data[:n] = np.frombuffer(b, np.uint8)
        self.hdr[1] = n
        self.hdr[2] = zlib.crc32(b)
        self.hdr[0] = s + 2

    @property
    def seq(self) -> int:
        return int(self.hdr[0])

    def read(self, tries: int = 3):
        """The latest value (the cached one if unchanged, a write is in progress or the read was torn)."""
        for _ in range(tries):
            s1 = int(self.hdr[0])
            if s1 == self._seq:
                return self._last
            if s1 & 1:
                continue
            n, crc = int(self.hdr[1]), int(self.hdr[2])
            if n > self.size:
                continue
            b = self.data[:n].tobytes()
            if int(self.hdr[0]) != s1 or zlib.crc32(b) != crc:
                self.torn += 1
                continue
            try:
                self._last = pickle.loads(b) if n else None
            except Exception:  # noqa: BLE001 - torn despite the checks: keep the previous value
                self.torn += 1
                continue
            self._seq = s1
            return self._last
        return self._last


class Ring:
    """Fixed-width float64 rows: one producer, one consumer, drop-oldest."""

    def __init__(self, buf, off: int, n: int, width: int):
        self.n, self.width = n, width
        self.w = np.ndarray(1, np.uint64, buf, off)
        self.tags = np.ndarray(n, np.uint64, buf, off + 8)
        self.rows = np.ndarray((n, width), np.float64, buf, off + 8 + 8 * n)
        self.r: Optional[int] = None       # the consumer's next index (its own, not shared)
        self.dropped = 0

    @staticmethod
    def nbytes(n: int, width: int) -> int:
        return 8 + 8 * n + 8 * n * width

    def write(self, row) -> None:
        i = int(self.w[0])
        k = i % self.n
        self.tags[k] = np.uint64(0xFFFFFFFFFFFFFFFF)    # being written
        self.rows[k] = row
        self.tags[k] = i
        self.w[0] = i + 1

    def write_many(self, rows) -> None:
        for r in np.atleast_2d(rows):
            self.write(r)

    def read(self, from_start: bool = True) -> np.ndarray:
        """The rows written since the previous read (oldest first); overflowed ones are dropped and counted."""
        w = int(self.w[0])
        if self.r is None:
            self.r = 0 if from_start else w
        r = self.r
        cap = self.n - 1                  # the slot after the newest may be the one being written
        if w - r > cap:
            self.dropped += w - r - cap
            r = w - cap
        if w <= r:
            self.r = max(r, w)
            return np.empty((0, self.width))
        idx = np.arange(r, w)
        k = idx % self.n
        out = self.rows[k].copy()
        ok = self.tags[k] == idx.astype(np.uint64)       # overwritten (or being written) during the copy: dropped
        if not ok.all():
            self.dropped += int((~ok).sum())
            out = out[ok]
        self.r = w
        return out


class Shared:
    """The shared-memory file: beats, the slots and the rings, at fixed offsets computed from the same layout on both
    sides."""

    def __init__(self, path: str, width: int, create: bool):
        self.path = path
        spec = [("beats", "beats", 8 * N_BEATS),
                ("twist", "slot", 1024),           # control -> server: the base twist command
                ("srv", "slot", 8192),             # server -> control: lease held, the base sender's health
                ("snap", "slot", 1 << 19),         # control -> server: the state snapshot (/state)
                ("rows", "ring", (2048, width)),   # control -> server: the stream rows
                ("est", "ring", (1024, len(EST_COLS))),     # server -> control: estimator rows
                ("sw", "ring", (256, len(SW_COLS)))]        # server -> control: swerve rows
        offs, size = {}, 0
        for name, kind, p in spec:
            offs[name] = size
            nb = (Slot.HDR + p) if kind == "slot" else Ring.nbytes(*p) if kind == "ring" else p
            size += (nb + ALIGN - 1) // ALIGN * ALIGN
        if create:
            fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_EXCL, 0o600)
            os.ftruncate(fd, size)
        else:
            fd = os.open(path, os.O_RDWR)
        try:
            self.mm = mmap.mmap(fd, size)
        finally:
            os.close(fd)
        b = self.mm
        for name, kind, p in spec:
            o = offs[name]
            obj = Beats(b, o) if kind == "beats" else Slot(b, o, p) if kind == "slot" else Ring(b, o, *p)
            setattr(self, name, obj)

    def close(self) -> None:
        for k in ("beats", "twist", "srv", "snap", "rows", "est", "sw"):
            setattr(self, k, None)
        try:
            self.mm.close()
        except (BufferError, ValueError):
            pass                           # a numpy view still alive somewhere: the mapping goes with the process


class Port:
    """One end of the command channel: pickled messages on a Unix datagram socket bound at ``local``, sent to
    ``peer``."""
    BUF = 1 << 20

    def __init__(self, local: str, peer: str):
        self.local, self.peer = local, peer
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        for opt in (socket.SO_SNDBUF, socket.SO_RCVBUF):
            try:
                self.sock.setsockopt(socket.SOL_SOCKET, opt, self.BUF)   # macOS: the largest datagram is SO_SNDBUF
            except OSError:
                pass
        self.sock.bind(local)
        os.chmod(local, 0o600)
        self.sock.setblocking(False)
        self.dropped = 0

    def fileno(self) -> int:
        return self.sock.fileno()

    def send(self, msg) -> bool:
        try:
            self.sock.sendto(pickle.dumps(msg, protocol=pickle.HIGHEST_PROTOCOL), self.peer)
            return True
        except OSError:                    # the peer's buffer full, or gone: dropped (never blocks)
            self.dropped += 1
            return False

    def recv_all(self, limit: int = 1000) -> list:
        out = []
        while len(out) < limit:
            try:
                data = self.sock.recv(self.BUF)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            try:
                out.append(pickle.loads(data))
            except Exception:  # noqa: BLE001
                self.dropped += 1
        return out

    def close(self) -> None:
        try:
            self.sock.close()
        finally:
            try:
                os.unlink(self.local)
            except OSError:
                pass
