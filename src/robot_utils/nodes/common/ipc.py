"""Between a node's two processes (a server process and its real-time control process: docs/wbc.md and docs/arm.md
"Processes"): one shared-memory file and two Unix datagram sockets.

Shared memory (an ``mmap`` of a file in a private 0700 directory on tmpfs: ``$XDG_RUNTIME_DIR`` on the robot):

- ``Beats``: float64 cells (heartbeats, counters); one writer per cell.
- ``Slot``: the latest value of a picklable object behind a seqlock (+ a CRC32 of the bytes), one writer. A reader
  never waits: a write in progress (or a torn read) returns the previous value. A Python store has no memory
  barrier; the CRC catches a reordered (torn) read on a weakly ordered CPU (the Pi's aarch64) as well.
- ``Ring``: fixed-width float64 rows, one producer, one consumer, drop-oldest (counted); each row is tagged with its
  index, so a row overwritten while it was copied is dropped, never mixed.

``SharedFile`` lays them out from a spec, the same on both sides.

Commands (``Port``): pickled messages on Unix datagram sockets (atomic, non-blocking; a full buffer drops the message
and the caller sees ``False``). Both sockets live in the private directory.

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


def runtime_dir(prefix: str) -> str:
    """A private (0700) directory for the shared memory and the command sockets: on tmpfs (``$XDG_RUNTIME_DIR``,
    ``/dev/shm``) where there is one; ``/tmp`` otherwise (macOS: short, for the 104-byte socket path limit)."""
    for d in (os.environ.get("XDG_RUNTIME_DIR"), "/dev/shm", "/tmp"):
        if d and os.path.isdir(d) and os.access(d, os.W_OK):
            return tempfile.mkdtemp(prefix=prefix, dir=d)
    return tempfile.mkdtemp(prefix=prefix)


class Beats:
    def __init__(self, buf, off: int, n: int = 16):
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


class SharedFile:
    """A shared-memory file laid out by ``spec``: ``(name, kind, param)`` with kind ``beats`` (param: cells),
    ``slot`` (param: bytes) or ``ring`` (param: (rows, width)). Each part becomes an attribute at a fixed offset
    computed from the same spec on both sides; ``create`` makes the file (the owner), else it is opened."""

    def __init__(self, path: str, spec: list, create: bool):
        self.path = path
        self._names = [name for name, _, _ in spec]
        offs, size = {}, 0
        for name, kind, p in spec:
            offs[name] = size
            nb = (Slot.HDR + p) if kind == "slot" else Ring.nbytes(*p) if kind == "ring" else 8 * p
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
        # Every page allocated and mapped now, not on its first write from the control loop (a page fault there:
        # the file's page allocated, on macOS's disk-backed /tmp even I/O). The creator writes zeros (nothing else
        # uses the file yet); the other side reads one byte per page (it must not write: the creator's beats run).
        if create:
            self.mm.write(bytes(size))
            self.mm.seek(0)
        else:
            for i in range(0, size, mmap.PAGESIZE):
                self.mm[i]
        b = self.mm
        for name, kind, p in spec:
            o = offs[name]
            obj = Beats(b, o, p) if kind == "beats" else Slot(b, o, p) if kind == "slot" else Ring(b, o, *p)
            setattr(self, name, obj)

    def close(self) -> None:
        for k in self._names:
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
        return self.send_bytes(pickle.dumps(msg, protocol=pickle.HIGHEST_PROTOCOL))

    def send_bytes(self, data: bytes) -> bool:
        """A pickled message. Linux caps a datagram at the socket's send buffer (``net.core.wmem_max``, 208 KiB by
        default): a larger message is refused (``False``); put it in a shared slot instead."""
        try:
            self.sock.sendto(data, self.peer)
            return True
        except OSError:                    # the peer's buffer full, too large, or gone: dropped (never blocks)
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
