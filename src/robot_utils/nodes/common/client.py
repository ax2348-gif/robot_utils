"""Python client base for any node: clock sync, JSON calls, and record streams. Needs httpx.

    from robot_utils.nodes.common.client import NodeClient

    node = NodeClient("swerve")                  # instance name: its unix socket, on the robot
    node = NodeClient("http://botcore/nodes/swerve")   # from another machine (through the proxy)
    est = node.sync()                       # NTP-style; node.clock.to_server(time.monotonic())
    for header, rows in node.rows("/stream"):
        print(rows["t_tick"][-1], est.uncertainty)

Recording on the client: ``node.record("/stream", "run1.bin", seconds=10)`` writes the raw
stream; read it back with ``protocol.iter_records`` / ``protocol.iter_rows``.

Reaching another node by instance name (the one mechanism every node uses for its peers):
``node_url(name)`` / ``NodeClient.for_instance(name)`` use the instance's unix socket
``socket_path(name)`` when it exists on this machine, else ``http://localhost/nodes/<name>``
(the robot's port-80 proxy). A URL is used as given.
"""
from __future__ import annotations

import time
from typing import Iterator, Optional

import httpx

from .protocol import RecordReader, RowsDecoder
from .serve import check_name, socket_path   # noqa: F401 (socket_path re-exported)
from .timesync import ClockSync, SyncEstimate, SyncSample, estimate_offset


class NodeError(RuntimeError):
    def __init__(self, status: int, detail: str):
        super().__init__(f"{status}: {detail}")
        self.status = status
        self.detail = detail


def _resolve(name_or_url: str) -> tuple[str, Optional[str]]:
    """(base_url, unix socket or None): a URL as given; an instance name -> its socket if that
    exists here, else through the robot's port-80 proxy on this machine."""
    if "://" in name_or_url:
        return name_or_url.rstrip("/"), None
    path = socket_path(check_name(name_or_url))
    if path.exists():
        return f"http://{name_or_url}", str(path)
    return f"http://localhost/nodes/{name_or_url}", None


def node_url(name_or_url: str) -> str:
    """Base URL for a node: a URL passes through; an instance name -> ``http://<name>`` when its
    unix socket exists (use it with ``NodeClient.for_instance``, which adds the socket
    transport), else ``http://localhost/nodes/<name>`` via nginx."""
    return _resolve(name_or_url)[0]


class NodeClient:
    @classmethod
    def for_instance(cls, name_or_url: str, **kw) -> "NodeClient":
        """Client for an instance name (its unix socket if it exists, else via nginx on
        localhost) or a URL. Same as ``NodeClient(name_or_url)``."""
        return cls(name_or_url, **kw)

    def __init__(self, target: str, timeout: float = 5.0):
        """``target``: a URL (``"http://botcore/nodes/swerve"``) or an instance name on this
        machine (``"swerve"``: its unix socket, else ``http://localhost/nodes/swerve``)."""
        self.base_url, uds = _resolve(target)
        transport = httpx.HTTPTransport(uds=uds) if uds else None
        self.http = httpx.Client(base_url=self.base_url, timeout=timeout, transport=transport)
        self.clock = ClockSync()
        self.headers: dict[str, str] = {}

    def close(self) -> None:
        self.http.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # -- JSON ----------------------------------------------------------------------------------

    def _check(self, r: httpx.Response):
        if r.status_code >= 400:
            try:
                detail = r.json().get("detail", r.text)
            except ValueError:
                detail = r.text
            raise NodeError(r.status_code, str(detail))
        return r.json() if r.content else None

    def get(self, path: str, **params):
        return self._check(self.http.get(path, params=params, headers=self.headers))

    def post(self, path: str, json=None, **params):
        return self._check(self.http.post(path, json=json, params=params, headers=self.headers))

    def delete(self, path: str):
        return self._check(self.http.delete(path, headers=self.headers))

    # -- time ----------------------------------------------------------------------------------

    def sync(self, n: int = 16) -> SyncEstimate:
        """One NTP-style burst against GET /time over a kept-alive connection."""
        self.http.get("/time")   # warm the connection
        samples = []
        for _ in range(n):
            t0 = time.monotonic()
            r = self.http.get("/time")
            t1 = time.monotonic()
            samples.append(SyncSample(t0, r.json()["mono"], t1))
        est = estimate_offset(samples)
        self.clock.add(est)
        return est

    # -- streams -------------------------------------------------------------------------------

    def records(self, path: str, **params) -> Iterator[tuple[dict, bytes, float]]:
        """(header, payload, t_receive) for every record of a streaming endpoint (client monotonic).

        Bytes are handed on as they arrive (``iter_raw()`` without a chunk size: with one, httpx
        waits until that many bytes have accumulated, which delayed 100 Hz ticks by ~120 ms)."""
        reader = RecordReader()
        with self.http.stream("GET", path, params=params, timeout=None, headers=self.headers) as r:
            if r.status_code >= 400:
                r.read()
                self._check(r)
            for data in r.iter_raw():
                t = time.monotonic()
                for h, p in reader.feed(data):
                    yield h, p, t

    def rows(self, path: str = "/stream", **params):
        """(header, record array) for every rows record of a stream (schema handled for you)."""
        dec = RowsDecoder()
        for h, p, _ in self.records(path, **params):
            rows = dec.decode(h, p)
            if rows is not None:
                yield h, rows

    def record(self, path: str, filename: str, seconds: Optional[float] = None, **params) -> int:
        """Write a stream's raw bytes to a file (the client-side recording); returns bytes written."""
        n, t_end = 0, None if seconds is None else time.monotonic() + seconds
        with open(filename, "wb") as f, httpx.stream("GET", self.base_url + path, params=params,
                                                     timeout=None, headers=self.headers) as r:
            for data in r.iter_raw():
                f.write(data)
                n += len(data)
                if t_end is not None and time.monotonic() > t_end:
                    break
        return n
