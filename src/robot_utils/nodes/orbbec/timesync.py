"""Clock handling shared by the server and its clients (pure Python, no hardware).

The server's reference clock is ``time.monotonic()`` (CLOCK_MONOTONIC on Linux: system wide, so
every node on the same machine shares it; it never steps, unlike the wall clock).

Server side: the Orbbec SDK stamps frames with the host *wall* clock (Unix epoch, microseconds).
``epoch_minus_mono()`` measures the current wall - monotonic offset (measure it fresh for every
frame: NTP can step or slew the wall clock) and ``epoch_to_mono`` converts.

Client side: an NTP-style exchange against ``GET /time``. The client reads its own monotonic clock
before (t0) and after (t1) the request; the server answers with its monotonic time ts taken in
between. Then the true offset (server - client) lies in ``[ts - t1, ts - t0]``:

    offset      = ts - (t0 + t1) / 2
    uncertainty = (t1 - t0) / 2        (half the round trip; a hard bound, not a std dev)

Several samples are taken; the bound intervals are intersected (each one must contain the true
offset if the clocks do not drift during the burst), which is at least as tight as the best
single sample. ``ClockSync`` keeps the history of bursts and, once they span long enough, fits a
linear drift so ``to_server()`` stays good between re-syncs.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Iterable, Optional, Sequence


def epoch_minus_mono() -> float:
    """time.time() - time.monotonic(), read with the monotonic clock on both sides (seconds)."""
    m0 = time.monotonic()
    w = time.time()
    m1 = time.monotonic()
    return w - (m0 + m1) / 2


def epoch_to_mono(t_epoch: float, offset: Optional[float] = None) -> float:
    """Convert a wall-clock (Unix epoch) time to this machine's monotonic clock."""
    return t_epoch - (epoch_minus_mono() if offset is None else offset)


@dataclass(frozen=True)
class SyncSample:
    """One request/response exchange: client t0 (sent), server ts, client t1 (received)."""
    t0: float
    ts: float
    t1: float

    @property
    def rtt(self) -> float:
        return self.t1 - self.t0

    @property
    def offset(self) -> float:
        """Server clock minus client clock, assuming a symmetric path."""
        return self.ts - (self.t0 + self.t1) / 2

    @property
    def bounds(self) -> tuple[float, float]:
        """Interval that must contain the true offset (request and response take >= 0 s)."""
        return self.ts - self.t1, self.ts - self.t0


@dataclass(frozen=True)
class SyncEstimate:
    offset: float          # server - client (s): t_server = t_client + offset
    uncertainty: float     # half width of the interval the true offset lies in (s)
    rtt_min: float         # smallest round trip in the burst (s)
    rtt_median: float
    n: int                 # samples in the burst
    t_client: float        # client time of the estimate (middle of the best sample)
    method: str            # "intersection" or "min_rtt"

    def to_server(self, t_client: float) -> float:
        return t_client + self.offset

    def as_dict(self) -> dict:
        return {"offset": self.offset, "uncertainty": self.uncertainty, "rtt_min": self.rtt_min,
                "rtt_median": self.rtt_median, "n": self.n, "t_client": self.t_client,
                "method": self.method}


def estimate_offset(samples: Iterable[SyncSample], keep: int = 8) -> SyncEstimate:
    """Combine a burst of samples.

    The ``keep`` samples with the smallest round trip are used (long round trips are queueing
    or scheduling noise). Their bound intervals are intersected; if the intersection is empty
    (clock drift or a server stamp outside the request, which should not happen), the single
    min-RTT sample is used instead.
    """
    samples = sorted(samples, key=lambda s: s.rtt)
    if not samples:
        raise ValueError("no samples")
    if any(s.rtt < 0 for s in samples):
        raise ValueError("negative round trip (client clock not monotonic?)")
    rtts = [s.rtt for s in samples]
    best = samples[:max(1, keep)]
    lo = max(s.bounds[0] for s in best)
    hi = min(s.bounds[1] for s in best)
    t_mid = (best[0].t0 + best[0].t1) / 2
    med = rtts[len(rtts) // 2] if len(rtts) % 2 else (rtts[len(rtts) // 2 - 1] + rtts[len(rtts) // 2]) / 2
    if lo <= hi:
        return SyncEstimate((lo + hi) / 2, (hi - lo) / 2, rtts[0], med, len(samples), t_mid, "intersection")
    b = best[0]
    return SyncEstimate(b.offset, b.rtt / 2, rtts[0], med, len(samples), t_mid, "min_rtt")


@dataclass
class ClockSync:
    """Offset history plus a linear drift fit; maps client monotonic time to server time.

    ``add(estimate)`` after every burst; ``to_server(t)`` uses the latest offset, corrected by the
    fitted drift (s/s) for the time since that estimate. Drift is fitted only when the bursts
    span at least ``min_span`` seconds (crystal drift is ~1e-5, i.e. 10 us/s, so it only
    matters for long sessions with rare re-syncs).
    """
    history: list = field(default_factory=list)
    max_history: int = 64
    min_span: float = 20.0
    drift: float = 0.0

    @property
    def latest(self) -> Optional[SyncEstimate]:
        return self.history[-1] if self.history else None

    def add(self, est: SyncEstimate) -> None:
        self.history.append(est)
        del self.history[:-self.max_history]
        self.drift = fit_drift(self.history, self.min_span)

    def to_server(self, t_client: float) -> float:
        est = self.latest
        if est is None:
            raise RuntimeError("clock not synchronised yet (call sync first)")
        return t_client + est.offset + self.drift * (t_client - est.t_client)

    def to_client(self, t_server: float) -> float:
        est = self.latest
        if est is None:
            raise RuntimeError("clock not synchronised yet (call sync first)")
        # invert t_s = t_c + off + d (t_c - tc0)
        return (t_server - est.offset + self.drift * est.t_client) / (1.0 + self.drift)

    @property
    def uncertainty(self) -> Optional[float]:
        return self.latest.uncertainty if self.history else None


def fit_drift(history: Sequence[SyncEstimate], min_span: float = 20.0) -> float:
    """Weighted least-squares slope of offset vs client time (weights 1/uncertainty^2)."""
    if len(history) < 2 or history[-1].t_client - history[0].t_client < min_span:
        return 0.0
    w = [1.0 / max(e.uncertainty, 1e-6) ** 2 for e in history]
    x = [e.t_client for e in history]
    y = [e.offset for e in history]
    sw = sum(w)
    mx = sum(wi * xi for wi, xi in zip(w, x)) / sw
    my = sum(wi * yi for wi, yi in zip(w, y)) / sw
    sxx = sum(wi * (xi - mx) ** 2 for wi, xi in zip(w, x))
    if sxx <= 0:
        return 0.0
    return sum(wi * (xi - mx) * (yi - my) for wi, xi, yi in zip(w, x, y)) / sxx


def collect_samples(request_server_time: Callable[[], float], n: int = 16,
                    clock: Callable[[], float] = time.monotonic) -> list[SyncSample]:
    """Run ``n`` exchanges. ``request_server_time()`` performs one request and returns ts."""
    out = []
    for _ in range(n):
        t0 = clock()
        ts = request_server_time()
        t1 = clock()
        out.append(SyncSample(t0, ts, t1))
    return out
