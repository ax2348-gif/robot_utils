"""On-demand stack sampler for a running node (standard library only): which code each thread is in.

``GET /debug/profile?seconds=20&hz=100`` (every node, added by ``serve``) samples every Python thread's stack
``hz`` times a second for ``seconds`` from a worker thread (the event loop keeps serving), and reports:

- per Python thread (by its ``threading`` name, with the OS thread id): CPU % and run-queue wait % over the
  window (``/proc/self/task/<tid>/schedstat``), and the fraction of samples it was runnable (OS state R:
  on a core or waiting for one) vs sleeping (S/D: blocked in a call, or waiting for the GIL);
- the stacks it was in, separately for runnable and sleeping samples (innermost frame last, ``file:func:line``);
  the runnable stacks are where its CPU goes;
- the OS threads that are not Python threads (e.g. a camera SDK's), with their CPU and run-queue wait;
- ``sampler_late_ms``: how late the sampler woke for each sample. Waking needs a core and then the GIL, so
  this is the delay any Python thread in the node sees before it can run: a GIL held for 50 ms shows up here.
  For samples later than ``LATE_MS`` the stacks of the threads that were runnable then are kept as suspects.

Costs while it runs: the sampler holds the GIL for each sample (about 0.1-0.5 ms with ~10 threads) and reads
one small /proc file per thread; nothing otherwise. One window at a time per node.
"""
from __future__ import annotations

import os
import sys
import threading
import time
from collections import Counter, defaultdict
from typing import Optional

MAX_SECONDS = 120.0
MAX_HZ = 500.0
MAX_DEPTH = 48
LATE_MS = 20.0
TOP = 12

_busy = threading.Lock()


def _os_state(tid: int) -> Optional[str]:
    try:
        with open(f"/proc/self/task/{tid}/stat", "rb") as f:
            return f.read().rsplit(b")", 1)[1].split()[0].decode()
    except OSError:
        return None


def _schedstat() -> dict[int, tuple[int, int, str]]:
    out = {}
    try:
        tids = os.listdir("/proc/self/task")
    except OSError:
        return out
    for t in tids:
        try:
            run, wait, _ = map(int, open(f"/proc/self/task/{t}/schedstat").read().split())
            comm = open(f"/proc/self/task/{t}/comm").read().strip()
            out[int(t)] = (run, wait, comm)
        except (OSError, ValueError):
            pass
    return out


def _stack(frame) -> tuple:
    out = []
    while frame is not None and len(out) < MAX_DEPTH:
        c = frame.f_code
        out.append((c.co_filename, c.co_name, frame.f_lineno))
        frame = frame.f_back
    out.reverse()
    return tuple(out)


def _fmt(stack: tuple) -> str:
    parts = []
    for fn, func, line in stack:
        p = fn.replace("\\", "/")
        for marker in ("/site-packages/", "/robot_utils/", "/lib/python3"):
            if marker in p:
                p = p.split(marker, 1)[1]
                break
        parts.append(f"{p}:{func}:{line}")
    return ";".join(parts)


def _pct(xs: list, q: float) -> Optional[float]:
    if not xs:
        return None
    s = sorted(xs)
    return round(s[min(len(s) - 1, int(q / 100 * len(s)))], 2)


def profile(seconds: float = 20.0, hz: float = 100.0, folded: bool = False) -> dict:
    """Sample this process for ``seconds`` (blocking; run it in a worker thread)."""
    seconds = max(0.5, min(float(seconds), MAX_SECONDS))
    hz = max(1.0, min(float(hz), MAX_HZ))
    if not _busy.acquire(blocking=False):
        raise RuntimeError("a profile is already running in this node")
    try:
        return _profile(seconds, hz, folded)
    finally:
        _busy.release()


def _profile(seconds: float, hz: float, folded: bool) -> dict:
    me = threading.get_ident()
    period = 1.0 / hz
    run_stacks: dict[int, Counter] = defaultdict(Counter)
    wait_stacks: dict[int, Counter] = defaultdict(Counter)
    states: dict[int, Counter] = defaultdict(Counter)
    names: dict[int, tuple[str, Optional[int]]] = {}
    late: list[float] = []
    suspects: Counter = Counter()
    n = 0
    s0 = _schedstat()
    t0 = time.monotonic()
    due = t0
    end = t0 + seconds
    while True:
        due += period
        now = time.monotonic()
        if due > end:
            break
        if due > now:
            time.sleep(due - now)
        woke = time.monotonic()
        lag_ms = (woke - due) * 1e3
        late.append(lag_ms)
        if lag_ms > 1e3 * period:              # fell behind (a stall): resynchronise, don't burst
            due = woke
        threads = {t.ident: t for t in threading.enumerate()}
        frames = sys._current_frames()
        n += 1
        for ident, frame in frames.items():
            if ident == me:
                continue
            t = threads.get(ident)
            if ident not in names:
                names[ident] = (t.name if t else f"thread-{ident}", getattr(t, "native_id", None))
            tid = names[ident][1]
            st = _os_state(tid) if tid else None
            stack = _stack(frame)
            key = "R" if st == "R" else "S"
            states[ident][key] += 1
            (run_stacks if key == "R" else wait_stacks)[ident][stack] += 1
            if key == "R" and lag_ms > LATE_MS:
                suspects[(names[ident][0], stack[-3:])] += 1
    dt = time.monotonic() - t0
    s1 = _schedstat()
    sampler_tid = threading.get_native_id()

    def cpu(tid):
        a, b = s0.get(tid), s1.get(tid)
        if not a or not b:
            return None, None
        return round(100 * (b[0] - a[0]) / 1e9 / dt, 1), round(100 * (b[1] - a[1]) / 1e9 / dt, 1)

    py_tids = set()
    threads_out = []
    for ident, (name, tid) in names.items():
        py_tids.add(tid)
        c, w = cpu(tid) if tid else (None, None)
        tot = sum(states[ident].values()) or 1
        rs, ws = run_stacks[ident], wait_stacks[ident]
        threads_out.append({
            "name": name, "native_id": tid, "cpu_pct": c, "runq_wait_pct": w,
            "runnable_pct": round(100 * states[ident]["R"] / tot, 1), "samples": tot,
            "runnable_stacks": [{"pct": round(100 * k / tot, 1), "stack": _fmt(s)} for s, k in rs.most_common(TOP)],
            "sleeping_stacks": [{"pct": round(100 * k / tot, 1), "stack": _fmt(s)} for s, k in ws.most_common(6)],
        })
    threads_out.sort(key=lambda x: -(x["cpu_pct"] or 0))
    others = []
    for tid in s1:
        if tid in py_tids or tid == sampler_tid:
            continue
        c, w = cpu(tid)
        if c is not None and (c >= 0.5 or w >= 0.5):
            others.append({"native_id": tid, "comm": s1[tid][2], "cpu_pct": c, "runq_wait_pct": w})
    others.sort(key=lambda x: -x["cpu_pct"])
    sc, sw = cpu(sampler_tid)
    out = {
        "pid": os.getpid(), "seconds": round(dt, 2), "hz": hz, "samples": n,
        "sampler_late_ms": {"p50": _pct(late, 50), "p90": _pct(late, 90), "p99": _pct(late, 99),
                            "max": round(max(late), 2) if late else None,
                            "n_over_10": sum(x > 10 for x in late), "n_over_50": sum(x > 50 for x in late)},
        "sampler_cost": {"cpu_pct": sc, "runq_wait_pct": sw},
        "threads": threads_out,
        "other_os_threads": others[:20],
        "late_suspects": [{"thread": t, "samples": k, "where": _fmt(s)} for (t, s), k in suspects.most_common(TOP)],
    }
    if folded:
        lines = []
        for ident, (name, _) in names.items():
            for s, k in run_stacks[ident].items():
                lines.append(f"{name};{_fmt(s)} {k}")
        out["folded_runnable"] = "\n".join(sorted(lines))
    return out


def add_profile_route(app) -> None:
    """``GET /debug/profile`` on a FastAPI app (``serve`` adds it to every node)."""
    import asyncio

    from fastapi import HTTPException, Query

    if any(getattr(r, "path", None) == "/debug/profile" for r in app.routes):
        return

    async def debug_profile(seconds: float = Query(20.0, gt=0, le=MAX_SECONDS),
                            hz: float = Query(100.0, gt=0, le=MAX_HZ), folded: bool = False):
        try:
            return await asyncio.to_thread(profile, seconds, hz, folded)
        except RuntimeError as e:
            raise HTTPException(409, str(e))

    app.add_api_route("/debug/profile", debug_profile, methods=["GET"], tags=["debug"],
                      summary="Sample every thread's stack for a while: where this node's CPU and waits go "
                              "(blocking for `seconds`; one at a time)")
