#!/usr/bin/env python3
"""How the base hint's candidate sources behave in a recording: jitter, filter lag, head-vs-body decoupling.

    PYTHONPATH=src python tools/quest_teleop/hint_analysis.py <session or episode dir> [--json]
        [--tau 0.15] [--deadband 0.02 0.03] [--min-confidence 0.5] [--recompute]

Reads the operator stream (``quest_teleop.oplog``) of a recording (docs/collect.md). It uses the ``hint`` records
the node logs while teleop runs (the three candidates side by side, raw and smoothed: ``hint_source.py``). Without
them (teleop not running, an older node, or ``--recompute``) it recomputes the candidates from the ``headset``
frames with the given filter settings, which also lets you try other settings on the same data. Body candidates
need frames recorded with body tracking on (quest_env ``--body``).

Per candidate (head, chest, hips):

- ``ok``: the fraction of frames where it was usable;
- **jitter at rest**: over the frames where the operator stands still (its raw position moved less than 3 cm/s and
  turned less than 0.1 rad/s on average over 0.5 s around the frame), the std of the yaw rate (rad/s) and of the
  planar position about its 0.5 s mean (mm), raw and smoothed;
- **filter lag**: the delay (ms) maximising the correlation of the smoothed yaw rate with the raw one;
- **divergence from the head** (chest, hips): their yaw change minus the head's (both relative to their median
  offset: a body joint's heading has no absolute meaning), std, 95th percentile and max |.| in degrees: how much
  the head turns without the body (and the reverse).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from quest_teleop.hint_source import SOURCES, HintFilterConfig, HintSources   # noqa: E402
else:
    from .hint_source import SOURCES, HintFilterConfig, HintSources

REST_SPEED = 0.03      # m/s
REST_OMEGA = 0.1       # rad/s
WINDOW = 0.5           # s


def _records(path: Path):
    from robot_utils.collect.records import iter_raw   # noqa: PLC0415
    from robot_utils.replay.log import _stream_files   # noqa: PLC0415
    files = _stream_files(path, "oplog")
    if not files:
        raise SystemExit(f"{path}: no operator stream (quest_teleop.oplog) recorded")
    for _, h, _ in iter_raw(files):
        yield h


def load(path: Path, cfg: HintFilterConfig, recompute: bool = False) -> tuple[dict, str]:
    """name -> dict of arrays: t (s, sample time), ok, x, z (LOCAL horizontal), yaw, fx, fz, fyaw; and the origin
    ("logged" | "recomputed")."""
    hints, frames = [], []
    for h in _records(path):
        if h.get("type") == "hint" and isinstance(h.get("candidates"), dict):
            hints.append(h)
        elif h.get("type") == "headset" and isinstance(h.get("frame"), dict):
            frames.append(h["frame"])
    rows: dict = {n: [] for n in SOURCES}
    if hints and not recompute:
        origin = "logged"
        for h in hints:
            ns = h.get("sample_time_ns")
            if not isinstance(ns, int):
                continue
            for n in SOURCES:
                c = h["candidates"].get(n) or {}
                rows[n].append(_row(ns * 1e-9, c.get("ok"), c.get("p"), c.get("yaw"), c.get("f_p"), c.get("f_yaw")))
    else:
        origin = "recomputed"
        src = HintSources(cfg)
        seen = set()
        for f in frames:
            ns = f.get("sample_time_ns")
            key = (f.get("session_id"), f.get("frame_seq"))
            if not isinstance(ns, int) or key in seen:
                continue
            seen.add(key)
            out = src.update(f)
            for n in SOURCES:
                s = out[n]
                rows[n].append(_row(ns * 1e-9, s.ok, s.p, s.yaw, s.f_p, s.f_yaw))
    data = {}
    for n, r in rows.items():
        a = np.array(r, float).reshape(-1, 8)
        a = a[np.argsort(a[:, 0], kind="stable")] if len(a) else a
        data[n] = {k: a[:, i] for i, k in enumerate(("t", "ok", "x", "z", "yaw", "fx", "fz", "fyaw"))}
    return data, origin


def _row(t, ok, p, yaw, fp, fyaw):
    if not ok or p is None:
        return [t, 0.0] + [math.nan] * 6
    return [t, 1.0, p[0], p[2], yaw, fp[0], fp[2], fyaw]


def _rate(t, x):
    d = np.full_like(x, math.nan)
    if len(x) > 2:
        d[1:-1] = (x[2:] - x[:-2]) / np.maximum(t[2:] - t[:-2], 1e-6)
    return d


def _moving_mean(t, x, window):
    out = np.full_like(x, math.nan)
    j0 = 0
    j1 = 0
    for i in range(len(t)):
        while j0 < len(t) and t[j0] < t[i] - window / 2:
            j0 += 1
        while j1 < len(t) and t[j1] <= t[i] + window / 2:
            j1 += 1
        seg = x[j0:j1]
        seg = seg[np.isfinite(seg)]
        if seg.size:
            out[i] = seg.mean()
    return out


def _lag(t, raw, filt, max_lag=0.5) -> float | None:
    ok = np.isfinite(raw) & np.isfinite(filt)
    if ok.sum() < 20:
        return None
    dt = float(np.median(np.diff(t[ok]))) if ok.sum() > 1 else 0.0
    if dt <= 0:
        return None
    a, b = raw[ok] - raw[ok].mean(), filt[ok] - filt[ok].mean()
    if not a.any() or not b.any():
        return None
    best, best_k = -math.inf, 0
    for k in range(0, int(max_lag / dt) + 1):
        c = float(np.dot(a[:len(a) - k], b[k:])) if k else float(np.dot(a, b))
        if c > best:
            best, best_k = c, k
    return round(best_k * dt * 1e3, 1)


def analyse(data: dict) -> dict:
    out = {}
    for n in SOURCES:
        d = data[n]
        t = d["t"]
        if not len(t):
            out[n] = {"frames": 0}
            continue
        okm = d["ok"] > 0.5
        r = {"frames": int(len(t)), "ok": round(float(okm.mean()), 3)}
        if okm.sum() >= 10:
            w, fw = _rate(t, d["yaw"]), _rate(t, d["fyaw"])
            speed = np.hypot(_rate(t, d["x"]), _rate(t, d["z"]))
            rest = (np.abs(_moving_mean(t, w, WINDOW)) < REST_OMEGA) & (_moving_mean(t, speed, WINDOW) < REST_SPEED)
            rest &= okm & np.isfinite(w)
            r["rest_s"] = round(float(rest.sum() * np.median(np.diff(t))), 2) if len(t) > 1 else 0.0
            if rest.sum() >= 10:
                def pos_jitter(x, z):
                    mx, mz = _moving_mean(t, x, WINDOW), _moving_mean(t, z, WINDOW)
                    return round(float(np.nanstd(np.hypot(x - mx, z - mz)[rest]) * 1e3), 2)
                r["jitter_yaw_rate_std"] = {"raw": round(float(np.nanstd(w[rest])), 4),
                                            "smoothed": round(float(np.nanstd(fw[rest])), 4)}
                r["jitter_pos_mm"] = {"raw": pos_jitter(d["x"], d["z"]), "smoothed": pos_jitter(d["fx"], d["fz"])}
            r["filter_lag_ms"] = _lag(t, w, fw)
        out[n] = r
    head = data["head"]
    for n in ("chest", "hips"):
        d = data[n]
        if not len(d["t"]) or not len(head["t"]):
            continue
        hy = np.interp(d["t"], head["t"], np.where(head["ok"] > 0.5, head["yaw"], np.nan))
        diff = d["yaw"] - hy
        ok = np.isfinite(diff)
        if ok.sum() >= 10:
            dv = np.degrees(diff[ok] - np.median(diff[ok]))
            out[n]["divergence_from_head_deg"] = {"std": round(float(dv.std()), 2),
                                                  "p95": round(float(np.percentile(np.abs(dv), 95)), 2),
                                                  "max": round(float(np.abs(dv).max()), 2)}
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", type=Path, help="a session or episode directory")
    ap.add_argument("--tau", type=float, default=0.15)
    ap.add_argument("--deadband", type=float, nargs=2, default=(0.02, 0.03), metavar=("M", "RAD"))
    ap.add_argument("--min-confidence", type=float, default=0.5)
    ap.add_argument("--recompute", action="store_true", help="recompute from the headset frames even if hint "
                                                             "records exist (try other filter settings)")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    cfg = HintFilterConfig(a.tau, a.deadband[0], a.deadband[1], a.min_confidence)
    data, origin = load(a.path, cfg, a.recompute)
    res = analyse(data)
    if a.json:
        print(json.dumps({"origin": origin, "candidates": res}, indent=1))
        return 0
    print(f"{a.path}: candidates {origin}")
    for n, r in res.items():
        print(f"  {n:5s} frames {r.get('frames', 0)}, ok {r.get('ok', 0):.0%}, at rest {r.get('rest_s', 0)} s")
        if "jitter_yaw_rate_std" in r:
            j, p = r["jitter_yaw_rate_std"], r["jitter_pos_mm"]
            print(f"        jitter at rest: yaw rate std {j['raw']:.4f} -> {j['smoothed']:.4f} rad/s, "
                  f"position {p['raw']:.1f} -> {p['smoothed']:.1f} mm")
        if r.get("filter_lag_ms") is not None:
            print(f"        filter lag {r['filter_lag_ms']:.0f} ms")
        if "divergence_from_head_deg" in r:
            dv = r["divergence_from_head_deg"]
            print(f"        yaw vs the head: std {dv['std']:.1f}, p95 {dv['p95']:.1f}, max {dv['max']:.1f} deg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
