"""Time until the Pi loses power, and which event that will be (``/state`` ``sources[ups].power_loss``).

The pack's percent and the gauge's time to empty are no use here: on 2026-11-20 the UPS cut power with the gauge at
45 % and the pack at 13.32 V (above the 13.2 V low-voltage threshold) because one weak cell (cell 3) reached the
UPS's own cutoff, about 3.05 V under load, while the 5 V input was present and the charger bit said "charging". So
the estimate follows the **weakest cell's voltage**:

- **Samples.** Every UPS poll (1 Hz) goes into a ring of the last ``KEEP_S``; at startup it is refilled from the
  disk log. Readings are grouped per minute (median of each column), which drops load spikes.
- **Segment.** Only the trailing run of minutes that are draining (median current below ``DRAIN_A``) with the same
  input state as now is used, at most ``WINDOW_S`` (30 min). Less than ``MIN_SPAN_S`` (10 min) of it: "estimating".
  The latest minute not draining: "not draining".
- **Trend.** A least-squares line through the per-minute medians of the weakest cell gives its slope at the middle
  of the window.
- **Knee.** A cell's voltage falls faster as it nears empty. The model is that the slope grows exponentially as the
  voltage falls, ``dv/dt = s(v0) * exp((v0 - v) / LAMBDA_V)``. The measured slope is moved from the window's
  middle to now with that rule, and the time to fall from ``v`` to a target ``vt`` is then
  ``LAMBDA_V / s * (1 - exp(-(v - vt) / LAMBDA_V))`` (a straight line would be ``(v - vt) / s``: optimistic near
  the end, hours too long on the plateau). ``LAMBDA_V`` 0.15 V was picked on the 2026-11-20 log (backtest in
  docs/power.md and tests/test_power_loss.py): from 70 min before the cutoff to the end the estimate was 0.5-0.9x
  the real time (on the short side). ``range_min`` uses ``LAMBDA_LO`` / ``LAMBDA_HI``.
- **Limits.** The UPS cutoff: the weakest cell reaching ``CELL_CUTOFF_V``. The low-voltage auto-shutdown (only when
  enabled): the weakest cell reaching ``cell_volts`` or the pack reaching ``volts`` (the pack's trend with
  ``N_CELLS * LAMBDA_V``, which errs towards "later", i.e. towards calling it a UPS cutoff), plus its hold, its
  countdown and ``POWEROFF_S``. The auto-shutdown is "clean" when that all ends before the cutoff.
- **Confidence.** "low" while the weakest cell is above ``PLATEAU_V``: on the plateau the knee isn't visible yet and
  the estimate can be several times too long (2-4x at 90-120 min before the 2026-11-20 cutoff). "medium" below.

One observed event set these constants; treat the minutes as a guide, not a promise.
"""
from __future__ import annotations

import math
import statistics
import threading
from collections import deque
from typing import Optional

CELL_CUTOFF_V = 3.05      # the UPS cut power when cell 3 read 3.05 V under ~0.22 A load (2026-11-20, one event)
N_CELLS = 4
from .sources import DRAIN_A   # noqa: E402  (-0.02 A: one threshold for the whole node)
BUCKET_S = 60.0
WINDOW_S = 30 * 60.0      # trailing draining segment used for the trend
MIN_SPAN_S = 10 * 60.0    # less than this of it: "estimating"
KEEP_S = WINDOW_S + 5 * 60.0
GAP_S = 3 * BUCKET_S      # a gap longer than this ends the segment
LAMBDA_V = 0.15           # knee model (see above); backtested on 2026-11-20
LAMBDA_LO, LAMBDA_HI = 0.10, 0.25
PLATEAU_V = 3.65          # weakest cell above this: the knee isn't visible yet -> confidence "low"
POWEROFF_S = 30.0         # from the end of the countdown (e-stop, systemctl poweroff) until the Pi is down
MIN_SLOPE_V_S = 1e-7      # 0.006 mV/min: below this the cell isn't falling
METHOD = (f"weakest-cell voltage trend over the last {WINDOW_S / 60:.0f} min of draining (per-minute medians), "
          f"knee-aware (slope grows as exp(-dv / {LAMBDA_V} V))")


def sample_of(wall: float, d: dict) -> tuple:
    """(wall, cells V, pack V, current A, input present, charging) from a decoded HAT reading."""
    return (wall, tuple(c / 1000 for c in d["cells_mv"]), d["pack_mv"] / 1000, d["current_ma"] / 1000,
            bool(d["vbus_present"]), bool(d["charging"]))


def sample_of_log(r: dict) -> Optional[tuple]:
    """The same from a disk-log CSV row (``DiskLog.recent``); None if the row lacks a value."""
    try:
        cells = tuple(float(r[f"cell{i}_v"]) for i in range(1, N_CELLS + 1))
        return (float(r["wall"]), cells, float(r["pack_v"]), float(r["current_a"]),
                bool(int(float(r["input_present"]))), bool(int(float(r["charging"]))))
    except (KeyError, TypeError, ValueError):
        return None


def time_to(v: float, slope: float, target: float, lam: float) -> float:
    """Seconds for a voltage at ``v`` falling at ``slope`` V/s now to reach ``target`` with the knee model."""
    if v <= target:
        return 0.0
    return lam / slope * (1 - math.exp(-(v - target) / lam))


def _fit(xs: list, ys: list) -> tuple[float, float]:
    """Least squares y = a + b x: (b, mean x)."""
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx else 0.0
    return b, mx


def _mins(s: float) -> float:
    return round(s / 60, 1)


def _dur(s: float) -> str:
    m = round(s / 60)
    if s < 60:
        return "< 1 min"
    return f"{m} min" if m < 60 else f"{m // 60} h {m % 60:02d} min"


class PowerLoss:
    """The estimator: ``add()`` from the owner thread every poll, ``estimate()`` every few seconds (cached)."""

    def __init__(self):
        self.samples: deque = deque()
        self._lock = threading.Lock()

    def add(self, s: tuple) -> None:
        with self._lock:
            if self.samples and s[0] <= self.samples[-1][0]:
                return                                   # out of order / duplicate (a refill overlapping live polls)
            self.samples.append(s)
            while self.samples and self.samples[0][0] < s[0] - KEEP_S:
                self.samples.popleft()

    def extend(self, samples) -> None:
        for s in sorted(x for x in samples if x is not None):
            self.add(s)

    def buckets(self) -> list[dict]:
        """Per-minute medians, oldest first."""
        with self._lock:
            data = list(self.samples)
        out, cur, key = [], [], None
        for s in data:
            k = int(s[0] // BUCKET_S)
            if k != key and cur:
                out.append(self._bucket(cur))
                cur = []
            key = k
            cur.append(s)
        if cur:
            out.append(self._bucket(cur))
        return out

    @staticmethod
    def _bucket(ss: list) -> dict:
        med = statistics.median
        return {"wall": med(s[0] for s in ss),
                "cells": [med(s[1][i] for s in ss) for i in range(len(ss[-1][1]))],
                "pack": med(s[2] for s in ss), "a": med(s[3] for s in ss),
                "input": ss[-1][4], "charging": ss[-1][5], "n": len(ss)}

    def estimate(self, wall_now: float, low: Optional[dict] = None, delay_s: float = 10.0) -> dict:
        """``low``: the low-voltage settings (``LowVoltage.settings``: enabled, volts, cell_volts, hold_s).
        ``delay_s``: the countdown of a low-voltage shutdown (shutdown.DELAY_MIN_S)."""
        out = {"status": "no_data", "in_min": None, "range_min": None, "at": None, "event": None, "clean": None,
               "limit": None, "cutoff_in_min": None, "blocked_reason": None, "method": METHOD, "confidence": None,
               "note": None, "cell": None, "cell_v": None, "slope_mv_min": None, "current_a": None,
               "on_input": None, "window_min": None, "text": "power loss: no readings yet"}
        bs = self.buckets()
        if not bs or wall_now - bs[-1]["wall"] > 2 * BUCKET_S:
            return out
        last = bs[-1]
        out.update(current_a=round(last["a"], 3), on_input=last["input"])
        watts = last["a"] * last["pack"]
        flow = f"{'+' if watts >= 0 else '-'}{abs(watts):.2f} W"
        if last["a"] >= DRAIN_A:
            out.update(status="not_draining",
                       text=(f"charging {flow}" if last["a"] > -DRAIN_A else f"steady: net {last['a']:+.3f} A ({flow})")
                       + ": no power loss expected")
            return out
        # the trailing draining run with the same input state: one odd minute (a spike) is skipped, two end it,
        # as does a gap in the readings
        seg, odd = [last], 0
        for b in reversed(bs[:-1]):
            if b["input"] != last["input"] or seg[0]["wall"] - b["wall"] > GAP_S or last["wall"] - b["wall"] > WINDOW_S:
                break
            if b["a"] >= DRAIN_A:
                odd += 1
                if odd > 1:
                    break
                continue
            odd = 0
            seg.insert(0, b)
        span = seg[-1]["wall"] - seg[0]["wall"]
        out["window_min"] = _mins(span)
        if span < MIN_SPAN_S or len(seg) < 5:
            out.update(status="estimating", text=f"draining {abs(watts):.2f} W; estimating power loss ({_dur(span)} of "
                                                 f"{_dur(MIN_SPAN_S)} of draining history so far)")
            return out
        cell = min(range(len(last["cells"])), key=lambda i: last["cells"][i])
        xs = [b["wall"] - wall_now for b in seg]
        b_cell, xm = _fit(xs, [b["cells"][cell] for b in seg])
        b_pack, _ = _fit(xs, [b["pack"] for b in seg])
        mean = lambda k: sum(k(b) for b in seg) / len(seg)    # noqa: E731
        # the fitted value now (end of the line), robust to the last minute's noise
        v_mid, p_mid = mean(lambda b: b["cells"][cell]), mean(lambda b: b["pack"])
        v_now, p_now = v_mid + b_cell * (0 - xm), p_mid + b_pack * (0 - xm)
        v_now, p_now = min(v_now, last["cells"][cell] + 0.02), min(p_now, last["pack"] + 0.08)
        out.update(cell=cell + 1, cell_v=round(last["cells"][cell], 3), slope_mv_min=round(-b_cell * 60000, 2))
        if -b_cell < MIN_SLOPE_V_S:
            out.update(status="estimating", text=f"draining {abs(watts):.2f} W, too slowly to project yet (cell {cell + 1} "
                                                 f"falling < {MIN_SLOPE_V_S * 60000:.1f} mV/min): power loss is hours away")
            return out

        def times(lam: float) -> dict:
            s_cell = -b_cell * math.exp((v_mid - v_now) / lam)       # the slope moved from the window's middle to now
            t = {"cutoff": time_to(v_now, s_cell, CELL_CUTOFF_V, lam)}
            if low and low.get("enabled"):
                if low.get("cell_volts") is not None:
                    t["cell"] = time_to(v_now, s_cell, low["cell_volts"], lam)
                if low.get("volts") is not None and -b_pack > MIN_SLOPE_V_S:
                    lp = N_CELLS * lam
                    t["pack"] = time_to(p_now, -b_pack * math.exp((p_mid - p_now) / lp), low["volts"], lp)
            return t

        def first_auto(t: dict) -> tuple:
            auto = [(t[k], k) for k in ("cell", "pack") if k in t]
            if not auto:
                return None, None
            ta, which = min(auto)
            return ta + float(low.get("hold_s", 10.0)) + delay_s, which

        # Clean or not is decided on the slow-knee times (LAMBDA_HI): the steep central knee squeezes the last
        # 0.1 V into too little time (on 2026-11-20 cell 3 took ~2.5 min from 3.15 V to the cutoff; LAMBDA_V gave
        # ~45 s an hour before), which would call a clean shutdown a race it wasn't.
        t, t_hi = times(LAMBDA_V), times(LAMBDA_HI)
        end_hi, which = first_auto(t_hi)
        clean = end_hi is not None and end_hi + POWEROFF_S <= t_hi["cutoff"]

        def when(tt: dict) -> float:
            return min(first_auto(tt)[0], tt["cutoff"]) if clean else tt["cutoff"]

        event = "auto_shutdown" if clean else "ups_cutoff"
        in_s, lo, hi = when(t), when(times(LAMBDA_LO)), when(t_hi)
        cut_label = f"cell {cell + 1} → {CELL_CUTOFF_V:.2f} V"
        if event == "auto_shutdown":
            which = first_auto(t)[1]                     # the limit the central estimate reaches first
            limit = ({"kind": "cell", "cell": cell + 1, "target_v": low["cell_volts"], "v": round(v_now, 3),
                      "label": f"cell {cell + 1} → {low['cell_volts']:.2f} V"} if which == "cell" else
                     {"kind": "pack", "cell": None, "target_v": low["volts"], "v": round(p_now, 3),
                      "label": f"pack → {low['volts']:.2f} V"})
            blocked = None
            what = f"clean auto-shutdown ({limit['label']})"
        else:
            limit = {"kind": "cutoff", "cell": cell + 1, "target_v": CELL_CUTOFF_V, "v": round(v_now, 3),
                     "label": cut_label}
            if not low or not low.get("enabled"):
                blocked = "auto-shutdown is off"
            elif which is None:
                blocked = "auto-shutdown: no trend for its limits"
            else:
                lab = (f"cell {cell + 1} → {low['cell_volts']:.2f} V" if which == "cell" else
                       f"pack → {low['volts']:.2f} V")
                blocked = f"auto-shutdown ({lab}) would come too late"
            what = f"UPS cutoff ({cut_label}); {blocked}"
        plateau = last["cells"][cell] > PLATEAU_V
        conf = "low" if plateau else "medium"
        note = ("weakest cell still on the plateau: the knee isn't visible yet, so this is rough (on 2026-11-20 it was "
                "up to 3.5x too long 2 h before the cutoff)" if plateau else "near the knee: the estimate errs short")
        out.update(status="draining", in_min=_mins(in_s), range_min=[_mins(min(lo, in_s)), _mins(max(hi, in_s))],
                   at=round(wall_now + in_s, 1), event=event, clean=event == "auto_shutdown", limit=limit,
                   cutoff_in_min=_mins(t["cutoff"]), blocked_reason=blocked, confidence=conf, note=note,
                   text=f"power loss in ≈ {_dur(in_s)}: {what}" + (" (rough: knee not visible yet)" if plateau else ""))
        return out
