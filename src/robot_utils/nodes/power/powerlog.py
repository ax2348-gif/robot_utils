"""The power node's disk log and watch: a record that survives a brownout, and journal lines when it matters.

``DiskLog``: one CSV row per UPS poll (1 Hz) in ``<state dir>/log/ups-YYYY-MM-DD.csv`` (local date, a header
per file): time, gauge, pack and every cell, current, input, charger. Each row is flushed and the file is
fsynced every ``FSYNC_S``, so a power cut loses at most that much; a failed read is logged as a row with
only the time and the error. Files older than ``KEEP_DAYS`` are deleted. ``recent()`` reads it back (the
node's history after a restart). Disk errors never reach the owner loop: logged once, then retried.

``Watch``: conditions in the readings worth a journal line (and the page's events), each with a hold and
hysteresis so it reports changes, not every poll: input lost / restored, the pack draining while on input
(the load is more than the input supplies, though the charger says "charging"), a low cell, cells apart.
"""
from __future__ import annotations

import csv
import logging
import os
import time
from pathlib import Path
from typing import Callable, Optional

log = logging.getLogger("power")

FIELDS = ["wall", "time", "percent", "pack_v", "cell1_v", "cell2_v", "cell3_v", "cell4_v", "current_a", "power_w",
          "input_v", "input_a", "input_present", "charging", "fast_charge", "charger_state", "remaining_mah", "error"]
FSYNC_S = 5.0
KEEP_DAYS = 30
PREFIX = "ups-"

DRAIN_MA = -50          # on input and below this (net out of the pack) -> draining
DRAIN_HOLD_S = 10.0
CELL_LOW_V, CELL_OK_V = 3.30, 3.40
SPREAD_V, SPREAD_OK_V = 0.20, 0.15
CELL_HOLD_S = 5.0


def row_of(wall: float, d: Optional[dict], error: Optional[str]) -> list:
    stamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(wall)) + f".{int(wall % 1 * 1000):03d}"
    if d is None:
        return [f"{wall:.3f}", stamp] + [""] * (len(FIELDS) - 3) + [error or "no reading"]
    cells = [f"{c / 1000:.3f}" for c in d["cells_mv"]] + [""] * (4 - len(d["cells_mv"]))
    pack_v = d["pack_mv"] / 1000
    return [f"{wall:.3f}", stamp, d["percent"], f"{pack_v:.3f}", *cells[:4], f"{d['current_ma'] / 1000:.3f}",
            f"{pack_v * d['current_ma'] / 1000:.2f}", f"{d['vbus_mv'] / 1000:.3f}", f"{d['vbus_ma'] / 1000:.3f}",
            int(d["vbus_present"]), int(d["charging"]), int(d["fast_charge"]), d["charger_state"],
            d["remaining_mah"], error or ""]


class DiskLog:
    def __init__(self, directory: Path, keep_days: int = KEEP_DAYS, fsync_s: float = FSYNC_S,
                 clock: Callable[[], float] = time.monotonic):
        self.dir, self.keep_days, self.fsync_s, self.clock = Path(directory), keep_days, fsync_s, clock
        self._f = None
        self._w = None
        self._day: Optional[str] = None
        self._synced = -1e18
        self.error: Optional[str] = None
        self.rows = 0

    def path(self, day: str) -> Path:
        return self.dir / f"{PREFIX}{day}.csv"

    def write(self, wall: float, d: Optional[dict], error: Optional[str] = None) -> None:
        try:
            day = time.strftime("%Y-%m-%d", time.localtime(wall))
            if day != self._day:
                self._open(day)
            self._w.writerow(row_of(wall, d, error))
            self._f.flush()
            now = self.clock()
            if now - self._synced >= self.fsync_s:
                os.fsync(self._f.fileno())
                self._synced = now
            self.rows += 1
            if self.error is not None:
                log.info("power log writing again: %s", self.path(day))
                self.error = None
        except (OSError, ValueError) as e:
            msg = f"power log write failed: {e}"
            if msg != self.error:
                log.error(msg)
            self.error = msg
            self.close()

    def _open(self, day: str) -> None:
        self.close()
        self.dir.mkdir(parents=True, exist_ok=True)
        p = self.path(day)
        new = not p.exists() or p.stat().st_size == 0
        self._f = open(p, "a", newline="", buffering=1)
        self._w = csv.writer(self._f)
        if new:
            self._w.writerow(FIELDS)
        self._day = day
        self._synced = -1e18
        self.prune(day)

    def prune(self, today: str) -> None:
        cutoff = time.strftime("%Y-%m-%d", time.localtime(time.mktime(time.strptime(today, "%Y-%m-%d"))
                                                            - self.keep_days * 86400))
        for p in self.dir.glob(f"{PREFIX}*.csv"):
            if p.stem[len(PREFIX):] < cutoff:
                try:
                    p.unlink()
                except OSError as e:
                    log.warning("power log: cannot delete %s: %s", p, e)

    def close(self) -> None:
        f, self._f, self._w, self._day = self._f, None, None, None
        if f is not None:
            try:
                f.flush()
                os.fsync(f.fileno())
                f.close()
            except (OSError, ValueError):
                pass

    def recent(self, wall_now: float, seconds: float, step_s: float = 0.0) -> list[dict]:
        """Rows with a reading from the last ``seconds``, oldest first, at most one per ``step_s``."""
        start = wall_now - seconds
        days = sorted(p for p in self.dir.glob(f"{PREFIX}*.csv")
                      if p.stem[len(PREFIX):] >= time.strftime("%Y-%m-%d", time.localtime(start)))
        out, last = [], -1e18
        for p in days:
            try:
                with open(p, newline="") as f:
                    for r in csv.DictReader(f):
                        try:
                            w = float(r["wall"])
                        except (TypeError, ValueError):
                            continue
                        if w < start or w > wall_now or not r.get("pack_v") or w - last < step_s:
                            continue
                        out.append(r)
                        last = w
            except OSError as e:
                log.warning("power log: cannot read %s: %s", p, e)
        return out


class Watch:
    """Report conditions as they start and end; ``note(msg, level)`` puts them in the journal and the events."""

    def __init__(self, note: Callable[[str, str], None]):
        self.note = note
        self.active: dict[str, bool] = {}
        self.since: dict[str, float] = {}

    def update(self, now: float, d: Optional[dict]) -> None:
        if d is None:
            return
        cells = [c / 1000 for c in d["cells_mv"]]
        lo, spread = min(cells), max(cells) - min(cells)
        snap = (f"pack {d['pack_mv'] / 1000:.2f} V, {d['percent']} %, cells "
                + "/".join(f"{c:.2f}" for c in cells)
                + f" V, {d['current_ma'] / 1000:+.2f} A, input {d['vbus_mv'] / 1000:.2f} V {d['vbus_ma'] / 1000:.2f} A")
        on_input = d["vbus_present"]
        self._cond("input", now, not on_input, on_input, 0.0,
                   f"UPS input power LOST: the Pi runs on the pack ({snap})", f"UPS input power back ({snap})")
        self._cond("drain", now, on_input and d["current_ma"] < DRAIN_MA, d["current_ma"] >= 0 or not on_input,
                   DRAIN_HOLD_S, f"pack DRAINING while on input: the load is more than the input supplies ({snap})",
                   f"pack no longer draining on input ({snap})")
        self._cond("cell_low", now, lo < CELL_LOW_V, lo > CELL_OK_V, CELL_HOLD_S,
                   f"cell {cells.index(lo) + 1} LOW at {lo:.2f} V (< {CELL_LOW_V:.2f}): brownout risk under load ({snap})",
                   f"all cells above {CELL_OK_V:.2f} V again ({snap})")
        self._cond("spread", now, spread > SPREAD_V, spread < SPREAD_OK_V, CELL_HOLD_S,
                   f"cells {spread:.2f} V apart (> {SPREAD_V:.2f}): unbalanced or a weak cell ({snap})",
                   f"cells within {SPREAD_OK_V:.2f} V again ({snap})")

    def _cond(self, key: str, now: float, on: bool, off: bool, hold: float, msg_on: str, msg_off: str) -> None:
        act = self.active.get(key, False)
        if not (off if act else on):
            self.since.pop(key, None)
            return
        t0 = self.since.setdefault(key, now)
        if now - t0 >= hold:
            self.active[key] = not act
            self.since.pop(key, None)
            self.note(msg_off if act else msg_on, "info" if act else "warn")
