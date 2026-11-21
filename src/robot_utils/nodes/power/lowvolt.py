"""Low-voltage auto-shutdown, ported from the robot's old ``ups-dashboard`` service.

When enabled and the UPS pack is net draining (current below ``DRAIN_MA``, whatever the charger's
"charging" bit says: an undersized input can leave it set while the pack drains) with the pack below
``volts`` or its weakest cell below ``cell_volts``, for ``hold_s`` seconds, the node schedules a shutdown through the same path as a manual one (countdown of
``DELAY_MIN_S``, e-stop, power-off), so it shows on the page and can be cancelled. A cancelled
low-voltage shutdown re-arms after ``REARM_S`` if the pack is still low: to stop it for good,
turn the feature off.

It never acts on missing, stale or invalid readings (an I2C failure is not a low battery).

**Off by default on first run**, because the old dashboard does the same thing until it is
retired (docs/power.md, "Cutover"). Settings live in ``~/.config/robot_utils/power.json``.
"""
from __future__ import annotations

import json
import os
import threading
from pathlib import Path
from typing import Callable, Optional

from ..common.lease import RequestError

# volts: 3.3 V per cell for the 4-cell pack. cell_volts: the weakest cell; a weak cell reaches the UPS's own cutoff
# (observed at 3.05 V under load, 2026-11-20) long before the pack average does, and the UPS then cuts power at once.
DEFAULTS = {"enabled": False, "volts": 13.2, "hold_s": 10.0, "cell_volts": 3.15}
VOLTS_RANGE = (11.0, 16.0)       # 4 cells, 3.0-4.2 V each
CELL_RANGE = (2.9, 3.6)
DRAIN_MA = -20                   # net pack current below this = draining (sources.DISCHARGE_MA)
HOLD_RANGE = (5.0, 120.0)        # motor load sags the pack briefly
REARM_S = 120.0                  # after a cancelled low-voltage shutdown


def default_path() -> Path:
    base = os.environ.get("XDG_CONFIG_HOME", os.path.expanduser("~/.config"))
    return Path(base) / "robot_utils" / "power.json"


class LowVoltage:
    """Settings + the check. ``trigger(pack_mv)`` schedules the shutdown (the node wires it)."""

    def __init__(self, path: Optional[Path], trigger: Callable[[int], None]):
        self.path = Path(path) if path else None
        self.trigger = trigger
        self.settings = dict(DEFAULTS)
        self.first_run = True
        if self.path and self.path.exists():
            try:
                saved = json.loads(self.path.read_text()).get("low_voltage", {})
                self.settings.update({k: saved[k] for k in DEFAULTS if k in saved})
                self.first_run = False
            except (OSError, ValueError, AttributeError):
                pass
        self._lock = threading.Lock()
        self.low_since: Optional[float] = None
        self.triggered_at: Optional[float] = None
        self.rearm_at: Optional[float] = None
        self.last: Optional[dict] = None          # last battery reading seen (battery() of the UPS source)
        self.reason: Optional[str] = None         # why it is low now (or last triggered): for the event / shutdown

    @staticmethod
    def draining(bat: dict) -> bool:
        """Net out of the pack. Without a current reading (older callers), not charging counts as draining."""
        ma = bat.get("current_ma")
        return (not bat["charging"]) if ma is None else ma < DRAIN_MA

    def would_trigger(self, bat: Optional[dict], settings: Optional[dict] = None) -> Optional[str]:
        """'pack' or 'cell' when the reading is low enough to start the hold (enabled or not), else None.
        A net-charging pack never is, so a Pi that boots with its nodes off and charges can't shut down again."""
        s = settings or self.settings
        if bat is None or not self.draining(bat):
            return None
        if bat["pack_mv"] < s["volts"] * 1000:
            return "pack"
        cells = [c for c in (bat.get("cells_mv") or []) if c]
        if cells and min(cells) < s["cell_volts"] * 1000:
            return "cell"
        return None

    def describe(self, bat: dict, which: str) -> str:
        s = self.settings
        if which == "cell":
            cells = bat["cells_mv"]
            i = cells.index(min(c for c in cells if c))
            what = f"cell {i + 1} at {cells[i] / 1000:.2f} V below {s['cell_volts']:.2f} V"
        else:
            what = f"pack {bat['pack_mv'] / 1000:.2f} V below {s['volts']:.2f} V"
        ma = bat.get("current_ma")
        flow = f"draining {abs(ma) / 1000:.2f} A" if ma is not None else "not charging"
        return f"{what}, {flow}" + (" although the charger reports charging" if bat["charging"] else "")

    def is_low(self, bat: Optional[dict]) -> bool:
        return bool(self.settings["enabled"] and self.would_trigger(bat) is not None)

    def check(self, now: float, bat: Optional[dict], shutdown_pending: bool) -> None:
        """One check (owner thread). ``bat`` is None when there is no fresh, valid reading."""
        with self._lock:
            self.last = bat
            if not self.is_low(bat):
                self.low_since = self.triggered_at = self.rearm_at = None
                return
            self.reason = self.describe(bat, self.would_trigger(bat))
            if self.triggered_at is not None:
                if shutdown_pending:
                    return
                if self.rearm_at is None:               # cancelled (or failed): back off, then re-arm
                    self.rearm_at = now + REARM_S
                if now < self.rearm_at:
                    return
                self.triggered_at = self.rearm_at = None
                self.low_since = now
            if self.low_since is None:
                self.low_since = now
            if now - self.low_since < self.settings["hold_s"]:
                return
            self.triggered_at = now
        self.trigger(bat["pack_mv"])

    def update(self, enabled: bool, volts: float, hold_s: Optional[float], confirm: bool,
               bat: Optional[dict], cell_volts: Optional[float] = None) -> dict:
        lo, hi = VOLTS_RANGE
        if not lo <= volts <= hi:
            raise RequestError(422, f"volts must be {lo}..{hi}")
        hold_s = self.settings["hold_s"] if hold_s is None else float(hold_s)
        if not HOLD_RANGE[0] <= hold_s <= HOLD_RANGE[1]:
            raise RequestError(422, f"hold_s must be {HOLD_RANGE[0]:.0f}..{HOLD_RANGE[1]:.0f}")
        cell_volts = self.settings["cell_volts"] if cell_volts is None else float(cell_volts)
        if not CELL_RANGE[0] <= cell_volts <= CELL_RANGE[1]:
            raise RequestError(422, f"cell_volts must be {CELL_RANGE[0]}..{CELL_RANGE[1]}")
        new = {"enabled": bool(enabled), "volts": round(float(volts), 2), "hold_s": round(hold_s, 1),
               "cell_volts": round(cell_volts, 3)}
        which = self.would_trigger(bat, new)
        if enabled and not confirm and which:
            raise RequestError(409, f"{self.describe(bat, which)}: enabling starts a shutdown in about "
                                    f"{hold_s:.0f} s. Send confirm: true to enable anyway")
        with self._lock:
            self.settings = new
            self.low_since = self.triggered_at = self.rearm_at = None
            self.first_run = False
        self._save()
        return self.public(None)

    def _save(self) -> None:
        if not self.path:
            return
        try:
            data = json.loads(self.path.read_text()) if self.path.exists() else {}
        except (OSError, ValueError):
            data = {}
        data["low_voltage"] = self.settings
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, indent=1))
        os.replace(tmp, self.path)

    def status(self, now: Optional[float]) -> str:
        s = self.settings
        if not s["enabled"]:
            return "off"
        if self.last is None:
            return "no data: not acting"
        if self.triggered_at is not None:
            if self.rearm_at is not None and now is not None:
                return f"shutdown cancelled; re-arms in {max(0, self.rearm_at - now):.0f} s if still low"
            return "triggered: shutdown scheduled"
        if self.low_since is not None and now is not None:
            return (f"low ({self.reason}): shutdown in {max(0, s['hold_s'] - (now - self.low_since)):.0f} s "
                    "unless it recovers")
        return "armed"

    def public(self, now: Optional[float]) -> dict:
        with self._lock:
            bat = self.last
            cells = [c for c in ((bat or {}).get("cells_mv") or []) if c]
            return dict(self.settings, status=self.status(now), low=self.is_low(bat),
                        reason=self.reason if self.is_low(bat) or self.triggered_at is not None else None,
                        pack_v=None if bat is None else round(bat["pack_mv"] / 1000, 3),
                        min_cell_v=round(min(cells) / 1000, 3) if cells else None,
                        range_v=list(VOLTS_RANGE), range_hold_s=list(HOLD_RANGE), range_cell_v=list(CELL_RANGE),
                        first_run=self.first_run, source="ups")
