"""Low-voltage auto-shutdown, ported from the robot's old ``ups-dashboard`` service.

When enabled and the UPS pack stays below the threshold for ``hold_s`` seconds while not
charging, the node schedules a shutdown through the same path as a manual one (countdown of
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

DEFAULTS = {"enabled": False, "volts": 13.2, "hold_s": 10.0}   # 3.3 V per cell for the 4-cell pack
VOLTS_RANGE = (11.0, 16.0)       # 4 cells, 3.0-4.2 V each
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
        self.last: Optional[dict] = None          # last battery reading seen (pack_mv, charging)

    def is_low(self, bat: Optional[dict]) -> bool:
        s = self.settings
        return bool(s["enabled"] and bat is not None and not bat["charging"] and bat["pack_mv"] < s["volts"] * 1000)

    def check(self, now: float, bat: Optional[dict], shutdown_pending: bool) -> None:
        """One check (owner thread). ``bat`` is None when there is no fresh, valid reading."""
        with self._lock:
            self.last = bat
            if not self.is_low(bat):
                self.low_since = self.triggered_at = self.rearm_at = None
                return
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
               bat: Optional[dict]) -> dict:
        lo, hi = VOLTS_RANGE
        if not lo <= volts <= hi:
            raise RequestError(422, f"volts must be {lo}..{hi}")
        hold_s = self.settings["hold_s"] if hold_s is None else float(hold_s)
        if not HOLD_RANGE[0] <= hold_s <= HOLD_RANGE[1]:
            raise RequestError(422, f"hold_s must be {HOLD_RANGE[0]:.0f}..{HOLD_RANGE[1]:.0f}")
        if enabled and not confirm and bat is not None and not bat["charging"] and bat["pack_mv"] < volts * 1000:
            raise RequestError(409, f"the pack is at {bat['pack_mv'] / 1000:.2f} V and not charging, below "
                                    f"{volts:.2f} V: enabling starts a shutdown in about {hold_s:.0f} s. "
                                    "Send confirm: true to enable anyway")
        new = {"enabled": bool(enabled), "volts": round(float(volts), 2), "hold_s": round(hold_s, 1)}
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
            return f"pack low: shutdown in {max(0, s['hold_s'] - (now - self.low_since)):.0f} s unless it recovers"
        return "armed"

    def public(self, now: Optional[float]) -> dict:
        with self._lock:
            bat = self.last
            return dict(self.settings, status=self.status(now), low=self.is_low(bat),
                        pack_v=None if bat is None else round(bat["pack_mv"] / 1000, 3),
                        range_v=list(VOLTS_RANGE), range_hold_s=list(HOLD_RANGE),
                        first_run=self.first_run, source="ups")
