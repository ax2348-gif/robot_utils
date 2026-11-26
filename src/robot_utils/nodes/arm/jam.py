"""Jam detection for the arm node: a STATIC arm still pushing (the aftermath of a contact, not the instant of
it). The detector only judges and keeps its record; the node applies the response (hold or report)."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .config import ARM_TYPE, N

REBOT_BANDS = [1.0, 4.0, 4.0, 1.5, 0.8, 0.6]   # Nm per joint (draft, offline study)

JAM_RESPONSES = ("hold", "report", "off")


@dataclass
class JamConfig:
    """Jam detection: a STATIC arm still pushing (the aftermath of a contact, not the instant of it). Judged only
    while servoing with no motion executing (a finished move counts; a move still settling too), every joint
    slower than ``static_qd`` for ``settle_s``. A joint's residual |tau - tau_g| above its band for ``hold_s``
    fires; it clears below ``release`` x band."""
    # Nm per joint: the reBot's, or the arm type's (robots/arm_types.py jam_bands)
    bands: list = field(default_factory=lambda: list(REBOT_BANDS if ARM_TYPE is None else ARM_TYPE["jam_bands"]))
    hold_s: float = 1.0
    release: float = 0.7
    static_qd: float = 0.05          # rad/s: every joint slower than this counts as static
    settle_s: float = 0.5            # s static before judging
    response: str = "hold"           # hold: command pulled to the reading (a move ends `jammed`) | report | off
    refire: bool = True              # fire again after clearing (e.g. the base keeps pushing the arm in)

    def public(self) -> dict:
        return {"bands": list(self.bands), "hold_s": self.hold_s, "release": self.release, "static_qd": self.static_qd,
                "settle_s": self.settle_s, "response": self.response, "refire": self.refire}


def updated(cfg: JamConfig, changes: dict) -> JamConfig:
    """``cfg`` with ``changes`` (None values ignored), validated (ValueError)."""
    new = JamConfig(**{**cfg.public(), **{k: v for k, v in changes.items() if v is not None}})
    if len(new.bands) != N or not all(math.isfinite(b) and b > 0 for b in new.bands):
        raise ValueError(f"bands: {N} positive numbers (Nm, one per joint)")
    if new.response not in JAM_RESPONSES:
        raise ValueError(f"response: one of {', '.join(JAM_RESPONSES)}")
    if not (0 < new.release < 1) or new.hold_s <= 0 or new.settle_s < 0 or new.static_qd <= 0:
        raise ValueError("release in (0, 1); hold_s > 0; settle_s >= 0; static_qd > 0")
    return new


@dataclass
class JamEvent:
    """What a check found: a jam ``cleared`` (``joint``), or a new ``jam`` (its record) with ``message``."""
    cleared: bool
    joint: int
    jam: Optional[dict] = None
    message: str = ""


class JamDetector:
    """The detector's state (control thread only): ``cfg``, the jam in force (``active``), the recent ones
    (``recent``, last 20, newest last) and their ``count``."""

    def __init__(self, cfg: Optional[JamConfig] = None):
        self.cfg = cfg or JamConfig()
        self.active: Optional[dict] = None       # the jam in force: joint, residual, since, response
        self.recent: list[dict] = []              # recent jams (last 20), newest last
        self.count = 0
        self.static_since: Optional[float] = None
        self.over_since = np.full(N, np.nan)      # per joint: when its residual went over its band (NaN = under)
        self.fired_cmd: Optional[int] = None      # with refire off: the command it fired for (fires once per command)

    def reset(self) -> None:
        self.static_since = None
        self.over_since[:] = np.nan
        self.active = None

    def check(self, now: float, qd, tau, tau_g, executing: bool, src: str, cmd: int) -> Optional[JamEvent]:
        """While servoing. ``executing``: a planned motion still executing. ``cmd``: the node's command count
        (refire off: one jam per command). Planned motions and holds get the configured response; track (a
        teleop stream overwrites the command every tick) is report-only."""
        cfg = self.cfg
        if cfg.response == "off" or tau_g is None:
            self.reset()
            return None
        moving = executing or bool(np.any(np.abs(qd) > cfg.static_qd))
        if moving:
            self.static_since = None
            self.over_since[:] = np.nan
            return None
        self.static_since = self.static_since or now
        if now - self.static_since < cfg.settle_s:
            return None
        res = np.asarray(tau, float) - np.asarray(tau_g, float)
        bands = np.asarray(cfg.bands, float)
        if self.active is not None:                       # hysteresis: clear only well under every band
            if np.all(np.abs(res) < cfg.release * bands):
                joint = self.active["joint"]
                self.active = None
                self.over_since[:] = np.nan
                return JamEvent(cleared=True, joint=joint)
            return None
        over = np.abs(res) > bands
        self.over_since = np.where(over, np.where(np.isnan(self.over_since), now, self.over_since), np.nan)
        held = np.where(over, now - self.over_since, 0.0)
        if not np.any(held >= cfg.hold_s):
            return None
        if not cfg.refire and self.fired_cmd == cmd:
            return None
        i = int(np.argmax(np.where(held >= cfg.hold_s, np.abs(res) / bands, 0.0)))
        act = "report" if (cfg.response == "report" or src in ("track", "stream")) else "hold"
        jam = {"t": now, "wall": time.time(), "joint": i + 1, "residual_nm": round(float(res[i]), 3),
               "band_nm": float(bands[i]), "held_s": round(float(held[i]), 2), "response": act, "source": src,
               "residuals_nm": [round(float(v), 3) for v in res]}
        self.count += 1
        jam["count"] = self.count
        self.active, self.fired_cmd = jam, cmd
        self.recent = (self.recent + [jam])[-20:]
        what = (f"jam: joint{i + 1} residual {res[i]:+.1f} Nm (band {bands[i]:g}) for {held[i]:.1f} s while static")
        return JamEvent(cleared=False, joint=i + 1, jam=jam, message=what)
