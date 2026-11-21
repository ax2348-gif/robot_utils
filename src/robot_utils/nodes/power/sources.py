"""Power sources: one class per kind of supply. ``/state`` lists ``Source.public()`` of each.

Every source has the same envelope, so a page or the landing tile can render one it has never
seen (docs/power.md, "Sources"):

    id          stable source id ("ups", "main", later "aux")
    type        "ups" | "tether" | (later) "battery"
    role        what it powers: "pi" (keeps the Pi up), "main" (the robot), "aux"
    telemetry   false when there is nothing to read (a bench tether)
    ok          true / false: last read succeeded and is fresh; null without telemetry
    stale       true when the last good reading is older than STALE_S
    error       the last read error, or null
    age_s       seconds since the last good reading (null if none)

and, for a source with a battery, the common battery fields (null when unknown):

    percent, pack_v, cells_v [..], current_a (+ = charging), power_w, charge_state
    ("charging" | "full" | "discharging" | "idle"), charging, input_present,
    runtime_min (+ runtime_source "gauge" | "estimate"), time_to_full_min, remaining_mah

A main or aux battery later is one more ``Source`` subclass with these fields (e.g. read from a
BMS over CAN); the API, the page's source cards and the history keep working unchanged.
"""
from __future__ import annotations

from typing import Optional

from .ups import UpsHat

STALE_S = 5.0                 # a reading older than this is stale
DISCHARGE_MA = -20            # below this the pack is discharging (noise around 0 is "idle")
# THE drain threshold, used everywhere (charge state, draining on input, the low-voltage trigger, the power-loss
# estimate, the history's anomaly spans): net pack current below this = draining. The readings are steady to about
# +-5 mA; a slow drain of -0.03 A (-0.4 W) is real and must not read as "not draining".
DRAIN_A = DISCHARGE_MA / 1000   # -0.02 A
# one row per HISTORY_S; the power-flow and cell columns are appended so older readers keep their indices
HISTORY_FIELDS = ["t", "wall", "percent", "pack_v", "current_a", "input_present", "charging",
                  "input_w", "battery_w", "load_w", "cell_min_v", "cell_spread_v"]
HISTORY_UNITS = ["s (monotonic)", "s (epoch)", "%", "V", "A (+ charging)", "0/1", "0/1",
                 "W (USB-C input)", "W (pack net: + charging, - draining)",
                 "W (derived: input - pack net; includes converter losses)", "V (lowest cell)", "V (max - min cell)"]


def flow(input_w: Optional[float], battery_w: Optional[float], cells_v: list) -> list:
    """The appended history columns: input_w, battery_w, load_w (derived), cell_min_v, cell_spread_v.
    None where an input is unknown (e.g. a log row without cells)."""
    load = None if input_w is None or battery_w is None else round(input_w - battery_w, 2)
    cells = [c for c in cells_v if c is not None]
    return [None if input_w is None else round(input_w, 2), None if battery_w is None else round(battery_w, 2), load,
            round(min(cells), 3) if cells else None, round(max(cells) - min(cells), 3) if cells else None]


def history_row_of(t: float, wall: float, d: dict) -> list:
    """One HISTORY_FIELDS row from a decoded HAT reading (the live history and the --fake backfill)."""
    pack_v = d["pack_mv"] / 1000
    input_w = d["vbus_mw"] / 1000 if d["vbus_present"] else 0.0
    return [t, wall, d["percent"], pack_v, d["current_ma"] / 1000,
            1.0 if d["vbus_present"] else 0.0, 1.0 if d["charging"] else 0.0,
            *flow(input_w, pack_v * d["current_ma"] / 1000, [c / 1000 for c in d["cells_mv"]])]


def draining_on_input(d: dict) -> bool:
    """Input present but the pack net discharging: the load is more than the input supplies (the charger
    bit can still say "charging"). One reading, no hold (powerlog.Watch holds it 10 s for its events)."""
    return bool(d["vbus_present"]) and d["current_ma"] / 1000 < DRAIN_A


class Source:
    name = "source"
    type = "unknown"
    role = "unknown"
    telemetry = False

    def poll(self, now: float) -> None:
        """Read the hardware (owner thread only)."""

    def public(self, now: float) -> dict:
        return {"id": self.name, "type": self.type, "role": self.role, "telemetry": self.telemetry,
                "ok": None, "stale": False, "error": None, "age_s": None}

    def history_row(self, now: float, wall: float) -> Optional[list]:
        """One row of HISTORY_FIELDS, or None (no telemetry / no fresh reading)."""
        return None

    def battery(self) -> Optional[dict]:
        """(pack_mv, charging, valid, percent) for the low-voltage check, or None."""
        return None


class TetherSource(Source):
    """A bench power supply on a tether: nothing to read."""
    type = "tether"
    telemetry = False

    def __init__(self, name: str = "main", role: str = "main", note: str = "bench supply on a tether; no battery installed"):
        self.name, self.role, self.note = name, role, note

    def public(self, now: float) -> dict:
        return dict(super().public(now), note=self.note)


class UpsSource(Source):
    """The Pi's UPS HAT (E): pack, cells, input, charger, gauge."""
    type = "ups"
    telemetry = True

    def __init__(self, ups: Optional[UpsHat], name: str = "ups", role: str = "pi", error: Optional[str] = None):
        self.ups, self.name, self.role = ups, name, role
        self.model = "Waveshare UPS HAT (E)"
        self.data: Optional[dict] = None
        self.t_ok: Optional[float] = None
        self.error: Optional[str] = error       # e.g. no I2C library: set once, never cleared
        self.fixed_error = error is not None
        self.reads = 0
        self.failures = 0

    def poll(self, now: float) -> None:
        if self.ups is None:
            return
        try:
            d = self.ups.read()
        except OSError as e:
            self.failures += 1
            self.error = f"I2C read failed: {e}"
            return
        self.reads += 1
        if not d["valid"]:
            self.failures += 1
            self.error = f"unexpected ID 0x{(d['id'] or 0):02x} (want 0x0a): not the UPS HAT?"
            return
        self.data, self.t_ok = d, now
        if not self.fixed_error:
            self.error = None

    def fresh(self, now: float) -> bool:
        return self.t_ok is not None and now - self.t_ok <= STALE_S

    def battery(self) -> Optional[dict]:
        if self.data is None:
            return None
        d = self.data
        return {"pack_mv": d["pack_mv"], "charging": d["charging"], "percent": d["percent"],
                "current_ma": d["current_ma"], "cells_mv": list(d["cells_mv"]), "vbus_present": d["vbus_present"]}

    @staticmethod
    def readings(d: dict) -> dict:
        """The common battery fields from a decoded HAT reading."""
        ma = d["current_ma"]
        if d["charger_state"] == "full" or (d["charging"] and ma <= 0 and d["percent"] >= 100):
            cs = "full"
        elif d["charging"]:
            cs = "charging"
        elif ma < DISCHARGE_MA:
            cs = "discharging"
        else:
            cs = "idle"
        runtime, how = None, None
        if cs == "discharging":
            if d["time_to_empty_min"] is not None:
                runtime, how = d["time_to_empty_min"], "gauge"
            elif ma < 0 and d["remaining_mah"]:
                runtime, how = round(d["remaining_mah"] / -ma * 60), "estimate"
        pack_v = d["pack_mv"] / 1000
        return {
            "percent": d["percent"], "pack_v": round(pack_v, 3),
            "cells_v": [round(c / 1000, 3) for c in d["cells_mv"]],
            "current_a": round(ma / 1000, 3), "power_w": round(pack_v * ma / 1000, 2),
            "charge_state": cs, "charging": d["charging"], "fast_charge": d["fast_charge"],
            "input_present": d["vbus_present"],
            "runtime_min": runtime, "runtime_source": how,
            "time_to_full_min": d["time_to_full_min"] if d["charging"] else None,
            "remaining_mah": d["remaining_mah"],
        }

    def public(self, now: float) -> dict:
        fresh = self.fresh(now)
        out = dict(super().public(now), model=self.model, ok=fresh and self.error is None,
                   stale=self.t_ok is not None and not fresh, error=self.error,
                   age_s=None if self.t_ok is None else round(now - self.t_ok, 1))
        d = self.data
        if d is None:
            return out
        out.update(self.readings(d))
        out.update({
            "input": {"present": d["vbus_present"], "v": round(d["vbus_mv"] / 1000, 3),
                      "a": round(d["vbus_ma"] / 1000, 3), "w": round(d["vbus_mw"] / 1000, 2)},
            "draining_on_input": draining_on_input(d),
            "charger_state": d["charger_state"], "links": d["links"], "firmware": d["firmware"],
            "power_off_pending": d["power_off_pending"],
            "control": d["control"],
        })
        return out

    def history_row(self, now: float, wall: float) -> Optional[list]:
        if not self.fresh(now):
            return None
        return history_row_of(now, wall, self.data)
