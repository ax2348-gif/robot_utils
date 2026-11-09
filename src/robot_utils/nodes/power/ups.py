"""The Waveshare UPS HAT (E) on the Pi: register map, decoding, and a fake bus.

The HAT's MCU sits on I2C bus 1 at address 0x2d (Waveshare "UPS HAT (E) Register" manual,
firmware V2). It fronts an IP2368 charger and a BQ4050 fuel gauge. 16-bit values are
little-endian. Ported from the robot's standalone ``~/ups-dashboard/ups_dashboard.py``.

  0x00     R   ID, always 0x0a
  0x01     R/W power-off trigger: writing 0x55 cuts power to the Pi after ~30 s, then turns it
               back on once input power is present. IRREVERSIBLE once written.
  0x02     R   charge status: b7 charging, b6 fast charge, b5 VBUS present, b2..0 charger state
  0x03     R   communication: b1 BQ4050 ok, b0 IP2368 ok
  0x10-15  R   USB-C VBUS mV, mA, mW
  0x20-2b  R   battery mV, mA (signed, + = charging), %, mAh, minutes to empty, minutes to full
               (0xffff = n/a)
  0x30-37  R   cell 1..4 mV
  0x40     R/W control: b1 watchdog enable (default 0), b0 auto power-on (default 1)
  0x41     R/W I2C address, 0x08..0x77 (default 0x2d)
  0x42     R/W watchdog timeout, 0..255 s (default 255)
  0x43     R/W watchdog start-up delay, 0..255 s (default 255)
  0x50     R   firmware version x10 (0x14 = V2.0)

This module only ever **reads** the HAT, with one exception: ``UpsHat.arm_power_off()`` writes
0x01 <- 0x55, and only the node's injected power-off function calls it (never in tests or
``--fake``). Registers 0x40-0x43 are never written: the watchdog and address are left alone.

``smbus2`` (pure Python) or the system ``smbus`` is imported only when a real bus is opened.
"""
from __future__ import annotations

import math
import threading
from typing import Optional

BUS = 1
ADDR = 0x2d
ID_VALUE = 0x0a
REG_POWER_OFF = 0x01
POWER_OFF_MAGIC = 0x55
BLOCKS = [(0x00, 4), (0x10, 6), (0x20, 12), (0x30, 8), (0x40, 4), (0x50, 1)]
NA = 0xFFFF
CELLS = 4
CHARGER_STATE = ["standby", "trickle charge", "constant current", "constant voltage",
                 "waiting", "full", "timeout", "undefined"]

# A real reading from the robot's HAT (captured by the old dashboard): on USB-C input, fast
# charging at 512 mA, 59 %, 15.587 V.
CAPTURED = {
    0x00: [0x0a, 0x0b, 0xe2, 0x03],
    0x10: [0xe3, 0x21, 0x2a, 0x08, 0xfc, 0x46],
    0x20: [0xe3, 0x3c, 0x00, 0x02, 0x3b, 0x00, 0xff, 0x0a, 0xff, 0xff, 0xfa, 0x00],
    0x30: [0x50, 0x0f, 0x22, 0x0f, 0x25, 0x0f, 0x4c, 0x0f],
    0x40: [0x01, 0x2d, 0xff, 0xff],
    0x50: [0x14],
}
# Read from the robot on 2026-11-09 (old dashboard's /api/state): tethered, full, topping off at 0 mA.
CAPTURED_FULL = {
    0x00: [10, 11, 163, 3],
    0x10: [206, 18, 162, 10, 124, 50],
    0x20: [8, 64, 0, 0, 100, 0, 165, 18, 255, 255, 255, 255],
    0x30: [25, 16, 230, 15, 234, 15, 32, 16],
    0x40: [1, 45, 255, 255],
    0x50: [20],
}


class BusUnavailable(RuntimeError):
    pass


def open_bus(bus: int = BUS):
    """``SMBus(bus)`` from smbus2, else the system smbus. Raises BusUnavailable with a hint."""
    try:
        from smbus2 import SMBus
    except ImportError:
        try:
            from smbus import SMBus
        except ImportError:
            raise BusUnavailable("no I2C library: install smbus2 in the robot venv "
                                 "(.venv/bin/pip install smbus2), or run with --fake") from None
    try:
        return SMBus(bus)
    except OSError as e:
        raise BusUnavailable(f"cannot open /dev/i2c-{bus}: {e} (is the user in the i2c group?)") from e


def flatten(blocks: dict[int, list[int]]) -> dict[int, int]:
    return {start + i: b for start, data in blocks.items() for i, b in enumerate(data)}


def decode(r: dict[int, int]) -> dict:
    """Register bytes {addr: byte} -> named values with units (mV, mA, mW, %, mAh, min)."""
    def u16(a):
        return r.get(a, 0) | r.get(a + 1, 0) << 8

    def mins(a):
        v = u16(a)
        return None if v == NA else v

    st, cm, ctl = r.get(0x02, 0), r.get(0x03, 0), r.get(0x40, 0)
    ma = u16(0x22)
    if ma & 0x8000:
        ma -= 0x10000
    return {
        "valid": r.get(0x00) == ID_VALUE,
        "id": r.get(0x00),
        "firmware": f"{r.get(0x50, 0) / 10:.1f}",
        "power_off_pending": r.get(REG_POWER_OFF) == POWER_OFF_MAGIC,
        "charging": bool(st & 0x80),
        "fast_charge": bool(st & 0x40),
        "vbus_present": bool(st & 0x20),
        "charger_state": CHARGER_STATE[st & 0x07],
        "links": {"fuel_gauge": bool(cm & 0x02), "charger": bool(cm & 0x01)},
        "vbus_mv": u16(0x10), "vbus_ma": u16(0x12), "vbus_mw": u16(0x14),
        "pack_mv": u16(0x20),
        "current_ma": ma,                       # + = charging
        "percent": u16(0x24),
        "remaining_mah": u16(0x26),
        "time_to_empty_min": mins(0x28),
        "time_to_full_min": mins(0x2a),
        "cells_mv": [u16(0x30 + 2 * i) for i in range(CELLS)],
        "control": {"auto_power_on": bool(ctl & 0x01), "watchdog": bool(ctl & 0x02),
                    "address": r.get(0x41), "wd_timeout_s": r.get(0x42), "wd_delay_s": r.get(0x43)},
    }


class UpsHat:
    """Reads the HAT (block reads only). ``arm_power_off`` is the only write, see the module doc."""

    def __init__(self, bus, addr: int = ADDR):
        self.bus, self.addr = bus, addr
        self._lock = threading.Lock()

    def registers(self) -> dict[int, int]:
        out = {}
        with self._lock:
            for start, n in BLOCKS:
                for i, b in enumerate(self.bus.read_i2c_block_data(self.addr, start, n)):
                    out[start + i] = b
        return out

    def read(self) -> dict:
        return decode(self.registers())

    def arm_power_off(self) -> None:
        """0x01 <- 0x55: the HAT cuts power in ~30 s. Irreversible. Only for the real power-off."""
        with self._lock:
            self.bus.write_byte_data(self.addr, REG_POWER_OFF, POWER_OFF_MAGIC)


class FakeBus:
    """Dict-backed stand-in for SMBus holding a real reading (``CAPTURED``). Records writes."""

    def __init__(self, blocks: Optional[dict] = None):
        self.regs = flatten(blocks or CAPTURED)
        self.writes: list[tuple[int, int, int]] = []
        self.fail: Optional[str] = None          # tests: make reads raise OSError(fail)

    def read_i2c_block_data(self, addr, reg, n):
        if self.fail:
            raise OSError(self.fail)
        return [self.regs.get(reg + i, 0) for i in range(n)]

    def write_byte_data(self, addr, reg, value):
        self.writes.append((addr, reg, value))
        self.regs[reg] = value

    def set_word(self, reg: int, value: int) -> None:
        value &= 0xFFFF
        self.regs[reg], self.regs[reg + 1] = value & 0xFF, value >> 8

    def set_status(self, charging: bool, vbus: bool, fast: bool = False, state: int = 2) -> None:
        self.regs[0x02] = (0x80 if charging else 0) | (0x40 if fast else 0) | (0x20 if vbus else 0) | (state & 7)


class SimBus(FakeBus):
    """``--fake``: the captured reading, animated on a 4 h cycle so pages have something to show.

    2 h on input (charging at ~1.5 A), then 2 h on battery (~1.2 A load). ``clock`` gives
    seconds; ``regs_at(t)`` is pure, so a history can be back-filled with the same curve."""

    PERIOD_S = 4 * 3600.0

    def __init__(self, clock=None):
        super().__init__()
        import time
        self.clock = clock or time.monotonic

    def regs_at(self, t: float) -> dict[int, int]:
        ph = (t % self.PERIOD_S) / self.PERIOD_S
        on_input = ph < 0.5
        frac = ph / 0.5 if on_input else (ph - 0.5) / 0.5
        pct = 45 + 45 * frac if on_input else 90 - 45 * frac
        pct += 1.5 * math.sin(t / 37.0)
        cell = 3.30 + 0.9 * pct / 100 + (0.06 if on_input else -0.05)
        ma = int(1500 - 600 * frac) if on_input else int(-1200 - 250 * math.sin(t / 23.0))
        f = FakeBus(CAPTURED)
        f.set_status(charging=on_input, vbus=on_input, fast=on_input and pct < 80, state=2 if pct < 80 else 3)
        offs = [0.012, -0.018, -0.006, 0.010]
        cells = [int((cell + o) * 1000) for o in offs]
        f.set_word(0x20, sum(cells))
        f.set_word(0x22, ma)
        f.set_word(0x24, int(round(pct)))
        f.set_word(0x26, int(5000 * pct / 100))
        f.set_word(0x28, int(5000 * pct / 100 / abs(ma) * 60) if ma < 0 else NA)
        f.set_word(0x2a, int(5000 * (100 - pct) / 100 / ma * 60) if ma > 0 else NA)
        for i, c in enumerate(cells):
            f.set_word(0x30 + 2 * i, c)
        if on_input:
            f.set_word(0x10, 8600); f.set_word(0x12, 2100); f.set_word(0x14, 18000)
        else:
            f.set_word(0x10, 0); f.set_word(0x12, 0); f.set_word(0x14, 0)
        return f.regs

    def read_i2c_block_data(self, addr, reg, n):
        if self.fail:
            raise OSError(self.fail)
        live = self.regs_at(self.clock())
        live[REG_POWER_OFF] = self.regs.get(REG_POWER_OFF, live[REG_POWER_OFF])
        return [live.get(reg + i, 0) for i in range(n)]
