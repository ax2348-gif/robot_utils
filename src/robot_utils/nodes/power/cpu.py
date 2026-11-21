"""CPU frequency governor: read it, switch it, keep the choice (the power node's toggle).

The governor trades latency for power on the Pi 5 (one cpufreq policy for all four cores). Measured
2026-11-20 at a light robot load: ``ondemand`` kept the clock around 1.9 GHz with ~62 frequency changes/s (each a
firmware mailbox call), costing ~16% more CPU time and ~6 ms of estimator lag; ``performance`` pinned 2.4 GHz and
drew ~0.45 W more from the UPS pack.

Writing ``scaling_governor`` needs root. The node runs ``systemctl start robot-cpu-governor@<governor>.service``
(deploy/power/: a oneshot that validates the name against the kernel's list and writes it to every policy), which
a polkit rule lets this user run without sudo, and nothing else. The command runner is injected
(``CpuGovernor(run=...)``); the default refuses, so tests and ``--fake`` never touch the system.

The chosen governor is saved in the node's settings file (``power.json``, key ``cpu``) and re-applied at startup,
since the kernel starts every boot on its default.
"""
from __future__ import annotations

import json
import os
import subprocess
import threading
import time
from pathlib import Path
from typing import Callable, Optional

from ..common.lease import RequestError

SYSFS = Path("/sys/devices/system/cpu/cpufreq")
UNIT = "robot-cpu-governor@{}.service"
HELPER_UNIT_FILE = Path("/etc/systemd/system/robot-cpu-governor@.service")


def refuse_run(argv: list[str]) -> None:
    raise RuntimeError(f"no system access here (would run: {' '.join(argv)})")


def system_run(argv: list[str]) -> None:
    """Run a command (the real one, from __main__): raises with its stderr on failure."""
    done = subprocess.run(argv, capture_output=True, text=True, timeout=20)
    if done.returncode != 0:
        raise RuntimeError((done.stderr or done.stdout).strip() or f"exit {done.returncode}")


class CpuGovernor:
    def __init__(self, settings_path: Optional[Path], run: Callable[[list[str]], None] = refuse_run,
                 sysfs: Path = SYSFS, helper_unit_file: Path = HELPER_UNIT_FILE,
                 clock: Callable[[], float] = time.monotonic):
        self.path = Path(settings_path) if settings_path else None
        self.run, self.sysfs, self.helper_unit_file, self.clock = run, Path(sysfs), Path(helper_unit_file), clock
        self._lock = threading.Lock()
        self._trans: Optional[tuple[float, int]] = None      # (t, total_trans) of the last read
        self._rate: Optional[float] = None
        self.saved: Optional[str] = None
        self.last_error: Optional[str] = None
        if self.path and self.path.exists():
            try:
                self.saved = (json.loads(self.path.read_text()).get("cpu") or {}).get("governor")
            except (OSError, ValueError, AttributeError):
                pass

    # -- reading -----------------------------------------------------------------------------------

    def _policies(self) -> list[Path]:
        return sorted(self.sysfs.glob("policy*"))

    def _read(self, p: Path, name: str) -> Optional[str]:
        try:
            return (p / name).read_text().strip()
        except OSError:
            return None

    def read(self) -> dict:
        """The governor and clock now; ``transitions_s`` is the frequency-change rate since the last read."""
        pols = self._policies()
        if not pols:
            return {"available": False, "governor": None, "governors": [], "error": "no cpufreq on this machine"}
        p = pols[0]
        govs = sorted(set.intersection(*[set((self._read(q, "scaling_available_governors") or "").split())
                                         for q in pols]))
        current = {self._read(q, "scaling_governor") for q in pols}
        khz = lambda n: (int(v) / 1e6 if (v := self._read(p, n)) and v.isdigit() else None)   # noqa: E731
        total = self._read(p, "stats/total_trans")
        now = self.clock()
        with self._lock:
            if total is not None and total.isdigit():
                t = int(total)
                if self._trans is not None and now - self._trans[0] >= 1.0:
                    self._rate = round((t - self._trans[1]) / (now - self._trans[0]), 1)
                if self._trans is None or now - self._trans[0] >= 1.0:
                    self._trans = (now, t)
            rate = self._rate
        return {"available": True, "governor": current.pop() if len(current) == 1 else "mixed",
                "governors": govs, "ghz": khz("scaling_cur_freq"),
                "ghz_range": [khz("scaling_min_freq"), khz("scaling_max_freq")], "transitions_s": rate,
                "policies": len(pols), "saved": self.saved, "helper_installed": self.helper_unit_file.exists(),
                "error": self.last_error}

    # -- switching ---------------------------------------------------------------------------------

    def set(self, governor: str) -> dict:
        """Switch every policy to ``governor`` (validated against the kernel's list) and save the choice."""
        st = self.read()
        if not st["available"]:
            raise RequestError(503, st["error"])
        if governor not in st["governors"]:
            raise RequestError(422, f"unknown governor {governor!r}; this kernel has {', '.join(st['governors'])}")
        if not st["helper_installed"]:
            raise RequestError(503, "the governor helper is not installed: run `sudo sh "
                                    "~/robot_utils/deploy/power/install.sh` once (docs/power.md, CPU governor)")
        try:
            self.run(["systemctl", "start", UNIT.format(governor)])
        except Exception as e:  # noqa: BLE001
            self.last_error = f"switching to {governor} failed: {e}"
            raise RequestError(502, self.last_error)
        got = self.read()["governor"]
        if got != governor:
            self.last_error = f"asked for {governor}, the kernel reports {got}"
            raise RequestError(502, self.last_error)
        self.last_error = None
        self.saved = governor
        self._save()
        return self.read()

    def apply_saved(self) -> Optional[str]:
        """At startup: re-apply the saved governor if the kernel is on another one. Returns what happened."""
        if not self.saved:
            return None
        st = self.read()
        if not st["available"] or st["governor"] == self.saved:
            return None
        try:
            self.set(self.saved)
            return f"CPU governor {st['governor']} -> {self.saved} (saved choice)"
        except RequestError as e:
            return f"CPU governor: could not re-apply the saved {self.saved}: {e}"

    def _save(self) -> None:
        if not self.path:
            return
        try:
            data = json.loads(self.path.read_text()) if self.path.exists() else {}
        except (OSError, ValueError):
            data = {}
        data["cpu"] = {"governor": self.saved}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".cpu.tmp")
        tmp.write_text(json.dumps(data, indent=1))
        os.replace(tmp, self.path)
