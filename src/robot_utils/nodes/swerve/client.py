"""Python client for the swerve node (needs httpx).

    from robot_utils.nodes.swerve.client import SwerveClient

    with SwerveClient("http://botcore/nodes/swerve") as base:   # on the robot: SwerveClient("swerve")
        base.sync()                              # clock sync for the stream's timestamps
        with base.control("my-experiment"):      # lease + enable; stops, disables, releases on exit
            base.drive_for((0.2, 0.0, 0.0), seconds=1.0)   # resends every 50 ms (0.25 s timeout)
        for header, rows in base.rows():         # every 100 Hz tick
            print(rows["FL_drive_vel"])

Recording: ``base.record("/stream", "run.bin", seconds=10)``; read with
``robot_utils.nodes.common.protocol.iter_rows(iter_records(open("run.bin", "rb")))``.
"""
from __future__ import annotations

import contextlib
import threading
import time
from typing import Optional

from ..common.client import NodeClient, NodeError   # noqa: F401 (re-exported)


class SwerveClient(NodeClient):
    def __init__(self, target: str = "swerve", timeout: float = 5.0):
        super().__init__(target, timeout)
        self.token: Optional[str] = None
        self._seq = 0

    # -- lease ---------------------------------------------------------------------------------

    def acquire(self, client: str = "python") -> dict:
        out = self.post("/lease", {"client": client})
        self.token = out["token"]
        self.headers["X-Lease-Token"] = self.token
        return out

    def release(self) -> None:
        if self.token is not None:
            try:
                self.delete("/lease")
            finally:
                self.token = None
                self.headers.pop("X-Lease-Token", None)

    def renew(self) -> dict:
        return self.post("/lease/renew")

    @contextlib.contextmanager
    def control(self, client: str = "python", keepalive: bool = True):
        """Acquire + enable; on exit (or error) disable and release. ``keepalive`` renews the
        lease in the background so slow scripts between commands don't lose it."""
        self.acquire(client)
        stop = threading.Event()
        if keepalive:
            def renew_loop():
                while not stop.wait(0.3):
                    try:
                        self.renew()
                    except Exception:
                        return
            threading.Thread(target=renew_loop, daemon=True).start()
        try:
            self.enable()
            yield self
        finally:
            stop.set()
            try:
                self.post("/disable")
            except Exception:
                pass
            self.release()

    # -- control -------------------------------------------------------------------------------

    def enable(self) -> dict:
        return self.post("/enable")

    def disable(self) -> dict:
        return self.post("/disable")

    def command(self, vx: float, vy: float, omega: float, seq: int = 0) -> dict:
        """One twist (holds 0.25 s). ``seq`` is optional, for debugging (echoed as cmd_seq)."""
        return self.post("/command", {"twist": [vx, vy, omega], "seq": seq})

    def drive_for(self, twist, seconds: float, rate: float = 20.0) -> None:
        """Hold a twist for ``seconds`` by resending it at ``rate`` Hz, then send zero."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.command(*twist)
            time.sleep(1.0 / rate)
        self.command(0.0, 0.0, 0.0)

    def set_limits(self, max_speed: Optional[float] = None, max_omega: Optional[float] = None) -> dict:
        return self.post("/limits", {"max_speed": max_speed, "max_omega": max_omega})

    def estop(self, reason: str = "", source: str = "python") -> dict:
        return self.post("/estop", {"reason": reason, "source": source})

    def clear_estop(self) -> dict:
        return self.post("/estop/clear")

    def reset(self) -> dict:
        return self.post("/reset")

    def state(self) -> dict:
        return self.get("/state")

    # -- drive encoders (anyone; moves nothing) ----------------------------------------------------

    def encoders(self) -> dict:
        return self.get("/encoders")

    def rebase_encoders(self) -> dict:
        """Drive encoders to 0 now; NodeError 409 unless the base is stationary."""
        return self.post("/encoders/rebase")

    def set_auto_rebase(self, enabled: Optional[bool] = None, interval_s: Optional[float] = None,
                        still_s: Optional[float] = None) -> dict:
        return self.post("/encoders/auto", {"enabled": enabled, "interval_s": interval_s, "still_s": still_s})
