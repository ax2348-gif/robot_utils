"""The WBC's links to its peers: the arm (HTTP control plane; the joint channel is arm_channel.py) and the swerve
base (HTTP: lease, limits, enable, and the existing ``POST /command`` every tick from a sender thread, so a slow
HTTP round trip never stalls the 100 Hz loop; newest twist wins)."""
from __future__ import annotations

import threading
import time
from typing import Optional

from ..common.client import NodeClient
from ..swerve.client import SwerveClient


class ArmHttp(NodeClient):
    """The arm node's HTTP control plane (the same lease convention as the swerve's)."""

    def __init__(self, target: str = "arm", timeout: float = 2.0):
        super().__init__(target, timeout)
        self.token: Optional[str] = None

    def acquire(self, client: str) -> dict:
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

    def enable(self) -> dict:
        return self.post("/enable", request_timeout=10.0)     # the first enable checks the gripper (seconds)

    def disable(self) -> dict:
        return self.post("/disable")

    def estop(self, reason: str, source: str = "wbc") -> dict:
        return self.post("/estop", {"reason": reason, "source": source})

    def tcp(self) -> Optional[list]:
        try:
            out = self.get("/tcp")
        except Exception:  # noqa: BLE001 - optional
            return None
        off = out.get("offset") if isinstance(out, dict) else None
        return off if isinstance(off, list) and len(off) == 3 else None


class LimitsError(ValueError):
    """The WBC's base caps exceed the swerve's hard limits."""


class SwerveBase:
    """The swerve node as the WBC's base: lease + limits + enable, then one twist per tick through a sender
    thread (``send`` never blocks). ``failing_for()`` says how long commands have been failing."""

    def __init__(self, target: str, timeout: float = 0.25):
        self.target = target
        self.timeout = timeout
        self.client: Optional[SwerveClient] = None
        self.lease_id: Optional[int] = None
        self.hard: dict = {}                      # the swerve's GET /limits "hard", read at open
        self._cond = threading.Condition()
        self._pending: Optional[tuple] = None
        self._muted = False
        self._stop = False
        self._thread: Optional[threading.Thread] = None
        self.sent = 0
        self.errors = 0
        self.last_error: Optional[str] = None
        self._fail_since: Optional[float] = None
        self.last_ok: Optional[float] = None

    def open(self, client: str, max_speed: float, max_omega: float) -> int:
        c = SwerveClient(self.target, timeout=self.timeout)
        try:
            out = c.acquire(client)
        except Exception:
            c.close()
            raise
        try:
            hard = (c.get("/limits") or {}).get("hard") or {}
            self.hard = hard
            hs, hw = hard.get("max_speed"), hard.get("max_omega")
            if (hs is not None and max_speed > hs) or (hw is not None and max_omega > hw):
                raise LimitsError(f"the WBC's caps ({max_speed} m/s, {max_omega} rad/s) exceed the swerve's hard "
                                  f"limits ({hs} m/s, {hw} rad/s): lower max_speed / max_omega")
            c.set_limits(max_speed, max_omega)
            c.enable()
        except Exception:
            try:
                c.release()
            finally:
                c.close()
            raise
        self.client, self.lease_id = c, int(out["lease_id"])
        self._stop, self._pending, self._fail_since = False, None, None
        self.last_ok = time.monotonic()
        self._thread = threading.Thread(target=self._run, name="wbc-base", daemon=True)
        self._thread.start()
        return self.lease_id

    def send(self, twist) -> None:
        with self._cond:
            if self._muted:
                return
            self._pending = tuple(float(v) for v in twist)
            self._cond.notify()

    def mute(self) -> None:
        """No further commands (the e-stop): a queued twist is dropped and ``send`` does nothing from now on. Not a
        zero twist: the swerve's own e-stop / disable coasts, the robot never brakes of its own volition."""
        with self._cond:
            self._muted = True
            self._pending = None

    def failing_for(self) -> float:
        f = self._fail_since
        return 0.0 if f is None else time.monotonic() - f

    def _run(self) -> None:
        while True:
            with self._cond:
                while self._pending is None and not self._stop:
                    self._cond.wait(0.5)
                if self._stop:
                    return
                tw, self._pending = self._pending, None
            c = self.client
            if c is None:
                continue
            try:
                c.command(*tw)
                self.sent += 1
                self._fail_since = None
                self.last_ok = time.monotonic()
            except Exception as e:  # noqa: BLE001 - counted; the node faults after a while
                self.errors += 1
                self.last_error = f"{type(e).__name__}: {e}"
                if self._fail_since is None:
                    self._fail_since = time.monotonic()

    def close(self, disable: bool = True) -> None:
        """Stop the sender; disable and release the swerve (best effort, in the background)."""
        with self._cond:
            self._stop = True
            self._cond.notify()
        c, self.client = self.client, None
        self.lease_id = None
        if c is None:
            return

        def release():
            for call in ((c.disable, c.release) if disable else (c.release,)):
                try:
                    call()
                except Exception:  # noqa: BLE001
                    pass
            c.close()
        threading.Thread(target=release, daemon=True).start()

    def estop(self, reason: str, source: str = "wbc") -> None:
        with SwerveClient(self.target, timeout=0.5) as c:
            c.estop(reason, source)
