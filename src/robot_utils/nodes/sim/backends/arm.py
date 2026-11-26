"""``WorldArmBus``: the arm node's ``ArmBus`` on the sim world (docs/sim.md, "Arm backend").

    ArmNode(lambda: WorldArmBus("sim", limits=...))

The world holds the motors (enable state, the last MIT frames, a CAN watchdog); this bus speaks the
``arm.sock`` protocol (``hello``, ``enable``, ``disable``, ``gripper_motor``, ``check_gripper``, ``send``,
``read``) and reproduces what the node sees of the hardware:

- **pipelined reads**, as ``HardwareBus``: each ``read`` returns the replies to the requests sent at the
  previous read (the world's state as of then) and sends the next requests; the first read after
  connecting waits for its own. ``reply_latency`` is the world's configured value.
- **the driver's setpoint check**, as ``SimBus`` / rebot_ctl: a setpoint outside the limits is refused
  (``BusError``) unless no further outside than the last one sent; kp = 0 positions are not checked.
- **measurement times** ``Reading.t_meas`` / ``t_meas_mot`` (the arm row's columns), as the hardware's:
  ``t_meas``: when the requests whose replies gave q were sent (the previous read, so about one tick
  before the reading); ``t_meas_mot``: when the world delivered the motor feedback used for q_mot / qd /
  tau (its answer to the last MIT frames, or to the read request when none were sent).
"""
from __future__ import annotations

import math
import time
from typing import Optional

import numpy as np

from ...arm.bus import ArmBus, BusError, FeedbackRefresh, Reading
from ...arm.config import N
from ..protocol import IpcClient, IpcError, backend_socket


class WorldArmBus(ArmBus):
    sim = True

    def __init__(self, sim: str = "sim", limits: Optional[dict] = None, clock=time.monotonic, socket_path=None,
                 timeout: float = 2.0, arm: str = "arm"):
        """``sim``: the sim node's instance name (its ``<arm>.sock``: ``arm`` names the arm, robots/), or
        ``socket_path`` directly;
        ``limits``: {joint: (lo, hi)} as the driver enforces them (``joint_limits(load_cfg(profile))``)."""
        self.client = IpcClient(socket_path or backend_socket(sim, arm), timeout=timeout)
        self.clock = clock
        self.setpoint_limits = dict(limits or {})
        self.refresh = FeedbackRefresh(self.names, skip=True)
        self._pending: Optional[tuple] = None      # (t request sent, world reply) for the next read
        self._mit_reply_t: Optional[float] = None  # when the world answered the last MIT frames
        self._q_last = np.full(N, np.nan)
        self.connected = False

    def _call(self, op: str, **kw) -> dict:
        try:
            return self.client.call(op, **kw)[0]
        except IpcError as e:
            raise BusError(f"sim world: {e}") from None

    def connect(self) -> None:
        info = self._call("hello")
        self.reply_latency = float(info.get("reply_latency") or 0.0) or None
        self._pending = None
        self.connected = True

    def check_gripper(self) -> float:
        return float(self._call("check_gripper")["g_q"])

    def enable(self) -> None:
        self._call("enable")
        self._q_last = np.full(N, np.nan)

    def gripper_motor(self, on: bool) -> None:
        if not on:
            self.refresh.forget(["gripper"])
        self._call("gripper_motor", on=bool(on))

    def seed(self, q) -> None:
        self._q_last = np.asarray(q, float).copy()

    def disable(self) -> None:
        self.refresh.forget(self.names)
        self._call("disable")

    def _request(self) -> tuple:
        t = self.clock()
        r = self._call("read")
        return t, r, self.clock()

    def read(self) -> Reading:
        win = self.refresh.rotate()
        fed = {}
        if self._pending is None:                 # first read: request and wait
            self._pending = self._request()
        t_req, r, t_reply = self._pending
        self._pending = self._request()           # the requests for the next tick
        if win is not None:                       # every MIT frame was answered (with the read's reply)
            for n in win[1]:
                fed[n] = t_reply
        self.refresh.plan(fed)
        t_mot = self._mit_reply_t if (win is not None and self._mit_reply_t is not None) else t_reply
        st = np.asarray(r["status"], float)
        q = np.asarray(r["q"], float)
        out = Reading(self.clock(), q, q.copy(), np.asarray(r["qd"], float), np.asarray(r["tau"], float), st,
                      np.full(N + 1, 35.0), np.full(N + 1, 38.0), float(r["g_q"]), float(r["g_q"]),
                      float(r["g_qd"]), float(r["g_tau"]), t_meas=t_req, t_meas_mot=t_mot)
        return out

    def send(self, q, qd, kp, kd, tau, g_q, g_qd, g_kp, g_kd, g_tau, gripper: bool = True) -> None:
        names = [f"joint{i}" for i in range(1, N + 1)]
        lo = np.array([self.setpoint_limits.get(n, (-math.inf, math.inf))[0] for n in names])
        hi = np.array([self.setpoint_limits.get(n, (-math.inf, math.inf))[1] for n in names])
        q = np.asarray(q, float)
        out = np.maximum(lo - q, q - hi)
        last_out = np.maximum(lo - self._q_last, self._q_last - hi)
        checked = np.asarray(kp, float) != 0.0
        ok = ~checked | (out <= 0.0) | (out <= last_out)
        if not np.all(ok):
            raise BusError(f"the driver refused setpoints for {', '.join(n for n, k in zip(names, ok) if not k)}")
        self._q_last = np.where(checked, q, self._q_last)
        self.refresh.mit_sent(self.clock(), names + (["gripper"] if gripper else []))
        f = lambda v: [float(x) for x in np.asarray(v, float).reshape(-1)]   # noqa: E731
        self._call("send", q=f(q), qd=f(qd), kp=f(kp), kd=f(kd), tau=f(tau), g_q=float(g_q), g_qd=float(g_qd),
                   g_kp=float(g_kp), g_kd=float(g_kd), g_tau=float(g_tau), gripper=bool(gripper))
        self._mit_reply_t = self.clock()

    def close(self) -> None:
        self.connected = False
        self.client.close()
