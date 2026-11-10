"""The arm's bus: one interface over the real motors (``HardwareBus``, rebot_ctl's ``RebotArm``)
and a simulation (``SimBus``). Only the node's control thread calls these.

A ``Reading`` is complete or the read raises ``BusError``: a missing or stale value is never
replaced by 0 (``RebotArm.get_state`` would). Positions come from the output encoders (xout,
register 81) for proprioception; the motor-side feedback position, velocity and torque come from
the feedback frames (the motor's own MIT loop runs on those).

Damiao status codes (feedback ERR field): 0 disabled, 1 enabled, 8 over-voltage, 9 under-voltage,
0xA over-current, 0xB MOS over-temperature, 0xC rotor over-temperature, 0xD communication lost
(CAN watchdog), 0xE overload. 8 and above is a fault.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

N = 6
FAULT_CODES = {8: "over-voltage", 9: "under-voltage", 0xA: "over-current", 0xB: "MOS over-temperature",
               0xC: "rotor over-temperature", 0xD: "communication lost (CAN watchdog)", 0xE: "overload"}


class BusError(RuntimeError):
    """A read or write failed; the node faults (motors disabled)."""


@dataclass
class Reading:
    t: float
    q: np.ndarray          # 6, output encoders (rad)
    q_mot: np.ndarray      # 6, motor-side feedback position (rad)
    qd: np.ndarray         # 6, feedback velocity (rad/s)
    tau: np.ndarray        # 6, feedback torque (Nm)
    status: np.ndarray     # 7, Damiao status codes (arm joints + gripper)
    t_mos: np.ndarray      # 7, °C
    t_rotor: np.ndarray    # 7, °C
    g_q: float             # gripper, output encoder (rad; 0 closed, opening negative)
    g_q_mot: float
    g_qd: float
    g_tau: float

    def faults(self, names: list[str]) -> list[str]:
        return [f"{n}: {FAULT_CODES.get(int(c), f'status {int(c):#x}')}"
                for n, c in zip(names, self.status) if int(c) >= 8]


class ArmBus:
    """Interface. ``connect`` and ``check_gripper`` block (seconds); the rest are per tick."""
    sim = False
    names = [f"joint{i}" for i in range(1, N + 1)] + ["gripper"]
    setpoint_limits: dict = {}

    def connect(self) -> None: ...
    def check_gripper(self) -> float: ...
    def enable(self) -> None: ...
    def disable(self) -> None: ...
    def read(self) -> Reading: ...
    def send(self, q, qd, kp, kd, tau, g_q, g_qd, g_kp, g_kd, g_tau) -> None: ...
    def close(self) -> None: ...


class HardwareBus(ArmBus):
    """The real arm through rebot_ctl (``RebotArm``; hardware yaml = the gain profile file, which
    also names the serial bridge)."""

    def __init__(self, hw_yaml: str, channel: Optional[str] = None):
        from reBotArm_control_py.actuator import RebotArm
        self.arm = RebotArm(hw_yaml, channel=channel)
        self.setpoint_limits = dict(self.arm.setpoint_limits)
        self.names = [j.name for j in self.arm._all_joints]
        assert self.names == [f"joint{i}" for i in range(1, N + 1)] + ["gripper"], self.names

    def connect(self) -> None:
        # Watchdog (100 ms) and joint-limit (2π wrap) checks; the gripper check runs on enable.
        self.arm.connect(check_gripper=False)
        for g in (self.arm.arm, self.arm.gripper):
            g.clear_error()
            if not g.mode_mit():
                raise BusError(f"{g.name}: a motor did not confirm MIT mode")
        self.arm.disable_all()

    def check_gripper(self) -> float:
        """Close the gripper and require it to read near zero (rebot_ctl; up to 2 tries)."""
        from reBotArm_control_py.actuator import GripperCheckError
        for attempt in (1, 2):
            try:
                return float(self.arm.check_gripper_closed())
            except GripperCheckError:
                if attempt == 2:
                    raise
        raise AssertionError

    def enable(self) -> None:
        for g in (self.arm.arm, self.arm.gripper):
            g.clear_error()
            g.enable()

    def disable(self) -> None:
        self.arm.disable_all()

    def read(self) -> Reading:
        from motorbridge import CallError
        from reBotArm_control_py.actuator import rebotarm as rb
        mm, cm = self.arm._motor_map, self.arm._ctrl_map
        try:
            batched = rb._request_xout(mm, self.names)
            for n in self.names:
                mm[n].request_feedback()
            for c in cm.values():
                c.poll_feedback_once()
            xout = rb._collect_xout(mm, list(cm.values()), self.names, batched, rb.XOUT_TIMEOUT_S)
        except CallError as e:
            raise BusError(f"read failed: {e}") from e
        sts = [mm[n].get_state() for n in self.names]
        missing = [n for n, s in zip(self.names, sts) if s is None]
        if missing:
            raise BusError(f"no feedback from {', '.join(missing)}")
        a = lambda f: np.array([getattr(s, f) for s in sts[:N]], dtype=float)   # noqa: E731
        g = sts[N]
        return Reading(time.monotonic(), np.array([xout[n] for n in self.names[:N]]), a("pos"), a("vel"), a("torq"),
                       np.array([s.status_code for s in sts], dtype=float),
                       np.array([s.t_mos for s in sts], dtype=float), np.array([s.t_rotor for s in sts], dtype=float),
                       float(xout["gripper"]), float(g.pos), float(g.vel), float(g.torq))

    def send(self, q, qd, kp, kd, tau, g_q, g_qd, g_kp, g_kd, g_tau) -> None:
        self.arm.arm.send_mit(q, vel=qd, kp=kp, kd=kd, tau=tau)
        self.arm.gripper.send_mit(np.array([g_q]), vel=np.array([g_qd]), kp=np.array([g_kp]),
                                  kd=np.array([g_kd]), tau=np.array([g_tau]))

    def close(self) -> None:
        self.arm.disconnect()


class SimBus(ArmBus):
    """A simulated arm: each joint a unit-ish inertia under its MIT law, gravity compensated
    exactly, integrated in 1 ms steps between reads. Disabled joints coast to a stop where they
    are (it does not fall). The gripper closes onto ``object_gap_rad`` if set (an object), else 0.
    """
    sim = True
    INERTIA = np.array([0.08, 0.08, 0.05, 0.01, 0.01, 0.005])
    G_INERTIA = 0.002
    G_OPEN = -3.1

    def __init__(self, q0=(0.0, -0.3, -0.6, 0.0, 0.0, 0.0), clock=time.monotonic, limits: Optional[dict] = None):
        self.clock = clock
        self.q = np.array(q0, dtype=float)
        self.qd = np.zeros(N)
        self.tau = np.zeros(N)
        self.g_q, self.g_qd, self.g_tau = -0.5, 0.0, 0.0
        self.enabled = False
        self.connected = False
        self.cmd = None
        self.t = None
        self.object_gap_rad: Optional[float] = None   # e.g. -1.0: something between the jaws
        self.fail_reads = False                      # tests: make the next reads fail
        self.fault_code = 0                          # tests: a motor fault on joint1
        self.setpoint_limits = dict(limits or {})

    def connect(self) -> None:
        self.connected = True
        self.t = self.clock()

    def check_gripper(self) -> float:
        self.g_q = 0.0 if self.object_gap_rad is None else self.object_gap_rad
        if abs(self.g_q) > math.radians(20):
            from reBotArm_control_py.actuator import GripperCheckError
            raise GripperCheckError(f"sim: gripper blocked at {self.g_q:+.3f} rad")
        return self.g_q

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False
        self.cmd = None

    def _step(self, dt: float) -> None:
        c = self.cmd if self.enabled else None
        if c is None:
            self.tau[:] = 0.0
            self.qd *= math.exp(-dt / 0.05)
            self.q += self.qd * dt
            self.g_qd = 0.0
            self.g_tau = 0.0
            return
        q, qd, kp, kd, _tau, g_q, g_qd, g_kp, g_kd, g_tau = c
        self.tau = kp * (q - self.q) + kd * (qd - self.qd)     # gravity cancels the feedforward
        self.qd += self.tau / self.INERTIA * dt
        self.q += self.qd * dt
        gt = g_kp * (g_q - self.g_q) + g_kd * (g_qd - self.g_qd) + g_tau
        self.g_tau = gt
        self.g_qd += gt / self.G_INERTIA * dt
        self.g_q += self.g_qd * dt
        stop = 0.0 if self.object_gap_rad is None else self.object_gap_rad
        if self.g_q > stop:                      # closed onto the stop / the object
            self.g_q, self.g_qd = stop, min(self.g_qd, 0.0)
        if self.g_q < self.G_OPEN:
            self.g_q, self.g_qd = self.G_OPEN, max(self.g_qd, 0.0)

    def read(self) -> Reading:
        if self.fail_reads:
            raise BusError("sim: no xout reply within 5 ms from: joint3")
        now = self.clock()
        if self.t is not None:
            dt = min(max(now - self.t, 0.0), 0.2)
            steps = max(1, int(round(dt / 0.001)))
            for _ in range(steps):
                self._step(dt / steps)
        self.t = now
        status = np.full(N + 1, 1.0 if self.enabled else 0.0)
        status[0] = self.fault_code or status[0]
        return Reading(now, self.q.copy(), self.q.copy(), self.qd.copy(), self.tau.copy(), status,
                       np.full(N + 1, 35.0), np.full(N + 1, 38.0), self.g_q, self.g_q, self.g_qd, self.g_tau)

    def send(self, q, qd, kp, kd, tau, g_q, g_qd, g_kp, g_kd, g_tau) -> None:
        if self.enabled:
            self.cmd = (np.array(q, float), np.array(qd, float), np.array(kp, float), np.array(kd, float),
                        np.array(tau, float), float(g_q), float(g_qd), float(g_kp), float(g_kd), float(g_tau))

    def close(self) -> None:
        self.connected = False
        self.enabled = False
