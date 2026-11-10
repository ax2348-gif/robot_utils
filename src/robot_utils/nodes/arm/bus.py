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
    def gripper_motor(self, on: bool) -> None: ...
    def seed(self, q) -> None: ...
    def disable(self) -> None: ...
    def read(self) -> Reading: ...
    def send(self, q, qd, kp, kd, tau, g_q, g_qd, g_kp, g_kd, g_tau, gripper: bool = True) -> None: ...
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
        """Open the bus and check, nothing more: every motor's CAN watchdog (100 ms) and the joint
        readings against the limits (2π power-on wraps). Then disable-all, so motors a crashed
        process left enabled are off. No mode change, no fault clearing: those are part of
        switching the motors on (enable)."""
        self.arm.connect(check_gripper=False)
        self.arm.disable_all()

    @staticmethod
    def _prepare(g) -> None:
        """Before a group's motors go on: forget the driver's remembered setpoints (RebotArm resends
        the last one when it refuses a new one: across a disable that could be a pose the arm left
        long ago), clear latched faults, and switch to MIT mode, confirmed."""
        g._q_last[:] = np.nan
        g._q_refused[:] = False
        g.clear_error()
        if not g.mode_mit():
            raise BusError(f"{g.name}: a motor did not confirm MIT mode; not enabling")

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
        """Motors on (arm and gripper), each group prepared first (MIT mode confirmed)."""
        groups = (self.arm.arm, self.arm.gripper)
        for g in groups:
            self._prepare(g)
        for g in groups:
            g.enable()

    def gripper_motor(self, on: bool) -> None:
        """The gripper motor alone on (gainless until commanded) or off (damping)."""
        g = self.arm.gripper
        if on:
            # The arm motors are on (floating) meanwhile and need a frame every tick: no mode switch
            # here (its MIT mode was set and confirmed at the full enable, and switching the motor
            # off does not change it; mode_mit blocks for ~0.3 s, past the arm's 100 ms watchdog).
            g._q_last[:] = np.nan
            g._q_refused[:] = False
            g.clear_error()            # 20 ms settle
            g.enable()
        else:
            g.disable()

    def seed(self, q) -> None:
        """Tell the driver the arm's setpoints start at ``q`` (the pose enable holds), so holding a
        joint that sits just outside its limits is accepted (it refuses setpoints further outside
        than the last one sent) and nothing is ever corrected toward an old one."""
        self.arm.arm._q_last[:] = np.asarray(q, float)

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

    def send(self, q, qd, kp, kd, tau, g_q, g_qd, g_kp, g_kd, g_tau, gripper: bool = True) -> None:
        self.arm.arm.send_mit(q, vel=qd, kp=kp, kd=kd, tau=tau)
        groups = [self.arm.arm]
        if gripper:          # a gripper in damping (motor off) gets no frames
            self.arm.gripper.send_mit(np.array([g_q]), vel=np.array([g_qd]), kp=np.array([g_kp]),
                                      kd=np.array([g_kd]), tau=np.array([g_tau]))
            groups.append(self.arm.gripper)
        refused = [n for g in groups for n, r in zip(g.joint_names, g._q_refused) if r]
        if refused:   # the node keeps its command inside the limits, so this is a bug: stop
            raise BusError(f"the driver refused setpoints for {', '.join(refused)}")

    def close(self) -> None:
        """Release the serial port whatever state the motors are in. (RebotArm.disconnect disables
        all motors first and, if that raises, never closes the port: after a fault the next
        connect then finds it busy.)"""
        import time as _t
        a = self.arm
        try:
            a.stop_control_loop()
        except Exception:  # noqa: BLE001
            pass
        for g in (a.arm, a.gripper):
            try:
                g.disable()
            except Exception:  # noqa: BLE001
                pass
        _t.sleep(0.2)
        # Every motor handle references its controller: the serial port is released only when the
        # handles are freed too (RebotArm never frees them, so its port stays open in the process).
        for m in list(a._motor_map.values()):
            try:
                m.close()
            except Exception:  # noqa: BLE001
                pass
        for ctrl in list(a._ctrl_map.values()):
            for step in (ctrl.shutdown, ctrl.close):
                try:
                    step()
                except Exception:  # noqa: BLE001
                    pass
        a._ctrl_map.clear()
        a._motor_map.clear()
        a._connected = False


class SimBus(ArmBus):
    """A simulated arm: each joint a unit-ish inertia under its MIT law, gravity compensated
    exactly, integrated in 1 ms steps between reads. Disabled joints coast to a stop where they
    are (it does not fall). The gripper closes onto ``object_gap_rad`` if set (an object), else 0.
    """
    sim = True
    INERTIA = np.array([0.08, 0.08, 0.05, 0.01, 0.01, 0.005])
    G_INERTIA = 0.002
    G_OPEN = -3.1

    # the "true" gravity of the simulated arm: the model's, with the hardware's measured scaling
    TRUE_TAU_SCALE = np.array([1.0, 1.0, 1.0, 0.84, 0.84, 1.0])

    def __init__(self, q0=(0.0, -0.3, -0.6, 0.0, 0.0, 0.0), clock=time.monotonic, limits: Optional[dict] = None,
                 gravity: bool = False):
        """``gravity``: simulate gravity (the node's feedforward must cancel it; disabled joints
        fall onto stops at the joint limits). Off by default: the arm floats where it is."""
        self.clock = clock
        self.gravity = gravity
        self._kin = None
        self.g_mot_offset = 0.0
        self.q = np.array(q0, dtype=float)
        self.qd = np.zeros(N)
        self.tau = np.zeros(N)
        self.g_q, self.g_qd, self.g_tau = -0.5, 0.0, 0.0
        self.enabled = False
        self.g_enabled = False
        self.connected = False
        self.cmd = None
        self.gcmd = None
        self.t = None
        self.object_gap_rad: Optional[float] = None   # e.g. -1.0: something between the jaws
        self.mot_offset = np.zeros(N)                # motor side minus output (backlash, a slipped zero)
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
        self.fault_code = 0            # clear_error, as the real enable does
        self.enabled = True
        self.g_enabled = True
        self._q_last = np.full(N, np.nan)

    def gripper_motor(self, on: bool) -> None:
        self.g_enabled = on

    def seed(self, q) -> None:
        self._q_last = np.asarray(q, float).copy()

    def disable(self) -> None:
        self.enabled = False
        self.g_enabled = False
        self.cmd = None
        self.gcmd = None

    def _g(self) -> np.ndarray:
        if not self.gravity:
            return np.zeros(N)
        if self._kin is None:
            from .kin import ArmKinematics
            self._kin = ArmKinematics(self.setpoint_limits)
        return self._kin.gravity(self.q) * self.TRUE_TAU_SCALE

    def _stops(self) -> None:
        if self._kin is not None:      # hard stops at the joint limits
            lo, hi = self._kin.lo, self._kin.hi
            hit = (self.q < lo) | (self.q > hi)
            self.q = np.clip(self.q, lo, hi)
            self.qd[hit] = 0.0

    def _step(self, dt: float) -> None:
        g = self._g()                  # torque the motors must supply to hold still
        c = self.cmd if self.enabled else None
        if c is None:                  # off: damping
            self.tau[:] = 0.0
            if self.gravity:           # sinks (viscous joint friction)
                self.qd += (-g - 0.5 * self.qd) / self.INERTIA * dt
            else:
                self.qd *= math.exp(-dt / 0.05)
        else:
            q, qd, kp, kd, tau_ff = c
            self.tau = kp * (q - self.q) + kd * (qd - self.qd) + (tau_ff if self.gravity else 0.0)
            self.qd += (self.tau - g - 0.1 * self.qd) / self.INERTIA * dt     # + a little joint friction
        self.q += self.qd * dt
        self._stops()
        gc = self.gcmd if (self.enabled and self.g_enabled) else None
        if gc is None:                 # damping: comes to rest
            self.g_tau = 0.0
            self.g_qd *= math.exp(-dt / 0.02)
        else:
            g_q, g_qd, g_kp, g_kd, g_tau = gc
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
        return Reading(now, self.q.copy(), self.q + self.mot_offset, self.qd.copy(), self.tau.copy(), status,
                       np.full(N + 1, 35.0), np.full(N + 1, 38.0), self.g_q, self.g_q + self.g_mot_offset,
                       self.g_qd, self.g_tau)

    def send(self, q, qd, kp, kd, tau, g_q, g_qd, g_kp, g_kd, g_tau, gripper: bool = True) -> None:
        # as the real driver (JointGroup._legal): a setpoint outside the limits is refused unless
        # it is no further outside than the last one sent
        names = [f"joint{i}" for i in range(1, N + 1)]
        lo = np.array([self.setpoint_limits.get(n, (-math.inf, math.inf))[0] for n in names])
        hi = np.array([self.setpoint_limits.get(n, (-math.inf, math.inf))[1] for n in names])
        q = np.asarray(q, float)
        out = np.maximum(lo - q, q - hi)
        last = getattr(self, "_q_last", np.full(N, np.nan))
        last_out = np.maximum(lo - last, last - hi)
        checked = np.asarray(kp, float) != 0.0       # kp = 0 (floating): position ignored, not checked
        ok = ~checked | (out <= 0.0) | (out <= last_out)      # exactly as strict as the driver
        if not np.all(ok):
            raise BusError(f"the driver refused setpoints for {', '.join(n for n, k in zip(names, ok) if not k)}")
        self._q_last = np.where(checked, q, last)
        if self.enabled:
            self.cmd = (np.array(q, float), np.array(qd, float), np.array(kp, float), np.array(kd, float),
                        np.array(tau, float))
            if gripper:
                self.gcmd = (float(g_q), float(g_qd), float(g_kp), float(g_kd), float(g_tau))

    def close(self) -> None:
        self.connected = False
        self.enabled = False
