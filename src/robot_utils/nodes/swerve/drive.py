#!/usr/bin/env python3
"""Twist (vx, vy, omega) control of the MAXSwerve drive base.

    from robot_utils.nodes.swerve.drive import SwerveDrive

    drive = SwerveDrive("slcan:/dev/ttyACM0")
    drive.drive(0.2, 0.0, 0.0)   # robot frame: m/s forward, m/s left, rad/s counter-clockwise
    ...                          # call drive() again at least every COMMAND_TIMEOUT seconds
    drive.close()

A background loop sends the setpoints and the enable heartbeat every 10 ms (a fixed-rate
schedule; each tick's frames go out in one serial write on slcan). If drive() is
not called for COMMAND_TIMEOUT seconds (or stop() is called), the loop sends one heartbeat
that enables no SPARKs and then stays silent, so every motor stops and other tools can use
the bus. Configure the SPARKs with ``python -m robot_utils.nodes.swerve.setup_sparks`` first:
the drive setpoint is in m/s and the steering setpoint in encoder rotations.

Command-line test (wheels off the ground):

    python -m robot_utils.nodes.swerve.drive slcan:/dev/ttyACM0 0.1 0 0 --seconds 2

Ported from rev_ctl ``examples/swerve_drive.py`` (commit cfcfc32); the kinematics functions
moved to ``kinematics.py`` and are re-exported here.
"""
from __future__ import annotations

import argparse
import math
import threading
import time

from sparkcan import ControlType, PeriodicFrame, SparkFlex
from sparkcan.protocol import to_slcan

from .constants import COMMAND_TIMEOUT, MAX_WHEEL_SPEED, MODULES
from .kinematics import (MIN_SPEED, module_command, robot_twist,  # noqa: F401 (re-exported)
                         wheel_velocities, wrap_angle)

LOOP_PERIOD = 0.01   # s; the heartbeat must come at least every 20 ms
ANGLE_MAX_AGE = 0.1  # s; an older steering reading is stale
# Status frames, set at runtime by SwerveDrive (not persisted; setup_sparks stores 20 ms):
# STATUS_0 and the one signal frame each SPARK needs come every loop tick (drives: STATUS_2,
# position and velocity; steers: STATUS_5, absolute angle). The other signal frame is turned
# off and STATUS_1 (faults/warnings) slowed down, which keeps the bus near 40% at 100 Hz and
# halves the frames the receive thread has to parse (measured on the robot: 2,990 -> 1,750/s).
STATUS_PERIOD_MS = 10
STATUS_1_PERIOD_MS = 250


class SwerveDrive:
    def __init__(self, interface: str):
        self.drives = [SparkFlex(interface, m.drive_id) for m in MODULES]
        self.steers = [SparkFlex(interface, m.steer_id) for m in MODULES]
        self.bus = self.drives[0].bus
        self._configure_status_frames()
        for i in range(len(MODULES)):
            self._wait_for_reading(i)
        # Frames the loop thread sends during a tick are collected and written at once: on the
        # CANable each serial write costs about 0.6 ms (one USB transfer), so 9 separate writes
        # took most of the 10 ms tick (measured: tick body 6 ms -> 1 ms).
        self._batch = None
        self.bus.send = self._send
        self.error = None   # set if the control loop stopped on an exception
        self._lock = threading.Lock()
        self._command = ("twist", (0.0, 0.0, 0.0))
        self._command_time = -math.inf
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="swerve", daemon=True)
        self._thread.start()

    def drive(self, vx: float, vy: float, omega: float) -> None:
        """Command a robot twist; it holds for COMMAND_TIMEOUT seconds."""
        self._set_command("twist", vx, vy, omega)

    def drive_wheels(self, angle: float, speed: float = 0.0) -> None:
        """Point every wheel at ``angle`` (rad, 0 = forward) and drive it at ``speed`` (m/s).

        For bench tests. Unlike drive(), a wheel never reverses instead of turning, so it ends
        up exactly at ``angle``. Holds for COMMAND_TIMEOUT seconds.
        """
        self._set_command("wheels", angle, max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, speed)))

    def _set_command(self, kind: str, *values: float) -> None:
        if not all(math.isfinite(v) for v in values):
            raise ValueError("command values must be finite")
        with self._lock:
            self._command = (kind, values)
            self._command_time = time.monotonic()

    def stop(self) -> None:
        """Disable every motor now (the next drive() enables them again)."""
        with self._lock:
            self._command_time = -math.inf

    def close(self) -> None:
        self._running = False
        self._thread.join()
        if self.bus.__dict__.get("send") == self._send:
            del self.bus.send   # back to CANBus.send
        self.bus.disable()
        for spark in self.drives + self.steers:
            spark.close()

    def _configure_status_frames(self) -> None:
        for sparks, keep, off in ((self.drives, PeriodicFrame.STATUS_2, PeriodicFrame.STATUS_5),
                                  (self.steers, PeriodicFrame.STATUS_5, PeriodicFrame.STATUS_2)):
            for s in sparks:
                s.set_periodic_frames_enabled({PeriodicFrame.STATUS_0: True, keep: True, off: False})
                s.set_periodic_frame_period(PeriodicFrame.STATUS_0, STATUS_PERIOD_MS)
                s.set_periodic_frame_period(keep, STATUS_PERIOD_MS)
                s.set_periodic_frame_period(PeriodicFrame.STATUS_1, STATUS_1_PERIOD_MS)

    def _send(self, frame, **kw) -> None:
        """CANBus.send for this bus: frames from the loop thread are batched during a tick."""
        if self._batch is not None and threading.current_thread() is self._thread:
            self._batch.append(frame)
        else:
            type(self.bus).send(self.bus, frame, **kw)

    def _flush(self) -> None:
        frames, self._batch = self._batch, None
        if not frames:
            return
        transport = getattr(self.bus, "_transport", None)
        ser, lock = getattr(transport, "_ser", None), getattr(transport, "_lock", None)
        if ser is None or lock is None:   # not slcan (SocketCAN, simulator): one at a time
            for f in frames:
                type(self.bus).send(self.bus, f)
            return
        with self.bus._tx_lock, lock:
            ser.write(b"".join(to_slcan(f) for f in frames))

    def encoder_reading(self, i: int):
        """Steering encoder position (rotations, 0-1), or None if there is no fresh reading."""
        status = self.steers[i].get_periodic_status(PeriodicFrame.STATUS_5)  # also enables it
        if status.timestamp is None or time.monotonic() - status.timestamp > ANGLE_MAX_AGE:
            return None
        return status["DUTY_CYCLE_ENCODER_POSITION"]

    def measured_twist(self):
        """Achieved robot twist (vx, vy, omega) from the wheels' measured headings and speeds,
        or None if a steering reading is missing."""
        velocities = []
        for i in range(len(MODULES)):
            reading = self.encoder_reading(i)
            if reading is None:
                return None
            heading = self.wheel_angle(reading, i)
            speed = self.drives[i].get_encoder().get_velocity()   # m/s (setup_sparks)
            velocities.append((speed * math.cos(heading), speed * math.sin(heading)))
        return robot_twist(velocities)

    def wheel_angle(self, reading: float, i: int) -> float:
        """Wheel angle (rad, 0 = forward) for a steering encoder reading."""
        return wrap_angle((reading - MODULES[i].steer_offset) * 2 * math.pi)

    def _wait_for_reading(self, i: int, timeout: float = 2.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.encoder_reading(i) is not None:
                return
            time.sleep(0.02)
        raise RuntimeError(f"{MODULES[i].name}: no steering encoder reading (run python -m robot_utils.nodes.swerve.setup_sparks?)")

    def _loop(self) -> None:
        enabled_ids = [s.get_device_id() for s in self.drives + self.steers]
        enabled = False
        deadline = time.monotonic()
        try:
            while self._running:
                self._batch = []
                with self._lock:
                    command, age = self._command, time.monotonic() - self._command_time
                if age > COMMAND_TIMEOUT:
                    if enabled:  # one disabling heartbeat, then silence: other tools may use the bus
                        self.bus.disable()
                        enabled = False
                else:
                    enabled = True
                    kind, values = command
                    if kind == "twist":
                        velocities = wheel_velocities(*values)
                    for i, m in enumerate(MODULES):
                        drive = self.drives[i].get_closed_loop_controller()
                        reading = self.encoder_reading(i)
                        if reading is None:  # no fresh steering reading: stop this wheel
                            drive.set_setpoint(0.0, ControlType.VELOCITY)
                            continue
                        heading = self.wheel_angle(reading, i)
                        if kind == "twist":
                            angle, speed = module_command(*velocities[i], heading)
                        else:  # "wheels": exactly this angle (bench tests)
                            angle, speed = values[0], values[1] * max(0.0, math.cos(values[0] - heading))
                        # Position PID on the absolute encoder, which wraps at 0/1 (setup_sparks),
                        # so the setpoint is simply the angle's encoder reading.
                        target = (angle / (2 * math.pi) + m.steer_offset) % 1.0
                        self.steers[i].get_closed_loop_controller().set_setpoint(target, ControlType.POSITION)
                        drive.set_setpoint(speed, ControlType.VELOCITY)
                    self.bus.heartbeat(enabled_ids)
                self._flush()
                # Fixed-rate schedule: sleeping LOOP_PERIOD after the work made the period
                # work + 10 ms (about 60 Hz on the robot). After an overrun, start again from
                # now rather than sending a burst to catch up.
                deadline += LOOP_PERIOD
                now = time.monotonic()
                if deadline < now:
                    deadline = now
                time.sleep(deadline - now)
        except Exception as e:  # never leave the motors enabled after a crash
            self._batch = None
            self.error = e
            self.bus.disable()
            raise


def main() -> None:
    ap = argparse.ArgumentParser(description="Drive the swerve base with a constant twist")
    ap.add_argument("interface", help='e.g. "slcan:/dev/ttyACM0"')
    ap.add_argument("vx", type=float, help="m/s forward")
    ap.add_argument("vy", type=float, help="m/s left")
    ap.add_argument("omega", type=float, help="rad/s counter-clockwise")
    ap.add_argument("--seconds", type=float, default=2.0)
    args = ap.parse_args()

    if input("Wheels off the ground and clear? Type GO: ").strip() != "GO":
        return
    drive = SwerveDrive(args.interface)
    try:
        end = time.monotonic() + args.seconds
        while time.monotonic() < end and drive.error is None:
            drive.drive(args.vx, args.vy, args.omega)
            time.sleep(0.05)
    finally:
        drive.close()
    if drive.error:
        raise SystemExit(f"control loop failed: {drive.error}")


if __name__ == "__main__":
    main()
