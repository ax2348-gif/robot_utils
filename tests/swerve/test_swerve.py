"""robot_utils.nodes.swerve: kinematics, setup and the drive loop (simulated).

Ported from rev_ctl tests/test_swerve.py (commit cfcfc32); test_bench_commands stays there
with bench.py.
"""
import math
import sys
import time

import pytest

from robot_utils.nodes.swerve import constants as c
from robot_utils.nodes.swerve import setup_sparks as swerve_setup
from sparkcan import ControlType
from sparkcan._params import SparkParameter as P
from sparkcan.sim import SimulatedSpark
from robot_utils.nodes.swerve.drive import SwerveDrive, wrap_angle


def wait_for(predicate, timeout=1.0):
    """Poll until ``predicate()`` is truthy; return its value or fail."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.005)
    raise AssertionError("condition not met in time")


def test_geometry_and_drive_ratio():
    assert c.HALF_WHEELBASE == pytest.approx(0.1656, abs=1e-4)   # 17 in frame, 1.98 in inset
    assert c.DRIVE_REDUCTION == pytest.approx(5.077, abs=1e-3)   # 13T pinion
    assert c.DRIVE_MPS_PER_RPM == pytest.approx(7.8587e-4, rel=1e-4)
    assert c.DRIVE_KV == pytest.approx(2.25, abs=0.01)


@pytest.fixture
def robot(iface):
    sims = {}
    for m in c.MODULES:
        sims[m.steer_id] = SimulatedSpark(iface, m.steer_id).start()
        sims[m.drive_id] = SimulatedSpark(iface, m.drive_id).start()
    yield sims
    for s in sims.values():
        s.close()


def run_setup(monkeypatch, *args):
    monkeypatch.setattr(sys, "argv", ["setup_sparks", *args])
    swerve_setup.main()


def test_setup_dry_run_writes_nothing(iface, robot, monkeypatch, capsys):
    run_setup(monkeypatch, iface)
    assert "differs" in capsys.readouterr().out
    assert all(not s.frames("PARAMETER_WRITE") for s in robot.values())


def test_setup_apply_saves_steering_and_drive_settings(iface, robot, monkeypatch, capsys):
    run_setup(monkeypatch, iface, "--apply")
    assert capsys.readouterr().out.count("applied and saved") == 8
    steer, drive = robot[c.MODULES[0].steer_id], robot[c.MODULES[0].drive_id]
    assert steer.persisted[int(P.DUTY_CYCLE_INVERTED)] is True
    assert steer.persisted[int(P.CLOSED_LOOP_CONTROL_SENSOR)] == 4   # absolute encoder
    assert steer.persisted[int(P.P_0)] == pytest.approx(c.STEER_P)
    assert steer.persisted[int(P.OUTPUT_MAX_0)] == pytest.approx(c.STEER_MAX_OUTPUT)
    assert drive.persisted[int(P.OUTPUT_MIN_0)] == pytest.approx(-c.DRIVE_MAX_OUTPUT)
    assert steer.persisted[int(P.STATUS_5_PERIOD)] == 20
    assert drive.persisted[int(P.VELOCITY_CONVERSION_FACTOR)] == pytest.approx(c.DRIVE_MPS_PER_RPM)
    assert drive.persisted[int(P.P_0)] == pytest.approx(c.DRIVE_P)
    assert steer.persisted[int(P.SMART_CURRENT_STALL_LIMIT)] == c.STEER_CURRENT_LIMIT
    assert drive.persisted[int(P.SMART_CURRENT_STALL_LIMIT)] == c.DRIVE_CURRENT_LIMIT
    for s in robot.values():
        assert s.persisted[int(P.CURRENT_CHOP)] == pytest.approx(115.0)
        assert s.persisted[int(P.IDLE_MODE)] == 0                    # coast
        assert not s.enabled and not s.setpoint


@pytest.fixture
def configured(iface, robot):
    for label, can_id, config in swerve_setup.targets():
        robot[can_id].params.update({int(p): v for p, v in config.flatten()})  # as if applied
    return robot


def test_drive_commands_all_modules_and_enables_them(iface, configured):
    drive = SwerveDrive(iface)
    try:
        for _ in range(10):
            drive.drive(0.2, 0.0, 0.0)
            time.sleep(0.02)
        wait_for(lambda: all(s.enabled and s.setpoint for s in configured.values()))
        for m in c.MODULES:
            steer, wheel = configured[m.steer_id].setpoint, configured[m.drive_id].setpoint
            assert steer[0] == ControlType.POSITION and 0 <= steer[1] < 1
            assert wheel[0] == ControlType.VELOCITY and abs(wheel[1]) <= 0.2 + 1e-6
    finally:
        drive.close()
    assert drive.error is None


def test_steering_follows_the_measured_heading_not_the_last_goal(iface, configured):
    # The wheels read straight ahead and never turn (headings frozen at 0). The travel
    # direction swings 80 deg left, then 160 deg left. Steering chosen against the previous
    # goal would follow it to 160 deg (more than 90 deg from the wheel) and drive forwards;
    # against the measured heading it must pick -20 deg and drive backwards.
    for m in c.MODULES:
        configured[m.steer_id].overrides[5] = {"DUTY_CYCLE_ENCODER_POSITION": m.steer_offset}
    drive = SwerveDrive(iface)
    try:
        for direction in (math.radians(80), math.radians(160)):
            v = 0.2
            end = time.monotonic() + 0.15
            while time.monotonic() < end:
                drive.drive(v * math.cos(direction), v * math.sin(direction), 0.0)
                time.sleep(0.01)
            for m in c.MODULES:
                target, speed = configured[m.steer_id].setpoint[1], configured[m.drive_id].setpoint[1]
                angle = wrap_angle((target - m.steer_offset) * 2 * math.pi)
                assert abs(angle) <= math.pi / 2 + 1e-6, m.name                # heading is 0
                moving = angle if speed > 0 else angle + math.pi                # signed speed's direction
                assert abs(wrap_angle(moving - direction)) < 1e-6, m.name
                assert abs(speed) == pytest.approx(v * math.cos(angle), abs=1e-6), m.name
    finally:
        drive.close()
    assert drive.error is None


def test_stop_and_command_timeout_disable_every_motor(iface, configured):
    drive = SwerveDrive(iface)
    try:
        drive.drive(0.1, 0.0, 0.0)
        wait_for(lambda: all(s.enabled for s in configured.values()))
        drive.stop()
        wait_for(lambda: not any(s.enabled for s in configured.values()))
        drive.drive(0.1, 0.0, 0.0)   # then nothing: the timeout must disable them
        wait_for(lambda: all(s.enabled for s in configured.values()))
        wait_for(lambda: not any(s.enabled for s in configured.values()), timeout=c.COMMAND_TIMEOUT + 1.0)
    finally:
        drive.close()


def test_steering_setpoint_is_the_goals_encoder_reading(iface, configured):
    # FR (offset 0) wants forward: setpoint 0.0, even when it reads 0.95 (the SPARK wraps 0/1).
    fr = next(m for m in c.MODULES if m.name == "FR")
    configured[fr.steer_id].overrides[5] = {"DUTY_CYCLE_ENCODER_POSITION": 0.95}
    drive = SwerveDrive(iface)
    try:
        drive.drive_wheels(0.0, 0.0)
        wait_for(lambda: configured[fr.steer_id].setpoint is not None)
        assert configured[fr.steer_id].setpoint[1] == pytest.approx(0.0)
        drive.drive_wheels(math.pi / 2, 0.0)   # FL offset 0.75: 90 deg left reads 0.0
        fl = c.MODULES[0]
        wait_for(lambda: (configured[fl.steer_id].setpoint or (None, None))[1] == pytest.approx(0.0))
    finally:
        drive.close()


def test_wheel_without_steering_reading_gets_zero_drive(iface, configured):
    fl = c.MODULES[0]
    drive = SwerveDrive(iface)
    try:
        configured[fl.steer_id].params[int(P.STATUS_5_PERIOD)] = 5000   # its readings go stale
        time.sleep(0.3)
        for _ in range(20):
            drive.drive(0.2, 0.0, 0.0)
            time.sleep(0.02)
        assert configured[fl.drive_id].setpoint[1] == 0.0
        assert configured[c.MODULES[1].drive_id].setpoint[1] > 0
    finally:
        drive.close()


def test_status_frames_configured_for_the_loop(iface, configured):
    from robot_utils.nodes.swerve.drive import STATUS_1_PERIOD_MS, STATUS_PERIOD_MS
    drive = SwerveDrive(iface)
    try:
        for m in c.MODULES:
            d, s = configured[m.drive_id], configured[m.steer_id]
            assert d.params[int(P.STATUS_2_PERIOD)] == STATUS_PERIOD_MS
            assert s.params[int(P.STATUS_5_PERIOD)] == STATUS_PERIOD_MS
            for spark in (d, s):
                assert spark.params[int(P.STATUS_0_PERIOD)] == STATUS_PERIOD_MS
                assert spark.params[int(P.STATUS_1_PERIOD)] == STATUS_1_PERIOD_MS
            assert not d.enabled_statuses >> 5 & 1 and d.enabled_statuses >> 2 & 1
            assert not s.enabled_statuses >> 2 & 1 and s.enabled_statuses >> 5 & 1
    finally:
        drive.close()


def test_tick_frames_go_out_in_one_serial_write():
    import threading
    from types import SimpleNamespace
    from sparkcan.protocol import heartbeat_frame, to_slcan

    writes = []
    transport = SimpleNamespace(_ser=SimpleNamespace(write=writes.append), _lock=threading.Lock())
    drive = object.__new__(SwerveDrive)
    drive.bus = SimpleNamespace(_transport=transport, _tx_lock=threading.Lock())
    frames = [heartbeat_frame([1, 2]), heartbeat_frame(())]
    drive._batch = list(frames)
    drive._flush()
    assert writes == [b"".join(to_slcan(f) for f in frames)]
    assert drive._batch is None


def test_failed_start_unregisters_its_sparks(iface):
    from sparkcan.bus import CANBus
    with pytest.raises(RuntimeError):
        SwerveDrive(iface)                       # no SPARKs on this bus: no steering reading
    assert CANBus.get(iface).device_ids == []


def test_module_without_steering_reading_is_left_disabled(iface, configured):
    fl, fr = c.MODULES[0], c.MODULES[1]
    drive = SwerveDrive(iface)
    try:
        configured[fl.steer_id].params[int(P.STATUS_5_PERIOD)] = 5000   # its readings go stale
        time.sleep(0.3)
        for _ in range(20):
            drive.drive(0.2, 0.0, 0.0)
            time.sleep(0.02)
        assert not configured[fl.steer_id].enabled and not configured[fl.drive_id].enabled
        assert configured[fr.steer_id].enabled and configured[fr.drive_id].enabled
        assert math.isnan(drive.setpoints[0][0]) and drive.setpoints[1][1] > 0
    finally:
        drive.close()
