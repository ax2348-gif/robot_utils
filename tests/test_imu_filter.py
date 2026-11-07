"""Orbbec node attitude filter (imu_filter): calibration, drift, gating, delta angle, ZARU."""
import math

import numpy as np
import pytest

from robot_utils.nodes.orbbec.imu_filter import (FLAG_CALIBRATED, FLAG_INITIALIZING, FLAG_STILL, STATE_FIELDS,
                                                 AttitudeFilter, Calibration, FilterConfig, euler_zyx, qexp, qlog,
                                                 qmul)

F = {n: i for i, n in enumerate(STATE_FIELDS)}
HZ = 1000.0


def rows(n, gyro=(0.0, 0.0, 0.0), accel=(0.0, 0.0, 9.81), t0=0.0, noise=0.0, seed=0):
    """Raw IMU rows (protocol.IMU_FIELDS) at 1 kHz with constant gyro/accel (+ noise)."""
    rng = np.random.default_rng(seed)
    t = t0 + np.arange(n) / HZ
    r = np.zeros((n, 11))
    r[:, 0] = np.arange(n) + 1
    r[:, 1] = t + 0.004
    r[:, 2] = t
    r[:, 3] = t * 1e6
    r[:, 4:7] = np.asarray(accel) + rng.normal(0, noise * 10, (n, 3))
    r[:, 7:10] = np.asarray(gyro) + rng.normal(0, noise, (n, 3))
    r[:, 10] = 35.0
    return r


def run(f, r):
    out = []
    for i in range(0, len(r), 10):
        out += f.feed(r[i:i + 10].copy())
    return np.array(out)


def test_startup_calibration_bias_gravity_and_no_drift():
    bias = (0.001, -0.0015, 0.006)
    f = AttitudeFilter(FilterConfig())
    cals = []
    f.on_calibration = cals.append
    out = run(f, rows(30_000, gyro=bias, accel=(0.0, 0.5, 10.1), noise=0.001))
    assert len(cals) == 1 and cals[0].source == "startup"
    assert np.allclose(cals[0].gyro_bias, bias, atol=1e-4)
    assert cals[0].g_ref == pytest.approx(math.hypot(0.5, 10.1), abs=0.01)
    assert int(out[0, F["flags"]]) & FLAG_INITIALIZING and not int(out[-1, F["flags"]]) & FLAG_INITIALIZING
    assert int(out[-1, F["flags"]]) & FLAG_CALIBRATED and int(out[-1, F["flags"]]) & FLAG_STILL
    yaw = [euler_zyx(tuple(r[F["qw"]:F["qz"] + 1]))[2] for r in out[200:]]
    assert abs(yaw[-1] - yaw[0]) < 0.05            # degrees over ~28 s (raw bias would give ~10°)
    roll = euler_zyx(tuple(out[-1, F["qw"]:F["qz"] + 1]))[0]
    assert roll == pytest.approx(math.degrees(math.atan2(0.5, 10.1)), abs=0.05)


def test_output_rate_and_window_contents():
    f = AttitudeFilter(FilterConfig(output_rate_hz=100.0), Calibration(source="stored"))
    out = run(f, rows(5_000))
    assert len(out) == pytest.approx(500, abs=2)
    assert set(out[5:, F["n"]]) == {10.0}
    assert np.allclose(np.diff(out[5:, F["t_end"]]), 0.01, atol=1e-6)
    assert np.allclose(out[5:, F["dvz"]], 9.81 * 0.01, atol=1e-6)    # delta velocity incl. gravity


def test_delta_angle_composes_samples_exactly():
    rate = (0.0, 0.0, 2.0)          # 2 rad/s about z: no gravity error, no correction needed
    f = AttitudeFilter(FilterConfig(zaru=False), Calibration(source="stored"))
    refused = []
    f.on_calibration_failed = refused.append
    out = run(f, rows(3_000, gyro=rate))
    assert refused                                     # a steady 2 rad/s is not a bias
    assert np.allclose(out[5:, F["daz"]], 2.0 * 0.01, atol=1e-9)
    yaw = [math.radians(euler_zyx(tuple(r[F["qw"]:F["qz"] + 1]))[2]) for r in (out[-101], out[-1])]
    assert math.remainder(yaw[1] - yaw[0] - 2.0 * 1.0, 2 * math.pi) == pytest.approx(0.0, abs=1e-3)


def test_calibration_refused_while_moving():
    f = AttitudeFilter(FilterConfig())
    fails = []
    f.on_calibration_failed = fails.append
    t = np.arange(3000) / HZ
    r = rows(3000)
    r[:, 7] = 0.5 * np.sin(2 * math.pi * 2 * t)   # wobbling
    run(f, r)
    assert fails and "moved" in fails[0] and not f.calibrated and not f.initializing


def test_accel_gate_ignores_linear_acceleration():
    f = AttitudeFilter(FilterConfig(accel_gate=0.5), Calibration(g_ref=9.81, source="stored"))
    run(f, rows(1_500))                                   # startup calibration: g_ref = 9.81
    assert f.cal.g_ref == pytest.approx(9.81)
    out = run(f, rows(1_000, accel=(3.0, 0.0, 9.81), t0=1.5))   # |a| = 10.26: 0.45 over -> used
    assert out[-1, F["accel_used"]] == 1.0
    out = run(f, rows(1_000, accel=(4.0, 0.0, 9.81), t0=2.5))   # |a| = 10.59: rejected
    assert out[-1, F["accel_used"]] == 0.0


def test_zaru_learns_a_new_bias_while_still():
    f = AttitudeFilter(FilterConfig(zaru_tau_s=1.0), Calibration(source="stored"))
    run(f, rows(10_000, gyro=(0.0, 0.0, 0.004)))          # bias appears after calibration
    assert f.bias[2] == pytest.approx(0.004, abs=2e-4)


def test_request_calibration_updates_bias():
    f = AttitudeFilter(FilterConfig(), Calibration(source="stored"))
    cals = []
    f.on_calibration = cals.append
    run(f, rows(500))
    f.request_calibration(1.0)
    run(f, rows(1500, gyro=(0.002, 0.0, 0.0), t0=0.5))
    assert cals and cals[-1].source == "request" and cals[-1].gyro_bias[0] == pytest.approx(0.002, abs=1e-6)


def test_quaternion_helpers_roundtrip():
    v = (0.3, -0.2, 0.9)
    assert np.allclose(qlog(qexp(*v)), v)
    q = qmul(qexp(0.1, 0, 0), qexp(-0.1, 0, 0))
    assert np.allclose(q, (1, 0, 0, 0))
