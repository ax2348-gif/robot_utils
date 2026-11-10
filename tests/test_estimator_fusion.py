"""Estimator fusion core: odometry master + interpolated IMU heading (no I/O)."""
import math

import pytest

from robot_utils.nodes.estimator.fusion import (FIELDS, MASTER_IMU, HeadingTrack, PoseFusion, odometry_sample,
                                                quat_rotate)
from robot_utils.nodes.swerve.constants import MODULES

F = {n: i for i, n in enumerate(FIELDS)}


def swerve_row(t, vx, vy, omega, pos, dt):
    """A swerve telemetry row for a rigid body twist; ``pos`` (per module travel) is advanced in place."""
    row = {"t_tick": t}
    for i, m in enumerate(MODULES):
        wx, wy = vx - omega * m.y, vy + omega * m.x
        speed, ang = math.hypot(wx, wy), math.atan2(wy, wx)
        pos[i] += speed * dt
        row.update({f"{m.name}_drive_pos": pos[i], f"{m.name}_steer_angle": ang,
                    f"{m.name}_drive_vel": speed, f"{m.name}_t_drive": t})
    return row


def yaw_q(psi, tilt=0.0):
    """IMU orientation: yaw psi about world z, after a fixed mounting tilt about x."""
    qz = (math.cos(psi / 2), 0.0, 0.0, math.sin(psi / 2))
    qt = (math.cos(tilt / 2), math.sin(tilt / 2), 0.0, 0.0)
    w1, x1, y1, z1 = qz
    w2, x2, y2, z2 = qt
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def imu_record(t0, t1, psi1, omega, tilt=0.0):
    """Filter record for a constant world yaw rate, IMU mounted with ``tilt``: da in the IMU frame."""
    q = yaw_q(psi1, tilt)
    # world rotation omega*(t1-t0) about z, expressed in the IMU frame: R(q)^T e_z * angle
    qc = (q[0], -q[1], -q[2], -q[3])
    ez_body = quat_rotate(qc, (0.0, 0.0, 1.0))
    a = omega * (t1 - t0)
    return t0, t1, q, tuple(c * a for c in ez_body)


def run(fusion, duration, vx, vy, omega_true, omega_wheel=None, imu_offset=0.0, tilt=0.0, imu=True):
    """Drive for ``duration`` s: odometry at 100 Hz, IMU records at 100 Hz shifted by imu_offset."""
    omega_wheel = omega_true if omega_wheel is None else omega_wheel
    pos = [0.0] * 4
    rows, psi = [], 0.0
    n = int(duration * 100)
    if imu:   # IMU first, covering the whole span (the fusion waits for coverage in the node)
        t = imu_offset
        while t < duration + 0.05:
            psi += omega_true * 0.01
            fusion.heading.add(*imu_record(t, t + 0.01, psi, omega_true, tilt))
            t += 0.01
    for k in range(n + 1):
        t = k * 0.01
        r = fusion.odom(odometry_sample(swerve_row(t, vx, vy, omega_wheel, pos, 0.01 if k else 0.0)),
                        fusion.heading.end)
        if r is not None:
            rows.append(r)
    return rows


def test_straight_line():
    f = PoseFusion()
    rows = run(f, 2.0, 0.5, 0.0, 0.0)
    last = rows[-1]
    assert last[F["x"]] == pytest.approx(1.0, abs=1e-6) and abs(last[F["y"]]) < 1e-9
    assert last[F["imu_ok"]] == 1.0 and last[F["vx"]] == pytest.approx(0.5)
    assert last[F["distance"]] == pytest.approx(1.0, abs=1e-6)


def test_arc_uses_imu_heading_and_odometry_matches_when_no_slip():
    f = PoseFusion()
    rows = run(f, 2.0, 0.4, 0.0, 0.5)
    th = rows[-1][F["theta"]]
    assert th == pytest.approx(1.0, abs=1e-3)
    r = 0.4 / 0.5     # circle radius; after angle th: x = r sin th, y = r (1 - cos th)
    assert rows[-1][F["x"]] == pytest.approx(r * math.sin(th), abs=2e-3)
    assert rows[-1][F["y"]] == pytest.approx(r * (1 - math.cos(th)), abs=2e-3)
    assert rows[-1][F["x_odom"]] == pytest.approx(rows[-1][F["x"]], abs=1e-3)


def test_wheel_slip_heading_follows_imu():
    f = PoseFusion()
    rows = run(f, 2.0, 0.0, 0.0, 0.5, omega_wheel=0.8)   # wheels over-report the turn
    assert rows[-1][F["theta"]] == pytest.approx(1.0, abs=1e-3)
    assert rows[-1][F["theta_odom"]] == pytest.approx(1.6, abs=1e-3)
    assert rows[-1][F["omega_wheel"]] - rows[-1][F["omega_imu"]] == pytest.approx(0.3, abs=1e-3)


@pytest.mark.parametrize("offset", [0.003, 0.0049, -0.004])
def test_misaligned_streams_are_interpolated(offset):
    """IMU records start between odometry ticks: interpolation recovers the heading exactly
    for a constant rate (ticks at master times, not IMU times)."""
    f = PoseFusion()
    rows = run(f, 1.0, 0.0, 0.0, 0.7, imu_offset=offset)
    assert all(r[F["imu_ok"]] == 1.0 for r in rows[1:])
    assert [r[F["t"]] for r in rows[:3]] == pytest.approx([0.01, 0.02, 0.03])   # master timestamps
    assert rows[-1][F["theta"]] == pytest.approx(0.7, abs=1e-6)


def test_heading_independent_of_camera_mounting():
    for tilt in (0.0, -math.pi / 2, 0.3):          # upright, facing forward (y down), tilted
        f = PoseFusion()
        rows = run(f, 1.0, 0.0, 0.0, 0.6, tilt=tilt)
        assert rows[-1][F["theta"]] == pytest.approx(0.6, abs=1e-6), tilt


def test_falls_back_to_wheels_without_imu():
    f = PoseFusion()
    rows = run(f, 1.0, 0.0, 0.0, 0.5, imu=False)
    assert all(r[F["imu_ok"]] == 0.0 for r in rows) and rows[-1][F["theta"]] == pytest.approx(0.5, abs=1e-6)
    assert math.isnan(rows[-1][F["omega_imu"]])


def test_imu_only_mode_and_reset():
    f = PoseFusion()
    for k in range(101):
        t = k * 0.01
        f.heading.add(*imu_record(t, t + 0.01, 0.3 * (t + 0.01), 0.3))
        r = f.imu_only(t + 0.01)
    assert r[F["master"]] == MASTER_IMU and r[F["theta"]] == pytest.approx(0.3, abs=1e-6)
    f.reset(1.0, 2.0, 0.5)
    assert (f.x, f.y, f.theta, f.thetao) == (1.0, 2.0, 0.5, 0.5)


def test_heading_track_gap_is_not_bridged():
    h = HeadingTrack()
    h.add(0.0, 0.01, yaw_q(0.0), (0.0, 0.0, 0.0))
    h.add(0.5, 0.51, yaw_q(0.0), (0.0, 0.0, 0.1))      # 0.49 s gap
    assert not h.covers(0.005, 0.505) and h.covers(0.5, 0.51)


def test_initializing_records_are_ignored():
    h = HeadingTrack()
    assert h.add(0.0, 0.01, yaw_q(0.0), (0, 0, 0.1), flags=4) is None and h.end is None


def drive_rows(segments, epoch_of=None, pos=None):
    """Swerve rows for (seconds, vx) segments at 100 Hz, wheels straight ahead. ``epoch_of(k, pos)``
    may return (enc_epoch, enc_rebasing) and edit ``pos`` in place (a rebase); else no epoch columns."""
    pos = [0.0] * 4 if pos is None else pos
    out, k = [], 0
    for seconds, vx in segments:
        for _ in range(int(round(seconds * 100))):
            row = swerve_row(k * 0.01, vx, 0.0, 0.0, pos, 0.01 if k else 0.0)
            if epoch_of is not None:
                ep = epoch_of(k, pos)
                if ep is not None:
                    for i, m in enumerate(MODULES):
                        row[f"{m.name}_drive_pos"] = pos[i]
                    row["enc_epoch"], row["enc_rebasing"] = ep
            out.append(row)
            k += 1
    return out


def fuse(f, rows):
    out = [r for r in (f.odom(odometry_sample(row)) for row in rows) if r is not None]
    xs = [r[F["x"]] for r in out]
    return out, max(abs(b - a) for a, b in zip(xs, xs[1:]))


def test_rebase_is_invisible_in_the_pose():
    """Drive 1 m, stop, the swerve node rebases (3 ticks in flight, then epoch 1 with travel 0), drive 1 m."""
    def epoch_of(k, pos):
        if k < 250:
            return (0.0, 0.0)
        if k < 253:                    # in flight (while stopped): the SPARKs apply it in here
            if k == 251:
                pos[:] = [0.0] * 4
            return (0.0, 1.0)
        return (1.0, 0.0)
    f = PoseFusion()
    rows, step = fuse(f, drive_rows([(2.0, 0.5), (1.0, 0.0), (2.0, 0.5)], epoch_of))
    assert rows[-1][F["x"]] == pytest.approx(2.0 - 0.005, abs=1e-6)     # the first tick only references
    assert step < 0.0051 and f.rebases_seen == 1 and f.discontinuities == 0
    assert rows[-1][F["discontinuities"]] == 0


def test_unflagged_jump_is_skipped_not_integrated():
    """A SPARK brownout: one wheel's travel drops to 0 while driving; no epoch column (old recordings)."""
    pos = [0.0] * 4
    rows = drive_rows([(1.0, 0.5)], pos=pos)
    rows[60][f"{MODULES[2].name}_drive_pos"] = 0.0
    for r in rows[61:]:                              # that wheel counts on from 0
        r[f"{MODULES[2].name}_drive_pos"] -= rows[59][f"{MODULES[2].name}_drive_pos"] + 0.005
    f = PoseFusion()
    out, step = fuse(f, rows)
    assert step < 0.0051                             # no false motion
    assert out[-1][F["x"]] == pytest.approx(0.5 - 0.005 - 0.005, abs=1e-6)   # one tick of motion lost
    assert f.discontinuities == 1 and out[-1][F["discontinuities"]] == 1
    assert "jumped" in f.last_discontinuity["reason"] and f.last_discontinuity["t"] == pytest.approx(0.6)


def test_large_but_plausible_steps_are_integrated():
    f = PoseFusion()
    out, _ = fuse(f, drive_rows([(1.0, 4.0)]))       # 4 m/s: 4 cm per tick, below the jump limit
    assert f.discontinuities == 0 and out[-1][F["x"]] == pytest.approx(4.0 - 0.04, abs=1e-6)
