"""Description node routines with simulated camera/swerve I/O (real time, no hardware)."""
import contextlib
import math
import threading
import time

import numpy as np
import pytest

from robot_utils.nodes.description.frames import rpy_to_matrix
from robot_utils.nodes.description.routines import (Aborted, GroundParams, YawParams, run_ground, run_yaw,
                                                    validate_yaw)
from robot_utils.nodes.description.solvers import angle_deg
from robot_utils.nodes.swerve.constants import MODULES

OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
R_TRUE = rpy_to_matrix(math.radians(1.5), math.radians(20), math.radians(-7)) @ OPT     # base <- depth_optical
HEIGHT = 0.55
K = {"fx": 415.5, "fy": 415.5, "cx": 423.3, "cy": 238.7, "width": 848, "height": 480}


class FakeSwerve:
    """Base whose velocity follows the command with a first-order lag; 100 Hz ticks."""

    def __init__(self, lag=0.08):
        self.lag, self.cmd, self.v, self.acc = lag, np.zeros(2), np.zeros(2), np.zeros(2)
        self.lock = threading.Lock()
        self.controlled = False
        self.estops, self.commands = [], 0
        self.stop = threading.Event()
        self.ticks = []
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        t_prev = time.monotonic()
        while not self.stop.is_set():
            time.sleep(0.01)
            t = time.monotonic()
            dt = t - t_prev
            t_prev = t
            with self.lock:
                cmd = self.cmd if self.controlled else np.zeros(2)
                a = (cmd - self.v) / self.lag
                self.v = self.v + a * dt
                self.acc = a
                ang = math.atan2(self.v[1], self.v[0]) if np.linalg.norm(self.v) > 1e-4 else 0.0
                spd = float(np.linalg.norm(self.v))
                row = {"t_tick": t}
                for m in MODULES:
                    row[f"{m.name}_steer_angle"], row[f"{m.name}_drive_vel"] = ang, spd
                self.ticks.append(row)

    def state(self):
        return {"state": "running", "estop": {"latched": bool(self.estops)}, "lease": {"held": False}}

    @contextlib.contextmanager
    def control(self, who):
        self.controlled = True
        try:
            yield self
        finally:
            with self.lock:
                self.cmd = np.zeros(2)
            self.controlled = False

    def command(self, vx, vy):
        with self.lock:
            self.cmd = np.array([vx, vy])
        self.commands += 1

    def estop(self, reason):
        self.estops.append(reason)

    def tick_rows(self, stop):
        i = 0
        while not stop.is_set():
            time.sleep(0.02)
            with self.lock:
                new, i = self.ticks[i:], len(self.ticks)
            if new:
                names = list(new[0])
                yield np.rec.fromarrays([np.array([r[n] for r in new]) for n in names], names=names), 0.0


class FakeCamera:
    """IMU (= depth frame) seeing the fake base's acceleration through R_TRUE, plus gravity, noise, gyro bias."""

    def __init__(self, base: FakeSwerve, yaw_rate_dps: float = 0.0, noise=0.02, spikes_dps: float = 0.0):
        self.base, self.yaw_rate, self.noise, self.spikes = base, yaw_rate_dps, noise, spikes_dps
        self.rng = np.random.default_rng(0)

    def description(self):
        return {"device": {"serial": "FAKE0001"}, "aligned_depth": False, "intrinsics": {"depth": K},
                "frames": {"depth_optical": {"parent": None},
                           "imu": {"parent": "depth_optical", "rotation": np.eye(3).tolist(), "translation": [0.01, 0, 0]}}}

    def imu_rows(self, stop):
        up_cam = R_TRUE.T @ np.array([0, 0, 1.0])
        bias = np.array([0.001, -0.0015, 0.006])
        while not stop.is_set():
            time.sleep(0.01)
            t = time.monotonic() - np.arange(10)[::-1] * 0.001
            with self.base.lock:
                a_base = np.array([*self.base.acc, 0.0])
            f = R_TRUE.T @ a_base + 9.81 * up_cam
            A = f + self.rng.normal(0, self.noise, (10, 3))
            turning = self.yaw_rate if self.base.controlled else 0.0     # starts once the base moves
            G = bias + np.radians(turning) * up_cam + self.rng.normal(0, 0.001, (10, 3))
            if self.spikes and self.base.controlled:      # vibration: single-sample spikes, no net rotation
                G[::5] += np.radians(self.spikes) * up_cam * self.rng.choice([-1, 1], (2, 1))
            yield np.rec.fromarrays([t, A[:, 0], A[:, 1], A[:, 2], G[:, 0], G[:, 1], G[:, 2]],
                                    names=["t_sensor", "ax", "ay", "az", "gx", "gy", "gz"]), 0.0

    def depth_frames(self, n):
        from test_description_solvers import floor_depth
        from robot_utils.nodes.description.frames import make_T
        return [(floor_depth(make_T(R_TRUE, [0, 0, HEIGHT]), K, noise=0.003, seed=i).astype(np.float32),
                 {"seq": i}) for i in range(n)]

    def gravity_up(self):
        return R_TRUE.T @ np.array([0, 0, 1.0]) * 9.81


@pytest.fixture
def rig():
    base = FakeSwerve()
    yield base, FakeCamera(base)
    base.stop.set()


def test_validation_refuses_unsafe_requests():
    for bad, msg in [(dict(confirm=False), "confirm"), (dict(amplitude=0.3, confirm=True), "amplitude"),
                     (dict(amplitude=0.15, frequency=2.0, confirm=True), "peak speed"),
                     (dict(vectors=[[0, 0]], confirm=True), "bad vector"), (dict(cycles=1, confirm=True), "cycles")]:
        with pytest.raises(ValueError, match=msg):
            validate_yaw(YawParams(**bad))
    assert validate_yaw(YawParams(vectors=[[2, 0], [0, -3]], confirm=True)) == [[1.0, 0.0], [0.0, -1.0]]


def test_yaw_recovers_directions_and_floor_normal(rig):
    base, cam = rig
    p = YawParams(vectors=[[1, 0], [0, 1]], amplitude=0.06, frequency=1.5, cycles=3, settle_s=0.4, confirm=True)
    meta, raw = run_yaw(cam, base, p)
    for v in meta["vectors"]:
        truth = R_TRUE.T @ np.array(v["vector"] + [0.0])
        assert angle_deg(v["direction_depth"], truth) < 1.0, v
        assert v["sign_corr"] > 0.5 and v["linearity"] < 0.3
    assert abs(meta["checks"]["angle_0_1"]["error_deg"]) < 1.5
    assert angle_deg(meta["floor_normal_depth"], R_TRUE.T @ np.array([0, 0, 1.0])) < 1.5
    assert base.commands > 100 and not base.estops and not base.controlled
    assert "imu_ax" in raw and "swerve_base_vel" in raw


def test_yaw_aborts_and_estops_on_rotation(rig):
    base, _ = rig
    cam = FakeCamera(base, yaw_rate_dps=25.0)                 # the base is turning (e.g. tether pull)
    with pytest.raises(Aborted, match="yaw rate") as e:
        run_yaw(cam, base, YawParams(amplitude=0.05, frequency=1.0, cycles=2, confirm=True))
    assert base.estops and not base.controlled
    raw = e.value.raw                                         # the recording, for diagnosis
    assert len(raw["imu_gz"]) > 100 and "swerve_t_tick" in raw and any("aborted" in m for m in raw["marks"])


def test_yaw_ignores_gyro_spikes_without_rotation(rig):
    base, _ = rig
    cam = FakeCamera(base, spikes_dps=14.0)                   # like the robot: raw samples > 10 °/s
    meta, _ = run_yaw(cam, base, YawParams(vectors=[[1, 0]], amplitude=0.05, frequency=1.0, cycles=2,
                                           settle_s=0.3, confirm=True))
    assert not base.estops and len(meta["vectors"]) == 1


def test_yaw_aborts_on_slow_heading_drift(rig):
    base, _ = rig
    cam = FakeCamera(base, yaw_rate_dps=6.0)                  # under the rate limit; the heading limit catches it
    with pytest.raises(Aborted, match="heading drifted"):
        run_yaw(cam, base, YawParams(amplitude=0.05, frequency=1.0, cycles=5, confirm=True))
    assert base.estops and not base.controlled


def test_yaw_user_abort_stops_without_estop(rig):
    base, cam = rig
    t0 = time.monotonic()
    with pytest.raises(Aborted, match="user"):
        run_yaw(cam, base, YawParams(amplitude=0.05, frequency=1.0, cycles=5, confirm=True),
                should_abort=lambda: "user" if time.monotonic() - t0 > 1.5 else None)
    assert not base.estops and not base.controlled and time.monotonic() - t0 < 3.0


def test_refuses_to_start_when_not_still(rig):
    base, _ = rig
    cam = FakeCamera(base, yaw_rate_dps=5.0)
    cam.yaw_rate = 5.0
    base.controlled = True        # the fake rotates whenever controlled: rotating during the still window
    try:
        with pytest.raises(RuntimeError, match="not still"):
            run_yaw(cam, base, YawParams(amplitude=0.05, frequency=1.0, cycles=2, confirm=True))
    finally:
        base.controlled = False


def test_ground_fit_from_depth_frames(rig):
    _, cam = rig
    meta, raw = run_ground(cam, GroundParams(frames=3))
    assert meta["result"]["height"] == pytest.approx(HEIGHT, abs=0.003)
    assert angle_deg(meta["result"]["normal"], R_TRUE.T @ np.array([0, 0, 1.0])) < 0.3
    assert meta["result"]["angle_to_gravity_deg"] < 0.3 and raw["depth_mm"].dtype == np.uint16
    from robot_utils.nodes.description.node import ground_quality
    assert ground_quality(meta) == []
    poly = run_ground(cam, GroundParams(frames=1, polygons=[[[0, 0.6], [1, 0.6], [1, 1], [0, 1]]]))[0]
    assert poly["result"]["height"] == pytest.approx(HEIGHT, abs=0.005) and "mask" in run_ground(
        cam, GroundParams(frames=1, polygons=[[[0, 0.6], [1, 0.6], [1, 1], [0, 1]]]))[1]
