"""Estimator pose history (ring buffer, time queries, reset) and the SE(2) relative transform. No I/O."""
import math
import random
import threading

import numpy as np
import pytest

from robot_utils.nodes.estimator.fusion import FIELDS
from robot_utils.nodes.estimator.history import FUTURE_TOL, HistoryError, PoseHistory, relative

F = {n: i for i, n in enumerate(FIELDS)}


def rows(t0, n, reset_id=0, dt=0.01, f=None):
    """n pose rows from t0; every value a known function of t (linear, so interpolation is exact)."""
    f = f or (lambda t: (2 * t + 1, -t, 0.5 * t, 0.1 * t, 3.0, -1.0))
    out = np.zeros((n, len(FIELDS)))
    for k in range(n):
        t = t0 + k * dt
        x, y, th, vx, vy, w = f(t)
        out[k, [F["t"], F["x"], F["y"], F["theta"], F["vx"], F["vy"], F["omega"]]] = t, x, y, th, vx, vy, w
        out[k, F["imu_ok"]] = 1.0 if k % 2 == 0 else 0.0
        out[k, F["odom_ok"]] = 1.0
        out[k, F["reset_id"]] = reset_id
    return out


def test_interpolation_exact_and_latest():
    h = PoseHistory(60.0)
    h.append(rows(100.0, 500))
    t_end = 100.0 + 499 * 0.01
    snap = h.poses_at([100.0 + 0.123456, 100.5, None, t_end + FUTURE_TOL / 2], now=t_end + 0.02)
    p, q, now, late = snap.poses
    assert snap.reset_id == 0
    assert p["source"] == "interpolated" and p["t"] == p["t_query"] == pytest.approx(100.123456)
    assert p["x"] == pytest.approx(2 * 100.123456 + 1, abs=1e-9) and p["y"] == pytest.approx(-100.123456, abs=1e-9)
    assert p["theta"] == pytest.approx(0.5 * 100.123456, abs=1e-9) and p["vx"] == pytest.approx(10.0123456)
    assert p["imu_ok"] is False and p["odom_ok"] is True            # neighbours: one without the IMU
    assert q["source"] == "exact" and q["t"] == 100.5 and q["imu_ok"] is True
    assert now["source"] == "latest" and now["t"] == pytest.approx(t_end) and now["t_query"] == t_end + 0.02
    assert late["source"] == "latest"
    with pytest.raises(HistoryError) as e:
        h.poses_at([t_end + FUTURE_TOL + 0.01], now=0)
    assert e.value.status == 409 and "not yet" in str(e.value)
    assert e.value.available == {"t_start": 100.0, "t_end": pytest.approx(t_end), "reset_id": 0}


def test_continuous_theta_interpolates_across_pi():
    h = PoseHistory(10.0)
    h.append(rows(0.0, 2, dt=1.0, f=lambda t: (0, 0, math.pi - 0.1 + 0.2 * t, 0, 0, 0)))
    p = h.poses_at([0.5], now=1.0).poses[0]
    assert p["theta"] == pytest.approx(math.pi)                        # not the wrapped average 0


def test_window_eviction_and_wraparound_matches_a_full_reference():
    h = PoseHistory(1.0)                                               # 105 samples
    data = rows(0.0, 350)
    for k in range(0, 350, 7):                                         # uneven batches, wraps 3 times
        h.append(data[k:k + 7])
    info = h.info()
    assert info["n"] == h.capacity == 105 and info["t_end"] == pytest.approx(3.49)
    assert info["t_start"] == pytest.approx(3.49 - 1.0)                # trimmed to the window
    rng = random.Random(1)
    ts = [rng.uniform(info["t_start"], info["t_end"]) for _ in range(300)] + [info["t_start"], info["t_end"]]
    for p in h.poses_at(ts, now=3.5).poses:
        assert p["x"] == pytest.approx(np.interp(p["t_query"], data[:, 0], data[:, F["x"]]), abs=1e-9)
    with pytest.raises(HistoryError) as e:
        h.poses_at([2.0], now=3.5)
    assert e.value.status == 404 and "window" in str(e.value)


def test_reset_clears_history():
    h = PoseHistory(60.0)
    h.append(rows(0.0, 100))
    assert h.poses_at([0.5], now=1.0).poses[0]["source"] == "exact"
    h.clear(1)
    with pytest.raises(HistoryError) as e:
        h.poses_at([0.5], now=1.0)
    assert e.value.status == 404 and "no poses" in str(e.value) and e.value.available["t_start"] is None
    h.append(rows(0.5, 10, reset_id=0))                                # a stale-frame row is never stored
    assert h.info()["n"] == 0
    h.append(rows(1.0, 100, reset_id=1))
    with pytest.raises(HistoryError) as e:
        h.poses_at([0.5], now=2.0)
    assert e.value.status == 404 and "reset" in str(e.value) and e.value.available["reset_id"] == 1
    assert h.poses_at([1.5], now=2.0).reset_id == 1


def test_one_reset_id_per_reply_under_concurrent_resets():
    h = PoseHistory(60.0)
    stop = threading.Event()

    base = rows(0.0, 50)

    def writer():
        rid = 0
        while not stop.is_set():
            rid += 1
            r = base.copy()
            r[:, F["t"]] += rid
            r[:, F["x"]] = r[:, F["reset_id"]] = rid                   # x encodes the frame
            h.clear(rid)
            h.append(r)
    th = threading.Thread(target=writer, daemon=True)
    th.start()
    checked = 0
    try:
        while checked < 100:
            info = h.info()
            if not info["n"]:
                continue
            try:
                snap = h.poses_at([info["t_start"], None, info["t_end"]], now=info["t_end"])
            except HistoryError:
                continue                                               # cleared in between: fine
            assert {p["x"] for p in snap.poses} == {snap.reset_id}
            checked += 1
    finally:
        stop.set()
        th.join()


def compose(a, d):
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * d[0] - s * d[1], a[1] + s * d[0] + c * d[1], a[2] + d[2]


def test_relative_is_the_inverse_composition():
    a = {"x": 1.0, "y": 2.0, "theta": math.pi / 2}
    b = {"x": 1.0, "y": 3.0, "theta": math.pi / 2 + 0.3}
    r = relative(a, b)                                                 # 1 m along +y world = forward
    assert (r["dx"], r["dy"], r["dtheta"]) == pytest.approx((1.0, 0.0, 0.3))
    r = relative({"x": 0, "y": 0, "theta": 0}, {"x": 0, "y": 1, "theta": 0})
    assert (r["dx"], r["dy"]) == pytest.approx((0.0, 1.0))             # left
    rng = random.Random(2)
    for _ in range(200):
        pa = (rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-20, 20))
        pb = (rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-20, 20))
        r = relative(dict(zip("x y theta".split(), pa)), dict(zip("x y theta".split(), pb)))
        x, y, th = compose(pa, (r["dx"], r["dy"], r["dtheta_total"]))
        assert (x, y, th) == pytest.approx(pb, abs=1e-9)               # T_a * rel == T_b
        assert -math.pi < r["dtheta"] <= math.pi
        assert math.remainder(r["dtheta"] - r["dtheta_total"], 2 * math.pi) == pytest.approx(0, abs=1e-9)


def test_four_hour_buffer_memory():
    h = PoseHistory(4 * 3600.0)
    assert h.capacity >= 4 * 3600 * 100
    assert h.memory_bytes < 100e6                                      # ~86 MB
