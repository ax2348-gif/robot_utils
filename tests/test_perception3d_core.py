"""3d_perception core: registration geometry, motion interpolation, compensation modes (no I/O).

Synthetic scene: a wall 3 m away and a box face 1.5 m away in front of it, seen by a depth camera
at t_d; the camera then moves (rotation and/or translation) until the colour frame at t_c. The
ground truth depth in the colour image at t_c is ray-cast analytically, independently of the
registration code; registering with the true motion must match it, registering without must not.
"""
import math

import numpy as np
import pytest

from robot_utils.nodes.perception3d.motion import (Compensator, PoseTrack, QuatTrack, base_T, quat_from_axis_angle,
                                                   quat_from_R, quat_to_R, rotation_angle, rz, slerp)
from robot_utils.nodes.perception3d.register import Intrinsics, Registrar, inv, query_point, rigid

# the robot's camera (848x480 depth, 1280x720 colour, real distortion and extrinsic)
DEPTH = Intrinsics(415.5, 415.5, 423.34, 238.68, 848, 480)
COLOR = Intrinsics(609.52, 609.65, 640.54, 354.30, 1280, 720,
                   (-0.0322, 0.0386, -0.0135, 0.0, 0.0, 0.0, -0.000344, -0.000119))
E = rigid([[0.999992, 0.003328, -0.002076], [-0.003328, 0.999994, 0.000163], [0.002076, -0.000156, 0.999998]],
          [-0.0236931, -0.0000123, 0.000388])

WALL_Z, BOX_Z = 3.0, 1.5
BOX = (-0.35, 0.35, -0.25, 0.25)      # x0 x1 y0 y1 of the box face, scene frame


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def raycast(origin, dirs):
    """z-distance along dirs (…, 3) from origin to the nearest scene surface, in units of dirs."""
    o = np.asarray(origin, float)
    best = np.full(dirs.shape[:-1], np.inf)
    for k, extent in ((WALL_Z, None), (BOX_Z, BOX)):
        with np.errstate(divide="ignore", invalid="ignore"):
            s = (k - o[2]) / dirs[..., 2]
        p = o + s[..., None] * dirs
        ok = s > 0
        if extent is not None:
            ok &= (p[..., 0] >= extent[0]) & (p[..., 0] <= extent[1]) & (p[..., 1] >= extent[2]) & (p[..., 1] <= extent[3])
        best = np.where(ok & (s < best), s, best)
    return best


def render_depth():
    """Depth image (uint16 mm) of the scene from the depth camera at t_d (= scene frame)."""
    u, v = np.meshgrid(np.arange(DEPTH.width), np.arange(DEPTH.height))
    x, y = DEPTH.ray(u, v)
    z = raycast([0, 0, 0], np.stack([x, y, np.ones_like(x)], -1))
    return np.where(np.isfinite(z), np.round(z * 1000), 0).astype(np.uint16)


def truth(out, M_true):
    """Ground-truth z (mm) in the colour camera at t_c on the ``out`` grid. The colour camera at t_c
    sits at T_scene_c = inv(M_true) · inv(E) in the scene frame (depth camera at t_d)."""
    T = inv(M_true) @ inv(E)
    u, v = np.meshgrid(np.arange(out.width), np.arange(out.height))
    x, y = out.ray(u, v)
    d = np.einsum("ij,...j->...i", T[:3, :3], np.stack([x, y, np.ones_like(x)], -1))    # ray directions in the scene; z_c = 1
    s = raycast(T[:3, 3], d)
    return np.where(np.isfinite(s), s * 1000, 0)


def error_stats(reg, gt):
    both = (reg > 0) & (gt > 0)
    err = np.abs(reg.astype(float) - gt)[both]
    return {"bad": float(np.mean(err > 30)), "median": float(np.median(err)), "coverage": float(both.mean())}


@pytest.fixture(scope="module")
def depth():
    return render_depth()


def yaw_motion(rate=1.0, dt=0.05):
    """A camera turning about its own vertical axis (optical y) at `rate` rad/s for dt s."""
    return rigid(Ry(rate * dt).T, [0, 0, 0])


def test_static_registration_matches_raycast(depth):
    r = Registrar(DEPTH, COLOR, E, scale=0.5)
    s = error_stats(r.register(depth), truth(r.out, np.eye(4)))
    assert s["bad"] < 0.005 and s["median"] < 2 and s["coverage"] > 0.8, s


@pytest.mark.parametrize("M_true", [
    yaw_motion(1.0, 0.05),                                    # 1 rad/s over 50 ms: ~2.9 deg
    rigid(Rx(0.03) @ Ry(-0.04), [0.05, -0.02, 0.06]),          # rotation + 8 cm translation
])
def test_motion_compensation(depth, M_true):
    r = Registrar(DEPTH, COLOR, E, scale=0.5)
    gt = truth(r.out, M_true)
    comp = error_stats(r.register(depth, motion=M_true), gt)
    plain = error_stats(r.register(depth), gt)
    assert comp["bad"] < 0.005 and comp["median"] < 2, comp
    assert plain["bad"] > 0.5, plain                           # wrong z everywhere, box edges misplaced


def test_yaw_shift_in_pixels(depth):
    """The dominant error: 1 rad/s over 50 ms moves the image fx·ω·dt ≈ 30 px at 1280x720."""
    r = Registrar(DEPTH, COLOR, E, scale=1.0, splat=2)
    plain, comp = r.register(depth), r.register(depth, motion=yaw_motion())
    row = 360

    def left_edge(img):   # first box pixel (z ≈ 1.5 m) along a row
        return int(np.argmax((img[row] > 1400) & (img[row] < 1600)))
    gt = truth(r.out, yaw_motion())
    gt_edge = int(np.argmax((gt[row] > 1400) & (gt[row] < 1600)))
    assert abs(left_edge(comp) - gt_edge) <= 1
    assert 27 <= abs(left_edge(plain) - gt_edge) <= 36


def test_stride_and_splat(depth):
    gt = truth(COLOR.scaled(0.25), np.eye(4))
    r = Registrar(DEPTH, COLOR, E, scale=0.25, stride=2)
    s = error_stats(r.register(depth), gt)
    assert r.out.width == 320 and s["bad"] < 0.01 and s["coverage"] > 0.8, s
    full = Registrar(DEPTH, COLOR, E, scale=1.0)
    holes = error_stats(full.register(depth), truth(full.out, np.eye(4)))["coverage"]
    splat = Registrar(DEPTH, COLOR, E, scale=1.0, splat=2)
    filled = error_stats(splat.register(depth), truth(splat.out, np.eye(4)))["coverage"]
    assert filled > holes + 0.2                    # denser output than depth: splat fills the gaps


def test_distortion_roundtrip_and_scaling():
    u, v = np.meshgrid(np.linspace(0, 1279, 9), np.linspace(0, 719, 7))
    x, y = COLOR.ray(u, v)
    pu, pv = COLOR.project(np.stack([x, y, np.ones_like(x)], -1))
    assert np.abs(pu - u).max() < 1e-6 and np.abs(pv - v).max() < 1e-6
    h = COLOR.scaled(0.5)
    assert (h.width, h.height) == (640, 360)
    # the same 3D point lands on the same spot (in units of each grid)
    p = np.array([0.4, -0.2, 1.7])
    (a, b), (c, d) = COLOR.project(p), h.project(p)
    assert abs((a + 0.5) * 0.5 - 0.5 - c) < 1e-9 and abs((b + 0.5) * 0.5 - 0.5 - d) < 1e-9


def test_query_point(depth):
    r = Registrar(DEPTH, COLOR, E, scale=0.5)
    reg = r.register(depth)
    p = COLOR.project(E[:3, :3] @ np.array([0.0, 0.0, BOX_Z]) + E[:3, 3])     # box centre in colour pixels
    q = query_point(reg, r.out, COLOR, float(p[0]), float(p[1]))
    assert q["source"] == "pixel" and abs(q["depth_mm"] - BOX_Z * 1000) < 5
    pt_depth = inv(E) @ np.array([*q["point"], 1.0])       # back in the depth frame: the box centre
    assert np.allclose(pt_depth[:3], [0, 0, BOX_Z], atol=0.01)
    hole = reg.copy()
    iu, iv = q["u_out"], q["v_out"]
    hole[iv, iu] = 0
    q2 = query_point(hole, r.out, COLOR, float(p[0]), float(p[1]))
    assert q2["source"] == "window_median" and abs(q2["depth_mm"] - 1500) < 5
    assert query_point(reg, r.out, COLOR, -50, 10)["source"] == "outside"


# -- interpolation -----------------------------------------------------------------------------

def test_slerp_and_quat_track():
    q0, q1 = quat_from_axis_angle([0, 0, 1], 0.0), quat_from_axis_angle([0, 0, 1], 0.2)
    assert rotation_angle(quat_to_R(slerp(q0, q1, 0.5))) == pytest.approx(0.1, abs=1e-9)
    tr = QuatTrack(max_gap=0.1, max_extrapolate=0.05)
    for k in range(11):       # 1 rad/s about z, 100 Hz; every other one sign-flipped (same rotation)
        q = quat_from_axis_angle([0, 0, 1], 0.01 * k)
        tr.add(k * 0.01, q if k % 2 else -q)
    assert rotation_angle(quat_to_R(tr.at(0.055))) == pytest.approx(0.055, abs=1e-9)
    assert rotation_angle(quat_to_R(tr.at(0.13))) == pytest.approx(0.13, abs=1e-12)   # extrapolated 30 ms
    assert tr.at(0.2) is None and tr.at(-0.01) is None
    tr.add(0.5, quat_from_axis_angle([0, 0, 1], 0.5))        # 390 ms gap: not bridged
    assert tr.at(0.3) is None and tr.at(0.05) is not None
    tr.add(2.0, quat_from_axis_angle([0, 0, 1], 0.5))        # > 1 s gap (reconnected): history dropped
    assert tr.start == 2.0 and tr.at(0.05) is None


def test_quat_from_R_roundtrip():
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        assert np.allclose(quat_to_R(quat_from_R(quat_to_R(q))), quat_to_R(q), atol=1e-12)


def test_pose_track():
    tr = PoseTrack(max_gap=0.1, max_extrapolate=0.1)
    for k in range(10):
        tr.add(k * 0.01, 0.01 * k, 0.0, 0.02 * k)
    assert np.allclose(tr.at(0.045), [0.045, 0, 0.09])
    assert np.allclose(tr.at(0.15), [0.15, 0, 0.3])      # extrapolated 60 ms
    assert tr.at(0.25) is None


# -- compensation modes ------------------------------------------------------------------------

R_DI = Rx(0.02) @ Ry(-0.01)            # IMU -> depth: deliberately not the identity
R_BD = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]) @ Rx(-0.4)   # optical in base, tilted down
T_BD = rigid(R_BD, [0.2, 0.05, 1.1])


def robot_at(t, v=1.0, w=1.0):
    """Base on an arc: forward v m/s, yaw w rad/s, from the origin."""
    if abs(w) < 1e-12:
        return v * t, 0.0, 0.0
    th = w * t
    return v / w * math.sin(th), v / w * (1 - math.cos(th)), th


def feed(comp, t0=0.0, t1=0.3, imu=True, pose=True):
    for k in range(int((t1 - t0) * 100) + 1):
        t = t0 + k * 0.01
        x, y, th = robot_at(t)
        if pose:
            comp.pose.add(t, x, y, th)
        if imu:   # IMU orientation (IMU -> world): R_wd · R_di with R_wd = Rz(theta) R_bd
            comp.imu.add(t + 0.003, quat_from_R(rz(th) @ R_BD @ R_DI))


def true_M(t_d, t_c):
    Twd = lambda t: base_T(*robot_at(t)) @ T_BD    # noqa: E731
    return inv(Twd(t_c)) @ Twd(t_d)


def test_modes_and_fallbacks():
    t_d, t_c = 0.105, 0.187
    c = Compensator()
    c.set_imu_to_depth(R_DI)
    assert c.relative(t_d, t_c)[1] == "none"
    feed(c, pose=False)
    M, mode = c.relative(t_d, t_c)
    assert mode == "rotation"
    assert np.allclose(M[:3, :3], true_M(t_d, t_c)[:3, :3], atol=1e-6) and np.allclose(M[:3, 3], 0)
    feed(Compensator(), imu=False)   # (pose without T_bd: still rotation only)
    c2 = Compensator()
    c2.set_imu_to_depth(R_DI)
    feed(c2)
    assert c2.relative(t_d, t_c)[1] == "rotation"          # no description calibration yet
    c2.set_base_to_depth(T_BD)
    M, mode = c2.relative(t_d, t_c)
    assert mode == "full"
    assert np.allclose(M, true_M(t_d, t_c), atol=2e-4), (M, true_M(t_d, t_c))   # linear pose interp.
    c2.use_imu = False
    M, mode = c2.relative(t_d, t_c)
    assert mode == "base" and np.allclose(M, true_M(t_d, t_c), atol=2e-4)
    c3 = Compensator()
    c3.set_base_to_depth(T_BD)
    feed(c3, imu=False)
    assert c3.relative(t_d, t_c)[1] == "base"
    assert c3.relative(t_d, 5.0)[1] == "none"               # colour time far beyond the tracks


def test_imu_compensation_end_to_end(depth):
    """IMU track -> M -> registration: a 1 rad/s turn of the robot, camera tilted down."""
    t_d, t_c = 0.10, 0.16
    c = Compensator()
    c.set_imu_to_depth(R_DI)
    feed(c, pose=False)
    M, mode = c.relative(t_d, t_c)
    M_true = true_M(t_d, t_c)
    M_rot = rigid(M_true[:3, :3], [0, 0, 0])   # what the IMU can know (no lever arm / translation)
    assert mode == "rotation" and np.allclose(M, M_rot, atol=1e-6)
    r = Registrar(DEPTH, COLOR, E, scale=0.5)
    gt = truth(r.out, M_rot)
    assert error_stats(r.register(depth, motion=M), gt)["bad"] < 0.005
    assert error_stats(r.register(depth), gt)["bad"] > 0.5
