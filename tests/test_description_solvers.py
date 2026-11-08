"""Description node maths: frames, ground plane, motion line, rotation (no I/O)."""
import math

import numpy as np
import pytest

from robot_utils.nodes.description.frames import (FrameTree, describe_T, make_T, matrix_to_rpy, orthonormalize,
                                                  rpy_to_matrix, wahba)
from robot_utils.nodes.description.solvers import (angle_deg, fit_ground_plane, fit_motion_line, polygon_mask,
                                                   solve_rotation, unproject)

# A camera mounted 0.6 m up, looking forward and 25 deg down, rolled 2 deg, yawed 5 deg (base <- camera).
OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)   # optical (x right, y down, z fwd) -> base (x fwd, y left, z up)
R_TRUE = rpy_to_matrix(math.radians(2), math.radians(25), math.radians(5)) @ OPT
T_TRUE = make_T(R_TRUE, [0.1, -0.05, 0.6])


def test_rpy_roundtrip_and_tree():
    for rpy in [(0.1, -0.4, 2.0), (0.0, 0.0, 0.0), (-1.0, 1.2, -3.0)]:
        assert matrix_to_rpy(rpy_to_matrix(*rpy)) == pytest.approx(rpy, abs=1e-9)
    tree = FrameTree({"base": {"parent": None},
                      "cam": {"parent": "base", "xyz": [0.1, 0, 0.6], "rpy": [0, 0.3, 0]},
                      "imu": {"parent": "cam", "translation": [0.01, 0, 0], "rotation": np.eye(3).tolist()}})
    T = tree.T("base", "imu")
    assert T[:3, 3] == pytest.approx([0.1 + 0.01 * math.cos(0.3), 0, 0.6 - 0.01 * math.sin(0.3)])
    assert np.allclose(tree.T("imu", "base"), np.linalg.inv(T))
    with pytest.raises(KeyError):
        FrameTree({"a": {"parent": "nope"}})
    assert describe_T(T)["rpy"][1] == pytest.approx(0.3)


def test_wahba_recovers_rotation():
    R = orthonormalize(rpy_to_matrix(0.2, -0.3, 1.1))
    b = [np.array(v, float) for v in ([1, 0, 0], [0, 1, 0], [0.3, 0.2, 0.9])]
    assert np.allclose(wahba(b, [R @ v / np.linalg.norm(v) for v in b]), R, atol=1e-9)


def floor_depth(T, K, h=120, w=212, noise=0.0, seed=0):
    """Depth image (m) of an infinite floor (base z = 0) seen by a camera at pose T."""
    R, t = T[:3, :3], T[:3, 3]
    v, u = np.mgrid[0:h, 0:w]
    ray = np.stack([(u - K["cx"] * w / K["width"]) / (K["fx"] * w / K["width"]),
                    (v - K["cy"] * h / K["height"]) / (K["fy"] * h / K["height"]), np.ones((h, w))], -1)
    rz = ray @ R[2]                      # base z of the ray per unit depth
    z = np.where(rz < -1e-6, -t[2] / np.where(rz < -1e-6, rz, -1), 0.0)
    z[z > 8] = 0
    return z + np.random.default_rng(seed).normal(0, noise, z.shape) * (z > 0)


K = {"fx": 415.5, "fy": 415.5, "cx": 423.3, "cy": 238.7, "width": 848, "height": 480}


def test_ground_plane_recovers_height_and_normal():
    d = floor_depth(T_TRUE, K, noise=0.005)
    d[:20] = 2.0                          # clutter (a wall-ish patch) in the top rows
    P = unproject(d, K)
    up_cam = R_TRUE.T @ np.array([0, 0, 1.0])
    for gravity in (None, up_cam):
        g = fit_ground_plane(P, gravity_up=gravity)
        assert g["height"] == pytest.approx(0.6, abs=0.003)
        assert angle_deg(g["normal"], up_cam) < 0.3
    assert g["angle_to_gravity_deg"] < 0.3


def test_polygon_mask_selects_region():
    m = polygon_mask((100, 200), [[[0.0, 0.5], [1.0, 0.5], [1.0, 1.0], [0.0, 1.0]]])
    assert m[:50].sum() == 0 and m[50:].all()


def synth_accel(direction_cam, n=4000, amp=0.4, f=1.0, rate=1000.0, g_cam=(0, -9.8, 0.5), noise=0.02, seed=1):
    t = np.arange(n) / rate
    a = amp * np.sin(2 * math.pi * f * t)
    A = np.outer(a, direction_cam) + np.asarray(g_cam) + np.random.default_rng(seed).normal(0, noise, (n, 3))
    return t, A, np.sign(a)


def test_motion_line_direction_sign_and_centroid():
    d_true = R_TRUE.T @ np.array([1.0, 0, 0])          # base +x seen in the camera frame
    t, A, lab = synth_accel(d_true)
    fit = fit_motion_line(A, lab)
    assert angle_deg(fit["direction"], d_true) < 0.2 and fit["sign_corr"] > 0.8
    assert fit["centroid"] == pytest.approx([0, -9.8, 0.5], abs=0.01) and fit["linearity"] < 0.1
    lagged = np.roll(lab, 150)                           # 150 ms label lag at 1 Hz (54 deg): still the right sign
    assert angle_deg(fit_motion_line(A, lagged)["direction"], d_true) < 0.2


def test_rotation_from_ground_and_two_lines():
    up_cam = R_TRUE.T @ np.array([0, 0, 1.0])
    dx = R_TRUE.T @ np.array([1.0, 0, 0])
    dy = R_TRUE.T @ np.array([0, 1.0, 0])
    R, info = solve_rotation([(up_cam, [0, 0, 1], 10.0), (dx, [1, 0, 0], 1.0), (dy, [0, 1, 0], 1.0)])
    assert np.allclose(R, R_TRUE, atol=1e-9) and max(info["residual_deg"]) < 1e-6
