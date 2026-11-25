"""The operator view replay (robot_utils.replay, docs/collect.md "Operator view replay") on a synthetic recorded
session (tests/replay_fakes.py): the camera is the logged head through the logged placement, what is drawn lands
where an independent pinhole projection puts it (asymmetric eye frusta, a scaled / offset / turned placement),
the controllers where they were logged, the ghost in its frame colour, old logs reconstructed, and a round trip to
an mp4."""
import json
import subprocess
from pathlib import Path

import numpy as np
import pytest

import robot_utils.nodes.quest_teleop.client  # noqa: F401 (puts tools/ on the path: the fake headset)
from replay_fakes import GHOST_RGBA, head_at, marker_at, right_at, write_session
from robot_utils.replay import ar
from robot_utils.replay.__main__ import main as replay_main
from robot_utils.replay.log import load
from robot_utils.replay.operator_view import OperatorView, ViewOptions, find_ffmpeg
from robot_utils.replay.render import CONTROLLER_RGBA, DEFAULT_BG

BG = (np.asarray(DEFAULT_BG) * 255).round()


def frames(rec, **kw):
    kw.setdefault("width", 480)
    kw.setdefault("controllers", "off")
    return list(OperatorView(rec, ViewOptions(**kw)).frames())


def truth_at(info, t):
    """The writer's own placement (not the logged JSON) at the sample the frame used."""
    return min((x for x in info["truth"] if x["t"] <= t + 1e-9), key=lambda x: t - x["t"])


def blob_centroid(img, pred):
    m = pred(img.astype(int))
    ys, xs = np.nonzero(m)
    assert len(xs) > 3, "nothing of that colour in the image"
    return np.array([xs.mean() + 0.5, ys.mean() + 0.5])


def is_white(a):
    return (a.min(-1) > 90) & (np.ptp(a, -1) < 12)


def is_green(a):
    return (a[..., 1] > a[..., 0] + 40) & (a[..., 1] > a[..., 2] + 40)


def test_camera_is_the_logged_head_through_the_logged_placement_and_the_marker_lands_where_projected(tmp_path):
    info = write_session(tmp_path / "s")
    rec = load(tmp_path / "s")
    assert rec.has_logged_placement
    view = OperatorView(rec, ViewOptions(width=480, controllers="off"))
    assert view.summary["placement"] == "logged" and view.summary["placements"] == 2   # first head + the reset
    out = list(view.frames())
    assert len(out) == 61
    eye = view.eye
    w, h = view.width, view.height
    for img, fi in out[::6]:
        tr = truth_at(info, fi.t)
        p_s, r_s = tr["sim_in_local"]
        hp, hr = head_at(fi.head_t - info["t0"])
        # the camera: the logged head (VIEW = the centre eye) mapped into the sim through the placement
        cp, cr = fi.camera_sim
        np.testing.assert_allclose(cp, r_s.T @ (hp - p_s) / tr["scale"], atol=2e-4)
        np.testing.assert_allclose(cr, r_s.T @ hr, atol=2e-4)
        # the picture: the marker where a pinhole projection of its LOCAL position puts it
        x_local = r_s @ (tr["scale"] * marker_at(fi.t - info["t0"])) + p_s
        uv = ar.project((hp, hr), eye, x_local, w, h)
        assert uv is not None and 0 < uv[0] < w and 0 < uv[1] < h
        np.testing.assert_allclose(blob_centroid(img, is_white), uv, atol=1.5)
    # after the reset the scene was re-anchored at the head then: the frames say so
    assert {fi.placement for _, fi in out} == {"logged"}


@pytest.mark.parametrize("eye_name", ["left", "right"])
def test_one_eye_with_its_own_pose_and_asymmetric_field(tmp_path, eye_name):
    info = write_session(tmp_path / "s", seconds=0.5)
    rec = load(tmp_path / "s")
    view = OperatorView(rec, ViewOptions(width=480, eye=eye_name, controllers="off"))
    eye = view.eye
    assert eye.source == "logged" and eye.fov == ar.DEFAULT_EYE_FOV[eye_name]
    for img, fi in list(view.frames())[::5]:
        tr = truth_at(info, fi.t)
        p_s, r_s = tr["sim_in_local"]
        hp, hr = head_at(fi.head_t - info["t0"])
        el = (hp + hr @ eye.p, hr @ eye.r)
        np.testing.assert_allclose(fi.eye_local[0], el[0], atol=1e-6)
        uv = ar.project(el, eye, r_s @ marker_at(fi.t - info["t0"]) + p_s, view.width, view.height)
        np.testing.assert_allclose(blob_centroid(img, is_white), uv, atol=1.5)


def test_scaled_offset_turned_placement(tmp_path):
    info = write_session(tmp_path / "s", seconds=0.5, offset=(0.2, -0.1, 0.3), yaw=0.4, scale=0.5, floor=0.3)
    rec = load(tmp_path / "s")
    for img, fi in frames(rec)[::5]:
        tr = truth_at(info, fi.t)
        p_s, r_s = tr["sim_in_local"]
        assert tr["scale"] == 0.5
        hp, hr = head_at(fi.head_t - info["t0"])
        uv = ar.project((hp, hr), OperatorView(rec, ViewOptions(width=480)).eye,
                        r_s @ (0.5 * marker_at(fi.t - info["t0"])) + p_s, img.shape[1], img.shape[0])
        np.testing.assert_allclose(blob_centroid(img, is_white), uv, atol=1.5)


def test_controllers_are_drawn_where_logged_and_hidden_when_untracked(tmp_path):
    info = write_session(tmp_path / "s", seconds=1.0)
    rec = load(tmp_path / "s")
    out = frames(rec, controllers="overlay")
    right = np.round(np.asarray(CONTROLLER_RGBA["right"][:3]) * 255)
    left = np.round(np.asarray(CONTROLLER_RGBA["left"][:3]) * 255)

    def hue_like(c):
        u = c / np.linalg.norm(c)
        return lambda a: (np.einsum("...k,k->...", a, u) / (np.linalg.norm(a, axis=-1) + 1e-9) > 0.995) & \
            (a.max(-1) > 60)
    view = OperatorView(rec, ViewOptions(width=480))
    for img, fi in out[::4]:
        hp, hr = head_at(fi.head_t - info["t0"])
        rp, _ = right_at(fi.head_t - info["t0"])
        uv = ar.project((hp, hr), view.eye, rp, img.shape[1], img.shape[0])
        m = hue_like(right)(img.astype(float))
        ys, xs = np.nonzero(m)
        assert len(xs) > 50
        # the grip origin is inside the drawn controller's extent, near its centre
        assert xs.min() - 2 <= uv[0] <= xs.max() + 2 and ys.min() - 2 <= uv[1] <= ys.max() + 2
        assert np.linalg.norm(np.array([xs.mean(), ys.mean()]) - uv) < 0.25 * max(np.ptp(xs), np.ptp(ys)) + 3
        left_px = hue_like(left)(img.astype(float)).sum()
        assert (left_px > 50) == fi.hands["left"], (fi.index, left_px, fi.hands)
    # on top of the scene (overlay) vs hidden by what is in front (depth): both draw it here (nothing in front)
    d = frames(rec, controllers="depth", max_frames=1)[0][0]
    assert hue_like(right)(d.astype(float)).sum() > 50


def test_the_ghost_is_drawn_in_its_frame_colour(tmp_path):
    write_session(tmp_path / "s", seconds=0.3)
    rec = load(tmp_path / "s")
    img = frames(rec, max_frames=1)[0][0]
    g = img[is_green(img.astype(int))].astype(float)
    assert len(g) > 10                                          # grey in the scene, green from the frame override
    u = np.asarray(GHOST_RGBA[:3]) / np.linalg.norm(GHOST_RGBA[:3])
    assert np.median(g @ u / np.linalg.norm(g, axis=1)) > 0.98
    # planes are left out, the rest of the image is the plain background
    assert (np.abs(img.astype(int) - BG).sum(-1) < 3).mean() > 0.9


def test_old_logs_reconstruct_the_placement_from_the_first_head_and_reset_seq(tmp_path):
    write_session(tmp_path / "logged", seconds=2.0, floor=0.0)
    write_session(tmp_path / "old", seconds=2.0, logged=False)
    a, b = load(tmp_path / "logged"), load(tmp_path / "old")
    assert not b.has_logged_placement
    va, vb = OperatorView(a, ViewOptions(width=240)), OperatorView(b, ViewOptions(width=240))
    assert vb.summary["placement"] == "reconstructed" and vb.summary["placements"] == 2
    assert vb.eye.source == "default"
    fa = [fi for _, fi in va.frames()]
    fb = [fi for _, fi in vb.frames()]
    assert len(fa) == len(fb)
    for x, y in zip(fa, fb):     # a headset with a floor at head - 1.6 m and no config: the same placement
        np.testing.assert_allclose(x.camera_sim[0], y.camera_sim[0], atol=1e-4)
        np.testing.assert_allclose(x.camera_sim[1], y.camera_sim[1], atol=1e-4)


def test_episode_window_of_a_session(tmp_path):
    info = write_session(tmp_path / "s", seconds=2.0)
    t0 = info["t0"]
    (tmp_path / "s" / "episodes.jsonl").write_text(json.dumps(
        {"id": "e1", "task": "", "t_start": t0 + 1.2, "t_stop": t0 + 1.5, "outcome": "kept", "reason": None}) + "\n")
    rec = load(tmp_path / "s", "e1")
    out = frames(rec)
    assert len(out) == 10 and abs(out[0][1].t - (t0 + 1.2)) < 1e-9 and out[-1][1].t <= t0 + 1.5
    assert out[0][1].placement == "logged"


def test_round_trip_to_an_mp4(tmp_path, capsys):
    try:
        exe = find_ffmpeg()
    except RuntimeError:
        pytest.skip("no ffmpeg on this machine")
    write_session(tmp_path / "s", seconds=1.0)
    out = tmp_path / "view.mp4"
    assert replay_main([str(tmp_path / "s"), "-o", str(out), "--width", "320", "--fps", "20"]) == 0
    summary = json.loads(capsys.readouterr().out)
    assert summary["frames"] == 21 and summary["placement"] == "logged" and summary["blank_frames"] == 0
    w, h = summary["width"], summary["height"]
    raw = subprocess.run([exe, "-loglevel", "error", "-i", str(out), "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
                         capture_output=True, check=True).stdout
    assert len(raw) == 21 * w * h * 3
    side = json.loads(Path(str(out) + ".json").read_text())
    assert len(side["frames"]) == 21 and side["frames"][0]["camera_sim"] is not None
    first = np.frombuffer(raw[:w * h * 3], np.uint8).reshape(h, w, 3).astype(int)
    assert is_white(first).sum() > 3                              # the marker survives the encode


# -- the placement port and the fake headset's log ----------------------------------------------------------------

def _rot(axis, a):
    from quest_teleop.frames import rot_axis   # noqa: PLC0415 (tools/, via the adapter)
    return rot_axis(axis, a)


def test_placement_port_follows_the_native_rules():
    """The cases of quest_env test_robot_ar.cpp testPlacement, on the Python port."""
    look_left = _rot((0, 1, 0), np.pi / 2) @ _rot((1, 0, 0), -0.3)
    head = ar.flu(np.array([0.1, 1.7, -0.2]), look_left)
    ahead = ar.flu(np.array([0.0, 1.6, 0.0]), np.eye(3))
    left_origin, ahead_origin = np.array([0.2, 1.5, 0.1]), np.array([1.6, 0.0, 0.0])
    pl = ar.Placement()
    assert not pl.update(7, None) and not pl.placed
    assert not pl.update(7, head)
    np.testing.assert_allclose(pl.origin_p, left_origin, atol=1e-6)
    np.testing.assert_allclose(pl.origin_r, ar.rot_z(np.pi / 2), atol=1e-9)
    assert pl.reason == "first" and pl.placements == 1
    assert not pl.update(7, ahead) and np.allclose(pl.origin_p, left_origin)
    assert pl.update(7, ahead, None, True) and np.allclose(pl.origin_p, ahead_origin) and pl.reason == "A"
    assert not pl.update(7, None, None, True) and not pl.update(7, head)          # A untracked: ignored
    assert not pl.update(7, head, 5) and not pl.update(7, head, 5, True)          # baseline; A ignored with seq
    assert pl.placed_seq == 5 and np.allclose(pl.origin_p, ahead_origin)
    assert pl.update(7, head, 6) and np.allclose(pl.origin_p, left_origin) and pl.reason == "reset_seq"
    assert not pl.update(7, None, 7) and pl.pending                               # untracked: deferred
    assert pl.update(7, ahead, 7) and np.allclose(pl.origin_p, ahead_origin) and pl.placed_seq == 7
    assert not pl.update(8, None, 7) and not pl.placed
    assert not pl.update(8, head, 7) and pl.placed and pl.reason == "session" and pl.placements == 5
    f = ar.Placement()
    high = ar.flu(np.array([0.0, 0.5, 0.0]), np.eye(3))
    f.update(1, high, None, False, -1.2)
    np.testing.assert_allclose(f.origin_p, [1.6, 0, -1.2], atol=1e-9)
    assert f.update(1, high, None, True, None) and np.allclose(f.origin_p, [1.6, 0, 0.5 - 1.6])
    b = f.block("STAGE")
    assert b["floor"] == "estimated" and b["placements"] == 2 and b["reset_seq"] is None
    down = _rot((0, 1, 0), np.pi / 2) @ _rot((1, 0, 0), -np.pi / 2)              # looking straight down
    np.testing.assert_allclose(ar.Placement.head_yaw(ar.flu(np.zeros(3), down)[1]), ar.rot_z(np.pi / 2), atol=1e-9)


def test_fake_headset_logs_the_placement_a_headset_logs():
    from quest_teleop.fake_headset import ArView, Headset, HeadsetState   # noqa: PLC0415
    seq = {"v": 3}
    hs = Headset(ar=ArView(lambda: seq["v"], floor_z=-1.5))
    f0 = hs.frame(HeadsetState(), 1.0)
    b0 = f0["robot_ar"]
    assert b0["placed"] and b0["reason"] == "first" and b0["floor"] == "LOCAL_FLOOR" and b0["placed_reset_seq"] == 3
    sil = ar.logged_placement(f0)
    # the identity head (LOCAL origin, facing -Z): the sim origin 1.6 m ahead at the floor, +x forward
    np.testing.assert_allclose(sil.to_local([0, 0, 0]), [0, -1.5, -1.6], atol=1e-6)
    np.testing.assert_allclose(sil.to_local([1, 0, 0]), [0, -1.5, -2.6], atol=1e-6)
    np.testing.assert_allclose(sil.to_local([0, 0, 1]), [0, -0.5, -1.6], atol=1e-6)
    assert tuple(ar.logged_views(f0)["left"].fov) == ar.DEFAULT_EYE_FOV["left"]
    seq["v"] = 4
    f1 = hs.frame(HeadsetState(head_yaw=0.5, head_pos=(0.3, 0.0, 0.0)), 1.02)
    assert f1["robot_ar"]["reason"] == "reset_seq" and f1["robot_ar"]["placements"] == 2
