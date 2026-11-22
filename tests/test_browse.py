"""The episode browser's data side (robot_utils.collect.browse): the time index, lazy rows / frames, the operator
tables and the clutch anchors, the model chain, export planning / encoding, and the HTTP API, on a synthetic
session in the recorder's exact layout (browse/synth.py)."""
import json
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

from robot_utils.collect.browse import export as X
from robot_utils.collect.browse.dataset import Library, Recording
from robot_utils.collect.browse.index import StreamIndex
from robot_utils.collect.browse.oplog import C, matrix_from_quat
from robot_utils.collect.browse.synth import T0, make_session
from robot_utils.collect.cut import cut_session
from robot_utils.collect.records import iter_raw
from robot_utils.collect.streams import rotated_files
from robot_utils.nodes.common.protocol import RowsDecoder

SECONDS = 30.0


@pytest.fixture(scope="module")
def session(tmp_path_factory):
    root = tmp_path_factory.mktemp("data")
    truth = make_session(root / "s1", SECONDS, cam_fps=10.0, cam_size=(96, 64), rows_hz=50.0, rotate=200_000,
                         jpeg_pool=5)
    return root, root / "s1", truth


@pytest.fixture()
def cache(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBOT_UTILS_BROWSE_CACHE", str(tmp_path / "cache"))
    return tmp_path / "cache"


def brute_rows(files, fields):
    """Every row of a rows stream by the protocol's own decoder."""
    dec = RowsDecoder()
    ts, vs = [], []
    for _, h, payload in iter_raw(files):
        r = dec.decode(h, payload)
        if r is None:
            continue
        ts.append(np.asarray(r["t_tick"] if "t_tick" in r.dtype.names else r["t"], float))
        vs.append(np.stack([np.asarray(r[f], float) for f in fields], 1))
    return np.concatenate(ts), np.concatenate(vs)


def test_rotation_and_index_match_the_protocol_decoder(session, cache):
    _, s, _ = session
    files = rotated_files(s, "wbc")
    assert len(files) > 1                                   # rotated: later files repeat the schema
    idx = StreamIndex.open("wbc", files)
    assert idx.kind == "rows" and len(idx.schemas) == 1
    fields = ["base_x", "arm_tgt_x", "armed"]
    t_all, v_all = brute_rows(files, fields)
    a, b = T0 + 7.13, T0 + 19.4
    t, v = idx.rows(a, b, fields)
    m = (t_all >= a) & (t_all <= b)
    np.testing.assert_array_equal(t, t_all[m])
    np.testing.assert_allclose(v, v_all[m])
    assert idx.t_range == (pytest.approx(t_all.min()), pytest.approx(t_all.max()))
    t2, v2 = idx.rows(a, b, ["base_x", "no_such_field"], stride=3)
    np.testing.assert_array_equal(t2, t_all[m][::3])
    assert np.all(np.isnan(v2[:, 1]))
    t3, _ = idx.rows(T0 - 10, T0 - 5, fields)
    assert len(t3) == 0


def test_index_cache_reopens_memory_mapped(session, cache):
    _, s, _ = session
    files = rotated_files(s, "arm")
    a = StreamIndex.open("arm", files)
    assert any(cache.rglob("meta.json"))
    b = StreamIndex.open("arm", files)
    assert isinstance(b.a["t0"], np.memmap)
    np.testing.assert_array_equal(np.asarray(a.a["poff"]), np.asarray(b.a["poff"]))
    t1, v1 = a.rows(T0 + 1, T0 + 3, ["j1_q", "g_gap"])
    t2, v2 = b.rows(T0 + 1, T0 + 3, ["j1_q", "g_gap"])
    np.testing.assert_array_equal(v1, v2)


def test_frames_by_time(session, cache):
    _, s, _ = session
    idx = StreamIndex.open("camera.color", rotated_files(s, "camera.color"))
    assert idx.kind == "frames"
    n = int(SECONDS * 10)
    assert idx.n_records == n and bool(np.all(np.asarray(idx.a["jpeg"])))
    i = idx.at(T0 + 12.34, "frame")
    assert np.asarray(idx.a["t0"])[i] == pytest.approx(T0 + 12.3)    # the newest frame at or before
    assert idx.payload(i)[:2] == b"\xff\xd8"
    assert idx.header(i)["seq"] == 123
    w = idx.window(T0 + 5, T0 + 6, "frame")
    assert len(w) == 11
    assert idx.at(T0 - 1) == -1


def test_oplog_tables_in_operator_axes(session, cache):
    _, s, _ = session
    rec = Recording(s)
    hs, sent = rec.streams["oplog.headset"], rec.streams["oplog.sent"]
    assert hs.n_records == int(SECONDS * 72) and sent.n_records == hs.n_records
    # the fake headset's head starts at the LOCAL origin facing forward: FLU position 0, orientation = C
    t, v = hs.rows(T0, T0 + 0.5, ["head_x", "head_y", "head_z", "head_qx", "head_qy", "head_qz", "head_qw"])
    np.testing.assert_allclose(v[0, :3], 0, atol=1e-5)
    np.testing.assert_allclose(matrix_from_quat(v[0, 3:]), C, atol=1e-5)
    # the right controller's default pose (operator FLU (0.35, -0.2, -0.35)) comes back in FLU
    _, r = hs.rows(T0, T0 + 0.5, ["right_x", "right_y", "right_z"])
    np.testing.assert_allclose(r[0], [0.35, -0.2, -0.35], atol=1e-5)
    _, m = sent.rows(T0, T0 + SECONDS, ["base_mode", "armed"])
    assert set(np.unique(m[:, 0])) <= {1.0, 3.0} and np.all(m[:, 1] == 1)
    kinds = {e["kind"] for e in rec.oplog.events}
    assert {"episode_start", "episode_stop", "button"} <= kinds


def test_anchors_reconstructed_exactly(session, cache):
    _, s, truth = session
    rec = Recording(s)
    got = [a for a in rec.oplog.anchors() if a["hand"] == "right"]
    assert len(got) == len(truth["anchors"]) >= 1
    for a, tr in zip(got, truth["anchors"]):
        assert a["accepted"]
        assert a["t_start"] == pytest.approx(tr["t"])
        np.testing.assert_allclose(a["robot_frame"], tr["robot_frame"], atol=1e-9)
        np.testing.assert_allclose(a["ee_anchor"], tr["ee_anchor"], atol=1e-9)
        head = np.asarray(tr["head16"], float).reshape(4, 4)
        np.testing.assert_allclose(a["operator"]["origin"], C @ head[:3, 3], atol=1e-5)
        hand = np.asarray(tr["hand16"], float).reshape(4, 4)
        np.testing.assert_allclose(a["hand_world"][:3], C @ hand[:3, 3], atol=1e-5)
        # the hand anchor in the operator frame: Rz(-yaw) (p_hand - p_head), as frames.OperatorFrame.pose
        yaw = a["operator"]["yaw"]
        rz = np.array([[np.cos(-yaw), -np.sin(-yaw), 0], [np.sin(-yaw), np.cos(-yaw), 0], [0, 0, 1]])
        np.testing.assert_allclose(a["hand_anchor"][:3], rz @ (C @ hand[:3, 3] - C @ head[:3, 3]), atol=1e-5)


def test_anchor_matches_the_teleop_clients_capture(session, cache):
    """The operator frame and hand anchor agree with tools/quest_teleop's own capture on the same frame."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
    from quest_teleop import frames as F
    _, s, truth = session
    rec = Recording(s)
    a = next(x for x in rec.oplog.anchors() if x["hand"] == "right")
    tr = truth["anchors"][0]
    hp, hr = F.pose_from_matrix16(tr["head16"])
    op = F.OperatorFrame.from_head(hp, hr)
    p, r = op.pose(*F.pose_from_matrix16(tr["hand16"]))
    assert a["operator"]["yaw"] == pytest.approx(op.yaw, abs=1e-6)
    np.testing.assert_allclose(a["hand_anchor"][:3], p, atol=1e-5)
    np.testing.assert_allclose(matrix_from_quat(a["hand_anchor"][3:]), r, atol=1e-5)


def test_library_and_cut_episodes(session, cache):
    root, s, truth = session
    eps = cut_session(s, pre_roll=0.5, post_roll=0.5)
    assert len(eps) == len(truth["episodes"]) >= 2
    lib = Library(root)
    items = lib.items()
    keys = [i["key"] for i in items]
    assert "s1" in keys and any(k.startswith("s1/episodes/") for k in keys)
    ses = next(i for i in items if i["key"] == "s1")
    assert [e["id"] for e in ses["episodes"]] == [e["id"] for e in truth["episodes"]]
    ep = lib.open(f"s1/episodes/{eps[0]['id']}")
    assert ep.kind == "episode"
    lo, hi = ep.t_range
    assert lo >= eps[0]["window"][0] - 1e-9 and hi <= eps[0]["window"][1] + 1e-9
    t, v = ep.streams["wbc"].rows(lo, hi, ["base_x"])
    assert len(t) and t.min() >= lo
    summ = ep.summary()
    assert summ["episodes"][0]["id"] == eps[0]["id"] and "camera.color" in summ["streams"]
    with pytest.raises(KeyError):
        lib.open("../../etc")


def test_model_chain_matches_the_arm_node():
    kin = pytest.importorskip("robot_utils.nodes.arm.kin")
    from robot_utils.collect.browse.model import arm_chain, chain_fk
    chain, how = arm_chain()
    if chain is None:
        pytest.skip(how)
    k = kin.ArmKinematics()
    q = np.array([0.3, -0.5, 0.8, 0.2, -0.4, 1.1])
    for a, b in zip(chain_fk(chain, q), k.frames(q)):
        np.testing.assert_allclose(a[:3, 3], b.translation, atol=1e-9)
        np.testing.assert_allclose(a[:3, :3], b.rotation, atol=1e-9)


def test_description_package_default():
    from robot_utils.collect.browse.model import load_description
    d, how = load_description(None, None)
    assert d and "arm" in d["frames_in_base"] and "package default" in how


def test_estimate_and_encoder_args():
    s = X.Settings(width=1280, height=720, fps=30, target_mb=10)
    e = X.estimate(s, 50.0)
    assert e["bytes"] == 10e6
    assert e["bitrate"] == pytest.approx(8 * 10e6 * (1 - X.CONTAINER_OVERHEAD) / 50.0)
    q = X.estimate(X.Settings(crf=23), 10.0)
    q29 = X.estimate(X.Settings(crf=29), 10.0)
    assert q29["bytes"] == pytest.approx(q["bytes"] / 2)                # halves every +6 CRF
    m = X.estimate(X.Settings(width=640, height=360, fps=10), 10.0, bpp=0.1)
    assert m["bitrate"] == pytest.approx(0.1 * 640 * 360 * 10) and m["how"] == "measured"
    assert X.estimate(X.Settings(width=1920, height=1080, fps=60, target_mb=0.5), 60)["warning"]
    a = X.video_args(X.Settings(), {"libx264"}, bitrate=2e6, pass_no=1, passlog="/tmp/p")
    assert a[:2] == ["-c:v", "libx264"] and "-pass" in a and "2000000" in a
    a = X.video_args(X.Settings(preset="small"), {"libx265"})
    assert "libx265" in a and "-crf" in a
    a = X.video_args(X.Settings(preset="draft"), {"libx264", "h264_videotoolbox"})
    assert "h264_videotoolbox" in a
    assert X.Settings(width=1281, height=721).resolved().width == 1280


def test_camera_frame_plan(session, cache):
    _, s, _ = session
    idx = StreamIndex.open("camera.color", rotated_files(s, "camera.color"))
    plan = list(X.camera_frames(idx, T0 + 2, T0 + 6, fps=30, rate=1.0))
    assert sum(n for _, n in plan) == int(4 * 30) + 1
    assert all(n in (2, 3, 4) for _, n in plan[1:-1])                   # 10 fps camera into 30 fps video
    fast = list(X.camera_frames(idx, T0 + 2, T0 + 6, fps=10, rate=4.0))
    assert sum(n for _, n in fast) == 11 and len(fast) == 11           # every 4th frame at 4x


@pytest.mark.skipif(X.find_ffmpeg() is None, reason="no ffmpeg binary")
def test_encode_to_a_target_size(session, cache, tmp_path):
    _, s, _ = session
    idx = StreamIndex.open("camera.color", rotated_files(s, "camera.color"))
    ex = X.Exporter(X.find_ffmpeg(), tmp_path / "out")
    st = X.Settings(width=320, height=240, fps=20, preset="balanced", target_mb=0.25)
    plan = list(X.camera_frames(idx, T0, T0 + 20, st.fps))
    job = ex.create(st, sum(n for _, n in plan), "t1")
    for i, n in plan:
        ex.add_frame(job.id, idx.payload(i), n)
    ex.finish(job.id, wait=True)
    assert job.state == "done", job.error
    assert job.out.exists() and job.result["passes"] == 2
    assert abs(job.result["target_error"]) < 0.08
    with pytest.raises(ValueError):
        job2 = ex.create(st, 1, "t2")
        ex.add_frame(job2.id, b"not a jpeg")


def test_http_api(session, cache):
    from fastapi.testclient import TestClient
    from robot_utils.collect.browse.server import build_app
    root, s, _ = session
    app = build_app(Library(root), X.Exporter(None, root / "exports"))
    c = TestClient(app)
    assert c.get("/").status_code == 200 and "Episode browser" in c.get("/").text
    assert c.get("/common/viz.js").status_code == 200
    items = c.get("/api/library").json()["items"]
    assert any(i["key"] == "s1" for i in items)
    rec = c.get("/api/recording", params={"key": "s1"}).json()
    assert rec["streams"]["camera.color"]["frames"] == int(SECONDS * 10)
    json.dumps(rec, allow_nan=False)                                      # strict JSON: no NaN
    r = c.get("/api/rows", params={"key": "s1", "stream": "wbc", "fields": "base_x,armed", "t0": T0 + 1, "t1": T0 + 2})
    buf = r.content
    hl = struct.unpack(">I", buf[:4])[0]
    assert (4 + hl) % 8 == 0
    h = json.loads(buf[4:4 + hl])
    n = h["n"]
    t = np.frombuffer(buf, "<f8", n, 4 + hl)
    v = np.frombuffer(buf, "<f4", n * 2, 4 + hl + 8 * n).reshape(n, 2)
    assert n == 51 and np.all(np.diff(t) > 0) and np.all(v[:, 1] == 1)
    fr = c.get("/api/frames", params={"key": "s1", "stream": "camera.color", "t0": T0, "t1": T0 + 1}).json()
    assert len(fr["i"]) == 11
    j = c.get("/api/frame", params={"key": "s1", "stream": "camera.color", "i": fr["i"][3]})
    assert j.headers["content-type"] == "image/jpeg" and j.content[:2] == b"\xff\xd8"
    assert "immutable" in j.headers["cache-control"]
    env = c.get("/api/envelope", params={"key": "s1", "stream": "wbc", "field": "base_x", "t0": T0, "t1": T0 + 10,
                                         "buckets": 20}).json()
    assert len(env["min"]) == 20 and all(lo <= hi for lo, hi in zip(env["min"], env["max"]))
    assert c.get("/api/rows", params={"key": "s1", "stream": "nope", "t0": 0, "t1": 1}).status_code == 404
    assert c.get("/api/recording", params={"key": "../x"}).status_code == 404
    assert c.post("/api/export", json={"settings": {}, "frames": 1}).status_code == 409        # no ffmpeg


def test_recorded_description_wins(tmp_path):
    """A session's own description.json (collect run since 6f5a844) beats --description and the package default,
    for the session and its cut episodes; model_version comes from session.json."""
    from robot_utils.collect.browse.model import load_description, recorded_model
    s = tmp_path / "s"
    (s / "episodes" / "e1").mkdir(parents=True)
    (s / "description.json").write_text(json.dumps({"frames_in_base": {"arm": [[1, 0, 0, 0]] * 4}, "tag": "rec"}))
    (s / "session.json").write_text(json.dumps({"description": {"model_version": "abc123", "options": {"mounts": "calibrated"}}}))
    other = tmp_path / "other.json"
    other.write_text(json.dumps({"tag": "flag"}))
    for d in (s, s / "episodes" / "e1"):
        desc, how = load_description(str(other), d)
        assert desc["tag"] == "rec" and "abc123" in how
        assert recorded_model(d)[1]["model_version"] == "abc123"
    desc, how = load_description(str(other), tmp_path / "old")        # not recorded: the flag
    assert desc["tag"] == "flag" and "not recorded" in how
