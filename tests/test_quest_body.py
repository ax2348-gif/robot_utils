"""Body tracking forwarded by the headset (quest_env BODY_TRACKING.md; tools/quest_teleop/body.py): the schema and
its validation, the native app's own output (the golden fixture from quest_env's test_body_tracking.py), the reader
with and without a body block, the fake headset's skeleton, no change to teleop, the node's /state summary, the
operator stream (what recording saves) and the replay."""
import copy
import json
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from quest_teleop import body as B  # noqa: E402
from quest_teleop import frames as F  # noqa: E402
from quest_teleop.body_frozen import Config, RobotState, TeleopCore  # noqa: E402
from quest_teleop.fake_headset import FakeHeadsetServer, Headset, HeadsetState  # noqa: E402
from quest_teleop.quest_stream import Frame, QuestGate, QuestReader  # noqa: E402

FIXTURE = Path(__file__).parent / "data" / "quest_body_frames.jsonl"   # quest_env fixtures/body_frames.jsonl
IDLE = HeadsetState()
GRIP = replace(IDLE, right=replace(IDLE.right, grip=1.0))
REACH = replace(GRIP, right=replace(GRIP.right, pos=(0.45, -0.2, -0.30)), head_yaw=0.2)


def fixture():
    return {r["case"]: r for r in map(json.loads, FIXTURE.read_text().splitlines())}


# -- the schema ----------------------------------------------------------------------------------------------

def test_joint_names_follow_the_openxr_enums():
    assert B.JOINT_IDS["left_arm_lower"] == 11 and B.JOINT_IDS["right_arm_lower"] == 16
    assert B.JOINT_IDS["left_hand_palm"] == 18 and B.JOINT_IDS["right_hand_wrist"] == 45
    assert B.JOINT_IDS["right_foot_ball"] == 83
    assert len(B.core_ids(70)) == 22 and len(B.core_ids(84)) == 36


def test_the_native_output_parses():
    """The block exactly as native/body_tracking.h writes it (golden fixture shared with quest_env)."""
    recs = fixture()
    b = B.parse_body(recs["default_core"])
    assert b.available and b.active and b.joint_set == "default" and b.joint_count == 70 and b.subset == "core"
    assert b.ids.tolist() == B.core_ids(70) and b.poses.shape == (22, 7)
    assert b.fidelity == "high" and b.confidence == 0.875 and b.skeleton_changed_count == 3
    assert b.time_ns == 123456789012345
    p, q, flags = b.joint("left_elbow")
    np.testing.assert_allclose(p, [0.11, 1.5 - 0.22, -0.011], atol=1e-4)
    assert flags == 15 and abs(np.linalg.norm(q) - 1) < 1e-3
    assert b.valid_count == 22 and b.tracked_count == 22 - sum(1 for i in B.core_ids(70) if i % 5 == 4)
    assert B.parse_body(recs["default_all"]).ids.size == 70
    full = B.parse_body(recs["full_body_core"])
    assert full.joint_set == "full_body" and full.joint_count == 84 and full.joint("right_foot_ball") is not None
    assert full.joint("left_hand_index_tip") is None                       # a finger: not in core
    inactive = B.parse_body(recs["inactive"])
    assert inactive.available and not inactive.active and inactive.ids.size == 0 and inactive.fidelity is None
    nonfinite = B.parse_body(recs["nonfinite"])
    assert nonfinite.joint(11)[2] == 0 and nonfinite.fidelity_requested is None
    off = B.parse_body(recs["unavailable"])
    assert not off.available and "not granted" in off.reason
    assert off.summary() == {"available": False, "reason": off.reason}


def test_encode_parse_round_trip():
    rng = np.random.default_rng(3)
    joints = {}
    for i in B.core_ids(84):
        q = rng.normal(size=4)
        joints[i] = (rng.normal(size=3), F.matrix_from_quat(q / np.linalg.norm(q)), 15 if i % 3 else 3)
    blk = json.loads(json.dumps(B.encode(joints, joint_set="full_body", confidence=0.5, fidelity="low",
                                         time_ns=42)))
    b = B.parse_block(blk)
    assert b.joint_set == "full_body" and b.ids.tolist() == B.core_ids(84) and b.fidelity == "low"
    for i, (p, r, f) in joints.items():
        bp, bq, bf = b.joint(i)
        np.testing.assert_allclose(bp, p, atol=6e-5)
        np.testing.assert_allclose(F.matrix_from_quat(bq), r, atol=5e-4)
        assert bf == f
    blk = B.encode({}, active=False)
    assert B.parse_block(blk).active is False and blk["joints"] == []
    missing = B.parse_block(B.encode({0: (np.zeros(3), np.eye(3), 15)}))
    assert missing.joint(0)[2] == 15 and missing.joint("left_elbow")[2] == 0     # unsent by the runtime: flags 0
    assert B.parse_block(B.unavailable("no permission")).reason == "no permission"


def _bad(mutate):
    blk = copy.deepcopy(fixture()["default_core"]["body"])
    mutate(blk)
    return blk


@pytest.mark.parametrize("mutate", [
    lambda b: b.update(schema="teleop.something"),
    lambda b: b.update(available="yes"),
    lambda b: b.update(active=None),
    lambda b: b.update(joint_set="legs"),
    lambda b: b.update(joint_count=84),
    lambda b: b.update(joint_set_id=5),
    lambda b: b.update(fidelity="ultra"),
    lambda b: b.update(joints=b["joints"][:-1]),
    lambda b: b.update(flags=b["flags"] + [15]),
    lambda b: b.update(poses=b["poses"][:-1]),
    lambda b: b["joints"].__setitem__(1, 0),                  # not ascending
    lambda b: b["joints"].__setitem__(-1, 70),                # outside the set
    lambda b: b["poses"].__setitem__(3, "x"),
    lambda b: b["poses"].__setitem__(3, float("nan")),
    lambda b: b["flags"].__setitem__(0, True),
    lambda b: b.update(confidence="high"),
])
def test_malformed_blocks_raise(mutate):
    with pytest.raises(B.BodyError):
        B.parse_block(_bad(mutate))


def test_absent_or_newer_blocks_are_ignored():
    assert B.parse_body({"schema": "teleop.quest_state.v1"}) is None
    assert B.parse_body({"body": {"schema": "teleop.quest_body.v2", "anything": []}}) is None
    with pytest.raises(B.BodyError):
        B.parse_body({"body": []})


# -- the fake headset --------------------------------------------------------------------------------------

def test_the_fake_headset_sends_a_plausible_upper_body():
    hs = Headset(body="core")
    f = json.loads(json.dumps(hs.frame(REACH, 1.0)))
    b = B.parse_body(f)
    assert b.active and b.tracked_count == 22 and b.time_ns == 1_000_000_000
    head_p, _ = F.pose_from_matrix16(f["head"]["pose"])
    right_p, _ = F.pose_from_matrix16(f["hands"]["right"]["pose"])
    sh, el, wr = (b.joint(n)[0] for n in ("right_shoulder", "right_elbow", "right_hand_wrist"))
    np.testing.assert_allclose(wr, right_p, atol=2e-4)                           # the wrist is at the controller
    assert abs(np.linalg.norm(el - sh) - B.UPPER_ARM) < 1e-3 and abs(np.linalg.norm(wr - el) - B.FOREARM) < 1e-3
    assert el[1] < sh[1] and el[1] < head_p[1]                                  # LOCAL +Y up: the elbow hangs
    ls, rs = b.joint("left_shoulder")[0], b.joint("right_shoulder")[0]
    assert abs(np.linalg.norm(ls - rs) - 0.36) < 1e-3 and ls[1] < head_p[1]     # neck-worn: shoulders below it
    for _, q, _ in (b.joint(i) for i in b.ids):
        assert abs(np.linalg.norm(q) - 1) < 1e-3
    # all joints, and an untracked head: an inactive block
    assert B.parse_body(Headset(body="all").frame(IDLE, 0.0)).ids.size == 70
    assert not B.parse_body(hs.frame(replace(IDLE, head_tracked=False), 1.1)).active
    assert "body" not in Headset().frame(IDLE, 0.0)


# -- the reader, the gate and teleop: with and without the block ---------------------------------------------

def _read(headset: Headset, n: int, *, tamper=None):
    srv = FakeHeadsetServer()
    reader = QuestReader("127.0.0.1", srv.port).start()
    try:
        deadline = time.monotonic() + 5
        while srv.n_clients == 0 and time.monotonic() < deadline:
            time.sleep(0.01)
        for k in range(n):
            f = headset.frame(IDLE, 1.0 + k / 72)
            srv.send(tamper(f) if tamper else f)
        got = []
        while len(got) < n and time.monotonic() < deadline:
            fr = reader.get(timeout=0.5)
            if fr is not None:
                got.append(fr)
        return got, reader
    finally:
        reader.close()
        srv.close()


def test_the_reader_parses_the_block_and_leaves_frames_without_one_alone():
    got, reader = _read(Headset(), 5)
    assert len(got) == 5 and all(f.body is None for f in got) and reader.counts["body"] == 0
    got, reader = _read(Headset(body="core"), 5)
    assert len(got) == 5 and all(f.body is not None and f.body.active for f in got)
    assert reader.counts["body"] == 5 and reader.counts["body_bad"] == 0
    assert "body" in got[0].data                                  # the frame itself is unchanged


def test_a_malformed_block_is_counted_and_the_frame_still_delivered():
    def tamper(f):
        f["body"]["joints"] = f["body"]["joints"][:-1]
        return f
    got, reader = _read(Headset(body="core"), 4, tamper=tamper)
    assert len(got) == 4 and all(f.body is None for f in got)
    assert reader.counts["body_bad"] == 4 and "joints" in reader.last_body_error


class Rig:
    """A headset, the gate and the core on a hand-advanced clock (as test_quest_teleop's)."""

    def __init__(self, body=None):
        self.hs = Headset(body=body)
        self.gate = QuestGate(rate_window=0.0, recover_s=0.0)
        self.core = TeleopCore(Config())
        self.t = 100.0
        self.rob = RobotState(0.0, np.array([1.5, 2.1, 0.8]), F.rot_axis((1, 1, 0), 0.7), 1.0, 2.0, 0.3, 0.0)

    def run(self, s, n=3):
        for _ in range(n):
            self.t += 0.01
            d = self.hs.frame(s, self.t)
            self.rob.t_recv = self.t
            self.core.on_frame(d, self.gate.check(Frame(d, self.t, 1, B.parse_body(d))), self.rob, self.t)
        return self.core.command(self.t, self.gate.fresh(self.t), self.rob)


def test_teleop_commands_are_identical_with_and_without_the_block():
    plain, withbody = Rig(), Rig("all")
    for s in (IDLE, GRIP, REACH, REACH, IDLE, replace(IDLE, focused=False), IDLE, GRIP):
        a, b = plain.run(s), withbody.run(s)
        assert json.dumps(a, sort_keys=True, default=str) == json.dumps(b, sort_keys=True, default=str)
        assert plain.core.status.state == withbody.core.status.state


# -- the node: /state summary, the operator stream ------------------------------------------------------------

def test_node_state_summarises_the_body_and_the_operator_stream_records_it(tmp_path):
    import robot_utils.nodes.quest_teleop.client  # noqa: F401,PLC0415 (tools/ on the path)
    from robot_utils.nodes.quest_teleop.node import HeadsetMonitor   # noqa: PLC0415
    from robot_utils.nodes.quest_teleop.oplog import OpLog            # noqa: PLC0415

    class FakeReader:
        connected, counts, last_error, last_body_error = True, {"connects": 1, "body_bad": 2}, None, "bad joints"

        def __init__(self):
            self.q = []

        def get(self, timeout=None):
            return self.q.pop(0) if self.q else None

        def drain(self):
            out, self.q = self.q, []
            return out

    reader, clock = FakeReader(), [10.0]
    mon = HeadsetMonitor(reader, clock=lambda: clock[0])
    oplog = OpLog()
    sub = oplog.subscribe()
    mon.on_frame = oplog.headset
    assert mon.snapshot(10.0)["body"] == {"present": False, "age": None, "frames": 0, "bad": 2,
                                         "last_error": "bad joints"}
    hs = Headset(body="core")
    d = hs.frame(REACH, 10.0)
    reader.q.append(Frame(d, 10.0, 1, B.parse_body(d)))
    mon.drain()
    s = mon.snapshot(10.25)["body"]
    assert s["present"] and s["available"] and s["active"] and s["age"] == 0.25 and s["tracked"] == 22
    assert s["fidelity"] == "high" and s["joint_set"] == "default" and s["sent"] == 22
    d2 = Headset().frame(IDLE, 10.3)                                  # body off again: the old one ages out
    reader.q.append(Frame(d2, 10.3, 1, None))
    mon.drain()
    assert not mon.snapshot(11.5)["body"]["present"] and mon.snapshot(11.5)["body"]["age"] == 1.5
    items, _ = oplog.take(sub)
    assert [i["type"] for i in items] == ["headset", "headset"]
    assert items[0]["frame"]["body"] == d["body"] and "body" not in items[1]["frame"]   # recorded as received


# -- replay: recordings with the block load and render as before ------------------------------------------------

def test_replay_counts_body_frames_and_ignores_them(tmp_path):
    from replay_fakes import write_session   # noqa: PLC0415
    from robot_utils.collect.records import iter_raw   # noqa: PLC0415
    from robot_utils.nodes.common.protocol import pack_record   # noqa: PLC0415
    from robot_utils.replay.log import load   # noqa: PLC0415
    from robot_utils.replay.operator_view import OperatorView, ViewOptions   # noqa: PLC0415

    plain, body = tmp_path / "plain", tmp_path / "body"
    write_session(plain, seconds=0.5)
    write_session(body, seconds=0.5)
    path = body / "quest_teleop.oplog.000.bin"
    recs = list(iter_raw([path]))
    blk = B.encode({i: (p, r, 15) for i, (p, r) in B.plausible_skeleton((0, 0, 1.6), 0.0, {}).items()},
                   time_ns=1)                                       # FLU poses: fine for this test
    with open(path, "wb") as f:
        for _, h, _ in recs:
            if h.get("type") == "headset":
                h["frame"]["body"] = blk
            f.write(pack_record(h))
    a, b = load(plain), load(body)
    assert a.body_frames == 0 and b.body_frames == len(b.headset) > 0
    va, vb = OperatorView(a, ViewOptions(width=96)), OperatorView(b, ViewOptions(width=96))
    assert vb.summary["body_frames"] == b.body_frames
    for (ia, _), (ib, _) in zip(list(va.frames())[:5], list(vb.frames())[:5]):
        assert np.array_equal(ia, ib)
