"""Two hands, two arms (tools/quest_teleop/body_frozen.py ``MultiCore``): the Quest client between the fake headset
and a fake two-arm WBC (rows with arm_left_* / arm_right_*). Each grip clutches its own arm, each trigger is its own
gripper, a released arm rides; the base follows the first-engaged hand, then the other when it releases. Real
sockets, real time (about 4 s). Also the node's hand mapping from the robot profile."""
import sys
import threading
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from helpers import serve, stop, wait_for
from quest_teleop_fakes import FakeWbc

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from quest_teleop.body_frozen import Config, MultiCore, TeleopCore, make_core  # noqa: E402
from quest_teleop.fake_headset import FakeHeadsetServer, HeadsetState, mirror_scenario, play  # noqa: E402
from quest_teleop.quest_stream import QuestReader  # noqa: E402
from quest_teleop.teleop import TeleopClient, close_wbc, default_hints, open_wbc  # noqa: E402
from quest_teleop.wbc_client import WbcClient, WbcStream  # noqa: E402

HANDS = {"left": "arm_left", "right": "arm_right"}
EE0 = np.array([1.52, 2.1, 0.8])                     # the fake WBC's commanded EE (both arms start there)

IDLE = HeadsetState()
Y = replace(IDLE, left=replace(IDLE.left, secondary=True))
R = replace(IDLE, right=replace(IDLE.right, grip=1.0))                                    # right engaged
R_MOVE = replace(R, right=replace(R.right, pos=(0.45, -0.2, -0.35)))                    # right reaches 10 cm
RL = replace(R_MOVE, left=replace(R_MOVE.left, grip=1.0))                                # left engages too
RL_MOVE = replace(RL, left=replace(RL.left, pos=(0.35, 0.2, -0.25), trigger=0.6))       # left lifts 10 cm, squeezes
L_ONLY = replace(RL_MOVE, right=replace(RL_MOVE.right, grip=0.0))                        # right releases
NONE = replace(L_ONLY, left=replace(L_ONLY.left, grip=0.0))
LEAD = 0.5
SCENARIO = [(0.0, IDLE), (LEAD + 0.2, IDLE), (LEAD + 0.21, Y), (LEAD + 0.3, Y), (LEAD + 0.31, IDLE),
            (LEAD + 0.6, IDLE), (LEAD + 0.61, R), (LEAD + 1.0, R), (LEAD + 1.5, R_MOVE), (LEAD + 1.7, R_MOVE),
            (LEAD + 1.71, RL), (LEAD + 2.0, RL), (LEAD + 2.5, RL_MOVE), (LEAD + 2.7, RL_MOVE), (LEAD + 2.71, L_ONLY),
            (LEAD + 3.1, L_ONLY), (LEAD + 3.11, NONE), (LEAD + 3.4, NONE)]


def test_make_core_and_hints():
    assert isinstance(make_core(Config()), TeleopCore)
    mc = make_core(Config(hand="right", ee="arm_right", hands=HANDS))
    assert isinstance(mc, MultiCore) and mc.ees == ["arm_left", "arm_right"]
    assert mc.cores["left"].cfg.hand == "left" and mc.cores["left"].cfg.ee == "arm_left"
    cfg = mc.cfg
    cfg.mode = "first"                                  # the page's switches reach both hands
    assert mc.cores["left"].cfg.mode == "first" and mc.cores["right"].cfg.mode == "first"
    assert set(default_hints(cfg)["ees"]) == {"arm_left", "arm_right"}


def test_hand_mapping_from_the_robot():
    from robot_utils.nodes.quest_teleop.__main__ import hand_mapping
    assert hand_mapping(["arm"]) == ("right", "arm", None)                      # the single-arm robot as before
    assert hand_mapping(["arm"], "left") == ("left", "arm", None)
    assert hand_mapping(["arm_left", "arm_right"]) == ("right", "arm_right", HANDS)   # two hands by default
    assert hand_mapping(["arm_left", "arm_right"], "right") == ("right", "arm_right", None)   # one hand: arm_right
    assert hand_mapping(["arm_left", "arm_right"], None, "arm_left") == ("right", "arm_left", None)
    from robot_utils import robots
    arms = robots.load("rebot_bimanual").arms
    assert hand_mapping(arms) == ("right", "arm_right", HANDS)               # by the profile's side
    assert [a.side for a in arms] == ["left", "right"] and robots.load("rebot").arms[0].side is None
    swapped = [type(a)(**{**a.__dict__, "side": {"left": "right", "right": "left"}[a.side]}) for a in arms]
    assert hand_mapping(swapped)[2] == {"left": "arm_right", "right": "arm_left"}   # side, not the name
    assert hand_mapping(["arm_left", "arm_right"], None, None, "left=arm_right,right=arm_left") == \
        ("right", "arm_left", {"left": "arm_right", "right": "arm_left"})


@pytest.fixture
def wbc_peer():
    fake = FakeWbc(ees=("arm_left", "arm_right"))
    url, srv, th = serve(fake.app)
    yield fake, url
    stop((srv, th))
    fake.close()


def test_two_hands_drive_their_own_arms(wbc_peer):
    fake, url = wbc_peer
    fake.row["state"] = 2.0
    headset = FakeHeadsetServer()
    reader = QuestReader("127.0.0.1", headset.port, reconnect_s=0.05).start()
    stream = WbcStream(url).start()
    wbc = WbcClient(url, udp_host="127.0.0.1")
    client = TeleopClient(Config(hand="right", ee="arm_right", hands=HANDS), reader, stream, wbc)
    owners = []                                          # (time, the hand the base follows, engaged hands)
    client.on_step = lambda now, fresh, why: owners.append((now, client.core.owner, tuple(client.core.engaged)))
    halt = threading.Event()
    try:
        info = open_wbc(wbc, client.cfg)
        client.session_opened(info)
        assert set(fake.calls[2][1]["ees"]) == {"arm_left", "arm_right"}     # session hints for both EEs
        wait_for(lambda: stream.latest()[0] is not None and headset.n_clients == 1)
        loop = threading.Thread(target=client.run, args=(halt,), daemon=True)
        loop.start()
        play(headset, SCENARIO, rate=100)
        wait_for(lambda: len(fake.datagrams) > 0)
    finally:
        halt.set()
        close_wbc(wbc, log=lambda s: None)
        reader.close()
        stream.close()
        wbc.close()
        headset.close()

    msgs = [m for _, m in fake.datagrams if m["type"] == "target"]
    assert msgs and all(set(m["ees"]) == {"arm_left", "arm_right"} for m in msgs)     # every target: both arms
    pos = lambda m, e: np.array(m["ees"][e]["pos"])                                     # noqa: E731
    # right engaged alone: it moves its arm (10 cm forward in the operator frame = robot forward, yaw 0.3), the left
    # arm rides (the fake base is still: at its command); the base follows the right hand (a hint)
    right_only = [m for m in msgs if m["base"]["mode"] == "hint" and np.linalg.norm(pos(m, "arm_left") - EE0) < 1e-6]
    assert right_only
    m = right_only[-1]
    assert np.linalg.norm(pos(m, "arm_right") - EE0) == pytest.approx(0.10, abs=0.01)
    assert "gripper" in m["ees"]["arm_right"] and m["ees"]["arm_right"]["gripper"] == pytest.approx(1.0)
    # both engaged: the left lifts its arm 10 cm and squeezes its gripper; the right keeps its reach
    both = [m for m in msgs if pos(m, "arm_left")[2] > EE0[2] + 0.099 and np.linalg.norm(pos(m, "arm_right") - EE0) > 0.09]
    assert both
    m = both[-1]
    np.testing.assert_allclose(pos(m, "arm_left"), EE0 + [0, 0, 0.10], atol=0.01)
    assert np.linalg.norm(pos(m, "arm_right") - EE0) == pytest.approx(0.10, abs=0.01)
    assert m["ees"]["arm_left"]["gripper"] == pytest.approx(0.4, abs=0.02)
    assert m["ees"]["arm_right"]["gripper"] == pytest.approx(1.0)
    assert m["base"]["mode"] == "hint"
    # the base: the right hand (first engaged) while both are, then the left once the right releases, then sticks
    seq = [o for _, o, eng in owners if eng]
    order = [o for i, o in enumerate(seq) if i == 0 or seq[i - 1] != o]
    assert order == ["right", "left"], order
    assert any(o == "right" and set(eng) == {"left", "right"} for _, o, eng in owners)
    assert msgs[-1]["base"]["mode"] == "locked"          # both released: the sticks (idle)
    # released, the right arm rides at its command (the fake WBC's command did not move)
    np.testing.assert_allclose(pos(msgs[-1], "arm_right"), EE0, atol=1e-6)


def test_mirror_auto_over_the_fake_headset(wbc_peer):
    """``--mirror auto`` end to end (fake_headset ``mirror_scenario``): facing the robot, the right grip drives the
    left arm and the right trigger its gripper; turning back mid-engagement keeps it; after the release, facing the
    robot's way, the right grip drives the right arm again. One ``pairing`` event per decision."""
    fake, url = wbc_peer
    fake.row["state"] = 2.0
    headset = FakeHeadsetServer()
    reader = QuestReader("127.0.0.1", headset.port, reconnect_s=0.05).start()
    stream = WbcStream(url).start()
    wbc = WbcClient(url, udp_host="127.0.0.1")
    client = TeleopClient(Config(hand="right", ee="arm_right", hands=HANDS, mirror="auto"), reader, stream, wbc)
    events, pairs = [], []
    client.on_event = events.append
    client.on_step = lambda now, fresh, why: pairs.append((client.core.mirrored, tuple(client.core.engaged)))
    halt = threading.Event()
    try:
        client.session_opened(open_wbc(wbc, client.cfg))
        wait_for(lambda: stream.latest()[0] is not None and headset.n_clients == 1)
        loop = threading.Thread(target=client.run, args=(halt,), daemon=True)
        loop.start()
        play(headset, mirror_scenario(0.5)[:-1] + [(7.2, mirror_scenario(0.5)[-2][1])], rate=100)
        wait_for(lambda: len(fake.datagrams) > 0)
    finally:
        halt.set()
        close_wbc(wbc, log=lambda s: None)
        reader.close()
        stream.close()
        wbc.close()
        headset.close()

    decisions = [e[1] for e in events if e[0] == "pairing"]
    assert [d["pairing"] for d in decisions] == ["mirror", "straight"], decisions
    assert abs(decisions[0]["heading_diff_deg"]) > 170 and abs(decisions[1]["heading_diff_deg"]) < 5
    assert decisions[0]["hands"] == {"left": "arm_right", "right": "arm_left"} and decisions[0]["hand"] == "right"
    engaged_pairs = {m for m, eng in pairs if eng}
    assert engaged_pairs == {True, False}            # mirrored for the first engagement, straight for the second
    msgs = [m for _, m in fake.datagrams if m["type"] == "target"]
    pos = lambda m, e: np.array(m["ees"][e]["pos"])                                     # noqa: E731
    mirrored = [m for m in msgs if np.linalg.norm(pos(m, "arm_left") - EE0) > 0.09]
    assert mirrored                                  # the right hand reached: the left arm moved, the right rode
    m = mirrored[0]
    np.testing.assert_allclose(pos(m, "arm_right"), EE0, atol=1e-6)
    assert min(x["ees"]["arm_left"]["gripper"] for x in mirrored) == pytest.approx(0.3, abs=0.02)  # the right trigger
    assert all(x["ees"]["arm_right"]["gripper"] == pytest.approx(1.0) for x in mirrored)   # the left trigger: open
    assert msgs[0]["ees"]["arm_left"]["pos"] == pytest.approx(list(EE0), abs=1e-6)   # nothing jumped
    straight = [m for m in msgs if np.linalg.norm(pos(m, "arm_right") - EE0) > 0.09]
    assert straight and m is not straight[0] and msgs.index(straight[0]) > msgs.index(m)
