"""Mirror pairing (tools/quest_teleop/body_frozen.py, "Mirror pairing"): with two hands, the hand-to-arm pairing is
decided when the first hand engages from all released (auto: mirrored when the operator's and the robot's headings
are more than 90 deg apart) and locked until both hands are released. Only the pairing changes: the EE motions stay
world-relative, the trigger follows the paired arm, nothing jumps. Pure: the fake headset's frames, the gate, a fake
WBC row per arm that tracks the targets exactly (the command = the last target)."""
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from quest_teleop import frames as F  # noqa: E402
from quest_teleop.body_frozen import MIRROR_MODES, Config, MultiCore, RobotState  # noqa: E402
from quest_teleop.fake_headset import Headset, HeadsetState  # noqa: E402
from quest_teleop.quest_stream import Frame, QuestGate  # noqa: E402

HANDS = {"left": "arm_left", "right": "arm_right"}
ROBOT_YAW = 0.3

IDLE = HeadsetState()


def facing(s, yaw):
    return replace(s, head_yaw=yaw)


def grip(s, hand, g=1.0, **kw):
    return replace(s, **{hand: replace(getattr(s, hand), grip=g, **kw)})


def move(s, hand, d):
    h = getattr(s, hand)
    return replace(s, **{hand: replace(h, pos=tuple(np.add(h.pos, d)))})


class Rig:
    def __init__(self, mirror="off", **kw):
        self.hs, self.gate = Headset(), QuestGate(rate_window=0.0, recover_s=0.0)
        self.core = MultiCore(Config(hand="right", ee="arm_right", hands=dict(HANDS), mirror=mirror, **kw))
        self.t = 100.0
        self.rob = {"arm_left": RobotState(0.0, np.array([1.5, 2.4, 0.8]), F.rot_axis((1, 1, 0), 0.7), 1.0, 2.0,
                                           ROBOT_YAW, 0.0),
                    "arm_right": RobotState(0.0, np.array([1.6, 1.7, 0.9]), F.rot_axis((0, 1, 1), -0.4), 1.0, 2.0,
                                            ROBOT_YAW, 0.0)}
        self.missing: set = set()           # arms with no stream row
        self.cmd = None
        self.events: list = []

    def robot(self):
        return {e: (None if e in self.missing else r) for e, r in self.rob.items()}

    def frame(self, s, track=True):
        self.t += 0.01
        d = self.hs.frame(s, self.t)
        for r in self.rob.values():
            r.t_recv = self.t
        self.core.on_frame(d, self.gate.check(Frame(d, self.t, 1)), self.robot(), self.t)
        self.events += self.core.events
        self.core.events.clear()
        self.cmd = self.core.command(self.t, self.gate.fresh(self.t), self.robot())
        if track and isinstance(self.cmd, dict):       # the WBC tracks exactly: its command is the target
            for e, ee in self.cmd["ees"].items():
                if e in self.rob:
                    self.rob[e].ee_p = np.array(ee["pos"])
                    self.rob[e].ee_r = F.matrix_from_quat(ee["quat"])
        self.last = d
        return self.cmd

    def frames(self, s, n=5):
        for _ in range(n):
            self.frame(s)
        return self.cmd

    def arm(self, s=IDLE):
        """Released frames, then Y: the registration is made here (the operator faces where the robot faces)."""
        self.frames(s, 3)
        self.core.on_arm(self.last)
        return self.frames(s, 3)

    def pos(self, ee):
        return np.array(self.cmd["ees"][ee]["pos"])


def test_facing_the_robot_mirrors_behind_it_straight():
    rig = Rig("auto")
    rig.arm()
    rig.frames(facing(IDLE, math.pi), 3)                       # turned round: facing the robot
    before = {e: rig.pos(e) for e in HANDS.values()}
    rig.frame(grip(facing(IDLE, math.pi), "left"))
    assert rig.core.mirrored and rig.core.pairing == {"left": "arm_right", "right": "arm_left"}
    assert rig.core.cores["left"].eng is not None and rig.core.engaged == ["left"]
    for e in HANDS.values():                                   # no jump at engage or swap
        np.testing.assert_allclose(rig.pos(e), before[e], atol=1e-9)
    (kind, d), = rig.events
    assert kind == "pairing" and d["pairing"] == "mirror" and d["mode"] == "auto" and d["hand"] == "left"
    assert abs(abs(d["heading_diff_deg"]) - 180.0) < 0.5
    rig2 = Rig("auto")
    rig2.arm()
    rig2.frames(facing(IDLE, 0.5), 3)                          # behind the robot (29 deg off): straight
    rig2.frame(grip(facing(IDLE, 0.5), "left"))
    assert not rig2.core.mirrored and rig2.core.pairing == HANDS
    assert rig2.events[-1][1]["pairing"] == "straight" and abs(rig2.events[-1][1]["heading_diff_deg"] - 28.6) < 0.5
    rig3 = Rig("auto")
    rig3.arm()
    rig3.frame(grip(facing(IDLE, 1.7), "left"))                 # 97 deg: past the threshold
    assert rig3.core.mirrored


def test_unregistered_or_first_person_is_straight():
    rig = Rig("auto")                                          # never armed: no registration until this clutch
    rig.frames(facing(IDLE, math.pi), 3)
    rig.frame(grip(facing(IDLE, math.pi), "right"))
    assert not rig.core.mirrored and rig.events[-1][1]["heading_diff_deg"] == 0.0
    rig = Rig("auto", mode="first")
    rig.arm()
    rig.frames(facing(IDLE, math.pi), 3)
    rig.frame(grip(facing(IDLE, math.pi), "right"))
    assert not rig.core.mirrored


def test_locked_while_any_hand_is_engaged_and_re_decided_after_both_release():
    rig = Rig("auto")
    rig.arm()
    s = grip(facing(IDLE, math.pi), "left")
    rig.frames(facing(IDLE, math.pi), 2)
    rig.frame(s)
    assert rig.core.mirrored
    for k in range(1, 11):                                     # turning back mid-engagement: still mirrored
        rig.frame(facing(s, math.pi * (1 - k / 10)))
    s = facing(s, 0.0)
    assert rig.core.mirrored and len(rig.events) == 1
    s = grip(s, "right")                                       # the second hand: the pairing is already locked
    rig.frames(s, 3)
    assert rig.core.mirrored and rig.core.cores["right"].eng is not None
    assert rig.core.cores["right"].cfg.ee == "arm_left" and len(rig.events) == 1
    s = grip(s, "left", 0.0)                                   # one hand released: the lock holds
    rig.frames(s, 3)
    assert rig.core.engaged == ["right"] and rig.core.mirrored and rig.core.pairing_info()["locked"]
    s = grip(s, "left", 1.0)                                   # and engaging it again does not re-decide
    rig.frames(s, 3)
    assert rig.core.mirrored and len(rig.events) == 1 and rig.core.cores["left"].eng is not None
    s = grip(grip(s, "left", 0.0), "right", 0.0)               # both released: unlocked, the pairing stays
    rig.frames(s, 3)
    assert not rig.core.engaged and rig.core.mirrored and not rig.core.pairing_info()["locked"]
    rig.frame(grip(s, "right"))                                # re-engaging (now behind the robot): straight
    assert not rig.core.mirrored and rig.core.pairing == HANDS and len(rig.events) == 2
    assert rig.events[-1][1]["pairing"] == "straight"


def test_ee_motion_unchanged_and_grippers_follow_the_pairing():
    out = {}
    for mode in ("straight", "mirror"):
        rig = Rig(mode)
        rig.arm(facing(IDLE, math.pi))
        s = grip(facing(IDLE, math.pi), "left", trigger=0.25)
        s = replace(s, right=replace(s.right, trigger=0.9))
        rig.frame(s)
        ee = rig.core.pairing["left"]
        start = rig.pos(ee)
        for k in range(1, 21):                                 # the left hand reaches 10 cm forward, 5 cm up
            rig.frame(move(s, "left", (-0.005 * k, 0.0, 0.0025 * k)))
        out[mode] = (ee, rig.pos(ee) - start)
        g = {e: rig.cmd["ees"][e]["gripper"] for e in HANDS.values()}
        assert g[ee] == pytest.approx(0.75)                    # the left trigger -> its paired arm
        assert g[rig.core.pairing["right"]] == pytest.approx(0.1)
        other = rig.core.pairing["right"]                      # released: rides where it was
        np.testing.assert_allclose(rig.pos(other), {"arm_left": [1.5, 2.4, 0.8],
                                                    "arm_right": [1.6, 1.7, 0.9]}[other], atol=1e-9)
    assert out["straight"][0] == "arm_left" and out["mirror"][0] == "arm_right"
    np.testing.assert_allclose(out["mirror"][1], out["straight"][1], atol=1e-9)   # the same world motion
    assert np.linalg.norm(out["mirror"][1]) > 0.1


def test_a_swap_while_released_keeps_the_riding_arms_still():
    rig = Rig("auto")
    rig.arm()
    s = grip(IDLE, "left")                                     # straight: move the left arm, then release it
    rig.frame(s)
    for k in range(1, 11):
        rig.frame(move(s, "left", (0.01 * k, 0.0, 0.0)))
    rig.frames(move(IDLE, "left", (0.1, 0.0, 0.0)), 3)
    assert not rig.core.engaged and not rig.core.mirrored
    held = {e: rig.pos(e) for e in HANDS.values()}
    assert np.linalg.norm(held["arm_left"] - [1.5, 2.4, 0.8]) > 0.05
    t = facing(move(IDLE, "left", (0.1, 0.0, 0.0)), math.pi)
    rig.frames(t, 3)
    for e in HANDS.values():
        np.testing.assert_allclose(rig.pos(e), held[e], atol=1e-9)
    rig.frame(grip(t, "right"))                                # facing: mirrored; the right hand drives arm_left
    assert rig.core.mirrored and rig.core.cores["right"].cfg.ee == "arm_left"
    for e in HANDS.values():                                   # both arms where they were (the engaged one too)
        np.testing.assert_allclose(rig.pos(e), held[e], atol=1e-9)
    rig.frames(grip(t, "right"), 5)
    for e in HANDS.values():
        np.testing.assert_allclose(rig.pos(e), held[e], atol=1e-9)


def test_a_missing_paired_arm_refuses_and_keeps_the_pairing():
    rig = Rig("mirror")
    rig.arm()
    rig.missing = {"arm_right"}
    rig.frame(grip(IDLE, "left"))                              # left -> arm_right: no row, refused
    assert not rig.core.engaged and rig.core.mirrored and not rig.events
    assert rig.core.status.refused.get("no fresh WBC state")
    rig2 = Rig("auto")
    rig2.arm()
    rig2.missing = {"arm_right"}
    rig2.frames(facing(IDLE, math.pi), 2)
    rig2.frame(grip(facing(IDLE, math.pi), "left"))
    assert not rig2.core.engaged and not rig2.core.mirrored and not rig2.events   # reverted, no event


def test_the_toggle_is_refused_while_engaged():
    rig = Rig("off")
    rig.arm()
    with pytest.raises(ValueError):
        rig.core.set_mirror("sideways")
    assert rig.core.set_mirror("mirror")["pairing"] == "mirror"          # released: forced modes apply at once
    assert rig.core.pairing == {"left": "arm_right", "right": "arm_left"}
    rig.frame(grip(IDLE, "right"))
    with pytest.raises(ValueError, match="release both"):
        rig.core.set_mirror("off")
    assert rig.core.cfg.mirror == "mirror" and rig.core.mirrored
    rig.frames(IDLE, 2)
    assert rig.core.set_mirror("off")["pairing"] == "straight" and rig.core.pairing == HANDS
    assert rig.core.set_mirror("auto")["mode"] == "auto"
    assert set(MIRROR_MODES) == {"off", "auto", "straight", "mirror"}


def test_off_is_the_old_behaviour():
    """off never decides: no event, straight even facing the robot, and the same commands as forced straight (whose
    only addition is the event)."""
    script = ([IDLE] * 3 + [facing(IDLE, math.pi)] * 3 + [grip(facing(IDLE, math.pi), "left", trigger=0.4)]
              + [move(grip(facing(IDLE, math.pi), "left", trigger=0.4), "left", (-0.01 * k, 0.01 * k, 0.0))
                 for k in range(10)] + [facing(IDLE, math.pi)] * 3)
    cmds = {}
    for mode in ("off", "straight"):
        rig = Rig(mode)
        rig.arm()
        cmds[mode] = [rig.frame(s) for s in script]
        assert not rig.core.mirrored
        assert (rig.core.decision is None) if mode == "off" else (rig.core.decision["pairing"] == "straight")
        assert bool(rig.events) == (mode == "straight")
    assert cmds["off"] == cmds["straight"]


def test_mirror_flag():
    rig = Rig("mirror")
    info = rig.core.pairing_info()
    assert info["pairing"] == "mirror" and info["mirror"] == {"arm_left": True, "arm_right": True}
    assert "elbow_sign" not in info and rig.core.cores["left"].mirrored   # the swivel needs no pairing sign
    hs = rig.core.hands_status()
    assert hs["left"]["ee"] == "arm_right" and hs["left"]["mirrored"]
    rig.core.set_mirror("straight")
    assert not rig.core.pairing_info()["mirror"]["arm_left"] and not rig.core.cores["right"].mirrored
    with pytest.raises(ValueError):
        MultiCore(Config(hands={"right": "arm"}, mirror="bogus"))
    single = MultiCore(Config(hands={"right": "arm"}))
    with pytest.raises(ValueError, match="left and a right"):
        single.set_mirror("auto")


# -- the node: the option, the toggle, /state and the event ---------------------------------------------------


def test_node_toggle_state_and_event(tmp_path):
    from types import SimpleNamespace

    from robot_utils.nodes.common.lease import RequestError
    from robot_utils.nodes.quest_teleop.node import QuestTeleopNode
    node = QuestTeleopNode("http://127.0.0.1:9", quest_host="127.0.0.1", quest_port=9,
                           cfg=Config(hand="right", ee="arm_right", hands=dict(HANDS)))
    st = node.state()
    assert st["config"]["mirror"] == "off" and st["teleop"]["pairing"]["pairing"] == "straight"
    with pytest.raises(ValueError):
        node.set_mirror("sideways")
    out = node.set_mirror("mirror")                            # not running: the configured pairing
    assert out["mirror"] == "mirror" and out["pairing"]["hands"] == {"left": "arm_right", "right": "arm_left"}
    assert out["pairing"]["mirror"] == {"arm_left": True, "arm_right": True}
    core = MultiCore(node.cfg)                                 # running: the client's core (shares node.cfg)
    node.client = SimpleNamespace(core=core)
    assert node.pairing_info()["pairing"] == "mirror"
    core.cores["left"].eng = object()                          # a hand engaged: refused, nothing changed
    with pytest.raises(RequestError) as e:
        node.set_mirror("auto")
    assert e.value.status == 409 and "release both" in str(e.value)
    assert node.cfg.mirror == "mirror" and node.pairing_info()["locked"]
    core.cores["left"].eng = None
    assert node.set_mirror("auto")["mirror"] == "auto" and node.cfg.mirror == "auto"
    node._on_client_event(("pairing", {"pairing": "mirror", "mode": "auto", "heading_diff_deg": -178.0, "hand": "left",
                                       "hands": {"left": "arm_right", "right": "arm_left"}}))
    assert node.eventlog.last["kind"] == "pairing" and node.eventlog.last["detail"]["heading_diff_deg"] == -178.0
    assert "178 deg apart" in str(node.events.list()[-1])
    one = QuestTeleopNode("http://127.0.0.1:9", quest_host="127.0.0.1", quest_port=9)
    with pytest.raises(RequestError):
        one.set_mirror("auto")
    assert one.set_mirror("off")["pairing"] is None
