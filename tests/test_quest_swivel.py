"""The elbow-swivel hint's teleop half (tools/quest_teleop/swivel.py): per engaged hand, the operator's swivel change
since the engagement (the elbow about the shoulder -> wrist axis, parallel transport), signed by the pairing, sent
as the paired EE's ``swivel`` object when ``Config.elbow_hint`` is on. The fake headset's scripted elbow
(``HeadsetState.elbow_swivel``) turns the elbow about that axis by a known angle. Pure rigs, then the node and one
run over real sockets."""
import math
import sys
import threading
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from helpers import serve, stop, wait_for
from quest_teleop_fakes import FakeWbc

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from quest_teleop import body as B  # noqa: E402
from quest_teleop import frames as F  # noqa: E402
from quest_teleop import swivel as SW  # noqa: E402
from quest_teleop.body_frozen import Config, MultiCore, RobotState, TeleopCore  # noqa: E402
from quest_teleop.fake_headset import FakeHeadsetServer, Headset, HeadsetState, elbow_scenario, play  # noqa: E402
from quest_teleop.quest_stream import Frame, QuestGate  # noqa: E402

HANDS = {"left": "arm_left", "right": "arm_right"}
IDLE = HeadsetState()
# the right shoulder in operator FLU for the fake's neck-worn skeleton at the origin (body.plausible_skeleton)
R_SHOULDER = np.array([-0.03, -0.18, -0.13])


def grip(s, hand="right", g=1.0):
    return replace(s, **{hand: replace(getattr(s, hand), grip=g)})


def swivel(s, psi, side="right"):
    sw = list(s.elbow_swivel)
    sw[0 if side == "left" else 1] = psi
    return replace(s, elbow_swivel=tuple(sw))


def hand_at(s, pos, hand="right"):
    return replace(s, **{hand: replace(getattr(s, hand), pos=tuple(float(x) for x in pos))})


class Rig:
    """The fake headset's frames (with a body block unless ``body=None``) through the gate into a core; a fake WBC
    row per arm that tracks the targets exactly."""

    def __init__(self, body="core", hands=None, mutate=None, **kw):
        self.hs, self.gate = Headset(body=body), QuestGate(rate_window=0.0, recover_s=0.0)
        if hands:
            self.core = MultiCore(Config(hand="right", ee="arm_right", hands=dict(hands), **kw))
            ees = list(hands.values())
        else:
            self.core = TeleopCore(Config(**kw))
            ees = ["arm"]
        self.rob = {e: RobotState(0.0, np.array([1.5, 2.1 + 0.3 * i, 0.8]), F.rot_axis((0, 0, 1), 0.2), 1.0, 2.0, 0.3,
                                  0.0) for i, e in enumerate(ees)}
        self.single = not hands
        self.mutate = mutate                 # (frame dict) -> None: edit the body block before parsing
        self.t = 100.0
        self.cmds: list = []

    def robot(self):
        return self.rob["arm"] if self.single else dict(self.rob)

    def frame(self, s):
        self.t += 0.01
        d = self.hs.frame(s, self.t)
        if self.mutate is not None:
            self.mutate(d)
        for r in self.rob.values():
            r.t_recv = self.t
        f = Frame(d, self.t, 1, body=B.parse_body(d))
        self.core.on_frame(d, self.gate.check(f), self.robot(), self.t, f.body)
        cmd = self.core.command(self.t, self.gate.fresh(self.t), self.robot())
        if isinstance(cmd, dict):
            for e, ee in cmd["ees"].items():
                self.rob[e].ee_p = np.array(ee["pos"])
                self.rob[e].ee_r = F.matrix_from_quat(ee["quat"])
        self.cmds.append(cmd)
        return cmd

    def frames(self, s, n=5):
        for _ in range(n):
            self.frame(s)
        return self.cmds[-1]

    def ramp(self, a, b, n=30):
        """Interpolate the states' elbow swivel (and the hand) from ``a`` to ``b`` over ``n`` frames."""
        for i in range(1, n + 1):
            u = i / n
            sw = tuple(x + (y - x) * u for x, y in zip(a.elbow_swivel, b.elbow_swivel))
            pos = tuple(x + (y - x) * u for x, y in zip(a.right.pos, b.right.pos))
            self.frame(replace(b, elbow_swivel=sw, right=replace(b.right, pos=pos)))
        return self.cmds[-1]

    def arm(self, s=IDLE):
        self.frames(s, 3)
        self.core.on_arm(self.hs.frame(s, self.t))
        return self.frames(s, 3)


def sw_of(cmd, ee="arm"):
    return cmd["ees"][ee].get("swivel")


def test_dpsi_sign_and_magnitude_straight():
    """Swivelling the right elbow +0.6 rad about the shoulder -> wrist axis (right-handed) gives dpsi -0.6 (the right
    arm's side sign: positive is out), and -0.4 gives +0.4, with the hand still; the elbow angle and the confidence
    are the body's."""
    rig = Rig(elbow_hint=True)
    rig.arm()
    g = grip(IDLE)
    c = rig.frames(g, 3)
    s0 = sw_of(c)
    assert s0 is not None and s0["dpsi"] == 0.0 and s0["conf"] == pytest.approx(0.9)
    sk = B.plausible_skeleton((0, 0, 0), 0.0, {"left": np.array(IDLE.left.pos), "right": np.array(IDLE.right.pos)})
    sh, el, wr = (sk[B.JOINT_IDS[f"right_{j}"]][0] for j in ("arm_upper", "arm_lower", "hand_wrist"))
    u, v = sh - el, wr - el
    assert s0["elbow"] == pytest.approx(math.acos(u @ v / np.linalg.norm(u) / np.linalg.norm(v)), abs=2e-3)
    c = rig.ramp(g, swivel(g, 0.6))
    assert sw_of(c)["dpsi"] == pytest.approx(-0.6, abs=3e-3) and sw_of(c)["seq"] == s0["seq"]
    assert sw_of(c)["elbow"] == pytest.approx(s0["elbow"], abs=2e-3)    # a swivel keeps the elbow angle
    c = rig.ramp(swivel(g, 0.6), swivel(g, -0.4), 40)
    assert sw_of(c)["dpsi"] == pytest.approx(0.4, abs=3e-3)
    info = rig.core.swivel_info()
    assert info["valid"] and info["sending"] and info["enabled"] and not info["straight"]


def test_the_pairing_does_not_change_the_sign():
    """Two hands: the same right-elbow motion is sent with the same dpsi whether the right hand drives arm_right
    (straight) or arm_left (mirrored): positive is out for every arm, so out maps to out (mirrored: its mirror
    image) with no pairing sign; the released arm carries no swivel. The left arm's right-handed angle is not
    negated (its side sign is +1)."""
    for mode, ee in (("straight", "arm_right"), ("mirror", "arm_left")):
        rig = Rig(hands=HANDS, mirror=mode, elbow_hint=True)
        rig.arm()
        g = grip(IDLE)
        rig.frames(g, 3)
        c = rig.ramp(g, swivel(g, 0.5))
        assert rig.core.pairing["right"] == ee
        assert sw_of(c, ee)["dpsi"] == pytest.approx(-0.5, abs=3e-3)
        other = ({"arm_left", "arm_right"} - {ee}).pop()
        assert "swivel" not in c["ees"][other]
        info = rig.core.swivel_info()
        assert info[ee]["hand"] == "right" and "elbow_sign" not in info[ee] and info[ee]["sending"]
        assert not info[other]["engaged"] and not info[other]["sending"] and info[other]["dpsi"] is None
    rig = Rig(hands=HANDS, elbow_hint=True)
    rig.arm()
    g = grip(IDLE, "left")
    rig.frames(g, 3)
    c = rig.ramp(g, swivel(g, -0.7, "left"))
    assert sw_of(c, "arm_left")["dpsi"] == pytest.approx(-0.7, abs=3e-3)


def _outward_raw(side: str) -> float:
    """The fake skeleton's right-handed swivel (rad) that moves ``side``'s elbow out, away from the body's midline,
    found from the geometry (not assumed)."""
    hands = {"left": np.array(IDLE.left.pos), "right": np.array(IDLE.right.pos)}
    sgn = 1.0 if side == "left" else -1.0          # FLU: +y is the operator's left
    elbow = B.JOINT_IDS[f"{side}_arm_lower"]
    lateral = {r: sgn * B.plausible_skeleton((0, 0, 0), 0.0, hands, swivel={side: r})[elbow][0][1]
               for r in (-0.5, 0.0, 0.5)}
    assert lateral[0.5] != lateral[-0.5]
    return 0.5 if lateral[0.5] > lateral[0.0] else -0.5


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("mode", ["straight", "mirror"])
def test_elbow_out_is_positive_for_either_arm_and_pairing(side, mode):
    """The WBC's convention (wbc/swivel_geom.py: +1 left, -1 right): the operator's elbow moving out (away from the
    body) is a positive dpsi on whichever arm that hand drives, straight or mirrored."""
    raw = _outward_raw(side)
    rig = Rig(hands=HANDS, mirror=mode, elbow_hint=True)
    rig.arm()
    g = grip(IDLE, side)
    rig.frames(g, 3)
    c = rig.ramp(g, swivel(g, raw, side))
    ee = rig.core.pairing[side]
    assert ee == HANDS[side] if mode == "straight" else ee != HANDS[side]
    assert sw_of(c, ee)["dpsi"] == pytest.approx(0.5, abs=3e-3)


@pytest.mark.parametrize("down", [True, False])
def test_the_vertical_arm_is_not_singular(down):
    """The arm hanging straight down (or raised straight up), bent: the swivel about the vertical axis is measured
    like any other, through more than a half turn and back (no fixed reference direction)."""
    rig = Rig(elbow_hint=True)
    wrist = R_SHOULDER + np.array([0.0, 0.0, -0.45 if down else 0.45])
    s = hand_at(IDLE, wrist)
    rig.arm(s)
    g = grip(s)
    rig.frames(g, 3)
    sk = B.plausible_skeleton((0, 0, 0), 0.0, {"left": None, "right": wrist})
    np.testing.assert_allclose(sk[B.JOINT_IDS["right_arm_upper"]][0], R_SHOULDER, atol=1e-9)   # the axis is vertical
    c = rig.ramp(g, swivel(g, 2.6), 60)                          # the right arm: dpsi is minus the right-handed angle
    assert sw_of(c)["dpsi"] == pytest.approx(-2.6, abs=5e-3)
    c = rig.ramp(swivel(g, 2.6), swivel(g, 4.0), 40)            # past pi: the sum is unbounded
    assert sw_of(c)["dpsi"] == pytest.approx(-4.0, abs=5e-3)
    c = rig.ramp(swivel(g, 4.0), swivel(g, -1.0), 120)
    assert sw_of(c)["dpsi"] == pytest.approx(1.0, abs=5e-3)


def test_a_new_seq_per_engagement():
    """Each engagement gets a new seq and its dpsi restarts at zero; the two hands' engagements never share one."""
    rig = Rig(hands=HANDS, elbow_hint=True)
    rig.arm()
    g = grip(IDLE)
    rig.frames(g, 3)
    c = rig.ramp(g, swivel(g, 0.5))
    seq1 = sw_of(c, "arm_right")["seq"]
    rel = grip(swivel(g, 0.5), g=0.0)
    c = rig.frames(rel, 3)
    assert "swivel" not in c["ees"]["arm_right"] and rig.core.swivel_info()["arm_right"]["seq"] is None
    g2 = grip(rel)
    c = rig.frames(g2, 3)
    seq2 = sw_of(c, "arm_right")["seq"]
    assert seq2 > seq1 and sw_of(c, "arm_right")["dpsi"] == 0.0
    c = rig.frames(grip(g2, "left"), 3)                          # the left engages on top: its own seq
    seq3 = sw_of(c, "arm_left")["seq"]
    assert seq3 > seq2 and sw_of(c, "arm_right")["seq"] == seq2
    s = SW.SeqCounter()                                          # clock-seeded, strictly increasing
    assert s.next(5.0) == 5000 and s.next(5.0) == 5001 and s.next(4.0) == 5002 and s.next(9.0) == 9000


def test_omitted_without_a_valid_body():
    """No body block, an inactive body, or an arm joint not valid: no swivel (the rest of the target unchanged).
    Inferred (valid, not tracked) joints are sent at the body's confidence (Meta never marks controller-driven arms
    tracked). When the body comes back
    within the engagement the change continues (bridged), not restarted."""
    g = grip(IDLE)
    rig = Rig(body=None, elbow_hint=True)
    rig.arm()
    c = rig.frames(g, 5)
    assert c["ees"]["arm"].get("swivel") is None and rig.core.swivel_info()["reason"] == "no body block"

    rig = Rig(elbow_hint=True)
    rig.arm()
    rig.frames(g, 3)
    c = rig.frames(replace(g, body_active=False), 3)
    assert sw_of(c) is None and rig.core.swivel_info()["reason"] == "body inactive"

    broken = {"on": False, "flags": 0}

    def mutate(d):
        if broken["on"]:
            b = d["body"]
            i = b["joints"].index(B.JOINT_IDS["right_arm_lower"])
            b["flags"][i] = broken["flags"]
    rig = Rig(elbow_hint=True, mutate=mutate)
    rig.arm()
    rig.frames(g, 3)
    rig.ramp(g, swivel(g, 0.3), 15)
    broken["on"] = True
    c = rig.ramp(swivel(g, 0.3), swivel(g, 0.5), 10)            # the elbow not valid: nothing sent, dpsi held
    assert sw_of(c) is None and rig.core.swivel_info()["reason"] == "right_arm_lower not valid"
    assert rig.core.swivel_info()["dpsi"] == pytest.approx(-0.3, abs=3e-3)    # the right arm: minus
    broken["flags"] = B.VALID                                    # inferred, not tracked
    c = rig.frames(swivel(g, 0.5), 1)
    assert sw_of(c)["dpsi"] == pytest.approx(-0.5, abs=3e-3)    # bridged across the gap
    assert sw_of(c)["conf"] == pytest.approx(0.9)               # not halved for an inferred joint
    broken["on"] = False
    assert sw_of(rig.frames(swivel(g, 0.5), 1))["conf"] == pytest.approx(0.9)


def test_a_straight_arm_holds_the_change():
    """The elbow on the shoulder -> wrist axis (arm straight): no swivel to measure, dpsi held (sent, the elbow
    angle near pi tells the WBC); bending again continues from the held value."""
    rig = Rig(elbow_hint=True)
    rig.arm()
    g = grip(IDLE)
    rig.frames(g, 3)
    rig.ramp(g, swivel(g, 0.4), 15)
    far = R_SHOULDER + 0.5699 * np.array([1.0, 0.0, 0.0])        # full reach, straight forward
    c = rig.ramp(swivel(g, 0.4), hand_at(swivel(g, 0.4), far), 30)
    assert sw_of(c)["elbow"] > math.radians(170) and rig.core.swivel_info()["straight"]
    held = sw_of(c)["dpsi"]
    c = rig.ramp(hand_at(swivel(g, 0.4), far), swivel(g, 0.4), 30)
    assert not rig.core.swivel_info()["straight"] and abs(sw_of(c)["dpsi"] - held) < 0.5


def test_toggle_off_sends_todays_datagrams():
    """elbow_hint off (the default): the same datagrams with or without a body block, the old behaviour; on: the
    same datagrams plus the swivel objects. On applies from the next grip, off at once."""
    def run(body, **kw):
        rig = Rig(body=body, hands=HANDS, **kw)
        rig.arm()
        g = grip(grip(IDLE), "left")
        rig.frames(g, 3)
        rig.ramp(g, swivel(swivel(g, 0.4), -0.3, "left"), 20)
        rig.frames(grip(g, g=0.0), 3)
        return rig.cmds
    assert Config().elbow_hint is False
    base = run(None)
    assert run("core") == base and run("core", elbow_hint=False) == base
    on = run("core", elbow_hint=True)
    stripped = [{**c, "ees": {e: {k: v for k, v in x.items() if k != "swivel"} for e, x in c["ees"].items()}}
                if isinstance(c, dict) else c for c in on]
    assert stripped == base and any("swivel" in x for c in on if isinstance(c, dict) for x in c["ees"].values())

    rig = Rig()                                                  # switched on while engaged: from the next grip
    rig.arm()
    g = grip(IDLE)
    rig.frames(g, 3)
    rig.core.cfg.elbow_hint = True
    c = rig.frames(swivel(g, 0.2), 3)
    assert sw_of(c) is None and rig.core.swivel_info()["pending"]
    rig.frames(grip(g, g=0.0), 2)
    c = rig.frames(g, 3)
    assert sw_of(c) is not None and sw_of(c)["dpsi"] == 0.0
    rig.core.cfg.elbow_hint = False                              # off: at once
    assert sw_of(rig.frames(g, 1)) is None and not rig.core.swivel_info()["sending"]


def test_the_robot_swivel_from_the_wbc_stream():
    """``<ee>_swivel`` / ``<ee>_swivel_w`` in the WBC row are shown with the input (absent: None)."""
    row = {"arm_cmd_x": 1.0, "arm_cmd_y": 0.0, "arm_cmd_z": 0.5, "arm_cmd_qx": 0.0, "arm_cmd_qy": 0.0,
           "arm_cmd_qz": 0.0, "arm_cmd_qw": 1.0, "base_x": 0.0, "base_y": 0.0, "base_yaw": 0.0}
    r = RobotState.from_row(row, 0.0, "arm")
    assert r.swivel is None and r.swivel_w is None
    r = RobotState.from_row({**row, "arm_swivel": 0.25, "arm_swivel_w": 0.8}, 0.0, "arm")
    assert (r.swivel, r.swivel_w) == (0.25, 0.8)
    assert RobotState.from_row({**row, "arm_swivel": float("nan")}, 0.0, "arm").swivel is None
    core = TeleopCore(Config())
    core.last_robot = r
    assert core.swivel_info()["robot_swivel"] == 0.25 and core.swivel_info()["robot_swivel_w"] == 0.8


def test_node_option_toggle_and_state(tmp_path):
    """The node: config.elbow_hint, POST /elbow_hint/{on,off} (an event), /state teleop.swivel per EE."""
    from fastapi.testclient import TestClient

    from robot_utils.nodes.quest_teleop.node import QuestTeleopNode
    from robot_utils.nodes.quest_teleop.server import create_app
    node = QuestTeleopNode("http://127.0.0.1:9", quest_host="127.0.0.1", quest_port=9,
                           cfg=Config(hand="right", ee="arm_right", hands=dict(HANDS)))
    st = node.state()
    assert st["config"]["elbow_hint"] is False and st["teleop"]["swivel"] is None
    c = TestClient(create_app(node, start=False))               # no lifespan: the node is never started
    assert c.post("/elbow_hint/maybe").status_code == 400
    assert c.post("/elbow_hint/on").json()["elbow_hint"] is True
    assert node.cfg.elbow_hint and "elbow hint: on" in str(node.events.list()[-1])
    node.client = SimpleNamespace(core=MultiCore(node.cfg))
    sw = node.swivel_info()                                     # /state teleop.swivel
    assert set(sw) == {"arm_left", "arm_right"} and sw["arm_right"]["hand"] == "right"
    assert sw["arm_right"]["enabled"] and not sw["arm_right"]["valid"] and not sw["arm_right"]["sending"]
    assert node.set_elbow_hint(False)["elbow_hint"] is False
    one = QuestTeleopNode("http://127.0.0.1:9", quest_host="127.0.0.1", quest_port=9)
    one.client = SimpleNamespace(core=TeleopCore(one.cfg))
    assert set(one.swivel_info()) == {"arm"}


@pytest.fixture
def wbc_peer():
    fake = FakeWbc()
    fake.fields += ["arm_swivel", "arm_swivel_w"]
    fake.row.update(arm_swivel=0.3, arm_swivel_w=0.7)
    url, srv, th = serve(fake.app)
    yield fake, url
    stop((srv, th))
    fake.close()


def test_elbow_scenario_over_the_fake_headset(wbc_peer):
    """End to end (fake_headset ``elbow_scenario``, ``--body core``; the reader parses the body): the right elbow
    swivels +0.6 (right-handed) then back to +0.2 in the first engagement and -0.4 in the second; the datagrams carry
    the swivel object with those changes, signed positive-out (the right arm: negated), under two seqs, and the client sees the WBC's swivel columns."""
    from quest_teleop.quest_stream import QuestReader
    from quest_teleop.teleop import TeleopClient, close_wbc, open_wbc
    from quest_teleop.wbc_client import WbcClient, WbcStream
    fake, url = wbc_peer
    fake.row["state"] = 2.0
    headset = FakeHeadsetServer()
    reader = QuestReader("127.0.0.1", headset.port, reconnect_s=0.05).start()
    stream = WbcStream(url).start()
    wbc = WbcClient(url, udp_host="127.0.0.1")
    client = TeleopClient(Config(elbow_hint=True), reader, stream, wbc)
    halt = threading.Event()
    lead = 0.5
    try:
        client.session_opened(open_wbc(wbc, client.cfg))
        wait_for(lambda: stream.latest()[0] is not None and headset.n_clients == 1)
        loop = threading.Thread(target=client.run, args=(halt,), daemon=True)
        loop.start()
        sc = elbow_scenario(lead)
        play(headset, sc[:-1] + [(lead + 6.3, sc[-2][1])], rate=100, headset=Headset(body="core"))
        wait_for(lambda: len(fake.datagrams) > 0)
        info = client.core.swivel_info()
    finally:
        halt.set()
        close_wbc(wbc, log=lambda s: None)
        reader.close()
        stream.close()
        wbc.close()
        headset.close()
    sws = [m["ees"]["arm"]["swivel"] for _, m in fake.datagrams if m["type"] == "target" and "swivel" in m["ees"]["arm"]]
    seqs = list(dict.fromkeys(s["seq"] for s in sws))
    assert len(seqs) == 2, seqs
    first = [s["dpsi"] for s in sws if s["seq"] == seqs[0]]
    second = [s["dpsi"] for s in sws if s["seq"] == seqs[1]]
    # the scenario turns the right elbow right-handed; positive-out flips the right arm's sign
    assert first[0] == pytest.approx(0.0, abs=1e-3) and min(first) == pytest.approx(-0.6, abs=0.01)
    assert first[-1] == pytest.approx(-0.2, abs=0.01)
    assert second[0] == pytest.approx(0.0, abs=1e-3) and second[-1] == pytest.approx(0.4, abs=0.01)
    assert info["robot_swivel"] == pytest.approx(0.3) and info["robot_swivel_w"] == pytest.approx(0.7)
