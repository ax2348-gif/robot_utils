"""The Quest whole-body teleop client (tools/quest_teleop): frame math, the body-frozen scheme, the remote-consumer
rules and the base/gripper commands. Pure: no sockets (the end-to-end test is test_quest_teleop_e2e.py)."""
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from quest_teleop import frames as F  # noqa: E402
from quest_teleop.body_frozen import Config, RobotState, TeleopCore, body_frozen_target, capture  # noqa: E402
from quest_teleop.fake_headset import Headset, HeadsetState, head_r_op, to_local  # noqa: E402
from quest_teleop.quest_stream import Frame, QuestGate  # noqa: E402

RNG = np.random.default_rng(7)


def consumer_gate():
    """The consumer rules without the latency / rate judgement (rate_window 0: fresh from the first frame): these
    tests are about the rules and the scheme; the latency gate is tested in test_quest_latency.py."""
    return QuestGate(rate_window=0.0, recover_s=0.0)


def rand_rot():
    q = RNG.normal(size=4)
    return F.matrix_from_quat(q / np.linalg.norm(q))


def robot(x=1.0, y=2.0, yaw=0.3, ee_p=(1.5, 2.1, 0.8), ee_r=None, t=0.0, reset=0.0):
    return RobotState(t, np.array(ee_p, float), F.rot_axis((1, 1, 0), 0.7) if ee_r is None else ee_r,
                      x, y, yaw, reset)


def head_local(pos_op=(0, 0, 0), yaw=0.0, pitch=0.0):
    return to_local(pos_op, head_r_op(yaw, pitch))


# -- frame math -----------------------------------------------------------------------------------

def test_c_maps_openxr_local_to_operator_flu():
    np.testing.assert_allclose(F.C @ [0, 0, -1], [1, 0, 0])    # LOCAL forward (-Z) -> X forward
    np.testing.assert_allclose(F.C @ [-1, 0, 0], [0, 1, 0])    # LOCAL left (-X) -> Y left
    np.testing.assert_allclose(F.C @ [0, 1, 0], [0, 0, 1])     # LOCAL up (+Y) -> Z up
    assert np.linalg.det(F.C) == pytest.approx(1.0)


def test_matrix16_is_row_major_with_translation_in_the_last_column():
    r = rand_rot()
    m = F.matrix16_from_pose([1, 2, 3], r)
    assert m[3] == 1 and m[7] == 2 and m[11] == 3
    p, r2 = F.pose_from_matrix16(m)
    np.testing.assert_allclose(p, [1, 2, 3])
    np.testing.assert_allclose(r2, r, atol=1e-12)
    assert F.pose_from_matrix16(None) is None
    assert F.pose_from_matrix16([0.0] * 16) is None             # not a rotation
    assert F.pose_from_matrix16([float("nan")] * 16) is None


@pytest.mark.parametrize("yaw", [0.0, 0.7, -2.5, 3.0])
def test_head_yaw_is_gravity_aligned(yaw):
    for pitch in (0.0, 0.6, -0.9):                              # looking down / up does not change the yaw
        _, r = head_local(yaw=yaw, pitch=pitch)
        assert F.head_yaw(r) == pytest.approx(yaw)
    _, r = head_local(yaw=yaw)
    rolled = r @ F.rot_axis((0, 0, 1), 0.5)                     # roll about the view axis (LOCAL Z)
    assert F.head_yaw(rolled) == pytest.approx(yaw)
    _, r = head_local(yaw=yaw, pitch=math.pi / 2 - 0.05)        # almost straight down: undefined
    assert F.head_yaw(r) is None


def test_quaternion_round_trip():
    for _ in range(20):
        r = rand_rot()
        np.testing.assert_allclose(F.matrix_from_quat(F.quat_from_matrix(r)), r, atol=1e-12)
    np.testing.assert_allclose(F.quat_from_matrix(F.rot_z(math.pi)), [0, 0, 1, 0], atol=1e-12)


def engaged(head_yaw=0.4, head_pos=(0.1, 0.2, 1.6), hand=(0.4, -0.2, 1.2), hand_r=None, rob=None):
    hp, hr = head_local(head_pos, head_yaw)
    rob = rob or robot()
    hand_r = rand_rot() if hand_r is None else hand_r
    eng = capture(1, hp, hr, *to_local(hand, hand_r), rob)
    return eng, hand_r, rob


def test_zero_target_jump_at_engagement():
    for _ in range(10):
        rob = robot(*RNG.normal(size=2), RNG.uniform(-3, 3), ee_p=RNG.normal(size=3), ee_r=rand_rot())
        eng, hand_r, rob = engaged(RNG.uniform(-3, 3), RNG.normal(size=3), RNG.normal(size=3), rob=rob)
        # the controller still where it was at engagement
        p, r = body_frozen_target(eng, eng.operator.origin + eng.operator.mapping.T @ eng.hand_anchor[0],
                                  eng.operator.mapping.T @ eng.hand_anchor[1])
        np.testing.assert_allclose(p, rob.ee_p, atol=1e-12)
        np.testing.assert_allclose(r, rob.ee_r, atol=1e-12)


def walk(eng, hand_op, hand_r_op_, step_op):
    """The operator (head and hand together) steps ``step_op`` in their own engagement frame."""
    d_local = eng.operator.mapping.T @ np.asarray(step_op, float)
    p, r = to_local(hand_op, hand_r_op_)
    return p + d_local, r


def test_operator_travel_alone_moves_the_target_through_the_world():
    hand, hand_r = (0.4, -0.2, 1.2), rand_rot()
    eng, _, rob = engaged(head_yaw=1.1, hand=hand, hand_r=hand_r, rob=robot(yaw=0.3))
    p, r = body_frozen_target(eng, *walk(eng, hand, hand_r, (0.5, 0.0, 0.0)))   # half a metre "forward"
    # forward in the operator's frame is forward of the robot's platform frame at engagement, in the world
    np.testing.assert_allclose(p, rob.ee_p + 0.5 * np.array([math.cos(0.3), math.sin(0.3), 0]), atol=1e-12)
    np.testing.assert_allclose(r, rob.ee_r, atol=1e-12)


def test_equal_operator_and_platform_travel_leaves_the_relative_target_unchanged():
    hand, hand_r = (0.4, -0.2, 1.2), rand_rot()
    rob0 = robot(x=1.0, y=2.0, yaw=0.3)
    eng, _, _ = engaged(head_yaw=-2.0, hand=hand, hand_r=hand_r, rob=rob0)
    step = np.array([0.7, -0.2, 0.0])
    p, r = body_frozen_target(eng, *walk(eng, hand, hand_r, step))
    moved = F.PlanarFrame(*(rob0.base.p[:2] + rob0.base.r[:2, :2] @ step[:2]), 0.3)   # the platform made the same trip
    rel = moved.to_local(p, r)
    rel0 = rob0.base.to_local(rob0.ee_p, rob0.ee_r)
    np.testing.assert_allclose(rel[0], rel0[0], atol=1e-12)
    np.testing.assert_allclose(rel[1], rel0[1], atol=1e-12)


def test_platform_motion_does_not_move_the_target():
    hand, hand_r = (0.4, -0.2, 1.2), rand_rot()
    eng, _, rob = engaged(hand=hand, hand_r=hand_r)
    # the target depends only on the frozen frames and the controller: the live base does not enter
    p, r = body_frozen_target(eng, *to_local(hand, hand_r))
    np.testing.assert_allclose(p, rob.ee_p, atol=1e-12)


@pytest.mark.parametrize("axis", [(0, 0, 1), (1, 0, 0), (0.3, -0.5, 0.8)])
def test_rotation_is_spatial_in_the_operator_frame_and_maps_through_the_platform_yaw(axis):
    hand, hand_r = (0.4, -0.2, 1.2), rand_rot()
    eng, _, rob = engaged(head_yaw=0.9, hand=hand, hand_r=hand_r, rob=robot(yaw=-0.6))
    d_op = F.rot_axis(axis, 0.8)                         # rotation in the operator's (engagement) frame
    # the controller turns by d_op in operator axes about its own position
    m = eng.operator.mapping
    r_local = m.T @ d_op @ m @ to_local(hand, hand_r)[1]
    p, r = body_frozen_target(eng, to_local(hand, hand_r)[0], r_local)
    np.testing.assert_allclose(p, rob.ee_p, atol=1e-12)
    expected = F.rot_z(-0.6) @ d_op @ F.rot_z(-0.6).T @ rob.ee_r
    np.testing.assert_allclose(r, expected, atol=1e-12)


def test_matches_the_native_body_frozen_delta():
    """The fake headset's port of native teleop::Clutch (body-frozen) gives the same dp/dR as our frames."""
    hs = Headset(body_frozen=True)
    s0 = HeadsetState(head_pos=(0.1, 0.0, 0.0), head_yaw=0.8)
    s1 = replace(s0, right=replace(s0.right, grip=1.0))
    s2 = replace(s1, head_pos=(0.6, 0.3, 0.0), head_yaw=1.3,
                 right=replace(s1.right, pos=(0.9, 0.2, -0.2), rpy=(0.3, -0.2, 0.9)))
    f0 = hs.frame(s0, 0.0)
    f1 = hs.frame(s1, 0.01)
    f2 = hs.frame(s2, 0.02)
    assert not f0["control_intent"]["hands"]["right"]["engaged"]
    assert f1["control_intent"]["hands"]["right"]["engaged"]
    head = F.pose_from_matrix16(f1["head"]["pose"])
    op = F.OperatorFrame.from_head(*head)
    a = op.pose(*F.pose_from_matrix16(f1["hands"]["right"]["pose"]))
    c = op.pose(*F.pose_from_matrix16(f2["hands"]["right"]["pose"]))
    dp, dr = F.delta(a, c)
    ih = f2["control_intent"]["hands"]["right"]
    np.testing.assert_allclose(dp, ih["delta_position"], atol=1e-5)
    np.testing.assert_allclose(dr.reshape(-1), ih["delta_rotation"], atol=1e-5)


# -- the consumer rules and the engagement state machine ------------------------------------------

class Rig:
    """A headset, the gate and the core, on a hand-advanced clock (frames every 10 ms)."""

    def __init__(self, cfg=Config(), rob=None):
        self.hs = Headset()
        self.gate = consumer_gate()
        self.core = TeleopCore(cfg)
        self.t = 100.0
        self.conn = 1
        self.rob = rob or robot()
        self.last = None

    def frame(self, s: HeadsetState, dt=0.01, *, data=None, deliver=True):
        self.t += dt
        d = data if data is not None else self.hs.frame(s, self.t)
        if deliver:
            self.rob.t_recv = self.t
            self.core.on_frame(d, self.gate.check(Frame(d, self.t, self.conn)), self.rob, self.t)
        return d

    def cmd(self):
        self.last = self.core.command(self.t, self.gate.fresh(self.t), self.rob)
        return self.last

    def run(self, s, n=3):
        for _ in range(n):
            self.frame(s)
        return self.cmd()


IDLE = HeadsetState()
GRIP = replace(IDLE, right=replace(IDLE.right, grip=1.0))
REACH = replace(GRIP, right=replace(GRIP.right, pos=(0.45, -0.2, -0.30)))


def ee_of(cmd):
    return cmd["ees"]["arm"]


def held(cmd):
    """The released arm: a world target riding with the base (the base's twist as feedforward; no base hint)."""
    e = ee_of(cmd)
    return e["frame"] == "world" and "twist" in e and cmd["base"]["mode"] != "hint"


def test_released_holds_then_engages_at_the_command_without_a_jump():
    rig = Rig()
    c = rig.run(IDLE)
    assert held(c) and c["base"] == {"mode": "locked"}
    c = rig.run(GRIP)
    assert rig.core.status.state == "engaged" and ee_of(c)["frame"] == "world"
    np.testing.assert_allclose(ee_of(c)["pos"], rig.rob.ee_p, atol=1e-5)
    np.testing.assert_allclose(F.matrix_from_quat(ee_of(c)["quat"]), rig.rob.ee_r, atol=1e-5)
    c = rig.run(REACH)
    np.testing.assert_allclose(ee_of(c)["pos"], rig.rob.ee_p + F.rot_z(0.3) @ [0.10, 0, 0.05], atol=1e-5)
    # the WBC stopped the arm short of the target; release: hold its command at release, in the base frame
    rig.rob.ee_p = rig.rob.ee_p + [0.03, 0.0, 0.0]
    c = rig.run(IDLE)
    assert held(c) and rig.core.status.state == "released"
    np.testing.assert_allclose(ee_of(c)["pos"], rig.rob.ee_p, atol=1e-9)
    np.testing.assert_allclose(F.matrix_from_quat(ee_of(c)["quat"]), rig.rob.ee_r, atol=1e-9)
    assert c["est_reset"] == 0


def test_start_mid_engagement_waits_for_release_and_a_new_engagement():
    rig = Rig()
    rig.frame(IDLE, deliver=False)                             # the headset engaged before we connected
    rig.frame(GRIP, deliver=False)
    c = rig.run(REACH)
    assert rig.core.status.state == "needs_release" and held(c)
    rig.run(IDLE)
    c = rig.run(GRIP)
    assert rig.core.status.state == "engaged"
    np.testing.assert_allclose(ee_of(c)["pos"], rig.rob.ee_p, atol=1e-5)   # the old delta is not applied


def test_stale_input_stops_sending_and_requires_a_release():
    rig = Rig()
    rig.run(IDLE)
    rig.run(GRIP)
    for _ in range(15):                                        # a network stall: the headset is fine
        rig.frame(GRIP, deliver=False)
    assert rig.cmd() is not None                               # 150 ms: still fresh
    for _ in range(10):
        rig.frame(GRIP, deliver=False)
    assert rig.cmd() is None                                   # > 200 ms: stop sending
    assert rig.core.status.state == "stale"
    c = rig.run(REACH)                                         # frames again, grip still held: hold, no engagement
    assert rig.core.status.state == "needs_release" and held(c)
    rig.run(IDLE)
    rig.run(GRIP)
    assert rig.core.status.state == "engaged"


def test_unfocused_or_invalid_hand_stops_sending():
    rig = Rig()
    rig.run(IDLE)
    rig.run(GRIP)
    assert rig.run(replace(GRIP, focused=False)) is None
    c = rig.run(GRIP)                                          # native requires the release too
    assert rig.core.eng is None and held(c)
    rig.run(IDLE)
    rig.run(GRIP)
    d = rig.hs.frame(GRIP, rig.t)
    d["input_focused"] = False                                 # our own rule, whatever the intent says
    rig.frame(None, data=d)
    assert rig.cmd() is None
    rig.run(REACH)
    assert rig.core.status.state == "needs_release"
    rig.run(IDLE)
    rig.run(GRIP)
    assert rig.run(replace(GRIP, right=replace(GRIP.right, tracked=False))) is None
    assert rig.run(replace(IDLE, left=replace(IDLE.left, tracked=False))) is not None   # the other hand: fine


def test_session_change_invalidates_the_engagement():
    rig = Rig()
    rig.run(IDLE)
    rig.run(GRIP)
    d = rig.hs.frame(REACH, rig.t)
    d["session_id"] = "other"                                  # our own rule, whatever the intent says
    rig.frame(None, data=d)
    assert rig.core.eng is None and "session" in rig.core.status.reason
    c = rig.run(REACH)                                         # still engaged in the intent: wait for a release
    assert rig.core.status.state == "needs_release" and held(c)
    rig.run(IDLE)
    rig.run(GRIP)
    c = rig.run(replace(REACH, session=2))                     # a real session change: native drops it as well
    assert rig.core.eng is None and held(c)


def test_revision_change_invalidates_the_engagement():
    rig = Rig()
    rig.run(IDLE)
    rig.run(GRIP)
    d = rig.hs.frame(REACH, rig.t)
    d["control_intent"]["operator_frame_revision"] += 1        # an A realignment we did not see
    rig.frame(None, data=d)
    assert rig.core.eng is None and "revision" in rig.core.status.reason


def test_out_of_order_frames_are_dropped():
    rig = Rig()
    rig.run(IDLE)
    rig.run(GRIP)
    old = rig.hs.frame(GRIP, rig.t)
    new = rig.frame(REACH)
    rig.frame(None, data=old)
    assert rig.gate.counts["dropped_order"] == 1
    assert rig.core.status.state == "engaged" and new["frame_seq"] > old["frame_seq"]


def test_reconnect_mid_engagement_waits_for_a_new_engagement():
    rig = Rig()
    rig.run(IDLE)
    rig.run(GRIP)
    rig.conn += 1
    c = rig.run(REACH)
    assert rig.core.status.state == "needs_release" and held(c)
    assert "reconnect" in rig.core.status.reason


def test_engagement_needs_fresh_robot_state_and_survives_no_estimator_reset():
    rig = Rig()
    rig.run(IDLE)
    rig.frame(GRIP, deliver=False)
    rig.t += 0.01
    d = rig.hs.frame(GRIP, rig.t)
    rig.rob.t_recv = rig.t - 1.0                               # WBC stream a second old
    rig.core.on_frame(d, rig.gate.check(Frame(d, rig.t, 1)), rig.rob, rig.t)
    assert rig.core.status.state == "needs_release"
    assert rig.core.status.refused == {"no fresh WBC state": 1}
    rig.run(IDLE)
    rig.run(GRIP)
    assert rig.core.status.state == "engaged"
    rig.rob.est_reset = 1.0
    rig.run(REACH)
    assert rig.core.status.state == "needs_release" and "estimator" in rig.core.status.reason


def predicted_base(rob, dt):
    """The stream row's base pose moved on by its commanded twist (base-frame vx, vy; yaw rate) for dt."""
    vx, vy, w = rob.base_twist
    v = F.rot_z(rob.base_yaw)[:2, :2] @ [vx, vy]
    return F.PlanarFrame(rob.base_x + v[0] * dt, rob.base_y + v[1] * dt, rob.base_yaw + w * dt)


def test_released_targets_ride_with_the_base_driven_by_the_stick():
    """Released while the base drives under the stick: the world targets follow the base. The pose relative to the
    base pose PREDICTED to when the WBC applies it (the row's age now - t_recv, plus ride_lead, by the commanded
    twist) stays the command at release, so the target does not trail a moving base; the target carries the
    base's rigid-body velocity at the EE as feedforward."""
    rig = Rig()
    c = rig.run(IDLE)
    rel0 = rig.rob.base.to_local(rig.rob.ee_p, rig.rob.ee_r)
    ee_cmd0 = rig.rob.ee_p.copy()
    vx, vy, w = 0.3, -0.1, 0.5
    rig.rob.base_twist = (vx, vy, w)
    lead = rig.core.cfg.ride_lead
    for k in range(50):
        rig.frame(stick(IDLE, (0.3, 0.6), -0.5))
        rig.rob.base_x += 0.003                                  # the base moves (and the WBC's command with it)
        rig.rob.base_y += 0.001
        rig.rob.base_yaw += 0.005
        rig.rob.ee_p = rig.rob.ee_p + [0.003, 0.001, 0.0]        # never used while released
        age = 0.02 if k % 2 else 0.0                             # every other command from a row 20 ms old
        rig.rob.t_recv = rig.t - age
        c = rig.cmd()
        assert held(c) and c["base"]["mode"] == "commanded"
        e = ee_of(c)
        pe, re_ = np.array(e["pos"]), F.matrix_from_quat(e["quat"])
        rel = predicted_base(rig.rob, age + lead).to_local(pe, re_)
        np.testing.assert_allclose(rel[0], rel0[0], atol=1e-9)
        np.testing.assert_allclose(rel[1], rel0[1], atol=1e-9)
        # ahead of the row's own base pose by exactly the predicted motion (it is not the unpredicted pose)
        assert np.linalg.norm(rig.rob.base.to_local(pe, re_)[0] - rel0[0]) > 0.5 * (age + lead) * 0.3
        # feedforward: v_base (world) + w x r, about z
        r = np.array(e["pos"]) - [rig.rob.base_x, rig.rob.base_y, 0]
        v = F.rot_z(rig.rob.base_yaw)[:2, :2] @ [vx, vy]
        np.testing.assert_allclose(e["twist"], [v[0] - w * r[1], v[1] + w * r[0], 0, 0, 0, w], atol=1e-9)
    assert not np.allclose(ee_of(c)["pos"], ee_cmd0)              # it moved through the world with the base


def test_no_robot_state_yet_sends_keep_alives():
    rig = Rig()
    rig.rob = None
    for _ in range(3):
        rig.t += 0.01
        d = rig.hs.frame(IDLE, rig.t)
        rig.core.on_frame(d, rig.gate.check(Frame(d, rig.t, 1)), None, rig.t)
    assert rig.core.command(rig.t, True, None) == "alive"


# -- base and gripper -----------------------------------------------------------------------------

def stick(state, left=(0.0, 0.0), right_x=0.0):
    return replace(state, left=replace(state.left, stick=left), right=replace(state.right, stick=(right_x, 0.0)))


def test_released_sticks_command_the_base_directly():
    rig = Rig()
    rig.run(IDLE)
    c = rig.run(stick(IDLE, (-1.0, 0.5), 0.5))
    v = c["base"]["velocity"]
    assert c["base"]["mode"] == "commanded" and v["frame"] == "base" and v["ttl"] == 0.2
    # the stream's own scaling: full scale 0.45 m/s and 0.8 rad/s
    assert (v["vx"], v["vy"], v["omega"]) == (pytest.approx(0.45 * 0.5), pytest.approx(0.45), pytest.approx(-0.4))
    assert rig.run(stick(IDLE, (0.1, 0.17)))["base"] == {"mode": "locked"}     # inside the 18 % dead zone


def hint_of(c):
    t = c["base"]["target"]
    return np.array([t["pose"]["x"], t["pose"]["y"], t["heading"]["yaw"]])


def test_engaged_hint_starts_at_the_base_pose_and_mirrors_the_head():
    rig = Rig()
    rig.run(replace(IDLE, head_yaw=1.0))
    c = rig.run(replace(GRIP, head_yaw=1.0))
    assert c["base"]["mode"] == "hint"
    np.testing.assert_allclose(hint_of(c), [1.0, 2.0, 0.3], atol=1e-9)
    h = c["base"]["target"]
    assert (h["pose"]["frame"], h["heading"]["frame"]) == ("world", "world")
    # one metre along the operator's engagement forward, head turned 0.4 and pitched: the platform's forward
    fwd = (math.cos(1.0), math.sin(1.0), 0.0)
    c = rig.run(replace(GRIP, head_pos=fwd, head_yaw=1.4, head_pitch=0.5))
    np.testing.assert_allclose(hint_of(c), [1.0 + math.cos(0.3), 2.0 + math.sin(0.3), 0.7], atol=1e-9)
    c = rig.run(replace(GRIP, head_pos=fwd, head_yaw=1.0 - 3.0))                 # yaw wraps
    assert hint_of(c)[2] == pytest.approx(F.wrap(0.3 - 3.0))
    rig2 = Rig(Config(head_walk=False))
    rig2.run(IDLE)
    rig2.run(GRIP)
    np.testing.assert_allclose(hint_of(rig2.run(replace(GRIP, head_pos=fwd, head_yaw=1.0))), [1.0, 2.0, 0.3])


def test_engaged_sticks_integrate_an_offset_in_the_robot_frame_while_standing_still():
    rig = Rig()
    rig.run(IDLE)
    rig.run(GRIP)
    for _ in range(100):                                       # full forward for 1 s (frames every 10 ms)
        rig.frame(stick(GRIP, (0.0, 1.0)))
    c = rig.cmd()
    assert c["base"]["mode"] == "hint"                         # engaged: never commanded
    np.testing.assert_allclose(rig.core.offset, [0.45 * math.cos(0.3), 0.45 * math.sin(0.3), 0.0], atol=0.01)
    rig.rob.base_yaw = 0.3 + math.pi / 2                       # the robot turned: stick forward = its new forward
    for _ in range(100):
        rig.frame(stick(GRIP, (0.0, 1.0), right_x=-1.0))
    c = rig.cmd()
    expected = [0.45 * (math.cos(0.3) - math.sin(0.3)), 0.45 * (math.sin(0.3) + math.cos(0.3)), 0.8]
    np.testing.assert_allclose(rig.core.offset, expected, atol=0.02)
    np.testing.assert_allclose(hint_of(c), np.array([1.0, 2.0, 0.3]) + rig.core.offset, atol=1e-9)
    assert rig.core.hint == c["base"]["target"]                # exposed for the status line


def test_engaged_target_moves_with_the_stick_offset():
    """Engaged, standing, driving with the sticks (forward, left and a turn): the EE target moves with the stick
    offset by the same planar rigid motion as the base hint (the hint without the offset to the hint with it), so
    the target in the hint's frame stays the engagement command in the base's frame at engagement: the arm does not
    fight the sticks. Without stick motion the target is the body-frozen one alone."""
    rig = Rig()
    rig.run(IDLE)
    c = rig.run(GRIP)
    ee0 = (rig.rob.ee_p.copy(), rig.rob.ee_r.copy())
    rel0 = F.PlanarFrame(1.0, 2.0, 0.3).to_local(*ee0)
    np.testing.assert_allclose(ee_of(c)["pos"], ee0[0], atol=1e-9)
    for _ in range(60):
        rig.frame(stick(GRIP, (0.5, 1.0), right_x=-1.0))
    # the sticks back to rest for a frame: the target is computed before that frame's stick step, so while the
    # sticks move it is one frame (10 ms of stick motion) behind the hint
    c = rig.run(GRIP, n=1)
    off = rig.core.offset
    assert np.linalg.norm(off[:2]) > 0.2 and off[2] > 0.4
    hx, hy, hyaw = hint_of(c)
    np.testing.assert_allclose([hx, hy, hyaw], np.array([1.0, 2.0, 0.3]) + off, atol=1e-9)
    e = ee_of(c)
    rel = F.PlanarFrame(hx, hy, hyaw).to_local(np.array(e["pos"]), F.matrix_from_quat(e["quat"]))
    np.testing.assert_allclose(rel[0], rel0[0], atol=1e-9)
    np.testing.assert_allclose(rel[1], rel0[1], atol=1e-9)
    assert np.linalg.norm(np.array(e["pos"]) - ee0[0]) > 0.2                  # it moved through the world


def test_walking_and_sticks_combine():
    rig = Rig()
    rig.run(IDLE)
    rig.run(GRIP)
    for k in range(1, 51):                                     # walk 0.5 m forward while pushing the stick left
        rig.frame(stick(replace(GRIP, head_pos=(0.01 * k, 0.0, 0.0)), (-1.0, 0.0)))
    c = rig.cmd()
    walk_ = F.rot_z(0.3)[:2, :2] @ [0.5, 0.0]
    np.testing.assert_allclose(hint_of(c)[:2], np.array([1.0, 2.0]) + walk_ + rig.core.offset[:2], atol=1e-9)
    np.testing.assert_allclose(rig.core.offset[:2], F.rot_z(0.3)[:2, :2] @ [0.0, 0.45 * 0.5], atol=0.01)


def rigid(s: HeadsetState, head_pos, head_yaw, head_pitch=0.0, base: HeadsetState = None) -> HeadsetState:
    """``s`` with the operator moved as a rigid body (neck-worn headset and hand together) so that the head is at
    ``head_pos`` with ``head_yaw``: the hand keeps its pose relative to the head's yaw frame. ``base`` is the state
    the hand's relative pose is taken from (default ``s``). Pitch changes only the headset's own orientation."""
    b = base or s
    dyaw = head_yaw - b.head_yaw
    rel = F.rot_z(-b.head_yaw) @ (np.array(b.right.pos) - np.array(b.head_pos))
    pos = np.array(head_pos, float) + F.rot_z(head_yaw) @ rel
    r, p, y = b.right.rpy
    return replace(s, head_pos=tuple(float(v) for v in head_pos), head_yaw=head_yaw, head_pitch=head_pitch,
                   right=replace(s.right, pos=tuple(pos), rpy=(r, p, y + dyaw)))


def ee_in_hint(c):
    """The EE target in the base hint's frame (link 2 as the WBC sees it)."""
    e = ee_of(c)
    return F.PlanarFrame(*hint_of(c)).to_local(np.array(e["pos"]), F.matrix_from_quat(e["quat"]))


RIGID_MOTIONS = {
    "walk": [((0.6, 0.2, 0.0), 0.0, 0.0)],
    "turn_in_place": [((0.0, 0.0, 0.0), a, 0.0) for a in np.linspace(0.05, 0.8, 16)],
    "turn_back_and_forth": [((0.0, 0.0, 0.0), 0.6 * math.sin(k / 4), 0.0) for k in range(1, 30)],
    "walk_and_turn": [((0.02 * k, 0.01 * k, 0.0), -0.03 * k, 0.2) for k in range(1, 40)],
    "turn_past_pi": [((0.1, 0.0, 0.0), a, 0.0) for a in np.linspace(0.2, 3.6, 18)],
}


@pytest.mark.parametrize("motion", sorted(RIGID_MOTIONS))
def test_rigid_operator_motion_keeps_the_ee_fixed_in_the_base_reference(motion):
    """The owner's invariant: once engaged, while the hand keeps its pose relative to the (neck-worn) headset, the
    EE target keeps its pose relative to the base reference, for any walk or turn. With the frozen-frame formula a
    turn swept the EE sideways by about |dyaw| x |ee_in_base - hand_in_head| (0.15 m at 30 degrees here)."""
    rig = Rig()
    start = replace(REACH, head_pos=(0.1, -0.1, 0.0), head_yaw=0.5,
                    right=replace(REACH.right, pos=(0.5, -0.3, -0.3), rpy=(0.2, -0.1, 0.4)))
    rig.run(replace(start, right=replace(start.right, grip=0.0)))
    c0 = rig.run(start)
    rel0 = ee_in_hint(c0)
    np.testing.assert_allclose(ee_of(c0)["pos"], rig.rob.ee_p, atol=1e-12)          # engaging moves nothing
    for head_pos, yaw, pitch in RIGID_MOTIONS[motion]:
        c = rig.run(rigid(start, head_pos, 0.5 + yaw, pitch), n=1)
        rel = ee_in_hint(c)
        np.testing.assert_allclose(rel[0], rel0[0], atol=1e-9)
        np.testing.assert_allclose(rel[1], rel0[1], atol=1e-9)


@pytest.mark.parametrize("yaw", [0.0, 0.7])
def test_crouching_lowers_the_arm(yaw):
    """The base cannot follow vertical motion, so the head frame keeps its height at engagement: crouching with the
    hand held against the body lowers the EE by the same amount, also after a turn (whose planar part is still
    absorbed: the EE's horizontal pose in the base reference is unchanged)."""
    rig = Rig()
    rig.run(IDLE)
    rel0 = ee_in_hint(rig.run(GRIP))
    for k in range(1, 31):
        rel = ee_in_hint(rig.run(rigid(GRIP, (0.2 * k / 30, 0.0, -0.3 * k / 30), yaw * k / 30), n=1))
    np.testing.assert_allclose(rel[0] - rel0[0], [0.0, 0.0, -0.3], atol=1e-9)
    np.testing.assert_allclose(rel[1], rel0[1], atol=1e-9)


def test_hand_motion_relative_to_the_head_moves_the_ee_in_the_base_reference():
    """After a turn, reaching forward relative to the body reaches forward relative to the robot."""
    rig = Rig()
    rig.run(IDLE)
    c0 = rig.run(GRIP)
    rel0 = ee_in_hint(c0)
    turned = rigid(GRIP, (0.3, 0.1, 0.0), 1.0)
    rig.run(turned)
    reach = replace(turned, right=replace(turned.right,
                                          pos=tuple(np.array(turned.right.pos) + F.rot_z(1.0) @ [0.10, 0.05, 0.02])))
    rel = ee_in_hint(rig.run(reach))
    np.testing.assert_allclose(rel[0] - rel0[0], [0.10, 0.05, 0.02], atol=1e-9)
    np.testing.assert_allclose(rel[1], rel0[1], atol=1e-9)


def test_without_head_walk_the_target_is_the_frozen_frame_formula():
    """``head_walk`` off: link 1 is frozen (the base reference moves only with the sticks) and the hand is read in
    the operator frame frozen at engagement, the DEMO.md body-frozen formula."""
    rig = Rig(Config(head_walk=False))
    rig.run(IDLE)
    rig.run(GRIP)
    eng = rig.core.eng
    moved = rigid(GRIP, (0.4, 0.2, 0.0), 0.7)
    c = rig.run(moved)
    expected = body_frozen_target(eng, *to_local(moved.right.pos, F.rot_z(0.7)))
    np.testing.assert_allclose(ee_of(c)["pos"], expected[0], atol=1e-9)
    np.testing.assert_allclose(F.matrix_from_quat(ee_of(c)["quat"]), expected[1], atol=1e-9)
    np.testing.assert_allclose(hint_of(c), [1.0, 2.0, 0.3], atol=1e-12)


def test_head_loss_and_reacquisition_do_not_move_the_robot():
    """Head lost mid-engagement: the base reference holds and the hand is read in the last tracked head frame. The
    operator walks and turns meanwhile; when the head comes back elsewhere both links re-anchor where they are, so
    the commands do not jump, and the invariant and hand mapping hold from the new head pose on."""
    rig = Rig()
    rig.run(IDLE)
    rig.run(GRIP)
    rig.run(rigid(GRIP, (0.2, 0.0, 0.0), 0.3))
    lost = replace(rigid(GRIP, (0.2, 0.0, 0.0), 0.3), head_tracked=False)
    c_lost = rig.run(lost)
    reacquired = rigid(GRIP, (0.9, -0.4, 0.0), 1.4)                    # the operator moved rigidly while lost
    c_back = rig.run(reacquired, n=1)
    np.testing.assert_allclose(hint_of(c_back), hint_of(c_lost), atol=1e-12)
    np.testing.assert_allclose(ee_of(c_back)["pos"], ee_of(c_lost)["pos"], atol=1e-12)
    np.testing.assert_allclose(F.matrix_from_quat(ee_of(c_back)["quat"]),
                               F.matrix_from_quat(ee_of(c_lost)["quat"]), atol=1e-12)
    rel_back = ee_in_hint(c_back)
    for k in range(1, 20):                                              # rigid motion from the new pose
        c = rig.run(rigid(GRIP, (0.9 + 0.02 * k, -0.4, 0.0), 1.4 - 0.05 * k, base=reacquired), n=1)
        rel = ee_in_hint(c)
        np.testing.assert_allclose(rel[0], rel_back[0], atol=1e-9)
        np.testing.assert_allclose(rel[1], rel_back[1], atol=1e-9)


def turned_by_stick(mode, frames=196):
    """Engaged at the robot's heading (yaw 0, operator yaw 0: the grip registers them), then the right stick
    turns the robot about 90 degrees while the operator stands still."""
    rig = Rig(Config(mode=mode))
    rig.rob.base_yaw = 0.0
    rig.run(IDLE)
    rig.run(GRIP)
    for _ in range(frames):
        rig.frame(stick(GRIP, right_x=-1.0))
    c = rig.run(GRIP, n=1)
    return rig, c


@pytest.mark.parametrize("mode", ["third", "first"])
def test_a_stick_turn_turns_the_robot_in_place(mode):
    rig, c = turned_by_stick(mode)
    np.testing.assert_allclose(hint_of(c)[:2], [1.0, 2.0], atol=1e-9)      # about the base: no translation
    assert hint_of(c)[2] == pytest.approx(math.pi / 2, abs=0.05)
    rob = robot()
    rel0 = F.PlanarFrame(1.0, 2.0, 0.0).to_local(rob.ee_p, rob.ee_r)        # the EE in the base at engagement
    rel = ee_in_hint(c)                                                      # the arm swings with the robot
    np.testing.assert_allclose(rel[0], rel0[0], atol=1e-9)
    np.testing.assert_allclose(rel[1], rel0[1], atol=1e-9)


def test_third_person_after_a_stick_turn_walking_and_reaching_stay_room_aligned():
    """Shared world: the grip registered the headset to the robot; a stick turn changes the robot's heading but
    not the directions. Walking forward (in the room) moves the robot forward in the room, sideways for itself;
    reaching forward moves the EE forward in the room."""
    rig, c0 = turned_by_stick("third")
    h0, e0 = hint_of(c0), np.array(ee_of(c0)["pos"])
    c = rig.run(replace(GRIP, right=replace(GRIP.right, pos=tuple(np.array(GRIP.right.pos) + [0.1, 0, 0]))), n=1)
    np.testing.assert_allclose(np.array(ee_of(c)["pos"]) - e0, [0.1, 0.0, 0.0], atol=1e-9)
    c = rig.run(rigid(GRIP, (1.0, 0.0, 0.0), 0.0), n=1)
    np.testing.assert_allclose(hint_of(c) - h0, [1.0, 0.0, 0.0], atol=1e-9)


def test_first_person_after_a_stick_turn_walking_and_reaching_follow_the_robot():
    """Egocentric: the stick turn shifted the anchor. Walking forward moves the robot along its own (new)
    forward; reaching forward moves the EE along the robot's forward."""
    rig, c0 = turned_by_stick("first")
    yaw = hint_of(c0)[2]
    fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    h0, e0 = hint_of(c0), np.array(ee_of(c0)["pos"])
    c = rig.run(replace(GRIP, right=replace(GRIP.right, pos=tuple(np.array(GRIP.right.pos) + [0.1, 0, 0]))), n=1)
    np.testing.assert_allclose(np.array(ee_of(c)["pos"]) - e0, 0.1 * fwd, atol=1e-9)
    c = rig.run(rigid(GRIP, (1.0, 0.0, 0.0), 0.0), n=1)
    np.testing.assert_allclose(hint_of(c)[:2] - h0[:2], fwd[:2], atol=1e-9)
    assert hint_of(c)[2] == pytest.approx(yaw)


def test_the_modes_agree_without_a_stick_turn():
    """The stick terms differ only in the yaw: with walking, turning, reaching, crouching and stick translation
    (no stick yaw), both modes command the same targets and hints."""
    seq = [rigid(GRIP, (0.03 * k, 0.01 * k, -0.005 * k), 0.04 * k) for k in range(1, 20)]
    seq += [stick(replace(s_, right=replace(s_.right, pos=tuple(np.array(s_.right.pos) + [0.05, -0.02, 0.03]))),
                  (0.6, 0.8)) for s_ in seq[-1:] * 15]
    out = {}
    for mode in ("third", "first"):
        rig = Rig(Config(mode=mode))
        rig.run(IDLE)
        rig.run(GRIP)
        out[mode] = [rig.run(s_, n=1) for s_ in seq]
    for a, b in zip(out["third"], out["first"]):
        np.testing.assert_allclose(ee_of(a)["pos"], ee_of(b)["pos"], atol=1e-9)
        np.testing.assert_allclose(ee_of(a)["quat"], ee_of(b)["quat"], atol=1e-9)
        np.testing.assert_allclose(hint_of(a), hint_of(b), atol=1e-9)


def test_first_person_walking_feedforward_turns_with_the_stick():
    rig, _ = turned_by_stick("first")
    for k in range(1, 15):                                    # walking forward at 0.4 m/s (operator frame)
        rig.frame(moving(GRIP, k, v_hand=(0.4, 0, 0), v_head=(0.4, 0, 0)))
    hv = rig.core.feedforward()[1]
    yaw = rig.core._hint_pose()[2]
    np.testing.assert_allclose(hv[:2], 0.4 * np.array([math.cos(yaw), math.sin(yaw)]), atol=0.03)


def test_the_mode_is_read_at_engagement():
    """Toggling the mode takes effect at the next grip, never mid-engagement."""
    cfg = Config(mode="third")
    rig = Rig(cfg)
    rig.run(IDLE)
    rig.run(GRIP)
    cfg.mode = "first"
    assert rig.core.eng.mode == "third"
    rig.run(IDLE)
    rig.run(GRIP)
    assert rig.core.eng.mode == "first"


def test_offset_restarts_at_zero_on_every_engagement():
    rig = Rig()
    rig.run(IDLE)
    rig.run(GRIP)
    for _ in range(50):
        rig.frame(stick(GRIP, (0.0, 1.0)))
    assert np.linalg.norm(rig.core.offset) > 0.2
    assert rig.run(IDLE)["base"] == {"mode": "locked"}          # release: the hint is dropped
    assert rig.core.hint is None
    rig.rob.base_x, rig.rob.base_y = 1.2, 2.1                   # the base got there meanwhile
    c = rig.run(GRIP)
    np.testing.assert_allclose(rig.core.offset, 0.0)
    np.testing.assert_allclose(hint_of(c), [1.2, 2.1, 0.3], atol=1e-9)


OPERATOR_MOTIONS = {
    "walk": dict(head_pos=(1.5, 0.4, 0.0)),
    "turn": dict(head_yaw=2.0),
    "walk_turn_reach": dict(head_pos=(-0.6, 1.0, 0.1), head_yaw=-1.2, head_pitch=0.4),
}


@pytest.mark.parametrize("motion", sorted(OPERATOR_MOTIONS))
@pytest.mark.parametrize("stick_while_released", [False, True])
def test_release_and_reengage_never_moves_the_robot(motion, stick_while_released):
    """The owner's rule: whatever the operator does between a release and the next engagement, the first datagram
    after re-engaging commands exactly the current commanded EE pose and the current base pose and heading."""
    rig = Rig()
    rig.run(IDLE)
    rig.run(GRIP)
    for k in range(1, 31):                                      # an engagement with arm, head and stick motion
        rig.frame(stick(replace(REACH, head_pos=(0.01 * k, 0, 0), head_yaw=0.01 * k), (0.3, 0.8), 0.4))
    rig.run(IDLE)
    # released: the operator moves arbitrarily, the hand too; maybe the sticks drive the base
    moved = replace(IDLE, **OPERATOR_MOTIONS[motion],
                    right=replace(IDLE.right, pos=(0.1, 0.5, 0.2), rpy=(0.5, -0.3, 1.0)))
    for k in range(1, 41):
        s_ = stick(moved, (0.7, -0.9), 0.6) if stick_while_released else moved
        rig.frame(s_)
        c = rig.cmd()
        assert held(c)                                          # base-attached: the arm rides with the base
        assert c["base"]["mode"] == ("commanded" if stick_while_released else "locked")
        if stick_while_released:
            rig.rob.base_x += 0.004                             # the base moves under the stick, the arm with it
            rig.rob.base_yaw += 0.01
            rig.rob.ee_p = rig.rob.ee_p + [0.004, 0.0, 0.0]
    # the WBC's command and the base pose are wherever they ended up
    rig.rob.ee_p = rig.rob.ee_p + [0.05, -0.02, 0.01]
    rig.rob.ee_r = F.rot_axis((0, 1, 1), 0.3) @ rig.rob.ee_r
    c = rig.run(replace(moved, right=replace(moved.right, grip=1.0)), n=1)
    assert rig.core.status.state == "engaged"
    e = ee_of(c)
    assert e["frame"] == "world"
    np.testing.assert_allclose(e["pos"], rig.rob.ee_p, atol=1e-12)
    np.testing.assert_allclose(F.matrix_from_quat(e["quat"]), rig.rob.ee_r, atol=1e-12)
    np.testing.assert_allclose(hint_of(c), [rig.rob.base_x, rig.rob.base_y, F.wrap(rig.rob.base_yaw)], atol=1e-12)


def test_target_datagrams_carry_the_estimator_epoch():
    rig = Rig(rob=robot(reset=3.0))
    assert rig.run(IDLE)["est_reset"] == 3
    assert rig.run(GRIP)["est_reset"] == 3


def test_squeezing_the_trigger_closes_the_gripper():
    """The headset's gripper intent follows the trigger (0 released .. 1 squeezed); the WBC's gripper is the opening
    (0 closed .. 1 open): the command is 1 - the intent, engaged or not."""
    rig = Rig()
    assert ee_of(rig.run(IDLE))["gripper"] == pytest.approx(1.0)                # trigger released: open
    c = rig.run(replace(GRIP, right=replace(GRIP.right, trigger=0.7)))
    assert ee_of(c)["gripper"] == pytest.approx(0.3)
    c = rig.run(replace(GRIP, right=replace(GRIP.right, trigger=1.0)))
    assert ee_of(c)["gripper"] == pytest.approx(0.0)                            # squeezed: closed
    c = rig.run(replace(IDLE, right=replace(IDLE.right, trigger=0.2)))     # independent of the clutch
    assert ee_of(c)["gripper"] == pytest.approx(0.8)
    rig2 = Rig(Config(gripper=False))
    assert "gripper" not in ee_of(rig2.run(IDLE))


def test_robot_state_from_a_stream_row():
    meas = {"arm_meas_x": 1.0, "arm_meas_y": 2.0, "arm_meas_z": 0.5, "arm_meas_qx": 0.0, "arm_meas_qy": 0.0,
            "arm_meas_qz": math.sin(0.25), "arm_meas_qw": math.cos(0.25)}
    cmd = {k.replace("meas", "cmd"): v for k, v in meas.items()}
    cmd["arm_cmd_x"] = 1.1
    row = {**meas, **cmd, "base_x": 0.1, "base_y": 0.2, "base_yaw": 0.3, "est_reset": 2.0}
    s = RobotState.from_row(row, 5.0)
    assert s.ee_source == "cmd" and s.ee_p[0] == 1.1                  # the command is the anchor
    np.testing.assert_allclose(s.ee_r, F.rot_z(0.5), atol=1e-12)
    assert (s.base_x, s.base_yaw, s.est_reset, s.t_recv) == (0.1, 0.3, 2.0, 5.0)
    s = RobotState.from_row({**row, "arm_cmd_x": float("nan")}, 5.0)    # no command yet: measured
    assert s.ee_source == "meas" and s.ee_p[0] == 1.0
    assert RobotState.from_row({**row, "arm_cmd_x": float("nan"), "arm_meas_x": float("nan")}, 5.0) is None
    assert RobotState.from_row({k: v for k, v in row.items() if k != "base_yaw"}, 5.0) is None


def test_a_live_headset_recording_drives_the_client():
    """53 frames recorded from the headset (body-frozen, 2026-11-21, quest_probe.py) around a left-grip
    engagement: every frame passes the gate, the grip engages without moving the target off the commanded anchor,
    the release holds it, and the live field types (integer gripper, null poses) parse."""
    import gzip
    import json
    path = Path(__file__).parent / "data" / "quest_live_body_frozen.jsonl.gz"
    recs = [json.loads(line) for line in gzip.open(path, "rt")]
    gate, core = consumer_gate(), TeleopCore(Config(hand="left"))
    r = robot(ee_r=np.eye(3))
    first_engaged, states = None, []
    for d in recs:
        now = d["rx"]
        r = replace(r, t_recv=now)
        v = gate.check(Frame(d["record"], now, 1))
        assert v.accept and v.valid, v.reason
        core.on_frame(d["record"], v, r, now)
        out = core.command(now, gate.fresh(now), r)
        assert isinstance(out, dict)
        states.append(core.status.state)
        if core.status.state == "engaged" and first_engaged is None:
            first_engaged = out
            assert np.allclose(out["ees"]["arm"]["pos"], r.ee_p)   # engaging never moves the target
            assert out["base"]["mode"] == "hint"
    assert first_engaged is not None and core.status.engagements == 1
    assert states[-1] == "released"
    assert 0.0 <= out["ees"]["arm"]["gripper"] <= 1.0


# -- velocity feedforward while engaged ----------------------------------------------------------------------

def moving(state, k, v_hand=(0.0, 0.0, 0.0), w_hand=0.0, v_head=(0.0, 0.0, 0.0), dt=0.01):
    """The state after k frames of the hand (and head) moving at constant velocity (operator frame)."""
    hp = np.array(state.right.pos) + np.array(v_hand) * k * dt
    rpy = (0.0, 0.0, w_hand * k * dt)
    return replace(state, head_pos=tuple(np.array(state.head_pos) + np.array(v_head) * k * dt),
                   right=replace(state.right, pos=tuple(hp), rpy=rpy))


def test_engaged_target_carries_the_hands_velocity_starting_at_zero():
    rig = Rig()
    rig.run(IDLE)
    rig.frame(GRIP)
    c = rig.cmd()
    assert ee_of(c)["twist"] == [0.0] * 6                       # engagement: no feedforward yet
    h = c["base"]["target"]
    assert (h["pose"]["vx"], h["pose"]["vy"], h["heading"]["omega"]) == (0.0, 0.0, 0.0)
    v, w = np.array([0.5, 0.1, 0.0]), 0.8
    ramp = []
    for k in range(1, 20):                                     # the hand moves at 0.5 m/s and turns at 0.8 rad/s
        rig.frame(moving(GRIP, k, v, w))
        ramp.append(np.linalg.norm(ee_of(rig.cmd())["twist"][:3]))
    assert ramp[0] < 0.05 and all(b >= a - 1e-9 for a, b in zip(ramp[:6], ramp[1:7]))   # ramps in
    tw = np.array(ee_of(rig.last)["twist"])
    # through the frozen frames: the operator's forward is the platform's (yaw 0.3 at engagement)
    np.testing.assert_allclose(tw[:3], F.rot_z(0.3) @ v, atol=1e-6)
    np.testing.assert_allclose(tw[3:], [0.0, 0.0, w], atol=1e-6)
    # release and re-engage: zero again
    rig.run(IDLE)
    rig.frame(moving(GRIP, 20, v, w))
    assert ee_of(rig.cmd())["twist"] == [0.0] * 6


def test_engaged_hint_carries_the_heads_velocity_plus_the_sticks():
    rig = Rig()
    rig.run(IDLE)
    rig.run(GRIP)
    v = np.array([0.4, -0.1, 0.0])
    for k in range(1, 15):                                     # walking forward and a little right
        rig.frame(moving(GRIP, k, v_head=v))
    h = rig.cmd()["base"]["target"]
    np.testing.assert_allclose([h["pose"]["vx"], h["pose"]["vy"]], (F.rot_z(0.3) @ v)[:2], atol=1e-6)
    assert h["heading"]["omega"] == pytest.approx(0.0, abs=1e-9)
    s = stick(moving(GRIP, 15, v_head=v), (0.0, 1.0))           # plus the left stick forward (0.45 m/s)
    rig.frame(s)
    h = rig.cmd()["base"]["target"]
    want = (F.rot_z(0.3) @ v)[:2] + F.rot_z(0.3)[:2, :2] @ [0.45, 0.0]
    want *= min(1.0, 0.6 / np.linalg.norm(want))                # clamped to the WBC's max_speed
    np.testing.assert_allclose([h["pose"]["vx"], h["pose"]["vy"]], want, atol=0.02)
    # the head lost: its velocity is zero (the last contribution is kept)
    rig.frame(replace(moving(GRIP, 16, v_head=v), head_tracked=False))
    h = rig.cmd()["base"]["target"]
    assert math.hypot(h["pose"]["vx"], h["pose"]["vy"]) == pytest.approx(0.0, abs=1e-9)


def test_feedforward_reduces_the_lag_of_a_walking_target_through_the_controller():
    """The client's target and twist for a hand moving at 0.15 m/s, fed to the WBC's QP (base locked): with the
    twist the EE keeps up; without it, it trails by about v / k_pos."""
    from robot_utils.nodes.swerve.constants import MODULES
    from robot_utils.nodes.wbc.controller import EEState, QPController, Snapshot
    from robot_utils.nodes.wbc.model import EESpec, WbcModel, arm_config_limits, fetch_urdf
    from robot_utils.nodes.wbc.spec import Config as WConfig, Hints
    from robot_utils.nodes.wbc.targets import TargetBuffer
    urdf, _ = fetch_urdf("local")
    m = WbcModel(urdf, (EESpec(arm_limits=arm_config_limits()),))
    q0 = np.array([0.0, -0.6, -0.9, 0.0, 0.5, 0.0])
    lags = {}
    for ff in (False, True):
        cfg = WConfig()
        ctl = QPController(m, cfg, [(mm.x, mm.y) for mm in MODULES])
        T0 = m.fk(m.config((0, 0, 0), {"arm": q0}), "arm")
        rig = Rig(Config(feedforward=ff), rob=robot(0.0, 0.0, 0.0, ee_p=T0.translation, ee_r=T0.rotation))
        rig.run(IDLE)
        rig.run(GRIP)
        q, v, buf, errs = q0.copy(), np.zeros(6), TargetBuffer("arm"), []
        for k in range(1, 120):                                # 1.2 s at 0.15 m/s upward
            rig.frame(moving(GRIP, k, (0.0, 0.0, 0.15)))
            e = ee_of(rig.cmd())
            tw = e.get("twist")
            buf.set_reactive(rig.t, e["pos"], F.matrix_from_quat(e["quat"]), tw)
            h = Hints(mode="locked")
            out = ctl.step(Snapshot(rig.t, (0.0, 0.0, 0.0), np.zeros(3),
                                    {"arm": EEState(q.copy(), v.copy(), buf.sample(rig.t), h.ee("arm", cfg), q0)}, h))
            q, v = q + out.dq["arm"], out.qd["arm"].copy()
            errs.append(np.linalg.norm(m.fk(m.config((0, 0, 0), {"arm": q}), "arm").translation - np.array(e["pos"])))
        lags[ff] = float(np.mean(errs[-20:]))
    assert lags[False] > 0.5 * 0.15 / WConfig().k_pos
    assert lags[True] < 0.3 * lags[False], lags
