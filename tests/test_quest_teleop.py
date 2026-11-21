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
        self.gate = QuestGate()
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


def test_released_holds_the_measured_ee_in_the_base_frame_then_engages_without_a_jump():
    rig = Rig()
    c = rig.run(IDLE)
    assert ee_of(c)["frame"] == "base" and c["base"] == {"mode": "locked"}
    rel = rig.rob.base.to_local(rig.rob.ee_p, rig.rob.ee_r)
    np.testing.assert_allclose(ee_of(c)["pos"], rel[0])
    c = rig.run(GRIP)
    assert rig.core.status.state == "engaged" and ee_of(c)["frame"] == "world"
    np.testing.assert_allclose(ee_of(c)["pos"], rig.rob.ee_p, atol=1e-5)
    np.testing.assert_allclose(F.matrix_from_quat(ee_of(c)["quat"]), rig.rob.ee_r, atol=1e-5)
    c = rig.run(REACH)
    np.testing.assert_allclose(ee_of(c)["pos"], rig.rob.ee_p + F.rot_z(0.3) @ [0.10, 0, 0.05], atol=1e-5)
    # release: hold the last target, now in the base frame
    c = rig.run(IDLE)
    assert ee_of(c)["frame"] == "base" and rig.core.status.state == "released"
    np.testing.assert_allclose(rig.rob.base.to_world(ee_of(c)["pos"], np.eye(3))[0],
                               rig.rob.ee_p + F.rot_z(0.3) @ [0.10, 0, 0.05], atol=1e-5)


def test_start_mid_engagement_waits_for_release_and_a_new_engagement():
    rig = Rig()
    rig.frame(IDLE, deliver=False)                             # the headset engaged before we connected
    rig.frame(GRIP, deliver=False)
    c = rig.run(REACH)
    assert rig.core.status.state == "needs_release" and ee_of(c)["frame"] == "base"
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
    assert rig.core.status.state == "needs_release" and ee_of(c)["frame"] == "base"
    rig.run(IDLE)
    rig.run(GRIP)
    assert rig.core.status.state == "engaged"


def test_unfocused_or_invalid_hand_stops_sending():
    rig = Rig()
    rig.run(IDLE)
    rig.run(GRIP)
    assert rig.run(replace(GRIP, focused=False)) is None
    c = rig.run(GRIP)                                          # native requires the release too
    assert rig.core.eng is None and ee_of(c)["frame"] == "base"
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
    assert rig.core.status.state == "needs_release" and ee_of(c)["frame"] == "base"
    rig.run(IDLE)
    rig.run(GRIP)
    c = rig.run(replace(REACH, session=2))                     # a real session change: native drops it as well
    assert rig.core.eng is None and ee_of(c)["frame"] == "base"


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
    assert rig.core.status.state == "needs_release" and ee_of(c)["frame"] == "base"
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


def test_no_robot_state_yet_sends_keep_alives():
    rig = Rig()
    rig.rob = None
    for _ in range(3):
        rig.t += 0.01
        d = rig.hs.frame(IDLE, rig.t)
        rig.core.on_frame(d, rig.gate.check(Frame(d, rig.t, 1)), None, rig.t)
    assert rig.core.command(rig.t, True, None) == "alive"


# -- base and gripper -----------------------------------------------------------------------------

def test_stick_commands_base_velocity_with_ttl_then_back_to_the_heading_hint():
    rig = Rig(Config(max_speed=0.3, max_omega=0.8, velocity_ttl=0.3))
    rig.run(IDLE)
    rig.run(GRIP)
    c = rig.run(replace(GRIP, left=replace(GRIP.left, stick=(-1.0, 0.5)), right=replace(GRIP.right, stick=(0.5, 0))))
    v = c["base"]["velocity"]
    assert c["base"]["mode"] == "commanded" and v["frame"] == "base" and v["ttl"] == 0.3
    assert v["vx"] == pytest.approx(0.3 * 0.5) and v["vy"] == pytest.approx(0.3)
    assert v["omega"] == pytest.approx(-0.8 * 0.5)
    c = rig.run(replace(GRIP, left=replace(GRIP.left, stick=(0.1, 0.17))))      # inside the 18 % dead zone
    assert c["base"]["mode"] == "hint"
    assert set(c["base"]["target"]) == {"heading"}


def test_head_yaw_drives_the_heading_hint_relative_to_the_engagement():
    rig = Rig(Config(heading_weight=0.5, heading_tol=0.05))
    rig.run(replace(IDLE, head_yaw=1.0))
    c = rig.run(replace(GRIP, head_yaw=1.0))
    h = c["base"]["target"]["heading"]
    assert h == {"yaw": pytest.approx(0.3), "frame": "world", "weight": 0.5, "tol": 0.05}
    c = rig.run(replace(GRIP, head_yaw=1.0 + 0.4, head_pitch=0.5))
    assert c["base"]["target"]["heading"]["yaw"] == pytest.approx(0.7)
    c = rig.run(replace(GRIP, head_yaw=1.0 - 3.0))             # wraps
    assert c["base"]["target"]["heading"]["yaw"] == pytest.approx(F.wrap(0.3 - 3.0))
    rig2 = Rig(Config(heading_hint=False))
    rig2.run(IDLE)
    assert rig2.run(GRIP)["base"] == {"mode": "free"}


def test_pose_hint_follows_head_displacement_when_enabled():
    rig = Rig(Config(pose_hint=True))
    rig.run(replace(IDLE, head_yaw=0.5))
    c = rig.run(replace(GRIP, head_yaw=0.5))
    pose = c["base"]["target"]["pose"]
    assert (pose["x"], pose["y"]) == (pytest.approx(1.0), pytest.approx(2.0))
    # one metre along the operator's engagement forward -> one metre along the platform's forward at engagement
    fwd = (math.cos(0.5), math.sin(0.5), 0.0)
    c = rig.run(replace(GRIP, head_yaw=0.5, head_pos=fwd))
    pose = c["base"]["target"]["pose"]
    assert (pose["x"], pose["y"]) == (pytest.approx(1.0 + math.cos(0.3)), pytest.approx(2.0 + math.sin(0.3)))
    assert "pose" not in Rig().run(GRIP)["base"].get("target", {})        # off by default


def test_stick_rereferences_the_head_hints():
    rig = Rig()
    rig.run(IDLE)
    rig.run(GRIP)
    rig.run(replace(GRIP, left=replace(GRIP.left, stick=(0.0, 0.0)), right=replace(GRIP.right, stick=(1.0, 0))))
    rig.rob.base_yaw = 1.2                                     # the stick turned the platform
    c = rig.run(replace(GRIP, head_yaw=0.0))
    assert c["base"]["target"]["heading"]["yaw"] == pytest.approx(1.2)


def test_trigger_drives_the_gripper():
    rig = Rig()
    rig.run(IDLE)
    c = rig.run(replace(GRIP, right=replace(GRIP.right, trigger=0.7)))
    assert ee_of(c)["gripper"] == pytest.approx(0.7)
    c = rig.run(replace(IDLE, right=replace(IDLE.right, trigger=0.2)))     # independent of the clutch
    assert ee_of(c)["gripper"] == pytest.approx(0.2)
    rig2 = Rig(Config(gripper=False))
    assert "gripper" not in ee_of(rig2.run(IDLE))


def test_robot_state_from_a_stream_row():
    row = {"arm_meas_x": 1.0, "arm_meas_y": 2.0, "arm_meas_z": 0.5, "arm_meas_qx": 0.0, "arm_meas_qy": 0.0,
           "arm_meas_qz": math.sin(0.25), "arm_meas_qw": math.cos(0.25), "base_x": 0.1, "base_y": 0.2,
           "base_yaw": 0.3, "est_reset": 2.0}
    s = RobotState.from_row(row, 5.0)
    np.testing.assert_allclose(s.ee_r, F.rot_z(0.5), atol=1e-12)
    assert (s.base_x, s.base_yaw, s.est_reset, s.t_recv) == (0.1, 0.3, 2.0, 5.0)
    assert RobotState.from_row({**row, "arm_meas_x": float("nan")}, 5.0) is None
    assert RobotState.from_row({k: v for k, v in row.items() if k != "base_yaw"}, 5.0) is None
