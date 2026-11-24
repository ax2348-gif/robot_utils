"""The WBC's armed latch (docs/wbc.md "The armed latch"): disarmed at enable; targets only while armed; every disarm
trigger (one event each, the targets cleared, the base braking within its rate limit, the gripper frozen, the arm
stopped at its commanded joints); no recovery re-arms (a new arm_seq is needed); the base caps refused while the base
moves; hint velocities clamped, not dropped. Lockstep against the fake arm (tests/wbc_fakes.py), a recording base."""
import json
import math
import socket
import tempfile
import time

import numpy as np
import pytest

from helpers import Clock, serve, stop, wait_for
from wbc_fakes import FakeArm
from robot_utils.nodes.wbc.model import EESpec, WbcModel, arm_config_limits, fetch_urdf
from robot_utils.nodes.wbc.node import STATES, WbcNode

DT = 0.01


@pytest.fixture(scope="module")
def model():
    urdf, _ = fetch_urdf("local")
    return WbcModel(urdf, (EESpec(arm_limits=arm_config_limits()),))


class FakeBase:
    """Stands in for links.SwerveBase: records every twist and limits push; never fails."""
    lease_id = 7
    hard: dict = {}
    last_error = None
    errors = 0

    class Client:
        def __init__(self):
            self.limits = []

        def set_limits(self, s, w):
            self.limits.append((s, w))

    def __init__(self):
        self.twists = []
        self.sent = 0
        self.client = self.Client()
        self.muted = False
        self.closed = None

    def send(self, tw):                       # what links.SwerveBase would put on the wire
        if not self.muted:
            self.twists.append(tuple(float(v) for v in tw))
            self.sent += 1

    def mute(self):
        self.muted = True

    def failing_for(self):
        return 0.0

    def close(self, disable=True):
        self.closed = disable


class Rig:
    def __init__(self, node, clock, arm):
        self.node, self.clock, self.arm = node, clock, arm
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.session = None
        self.aseq = 0                 # the client's arm_seq
        self.armed = False            # what the client sends

    def step(self, n=1):
        for _ in range(n):
            self.clock.t += DT
            self.node.step()
            time.sleep(0.0015)

    def row(self):
        return dict(zip(self.node.fields, self.node.hub.recent(1, "tick")[0]))

    def lease_enable(self, base=False):
        self.tok, _ = self.node.acquire("test")
        self.node.enable(self.tok)
        if base:
            with self.node.lock:
                self.node.base = FakeBase()
        out = self.node.open_session(self.tok)
        self.session = out["session"]
        return out

    def send(self, msg, latch=True, session=None):
        self.seq += 1
        m = {"session": self.session if session is None else session, "seq": self.seq, **msg}
        if latch:
            m.setdefault("armed", self.armed)
            m.setdefault("arm_seq", self.aseq)
        self.udp.sendto(json.dumps(m).encode(), ("127.0.0.1", self.node.receiver.udp_port))
        time.sleep(0.003)

    def cmd_pose(self):
        r = self.row()
        return np.array([r[f"arm_cmd_{k}"] for k in "xyz"]), np.array([r[f"arm_cmd_q{k}"] for k in "xyzw"])

    def target(self, pos, quat, base=None, **kw):
        msg = {"type": "target", "t": None, "ees": {"arm": {"frame": "world", "pos": list(map(float, pos)),
                                                            "quat": list(map(float, quat)), **kw}}}
        if base is not None:
            msg["base"] = base
        self.send(msg)

    def arm_now(self, pos=None, quat=None, **kw):
        """A Y press: a new arm_seq, armed, with a target (default: the commanded pose)."""
        if pos is None:
            pos, quat = self.cmd_pose()
        self.aseq += 1
        self.armed = True
        self.target(pos, quat, **kw)
        self.step(1)
        assert self.node.armed, self.node.state_dict()["disarm_reason"]

    def disarm_events(self):
        return [e["msg"] for e in self.node.events.list() if e["msg"].startswith("DISARMED")]


@pytest.fixture
def rig(model):
    d = tempfile.mkdtemp(prefix="wbca")
    arm = FakeArm(d + "/arm.joint.sock")
    url, srv, th = serve(arm.app)
    clock = Clock(time.monotonic())
    node = WbcNode(model, arm=url, swerve=None, estimator=None, clock=clock, udp_host="127.0.0.1", udp_port=0,
                   unix_socket=d + "/wbc.target.sock", arm_socket=d + "/arm.joint.sock")
    node.receiver.start()
    r = Rig(node, clock, arm)
    r.step(2)
    yield r
    if node.enabled:
        node.disable("test end")
    node.receiver.close()
    arm.close()
    stop((srv, th))


def moving_goal(rig, p0, q0, n=40, dx=(0.0, 0.0, 0.04), **kw):
    """Track toward p0 + dx for n ticks (armed); returns the goal."""
    goal = p0 + np.asarray(dx)
    for _ in range(n):
        rig.target(goal, q0, **kw)
        rig.step(1)
    return goal


# -- arming ---------------------------------------------------------------------------------------------------

def test_enabled_is_disarmed_and_targets_are_ignored_not_buffered(rig):
    node = rig.node
    rig.lease_enable()
    rig.step(2)
    assert node.enabled and not node.armed and node.state == "idle"
    st = node.state_dict()
    assert st["armed"] is False and st["disarm_reason"] in ("new session", "enable")
    assert "armed" in node.fields and rig.row()["armed"] == 0.0
    p0, q0 = rig.cmd_pose()
    q_before = node.q_cmd["arm"].copy()
    for _ in range(20):                                       # armed: false (and no arm_seq at all): ignored
        rig.target(p0 + [0.05, 0, 0], q0)
        rig.send({"type": "target", "t": None, "ees": {"arm": {"pos": list(p0 + [0.05, 0, 0]), "quat": list(q0)}}},
                 latch=False)
        rig.step(1)
    assert all(b.kind is None for b in node.buffers.values())   # not buffered
    assert np.max(np.abs(node.q_cmd["arm"] - q_before)) < 1e-9
    assert node.state == "idle" and node.state_dict()["latch"]["ignored_disarmed"] >= 40
    assert rig.row()["hold"] == 1.0 and node.state_dict()["hold_reason"].startswith("disarmed")


def test_arm_tracks_and_publishes_armed(rig):
    node = rig.node
    rig.lease_enable()
    rig.step(2)
    p0, q0 = rig.cmd_pose()
    rig.arm_now(p0, q0)
    goal = moving_goal(rig, p0, q0, 80)
    r = rig.row()
    assert node.state == "tracking" and r["armed"] == 1.0
    assert np.linalg.norm(np.array([r[f"arm_cmd_{k}"] for k in "xyz"]) - goal) < 0.003
    st = node.state_dict()
    assert st["armed"] is True and st["disarm_reason"] is None and st["latch"]["arm_seq"] == 1
    assert any(e["msg"].startswith("ARMED") for e in node.events.list())


def test_alive_never_arms(rig):
    node = rig.node
    rig.lease_enable()
    rig.step(2)
    rig.aseq, rig.armed = 5, True
    for _ in range(5):
        rig.send({"type": "alive"})
        rig.step(1)
    assert not node.armed
    p0, q0 = rig.cmd_pose()
    rig.target(p0, q0)                                        # the same arm_seq in a target arms (the press)
    rig.step(1)
    assert node.armed


def test_an_arming_datagram_that_fails_leaves_nothing(rig):
    node = rig.node
    rig.lease_enable()
    rig.step(2)
    p0, q0 = rig.cmd_pose()
    rig.aseq, rig.armed = 1, True
    rig.target(p0, q0, gripper=7.0)                            # invalid gripper: dropped, not armed
    rig.step(1)
    assert not node.armed and all(b.kind is None for b in node.buffers.values())
    rig.target(p0, q0)
    rig.step(1)
    assert node.armed


# -- every disarm trigger: one event each, the targets cleared, holding ------------------------------------------

def _armed_and_moving(rig, base=False, **kw):
    rig.lease_enable(base=base)
    rig.step(2)
    p0, q0 = rig.cmd_pose()
    rig.arm_now(p0, q0)
    moving_goal(rig, p0, q0, 10, **kw)
    assert rig.node.state == "tracking"
    return p0, q0


def _assert_disarmed(rig, why, events_before):
    node = rig.node
    assert not node.armed, why
    st = node.state_dict()
    assert why in st["disarm_reason"], (why, st["disarm_reason"])
    assert all(b.kind is None for b in node.buffers.values())
    assert len(rig.disarm_events()) == events_before + 1, rig.disarm_events()


@pytest.mark.parametrize("trigger", ["armed_false", "no_latch_fields", "post_disarm", "stale", "lease_release",
                                     "lease_lapse", "new_session", "arm_pause", "fault", "estop"])
def test_each_disarm_trigger(rig, trigger):
    node = rig.node
    p0, q0 = _armed_and_moving(rig)
    n_ev = len(rig.disarm_events())
    why = {"armed_false": "client: armed false", "no_latch_fields": "without armed", "post_disarm": "POST /disarm",
           "stale": "input stale", "lease_release": "client lease", "lease_lapse": "client lease",
           "new_session": "new session", "arm_pause": "arm paused", "fault": "fault: arm is fault",
           "estop": "e-stop"}[trigger]
    if trigger == "armed_false":
        rig.armed = False
        rig.target(p0, q0)
        rig.step(1)
    elif trigger == "no_latch_fields":
        rig.send({"type": "target", "t": None, "ees": {"arm": {"pos": list(p0), "quat": list(q0)}}}, latch=False)
        rig.step(1)
    elif trigger == "post_disarm":
        from fastapi.testclient import TestClient
        from robot_utils.nodes.wbc.server import create_app
        c = TestClient(create_app(node, start=False))              # (no lifespan: the node stays up)
        r = c.post("/disarm", json={"reason": "test", "source": "pytest"})          # no lease token
        assert r.status_code == 200 and r.json()["was_armed"] is True and r.json()["armed"] is False
    elif trigger == "stale":
        node.renew(rig.tok)
        rig.step(30)                                           # 0.3 s > hold_timeout without a datagram
    elif trigger == "lease_release":
        node.release(rig.tok)
        rig.step(1)
    elif trigger == "lease_lapse":
        node.session_token = None                              # the datagrams stop renewing it (renewals failing)
        for _ in range(110):                                   # 1.1 s: input fresh, the lease (1 s) lapses
            rig.target(p0, q0)
            rig.step(1)
    elif trigger == "new_session":
        out = node.open_session(rig.tok)
        rig.step(1)
        assert out["session"] != rig.session
    elif trigger == "arm_pause":
        with rig.arm.lock:
            rig.arm.paused = True
        rig.step(3)
    elif trigger == "fault":
        for _ in range(50):                                    # past the arm's grace after enable (0.5 s)
            rig.target(p0, q0)
            rig.step(1)
        rig.arm.set_state("fault")
        rig.step(3)
    elif trigger == "estop":
        node.trigger_estop("test", "pytest")
    _assert_disarmed(rig, why, n_ev)
    if trigger == "estop":
        node.clear_estop(rig.tok)


def test_estimator_reset_and_pose_jump_disarm(model):
    d = tempfile.mkdtemp(prefix="wbca")
    clock = Clock(1000.0)
    node = WbcNode(model, arm=None, swerve=None, estimator="est", clock=clock, udp_host=None,
                   unix_socket=d + "/t.sock")
    tok, _ = node.acquire("t")
    node.enable(tok)
    sess = node.open_session(tok)["session"]
    pose = {"rid": 3.0, "x": 0.0}
    seq = [0]

    def step(n=1):
        for _ in range(n):
            clock.t += DT
            node.feed_estimator({"t": clock.t, "x": pose["x"], "y": 0.0, "theta": 0.0, "reset_id": pose["rid"],
                                 "odom_ok": 1.0})
            node.step()

    def send(aseq, armed=True):
        T = node.model.fk(node.model.config((pose["x"], 0, 0), node.q_cmd), "arm")
        from robot_utils.nodes.wbc.targets import R_to_quat
        seq[0] += 1
        node.receiver.handle(json.dumps({"type": "target", "session": sess, "seq": seq[0], "t": None,
                                         "armed": armed, "arm_seq": aseq,
                                         "ees": {"arm": {"pos": list(T.translation), "quat": list(R_to_quat(T.rotation))}}
                                         }).encode())
    step(3)
    send(1)
    step(1)
    assert node.armed
    pose["rid"] = 4.0                                          # an estimator reset
    step(1)
    assert not node.armed and "estimator reset" in node.disarm_reason
    for _ in range(5):                                         # the estimator is fine again: the old press stays off
        send(1)
        step(1)
    assert not node.armed and node.latch_counts["arm_refused"] >= 5
    send(2)
    step(1)
    assert node.armed
    pose["x"] = 0.5                                            # a pose jump (0.5 m between two rows)
    step(1)
    assert not node.armed and "pose jumped" in node.disarm_reason
    node.disable("done")


def test_an_estimator_dropout_disarms_and_its_return_never_resumes(model):
    """The estimator stream dead for 0.5 s while the client keeps sending armed datagrams: disarmed (not a mere
    hold); the stream back: no tracking, no motion, until a new arm_seq."""
    from robot_utils.nodes.wbc.targets import R_to_quat
    d = tempfile.mkdtemp(prefix="wbca")
    clock = Clock(1000.0)
    node = WbcNode(model, arm=None, swerve=None, estimator="est", clock=clock, udp_host=None,
                   unix_socket=d + "/t.sock")
    tok, _ = node.acquire("t")
    node.enable(tok)
    sess = node.open_session(tok)["session"]
    seq, live = [0], {"est": True}

    def step(n=1, aseq=1, goal=(0.0, 0.0, 0.0)):
        for _ in range(n):
            clock.t += DT
            if live["est"]:
                node.feed_estimator({"t": clock.t, "x": 0.0, "y": 0.0, "theta": 0.0, "reset_id": 3.0, "odom_ok": 1.0})
            T = node.model.fk(node.model.config((0, 0, 0), node.q_cmd), "arm")
            seq[0] += 1
            node.receiver.handle(json.dumps({
                "type": "target", "session": sess, "seq": seq[0], "t": None, "armed": True, "arm_seq": aseq,
                "ees": {"arm": {"pos": list(T.translation + np.asarray(goal)) if seq[0] > 3 else list(T.translation),
                                "quat": list(R_to_quat(T.rotation))}}}).encode())
            node.step()
    step(5)
    assert node.armed
    step(10, goal=(0.0, 0.0, 0.02))                            # tracking a moving target
    assert node.state == "tracking"
    live["est"] = False
    step(50, goal=(0.0, 0.0, 0.02))                            # 0.5 s without an estimator row, still sending
    assert not node.armed and node.disarm_reason.startswith("estimator:"), node.disarm_reason
    q = node.q_cmd["arm"].copy()
    live["est"] = True
    step(50, goal=(0.0, 0.0, 0.02))                            # the stream is back, the old press still sent
    assert not node.armed and node.state == "idle"
    assert np.max(np.abs(node.q_cmd["arm"] - q)) < 1e-12        # no motion
    assert node.latch_counts["arm_refused"] > 0
    step(1, aseq=2)                                            # a new press arms
    assert node.armed
    node.disable("done")


def test_arming_refuses_a_target_far_from_the_commanded_pose(rig):
    """An ARMING datagram whose target is more than arm_target_pos / arm_target_rot from the commanded EE is
    refused (counted, an event at most once a second); that press never arms; a new press at the pose arms."""
    node = rig.node
    rig.lease_enable()
    rig.step(2)
    p0, q0 = rig.cmd_pose()
    assert node.cfg.arm_target_pos == 0.02 and abs(node.cfg.arm_target_rot - np.radians(5)) < 1e-3
    rig.aseq, rig.armed = 1, True
    for _ in range(5):
        rig.target(p0 + [0.03, 0, 0], q0)                     # 30 mm off
        rig.step(1)
    # the first refuses the press (counted arm_far); the same press after it is an old arm_seq (arm_refused)
    assert not node.armed and node.latch_counts["arm_far"] == 1 and node.latch_counts["arm_refused"] == 4
    assert all(b.kind is None for b in node.buffers.values())
    assert sum("arm refused" in e["msg"] for e in node.events.list()) == 1
    rig.target(p0, q0)                                        # the same press, now at the pose: still refused
    rig.step(1)
    assert not node.armed
    from robot_utils.nodes.wbc.targets import R_to_quat, quat_to_R
    rot = R_to_quat(quat_to_R(q0) @ np.array([[1, 0, 0], [0, np.cos(0.105), -np.sin(0.105)],
                                               [0, np.sin(0.105), np.cos(0.105)]]))   # 6 deg off
    rig.aseq += 1
    rig.target(p0, rot)
    rig.step(1)
    assert not node.armed and node.latch_counts["arm_far"] == 2
    rig.aseq += 1
    rig.target(p0 + [0.015, 0, 0], q0)                        # within 20 mm: arms
    rig.step(1)
    assert node.armed


def test_disabled_rows_carry_no_pose_and_a_stale_riding_pose_after_x_is_refused(rig):
    """X (disable) -> the arm floats away -> Y with the client's stale riding pose: while disabled the rows carry NaN
    for the commanded and measured EE (nothing stale to capture); after the enable the stale target is refused."""
    node, arm = rig.node, rig.arm
    rig.lease_enable()
    rig.step(2)
    p0, q0 = rig.cmd_pose()
    rig.arm_now(p0, q0)
    node.disable("X")
    rig.step(2)
    r = rig.row()
    assert all(math.isnan(r[f"arm_{w}_{k}"]) for w in ("cmd", "meas") for k in ("x", "y", "z", "qw"))
    assert all(math.isnan(r[f"arm_q_cmd{i}"]) for i in range(1, 7))
    wait_for(lambda: arm.token is None, 3)
    with arm.lock:                                            # floating: the operator pushed the arm
        arm.q = arm.q + np.radians([10, -8, 6, 0, 0, 0])
    node.enable(rig.tok)
    rig.step(2)
    p1, q1 = rig.cmd_pose()
    assert np.linalg.norm(p1 - p0) > 0.03                     # seeded from where the arm is now
    rig.aseq += 1
    rig.target(p0, q0)                                        # Y with the stale riding pose
    rig.step(1)
    assert not node.armed and node.latch_counts["arm_far"] == 1
    rig.aseq += 1
    rig.target(p1, q1)                                        # a fresh press at the current command
    rig.step(1)
    assert node.armed


# -- what disarmed means for the motors -----------------------------------------------------------------------

def test_disarm_brakes_the_base_within_its_rate_limit(rig):
    node = rig.node
    rig.lease_enable(base=True)
    rig.step(2)
    p0, q0 = rig.cmd_pose()
    vel = {"mode": "commanded", "velocity": {"vx": 0.4, "vy": 0.0, "omega": 0.0, "frame": "base", "ttl": 0.5}}
    rig.arm_now(p0, q0, base=vel)
    for _ in range(60):
        rig.target(p0, q0, base=vel)
        rig.step(1)
    v0 = node.base_twist.copy()
    assert v0[0] > 0.35, v0
    rig.armed = False
    rig.target(p0, q0, base=vel)                               # armed: false: disarm; its velocity is ignored
    speeds = []
    for _ in range(60):
        rig.step(1)
        speeds.append(float(node.base_twist[0]))
    assert not node.armed
    steps = np.diff([v0[0]] + speeds)
    acc = node.cfg.max_accel * DT
    assert np.all(steps <= 1e-12) and np.all(steps >= -acc - 1e-9), steps       # braking within max_accel
    assert speeds[-1] == 0.0 and np.all(np.asarray(node.base.twists[-5:]) == 0.0)   # exactly zero, held
    rig.target(p0, q0, base=vel)                               # still disarmed: the velocity never comes back
    rig.step(10)
    assert np.all(node.base_twist == 0.0)


def test_disarm_freezes_the_gripper_and_stops_the_arm_at_its_commanded_joints(rig):
    node, arm = rig.node, rig.arm
    p0, q0 = _armed_and_moving(rig, gripper=0.5)
    rig.step(2)
    assert abs(arm.gap - 0.5 * node.gap_max) < 1e-9
    n_cmd = len(arm.commands)
    for _ in range(10):                                       # moving fast toward a far goal
        rig.target(p0 + [0.0, 0.0, 0.1], q0, gripper=0.5)
        rig.step(1)
    assert np.max(np.abs(node.v_cmd["arm"])) > 0.1                     # moving
    q_at = node.q_cmd["arm"].copy()
    rig.armed = False
    rig.target(p0 + [0.0, 0.0, 0.1], q0, gripper=0.9)         # disarm: this gripper value is ignored
    rig.step(30)
    assert np.max(np.abs(node.q_cmd["arm"] - q_at)) < 1e-12            # held at its commanded joints at once
    assert np.all(node.v_cmd["arm"] == 0.0)
    assert np.allclose(arm.commands[-1][1], q_at, atol=1e-12) and np.all(arm.commands[-1][2] == 0.0)
    assert abs(arm.gap - 0.5 * node.gap_max) < 1e-9                    # the gripper keeps its last goal
    assert len(arm.commands) > n_cmd                                   # the hold is commanded (not silence)
    assert all(math.isnan(c[3]) for c in arm.commands[-10:])           # no gripper goal while disarmed


# -- no recovery re-arms -------------------------------------------------------------------------------------

def test_no_recovery_rearms_and_the_old_arm_seq_is_refused(rig):
    node = rig.node
    p0, q0 = _armed_and_moving(rig)
    node.renew(rig.tok)
    rig.step(30)                                               # stale: disarmed
    assert not node.armed and "input stale" in node.disarm_reason
    for _ in range(10):                                        # fresh again, the same press (arm_seq 1): refused
        rig.target(p0, q0)
        rig.send({"type": "alive"})
        node.renew(rig.tok)                                    # a lease renew neither
        rig.step(1)
    assert not node.armed and node.state == "idle" and node.state_dict()["latch"]["arm_refused"] >= 10
    q = node.q_cmd["arm"].copy()
    rig.step(5)
    assert np.max(np.abs(node.q_cmd["arm"] - q)) < 1e-12       # nothing moves
    rig.aseq += 1                                              # a new press arms (at the commanded pose, as the
    rig.target(*rig.cmd_pose())                                # client's riding target is)
    rig.step(1)
    assert node.armed and node.arm_seq == 2


def test_a_new_session_disarms_and_keeps_the_arm_seq_floor(rig):
    node = rig.node
    p0, q0 = _armed_and_moving(rig)
    old = rig.session
    out = node.open_session(rig.tok)                           # the client reopens: disarmed
    rig.session = out["session"]
    assert out["arm_seq"] == 1                                 # the floor: the next press must exceed it
    rig.step(1)
    assert not node.armed
    for _ in range(5):                                         # still sending the old press, in both sessions
        rig.target(p0, q0)
        rig.send({"type": "target", "t": None, "ees": {"arm": {"pos": list(p0), "quat": list(q0)}}}, session=old)
        rig.step(1)
    assert not node.armed and node.receiver.counts["dropped"].get("unknown_session", 0) >= 5
    rig.aseq = out["arm_seq"] + 1
    rig.target(p0, q0)
    rig.step(1)
    assert node.armed


def test_enable_disarms_and_queued_datagrams_never_arm(rig):
    node = rig.node
    p0, q0 = _armed_and_moving(rig)
    node.disable("X")
    assert not node.armed and "disabled" in node.disarm_reason
    for _ in range(5):                                         # the client keeps sending its old press while disabled
        rig.target(p0, q0)
        rig.step(1)
    rig.target(p0, q0)                                         # queued, not yet applied, at enable
    node.enable(rig.tok)
    rig.step(3)
    rig.target(p0, q0)
    rig.step(2)
    assert not node.armed and node.disarm_reason == "enable"
    rig.aseq += 1
    rig.target(p0, q0)
    rig.step(1)
    assert node.armed


def test_arming_refused_while_paused_faulted_or_estopped(rig):
    node, arm = rig.node, rig.arm
    rig.lease_enable()
    rig.step(2)
    p0, q0 = rig.cmd_pose()
    with arm.lock:
        arm.paused = True
    rig.step(2)
    rig.aseq, rig.armed = 1, True
    rig.target(p0, q0)
    rig.step(1)
    assert not node.armed                                      # paused by a planned motion
    with arm.lock:
        arm.paused = False
    rig.step(2)
    rig.aseq += 1
    rig.target(p0, q0)
    rig.step(1)
    assert node.armed
    node.trigger_estop("t", "pytest")
    rig.aseq += 1
    rig.target(p0, q0)
    rig.step(1)
    assert not node.armed and not node.enabled
    node.clear_estop(rig.tok)


# -- caps not changeable while moving; clamp, don't drop -----------------------------------------------------

def test_caps_refused_while_the_base_moves(rig):
    from fastapi.testclient import TestClient
    from robot_utils.nodes.wbc.server import create_app
    node = rig.node
    rig.lease_enable(base=True)
    rig.step(2)
    p0, q0 = rig.cmd_pose()
    vel = {"mode": "commanded", "velocity": {"vx": 0.3, "ttl": 0.5}}
    rig.arm_now(p0, q0, base=vel)
    for _ in range(20):
        rig.target(p0, q0, base=vel)
        rig.step(1)
    assert node.base_twist[0] > 0.1
    c = TestClient(create_app(node, start=False))              # (no lifespan: the node stays up)
    for k, v in (("max_speed", 0.5), ("max_omega", 0.5), ("max_accel", 0.5), ("max_alpha", 1.0),
                 ("wheel_speed", 0.8)):
        r = c.post("/config", json={k: v})
        assert r.status_code == 422 and "while the base moves" in r.json()["detail"], (k, r.text)
    assert node.cfg.max_speed == 0.8
    assert c.post("/config", json={"max_speed": 0.8, "k_pos": 4.5}).status_code == 200   # same value: no change
    assert node.cfg.k_pos == 4.5
    # a datagram's base.limits while moving: ignored (the limits in force stay), counted
    rig.target(p0, q0, base={**vel, "limits": {"max_speed": 0.1}})
    rig.step(1)
    assert node.effective_hints().limits.max_speed is None and node.latch_counts["limits_ignored"] == 1
    rig.armed = False                                      # disarm: the base brakes to exactly zero
    rig.target(p0, q0)
    rig.step(60)
    assert np.all(node.base_twist == 0.0)
    assert c.post("/config", json={"max_speed": 0.5}).status_code == 200
    assert node.cfg.max_speed == 0.5
    # still: a datagram's limits apply
    p0, q0 = rig.cmd_pose()                                # (the base drove on: re-arm at the EE's current pose)
    rig.aseq, rig.armed = rig.aseq + 1, True
    rig.target(p0, q0, base={"mode": "locked", "limits": {"max_speed": 0.1}})
    rig.step(1)
    assert node.effective_hints().limits.max_speed == 0.1


def test_hint_velocities_beyond_the_caps_are_clamped_not_dropped(rig):
    node = rig.node
    rig.lease_enable(base=True)
    rig.step(2)
    p0, q0 = rig.cmd_pose()
    rig.arm_now(p0, q0)
    dropped = dict(node.receiver.counts["dropped"])
    rig.target(p0, q0, base={"mode": "commanded", "velocity": {"vx": 2.0, "vy": 0.0, "omega": -5.0, "ttl": 0.3}})
    rig.step(1)
    h = node.effective_hints()
    assert node.armed and node.receiver.counts["dropped"] == dropped        # not refused as invalid
    assert abs(h.velocity.vx - node.cfg.max_speed) < 1e-12 and abs(h.velocity.omega + node.cfg.max_omega) < 1e-12
    rig.target(p0, q0, base={"mode": "hint", "target": {"pose": {"x": 1.0, "y": 0.0, "vx": 3.0, "vy": 4.0},
                                                        "heading": {"yaw": 0.0, "omega": 2.0}}})
    rig.step(1)
    h = node.effective_hints()
    assert abs(math.hypot(h.target_pose.vx, h.target_pose.vy) - node.cfg.max_speed) < 1e-12
    assert abs(h.target_pose.vx / h.target_pose.vy - 0.75) < 1e-12          # direction kept
    assert abs(h.target_heading.omega - node.cfg.max_omega) < 1e-12
    assert node.latch_counts["clamped"] == 4
    assert node.state_dict()["latch"]["clamped"] == 4
    with pytest.raises(ValueError):                                         # POST /hints still refuses (422)
        node.set_hints(rig.tok, {"base": {"mode": "commanded", "velocity": {"vx": 2.0}}})


def test_arm_stale_default_is_200_ms(rig):
    assert rig.node.cfg.arm_stale == 0.2


# -- the WBC's own e-stop: stop tracking first, then the arm and the swerve in parallel -----------------------------

def test_estop_stops_tracking_first_then_fans_out_in_parallel(rig):
    from fastapi import FastAPI
    node = rig.node
    _armed_and_moving(rig)
    seen = {}
    sw = FastAPI()

    @sw.post("/estop")
    def sw_estop(body: dict):
        seen["swerve"] = (time.monotonic(), node.armed, node.base_twist.copy(), node.v_cmd["arm"].copy())
        return {"latched": True}
    sw_url, sw_srv, sw_th = serve(sw)
    node.swerve_target = sw_url
    fast = node.arm_http.estop

    def slow_arm_estop(*a, **kw):                             # a slow arm /estop must not delay the swerve's
        time.sleep(1.0)
        seen["arm"] = time.monotonic()
        return fast(*a, **kw)
    node.arm_http.estop = slow_arm_estop
    q_at = node.q_cmd["arm"].copy()
    t0 = time.monotonic()
    try:
        out = node.trigger_estop("test", "pytest")
    finally:
        stop((sw_srv, sw_th))
    t_sw, armed_then, twist_then, v_then = seen["swerve"]
    assert t_sw - t0 < 0.5, t_sw - t0                         # not behind the arm's 1 s
    assert seen["arm"] - t0 >= 1.0
    assert out["forwarded"] == {"arm": True, "swerve": True} and out["latched"]
    assert armed_then is False and np.all(twist_then == 0.0) and np.all(v_then == 0.0)   # tracking stopped first
    assert rig.arm.estopped is not None and not node.enabled
    assert np.allclose(rig.arm.commands[-1][1], q_at, atol=1e-12) and np.all(rig.arm.commands[-1][2] == 0.0)
    assert "e-stop" in node.disarm_reason
    node.clear_estop(rig.tok)


def _driving(rig):
    node = rig.node
    rig.lease_enable(base=True)
    rig.step(2)
    p0, q0 = rig.cmd_pose()
    vel = {"mode": "commanded", "velocity": {"vx": 0.4, "ttl": 0.5}}
    rig.arm_now(p0, q0, base=vel)
    for _ in range(40):
        rig.target(p0, q0, base=vel)
        rig.step(1)
    assert node.base.twists[-1][0] > 0.3
    return node.base, p0, q0, vel


def test_estop_sends_the_base_nothing_it_coasts(rig):
    """No zero twist (an active brake) reaches the swerve on an e-stop: not from the e-stop itself, not from the ticks
    while the latch is set before the disable; the swerve's own e-stop coasts."""
    node = rig.node
    base, p0, q0, vel = _driving(rig)
    n = len(base.twists)
    node.estop.trigger("test", "pytest")          # the latch alone (as at the start of trigger_estop) ...
    for _ in range(5):                           # ... and ticks before the disable runs
        rig.target(p0, q0, base=vel)
        rig.step(1)
    assert len(base.twists) == n and base.muted and not node.armed
    node.estop.clear()
    base2, p0, q0, vel = _driving_again(rig)
    n = len(base2.twists)
    node.trigger_estop("test", "pytest")
    assert len(base2.twists) == n and base2.closed is True   # nothing sent; disabled (coast) and released
    node.clear_estop(rig.tok)


def _driving_again(rig):
    node = rig.node
    node.disable("again")
    wait_for(lambda: rig.arm.token is None, 3)       # the arm lease is given back in the background
    node.enable(rig.tok)
    with node.lock:
        node.base = FakeBase()
    rig.session = node.open_session(rig.tok)["session"]
    rig.step(2)
    p0, q0 = rig.cmd_pose()
    vel = {"mode": "commanded", "velocity": {"vx": 0.4, "ttl": 0.5}}
    rig.aseq += 1
    rig.armed = True
    for _ in range(40):
        rig.target(p0, q0, base=vel)
        rig.step(1)
    assert node.armed and node.base.twists[-1][0] > 0.3
    return node.base, p0, q0, vel


def test_disable_sends_the_base_nothing_it_coasts(rig):
    node = rig.node
    base, p0, q0, vel = _driving(rig)
    n = len(base.twists)
    node.disable("X")
    rig.step(5)
    assert len(base.twists) == n and base.muted and base.closed is True


def test_swerve_base_mute_drops_the_queued_twist():
    from robot_utils.nodes.wbc.links import SwerveBase
    b = SwerveBase("http://127.0.0.1:9")
    b.send((0.3, 0.0, 0.0))
    b.mute()
    assert b._pending is None
    b.send((0.0, 0.0, 0.0))
    assert b._pending is None
