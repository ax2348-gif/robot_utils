"""The WBC's control-tick timing (docs/wbc.md "Control tick timing"): every tick's period and compute time measured;
overruns (> 1.5 dt) and late ticks (> Config.late_tick, 0.1 s) counted; a late tick disarms ("control tick overrun:
N ms"), one between 3 dt and late_tick is only an overrun; the arm command integrated over the measured period clamped
to [dt, 3 dt]. Lockstep on a hand-advanced clock against the
fake arm, and once in real time with a stalled controller."""
import json
import socket
import tempfile
import threading
import time

import numpy as np
import pytest

from helpers import Clock, serve, stop, wait_for
from wbc_fakes import FakeArm
from robot_utils.nodes.wbc.model import EESpec, WbcModel, arm_config_limits, fetch_urdf
from robot_utils.nodes.wbc.node import WbcNode

DT = 0.01


@pytest.fixture(scope="module")
def model():
    urdf, _ = fetch_urdf("local")
    return WbcModel(urdf, (EESpec(arm_limits=arm_config_limits()),))


class Rig:
    def __init__(self, node, clock, arm):
        self.node, self.clock, self.arm = node, clock, arm
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0

    def step(self, n=1, dt=DT):
        for _ in range(n):
            self.clock.t += dt
            self.node.step()
            time.sleep(0.0015)

    def row(self):
        return dict(zip(self.node.fields, self.node.hub.recent(1, "tick")[0]))

    def target(self, pos, quat, aseq=1):
        self.seq += 1
        self.udp.sendto(json.dumps({"type": "target", "session": self.session, "seq": self.seq, "t": None,
                                    "armed": True, "arm_seq": aseq,
                                    "ees": {"arm": {"pos": list(map(float, pos)), "quat": list(map(float, quat))}}}
                                   ).encode(), ("127.0.0.1", self.node.receiver.udp_port))
        time.sleep(0.003)

    def armed_and_moving(self):
        tok, _ = self.node.acquire("t")
        self.node.enable(tok)
        self.session = self.node.open_session(tok)["session"]
        self.step(2)
        r = self.row()
        p0 = np.array([r[f"arm_cmd_{k}"] for k in "xyz"])
        q0 = np.array([r[f"arm_cmd_q{k}"] for k in "xyzw"])
        self.goal = (p0 + [0.0, 0.0, 0.08], q0)
        self.target(p0, q0)                                   # armed at the commanded pose, then a goal
        self.step(1)
        for _ in range(8):
            self.target(*self.goal)
            self.step(1)
        assert self.node.armed and np.max(np.abs(self.node.v_cmd["arm"])) > 0.05


@pytest.fixture
def rig(model):
    d = tempfile.mkdtemp(prefix="wbco")
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


def test_a_late_tick_disarms_and_is_counted(rig):
    node = rig.node
    rig.armed_and_moving()
    assert node.cfg.late_tick == 0.1
    late0, o0 = node.tick["late"], node.tick["overruns"]
    rig.target(*rig.goal)
    rig.step(1, dt=0.045)                                     # 45 ms: beyond 3 dt but within late_tick: an overrun only
    assert node.armed and node.tick["late"] == late0 and node.tick["overruns"] == o0 + 1
    rig.target(*rig.goal)
    rig.step(1, dt=0.12)                                      # 120 ms since the previous tick (> late_tick)
    assert not node.armed and node.disarm_reason == "control tick overrun: 120 ms"
    assert node.tick["late"] == late0 + 1 and node.tick["overruns"] == o0 + 2
    assert any(e["msg"] == "DISARMED: control tick overrun: 120 ms" for e in node.events.list())
    r = rig.row()
    assert r["armed"] == 0.0 and r["late_ticks"] == late0 + 1 and abs(r["tick_period"] - 120.0) < 1e-6
    st = node.state_dict()["tick"]
    assert st["late"] == late0 + 1 and st["max_period_ms"] >= 120.0 and st["compute_ms"] > 0
    q = node.q_cmd["arm"].copy()
    for _ in range(5):                                        # the client still sends its press: never re-armed
        rig.target(*rig.goal)
        rig.step(1)
    assert not node.armed and np.max(np.abs(node.q_cmd["arm"] - q)) < 1e-12
    r = rig.row()                                             # a new press arms (at the commanded pose)
    rig.target([r[f"arm_cmd_{k}"] for k in "xyz"], [r[f"arm_cmd_q{k}"] for k in "xyzw"], aseq=2)
    rig.step(1)
    assert node.armed


def test_overruns_below_3_dt_are_counted_and_keep_tracking(rig):
    node = rig.node
    rig.armed_and_moving()
    o0, l0 = node.tick["overruns"], node.tick["late"]
    rig.target(*rig.goal)
    rig.step(1, dt=0.02)                                      # 2 dt: an overrun, not late
    assert node.armed and node.tick["overruns"] == o0 + 1 and node.tick["late"] == l0
    assert rig.row()["overruns"] == o0 + 1
    rig.target(*rig.goal)
    rig.step(1, dt=0.014)                                     # 1.4 dt: not an overrun
    assert node.tick["overruns"] == o0 + 1


@pytest.mark.parametrize("period,expect", [(0.02, 0.02), (0.025, 0.025), (0.004, 0.01), (0.01, 0.01)])
def test_the_arm_command_is_integrated_over_the_measured_period(rig, period, expect):
    node = rig.node
    rig.armed_and_moving()
    q = node.q_cmd["arm"].copy()
    rig.target(*rig.goal)
    rig.step(1, dt=period)
    assert node.armed
    v = node.v_cmd["arm"]
    assert np.max(np.abs(v)) > 0.05
    # (the hand clock is ~1e5 s: its differences carry ~1e-11 s of rounding)
    assert np.allclose(node.q_cmd["arm"] - q, v * expect, atol=1e-9, rtol=0), node.q_cmd["arm"] - q - v * expect


def test_the_joint_damper_uses_the_measured_step(model):
    """On a long step the velocity bound shrinks so the integrated step still stops short of the limit."""
    from robot_utils.nodes.swerve.constants import MODULES
    from robot_utils.nodes.wbc.controller import QPController
    from robot_utils.nodes.wbc.spec import make_config
    cfg = make_config()
    ctl = QPController(model, cfg, [(m.x, m.y) for m in MODULES])
    lo, hi = model.arm_limits("arm")
    q = (lo + hi) / 2
    q[0] = hi[0] - 0.01                                       # 10 mrad below the first joint's limit
    for ds in (cfg.dt, 2 * cfg.dt, 3 * cfg.dt):
        L, U, *_ = ctl._joint_box("arm", q, np.zeros(len(q)), ds)
        assert U[0] * ds <= cfg.limit_gain * 0.01 + 1e-12, (ds, U[0])


def test_real_time_a_stalled_tick_disarms(model):
    """The real control loop: one controller step stalls 150 ms (> late_tick); the next tick is late and disarms."""
    d = tempfile.mkdtemp(prefix="wbcr")
    arm = FakeArm(d + "/arm.joint.sock")
    url, srv, th = serve(arm.app)
    node = WbcNode(model, arm=url, swerve=None, estimator=None, udp_host="127.0.0.1", udp_port=0,
                   unix_socket=d + "/wbc.target.sock", arm_socket=d + "/arm.joint.sock")
    node.receiver.start()
    node._thread = threading.Thread(target=node._run, daemon=True)
    node._thread.start()
    stop_send = threading.Event()
    try:
        tok, _ = node.acquire("t")
        node.enable(tok)
        sess = node.open_session(tok)["session"]
        wait_for(lambda: node.hub.recent(1, "tick").shape[0] > 0, 2)
        L = dict(zip(node.fields, node.hub.recent(1, "tick")[0]))
        p0 = [L[f"arm_cmd_{k}"] for k in "xyz"]
        q0 = [L[f"arm_cmd_q{k}"] for k in "xyzw"]
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        def sender():
            seq = 0
            while not stop_send.is_set():
                seq += 1
                udp.sendto(json.dumps({"type": "target", "session": sess, "seq": seq, "t": None, "armed": True,
                                       "arm_seq": 1, "ees": {"arm": {"pos": p0, "quat": q0}}}).encode(),
                           ("127.0.0.1", node.receiver.udp_port))
                time.sleep(0.01)
        threading.Thread(target=sender, daemon=True).start()
        wait_for(lambda: node.armed, 2)
        inner = node.ctl.step
        stalled = []

        def slow_step(snap):
            if not stalled:
                stalled.append(1)
                time.sleep(0.15)
            return inner(snap)
        node.ctl.step = slow_step
        wait_for(lambda: not node.armed, 2)
        assert node.disarm_reason.startswith("control tick overrun:"), node.disarm_reason
        ms = float(node.disarm_reason.split(":")[1].split("ms")[0])
        assert 145 <= ms < 400, ms
        assert node.tick["late"] >= 1 and node.tick["max_compute_ms"] >= 145
        time.sleep(0.2)
        assert not node.armed                                  # the sender's old press never re-arms
    finally:
        stop_send.set()
        node._stop.set()
        node._thread.join(2)
        if node.enabled:
            node.disable("test end")
        node.receiver.close()
        arm.close()
        stop((srv, th))
