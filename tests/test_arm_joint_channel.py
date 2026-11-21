"""The arm's internal joint channel (nodes/arm/stream.py, joint_channel.py; docs/arm.md "Internal joint channel"):
sessions, gating, preemption, staleness, validation, state packets. The simulated bus stepped by hand on a fake
clock; datagrams handed to the node directly (``stream_datagram``) with a recording channel, plus one test over the
real socket."""
import math
import os
import stat
import tempfile
import threading

import numpy as np
import pytest

from helpers import Clock, wait_for
from robot_utils.nodes.arm import config as C
from robot_utils.nodes.arm.bus import SimBus
from robot_utils.nodes.arm.node import COL, DT, FIELDS, LEASE_HEARTBEAT, SOURCES, STATES, ArmNode
from robot_utils.nodes.common import fastpath as F
from robot_utils.nodes.common.lease import RequestError
from robot_utils.nodes.common.locks import LockStore

Q0 = (0.0, -0.6, -0.9, 0.0, 0.5, 0.0)
ADDR = "/fake/wbc.sock"


@pytest.fixture(scope="session", autouse=True)
def _planner_process():
    """Start the planner process once (spawning it takes a couple of seconds)."""
    from robot_utils.nodes.arm.kin import ArmKinematics
    from robot_utils.nodes.arm.planner import Planner
    k = ArmKinematics()
    Planner.run("ik", k, k.fk(np.array(Q0)), np.array(Q0))
    yield


class Recorder:
    """Stands in for JointChannel: keeps what the node sends."""

    def __init__(self):
        self.sent = []

    def send(self, data, addr):
        self.sent.append((data, addr))
        return True

    def states(self):
        return [F.unpack_state(d) for d, _ in self.sent if not F.is_control(d)]

    def controls(self):
        return [F.decode_control(d) for d, _ in self.sent if F.is_control(d)]

    def close(self):
        pass


class Rig:
    def __init__(self, node, clock, sims):
        self.node, self.clock, self.sims = node, clock, sims
        self.ch = node.joint_channel = Recorder()
        self.seq = 0
        self.session = None

    @property
    def sim(self):
        return self.sims[-1]

    def step(self, n=1):
        for _ in range(n):
            self.clock.t += DT
            self.node.step()

    def blocking(self, fn, *a, **kw):
        out = {}

        def go():
            try:
                out["v"] = fn(*a, **kw)
            except Exception as e:  # noqa: BLE001
                out["e"] = e
        th = threading.Thread(target=go)
        th.start()
        for _ in range(2000):
            if not th.is_alive():
                break
            self.step(1)
            th.join(0.001)
        th.join(1)
        if "e" in out:
            raise out["e"]
        return out["v"]

    def ready(self):
        self.tok, _ = self.node.acquire("wbc")
        self.blocking(self.node.enable, self.tok)
        self.step(5)
        return self

    def hello(self, token="__mine__", addr=ADDR):
        r = self.node.stream_datagram(F.encode_control(F.hello(self.tok if token == "__mine__" else token, "wbc")),
                                      addr)
        msg = F.decode_control(r) if r is not None else None
        if msg and msg["type"] == "welcome":
            self.session, self.seq = msg["session"], 0
        return msg

    def goal(self, q, qd=(0.0,) * 6, gap=math.nan, seq=None, mode=F.MODE_GOAL, t_basis=math.nan, addr=ADDR,
             session=None):
        if seq is None:
            self.seq += 1
            seq = self.seq
        self.node.stream_datagram(F.pack_command(self.session if session is None else session, seq, q, qd, gap,
                                                 mode, t_basis=t_basis), addr)

    def q(self):
        return self.node.reading.q.copy()

    def row(self):
        return dict(zip(FIELDS, self.node.hub.recent(1, "tick")[0]))

    def dropped(self):
        return self.node.stream_session.dropped


@pytest.fixture
def rig(tmp_path):
    clock = Clock()
    sims = []

    def factory():
        from reBotArm_control_py.actuator.rebotarm import joint_limits, load_cfg
        b = SimBus(Q0, clock=clock, limits=joint_limits(load_cfg(C.PROFILES["soft"])))
        sims.append(b)
        return b

    node = ArmNode(factory, locks=LockStore(tmp_path / "locks"), clock=clock)
    r = Rig(node, clock, sims)
    r.step(1)
    return r


def stream_to(rig, q_goal, s, qd=(0.0,) * 6):
    """Send the same goal every tick for s seconds (a client at 100 Hz)."""
    for _ in range(int(round(s / DT))):
        rig.goal(q_goal, qd)
        rig.step(1)


# -- session -----------------------------------------------------------------------------------------------

def test_hello_needs_the_lease_token(rig):
    r = rig.hello(token=None)
    assert r["type"] == "refused" and "token" in r["reason"]
    rig.tok, _ = rig.node.acquire("wbc")
    assert rig.hello(token="wrong")["type"] == "refused"
    assert rig.node.stream_session is None
    assert rig.node.stream_datagram(F.encode_control(F.hello(rig.tok, "wbc")), None) is None   # unbound: no reply
    assert rig.node.stream_bad["unbound"] == 1
    w = rig.hello()
    assert w["type"] == "welcome" and w["stale_ms"] == 30.0 and 0 < w["session"] < 2 ** 32
    assert len(w["limits"]["lo"]) == 6 and w["limits"]["max_lead"] == pytest.approx(math.radians(20))
    st = rig.node.state_dict()["joint_channel"]
    assert st["session"]["id"] == w["session"] and st["session"]["client"] == "wbc" and st["session"]["address"] == ADDR


def test_state_packets_every_iteration_while_open(rig):
    rig.tok, _ = rig.node.acquire("wbc")
    rig.step(3)
    assert rig.ch.states() == []                      # no session: nothing sent
    rig.hello()
    rig.step(5)
    sts = rig.ch.states()
    assert len(sts) == 5 and all(s.session == rig.session for s in sts)
    assert [s.tick for s in sts] == list(range(sts[0].tick, sts[0].tick + 5))
    s = sts[-1]
    assert s.state == STATES.index("uninitialized") and s.source == SOURCES.index("none") and s.flags == 0
    assert np.allclose(s.q, rig.q(), atol=1e-6) and all(math.isnan(v) for v in s.q_cmd)
    assert s.t_tick == pytest.approx(rig.clock.t) and s.read_age == 0.0
    assert all(d[1] == ADDR for d in rig.ch.sent)


def test_goal_engages_and_tracks(rig):
    rig.ready().hello()
    q = rig.q()
    goal = q + np.radians([5, -3, 4, 0, 2, 0])
    with rig.node._lock:
        pass
    rig.goal(goal)
    with rig.node._lock:
        assert rig.node._arm_engaged() and not rig.node.servo      # accepted, engages on the next tick
    with pytest.raises(RequestError):
        rig.node.set_profile(rig.tok, "safe")                      # counts as under command already
    stream_to(rig, goal, 1.5)
    assert np.allclose(rig.sim.q, goal, atol=2e-3)
    row = rig.row()
    assert row["source"] == SOURCES.index("stream") and row["stream_seq"] == rig.seq
    assert row["stream_age"] == pytest.approx(DT, abs=1e-9)
    s = rig.ch.states()[-1]
    assert s.flags & F.STATE_SERVOING and s.source == SOURCES.index("stream") and s.seq == rig.seq
    assert np.allclose(s.q_cmd, goal, atol=1e-3)
    st = rig.node.state_dict()
    assert st["source"] == "stream" and st["joint_channel"]["goal"]["seq"] == rig.seq
    assert st["joint_channel"]["session"]["counts"]["goals"] == rig.seq


def test_datagrams_renew_the_lease(rig):
    rig.ready().hello()
    q = rig.q()
    for _ in range(int(3 * LEASE_HEARTBEAT / DT)):            # no HTTP renew for three heartbeats
        rig.goal(q)
        rig.step(1)
    assert rig.node.lease.held and rig.node.enabled and rig.node.stream_session is not None
    for _ in range(int(2 * LEASE_HEARTBEAT / DT)):            # keep-alives renew as well
        rig.goal(q, mode=F.MODE_KEEPALIVE)
        rig.step(1)
    assert rig.node.lease.held
    rig.step(int(1.2 * LEASE_HEARTBEAT / DT))                 # silence: it lapses and closes the session
    assert not rig.node.lease.held and rig.node.stream_session is None
    assert rig.ch.controls()[-1]["type"] == "bye" and "lapsed" in rig.ch.controls()[-1]["reason"]


def test_seq_out_of_order_and_duplicates_are_dropped(rig):
    rig.ready().hello()
    q = rig.q()
    rig.goal(q, seq=5)
    rig.goal(q + 0.01, seq=5)                                 # duplicate
    rig.goal(q + 0.02, seq=3)                                 # older
    assert rig.dropped() == {"old": 2} and rig.node.stream.seq == 5 and np.allclose(rig.node.stream.q, q)
    rig.goal(q, seq=6, mode=F.MODE_KEEPALIVE)
    rig.goal(q + 0.01, seq=6)                                 # seq is shared with keep-alives
    assert rig.dropped()["old"] == 3
    rig.goal(q, seq=9, session=rig.session + 1 if rig.session < 2 ** 32 - 1 else 1)   # wrong session
    rig.goal(q, seq=9, addr="/fake/other.sock")                                         # wrong address
    assert rig.node.stream_bad["session"] == 1 and rig.dropped()["addr"] == 1
    rig.goal(q, seq=7)
    assert rig.node.stream.seq == 7


# -- validation ----------------------------------------------------------------------------------------------

def test_bounds_lead_gap_and_nan_are_dropped_without_an_event_flood(rig):
    rig.ready().hello()
    n = rig.node
    q = rig.q()
    ev0 = len(n.events.list())
    nan = q.copy()
    nan[2] = math.nan
    far = q.copy()
    far[0] += math.radians(25)                                # past the 20° lead bound
    out = q.copy()
    out[1] = n.kin.hi[1] + math.radians(5)                    # outside the joint limits
    inf_qd = np.zeros(6)
    inf_qd[3] = math.inf
    for _ in range(50):
        rig.goal(nan)
        rig.goal(q, inf_qd)
        rig.goal(far)
        rig.goal(out)
        rig.goal(q, gap=0.2)                                  # 200 mm: wider than the gripper
        rig.goal(q, gap=math.inf)
    d = rig.dropped()
    assert d["nonfinite"] == 150 and d["lead"] == 50 and d["limits"] == 50 and d["gap"] == 50
    assert n.stream is None and not n.servo
    assert len(n.events.list()) - ev0 <= 5                   # one event per reason, not one per datagram
    rig.step(3)
    assert not n.servo                                        # nothing engaged


def test_engage_limit_is_checked_in_the_receiver(rig):
    rig.ready().hello()
    n = rig.node
    rig.sim.q[1] = n.kin.hi[1] + math.radians(8)              # moved by hand past a limit while floating
    rig.step(2)
    ev0 = len(n.events.list())
    for _ in range(100):
        rig.goal(np.clip(rig.q(), n.kin.lo, n.kin.hi))
        rig.step(1)
    assert rig.dropped()["engage"] == 100 and not n.servo
    assert len(n.events.list()) - ev0 <= 2                   # not a refusal event per tick


def test_command_step_backstops_a_nonfinite_goal(rig):
    rig.ready().hello()
    n = rig.node
    q = rig.q()
    stream_to(rig, q, 0.2)
    with n._lock:
        n.stream.q = np.full(6, math.nan)                     # past the receiver's checks somehow
    rig.step(2)                                               # (within stale_ms: the goal is still in force)
    assert np.all(np.isfinite(n.q_cmd)) and n.nonfinite_goals == 2
    assert n.state_dict()["joint_channel"]["nonfinite_goals"] == 2


def test_feedforward_rate_is_limited(rig):
    rig.ready().hello()
    n = rig.node
    q = rig.q()
    stream_to(rig, q, 0.1)
    qd = np.array([1.0, 0, 0, 0, 0, 0])
    prev = n.qd_ff.copy()
    for k in range(15):
        rig.goal(q + qd * DT * (k + 1), qd)
        rig.step(1)
        assert abs(n.qd_ff[0] - prev[0]) <= n.soft.joint_acc[0] * DT + 1e-9
        prev = n.qd_ff.copy()
    assert n.qd_ff[0] == pytest.approx(1.0)                   # reached after 10 ticks at 10 rad/s²


# -- gating ---------------------------------------------------------------------------------------------------

def test_goals_gated_until_enabled_and_disable_closes(rig):
    rig.tok, _ = rig.node.acquire("wbc")
    rig.hello()
    rig.step(2)
    q = rig.q()
    rig.goal(q)                                               # not enabled yet
    assert rig.dropped()["not_enabled"] == 1 and rig.node.stream is None
    rig.blocking(rig.node.enable, rig.tok)
    rig.step(3)
    assert not rig.node.servo                                 # the earlier datagram does not engage
    stream_to(rig, q, 0.1)
    assert rig.node.servo
    rig.node.disable(rig.tok)
    assert rig.node.stream is None and rig.node.stream_session is None
    assert rig.ch.controls()[-1] == {"type": "bye", "reason": "disabled"}
    rig.hello()                                               # a new session while disabled: goals refused
    rig.goal(q)
    rig.step(2)
    assert rig.dropped()["not_enabled"] == 1 and not rig.node.servo


def test_fault_keeps_the_session_and_sends_state(rig):
    rig.ready().hello()
    q = rig.q()
    stream_to(rig, q, 0.2)
    rig.sim.fault_code = 0xD
    rig.step(1)
    n = rig.node
    assert n.state == "fault" and n.stream is None and n.stream_session is not None
    k = len(rig.ch.states())
    rig.step(5)
    sts = rig.ch.states()[k:]
    assert len(sts) == 5 and all(s.state == STATES.index("fault") for s in sts)
    rig.goal(q)
    assert rig.dropped()["not_enabled"] == 1
    rig.sim.fail_reads = True                                 # reads failing as well: still one packet per tick
    k = len(rig.ch.states())
    rig.step(4)
    assert len(rig.ch.states()) - k == 4


def test_estop_and_lease_release_close_the_session(rig):
    rig.ready().hello()
    q = rig.q()
    stream_to(rig, q, 0.2)
    rig.node.trigger_estop("test", "pytest")
    assert rig.node.stream_session is None and rig.node.stream is None
    assert rig.ch.controls()[-1] == {"type": "bye", "reason": "e-stop"}
    assert rig.hello()["type"] == "refused"                  # latched: no new session
    rig.node.clear_estop(rig.tok)
    rig.blocking(rig.node.enable, rig.tok)
    rig.hello()
    rig.node.release(rig.tok)
    assert rig.node.stream_session is None and rig.ch.controls()[-1]["reason"] == "lease released"


def test_bye_ramps_to_a_hold_and_a_new_hello_replaces(rig):
    rig.ready().hello()
    q = rig.q()
    stream_to(rig, q, 0.2)
    first = rig.session
    rig.hello(addr="/fake/wbc2.sock")
    assert rig.session != first and rig.ch.controls()[-1]["reason"].startswith("replaced")
    assert rig.ch.sent[-1][1] == ADDR                         # the bye went to the old client
    rig.node.stream_datagram(F.encode_control(F.bye()), ADDR)  # the old client's bye: not this session's
    assert rig.node.stream_session is not None
    rig.node.stream_datagram(F.encode_control(F.bye()), "/fake/wbc2.sock")
    assert rig.node.stream_session is None
    rig.step(20)
    assert rig.node.servo and rig.node.stream.ramp_v is not None   # holding where the ramp ended


# -- preemption --------------------------------------------------------------------------------------------------

def test_track_and_stream_preempt_each_other(rig):
    rig.ready().hello()
    n = rig.node
    q = rig.q()
    stream_to(rig, q, 0.2)
    T = n.kin.fk(rig.sim.q)
    assert n.track_cmd(rig.tok, T)["ok"]
    assert n.stream is None and n.track is not None
    rig.step(2)
    assert rig.row()["source"] == SOURCES.index("track")
    rig.goal(rig.q())
    assert n.track is None and n.stream is not None
    rig.step(2)
    assert rig.row()["source"] == SOURCES.index("stream")


@pytest.mark.parametrize("kind", ["home", "stow", "servo"])
def test_planned_motions_pause_the_stream(rig, kind):
    rig.ready().hello()
    n = rig.node
    q = rig.q()
    stream_to(rig, q, 0.2)
    if kind == "home":
        m = n.home(rig.tok)
    elif kind == "stow":
        m = n.stow(rig.tok)["motion"]
    else:
        m = n.servo_cmd(rig.tok, name="home")
    assert n.stream is None and n.stream_session.paused
    t_end = m["duration"] + 1.5
    dropped = 0
    for _ in range(int(t_end / DT)):
        if n.motion is None:
            break
        rig.goal(rig.q())                                     # the client keeps streaming where the arm is
        dropped += 1
        rig.step(1)
        assert n.motion is None or n.motion.phase in ("executing", "settling")
        if n.motion is not None:
            s = rig.ch.states()[-1]
            assert s.flags & F.STATE_PAUSED
    assert n.last_motion.phase == "reached", n.last_motion.reason          # never preempted by the stream
    assert rig.dropped()["paused"] == dropped
    assert np.allclose(rig.sim.q, n.settings.named_poses["home"], atol=5e-3)
    if kind == "stow":
        rig.step(1)
        assert n.state == "off"
        return
    rig.goal(rig.q())                                         # the first goal after it ended resumes
    assert n.stream is not None and not n.stream_session.paused
    rig.step(2)
    assert rig.row()["source"] == SOURCES.index("stream")


# -- staleness -----------------------------------------------------------------------------------------------------

def test_stale_ramps_to_a_hold_then_resumes(rig):
    rig.ready().hello()
    n = rig.node
    v = np.array([0.8, 0.0, 0.0, 0.0, 0.0, 0.0])              # rad/s on joint1
    q = rig.q()
    for k in range(40):                                       # a moving goal with a matching feedforward
        rig.goal(q + v * DT * (k + 1), v)
        rig.step(1)
    assert n.v_cmd[0] == pytest.approx(0.8, abs=1e-4)
    vs = []
    for _ in range(20):                                       # the client goes silent
        rig.step(1)
        vs.append(n.v_cmd[0])
    acc = n.soft.joint_acc[0]
    assert vs[0] == pytest.approx(0.8, abs=1e-4)              # the goal moves on at its qd until stale_ms
    dec = -np.diff(vs)
    assert np.all(dec <= acc * DT + 1e-9)                     # then no instant brake: at most joint_acc
    assert vs[-1] == 0.0 and n.stream.ramp_v is not None and n.state_dict()["joint_channel"]["session"]["stale"]
    assert n.stream_session.counts["stale"] == 1
    assert rig.ch.states()[-1].flags & F.STATE_STALE
    assert abs(n.qd_ff[0]) < 1e-12                            # the feedforward decayed too
    hold = n.q_cmd.copy()
    rig.step(10)
    assert np.allclose(n.q_cmd, hold)                         # holding
    stream_to(rig, hold + np.radians([2, 0, 0, 0, 0, 0]), 1.0)   # fresh again: resumes
    assert not n.state_dict()["joint_channel"]["session"]["stale"] and n.stream.ramp_v is None
    assert np.allclose(rig.sim.q[0], hold[0] + math.radians(2), atol=2e-3)
    assert not rig.ch.states()[-1].flags & F.STATE_STALE


def test_row_latency_columns(rig):
    rig.ready().hello()
    q = rig.q()
    rig.step(1)
    for _ in range(20):
        s = rig.ch.states()[-1]                               # the client answers the newest state packet
        rig.goal(q, t_basis=s.t_tick)
        rig.step(1)
    row = rig.row()
    assert row["stream_lat"] == pytest.approx(DT, abs=1e-9)   # computed from the previous tick's state
    assert row["stream_age"] == pytest.approx(DT, abs=1e-9)
    assert list(FIELDS[-3:]) == ["stream_seq", "stream_age", "stream_lat"] and COL["stream_lat"] == len(FIELDS) - 1


def test_sources_append_only():
    assert SOURCES[:8] == ["none", "hold", "track", "servo", "pose", "home", "stow", "chunk"]
    assert SOURCES[8] == "stream"


# -- the real socket -------------------------------------------------------------------------------------------------

def test_over_the_socket(tmp_path):
    d = tempfile.mkdtemp(prefix="rujc")                       # short path: macOS limits socket paths to ~104 bytes
    path = os.path.join(d, "arm.joint.sock")
    from reBotArm_control_py.actuator.rebotarm import joint_limits, load_cfg
    limits = joint_limits(load_cfg(C.PROFILES["soft"]))
    node = ArmNode(lambda: SimBus(Q0, limits=limits), locks=LockStore(tmp_path / "locks"), joint_socket=path)
    node.start()
    c = None
    try:
        assert stat.S_IMODE(os.lstat(path).st_mode) == 0o600
        wait_for(lambda: node.state == "uninitialized", 10)
        tok, _ = node.acquire("wbc")
        node.enable(tok)
        c = F.Client(path, os.path.join(d, "c.sock"))
        assert c.hello("nope")["type"] == "refused"
        w = c.hello(tok, "test")
        assert w["type"] == "welcome"
        st = c.recv(1.0)
        assert isinstance(st, F.State) and st.session == w["session"]
        goal = np.array(st.q) + np.radians([3, 0, 0, 0, 0, 0])
        for _ in range(100):                                  # paced by the state packets: one goal per tick
            st = c.latest_state(1.0)
            c.send(goal, t_basis=st.t_tick)
        wait_for(lambda: abs(c.latest_state(1.0).q[0] - goal[0]) < 2e-3, 5)
        assert node.state_dict()["source"] == "stream"
        c.bye()
        wait_for(lambda: node.stream_session is None, 2)
    finally:
        if c is not None:
            c.close()
        node.close()
        os.rmdir(d)
    assert not os.path.exists(path)
