"""The WBC node: leases, enable with rollback, the datagram session, tracking, staleness, drops, cross-bus faults,
e-stop fan-out, the stream. Lockstep tests run the node's ``step()`` on a hand-advanced clock against a fake arm
(HTTP + joint-channel peer, tests/wbc_fakes.py); the integration test runs the real swerve node on its virtual
SPARKs and the real estimator in real time. docs/wbc.md."""
import json
import math
import socket
import tempfile
import time

import httpx
import numpy as np
import pytest

from helpers import Clock, serve, stop, wait_for
from wbc_fakes import Q0, FakeArm
from robot_utils.nodes.common.lease import RequestError
from robot_utils.nodes.wbc.model import EESpec, WbcModel, arm_config_limits, fetch_urdf
from robot_utils.nodes.wbc.node import STATES, WbcNode
from robot_utils.nodes.wbc.targets import R_to_quat

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
        self.session = None

    def step(self, n=1):
        for _ in range(n):
            self.clock.t += DT
            self.node.step()
            time.sleep(0.0015)                 # the arm peer answers the command (its own thread)

    def row(self):
        return dict(zip(self.node.fields, self.node.hub.recent(1, "tick")[0]))

    def lease_enable(self):
        self.tok, _ = self.node.acquire("test")
        self.node.enable(self.tok)
        out = self.node.open_session(self.tok)
        self.session = out["session"]
        return out

    def send(self, msg, raw=None):
        self.seq += 1
        m = {"session": self.session, "seq": self.seq, **msg}
        self.udp.sendto(raw if raw is not None else json.dumps(m).encode(), ("127.0.0.1", self.node.receiver.udp_port))
        time.sleep(0.003)

    def cmd_pose(self):
        r = self.row()
        return np.array([r[f"arm_cmd_{k}"] for k in "xyz"]), np.array([r[f"arm_cmd_q{k}"] for k in "xyzw"])

    def target(self, pos, quat, **kw):
        self.send({"type": "target", "t": None, "ees": {"arm": {"frame": "world", "pos": list(map(float, pos)),
                                                                "quat": list(map(float, quat)), **kw}}})

    def tcp_of_arm(self):
        m = self.node.model
        return m.fk(m.config((0, 0, 0), {"arm": self.arm.q}), "arm")


@pytest.fixture
def rig(model):
    d = tempfile.mkdtemp(prefix="wbc")
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


def test_enable_session_track_and_stream(rig):
    node, arm = rig.node, rig.arm
    with pytest.raises(RequestError):
        node.enable(None)                                   # the WBC's own lease first
    out = rig.lease_enable()
    assert out["udp_port"] == node.receiver.udp_port and out["unix_socket"].endswith("wbc.target.sock")
    assert arm.token is not None and arm.enabled and arm.session is not None
    rig.step(5)
    assert node.state == "idle"
    p0, q0 = rig.cmd_pose()
    T0 = rig.tcp_of_arm()
    assert np.allclose(p0, T0.translation, atol=1e-9)        # seeded from the measured joints
    # a world target at the commanded pose: no jump
    qc = node.q_cmd["arm"].copy()
    rig.target(p0, q0)
    rig.step(3)
    assert node.state == "tracking" and np.max(np.abs(node.q_cmd["arm"] - qc)) < 1e-9
    # a move: the fake arm follows the commands; commanded and measured agree, the target is reached
    goal = p0 + [0.04, -0.03, 0.03]
    for _ in range(150):
        rig.target(goal, q0, gripper=0.5)
        rig.step(1)
    r = rig.row()
    assert np.linalg.norm(np.array([r[f"arm_cmd_{k}"] for k in "xyz"]) - goal) < 0.002
    assert np.linalg.norm(rig.tcp_of_arm().translation - goal) < 0.003
    assert np.linalg.norm(np.array([r[f"arm_meas_{k}"] for k in "xyz"]) - rig.tcp_of_arm().translation) < 0.003
    assert r["status"] == 1 and r["state"] == STATES.index("tracking") and r["solve_ms"] > 0
    assert r["arm_gripper"] == 0.5 and abs(arm.gap - 0.5 * 0.0496) < 1e-6
    assert all(math.isfinite(r[k]) for k in ("arm_twist_vx", "arm_err_pos", "input_age", "target_age"))
    assert node.fields[:13] == ["t_tick", "seq", "state", "base_mode", "solve_ms", "status", "base_x", "base_y",
                                "base_yaw", "est_reset", "base_vx", "base_vy", "base_w"]
    # the unix socket path delivers too
    u = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    rig.seq += 1
    u.sendto(json.dumps({"type": "alive", "session": rig.session, "seq": rig.seq}).encode(), node.receiver.unix_path)
    time.sleep(0.01)
    rig.step(1)
    assert rig.row()["seq"] == rig.seq
    node.disable("done")
    time.sleep(0.2)
    assert arm.token is None and not arm.enabled             # released


def test_staleness_two_stages_and_resume(rig):
    node = rig.node
    rig.lease_enable()
    rig.step(2)
    p0, q0 = rig.cmd_pose()
    rig.target(p0 + [0.02, 0, 0], q0, twist=[0.05, 0, 0, 0, 0, 0])
    rig.step(5)
    assert rig.row()["arm_ff_scale"] == 1.0
    rig.step(10)                                             # 0.15 s: the feedforward fades
    assert 0.0 < rig.row()["arm_ff_scale"] < 1.0 and node.state == "tracking"
    node.renew(rig.tok)
    rig.step(15)                                             # past hold_timeout (0.25 s): holding
    assert node.state == "holding" and rig.row()["hold"] == 1.0 and "stale" in node.state_dict()["hold_reason"]
    q_hold = node.q_cmd["arm"].copy()
    node.renew(rig.tok)
    rig.step(20)
    assert np.max(np.abs(node.q_cmd["arm"] - q_hold)) < 0.01    # stops (decelerating within the limits)
    rig.send({"type": "alive"})                              # the client is back: the pose in force is tracked again
    rig.step(1)
    assert node.state == "tracking"


def test_client_lease_lapse_holds(rig):
    node = rig.node
    rig.lease_enable()
    p0, q0 = rig.cmd_pose()
    rig.target(p0, q0)
    rig.step(1)
    rig.clock.t += 2.0                                       # the lease (1 s) lapses
    rig.step(1)
    rig.send({"type": "alive"})                              # its session is gone too: dropped
    rig.step(1)
    assert node.state == "holding" and node.state_dict()["hold_reason"] == "no client lease"
    assert node.receiver.counts["dropped"].get("unknown_session", 0) >= 1
    assert node.enabled                                      # the downstream leases stay until POST /disable


def test_datagram_drops_are_counted(rig):
    node = rig.node
    rig.lease_enable()
    p0, q0 = rig.cmd_pose()
    rig.send({}, raw=b"not json")
    rig.send({"type": "target", "session": rig.session + 1, "ees": {}})      # unknown session
    rig.seq += 1                                                              # (session overridden by **msg)
    rig.target(p0, q0)
    rig.seq -= 5
    rig.target(p0, q0)                                                        # old seq
    rig.seq += 10
    rig.send({"type": "target", "t": rig.clock.t - 5.0, "ees": {"arm": {"pos": list(p0), "quat": list(q0)}}})
    rig.send({"type": "target", "t": None, "ees": {"arm": {"frame": "base", "pos": list(p0), "quat": list(q0)}}})
    rig.send({"type": "target", "t": None, "ees": {"leg": {"pos": list(p0), "quat": list(q0)}}})
    rig.send({"type": "target", "t": None, "ees": {"arm": {"pos": [0, 0, float("nan")], "quat": list(q0)}}},
             raw=None)
    rig.step(2)
    d = node.state_dict()["session"]["dropped"]
    assert d["not_json"] == 1 and d["unknown_session"] == 1 and d["old_seq"] == 1
    assert d["bad_t"] == 1 and d["frame"] == 1 and d["invalid"] >= 2


def test_est_reset_drops_and_aborts(model):
    d = tempfile.mkdtemp(prefix="wbc")
    clock = Clock(1000.0)
    node = WbcNode(model, arm=None, swerve=None, estimator="est", clock=clock, udp_host=None,
                   unix_socket=d + "/t.sock")
    tok, _ = node.acquire("t")
    node.enable(tok)
    sess = node.open_session(tok)["session"]

    def est(rid, x=0.0):
        node.feed_estimator({"t": clock.t, "x": x, "y": 0.0, "theta": 0.0, "reset_id": float(rid), "odom_ok": 1.0})

    def step(n=1):
        for _ in range(n):
            clock.t += DT
            est(node.est.reset_id if node.est.reset_id is not None else 3)
            node.step()
    est(3)
    step(2)
    T = node.model.fk(node.model.config((0, 0, 0), node.q_cmd), "arm")
    msg = {"type": "target", "session": sess, "t": None,
           "ees": {"arm": {"pos": list(T.translation + [0.02, 0, 0]), "quat": list(R_to_quat(T.rotation))}}}
    assert node.receiver.handle(json.dumps({**msg, "seq": 1, "est_reset": 2}).encode()) is None
    step(1)
    assert node.receiver.counts["dropped"]["est_reset"] == 1 and node.state == "idle"
    node.receiver.handle(json.dumps({**msg, "seq": 2, "est_reset": 3}).encode())
    step(1)
    assert node.state == "tracking"
    est(4)                                                    # an estimator reset: the world targets are aborted
    step(1)
    assert node.state == "idle" and all(b.kind is None for b in node.buffers.values())
    assert any("estimator reset" in e["msg"] for e in node.events.list())
    node.disable("done")
    node.receiver.close()


def test_enable_rolls_back_when_the_arm_channel_refuses(rig):
    node, arm = rig.node, rig.arm
    arm.refuse = "busy"
    tok, _ = node.acquire("test")
    with pytest.raises(RequestError) as e:
        node.enable(tok)
    assert "refused: busy" in str(e.value) and e.value.status == 503
    wait_for(lambda: arm.token is None, 2)                   # the arm lease was given back
    assert not node.enabled and node.state == "disabled"
    arm.refuse = None
    node.enable(tok)
    assert node.enabled


def test_cross_bus_fault_and_estop_fan_out(rig):
    node, arm = rig.node, rig.arm
    rig.lease_enable()
    p0, q0 = rig.cmd_pose()
    rig.target(p0, q0)
    rig.step(60)
    arm.set_state("fault")                                   # the arm faults: the WBC faults (the base would stop)
    rig.step(3)
    assert node.state == "fault" and "arm is fault" in node.fault
    assert rig.row()["state"] == STATES.index("fault")
    node.disable("clear")
    assert node.state == "disabled"
    arm.set_state("disabled")
    time.sleep(0.2)
    node.enable(rig.tok)
    out = node.trigger_estop("test", "pytest")              # anyone: forwarded to the arm (and the swerve)
    assert out["forwarded"]["arm"] and arm.estopped is not None and not node.enabled
    with pytest.raises(RequestError):
        node.enable(rig.tok)                                  # latched until cleared
    node.clear_estop(rig.tok)


def test_arm_pause_holds_then_resumes_from_the_measured_joints_and_bye_faults(rig):
    """A planned motion on the arm pauses the session: the WBC holds and sends keep-alives; when it ends the
    commands restart from the measured joints. The arm's bye (lease end, e-stop, disable) is losing the arm."""
    node, arm = rig.node, rig.arm
    rig.lease_enable()
    p0, q0 = rig.cmd_pose()
    rig.target(p0, q0)
    rig.step(5)
    n_goals = len(arm.commands)
    with arm.lock:
        arm.paused = True
    rig.step(3)
    assert node.state == "holding" and node.state_dict()["hold_reason"] == "arm paused by a planned motion"
    with arm.lock:
        arm.q = arm.q + np.radians([5, -5, 0, 0, 0, 0])          # the planned motion moved the arm
    rig.send({"type": "alive"})
    rig.step(5)
    assert len(arm.commands) == n_goals                       # only keep-alives while paused
    with arm.lock:
        arm.paused = False
    rig.step(2)
    assert np.allclose(node.q_cmd["arm"], arm.q, atol=1e-6)   # resumed from the measured joints
    assert node.resyncs == 0 and len(arm.commands) > n_goals
    with arm.lock:                                            # the arm closes the session
        arm.sock.sendto(json.dumps({"type": "bye", "reason": "lease ended"}).encode(), arm.client_addr)
    rig.step(2)
    assert node.state == "fault" and "closed by the arm: lease ended" in node.fault


def test_http_api_and_page(rig):
    from fastapi.testclient import TestClient
    from robot_utils.nodes.wbc.server import create_app
    node = rig.node
    with TestClient(create_app(node, start=False)) as c:
        assert c.get("/meta").json()["actuated"] is True
        assert "Whole-body control" in c.get("/").text
        f = c.get("/fields").json()
        assert f["fields"] == node.fields and "base_vx" in f["doc"]
        r = c.post("/lease", json={"client": "http"}).json()
        h = {"X-Lease-Token": r["token"]}
        assert r["heartbeat"] == 1.0
        assert c.post("/session", json={}).status_code == 401
        s = c.post("/session", json={}, headers=h).json()
        assert set(s) == {"session", "udp_port", "unix_socket"}
        assert c.post("/hints", json={"base": {"mode": "locked"}}, headers=h).json()["hints"]["base"]["mode"] == "locked"
        assert c.post("/hints", json={"base": {"mode": "flying"}}, headers=h).status_code == 422
        assert c.post("/hints", json={"base": {"limits": {"max_speed": 9}}}, headers=h).status_code == 422
        st = c.get("/state").json()
        assert st["state"] == "disabled" and st["lease"]["holder"] == "http" and st["limited_by"] == "none"
        assert c.post("/lease/renew", headers=h).status_code == 200
        assert c.delete("/lease", headers=h).json() == {"released": True}


# -- integration: the real swerve node (virtual SPARKs) and estimator, real time ------------------------------

@pytest.fixture
def stack(model):
    from robot_utils.nodes.estimator.node import EstimatorNode
    from robot_utils.nodes.estimator.server import create_app as estimator_app
    from robot_utils.nodes.swerve.node import SwerveNode
    from robot_utils.nodes.swerve.server import create_app as swerve_app
    from robot_utils.nodes.swerve.sim import start_sim, stop_sim
    from robot_utils.nodes.wbc.server import create_app as wbc_app
    sims, iface = start_sim()
    sw = SwerveNode(iface)
    sw_url, sw_srv, sw_t = serve(swerve_app(sw))
    wait_for(lambda: sw.status == "running", 10)
    est = EstimatorNode(sw_url, None)
    est_url, est_srv, est_t = serve(estimator_app(est))
    d = tempfile.mkdtemp(prefix="wbc")
    arm = FakeArm(d + "/arm.joint.sock")
    arm_url, arm_srv, arm_t = serve(arm.app)
    node = WbcNode(model, arm=arm_url, swerve=sw_url, estimator=est_url, udp_host="127.0.0.1", udp_port=0,
                   unix_socket=d + "/wbc.target.sock", arm_socket=d + "/arm.joint.sock")
    url, srv, t = serve(wbc_app(node))
    wait_for(lambda: node.estimator_health(time.monotonic()) is None and node.sw is not None, 15)
    yield {"node": node, "url": url, "arm": arm, "swerve": sw_url, "est": est_url, "sw": sw}
    stop((srv, t), (arm_srv, arm_t), (est_srv, est_t), (sw_srv, sw_t))
    arm.close()
    stop_sim(sims, iface)


def test_whole_body_reach_with_the_real_swerve_and_estimator(stack):
    node, url, arm = stack["node"], stack["url"], stack["arm"]
    tok = httpx.post(url + "/lease", json={"client": "it"}).json()["token"]
    h = {"X-Lease-Token": tok}
    from robot_utils.nodes.wbc.spec import make_config
    cfg = node.cfg
    node.cfg = make_config({"max_speed": 2.5})               # beyond the swerve's hard limit (its GET /limits)
    r = httpx.post(url + "/enable", headers=h, timeout=10)
    assert r.status_code == 422 and "hard limits" in r.json()["detail"], r.text
    wait_for(lambda: arm.token is None, 3)                    # rolled back: the arm lease was given back
    assert httpx.get(stack["swerve"] + "/state").json()["lease"]["held"] is False
    node.cfg = cfg
    r = httpx.post(url + "/enable", headers=h, timeout=10)
    assert r.status_code == 200, r.text
    sw = httpx.get(stack["swerve"] + "/state").json()
    assert sw["lease"]["holder"].startswith("wbc") and sw["enabled"] and arm.token is not None
    s = httpx.post(url + "/session", json={}, headers=h).json()
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    latest = lambda: httpx.get(url + "/state").json()["latest"]   # noqa: E731
    wait_for(lambda: latest() and latest()["state"] == STATES.index("idle"), 5)
    L = latest()
    p0 = np.array([L[f"arm_cmd_{k}"] for k in "xyz"])
    q0 = [L[f"arm_cmd_q{k}"] for k in "xyzw"]
    goal = p0 + [0.45, 0.10, 0.0]                             # beyond the arm alone: the base must drive
    seq = 0
    t_end = time.monotonic() + 12.0
    moved = False
    while time.monotonic() < t_end:
        seq += 1
        udp.sendto(json.dumps({"type": "target", "session": s["session"], "seq": seq, "t": None,
                               "ees": {"arm": {"pos": list(goal), "quat": q0}}}).encode(),
                   ("127.0.0.1", s["udp_port"]))
        if seq % 30 == 0:
            L = latest()
            moved = moved or abs(L["base_x"]) > 0.1
            meas = np.array([L[f"arm_meas_{k}"] for k in "xyz"])
            if moved and np.linalg.norm(meas - goal) < 0.01 and abs(L["base_vx"]) < 1e-9:
                break
        time.sleep(0.02)
    L = latest()
    st = node.state_dict()
    assert moved, ("the base never drove", {k: st[k] for k in ("state", "fault", "hold_reason", "limited_by", "session", "estimator", "swerve")}, st["events"][-8:], L)
    pose = httpx.get(stack["est"] + "/pose").json()
    assert pose["x"] > 0.1                                    # the estimator saw the base drive
    meas = np.array([L[f"arm_meas_{k}"] for k in "xyz"])
    assert np.linalg.norm(meas - goal) < 0.02, (meas, goal)  # the EE (measured arm at the estimated base) is there
    assert L["status"] == 1 and node.state_dict()["fault"] is None
    st = node.state_dict()
    assert st["loop_hz"] > 80, st["loop_hz"]
    # a swerve e-stop: the WBC faults and e-stops the arm too (cross-bus)
    httpx.post(stack["swerve"] + "/estop", json={"reason": "test"})
    wait_for(lambda: node.state == "fault", 3)
    wait_for(lambda: arm.estopped is not None, 3)
    assert "swerve e-stop" in node.fault
    assert httpx.post(url + "/disable", headers=h).status_code == 200
    wait_for(lambda: arm.token is None, 3)
