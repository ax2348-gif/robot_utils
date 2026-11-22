"""The WBC with its control loop in its own process (docs/wbc.md "Processes"; wbc/proc_node.py, control_proc.py,
ipc.py): the spawn, the shared memory and command round trips (enable, session, datagrams arming, rows through the
ring, config, hints, disarm, the e-stop, the lease), heartbeat loss both ways, a killed control process, a clean
shutdown; and a load test: the control tick's period while the server process is hammered, against the old
single-process structure under the same load."""
import json
import os
import signal
import socket
import tempfile
import threading
import time

import httpx
import numpy as np
import pytest

from helpers import serve, stop, wait_for
from wbc_fakes import FakeArm
from robot_utils.nodes.wbc import ipc
from robot_utils.nodes.wbc.model import EESpec, WbcModel, arm_config_limits, fetch_urdf
from robot_utils.nodes.wbc.proc_node import ProcNode
from robot_utils.nodes.wbc.server import create_app


@pytest.fixture(scope="module")
def model():
    urdf, _ = fetch_urdf("local")
    return WbcModel(urdf, (EESpec(arm_limits=arm_config_limits()),))


class Rig:
    def __init__(self, node, url, arm):
        self.node, self.url, self.arm = node, url, arm
        self.http = httpx.Client(base_url=url, timeout=10)
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.h = {}

    def lease(self):
        tok = self.http.post("/lease", json={"client": "t"}).json()["token"]
        self.h = {"X-Lease-Token": tok}
        return tok

    def enable_session(self):
        self.lease()
        r = self.http.post("/enable", headers=self.h)
        assert r.status_code == 200, r.text
        self.s = self.http.post("/session", json={}, headers=self.h).json()
        return self.s

    def latest(self):
        rows = self.node.hub.recent(1, "tick")
        return dict(zip(self.node.fields, rows[0])) if len(rows) else None

    def cmd_pose(self):
        wait_for(lambda: self.latest() is not None and np.isfinite(self.latest()["arm_cmd_x"]), 3)
        L = self.latest()
        return np.array([L[f"arm_cmd_{k}"] for k in "xyz"]), [L[f"arm_cmd_q{k}"] for k in "xyzw"]

    def target(self, pos, quat, aseq=1, armed=True):
        self.seq += 1
        self.udp.sendto(json.dumps({"type": "target", "session": self.s["session"], "seq": self.seq, "t": None,
                                    "armed": armed, "arm_seq": aseq,
                                    "ees": {"arm": {"pos": [float(v) for v in pos],
                                                    "quat": [float(v) for v in quat]}}}).encode(),
                        ("127.0.0.1", self.s["udp_port"]))

    def arm_now(self, aseq=1):
        p0, q0 = self.cmd_pose()
        for _ in range(3):
            self.target(p0, q0, aseq)
            time.sleep(0.01)
        wait_for(lambda: self.node.armed, 2)
        return p0, q0

    def state(self):
        r = self.http.get("/state")
        assert r.status_code == 200, r.text
        return r.json()


@pytest.fixture
def rig(model):
    d = tempfile.mkdtemp(prefix="wbcp")
    arm = FakeArm(d + "/arm.joint.sock")
    aurl, asrv, ath = serve(arm.app)
    node = ProcNode(model, arm=aurl, swerve=None, estimator=None, udp_host="127.0.0.1", udp_port=0,
                    unix_socket=d + "/t.sock", arm_socket=d + "/arm.joint.sock")
    url, srv, th = serve(create_app(node))           # the lifespan spawns the control process
    r = Rig(node, url, arm)
    yield r
    r.http.close()
    stop((srv, th))                                  # the lifespan closes the node
    node.close()
    arm.close()
    stop((asrv, ath))


def test_spawn_enable_arm_track_and_rows_through_the_ring(rig):
    node = rig.node
    st = rig.state()
    assert st["process"]["alive"] and st["process"]["pid"] != os.getpid() and st["state"] == "disabled"
    rig.enable_session()
    p0, q0 = rig.arm_now()
    assert rig.state()["armed"] and rig.state()["state"] in ("tracking", "idle")
    goal = p0 + [0.0, 0.0, 0.05]
    for _ in range(80):                                  # 0.8 s of targets toward a goal 5 cm up
        rig.target(goal, q0)
        time.sleep(0.01)
    L = rig.latest()
    assert np.linalg.norm(np.array([L[f"arm_cmd_{k}"] for k in "xyz"]) - goal) < 0.005
    assert len(rig.arm.commands) > 50                    # the joint channel, owned by the control process
    # every tick reaches the hub in order through the ring, none dropped
    rows = node.hub.recent(300, "tick")
    t = rows[:, node.fields.index("t_tick")]
    assert np.all(np.diff(t) > 0) and np.max(np.diff(t)) < 0.05
    st = rig.state()
    assert st["process"]["rows_dropped"] == 0 and st["loop_hz"] > 80
    assert st["latest"]["armed"] == 1.0 and st["session"]["accepted"] >= 80
    assert any(e["msg"] == "ARMED (arm_seq 1)" for e in st["events"])     # the control process's events


def test_commands_config_hints_session_disarm(rig):
    node = rig.node
    rig.enable_session()
    rig.arm_now()
    # config: applied in both processes; structural keys refused while enabled; lead_torque beyond the ceiling
    r = rig.http.post("/config", json={"hold_timeout": 0.3})
    assert r.status_code == 200 and r.json()["config"]["hold_timeout"] == 0.3
    assert rig.state()["config"]["hold_timeout"] == 0.3 and node.cfg.hold_timeout == 0.3
    assert rig.http.post("/config", json={"rate_hz": 50}).status_code == 422
    assert rig.http.post("/config", json={"lead_torque": [99, 6, 6, 1.5, 1.5, 1.5]}).status_code == 422
    assert rig.http.post("/config", json={"nope": 1}).status_code == 422
    # hints
    r = rig.http.post("/hints", json={"base": {"mode": "locked"}}, headers=rig.h)
    assert r.status_code == 200 and r.json()["hints"]["base"]["mode"] == "locked"
    assert rig.http.get("/hints").json()["in_force"]["base"]["mode"] == "locked"
    assert rig.http.post("/hints", json={"base": {"mode": "locked"}}).status_code in (401, 403)
    # POST /disarm (anyone): the reply carries the control process's answer
    r = rig.http.post("/disarm", json={"reason": "why"}).json()
    assert r == {"armed": False, "was_armed": True, "disarm_reason": "POST /disarm (testclient): why"} or (
        r["was_armed"] and r["disarm_reason"].startswith("POST /disarm"))
    assert not node.armed
    # a new session: the arm_seq floor kept; an old press never re-arms
    s2 = rig.http.post("/session", json={}, headers=rig.h).json()
    assert s2["arm_seq"] >= 1 and s2["session"] != rig.s["session"]
    rig.s = s2
    p0, q0 = rig.cmd_pose()
    for _ in range(10):
        rig.target(p0, q0, aseq=1)
        time.sleep(0.01)
    assert not node.armed and rig.state()["latch"]["arm_refused"] >= 1
    rig.arm_now(aseq=2)


def test_datagrams_renew_the_client_lease_and_its_lapse_disarms(rig):
    node = rig.node
    rig.enable_session()                                   # heartbeat 1 s; never renewed over HTTP below
    p0, q0 = rig.arm_now()
    t_end = time.monotonic() + 2.0
    while time.monotonic() < t_end:                        # 2 s of datagrams keep the lease (and the WBC armed)
        rig.target(p0, q0)
        time.sleep(0.01)
    assert node.lease.held and node.armed
    wait_for(lambda: not node.lease.held, 2)               # silence: the lease lapses -> disarmed
    wait_for(lambda: not node.armed, 1)
    assert rig.state()["disarm_reason"] in ("client lease lapsed",) or "stale" in rig.state()["disarm_reason"]


def test_estop_holds_at_once_then_fans_out_and_disables(rig):
    node = rig.node
    rig.enable_session()
    p0, q0 = rig.arm_now()
    for _ in range(20):
        rig.target(p0 + [0, 0, 0.05], q0)
        time.sleep(0.01)
    n0 = len(rig.arm.commands)
    t0 = time.monotonic()
    r = rig.http.post("/estop", json={"reason": "test", "source": "me"})
    assert r.status_code == 200 and r.json()["forwarded"] == {"arm": True}
    holds = [c for c in rig.arm.commands[n0:] if c[0] >= t0 and np.all(c[2] == 0)]
    assert holds and holds[0][0] - t0 < 0.05, [c[0] - t0 for c in rig.arm.commands[n0:]]   # the hold, at once
    st = rig.state()
    assert st["estop"]["latched"] and not st["enabled"] and not st["armed"]
    assert rig.arm.estopped == "wbc: test"                 # (then disabled by the WBC's disable)
    assert rig.http.post("/enable", headers=rig.h).status_code == 409
    assert rig.http.post("/estop/clear", headers=rig.h).status_code == 200
    assert not rig.state()["estop"]["latched"]


def test_server_heartbeat_loss_disarms_and_holds(rig):
    node = rig.node
    rig.enable_session()
    p0, q0 = rig.arm_now()
    stop_tx = threading.Event()

    def tx():
        while not stop_tx.is_set():
            rig.target(p0, q0)
            time.sleep(0.01)
    threading.Thread(target=tx, daemon=True).start()
    try:
        time.sleep(0.2)
        assert node.armed
        node.freeze_heartbeat = True                       # the server process stalls, as the control process sees it
        t0 = time.monotonic()
        wait_for(lambda: not node.armed, 2)
        assert 0.4 < time.monotonic() - t0 < 1.0
        assert rig.state()["disarm_reason"].startswith("server process heartbeat lost")
        n = len(rig.arm.commands)
        time.sleep(0.2)
        assert len(rig.arm.commands) > n + 10              # the arm channel stays alive: hold goals
        assert not node.armed                              # no arming while the server is lost
        node.freeze_heartbeat = False
        wait_for(lambda: any("heartbeat back" in e["msg"] for e in node.events.list()), 2)
        time.sleep(0.2)
        assert not node.armed and node.state != "fault"    # the old press never re-arms
    finally:
        stop_tx.set()


@pytest.mark.parametrize("how", ["stall", "kill"])
def test_control_process_loss_faults_and_releases(rig, how):
    node = rig.node
    rig.enable_session()
    rig.arm_now()
    pid = node.ready["pid"]
    os.kill(pid, signal.SIGSTOP if how == "stall" else signal.SIGKILL)
    try:
        wait_for(lambda: node.proc_fault is not None, 2)
        assert ("heartbeat" in node.proc_fault) if how == "stall" else ("exited" in node.proc_fault), node.proc_fault
        st = rig.state()
        assert st["state"] == "fault" and st["fault"] == node.proc_fault and not st["enabled"] and not st["armed"]
        assert any(e["msg"].startswith("FAULT: ") and "restart the node" in e["msg"] for e in st["events"])
        wait_for(lambda: rig.arm.token is None, 3)         # the arm disabled and its lease released
        r = rig.http.post("/enable", headers=rig.h)
        assert r.status_code == 503 and "restart the node" in r.json()["detail"]
    finally:
        if how == "stall":
            os.kill(pid, signal.SIGCONT)


def test_clean_shutdown(model):
    d = tempfile.mkdtemp(prefix="wbcp")
    arm = FakeArm(d + "/arm.joint.sock")
    aurl, asrv, ath = serve(arm.app)
    node = ProcNode(model, arm=aurl, swerve=None, estimator=None, udp_host="127.0.0.1", udp_port=0,
                    unix_socket=d + "/t.sock", arm_socket=d + "/arm.joint.sock")
    try:
        node.start()
        tok, _ = node.acquire("t")
        node.enable(tok)
        assert arm.token is not None and arm.session is not None
        ipc_dir, proc = node.dir, node.proc
        node.close()
        assert proc.exitcode == 0 and not os.path.exists(ipc_dir) and not os.path.exists(d + "/t.sock")
        wait_for(lambda: arm.token is None and arm.session is None, 3)   # bye on the channel, lease released
    finally:
        arm.close()
        stop((asrv, ath))


def test_ipc_slot_and_ring(tmp_path):
    shm = ipc.Shared(str(tmp_path / "shm"), 5, create=True)
    other = ipc.Shared(str(tmp_path / "shm"), 5, create=False)
    shm.snap.write({"a": 1})
    assert other.snap.read() == {"a": 1} and other.snap.read() == {"a": 1}
    shm.snap.write({"a": 2})
    assert other.snap.read() == {"a": 2}
    for i in range(10):
        shm.rows.write([i] * 5)
    assert other.rows.read()[:, 0].tolist() == list(range(10))
    for i in range(10, 5000):                         # overflow: drop-oldest, counted
        shm.rows.write([i] * 5)
    out = other.rows.read()
    assert out[-1, 0] == 4999 and other.rows.dropped == 4990 - len(out) and np.all(np.diff(out[:, 0]) == 1)
    shm.close()
    other.close()


# -- load: the control tick while the server process is hammered ------------------------------------------------

def _load(url, stop_ev, stats):
    """Many /state + SSE + stream clients, and three simulated 100 Hz stream readers (Python work every 10 ms)."""
    def state_client():
        with httpx.Client(base_url=url, timeout=5) as c:
            while not stop_ev.is_set():
                c.get("/state")
                stats["state"] += 1

    def sse_client():
        with httpx.Client(base_url=url, timeout=None) as c, c.stream("GET", "/events") as r:
            for _ in r.iter_bytes():
                if stop_ev.is_set():
                    return

    def stream_client():
        with httpx.Client(base_url=url, timeout=None) as c, c.stream("GET", "/stream") as r:
            for _ in r.iter_bytes():
                if stop_ev.is_set():
                    return

    def reader():                                  # like an Upstream: parse a batch of rows every 10 ms
        nxt = time.monotonic()
        while not stop_ev.is_set():
            nxt += 0.01
            rows = np.random.rand(8, 40)
            for r in rows:
                {f"c{i}": float(v) for i, v in enumerate(r)}
            sum(i * i for i in range(3000))
            time.sleep(max(0.0, nxt - time.monotonic()))
    fns = [state_client] * 6 + [sse_client] * 3 + [stream_client] * 3 + [reader] * 3
    ts = [threading.Thread(target=f, daemon=True) for f in fns]
    for t in ts:
        t.start()
    return ts


def _measure(node, url, arm, seconds=6.0):
    http = httpx.Client(base_url=url, timeout=10)
    tok = http.post("/lease", json={"client": "load"}).json()["token"]
    h = {"X-Lease-Token": tok}
    assert http.post("/enable", headers=h).status_code == 200
    s = http.post("/session", json={}, headers=h).json()
    wait_for(lambda: len(node.hub.recent(1, "tick")), 3)
    time.sleep(0.2)
    L = dict(zip(node.fields, node.hub.recent(1, "tick")[0]))
    p0 = np.array([L[f"arm_cmd_{k}"] for k in "xyz"])
    q0 = [L[f"arm_cmd_q{k}"] for k in "xyzw"]
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stop_ev = threading.Event()
    stats = {"state": 0}

    def tx():                                       # a client at 100 Hz: armed, a target circling 3 cm
        seq, t0 = 0, time.monotonic()
        while not stop_ev.is_set():
            seq += 1
            a = 2 * np.pi * 0.5 * (time.monotonic() - t0)
            p = p0 + (0.03 * np.array([0, np.cos(a) - 1, np.sin(a)]) if seq > 5 else 0)
            udp.sendto(json.dumps({"type": "target", "session": s["session"], "seq": seq, "t": None,
                                   "armed": True, "arm_seq": 1, "ees": {"arm": {"pos": list(map(float, p)),
                                                                                "quat": q0}}}).encode(),
                       ("127.0.0.1", s["udp_port"]))
            time.sleep(0.01)
    threads = [threading.Thread(target=tx, daemon=True)] + _load(url, stop_ev, stats)
    threads[0].start()
    time.sleep(1.0)
    t_from = time.monotonic()
    time.sleep(seconds)
    rows = node.hub.recent(int(seconds * 100) + 50, "tick")
    stop_ev.set()
    st = http.get("/state").json()
    http.post("/disable", headers=h)
    http.close()
    t = rows[:, node.fields.index("t_tick")]
    per = rows[:, node.fields.index("tick_period")][t >= t_from]
    per = per[np.isfinite(per)]
    return {"p50": float(np.percentile(per, 50)), "p99": float(np.percentile(per, 99)), "max": float(per.max()),
            "n": len(per), "late": st["tick"]["late"], "overruns": st["tick"]["overruns"],
            "disarms": st["latch"]["disarms"], "state_requests": stats["state"]}


def _rig_for(model, kind):
    from robot_utils.nodes.wbc.node import WbcNode
    d = tempfile.mkdtemp(prefix="wbcl")
    arm = FakeArm(d + "/arm.joint.sock")
    aurl, asrv, ath = serve(arm.app)
    cls = ProcNode if kind == "process" else WbcNode
    node = cls(model, arm=aurl, swerve=None, estimator=None, udp_host="127.0.0.1", udp_port=0,
               unix_socket=d + "/t.sock", arm_socket=d + "/arm.joint.sock")
    url, srv, th = serve(create_app(node))
    return node, url, arm, lambda: (stop((srv, th)), node.close(), arm.close(), stop((asrv, ath)))


def test_load_the_control_tick_stays_on_time_while_the_server_is_hammered(model, capsys):
    res = {}
    for kind in ("thread", "process"):
        node, url, arm, done = _rig_for(model, kind)
        try:
            res[kind] = _measure(node, url, arm)
        finally:
            done()
    with capsys.disabled():
        for k, v in res.items():
            print(f"\n  wbc load, control loop in a {k:8s}: tick period p50 {v['p50']:.2f} ms, p99 {v['p99']:.2f} ms, "
                  f"max {v['max']:.1f} ms; overruns {v['overruns']}, late {v['late']}, disarms {v['disarms']} "
                  f"({v['n']} ticks, {v['state_requests']} /state requests)")
    assert res["process"]["p99"] <= 15.0, res
    assert res["process"]["late"] == 0, res
