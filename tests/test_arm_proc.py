"""The arm node with its control loop and bus in their own process (docs/arm.md "Processes"; arm/proc_node.py,
control_proc.py, common/ipc.py) against the simulated bus: the HTTP API end to end in both modes (the same /state
document), the joint channel owned by the control process (its datagrams renewing the server's lease, the lease's
lapse ending the session), the e-stop switching the motors off on receipt, heartbeat loss both ways, a stalled or
killed control process, a clean shutdown; and a load test: the control tick's period and timing, and the joint
channel's goal-to-state latency, while the server process is hammered, against the old single-process structure."""
import functools
import math
import multiprocessing as mp
import os
import signal
import tempfile
import threading
import time

import httpx
import numpy as np
import pytest
from reBotArm_control_py.actuator.rebotarm import joint_limits, load_cfg

from arm_proc_fakes import SlowGripperSim, burn, jc_client
from helpers import serve, stop, wait_for
from robot_utils.nodes.arm import config as C
from robot_utils.nodes.arm.bus import SimBus
from robot_utils.nodes.arm.models import ArmStateOut
from robot_utils.nodes.arm.node import COL, FIELDS, ArmNode
from robot_utils.nodes.arm.proc_node import ProcNode
from robot_utils.nodes.arm.server import create_app
from robot_utils.nodes.common import fastpath as F
from robot_utils.nodes.common.locks import LockStore

Q0 = (0.0, -0.6, -0.9, 0.0, 0.5, 0.0)
LIMITS = joint_limits(load_cfg(C.PROFILES["soft"]))


@pytest.fixture(scope="module", autouse=True)
def _planner_process():
    """Start this process's planner process once (spawning it takes a couple of seconds)."""
    from robot_utils.nodes.arm.kin import ArmKinematics
    from robot_utils.nodes.arm.planner import Planner
    k = ArmKinematics()
    Planner.run("ik", k, k.fk(np.array(Q0)), np.array(Q0))
    yield


def make_node(kind: str, d: str):
    factory = functools.partial(SimBus, Q0, limits=LIMITS)          # picklable: built in the control process
    kw = dict(locks=LockStore(os.path.join(d, "locks")), joint_socket=os.path.join(d, "arm.joint.sock"))
    return ProcNode(factory, **kw) if kind == "process" else ArmNode(factory, **kw)


class Rig:
    def __init__(self, kind):
        self.d = tempfile.mkdtemp(prefix="arp")                     # short: macOS limits socket paths to ~104 bytes
        self.node = make_node(kind, self.d)
        self.url, self.srv, self.th = serve(create_app(self.node, sim=True))   # the lifespan starts the node
        self.http = httpx.Client(base_url=self.url, timeout=10)
        self.h = {}
        self.jc = None
        wait_for(lambda: self.node.state == "uninitialized", 10)

    def lease(self):
        tok = self.http.post("/lease", json={"client": "t"}).json()["token"]
        self.h = {"X-Lease-Token": tok}
        return tok

    def enable(self):
        tok = self.lease()
        r = self.http.post("/enable", headers=self.h)
        assert r.status_code == 200, r.text
        return tok

    def state(self):
        r = self.http.get("/state")
        assert r.status_code == 200, r.text
        return r.json()

    def rows(self, n=200):
        return self.node.hub.recent(n, "tick")

    def client(self, tok):
        self.jc = F.Client(os.path.join(self.d, "arm.joint.sock"), os.path.join(self.d, "c.sock"))
        w = self.jc.hello(tok, "test")
        assert w["type"] == "welcome", w
        return self.jc

    def close(self):
        self.http.close()
        stop((self.srv, self.th))                                    # the lifespan closes the node
        self.node.close()
        if self.jc is not None:
            self.jc.sock.close()


@pytest.fixture
def rig():
    r = Rig("process")
    yield r
    r.close()


# -- both modes: the same API ------------------------------------------------------------------------------------

@pytest.mark.parametrize("kind", ["thread", "process"])
def test_http_api_end_to_end_in_both_modes(kind):
    rig = Rig(kind)
    c = rig.http
    try:
        assert c.get("/meta").json()["fake"] is True
        tok = rig.enable()
        h = rig.h
        st = rig.state()
        assert st["enabled"] and st["state"] == "floating" and st["process"]["mode"] == kind
        ArmStateOut.model_validate(st)
        wait_for(lambda: c.get("/kinematics").status_code == 200, 3)
        tcp = c.get("/kinematics").json()["tcp"]
        pose = {"pos": [tcp["pos"][0], tcp["pos"][1], tcp["pos"][2] + 0.01], "quat": tcp["quat"]}
        r = c.post("/track", headers=h, json={"pose": pose, "twist": [0, 0, 0.01, 0, 0, 0], "seq": 3}).json()
        assert r["ok"] and r["seq"] == 3
        wait_for(lambda: rig.state()["source"] == "track", 2)
        assert rig.state()["track"]["seq"] == 3
        old = c.post("/track", headers=h, json={"pose": pose, "seq": 2}).json()
        assert not old["ok"] and old["reason"].startswith("stale")
        assert not c.post("/track", headers=h, json={"pose": {"pos": [3, 0, 0], "rpy": [0, 0, 0]}}).json()["ok"]
        r = c.post("/servo", headers=h, json={"pose": {"pos": [3, 0, 0], "rpy": [0, 0, 0]}})
        assert r.status_code == 422 and "not planned" in r.json()["detail"]
        assert c.post("/servo", headers=h, json={"name": "home"}).json()["kind"] == "pose"
        wait_for(lambda: rig.state()["source"] == "pose", 2)
        assert c.post("/cancel", headers=h).json()["cancelled"]
        assert rig.state()["source"] == "hold"
        assert c.post("/ik", json={"pose": pose}).json()["ok"]
        assert c.post("/jam", json={"response": "report"}, headers={"X-Operator": "human"}).json()["config"]["response"] == "report"
        assert rig.state()["jam"]["config"]["response"] == "report"
        assert c.post("/gripper", headers=h, json={"gap_mm": 20, "max_force_n": 30}).json()["max_force_n"] == 30
        assert c.post("/gripper", headers=h, json={"gap_mm": 999}).status_code == 422
        assert c.post("/gripper", headers={"X-Lease-Token": "nope"}, json={"gap_mm": 20}).status_code == 403
        assert c.post("/profile", headers=h, json={"name": "safe"}).status_code == 409   # servoing
        r = c.post("/limits", headers=h, json={"lin_vel": 0.1})
        assert r.status_code == 200 and r.json()["soft"]["lin_vel"] == 0.1 and c.get("/limits").json()["soft"]["lin_vel"] == 0.1
        assert c.post("/float", headers=h).status_code == 200
        wait_for(lambda: rig.state()["source"] == "none", 2)
        r = c.post("/tcp", headers=h, json={"offset": [0, 0, 0.12]})
        assert r.status_code == 200 and c.get("/tcp").json()["offset"] == [0, 0, 0.12]
        assert rig.state()["tcp_offset"] == [0, 0, 0.12]
        for path in ("/joints", "/tracking", "/gripper", "/links", "/geometry", "/jam", "/"):
            assert c.get(path).status_code == 200, path
        assert c.post("/profile", headers=h, json={"name": "safe"}).status_code == 409   # the gripper holds
        assert c.post("/disable", headers=h).json()["enabled"] is False
        assert c.post("/profile", headers=h, json={"name": "safe"}).status_code == 200
        assert c.get("/gains").json()["name"] == "safe" and rig.state()["profile"] == "safe"
        assert c.post("/track", headers=h, json={"pose": pose}).status_code == 409
        assert c.post("/enable", headers=h).status_code == 200
        assert c.post("/estop", json={"reason": "t"}).json()["latched"]
        wait_for(lambda: not rig.state()["motors_on"], 1)
        st = rig.state()
        assert st["estop"]["latched"] and st["state"] == "off"
        assert c.post("/enable", headers=h).status_code == 409
        assert c.post("/estop/clear", headers=h).status_code == 200 and not rig.state()["estop"]["latched"]
        assert c.post("/enable", headers=h).status_code == 200
        assert c.delete("/lease", headers=h).status_code == 200
        wait_for(lambda: not rig.state()["enabled"], 1)                # the lease's end disables
        msgs = [e["msg"] for e in rig.state()["events"]]
        assert any(m.startswith("lease released") or m == "lease released" for m in msgs), msgs
        ArmStateOut.model_validate(rig.state())
    finally:
        rig.close()


def test_the_state_document_is_the_same_in_both_modes():
    docs = {}
    for kind in ("thread", "process"):
        rig = Rig(kind)
        try:
            rig.enable()
            time.sleep(0.7)
            docs[kind] = rig.state()
        finally:
            rig.close()
    a, b = docs["thread"], docs["process"]
    assert set(a) == set(b)
    for k in a:
        if isinstance(a[k], dict) and k not in ("latest",):
            assert set(a[k]) <= set(b[k]) | {"mode"} and set(b[k]) - set(a[k]) <= {"pid", "alive", "fault",
                                                                                   "heartbeat_age", "snapshot_age",
                                                                                   "busy", "rows_dropped",
                                                                                   "commands_dropped",
                                                                                   "service_max_ms"}, k
    for k in ("state", "enabled", "motors_on", "servo", "initialized", "source", "profile", "limits", "named_poses",
              "lease_heartbeat"):
        assert a[k] == b[k], k
    assert b["process"]["tick_time"]["n"] >= 50 and b["loop_hz"] > 80


# -- the joint channel, owned by the control process -------------------------------------------------------------

def test_joint_channel_goals_state_and_the_lease(rig):
    node = rig.node
    tok = rig.enable()
    c = rig.client(tok)
    st = c.latest_state()
    q0 = np.array(st.q)
    for k in range(60):                                       # 0.6 s of goals 0.1 rad along joint 1
        c.send(q=q0 + [0.1 * min(1.0, k / 30), 0, 0, 0, 0, 0])
        time.sleep(0.01)
    wait_for(lambda: abs(c.latest_state().q[0] - (q0[0] + 0.1)) < 0.01, 3)
    st = rig.state()
    assert st["source"] == "stream" and st["joint_channel"]["session"]["counts"]["goals"] >= 60
    assert sum(st["joint_channel"]["session"]["dropped"].values()) == 0
    # datagrams renew the server's lease: 4 s (> the 3 s heartbeat) without any HTTP renew
    t_end = time.monotonic() + 4.0
    while time.monotonic() < t_end:
        c.keepalive()
        c.latest_state()                                      # drained: a full client socket would drop the bye
        time.sleep(0.02)
    assert node.lease.held and rig.state()["enabled"]
    t_end = time.monotonic() + 5.0
    while node.lease.held and time.monotonic() < t_end:
        c.recv(0.05)
    # silence: the lease lapses (the server ends it): the arm disabled, the session closed with a bye
    assert not node.lease.held
    wait_for(lambda: not rig.state()["enabled"], 1)
    for _ in range(100):
        if any(m.get("type") == "bye" for m in c.controls) or c.recv(0.2) is None:
            break
    assert any(m.get("type") == "bye" for m in c.controls), ([e["msg"] for e in rig.state()["events"]][-8:], c.controls)
    # a hello with the old token is refused (the server's slot says no lease)
    assert c.hello(tok)["type"] == "refused"


def test_a_long_planned_motion_goes_through_the_bulk_slot(rig, monkeypatch):
    """A planned motion too large for one datagram on the robot (Linux caps them at 208 KiB) reaches the control
    process through the bulk slot."""
    import pickle
    from robot_utils.nodes.arm import control_proc
    rig.enable()
    sent = []
    send = rig.node.port.send_bytes
    monkeypatch.setattr(rig.node.port, "send_bytes", lambda b: (sent.append(len(b)), send(b))[1])
    tcp = rig.http.get("/kinematics").json()["tcp"]
    pose = {"pos": [tcp["pos"][0], tcp["pos"][1], tcp["pos"][2] + 0.15], "quat": tcp["quat"]}
    r = rig.http.post("/servo", headers=rig.h, json={"pose": pose, "lin_vel": 0.012})
    assert r.status_code == 200, r.text
    assert r.json()["duration"] > 15                                 # > 1500 samples
    assert max(sent) < 1000                                          # every datagram small: the plan went in bulk
    assert len(pickle.dumps(rig.node.shm.bulk.read())) > 200_000 > control_proc.BULK_MIN
    wait_for(lambda: rig.state()["source"] == "servo", 2)
    assert rig.state()["motion"]["phase"] == "executing"


def test_estop_switches_the_motors_off_on_receipt(rig):
    rig.enable()
    rig.http.post("/servo", headers=rig.h, json={"name": "home"})
    wait_for(lambda: rig.state()["servo"], 2)
    t0 = time.monotonic()
    assert rig.http.post("/estop", json={"reason": "t"}).status_code == 200
    wait_for(lambda: len(rig.rows(1)) and rig.rows(1)[0, COL["t_tick"]] > t0 + 0.03, 2)
    rows = rig.rows(20)
    after = rows[rows[:, COL["t_tick"]] > t0]
    on = after[:, COL["motors_on"]]
    # the e-stop is applied between ticks: at most the tick already running when it arrived shows the motors on
    assert on[1:].max() == 0.0, on
    assert any("E-STOP" in e["msg"] for e in rig.state()["events"])


# -- failures ----------------------------------------------------------------------------------------------------

def test_server_heartbeat_loss_holds_and_drops_new_goals(rig):
    node = rig.node
    tok = rig.enable()
    c = rig.client(tok)
    q0 = np.array(c.latest_state().q)
    stop_tx = threading.Event()
    sent = {"k": 0}

    def tx():
        while not stop_tx.is_set():
            sent["k"] += 1
            c.send(q=q0 + [0.15 * math.sin(sent["k"] * 0.02), 0, 0, 0, 0, 0])
            time.sleep(0.01)
    threading.Thread(target=tx, daemon=True).start()
    try:
        wait_for(lambda: rig.state()["source"] == "stream", 2)
        node.freeze_heartbeat = True                     # the server process stalls, as the control process sees it
        t0 = time.monotonic()
        wait_for(lambda: node.snap().get("srv_lost"), 2)
        assert 0.4 < time.monotonic() - t0 < 1.0
        time.sleep(0.2)
        core = node.snap()["core"]
        assert core["source"] == "hold" and core["servo"] and core["motors_on"]       # holding, not dropped
        assert core["joint_channel"]["session"]["dropped"].get("server", 0) > 5        # no new motion
        c.latest_state()                                  # drained (a full queue keeps old packets)
        time.sleep(0.03)
        q_hold = np.array(c.latest_state().q_cmd)
        time.sleep(0.2)
        c.latest_state()
        time.sleep(0.03)
        assert np.allclose(c.latest_state().q_cmd, q_hold)                             # the command stays put
        node.freeze_heartbeat = False
        wait_for(lambda: any("heartbeat back" in e["msg"] for e in node.events.list()), 2)
        wait_for(lambda: rig.state()["source"] == "stream", 2)                          # goals accepted again
    finally:
        stop_tx.set()


@pytest.mark.parametrize("how", ["stall", "kill"])
def test_control_process_loss_faults(rig, how):
    node = rig.node
    rig.enable()
    pid = node.ready["pid"]
    os.kill(pid, signal.SIGSTOP if how == "stall" else signal.SIGKILL)
    try:
        wait_for(lambda: node.proc_fault is not None, 2)
        assert ("heartbeat" in node.proc_fault) if how == "stall" else ("exited" in node.proc_fault), node.proc_fault
        st = rig.state()
        assert st["state"] == "fault" and st["error"] == node.proc_fault and not st["enabled"] and not st["motors_on"]
        assert st["process"]["fault"] == node.proc_fault
        assert any(e["msg"].startswith("FAULT: ") and "restart the node" in e["msg"] for e in st["events"])
        r = rig.http.post("/enable", headers=rig.h)
        assert r.status_code == 503 and "restart the node" in r.json()["detail"]
        assert rig.http.post("/estop", json={"reason": "t"}).status_code == 200          # still answers
        if how == "stall":                                # still silent: killed (the port released)
            wait_for(lambda: not node.proc.is_alive(), 3)
    finally:
        if how == "stall":
            try:
                os.kill(pid, signal.SIGCONT)
            except ProcessLookupError:
                pass


def test_a_blocking_enable_is_excused_and_the_server_answers_meanwhile():
    """A blocking step (here a 0.8 s gripper check) announces itself: the control heartbeat is excused (no fault),
    the server answers meanwhile and says what the control process is busy with."""
    d = tempfile.mkdtemp(prefix="arp")
    node = ProcNode(functools.partial(SlowGripperSim, Q0, limits=LIMITS), locks=LockStore(os.path.join(d, "locks")),
                    joint_socket=os.path.join(d, "arm.joint.sock"))
    node.start()
    try:
        wait_for(lambda: node.state == "uninitialized", 10)
        tok, _ = node.acquire("t")
        out = {}
        th = threading.Thread(target=lambda: out.setdefault("r", node.enable(tok)))
        th.start()
        wait_for(lambda: node.busy == "enabling", 2)
        t0 = time.monotonic()
        st = node.state_dict()                               # answered while the control process is blocked
        assert time.monotonic() - t0 < 0.1 and st["state"] == "initializing" and st["process"]["busy"] == "enabling"
        th.join(5)
        assert out["r"]["enabled"] and node.proc_fault is None and node.busy is None
    finally:
        node.close()


def test_clean_shutdown():
    rig = Rig("process")
    node = rig.node
    tok = rig.enable()
    c = rig.client(tok)
    ipc_dir, proc = node.dir, node.proc
    rig.http.close()
    stop((rig.srv, rig.th))
    node.close()
    assert proc.exitcode == 0 and not os.path.exists(ipc_dir)
    assert not os.path.exists(os.path.join(rig.d, "arm.joint.sock"))
    wait_for(lambda: c.recv(0.2) is not None and any(m.get("type") == "bye" for m in c.controls), 2)
    c.sock.close()


# -- load: the control tick while the server process is hammered --------------------------------------------------

def _load(url, stop_ev, stats):
    """/state pollers, SSE and /stream clients, /kinematics pollers (model work in the server process), and three
    threads of plain Python work every 10 ms (like the readers / publishers of a busy server process)."""
    def poll(path):
        def run():
            with httpx.Client(base_url=url, timeout=5) as c:
                while not stop_ev.is_set():
                    c.get(path)
                    stats[path] = stats.get(path, 0) + 1
        return run

    def streaming(path):
        def run():
            with httpx.Client(base_url=url, timeout=None) as c, c.stream("GET", path) as r:
                for _ in r.iter_bytes():
                    if stop_ev.is_set():
                        return
        return run

    def worker():
        nxt = time.monotonic()
        while not stop_ev.is_set():
            nxt += 0.01
            rows = np.random.rand(8, 40)
            for r in rows:
                {f"c{i}": float(v) for i, v in enumerate(r)}
            sum(i * i for i in range(3000))
            time.sleep(max(0.0, nxt - time.monotonic()))
    fns = ([poll("/state")] * 6 + [poll("/kinematics")] * 2 + [streaming("/events")] * 3
           + [streaming("/stream")] * 3 + [worker] * 3)
    ts = [threading.Thread(target=f, daemon=True) for f in fns]
    for t in ts:
        t.start()
    return ts


def _measure(kind, busy: bool, seconds=6.0):
    rig = Rig(kind)
    burners, burn_stop = [], None
    try:
        tok = rig.enable()
        stop_ev = threading.Event()
        stats: dict = {}
        ctx = mp.get_context("spawn")
        q = ctx.Queue()
        p = ctx.Process(target=jc_client, args=(os.path.join(rig.d, "arm.joint.sock"), os.path.join(rig.d, "l.sock"),
                                                 tok, seconds + 1.0, q), daemon=True)
        p.start()
        if busy:                                          # every core busy at normal priority, as the robot's
            burn_stop = ctx.Event()
            burners = [ctx.Process(target=burn, args=(burn_stop,), daemon=True) for _ in range(os.cpu_count() or 4)]
            for b in burners:
                b.start()
        _load(rig.url, stop_ev, stats)
        time.sleep(1.0)
        t_from = time.monotonic()
        time.sleep(seconds)
        rows = rig.rows(int(seconds * 100) + 50)
        st = rig.state()
        lat, gaps = q.get(timeout=20)
        stop_ev.set()
        p.join(5)
        if burn_stop is not None:
            burn_stop.set()
            for b in burners:
                b.join(5)
        t = rows[:, COL["t_tick"]]
        per = rows[:, COL["tick_dt"]][t >= t_from]
        per = per[np.isfinite(per)] * 1e3
        lat = np.array(lat[100:]) * 1e3                   # past the start
        gaps = np.array(gaps[100:]) * 1e3
        return {"p50": float(np.percentile(per, 50)), "p99": float(np.percentile(per, 99)), "max": float(per.max()),
                "n": len(per), "overruns": st["overruns"], "late": st["late_ticks"],
                "tick": st["process"]["tick_time"], "lat_p50": float(np.percentile(lat, 50)),
                "lat_p99": float(np.percentile(lat, 99)), "lat_max": float(lat.max()),
                "gap_p99": float(np.percentile(gaps, 99)), "gap_max": float(gaps.max()),
                "dropped": sum(st["joint_channel"]["session"]["dropped"].values()),
                "state_requests": stats.get("/state", 0)}
    finally:
        if burn_stop is not None:
            burn_stop.set()
        for b in burners:
            b.join(5)
            if b.is_alive():
                b.kill()
        rig.close()


@pytest.mark.parametrize("busy", [False, True], ids=["idle_cpus", "all_cpus_busy"])
def test_load_the_control_tick_stays_on_time_while_the_server_is_hammered(capsys, busy):
    res = {kind: _measure(kind, busy) for kind in ("thread", "process")}
    with capsys.disabled():
        for k, v in res.items():
            tt = v["tick"]
            print(f"\n  arm load{' + every core busy' if busy else ''}, control loop in a {k:8s}: tick period p50 {v['p50']:.2f} ms, p99 {v['p99']:.2f} ms, "
                  f"max {v['max']:.1f} ms; overruns {v['overruns']}, late {v['late']} ({v['n']} ticks); tick wall p99 "
                  f"{tt['wall_p99_ms']:.2f} ms, cpu p99 {tt['cpu_p99_ms']:.2f} ms, off-cpu p50 {tt['off_cpu_p50_ms']:.2f} "
                  f"p99 {tt['off_cpu_p99_ms']:.2f} max {tt['off_cpu_max_ms']:.2f} ms; joint channel goal->state p50 "
                  f"{v['lat_p50']:.1f} ms, p99 {v['lat_p99']:.1f} ms, max {v['lat_max']:.1f} ms, state packet gaps p99 "
                  f"{v['gap_p99']:.1f} max {v['gap_max']:.1f} ms, dropped {v['dropped']} "
                  f"({v['state_requests']} /state requests)")
    p, t = res["process"], res["thread"]
    assert p["late"] == 0 and p["dropped"] == 0, res
    if not busy:
        # alone in its process the tick never waits for anything (no GIL to share): off the CPU ~0
        assert p["p99"] <= 13.0 and p["tick"]["off_cpu_p99_ms"] <= min(0.5, t["tick"]["off_cpu_p99_ms"]), res
        assert p["lat_p99"] <= t["lat_p99"] + 2.0 and p["lat_max"] <= max(t["lat_max"], 25.0), res
    else:
        # every core busy: both are preempted (no SCHED_FIFO on a Mac; the robot's control process has it), the
        # thread also waits for the GIL behind preempted threads of its own process
        assert p["p99"] <= t["p99"] + 1.0 and p["lat_p99"] <= t["lat_p99"] + 2.0, res
