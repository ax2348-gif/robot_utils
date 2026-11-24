"""The quest_teleop node (robot_utils.nodes.quest_teleop): the Quest teleop client run as a node against a fake WBC
(tests/quest_teleop_fakes.py: HTTP, a rows stream, and here its unix datagram socket) and the fake headset
(tools/quest_teleop/fake_headset.py). Headset control (quest up / down) against a scripted ssh runner: no real ssh.
Real sockets, real time (a few seconds)."""
import json
import os
import threading
import time
from dataclasses import replace
from pathlib import Path

import httpx
import pytest

from helpers import serve, stop, wait_for
from quest_teleop_fakes import FakeWbc
from robot_utils.nodes.common.client import NodeClient
from robot_utils.nodes.quest_teleop.client import quest_ctl
from robot_utils.nodes.quest_teleop.node import CLIENT_NAME, CLIENT_STATES, FIELDS, DeadlineRunner, QuestTeleopNode
from robot_utils.nodes.quest_teleop.server import create_app

from quest_teleop.fake_headset import FakeHeadsetServer, HeadsetState, play  # noqa: E402 (tools/, via the adapter)

IDLE = HeadsetState()
GRIP = replace(IDLE, right=replace(IDLE.right, grip=1.0))
REACH = replace(GRIP, right=replace(GRIP.right, pos=(0.45, -0.2, -0.30), trigger=0.8))
RELEASED = replace(REACH, right=replace(REACH.right, grip=0.0))
Y = replace(IDLE, left=replace(IDLE.left, secondary=True))     # the Y button (left): arms (Start is not live)
# (input is fresh only after a full 0.5 s window after the connect: Y at 0.7 s)
SCENARIO = [(0.0, IDLE), (0.7, IDLE), (0.71, Y), (0.8, Y), (0.81, IDLE), (0.9, IDLE), (0.91, GRIP), (1.3, GRIP),
            (1.7, REACH), (2.1, REACH), (2.11, RELEASED), (2.5, RELEASED)]


class FakeSsh:
    """The headset's shell through ssh, scripted: answers what quest_ctl sends and records every command."""

    def __init__(self, behind=0):
        self.behind = behind
        self.commands: list[str] = []
        self.running = False
        self.hosts: list[str] = []
        self.delay = 0.0                       # s: `quest run` takes this long

    def factory(self, host):
        self.hosts.append(host)
        return self

    def __call__(self, command, timeout=60.0):
        self.commands.append(command)
        R = quest_ctl.Result
        if command == "echo quest-ok":
            return R(0, "quest-ok\n")
        if "rev-list" in command:
            return R(0, f"behind={self.behind}\ndirty=0\n")
        if command.startswith("cat ") and "demo-mode" in command:
            return R(0, "off body-frozen\n" if self.running else "")
        if command.startswith("cat ") and "robot-ar" in command:
            return R(0, "")
        if '"$R/quest" run ' in command:
            time.sleep(self.delay)
            self.running = True
            return R(0, "started\n")
        if '"$R/quest" stop' in command:
            self.running = False
            return R(0, "stopped\n")
        if '"$R/quest" pull' in command:
            return R(0, "")
        if '"$R/quest" status' in command:
            return R(0, json.dumps({"last_load": {"pid": 1}, "last_load_process_alive": self.running}))
        if "capacity" in command:
            return R(0, "87\n")
        return R(1, "", f"unexpected: {command}")

    def probe(self, host, port, seconds=1.0):
        if not self.running:
            return quest_ctl.StreamInfo(False, error="connection refused")
        return quest_ctl.StreamInfo(True, 72, 72.0, "s1", True, "FOCUSED", "body-frozen")

    def quest_args(self):
        return [c.split('"$R/quest" ', 1)[1] for c in self.commands if '"$R/quest" ' in c]


def files_under(*dirs):
    return [str(p) for d in dirs for p in Path(d).rglob("*")]


@pytest.fixture
def env(tmp_path, tmp_sock_dir, monkeypatch):
    """A fake WBC with a unix datagram socket, the fake headset, and the node served over HTTP, with HOME and the
    working directory set to empty folders (the node must write nothing)."""
    home, cwd = tmp_path / "home", tmp_path / "cwd"
    home.mkdir()
    cwd.mkdir()
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.chdir(cwd)
    fake = FakeWbc(unix_path=os.path.join(tmp_sock_dir, "wbc.t.sock"))
    wurl, wsrv, wth = serve(fake.app)
    headset = FakeHeadsetServer()
    ssh = FakeSsh()
    node = QuestTeleopNode(wurl, quest_host="127.0.0.1", quest_port=headset.port, ssh_host="quest", ar=":8080",
                           runner_factory=ssh.factory, probe=ssh.probe, wait=3.0)
    url, srv, th = serve(create_app(node))
    http = httpx.Client(base_url=url, timeout=10)
    yield {"fake": fake, "node": node, "http": http, "headset": headset, "ssh": ssh, "url": url,
           "dirs": (home, cwd)}
    http.close()
    stop((srv, th), (wsrv, wth))
    headset.close()
    fake.close()


def test_meta_and_state_follow_the_node_contract(env):
    http = env["http"]
    meta = http.get("/meta").json()
    assert meta["kind"] == "quest_teleop" and meta["actuated"] and meta["endpoints"]["estop"] == "estop"
    s = http.get("/state").json()
    assert s["state"] == "stopped" and s["error"] is None
    assert set(s["lease"]) >= {"held", "holder"} and s["estop"]["latched"] is False
    assert http.get("/fields").json()["fields"] == FIELDS
    assert "Quest teleop" in http.get("/").text


def test_start_teleop_stop_over_the_unix_socket_writing_nothing(env):
    fake, http, headset, ssh, node = env["fake"], env["http"], env["headset"], env["ssh"], env["node"]
    fake.row["state"] = 2.0                                     # the WBC's stream shows it enabled (idle): Y arms
    wait_for(lambda: headset.n_clients == 1 and node.stream.latest()[0] is not None)
    r = http.post("/start")
    assert r.status_code == 200, r.text
    assert r.json()["transport"].startswith("unix ")
    assert fake.lease_client == CLIENT_NAME
    seen = set()
    rows = []
    halt = threading.Event()

    def watch():
        while not halt.is_set():
            seen.add(http.get("/state").json()["state"])
            time.sleep(0.03)

    def read_rows():
        with NodeClient(env["url"]) as c:
            for _, r in c.rows("/stream"):
                rows.append(r)
                if halt.is_set() or len(rows) > 30:
                    return

    ts = [threading.Thread(target=watch, daemon=True), threading.Thread(target=read_rows, daemon=True)]
    for t in ts:
        t.start()
    play(headset, SCENARIO, rate=100)
    halt.set()
    for t in ts:
        t.join(5)
    s = http.get("/state").json()
    assert s["teleop"]["engagements"] == 1 and s["lease"]["ours"]
    assert s["headset"]["connected"] and s["headset"]["hands"]["right"]["valid"]
    assert {"released", "engaged"} <= seen, seen
    r = http.post("/stop")
    assert r.status_code == 200 and r.json()["stopped"]
    assert http.get("/state").json()["state"] == "stopped"

    assert fake.names() == ["lease", "config", "hints", "enable", "session", "disable", "config", "release"]
    assert fake.config["max_speed"] == 0.6 and fake.config["max_omega"] == 0.8   # restored for the next holder
    targets = [m for _, m in fake.datagrams if m["type"] == "target"]
    assert targets and fake.via == {"udp": 0, "unix": len(fake.datagrams)}    # local: the unix socket only
    # Start is not live: keep-alives with armed:false until the Y press; targets only armed (one press)
    assert fake.datagrams[0][1]["type"] == "alive" and fake.datagrams[0][1]["armed"] is False
    assert all(m["armed"] is True and m["arm_seq"] == 1 for m in targets)
    assert {m["base"]["mode"] for m in targets} >= {"locked", "hint"}
    assert rows and rows[-1].dtype.names == tuple(FIELDS)
    states = {CLIENT_STATES[int(v)] for r in rows for v in r["client_state"]}
    assert states & {"released", "engaged", "waiting", "needs_release"}
    assert ssh.commands == [], "the headset is never driven unless a button asks"
    assert files_under(*env["dirs"]) == []


def test_start_refused_with_the_wbc_reason(env):
    fake, http = env["fake"], env["http"]
    fake.estops.append({})             # the WBC is e-stopped: enable answers 409
    r = http.post("/start")
    assert r.status_code == 409 and "e-stop latched" in r.json()["detail"]
    s = http.get("/state").json()
    assert s["state"] == "stopped" and "e-stop latched" in s["error"]
    assert fake.names() == ["lease", "config", "hints", "enable", "disable", "config", "release"]   # undone
    assert fake.config["max_speed"] == 0.6 and fake.config["max_omega"] == 0.8


def test_estop_reaches_the_wbc_and_stops_teleop(env):
    fake, http, headset, node = env["fake"], env["http"], env["headset"], env["node"]
    wait_for(lambda: headset.n_clients == 1 and node.stream.latest()[0] is not None)
    assert http.post("/start").status_code == 200
    r = http.post("/estop", json={"reason": "test", "source": "pytest"})
    assert r.status_code == 200
    body = r.json()
    assert body["latched"] is True and body["wbc_forwarded"] == {"arm": True, "swerve": True}
    assert fake.estops == [{"reason": "test", "source": "pytest"}]
    wait_for(lambda: http.get("/state").json()["state"] == "stopped")
    s = http.get("/state").json()
    assert s["estop"]["latched"] and s["teleop"]["stop_reason"] == "e-stop"
    assert fake.names()[-3:] == ["disable", "config", "release"]
    # the landing page's fan-out: no body, no lease
    assert http.post("/estop").json()["latched"] is True
    assert files_under(*env["dirs"]) == []


def test_estop_without_a_wbc_reports_not_latched(tmp_path):
    node = QuestTeleopNode("http://127.0.0.1:9", quest_host="127.0.0.1", quest_port=9, runner_factory=FakeSsh().factory)
    out = node.estop("x", "y")
    assert out["latched"] is False and out["error"]


def wait_ctl(http):
    return wait_for(lambda: (lambda c: c if not c["busy"] and c["rc"] is not None else None)(
        http.get("/state").json()["ctl"]), 10)


def test_quest_up_and_down_buttons_run_quest_ctl(env):
    http, ssh = env["http"], env["ssh"]
    ssh.delay = 0.5
    r = http.post("/quest/up")
    assert r.status_code == 200 and r.json()["busy"]
    assert http.post("/quest/down").status_code == 409            # one headset command at a time
    c = wait_ctl(http)
    assert c["command"] == "up" and c["rc"] == 0, c
    assert ssh.hosts == ["quest"]
    # the AR view: this machine's address toward the headset (127.0.0.1 here) at the configured port
    assert ssh.quest_args() == ["run --no-viewer --scheme=body-frozen --ar=http://127.0.0.1:8080"]
    assert ssh.commands[0] == "echo quest-ok"
    lines = "\n".join(x["line"] for x in c["lines"])
    assert "robot AR: http://127.0.0.1:8080" in lines and "stream: live" in lines
    assert http.post("/quest/down").status_code == 200
    c = wait_ctl(http)
    assert c["command"] == "down" and c["rc"] == 0
    assert ssh.quest_args()[-1] == "stop"
    assert files_under(*env["dirs"]) == []


def test_quest_up_with_the_ar_view_off(env):
    env["node"].ar = "off"
    env["http"].post("/quest/up")
    assert wait_ctl(env["http"])["rc"] == 0
    assert env["ssh"].quest_args() == ["run --no-viewer --scheme=body-frozen"]


def test_headset_commands_run_under_one_deadline():
    t = [0.0]
    calls = []

    def inner(cmd, timeout):
        calls.append(timeout)
        t[0] += 50
        return quest_ctl.Result(0, "")

    run = DeadlineRunner(inner, 120.0, clock=lambda: t[0])
    run("a", 60)
    run("b", 180)
    r = run("c", 60)
    assert calls == [60, 70, 20]
    assert run("d").rc == 124 and len(calls) == 3


def test_the_landing_page_lists_the_tool_and_the_registries_the_node():
    from robot_utils.frontend.app import list_tools
    from robot_utils.frontend.config import load_config
    from robot_utils.nodes.sim.stack import DEFAULT_NODES, registry_path
    tool = next(t for t in list_tools() if t["name"] == "quest_teleop")
    assert tool["title"] == "Quest teleop"
    # the robot's registry (no sim: A never resets), the sim stack's (A resets the sim first)
    for cfg, args in ((None, ["--ar", ":80"]), (registry_path(), ["--ar", ":8080", "--sim", "sim"])):
        n = next(n for n in load_config(cfg) if n.name == "quest_teleop")
        assert n.start.module == "robot_utils.nodes.quest_teleop" and list(n.start.args) == args
    assert "quest_teleop" not in DEFAULT_NODES.split(",")          # the sim stack: a Start button


# -- with the real WBC node (arm only, the fake arm of tests/wbc_fakes.py), real time --------------------------

def test_with_the_real_wbc_over_its_unix_socket(tmp_path, monkeypatch):
    import tempfile

    from wbc_fakes import FakeArm
    from robot_utils.nodes.wbc.model import EESpec, WbcModel, arm_config_limits, fetch_urdf
    from robot_utils.nodes.wbc.node import WbcNode
    from robot_utils.nodes.wbc.server import create_app as wbc_app
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.chdir(tmp_path)
    urdf, _ = fetch_urdf("local")
    d = tempfile.mkdtemp(prefix="qtw")
    arm = FakeArm(d + "/arm.joint.sock")
    arm_url, arm_srv, arm_t = serve(arm.app)
    wbc = WbcNode(WbcModel(urdf, (EESpec(arm_limits=arm_config_limits()),)), arm=arm_url, swerve=None,
                  estimator=None, udp_host=None, unix_socket=d + "/wbc.target.sock", arm_socket=d + "/arm.joint.sock")
    wurl, wsrv, wth = serve(wbc_app(wbc))
    headset = FakeHeadsetServer()
    ssh = FakeSsh()
    node = QuestTeleopNode(wurl, quest_host="127.0.0.1", quest_port=headset.port, runner_factory=ssh.factory)
    url, srv, th = serve(create_app(node))
    try:
        wait_for(lambda: headset.n_clients == 1 and node.stream.latest()[0] is not None, 10)
        r = httpx.post(url + "/start", timeout=10)
        assert r.status_code == 200, r.text
        assert r.json()["transport"] == f"unix {d}/wbc.target.sock"        # no UDP at all on this WBC
        play(headset, SCENARIO, rate=100)
        st = wbc.state_dict()
        assert st["session"]["accepted"] > 50 and st["session"]["last_source"] == "unix", st["session"]
        assert not st["session"]["dropped"], st["session"]["dropped"]
        assert node.state()["teleop"]["engagements"] == 1
        assert wbc.enabled and wbc.state in ("tracking", "holding", "idle")
        assert any(e["msg"] == "ARMED (arm_seq 1)" for e in wbc.events.list())   # the Y press armed the real WBC
        # e-stop through this node: the WBC latches and forwards to the arm
        assert httpx.post(url + "/estop").json()["latched"] is True
        wait_for(lambda: arm.estopped is not None, 3)
        wait_for(lambda: node.client_state() == "stopped", 5)
        assert wbc.estop.active and not wbc.enabled and wbc.lease.info is None
        assert ssh.commands == []
        assert files_under(tmp_path) == []
    finally:
        stop((srv, th), (wsrv, wth), (arm_srv, arm_t))
        headset.close()
        arm.close()


def test_top_speed_asserts_the_wbc_caps_only_while_the_base_is_still_and_stop_restores_the_original(env):
    fake, http, headset, node = env["fake"], env["http"], env["headset"], env["node"]
    # stopped: the client's top speed only (the WBC caps at the next Start)
    assert http.post("/top_speed", json={"max_speed": 0.9}).json() == {"max_speed": 0.9, "max_omega": 1.0}
    assert "config" not in fake.names()
    assert http.post("/top_speed", json={"max_speed": 2.5}).status_code == 400          # beyond the hard limit
    wait_for(lambda: headset.n_clients == 1 and node.stream.latest()[0] is not None)
    assert http.post("/start").status_code == 200
    assert fake.config["max_speed"] == 0.9 and fake.config["max_omega"] == 1.0
    # running and enabled: the WBC refuses (the base may move): 409, nothing changes
    r = http.post("/top_speed", json={"max_speed": 1.5})
    assert r.status_code == 409 and "still" in r.json()["detail"]
    assert node.cfg.stick_max_speed == 0.9 and fake.config["max_speed"] == 0.9
    # disabled (X): taken, the WBC too
    fake.enabled = False
    assert http.post("/top_speed", json={"max_speed": 1.5, "max_omega": 0.7}).status_code == 200
    assert fake.config["max_speed"] == 1.5 and fake.config["max_omega"] == 0.7
    assert http.get("/state").json()["teleop"]["wbc_caps"]["previous"] == {"max_speed": 0.6, "max_omega": 0.8}
    # someone else retunes max_omega meanwhile (POST /config needs no lease): theirs stays at stop
    fake.config["max_omega"] = 0.5
    assert http.post("/stop").status_code == 200
    assert fake.config == {"max_speed": 0.6, "max_omega": 0.5, "wheel_speed": 1.0}     # the original speed back
