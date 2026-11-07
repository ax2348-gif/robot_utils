"""Starting/stopping nodes from the landing page (frontend/manager.py). No systemd needed."""
import asyncio
import socket
import sys
import time

import httpx
import pytest
from fastapi.testclient import TestClient

from robot_utils.frontend.app import create_app
from robot_utils.frontend.config import ConfigError, NodeEntry, StartRecipe, load_config, parse_config
from robot_utils.frontend.manager import (ManagerError, NodeManager, SubprocessRunner, SystemdRunner,
                                          UnitStatus, port_in_use)


# --- config ---------------------------------------------------------------------------------

def test_start_recipe_parsed_and_packaged():
    nodes = {n.name: n for n in load_config()}
    assert nodes["camera"].start == StartRecipe("robot_utils.nodes.orbbec")
    assert nodes["swerve"].start == StartRecipe("robot_utils.nodes.swerve")
    assert nodes["arm"].start is None
    n = parse_config({"nodes": [{"name": "a", "port": 1, "start": {"module": "x.y", "args": ["--v"]}}]})[0]
    assert n.start.argv("/py") == ["/py", "-m", "x.y", "--v"]


@pytest.mark.parametrize("start, msg", [
    ("robot_utils.nodes.x", "'start' must be"),
    ({"module": "bad module"}, "Python module path"),
    ({"module": "x", "args": "--v"}, "list of strings"),
    ({"module": "x", "extra": 1}, "'start' must be"),
])
def test_start_recipe_errors(start, msg):
    with pytest.raises(ConfigError, match=msg):
        parse_config({"nodes": [{"name": "a", "port": 1, "start": start}]})


def test_start_needs_local_node():
    with pytest.raises(ConfigError, match="this machine"):
        parse_config({"nodes": [{"name": "a", "port": 1, "host": "other", "start": {"module": "x"}}]})


# --- policy (fake runner) -------------------------------------------------------------------

class FakeRunner:
    kind = "fake"

    def __init__(self):
        self.state = {}
        self.calls = []

    async def status(self, node):
        return UnitStatus(self.state.get(node.name, "stopped"))

    async def start(self, node, argv, cwd):
        self.calls.append(("start", node.name, argv, cwd))
        self.state[node.name] = "running"
        return UnitStatus("running")

    async def stop(self, node):
        self.calls.append(("stop", node.name))
        self.state[node.name] = "stopped"

    async def logs(self, node, lines):
        return ["line"] * min(lines, 3)


NODES = [NodeEntry("cam", 8101, start=StartRecipe("robot_utils.nodes.orbbec")),
         NodeEntry("view", 8105), NodeEntry("off", 8103, enabled=False, start=StartRecipe("m"))]


def manager(busy_ports=()):
    return NodeManager(NODES, FakeRunner(), cwd="/repo", python="/venv/python",
                       port_check=lambda port: port in busy_ports)


def run(coro):
    return asyncio.run(coro)


def test_start_runs_recipe_and_stop_only_own():
    m = manager()
    st = run(m.start("cam"))
    assert st["managed"]["state"] == "running" and st["startable"] and st["unit"] == "robot-node-cam.service"
    assert m.runner.calls == [("start", "cam", ["/venv/python", "-m", "robot_utils.nodes.orbbec"], "/repo")]
    with pytest.raises(ManagerError) as e:
        run(m.start("cam"))                     # unit already running
    assert e.value.status == 409
    run(m.stop("cam"))
    with pytest.raises(ManagerError) as e:
        run(m.stop("cam"))                      # not ours / not running
    assert e.value.status == 409 and "only stops nodes it started" in e.value.detail


def test_start_refused_when_port_in_use():
    m = manager(busy_ports={8101})
    with pytest.raises(ManagerError) as e:
        run(m.start("cam"))
    assert e.value.status == 409 and "already in use" in e.value.detail
    assert m.runner.calls == []                 # never started, never stopped anything
    st = run(m.status("cam"))
    assert st["port_in_use"] and st["managed"]["state"] == "stopped"


def test_no_recipe_or_unknown_node():
    m = manager()
    with pytest.raises(ManagerError) as e:
        run(m.start("view"))
    assert e.value.status == 400
    for name in ("off", "nope"):               # disabled or unknown
        with pytest.raises(ManagerError) as e:
            run(m.start(name))
        assert e.value.status == 404


def test_endpoints():
    m = manager()
    with TestClient(create_app(NODES, manager=m)) as c:
        listing = c.get("/api/managed").json()["nodes"]
        assert {n["name"]: n["startable"] for n in listing} == {"cam": True, "view": False}
        assert c.post("/api/nodes/cam/start").json()["managed"]["state"] == "running"
        r = c.post("/api/nodes/cam/start")
        assert r.status_code == 409 and "already" in r.json()["detail"]
        assert c.get("/api/nodes/cam/logs?lines=2").json() == {"lines": ["line", "line"]}
        assert c.post("/api/nodes/cam/stop").json()["managed"]["state"] == "stopped"
        assert c.post("/api/nodes/cam/stop").status_code == 409
        assert c.post("/api/nodes/view/start").status_code == 400
        assert c.post("/api/nodes/nope/stop").status_code == 404
        page = c.get("/").text
        assert 'fetchJson("api/managed"' in page and '"/" + action' in page


# --- systemd runner: exact commands and status parsing (recorded, no systemd) ----------------

class Recorder:
    def __init__(self, show_output=""):
        self.calls = []
        self.show_output = show_output

    async def __call__(self, *argv, timeout=15.0):
        self.calls.append(list(argv))
        if argv[:3] == ("systemctl", "--user", "show"):
            return 0, self.show_output, ""
        return 0, "", ""


def test_systemd_commands():
    rec = Recorder("LoadState=loaded\nActiveState=active\nMainPID=4242\nResult=success\n"
                   "StateChangeTimestampMonotonic=0\n")
    r = SystemdRunner(run=rec)
    node = NODES[0]
    st = run(r.start(node, ["/venv/python", "-m", "robot_utils.nodes.orbbec"], "/repo"))
    assert st.state == "running" and st.pid == 4242
    assert rec.calls[0] == ["systemctl", "--user", "reset-failed", "robot-node-cam.service"]
    sr = rec.calls[1]
    assert sr[:3] == ["systemd-run", "--user", "--unit=robot-node-cam.service"]
    assert "--working-directory=/repo" in sr and "--property=TimeoutStopSec=10" in sr
    assert not any("Restart=" in a for a in sr)                     # no restart policy
    assert sr[sr.index("--") + 1:] == ["/venv/python", "-m", "robot_utils.nodes.orbbec"]
    run(r.stop(node))
    assert rec.calls[-1] == ["systemctl", "--user", "stop", "robot-node-cam.service"]
    run(r.logs(node, 5))
    assert rec.calls[-1][:4] == ["journalctl", "--user", "-u", "robot-node-cam.service"]


@pytest.mark.parametrize("show, state, result", [
    ("LoadState=not-found\nActiveState=inactive\n", "stopped", None),
    ("LoadState=loaded\nActiveState=failed\nResult=exit-code\nMainPID=0\n", "failed", "exit-code"),
    ("LoadState=loaded\nActiveState=deactivating\nMainPID=7\n", "stopping", None),
    ("LoadState=loaded\nActiveState=inactive\nMainPID=0\n", "stopped", None),
])
def test_systemd_status_parsing(show, state, result):
    st = run(SystemdRunner(run=Recorder(show)).status(NODES[0]))
    assert (st.state, st.result) == (state, result)


# --- subprocess runner: a real child process (python -m http.server as a stand-in node) -----

def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_subprocess_runner_end_to_end(tmp_path):
    port = free_port()
    node = NodeEntry("web", port, start=StartRecipe("http.server", (str(port), "--bind", "127.0.0.1")))

    async def scenario():
        m = NodeManager([node], SubprocessRunner(), cwd=str(tmp_path), python=sys.executable)
        st = await m.start("web")
        assert st["managed"]["state"] == "running"
        for _ in range(50):
            if port_in_use(port):
                break
            await asyncio.sleep(0.1)
        assert port_in_use(port)
        with pytest.raises(ManagerError):
            await m.start("web")                       # already running
        st = await m.stop("web")
        assert st["managed"]["state"] == "stopped"
        await asyncio.sleep(0.2)
        assert not port_in_use(port)
    asyncio.run(scenario())
