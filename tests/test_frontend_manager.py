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
                                          UnitStatus, instance_up)


# --- config ---------------------------------------------------------------------------------

def test_start_recipe_parsed_and_packaged():
    nodes = {n.name: n for n in load_config()}
    assert nodes["camera"].start == StartRecipe("robot_utils.nodes.orbbec")
    assert nodes["swerve"].start == StartRecipe("robot_utils.nodes.swerve")
    assert nodes["arm"].start == StartRecipe("robot_utils.nodes.arm")
    n = parse_config({"nodes": [{"name": "a", "port": 1, "start": {"module": "x.y", "args": ["--v"]}}]})[0]
    assert n.start.argv("/py", "a") == ["/py", "-m", "x.y", "--name", "a", "--v"]


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


def manager(up=()):
    return NodeManager(NODES, FakeRunner(), cwd="/repo", python="/venv/python",
                       up_check=lambda node: node.name in up)


def run(coro):
    return asyncio.run(coro)


def test_start_runs_recipe_and_stop_only_own():
    m = manager()
    st = run(m.start("cam"))
    assert st["managed"]["state"] == "running" and st["startable"] and st["unit"] == "robot-node-cam.service"
    assert m.runner.calls == [("start", "cam", ["/venv/python", "-m", "robot_utils.nodes.orbbec",
                                                "--name", "cam"], "/repo")]
    with pytest.raises(ManagerError) as e:
        run(m.start("cam"))                     # unit already running
    assert e.value.status == 409
    run(m.stop("cam"))
    with pytest.raises(ManagerError) as e:
        run(m.stop("cam"))                      # not ours / not running
    assert e.value.status == 409 and "only stops nodes it started" in e.value.detail


def test_start_refused_when_instance_already_up():
    m = manager(up={"cam"})
    with pytest.raises(ManagerError) as e:
        run(m.start("cam"))
    assert e.value.status == 409 and "already running outside" in e.value.detail
    assert m.runner.calls == []                 # never started, never stopped anything
    st = run(m.status("cam"))
    assert st["listening"] and st["managed"]["state"] == "stopped" and st["socket"].endswith("/robot/cam.sock")


def test_instance_up_uses_the_socket_then_the_debug_port():
    import socket as so
    from robot_utils.nodes.common.serve import bind_uds, socket_path
    node = NodeEntry("probe")
    assert not instance_up(node)
    s = bind_uds(socket_path("probe"))
    try:
        assert instance_up(node)
    finally:
        s.close()
    with so.socket() as t:
        t.bind(("127.0.0.1", 0)); t.listen()
        assert instance_up(NodeEntry("other", t.getsockname()[1]))   # debug port in use


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
    assert rec.calls[1][:4] == ["systemctl", "--user", "show", "robot-node-cam.service"]   # unit file? (boot nodes)
    sr = rec.calls[2]
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

STAND_IN = """
import argparse
from fastapi import FastAPI
from robot_utils.nodes.common.serve import add_serve_args, serve
from robot_utils.nodes.common.server import add_page_routes, node_meta
p = argparse.ArgumentParser(); add_serve_args(p, "standin"); a = p.parse_args()
app = FastAPI(); add_page_routes(app, node_meta("standin", "Stand-in", "0", "standin", actuated=False))
serve(app, a)
"""


def test_subprocess_runner_end_to_end(tmp_path, monkeypatch):
    """A real child through the real serve() helper: --name -> socket, up, stop, socket gone."""
    import os
    import httpx
    from robot_utils.nodes.common.serve import socket_path
    (tmp_path / "standin_node.py").write_text(STAND_IN)
    src = os.path.join(os.path.dirname(__file__), os.pardir, "src")
    monkeypatch.setenv("PYTHONPATH", os.pathsep.join([str(tmp_path), os.path.abspath(src),
                                                      os.environ.get("PYTHONPATH", "")]))
    node = NodeEntry("web", start=StartRecipe("standin_node"))

    async def scenario():
        m = NodeManager([node], SubprocessRunner(), cwd=str(tmp_path), python=sys.executable)
        st = await m.start("web")
        assert st["managed"]["state"] == "running"
        for _ in range(100):
            if instance_up(node):
                break
            await asyncio.sleep(0.1)
        assert instance_up(node)
        with httpx.Client(transport=httpx.HTTPTransport(uds=str(socket_path("web")))) as c:
            assert c.get("http://web/meta").json()["name"] == "web"     # --name reached /meta
        with pytest.raises(ManagerError):
            await m.start("web")                       # already running
        st = await m.stop("web")
        assert st["managed"]["state"] == "stopped"
        await asyncio.sleep(0.2)
        assert not instance_up(node) and not socket_path("web").exists()
    asyncio.run(scenario())
