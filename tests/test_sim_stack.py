"""The sim stack (python -m robot_utils.nodes.sim.stack): the landing page with the sim registry starts the
description node, the sim world and the nodes in --sim world mode; they work together (the swerve node moves
the world's base, the arm node connects, the estimator integrates the odometry); Ctrl-C stops everything.
docs/sim.md, "The sim stack"."""
import json
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import httpx
import pytest
import yaml

pytest.importorskip("mujoco")

ROOT = Path(__file__).resolve().parents[1]


def test_stack_registry_parses_and_names_the_robot_instances():
    from robot_utils.frontend.config import load_config
    from robot_utils.nodes.sim.stack import DEFAULT_NODES, registry_path
    nodes = {n.name: n for n in load_config(registry_path())}
    assert set(DEFAULT_NODES.split(",")) <= set(nodes)
    robot = {n.name for n in load_config(None)}                    # the robot's own registry
    assert robot <= set(nodes) | {"sim"}                           # same instance names: pages and links unchanged
    for name in ("arm", "swerve", "camera", "wrist_camera"):
        assert nodes[name].start.args[:2] == ("--sim", "world"), name
    assert "--fake" in nodes["power"].start.args


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture
def stack(tmp_path):
    rt = tempfile.mkdtemp(prefix="rt")                             # short: unix socket paths are limited
    reg = yaml.safe_load(Path(ROOT / "src/robot_utils/nodes/sim/stack_nodes.yaml").read_text())
    for n in reg["nodes"]:
        if n["name"] == "description":                             # keep the test's state out of ~/.local
            n["start"]["args"] = ["--data-dir", str(tmp_path / "description")]
    cfg = tmp_path / "nodes.yaml"
    cfg.write_text(yaml.safe_dump(reg))
    # the robot's description snapshot: a fake robot store, pulled with a local "ssh"
    from test_sim_snapshot import LOCAL, robot_store

    from robot_utils.nodes.sim import snapshot
    _, active = robot_store(tmp_path / "robot")
    snapshot.pull("robot", str(tmp_path / "description"), command=LOCAL, remote_dir=str(tmp_path / "robot"))
    port = free_port()
    env = dict(os.environ, PYTHONPATH=str(ROOT / "src") + os.pathsep + os.environ.get("PYTHONPATH", ""))
    proc = subprocess.Popen([sys.executable, "-m", "robot_utils.nodes.sim.stack", "--port", str(port), "--config",
                             str(cfg), "--runtime-dir", rt, "--nodes", "description,sim,arm,swerve,estimator",
                             "--no-pull"],
                            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=tmp_path)
    lines = []
    end = time.monotonic() + 150
    while time.monotonic() < end:
        line = proc.stdout.readline()
        if not line:
            break
        lines.append(line)
        if "Ctrl-C stops everything" in line:
            break
    assert any("[sim stack] up:" in x for x in lines), "".join(lines)
    try:
        yield f"http://127.0.0.1:{port}", Path(rt) / "robot", proc, lines, active
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(60)
            except subprocess.TimeoutExpired:
                proc.kill()


def test_sim_stack_runs_and_stops_cleanly(stack):
    base, sockets, proc, lines, active = stack
    c = httpx.Client(base_url=base, timeout=20)
    managed = {n["name"]: n for n in c.get("/api/managed").json()["nodes"]}
    for name in ("description", "sim", "arm", "swerve", "estimator"):
        assert managed[name]["listening"] and managed[name]["managed"]["state"] == "running", name
    assert not managed["camera"]["listening"]                       # not in --nodes: a Start button
    meta = c.get("/nodes/sim/meta").json()
    assert meta["kind"] == "sim" and meta["name"] == "sim"
    assert "Sim world" in c.get("/nodes/sim/").text
    st = c.get("/nodes/sim/state").json()
    assert st["state"] == "running" and st["model"]["source"].startswith("description node")
    assert st["backends"]["swerve"]["clients"] == 1
    # the robot's snapshot: the head camera reports the robot's serial and its mount is calibrated
    assert st["model"]["camera_serials"] == {"camera": "CPC64630004G", "wrist_camera": "SIMWRIST0001"}
    assert st["model"]["camera_mounts"]["camera"]["ground"] == active
    assert any(f"ground {active}" in x for x in lines) and any("serial CPC64630004G" in x for x in lines), lines
    # the swerve node (--sim world) moves the world's base; the estimator sees it
    tok = c.post("/nodes/swerve/lease", json={"client": "test"}).json()["token"]
    H = {"X-Lease-Token": tok}
    assert c.post("/nodes/swerve/enable", headers=H).json()["enabled"]
    end = time.monotonic() + 3.0
    while time.monotonic() < end:
        c.post("/nodes/swerve/command", json={"twist": [0.3, 0.0, 0.0]}, headers=H)
        time.sleep(0.1)
    c.post("/nodes/swerve/command", json={"twist": [0.0, 0.0, 0.0]}, headers=H)
    x = c.get("/nodes/sim/viz/frame").json()["base"]["xyz"][0]
    assert x > 0.3, x
    pose = c.get("/nodes/estimator/state").json()["pose"]
    assert pose["x"] == pytest.approx(x, abs=0.1)
    c.post("/nodes/swerve/disable", headers=H)
    # the arm node (--sim world) connected to the world's arm
    arm = c.get("/nodes/arm/state").json()
    assert arm["state"] in ("uninitialized", "ready"), arm["state"]
    # markers through the landing page's proxy
    assert c.post("/nodes/sim/markers", json={"markers": [{"name": "t", "pos": [1, 0, 0.5]}]}).status_code == 200
    # robot_state.v1 as JSON lines, read by a raw TCP client through the landing page;
    # its joints are exactly the description tree's movable joints
    tree = c.get("/nodes/description/model/tree").json()
    port = int(base.rsplit(":", 1)[1])
    with socket.create_connection(("127.0.0.1", port), timeout=10) as s:
        s.sendall(b"GET /nodes/sim/robot_state/stream?format=jsonl&hz=30 HTTP/1.0\r\nHost: x\r\n\r\n")
        f = s.makefile("rb")
        assert b" 200 " in f.readline()
        while f.readline() not in (b"\r\n", b"\n"):
            pass
        recs = []
        while len(recs) < 4:                       # uvicorn chunks even HTTP/1.0: a record per chunk, skip the sizes
            line = f.readline()
            if line.startswith(b"{"):
                recs.append(json.loads(line))
    assert [r["schema"] for r in recs] == ["robot_state.v1"] * 4 and recs[-1]["seq"] > recs[0]["seq"]
    assert set(recs[0]["joints"]) == set(tree["movable"]) and recs[0]["model_version"] == tree["model_version"]
    assert recs[-1]["base"][0] >= x - 0.01 and len(recs[-1]["base"]) == 7      # (it may still be coasting)
    assert recs[-1]["overlays"]["markers"][0]["name"] == "t"
    # Ctrl-C: every node stops, sockets go
    proc.send_signal(signal.SIGINT)
    out, _ = proc.communicate(timeout=60)
    assert proc.returncode == 0, out
    assert "stopped" in out
    left = [p.name for p in sockets.glob("*.sock")]
    assert left == [], left
