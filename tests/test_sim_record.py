"""Sim teleop records itself (docs/collect.md, "Sim teleop records itself"): the rigid scene's pose stream recorded as
records (RSF1 frames, exact bytes), each scene saved once per scene_id, both cut per episode; the quest_teleop node
starts the collect script when teleop starts and stops it when teleop stops, never failing either; the real child
process against the fake robot (tests/collect_fakes.py)."""
import asyncio
import io
import json
import os
import threading
import time
from types import SimpleNamespace

import httpx
import pytest

from collect_fakes import FakeRobot, FakeScene
from helpers import serve, stop, waiter
from quest_teleop_fakes import FakeWbc
from robot_utils.collect.cut import cut_session
from robot_utils.collect.episodes import read_episodes
from robot_utils.collect.local import NodeSocketTransport
from robot_utils.collect.records import Rsf1Splitter, header_time, iter_raw
from robot_utils.collect.scenes import read_scenes
from robot_utils.collect.session import Session
from robot_utils.collect.streams import StreamSpec, rotated_files
from robot_utils.nodes.quest_teleop.node import QuestTeleopNode
from robot_utils.nodes.quest_teleop.recording import RecordConfig, SimRecorder
from robot_utils.nodes.quest_teleop.server import create_app
from robot_utils.scene3d.format import decode_frame

from quest_teleop.fake_headset import FakeHeadsetServer  # noqa: E402 (tools/, via the adapter)

wait = waiter(15.0, 0.02)


@pytest.fixture
def robot():
    fake = FakeRobot()
    url, srv, th = serve(fake.app)
    fake.url = url
    yield fake
    stop((srv, th))


# -- the pose stream as records ----------------------------------------------------------------------------------

def test_rsf1_frames_become_records_with_their_exact_bytes():
    sc = FakeScene(cube=True)
    frames = [sc.frame(k, 0.01 * k, k // 3) for k in range(1, 8)]
    data = b"".join(frames)
    sp = Rsf1Splitter(clock=lambda: 42.0)
    out = []
    for i in range(0, len(data), 37):                     # arbitrary chunking
        out += sp.feed(data[i:i + 37])
    assert sp.pending == 0 and len(out) == len(frames)
    for (raw, h), want in zip(out, frames):
        payload = raw[len(raw) - h["len"]:]
        assert payload == want                             # byte for byte
        f, _ = decode_frame(payload)
        assert (h["type"], h["t_rx"], h["seq"], h["reset_seq"], h["scene_id"], h["n"]) == \
            ("scene3d", 42.0, f.seq, f.reset_seq, sc.id, 2)
        assert header_time(h) == 42.0                      # cut by receipt (robot clock), never by sim time
    with pytest.raises(ValueError):
        Rsf1Splitter().feed(b"XXXX" + b"\0" * 40)


def test_scene3d_stream_spec():
    s = StreamSpec.parse("sim/scene3d/stream")
    assert (s.name, s.path, s.params, s.framing) == ("sim.scene3d", "sim/scene3d/stream", {"format": "bin"}, "rsf1")
    assert not s.is_rows_stream() and s.to_json("http://x")["framing"] == "rsf1"
    s = StreamSpec.parse("sim/scene3d/stream?format=bin&hz=60")
    assert s.framing == "rsf1" and s.params == {"format": "bin", "hz": "60"}
    assert StreamSpec.parse("sim/scene3d/stream?format=jsonl").framing == "records"


# -- the scene saved per scene_id, the stream cut per episode ------------------------------------------------------

def _run(session):
    box = {}

    def go():
        try:
            asyncio.run(session.run(install_signals=False))
        except Exception as e:   # noqa: BLE001
            box["error"] = e
    th = threading.Thread(target=go, daemon=True)
    th.start()
    return th, box


def test_scene_saved_per_scene_id_and_the_pose_stream_cut_per_episode(robot, tmp_path):
    s = Session(robot.url, tmp_path, "stack the cube", ["quest_teleop/oplog/stream", "sim/scene3d/stream"],
                session_name="s", settle=0.3, status_every=0.3, status_stream=io.StringIO())
    th, box = _run(s)
    first = robot.scene.id
    wait(lambda: (s.dir / "scene3d" / f"{first}.blob").exists())
    time.sleep(0.3)
    eid = robot.start_episode()
    time.sleep(0.3)
    second = robot.new_scene()                               # an object added: a new scene_id
    wait(lambda: (s.dir / "scene3d" / f"{second}.blob").exists())
    robot.sim_reset()
    time.sleep(0.3)
    robot.stop_episode()
    time.sleep(0.3)
    s.request_stop()
    th.join(20)
    assert "error" not in box, box
    lines = read_scenes(s.dir)
    assert [x["scene_id"] for x in lines] == [first, second]       # once each
    assert lines[0]["view"] == {"style": "flat", "role": "auto"} and lines[0]["files"] == [f"{first}.json",
                                                                                           f"{first}.blob"]
    assert json.loads((s.dir / "scene3d" / f"{second}.json").read_text()) == robot.scene.index
    assert (s.dir / "scene3d" / f"{second}.blob").read_bytes() == robot.scene.blob
    assert robot.scene_requests.count("index") == 2
    meta = json.loads((s.dir / "session.json").read_text())
    assert meta["scenes"][0]["saved"] == sorted([first, second]) and not meta["scenes"][0]["missed"]
    assert "2 scenes" in s.status_line()
    recs = [(h, p) for _, h, p in iter_raw(rotated_files(s.dir, "sim.scene3d"))]
    assert len(recs) > 40 and {h["scene_id"] for h, _ in recs} == {first, second}
    assert all(decode_frame(p)[0].scene_id == h["scene_id"] for h, p in recs)
    assert {h["reset_seq"] for h, _ in recs} == {0, 1}

    eps = cut_session(s.dir)
    ep = next(e for e in eps if e["id"] == eid)
    assert ep["streams"]["sim.scene3d"]["records"] > 10
    assert ep["streams"]["sim.scene3d"]["time_fields"] == {"scene3d": "t_rx"}
    assert ep["scenes"] == [{"scene_id": first, "saved": True}, {"scene_id": second, "saved": True}]
    d = s.dir / "episodes" / eid
    cut = [h for _, h, _ in iter_raw([d / "sim.scene3d.bin"])]
    assert all(ep["window"][0] <= h["t_rx"] <= ep["window"][1] for h in cut)
    assert (d / "scene3d" / f"{second}.blob").read_bytes() == robot.scene.blob
    assert [x["scene_id"] for x in read_scenes(d)] == [first, second]


def test_node_socket_transport_routes_nodes_to_their_sockets(tmp_sock_dir):
    from fastapi import FastAPI
    import uvicorn
    from robot_utils.nodes.common.serve import socket_path
    app = FastAPI()

    @app.get("/thing")
    def thing(x: int = 0):
        return {"x": x}

    path = socket_path("fakenode")
    path.parent.mkdir(parents=True, exist_ok=True)
    srv = uvicorn.Server(uvicorn.Config(app, uds=str(path), log_level="warning"))
    th = threading.Thread(target=srv.run, daemon=True)
    th.start()
    wait(lambda: srv.started)

    async def go():
        async with httpx.AsyncClient(transport=NodeSocketTransport()) as c:
            return (await c.get("http://localhost/nodes/fakenode/thing", params={"x": 3})).json()
    try:
        assert asyncio.run(go()) == {"x": 3}
    finally:
        stop((srv, th))


# -- the node: Start records, Stop ends it -------------------------------------------------------------------------

class FakeRecorder:
    instances: list = []

    def __init__(self, node, sim, cfg):
        self.node, self.sim, self.cfg = node, sim, cfg
        self.calls = []
        FakeRecorder.instances.append(self)
        self.fail = False

    def start(self, baseline_seq, task):
        self.calls.append(("start", baseline_seq, task, self.node.eventlog.seq))

    def stop(self):
        self.calls.append(("stop", self.node.eventlog.last["kind"]))

    def close(self):
        self.calls.append(("close",))

    def info(self):
        return {"enabled": True, "active": bool(self.calls and self.calls[-1][0] == "start")}


@pytest.fixture
def node_env(tmp_path, tmp_sock_dir):
    fake = FakeWbc(unix_path=os.path.join(tmp_sock_dir, "wbc.t.sock"))
    wurl, wsrv, wth = serve(fake.app)
    headset = FakeHeadsetServer()
    made = []

    def factory(node, sim, cfg):
        made.append(FakeRecorder(node, sim, cfg))
        return made[-1]

    def build(**kw):
        node = QuestTeleopNode(wurl, arm=None, swerve=None, quest_host="127.0.0.1", quest_port=headset.port,
                               ar="off", sim="http://127.0.0.1:9", reset_timeout=0.5, **kw)
        url, srv, th = serve(create_app(node))
        servers.append((srv, th))
        return node, httpx.Client(base_url=url, timeout=10)
    servers = []
    yield SimpleNamespace(fake=fake, headset=headset, build=build, made=made, factory=factory, tmp=tmp_path)
    stop(*servers, (wsrv, wth))
    headset.close()
    fake.close()


def test_sim_teleop_start_records_and_stop_ends_the_session(node_env):
    node, http = node_env.build(record=RecordConfig(out=str(node_env.tmp / "data")),
                                recorder_factory=node_env.factory)
    rec = node_env.made[0]
    assert rec.sim == "http://127.0.0.1:9" and rec.cfg.out.endswith("data")
    assert http.get("/state").json()["recording"] == {"enabled": True, "active": False}
    http.post("/task", json={"text": "pick the cube"})
    assert http.post("/start").status_code == 200
    kind, baseline, task, seq_then = rec.calls[0]
    assert (kind, task) == ("start", "pick the cube") and baseline == seq_then
    start_ev = next(json.loads(x) for x in node.eventlog.since(0) if json.loads(x)["kind"] == "teleop_start")
    assert start_ev["seq"] == baseline + 1                      # teleop_start is the session's first event
    assert http.get("/state").json()["recording"]["active"] is True
    assert http.post("/stop").json()["stopped"]
    assert rec.calls[-1] == ("stop", "teleop_stop")            # after the stop event: it is recorded
    assert len([c for c in rec.calls if c[0] == "start"]) == 1


def test_no_recorder_without_a_sim_or_with_record_off(node_env, tmp_sock_dir):
    node, http = node_env.build(record=None, recorder_factory=node_env.factory)
    assert node.recorder is None and http.get("/state").json()["recording"] == {"enabled": False}
    node2 = QuestTeleopNode("http://127.0.0.1:9", arm=None, swerve=None, quest_host="127.0.0.1", quest_port=1,
                            ar="off", sim=None, record=RecordConfig(), recorder_factory=node_env.factory)
    assert node2.recorder is None and not node_env.made


def test_a_recorder_that_cannot_start_never_fails_teleop(node_env):
    def broken_popen(*a, **kw):
        raise OSError("no such interpreter (test)")
    node, http = node_env.build(record=RecordConfig(out=str(node_env.tmp / "data")),
                                recorder_factory=lambda n, s, c: SimRecorder(n, s, c, popen=broken_popen))
    r = http.post("/start")
    assert r.status_code == 200, r.text
    st = http.get("/state").json()
    assert st["state"] != "stopped" and "recording not started" in st["recording"]["error"]
    assert any("recording not started" in e["msg"] for e in st["events"])
    assert http.post("/stop").json()["stopped"]


# -- the real child process ----------------------------------------------------------------------------------------

class NodeStub:
    name = "quest_teleop"

    def __init__(self):
        self.events = SimpleNamespace(msgs=[], add=lambda msg, level="info": self.events.msgs.append((level, msg)))
        self.hub = SimpleNamespace(state_changed=lambda: None)


def test_sim_recorder_runs_the_collect_script_and_sigterm_ends_it_cleanly(robot, tmp_path):
    node = NodeStub()
    cfg = RecordConfig(out=str(tmp_path / "data"), robot=robot.url, settle=0.2,
                       streams="arm/stream,quest_teleop/oplog/stream")
    r = SimRecorder(node, "sim", cfg)
    baseline = robot.emit("teleop_start")["seq"] - 1
    r.start(baseline, "stack the cube")
    d = r.dir
    assert d.parent == tmp_path / "data"
    wait(lambda: r.status is not None and r.status["streams"].get("sim.scene3d", {}).get("records", 0) > 5
         and (d / "scene3d" / f"{robot.scene.id}.blob").exists())
    info = r.info()
    assert info["active"] and info["dir"] == str(d) and set(info["streams"]) == {"arm", "quest_teleop.oplog",
                                                                                   "sim.scene3d"}
    eid = robot.start_episode()
    time.sleep(0.4)
    robot.stop_episode()
    time.sleep(0.3)
    r.stop()
    wait(lambda: r.exit_code is not None)
    assert r.exit_code == 0 and r.error is None, (r.error, list(r.lines))
    meta = json.loads((d / "session.json").read_text())
    assert meta["stop_reason"] == "SIGTERM" and meta["stopped_wall"] and meta["task"] == "stack the cube"
    assert meta["robot"] == robot.url and meta["eventlog_baseline_seq"] == baseline     # from teleop_start on
    assert ("task", {"text": "stack the cube"}) not in robot.calls                # the node's task stands
    assert [e["id"] for e in read_episodes(d / "episodes.jsonl")] == [eid]
    assert any(m == f"recorded episode {eid} kept" for _, m in node.events.msgs), node.events.msgs
    assert not r.info()["active"]
