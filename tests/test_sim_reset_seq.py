"""The sim's reset counter (docs/plans/data_collection_v1.md, "Reset (sim) is one event"): ``reset_seq`` starts at 0
(the world's own initial reset does not count), increments on every ``POST /scene/reset`` (and the start-scene
variant), and is in ``/state``, the reset's reply and every robot_state.v1 record (one JSON line and the stream),
so the headset can re-anchor its AR scene on a change."""
import json

import pytest
from fastapi.testclient import TestClient
from sim_helpers import make_world

from robot_utils.nodes.sim.node import SimNode
from robot_utils.nodes.sim.server import create_app


@pytest.fixture(scope="module")
def world():
    return make_world()


@pytest.fixture
def api(world, tmp_path):
    node = SimNode(world, "rseq", str(tmp_path / "scenes"))
    with TestClient(create_app(node, start=False)) as c:
        yield c, node


def test_reset_seq_counts_every_reset_in_state_reply_and_robot_state(api, world):
    c, node = api
    base = world.reset_seq
    assert make_world().reset_seq == 0                       # a new world: its initial reset is not counted
    assert c.get("/state").json()["reset_seq"] == base
    assert c.get("/robot_state").json()["reset_seq"] == base
    r = c.post("/scene/reset").json()
    assert r["reset_seq"] == base + 1 and r["t"] == 0.0
    assert c.get("/state").json()["reset_seq"] == base + 1
    rs = c.get("/robot_state").json()
    assert rs["reset_seq"] == base + 1 and rs["schema"] == "robot_state.v1"
    assert c.post("/scene/reset", json={"start": True}).json()["reset_seq"] == base + 2
    assert node.robot_state()["reset_seq"] == base + 2
    assert any("reset_seq" in e["msg"] for e in node.events.list())


def test_reset_seq_in_every_robot_state_stream_line(api, world):
    c, node = api
    with c.stream("GET", "/robot_state/stream", params={"hz": 50}) as r:
        lines = []
        for ln in r.iter_lines():
            if ln.startswith("{"):
                lines.append(json.loads(ln))
            if len(lines) >= 3:
                break
    assert all(rec["reset_seq"] == world.reset_seq for rec in lines)
