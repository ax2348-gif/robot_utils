"""Typed responses of agent: every JSON endpoint declares a response model, and that model passes
the handler's dict through unchanged (response_model silently drops undeclared keys and turns ints
into floats, so this compares the raw handler output with what the client gets: same keys, same
order, same JSON types). Runs on the simulated stack of test_agent (swerve sim, estimator,
pose_servo, fake 3d_perception), with agent itself behind a TestClient."""
import json
import socket
import threading
import time

import fastapi.routing
import pytest
import uvicorn
from fastapi.routing import APIRoute
from fastapi.testclient import TestClient
from pydantic import TypeAdapter

from robot_utils.nodes.agent_interface.sim import demo_scene, fake_perception_app

NON_JSON = {"/guide.md", "/events", "/sessions/{sid}/export.zip", "/sessions/{sid}/log.jsonl",
            "/sessions/{sid}/thumbs/{fn}", "/images/{name}", "/images/{sid}/grid.jpg"}
HUMAN = {"X-Operator": "human"}
BOARD = [380, 30, 660, 110]   # colour pixels inside the demo scene's whiteboard, seen from the start pose


def wait_for(pred, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        v = pred()
        if v:
            return v
        time.sleep(0.05)
    raise AssertionError("condition not met in time")


def serve(app):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    srv = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning", timeout_graceful_shutdown=1))
    t = threading.Thread(target=srv.run, daemon=True)
    t.start()
    wait_for(lambda: srv.started)
    return f"http://127.0.0.1:{port}", (srv, t)


def settle(nav):
    """After a quick disable / enable: let agent's pose_servo watcher see it enabled again, so a
    late 'disabled' poll does not abort the next goal."""
    wait_for(lambda: (nav._ps_status or {}).get("status") == "idle")
    time.sleep(0.3)


def same(a, b, path="$"):
    """Equal as JSON, including key order and int / float / bool types."""
    assert type(a) is type(b), f"{path}: {a!r} ({type(a).__name__}) != {b!r} ({type(b).__name__})"
    if isinstance(a, dict):
        assert list(a) == list(b), f"{path}: keys {list(a)} != {list(b)}"
        for k in a:
            same(a[k], b[k], f"{path}.{k}")
    elif isinstance(a, list):
        assert len(a) == len(b), f"{path}: length {len(a)} != {len(b)}"
        for i, (x, y) in enumerate(zip(a, b)):
            same(x, y, f"{path}[{i}]")
    else:
        assert a == b, f"{path}: {a!r} != {b!r}"


@pytest.fixture
def api(monkeypatch, tmp_path):
    """(call, nav, routes): ``call(method, path, ...)`` returns the response JSON after checking it."""
    from robot_utils.nodes.agent_interface.node import AgentNav
    from robot_utils.nodes.agent_interface.server import create_app
    from robot_utils.nodes.estimator.node import EstimatorNode
    from robot_utils.nodes.estimator.server import create_app as est_app
    from robot_utils.nodes.pose_servo.node import PoseServoNode
    from robot_utils.nodes.pose_servo.server import create_app as ps_app
    from robot_utils.nodes.swerve.node import SwerveNode
    from robot_utils.nodes.swerve.server import create_app as sw_app
    from robot_utils.nodes.swerve.sim import start_sim, stop_sim

    raw = []      # (response field, handler output as plain JSON); the peer servers record here too
    orig = fastapi.routing.serialize_response

    async def recording(**kw):
        raw.append((kw.get("field"), json.loads(json.dumps(kw["response_content"]))))
        return await orig(**kw)
    monkeypatch.setattr(fastapi.routing, "serialize_response", recording)

    sims, iface = start_sim()
    sw = SwerveNode(iface)
    servers = []
    sw_url, s = serve(sw_app(sw)); servers.append(s)
    wait_for(lambda: sw.status == "running")
    est = EstimatorNode(sw_url, None)
    est_url, s = serve(est_app(est)); servers.append(s)

    def pose():
        p = est.state().get("pose")
        return (p["x"], p["y"], p["theta"]) if p else (0.0, 0.0, 0.0)

    per_url, s = serve(fake_perception_app(demo_scene(), pose)); servers.append(s)
    ps_url, s = serve(ps_app(PoseServoNode(sw_url, est_url))); servers.append(s)
    nav = AgentNav(ps_url, per_url, est_url, str(tmp_path / "nav"), "cautious", require_imu=False, description=None)
    app = create_app(nav)
    routes = {(m, r.path): r for r in app.routes if isinstance(r, APIRoute) for m in r.methods}

    def call(method, path, route=None, status=200, **kw):
        raw.clear()
        r = client.request(method, path, **kw)
        assert r.status_code == status, (path, r.text)
        rt = routes[(method, route or path)]
        assert rt.response_model is not None, f"{method} {path} has no response model"
        TypeAdapter(rt.response_model).validate_python(r.json())
        mine = [c for f, c in raw if f is rt.response_field]
        assert len(mine) == 1, f"{method} {path}: handler output not captured"
        same(r.json(), mine[0], f"{method} {path}")
        return r.json()

    with TestClient(app) as client:
        wait_for(lambda: nav.pose_now() is not None, 15)
        yield call, nav, routes
    for srv, t in reversed(servers):
        srv.should_exit = True
        t.join(10)
    stop_sim(sims, iface)


def test_every_json_endpoint_is_typed_and_lossless(api):
    call, nav, routes = api
    untyped = {f"{m} {p}" for (m, p), r in routes.items()
               if r.include_in_schema and p not in NON_JSON and r.response_model is None}
    assert not untyped
    call("GET", "/time")
    assert call("GET", "/health")["session"] is None                      # an extra key survives
    assert call("GET", "/meta")["guide"] == "guide.md"
    st = call("GET", "/state")                                            # no session, nothing enabled
    assert st["session"] is None and st["caps"] is None and st["latch"] is None
    assert call("DELETE", "/session") == {"closed": False}
    assert call("GET", "/sessions") == []

    out = call("POST", "/session", json={"agent": "Typed", "avatar": "🦉"})
    assert "camera_summary" in out
    call("GET", "/robot", params={"comment": "describe"})
    call("POST", "/enable", json={"caps": {"max_speed": 0.12}})
    look = call("POST", "/look", json={"grid": True})
    sid = look["snapshot"]
    call("POST", "/look", json={"grid": {"step": 100, "values": "z"}})
    call("POST", "/zoom", json={"snapshot": sid, "box": BOARD, "values": "xyz"})
    call("POST", "/zoom", json={"snapshot": sid, "box": BOARD, "values": "xz"})
    call("POST", "/zoom", json={"snapshot": sid})
    call("GET", "/session/geometry")
    call("POST", "/session/geometry", json={"camera": {"xyz": [0.01, 0, 0]}})
    call("POST", "/plan/check", json={"frame": "local", "x": 0.3})
    call("POST", "/session/geometry", json={"camera": {}})
    call("POST", "/query/region", json={"select": {"box": [400, 20, 700, 160]}, "exclude": [{"disk": [520, 60, 25]}]})
    call("POST", "/query/region", json={"select": {"box": [0, 0, 4, 4]}})        # (almost) no depth: few keys
    plane = call("POST", "/query/plane", json={"select": {"box": BOARD}})     # the board, 3 m ahead
    assert plane["kind"] == "plane" and "world" in plane
    call("POST", "/query/line", json={"points": [[400, 120], [500, 125], [600, 130]], "snapshot": sid})
    call("POST", "/plan/check", json={"frame": "local", "x": 1.4, "y": -0.75})
    call("POST", "/plan/check", json={"frame": "local", "theta": 0.5})          # turn in place: note
    ap = call("POST", "/plan/approach", json={"query": plane["query"], "standoff": 2.4})

    # chat: each command replies as its endpoint
    box = " ".join(str(v) for v in BOARD)
    for text in ("help", "robot", "look grid", "region box 400 20 700 160", f"plane box {box}",
                 "line 400,120 500,125 600,130", "check 0.2 0 10d", f"approach {plane['query']} 2.4",
                 "note chat note", "place spot", "place turned 0 0 20d", "disable", "release", "enable"):
        call("POST", "/chat", json={"text": text})
    settle(nav)

    g = call("POST", "/goal", json={**ap["proposal"], "speed": 0.1, "comment": "approach"})
    gid = g["goal_id"]
    st = call("GET", f"/goals/{gid}", route="/goals/{gid}")
    assert st["done"] or "progress" in st
    res = call("GET", f"/goals/{gid}/wait", route="/goals/{gid}/wait", params={"timeout": 40})
    assert res["done"] and res["result"]["result"], res                  # reached (or aborted by a race)

    assert call("POST", "/memory/note", json={"text": "board ahead"}) == {"notes": 2}
    call("POST", "/memory/place", json={"name": "here"})
    call("POST", "/memory/place", json={"name": "facing", "theta": 0.3})
    mem = call("GET", "/memory")
    assert mem["places"]["here"]["theta"] is None
    g2 = call("POST", "/goal", json={"frame": "place:facing"})
    call("POST", "/stop")
    wait_for(lambda: nav.goals[g2["goal_id"]].done.is_set(), 20)
    call("GET", f"/goals/{g2['goal_id']}", route="/goals/{gid}")

    assert call("POST", "/reflex", params={"enabled": True}, headers=HUMAN) == {"reflex": True}
    call("POST", "/profile/normal", route="/profile/{name}", headers=HUMAN)
    assert call("POST", "/disable", json={}, headers=HUMAN) == {"enabled": False, "latched": True}
    st = call("GET", "/state")                                            # held, goals, memory, reflex
    assert st["latch"] and st["goals"] and st["memory"]["notes"]
    assert call("POST", "/release", json={}, headers=HUMAN) == {"released": True}
    call("POST", "/chat", json={"text": "enable"})
    settle(nav)
    cg = call("POST", "/chat", json={"text": "goal 0.1 0 5d"})
    call("GET", f"/goals/{cg['goal_id']}/wait", route="/goals/{gid}/wait", params={"timeout": 30})
    call("POST", "/chat", json={"text": "stop"})

    log = call("GET", "/log", params={"since": 0})
    assert {"session", "look", "query_plane", "goal", "event"} <= {t["kind"] for t in log["turns"]}
    assert all("thumbs" in t for t in log["turns"] if t["session"])
    call("POST", "/estop", json={"reason": "typed test"})                  # last: it latches the swerve
    call("POST", "/chat", json={"text": "estop chat"})
    call("GET", "/state")

    live = log["session"]["id"]
    rows = call("GET", "/sessions")                                       # live: no `ended`
    assert rows[0]["live"] and "ended" not in rows[0]
    closed = call("DELETE", "/session")
    assert closed == {"closed": live}
    rows = call("GET", "/sessions")                                       # ended: `ended`, `turns`
    assert rows[0]["ended"] and rows[0]["turns"] > 10 and not rows[0]["live"]
    stored = call("GET", f"/sessions/{live}", route="/sessions/{sid}")
    assert stored["turns"] and stored["session"]["id"] == live
    assert call("DELETE", f"/sessions/{live}", route="/sessions/{sid}", headers=HUMAN) == {"deleted": live}

    memory = nav.memory                                                   # the check catches a dropped key
    nav.memory = {**memory, "undeclared": 1}
    with pytest.raises(AssertionError, match="keys"):
        call("GET", "/memory")
    nav.memory = memory


def test_openapi_documents_shapes_and_media_types(tmp_path):
    from robot_utils.nodes.agent_interface.node import AgentNav
    from robot_utils.nodes.agent_interface.server import create_app
    spec = create_app(AgentNav(None, None, None, str(tmp_path), description=None), start=False).openapi()
    ok = {(m.upper(), p): op["responses"]["200"] for p, ops in spec["paths"].items() for m, op in ops.items()}
    media = {p: set(r["content"]) for (m, p), r in ok.items() if p in NON_JSON}
    assert media == {"/guide.md": {"text/markdown"}, "/events": {"text/event-stream"},
                     "/sessions/{sid}/export.zip": {"application/zip"},
                     "/sessions/{sid}/log.jsonl": {"application/x-ndjson"},
                     "/sessions/{sid}/thumbs/{fn}": {"image/jpeg"}, "/images/{name}": {"image/jpeg", "image/png"},
                     "/images/{sid}/grid.jpg": {"image/jpeg"}}
    assert "pose_world" in ok[("GET", "/events")]["description"]
    for (m, p), resp in ok.items():
        if p not in NON_JSON:
            assert "$ref" in json.dumps(resp["content"]["application/json"]["schema"]), (m, p)
    schemas = spec["components"]["schemas"]
    assert "m/s" in schemas["Caps"]["properties"]["max_speed"]["description"]
    assert "snapshot:<id>" in schemas["Frames"]["properties"]
