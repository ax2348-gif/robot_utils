"""Typed responses of the estimator node: every JSON endpoint declares a response model, and that
model passes the handler's dict through unchanged (response_model silently drops undeclared keys,
so this compares the raw handler output with what the client gets, key order and int/float included)."""
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

from robot_utils.nodes.estimator.fusion import FIELDS
from robot_utils.nodes.estimator.node import EstimatorNode
from robot_utils.nodes.estimator.server import EstimatorPose, create_app

NON_JSON = {"/events", "/stream"}
NOWHERE = "http://127.0.0.1:9"      # nothing listens: the input reports its error


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.02)
    raise AssertionError("condition not met in time")


def serve(app):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning",
                                           timeout_graceful_shutdown=1))
    t = threading.Thread(target=server.run, daemon=True)
    t.start()
    wait_for(lambda: server.started, 10)
    return f"http://127.0.0.1:{port}", server, t


@pytest.fixture
def raw(monkeypatch):
    """The handlers' raw return values, by response field (other apps in the process also serialize)."""
    out = {}
    orig = fastapi.routing.serialize_response

    async def recording(**kw):
        out[id(kw.get("field"))] = json.loads(json.dumps(kw["response_content"]))   # plain JSON, before the model
        return await orig(**kw)
    monkeypatch.setattr(fastapi.routing, "serialize_response", recording)
    return out


def checker(app, client, raw):
    routes = {(m, r.path): r for r in app.routes if isinstance(r, APIRoute) for m in r.methods}

    def call(method, path, **kw):
        r = client.request(method, path, **kw)
        assert r.status_code == 200, (path, r.text)
        route = routes[(method, path)]
        assert route.response_model is not None, f"{method} {path} has no response model"
        TypeAdapter(route.response_model).validate_python(r.json())
        expected = raw.pop(id(route.response_field))
        assert json.dumps(r.json()) == json.dumps(expected), f"{method} {path}: response differs from the handler's dict"
        return r.json()
    return call, routes


@pytest.fixture
def swerve_url():
    from robot_utils.nodes.swerve.node import SwerveNode
    from robot_utils.nodes.swerve.server import create_app as swerve_app
    from robot_utils.nodes.swerve.sim import start_sim, stop_sim

    sims, iface = start_sim()
    sw = SwerveNode(iface)
    url, srv, t = serve(swerve_app(sw))
    wait_for(lambda: sw.status == "running", 10)
    yield url
    srv.should_exit = True
    t.join(10)
    stop_sim(sims, iface)


def test_every_json_endpoint_is_typed_and_lossless_waiting(raw):
    """No input reachable: waiting, an error, no pose."""
    app = create_app(EstimatorNode(NOWHERE, NOWHERE))
    with TestClient(app) as client:
        call, routes = checker(app, client, raw)
        untyped = {f"{m} {p}" for (m, p), r in routes.items()
                   if r.include_in_schema and p not in NON_JSON and r.response_model is None}
        assert not untyped
        st = wait_for(lambda: (s := call("GET", "/state"))["error"] and s)
        assert st["state"] == "waiting" and st["pose"] is None and not st["inputs"]["imu"]["connected"]
        assert call("GET", "/pose") is None
        call("GET", "/time")
        assert call("GET", "/health") == {"ok": True, "node": "estimator", "state": "waiting"}
        assert call("GET", "/meta")["footprint"]["half_length"] > 0          # an extra key survives
        assert call("GET", "/fields")["fields"] == FIELDS
        assert call("POST", "/reset", json={}) == {"x": 0.0, "y": 0.0, "theta": 0.0, "theta_deg": 0.0}
        assert call("POST", "/reset", json={"x": 1.0, "y": -0.5, "theta": 1.0})["theta_deg"] == pytest.approx(57.2958, abs=1e-3)
        assert call("POST", "/reset") == {"x": 0.0, "y": 0.0, "theta": 0.0, "theta_deg": 0.0}   # no body


def test_every_json_endpoint_is_typed_and_lossless_running(raw, swerve_url):
    """Swerve odometry fused, the IMU input unreachable: running, a pose, both input shapes."""
    app = create_app(EstimatorNode(swerve_url, NOWHERE))
    with TestClient(app) as client:
        call, _ = checker(app, client, raw)
        st = wait_for(lambda: (s := call("GET", "/state"))["state"] == "running" and s["rate_hz"] and s, 15)
        assert st["mode"].startswith("odometry only") and st["events"]
        assert st["inputs"]["swerve"]["connected"] and st["inputs"]["swerve"]["clock_uncertainty"] is not None
        assert st["inputs"]["imu"]["error"]
        assert set(st["pose"]) == set(FIELDS) == set(EstimatorPose.model_fields)
        p = call("GET", "/pose")
        assert set(p) == set(FIELDS) and p["omega_imu"] is None                 # NaN -> null
        assert st["odometry"]["last_discontinuity"] is None and st["history"]["n"] > 0
        h = call("GET", "/history")
        t = (h["t_start"] + h["t_end"]) / 2
        got = call("GET", "/pose_at", params={"t": [str(t), str(h["t_end"]), "now"]})
        assert [p["source"] for p in got["poses"]][:2] == ["interpolated", "exact"]
        rel = call("GET", "/relative", params={"from": str(t)})
        assert rel["to_pose"]["source"] == "latest"
        call("POST", "/reset", json={"x": 1.0})
        assert call("GET", "/health")["state"] == "running"


def test_check_catches_a_dropped_key(raw):
    node = EstimatorNode(None, None)
    app = create_app(node, start=False)
    state = node.state
    node.state = lambda: {**state(), "undeclared": 1}
    with TestClient(app) as client:
        call, _ = checker(app, client, raw)
        with pytest.raises(AssertionError, match="differs"):
            call("GET", "/state")


def test_openapi_documents_shapes_and_stream_media_types():
    spec = create_app(EstimatorNode(None, None), start=False).openapi()
    ok = {(m.upper(), p): op["responses"]["200"] for p, ops in spec["paths"].items() for m, op in ops.items()}
    assert set(ok[("GET", "/stream")]["content"]) == {"application/octet-stream"}
    assert "schema" in ok[("GET", "/stream")]["description"]
    assert set(ok[("GET", "/events")]["content"]) == {"text/event-stream"}
    for (m, p), resp in ok.items():
        if p not in NON_JSON:
            assert "$ref" in json.dumps(resp["content"]["application/json"]["schema"]), (m, p)
    schemas = spec["components"]["schemas"]
    assert "monotonic" in schemas["UpstreamStatus"]["properties"]["clock_offset"]["description"]
    assert set(schemas["EstimatorPose"]["properties"]) == set(FIELDS)
