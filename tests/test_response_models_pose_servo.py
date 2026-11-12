"""Typed responses of the pose servo node: every JSON endpoint declares a response model, and that
model passes the handler's dict through unchanged (response_model silently drops undeclared keys,
so this compares the raw handler output with what the client gets, key order and int/float included).
Simulated swerve + estimator (odometry only) served for real; the pose servo through a TestClient."""
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

from robot_utils.nodes.common.locks import LockStore
from robot_utils.nodes.estimator.node import EstimatorNode
from robot_utils.nodes.estimator.server import create_app as estimator_app
from robot_utils.nodes.pose_servo.node import FIELDS, PoseServoNode
from robot_utils.nodes.pose_servo.server import ServoTick, create_app

NON_JSON = {"/events", "/stream"}
NOWHERE = "http://127.0.0.1:9"      # nothing listens


def wait_for(predicate, timeout=5.0, interval=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(interval)
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


def stop(srv, t):
    srv.should_exit = True
    t.join(10)


@pytest.fixture
def raw(monkeypatch):
    """The handlers' raw return values, by response field (the swerve and estimator servers in this
    process serialize too)."""
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
def upstreams():
    from robot_utils.nodes.swerve.node import SwerveNode
    from robot_utils.nodes.swerve.server import create_app as swerve_app
    from robot_utils.nodes.swerve.sim import start_sim, stop_sim

    sims, iface = start_sim()
    sw = SwerveNode(iface)
    sw_url, sw_srv, sw_t = serve(swerve_app(sw))
    wait_for(lambda: sw.status == "running", 10)
    est_url, est_srv, est_t = serve(estimator_app(EstimatorNode(sw_url, None)))
    yield sw_url, est_url
    stop(est_srv, est_t)
    stop(sw_srv, sw_t)
    stop_sim(sims, iface)


def test_every_json_endpoint_is_typed_and_lossless(raw, upstreams, tmp_locks_dir):
    sw_url, est_url = upstreams
    app = create_app(PoseServoNode(sw_url, est_url))
    with TestClient(app) as client:
        call, routes = checker(app, client, raw)
        untyped = {f"{m} {p}" for (m, p), r in routes.items()
                   if r.include_in_schema and p not in NON_JSON and r.response_model is None}
        assert not untyped
        st = wait_for(lambda: (s := call("GET", "/state"))["estimator"]["health"] != "no estimator pose yet"
                      and s["swerve"] and s["latest"] and s, 15)
        assert st["status"] == "disabled" and st["goal"] is None and st["holder"] is None   # disabled, no goal
        assert set(st["latest"]) == set(FIELDS) == set(ServoTick.model_fields)
        call("GET", "/time")
        assert call("GET", "/health")["node"] == "pose_servo"
        assert call("GET", "/meta")["rate_hz"] == 50                             # an extra key survives
        assert call("GET", "/fields")["fields"] == FIELDS
        assert call("GET", "/config")["enabled"] is False

        out = call("POST", "/enable", json={"client": "test", "config": {"require_imu": False, "max_speed": 0.2}})
        assert out["enabled"] and out["config"]["max_speed"] == 0.2
        assert call("POST", "/config", json={"pos_tol": 0.03})["config"]["pos_tol"] == 0.03
        wait_for(lambda: client.get("/state").json()["estimator"]["health"] == "ok", 5)
        g = call("POST", "/goal", json={"x": 0.05, "theta": 0.05})
        assert g["result"] is None and g["from"] and g["motion"]["speed"] == 0.2
        st = call("GET", "/state")                                               # a goal in progress
        assert st["goal"]["goal_id"] == g["goal_id"] and st["swerve_lease_id"] is not None
        st = wait_for(lambda: (s := call("GET", "/state"))["goal"]["result"] and s, 30)
        assert st["goal"]["result"] == "reached", st["goal"]                     # a goal ended
        assert call("POST", "/stop")["status"] in ("idle", "braking")
        call("POST", "/goal", json={"frame": "global", "x": 0.0, "y": 0.0, "theta": 0.0})
        assert call("POST", "/stop")["status"] in ("idle", "braking")
        st = wait_for(lambda: (s := call("GET", "/state"))["goal"]["result"] and s, 5)
        assert st["goal"]["result"] == "stopped"
        wait_for(lambda: call("GET", "/state")["status"] == "idle", 5)
        assert call("POST", "/align", json={"twist": [2.0, 0.0, 0.0]})["twist"] == [1.0, 0.0, 0.0]
        assert call("GET", "/state")["align"]["twist"] == [1.0, 0.0, 0.0]
        assert call("DELETE", "/align")["twist"] is None
        assert call("POST", "/disable") == {"enabled": False, "was_enabled": True}
        assert call("POST", "/disable") == {"enabled": False, "was_enabled": False}
        st = call("GET", "/state")
        assert st["disabled_reason"] == "requested"

        LockStore(tmp_locks_dir).place("chock", "pose_servo", "pytest", "test")
        st = wait_for(lambda: (s := call("GET", "/state"))["chocked"] and s)     # a chock in force
        assert st["locks"][0]["by"] == "pytest" and st["covered"] is False

        out = call("POST", "/estop", json={"reason": "test", "source": "pytest"})
        assert out == {"forwarded": True}                                         # no `error` key added


def test_estop_forward_failure_and_dropped_key(raw):
    node = PoseServoNode(NOWHERE, None)
    app = create_app(node, start=False)
    with TestClient(app) as client:
        call, _ = checker(app, client, raw)
        out = call("POST", "/estop", json={})
        assert out["forwarded"] is False and out["error"]
        st = call("GET", "/state")                                               # no inputs, nothing yet
        assert st["swerve"] is None and st["latest"] is None and st["estimator"]["reset_id"] is None
        get_config = node.get_config                                             # the check catches a dropped key
        node.get_config = lambda: {**get_config(), "undeclared": 1}
        with pytest.raises(AssertionError, match="differs"):
            call("GET", "/config")


def test_openapi_documents_shapes_and_stream_media_types():
    spec = create_app(PoseServoNode(None, None), start=False).openapi()
    ok = {(m.upper(), p): op["responses"]["200"] for p, ops in spec["paths"].items() for m, op in ops.items()}
    assert set(ok[("GET", "/stream")]["content"]) == {"application/octet-stream"}
    assert "schema" in ok[("GET", "/stream")]["description"]
    assert set(ok[("GET", "/events")]["content"]) == {"text/event-stream"}
    for (m, p), resp in ok.items():
        if p not in NON_JSON:
            assert "$ref" in json.dumps(resp["content"]["application/json"]["schema"]), (m, p)
    schemas = spec["components"]["schemas"]
    assert "from" in schemas["GoalOut"]["properties"]                        # the wire name, not from_
    assert set(schemas["ServoTick"]["properties"]) == set(FIELDS)
