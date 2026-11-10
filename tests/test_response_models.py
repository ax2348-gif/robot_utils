"""Typed responses: every JSON endpoint of the swerve node (and the common ones) declares a
response model, and that model passes the handler's dict through unchanged (response_model
silently drops undeclared keys, so this compares the raw handler output with what the client gets)."""
import json
import time

import fastapi.routing
import pytest
from fastapi.routing import APIRoute
from fastapi.testclient import TestClient
from pydantic import TypeAdapter

from robot_utils.nodes.common.locks import LockStore
from robot_utils.nodes.swerve.node import FIELDS, SwerveNode
from robot_utils.nodes.swerve.server import Telemetry, create_app
from robot_utils.nodes.swerve.sim import start_sim, stop_sim

NON_JSON = {"/events", "/stream"}


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("condition not met in time")


@pytest.fixture
def api(monkeypatch):
    """(call, node): ``call(method, path, ...)`` returns the response JSON after checking it."""
    raw = []
    orig = fastapi.routing.serialize_response

    async def recording(**kw):
        raw.append(json.loads(json.dumps(kw["response_content"])))   # as plain JSON, before the model
        return await orig(**kw)
    monkeypatch.setattr(fastapi.routing, "serialize_response", recording)

    sims, iface = start_sim()
    node = SwerveNode(iface)
    app = create_app(node)
    routes = {(m, r.path): r for r in app.routes if isinstance(r, APIRoute) for m in r.methods}

    def call(method, path, **kw):
        raw.clear()
        r = client.request(method, path, **kw)
        assert r.status_code == 200, (path, r.text)
        route = routes[(method, path)]
        assert route.response_model is not None, f"{method} {path} has no response model"
        TypeAdapter(route.response_model).validate_python(r.json())
        assert r.json() == raw[-1], f"{method} {path}: response differs from the handler's dict"
        return r.json()

    with TestClient(app) as client:
        wait_for(lambda: node.status == "running")
        yield call, node, routes
    stop_sim(sims, iface)


def test_every_json_endpoint_is_typed_and_lossless(api, tmp_locks_dir):
    call, node, routes = api
    untyped = {f"{m} {p}" for (m, p), r in routes.items()
               if r.include_in_schema and p not in NON_JSON and r.response_model is None}
    assert not untyped
    assert call("GET", "/lease") is None
    call("GET", "/time")
    assert call("GET", "/health")["state"] == "running"
    assert call("GET", "/meta")["rate_hz"] == 100                        # an extra key survives
    assert call("GET", "/fields")["fields"] == FIELDS
    call("GET", "/limits")
    st = call("GET", "/state")                                          # idle: lease, command, estop null
    assert st["command"] is None and st["lease"]["holder"] is None and st["estop"]["t"] is None

    tok = call("POST", "/lease", json={"client": "test"})["token"]
    h = {"X-Lease-Token": tok}
    call("GET", "/lease")
    call("POST", "/lease/renew", headers=h)
    call("POST", "/limits", headers=h, json={"max_speed": 0.5})
    assert call("POST", "/enable", headers=h) == {"enabled": True}
    out = call("POST", "/command", headers=h, json={"twist": [1.0, 0.0, 0.2], "seq": 3})
    assert out["clamped"] and out["soft_limits"]["max_speed"] == 0.5
    wait_for(lambda: len(node.hub.recent(20, "tick")) >= 20)
    st = call("GET", "/state")                                          # lease, command, events, telemetry
    assert st["command"]["seq"] == 3 and st["lease"]["holder"] == "test" and st["events"]
    assert set(st["latest"]) == set(FIELDS) == set(Telemetry.model_fields)
    assert call("POST", "/disable", headers=h) == {"enabled": False}

    assert call("POST", "/estop", json={"reason": "test", "source": "pytest"})["latched"]
    call("GET", "/state")                                               # e-stop latched
    assert call("POST", "/estop/clear", headers=h) == {"latched": False}
    assert call("POST", "/reset", headers=h)["state"] == "running"

    LockStore(tmp_locks_dir).place("chock", "swerve", "pytest", "test")
    wait_for(lambda: node.chocks())
    st = call("GET", "/state")                                          # a chock in force
    assert st["chocked"] and st["locks"][0]["by"] == "pytest"
    assert call("DELETE", "/lease", headers=h) == {"released": True}

    limits = node.limits                                                # the check catches a dropped key
    node.limits = lambda: {**limits(), "undeclared": 1}
    with pytest.raises(AssertionError, match="differs"):
        call("GET", "/limits")


def test_openapi_documents_shapes_and_stream_media_types():
    spec = create_app(SwerveNode("virtual:doc"), start=False).openapi()
    ok = {(m.upper(), p): op["responses"]["200"] for p, ops in spec["paths"].items() for m, op in ops.items()}
    assert set(ok[("GET", "/stream")]["content"]) == {"application/octet-stream"}
    assert "schema" in ok[("GET", "/stream")]["description"]
    assert set(ok[("GET", "/events")]["content"]) == {"text/event-stream"}
    for (m, p), resp in ok.items():
        if p not in NON_JSON:
            assert "$ref" in json.dumps(resp["content"]["application/json"]["schema"]), (m, p)
    state = spec["components"]["schemas"]["SwerveState"]["properties"]
    assert "monotonic" in state["server_mono"]["description"]
