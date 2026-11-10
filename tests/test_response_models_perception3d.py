"""Typed responses of 3d_perception: every JSON endpoint declares a response model, and that model
passes the handler's dict through unchanged (same keys, same order, same JSON types: response_model
silently drops undeclared keys and turns ints into floats). Runs on the fake camera node (+ the stub
estimator and description of test_perception3d_node), with 3d_perception behind a TestClient."""
import json

import fastapi.routing
import pytest
from fastapi.routing import APIRoute
from fastapi.testclient import TestClient
from pydantic import TypeAdapter

from robot_utils.nodes.perception3d.node import Params, PerceptionNode
from robot_utils.nodes.perception3d.server import create_app
from test_perception3d_node import camera, serve, stop, stub_description, stub_estimator, wait_for  # noqa: F401
from test_response_models_agent_nav import same

NON_JSON = {"/events", "/stream", "/frame", "/color.jpg", "/depth.png"}


@pytest.fixture
def recorder(monkeypatch):
    """make(node) -> (call, routes) for a TestClient on create_app(node)."""
    raw = []      # (response field, handler output as plain JSON); the peer servers record here too
    orig = fastapi.routing.serialize_response

    async def recording(**kw):
        raw.append((kw.get("field"), json.loads(json.dumps(kw["response_content"]))))
        return await orig(**kw)
    monkeypatch.setattr(fastapi.routing, "serialize_response", recording)

    def make(client, app):
        routes = {(m, r.path): r for r in app.routes if isinstance(r, APIRoute) for m in r.methods}

        def call(method, path, **kw):
            raw.clear()
            r = client.request(method, path, **kw)
            assert r.status_code == 200, (path, r.text)
            rt = routes[(method, path)]
            assert rt.response_model is not None, f"{method} {path} has no response model"
            TypeAdapter(rt.response_model).validate_python(r.json())
            mine = [c for f, c in raw if f is rt.response_field]
            assert len(mine) == 1, f"{method} {path}: handler output not captured"
            same(r.json(), mine[0], f"{method} {path}")
            return r.json()
        return call, routes
    return make


def test_no_inputs(recorder, monkeypatch):
    """Not started, no inputs: not ready, waiting, ints in the params stay ints."""
    node = PerceptionNode(None, None, None, Params(idle_s=0))
    app = create_app(node, start=False)
    with TestClient(app) as client:
        call, routes = recorder(client, app)
        untyped = {f"{m} {p}" for (m, p), r in routes.items()
                   if r.include_in_schema and p not in NON_JSON and r.response_model is None}
        assert not untyped
        call("GET", "/time")
        assert call("GET", "/health")["state"] == "waiting"
        assert call("GET", "/meta")["api"]["point"] == "point"             # an extra key survives
        assert call("GET", "/calibration") == {"ready": False}
        st = call("GET", "/state")
        assert st["latest"] is None and st["inputs"] == {} and st["params"]["idle_s"] == 0
        assert call("GET", "/settings")["idle_s"] == 0
        assert call("POST", "/settings", json={"scale": 0.25, "compensation": "none"})["scale"] == 0.25

        to_json = Params.to_json                                           # the check catches a dropped key
        monkeypatch.setattr(Params, "to_json", lambda self: {**to_json(self), "undeclared": 1})
        with pytest.raises(AssertionError, match="keys"):
            call("GET", "/settings")


def test_fake_camera_with_estimator_and_description(recorder, camera):
    est_url, *est = serve(stub_estimator())
    desc_url, *desc = serve(stub_description())
    node = PerceptionNode(camera, est_url, desc_url, Params(idle_s=0))
    app = create_app(node)
    try:
        with TestClient(app) as client:
            call, _ = recorder(client, app)
            wait_for(lambda: client.get("/state").json()["mode"] == "full", 20)
            st = call("GET", "/state")
            assert set(st["inputs"]) == {"frames", "imu", "pose"} and st["latest"]["seq"] > 0
            assert st["sources"]["description"]["from"]["ground"]
            cal = call("GET", "/calibration")
            assert cal["ready"] and cal["base_calibration"]["calibrated"] and len(cal["base_to_color"]) == 4
            q = call("GET", "/point", params={"u": 640, "v": 500})
            assert q["depth_mm"] is not None and q["point_base"] is not None and "window_mm" in q
            call("GET", "/point", params={"u": 0.5, "v": 0.7, "norm": True})
            out = call("GET", "/point", params={"u": -50, "v": 10})              # outside: no window
            assert out["source"] == "outside" and "window_mm" not in out
            call("GET", "/health")
            call("GET", "/settings")
    finally:
        stop(tuple(est), tuple(desc))


def test_openapi_documents_shapes_and_media_types():
    spec = create_app(PerceptionNode(None, None, None), start=False).openapi()
    ok = {(m.upper(), p): op["responses"]["200"] for p, ops in spec["paths"].items() for m, op in ops.items()}
    media = {p: set(r["content"]) for (m, p), r in ok.items() if p in NON_JSON}
    assert media == {"/events": {"text/event-stream"}, "/stream": {"application/octet-stream"},
                     "/frame": {"application/octet-stream"}, "/color.jpg": {"image/jpeg"},
                     "/depth.png": {"image/png"}}
    assert "PerceptionState" in ok[("GET", "/events")]["description"]
    for (m, p), resp in ok.items():
        if p not in NON_JSON:
            assert "$ref" in json.dumps(resp["content"]["application/json"]["schema"]), (m, p)
    schemas = spec["components"]["schemas"]
    assert schemas["ParamsOut"]["properties"]["scale"]["type"] == "number"
    assert "from" in schemas["BaseCalibration"]["properties"]
