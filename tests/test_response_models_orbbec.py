"""Typed responses of the orbbec camera node: every JSON endpoint declares a response model, and
that model passes the handler's output through unchanged (response_model silently drops undeclared
keys and coerces ints to floats, so this compares the raw handler output with what the client gets,
key by key and type by type). Runs on the synthetic camera (FakeBackend), no hardware."""
import json
import time

import pytest

pytest.importorskip("fastapi")
pytest.importorskip("httpx")
import fastapi.routing  # noqa: E402
from fastapi.encoders import jsonable_encoder  # noqa: E402
from fastapi.routing import APIRoute  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402
from pydantic import TypeAdapter  # noqa: E402

from robot_utils.nodes.orbbec.fake_backend import FakeBackend  # noqa: E402
from robot_utils.nodes.orbbec.hub import Hub  # noqa: E402
from robot_utils.nodes.orbbec.imu_filter import STATE_FIELDS  # noqa: E402
from robot_utils.nodes.orbbec.models import Settings  # noqa: E402
from robot_utils.nodes.orbbec.owner import Owner  # noqa: E402
from robot_utils.nodes.orbbec.protocol import IMU_FIELDS  # noqa: E402
from robot_utils.nodes.orbbec.server import create_app  # noqa: E402

NON_JSON = {"/events", "/stream", "/frame/{name}", "/snapshot/{name}", "/mjpeg/{name}", "/imu/stream"}


def wait_for(predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("condition not met in time")


def _nan_to_null(x):
    """What JSON can carry: the handlers' NaN floats (e.g. a default calibration's temperature) go out as null."""
    if isinstance(x, float) and x != x:
        return None
    if isinstance(x, dict):
        return {k: _nan_to_null(v) for k, v in x.items()}
    if isinstance(x, list):
        return [_nan_to_null(v) for v in x]
    return x


def _drop_added_nulls(resp, raw):
    """``resp`` without the null keys its model added where ``raw`` has none (PropertyInfo does that; see
    test_properties_model_adds_null_keys)."""
    if isinstance(resp, dict) and isinstance(raw, dict):
        return {k: _drop_added_nulls(v, raw.get(k)) for k, v in resp.items() if k in raw or v is not None}
    if isinstance(resp, list) and isinstance(raw, list) and len(resp) == len(raw):
        return [_drop_added_nulls(a, b) for a, b in zip(resp, raw)]
    return resp


def _same(a, b) -> bool:
    """Equal including JSON number types (1 vs 1.0); key order is not part of the contract."""
    return json.dumps(a, sort_keys=True) == json.dumps(b, sort_keys=True)


@pytest.fixture
def api(monkeypatch, tmp_path, tmp_locks_dir):
    """(call, owner, routes, client): ``call(method, path, ...)`` returns the response JSON after checking it."""
    raw = []
    orig = fastapi.routing.serialize_response

    async def recording(**kw):
        raw.append(_nan_to_null(jsonable_encoder(kw["response_content"])))   # before the model, as JSON
        return await orig(**kw)
    monkeypatch.setattr(fastapi.routing, "serialize_response", recording)

    hub = Hub()
    owner = Owner(FakeBackend(), hub, Settings(), startup_s=1.5, calibration_file=str(tmp_path / "imu.json"))
    app = create_app(owner, hub)
    routes = {(m, r.path): r for r in app.routes if isinstance(r, APIRoute) for m in r.methods}
    client = TestClient(app)

    def call(method, path, status=200, adds_nulls=False, **kw):
        raw.clear()
        r = client.request(method, path, **kw)
        assert r.status_code == status, (path, r.text)
        route = routes[(method, path)]
        assert route.response_model is not None, f"{method} {path} has no response model"
        TypeAdapter(route.response_model).validate_python(r.json())
        got = _drop_added_nulls(r.json(), raw[-1]) if adds_nulls else r.json()
        assert got == raw[-1] and (adds_nulls or _same(got, raw[-1])), \
            f"{method} {path}: response differs from the handler's output"
        return r.json()

    yield call, owner, routes, client
    client.close()


def test_every_json_endpoint_is_typed_and_lossless(api):
    call, owner, routes, client = api
    untyped = {f"{m} {p}" for (m, p), r in routes.items()
               if r.include_in_schema and p not in NON_JSON and r.response_model is None}
    assert not untyped

    # before the lifespan starts the owner: no calibration, no device, no depth frame
    assert call("GET", "/calibration") == {}
    d = call("GET", "/description")
    assert d["device"] is None and d["settings_version"] is None and d["frames"] == {"depth_optical": {"parent": None}}
    assert call("GET", "/depth_at") == {"mm": None}

    with client:
        wait_for(lambda: owner.state == "streaming" and owner.imu["running"])
        call("GET", "/time")
        assert call("GET", "/health") == {"ok": True, "state": "streaming"}       # no key added
        assert call("GET", "/meta")["kind"] == "camera"
        call("GET", "/state")
        call("GET", "/catalog")
        call("GET", "/settings")
        cal = call("GET", "/calibration")
        assert cal["depth_to_color"]["rotation"][0] == [1.0, 0.0, 0.0]
        assert "distortion" not in cal["depth"] and cal["imu"] == {"accel_to_depth": None}
        d = call("GET", "/description")
        assert set(d["frames"]) == {"depth_optical", "color_optical"} and d["device"]["serial"]
        assert "rotation" not in d["frames"]["depth_optical"]
        r = call("POST", "/settings", json={"display": {"colormap": "jet"}})
        assert r["ok"] and not r["restarted"]
        props = call("GET", "/properties", adds_nulls=True)
        call("GET", "/properties", params={"all": True}, adds_nulls=True)
        call("POST", "/properties", json={"id": props[-1]["id"], "value": 5}, adds_nulls=True)
        wait_for(lambda: owner.hub.latest(["depth"]).get("depth") is not None)
        assert set(call("GET", "/depth_at", params={"x": 0.3, "y": 0.6})) == {"u", "v", "seq", "mm", "median5"}
        for raw in (False, True):
            names = list(IMU_FIELDS if raw else STATE_FIELDS)
            assert call("GET", "/imu/fields", params={"raw": raw})["fields"] == names
            wait_for(lambda: owner.hub.recent_imu(1, "raw" if raw else "state"))
            out = call("GET", "/imu/latest", params={"n": 5, "raw": raw})
            assert out["fields"] == names and out["samples"] and len(out["samples"][0]) == len(names)
        assert call("POST", "/imu/reset") == {"ok": True}
        out = call("POST", "/imu/calibrate", json={"seconds": 0.2})       # the fake IMU is still enough
        assert out["source"] == "request" and len(out["gyro_bias"]) == 3 and out["temperature"] == 30.0

        owner.calibration = {**owner.calibration, "undeclared": 1}   # the check catches a dropped key
        with pytest.raises(AssertionError, match="differs"):
            call("GET", "/calibration")


def test_properties_model_adds_null_keys(api):
    """Known, pre-existing: PropertyInfo serialises every optional key, so a bool property goes out with
    min/max/step/default/error null although the backend leaves them out, and its float range fields turn
    an int property's int range into floats (0 -> 0.0). Kept (it is today's wire format); this test pins it
    so a change is deliberate."""
    call, owner, routes, client = api
    with client:
        wait_for(lambda: owner.state == "streaming")
        p = next(p for p in client.get("/properties").json() if p["type"] == "bool")
        assert {k: p[k] for k in ("min", "max", "step", "default", "error")} == dict.fromkeys(
            ("min", "max", "step", "default", "error"))
        p = next(p for p in client.get("/properties").json() if p["type"] == "int")
        assert isinstance(p["min"], float) and isinstance(p["value"], int)


def test_openapi_documents_shapes_and_binary_media_types(tmp_path):
    app = create_app(Owner(FakeBackend(), Hub(), Settings(), calibration_file=None), Hub())
    spec = app.openapi()
    ok = {(m.upper(), p): op["responses"]["200"] for p, ops in spec["paths"].items() for m, op in ops.items()}
    assert set(ok[("GET", "/stream")]["content"]) == {"application/octet-stream"}
    assert set(ok[("GET", "/imu/stream")]["content"]) == {"application/octet-stream"}
    assert set(ok[("GET", "/events")]["content"]) == {"text/event-stream"}
    assert set(ok[("GET", "/snapshot/{name}")]["content"]) == {"image/jpeg", "image/png"}
    assert set(ok[("GET", "/frame/{name}")]["content"]) == {"application/octet-stream"}
    assert "multipart/x-mixed-replace" in json.dumps(ok[("GET", "/mjpeg/{name}")]["content"])
    for (m, p), resp in ok.items():
        if p not in NON_JSON:
            assert "$ref" in json.dumps(resp["content"]["application/json"]["schema"]), (m, p)
    schemas = spec["components"]["schemas"]
    assert "rad/s" in schemas["ImuCalibrationOut"]["properties"]["gyro_bias"]["description"]
    assert "mm" in schemas["Extrinsic"]["properties"]["translation_mm"]["description"]
