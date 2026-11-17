"""Typed responses of the arm node: every JSON endpoint declares a response model, and that model passes
the handler's output through unchanged: same keys, same key order, same JSON types (int stays int).
response_model silently drops undeclared keys and turns ints into floats, so this compares the raw
handler output with what the client gets. Runs on the simulated arm (SimBus) behind a TestClient."""
import json
import time

import fastapi.routing
import pytest
from fastapi.routing import APIRoute
from fastapi.testclient import TestClient
from pydantic import TypeAdapter

from robot_utils.nodes.arm import config as C
from robot_utils.nodes.arm.bus import SimBus
from robot_utils.nodes.arm.node import FIELDS, ArmNode
from robot_utils.nodes.arm.server import create_app
from robot_utils.nodes.common.locks import LockStore

Q0 = (0.0, -0.6, -0.9, 0.0, 0.5, 0.0)
COMMON = {"Event", "LockOut", "MetaEndpoints", "NodeMeta", "EStopState", "EStopTriggered", "EStopCleared",
          "TimeResponse", "Health"}   # shared models (nodes/common), described there
NON_JSON = {"/", "/events", "/stream", "/ik/render"}


def wait_for(pred, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        v = pred()
        if v:
            return v
        time.sleep(0.02)
    raise AssertionError("condition not met in time")


def same(a, b, path="$"):
    """Equal as JSON, including key order and int / float / bool types."""
    assert type(a) is type(b), f"{path}: {a!r} ({type(a).__name__}) != {b!r} ({type(b).__name__})"
    if isinstance(a, dict):
        assert list(a) == list(b), f"{path}: keys {list(a)} != {list(b)}"
        for k in a:
            same(a[k], b[k], f"{path}.{k}")
    elif isinstance(a, list):
        assert len(a) == len(b), f"{path}: length"
        for i, (x, y) in enumerate(zip(a, b)):
            same(x, y, f"{path}[{i}]")
    else:
        assert a == b, f"{path}: {a!r} != {b!r}"


@pytest.fixture
def api(monkeypatch, tmp_path):
    """(call, node, routes, client): ``call(method, path, ...)`` returns the response JSON after checking
    that the route is typed, the reply validates, and it equals the handler's raw output."""
    from reBotArm_control_py.actuator.rebotarm import joint_limits, load_cfg
    raw = []
    fields = set()
    orig = fastapi.routing.serialize_response

    async def recording(**kw):
        if id(kw.get("field")) in fields:
            raw.append(json.loads(json.dumps(kw["response_content"])))   # as plain JSON, before the model
        return await orig(**kw)
    monkeypatch.setattr(fastapi.routing, "serialize_response", recording)

    limits = joint_limits(load_cfg(C.PROFILES["soft"]))
    node = ArmNode(lambda: SimBus(Q0, limits=limits), locks=LockStore(tmp_path / "locks"))
    node.description_node = "/nonexistent/description.sock"     # the shipped default arm frame, at once
    app = create_app(node, sim=True)
    routes = {(m, r.path): r for r in app.routes if isinstance(r, APIRoute) for m in r.methods}
    fields.update(id(r.response_field) for r in routes.values() if r.response_field is not None)

    def call(method, path, route=None, **kw):
        raw.clear()
        r = client.request(method, path, **kw)
        assert r.status_code == 200, (method, path, r.text)
        rt = routes[(method, route or path.split("?")[0])]
        assert rt.response_model is not None, f"{method} {path} has no response model"
        TypeAdapter(rt.response_model).validate_python(r.json())
        assert len(raw) == 1, f"{method} {path}: handler output not captured"
        try:
            same(r.json(), raw[0])
        except AssertionError as e:
            raise AssertionError(f"{method} {path}: response differs from the handler's output: {e}") from None
        return r.json()

    with TestClient(app) as client:
        wait_for(lambda: client.get("/health").json()["state"] == "uninitialized")
        wait_for(lambda: client.get("/state").json()["latest"])
        yield call, node, routes, client


def test_every_json_endpoint_is_typed_and_lossless(api):
    call, node, routes, client = api
    untyped = {f"{m} {p}" for (m, p), r in routes.items()
               if r.include_in_schema and p not in NON_JSON and r.response_model is None}
    assert not untyped

    call("GET", "/time")
    call("GET", "/health")
    assert call("GET", "/meta")["fake"] is True
    st = call("GET", "/state")                                          # never enabled: no lease, no motion
    assert st["state"] == "uninitialized" and st["motion"] is None and list(st["latest"]) == FIELDS
    assert call("GET", "/lease") is None
    for ch in ("tick", "jacobian"):
        call("GET", f"/fields?channel={ch}")
    call("GET", "/tcp")
    call("GET", "/limits")
    call("GET", "/gains")
    call("GET", "/gripper")
    call("GET", "/jam")
    call("POST", "/jam", json={"hold_s": 1.5}, headers={"X-Operator": "human"})
    call("POST", "/jam", json={"hold_s": 1.0}, headers={"X-Operator": "human"})

    tok = call("POST", "/lease", json={"client": "t"})["token"]
    h = {"X-Lease-Token": tok}
    assert isinstance(call("GET", "/lease")["lease_id"], int)
    call("POST", "/lease/renew", headers=h)
    assert call("POST", "/enable", headers=h)["enabled"]
    call("GET", "/joints")
    call("GET", "/tracking")
    tcp = call("GET", "/kinematics")["tcp"]
    pose = {"pos": [tcp["pos"][0], tcp["pos"][1], tcp["pos"][2] + 0.01], "quat": tcp["quat"]}
    far = {"pos": [3, 0, 0], "rpy": [0, 0, 0]}
    call("POST", "/kinematics", json={"q": [list(Q0), [0.0] * 6], "frame": "tcp"})
    assert call("POST", "/ik", json={"pose": pose})["ok"]
    assert not call("POST", "/ik", json={"pose": far, "seed": "measured"})["ok"]
    r = call("POST", "/ik", json={"pose": pose, "render": {"width": 320, "height": 240, "azimuth_deg": 0}})
    assert r["view"]["width"] == 320 and r["image"].startswith("data:image/png")
    call("POST", "/ik", json={"pose": pose, "render": {}})
    g = call("GET", "/geometry")
    assert g["ik"] is None and len(g["boxes"]) > 10
    call("POST", "/geometry", json={"q": list(Q0)})
    assert call("POST", "/geometry", json={"pose": pose, "gap_mm": 30})["which"] == "ik"
    call("GET", "/links")

    r = call("POST", "/track", headers=h, json={"pose": pose, "twist": [0, 0, 0.01, 0, 0, 0], "seq": 3})
    assert r["ok"] and "twist_scaled" in r
    assert not call("POST", "/track", headers=h, json={"pose": far, "seq": 4})["ok"]
    r = call("POST", "/track", headers=h, json={"pose": pose, "seq": 1})        # stale: no twist_scaled
    assert not r["ok"] and r["ik"] is None and "twist_scaled" not in r
    st = call("GET", "/state")                                          # tracking
    assert st["track"]["seq"] == 3
    assert call("GET", "/kinematics?which=command&frame=tcp")["which"] == "command"
    call("GET", "/tracking")
    call("POST", "/gripper", headers=h, json={"gap_mm": 20, "max_force_n": 30})
    call("GET", "/gripper")

    m = call("POST", "/servo", headers=h, json={"pose": {"pos": pose["pos"][:2] + [pose["pos"][2] + 0.01],
                                                         "quat": pose["quat"]}})
    assert m["kind"] == "servo" and "distance_m" in m["info"]
    st = call("GET", "/state")                                          # a servo motion in force
    assert st["motion"]["id"] == m["id"]
    assert call("POST", "/cancel", headers=h)["cancelled"]
    st = call("GET", "/state")
    assert st["motion"]["phase"] == "aborted"
    assert not call("POST", "/cancel", headers=h)["cancelled"]
    m = call("POST", "/servo", headers=h, json={"name": "home"})
    assert m["kind"] == "pose" and m["info"] == {}
    call("POST", "/cancel", headers=h)
    m = call("POST", "/home", headers=h)
    assert m["kind"] == "home"
    call("POST", "/cancel", headers=h)
    now = client.get("/time").json()["mono"]
    here = call("GET", "/kinematics")["tcp"]
    m = call("POST", "/chunk", headers=h, json={"poses": [{"pos": here["pos"], "quat": here["quat"]}] * 3,
                                                "dt": 0.1, "t0": now + 0.3, "gaps_mm": [20, 20, 20]})
    assert m["kind"] == "chunk" and isinstance(m["info"]["n"], int)
    call("GET", "/state")
    call("POST", "/limits", headers=h, json={"lin_vel": 0.1, "joint_vel": [1, 1, 1, 2, 2, 2]})
    assert call("POST", "/float", headers=h)["state"] == "floating"
    st = call("POST", "/stow", headers=h)
    assert st["stowed"] is False and st["motion"]["kind"] == "stow"
    assert call("POST", "/disable", headers=h)["enabled"] is False
    call("POST", "/tcp", headers=h, json={"offset": [-0.14, 0, 0]})
    assert call("POST", "/profile", headers=h, json={"name": "safe"})["name"] == "safe"
    assert call("POST", "/estop", json={"reason": "t", "source": "pytest"})["latched"]
    st = call("GET", "/state")                                          # e-stop latched
    assert st["estop"]["latched"]
    assert call("POST", "/estop/clear", headers=h) == {"latched": False}
    call("POST", "/reset", headers=h)
    assert call("DELETE", "/lease", headers=h) == {"released": True}

    tracking = node.tracking                                            # the check catches a dropped key
    node.tracking = lambda: {**tracking(), "undeclared": 1}
    with pytest.raises(AssertionError, match="differs"):
        call("GET", "/tracking")


def test_openapi_documents_shapes(tmp_path):
    node = ArmNode(lambda: SimBus(Q0), locks=LockStore(tmp_path / "locks"))
    spec = create_app(node, start=False, sim=True).openapi()
    for p, ops in spec["paths"].items():
        for m, op in ops.items():
            if p in NON_JSON:
                continue
            schema = op["responses"]["200"]["content"]["application/json"]["schema"]
            assert "$ref" in json.dumps(schema), (m, p)
    assert set(spec["paths"]["/events"]["get"]["responses"]["200"]["content"]) == {"text/event-stream"}
    schemas = spec["components"]["schemas"]
    for name, sch in schemas.items():                                   # every arm field is described
        if name.startswith(("HTTPValidationError", "ValidationError")) or name in COMMON:
            continue
        for f, p in (sch.get("properties") or {}).items():
            assert p.get("description") or "$ref" in json.dumps(p), f"{name}.{f}"
    assert set(schemas["TickRow"]["properties"]) == set(FIELDS)
