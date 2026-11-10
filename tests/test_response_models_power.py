"""Typed responses of the power node: every JSON endpoint declares a response model, and that model
passes the handler's output through unchanged: same keys, same key order, same JSON types (int stays
int). response_model silently drops undeclared keys, so this compares the raw handler output with
what the client gets. Fake bus, injected e-stop / power-off: nothing is ever powered off."""
import json

import fastapi.routing
import pytest
from fastapi.routing import APIRoute
from fastapi.testclient import TestClient
from pydantic import TypeAdapter

from robot_utils.nodes.power import node as node_mod
from robot_utils.nodes.power.node import PowerNode
from robot_utils.nodes.power.server import PowerSource, create_app
from robot_utils.nodes.power.sources import TetherSource, UpsSource
from robot_utils.nodes.power.ups import CAPTURED_FULL, FakeBus, UpsHat, flatten

NON_JSON = {"/events"}


@pytest.fixture(autouse=True)
def no_system_power_off(monkeypatch):
    import subprocess

    def boom(*a, **k):
        raise AssertionError(f"tests must never run processes: {a}")
    monkeypatch.setattr(subprocess, "run", boom)
    monkeypatch.setattr(node_mod, "estop_robot", lambda *a, **k: (_ for _ in ()).throw(AssertionError("real e-stop")))


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def same(a, b, path="$"):
    """Equal JSON with the same key order and the same types (1 and 1.0 differ)."""
    assert type(a) is type(b), f"{path}: {a!r} vs {b!r}"
    if isinstance(a, dict):
        assert list(a) == list(b), f"{path}: keys {list(a)} vs {list(b)}"
        for k in a:
            same(a[k], b[k], f"{path}.{k}")
    elif isinstance(a, list):
        assert len(a) == len(b), f"{path}: length"
        for i, (x, y) in enumerate(zip(a, b)):
            same(x, y, f"{path}[{i}]")
    else:
        assert a == b, f"{path}: {a!r} vs {b!r}"


@pytest.fixture
def api(monkeypatch, tmp_path):
    """(call, node, clock, bus, routes): ``call(method, path, ...)`` returns the response JSON after checking it."""
    raw = []
    orig = fastapi.routing.serialize_response

    async def recording(**kw):
        raw.append(json.loads(json.dumps(kw["response_content"])))   # as plain JSON, before the model
        return await orig(**kw)
    monkeypatch.setattr(fastapi.routing, "serialize_response", recording)

    bus, clock, calls = FakeBus(), Clock(), []

    def estop():
        calls.append("estop")
        return [{"name": "swerve", "url": "http://x/estop", "status": "latched", "detail": ""},
                {"name": "camera", "status": "skipped"}]

    def power_off():
        calls.append("power_off")
        raise RuntimeError("refused in tests")                       # also covers a failed power-off

    node = PowerNode([UpsSource(UpsHat(bus)), TetherSource("main")], estop=estop, power_off=power_off, clock=clock,
                     settings_path=tmp_path / "power.json", audit_path=tmp_path / "shutdown.jsonl",
                     spawn=lambda fn: fn())
    app = create_app(node, start=False, identify=lambda ip: f"user@{ip}")
    routes = {(m, r.path): r for r in app.routes if isinstance(r, APIRoute) for m in r.methods}

    def call(method, path, status=200, **kw):
        raw.clear()
        r = client.request(method, path, **kw)
        assert r.status_code == status, (path, r.text)
        route = routes[(method, path)]
        assert route.response_model is not None, f"{method} {path} has no response model"
        TypeAdapter(route.response_model).validate_python(r.json())
        assert len(raw) == 1, f"{method} {path}: handler output not captured"
        try:
            same(r.json(), raw[0])
        except AssertionError as e:
            raise AssertionError(f"{method} {path}: response differs from the handler's output: {e}") from None
        return r.json()

    with TestClient(app) as client:
        yield call, node, clock, bus, routes
    assert calls in ([], ["estop", "power_off"])


def test_every_json_endpoint_is_typed_and_lossless(api):
    call, node, clock, bus, routes = api
    untyped = {f"{m} {p}" for (m, p), r in routes.items()
               if r.include_in_schema and p not in NON_JSON and r.response_model is None}
    assert not untyped
    call("GET", "/time")
    call("GET", "/health")
    assert call("GET", "/meta")["fake"] is False                        # an extra key survives
    st = call("GET", "/state")                                          # no reading yet: envelope only
    assert "percent" not in st["sources"][0] and st["sources"][1]["note"]
    assert call("GET", "/history")["rows"] == []

    node.step(clock.t)                                                  # a reading (charging at 512 mA)
    st = call("GET", "/state")
    ups, main = st["sources"]
    assert ups["percent"] == 59 and ups["input"]["present"] and ups["control"]["address"] == 0x2d
    assert set(ups) | set(main) == set(PowerSource.model_fields)
    h = call("GET", "/history")
    assert h["rows"][0][2] == 59                                        # an int stays an int
    call("GET", "/history", params={"seconds": 60})
    call("GET", "/low_voltage")
    call("POST", "/low_voltage", json={"enabled": True, "volts": 13.0, "hold_s": 20, "by": "me"})

    bus.regs = flatten(CAPTURED_FULL)                                   # full, then discharging: a runtime
    bus.set_status(charging=False, vbus=False)
    bus.set_word(0x22, -1500)
    clock.t += 1
    node.step(clock.t)
    st = call("GET", "/state")
    assert st["sources"][0]["charge_state"] == "discharging" and st["sources"][0]["runtime_min"]

    h = {"X-Forwarded-For": "100.64.0.7"}
    sd = call("POST", "/shutdown", status=202, headers=h,
              json={"confirm": "shutdown", "reason": "test", "by": "me", "delay_s": 20})
    assert sd["pending"] and sd["client"] == "user@100.64.0.7"
    st = call("GET", "/state")                                          # pending
    assert st["shutdown"]["phase"] == "countdown"
    sd = call("POST", "/shutdown/cancel", json={"by": "you"}, headers={"X-Forwarded-For": "100.64.0.8"})
    assert sd["last"]["cancelled_by"] == "you"
    sd = call("POST", "/shutdown", status=202, json={"confirm": "shutdown", "reason": "again", "by": "me",
                                                     "delay_s": 10})    # local: not rate-limited
    clock.t += 10
    node.step(clock.t)                                                  # e-stop, then the (refused) power-off
    sd = call("GET", "/shutdown")
    assert sd["last"]["phase"] == "failed" and sd["log"]
    st = call("GET", "/state")
    assert st["shutdown"]["last"]["error"] and st["events"]
    log = call("GET", "/log", params={"n": 100})["log"]
    assert {"requested", "cancelled", "executing", "estopped", "failed", "low_voltage"} <= {x["event"] for x in log}
    assert next(x for x in log if x["event"] == "estopped")["results"][0]["status"] == "latched"   # extra keys kept

    bus.fail = "bus gone"                                               # a read error
    clock.t += 6
    node.step(clock.t)
    st = call("GET", "/state")
    assert st["sources"][0]["stale"] and st["error"]

    low = node.low.public                                               # the check catches a dropped key
    node.low.public = lambda now: {**low(now), "undeclared": 1}
    with pytest.raises(AssertionError, match="differs"):
        call("GET", "/low_voltage")


def test_openapi_documents_shapes_and_media_types(tmp_path):
    spec = create_app(PowerNode([], settings_path=tmp_path / "p.json", audit_path=tmp_path / "a.jsonl"),
                      start=False).openapi()
    ok = {(m.upper(), p): op["responses"] for p, ops in spec["paths"].items() for m, op in ops.items()}
    assert set(ok[("GET", "/events")]["200"]["content"]) == {"text/event-stream"}
    for (m, p), resp in ok.items():
        if p not in NON_JSON:
            resp = resp["202" if (m, p) == ("POST", "/shutdown") else "200"]
            assert "$ref" in json.dumps(resp["content"]["application/json"]["schema"]), (m, p)
    schemas = spec["components"]["schemas"]
    assert "charging" in schemas["PowerSource"]["properties"]["current_a"]["description"]
    assert schemas["AuditRecord"]["additionalProperties"] is True
