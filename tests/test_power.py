"""Power node (nodes/power): UPS register decoding, sources, shutdown, low-voltage, the API and the CLI.

Everything runs on the fake bus with injected e-stop / power-off fakes: no test can reach the I2C
bus, the system's power-off, or another node.
"""
import json

import pytest
from fastapi.testclient import TestClient

from robot_utils.admin import power as cli
from robot_utils.nodes.common.lease import RequestError
from robot_utils.nodes.power import node as node_mod
from robot_utils.nodes.power import shutdown as sd_mod
from robot_utils.nodes.power.lowvolt import REARM_S, LowVoltage
from robot_utils.nodes.power.node import PowerNode
from robot_utils.nodes.power.server import create_app
from robot_utils.nodes.power.shutdown import AuditLog, RateLimit, ShutdownController, refuse_power_off
from robot_utils.nodes.power.sources import STALE_S, TetherSource, UpsSource
from robot_utils.nodes.power.ups import (CAPTURED, CAPTURED_FULL, POWER_OFF_MAGIC, REG_POWER_OFF, FakeBus, SimBus,
                                         UpsHat, decode, flatten)


@pytest.fixture(autouse=True)
def no_system_power_off(monkeypatch):
    """Belt and braces: any attempt to run a process or the real e-stop fan-out fails the test."""
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


class Recorder:
    """Injected e-stop and power-off: record the order they ran in; power-off never touches anything."""

    def __init__(self):
        self.calls = []

    def estop(self):
        self.calls.append("estop")
        return [{"name": "swerve", "status": "latched"}, {"name": "camera", "status": "skipped"}]

    def power_off(self):
        self.calls.append("power_off")


def make_node(tmp_path, bus=None, clock=None, rec=None, fresh_settings=True):
    bus = bus or FakeBus()
    clock = clock or Clock()
    rec = rec or Recorder()
    node = PowerNode([UpsSource(UpsHat(bus)), TetherSource("main")], estop=rec.estop, power_off=rec.power_off,
                     clock=clock, settings_path=tmp_path / "power.json", audit_path=tmp_path / "shutdown.jsonl",
                     spawn=lambda fn: fn())
    return node, bus, clock, rec


# -- registers ------------------------------------------------------------------------------------

def test_decode_captured_reading():
    d = decode(flatten(CAPTURED))
    assert d["valid"] and d["firmware"] == "2.0" and not d["power_off_pending"]
    assert d["charging"] and d["fast_charge"] and d["vbus_present"] and d["charger_state"] == "constant current"
    assert d["links"] == {"fuel_gauge": True, "charger": True}
    assert (d["vbus_mv"], d["vbus_ma"], d["vbus_mw"]) == (8675, 2090, 18172)
    assert (d["pack_mv"], d["current_ma"], d["percent"], d["remaining_mah"]) == (15587, 512, 59, 2815)
    assert d["time_to_empty_min"] is None and d["time_to_full_min"] == 250
    assert d["cells_mv"] == [3920, 3874, 3877, 3916] and sum(d["cells_mv"]) == pytest.approx(15587, abs=5)
    assert d["control"] == {"auto_power_on": True, "watchdog": False, "address": 0x2d, "wd_timeout_s": 255,
                            "wd_delay_s": 255}


def test_decode_full_reading_and_signed_current():
    d = decode(flatten(CAPTURED_FULL))
    assert (d["pack_mv"], d["current_ma"], d["percent"], d["charger_state"]) == (16392, 0, 100, "constant voltage")
    assert UpsSource.readings(d)["charge_state"] == "full"
    bus = FakeBus()
    bus.set_word(0x22, -1234)
    bus.set_status(charging=False, vbus=False)
    bus.set_word(0x28, 0xFFFF)
    r = UpsSource.readings(decode(bus.regs))
    assert r["current_a"] == -1.234 and r["charge_state"] == "discharging" and not r["input_present"]
    assert r["runtime_source"] == "estimate" and r["runtime_min"] == round(2815 / 1234 * 60)
    bus.set_word(0x28, 95)
    assert UpsSource.readings(decode(bus.regs))["runtime_min"] == 95


def test_ups_hat_only_reads():
    bus = FakeBus()
    UpsHat(bus).read()
    assert bus.writes == []


def test_sim_bus_moves_but_keeps_shape():
    s = SimBus(clock=Clock(0.0))
    a, b = decode(s.regs_at(100.0)), decode(s.regs_at(SimBus.PERIOD_S * 0.75))
    assert a["valid"] and b["valid"] and a["charging"] and not b["charging"] and b["current_ma"] < 0


# -- sources ----------------------------------------------------------------------------------------

def test_sources_shape(tmp_path):
    node, bus, clock, _ = make_node(tmp_path)
    node.step(clock.t)
    st = node.state()
    assert [s["id"] for s in st["sources"]] == ["ups", "main"]
    envelope = {"id", "type", "role", "telemetry", "ok", "stale", "error", "age_s"}
    for s in st["sources"]:
        assert envelope <= set(s)
    ups, main = st["sources"]
    assert (ups["type"], ups["role"], ups["telemetry"], ups["ok"], ups["stale"]) == ("ups", "pi", True, True, False)
    for k in ("percent", "pack_v", "cells_v", "current_a", "charge_state", "input_present", "runtime_min",
              "time_to_full_min"):
        assert k in ups
    assert ups["percent"] == 59 and ups["pack_v"] == 15.587 and ups["input"]["present"] is True
    assert main == {"id": "main", "type": "tether", "role": "main", "telemetry": False, "ok": None,
                    "stale": False, "error": None, "age_s": None, "note": "bench supply on a tether; no battery installed"}
    assert st["state"] == "UPS 59 % · charging" and st["error"] is None
    assert len(json.dumps(st)) < 6000          # the landing tile polls it every second
    sd = st["shutdown"]                        # flat, same keys whether or not one is pending
    assert {"pending", "phase", "at", "in_s", "by", "reason", "requested_at", "last"} <= set(sd)
    assert sd["pending"] is False and sd["at"] is None and sd["in_s"] is None


def test_ups_stale_and_errors(tmp_path):
    node, bus, clock, _ = make_node(tmp_path)
    node.step(clock.t)
    bus.fail = "Remote I/O error"
    clock.t += 1.0
    node.step(clock.t)
    ups = node.state()["sources"][0]
    assert ups["ok"] is False and not ups["stale"] and "I2C read failed" in ups["error"]
    clock.t += STALE_S + 1
    node.step(clock.t)
    st = node.state()
    assert st["sources"][0]["stale"] and st["state"] == "UPS stale"
    bus.fail = None
    bus.regs[0x00] = 0x00          # wrong ID: refused, never decoded as a battery
    clock.t += 1
    node.step(clock.t)
    assert "unexpected ID" in node.state()["sources"][0]["error"]


def test_no_i2c_library_reported(tmp_path):
    node = PowerNode([UpsSource(None, error="no I2C library: install smbus2"), TetherSource()],
                     estop=lambda: [], settings_path=tmp_path / "p.json", audit_path=None, clock=Clock())
    node.step(1000.0)
    st = node.state()
    assert st["state"] == "UPS unavailable" and "smbus2" in st["error"]


def test_history_ring(tmp_path):
    node, bus, clock, _ = make_node(tmp_path)
    for _ in range(5):
        node.step(clock.t)
        clock.t += node_mod.HISTORY_S
    h = node.history_rows("ups", 86400)
    assert h["fields"][:3] == ["t", "wall", "percent"] and len(h["rows"]) == 5 and h["rows"][0][2] == 59
    assert len(node.history_rows("ups", 2 * node_mod.HISTORY_S)["rows"]) == 2
    assert node.history["ups"].maxlen == 24 * 3600 / node_mod.HISTORY_S
    with pytest.raises(RequestError):
        node.history_rows("main", 60)


# -- shutdown -----------------------------------------------------------------------------------------

def controller(tmp_path, rec=None, clock=None, spawn=lambda fn: fn()):
    rec = rec or Recorder()
    clock = clock or Clock()
    return ShutdownController(rec.estop, rec.power_off, audit=AuditLog(tmp_path / "a.jsonl"), clock=clock,
                              spawn=spawn), rec, clock


def test_shutdown_refused_without_exact_confirm(tmp_path):
    c, rec, _ = controller(tmp_path)
    for bad in (None, "", "yes", "SHUTDOWN", "shutdown ", True):
        with pytest.raises(RequestError) as e:
            c.request(confirm=bad, reason="r", by="me", client="ip")
        assert e.value.status == 400
    with pytest.raises(RequestError) as e:
        c.request(confirm="shutdown", reason="  ", by="me", client="ip")
    assert e.value.status == 422
    for d in (9.9, 301, "x"):
        with pytest.raises(RequestError) as e:
            c.request(confirm="shutdown", reason="r", by="me", client="ip", delay_s=d)
        assert e.value.status == 422
    assert not c.pending and rec.calls == []
    log = c.audit.list()
    assert len(log) == 10 and all(r["event"] == "refused" and r["by"] == "me" and r["client"] == "ip" for r in log)


def test_shutdown_countdown_estop_then_power_off(tmp_path):
    c, rec, clock = controller(tmp_path)
    out = c.request(confirm="shutdown", reason="end of day", by="avery", client="avery · laptop", delay_s=30)
    assert out["pending"] and out["phase"] == "countdown" and out["in_s"] == 30 and out["by"] == "avery"
    c.tick(clock.t + 29.9)
    assert rec.calls == [] and c.pending["phase"] == "countdown"
    clock.t += 30
    c.tick()
    assert rec.calls == ["estop", "power_off"]          # e-stop strictly before power-off
    st = c.public()
    assert not st["pending"] and st["last"]["phase"] == "done"
    events = [r["event"] for r in c.audit.list()]
    assert events == ["requested", "executing", "estopped", "powered_off"]
    c.tick(clock.t + 100)                                # once only
    assert rec.calls == ["estop", "power_off"]
    # the audit file survives (it is what you read after the reboot)
    lines = [json.loads(x) for x in (tmp_path / "a.jsonl").read_text().splitlines()]
    assert lines[0]["event"] == "requested" and lines[0]["reason"] == "end of day"


def test_shutdown_cancel(tmp_path):
    c, rec, clock = controller(tmp_path)
    c.request(confirm="shutdown", reason="r", by="a", client="x")
    clock.t += 10
    out = c.cancel(by="b", client="y")
    assert not out["pending"] and out["last"]["phase"] == "cancelled" and out["last"]["cancelled_by"] == "b"
    c.tick(clock.t + 1000)
    assert rec.calls == []
    with pytest.raises(RequestError) as e:
        c.cancel(by="b", client="y")
    assert e.value.status == 409


def test_second_request_refused_while_pending(tmp_path):
    c, rec, _ = controller(tmp_path)
    c.request(confirm="shutdown", reason="first", by="a", client="x")
    with pytest.raises(RequestError) as e:
        c.request(confirm="shutdown", reason="second", by="b", client="y")
    assert e.value.status == 409 and "first" in str(e.value)
    assert c.pending["reason"] == "first"


def test_cancel_too_late_once_executing(tmp_path):
    jobs = []
    c, rec, clock = controller(tmp_path, spawn=jobs.append)
    c.request(confirm="shutdown", reason="r", by="a", client="x", delay_s=10)
    c.tick(clock.t + 10)
    with pytest.raises(RequestError) as e:
        c.cancel(by="b", client="y")
    assert e.value.status == 409 and "too late" in str(e.value)
    jobs[0]()
    assert rec.calls == ["estop", "power_off"]


def test_power_off_runs_even_if_estop_fails_and_failure_is_reported(tmp_path):
    calls = []

    def estop():
        calls.append("estop")
        raise RuntimeError("socket gone")

    def power_off():
        calls.append("power_off")
        raise RuntimeError("polkit says no")
    c = ShutdownController(estop, power_off, clock=Clock(), spawn=lambda fn: fn())
    c.request(confirm="shutdown", reason="r", by="a", client="x", delay_s=10)
    c.tick(2000)
    assert calls == ["estop", "power_off"]
    last = c.public()["last"]
    assert last["phase"] == "failed" and "polkit" in last["error"]


def test_default_power_off_refuses():
    with pytest.raises(RuntimeError, match="not wired"):
        refuse_power_off()
    c = ShutdownController(lambda: [], clock=Clock(), spawn=lambda fn: fn())
    c.request(confirm="shutdown", reason="r", by="a", client="x", delay_s=10)
    c.tick(5000)
    assert c.public()["last"]["phase"] == "failed"


def test_rate_limit():
    r = RateLimit(min_interval=60, per_hour=3)
    r.check("a", 0)
    r.note("a", 0)
    with pytest.raises(RequestError) as e:
        r.check("a", 30)
    assert e.value.status == 429
    r.check("b", 30)
    for t in (100, 200):
        r.check("a", t)
        r.note("a", t)
    with pytest.raises(RequestError, match="per hour"):
        r.check("a", 300)
    r.check("a", 3700)


# -- low voltage --------------------------------------------------------------------------------------

def low_bus(mv=12900, charging=False):
    bus = FakeBus()
    bus.set_word(0x20, mv)
    bus.set_status(charging=charging, vbus=charging)
    return bus


def test_low_voltage_off_by_default_and_persisted(tmp_path):
    lv = LowVoltage(tmp_path / "power.json", trigger=lambda mv: None)
    assert lv.settings == {"enabled": False, "volts": 13.2, "hold_s": 10.0} and lv.first_run
    lv.update(True, 13.0, 15, False, None)
    again = LowVoltage(tmp_path / "power.json", trigger=lambda mv: None)
    assert again.settings == {"enabled": True, "volts": 13.0, "hold_s": 15.0} and not again.first_run
    for v, h in ((10.9, None), (16.1, None), (13.0, 2), (13.0, 500)):
        with pytest.raises(RequestError) as e:
            lv.update(True, v, h, False, None)
        assert e.value.status == 422


def test_low_voltage_triggers_after_hold_not_when_charging_or_stale(tmp_path):
    fired = []
    lv = LowVoltage(None, trigger=fired.append)
    low = {"pack_mv": 12900, "charging": False, "percent": 5}
    lv.check(0, low, False)
    assert not fired                                   # off by default
    lv.update(True, 13.2, 10, True, None)
    lv.check(0, low, False)
    lv.check(9.9, low, False)
    assert not fired and "pack low" in lv.status(9.9)
    lv.check(10, low, False)
    assert fired == [12900]
    lv.check(20, low, True)
    assert fired == [12900]                            # once
    # recovers -> re-armed
    lv.check(30, {"pack_mv": 13500, "charging": False, "percent": 9}, False)
    assert lv.status(30) == "armed"
    # charging, or no fresh reading, never triggers
    for bat in ({"pack_mv": 12000, "charging": True, "percent": 1}, None):
        lv.check(100, bat, False)
        lv.check(200, bat, False)
    assert fired == [12900] and lv.status(200) == "no data: not acting"


def test_low_voltage_rearms_after_cancel(tmp_path):
    fired = []
    lv = LowVoltage(None, trigger=fired.append)
    lv.update(True, 13.2, 10, True, None)
    low = {"pack_mv": 12900, "charging": False, "percent": 5}
    lv.check(0, low, False)
    lv.check(10, low, False)
    assert len(fired) == 1
    lv.check(11, low, False)                           # no longer pending: cancelled
    assert "re-arms" in lv.status(11)
    lv.check(11 + REARM_S, low, False)
    lv.check(11 + REARM_S + 10, low, False)
    assert len(fired) == 2


def test_low_voltage_enable_while_low_needs_confirm(tmp_path):
    lv = LowVoltage(None, trigger=lambda mv: None)
    low = {"pack_mv": 12900, "charging": False, "percent": 5}
    with pytest.raises(RequestError) as e:
        lv.update(True, 13.2, None, False, low)
    assert e.value.status == 409
    assert lv.update(True, 13.2, None, True, low)["enabled"]


def test_node_low_voltage_schedules_a_cancellable_shutdown(tmp_path):
    node, bus, clock, rec = make_node(tmp_path, bus=low_bus())
    node.set_low_voltage(True, 13.2, 10, True, "tester", "local")
    for _ in range(12):
        node.step(clock.t)
        clock.t += 1
    sd = node.state()["shutdown"]
    assert sd["pending"] and sd["source"] == "low_voltage" and sd["delay_s"] == 10 and "low voltage" in sd["reason"]
    node.shutdown.cancel(by="me", client="x")
    for _ in range(30):
        node.step(clock.t)
        clock.t += 1
    assert rec.calls == []                             # backing off (REARM_S) after the cancel
    for _ in range(int(REARM_S) + 30):
        node.step(clock.t)
        clock.t += 1
    assert rec.calls == ["estop", "power_off"]         # still low: re-armed, scheduled, ran


def test_node_low_voltage_not_on_stale_reading(tmp_path):
    node, bus, clock, rec = make_node(tmp_path, bus=low_bus())
    node.set_low_voltage(True, 13.2, 10, True, "tester", "local")
    node.step(clock.t)
    bus.fail = "gone"
    for _ in range(60):
        clock.t += 1
        node.step(clock.t)
    assert not node.state()["shutdown"]["pending"] and rec.calls == []


# -- HTTP API -----------------------------------------------------------------------------------------

@pytest.fixture
def api(tmp_path):
    node, bus, clock, rec = make_node(tmp_path)
    node.step(clock.t)
    app = create_app(node, start=False, identify=lambda ip: f"user@{ip}")
    with TestClient(app) as c:
        yield c, node, clock, rec


def test_meta_and_state(api):
    c, node, _, _ = api
    m = c.get("/meta").json()
    assert (m["kind"], m["actuated"], m["has_page"], m["endpoints"]["estop"]) == ("power", False, True, None)
    st = c.get("/state").json()
    assert {"state", "error", "sources", "low_voltage", "shutdown", "events"} <= set(st)
    assert c.get("/history").json()["source"] == "ups"
    assert c.get("/history?source=nope").status_code == 404
    page = c.get("/")
    assert page.status_code == 200 and 'data-page="power"' in page.text
    assert c.get("/common/viz.js").status_code == 200


def test_api_shutdown_flow_with_identity(api):
    c, node, clock, rec = api
    h = {"X-Forwarded-For": "100.64.0.7"}
    r = c.post("/shutdown", json={"reason": "x", "by": "me"}, headers=h)
    assert r.status_code == 400 and "confirm" in r.json()["detail"]
    r = c.post("/shutdown", headers=h)
    assert r.status_code == 400
    r = c.post("/shutdown", json={"confirm": "shutdown", "reason": "test", "by": "me", "delay_s": 5}, headers=h)
    assert r.status_code == 422
    r = c.post("/shutdown", json={"confirm": "shutdown", "reason": "test", "by": "me", "delay_s": 20}, headers=h)
    assert r.status_code == 202 and r.json()["client"] == "user@100.64.0.7" and r.json()["source"] == "web"
    st = c.get("/state").json()
    assert st["shutdown"]["pending"] and st["state"].startswith("SHUTDOWN in")
    r = c.post("/shutdown", json={"confirm": "shutdown", "reason": "again", "by": "you"},
               headers={"X-Forwarded-For": "100.64.0.8"})
    assert r.status_code == 409
    r = c.post("/shutdown/cancel", json={"by": "you"}, headers={"X-Forwarded-For": "100.64.0.8"})
    assert r.status_code == 200 and r.json()["last"]["cancelled_by"] == "you"
    # the same web client again within a minute: rate-limited
    r = c.post("/shutdown", json={"confirm": "shutdown", "reason": "test", "by": "me"}, headers=h)
    assert r.status_code == 429
    # a forged first hop doesn't change who you are: the proxy's own hop (the last) counts
    r = c.post("/shutdown", json={"confirm": "shutdown", "reason": "test", "by": "me"},
               headers={"X-Forwarded-For": "100.64.0.9, 100.64.0.7"})
    assert r.status_code == 429
    log = c.get("/log").json()["log"]
    assert [x["event"] for x in log] == ["refused", "refused", "refused", "requested", "refused", "cancelled", "refused",
                                                  "refused"]
    assert log[3]["client"] == "user@100.64.0.7"
    assert rec.calls == []


def test_api_shutdown_runs_estop_then_power_off(api):
    c, node, clock, rec = api
    c.post("/shutdown", json={"confirm": "shutdown", "reason": "test", "by": "me", "delay_s": 10})
    clock.t += 10
    node.step(clock.t)
    assert rec.calls == ["estop", "power_off"]
    sd = c.get("/shutdown").json()
    assert sd["last"]["phase"] == "done" and sd["log"][-1]["event"] == "powered_off"


def test_api_low_voltage(api):
    c, node, _, _ = api
    assert c.get("/low_voltage").json()["enabled"] is False
    r = c.post("/low_voltage", json={"enabled": True, "volts": 13.0, "by": "me"})
    assert r.status_code == 200 and r.json()["enabled"] and r.json()["status"] == "armed"
    assert c.post("/low_voltage", json={"enabled": True, "volts": 20}).status_code == 422
    assert c.get("/log").json()["log"][-1]["event"] == "low_voltage"


# -- CLI ----------------------------------------------------------------------------------------------

def test_cli(api, capsys):
    c, node, clock, rec = api
    assert cli.main(["status"], client=c) == 0
    out = capsys.readouterr().out
    assert "UPS 59 %" in out and "tether" in out and "low-voltage shutdown: off" in out
    # not a terminal and no --yes: refused, nothing scheduled
    assert cli.main(["shutdown", "--reason", "r"], client=c, isatty=False) == 2
    # wrong word typed: nothing scheduled
    assert cli.main(["shutdown", "--reason", "r"], client=c, isatty=True, ask=lambda p: "yes") == 1
    assert not node.shutdown.pending
    assert cli.main(["shutdown", "--in", "45", "--reason", "cli test", "--as", "admin"], client=c, isatty=True,
                    ask=lambda p: "shutdown") == 0
    assert "SHUTDOWN in 45 s" in capsys.readouterr().out
    assert node.shutdown.pending["by"] == "admin" and node.shutdown.pending["reason"] == "cli test"
    assert cli.main(["shutdown", "--yes", "--as", "admin"], client=c) == 2      # already pending
    assert cli.main(["cancel", "--as", "admin"], client=c) == 0
    assert cli.main(["cancel", "--as", "admin"], client=c) == 2                 # nothing pending
    assert cli.main(["log"], client=c) == 0
    assert "cancelled by admin" in capsys.readouterr().out
    assert rec.calls == []


def test_power_off_write_is_the_only_write():
    """The real power-off arms 0x01 <- 0x55 and nothing else (checked on the fake bus, never run)."""
    bus = FakeBus()
    UpsHat(bus).arm_power_off()
    assert bus.writes == [(0x2d, REG_POWER_OFF, POWER_OFF_MAGIC)]
    assert sd_mod.CONFIRM == "shutdown"
