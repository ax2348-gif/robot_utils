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
    bus.set_word(0x22, 400 if charging else -800)     # net current: + into the pack
    return bus


def test_low_voltage_off_by_default_and_persisted(tmp_path):
    lv = LowVoltage(tmp_path / "power.json", trigger=lambda mv: None)
    assert lv.settings == {"enabled": False, "volts": 13.2, "hold_s": 10.0, "cell_volts": 3.15} and lv.first_run
    lv.update(True, 13.0, 15, False, None)
    again = LowVoltage(tmp_path / "power.json", trigger=lambda mv: None)
    assert again.settings == {"enabled": True, "volts": 13.0, "hold_s": 15.0, "cell_volts": 3.15} and not again.first_run
    lv.update(True, 13.0, 15, False, None, cell_volts=3.2)
    assert LowVoltage(tmp_path / "power.json", trigger=lambda mv: None).settings["cell_volts"] == 3.2
    with pytest.raises(RequestError):
        lv.update(True, 13.0, 15, False, None, cell_volts=2.0)
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
    assert not fired and "low (pack 12.90 V below 13.20 V" in lv.status(9.9)
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


# -- disk log and watch -------------------------------------------------------------------------------

def logged_node(tmp_path, bus=None, clock=None):
    bus, clock = bus or FakeBus(), clock or Clock()
    node = PowerNode([UpsSource(UpsHat(bus)), TetherSource("main")], estop=Recorder().estop,
                     power_off=Recorder().power_off, clock=clock, settings_path=tmp_path / "power.json",
                     audit_path=tmp_path / "shutdown.jsonl", spawn=lambda fn: fn(), log_dir=tmp_path / "log")
    return node, bus, clock


def test_disk_log_every_poll_with_cells_and_input_and_restores_history(tmp_path):
    import csv
    from robot_utils.nodes.power.powerlog import FIELDS
    node, bus, clock = logged_node(tmp_path)
    for _ in range(3):
        node.step(clock.t)
        clock.t += 1.0
    bus.fail = "Remote I/O error"
    node.step(clock.t)
    node.close()
    files = list((tmp_path / "log").glob("ups-*.csv"))
    assert len(files) == 1
    rows = list(csv.reader(open(files[0])))
    assert rows[0] == FIELDS and len(rows) == 5
    r = dict(zip(FIELDS, rows[1]))
    assert (r["percent"], r["pack_v"], r["cell2_v"], r["current_a"]) == ("59", "15.587", "3.874", "0.512")
    assert (r["input_v"], r["input_a"], r["input_present"], r["charging"]) == ("8.675", "2.090", "1", "1")
    bad = dict(zip(FIELDS, rows[4]))
    assert bad["pack_v"] == "" and "Remote I/O error" in bad["error"]
    again, _, _ = logged_node(tmp_path)          # after a restart the history comes back from the disk log
    assert again.restore_history() == 1           # one row per HISTORY_S: the three good polls were 1 s apart
    assert again.history_rows("ups", 86400)["rows"][0][2:5] == [59, 15.587, 0.512]


def test_disk_log_prunes_old_files_and_survives_write_errors(tmp_path):
    from robot_utils.nodes.power.powerlog import DiskLog
    d = tmp_path / "log"
    d.mkdir()
    (d / "ups-2020-01-01.csv").write_text("old\n")
    (d / "notes.txt").write_text("keep\n")
    dl = DiskLog(d)
    dl.write(1794956581.0, None, "no HAT")
    assert not (d / "ups-2020-01-01.csv").exists() and (d / "notes.txt").exists() and dl.rows == 1
    dl.close()
    blocked = DiskLog(tmp_path / "notes-file")
    (tmp_path / "notes-file").write_text("a file, not a directory")
    blocked.write(1794956581.0, None)           # must not raise into the owner loop
    assert blocked.error and blocked.rows == 0


def test_watch_reports_drain_on_input_low_cell_spread_and_input_loss(tmp_path, caplog):
    node, bus, clock = logged_node(tmp_path)
    bus.set_word(0x22, -300)                     # on input, but net out of the pack
    for i, mv in enumerate([3570, 3316, 3250, 3712]):
        bus.set_word(0x30 + 2 * i, mv)
    with caplog.at_level("INFO", logger="power"):
        for _ in range(12):
            node.step(clock.t)
            clock.t += 1.0
        bus.set_status(charging=False, vbus=False)
        node.step(clock.t)
    msgs = [e["msg"] for e in node.events.list()]
    text = "\n".join(msgs)
    assert text.count("DRAINING while on input") == 1
    assert "cell 3 LOW at 3.25 V" in text and "0.46 V apart" in text and "input power LOST" in text
    assert "DRAINING while on input" in caplog.text            # in the journal too, not only the page


# -- power flow, cells and anomaly spans --------------------------------------------------------------

def test_history_has_power_flow_and_cell_columns(tmp_path):
    node, bus, clock, _ = make_node(tmp_path)
    node.step(clock.t)
    h = node.history_rows("ups", 86400)
    assert h["fields"] == ["t", "wall", "percent", "pack_v", "current_a", "input_present", "charging",
                           "input_w", "battery_w", "load_w", "cell_min_v", "cell_spread_v"]
    assert len(h["units"]) == len(h["fields"]) and "derived" in h["units"][9]
    assert h["limits"] == {"drain_a": -0.02, "cell_low_v": 3.30, "spread_v": 0.20}
    row = dict(zip(h["fields"], h["rows"][0]))
    # the captured reading: input 18.17 W, pack 15.587 V x +0.512 A, cells 3.920 3.874 3.877 3.916
    assert row["input_w"] == 18.17 and row["battery_w"] == 7.98 and row["load_w"] == pytest.approx(10.19)
    assert (row["cell_min_v"], row["cell_spread_v"]) == (3.874, 0.046)


def test_drain_on_input_shows_in_state_source_and_conditions(tmp_path):
    node, bus, clock, _ = make_node(tmp_path)
    node.step(clock.t)
    st = node.state()
    assert st["state"] == "UPS 59 % · charging" and st["sources"][0]["draining_on_input"] is False
    assert st["conditions"] == []
    node.watch.wall = lambda: 1.8e9 + clock.t      # a wall clock that moves with the test clock
    bus.set_word(0x22, -130)                      # on input, the charger bit still set, but net out of the pack
    t_start = clock.t + 1
    for _ in range(12):
        clock.t += 1.0
        node.step(clock.t)
    st = node.state()
    ups = st["sources"][0]
    assert ups["draining_on_input"] is True and ups["charging"] is True and ups["power_w"] < 0
    assert st["state"] == "UPS 59 % · draining on input"
    (c,) = st["conditions"]
    assert (c["key"], c["label"], c["active"], c["end"]) == ("drain", "draining on input", True, None)
    assert "-0.13 A" in c["detail"]
    assert c["start"] == 1.8e9 + t_start          # when it first held, not when its 10 s hold ran out
    clock.t += node_mod.HISTORY_S                 # the next history row
    node.step(clock.t)
    h = node.history_rows("ups", 86400)
    row = dict(zip(h["fields"], h["rows"][-1]))
    assert row["battery_w"] < 0 and row["load_w"] > row["input_w"]
    bus.set_word(0x22, 400)                       # charging again: the span ends
    t_end = clock.t + 1
    for _ in range(12):
        clock.t += 1.0
        node.step(clock.t)
    st = node.state()
    (c,) = st["conditions"]
    assert c["active"] is False and c["end"] == 1.8e9 + t_end and st["state"] == "UPS 59 % · charging"


def test_restore_history_carries_power_flow_and_cells_and_tolerates_missing(tmp_path):
    import csv
    from robot_utils.nodes.power.powerlog import FIELDS
    node, bus, clock = logged_node(tmp_path)
    node.step(clock.t)
    node.close()
    (path,) = (tmp_path / "log").glob("ups-*.csv")
    rows = list(csv.reader(open(path)))
    new = dict(zip(FIELDS, rows[1]), wall=f"{float(rows[1][0]) - 60:.3f}")   # a minute ago: never "in the future"
    old = dict(new, wall=f"{float(rows[1][0]) - 120:.3f}", cell1_v="", cell2_v="", cell3_v="", cell4_v="",
               input_v="", input_a="")         # a row before that lacks cells and input (e.g. hand-edited)
    with open(path, "w", newline="") as f:
        csv.writer(f).writerows([rows[0], [old[k] for k in FIELDS], [new[k] for k in FIELDS]])
    again, _, _ = logged_node(tmp_path)
    assert again.restore_history() == 2
    h = again.history_rows("ups", 86400)
    b, a = (dict(zip(h["fields"], r)) for r in h["rows"])
    assert a["input_w"] == pytest.approx(8.675 * 2.09, abs=0.01) and a["battery_w"] == 7.98
    assert a["load_w"] == pytest.approx(a["input_w"] - 7.98, abs=0.01)
    assert (a["cell_min_v"], a["cell_spread_v"]) == (3.874, 0.046)
    assert (b["input_w"], b["load_w"], b["cell_min_v"], b["cell_spread_v"]) == (None, None, None, None)
    assert b["battery_w"] == 7.98
    c = TestClient(create_app(again, start=False, identify=lambda ip: ip))
    assert c.get("/history").json()["rows"][0][7] is None          # null passes the response model


def test_sim_bus_has_a_drain_on_input_stretch_with_a_sagging_cell():
    from robot_utils.nodes.power.sources import UpsSource as U, draining_on_input
    s = SimBus(clock=Clock(0.0))
    mid = decode(s.regs_at(SimBus.PERIOD_S * 0.44))
    end = decode(s.regs_at(SimBus.PERIOD_S * 0.499))
    assert mid["vbus_present"] and mid["charging"] and mid["current_ma"] < -50 and draining_on_input(mid)
    assert U.readings(mid)["charge_state"] == "charging"               # what the charger bit alone says
    cells = [c / 1000 for c in end["cells_mv"]]
    assert min(cells) < 3.30 and max(cells) - min(cells) > 0.20
    assert not draining_on_input(decode(s.regs_at(100.0)))             # charging


def test_page_has_power_flow_chart_and_anomaly_spans():
    from robot_utils.nodes.power.page import PAGE
    assert 'id="ch-flow"' in PAGE and 'fill: "battery_w"' in PAGE and '"load_w"' in PAGE and '"input_w"' in PAGE
    assert 'id="ch-cell"' in PAGE and 'id="ch-spread"' in PAGE
    assert "spansOf(" in PAGE and 'marks: ["drain"]' in PAGE and "cell_low" in PAGE and "--caution" in PAGE
    assert 'id="flow"' in PAGE and "draining_on_input" in PAGE and "renderConds" in PAGE


def test_low_voltage_acts_on_net_drain_not_the_charging_bit(tmp_path):
    """The undersized-input case (2026-11-20): the charger bit says charging while the pack drains. That must
    trigger (pack or weakest cell low); a net-charging pack never does, even below the thresholds."""
    fired = []
    lv = LowVoltage(None, trigger=fired.append)
    lv.update(True, 13.2, 10, True, None)
    cells = [3420, 3240, 3050, 3640]
    draining_on_input = {"pack_mv": 13350, "charging": True, "percent": 45, "current_ma": -225,
                         "cells_mv": cells, "vbus_present": True}
    assert lv.would_trigger(draining_on_input) == "cell"            # pack 13.35 V is above 13.2; cell 3 is not
    lv.check(0, draining_on_input, False)
    lv.check(10, draining_on_input, False)
    assert fired == [13350]
    assert "cell 3 at 3.05 V below 3.15 V" in lv.reason and "although the charger reports charging" in lv.reason
    pack_low = dict(draining_on_input, pack_mv=13100, cells_mv=[3300, 3270, 3260, 3270])
    assert lv.would_trigger(pack_low) == "pack"
    charging = dict(draining_on_input, pack_mv=12900, current_ma=+190)  # net into the pack: never
    assert lv.would_trigger(charging) is None
    fired.clear()
    lv2 = LowVoltage(None, trigger=fired.append)
    lv2.update(True, 13.2, 10, True, None)
    for t in range(0, 60, 5):
        lv2.check(t, charging, False)
    assert fired == [] and lv2.status(60) == "armed"


def test_node_low_voltage_fires_while_charger_bit_set(tmp_path):
    node, bus, clock, rec = make_node(tmp_path)
    bus.set_status(charging=True, vbus=True)
    bus.set_word(0x22, -225)                           # net draining
    for i, mv in enumerate([3420, 3240, 3050, 3640]):
        bus.set_word(0x30 + 2 * i, mv)
    node.set_low_voltage(True, 13.2, 10, True, "tester", "local")
    for _ in range(12):
        node.step(clock.t)
        clock.t += 1
    sd = node.state()["shutdown"]
    assert sd["pending"] and sd["source"] == "low_voltage" and "cell 3" in sd["reason"]
    lv = node.state()["low_voltage"]
    assert lv["min_cell_v"] == 3.05 and lv["cell_volts"] == 3.15


# -- time until power loss (powerloss.py) ----------------------------------------------------------

from robot_utils.nodes.power import powerloss as pl_mod  # noqa: E402
from robot_utils.nodes.power.powerloss import PowerLoss  # noqa: E402

W0 = 1.8e9
LV_ON = {"enabled": True, "volts": 13.2, "hold_s": 10.0, "cell_volts": 3.15}
LV_OFF = dict(LV_ON, enabled=False)


def ramp(minutes, v3_start, mv_per_min, amps=-0.14, on_input=True, charging=True, t0=W0, step=1.0, others=3.60):
    """1 Hz samples: cell 3 falling linearly, the others steady; (wall, cells, pack, A, input, charging)."""
    out = []
    for k in range(int(minutes * 60 / step)):
        t = t0 + k * step
        v3 = v3_start - mv_per_min / 1000 * (t - t0) / 60
        cells = (others, others - 0.02, v3, others + 0.03)
        out.append((t, cells, sum(cells), amps, on_input, charging))
    return out


def test_power_loss_not_draining_and_too_little_history():
    pl = PowerLoss()
    assert pl.estimate(W0)["status"] == "no_data"
    pl.extend(ramp(20, 3.6, 0, amps=+0.2))                      # charging
    e = pl.estimate(W0 + 20 * 60)
    assert e["status"] == "not_draining" and e["in_min"] is None and e["text"].startswith("charging +")
    pl = PowerLoss()
    pl.extend(ramp(20, 3.6, 0, amps=-0.005))                     # noise around zero
    assert pl.estimate(W0 + 20 * 60)["text"].startswith("steady: net -0.005 A")
    pl = PowerLoss()
    pl.extend(ramp(5, 3.6, 3.0))                                # draining, 5 min of it
    e = pl.estimate(W0 + 5 * 60, LV_ON)
    assert e["status"] == "estimating" and e["in_min"] is None and "estimating" in e["text"]
    pl = PowerLoss()                                            # charged until 5 min ago: only the drain counts
    pl.extend(ramp(20, 3.6, 0, amps=+0.3) + ramp(5, 3.6, 3.0, t0=W0 + 20 * 60))
    assert pl.estimate(W0 + 25 * 60)["status"] == "estimating"
    assert pl.estimate(W0 + 40 * 60)["status"] == "no_data"     # the readings stopped: no stale estimate


def test_power_loss_draining_on_input_cutoff_or_clean_auto_shutdown():
    pl = PowerLoss()
    pl.extend(ramp(20, 3.55, 4.0))       # on input, the charger bit set, cell 3 at 3.47 V falling 4 mV/min
    now = W0 + 20 * 60
    off = pl.estimate(now, LV_OFF)
    assert off["status"] == "draining" and off["on_input"] is True and off["cell"] == 3
    assert off["event"] == "ups_cutoff" and off["clean"] is False and off["blocked_reason"] == "auto-shutdown is off"
    assert off["limit"]["kind"] == "cutoff" and off["limit"]["target_v"] == pl_mod.CELL_CUTOFF_V
    straight = (3.47 - pl_mod.CELL_CUTOFF_V) / 0.004             # a straight line: 105 min
    assert 0.2 * straight < off["in_min"] < 0.5 * straight        # the knee model makes it much shorter
    lo, hi = off["range_min"]
    assert lo <= off["in_min"] <= hi and off["confidence"] == "medium"
    assert off["text"].startswith("power loss in ≈ ") and "UPS cutoff (cell 3 → 3.05 V); auto-shutdown is off" in off["text"]
    assert off["at"] == pytest.approx(now + off["in_min"] * 60, abs=6)
    on = pl.estimate(now, LV_ON)
    assert on["event"] == "auto_shutdown" and on["clean"] is True and on["blocked_reason"] is None
    assert on["limit"] == {"kind": "cell", "cell": 3, "target_v": 3.15, "v": on["limit"]["v"], "label": "cell 3 → 3.15 V"}
    assert on["in_min"] < off["in_min"] and on["cutoff_in_min"] == off["in_min"]
    assert "clean auto-shutdown (cell 3 → 3.15 V)" in on["text"]
    late = pl.estimate(now, dict(LV_ON, cell_volts=3.06, hold_s=120.0))   # threshold too close to the cutoff
    assert late["event"] == "ups_cutoff" and "would come too late" in late["blocked_reason"]


def test_power_loss_on_battery_uses_only_the_battery_minutes():
    pl = PowerLoss()
    pl.extend(ramp(15, 3.80, 0.5, amps=-0.14, others=3.95) +                  # draining on input, then on battery
              ramp(15, 3.79, 8.0, amps=-0.9, on_input=False, charging=False, t0=W0 + 15 * 60, others=3.95))
    e = pl.estimate(W0 + 30 * 60, LV_OFF)
    assert e["status"] == "draining" and e["on_input"] is False and e["window_min"] <= 15
    assert e["slope_mv_min"] == pytest.approx(8.0, rel=0.05)
    assert e["cell"] == 3 and e["confidence"] == "low"                       # 3.67 V: still on the plateau
    assert 10 < e["in_min"] < (3.67 - 3.05) / 0.008


def test_power_loss_ignores_load_spikes():
    base = ramp(20, 3.50, 4.0)
    spiky = [(t, tuple(c - 0.15 for c in cells), p - 0.6, -1.0, i, ch) if k % 97 == 0 else (t, cells, p, a, i, ch)
             for k, (t, cells, p, a, i, ch) in enumerate(base)]
    a, b = PowerLoss(), PowerLoss()
    a.extend(base)
    b.extend(spiky)
    now = W0 + 20 * 60
    assert b.estimate(now)["in_min"] == pytest.approx(a.estimate(now)["in_min"], rel=0.05)


def load_cutoff_fixture():
    """The 2026-11-20 log around the UPS cutoff at 18:21:25 (cell 3 at 3.05 V, input present, charger bit set):
    one row per 30 s, 35 min before 6 h and 3 h ahead, and the last 95 min (tests/data, from the 1 Hz power log)."""
    import csv
    import gzip
    from pathlib import Path
    with gzip.open(Path(__file__).parent / "data" / "ups-2026-11-20-cutoff.csv.gz", "rt", newline="") as f:
        return [pl_mod.sample_of_log(r) for r in csv.DictReader(f)]


CUTOFF_WALL = 1795216885.769     # last reading before the UPS cut power


@pytest.mark.parametrize("before_min", [360, 180, 60, 30, 10, 5])
def test_power_loss_backtest_2026_11_20(before_min):
    """Replay the estimator up to ``before_min`` before the real cutoff. Near the end (the last hour) it must be
    on the short side, within 0.4-1.0x of the real time, with the real time inside its range (5 % slack: the 30 s
    fixture puts the top of the range at 59.1 min an hour out, 59.4 at 1 Hz); earlier, on the
    plateau, it can be hours off and must say so (confidence low)."""
    now = CUTOFF_WALL - before_min * 60
    pl = PowerLoss()
    pl.extend(s for s in load_cutoff_fixture() if s[0] <= now)
    e = pl.estimate(now, LV_OFF)
    assert e["status"] == "draining" and e["event"] == "ups_cutoff" and e["on_input"] is True
    if before_min >= 90:
        assert e["confidence"] == "low" and "too long" in e["note"]
        return
    assert e["cell"] == 3 and e["confidence"] == "medium"
    assert 0.4 * before_min <= e["in_min"] <= 1.0 * before_min
    assert e["range_min"][0] <= before_min <= e["range_min"][1] * 1.05
    on = pl.estimate(now, LV_ON)                 # with the auto-shutdown on, a clean shutdown comes first
    assert on["event"] == "auto_shutdown" and on["limit"]["kind"] == "cell" and on["in_min"] <= e["in_min"]


def test_node_state_has_power_loss_and_the_tile_says_it(tmp_path):
    import time as _time
    node, bus, clock, _ = make_node(tmp_path)
    bus.set_word(0x22, -140)                     # on input, the charger bit set, net draining
    for i, mv in enumerate([3600, 3580, 3470, 3630]):
        bus.set_word(0x30 + 2 * i, mv)
    wall = _time.time()
    node.loss.extend(ramp(20, 3.55, 4.0, t0=wall - 20 * 60 - 1))
    node.step(clock.t)
    st = node.state()
    pl = st["sources"][0]["power_loss"]
    assert pl["status"] == "draining" and pl["event"] == "ups_cutoff" and pl["blocked_reason"] == "auto-shutdown is off"
    assert st["state"].startswith("UPS 59 % · ") and " · power cut in ≈ " in st["state"]
    assert "power_loss" not in st["sources"][1]                      # the tether has none
    node.set_low_voltage(True, 13.2, 10, True, "tester", "local")
    clock.t += 1                                 # the next poll refreshes the estimate
    node._last_est = -1e18
    node.step(clock.t)
    pl = node.state()["sources"][0]["power_loss"]
    assert pl["event"] == "auto_shutdown" and pl["clean"] is True and " · auto-shutdown in ≈ " in node.state()["state"]
    node.shutdown.request(confirm="shutdown", reason="test", by="t", client="c", source="web", delay_s=30)
    pl = node.state()["sources"][0]["power_loss"]
    assert pl["status"] == "shutdown_pending" and pl["event"] == "shutdown" and pl["in_min"] == 0.5
    assert pl["text"] == "shutdown in 30 s (countdown)"


def test_page_shows_power_loss():
    from robot_utils.nodes.power.page import PAGE
    assert 'id="loss"' in PAGE and "renderLoss(" in PAGE and "power_loss" in PAGE and "lim.loss_v" in PAGE


def test_power_loss_slow_drain_is_never_not_draining():
    """2026-11-20 21:30: a steady -0.03 A (-0.4 W) drain on input read 'not draining' under a -0.05 A threshold
    while the page showed the pack at -0.5 W. One -0.02 A threshold everywhere; a drain too slow to project says so."""
    from robot_utils.nodes.power import sources
    assert pl_mod.DRAIN_A == sources.DRAIN_A == -0.02
    pl = PowerLoss()
    pl.extend(ramp(20, 3.60, 0.0, amps=-0.03))                   # draining, cell not (yet) visibly falling
    e = pl.estimate(W0 + 20 * 60, LV_ON)
    assert e["status"] == "estimating" and "draining 0." in e["text"] and "too slowly to project" in e["text"]
    pl = PowerLoss()
    pl.extend(ramp(20, 3.50, 1.0, amps=-0.03))                   # the same drain, cell 3 falling 1 mV/min
    e = pl.estimate(W0 + 20 * 60, LV_ON)
    assert e["status"] == "draining" and e["in_min"] is not None
