"""The swerve node end to end: simulated SPARKs, real uvicorn server, the Python client."""
import math
import socket
import threading
import time

import pytest
import uvicorn

from helpers import waiter
from robot_utils.nodes.swerve import constants as c
from robot_utils.nodes.swerve.client import NodeError, SwerveClient
from robot_utils.nodes.swerve.node import FIELDS, HARD_MAX_OMEGA, HARD_MAX_SPEED, CommandSlot, SwerveNode, clamp
from robot_utils.nodes.swerve.server import create_app
from robot_utils.nodes.swerve.sim import start_sim, stop_sim


wait_for = waiter(2.0, 0.01)


@pytest.fixture
def node_url():
    sims, iface = start_sim()
    node = SwerveNode(iface)
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    server = uvicorn.Server(uvicorn.Config(create_app(node), host="127.0.0.1", port=port, log_level="warning"))
    t = threading.Thread(target=server.run, daemon=True)
    t.start()
    wait_for(lambda: server.started, 10)
    wait_for(lambda: node.status == "running", 10)
    yield f"http://127.0.0.1:{port}", node, sims
    server.should_exit = True
    t.join(10)
    stop_sim(sims, iface)


def latest(node):
    return dict(zip(FIELDS, node.hub.recent(1, "tick")[0]))


def test_clamp_keeps_direction():
    (vx, vy, w), clamped = clamp((3.0, 4.0, -9.0), 1.0, 2.0)
    assert clamped and vx == pytest.approx(0.6) and vy == pytest.approx(0.8) and w == -2.0
    assert clamp((0.1, 0, 0), 1.0, 2.0) == ((0.1, 0, 0), False)


def test_command_slot_keeps_the_latest_write():
    slot = CommandSlot()
    slot.write((1, 0, 0), 1, "slow")
    slot.write((2, 0, 0), 0, "fast")
    assert slot.read().twist == (2.0, 0.0, 0.0) and slot.read().source == "fast"
    slot.clear()
    assert slot.read() is None


def test_lease_enable_command_drives_and_is_echoed(node_url):
    url, node, sims = node_url
    with SwerveClient(url) as a, SwerveClient(url) as b:
        with pytest.raises(NodeError) as e:
            a.command(0.2, 0, 0)
        assert e.value.status == 401
        a.acquire("alice")
        with pytest.raises(NodeError) as e:
            b.acquire("bob")
        assert e.value.status == 409
        with pytest.raises(NodeError) as e:
            a.command(0.2, 0, 0)                 # not enabled yet
        assert e.value.status == 409
        a.enable()
        for _ in range(10):
            a.command(0.2, 0.0, 0.0, seq=42)
            time.sleep(0.02)
        wait_for(lambda: all(s.enabled for s in sims.values()))
        row = latest(node)
        assert row["req_vx"] == pytest.approx(0.2) and row["cmd_vx"] == pytest.approx(0.2)
        assert row["cmd_seq"] == 42 and row["cmd_source"] == 1 and row["motors_enabled"] == 1
        assert row["lease_id"] == node.lease.info.lease_id
        assert sims[c.MODULES[0].drive_id].setpoint[1] > 0
        assert b.get("/state")["lease"] == {"held": True, "holder": "alice", "lease_id": 1, "expires_in": pytest.approx(1.0, abs=0.3)}


def test_steer_only_holds_the_wheels_on_an_axis_with_the_drives_off(node_url):
    """Steer-only: steering enabled and set the way each wheel would roll for the unit twist (either direction
    along that line), drive motors left disabled (they coast); the stream shows the twist; a twist drives again."""
    url, node, sims = node_url
    with SwerveClient(url) as a:
        a.acquire("aligner")
        a.enable()
        for steer, want in (((math.cos(0.5), math.sin(0.5), 0.0), lambda m: 0.5),          # a translation: all at 0.5 rad
                            ((0.0, 0.0, 1.0), lambda m: math.atan2(m.x, -m.y))):           # a turn in place: tangent
            for _ in range(60):
                a.steer(*steer)
                time.sleep(0.02)
            wait_for(lambda: all(sims[m.steer_id].enabled for m in c.MODULES))
            assert not any(sims[m.drive_id].enabled for m in c.MODULES)
            row = latest(node)
            assert [row["steer_vx"], row["steer_vy"], row["steer_omega"]] == pytest.approx(list(steer))
            for m in c.MODULES:
                sp, w = row[f"{m.name}_steer_sp"], want(m)
                assert min(abs(math.remainder(sp - w, 2 * math.pi)), abs(math.remainder(sp - w - math.pi, 2 * math.pi))) < 1e-6
                assert math.isnan(row[f"{m.name}_drive_sp"])
        for bad in ({"twist": [0.1, 0, 0], "steer": [1, 0, 0]}, {"steer": [0, 0, 0]}):
            with pytest.raises(NodeError) as e:
                a.post("/command", bad)
            assert e.value.status == 422
        for _ in range(10):
            a.command(0.2, 0.0, 0.0)
            time.sleep(0.02)
        wait_for(lambda: all(sims[m.drive_id].enabled for m in c.MODULES))
        assert math.isnan(latest(node)["steer_vx"])


def test_soft_limits_clamp_and_reset_with_the_lease(node_url):
    url, node, _ = node_url
    with SwerveClient(url) as a:
        a.acquire()
        with pytest.raises(NodeError) as e:
            a.set_limits(max_speed=HARD_MAX_SPEED + 1)
        assert e.value.status == 422
        a.set_limits(max_speed=0.1, max_omega=0.5)
        a.enable()
        out = a.command(0.6, 0.8, 2.0)
        assert out["clamped"] and out["applied"] == pytest.approx([0.06, 0.08, 0.5])
        wait_for(lambda: latest(node)["cmd_vx"] == pytest.approx(0.06))
        assert latest(node)["soft_max_speed"] == 0.1
        a.release()
        assert a.get("/limits")["soft"] == {"max_speed": HARD_MAX_SPEED, "max_omega": HARD_MAX_OMEGA}


def test_command_times_out_after_quarter_second(node_url):
    url, node, sims = node_url
    with SwerveClient(url) as a:
        a.acquire()
        a.enable()
        a.command(0.2, 0, 0)
        wait_for(lambda: latest(node)["motors_enabled"] == 1)
        time.sleep(c.COMMAND_TIMEOUT + 0.1)
        assert latest(node)["motors_enabled"] == 0 and math.isnan(latest(node)["cmd_vx"])
        wait_for(lambda: not any(s.enabled for s in sims.values()))
        a.renew()                                # lease and enable survive the timeout
        a.command(0.1, 0, 0)
        wait_for(lambda: latest(node)["motors_enabled"] == 1)


def test_lease_lapse_stops_and_disables(node_url):
    url, node, _ = node_url
    with SwerveClient(url) as a:
        a.acquire()
        a.enable()
        a.command(0.2, 0, 0)
        time.sleep(node.lease.heartbeat + 0.3)   # no renew, no command
        st = a.state()
        assert st["lease"]["held"] is False and st["lease"]["holder"] is None and not st["enabled"]
        assert any("lapsed" in e["msg"] for e in st["events"])
        with pytest.raises(NodeError) as e:
            a.command(0.2, 0, 0)
        assert e.value.status == 403


def test_estop_latches_for_anyone_and_needs_holder_to_clear(node_url):
    url, node, sims = node_url
    with SwerveClient(url) as a, SwerveClient(url) as viewer:
        a.acquire()
        a.enable()
        a.command(0.2, 0, 0)
        wait_for(lambda: latest(node)["motors_enabled"] == 1)
        assert viewer.post("/estop") == {"latched": True, "reason": "e-stop", "source": "127.0.0.1"}   # no body, no lease
        assert viewer.get("/state")["estop"]["latched"] is True
        wait_for(lambda: latest(node)["motors_enabled"] == 0 and latest(node)["estop"] == 1)
        for call in (lambda: a.command(0.2, 0, 0), a.enable):
            with pytest.raises(NodeError) as e:
                call()
            assert e.value.status == 409
        with pytest.raises(NodeError) as e:
            viewer.clear_estop()
        assert e.value.status == 401
        a.clear_estop()
        with pytest.raises(NodeError):
            a.command(0.2, 0, 0)                 # cleared but still disabled
        a.enable()
        a.command(0.2, 0, 0)
        wait_for(lambda: latest(node)["motors_enabled"] == 1)


def test_stream_framing_every_tick(node_url):
    url, node, _ = node_url
    with SwerveClient(url) as a:
        est = a.sync()
        assert est.uncertainty < 0.01
        got, t0 = [], time.monotonic()
        for header, rows in a.rows("/stream"):
            got.append(rows)
            if time.monotonic() - t0 > 1.0:
                break
        n = sum(len(r) for r in got)
        assert 80 <= n <= 120                     # 100 Hz
        ticks = [t for r in got for t in r["tick"]]
        assert ticks == list(range(int(ticks[0]), int(ticks[0]) + len(ticks)))   # none missing
        assert set(got[0].dtype.names) == set(FIELDS)
        assert header["kind"] == "tick"
        assert a.get("/fields")["fields"] == FIELDS


def test_status_period_is_set_live_and_kept_across_a_reset(node_url):
    url, node, sims = node_url
    from sparkcan._params import SparkParameter as P
    drives, steers = [sims[m.drive_id] for m in c.MODULES], [sims[m.steer_id] for m in c.MODULES]

    def periods():   # the per-tick frames as the simulated SPARKs hold them (STATUS_0 and the signal frame)
        return ({s.params[int(P.STATUS_0_PERIOD)] for s in drives + steers},
                {s.params[int(P.STATUS_2_PERIOD)] for s in drives}, {s.params[int(P.STATUS_5_PERIOD)] for s in steers})
    with SwerveClient(url) as a:
        assert a.status_period() == {"status_period_ms": 10, "applied_ms": 10}
        assert a.state()["status_period_ms"] == 10 and periods() == ({10}, {10}, {10})
        a.set_status_period(20)                                  # anyone: no lease
        wait_for(lambda: a.status_period()["applied_ms"] == 20)
        wait_for(lambda: periods() == ({20}, {20}, {20}))
        assert a.state()["status_period_ms"] == 20
        assert any("status frames every 20 ms" in e["msg"] for e in a.state()["events"])
        for bad in (4, 51, 12.5):
            with pytest.raises(NodeError) as e:
                a.post("/status_period", {"status_period_ms": bad})
            assert e.value.status == 422
        a.acquire()
        a.reset()                                                # a rebuilt drive keeps the setting
        wait_for(lambda: a.status_period() == {"status_period_ms": 20, "applied_ms": 20})
        wait_for(lambda: periods() == ({20}, {20}, {20}))
        a.enable()
        a.command(0.1, 0, 0)                                     # and still drives
        wait_for(lambda: latest(node)["motors_enabled"] == 1)


def test_status_period_start_value_and_range():
    from robot_utils.nodes.swerve.drive import check_status_period
    assert SwerveNode("virtual:unused", status_period_ms=25).status_period() == {"status_period_ms": 25, "applied_ms": None}
    for bad in (4, 51, 12.5, True, "20"):
        with pytest.raises(ValueError):
            check_status_period(bad)


def test_reset_rebuilds_a_crashed_drive(node_url):
    url, node, _ = node_url

    def boom(drive):
        raise RuntimeError("boom")
    node.drive.pre_tick = boom
    wait_for(lambda: node.status == "error")
    with SwerveClient(url) as a:
        a.acquire()
        with pytest.raises(NodeError) as e:
            a.enable()
        assert e.value.status == 503
        assert "boom" in a.state()["error"]
        assert a.reset()["state"] == "running"
        a.enable()
        a.command(0.1, 0, 0)
        wait_for(lambda: latest(node)["motors_enabled"] == 1)


def test_meta_toolkit_and_page_use_relative_urls(node_url):
    import re
    import httpx
    url, node, _ = node_url
    meta = httpx.get(url + "/meta").json()
    assert meta == {"name": "swerve", "title": "Swerve base", "version": meta["version"], "kind": "swerve",
                    "has_page": True, "page": "./", "actuated": True,
                    "endpoints": {"health": "health", "state": "state", "events": "events", "estop": "estop"},
                    "rate_hz": 100}
    js = httpx.get(url + "/common/ui.js")
    assert js.status_code == 200 and "javascript" in js.headers["content-type"] and "RU.leaseWidget" in js.text
    assert httpx.get(url + "/common/ui.css").status_code == 200
    page = httpx.get(url + "/").text
    assert 'src="common/ui.js"' in page
    assert not re.search(r'''(?:src|href)="/|RU\.(?:api|sse|rows|stream)\(\s*["'`]/|call\(\s*["'`]/''', page)
    st = httpx.get(url + "/state").json()
    assert isinstance(st["state"], str) and "error" in st
    assert st["lease"]["held"] is False and st["estop"]["latched"] is False
    assert httpx.get(url + "/health").json()["ok"] is True


def test_cross_origin_estop_and_preflight(node_url):
    import httpx
    url, node, _ = node_url
    origin = {"Origin": "http://frontend.example:80"}
    pre = httpx.options(url + "/estop", headers={**origin, "Access-Control-Request-Method": "POST",
                                                  "Access-Control-Request-Headers": "content-type"})
    assert pre.status_code == 200 and pre.headers["access-control-allow-origin"] in ("*", origin["Origin"])
    r = httpx.post(url + "/estop", headers=origin)      # no body, no lease
    assert r.status_code == 200 and r.json()["latched"] is True
    assert r.headers["access-control-allow-origin"] in ("*", origin["Origin"])
    for path in ("/meta", "/state", "/health"):
        g = httpx.get(url + path, headers=origin)
        assert g.headers["access-control-allow-origin"] in ("*", origin["Origin"])
    t = time.monotonic()
    httpx.get(url + "/state")
    assert time.monotonic() - t < 0.2                    # the dashboard polls with a 0.9 s timeout


def test_changing_the_status_period_never_stalls_the_control_loop(node_url):
    """POST /status_period used to apply 16 confirmed SPARK writes on the loop thread, whose frames are batched
    until the tick ends: every write timed out (50 ms), so the loop and its heartbeats stopped for ~0.8 s (code
    review 2026-11-21). The writes now run beside the loop."""
    url, node, _ = node_url
    time.sleep(0.3)
    t0 = time.monotonic()
    with SwerveClient(url) as c:
        out = c.post("/status_period", {"status_period_ms": 20})
    assert out["status_period_ms"] == 20
    wait_for(lambda: node.status_period()["applied_ms"] == 20, 3)
    time.sleep(0.3)
    rows = [r for r in node.hub.recent(200, "tick") if r[0] >= t0 - 0.05]
    gaps = [b[0] - a[0] for a, b in zip(rows, rows[1:])]
    assert gaps and max(gaps) < 0.05, f"the loop stalled {max(gaps) * 1e3:.0f} ms"
    assert "error" not in node.status_period()
