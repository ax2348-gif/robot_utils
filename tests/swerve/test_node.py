"""The swerve node end to end: simulated SPARKs, real uvicorn server, the Python client."""
import math
import socket
import threading
import time

import pytest
import uvicorn

from robot_utils.nodes.swerve import constants as c
from robot_utils.nodes.swerve.client import NodeError, SwerveClient
from robot_utils.nodes.swerve.node import FIELDS, HARD_MAX_OMEGA, HARD_MAX_SPEED, CommandSlot, SwerveNode, clamp
from robot_utils.nodes.swerve.server import create_app
from robot_utils.nodes.swerve.sim import start_sim, stop_sim


def wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.01)
    raise AssertionError("condition not met in time")


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
    return dict(zip(FIELDS, node.hub.recent(1)[0]))


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
        assert a.get("/fields")["fields"] == FIELDS


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
