"""Drive-encoder rebase on the swerve node: simulated SPARKs, no hardware."""
import math
import socket
import threading
import time

import httpx
import pytest
import uvicorn
from sparkcan import protocol

from robot_utils.nodes.common.lease import RequestError
from robot_utils.nodes.swerve import constants as c
from robot_utils.nodes.swerve import node as swerve_node
from robot_utils.nodes.swerve.client import NodeError, SwerveClient
from robot_utils.nodes.swerve.node import FIELDS, SwerveNode
from robot_utils.nodes.swerve.server import create_app
from robot_utils.nodes.swerve.sim import start_sim, stop_sim

F = {n: i for i, n in enumerate(FIELDS)}
DRIVES = [m.drive_id for m in c.MODULES]


def wait_for(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.01)
    raise AssertionError("condition not met in time")


@pytest.fixture
def base():
    """(node, sims), the drive running, auto rebase off and no rebase yet."""
    sims, iface = start_sim()
    node = SwerveNode(iface)
    node.auto_rebase["enabled"] = False      # before the first tick: no rebase at start
    node.start()
    assert node.status == "running"
    yield node, sims
    node.close()
    stop_sim(sims, iface)


def set_travel(sims, rotations):
    for i in DRIVES:
        with sims[i]._lock:
            sims[i].position = rotations


class Driver:
    """Holds a twist on the node (lease + enable + 20 Hz commands) until stopped."""

    def __init__(self, node, twist):
        self.node, self.twist = node, twist
        self.token, _ = node.acquire("test")
        node.enable(self.token)
        self.stop = threading.Event()
        self.t = threading.Thread(target=self._run, daemon=True)
        self.t.start()

    def _run(self):
        while not self.stop.wait(0.05):
            self.node.command(self.token, self.twist)

    def halt(self):
        self.stop.set()
        self.t.join()
        self.node.command(self.token, (0.0, 0.0, 0.0))     # enabled, holding zero


def test_rebase_confirms_then_epoch_increments(base):
    node, sims = base
    set_travel(sims, 250.0)                                  # far from 0, like after a long drive
    wait_for(lambda: node.encoders()["stationary_s"] and node.encoders()["stationary_s"] > 0.6)
    assert node.encoders()["max_abs_drive_pos"] > 1.0
    out = node.rebase()
    assert out["epoch"] == 1 and out["last_error"] is None and not out["rebasing"]
    assert out["max_abs_drive_pos"] <= swerve_node.REBASE_TOL
    rows = node.hub.recent(200, "tick")
    pos = rows[:, [F[f"{m.name}_drive_pos"] for m in c.MODULES]]
    epoch, rebasing = rows[:, F["enc_epoch"]], rows[:, F["enc_rebasing"]]
    first = int((epoch == 1).argmax())
    assert (abs(pos[first]) <= swerve_node.REBASE_TOL).all()         # confirmed by the readings
    assert (abs(pos[:first][epoch[:first] == 0]) > 1.0).any()          # old travel before it
    assert rebasing[first - 1] == 1 and rebasing[first] == 0           # in flight, then done
    assert rows[first, F["rebase_age"]] == pytest.approx(0.0, abs=0.02)


def test_epoch_does_not_increment_without_confirmation(base, monkeypatch):
    node, sims = base
    set_travel(sims, 250.0)
    deaf = sims[DRIVES[2]]
    orig = deaf._handle

    def drop_rebase(frame):
        hit = protocol.lookup(frame.arb_id)
        if hit and hit[0].name == "SET_PRIMARY_ENCODER_POSITION":
            return                                           # this SPARK never applies it
        orig(frame)
    deaf._handle = drop_rebase
    wait_for(lambda: (node.encoders()["stationary_s"] or 0) > 0.6)
    with pytest.raises(RequestError) as e:
        node.rebase()
    assert e.value.status == 503 and "not confirmed" in str(e.value)
    enc = node.encoders()
    assert enc["epoch"] == 0 and enc["last_rebase_t"] is None and "not confirmed" in enc["last_error"]
    rows = node.hub.recent(100, "tick")
    assert (rows[:, F["enc_epoch"]] == 0).all() and rows[:, F["enc_rebasing"]].max() == 1
    assert rows[-1, F["enc_rebasing"]] == 0


def test_rebase_refused_while_moving_allowed_when_stopped(base):
    node, sims = base
    d = Driver(node, (0.3, 0.0, 0.0))
    try:
        wait_for(lambda: abs(dict(zip(FIELDS, node.hub.recent(1, "tick")[0]))["FL_drive_vel"]) > 0.1)
        with pytest.raises(RequestError) as e:
            node.rebase()
        assert e.value.status == 409 and "not stationary" in str(e.value)
        assert node.encoders()["motion"]
    finally:
        d.halt()
    # enabled and holding zero: fine once the wheels have been still for the dwell
    wait_for(lambda: (node.encoders()["stationary_s"] or 0) > swerve_node.REBASE_STILL_S)
    assert node.enabled and node.rebase()["epoch"] == 1


def test_command_held_at_zero_while_a_rebase_is_in_flight(base):
    node, sims = base
    set_travel(sims, 250.0)
    deaf = sims[DRIVES[0]]
    orig = deaf._handle
    deaf._handle = lambda f: None if (h := protocol.lookup(f.arb_id)) and h[0].name == "SET_PRIMARY_ENCODER_POSITION" else orig(f)
    wait_for(lambda: (node.encoders()["stationary_s"] or 0) > 0.6)
    token, _ = node.acquire("test")
    node.enable(token)
    threading.Thread(target=lambda: pytest.raises(RequestError, node.rebase), daemon=True).start()
    wait_for(lambda: node.encoders()["rebasing"])
    node.command(token, (0.3, 0.0, 0.0))
    time.sleep(0.05)
    row = dict(zip(FIELDS, node.hub.recent(1, "tick")[0]))
    assert row["req_vx"] == 0.3 and row["cmd_vx"] == 0.0 and row["enc_rebasing"] == 1
    wait_for(lambda: node.encoders()["last_error"])            # times out; then the command goes through
    node.command(token, (0.3, 0.0, 0.0))
    wait_for(lambda: dict(zip(FIELDS, node.hub.recent(1, "tick")[0]))["cmd_vx"] == 0.3)


def test_auto_rebase_at_start_and_after_the_interval_only_when_still():
    sims, iface = start_sim()
    node = SwerveNode(iface)
    node.auto_rebase.update(interval_s=1.0, still_s=0.5)
    node.start()
    try:
        wait_for(lambda: node.encoders()["epoch"] == 1, 3)    # at start, once stationary
        d = Driver(node, (0.3, 0.0, 0.0))
        try:
            time.sleep(2.0)                                  # interval passes while moving
            assert node.encoders()["epoch"] == 1
        finally:
            d.halt()
        t_stop = time.monotonic()
        wait_for(lambda: node.encoders()["epoch"] == 2, 3)
        assert time.monotonic() - t_stop >= 0.5               # waited for still_s
        assert node.encoders()["last_rebase_t"] > t_stop
        time.sleep(0.5)
        assert node.encoders()["epoch"] == 2                  # not again before the interval
        node.set_auto_rebase(enabled=False)
        time.sleep(1.5)
        assert node.encoders()["epoch"] == 2
    finally:
        node.close()
        stop_sim(sims, iface)


def test_http_endpoints():
    sims, iface = start_sim()
    node = SwerveNode(iface)
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    server = uvicorn.Server(uvicorn.Config(create_app(node), host="127.0.0.1", port=port, log_level="warning"))
    t = threading.Thread(target=server.run, daemon=True)
    t.start()
    try:
        wait_for(lambda: server.started and node.status == "running", 10)
        with SwerveClient(f"http://127.0.0.1:{port}") as a:
            auto = a.get("/encoders/auto")
            assert auto == {"enabled": True, "interval_s": swerve_node.AUTO_INTERVAL_S,
                            "still_s": swerve_node.AUTO_STILL_S}
            wait_for(lambda: a.encoders()["epoch"] == 1)       # the rebase at start
            assert a.set_auto_rebase(interval_s=300)["interval_s"] == 300.0
            assert a.get("/encoders/auto")["enabled"] is True
            with pytest.raises(NodeError) as e:
                a.post("/encoders/auto", {"still_s": 0.1})
            assert e.value.status == 422
            r = httpx.post(f"http://127.0.0.1:{port}/encoders/auto", content=b'{"interval_s": 1e400}',
                           headers={"content-type": "application/json"})   # JSON 1e400 = inf
            assert r.status_code == 422                        # (it used to switch auto off)
            with pytest.raises(ValueError):
                node.set_auto_rebase(interval_s=math.inf)
            assert a.get("/encoders/auto")["interval_s"] == 300.0
            out = a.rebase_encoders()                          # anyone, no lease
            assert out["epoch"] == 2 and a.state()["encoders"]["epoch"] == 2
            assert a.state()["latest"]["enc_epoch"] == 2
            with SwerveClient(f"http://127.0.0.1:{port}") as b, b.control("driver"):
                b.command(0.3, 0, 0)
                time.sleep(0.1)
                with pytest.raises(NodeError) as e:
                    a.rebase_encoders()
                assert e.value.status == 409 and "not stationary" in e.value.detail
    finally:
        server.should_exit = True
        t.join(10)
        stop_sim(sims, iface)
