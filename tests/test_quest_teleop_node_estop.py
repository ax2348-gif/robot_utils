"""The quest_teleop node's stop paths: the e-stop goes to the arm, the swerve and the WBC directly and in parallel (a
slow or dead one never delays the others), then teleop stops; Start is not live (nothing but disarmed keep-alives
until Y); X disables the WBC and the node keeps the session open (its watchdog does not stop teleop); the page /
state / stream carry armed, latency and the headset battery. Real sockets: fake nodes (FastAPI), the fake WBC
(tests/quest_teleop_fakes.py) and the fake headset."""
import os
import threading
import time
from dataclasses import replace

import pytest
from fastapi import FastAPI

from helpers import serve, stop, wait_for
from quest_teleop_fakes import FakeWbc
from robot_utils.nodes.quest_teleop.node import FIELDS, QuestTeleopNode

from quest_teleop.fake_headset import FakeHeadsetServer, Headset, HeadsetState  # noqa: E402 (tools/, via the adapter)

IDLE = HeadsetState()


def estop_app(name, seen, delay=0.0):
    app = FastAPI()

    @app.post("/estop")
    def estop(body: dict):
        time.sleep(delay)
        seen[name] = time.monotonic()
        return {"latched": True, "reason": body.get("reason")}
    return app


@pytest.fixture
def nodes():
    seen = {}
    servers = {n: serve(estop_app(n, seen, 0.7 if n == "arm" else 0.0)) for n in ("arm", "swerve", "wbc")}
    yield seen, {n: s[0] for n, s in servers.items()}
    stop(*[(s[1], s[2]) for s in servers.values()])


def test_estop_goes_to_all_three_in_parallel(nodes):
    seen, urls = nodes
    node = QuestTeleopNode(urls["wbc"], arm=urls["arm"], swerve=urls["swerve"], quest_host="127.0.0.1",
                           quest_port=1, ar="off")
    t0 = time.monotonic()
    out = node.estop("test", "pytest")
    assert seen["swerve"] - t0 < 0.4 and seen["wbc"] - t0 < 0.4      # not behind the slow arm (0.7 s)
    assert seen["arm"] - t0 >= 0.7
    assert out["latched"] and all(out["targets"][n]["latched"] for n in ("arm", "swerve", "wbc"))
    assert out["forwarded"] is True and "error" not in out


def test_a_dead_target_does_not_stop_the_others(nodes):
    seen, urls = nodes
    node = QuestTeleopNode(urls["wbc"], arm=urls["arm"], swerve="http://127.0.0.1:9", quest_host="127.0.0.1",
                           quest_port=1, ar="off")
    out = node.estop("test", "pytest")
    assert out["latched"] and out["targets"]["arm"]["latched"] and out["targets"]["wbc"]["latched"]
    assert not out["targets"]["swerve"]["latched"] and "swerve" in out["error"]
    assert any("e-stop to the swerve: FAILED" in e["msg"] for e in node.events.list())


# -- the node with the fake WBC and the fake headset ---------------------------------------------------------

@pytest.fixture
def env(tmp_path, tmp_sock_dir, monkeypatch):
    monkeypatch.chdir(tmp_path)
    fake = FakeWbc(unix_path=os.path.join(tmp_sock_dir, "wbc.e.sock"))
    wurl, wsrv, wth = serve(fake.app)
    headset = FakeHeadsetServer()
    node = QuestTeleopNode(wurl, quest_host="127.0.0.1", quest_port=headset.port, ar="off")
    node.start()
    stop_ev = threading.Event()
    state = {"s": IDLE}
    hs = Headset()

    def play():
        while not stop_ev.is_set():
            f = hs.frame(state["s"], time.monotonic())
            f["device"] = {"battery_pct": 87, "charging": False, "plugged": True, "temperature_c": 35.2}
            headset.send(f)
            time.sleep(1 / 72)
    th = threading.Thread(target=play, daemon=True)
    th.start()
    wait_for(lambda: headset.n_clients > 0 and node.monitor.frames > 10, 5)
    yield {"fake": fake, "node": node, "state": state}
    stop_ev.set()
    th.join(1)
    node.close()
    stop((wsrv, wth))
    headset.close()
    fake.close()


def test_start_is_not_live_and_x_disables_without_stopping_teleop(env):
    fake, node, state = env["fake"], env["node"], env["state"]
    node.start_teleop()
    grip = replace(IDLE, right=replace(IDLE.right, grip=1.0, pos=(0.5, -0.2, -0.3)))
    state["s"] = grip                                           # squeeze and move: nothing is commanded
    time.sleep(0.8)
    dg = [d for _, d in fake.datagrams]
    assert dg and all(d["type"] == "alive" and d["armed"] is False for d in dg)
    st = node.state()
    assert st["teleop"]["armed"] is False and st["teleop"]["input"]["fresh"] is True
    assert st["teleop"]["input"]["latency_ms"] is not None and st["teleop"]["input"]["rate_hz"] > 30
    state["s"] = replace(IDLE, left=replace(IDLE.left, primary=True))   # X: disable
    wait_for(lambda: ("disable", None) in fake.calls, 3)
    state["s"] = IDLE
    time.sleep(2.6)                                             # past two watchdog polls: still running
    assert node.phase == "running" and node.state()["teleop"]["disabled_by_x"] is True
    node.stop_teleop()


def test_y_arms_through_the_node_and_the_estop_stops_teleop(env):
    fake, node, state = env["fake"], env["node"], env["state"]
    node.start_teleop()
    time.sleep(0.2)
    state["s"] = replace(IDLE, left=replace(IDLE.left, secondary=True))   # Y held
    wait_for(lambda: node.client is not None and node.client.latch.armed, 3)
    n = len(fake.datagrams)
    wait_for(lambda: any(d["type"] == "target" and d["armed"] for _, d in fake.datagrams[n:]), 2)
    state["s"] = IDLE
    out = node.estop("test", "pytest")
    assert out["targets"]["wbc"]["latched"]
    wait_for(lambda: node.phase == "stopped", 3)


def test_latency_and_armed_in_the_stream(env):
    node = env["node"]
    assert FIELDS[-4:] == ["armed", "wbc_armed", "quest_latency", "quest_rate_short"]
    node.start_teleop()
    wait_for(lambda: node.latest and node.latest.get("quest_rate_short") and node.latest["quest_rate_short"] > 30, 3)
    assert node.latest["armed"] == 0.0 and node.latest["quest_latency"] is not None
    node.stop_teleop()
