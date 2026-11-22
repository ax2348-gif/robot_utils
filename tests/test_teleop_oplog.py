"""The operator stream (``GET /oplog/stream``; the contract in docs/plans/data_collection_v1.md): node-protocol
records, JSON header, no payload: ``headset`` {t_rx, frame}, ``sent`` {t_send, datagram}, ``event`` (the event line,
flat in the header); every record in order; a slow subscriber loses the oldest records and the next record carries
``dropped``; publishing never blocks. End to end with the fake headset and the fake WBC: every frame and every
datagram the WBC received, in order with the events."""
import asyncio
import os
import threading
import time
from dataclasses import replace

import httpx
import pytest

from helpers import serve, stop, wait_for
from quest_teleop_fakes import FakeWbc
from robot_utils.nodes.common.protocol import RecordReader
from robot_utils.nodes.quest_teleop import oplog as O
from robot_utils.nodes.quest_teleop.node import QuestTeleopNode
from robot_utils.nodes.quest_teleop.server import create_app

from quest_teleop.fake_headset import FakeHeadsetServer, Headset, HeadsetState  # noqa: E402 (tools/, via the adapter)

IDLE = HeadsetState()


def decode(data: bytes) -> list[dict]:
    return [h for h, _ in RecordReader().feed(data)]


def test_order_across_types_and_dropped_counts():
    async def main():
        op = O.OpLog()
        op.attach(asyncio.get_running_loop())
        fast, slow = op.subscribe(100), op.subscribe(3)
        op.headset(1.0, {"frame_seq": 1})
        op.sent(1.1, {"type": "alive", "seq": 1})
        op.event({"seq": 7, "t": 1.2, "wall": 5.0, "kind": "armed", "episode": None, "detail": {"arm_seq": 1}})
        for i in range(5):
            op.headset(2.0 + i, {"frame_seq": 2 + i})
        await asyncio.sleep(0)
        items, dropped = op.take(fast)
        assert [h["type"] for h in items] == ["headset", "sent", "event"] + ["headset"] * 5 and dropped == 0
        assert items[2] == {"type": "event", "seq": 7, "t": 1.2, "wall": 5.0, "kind": "armed", "episode": None,
                            "detail": {"arm_seq": 1}}                       # flat, the event line as is
        items, dropped = op.take(slow)
        assert [h["frame"]["frame_seq"] for h in items] == [4, 5, 6] and dropped == 5
        recs = decode(O.encode(items, dropped))
        assert recs[0]["dropped"] == 5 and "dropped" not in recs[1] and all(r["len"] == 0 for r in recs)
        assert op.take(slow) == ([], 0)                                     # counted once
        assert not slow.event.is_set() and not slow.wake_pending
        op.unsubscribe(fast)
        op.unsubscribe(slow)
        op.headset(9.0, {})                                                 # no subscribers: nothing kept
        assert op.subscribers == 0
    asyncio.run(main())


def test_publishing_never_blocks_on_a_stuck_subscriber():
    async def main():
        op = O.OpLog()
        op.attach(asyncio.get_running_loop())
        sub = op.subscribe(50)
        t0 = time.monotonic()
        threading.Thread(target=lambda: [op.headset(i, {"i": i}) for i in range(20000)]).start()
        await asyncio.sleep(0.5)
        items, dropped = op.take(sub)
        assert len(items) == 50 and dropped == 20000 - 50 and items[-1]["frame"] == {"i": 19999}
        assert time.monotonic() - t0 < 2
    asyncio.run(main())


def read_records(url: str, out: list, halt: threading.Event, path="/oplog/stream", params=None):
    with httpx.Client(base_url=url, timeout=10) as c:
        with c.stream("GET", path, params=params or {}) as r:
            rr = RecordReader()
            for chunk in r.iter_raw():
                for h, _ in rr.feed(chunk):
                    out.append(h)
                if halt.is_set():
                    return


@pytest.fixture
def served(tmp_path):
    node = QuestTeleopNode("http://127.0.0.1:9", quest_host="127.0.0.1", quest_port=1, ar="off",
                           data_dir=str(tmp_path / "ev"))
    url, srv, th = serve(create_app(node, start=False))
    yield node, url
    stop((srv, th))
    node.eventlog.close()


def test_the_stream_over_http_in_order_with_slow_client_drops(served):
    node, url = served
    got, halt = [], threading.Event()
    th = threading.Thread(target=read_records, args=(url, got, halt), daemon=True)
    th.start()
    wait_for(lambda: node.oplog.subscribers == 1 and got)           # a ping first
    assert got[0]["type"] == "ping"
    node.oplog.headset(10.0, {"frame_seq": 1})
    node.emit("button", {"button": "A", "pressed": True})
    node.oplog.sent(10.2, {"type": "alive", "armed": False})
    node.emit("button", {"button": "A", "pressed": False})
    wait_for(lambda: len([h for h in got if h["type"] != "ping"]) == 4)
    recs = [h for h in got if h["type"] != "ping"]
    assert [h["type"] for h in recs] == ["headset", "event", "sent", "event"]
    assert recs[1]["kind"] == "button" and recs[1]["detail"] == {"button": "A", "pressed": True}
    assert recs[1]["seq"] + 1 == recs[3]["seq"] and recs[2]["datagram"] == {"type": "alive", "armed": False}
    assert recs[0] == {"type": "headset", "t_rx": 10.0, "frame": {"frame_seq": 1}, "len": 0}
    halt.set()
    node.oplog.headset(11.0, {})
    th.join(5)
    # a subscriber with a small backlog that reads late loses the oldest, with the count (as the endpoint encodes)

    async def burst():
        sub = node.oplog.subscribe(10)
        for i in range(25):
            node.oplog.headset(20.0 + i, {"i": i})
        items, dropped = node.oplog.take(sub)
        node.oplog.unsubscribe(sub)
        return decode(O.encode(items, dropped))
    recs = asyncio.run(burst())
    assert recs[0]["dropped"] == 15 and [r["frame"]["i"] for r in recs] == list(range(15, 25))


# -- end to end: the fake headset, the fake WBC, the node -----------------------------------------------------

@pytest.fixture
def env(tmp_path, tmp_sock_dir):
    fake = FakeWbc(unix_path=os.path.join(tmp_sock_dir, "wbc.o.sock"))
    wurl, wsrv, wth = serve(fake.app)
    headset = FakeHeadsetServer()
    node = QuestTeleopNode(wurl, arm=None, swerve=None, quest_host="127.0.0.1", quest_port=headset.port, ar="off",
                           data_dir=str(tmp_path / "ev"))
    url, srv, th = serve(create_app(node))
    stop_ev = threading.Event()
    state = {"s": IDLE}
    hs = Headset()

    def play():
        while not stop_ev.is_set():
            headset.send(hs.frame(state["s"], time.monotonic()))
            time.sleep(1 / 72)
    pt = threading.Thread(target=play, daemon=True)
    pt.start()
    wait_for(lambda: headset.n_clients > 0 and node.monitor.frames > 5, 5)
    yield {"fake": fake, "node": node, "url": url, "state": state}
    stop_ev.set()
    pt.join(1)
    stop((srv, th), (wsrv, wth))
    headset.close()
    fake.close()


def test_every_frame_and_datagram_with_the_events_in_order(env):
    fake, node, url, state = env["fake"], env["node"], env["url"], env["state"]
    got, halt = [], threading.Event()
    th = threading.Thread(target=read_records, args=(url, got, halt), daemon=True)
    th.start()
    wait_for(lambda: node.oplog.subscribers == 1)
    with httpx.Client(base_url=url, timeout=10) as c:
        assert c.post("/start").status_code == 200
        time.sleep(0.8)          # past the latency gate's first full 0.5 s rate window (S4): input fresh, Y can arm
        state["s"] = replace(IDLE, left=replace(IDLE.left, secondary=True))      # Y: arm
        wait_for(lambda: node.client is not None and node.client.latch.armed, 3)
        state["s"] = IDLE
        time.sleep(0.3)
        assert c.post("/stop").status_code == 200
    time.sleep(0.3)
    halt.set()
    node.oplog.headset(0.0, {})
    th.join(5)
    recs = [h for h in got if h["type"] != "ping"]
    assert not any("dropped" in h for h in recs)
    kinds = [h["kind"] for h in recs if h["type"] == "event"]
    for k in ("teleop_start", "enabled", "button", "armed", "teleop_stop"):
        assert k in kinds, kinds
    assert kinds.index("teleop_start") < kinds.index("armed") < kinds.index("teleop_stop")
    seqs = [h["seq"] for h in recs if h["type"] == "event"]
    assert seqs == list(range(seqs[0], seqs[0] + len(seqs)))                  # gap-free
    sent = [h["datagram"] for h in recs if h["type"] == "sent"]
    received = [d for _, d in fake.datagrams]
    assert sent == received and sent                                          # exactly what the WBC got
    t_send = [h["t_send"] for h in recs if h["type"] == "sent"]
    assert t_send == sorted(t_send)
    frames = [h for h in recs if h["type"] == "headset" and h["frame"]]
    fs = [h["frame"]["frame_seq"] for h in frames]
    assert fs == sorted(fs) and len(fs) > 50 and fs == list(range(fs[0], fs[0] + len(fs)))   # every frame
    assert all(h["frame"]["schema"] == "teleop.quest_state.v1" for h in frames)
    # the armed event follows the frame that completed the Y press, and precedes the first armed datagram
    i_armed = next(i for i, h in enumerate(recs) if h["type"] == "event" and h["kind"] == "armed")
    i_first_armed = next(i for i, h in enumerate(recs) if h["type"] == "sent" and h["datagram"]["armed"])
    assert i_armed < i_first_armed
    assert any(h["type"] == "headset" for h in recs[:i_armed])
    # the same events are in the log
    logged = [ln["seq"] for ln in map(__import__("json").loads, node.eventlog.since(seqs[0] - 1))]
    assert logged[:len(seqs)] == seqs
