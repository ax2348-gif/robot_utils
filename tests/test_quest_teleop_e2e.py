"""End to end: the Quest client (tools/quest_teleop) between the fake headset (TCP JSON lines) and a fake WBC node
(HTTP control plane, a rows stream, UDP target datagrams). Real sockets, real time (about 5 s)."""
import sys
import threading
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from helpers import serve, stop, wait_for
from quest_teleop_fakes import FakeWbc

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from quest_teleop import frames as F  # noqa: E402
from quest_teleop.body_frozen import Config  # noqa: E402
from quest_teleop.fake_headset import FakeHeadsetServer, HeadsetState, play  # noqa: E402
from quest_teleop.quest_stream import QuestReader  # noqa: E402
from quest_teleop.teleop import TeleopClient, close_wbc, open_wbc  # noqa: E402
from quest_teleop.wbc_client import WbcClient, WbcStream  # noqa: E402

IDLE = HeadsetState()
GRIP = replace(IDLE, right=replace(IDLE.right, grip=1.0))
REACH = replace(GRIP, right=replace(GRIP.right, pos=(0.45, -0.2, -0.30), trigger=0.8))
WALK = replace(REACH, head_pos=(0.5, 0.0, 0.0), right=replace(REACH.right, pos=(0.95, -0.2, -0.30)))
STICK = replace(WALK, left=replace(WALK.left, stick=(0.0, 1.0)))
RELEASED = replace(WALK, right=replace(WALK.right, grip=0.0))
REGRIP = replace(WALK, right=replace(WALK.right, grip=1.0, trigger=0.0))


def Y(s):
    """The Y button (left controller) pressed: arms (README "Arming: Y and X")."""
    return replace(s, left=replace(s.left, secondary=True))


# Y arms at the start; the outage (a gap) and the reconnect disarm, so each is followed by a new Y press while
# released, once the input is fresh again (0.5 s of good frames after the gap / reconnect). Input is fresh only after
# a full 0.5 s window after the connect: the whole timeline starts 0.5 s in.
_TIMELINE = [(0.0, IDLE), (0.2, IDLE), (0.21, Y(IDLE)), (0.3, Y(IDLE)), (0.31, IDLE),              # Y: armed
            (0.5, IDLE), (0.51, GRIP), (1.0, GRIP), (1.5, REACH), (2.0, WALK), (2.01, STICK),
            (2.3, STICK), (2.31, WALK), (2.5, WALK), (2.51, RELEASED), (3.2, RELEASED), (3.21, REGRIP),  # in the outage
            (3.6, REGRIP), (3.61, RELEASED), (4.2, RELEASED), (4.21, Y(RELEASED)), (4.3, Y(RELEASED)),  # Y again
            (4.31, RELEASED), (4.5, RELEASED), (4.51, REGRIP), (5.6, REGRIP),     # a new engagement; the reconnect
            (5.61, RELEASED), (5.8, RELEASED), (5.81, Y(RELEASED)), (5.9, Y(RELEASED)), (5.91, RELEASED),   # Y
            (6.3, RELEASED)]
LEAD = 0.5
SCENARIO = [(0.0, IDLE)] + [(t + LEAD, s) for t, s in _TIMELINE]
OUTAGE = (3.0 + LEAD, 3.4 + LEAD)
RECONNECT = 5.0 + LEAD


@pytest.fixture
def wbc_peer():
    fake = FakeWbc()
    url, srv, th = serve(fake.app)
    yield fake, url
    stop((srv, th))
    fake.close()


def phases(msgs):
    """Consecutive datagrams grouped by (EE frame, base mode)."""
    out = []
    for t, m in msgs:
        e = m["ees"]["arm"]
        assert e["frame"] == "world"                             # the WBC takes world targets only
        key = ("follow" if m["base"]["mode"] == "hint" else "ride", m["base"]["mode"])   # engaged = a base hint
        if not out or out[-1][0] != key:
            out.append([key, []])
        out[-1][1].append((t, m))
    return out


def test_client_drives_the_wbc_from_the_headset(wbc_peer):
    fake, url = wbc_peer
    fake.row["state"] = 2.0                                     # the WBC enabled (idle): Y arms without an enable
    headset = FakeHeadsetServer()
    reader = QuestReader("127.0.0.1", headset.port, reconnect_s=0.05).start()
    stream = WbcStream(url).start()
    wbc = WbcClient(url, udp_host="127.0.0.1")
    client = TeleopClient(Config(), reader, stream, wbc)
    halt = threading.Event()
    try:
        info = open_wbc(wbc, client.cfg)
        client.session_opened(info)
        assert info["session"] == fake.session and wbc.udp_port == fake.udp_port
        wait_for(lambda: stream.latest()[0] is not None and headset.n_clients == 1)
        loop = threading.Thread(target=client.run, args=(halt,), daemon=True)
        loop.start()
        dropped = []

        def on_tick(t):
            if t >= RECONNECT and not dropped:
                dropped.append(t)
                headset.drop_clients()

        play(headset, SCENARIO, rate=100, pause=lambda t: OUTAGE[0] <= t < OUTAGE[1], on_tick=on_tick)
        wait_for(lambda: len(fake.datagrams) > 0)
    finally:
        halt.set()
        close_wbc(wbc, log=lambda s: None)
        reader.close()
        stream.close()
        wbc.close()
        headset.close()

    # control plane: lease, session defaults, enable, session; disable and release at the end; renewed meanwhile
    assert fake.names() == ["lease", "hints", "enable", "session", "disable", "release"]
    assert ("renew", None) in fake.calls
    assert fake.calls[1][1]["base"] == {"mode": "locked"}

    dg = fake.datagrams
    msgs = [(t, m) for t, m in dg if m["type"] == "target"]
    assert all(m["session"] == fake.session for _, m in dg)
    seqs = [m["seq"] for _, m in dg]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs)
    assert all(m["t"] is None for _, m in msgs)
    # the armed latch: Start is not live (keep-alives with armed:false until Y); targets only while armed; three
    # presses (start, after the outage, after the reconnect), one arm_seq each
    assert dg[0][1]["type"] == "alive" and dg[0][1]["armed"] is False
    assert all(m["armed"] is True for _, m in msgs)
    assert sorted({m["arm_seq"] for _, m in msgs}) == [1, 2, 3]
    assert all(not m["armed"] for _, m in dg if m["type"] == "alive")

    ph = phases(msgs)
    keys = [k for k, _ in ph]
    assert keys == [("ride", "locked"),          # armed, released: hold
                    ("follow", "hint"),           # engaged: body-frozen targets, base hint (the stick adds to it)
                    ("ride", "locked"),          # released; the outage disarms (silence); Y again: hold
                    ("follow", "hint"),           # release + new engagement
                    ("ride", "locked")], keys    # reconnect mid-engagement disarms; Y again: hold
    assert all(m["est_reset"] == 0 for _, m in msgs)

    ee_p = np.array([1.52, 2.1, 0.8])            # the COMMANDED EE (the measured one is 2 cm behind)
    ee_r = F.rot_z(0.4)
    base_yaw = 0.3
    # zero jump at engagement; the reach and the walk move the target through the world (the fake base is still)
    first = ph[1][1][0][1]["ees"]["arm"]
    np.testing.assert_allclose(first["pos"], ee_p, atol=1e-4)
    np.testing.assert_allclose(F.matrix_from_quat(first["quat"]), ee_r, atol=1e-4)
    last = ph[1][1][-1][1]["ees"]["arm"]
    assert last["gripper"] == pytest.approx(1.0 - 0.8, abs=0.01)     # trigger 0.8 squeezed: the opening is 0.2
    # the base hint: the base pose at engagement, then + the walk (0.5 m forward) + the stick offset (~0.3 s forward)
    h0 = ph[1][1][0][1]["base"]["target"]
    assert (h0["pose"]["x"], h0["pose"]["y"], h0["heading"]["yaw"]) == (1.0, 2.0, pytest.approx(base_yaw))
    h1 = ph[1][1][-1][1]["base"]["target"]
    fwd = np.array([np.cos(base_yaw), np.sin(base_yaw)])
    offset = np.array([h1["pose"]["x"], h1["pose"]["y"]]) - [1.0, 2.0] - 0.5 * fwd
    assert 0.08 < offset @ fwd < 0.2
    assert h1["heading"]["yaw"] == pytest.approx(base_yaw, abs=1e-6)
    # the EE target: the hand's reach and walk, moved with the same stick offset as the hint (no turn: a translation)
    np.testing.assert_allclose(last["pos"], ee_p + F.rot_z(base_yaw) @ [0.6, 0.0, 0.05] + [*offset, 0.0], atol=2e-3)
    # the outage: nothing sent for most of it
    gaps = np.diff([t for t, _ in ph[2][1]])
    assert gaps.max() > 0.3
    # the new engagement starts again exactly at the command
    np.testing.assert_allclose(ph[3][1][0][1]["ees"]["arm"]["pos"], ee_p, atol=1e-4)
    assert client.gate.counts["discontinuities"] >= 2          # the gap and the reconnect
