"""A pure rig for the Quest client loop (tools/quest_teleop/teleop.TeleopClient) with the armed latch: a hand-advanced
clock, a frame queue instead of the TCP reader, a WBC row instead of the stream, and a WBC client that records every
datagram and HTTP call and emulates the WBC's latch in its row (state, armed, seq) as nodes/wbc does."""
import math
import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from quest_teleop.body_frozen import Config  # noqa: E402
from quest_teleop.fake_headset import Headset, HeadsetState  # noqa: E402
from quest_teleop.quest_stream import Frame, QuestGate  # noqa: E402
from quest_teleop.teleop import TeleopClient  # noqa: E402

IDLE = HeadsetState()
GRIP = replace(IDLE, right=replace(IDLE.right, grip=1.0))


def with_left(s, **kw):
    return replace(s, left=replace(s.left, **kw))


def Y(s=IDLE):
    return with_left(s, secondary=True)


def X(s=IDLE):
    return with_left(s, primary=True)


def stick(s, left=(0.0, 0.0), right_x=0.0):
    return replace(s, left=replace(s.left, stick=left), right=replace(s.right, stick=(right_x, 0.0)))


def trigger(s, v):
    return replace(s, right=replace(s.right, trigger=v))


class Reader:
    def __init__(self):
        self.q = []

    def get(self, timeout=None):
        return self.q.pop(0) if self.q else None

    def drain(self):
        out, self.q = self.q, []
        return out


class Stream:
    def __init__(self, rig):
        self.rig = rig
        self.row = {"state": 2.0, "armed": 0.0, "seq": 0.0, "base_x": 1.0, "base_y": 2.0, "base_yaw": 0.3,
                    "est_reset": 0.0, "base_vx": 0.0, "base_vy": 0.0, "base_w": 0.0,
                    "arm_cmd_x": 1.5, "arm_cmd_y": 2.1, "arm_cmd_z": 0.8, "arm_cmd_qx": 0.0, "arm_cmd_qy": 0.0,
                    "arm_cmd_qz": math.sin(0.2), "arm_cmd_qw": math.cos(0.2)}
        self.t = None
        self.frozen = False          # True: the row stops arriving (stale)

    def latest(self):
        if not self.frozen:
            self.t = self.rig.t
        return dict(self.row), self.t


class Wbc:
    """The client's WbcClient: datagrams and HTTP calls recorded; the WBC's latch emulated in the row."""

    def __init__(self, stream):
        self.stream = stream
        self.seq = 0
        self.msgs = []
        self.calls = []
        self.sent = {"target": 0, "alive": 0}
        self.arm_seq_floor = 0
        self.refuse_enable = None
        self.cur = 0                 # the WBC's armed arm_seq
        self.seen = 0

    def _dgram(self, m):
        self.seq += 1
        m["seq"] = self.seq
        self.msgs.append(m)
        self.sent[m["type"]] += 1
        row = self.stream.row
        row["seq"] = float(self.seq)
        self.seen = max(self.seen, m["arm_seq"])
        if row["armed"] and (not m["armed"] or m["arm_seq"] < self.cur):
            row["armed"], self.arm_seq_floor = 0.0, self.seen
        elif (not row["armed"] and m["type"] == "target" and m["armed"] and m["arm_seq"] > self.arm_seq_floor
              and row["state"] in (2.0, 3.0, 4.0)):
            row["armed"], self.cur = 1.0, m["arm_seq"]
        return m

    def send_target(self, body, armed=False, arm_seq=0):
        return self._dgram({"type": "target", "t": None, **body, "armed": armed, "arm_seq": arm_seq})

    def send_alive(self, armed=False, arm_seq=0):
        return self._dgram({"type": "alive", "armed": armed, "arm_seq": arm_seq})

    def wbc_disarm(self):
        """The WBC disarms on its own (an overrun, a fault, ...)."""
        self.stream.row["armed"], self.arm_seq_floor = 0.0, self.seen

    def enable(self):
        self.calls.append("enable")
        if self.refuse_enable:
            raise RuntimeError(self.refuse_enable)
        self.stream.row["state"], self.stream.row["armed"] = 2.0, 0.0
        self.arm_seq_floor = self.seen
        return {"enabled": True}

    def disable(self):
        self.calls.append("disable")
        self.stream.row["state"], self.stream.row["armed"] = 0.0, 0.0
        self.arm_seq_floor = self.seen
        return {"enabled": False}


def no_latency_gate(stale_s=0.2):
    """The consumer rules without the latency / rate judgement (fresh at once): for tests of other logic. The
    latency gate itself is tested with the real one (test_quest_latency.py)."""
    return QuestGate(stale_s, rate_window=0.0, recover_s=0.0)


class Rig:
    def __init__(self, cfg=Config(), latency=False, **kw):
        """``latency``: the real latency gate (a full 0.5 s window before any input is fresh); otherwise off."""
        self.t = 100.0
        self.hs = Headset()
        self.reader = Reader()
        self.stream = Stream(self)
        self.wbc = Wbc(self.stream)
        self.client = TeleopClient(cfg, self.reader, self.stream, self.wbc, clock=lambda: self.t, sync_http=True,
                                   **kw)
        if not latency:
            self.client.gate = no_latency_gate()
        self.client.sent_log = []
        self.conn = 1

    @property
    def latch(self):
        return self.client.latch

    @property
    def core(self):
        return self.client.core

    def frame(self, s, dt=0.01, late=0.0, data=None, deliver=True):
        """One headset frame sampled ``late`` s before it arrives (latency); one client step."""
        self.t += dt
        d = data if data is not None else self.hs.frame(s, self.t - late)
        if deliver:
            self.reader.q.append(Frame(d, self.t, self.conn))
        return self.client.step(wait=0)

    def run(self, s, n=3, **kw):
        out = None
        for _ in range(n):
            out = self.frame(s, **kw)
        return out

    def arm(self, s=IDLE):
        """A deliberate Y press: released, then held for 3 frames; then released."""
        self.run(s, 2)
        self.run(Y(s), 3)
        assert self.latch.armed, self.latch.reason
        return self.run(s, 1)

    def last(self, kind=None):
        ms = [m for m in self.wbc.msgs if kind is None or m["type"] == kind]
        return ms[-1] if ms else None

    def since(self, n):
        return self.wbc.msgs[n:]
