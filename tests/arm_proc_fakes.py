"""Picklable pieces for tests/test_arm_proc.py that run in spawned processes (the arm's control process, a
joint-channel client): a module of their own, so a child imports only this."""
import math
import select
import time

import numpy as np

from robot_utils.nodes.arm.bus import SimBus
from robot_utils.nodes.common import fastpath as F


class SlowGripperSim(SimBus):
    """The simulated bus with a gripper check that blocks for 0.8 s (as the real one closes the gripper)."""

    def check_gripper(self) -> float:
        time.sleep(0.8)
        return super().check_gripper()


def jc_client(server_path, client_path, token, seconds, q):
    """A joint-channel client in its own process (the WBC's role): goals at ~97 Hz along a sine; it waits on its
    socket between goals and stamps every state packet as it arrives: the latency from each goal sent to the first
    state packet reporting it applied, and the intervals between state packets."""
    c = F.Client(server_path, client_path)
    try:
        if c.hello(token, "load")["type"] != "welcome":
            q.put(None)
            return
        st = c.latest_state()
        q0 = np.array(st.q)
        sent: dict = {}
        lat, gaps = [], []
        seen, last = 0, None
        t0 = nxt = time.monotonic()
        while time.monotonic() - t0 < seconds:
            while True:
                left = nxt - time.monotonic()
                if left <= 0:
                    break
                r, _, _ = select.select([c.sock], [], [], left)
                if not r:
                    continue
                data = c.sock.recv(65536)
                if F.is_control(data):
                    continue
                now = time.monotonic()
                s = F.unpack_state(data)
                if last is not None:
                    gaps.append(now - last)
                last = now
                for k in range(seen + 1, s.seq + 1):
                    if k in sent:
                        lat.append(now - sent.pop(k))
                seen = max(seen, s.seq)
            a = 0.1 * math.sin(2 * math.pi * 0.5 * (time.monotonic() - t0))
            k = c.send(q=q0 + [a, a, 0, 0, a, 0])
            sent[k] = time.monotonic()
            nxt += 0.0103                                # not 100 Hz: every phase against the ticks
        q.put((lat, gaps))
    finally:
        c.sock.close()


def burn(stop_ev) -> None:
    """A CPU burner (one core busy at normal priority) until ``stop_ev``: the robot's other nodes."""
    x = 0
    while not stop_ev.is_set():
        for i in range(20000):
            x += i * i
