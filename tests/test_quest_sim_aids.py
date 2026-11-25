"""The teleop node's sim aids (nodes/quest_teleop/sim_aids.py): the goal ghost's pose and colour from WBC rows, and
the auto pause's decisions, against a fake node and a fake sim."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from robot_utils.nodes.quest_teleop.sim_aids import GHOST, AidsConfig, SimAids, goal_in_link, severity_color
from robot_utils.scene3d.pose import compose, inverse

KEYS = ("x", "y", "z", "qx", "qy", "qz", "qw")


def row(tgt, meas, ee="arm"):
    out = {f"{ee}_tgt_{k}": v for k, v in zip(KEYS, tgt)}
    out.update({f"{ee}_meas_{k}": v for k, v in zip(KEYS, meas)})
    return out


def test_goal_in_link_is_the_goal_seen_from_the_measured_end_link():
    tcp = [0.1, 0.0, 0.02]
    meas = np.array([0.5, 0.1, 0.4, 0, np.sin(0.3), 0, np.cos(0.3)])
    tgt = np.array([0.55, 0.12, 0.38, 0, 0, np.sin(0.1), np.cos(0.1)])
    delta, ep, er = goal_in_link(row(tgt, meas), "arm", tcp)
    t = np.array([*tcp, 0, 0, 0, 1])
    end_meas = compose(meas, inverse(t))                  # the end link that carries this TCP
    goal_tcp = compose(compose(end_meas, delta), t)       # the end link moved by delta carries the target
    assert np.allclose(goal_tcp[:3], tgt[:3], atol=1e-9)
    assert np.allclose(abs(np.dot(goal_tcp[3:], tgt[3:])), 1, atol=1e-9)
    assert ep == pytest.approx(np.linalg.norm(tgt[:3] - meas[:3]))
    assert er > 0
    same, ep0, er0 = goal_in_link(row(meas, meas), "arm", tcp)
    assert np.allclose(same, [0, 0, 0, 0, 0, 0, 1], atol=1e-9) and ep0 == 0 and er0 < 1e-6
    assert goal_in_link(row([math.nan] * 7, meas), "arm", tcp) is None


def test_severity_runs_green_yellow_red_and_keeps_alpha():
    c = AidsConfig(ghost_alpha=0.3)
    assert severity_color(0, 0, c) == [0.25, 0.9, 0.45, 0.3]
    assert severity_color(0.025, 0, c) == [1.0, 0.85, 0.2, 0.3]                # half of 5 cm: yellow
    assert severity_color(0.2, 0, c) == [1.0, 0.25, 0.2, 0.3]                  # beyond: red
    assert severity_color(0, math.radians(20), c) == [1.0, 0.25, 0.2, 0.3]     # rotation alone


class FakeSim:
    def __init__(self):
        self.running, self.calls = True, []

    def __call__(self, method, path, body=None):
        self.calls.append((method, path, body))
        if path == "/state":
            return {"state": "running" if self.running else "paused", "t": 1.0}
        if path == "/pause":
            self.running = False
        if path == "/run":
            self.running = True
        return {}


def fake_node(engaged=False, sticks=(0.0, 0.0)):
    events, hub = [], SimpleNamespace(state_changed=lambda: None)
    frame = {"hands": {"left": {"joystick": list(sticks)}, "right": {"joystick": [0.0, 0.0]}}}
    status = SimpleNamespace(state="engaged" if engaged else "released")
    return SimpleNamespace(
        phase="running", client=SimpleNamespace(core=SimpleNamespace(status=status)),
        monitor=SimpleNamespace(last=frame), episodes=SimpleNamespace(state="idle"),
        emit=lambda kind, detail=None: events.append((kind, detail)), events=SimpleNamespace(add=lambda *a: None),
        hub=hub, clock=lambda: 0.0, cfg=SimpleNamespace(ee="arm"), stream=SimpleNamespace(latest=lambda: (None, None)),
        wbc_target="wbc", log=events)


def aids_for(node, **cfg):
    a = SimAids(node, "sim", AidsConfig(ghost=False, auto_pause=True, pause_hold=0.25, **cfg))
    sim = FakeSim()
    a._call = sim
    return a, sim


def test_auto_pause_after_the_hold_and_resume_on_grip_or_stick():
    node = fake_node()
    a, sim = aids_for(node)
    a.tick(0.0)
    a.tick(0.2)
    assert sim.running                                        # idle, but not for pause_hold yet
    a.tick(0.3)
    assert not sim.running and a.paused_by_us and node.log[-1][0] == "sim_pause"
    node.client.core.status.state = "engaged"                 # a grip: runs at once
    a.tick(0.35)
    assert sim.running and not a.paused_by_us and node.log[-1] == ("sim_resume", {"reason": "operator active"})
    node.client.core.status.state = "released"
    a.tick(0.4)
    a.tick(0.7)
    assert not sim.running
    node.monitor.last["hands"]["left"]["joystick"] = [0.6, 0.0]        # a stick: runs at once
    a.tick(0.72)
    assert sim.running


def test_auto_pause_never_pauses_blind_during_a_reset_or_a_manual_pause():
    node = fake_node()
    a, sim = aids_for(node)
    node.monitor.last = None                                  # no headset frame: counts as active
    a.tick(0.0)
    a.tick(1.0)
    assert sim.running
    node.monitor.last = {"hands": {"left": {"joystick": [0, 0]}, "right": {"joystick": [0, 0]}}}
    a.tick(1.1)
    a.tick(1.5)
    assert not sim.running
    node.episodes.state = "resetting"                         # an episode's sim reset: run and stay running
    a.tick(1.6)
    a.tick(3.0)
    assert sim.running and node.log[-1] == ("sim_resume", {"reason": "sim reset"})
    node.episodes.state = "idle"
    sim.running = False                                       # paused by someone else
    n = len(sim.calls)
    a.tick(3.1)
    a.tick(3.5)
    assert not a.paused_by_us and ("post", "/run", None) not in sim.calls[n:]    # not ours to resume
    node.phase = "stopped"
    a.tick(4.0)
    assert not sim.running                                    # still not ours


def test_ghost_posts_a_relative_goal_with_its_colour():
    node = fake_node()
    meas = [0.5, 0, 0.4, 0, 0, 0, 1]
    node.stream.latest = lambda: (row([0.53, 0, 0.4, 0, 0, 0, 1], meas), 0.0)
    a = SimAids(node, "sim", AidsConfig(ghost=True, auto_pause=False))
    sim = FakeSim()
    a._call = sim
    a.ee_link, a.tcp = "arm_end_link", [0.0, 0.0, 0.0]
    a.tick(0.0)
    (m, path, body), = [c for c in sim.calls if c[1] == "/ghosts"]
    g = body["ghosts"][0]
    assert g["name"] == GHOST and g["body"] == "arm_end_link" and g["subtree"]
    assert np.allclose(g["relative"], [0.03, 0, 0, 0, 0, 0, 1]) and g["rgba"][:3] == [1.0, 0.73, 0.2]   # 3 cm of 5: past yellow
    node.stream.latest = lambda: ({}, 0.0)                    # no target: nothing posted (the ghost lapses)
    sim.calls.clear()
    a.tick(0.1)
    assert not [c for c in sim.calls if c[1] == "/ghosts"]
