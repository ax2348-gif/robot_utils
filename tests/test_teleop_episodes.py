"""Episodes in the teleop node (docs/quest_teleop_node.md, "Episodes"): A (right primary) starts one, a B tap
(right secondary, released < 2 s) stops and keeps it; the press rules (seen released first, fresh valid frames, a
press during focus loss never counts, one action per press); A refused while disarmed or disabled; A while recording
stops (kept) then starts, in that order; every abort reason; the sim reset sequence (disarm -> disable -> reset ->
est_reset changed -> enable, still disarmed; a timeout aborts) and no arming anywhere but a Y press.

The node runs with the pure client rig of tests/quest_latch_fakes.py (a hand-advanced clock, frames queued, the WBC
emulated in its row) as its teleop client; the sim is a small FastAPI fake served over HTTP."""
import threading
import time
from dataclasses import replace
from types import SimpleNamespace

import pytest
from fastapi import FastAPI

from helpers import serve, stop, wait_for
from quest_latch_fakes import IDLE, Rig, X, Y
from robot_utils.nodes.quest_teleop.node import QuestTeleopNode


def A(s=IDLE):
    return replace(s, right=replace(s.right, primary=True))


def B(s=IDLE):
    return replace(s, right=replace(s.right, secondary=True))


class Env:
    def __init__(self, sim=None, reset_timeout=1.0):
        self.rig = rig = Rig()
        rig.wbc.release = lambda: rig.wbc.calls.append("release")
        rig.wbc.close = lambda: None
        rig.wbc.renew_errors, rig.wbc.transport, rig.wbc.session = 0, "rig", 1
        rig.stream.last_error = None
        self.node = node = QuestTeleopNode("http://127.0.0.1:9", arm=None, swerve=None, quest_host="127.0.0.1",
                                           quest_port=1, ar="off", clock=lambda: rig.t, sim=sim,
                                           reset_timeout=reset_timeout, estimator=None)   # the sim reset only
        node.stream = rig.stream
        node.reader = SimpleNamespace(connected=True)
        node.client, node.wbc, node.phase, node.started_at = rig.client, rig.wbc, "running", rig.t
        node._hook(rig.client)
        node.set_enabled(True, "teleop start")

    def events(self, *kinds):
        return [e for e in map(__import__("json").loads, self.node.eventlog.since(0))
                if not kinds or e["kind"] in kinds]

    def kinds(self):
        return [e["kind"] for e in self.events() if e["kind"] != "button"]

    def press(self, s_down, n=4, s_up=IDLE):
        self.rig.run(s_up, 2)
        self.rig.run(s_down, n)
        self.rig.run(s_up, 2)

    @property
    def ep(self):
        return self.node.episodes


@pytest.fixture
def env():
    return Env()


def armed_env(env):
    env.rig.arm()
    assert env.rig.latch.armed
    return env


# -- A / B ------------------------------------------------------------------------------------------------------

def test_a_starts_and_a_b_tap_stops_and_keeps(env):
    armed_env(env)
    env.node.set_task("pick the red cube")
    env.press(A())
    assert env.ep.state == "recording"
    start = env.events("episode_start")[-1]
    eid = start["episode"]
    assert start["detail"] == {"task": "pick the red cube"} and len(eid) == 13 and eid[8] == "-"
    st = env.node.state()
    assert st["episode"] == {"id": eid, "task": "pick the red cube", "t_start": start["t"], "start_seq": start["seq"]}
    assert st["task"] == "pick the red cube"
    # events while recording carry the episode
    env.rig.run(IDLE, 1)
    env.press(B())
    stop_ev = env.events("episode_stop")[-1]
    assert stop_ev["episode"] == eid and env.ep.state == "idle" and env.node.state()["episode"] is None
    assert env.ep.last["outcome"] == "kept"
    buttons = [(e["detail"]["button"], e["detail"]["pressed"]) for e in env.events("button")]
    assert ("A", True) in buttons and ("A", False) in buttons and ("B", True) in buttons and ("B", False) in buttons
    b_down = next(e for e in env.events("button") if e["detail"] == {"button": "B", "pressed": True})
    assert b_down["episode"] == eid                                    # events while open carry its id


def test_a_needs_three_fresh_valid_frames_and_one_action_per_press(env):
    armed_env(env)
    env.press(A(), n=2)
    assert env.ep.state == "idle"
    env.rig.run(IDLE, 2)
    env.rig.run(A(), 40)                                                # held: one start only
    assert len(env.events("episode_start")) == 1
    env.rig.run(IDLE, 2)
    env.rig.run(A(), 4)                                                 # the next press: stop (kept), then start
    starts, stops = env.events("episode_start"), env.events("episode_stop")
    assert len(starts) == 2 and len(stops) == 1
    assert stops[0]["episode"] == starts[0]["episode"] and stops[0]["seq"] < starts[1]["seq"]
    assert starts[1]["episode"] != starts[0]["episode"]
    assert int(starts[1]["episode"][-4:]) == int(starts[0]["episode"][-4:]) + 1


def test_a_held_through_focus_loss_does_not_count(env):
    armed_env(env)
    env.rig.run(IDLE, 2)
    env.rig.run(replace(A(), focused=False), 3)                        # pressed while unfocused (disarms too)
    env.rig.run(A(), 6)                                                 # focus back, A still held
    assert not env.events("episode_start")
    env.rig.run(IDLE, 2)
    env.rig.arm()
    env.press(A())
    assert len(env.events("episode_start")) == 1


def test_a_without_a_release_first_and_with_stale_input_does_not_count(env):
    armed_env(env)
    env.rig.run(A(), 5)                    # A held since before anything was seen released? it was: IDLE frames
    assert len(env.events("episode_start")) == 1
    env.press(B())
    # stale frames during a press spoil it
    env.rig.run(IDLE, 2)
    env.rig.run(A(), 2)
    env.rig.frame(A(), dt=0.25)            # a gap: a discontinuity (and stale): spoiled, and disarmed
    env.rig.run(A(), 5)
    assert len(env.events("episode_start")) == 1


def test_a_refused_while_disarmed_or_disabled(env):
    env.press(A())                                                      # never armed
    assert env.ep.state == "idle" and not env.events("episode_start")
    refused = [e for e in env.events("button") if e["detail"].get("action") == "refused"]
    assert refused and refused[-1]["detail"]["reason"].startswith("disarmed")
    env.press(X())                                                      # disabled
    env.press(A())
    refused = [e for e in env.events("button") if e["detail"].get("action") == "refused"]
    assert refused[-1]["detail"]["reason"].startswith("disabled")
    assert env.ep.counts["refused"] == 2 and not env.events("episode_start")


def test_b_hold_aborts_and_a_late_release_is_no_tap(env):
    armed_env(env)
    env.press(A())
    env.rig.run(IDLE, 2)
    env.rig.run(B(), 250)                                               # 2.5 s held
    ab = env.events("episode_abort")
    assert ab and ab[-1]["detail"]["reason"] == "b_hold"
    env.rig.run(IDLE, 2)
    assert not env.events("episode_stop")


def test_b_tap_needs_a_fresh_press(env):
    armed_env(env)
    env.press(A())
    env.press(B(), n=2)                                                 # too short to count
    assert env.ep.state == "recording"
    env.press(B(), n=3)
    assert env.ep.state == "idle" and len(env.events("episode_stop")) == 1


# -- aborts -----------------------------------------------------------------------------------------------------

def recording(env):
    armed_env(env)
    env.press(A())
    assert env.ep.state == "recording"
    return env.events("episode_start")[-1]["episode"]


def last_abort(env):
    ab = env.events("episode_abort")
    assert len(ab) == 1, ab
    return ab[0]["detail"]["reason"], ab[0]


def test_abort_on_x(env):
    eid = recording(env)
    env.rig.run(X(), 2)
    reason, ev = last_abort(env)
    assert reason == "x" and ev["episode"] == eid
    k = env.kinds()
    assert k.index("episode_abort") < k.index("disarmed")              # the abort comes first


def test_abort_on_any_disarm(env):
    recording(env)
    env.rig.wbc.wbc_disarm()                                            # the WBC disarms on its own
    env.rig.run(IDLE, 3)
    reason, ev = last_abort(env)
    assert reason == "disarm" and "WBC disarmed" in ev["detail"]["why"]


def test_abort_on_disable_seen_by_the_poll(env):
    recording(env)
    w = {"estop": {"latched": False}, "fault": None, "hold_reason": None, "enabled": True}
    env.node._poll_events(w)
    env.node._poll_events({**w, "enabled": False, "disabled_reason": "the WBC page"})
    assert last_abort(env)[0] == "disabled"
    assert env.events("disabled")[-1]["detail"] == {"reason": "the WBC page"}


def test_abort_on_estop(env):
    recording(env)
    env.node.estop("test", "pytest")
    reason, ev = last_abort(env)
    assert reason == "estop"
    assert env.events("estop")[-1]["detail"] == {"reason": "test", "source": "pytest"}
    # a WBC e-stop someone else pressed is seen by the poll (once)
    env2 = Env()
    recording(env2)
    w = {"estop": {"latched": False}, "fault": None, "hold_reason": None, "enabled": True}
    env2.node._poll_events(w)
    env2.node._poll_events({**w, "estop": {"latched": True, "reason": "button", "source": "wbc page"}})
    env2.node._poll_events({**w, "estop": {"latched": True, "reason": "button", "source": "wbc page"}})
    assert last_abort(env2)[0] == "estop" and len(env2.events("estop")) == 1


def test_abort_on_a_wbc_fault(env):
    recording(env)
    env.rig.stream.row["state"] = 5.0
    env.rig.run(IDLE, 2)
    assert last_abort(env)[0] == "fault"
    env.node._poll_events({"estop": {"latched": False}, "fault": "solver", "hold_reason": None, "enabled": True})
    assert len(env.events("fault")) == 1                                # seen once


def test_a_brief_dropout_holds_and_keeps_the_episode(env):
    """A transient (stale or late input) holds the robot and keeps the recording; a re-grip resumes in the same
    episode."""
    eid = recording(env)
    env.rig.frame(IDLE, dt=0.3, deliver=False)                          # no frame for 300 ms
    assert env.events("stale")[-1]["detail"]["reason"] == "staleness"
    assert len(env.events("held")) == 1 and not env.events("episode_abort")
    assert env.ep.state == "recording"
    env2 = Env()
    recording(env2)
    env2.rig.run(IDLE, 3, late=0.4)                                     # 400 ms late
    assert env2.events("stale")[-1]["detail"]["reason"] == "latency"
    assert not env2.events("episode_abort") and env2.ep.state == "recording"


def test_a_dropout_past_the_grace_window_escalates_and_aborts(env):
    recording(env)
    env.rig.frame(IDLE, dt=0.3, deliver=False)
    env.rig.frame(IDLE, dt=2.5, deliver=False)                          # still no input past the 2 s window
    assert len(env.events("escalated")) == 1
    assert last_abort(env)[0] == "disarm"


def test_abort_on_staleness_and_latency_with_resume_off():
    """--no-resume: every dropout disarms and aborts, as before the tiers."""
    env = Env()
    env.node.set_resume(enabled=False)
    env.rig.latch.resume = env.node.resume
    recording(env)
    env.rig.frame(IDLE, dt=0.3, deliver=False)
    reason, ev = last_abort(env)
    assert reason == "stale"
    env2 = Env()
    env2.node.set_resume(enabled=False)
    env2.rig.latch.resume = env2.node.resume
    recording(env2)
    env2.rig.run(IDLE, 3, late=0.4)
    assert last_abort(env2)[0] == "stale" and env2.events("stale")[-1]["detail"]["reason"] == "latency"


def test_a_telemetry_blackout_the_latch_holds_through_keeps_the_episode(env):
    """The reader drops a connection silent for 1 s and reopens it: armed (or holding), the episode goes on."""
    recording(env)
    env.node.reader.connected = False
    env.rig.run(IDLE, 1)
    assert not env.events("episode_abort") and env.ep.state == "recording"


def test_abort_on_telemetry_loss_when_not_resumable():
    env = Env()
    env.node.set_resume(enabled=False)
    env.rig.latch.resume = env.node.resume
    recording(env)
    env.node.reader.connected = False
    env.rig.run(IDLE, 1)
    assert last_abort(env)[0] == "telemetry_loss"


def test_abort_on_teleop_stop(env):
    recording(env)
    env.node.stop_teleop("requested")
    assert last_abort(env)[0] == "teleop_stop"
    k = env.kinds()
    assert k.index("episode_abort") < k.index("teleop_stop")
    assert env.events("teleop_stop")[-1]["detail"] == {"reason": "requested"}


# -- the sim reset ----------------------------------------------------------------------------------------------

class FakeSim:
    def __init__(self, rig, order, bump=True):
        self.reset_seq = 0
        app = FastAPI()

        @app.get("/state")
        def state():
            return {"state": "running", "reset_seq": self.reset_seq}

        @app.post("/scene/reset")
        def reset(body: dict | None = None):
            order.append(("reset", rig.stream.row["state"]))
            self.reset_seq += 1
            if bump:                    # the estimator's reset shows in the WBC stream a little later
                def later():
                    time.sleep(0.1)
                    rig.stream.row["est_reset"] += 1.0
                threading.Thread(target=later, daemon=True).start()
            return {"t": 0.0, "running": True, "rtf": 1.0, "reset_seq": self.reset_seq}
        self.app = app


@pytest.fixture
def sim_env():
    made = []

    def make(bump=True, reset_timeout=1.0):
        order = []
        env = Env(reset_timeout=reset_timeout)
        sim = FakeSim(env.rig, order, bump)
        url, srv, th = serve(sim.app)
        made.append((srv, th))
        env.node.episodes.sim = env.node.sim = url
        w = env.rig.wbc
        dis, en = w.disable, w.enable
        w.disable = lambda: (order.append(("disable", w.stream.row["est_reset"])), dis())[1]
        w.enable = lambda: (order.append(("enable", w.stream.row["est_reset"])), en())[1]
        return env, order, sim
    yield make
    stop(*made)


def drive_until(env, cond, s=IDLE, timeout=5.0):
    """Keep the headset frames coming (the client keeps stepping) until ``cond()``."""
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        env.rig.frame(s)
        if cond():
            return
        time.sleep(0.005)
    raise AssertionError("condition not met in time")


def test_sim_reset_sequence_ends_enabled_but_disarmed(sim_env):
    env, order, sim = sim_env()
    armed_env(env)
    presses = env.rig.latch.presses
    env.rig.run(IDLE, 2)
    env.rig.run(A(), 4)
    assert env.ep.state == "resetting"
    n_msgs = len(env.rig.wbc.msgs)
    assert not env.rig.latch.armed and env.rig.last()["armed"] is False     # disarmed at once
    # Y during the reset is refused: nothing arms or enables
    drive_until(env, lambda: env.ep.state != "resetting", s=Y())
    assert env.ep.state == "recording"
    names = [o[0] for o in order]
    assert names == ["disable", "reset", "enable"], order
    assert order[1][1] == 0.0                                          # the WBC was disabled across the jump
    assert order[2][1] == order[0][1] + 1.0                             # enabled after est_reset changed
    k = env.kinds()
    k = k[k.index("disarmed"):]                                         # from the A on
    seq = [k.index(x) for x in ("disarmed", "disabled", "sim_reset", "enabled", "episode_start")]
    assert seq == sorted(seq), k
    assert env.events("sim_reset")[-1]["detail"] == {"reset_seq": 1}
    assert env.events("disarmed")[-1]["detail"]["reason"] == "sim reset"
    # never armed automatically: no press counted, every datagram since disarmed, the WBC row disarmed
    assert env.rig.latch.presses == presses and not env.rig.latch.armed
    assert all(m["armed"] is False for m in env.rig.wbc.msgs[n_msgs - 1:])
    assert env.rig.stream.row["armed"] == 0.0 and env.rig.stream.row["state"] == 2.0
    assert env.rig.latch.inhibit is None and env.rig.latch.disabled_by_client is False
    assert not env.events("episode_abort")
    # the operator arms with Y as always (a new press)
    env.rig.run(IDLE, 3)
    env.rig.run(Y(), 3)
    assert env.rig.latch.armed and env.rig.latch.presses == presses + 1


def test_a_while_recording_in_the_sim_stops_then_resets_then_starts(sim_env):
    env, order, sim = sim_env()
    armed_env(env)
    env.press(A())
    drive_until(env, lambda: env.ep.state == "recording")
    first = env.ep.episode_id
    env.rig.run(IDLE, 3)
    env.rig.run(Y(), 3)                                                 # arm again
    assert env.rig.latch.armed
    env.press(A())
    drive_until(env, lambda: env.ep.state == "recording" and env.ep.episode_id != first)
    k = env.kinds()
    i_stop = k.index("episode_stop")
    assert k[i_stop:].index("sim_reset") < k[i_stop:].index("episode_start")
    assert env.events("episode_stop")[0]["episode"] == first and sim.reset_seq == 2


def test_sim_reset_timeout_aborts_and_stays_disabled(sim_env):
    env, order, sim = sim_env(bump=False, reset_timeout=0.3)
    armed_env(env)
    env.press(A())
    drive_until(env, lambda: env.ep.state == "idle")
    assert [o[0] for o in order] == ["disable", "reset"]               # never enabled
    ab = env.events("episode_abort")[-1]
    assert ab["detail"]["reason"] == "reset" and "did not show" in ab["detail"]["why"]
    assert not env.events("episode_start") and ab["episode"] is not None
    assert env.rig.stream.row["state"] == 0.0 and not env.rig.latch.armed
    assert env.rig.latch.disabled_by_client and env.rig.latch.inhibit is None
    assert env.node.state()["episode"] is None
    # Y enables and arms, as after X (a deliberate press)
    env.rig.run(IDLE, 3)
    env.rig.run(Y(), 4)
    assert env.rig.latch.armed


def test_x_during_the_sim_reset_cancels_it(sim_env):
    env, order, sim = sim_env(bump=False, reset_timeout=3.0)
    armed_env(env)
    env.press(A())
    drive_until(env, lambda: any(o[0] == "reset" for o in order))
    drive_until(env, lambda: env.ep.state == "idle", s=X())
    assert "enable" not in [o[0] for o in order]                       # (X's own disable may follow)
    assert env.events("episode_abort")[-1]["detail"]["reason"] == "x"
    assert not env.events("episode_start")


def test_a_without_a_sim_never_resets(env):
    armed_env(env)
    env.press(A())
    assert env.ep.state == "recording" and not env.events("sim_reset")
    assert "disable" not in env.rig.wbc.calls and env.rig.latch.armed   # the robot: the episode starts armed


def test_reset_world_resets_the_estimator_at_the_sims_base_pose():
    # a sim reset alone leaves the estimator where it was (the wheel jump is skipped as an odometry discontinuity),
    # so nothing waiting for a new est_reset (the WBC, the A handshake) ever sees one: reset_world resets both
    from robot_utils.nodes.common.client import NodeClient
    from robot_utils.nodes.quest_teleop.episodes import reset_world
    calls = []
    sim_app, est_app = FastAPI(), FastAPI()
    sim_app.post("/scene/reset")(lambda: calls.append("sim") or {"reset_seq": 7})
    sim_app.get("/state")(lambda: {"base": {"xyz": [0.4, -0.2, 0.0], "rpy": [0.0, 0.0, 1.2]}})

    @est_app.post("/reset")
    def est_reset(body: dict):
        calls.append(("est", body))
        return {"reset_id": 3}
    (surl, ssrv, sth), (eurl, esrv, eth) = serve(sim_app), serve(est_app)
    try:
        with NodeClient(surl) as sim:
            assert reset_world(sim, eurl) == {"reset_seq": 7}
            assert calls == ["sim", ("est", {"x": 0.4, "y": -0.2, "theta": 1.2})]
            calls.clear()
            reset_world(sim, None)                          # no estimator: the sim only
            assert calls == ["sim"]
    finally:
        stop((ssrv, sth), (esrv, eth))
