"""Haptics (robot_utils.nodes.quest_teleop.haptics; docs/quest_teleop_node.md "Haptics"): the cue table, pulse ids,
the levels (mirror-aware hands), the toggles, the published lines and the stream, the headset's ack; the fake
headset's player (a port of quest_env native/haptics.h); end to end, the node's GET /haptics?stream=1 played by the
fake headset with the ack coming back in its frames."""
import json
import threading
import time

import pytest

from helpers import Clock, wait_for
from robot_utils.nodes.quest_teleop import haptics as H
from robot_utils.nodes.quest_teleop.client import quest_ctl  # noqa: F401 (the adapter: tools/ on the path)
from robot_utils.nodes.quest_teleop.haptics import CUES, EVENT_CUES, GripContact, Haptics, Sources

from quest_teleop.fake_headset import HapticsPlayer  # noqa: E402 (tools/, via the adapter)


class FakeSources(Sources):
    def __init__(self):
        self.is_running, self.is_armed = True, True
        self.hands = {"right": "arm"}
        self.eng = {"right": True}
        self.row, self.row_t = None, None
        self.arms = {}

    def running(self):
        return self.is_running

    def armed(self):
        return self.is_armed

    def pairing(self):
        return dict(self.hands)

    def engaged(self):
        return dict(self.eng)

    def wbc_row(self):
        return self.row, self.row_t

    def arm_target(self, ee):
        return self.arms.get(ee)


class FakeWatch:
    rows: dict = {}

    def __init__(self, target):
        self.target = target
        self.t = None
        self.last_error = None
        self.closed = False

    def start(self):
        return self

    def latest(self):
        return FakeWatch.rows.get(self.target, (None, None))

    def close(self):
        self.closed = True


@pytest.fixture
def hx():
    clock = Clock()
    src = FakeSources()
    FakeWatch.rows = {}
    h = Haptics(src, clock=clock, epoch="e1", watch_factory=FakeWatch)
    return h, src, clock


def ev(kind, **detail):
    return {"kind": kind, "detail": detail}


# -- the cue table --------------------------------------------------------------------------------------------------

def test_every_event_cue_names_a_pulse_cue_and_the_defaults_are_state_on_continuous_off():
    for kind, (cue, _) in EVENT_CUES.items():
        assert CUES[cue].kind == "pulse", kind
    assert {k for k, c in CUES.items() if c.kind == "level"} == {"limit", "grip"}
    assert all(not CUES[k].enabled for k in ("limit", "grip"))
    assert all(CUES[k].enabled for k in ("armed", "resumed", "disarmed", "held", "disabled"))


def test_armed_is_one_short_pulse_on_both_hands_with_increasing_ids(hx):
    h, _, _ = hx
    made = h.on_event(ev("armed", arm_seq=1))
    assert [(p["hand"], p["delay_s"], p["duration_s"]) for p in made] == [("left", 0.0, 0.06), ("right", 0.0, 0.06)]
    assert [p["id"] for p in made] == [1, 2] and made[0]["group"] == 1 and made[1]["group"] == 2
    m = h.message()
    assert m["schema"] == H.SCHEMA and m["epoch"] == "e1" and m["seq"] == 1
    assert [p["id"] for p in m["hands"]["right"]["pulses"]] == [2]


@pytest.mark.parametrize("kind,detail,cue,n", [
    ("disarmed", {"reason": "x"}, "disarmed", 2), ("held", {}, "held", 2), ("hold", {"reason": "input stale"}, "held", 2),
    ("resumed", {}, "resumed", 1), ("resume_refused", {}, "held", 2), ("escalated", {}, "disarmed", 2),
    ("disabled", {"reason": "x"}, "disabled", 1), ("estop", {"reason": "page"}, "disabled", 1),
    ("y_refused", {"reason": "a grip is pressed"}, "refused", 3)])
def test_event_kinds_map_through_the_table(hx, kind, detail, cue, n):
    h, _, _ = hx
    made = h.on_event({"kind": kind, "detail": detail})
    assert {p["cue"] for p in made} == {cue}
    assert [p for p in made if p["hand"] == "left"] and len([p for p in made if p["hand"] == "right"]) == n
    if cue == "disabled":
        assert made[0]["duration_s"] == 0.4
    if n >= 2:
        right = [p for p in made if p["hand"] == "right"]
        assert right[1]["delay_s"] > right[0]["delay_s"] and right[0]["group"] == right[1]["group"]


def test_unmapped_kinds_and_the_wbcs_hold_while_disarmed_make_nothing(hx):
    h, _, _ = hx
    assert h.on_event(ev("button", button="Y")) == []
    assert h.on_event(ev("hold", reason="disarmed: press Y")) == []
    assert h.pulses == type(h.pulses)()


def test_a_disarm_and_a_disable_together_feel_like_one_long(hx):
    h, _, clock = hx
    assert h.on_event(ev("disabled"))
    clock.t += 0.05
    assert h.on_event(ev("disarmed")) == []                     # lower priority, within DEDUPE_S: suppressed
    clock.t += H.DEDUPE_S
    assert h.on_event(ev("disarmed"))                           # later: a new cue


def test_master_switch_cue_toggles_and_amplitudes(hx):
    h, _, _ = hx
    h.configure(enabled=False)
    assert h.on_event(ev("armed")) == []
    h.configure(enabled=True, cues={"armed": {"enabled": False}})
    assert h.on_event(ev("armed")) == []
    h.configure(cues={"armed": {"enabled": True, "amplitude": 0.25}})
    assert {p["amplitude"] for p in h.on_event(ev("armed"))} == {0.25}
    for bad in ({"nope": {}}, {"armed": {"amplitude": 2}}, {"armed": {"other": 1}}):
        with pytest.raises(ValueError):
            h.configure(cues=bad)
    with pytest.raises(ValueError):
        h.configure(limit_kinds=["none"])
    assert h.config()["cues"]["armed"]["amplitude"] == 0.25


def test_a_test_cue_plays_even_when_off(hx):
    h, _, _ = hx
    h.configure(enabled=False)
    made = h.cue("limit", ("left",), test=True)
    assert [(p["hand"], p["duration_s"]) for p in made] == [("left", 1.0)]


# -- levels ---------------------------------------------------------------------------------------------------------

def mirrored(src):
    src.hands = {"left": "right_arm", "right": "left_arm"}         # mirror pairing: the left hand drives right_arm
    src.eng = {"left": True, "right": True}


def test_limit_buzzes_the_hand_driving_the_limited_arm_mirror_aware(hx):
    h, src, clock = hx
    mirrored(src)
    src.row, src.row_t = {"right_arm_limited_by": 1.0, "left_arm_limited_by": 0.0}, clock.t
    h.update()
    assert h.levels == {"left": 0.0, "right": 0.0}               # off by default
    h.configure(cues={"limit": {"enabled": True}})
    assert h.levels == {"left": CUES["limit"].amplitude, "right": 0.0} and h.level_cue["left"] == "limit"
    src.hands = {"left": "left_arm", "right": "right_arm"}       # straight: the right hand drives right_arm
    h.update()
    assert h.levels == {"left": 0.0, "right": CUES["limit"].amplitude}


def test_limit_kinds_engagement_staleness_and_the_hold(hx):
    h, src, clock = hx
    mirrored(src)
    h.configure(cues={"limit": {"enabled": True}})
    src.row, src.row_t = {"right_arm_limited_by": 7.0, "left_arm_limited_by": 0.0}, clock.t      # lead: not a kind
    h.update()
    assert h.levels["left"] == 0.0
    src.row = {"right_arm_limited_by": 4.0, "left_arm_limited_by": 0.0}                         # joint_limit
    h.update()
    assert h.levels["left"] > 0
    src.row = {"right_arm_limited_by": 0.0, "left_arm_limited_by": 0.0}
    clock.t += H.LIMIT_HOLD_S / 2
    src.row_t = clock.t
    h.update()
    assert h.levels["left"] > 0                                  # held briefly: no chatter
    clock.t += H.LIMIT_HOLD_S
    src.row_t = clock.t
    h.update()
    assert h.levels["left"] == 0.0
    src.eng["left"] = False                                      # a released hand feels nothing
    src.row = {"right_arm_limited_by": 1.0}
    h.update()
    assert h.levels["left"] == 0.0
    src.eng["left"] = True
    src.row_t = clock.t - 1.0                                    # a stale WBC row
    h.update()
    assert h.levels["left"] == 0.0
    src.row_t, src.is_armed = clock.t, False                     # disarmed: nothing
    h.update()
    assert h.levels["left"] == 0.0


def test_limit_falls_back_to_the_global_limited_by_on_every_engaged_hand(hx):
    h, src, clock = hx
    mirrored(src)
    src.eng = {"left": True, "right": False}
    h.configure(cues={"limit": {"enabled": True}})
    src.row, src.row_t = {"limited_by": 2.0}, clock.t           # an older WBC: no per-arm column
    h.update()
    assert h.levels["left"] > 0 and h.levels["right"] == 0.0


def grip_row(gap, goal, qd=0.0, force=10.0, limiting=0.0):
    return {"g_gap": gap, "g_gap_goal": goal, "g_qd": qd, "g_force": force, "g_force_max": 40.0,
            "g_limiting": limiting}


def test_contact_pulses_the_hand_driving_that_gripper_and_grip_follows_the_force(hx):
    h, src, clock = hx
    mirrored(src)
    src.arms = {"right_arm": "right_arm", "left_arm": "left_arm"}
    FakeWatch.rows["right_arm"] = (grip_row(0.03, 0.03, qd=2.0), clock.t)     # closing, no contact
    h.update()
    assert set(h.watches) == {"right_arm", "left_arm"}                        # contact is on by default
    assert h.counts["cues"]["contact"] == 0
    FakeWatch.rows["right_arm"] = (grip_row(0.03, 0.0), clock.t)              # stalled short of the goal
    h.update()
    assert h.counts["cues"]["contact"] == 0                                    # for STALL_S first
    clock.t += H.STALL_S + 1e-3
    FakeWatch.rows["right_arm"] = (grip_row(0.03, 0.0), clock.t)
    h.update()
    assert h.counts["cues"]["contact"] == 1
    assert h.last_cue["hands"] == ["left"]                                     # mirror: the left trigger
    assert h.levels["left"] == 0.0                                             # grip: off by default
    h.configure(cues={"grip": {"enabled": True}})
    assert h.levels["left"] == pytest.approx(CUES["grip"].amplitude * max(0.3, 10 / 40))
    FakeWatch.rows["right_arm"] = (grip_row(0.03, 0.0, force=40.0, limiting=1.0), clock.t)
    h.update()
    assert h.levels["left"] == pytest.approx(CUES["grip"].amplitude)
    assert h.counts["cues"]["contact"] == 1                                    # still in contact: no new pulse
    h.configure(cues={"contact": {"enabled": False}, "grip": {"enabled": False}})
    assert h.watches == {} and h.levels["left"] == 0.0                         # streams closed when unneeded


def test_grip_contact_from_the_force_limit_alone():
    c = GripContact()
    assert c.update(grip_row(0.01, 0.01, qd=3.0, limiting=1.0), 0.0)
    assert not c.update(grip_row(0.02, 0.02, qd=3.0), 0.1)
    assert not c.update(None, 0.2)


# -- lines, the stream, the ack -------------------------------------------------------------------------------------

def test_lines_are_published_on_cues_and_level_changes_only(hx):
    h, src, clock = hx
    h.update()
    assert h._seq == 0
    h.on_event(ev("armed"))
    assert h._seq == 1
    h.update()
    assert h._seq == 1                                           # nothing changed
    h.configure(cues={"limit": {"enabled": True}})               # a config change publishes
    assert h._seq == 2
    src.row, src.row_t = {"arm_limited_by": 1.0}, clock.t
    h.update()
    assert h._seq == 3 and h.message()["hands"]["right"]["level"] > 0
    clock.t += 2.0
    m = h.message()
    assert m["hands"]["right"]["pulses"] == []                   # past KEEP_S: no longer repeated


def test_the_stream_writes_the_state_then_each_line_and_keepalives(hx, monkeypatch):
    h, _, clock = hx
    monkeypatch.setattr(H, "IDLE_S", 0.05)
    monkeypatch.setattr(H, "KEEPALIVE_S", 0.05)
    gen = h.stream_lines()
    first = json.loads(next(gen))
    assert first["seq"] == 0 and h.readers == 1
    threading.Timer(0.02, lambda: h.on_event(ev("armed"))).start()
    t0 = time.monotonic()
    line = json.loads(next(gen))
    assert line["seq"] == 1 and len(line["hands"]["left"]["pulses"]) == 1
    assert time.monotonic() - t0 < 0.045                         # pushed, not after a keepalive period
    keep = json.loads(next(gen))                                 # nothing new: a fresh line after the keepalive
    assert keep["seq"] == 1
    gen.close()
    assert h.readers == 0


def test_the_headsets_ack_gives_the_round_trip(hx):
    h, _, clock = hx
    made = h.on_event(ev("armed"))
    h.on_frame(clock.t + 0.03, {"haptics": {"schema": H.SCHEMA, "epoch": "e1", "last_id": made[-1]["id"],
                                             "connected": True}})
    i = h.info()
    assert i["headset"]["capable"] and i["headset"]["connected"]
    assert i["ack_ms"]["n"] == 2 and i["ack_ms"]["max"] == pytest.approx(30.0)
    h.on_frame(clock.t + 0.05, {"haptics": {"schema": H.SCHEMA, "epoch": "other", "last_id": 99}})
    assert h.info()["ack_ms"]["n"] == 2                          # another epoch: not ours
    h.on_frame(clock.t, {})
    assert not h.info()["headset"]["capable"]


# -- the fake headset's player (quest_env native/haptics.h, ported) ---------------------------------------------

def test_the_player_plays_each_pulse_once_in_order_and_levels_until_their_ttl():
    clock = Clock(0.0)
    p = HapticsPlayer(clock)
    h = Haptics(FakeSources(), clock=Clock(), epoch="e1")
    h.on_event(ev("disarmed"))
    line = json.dumps(h.message())
    assert p.receive(line)
    assert [c["id"] for c in p.tick()] == [1, 3]                 # the first of each hand's pair at once
    assert p.block() == {"schema": H.SCHEMA, "epoch": "e1", "last_id": 4, "connected": False}
    clock.t = 0.14
    assert [c["id"] for c in p.tick()] == [2, 4]
    assert p.receive(line)
    clock.t = 0.5
    assert p.tick() == []                                        # the same ids again: not replayed
    stale = h.message()
    stale["hands"]["left"]["pulses"] = [{**stale["hands"]["left"]["pulses"][0], "id": 10, "group": 10, "age_s": 0.9}]
    p.receive(json.dumps(stale))
    assert p.tick() == [] and p.last_id == 10                    # too old to mean anything: skipped, acked
    lv = h.message()
    lv["seq"], lv["hands"]["right"] = 9, {"level": 0.2, "frequency_hz": 0.0, "pulses": []}
    p.receive(json.dumps(lv))
    cmd = p.tick()
    assert [(c["hand"], c["action"], c["amplitude"]) for c in cmd] == [("right", "apply", 0.2)]
    clock.t += lv["ttl_s"] + 0.01
    assert [(c["hand"], c["action"]) for c in p.tick()] == [("right", "stop")]
    assert not p.receive("not json") and p.bad == 1
    lv["epoch"] = "e2"                                           # a new node life: ids start over
    lv["hands"]["left"]["pulses"] = [{**lv["hands"]["left"]["pulses"][0], "id": 1, "age_s": 0.0}] \
        if lv["hands"]["left"]["pulses"] else []
    p.receive(json.dumps(lv))
    assert p.epoch == "e2"


# -- end to end: the node's stream, played by the fake headset ----------------------------------------------------

def test_end_to_end_the_fake_headset_plays_the_nodes_stream_and_acks(tmp_path, monkeypatch):
    import httpx
    from helpers import serve, stop
    from quest_teleop_fakes import FakeWbc
    from robot_utils.nodes.quest_teleop.node import QuestTeleopNode
    from robot_utils.nodes.quest_teleop.server import create_app
    from quest_teleop.fake_headset import FakeHeadsetServer, HeadsetState, play

    monkeypatch.setenv("HOME", str(tmp_path))
    fake = FakeWbc()
    wurl, wsrv, wth = serve(fake.app)
    headset = FakeHeadsetServer()
    node = QuestTeleopNode(wurl, quest_host="127.0.0.1", quest_port=headset.port, ssh_host="quest", ar="off")
    url, srv, th = serve(create_app(node))
    http = httpx.Client(base_url=url, timeout=10)
    try:
        player = headset.start_haptics(url + "/haptics?stream=1")
        wait_for(lambda: node.haptics.readers == 1)
        snap = http.get("/haptics").json()
        assert snap["schema"] == H.SCHEMA and snap["epoch"] == node.haptics.epoch
        stopper = threading.Event()
        frames = threading.Thread(target=lambda: play(headset, [(0.0, HeadsetState()), (30.0, HeadsetState())],
                                                      72.0, stop=stopper), daemon=True)
        frames.start()
        r = http.post("/haptics/test", json={"cue": "armed"})
        assert r.status_code == 200 and len(r.json()["pulses"]) == 2
        wait_for(lambda: {c["hand"] for c in player.played if c["action"] == "apply"} == {"left", "right"})
        info = wait_for(lambda: (lambda i: i if i["ack_ms"] else None)(http.get("/haptics/info").json()))
        assert info["headset"]["capable"] and info["headset"]["connected"] and info["readers"] == 1
        assert info["ack_ms"]["max"] < 500
        assert http.post("/haptics/config", json={"cues": {"nope": {}}}).status_code == 422
        assert http.post("/haptics/config", json={"enabled": False}).json()["enabled"] is False
        stopper.set()
        frames.join(5)
    finally:
        http.close()
        headset.close()
        node.close()
        stop((srv, th), (wsrv, wth))
        fake.close()
