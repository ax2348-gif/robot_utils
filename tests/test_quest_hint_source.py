"""The base hint's source (tools/quest_teleop/hint_source.py, body_frozen.py "The hint's source"): head (the default,
unchanged), chest or hips from body tracking, smoothed, falling back to the head; never a jump on engage, toggle,
dropout or return. The fake headset scripts the pelvis / chest twist and the head turns separately."""
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from quest_teleop import frames as F  # noqa: E402
from quest_teleop.body_frozen import Config, RobotState, TeleopCore  # noqa: E402
from quest_teleop.fake_headset import Headset, HeadsetState  # noqa: E402
from quest_teleop.hint_source import HintFilterConfig, HintSources, Smoother  # noqa: E402
from quest_teleop.quest_stream import Frame, QuestGate  # noqa: E402

IDLE = HeadsetState(chest_yaw=0.0, hips_yaw=0.0)     # the body faces forward whatever the head does
GRIP = replace(IDLE, right=replace(IDLE.right, grip=1.0))
BASE_YAW = 0.3


class Rig:
    """Headset (with a body), gate and core on a hand-advanced clock (10 ms frames); the base stays put."""

    def __init__(self, body="core", **cfg):
        self.hs = Headset(body=body, neck_worn=cfg.pop("neck_worn", True))
        self.gate = QuestGate(rate_window=0.0, recover_s=0.0)
        self.core = TeleopCore(Config(**cfg))
        self.t = 100.0
        self.rob = RobotState(0.0, np.array([1.5, 2.1, 0.8]), F.rot_axis((1, 1, 0), 0.7), 1.0, 2.0, BASE_YAW, 0.0)
        self.hints = []
        self.records = []
        self.core.on_hint = self.records.append

    def run(self, s, n=1):
        cmd = None
        for _ in range(n):
            self.t += 0.01
            d = json.loads(json.dumps(self.hs.frame(s, self.t)))
            self.rob.t_recv = self.t
            self.core.on_frame(d, self.gate.check(Frame(d, self.t, 1)), self.rob, self.t)
            cmd = self.core.command(self.t, self.gate.fresh(self.t), self.rob)
            self.hints.append(hint(cmd))
        return cmd

    def ramp(self, a: HeadsetState, b: HeadsetState, n: int, **fields):
        """From ``a`` to ``b`` over n frames, interpolating the named fields (scalars or tuples)."""
        for k in range(1, n + 1):
            u = k / n
            upd = {}
            for f in fields:
                va, vb = getattr(a, f), getattr(b, f)
                va = 0.0 if va is None else va
                upd[f] = tuple(np.add(va, np.multiply(u, np.subtract(vb, va)))) if isinstance(vb, tuple) \
                    else va + u * (vb - va)
            self.run(replace(b, **upd))
        return self.run(b, 150)                   # settle (the filter: 1.5 s = 10 time constants)

    def max_step(self, since=0):
        h = [x for x in self.hints[since:] if x is not None]
        return max((max(math.hypot(b[0] - a[0], b[1] - a[1]), abs(F.wrap(b[2] - a[2])))
                    for a, b in zip(h, h[1:])), default=0.0)


def hint(cmd):
    if not isinstance(cmd, dict) or cmd["base"].get("mode") != "hint":
        return None
    t = cmd["base"]["target"]
    return np.array([t["pose"]["x"], t["pose"]["y"], t["heading"]["yaw"]])


def engaged(rig, s=GRIP):
    rig.run(IDLE, 3)
    rig.run(s, 3)
    assert rig.core.status.state == "engaged"
    h = rig.hints[-1]
    np.testing.assert_allclose(h, [1.0, 2.0, BASE_YAW], atol=1e-9)        # engaging moves nothing
    return h


def yaw_of(h, ref):
    return F.wrap(h[2] - ref[2])


# -- smoothing ----------------------------------------------------------------------------------------------

def test_smoother_low_passes_then_trails_by_the_deadband_without_steps():
    sm = Smoother(HintFilterConfig(tau=0.1, deadband_m=0.02, deadband_rad=0.03))
    out = [sm.update(k * 0.01, (0.0, 0, 0.0), 0.0) for k in range(5)]
    assert np.allclose(out[-1], 0)
    small = [sm.update(0.05 + k * 0.01, (0.015, 0, 0.0), 0.02) for k in range(100)]
    assert np.allclose(small[-1], 0)                                     # inside the deadband: nothing moves
    big = [sm.update(1.1 + k * 0.01, (0.5, 0, 0.0), 0.5) for k in range(200)]
    np.testing.assert_allclose(big[-1], [0.48, 0.0, 0.47], atol=1e-6)    # trails by the deadband
    steps = np.abs(np.diff(np.array(big), axis=0)).max()
    assert steps < 0.06                                                  # continuous: a low-passed step


def test_all_three_candidates_are_computed_and_logged_whichever_is_selected():
    rig = Rig(hint_source="head")
    rig.run(replace(IDLE, hips_yaw=0.2, chest_yaw=0.1), 5)
    rec = rig.records[-1]
    assert rec["selected"] == "head" and set(rec["candidates"]) == {"head", "chest", "hips"}
    for c in rec["candidates"].values():
        assert c["ok"] and len(c["p"]) == 3 and len(c["f_p"]) == 3 and "yaw" in c and "f_yaw" in c
    assert rec["candidates"]["hips"]["confidence"] == 0.9
    plain = Rig(body=None)
    plain.run(IDLE, 3)
    c = plain.records[-1]["candidates"]
    assert c["head"]["ok"] and not c["hips"]["ok"] and c["hips"]["reason"] == "no body"


# -- head: the default, unchanged ---------------------------------------------------------------------------

def test_head_default_is_identical_with_or_without_a_body():
    a, b = Rig(body=None), Rig(body="all")
    script = [IDLE] * 3 + [GRIP] * 3 + [replace(GRIP, head_yaw=0.2 * k / 10, hips_yaw=-0.1, head_pos=(0.01 * k, 0, 0))
                                        for k in range(10)] + [IDLE] * 3
    for s in script:
        ca, cb = a.run(s), b.run(s)
        assert json.dumps(ca, sort_keys=True) == json.dumps(cb, sort_keys=True)


def test_head_source_turns_the_base_with_the_head():
    rig = Rig(hint_source="head")
    h0 = engaged(rig)
    h = hint(rig.ramp(GRIP, replace(GRIP, head_yaw=0.4), 20, head_yaw=0))
    assert abs(yaw_of(h, h0) - 0.4) < 1e-6


# -- hips / chest ------------------------------------------------------------------------------------------

def test_hips_a_head_turn_alone_does_not_turn_the_base_a_pelvis_turn_does():
    rig = Rig(hint_source="hips")
    h0 = engaged(rig)
    h = hint(rig.ramp(GRIP, replace(GRIP, head_yaw=0.5, chest_yaw=0.0, hips_yaw=0.0), 20, head_yaw=0))
    assert abs(yaw_of(h, h0)) < 1e-6 and np.allclose(h[:2], h0[:2], atol=1e-6)
    assert rig.core.status.hint_in_force == "hips"
    s = replace(GRIP, head_yaw=0.5, chest_yaw=0.0, hips_yaw=0.4)
    h = hint(rig.ramp(replace(GRIP, head_yaw=0.5, hips_yaw=0.0), s, 20, hips_yaw=0))
    assert abs(yaw_of(h, h0) - (0.4 - 0.03)) < 2e-3                       # the deadband trails
    # walking: the pelvis 0.3 m forward in the operator's facing (head and all) moves the hint 0.3 m forward
    # in the robot's frame at the grip (the registration maps the operator's facing to the robot's)
    s2 = replace(s, head_pos=(0.3, 0.0, 0.0))
    h2 = hint(rig.ramp(s, s2, 30, head_pos=0))
    d = F.rot_z(-BASE_YAW)[:2, :2] @ (h2[:2] - h[:2])
    assert abs(d[0] - 0.3) <= 0.02 + 5e-3 and abs(d[1]) <= 0.02 + 5e-3    # within the deadband of the walk
    assert rig.max_step() < 0.03


def test_chest_follows_the_upper_body_not_the_pelvis():
    rig = Rig(hint_source="chest")
    h0 = engaged(rig)
    h = hint(rig.ramp(GRIP, replace(GRIP, hips_yaw=0.4, chest_yaw=0.0), 20, hips_yaw=0))
    assert abs(yaw_of(h, h0)) < 1e-6
    h = hint(rig.ramp(GRIP, replace(GRIP, hips_yaw=0.4, chest_yaw=-0.3), 20, chest_yaw=0))
    assert abs(yaw_of(h, h0) + (0.3 - 0.03)) < 2e-3


def test_engaging_twisted_and_toggling_never_jump():
    rig = Rig(hint_source="hips")
    twisted = replace(GRIP, head_yaw=0.6, chest_yaw=0.3, hips_yaw=-0.2)
    rig.run(replace(IDLE, head_yaw=0.6, chest_yaw=0.3, hips_yaw=-0.2), 3)
    rig.run(twisted, 3)
    np.testing.assert_allclose(rig.hints[-1], [1.0, 2.0, BASE_YAW], atol=1e-9)
    k = len(rig.hints)
    rig.core.cfg.hint_source = "chest"            # the page toggle mid-engagement: from the next grip
    rig.run(twisted, 20)
    assert rig.core.eng.hint_source == "hips" and rig.max_step(k - 1) < 1e-9
    rig.run(replace(IDLE, head_yaw=0.6, chest_yaw=0.3, hips_yaw=-0.2), 3)
    rig.run(twisted, 3)
    assert rig.core.eng.hint_source == "chest"
    np.testing.assert_allclose(rig.hints[-1], [1.0, 2.0, BASE_YAW], atol=1e-9)


def test_dropout_falls_back_to_the_head_without_a_jump_and_returns():
    rig = Rig(hint_source="hips", hint_return_s=0.3)
    h0 = engaged(rig)
    s = replace(GRIP, hips_yaw=0.3, chest_yaw=0.3, head_yaw=0.5)
    rig.ramp(GRIP, s, 20, hips_yaw=0, chest_yaw=0, head_yaw=0)
    before = rig.hints[-1].copy()
    k = len(rig.hints)
    lost = replace(s, body_active=False)
    rig.run(lost, 30)
    assert rig.core.status.hint_in_force == "head" and rig.core.status.hint_switches == 1
    assert rig.max_step(k - 1) < 0.01                                   # no jump at the switch
    np.testing.assert_allclose(rig.hints[-1], before, atol=1e-4)        # nothing moved: the head held still
    # on the head now: a head turn turns the base (smoothed)
    h = hint(rig.ramp(lost, replace(lost, head_yaw=0.8), 20, head_yaw=0))
    assert abs(yaw_of(h, before) - 0.3) < 3e-3          # the same direction as the head's last turn: no slack
    # the body returns: back to the hips after hint_return_s, from where the hint is
    k = len(rig.hints)
    back = replace(lost, body_active=True, head_yaw=0.8)
    rig.run(back, 20)
    assert rig.core.status.hint_in_force == "head"                      # not yet (0.2 s < 0.3 s)
    rig.run(back, 20)
    assert rig.core.status.hint_in_force == "hips" and rig.core.status.hint_switches == 2
    assert rig.max_step(k - 1) < 0.01
    h1 = rig.hints[-1].copy()
    h = hint(rig.ramp(back, replace(back, hips_yaw=0.0), 20, hips_yaw=0))     # the pelvis turns back 0.3
    assert abs(yaw_of(h, h1) + (0.3 - 0.03)) < 3e-3
    assert abs(yaw_of(h, h0) - 0.3) < 0.01            # in all: the head's 0.3 while it stood in, the pelvis back to 0


def test_low_confidence_falls_back_to_the_head():
    rig = Rig(hint_source="hips", hint_min_confidence=0.5)
    engaged(rig)
    rig.run(replace(GRIP, body_confidence=0.2), 5)
    assert rig.core.status.hint_in_force == "head" and "confidence" in rig.core.status.hint_reason
    assert rig.max_step() < 0.01


def test_no_body_at_all_runs_on_the_smoothed_head():
    rig = Rig(body=None, hint_source="hips")
    h0 = engaged(rig)
    assert rig.core.status.hint_in_force == "head"
    h = hint(rig.ramp(GRIP, replace(GRIP, head_yaw=0.4), 20, head_yaw=0))
    assert abs(yaw_of(h, h0) - (0.4 - 0.03)) < 3e-3


def test_a_head_worn_headset_works_the_same():
    rig = Rig(hint_source="hips", neck_worn=False)
    h0 = engaged(rig)
    h = hint(rig.ramp(GRIP, replace(GRIP, head_yaw=0.5), 20, head_yaw=0))
    assert abs(yaw_of(h, h0)) < 1e-6
    h = hint(rig.ramp(replace(GRIP, head_yaw=0.5), replace(GRIP, head_yaw=0.5, hips_yaw=0.4), 20, hips_yaw=0))
    assert abs(yaw_of(h, h0) - 0.37) < 2e-3


def test_the_hand_moves_the_ee_relative_to_the_hips_not_the_head():
    """Link 2 reads the hand in the source's frame: hips fixed, a head turn alone with the hand still leaves the EE
    target still."""
    rig = Rig(hint_source="hips")
    engaged(rig)
    c0 = rig.run(GRIP, 60)
    c1 = rig.ramp(GRIP, replace(GRIP, head_yaw=0.5), 20, head_yaw=0)
    np.testing.assert_allclose(c1["ees"]["arm"]["pos"], c0["ees"]["arm"]["pos"], atol=1e-6)


def test_sources_parse_the_frame_body_themselves():
    hs = Headset(body="core")
    src = HintSources()
    out = src.update(json.loads(json.dumps(hs.frame(replace(IDLE, hips_yaw=0.25), 1.0))))
    assert out["hips"].ok and out["chest"].ok and out["head"].ok


# -- the analysis script --------------------------------------------------------------------------------------

def _write_recording(d: Path, with_hints: bool, seconds: float = 4.0) -> None:
    from robot_utils.nodes.common.protocol import pack_record   # noqa: PLC0415
    d.mkdir(parents=True)
    (d / "session.json").write_text(json.dumps({"robot": "test", "streams": [
        {"spec": "quest_teleop/oplog/stream", "name": "quest_teleop.oplog", "url": "", "params": {}}]}))
    rng = np.random.default_rng(1)
    rig = Rig(hint_source="hips")
    with open(d / "quest_teleop.oplog.000.bin", "wb") as f:
        n = int(seconds * 100)
        for k in range(n):
            t = 100.0 + k * 0.01
            # still for 2 s (a little noise), then the head turns 0.6 rad and back while the pelvis stays
            head_yaw = 0.0 if k < 200 else 0.6 * math.sin(math.pi * (k - 200) / 200)
            s = replace(IDLE, head_yaw=head_yaw + 0.002 * rng.normal(), hips_yaw=0.003 * rng.normal(),
                      chest_yaw=0.003 * rng.normal())
            frame = json.loads(json.dumps(rig.hs.frame(s, t)))
            f.write(pack_record({"type": "headset", "t_rx": t, "frame": frame}))
            if with_hints:
                rig.records.clear()
                rig.core.on_frame(frame, rig.gate.check(Frame(frame, t, 1)), rig.rob, t)
                for r in rig.records:
                    f.write(pack_record({"type": "hint", **r}))


@pytest.mark.parametrize("with_hints", [True, False])
def test_hint_analysis_reports_jitter_lag_and_head_body_divergence(tmp_path, with_hints, capsys):
    from quest_teleop import hint_analysis as HA   # noqa: PLC0415
    d = tmp_path / "rec"
    _write_recording(d, with_hints)
    data, origin = HA.load(d, HintFilterConfig())
    assert origin == ("logged" if with_hints else "recomputed")
    res = HA.analyse(data)
    for n in ("head", "chest", "hips"):
        assert res[n]["ok"] == 1.0 and res[n]["rest_s"] > 1.0
        j = res[n]["jitter_yaw_rate_std"]
        assert j["smoothed"] < j["raw"]                      # the filter (and deadband) removes the jitter
    assert res["head"]["filter_lag_ms"] > 0
    assert 25 < res["hips"]["divergence_from_head_deg"]["max"] < 40      # the head turned ~34 deg alone
    assert HA.main([str(d)]) == 0 and "yaw vs the head" in capsys.readouterr().out
