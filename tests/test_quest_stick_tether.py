"""The stick tether (tools/quest_teleop/body_frozen.py, "The stick tether"): engaged, the stick offset leads the WBC's
commanded base by at most ``tether_pos`` / ``tether_yaw`` (zero while the base brakes from the stick), the base hint's
velocity reference keeps the raw stick, and link 2 takes the commanded base's yaw for the stick's yaw, so the hand's
motion since the grip maps exactly onto the arm relative to the robot. A stand-in base follows the hint with an
acceleration limit (and optionally a steering delay); the stream rows arrive at 50 Hz, the frames at 72 Hz. Pure."""
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from quest_teleop import frames as F  # noqa: E402
from quest_teleop.body_frozen import Config, RobotState, make_core  # noqa: E402
from quest_teleop.fake_headset import Headset, HeadsetState  # noqa: E402
from quest_teleop.quest_stream import Frame, QuestGate  # noqa: E402

IDLE = HeadsetState()
GRIP = replace(IDLE, right=replace(IDLE.right, grip=1.0))
BOTH = replace(GRIP, left=replace(GRIP.left, grip=1.0))
V, W = Config().stick_max_speed, Config().stick_max_omega
DT, ROW_DT = 1 / 72, 0.02
EE_REL = (np.array([0.55, -0.1, 0.8]), F.rot_axis((1, 1, 0), 0.7))     # the EE in the base frame at engagement
LEAD = Config().ride_lead


def stick(s, left=(0.0, 0.0), right_x=0.0):
    return replace(s, left=replace(s.left, stick=left), right=replace(s.right, stick=(right_x, 0.0)))


def hand_moved(s, d):
    return replace(s, right=replace(s.right, pos=tuple(np.array(s.right.pos) + d)))


class StandInBase:
    """The WBC's commanded base: the hint's velocity reference plus ``k`` times its pose error (the commanded
    velocity when released), capped at the stick maxima, limited to ``accel`` m/s² and ``alpha`` rad/s²; from rest
    a nonzero target starts moving only after ``steer`` s (the modules turning)."""

    def __init__(self, x=1.0, y=2.0, yaw=0.3, accel=1.0, alpha=1.5, k=2.0, steer=0.0):
        self.p, self.yaw = np.array([x, y], float), yaw
        self.v, self.w = np.zeros(2), 0.0
        self.accel, self.alpha, self.k, self.steer = accel, alpha, k, steer
        self.waited = 0.0
        self.track = []                                    # (p, yaw) per step

    def step(self, cmd, dt):
        vt, wt = np.zeros(2), 0.0
        b = cmd.get("base") if isinstance(cmd, dict) else None
        if b and b["mode"] == "hint":
            t = b["target"]
            vt = np.array([t["pose"]["vx"], t["pose"]["vy"]]) + self.k * (np.array([t["pose"]["x"], t["pose"]["y"]])
                                                                         - self.p)
            wt = t["heading"]["omega"] + self.k * F.wrap(t["heading"]["yaw"] - self.yaw)
        elif b and b["mode"] == "commanded":
            u = b["velocity"]
            vt, wt = F.rot_z(self.yaw)[:2, :2] @ [u["vx"], u["vy"]], u["omega"]
        n = np.linalg.norm(vt)
        vt = vt * (V / n) if n > V else vt
        wt = float(np.clip(wt, -W, W))
        at_rest = not np.any(self.v) and self.w == 0.0
        if at_rest and (np.linalg.norm(vt) > 1e-6 or abs(wt) > 1e-6) and self.waited < self.steer:
            self.waited += dt                              # the modules steer first
            vt, wt = np.zeros(2), 0.0
        elif at_rest:
            self.waited = 0.0                              # a new start steers again
        dv = vt - self.v
        n = np.linalg.norm(dv)
        self.v = self.v + (dv * (self.accel * dt / n) if n > self.accel * dt else dv)
        self.w += float(np.clip(wt - self.w, -self.alpha * dt, self.alpha * dt))
        self.p = self.p + self.v * dt
        self.yaw += self.w * dt
        self.track.append((self.p.copy(), self.yaw))

    def applied(self):
        """The base when the WBC applies a command sent now (``ride_lead`` ahead)."""
        return F.PlanarFrame(*(self.p + self.v * LEAD), self.yaw + self.w * LEAD)

    def row(self, t, ee="arm"):
        ee_p, ee_r = F.PlanarFrame(*self.p, self.yaw).to_world(*EE_REL)
        vb = F.rot_z(-self.yaw)[:2, :2] @ self.v
        return RobotState(t, ee_p, ee_r, float(self.p[0]), float(self.p[1]), float(self.yaw), 0.0,
                          base_twist=(float(vb[0]), float(vb[1]), self.w))


class Rig:
    def __init__(self, cfg=None, base=None):
        self.cfg = cfg or Config(tether=True)
        self.hs, self.gate = Headset(), QuestGate(rate_window=0.0, recover_s=0.0)
        self.core = make_core(self.cfg)
        self.base = base or StandInBase()
        self.t, self.t_row, self.row, self.cmd = 100.0, -1.0, None, None
        self.rows_on = True

    def rows(self):
        return {e: self.base.row(self.t_row, e) for e in self.cfg.hands.values()} if self.cfg.hands else \
            self.base.row(self.t_row)

    def frame(self, s, robot="row"):
        self.base.step(self.cmd, DT)
        self.t += DT
        if self.rows_on and self.t - self.t_row >= ROW_DT - 1e-9:
            self.t_row = self.t
            self.row = self.rows()
        d = self.hs.frame(s, self.t)
        rob = self.row if robot == "row" else robot
        self.core.on_frame(d, self.gate.check(Frame(d, self.t, 1)), rob, self.t)
        self.cmd = self.core.command(self.t, self.gate.fresh(self.t), self.row)
        return self.cmd

    def run(self, s, seconds):
        return [self.frame(s) for _ in range(round(seconds / DT))]

    def engage(self, s=GRIP):
        self.run(IDLE, 0.1)
        self.frame(s)
        assert self.core.eng is not None


def ee_pose(c, ee="arm"):
    e = c["ees"][ee]
    return np.array(e["pos"]), F.matrix_from_quat(e["quat"])


def riding_error(rig, c, ee="arm", rel=EE_REL):
    """(position m, yaw rad): the EE target against the engagement's EE riding rigidly on the commanded base."""
    p, r = ee_pose(c, ee)
    rp, rr = rig.base.applied().to_world(*rel)
    dr = r @ rr.T
    return float(np.linalg.norm(p - rp)), abs(math.atan2(dr[1, 0], dr[0, 0]))


def hint_lead(rig, c):
    t = c["base"]["target"]
    b = rig.base.applied()
    return float(np.hypot(t["pose"]["x"] - b.p[0], t["pose"]["y"] - b.p[1])), F.wrap(t["heading"]["yaw"] - b.yaw)


FWD = stick(GRIP, (0.0, 1.0))


# -- 1: full stick forward: the arm rides on the commanded base ----------------------------------------------------

def test_full_stick_forward_keeps_the_arm_on_the_commanded_base():
    rig, err = Rig(Config(tether=True)), []
    rig.engage()
    for _ in range(72):
        c = rig.frame(FWD)
        err.append(riding_error(rig, c)[0])
        assert hint_lead(rig, c)[0] <= 0.05 + 2e-3
    assert max(err) <= 0.05 + 2e-3, max(err)
    assert np.linalg.norm(rig.base.v) > 0.7                   # the base got up to speed (raw velocity reference)
    # before (tether off): the reference ran ahead of the accelerating base, the arm reaching for it
    old, before = Rig(Config(tether=False)), []
    old.engage()
    for _ in range(72):
        before.append(riding_error(old, old.frame(FWD))[0])
    assert max(before) > 0.2, max(before)
    print(f"arm deviation, 1 s full forward: before {max(before):.3f} m, after {max(err):.3f} m")


# -- 2: release: the base brakes and stops, never reverses ---------------------------------------------------------

@pytest.mark.parametrize("mode", ["third", "first"])
def test_release_the_stick_the_base_stops_where_its_braking_puts_it(mode):
    out = {}
    for tether in (True, False):
        rig = Rig(Config(mode=mode, tether=tether))
        rig.engage()
        rig.run(FWD, 1.0)
        n0 = len(rig.base.track)
        cs, arm = [], []
        for _ in range(round(2.0 / DT)):
            cs.append(rig.frame(GRIP))
            arm.append(riding_error(rig, cs[-1])[0])
        fwd = np.array([math.cos(0.3), math.sin(0.3)])
        along = [float(p @ fwd) for p, _ in rig.base.track[n0 - 1:]]
        out[tether] = (rig, cs, min(np.diff(along)), max(along) - along[-1], max(arm), max(arm[-10:]))
    rig, cs, step, back, arm, arm_end = out[True]
    assert step >= -1e-9 and back < 1e-6                       # never reverses: no snap-back
    assert np.linalg.norm(rig.base.v) < 1e-4                   # stopped
    lead = hint_lead(rig, cs[-1])
    assert lead[0] < 2e-3 and abs(lead[1]) < 1e-3              # the offset ends where the base stopped
    assert arm_end < 2e-3
    assert rig.core.eng.tether == {"xy": "idle", "yaw": "idle"}
    assert arm <= 0.05 + 2e-3
    assert out[False][4] > 0.2, out[False][4]                  # before: the arm reached ahead until the base got there
    print(f"{mode}: braking: arm deviation before {out[False][4]:.3f} m, after {arm:.3f} m; "
          f"snap-back before {out[False][3]:.3f} m, after {back:.4f} m")


# -- 3: modules perpendicular: the command is not clipped ----------------------------------------------------------

def test_a_steering_delay_does_not_stop_the_base_and_the_arm_stays_within_the_bound():
    rig = Rig(Config(tether=True), StandInBase(steer=0.3))
    rig.engage()
    err, refs = [], []
    for _ in range(round(1.2 / DT)):
        c = rig.frame(FWD)
        err.append(riding_error(rig, c)[0])
        t = c["base"]["target"]["pose"]
        refs.append(math.hypot(t["vx"], t["vy"]))
    assert max(err) <= 0.05 + 2e-3
    assert np.linalg.norm(rig.base.p - [1.0, 2.0]) > 0.3       # it started and moved
    assert min(refs[10:22]) > 0.6 * V                          # during the delay: the raw stick velocity, ramping in
    # the fit ramps in over its window, then the full stick speed
    assert refs[30] == pytest.approx(V, abs=1e-6)


# -- 4: yaw stick: link 2 has the commanded base's yaw -------------------------------------------------------------

@pytest.mark.parametrize("mode", ["third", "first"])
def test_a_yaw_stick_keeps_the_arm_on_the_base_and_the_heading_leads_within_the_bound(mode):
    res = {}
    for tether in (True, False):
        rig = Rig(Config(mode=mode, tether=tether))
        rig.engage()
        yaw_err, pos_err, leads = [], [], []
        for _ in range(72):
            c = rig.frame(stick(GRIP, right_x=-1.0))
            pe, ye = riding_error(rig, c)
            yaw_err.append(ye)
            pos_err.append(pe)
            leads.append(hint_lead(rig, c)[1])
        res[tether] = (max(yaw_err), max(pos_err), max(leads), rig)
    ye, pe, lead, rig = res[True]
    assert ye < 3e-3 and pe < 3e-3, (ye, pe)                   # exactly the base's yaw (but the row's prediction)
    assert 0.05 < lead <= 0.1 + 3e-3                           # the heading reference leads: the base keeps turning
    assert rig.base.yaw - 0.3 > 0.4
    assert res[False][0] > 0.15                                # before: the arm swung ahead with the reference
    print(f"{mode}: arm yaw off the base, 1 s full yaw: before {res[False][0]:.3f} rad, after {ye:.4f} rad")


# -- 5: the clutch criterion: a hand delta maps exactly in the commanded base frame -------------------------------

@pytest.mark.parametrize("mode,s", [("third", stick(GRIP, (0.4, 1.0))), ("first", stick(GRIP, (0.4, 1.0), -1.0))])
def test_a_hand_delta_while_driving_maps_exactly_in_the_commanded_base_frame(mode, s):
    d = np.array([0.1, 0.05, 0.03])                            # operator FLU: forward, left, up
    out = {}
    for tether in (True, False):
        runs = []
        for moved in (False, True):
            rig = Rig(Config(mode=mode, tether=tether))
            rig.engage()
            rig.run(s, 0.5)
            for k in range(1, 21):
                c = rig.frame(hand_moved(s, d * k / 20) if moved else s)
            rig.run(hand_moved(s, d) if moved else s, 0.1)
            c = rig.cmd
            b = rig.base.applied()
            runs.append(b.to_local(*ee_pose(c))[0])
        out[tether] = runs[1] - runs[0]
    np.testing.assert_allclose(out[True], d, atol=1e-3)        # direction and magnitude, relative to the robot
    err_before = np.linalg.norm(out[False] - d)
    print(f"{mode}: hand delta error in the base frame: before {err_before * 1e3:.1f} mm, "
          f"after {np.linalg.norm(out[True] - d) * 1e3:.2f} mm")
    if mode == "first":
        assert err_before > 0.01                               # before: rotated by the heading reference's lead


# -- 6: head walking, the stick idle: as before --------------------------------------------------------------------

def test_head_walking_with_the_stick_idle_is_unchanged():
    out = {}
    for tether in (True, False):
        rig = Rig(Config(tether=tether))
        rig.engage()
        cs = []
        for k in range(1, 150):                                # walk forward and turn, the hand with the head
            s = replace(GRIP, head_pos=(min(k, 100) * 0.006, 0.0, 0.0), head_yaw=0.002 * min(k, 120))
            cs.append(rig.frame(s))
        out[tether] = (cs, rig)
    assert out[True][0] == out[False][0]
    assert np.linalg.norm(out[True][1].base.p - [1.0, 2.0]) > 0.3
    assert out[True][1].core.eng.tether == {"xy": "idle", "yaw": "idle"}


# -- 7: stale or missing robot state: the offset freezes -----------------------------------------------------------

@pytest.mark.parametrize("mode", ["third", "first"])
def test_stale_or_missing_robot_state_freezes_the_offset(mode):
    rig = Rig(Config(mode=mode, tether=True))
    rig.engage()
    rig.run(FWD, 0.5)
    rig.rows_on = False                                        # the stream stops
    snap = lambda: (rig.core.offset.copy(), rig.core.eng.correction.copy())   # noqa: E731
    rig.run(FWD, 0.3)                                          # up to robot_stale_s: still tethered to the row
    o0 = snap()
    for k in range(30):
        rig.frame(FWD, robot=None if k % 2 else "row")         # missing (no row this frame) or stale
        o = snap()
        np.testing.assert_array_equal(o[0], o0[0])
        np.testing.assert_array_equal(o[1], o0[1])
    rig.rows_on = True                                         # back: tethered again, within the bound
    c = rig.run(FWD, 0.2)[-1]
    assert hint_lead(rig, c)[0] <= 0.05 + 2e-3
    assert not np.array_equal(snap()[0 if mode == "third" else 1], o0[0 if mode == "third" else 1])


# -- 8: two hands: the base handover stays continuous --------------------------------------------------------------

def test_two_hands_the_base_handover_is_continuous_with_the_tether():
    hands = {"left": "arm_left", "right": "arm_right"}
    rig = Rig(Config(hand="right", ee="arm_right", hands=hands, tether=True))
    rig.engage(GRIP)
    rig.run(stick(GRIP, (0.0, 1.0)), 0.3)
    rig.run(stick(BOTH, (0.0, 1.0)), 0.5)                      # the left engages while driving
    assert rig.core.owner == "right"
    prev = rig.cmd["base"]["target"]["pose"]
    left_only = replace(stick(BOTH, (0.0, 1.0)), right=replace(BOTH.right, grip=0.0, stick=(0.0, 0.0)))
    hint = []
    for _ in range(36):
        c = rig.frame(left_only)
        hint.append(c["base"]["target"]["pose"])
        assert riding_error(rig, c, "arm_left")[0] <= 0.05 + 0.03   # the left engaged at speed: its own anchor
    assert rig.core.owner == "left"
    jump = math.hypot(hint[0]["x"] - prev["x"], hint[0]["y"] - prev["y"])
    assert jump <= V * DT + 0.01, jump                          # one frame of motion, give or take a centimetre
    assert hint_lead(rig, rig.cmd)[0] <= 0.05 + 2e-3


# -- 9: tether off: the instant offset as before --------------------------------------------------------------------

def test_tether_off_is_the_raw_stick_integral():
    rig = Rig(Config(tether=False, headless=False))
    rig.engage()
    for _ in range(72):
        c = rig.frame(stick(GRIP, (0.0, 1.0), -0.5))
    t = rig.cmd["base"]["target"]
    o = rig.core.offset
    assert np.linalg.norm(o[:2]) > 0.6 and o[2] == pytest.approx(0.5 * W * 72 * DT, rel=1e-6)
    assert rig.core.eng.tether == {"xy": "idle", "yaw": "idle"} and rig.core.status.tether is None
    # the EE rides rigidly on the hint (the reference), not on the commanded base
    rel = F.PlanarFrame(t["pose"]["x"], t["pose"]["y"], t["heading"]["yaw"]).to_local(*ee_pose(c))
    np.testing.assert_allclose(rel[0], EE_REL[0], atol=1e-9)


def test_tethered_target_and_hint_share_this_frames_offset():
    """Tethered: the EE target is the engagement's EE on the hint's position with the arm's yaw (the hint's minus
    the stick's yaw share), in every frame."""
    rig = Rig(Config(tether=True))
    rig.engage()
    for k in range(90):
        c = rig.frame(stick(GRIP, (0.5 * math.cos(k / 9), 1.0), right_x=-math.sin(k / 7)))
        t = c["base"]["target"]
        frame = F.PlanarFrame(t["pose"]["x"], t["pose"]["y"], t["heading"]["yaw"] - rig.core.eng.arm_yaw)
        rel = frame.to_local(*ee_pose(c))
        np.testing.assert_allclose(rel[0], EE_REL[0], atol=1e-9)
        np.testing.assert_allclose(rel[1], EE_REL[1], atol=1e-9)
    st = rig.core.status.tether
    assert st["xy"] == "active" and st["lead_m"] <= 0.05 + 1e-9 and abs(st["lead_rad"]) <= 0.1 + 1e-9


def test_engaging_while_the_base_still_brakes_follows_it():
    """Gripped while the base still moves from a released drive: the capture frame is the command (engaging moves
    nothing), then the offset follows the braking base (bound zero), and the base stops where its braking puts it."""
    rig = Rig(Config(tether=True))
    rig.run(IDLE, 0.1)
    rig.run(stick(IDLE, (0.0, 1.0)), 1.0)                      # released driving
    c = rig.frame(GRIP)                                        # grip, the stick let go
    np.testing.assert_allclose(ee_pose(c)[0], rig.row.ee_p, atol=1e-12)
    assert rig.core.eng.tether["xy"] == "braking"
    n0 = len(rig.base.track)
    cs = rig.run(GRIP, 1.5)
    fwd = np.array([math.cos(0.3), math.sin(0.3)])
    along = [float(p @ fwd) for p, _ in rig.base.track[n0 - 1:]]
    assert min(np.diff(along)) >= -1e-9 and np.linalg.norm(rig.base.v) < 1e-4
    assert hint_lead(rig, cs[-1])[0] < 2e-3
    assert rig.core.eng.tether["xy"] == "idle"


def test_walking_and_the_stick_together_keep_the_walking_lead_and_settle():
    """Walking forward (0.3 m/s, the hand with the head) while the stick drives sideways (0.4 m/s): walking's share
    of the lead is the operator's and stays (credited down only by the base's progress towards it), the stick's share
    is bounded; after both stop the braking ends (no chase of a held walking lead) and the base stops on the
    reference, the walk realised as without the tether."""
    fwd = np.array([math.cos(0.3), math.sin(0.3)])
    out, shares, walks = {}, [], []
    for tether in (True, False):
        rig = Rig(Config(tether=tether))
        rig.engage()
        for k in range(1, 290):
            x = 0.3 * min(k, 150) * DT
            s = replace(GRIP, head_pos=(x, 0.0, 0.0), right=replace(GRIP.right, pos=(0.35 + x, -0.2, -0.35)))
            c = rig.frame(stick(s, (0.5, 0.0) if 30 < k < 180 else (0.0, 0.0)))
            if tether:
                shares.append(float(np.hypot(*rig.core.eng.tether_s[:2])))
                walks.append(float(np.hypot(*rig.core.eng.tether_w[:2])))
        out[tether] = (rig, c)
    rig, c = out[True]
    assert max(shares) <= 0.05 + 1e-9
    assert max(walks) > 0.03                                   # the walking lead was not dragged back
    assert np.linalg.norm(rig.base.v) < 1e-3 and rig.core.eng.tether == {"xy": "idle", "yaw": "idle"}
    assert hint_lead(rig, c)[0] < 2e-3
    # the walk realised as without the tether, up to the walk's own braking overshoot (0.3²/2 m at 1 m/s²), which the
    # stick channel absorbs while it brakes too (no snap-back) where the untethered reference pulls it back
    assert abs((rig.base.p - out[False][0].base.p) @ fwd) < 0.045


def test_read_at_engagement_and_the_node_toggle():
    """``Config.tether`` is read at each grip (toggling mid-engagement changes nothing until the next); the node:
    config.tether, POST /tether/{on,off} (an event), /state teleop.tether."""
    cfg = Config(tether=True)
    rig = Rig(cfg)
    rig.engage()
    cfg.tether = False
    rig.run(FWD, 0.5)
    assert rig.core.eng.tether_on and rig.core.status.tether["xy"] == "active"
    rig.run(IDLE, 0.3)
    rig.frame(GRIP)
    assert not rig.core.eng.tether_on and rig.core.status.tether is None

    from fastapi.testclient import TestClient

    from robot_utils.nodes.quest_teleop.node import QuestTeleopNode
    from robot_utils.nodes.quest_teleop.server import create_app
    node = QuestTeleopNode("http://127.0.0.1:9", quest_host="127.0.0.1", quest_port=9)
    st = node.state()
    assert st["config"]["tether"] is False and st["teleop"]["tether"] is None      # off by default (2026-11-26)
    c = TestClient(create_app(node, start=False))
    assert c.post("/tether/maybe").status_code == 400
    assert c.post("/tether/on").json() == {"tether": True}
    assert c.post("/tether/off").json() == {"tether": False}
    assert not node.cfg.tether and "stick tether off" in str(node.events.list()[-1])
    assert node.state()["config"]["tether"] is False
