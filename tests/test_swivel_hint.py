"""The operator's swivel hint, WBC side (docs/wbc.md "The operator's elbow: the swivel hint"): the incremental swivel
(swivel_geom.py: parallel transport, no fixed zero, nothing singular with the shoulder-wrist line vertical); the hint
{seq, dpsi, elbow, conf} anchored at the robot's commanded swivel on each engagement, null space only (the elbow
moves, the hand stays); no jump on a new seq, a dropout or a return; the fades (the operator's and the robot's elbow
extension, conf); a scripted dpsi past a pull-up-top IK-branch stall; no hint = identical; the datagram field (its
validation and drop count), the stream columns and /state."""
import json
import math
import tempfile

import numpy as np
import pytest

from robot_utils.nodes.wbc import swivel_geom as SG

pin = pytest.importorskip("pinocchio")

BENT = 2.0                  # rad: an operator elbow well bent (flexion ~65 deg: the full weight)


def _rot(axis, a):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


# -- the geometry ---------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_a_step_is_the_elbow_rotation_about_the_shoulder_wrist_line(sign):
    S, W = np.array([0.0, 0.2, 1.0]), np.array([0.35, 0.1, 0.8])
    E0 = np.array([0.15, 0.25, 0.75])
    a = (W - S) / np.linalg.norm(W - S)
    for d in (0.01, 0.3, -0.7, 2.0):
        E1 = S + _rot(a, d) @ (E0 - S)
        assert SG.step(S, E0, W, S, E1, W, sign) == pytest.approx(sign * d, abs=1e-9)
    # the whole arm turned rigidly about an axis normal to the S-W line (the line tilts, the elbow rides along): no
    # swivel (parallel transport carries the elbow's offset exactly there)
    n = np.cross(a, [0.0, 0.0, 1.0])
    R = _rot(n, 0.6)
    assert SG.step(S, E0, W, R @ S, R @ E0, R @ W, sign) == pytest.approx(0.0, abs=1e-9)


def test_vertical_shoulder_wrist_line_has_no_singularity():
    """The wrist straight below the shoulder (reaching down): the fixed-zero swivel (the world's down projected) is
    undefined there and flips by up to pi across it; the incremental swivel steps smoothly through it."""
    from robot_utils.nodes.wbc.humanlike import swivel_angle
    S = np.array([0.0, 0.0, 1.0])
    acc = SG.Accumulator(1.0)
    fixed, cont = [], []
    for k in range(201):
        u = -0.05 + 0.1 * k / 200                                     # the wrist passes under the shoulder
        W = S + np.array([u, 0.02 * u, -0.45])
        a = (W - S) / np.linalg.norm(W - S)
        n = np.cross(a, [0.0, 1.0, 0.0])
        n /= np.linalg.norm(n)
        E = S + 0.5 * (W - S) + 0.12 * n                              # a bent elbow, rigidly carried
        cont.append(acc.update(S, E, W))
        fixed.append(swivel_angle(S, E, W, 1.0))
    assert np.all(np.isfinite(cont))
    assert np.max(np.abs(np.diff(cont))) < 0.01 and abs(cont[-1]) < 0.02    # no jump, no drift through vertical
    assert np.max(np.abs(np.diff(np.unwrap(fixed)))) > 1.0                  # the fixed zero flips there


def test_flexion_and_fade():
    S, W = np.zeros(3), np.array([0.5, 0.0, 0.0])
    assert SG.flexion(S, np.array([0.25, 0.0, 0.0]), W) == pytest.approx(0.0, abs=1e-6)
    assert SG.flexion(S, np.array([0.0, 0.3, 0.0]), np.array([0.3, 0.3, 0.0])) == pytest.approx(math.pi / 2)
    assert SG.fade(0.0, 0.05, 0.25) == 0.0 and SG.fade(0.3, 0.05, 0.25) == 1.0
    f = [SG.fade(b, 0.05, 0.25) for b in np.linspace(0, 0.3, 61)]
    assert np.all(np.diff(f) >= 0) and np.max(np.abs(np.diff(f))) < 0.04
    assert SG.step(S, np.array([0.25, 0, 0]), W, S, np.array([0.25, 0.1, 0]), W) == 0.0   # straight: no step


# -- the WBC: the hint in the null space ----------------------------------------------------------------------------

@pytest.fixture(scope="module")
def wmodel():
    pytest.importorskip("qpsolvers")
    from robot_utils.nodes.wbc.model import WbcModel, ee_specs, fetch_urdf
    urdf, _ = fetch_urdf("local", "openarm_bimanual")
    return WbcModel(urdf, ee_specs("openarm_bimanual"))


def _oa():
    import test_openarm as T
    return T


HOLD = {"arm_left": [0, 0, 0], "arm_right": [0, 0, 0]}


def _ramp(t, t0, t1):
    return min(max((t - t0) / (t1 - t0), 0.0), 1.0)


@pytest.mark.parametrize("d,gain", [(0.5, 1.0), (-0.6, 1.0), (0.25, 2.0)])
def test_the_hint_moves_the_elbow_and_the_hand_stays(wmodel, d, gain):
    """Hand held still; the operator engages (dpsi 0) and turns d over 0.5 s: the right elbow moves gain d from where
    it was at the engagement (to within 0.02 rad in 3 s), the hand within 0.5 mm, the other arm as without a hint."""
    T = _oa()
    plain = T.Run(wmodel, T.POSTURE, HOLD, 300)
    r = T.Run(wmodel, T.POSTURE, HOLD, 300, cfg_over={"swivel_hint_gain": gain},
              hint_fn=lambda t, run: {"arm_right": (1, d * _ramp(t, 0.2, 0.7), BENT, 1.0)})
    pc = np.array(r.psi_cont["arm_right"])
    assert abs(pc[-1] - pc[0] - gain * d) < 0.02, pc[-1] - pc[0]
    assert max(r.err["arm_right"]) < 5e-4
    assert np.allclose(r.q["arm_left"], plain.q["arm_left"], atol=1e-6)
    pp = np.array(plain.psi_cont["arm_right"])
    assert abs(pp[-1] - pp[0] - gain * d) > 0.25                            # the model alone goes elsewhere
    # (-0.6: from the OpenArm posture re-solved for Anvil's J6 range the model alone swings the elbow -0.27 rad)


def test_ee_tracking_unchanged_while_the_hint_steers(wmodel):
    """Reaches with the operator sweeping the elbow (dpsi +-0.4 rad): the EE error history is the plain run's within
    2 mm at every tick, while the elbow went elsewhere."""
    T = _oa()
    for goals in T.REACHES[:2]:
        plain = T.Run(wmodel, T.POSTURE, goals, 300)
        r = T.Run(wmodel, T.POSTURE, goals, 300,
                  hint_fn=lambda t, run: {e: (7, 0.4 * math.sin(2 * t), BENT, 1.0) for e in wmodel.ees})
        for e in wmodel.ees:
            # within 2 mm of the plain run (measured 1.43 mm at k 5 / rate 2 / speed 1.2: the speed's price)
            assert np.max(np.abs(np.array(r.err[e]) - np.array(plain.err[e]))) < 2e-3, e
            assert r.err[e][-1] < 2e-3
            assert np.max(np.abs(np.array(r.psi[e]) - np.array(plain.psi[e]))) > 0.1


def test_no_jump_on_a_new_seq_a_dropout_or_a_return(wmodel):
    """The operator's cycle, hand held, the arm settled on its model first (3 s):
    3.0 engage (seq 1, dpsi 0), turn +0.5 over 3.5..4.5; 5.5 the input stops (stale after the timeout: with
    ``swivel_hint_hold`` off, fades back to the model, the target held); 6.5 it returns (seq 1, dpsi 0.6: the operator moved meanwhile): re-anchored, nothing
    jumps, then +0.2 more; 7.5 a new engagement (seq 2, dpsi 0) while the hint is in force, turn -0.3; 9.0 stop.
    Engaging moves nothing (< 0.005 rad/s change per tick; the arm's at rest 0.001); a new seq with the elbow still
    moving slows it smoothly; the fades (stale, return) are ramps: each tick's joint velocity change at most 0.06 rad/s and under 1/4 of the
    same cycle's with the weight switched in one tick (swivel_hint_blend 0.01: 0.23-0.29); the weight ramps, the elbow
    follows the operator's changes since each anchoring, the hand stays."""
    T = _oa()

    def fn(t, run):
        if 3.0 <= t < 5.5:
            return {"arm_right": (1, 0.5 * _ramp(t, 3.5, 4.5), BENT, 1.0)}
        if 6.5 <= t < 7.5:
            return {"arm_right": (1, 0.6 + 0.2 * _ramp(t, 6.8, 7.1), BENT, 1.0)}
        if 7.5 <= t < 9.0:
            return {"arm_right": (2, -0.3 * _ramp(t, 7.8, 8.1), BENT, 1.0)}
        return {}
    k = lambda t: int(round(t / 0.01))                  # noqa: E731

    def dq_of(run):                                     # rad/s change per tick
        return np.max(np.abs(np.diff(np.array(run.qd["arm_right"]), axis=0)), axis=1)
    nohold = {"swivel_hint_hold": False}               # this cycle tests the fade-out path (the option)
    r = T.Run(wmodel, T.POSTURE, HOLD, 1100, hint_fn=fn, cfg_over=nohold)
    dq = dq_of(r)
    assert np.max(dq[k(3.0) - 5:k(3.0) + 25]) < 5e-3, np.max(dq[k(3.0) - 5:k(3.0) + 25])   # engaging moves nothing
    # a new seq while in force, the elbow still converging on the old target (0.06 rad behind): re-anchored where the
    # robot is, the target eases there (swivel_hint_tau): the elbow slows over ~0.1 s, never stops in a tick
    qd75 = np.max(np.abs(np.array(r.qd["arm_right"])[k(7.5) - 1]))
    # (0.046 rad/s per tick at k_swivel_hint 5 in a 0.21 rad/s motion: smooth; 0.04 / 0.2 were set at k 3)
    assert np.max(dq[k(7.5) - 5:k(7.5) + 25]) < min(0.06, 0.25 * qd75), (np.max(dq[k(7.5) - 5:k(7.5) + 25]), qd75)
    sw = dq_of(T.Run(wmodel, T.POSTURE, HOLD, 1100, hint_fn=fn, cfg_over={**nohold, "swivel_hint_blend": 0.01}))
    for t0 in (5.8, 6.5, 9.3):                          # stale -> fade out; the return -> re-anchor, fade in
        win = slice(k(t0) - 5, k(t0) + 40)
        # 0.06 and 1/4: the ramp is swivel_hint_blend 0.15 s (0.3 when 0.04 and 1/5 were set; measured 0.21 of the
        # one-tick switch at the return): a ramp, not a step, is the point
        assert np.max(dq[win]) < 0.06 and np.max(dq[win]) < 0.25 * np.max(sw[win]), (t0, np.max(dq[win]),
                                                                                     np.max(sw[win]))
    pc = np.array(r.psi_cont["arm_right"])
    w = np.array(r.hint_w["arm_right"])
    assert np.max(np.abs(np.diff(w))) <= 0.01 / r.cfg.swivel_hint_blend + 1e-9    # ramps, never steps
    assert abs(pc[k(5.4)] - pc[k(3.0)] - 0.5) < 0.03    # followed the operator's 0.5 (0.9 s after)
    assert w[k(5.4)] > 0.99 and w[k(6.4)] < 1e-9        # in force, then faded out after the dropout
    tg = np.array(r.hint_target["arm_right"])
    # re-anchored at the robot's swivel at the return: the target is only the 0.2 after it, the elbow on its way
    assert abs(tg[k(7.4)] - pc[k(6.5)] - 0.2) < 0.005 and abs(pc[k(7.4)] - tg[k(7.4)]) < 0.1
    assert abs(tg[k(8.9)] - pc[k(7.5)] + 0.3) < 0.005    # the new seq: re-anchored there, then -0.3
    assert abs(pc[k(8.9)] - tg[k(8.9)]) < 0.04
    assert w[-1] < 1e-9                                 # faded out at the end
    assert np.max(np.abs(np.diff(pc))) <= r.cfg.swivel_hint_rate * 0.01 * 1.5    # the swivel never jumps
    assert max(r.err["arm_right"]) < 5e-4


def test_the_extension_fades(wmodel):
    """The weight vanishes toward either arm's full extension and with low confidence: the robot's elbow at 3 deg
    (the hint has no effect: the run is the plain one); the operator's elbow straight (pi - 3 deg) or conf 0.1: no
    weight; halfway along each fade (20 deg of flexion, conf 0.45): half the weight; well bent and confident: 1."""
    T = _oa()
    q = {e: np.array(T.POSTURE[e], float) for e in wmodel.ees}
    lo, _ = wmodel.arm_limits("arm_right")
    q["arm_right"][3] = max(math.radians(3.0), lo[3] + 0.01)
    plain = T.Run(wmodel, q, HOLD, 100)
    r = T.Run(wmodel, q, HOLD, 100, hint_fn=lambda t, run: {"arm_right": (1, 0.8 * _ramp(t, 0, 0.3), BENT, 1.0)})
    assert max(r.hint_w["arm_right"]) < 0.02
    assert np.allclose(r.q["arm_right"], plain.q["arm_right"], atol=2e-3)

    def w_end(elbow, conf):
        run = T.Run(wmodel, T.POSTURE, HOLD, 60, hint_fn=lambda t, run: {"arm_right": (1, 0.0, elbow, conf)})
        return run.hint_w["arm_right"][-1]
    assert w_end(math.pi - math.radians(3), 1.0) == 0.0
    assert w_end(BENT, 0.1) == 0.0
    assert w_end(math.pi - math.radians(20), 1.0) == pytest.approx(0.5, abs=0.01)
    assert w_end(BENT, 0.45) == pytest.approx(0.5, abs=0.01)
    assert w_end(BENT, 1.0) > 0.99


def test_the_hint_steers_with_the_shoulder_wrist_line_vertical(wmodel):
    """The right arm hanging with its wrist straight below the shoulder, elbow bent: the hint still moves the elbow
    (no singular gradient, the swivel finite and continuous), the hand stays."""
    T = _oa()
    m = wmodel
    from robot_utils.nodes.wbc.humanlike import Humanlike
    from robot_utils.nodes.wbc.spec import make_config
    h = Humanlike(m, make_config({}))
    qb = {e: np.array(T.POSTURE[e], float) for e in m.ees}
    qb["arm_right"] = np.array([-0.568, 0.73, -0.796, 1.758, 0.286, -0.17, 0.0])   # the wrist under the shoulder
    h.prepare(m.config((0, 0, 0), qb))
    a = h.arms["arm_right"]
    Sw, Ew, Wr = h._p(a.S)[0], h._p(a.E)[0], h._p(a.W)[0]
    assert math.degrees(math.acos((Sw - Wr)[2] / np.linalg.norm(Sw - Wr))) < 3.0
    assert SG.flexion(Sw, Ew, Wr) > math.radians(60)
    r = T.Run(m, qb, HOLD, 300, hint_fn=lambda t, run: {"arm_right": (1, 0.4 * _ramp(t, 0, 0.4), BENT, 1.0)})
    pc = np.array(r.psi_cont["arm_right"])
    assert np.all(np.isfinite(pc)) and abs(pc[-1] - pc[0] - 0.4) < 0.03, pc[-1] - pc[0]
    assert np.max(np.abs(np.diff(pc))) < 0.03
    assert max(r.err["arm_right"]) < 1e-3


def _pullup(m, T, goal, arm, hint_fn=None, P=4.0, steps=800):
    """The hand brought from the nominal posture to ``goal`` along a P s path, as an operator would: the final miss
    (mm, deg) and the run."""
    q0 = {e: np.array(T.POSTURE[e], float) for e in m.ees}
    T0 = {e: m.fk(m.config((0, 0, 0), q0), e) for e in m.ees}

    def goal_fn(t):
        s = _ramp(t, 0.0, P)
        s = s * s * (3 - 2 * s)
        R0 = T0[arm].rotation
        return {**T0, arm: pin.SE3(R0 @ pin.exp3(s * pin.log3(R0.T @ goal.rotation)),
                                   (1 - s) * T0[arm].translation + s * goal.translation)}
    run = T.Run(m, T.POSTURE, T0, steps, absolute=True, goal_fn=goal_fn, hint_fn=hint_fn)
    Tq = m.fk(m.config((0, 0, 0), {**q0, arm: run.q[arm]}), arm)
    return (1e3 * float(np.linalg.norm(Tq.translation - goal.translation)),
            math.degrees(float(np.linalg.norm(pin.log3(goal.rotation @ Tq.rotation.T))))), run


def test_a_scripted_dpsi_gets_past_a_pullup_top_stall(wmodel):
    """A pull-up-top grip (the workspace study's probe, target 11: the left hand 5 cm ahead of and 10 cm above its
    shoulder, overhand, palm forward; reachable within the WBC's limits), the hand brought there along a 4 s path: the
    plain QP stops at joint limits 126 mm short; the operator raising the elbow (dpsi +1 rad over the first second, as
    a person does to grip overhead) gets the hand there, inside the limits. docs/wbc.md has the probe's other cases."""
    T = _oa()
    m = wmodel
    goal = pin.SE3(pin.Quaternion(0.70711, 0.70711, 0.0, 0.0).normalized().toRotationMatrix(),
                   np.array([0.17, 0.2325, 1.174]))
    (e0, _), _ = _pullup(m, T, goal, "arm_left")
    (e1, r1), steered = _pullup(m, T, goal, "arm_left",
                                hint_fn=lambda t, run: {"arm_left": (1, 1.0 * _ramp(t, 0.0, 1.0), BENT, 1.0)})
    assert e0 > 100, e0                                                      # the plain QP: stuck 126 mm short
    assert e1 < 5 and r1 < 3.0, (e1, r1)                                     # steered: there
    lo, hi = m.arm_limits("arm_left")
    assert np.all(steered.q["arm_left"] >= lo - 1e-6) and np.all(steered.q["arm_left"] <= hi + 1e-6)


def test_no_hint_is_identical(wmodel):
    """No hint, or one with no weight (conf 0, or the operator's arm straight): the joints are identical tick for
    tick (the term is not formed)."""
    T = _oa()
    for goals in T.REACHES[:2]:
        a = T.Run(wmodel, T.POSTURE, goals, 150)
        for h in ((3, 1.0, BENT, 0.0), (3, 1.0, math.pi, 1.0)):
            b = T.Run(wmodel, T.POSTURE, goals, 150,
                      hint_fn=lambda t, run, h=h: {e: (h[0], h[1] * _ramp(t, 0, 0.5), *h[2:]) for e in wmodel.ees})
            for e in wmodel.ees:
                assert np.array_equal(np.array(a.q_hist[e]), np.array(b.q_hist[e])), (e, h)


# -- the node: datagram field, stream columns, /state ---------------------------------------------------------------

def test_parse_swivel_hint():
    from robot_utils.nodes.wbc.node import parse_swivel_hint
    ok = {"seq": 4, "dpsi": -0.2, "elbow": 2.0, "conf": 0.8}
    assert parse_swivel_hint(ok) == (4, -0.2, 2.0, 0.8)
    for bad in ({**ok, "seq": 1.5}, {**ok, "seq": True}, {**ok, "dpsi": math.nan}, {**ok, "dpsi": "x"},
                {**ok, "elbow": -0.1}, {**ok, "elbow": 3.3}, {**ok, "conf": 1.2}, {**ok, "extra": 1},
                {k: v for k, v in ok.items() if k != "conf"}, [1, 2], 0.3):
        with pytest.raises(ValueError):
            parse_swivel_hint(bad)


def test_node_takes_the_swivel_field_and_publishes_the_swivel(wmodel):
    from robot_utils.nodes.wbc.node import WbcNode
    from robot_utils.nodes.wbc.targets import R_to_quat

    class Clock:
        def __init__(self, t):
            self.t = t

        def __call__(self):
            return self.t
    d = tempfile.mkdtemp(prefix="wbcsw")
    clock = Clock(1000.0)
    node = WbcNode(wmodel, arm=None, swerve=None, estimator="est", clock=clock, udp_host=None,
                   unix_socket=d + "/t.sock", robot="openarm_bimanual")
    assert {"arm_right_swivel", "arm_right_swivel_w", "arm_left_swivel", "arm_left_swivel_w"} <= set(node.fields)
    tok, _ = node.acquire("t")
    node.enable(tok)
    sess = node.open_session(tok)["session"]
    seq = [0]

    def row():
        return dict(zip(node.fields, node.hub.recent(1, "tick")[0]))

    def step(n, swivel=None):
        for _ in range(n):
            clock.t += 0.01
            node.feed_estimator({"t": clock.t, "x": 0.0, "y": 0.0, "theta": 0.0, "reset_id": 3.0, "odom_ok": 1.0})
            seq[0] += 1
            ees = {}
            for e in wmodel.ees:
                T = node.model.fk(node.model.config((0, 0, 0), node.q_cmd), e)
                ees[e] = {"pos": list(T.translation), "quat": list(R_to_quat(T.rotation))}
            sw = swivel(seq[0]) if callable(swivel) else swivel
            if sw is not None:
                ees["arm_right"]["swivel"] = sw
            node.receiver.handle(json.dumps({"type": "target", "session": sess, "seq": seq[0], "t": None,
                                             "armed": True, "arm_seq": 1, "ees": ees}).encode())
            node.step()
    step(5)
    r0 = row()
    assert node.armed and math.isfinite(r0["arm_right_swivel"]) and r0["arm_right_swivel_w"] == 0.0
    psi0 = r0["arm_right_swivel"]
    n0 = seq[0]
    step(300, lambda s: {"seq": 9, "dpsi": 0.3 * min((s - n0 - 1) / 30, 1.0), "elbow": BENT, "conf": 1.0})
    r1 = row()
    assert abs(r1["arm_right_swivel"] - (psi0 + 0.3)) < 0.03 and r1["arm_right_swivel_w"] > 0.99
    post = node._arm_snapshot("arm_right", node._last["out"])["posture"]
    h = post["hint"]
    assert post["swivel"] == pytest.approx(r1["arm_right_swivel"], abs=1e-3)
    assert h["seq"] == 9 and h["live"] and h["weight"] > 0.99 and h["anchors"] == 1
    assert h["anchor"] == pytest.approx(psi0, abs=0.01) and h["anchor_dpsi"] == 0.0
    assert h["target"] == pytest.approx(h["anchor"] + 0.3, abs=1e-3)
    assert node._arm_snapshot("arm_left", node._last["out"])["posture"]["hint"] is None
    assert not node.receiver.counts["dropped"]
    # an invalid swivel: the datagram is dropped and counted (as any invalid field), the hint ages
    step(3, {"seq": 9, "dpsi": 0.3, "elbow": 4.0, "conf": 1.0})
    assert node.receiver.counts["dropped"].get("invalid") == 3
    with node.lock:
        assert node._snapshot_locked(clock.t)["session"]["dropped"].get("invalid") == 3     # /state.session.dropped
    step(80)                                                  # no field: stale after the timeout, held
    r2 = row()
    assert r2["arm_right_swivel_w"] > 0.99 and abs(r2["arm_right_swivel"] - r1["arm_right_swivel"]) < 0.02
    node.disable("done")


def test_a_released_hint_holds_the_elbow_and_the_next_grip_continues_from_there(wmodel):
    """swivel_hint_hold (default): the operator turns +0.4, releases (no field: stale), the elbow stays there (not back
    to the model's swivel) with the weight held; a new grip (seq 2, dpsi from 0) turns -0.3 from there, so the
    swivels accumulate across clutches. With the hold off, the release drifts back to the model."""
    T = _oa()

    def fn(t, run):
        if 0.5 <= t < 2.0:
            return {"arm_right": (1, 0.4 * _ramp(t, 0.6, 1.0), BENT, 1.0)}
        if 4.0 <= t < 6.0:
            return {"arm_right": (2, -0.3 * _ramp(t, 4.2, 4.6), BENT, 1.0)}
        return {}
    k = lambda t: int(round(t / 0.01))                  # noqa: E731
    r = T.Run(wmodel, T.POSTURE, HOLD, 800, hint_fn=fn)
    pc, w = np.array(r.psi_cont["arm_right"]), np.array(r.hint_w["arm_right"])
    assert abs(pc[k(1.9)] - pc[k(0.5)] - 0.4) < 0.03                       # followed the first grip
    assert abs(pc[k(3.9)] - pc[k(1.9)]) < 0.02 and w[k(3.9)] > 0.99       # released: held (2 s), weight kept
    assert abs(pc[k(5.9)] - pc[k(0.5)] - 0.1) < 0.03                       # the second grip: -0.3 from there
    assert max(r.err["arm_right"]) < 1e-3
    off = T.Run(wmodel, T.POSTURE, HOLD, 800, hint_fn=fn, cfg_over={"swivel_hint_hold": False})
    po = np.array(off.psi_cont["arm_right"])
    assert abs(po[k(3.9)] - po[k(1.9)]) > 0.1                              # without the hold it drifts back


def test_the_continuous_swivel_ignores_base_motion(wmodel):
    """The arm still on a turning, driving base: the continuous swivel stays put (it is measured in the base frame;
    in the world, parallel transport along the S-W line's turn about the vertical drifted it ~50 deg per 90 deg of
    yaw, and the held hint swung the elbows while driving by stick)."""
    from robot_utils.nodes.wbc.humanlike import Humanlike
    from robot_utils.nodes.wbc.spec import make_config
    T = _oa()
    qa = {e: np.array(T.POSTURE[e], float) for e in wmodel.ees}
    h = Humanlike(wmodel, make_config({}))
    for k in range(91):
        h.prepare(wmodel.config((0.3 * k / 90, -0.2 * k / 90, math.radians(k)), qa))
    assert all(abs(v) < 1e-6 for v in h.psi_acc.values()), h.psi_acc
