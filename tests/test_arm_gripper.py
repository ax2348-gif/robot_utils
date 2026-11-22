"""The gripper's squeeze limit applies in contact only (gripper.holding / detect).

A 1-DOF plant of the gripper (the arm SimBus's reflected inertia) under the motor's own MIT law, integrated at 1 ms
between the node's 10 ms ticks, the command held over a tick (as the motor does), feedback optionally a tick or two
late (the real bus). Objects: a rigid stop (the squeeze is the motor torque pressing on it) or a stiff spring (the
squeeze includes the jaws' momentum at impact). The uncapped MIT law in the same harness is the reference."""
import math
from collections import deque

import numpy as np
import pytest

from robot_utils.nodes.arm import config as C
from robot_utils.nodes.arm.bus import SimBus
from robot_utils.nodes.arm.gripper import Contact, holding

P = C.load_profile("soft")
KP, KD = P.g_kp, P.g_kd
DT = 0.01
CFG = C.GripperContact()


def run(goal, max_force=C.DEFAULT_MAX_FORCE, T=1.0, gap0=C.GAP_MAX, obj=None, k_obj=None, law="contact",
        delay=0, friction=0.0, noise=0.0, st=None, v0=0.0, c_obj=0.0):
    """``goal``: m gap or a function of t. ``obj``: m gap of an object (rigid, or a spring of ``k_obj`` Nm/rad
    of motor rotation, with losses ``c_obj`` Nm s/rad). Returns the per-tick history (arrays) and the detector."""
    rng = np.random.default_rng(0)
    I = SimBus.G_INERTIA
    th, v = C.gap_to_rad(gap0), v0
    th_obj = None if obj is None else C.gap_to_rad(obj)
    st = st if st is not None else Contact()
    fb = deque([(th, v)] * (delay + 1), maxlen=delay + 1)
    h = {k: [] for k in ("t", "gap", "squeeze", "squeeze_max", "contact", "limiting", "tau")}
    for k in range(int(round(T / DT))):
        t = k * DT
        g = goal(t) if callable(goal) else goal
        th_r, v_r = fb[0]
        v_r += noise * rng.standard_normal()
        if law == "raw":
            q, ff, contact, limiting = C.gap_to_rad(g), 0.0, False, False
        else:
            gc = holding(g, max_force, th_r, th_r, v_r, KP, KD, t, st, CFG)
            q, ff, contact, limiting = gc.q, gc.ff, gc.contact, gc.limiting
        smax, tau = 0.0, 0.0
        for _ in range(10):
            tau = KP * (q - th) + KD * (0.0 - v) + ff
            f_obj = 0.0
            if th_obj is not None and k_obj is not None and th > th_obj:
                f_obj = max(k_obj * (th - th_obj) + c_obj * v, 0.0)
            fr = friction * math.copysign(1.0, v) if abs(v) > 1e-6 else max(-friction, min(friction, tau - f_obj))
            v += (tau - f_obj - fr) / I * 0.001
            th += v * 0.001
            sq = f_obj
            if th_obj is not None and k_obj is None and th >= th_obj:      # rigid: pressed with the motor torque
                th, v = th_obj, min(v, 0.0)
                sq = max(tau, 0.0)
            if th > 0.0:
                th, v = 0.0, min(v, 0.0)
            if th < C.GRIPPER_OPEN_RAD:
                th, v = C.GRIPPER_OPEN_RAD, max(v, 0.0)
            smax = max(smax, C.force_of_torque(sq))
        fb.append((th, v))
        h["t"].append(t + DT)
        h["gap"].append(C.rad_to_gap(th))
        h["squeeze"].append(C.force_of_torque(sq))
        h["squeeze_max"].append(smax)
        h["contact"].append(contact)
        h["limiting"].append(limiting)
        h["tau"].append(tau)
    return {k: np.array(x) for k, x in h.items()}, st


def t_reach(h, gap, tol=0.0005):
    """First time the gap is within tol of ``gap`` and stays there."""
    ok = np.abs(h["gap"] - gap) <= tol
    bad = np.nonzero(~ok)[0]
    i = 0 if len(bad) == 0 else bad[-1] + 1
    assert i < len(ok), "never settled"
    return h["t"][i]


@pytest.mark.parametrize("delay,friction", [(0, 0.0), (1, 0.0), (2, 0.0), (1, 0.03)])
def test_free_close_from_fully_open_is_as_fast_as_the_raw_mit_law(delay, friction):
    """Never in contact in free air (no false stall at the start from rest, even with late feedback), and as fast
    as the uncapped law: within 10 %."""
    for goal in (0.0, 0.010, 0.025):
        raw, _ = run(goal, law="raw", delay=delay, friction=friction)
        new, _ = run(goal, delay=delay, friction=friction)
        assert not new["contact"].any() and not new["limiting"].any()
        tr, tn = t_reach(raw, goal, 0.001), t_reach(new, goal, 0.001)
        assert tn <= 1.10 * tr + 1e-9, (goal, tn, tr)


def test_the_old_cap_everywhere_was_slower():
    """What the redesign fixes: capping the law whenever it asks for more than the limit (the old law) slows the
    free close from fully open (the cap binds over most of the stroke)."""
    raw, _ = run(0.0, law="raw")
    st = Contact(on=True)                        # the cap held on for the whole stroke, as the old law did
    st_cfg = C.GripperContact(release=1.0)       # never released
    th, v, t, gaps = C.GRIPPER_OPEN_RAD, 0.0, 0.0, []
    for k in range(100):
        gc = holding(0.0, C.DEFAULT_MAX_FORCE, th, th, v, KP, KD, k * DT, st, st_cfg)
        for _ in range(10):
            tau = KP * (gc.q - th) - KD * v + gc.ff
            v += tau / SimBus.G_INERTIA * 0.001
            th = min(th + v * 0.001, 0.0)
        gaps.append(C.rad_to_gap(th))
    old = {"gap": np.array(gaps), "t": DT * np.arange(1, 101)}
    assert t_reach(old, 0.0, 0.001) > 1.3 * t_reach(raw, 0.0, 0.001)


@pytest.mark.parametrize("max_force", [10.0, C.DEFAULT_MAX_FORCE, C.HARD_MAX_FORCE])
@pytest.mark.parametrize("delay", [0, 1])
def test_steady_squeeze_on_a_rigid_object_is_max_force(max_force, delay):
    h, st = run(0.0, max_force, T=1.0, obj=0.040, delay=delay)       # the law alone: 156 N
    assert st.on and h["contact"][-20:].all() and h["limiting"][-20:].all()
    assert abs(h["gap"][-1] - 0.040) < 1e-6
    assert np.all(np.abs(h["squeeze_max"][-50:] - max_force) < 0.01 * max_force + 0.05), h["squeeze_max"][-50:]


def test_steady_squeeze_on_a_stiff_spring_object_is_max_force():
    k = 20.0                                       # Nm/rad: 50 N squeezes it 2.5 mm
    h, st = run(0.0, 50.0, T=1.5, obj=0.020, k_obj=k)
    assert st.on
    assert np.all(np.abs(h["squeeze"][-50:] - 50.0) < 1.0), h["squeeze"][-50:]


@pytest.mark.parametrize("delay", [0, 1, 2])
def test_impact_transient_is_bounded_in_time_and_size(delay):
    """First contact on an unknown object: the free law's torque (at most the raw MIT law's) for no longer than
    stall_s + 2 ticks after the jaws stop (plus the feedback delay), then max_force."""
    goal, obj, F = 0.0, 0.020, 50.0
    h, _ = run(goal, F, T=1.0, obj=obj, delay=delay)
    raw_squeeze = C.force_of_torque(KP * (C.gap_to_rad(goal) - C.gap_to_rad(obj)))     # 78 N
    assert h["squeeze_max"].max() <= raw_squeeze + 1e-6
    over = np.nonzero(h["squeeze_max"] > F * 1.01)[0]
    first_touch = np.nonzero(np.abs(h["gap"] - obj) < 1e-9)[0][0]
    t_over = (over[-1] - first_touch + 1) * DT
    assert t_over <= CFG.stall_s + (2 + delay) * DT + 1e-9, t_over


@pytest.mark.parametrize("delay", [0, 1])
def test_impact_on_a_stiff_lossy_object(delay):
    """A stiff object with losses (50 N squeezes it 0.25 mm): the impact peak is the jaws' momentum, the same as
    under the raw law (no law can take it back once they arrive at full speed); the squeeze then settles at the
    limit, where the raw law keeps its 78 N."""
    kw = dict(T=1.0, obj=0.020, k_obj=200.0, c_obj=0.5, delay=delay)
    h, st = run(0.0, 50.0, **kw)
    raw, _ = run(0.0, law="raw", **kw)
    assert h["squeeze_max"].max() <= raw["squeeze_max"].max() + 1e-6
    assert st.on and np.all(np.abs(h["squeeze_max"][-50:] - 50.0) < 1.0)
    assert abs(raw["squeeze"][-1] - 78.1) < 1.0


def test_regrasp_of_the_same_object_never_exceeds_max_force():
    """Loosen by 2 mm (within `approach` of the remembered contact) and squeeze again: capped all the way."""
    obj, F = 0.020, 30.0
    h, st = run(0.0, F, T=0.6, obj=obj, delay=1)
    assert st.on
    goal = lambda t: obj + 0.002 if t < 0.3 else 0.0             # noqa: E731
    gap0 = h["gap"][-1]
    h2, st = run(goal, F, T=0.8, gap0=gap0, obj=obj, delay=1, st=st)
    assert h2["gap"][29] > obj + 0.0015                            # it did open
    assert h2["squeeze_max"][31:].max() <= F * 1.01 + 1e-6, h2["squeeze_max"][31:].max()
    assert st.on and abs(h2["squeeze"][-1] - F) < 0.5


def test_opening_is_never_limited():
    """From a held squeeze at the limit, opening follows exactly the raw MIT law; from closed, too."""
    obj, F = 0.020, 20.0
    _, st = run(0.0, F, T=0.6, obj=obj)
    assert st.on
    for st0, gap0 in ((st, obj), (Contact(), 0.0)):
        new, _ = run(C.GAP_MAX, F, T=0.5, gap0=gap0, st=st0)
        raw, _ = run(C.GAP_MAX, law="raw", T=0.5, gap0=gap0)
        assert np.max(np.abs(new["gap"] - raw["gap"])) < 1e-9
        assert not new["limiting"].any()


@pytest.mark.parametrize("k_obj", [None, 20.0])
def test_no_chatter_at_the_contact_boundary(k_obj):
    """A slow teleop squeeze through the limit and back, and a goal parked right at the boundary, with noisy
    velocity feedback: one entry into contact, one exit, no toggling."""
    obj, F = 0.020, 20.0
    e_lim = C.torque_of_force(F) / KP * C.GAP_PER_RAD                # m of gap: the law alone reaches F (5.1 mm)
    # goal ramps from the object's surface to the boundary and slightly past, dwells, then back open
    edge = obj - e_lim

    def goal(t):
        if t < 0.5:
            return obj + 0.002 - (0.002 + e_lim + 0.0005) * t / 0.5
        if t < 1.5:
            return edge - 0.0005
        return min(obj + 0.002, edge - 0.0005 + (t - 1.5) * 0.01)
    h, _ = run(goal, F, T=2.5, gap0=obj + 0.002, obj=obj, k_obj=k_obj, delay=1, noise=0.05)
    c = h["contact"].astype(int)
    assert c[100:150].all()                                         # in contact while parked past the limit
    assert np.count_nonzero(np.diff(c)) == 2, np.nonzero(np.diff(c))
    # parked exactly at the boundary (the law alone asks for the limit): no toggling either
    h, _ = run(lambda t: edge if t > 0.2 else obj + 0.002, F, T=2.0, gap0=obj + 0.002, obj=obj, k_obj=k_obj,
               delay=1, noise=0.05)
    assert np.count_nonzero(np.diff(h["contact"].astype(int))) <= 1


def test_object_slipping_out_lets_it_close_and_forgets_the_contact():
    obj, F = 0.020, 30.0
    h, st = run(0.0, F, T=0.5, obj=obj)
    assert st.on and st.at is not None
    h2, st = run(0.0, F, T=1.0, gap0=h["gap"][-1], st=st)             # the object is gone
    assert abs(h2["gap"][-1]) < 0.0005 and not st.on and st.at is None


def test_settings_from_arm_yaml(tmp_path):
    p = tmp_path / "arm.yaml"
    p.write_text("gripper: {stall_vel: 0.8, stall_ms: 40, contact_err_mm: 1, release_mm: 2, approach_mm: 0, "
                 "forget_mm: 5}\n")
    g = C.load_settings(p).gripper
    assert (g.stall_vel, g.stall_s, g.contact_err, g.release, g.approach, g.forget) == \
        pytest.approx((0.8, 0.04, 0.001, 0.002, 0.0, 0.005))
    assert C.load_settings(tmp_path / "none.yaml").gripper == C.GripperContact()
    p.write_text("gripper: {stall_ms: 1}\n")
    with pytest.raises(ValueError):
        C.load_settings(p)
