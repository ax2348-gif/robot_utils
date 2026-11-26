"""The WBC's model, target buffers and QP controller with pinocchio only (no nodes, no network): reaching with
the base free / locked, the base and arm limits held on every step, the start charge (at rest the base starts only
when moving is worth more than base_start_charge), commanded and hinted base references with their bands, the
manipulability floor (off by default), infeasible solves, chunk blending continuity at seams, staleness, and the
teleop regressions (head sway, hand motion, walking, reach, stick). docs/wbc.md, docs/plans/wbc_v1.md."""
import math

import numpy as np
import pinocchio as pin
import pytest

from robot_utils.nodes.swerve.constants import MODULES
from robot_utils.nodes.wbc.controller import C8, LIMITED, QPController, EEState, Snapshot, oct_norm
from robot_utils.nodes.wbc.model import EESpec, WbcModel, arm_config_limits, fetch_urdf
from robot_utils.nodes.wbc.spec import (Config, Hints, HeadingHint, PoseHint, VelocityHint, make_config,
                                        parse_hints)
from robot_utils.nodes.wbc.targets import TargetBuffer, pose_diff

Q0 = np.array([0.0, -0.6, -0.9, 0.0, 0.5, 0.0])
WHEELS = [(m.x, m.y) for m in MODULES]


@pytest.fixture(scope="module")
def model():
    urdf, _ = fetch_urdf("local")
    return WbcModel(urdf, (EESpec(arm_limits=arm_config_limits()),))


# The base caps these controller tests were measured at (the WBC's first-test defaults; the defaults since
# 2026-11-24 are 0.8 m/s, 0.9 rad/s, 2.0 m/s², wheels 2.0 m/s). At the new defaults the reach-boundary braking
# transient is ~41 mm (not 6.6 mm) and the ramp is twice as steep: the measured numbers below are for these caps.
FIRST_CAPS = dict(max_speed=0.6, max_omega=0.8, max_accel=1.0, wheel_speed=1.0)


class Sim:
    """Perfect tracking: the base moves by the commanded twist, the arm by the commanded step."""

    def __init__(self, model, cfg=None, **kw):
        self.m, self.cfg = model, cfg or Config()
        self.ctl = QPController(model, self.cfg, WHEELS)
        self.base = np.zeros(3)
        self.q = Q0.copy()
        self.v = np.zeros(6)
        self.tw = np.zeros(3)
        self.t = 0.0
        self.buf = TargetBuffer("arm", self.cfg.ff_timeout, self.cfg.blend_tau)
        self.hints = Hints(**kw)
        self.log = []

    def tcp(self):
        return self.m.fk(self.m.config(self.base, {"arm": self.q}), "arm")

    def tcp_twist(self):
        q = self.m.config(self.base, {"arm": self.q})
        self.m.update(q)
        J = self.m.jacobian(q, "arm")
        c, s = math.cos(self.base[2]), math.sin(self.base[2])
        v = np.zeros(self.m.nv)
        v[self.m.base_v] = [c * self.tw[0] - s * self.tw[1], s * self.tw[0] + c * self.tw[1], self.tw[2]]
        v[self.m.arm_v["arm"]] = self.v
        return J @ v

    def step(self, hold=False, velocity_age=math.inf, target=True, hint_age=0.0):
        tg = self.buf.sample(self.t) if target else None
        h = self.hints
        snap = Snapshot(self.t, tuple(self.base), self.tw.copy(),
                        {"arm": EEState(self.q.copy(), self.v.copy(), tg, h.ee("arm", self.cfg), Q0)}, h,
                        hold=hold, velocity_age=velocity_age, hint_age=hint_age)
        out = self.ctl.step(snap)
        self.log.append((self.t, self.tw.copy(), out.base_twist.copy(), self.v.copy(), out.qd["arm"].copy(), out))
        if out.status != "ok":
            return out
        self.tw = out.base_twist.copy()
        c, s = math.cos(self.base[2]), math.sin(self.base[2])
        self.base += np.array([c * self.tw[0] - s * self.tw[1], s * self.tw[0] + c * self.tw[1], self.tw[2]]) * self.cfg.dt
        self.q = self.q + out.dq["arm"]
        self.v = out.qd["arm"].copy()
        self.t += self.cfg.dt
        return out

    def run(self, s, **kw):
        out = None
        for _ in range(int(round(s / self.cfg.dt))):
            out = self.step(**kw)
            assert out.status == "ok", out.reason
        return out

    def ride(self, rel):
        """The released arm's target (the Quest client's): ``rel`` (base-frame position, rotation) on the current
        base pose, with the base's rigid-body velocity at the EE as feedforward."""
        c, s = math.cos(self.base[2]), math.sin(self.base[2])
        Rb = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        p = Rb @ rel[0] + [self.base[0], self.base[1], 0.0]
        vx, vy, w = self.tw
        v = Rb[:2, :2] @ [vx, vy]
        r = p - [self.base[0], self.base[1], 0.0]
        self.buf.set_reactive(self.t, p, Rb @ rel[1], v=[v[0] - w * r[1], v[1] + w * r[0], 0, 0, 0, w])

    def moving(self, since=0):
        """Per logged tick: whether the base was commanded to move."""
        return [bool(np.any(r[2] != 0.0)) for r in self.log[since:]]

    def starts(self, since=0):
        """How often the base started from rest (the commanded twist went from exactly 0 to nonzero)."""
        return sum(1 for r in self.log[since:] if np.all(r[1] == 0.0) and np.any(r[2] != 0.0))

    def check_limits(self, cfg=None):
        """Every step: base speed / yaw rate / twist rate / wheel speeds, arm joint speed and acceleration."""
        cfg = cfg or self.cfg
        dt, tol = cfg.dt, 1e-5
        for t, prev, tw, vprev, qd, out in self.log:
            assert math.hypot(tw[0], tw[1]) <= cfg.max_speed + tol, (t, tw)
            assert oct_norm(tw[:2]) <= cfg.max_speed + tol
            assert abs(tw[2]) <= cfg.max_omega + tol
            for x, y in WHEELS:
                assert math.hypot(tw[0] - tw[2] * y, tw[1] + tw[2] * x) <= cfg.wheel_speed + tol, (t, tw)
            if out.base_fixed not in ("rate fallback",):
                assert math.hypot(*(tw[:2] - prev[:2])) <= cfg.max_accel * dt + tol, (t, prev, tw, out.base_fixed)
                assert abs(tw[2] - prev[2]) <= cfg.max_alpha * dt + tol
            vm, am = self.ctl.caps["arm"]
            assert np.all(np.abs(qd) <= vm + tol), (t, qd)
            assert np.all(np.abs(qd - vprev) <= am * dt + 1e-5), (t, qd - vprev)


def test_model_planar_base_and_tcp(model):
    assert model.nv == 9 and list(model.base_v) == [0, 1, 2]
    from robot_utils.nodes.arm.kin import DEFAULT_TCP_OFFSET
    assert model.ees["arm"].tcp == tuple(DEFAULT_TCP_OFFSET)
    # the base pose moves the TCP rigidly
    T0 = model.fk(model.config((0, 0, 0), {"arm": Q0}), "arm")
    T1 = model.fk(model.config((1.0, 2.0, math.pi / 2), {"arm": Q0}), "arm")
    Rz = pin.utils.rpyToMatrix(0, 0, math.pi / 2)
    assert np.allclose(T1.translation, Rz @ T0.translation + [1.0, 2.0, 0.0], atol=1e-9)
    lo, hi = model.arm_limits("arm")
    assert np.all(lo < Q0) and np.all(Q0 < hi)


def test_reach_with_the_base_free(model):
    s = Sim(model)
    T0 = s.tcp()
    goal = T0.translation + [0.6, 0.2, 0.0]
    s.buf.set_reactive(0.0, goal, T0.rotation)
    s.run(8.0)
    T = s.tcp()
    assert np.linalg.norm(T.translation - goal) < 0.002
    assert np.linalg.norm(pin.log3(T.rotation @ T0.rotation.T)) < 0.01
    assert np.linalg.norm(s.base[:2]) > 0.2               # out of the arm's reach: the base drove
    assert s.starts() == 1                                # one start, one stop: no chatter
    s.run(1.0)
    # converged: back at rest the start charge holds the base at exactly 0 (held is cheaper than free + the charge)
    for r in s.log[-100:]:
        assert np.all(r[2] == 0.0) and r[5].base_fixed == "at rest (start charge)"
        sc = r[5].diag["start_charge"]
        assert sc["free"] + sc["charge"] >= sc["hold"]
    assert not any(r[5].deadband for r in s.log)          # no deadband any more (Output.deadband is always False)
    s.check_limits()


def test_reach_with_the_base_locked(model):
    s = Sim(model, mode="locked")
    T0 = s.tcp()
    goal = T0.translation + [0.05, -0.03, 0.06]
    s.buf.set_reactive(0.0, goal, T0.rotation)
    s.run(4.0)
    assert np.linalg.norm(s.tcp().translation - goal) < 0.002
    assert np.all(s.base == 0.0) and all(np.all(r[2] == 0.0) for r in s.log)
    s.check_limits()


def test_limits_hold_under_demanding_targets(model):
    """A far target behind the base and a heading hint: the base twist saturates; every step stays within the
    speed, yaw-rate, twist-rate and wheel octagons, the arm within its joint speed and acceleration."""
    cfg = make_config({"max_speed": 0.3, "wheel_speed": 0.35, "max_omega": 0.8})
    s = Sim(model, cfg, mode="hint", target_heading=HeadingHint(yaw=2.5, weight=1.0, tol=0.02))
    T0 = s.tcp()
    s.buf.set_reactive(0.0, T0.translation + [-0.8, 0.5, -0.1], T0.rotation)
    s.run(10.0)
    s.check_limits()
    wheel = [max(math.hypot(tw[0] - tw[2] * y, tw[1] + tw[2] * x) for x, y in WHEELS) for _, _, tw, *_ in s.log]
    assert max(wheel) > 0.9 * cfg.wheel_speed * C8           # the wheel rows were the active limit
    # this target, behind and lower, ends with the arm at a joint limit (a local minimum of the velocity-level QP,
    # reported, never a yank); the heading hint yields to the EE task (stage 2)
    assert s.log[-1][5].limited_by == "joint_limit"
    assert s.log[-1][5].diag["limited_ees"] == {"arm": "joint_limit"}       # per arm (the haptics' hand)
    assert s.log[-1][5].diag["err_pos"]["arm"] < 0.1


def test_start_charge_keeps_small_corrections_on_the_arm(model):
    """At rest the base starts only when moving is worth more than base_start_charge: a 1 cm EE correction stays on
    the arm (exactly 0 base twist, decided by the two at-rest solves); a far target starts it once and it stops
    once (no chatter); without the charge the same 1 cm correction does move the base."""
    s = Sim(model)
    T0 = s.tcp()
    goal = T0.translation + [0.01, 0.005, 0.0]
    s.buf.set_reactive(0.0, goal, T0.rotation)
    s.run(2.0)
    assert not any(s.moving()), "a 1 cm correction must not start the base"
    for r in s.log:
        assert r[5].base_fixed == "at rest (start charge)" and not r[5].deadband
        sc = r[5].diag["start_charge"]
        assert sc["hold"] <= sc["free"] + sc["charge"]        # moving is not worth the charge
    assert np.linalg.norm(s.tcp().translation - goal) < 0.001
    n0 = len(s.log)
    s.buf.set_reactive(s.t, T0.translation + [0.5, 0.0, 0.0], T0.rotation)
    s.run(8.0)
    assert s.starts(n0) == 1 and not s.moving()[-1]
    assert np.linalg.norm(s.tcp().translation - (T0.translation + [0.5, 0.0, 0.0])) < 0.001
    s.check_limits()
    s = Sim(model, make_config({"base_start_charge": 0.0}))
    s.buf.set_reactive(0.0, goal, T0.rotation)
    s.run(2.0)
    assert any(s.moving()) and all(r[5].base_fixed is None and "start_charge" not in r[5].diag for r in s.log)


def test_commanded_velocity_moves_the_base_while_the_arm_holds_the_ee(model):
    """A commanded velocity is the base's reference (not a fixed twist): the base ramps up to it within the twist
    rate and tracks it while the arm keeps a world-fixed EE in place; past its ttl the reference is zero and the
    base brakes and is held at rest. (0.2 m/s: from rest a commanded velocity must be worth the start charge.)"""
    v = 0.2
    s = Sim(model, Config(**FIRST_CAPS), mode="commanded", velocity=VelocityHint(vx=v, vy=0.0, omega=0.0, ttl=0.3))
    T0 = s.tcp()
    s.buf.set_reactive(0.0, T0.translation, T0.rotation)
    errs = []
    for i in range(60):
        out = s.step(velocity_age=0.0)
        assert out.status == "ok" and out.base_fixed is None
        vd = out.diag["v_des"]                            # the commanded velocity in world axes (the base yaws
        assert math.hypot(vd[0], vd[1]) == pytest.approx(v) and vd[2] == 0.0   # by ~1e-5 rad)
        errs.append(np.linalg.norm(s.tcp().translation - T0.translation))
    dv = s.cfg.max_accel * s.cfg.dt * C8                  # the twist-rate octagon along x
    for i, r in enumerate(s.log[:20]):                    # up at the twist-rate limit
        assert r[2][0] == pytest.approx((i + 1) * dv, abs=1e-5) and r[5].limited_by == "base_limit"
    # then on the reference, 5 % slow: the world-fixed EE costs the arm's compensation (measured 0.1897 m/s)
    assert all(0.18 < r[2][0] < v and r[5].limited_by == "none" for r in s.log[20:])
    assert s.base[0] > 0.09
    assert max(errs) < 0.004, max(errs)                   # the arm compensates the base's motion
    s.run(0.5, velocity_age=1.0)                          # ttl expired: the base brakes to 0 and is held
    assert np.all(s.tw == 0.0) and s.log[-1][5].base_fixed == "at rest (start charge)"
    s.check_limits()


def test_hold_brakes_the_base_and_stops_the_arm_within_limits(model):
    s = Sim(model)
    T0 = s.tcp()
    s.buf.set_reactive(0.0, T0.translation + [0.6, 0.0, 0.1], T0.rotation)
    s.run(0.6)
    assert np.linalg.norm(s.tw) > 0.05 and np.linalg.norm(s.v) > 0.05
    s.run(1.5, hold=True)
    assert np.all(s.tw == 0.0) and np.linalg.norm(s.v) < 1e-6
    assert all(r[5].base_fixed == "hold" for r in s.log[-150:])
    s.check_limits()


def test_failed_solves_return_zero(model, monkeypatch):
    s = Sim(model)
    T0 = s.tcp()
    s.buf.set_reactive(0.0, T0.translation + [0.1, 0, 0], T0.rotation)
    s.run(0.3)
    import qpsolvers
    real = qpsolvers.solve_problem

    def no_solution(prob, solver):
        sol = real(prob, solver=solver)
        sol.found = False
        return sol
    monkeypatch.setattr(qpsolvers, "solve_problem", no_solution)
    out = s.step()
    assert out.status == "infeasible" and "infeasible" in out.reason
    assert np.all(out.base_twist == 0) and np.all(out.dq["arm"] == 0) and np.all(out.qd["arm"] == 0)
    monkeypatch.setattr(qpsolvers, "solve_problem", real)
    s.buf.set_reactive(s.t, [math.nan, 0, 0], T0.rotation)
    out = s.step()
    assert out.status == "error" and np.all(out.dq["arm"] == 0) and "non-finite" in out.reason


def test_joint_box_stays_feasible_beyond_a_limit(model):
    """A joint beyond its limit (e.g. after a resync) is not infeasible: it is brought back within the limits."""
    s = Sim(model, mode="locked")
    lo, hi = model.arm_limits("arm")
    s.q[1] = hi[1] + 0.05
    T0 = s.tcp()
    s.buf.set_reactive(0.0, T0.translation, T0.rotation)
    s.run(1.0)
    assert s.q[1] <= hi[1] + 1e-6
    s.check_limits()


def test_hint_heading_and_base_pose(model):
    """A heading / pose reference turns / drives the base toward it while the arm keeps a world-fixed EE. It stops
    short by what the start charge leaves: once its rate falls below rest_ang / rest_lin, i.e. within
    tol + rest / k of the reference (+ the lag the twist band allows), holding beats starting again.
    (Heading -1 rad: +1 rad would run joint 1 into its lower limit, -0.59 rad.)"""
    cfg = Config()
    s = Sim(model, mode="hint", target_heading=HeadingHint(yaw=-1.0, tol=0.02))
    T0 = s.tcp()
    s.buf.set_reactive(0.0, T0.translation, T0.rotation)
    s.run(5.0)
    assert 0.02 < abs(s.base[2] + 1.0) <= 0.02 + cfg.rest_ang / cfg.k_heading + 1e-3   # measured 0.044
    assert np.linalg.norm(s.tcp().translation - T0.translation) < 1e-4   # the EE stayed (the arm moved instead)
    assert np.all(s.tw == 0.0) and s.log[-1][5].base_fixed == "at rest (start charge)"
    s.check_limits()
    s = Sim(model)
    s.buf.set_reactive(0.0, T0.translation, T0.rotation)
    s.hints = parse_hints({"base": {"mode": "hint", "target": {"pose": {"x": -0.1, "y": 0.05, "tol": 0.01}}}},
                          cfg, ["arm"], {"home": Q0})
    s.run(6.0)
    d = math.hypot(s.base[0] + 0.1, s.base[1] - 0.05)
    assert 0.01 < d <= 0.01 + cfg.rest_lin / cfg.k_base_pose + 1e-3, d   # measured 20.3 mm
    assert np.all(s.tw == 0.0) and s.log[-1][5].base_fixed == "at rest (start charge)"
    assert np.linalg.norm(s.tcp().translation - T0.translation) < 1e-4
    s.check_limits()


def test_base_hint_yields_at_the_reach_boundary(model):
    """A base pose reference (with its heading, as the Quest client sends) far behind a world-anchored EE target:
    the base stops where the shoulder is reach_fraction of the nominal reach from the target (limited_by reach,
    a hard row) while the EE stays on target."""
    s = Sim(model, Config(**FIRST_CAPS))
    T0 = s.tcp()
    s.buf.set_reactive(0.0, T0.translation, T0.rotation)
    s.hints = parse_hints({"base": {"mode": "hint", "target": {"pose": {"x": -1.5, "y": 0.0},
                                                               "heading": {"yaw": 0.0}}}},
                          s.cfg, ["arm"], {"home": Q0})
    s.run(8.0)
    reach = model.reach["arm"]
    q = model.config(s.base, {"arm": s.q})
    model.update(q)
    model.jacobian(q, "arm")
    sh, _ = model.shoulder_point(q, "arm")
    d = float(np.linalg.norm(T0.translation - sh))
    assert d == pytest.approx(0.88 * reach, abs=0.01), (d / reach)
    assert s.base[0] > -0.6 and np.all(s.tw == 0.0)                  # stopped at the boundary, far from -1.5
    assert sum(r[5].limited_by == "reach" for r in s.log) > 50
    # per arm (diag limited_ees, the stream's <ee>_limited_by): the arm's own reach row, the same ticks
    assert all(r[5].diag["limited_ees"].get("arm") == "reach" for r in s.log if r[5].limited_by == "reach")
    assert all("arm" not in r[5].diag["limited_ees"] for r in s.log if r[5].limited_by == "none")
    # the EE stays on target: a transient while the base brakes at the boundary (measured 6.6 mm), then none
    assert max(r[5].diag["err_pos"]["arm"] for r in s.log) < 0.01
    assert np.linalg.norm(s.tcp().translation - T0.translation) < 1e-4
    s.check_limits()


def stretch_the_elbow(model, cfg):
    """Targets scripted from the elbow (joint 3) straightening toward its limit (-0.04 rad): the arm-only
    translational manipulability falls from ~0.011 to ~0.0026. Base locked. Returns (manipulability, error) per
    tick and the Sim."""
    s = Sim(model, cfg, mode="locked")
    ws, errs = [], []
    for i in range(400):
        qa = Q0.copy()
        qa[2] = Q0[2] + min(1.0, i / 250) * (-0.04 - Q0[2])
        Tt = model.fk(model.config((0, 0, 0), {"arm": qa}), "arm")
        s.buf.set_reactive(s.t, Tt.translation, Tt.rotation)
        out = s.step()
        assert out.status == "ok", out.reason
        ws.append(model.arm_manipulability(model.config(s.base, {"arm": s.q}), "arm"))
        errs.append(out.diag["err_pos"]["arm"] + 0.1 * out.diag["err_rot"]["arm"])
    s.check_limits()
    return ws, errs, s


def test_manipulability_floor_only_when_set(model):
    """The manipulability is translational (the wrist singularity no longer counts). By default there is no floor
    (w_min 0): stretching the elbow the arm follows its targets below w_influence (with the adaptive damping) and
    nothing reports manipulability. With w_min > 0 set explicitly the floor holds: the manipulability never drops
    below w_min, the target becomes unreachable and the error grows (reported) instead of a yank."""
    cfg = Config()
    assert cfg.w_min == 0.0 and cfg.w_manip == 0.0
    ws, errs, s = stretch_the_elbow(model, cfg)
    assert min(ws) < 0.9 * cfg.w_influence                # well below the influence distance: no floor
    assert not any(r[5].limited_by == "manipulability" for r in s.log)
    assert max(r[5].diag["lm_adaptive"]["arm"] for r in s.log) > 0      # damped below w_influence
    assert all(r[5].diag["lm_adaptive"]["arm"] == 0.0 for r in s.log if r[5].diag["manip"]["arm"] >= cfg.w_influence)
    assert errs[-1] < 0.001                               # reached
    cfg = make_config({"w_min": 0.004, "w_influence": 0.006})
    ws, errs, s = stretch_the_elbow(model, cfg)
    assert min(ws) >= cfg.w_min - 1e-6, min(ws)
    assert sum(r[5].limited_by == "manipulability" for r in s.log) > 50
    assert errs[-1] > 0.01                                # unreachable within the floor: the error is reported


def test_unreachable_target_gives_growing_error_not_a_yank(model):
    s = Sim(model, mode="locked")
    T0 = s.tcp()
    s.buf.set_reactive(0.0, T0.translation + [0.0, 0.0, 0.8], T0.rotation)     # far above the arm's reach
    s.run(4.0)
    errs = [r[5].diag["err_pos"]["arm"] for r in s.log]
    assert errs[-1] > 0.2
    s.check_limits()                                      # joint speed / acceleration bounded throughout
    assert np.max(np.abs(s.v)) < 0.05                     # and it settled instead of chattering


def base_pose_of(s):
    T = s.tcp()
    c, si = math.cos(s.base[2]), math.sin(s.base[2])
    Rb = np.array([[c, -si, 0], [si, c, 0], [0, 0, 1.0]])
    return Rb.T @ (T.translation - [s.base[0], s.base[1], 0.0]), Rb.T @ T.rotation


def test_world_target_at_the_commanded_pose_and_slow_secondary_drift(model):
    """A world target equal to the commanded EE pose: no jump on the first tick; held for 10 s, the secondary
    objectives (posture, manipulability) drift the joints only slowly; a base pose hint at the base pose does
    not move the base."""
    s = Sim(model)
    s.run(0.5, target=False)                              # idle (no target yet): nothing moves
    assert np.all(s.q == Q0) and np.all(s.tw == 0) and np.all(s.base == 0)
    T = s.tcp()
    s.buf.set_reactive(s.t, T.translation, T.rotation)
    s.hints = parse_hints({"base": {"mode": "hint", "target": {"pose": {"x": 0.0, "y": 0.0, "tol": 0.02}}}},
                          s.cfg, ["arm"], {"home": Q0})
    s.step()
    assert np.max(np.abs(s.q - Q0)) < 1e-9 and np.all(s.tw == 0)
    s.run(10.0)
    drift = np.max(np.abs(s.q - Q0)) / 10.0
    assert drift < 1e-4, drift                           # rad/s: bounded and slow (measured: none with the defaults)
    assert np.all(s.tw == 0) and np.all(s.base == 0)
    assert np.linalg.norm(s.tcp().translation - T.translation) < 1e-4


def test_release_locks_the_base_decelerating_within_limits(model):
    s = Sim(model)
    T = s.tcp()
    s.buf.set_reactive(0.0, T.translation + [0.5, 0.2, 0.0], T.rotation)
    s.run(0.8)
    assert np.linalg.norm(s.tw) > 0.05
    s.hints = Hints(mode="locked")                        # the release: base locked, the EE at its commanded pose
    Tc = s.tcp()
    s.buf.set_reactive(s.t, Tc.translation, Tc.rotation)
    n0 = len(s.log)
    s.run(2.0)
    assert np.all(s.tw == 0.0)
    tws = [r[2] for r in s.log[n0:]]
    assert all(np.linalg.norm(b) <= np.linalg.norm(a) + 1e-12 for a, b in zip(tws, tws[1:]))   # monotone braking
    b1 = s.base.copy()
    s.run(2.0)
    assert np.all(s.base == b1)
    s.check_limits()


def test_chunk_blending_is_continuous_at_seams():
    """Replacing a chunk mid-way: the blended target starts at the commanded state (pose and velocity) and
    converges to the new chunk; the stale prefix is dropped."""
    buf = TargetBuffer("arm", 0.1, 0.15)
    R0 = np.eye(3)
    t = np.arange(0.0, 2.0, 0.05)
    P = np.stack([0.3 + 0.1 * np.sin(t), 0.1 * t, 0.5 + 0 * t], 1)
    buf.set_chunk(0.0, t, P, [R0] * len(t))
    prev = buf.sample(0.70)
    # the next chunk was planned from an older observation: offset by 2 cm, starting in the past
    t2 = np.arange(0.55, 2.5, 0.05)
    P2 = np.stack([0.3 + 0.1 * np.sin(t2) + 0.02, 0.1 * t2, 0.5 + 0 * t2], 1)
    R2 = [pin.utils.rpyToMatrix(0, 0, 0.05)] * len(t2)
    buf.set_chunk(0.70, t2, P2, R2, anchor_p=prev.p, anchor_R=prev.R, anchor_v=prev.v)
    s0 = buf.sample(0.70)
    assert np.allclose(s0.p, prev.p, atol=1e-9) and np.allclose(s0.R, prev.R, atol=1e-9)
    assert np.allclose(s0.v, prev.v, atol=1e-6)
    # smooth after the seam: finite-difference velocity matches the reported one, no jumps
    dt = 0.002
    ts = np.arange(0.70, 1.5, dt)
    S = [buf.sample(x) for x in ts]
    for a, b in zip(S, S[1:]):
        fd = pose_diff(b.p, b.R, a.p, a.R) / dt
        assert np.linalg.norm(fd - a.v) < 0.02, (fd, a.v)
    assert buf._ct[0] <= 0.70 < buf._ct[1]              # the stale prefix (0.55 .. 0.65) was dropped
    late = buf.sample(1.5)                                # ~5 tau later: on the new chunk
    raw_p = np.interp(1.5, t2, P2[:, 0])
    assert abs(late.p[0] - raw_p) < 0.002


def test_chunk_through_the_controller_has_no_velocity_jump(model):
    s = Sim(model, mode="locked")
    T0 = s.tcp()
    t = np.arange(0.0, 3.0, 0.1)
    P = T0.translation + np.stack([0.05 * np.sin(2 * t), 0 * t, 0.03 * t / 3], 1)
    s.buf.set_chunk(0.0, t, P, [T0.rotation] * len(t))
    s.run(1.0)
    q = s.m.config(s.base, {"arm": s.q})
    T = s.tcp()
    v = s.tcp_twist()
    t2 = np.arange(1.0, 3.0, 0.1)
    P2 = T0.translation + np.stack([0.05 * np.sin(2 * t2) + 0.03, 0 * t2, 0.03 * t2 / 3], 1)
    s.buf.set_chunk(s.t, t2, P2, [T0.rotation] * len(t2), anchor_p=T.translation, anchor_R=T.rotation, anchor_v=v)
    s.run(2.0)
    s.check_limits()
    vs = np.array([r[4] for r in s.log])
    assert np.max(np.abs(np.diff(vs, axis=0))) <= max(s.ctl.caps["arm"][1]) * s.cfg.dt + 1e-6


def test_staleness_fades_the_feedforward_and_holds_the_pose():
    buf = TargetBuffer("arm", ff_timeout=0.1)
    buf.set_reactive(10.0, [0.3, 0, 0.5], np.eye(3), v=[0.1, 0, 0, 0, 0, 0.2], gripper=0.5)
    assert buf.sample(10.05).ff_scale == 1.0 and buf.sample(10.05).v[0] == pytest.approx(0.1)
    mid = buf.sample(10.15)
    assert mid.ff_scale == pytest.approx(0.5) and mid.v[0] == pytest.approx(0.05)
    late = buf.sample(10.25)
    assert late.ff_scale == 0.0 and np.all(late.v == 0) and np.allclose(late.p, [0.3, 0, 0.5])
    assert late.gripper == 0.5
    # a chunk is fresh until its end, then fades the same way
    buf.set_chunk(20.0, [20.0, 20.5], [[0.3, 0, 0.5], [0.35, 0, 0.5]], [np.eye(3)] * 2)
    assert buf.sample(20.25).ff_scale == 1.0 and buf.sample(20.25).v[0] == pytest.approx(0.1)
    assert buf.sample(20.65).ff_scale == pytest.approx(0.5)
    assert buf.sample(21.0).ff_scale == 0.0 and np.allclose(buf.sample(21.0).p, [0.35, 0, 0.5])


def test_hint_parsing_bounds_and_errors():
    cfg = Config()
    post = {"home": Q0}
    h = parse_hints({"base": {"mode": "hint", "target": {"heading": {"face": "arm"}},
                              "limits": {"max_speed": 0.1}, "arm_vs_base": 2.0},
                     "ees": {"arm": {"weights": {"position": 1.0, "orientation": 0.0},
                                     "orientation_mask": [1, 1, 0], "posture": "home", "posture_weight": 0.001}}},
                    cfg, ["arm"], post)
    assert h.mode == "hint" and h.target_heading.face == "arm" and h.limits.max_speed == 0.1
    assert h.ees["arm"].orientation_mask == (1.0, 1.0, 0.0)
    for bad in ({"base": {"mode": "flying"}}, {"base": {"limits": {"max_speed": 5.0}}},
                {"base": {"target": {"heading": {"face": "leg"}}}}, {"ees": {"leg": {}}},
                {"ees": {"arm": {"posture": "nowhere"}}}, {"base": {"velocity": {"vx": float("nan")}}},
                {"base": {"pose": {"x": 1, "y": 0, "frame": "base"}}}, {"bogus": 1}):
        with pytest.raises(ValueError):
            parse_hints(bad, cfg, ["arm"], post)
    with pytest.raises(ValueError):
        make_config({"deadband_lin_off": 0.5})
    with pytest.raises(ValueError):
        make_config({"wheel_speed": 4.0})                   # the base speed caps are checked against the swerve's
    with pytest.raises(ValueError):                         # hard limits at enable
        make_config({"joint_vel": [1, 1, 1, 1, 1, -1]})
    assert make_config({}).joint_vel is None                # the arm's own


def walk(model, v, ff: bool, seconds=3.0, hint_age=0.0):
    """The operator walks forward at v (m/s): the EE target and the base pose hint both move at v (world x), with or
    without their velocity feedforward. Returns (EE lag, base lag) at the end (m) and the Sim."""
    s = Sim(model)
    T0 = s.tcp()
    lag_ee, lag_base = [], []
    for i in range(int(seconds / s.cfg.dt)):
        t = s.t
        p = T0.translation + [v * t, 0.0, 0.0]
        s.buf.set_reactive(t, p, T0.rotation, v=[v, 0, 0, 0, 0, 0] if ff else None)
        s.hints = Hints(mode="hint", target_pose=PoseHint(v * t, 0.0, weight=1.0, tol=0.0,
                                                          vx=v if ff else 0.0))
        tg = s.buf.sample(t)
        snap = Snapshot(t, tuple(s.base), s.tw.copy(),
                        {"arm": EEState(s.q.copy(), s.v.copy(), tg, s.hints.ee("arm", s.cfg), Q0)}, s.hints,
                        hint_age=hint_age)
        out = s.ctl.step(snap)
        assert out.status == "ok", out.reason
        s.log.append((t, s.tw.copy(), out.base_twist.copy(), s.v.copy(), out.qd["arm"].copy(), out))
        s.tw = out.base_twist.copy()
        c, sn = math.cos(s.base[2]), math.sin(s.base[2])
        s.base += np.array([c * s.tw[0] - sn * s.tw[1], sn * s.tw[0] + c * s.tw[1], s.tw[2]]) * s.cfg.dt
        s.q = s.q + out.dq["arm"]
        s.v = out.qd["arm"].copy()
        s.t += s.cfg.dt
        lag_ee.append(float(np.linalg.norm(s.tcp().translation - (T0.translation + [v * s.t, 0, 0]))))
        lag_base.append(float(v * s.t - s.base[0]))
    return float(np.mean(lag_ee[-50:])), float(np.mean(lag_base[-50:])), s


def test_velocity_feedforward_removes_the_walking_lag(model):
    """At 0.5 m/s: without feedforward the EE trails its target by about v / k_pos and the base its hint by about
    v / k_base_pose; with the EE twist and the hint velocity it keeps up (measured lags printed)."""
    v = 0.5
    ee0, base0, s0 = walk(model, v, ff=False)
    ee1, base1, s1 = walk(model, v, ff=True)
    print(f"\nwalking at {v} m/s: EE lag {ee0 * 1e3:.1f} -> {ee1 * 1e3:.1f} mm, base lag {base0 * 1e3:.1f} -> "
          f"{base1 * 1e3:.1f} mm")
    assert ee0 > 0.5 * v / s0.cfg.k_pos                    # ~v / k_pos without feedforward
    assert ee1 < 0.01 and ee1 < 0.1 * ee0
    assert base1 < 0.03 and base1 < 0.3 * base0
    s1.check_limits()


def test_stale_hint_feedforward_fades(model):
    """The hint velocity obeys the EE twist's staleness: fresh = full, fading linearly past ff_timeout, none past
    2 x ff_timeout; the base's own reference v_des = the faded velocity + k (reference - pose)."""
    cfg = Config()
    for age, kff in ((0.0, 1.0), (cfg.ff_timeout, 1.0), (1.5 * cfg.ff_timeout, 0.5), (2.0 * cfg.ff_timeout, 0.0),
                     (2.5 * cfg.ff_timeout, 0.0)):
        s = Sim(model, mode="hint", target_pose=PoseHint(0.1, 0.0, tol=0.0, vx=0.3, vy=-0.2),
                target_heading=HeadingHint(yaw=0.0, tol=0.0, omega=0.4))
        T0 = s.tcp()
        s.buf.set_reactive(0.0, T0.translation, T0.rotation)
        out = s.step(hint_age=age)
        assert out.diag["v_des"] == pytest.approx([kff * 0.3 + cfg.k_base_pose * 0.1, kff * -0.2, kff * 0.4],
                                                  abs=1e-12), age
    v = 0.5
    _, fresh, _ = walk(model, v, ff=True, seconds=2.0, hint_age=0.0)
    _, stale, s = walk(model, v, ff=True, seconds=2.0, hint_age=2.5 * cfg.ff_timeout)
    assert s.log[-1][5].diag["v_des"][0] == pytest.approx(cfg.k_base_pose * stale, rel=0.05)   # pose term only
    assert stale > fresh + 0.05                              # without the feedforward the base trails again


# -- regressions of the one-QP formulation (the owner's sim checks) -------------------------------------------

def test_head_sway_does_not_start_the_base(model):
    """Standing: the head sways ±3 cm at 0.5 Hz, so the base pose reference does (with its velocity; tolerance 0,
    the strictest: the client's is 0.10 m) while the EE target rides with the base. Not worth the start charge:
    the base stays exactly still. Without the charge the same sway moves it."""
    def sway(cfg):
        s = Sim(model, cfg)
        rel = base_pose_of(s)
        w = 2 * math.pi * 0.5
        for _ in range(600):
            s.hints = Hints(mode="hint", target_pose=PoseHint(0.03 * math.sin(w * s.t), 0.0, tol=0.0,
                                                              vx=0.03 * w * math.cos(w * s.t)),
                            target_heading=HeadingHint(yaw=0.0, tol=0.05))
            s.ride(rel)
            assert s.step().status == "ok"
        return s
    s = sway(Config())
    assert not any(s.moving()) and np.all(s.base == 0.0)
    assert all(r[5].base_fixed == "at rest (start charge)" for r in s.log)
    s = sway(make_config({"base_start_charge": 0.0}))
    assert sum(s.moving()) > 300 and max(abs(r[2][0]) for r in s.log) > 0.05
    s.check_limits()


def test_hand_circling_while_standing_does_not_start_the_base(model):
    """Engaged, standing still (pose and heading references at the base): the hand draws a 15 cm circle (7.5 cm
    radius) at 1 Hz in the vertical plane in front of the arm, ramped in over 1 s, with its twist. The arm takes
    it all (at its joint speed caps: tracking error < 2 cm); the base never starts."""
    s = Sim(model, mode="locked")
    T0 = s.tcp()
    c = T0.translation + [0.15, 0.0, -0.10]
    s.buf.set_reactive(0.0, c, T0.rotation)
    s.run(2.0)                                            # the hand to the circle's centre (the arm alone)
    s.hints = Hints(mode="hint", target_pose=PoseHint(0.0, 0.0, tol=0.10), target_heading=HeadingHint(yaw=0.0))
    t0, rad, w = s.t, 0.075, 2 * math.pi
    errs = []
    for _ in range(500):
        t = s.t - t0
        g, gd = (t, 1.0) if t < 1.0 else (1.0, 0.0)       # the radius ramps in over 1 s
        off = rad * np.array([0.0, g * math.sin(w * t), g * (1 - math.cos(w * t))])
        vel = rad * np.array([0.0, g * w * math.cos(w * t) + gd * math.sin(w * t),
                              g * w * math.sin(w * t) + gd * (1 - math.cos(w * t))])
        s.buf.set_reactive(s.t, c + off, T0.rotation, v=list(vel) + [0, 0, 0])
        out = s.step()
        assert out.status == "ok"
        errs.append(out.diag["err_pos"]["arm"])
    assert not any(s.moving()) and np.all(s.base == 0.0)
    assert max(errs) < 0.02, max(errs)
    s.check_limits()


@pytest.mark.parametrize("v", [0.15, 0.3, 0.5])
def test_walking_with_feedforward_keeps_the_base_within_a_centimetre(model, v):
    """Walking at v with the hint and EE velocities: after the start (the twist rate) the base trails its reference
    by less than 1 cm, the EE its target by less than 1 mm."""
    ee, base, s = walk(model, v, ff=True, seconds=4.0)
    assert abs(base) < 0.01 and ee < 0.001, (base, ee)
    assert s.starts() == 1
    s.check_limits()


def test_hand_pushed_beyond_reach_starts_the_base(model):
    """Standing (pose reference at the base, the client's tolerance): the hand pushed 0.5 m forward, beyond the
    arm's reach. The reach row makes the base move toward the target (> 3 cm), within its pose band, and the EE
    gets there."""
    s = Sim(model, mode="hint", target_pose=PoseHint(0.0, 0.0, tol=0.10), target_heading=HeadingHint(yaw=0.0))
    T0 = s.tcp()
    s.buf.set_reactive(0.0, T0.translation + [0.5, 0.0, 0.0], T0.rotation)
    out = s.step()
    assert out.diag["reach"]["arm"] > s.cfg.reach_fraction           # the target lies beyond the reach limit
    s.run(4.0)
    assert s.base[0] > 0.03 and s.starts() == 1
    assert "reach" in {r[5].limited_by for r in s.log}
    assert s.log[-1][5].diag["err_pos"]["arm"] < 0.001
    assert math.hypot(s.base[0], s.base[1]) <= s.cfg.band_pos       # within the pose band
    s.check_limits()


@pytest.mark.parametrize("step,moves", [(0.12, True), (0.04, False)])
def test_a_reference_step_starts_the_base_only_when_worth_the_charge(model, step, moves):
    """The base pose reference stepped (no velocity, no tolerance), the EE target world-fixed: 12 cm is worth
    starting (the base goes there), 4 cm is not (it stays exactly still, the arm holds the EE)."""
    s = Sim(model, mode="hint", target_pose=PoseHint(step, 0.0, tol=0.0), target_heading=HeadingHint(yaw=0.0))
    T0 = s.tcp()
    s.buf.set_reactive(0.0, T0.translation, T0.rotation)
    s.run(4.0)
    if moves:
        cfg = s.cfg
        assert s.starts() == 1 and np.all(s.tw == 0.0)
        assert abs(s.base[0] - step) <= cfg.rest_lin / cfg.k_base_pose + 1e-3
    else:
        assert not any(s.moving()) and np.all(s.base == 0.0)
    assert np.linalg.norm(s.tcp().translation - T0.translation) < 0.001
    s.check_limits()


def test_full_stick_needs_the_arm_riding(model):
    """Released, full stick forward (commanded 0.45 m/s, base frame). With the arm riding with the base (the client's
    target) the base reaches the commanded speed. With the EE target fixed in the world the EE task holds the base
    back (the arm runs out of reach, then joint range): EE and base references must be consistent."""
    def drive(riding):
        s = Sim(model, mode="commanded", velocity=VelocityHint(vx=0.45, ttl=0.2))
        T0 = s.tcp()
        rel = base_pose_of(s)
        s.buf.set_reactive(0.0, T0.translation, T0.rotation)
        for _ in range(300):
            if riding:
                s.ride(rel)
            assert s.step(velocity_age=0.0).status == "ok"
        s.check_limits()
        return s
    s = drive(True)
    assert max(r[2][0] for r in s.log) > 0.43 and s.tw[0] == pytest.approx(0.45, abs=1e-3)
    s = drive(False)
    assert max(r[2][0] for r in s.log) < 0.43 and s.tw[0] < 0.2


def test_a_binding_twist_rate_reports_base_limit(model):
    """From rest toward a far reference (no start charge) the first tick accelerates at the twist-rate limit: the
    binding rate row reports base_limit (a value of the stream's enum)."""
    s = Sim(model, make_config({"base_start_charge": 0.0}), mode="hint",
            target_pose=PoseHint(1.0, 0.0, tol=0.0), target_heading=HeadingHint(yaw=0.0))
    s.ride(base_pose_of(s))
    out = s.step()
    assert oct_norm(out.base_twist[:2]) == pytest.approx(s.cfg.max_accel * s.cfg.dt, rel=1e-6)
    assert out.limited_by == "base_limit"


def test_limited_by_is_always_a_stream_value(model):
    """Every limited_by the controller produces over a mixed scenario (reach, joint limits, manipulability floor,
    bands, base caps and rates, holds, locked, commanded) is one of LIMITED (the stream's enum: the node faults
    on anything else)."""
    seen = set()
    s = Sim(model, make_config({"max_speed": 0.3, "wheel_speed": 0.35}), mode="hint",
            target_heading=HeadingHint(yaw=2.5, tol=0.02))
    T0 = s.tcp()
    s.buf.set_reactive(0.0, T0.translation + [-0.8, 0.5, -0.1], T0.rotation)          # behind: joint limits, caps
    s.run(3.0)
    s.hints = parse_hints({"base": {"mode": "hint", "target": {"pose": {"x": -1.5, "y": 0.0},
                                                               "heading": {"yaw": 0.0}}}}, s.cfg, ["arm"], {})
    s.buf.set_reactive(s.t, s.tcp().translation, s.tcp().rotation)                  # reach boundary, bands
    s.run(3.0)
    bx, by, byaw = s.base                                                          # the hand beyond reach
    s.hints = Hints(mode="hint", target_pose=PoseHint(bx, by, tol=0.10), target_heading=HeadingHint(yaw=byaw))
    T = s.tcp()
    s.buf.set_reactive(s.t, T.translation + 0.5 * np.array([math.cos(byaw), math.sin(byaw), 0.0]), T.rotation)
    s.run(2.0)
    s.hints = Hints(mode="commanded", velocity=VelocityHint(vx=0.3, vy=0.2, omega=0.5))
    s.run(1.0, velocity_age=0.0)
    s.run(0.5, hold=True)
    s.hints = Hints(mode="locked")
    s.run(0.5)
    seen |= {r[5].limited_by for r in s.log}
    _, _, s2 = stretch_the_elbow(model, make_config({"w_min": 0.004, "w_influence": 0.006}))   # the floor
    seen |= {r[5].limited_by for r in s2.log}
    assert seen <= set(LIMITED), seen - set(LIMITED)
    assert {"none", "reach", "joint_limit", "manipulability", "base_band", "base_limit"} <= seen, seen


