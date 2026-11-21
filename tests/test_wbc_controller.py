"""The WBC's model, target buffers and QP controller with pinocchio only (no nodes, no network): reaching with
the base free / locked, the base and arm limits held on every step, the deadband, commanded and hinted base
modes, infeasible solves, chunk blending continuity at seams, staleness. docs/wbc.md."""
import math

import numpy as np
import pinocchio as pin
import pytest

from robot_utils.nodes.swerve.constants import MODULES
from robot_utils.nodes.wbc.controller import C8, QPController, EEState, Snapshot, oct_norm
from robot_utils.nodes.wbc.model import EESpec, WbcModel, arm_config_limits, fetch_urdf
from robot_utils.nodes.wbc.spec import Config, Hints, VelocityHint, HeadingHint, make_config, parse_hints
from robot_utils.nodes.wbc.targets import TargetBuffer, pose_diff

Q0 = np.array([0.0, -0.6, -0.9, 0.0, 0.5, 0.0])
WHEELS = [(m.x, m.y) for m in MODULES]


@pytest.fixture(scope="module")
def model():
    urdf, _ = fetch_urdf("local")
    return WbcModel(urdf, (EESpec(arm_limits=arm_config_limits()),))


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

    def step(self, hold=False, velocity_age=math.inf, target=True):
        tg = self.buf.sample(self.t) if target else None
        h = self.hints
        snap = Snapshot(self.t, tuple(self.base), self.tw.copy(),
                        {"arm": EEState(self.q.copy(), self.v.copy(), tg, h.ee("arm", self.cfg), Q0)}, h,
                        hold=hold, velocity_age=velocity_age)
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
            assert np.all(np.abs(qd) <= np.array(cfg.joint_vel) + tol), (t, qd)
            assert np.all(np.abs(qd - vprev) <= np.array(cfg.joint_acc) * dt + 1e-5), (t, qd - vprev)


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
    s.run(1.0)
    assert np.all(s.tw == 0.0) and s.log[-1][5].deadband  # converged: the deadband holds the base at exactly 0
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
    assert s.log[-1][5].diag["err_pos"]["arm"] < 0.1


def test_deadband_keeps_small_corrections_on_the_arm(model):
    s = Sim(model)
    T0 = s.tcp()
    s.buf.set_reactive(0.0, T0.translation + [0.01, 0.005, 0.0], T0.rotation)
    s.run(2.0)
    assert all(np.all(r[2] == 0.0) for r in s.log), "a 1 cm correction must not start the base"
    assert all(r[5].deadband for r in s.log)
    assert np.linalg.norm(s.tcp().translation - (T0.translation + [0.01, 0.005, 0.0])) < 0.001
    # a far target starts it; once there it stops (hysteresis: no chatter in between)
    s.buf.set_reactive(s.t, T0.translation + [0.5, 0.0, 0.0], T0.rotation)
    s.run(8.0)
    moving = [bool(np.any(r[2] != 0.0)) for r in s.log]
    starts = sum(1 for a, b in zip(moving, moving[1:]) if b and not a)
    assert starts == 1 and not moving[-1], starts
    s.check_limits()


def test_commanded_velocity_moves_the_base_while_the_arm_holds_the_ee(model):
    s = Sim(model, mode="commanded", velocity=VelocityHint(vx=0.1, vy=0.0, omega=0.0, ttl=0.3))
    T0 = s.tcp()
    s.buf.set_reactive(0.0, T0.translation, T0.rotation)
    errs = []
    for i in range(60):
        out = s.step(velocity_age=0.0)
        assert out.status == "ok" and out.base_fixed == "commanded velocity"
        errs.append(np.linalg.norm(s.tcp().translation - T0.translation))
    assert s.tw[0] == pytest.approx(0.1) and s.base[0] > 0.03
    assert max(errs) < 0.005, max(errs)                   # the arm compensates the base's motion
    s.run(0.5, velocity_age=1.0)                          # ttl expired: the base brakes to 0
    assert np.all(s.tw == 0.0)
    s.check_limits()


def test_hold_brakes_the_base_and_stops_the_arm_within_limits(model):
    s = Sim(model)
    T0 = s.tcp()
    s.buf.set_reactive(0.0, T0.translation + [0.6, 0.0, 0.1], T0.rotation)
    s.run(1.5)
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
    s = Sim(model, mode="hint", target_heading=HeadingHint(yaw=1.0, weight=1.0, tol=0.02))
    T0 = s.tcp()
    s.buf.set_reactive(0.0, T0.translation, T0.rotation)
    s.run(5.0)
    assert abs(s.base[2] - 1.0) < 0.04
    assert np.linalg.norm(s.tcp().translation - T0.translation) < 0.004   # the EE stayed (the arm moved instead)
    s.check_limits()
    s = Sim(model)
    s.buf.set_reactive(0.0, T0.translation, T0.rotation)
    s.hints = parse_hints({"base": {"mode": "hint", "target": {"pose": {"x": -0.1, "y": 0.05, "weight": 2.0,
                                                                         "tol": 0.01}}}}, Config(), ["arm"], {"home": Q0})
    s.run(6.0)
    assert math.hypot(s.base[0] + 0.1, s.base[1] - 0.05) < 0.04      # a soft hint: stops at tol + the deadband
    s.check_limits()


def test_base_hint_yields_at_the_reach_boundary(model):
    """A base pose hint walking away from a world-anchored EE target: the base stops where the shoulder is
    reach_fraction of the nominal reach from the target (limited_by reach) while the EE stays on target."""
    s = Sim(model)
    T0 = s.tcp()
    s.buf.set_reactive(0.0, T0.translation, T0.rotation)
    s.hints = parse_hints({"base": {"mode": "hint", "target": {"pose": {"x": -1.5, "y": 0.0, "weight": 5.0}}}},
                          s.cfg, ["arm"], {"home": Q0})
    s.run(8.0)
    reach = model.reach["arm"]
    q = model.config(s.base, {"arm": s.q})
    model.update(q)
    model.jacobian(q, "arm")
    sh, _ = model.shoulder_point(q, "arm")
    d = float(np.linalg.norm(T0.translation - sh))
    assert d == pytest.approx(0.88 * reach, abs=0.01), (d / reach)
    assert s.base[0] > -0.6 and np.linalg.norm(s.tw) < 0.01          # stopped at the boundary, far from -1.5
    assert sum(r[5].limited_by == "reach" for r in s.log) > 50
    # the EE stays on target; once the base stops, an arm at a joint limit keeps at most deadband_lin_on / k_pos
    # (5 mm) of error (that correction would need the base)
    assert max(r[5].diag["err_pos"]["arm"] for r in s.log) < 0.02          # transient while it brakes (measured 15 mm)
    assert np.linalg.norm(s.tcp().translation - T0.translation) < 0.005
    s.check_limits()


def test_manipulability_floor_near_the_wrist_singularity(model):
    """Targets scripted from joint 5 heading into the wrist singularity (q5 -> pi/2): the arm-only
    manipulability never drops below w_min, joint speeds stay within their caps, the target becomes
    unreachable and the tracking error grows (reported) instead of a yank."""
    s = Sim(model, mode="locked")
    cfg = s.cfg
    ws, errs = [], []
    for i in range(400):
        qa = Q0.copy()
        qa[4] = 0.5 + min(1.0, i / 250) * (1.55 - 0.5)
        Tt = model.fk(model.config((0, 0, 0), {"arm": qa}), "arm")
        s.buf.set_reactive(s.t, Tt.translation, Tt.rotation)
        out = s.step()
        assert out.status == "ok"
        ws.append(model.arm_manipulability(model.config(s.base, {"arm": s.q}), "arm"))
        errs.append(out.diag["err_pos"]["arm"] + 0.1 * out.diag["err_rot"]["arm"])
    assert min(ws) >= cfg.w_min - 1e-5, min(ws)
    assert any(r[5].limited_by == "manipulability" for r in s.log)
    assert errs[-1] > 0.005                               # unreachable within the floor: the error is reported
    s.check_limits()


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
    assert np.max(np.abs(np.diff(vs, axis=0))) <= max(s.cfg.joint_acc) * s.cfg.dt + 1e-6


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
        make_config({"max_speed": 3.0})
