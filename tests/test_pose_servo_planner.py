"""Pose servo planner in a kinematic simulation (the base follows the commanded twist exactly).

The constraint checks recompute every quantity from the sequence of twists the servo returns
(``check_step``), independently of the limiter's own bookkeeping."""
import math
import random

import pytest

from robot_utils.nodes.pose_servo.planner import (STEER_RATE_MAX, WHEEL_SPEED_MAX, Caps, Cmd, Goal, Motion, Servo,
                                                  approach_time, check_step, limit_step, speed_for_distance,
                                                  steer_rate, stop_distance, within)

DT = 0.02
CAPS_SETS = [Caps(), Caps(2.0, 4.0, 5.0, 20.0, 50.0, 200.0), Caps(0.1, 0.3, 0.2, 0.5, 0.5, 1.0),
             Caps(1.0, 2.0, 3.0, 10.0, 5.0, 40.0), Caps(0.5, 1.5, 1.0, 3.0, 1.0, 3.0)]


class Sim:
    """Kinematic base + servo; records the worst of every constrained quantity."""

    def __init__(self, caps=Caps(), pose=(0.0, 0.0, 0.0), cmd=None, disturb=None):
        self.servo = Servo(caps)
        if cmd is not None:
            self.servo.cmd = cmd
        self.caps = caps
        self.x, self.y, self.th = pose
        self.prev = Cmd(**vars(self.servo.cmd))
        self.t = 0.0
        self.worst = {}
        self.bad = []
        self.log = []
        self.n_id = 0
        self.disturb = disturb

    def goal(self, x, y, th, **kw):
        self.n_id += 1
        self.servo.set_goal(Goal(x, y, th, Motion.from_caps(self.caps, **kw), self.n_id))

    def step(self):
        n, d = self.servo.update((self.x, self.y, self.th), DT)
        self.t += DT
        if n is None:
            self.prev = Cmd()
            self.log.append((self.t, d, None))
            return d
        c = check_step(self.prev, n, DT)
        for k, v in vars(c).items():
            self.worst[k] = max(self.worst.get(k, 0.0), v)
        if within(c, self.caps):
            self.bad.append((self.t, within(c, self.caps)))
        self.prev = Cmd(**vars(n))
        ct, st = math.cos(self.th), math.sin(self.th)
        self.x += (ct * n.vx - st * n.vy) * DT
        self.y += (st * n.vx + ct * n.vy) * DT
        self.th += n.w * DT
        if self.disturb:
            self.disturb(self)
        self.log.append((self.t, d, n))
        return d

    def run(self, seconds=60.0, until_idle=True):
        end = self.t + seconds
        while self.t < end:
            self.step()
            if until_idle and not self.servo.active:
                return True
        return not self.servo.active


def assert_clean(sim):
    assert not sim.bad, sim.bad[:5]
    assert sim.servo.violations == 0


# -- formulas ------------------------------------------------------------------------------------

@pytest.mark.parametrize("A,J", [(0.5, 2.0), (3.0, 5.0), (0.2, 0.5)])
def test_stop_distance_inverse(A, J):
    for v in (0.001, 0.05, 0.3, 1.0, 2.0):
        assert speed_for_distance(stop_distance(v, A, J), A, J) == pytest.approx(v, rel=1e-6)


def test_steer_rate_is_mod_pi_and_ignores_slow_wheels():
    assert steer_rate((0.5, 0, 0), (-0.5, 0, 0), DT) == pytest.approx(0.0, abs=1e-9)   # a flip, not a turn
    assert steer_rate((0.01, 0, 0), (0, 0.01, 0), DT) == 0.0                              # below the speed floor
    assert steer_rate((0.5, 0, 0), (0.5, 0.5, 0), DT) == pytest.approx(math.pi / 4 / DT)


def test_limiter_never_corners_itself():
    """From any recoverable state, any requested jerk gives a step within all constraints."""
    rnd = random.Random(3)
    for caps in CAPS_SETS:
        c = Cmd()
        for _ in range(3000):
            j = [rnd.uniform(-1, 1) * caps.max_jerk * 3, rnd.uniform(-1, 1) * caps.max_jerk * 3,
                 rnd.uniform(-1, 1) * caps.max_ang_jerk * 3]
            n, _ = limit_step(c, *j, caps, DT)
            assert not within(check_step(c, n, DT), caps)
            c = n


# -- goals -----------------------------------------------------------------------------------------

@pytest.mark.parametrize("goal", [(1.0, 0, 0), (0, 0, 2.5 * math.pi), (1.0, 0.5, 1.0), (-0.5, 0.5, -2.0),
                                  (0.02, 0.0, 0.0), (0.0, -2.0, math.pi)])
def test_reaches_goal_within_tolerance(goal):
    s = Sim()
    s.goal(*goal)
    assert s.run(60)
    assert s.servo.result["result"] == "reached"
    assert math.hypot(s.x - goal[0], s.y - goal[1]) < 0.025          # tolerance 0.02 + the final stop
    assert abs(s.th - goal[2]) < 0.04                                  # continuous heading: turns count
    assert s.servo.cmd.is_zero()
    assert_clean(s)


def test_limits_are_used_but_never_exceeded():
    s = Sim()
    s.goal(2.0, 0.0, 2.0)
    assert s.run(60)
    caps = s.caps
    assert s.worst["speed"] == pytest.approx(caps.max_speed, rel=0.02)
    assert s.worst["accel"] == pytest.approx(caps.max_accel, rel=0.02)
    assert s.worst["jerk"] == pytest.approx(caps.max_jerk, rel=0.02)
    assert s.worst["omega"] <= caps.max_omega + 1e-9
    assert_clean(s)


def test_synchronized_axes_finish_together_independent_do_not():
    gaps = {}
    for goal in [(1.0, 0.5, 1.0), (0.3, 0.0, 3.0), (2.0, 0.0, 0.3), (-0.5, 0.5, -2.0)]:
        done = {}
        for mode in ("synchronized", "independent"):
            s = Sim()
            s.goal(*goal, mode=mode)
            assert s.run(60)
            tp = next(t for t, d, _ in s.log if d.err_pos <= 0.02)
            tr = next(t for t, d, _ in s.log if abs(d.err_theta) <= 0.035)
            done[mode] = abs(tp - tr)
            assert_clean(s)
        assert done["synchronized"] < 0.25, (goal, done)
        gaps[goal] = done
    assert gaps[(2.0, 0.0, 0.3)]["independent"] > 5.0      # a quick turn inside a long move


def test_new_goal_mid_motion_continues_from_current_velocity():
    s = Sim()
    s.goal(1.5, 0, 0)
    for _ in range(int(1.5 / DT)):
        s.step()
    v_before = s.servo.cmd.vx
    assert v_before > 0.25
    s.goal(-1.0, 0.5, 1.0)                         # behind and to the side: must brake first
    d = s.step()
    assert s.servo.cmd.vx > v_before - s.caps.max_accel * DT - 1e-9     # no velocity jump
    assert s.run(60) and s.servo.result["result"] == "reached"
    assert math.hypot(s.x + 1.0, s.y - 0.5) < 0.025
    assert_clean(s)


def test_starts_from_a_moving_state():
    s = Sim(cmd=Cmd(0.25, -0.15, 0.6))           # e.g. the servo was replanned while moving
    s.goal(0.5, 0.5, -1.0)
    assert s.run(60) and s.servo.result["result"] == "reached"
    assert_clean(s)


def test_stop_brakes_within_limits():
    s = Sim()
    s.goal(2.0, 0.0, 1.0)
    for _ in range(int(2.0 / DT)):
        s.step()
    s.servo.stop("test")
    assert s.servo.status == "braking"
    assert s.run(10)
    assert s.servo.status == "idle" and s.servo.cmd.is_zero()
    assert s.servo.result["result"] == "stopped"
    assert_clean(s)


def test_acceptance_region_no_hunting_and_hold_reengages():
    pushes = []

    def push(sim):            # after the goal is held for a while, shove the base 3 cm, then 10 cm
        if sim.servo.status == "holding":
            held = sum(1 for _, d, _ in sim.log[-50:] if d.status == "holding")
            if held == 50 and len(pushes) < 2:
                dx = 0.03 if not pushes else 0.10
                sim.x += dx
                pushes.append(sim.t)

    s = Sim(disturb=push)
    s.goal(0.5, 0.0, 0.0, on_reach="hold")
    s.run(20, until_idle=False)
    statuses = [d.status for _, d, _ in s.log]
    assert len(pushes) == 2
    # 3 cm (< hold_band * tol = 4 cm): stays latched, no motion; 10 cm: moves back and holds again
    i1 = int(round(pushes[0] / DT))
    i2 = int(round(pushes[1] / DT))
    assert all(st == "holding" for st in statuses[i1:i2 - 1])
    assert "moving" in statuses[i2:]
    assert statuses[-1] == "holding"
    assert abs(s.x - 0.5) < 0.025
    # holding: a constant zero twist, no dithering
    tail = [n for _, _, n in s.log[-100:]]
    assert all(n is not None and n.is_zero() for n in tail)
    assert_clean(s)


def test_fuzz_random_goal_sequences_never_violate():
    rnd = random.Random(1)
    for i in range(300):
        caps = rnd.choice(CAPS_SETS)
        start = Cmd(rnd.uniform(-0.6, 0.6) * caps.max_speed, rnd.uniform(-0.6, 0.6) * caps.max_speed,
                    rnd.uniform(-0.9, 0.9) * caps.max_omega)
        s = Sim(caps, cmd=start)
        mode = rnd.choice(["synchronized", "independent"])
        for _ in range(rnd.randint(1, 4)):
            s.goal(rnd.uniform(-2, 2), rnd.uniform(-2, 2), rnd.uniform(-7, 7), mode=mode)
            for _ in range(int(rnd.uniform(0.1, 3.0) / DT)):
                s.step()
        assert s.run(120), i
        assert s.servo.result["result"] == "reached", i
        assert_clean(s)
        assert s.worst.get("wheel", 0) <= WHEEL_SPEED_MAX and s.worst.get("steer", 0) <= STEER_RATE_MAX * (1 + 1e-6)


def test_approach_time_is_a_reasonable_prediction():
    s = Sim()
    s.goal(1.0, 0.0, 0.0, mode="independent")
    d0 = s.step()
    assert s.run(60)
    t_reach = next(t for t, d, _ in s.log if d.err_pos <= 0.02)
    # planned with PLAN_MARGIN of the limits, tracked with all of them: an over-estimate (only
    # the ratio of the two axes matters for synchronization)
    assert t_reach < d0.ttg_lin < 1.3 * t_reach
    assert approach_time(0.01, 0, 0.3, 0.4, 1.6, 0.02) == 0.0
