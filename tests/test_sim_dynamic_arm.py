"""The sim world's dynamic arm fidelity (``components.arm.fidelity: dynamic``; docs/sim.md, "What the world is"):
the Damiao motors' MIT law applied as joint torque, clamped at the robot config's torque limits, no gravity
feedforward of the sim's own. The gains here are the arm node's (the soft profile's kp / kd, its gravity model)."""
import math

import numpy as np
import pytest

from sim_helpers import make_world

from robot_utils.nodes.arm import config as C


@pytest.fixture(scope="module")
def world():
    w = make_world(components={"arm": {"fidelity": "dynamic"}})
    yield w
    w.close()


@pytest.fixture
def w(world):
    world.load_scene([])
    world.reset()
    if world.arm.enabled or world.arm.fault:          # a clean motor state per test (reset keeps it)
        world.arm_op("disable", {})
        world.arm.fault = 0
        world.reset()
    return world


P = C.load_profile("soft")
DT = 0.01                                              # the arm node's tick


def q_start(w):
    return np.array(w.cfg["components"]["arm"]["start_q"], float)


def bias(w):
    """The torque that holds the arm still in this model: MuJoCo's own gravity (a perfect tau_ff)."""
    return w.d.qfrc_bias[w.arm_v].copy()


def send(w, q, tau, kp=None, kd=None, qd=None, g=None):
    a = {"q": [float(v) for v in q], "qd": [0.0] * 6 if qd is None else [float(v) for v in qd],
         "kp": list(P.kp if kp is None else kp), "kd": list(P.kd if kd is None else kd),
         "tau": [float(v) for v in tau]}
    if g is not None:
        a.update(gripper=True, g_q=float(g), g_qd=0.0, g_kp=P.g_kp, g_kd=P.g_kd, g_tau=0.0)
    w.arm_op("send", a)


def run(w, seconds, q, tau_fn, hold_base=False, **kw):
    """The node's loop: one frame per 10 ms tick, tau_ff recomputed from the measured pose. ``hold_base``: the
    swerve modules on at zero speed (a coasting base yaws under the arm's reaction)."""
    for _ in range(int(round(seconds / DT))):
        if hold_base:
            w.swerve_op("modules", {"steer": {n: [True, 0.0] for n in w.module_names},
                                    "drive": {n: [True, 0.0] for n in w.module_names}})
        q_t = q(w) if callable(q) else q
        send(w, q_t, tau_fn(w), **kw)
        w.step(int(round(DT / w.m.opt.timestep)))
    return w.arm_op("read", {})


def node_gravity_at(q):
    """The arm node's gravity feedforward: its model's g(q) x the profile's tau_scale."""
    from robot_utils.nodes.arm.kin import ArmKinematics
    if not hasattr(node_gravity_at, "kin"):
        node_gravity_at.kin = ArmKinematics({})
    return node_gravity_at.kin.gravity(np.asarray(q, float)) * P.tau_scale


def node_gravity(w):
    return node_gravity_at(w.d.qpos[w.arm_q].copy())


def test_limits_and_model_are_the_robot_configs(w):
    tau, g_tau = C.motor_torque_limits()
    assert tau.tolist() == [27.0] * 3 + [7.0] * 3 and g_tau == 7.0     # 4340P, 4310 (the profile's models)
    m = w.m
    assert np.allclose(m.actuator_forcerange[w.arm_a, 1], tau)
    assert m.actuator_forcerange[w.finger_a, 1] == pytest.approx(g_tau / (C.GAP_PER_RAD / 2))
    dc = w.cfg["components"]["arm"]["dynamic"]
    assert np.allclose(m.dof_armature[w.arm_v], dc["armature"]) and np.allclose(m.dof_frictionloss[w.arm_v], dc["coulomb"])
    assert np.allclose(m.dof_damping[w.arm_v], dc["viscous"])
    assert w.arm_op("hello", {})["fidelity"] == "dynamic"
    with pytest.raises(Exception, match="fidelity"):
        make_world(components={"arm": {"fidelity": "spring"}})


def test_tracks_a_target_with_the_node_gains_and_deflects_by_load_over_kp(w):
    w.arm_op("enable", {})
    q0 = q_start(w)
    goal = q0 + np.array([0.4, -0.3, 0.3, 0.2, 0.3, -0.4])
    T = 1.0                                             # a ramp at <= 0.5 rad/s (within every joint's speed cap)
    t0 = w.t
    ramp = lambda w: q0 + (goal - q0) * min(1.0, (w.t - t0) / T)       # noqa: E731
    r = run(w, 0.5, ramp, bias)
    lag = np.abs(np.array(r["q"]) - ramp(w))
    v = np.abs(goal - q0) / T
    m, dc = w.m, w.cfg["components"]["arm"]["dynamic"]
    delay = DT + float(dc["command_delay"])              # the frame's hold over a tick, the transport
    expect = ((P.kd + m.dof_damping[w.arm_v]) * v + m.dof_frictionloss[w.arm_v]) / P.kp + v * delay
    assert np.all(lag < 1.3 * expect + 0.002), (lag, expect)    # the MIT law's velocity lag (kd v / kp), no more
    r = run(w, 1.0, goal, bias)
    assert np.allclose(r["q"], goal, atol=0.006)        # at rest: within the dry friction's deadband (0.3 / 72)
    assert r["status"][:6] == [1] * 6
    # a static load the feedforward leaves out deflects the joint by load / kp: joint 3 (elbow) and 4 (wrist pitch)
    for kp3 in (96.0, 48.0):
        kp = P.kp.copy()
        kp[2] = kp3
        miss = np.array([0, 0, 1.0, 1.0, 0, 0])         # tau_ff without joints 3 and 4's gravity
        r = run(w, 1.5, goal, lambda w: bias(w) * (1 - miss), kp=kp)
        dq = np.array(r["q"]) - goal
        g = bias(w)
        for i, k in ((2, kp3), (3, P.kp[3])):
            expect = -g[i] / k
            assert dq[i] == pytest.approx(expect, abs=abs(expect) * 0.15 + w.m.dof_frictionloss[w.arm_v[i]] / k), (i, k)
        assert np.allclose(r["tau"][2:4], P.kp[2:4] * 0 + np.array([kp3, P.kp[3]]) * -dq[2:4] + 0, atol=0.05)


def test_gravity_is_fitted_to_the_calibrated_arm(w):
    """The arm's masses are fitted so MuJoCo's gravity is the node's model x the profile's tau_scale (what holds the
    real arm): on fresh poses the residual is within each joint's dry friction (stiction holds the rest)."""
    fit = w.gravity_fit
    assert fit is not None and fit["tau_scale"] == P.tau_scale.tolist()
    coulomb = np.array(w.cfg["components"]["arm"]["dynamic"]["coulomb"])
    assert np.all(np.array(fit["max"]) < coulomb)                     # validation poses, every joint
    assert np.all(np.array(fit["max"])[3:] < 0.05) and max(fit["max_before"]) > 0.6
    rng = np.random.default_rng(7)                                    # and here, independently, at the model now
    ids = [w.m.joint(n).id for n in ("arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5",
                                     "arm_joint6")]
    lo, hi = w.m.jnt_range[ids, 0], w.m.jnt_range[ids, 1]
    import mujoco
    d = mujoco.MjData(w.m)
    for _ in range(20):
        q = rng.uniform(lo, hi)
        tau, _ = w._gravity_rows(d, q, [])
        from robot_utils.nodes.arm.kin import ArmKinematics
        err = tau - ArmKinematics({}).gravity(q) * P.tau_scale
        assert np.all(np.abs(err) < coulomb), (q, err)


def test_holds_with_the_node_feedforward_and_sags_without(w):
    w.arm_op("enable", {})
    q0 = q_start(w)
    r = run(w, 1.5, q0, node_gravity)                                   # the node's model (pinocchio x tau_scale)
    node = np.array(r["q"]) - q0
    assert np.max(np.abs(node)) < 0.002                                 # no droop (was 0.037 rad on the wrist)
    g = bias(w)
    r = run(w, 1.5, q0, lambda w: np.zeros(6))                          # no feedforward: sags by g / kp
    none = np.array(r["q"]) - q0
    assert none[3] == pytest.approx(-g[3] / P.kp[3], rel=0.2) and abs(none[3]) > 0.08
    assert abs(none[2]) > 0.02


@pytest.mark.parametrize("pose", [[0, -0.3, -0.6, 0, 0, 0], [0, -0.6, -0.9, 0, 0.5, 0], [0.3, -1.0, -1.2, 0.3, 0.8, 0.2],
                                  [0, -1.2, -0.5, -0.5, -0.6, 1.0], [-0.5, -0.8, -1.6, 0.8, 1.0, -0.5],
                                  [0, -1.6, -1.4, 0, 1.2, 0]])
def test_float_holds_on_the_node_feedforward(w, pose):
    """The node's float: kp = kd = 0, tau_ff = its gravity model x tau_scale at the measured joints (a tick old).
    As on the robot it holds: under 1 deg in 10 s at each pose."""
    q0 = np.array(pose, float)
    w.arm_op("enable", {})
    w.d.qpos[w.arm_q] = q0
    w.d.qvel[w.arm_v] = 0.0
    prev = w.arm_op("read", {})
    for _ in range(1000):
        send(w, q0, node_gravity_at(prev["q"]), kp=np.zeros(6), kd=np.zeros(6))
        prev = w.arm_op("read", {})
        w.step(10)
    drift = np.degrees(np.abs(np.array(w.arm_op("read", {})["q"]) - q0))
    assert np.all(drift < 1.0), drift


def test_torque_is_clamped_at_the_motor_limit(w):
    w.arm_op("enable", {})
    q = w.d.qpos[w.arm_q].copy()
    tau = bias(w)
    tau[0], tau[3] = 100.0, -100.0                       # beyond 27 / 7 N m
    send(w, q, tau)
    w.step(3)
    r = w.arm_op("read", {})
    assert r["tau"][0] == pytest.approx(27.0) and r["tau"][3] == pytest.approx(-7.0)
    kp = P.kp.copy()
    send(w, q + np.array([0, 0, 0, 0, 2.0, 0]), bias(w), kp=kp)        # 18 x 2 rad = 36 N m demanded on joint 5
    w.step(3)
    assert w.arm_op("read", {})["tau"][4] == pytest.approx(7.0)
    w.arm_op("disable", {})


def test_kp_zero_floats_on_kd_and_tau_ff(w):
    w.arm_op("enable", {})
    q0 = w.d.qpos[w.arm_q].copy()
    zero = np.zeros(6)
    garbage = q0 + 5.0                                   # the position is ignored at kp = 0
    r = run(w, 1.0, garbage, bias, kp=zero)
    assert np.max(np.abs(np.array(r["q"]) - q0)) < 0.01  # floats where it is (gravity cancelled, friction)
    w.d.qvel[w.arm_v[0]] = 1.0                           # a shove of the base yaw joint: kd brakes it
    r = run(w, 0.3, garbage, bias, kp=zero)
    assert abs(r["qd"][0]) < 0.05
    moved = r["q"][0] - q0[0]
    inertia = 0.073 + w.m.dof_armature[w.arm_v[0]]      # joint 1's (the model's, about 0.07 kg m^2, + armature)
    assert moved == pytest.approx(1.0 * inertia / (P.kd[0] + w.m.dof_damping[w.arm_v[0]]), rel=0.5)  # v0 I / kd
    qd = np.zeros(6)
    qd[0] = 0.5                                          # kd (qd_des - qd): a velocity reference drives it
    r = run(w, 0.5, garbage, bias, kp=zero, qd=qd)
    assert r["qd"][0] == pytest.approx(0.5 - 0.3 / P.kd[0] - 0.1 * 0.5 / P.kd[0], abs=0.05)


def test_watchdog_and_disable_go_passive(w):
    w.arm_op("enable", {})
    q0 = q_start(w)
    run(w, 0.5, q0, bias)
    q_held = np.array(w.arm_op("read", {})["q"])
    w.step(150)                                          # no frames for > 0.1 s: the CAN watchdog
    r = w.arm_op("read", {})
    assert r["status"][:6] == [0xD] * 6 and not w.arm.enabled
    assert np.allclose(r["tau"], 0.0) and w.d.actuator_force[w.finger_a] == 0.0
    w.step(500)
    sunk = np.array(w.arm_op("read", {})["q"]) - q_held
    assert abs(sunk[3]) > 0.1                            # passive: the wrist sinks under its weight
    # disable: the same zero torque, status 0
    w.arm.fault = 0
    w.reset()
    w.arm_op("enable", {})
    run(w, 0.5, q0, bias)
    w.arm_op("disable", {})
    w.step(5)
    r = w.arm_op("read", {})
    assert r["status"] == [0] * 7 and np.allclose(r["tau"], 0.0)
    w.step(500)
    assert abs(w.arm_op("read", {})["q"][3] - q0[3]) > 0.1


def test_reset_keeps_the_motors_on_in_dynamic_mode(w):
    q0 = q_start(w)
    w.arm_op("enable", {})
    goal = q0 + np.array([0.3, -0.2, 0.2, 0.0, 0.2, -0.3])
    run(w, 1.5, goal, bias)
    assert np.allclose(w.arm_op("read", {})["q"], goal, atol=0.006)
    w.reset()
    r = w.arm_op("read", {})
    assert r["status"][:7] == [1] * 7 and w.arm.cmd is None and not w.arm.pending
    assert np.allclose(r["q"], q0, atol=2e-3)            # held at the start pose through the settle
    r = run(w, 1.5, goal, bias)
    assert np.allclose(r["q"], goal, atol=0.006) and r["status"][:6] == [1] * 6
    w.step(150)
    assert w.arm_op("read", {})["status"][:6] == [0xD] * 6
    w.reset()
    assert w.arm.fault == 0xD and not w.arm.enabled


def test_command_delay_holds_a_frame_back(w):
    delay = float(w.cfg["components"]["arm"]["dynamic"]["command_delay"])
    assert delay > 0
    w.arm_op("enable", {})
    q = w.d.qpos[w.arm_q].copy()
    send(w, q, bias(w))
    n = int(round(delay / w.m.opt.timestep))
    w.step(n)
    assert w.arm.cmd is None and len(w.arm.pending) == 1 and np.allclose(w.d.actuator_force[w.arm_a], 0.0)
    w.step(1)
    assert w.arm.cmd is not None and not w.arm.pending


def test_push_into_a_fixed_box_deflects_by_tau_over_kp(w):
    """Joint 1 (yaw: no gravity) commanded through a fixed box: the hand stops at the box, the servo's push
    kp (q_cmd - q) is what the contact carries (within the joint's dry friction), and the deflection is that torque / kp."""
    import mujoco
    q0 = q_start(w)
    w.arm_op("enable", {})
    run(w, 0.5, q0, bias)
    m, d = w.m, w.d
    j1 = m.joint("arm_joint1")
    anchor, axis = d.xanchor[j1.id].copy(), d.xaxis[j1.id].copy()
    p = d.xpos[m.body("arm_end_link").id].copy()

    def rot(v, a):                                        # Rodrigues about the joint axis
        return v * math.cos(a) + np.cross(axis, v) * math.sin(a) + axis * np.dot(axis, v) * (1 - math.cos(a))
    c = anchor + rot(p - anchor, 0.25)                    # where the hand would be 0.25 rad further on
    w.add_object({"name": "wall", "shape": "box", "size": [0.06, 0.06, 0.06], "fixed": True, "xyz": c.tolist()})
    push = 0.5
    goal = q0 + np.array([push, 0, 0, 0, 0, 0])
    t0 = w.t
    ramp = lambda w: q0 + (goal - q0) * min(1.0, (w.t - t0) / 1.0)     # noqa: E731
    run(w, 1.0, ramp, bias, hold_base=True)
    r = run(w, 1.0, goal, bias, hold_base=True)
    q = np.array(r["q"])
    assert 0.03 < q[0] - q0[0] < push - 0.05              # stopped by the box, short of the goal
    dq = goal[0] - q[0]                                   # the deflection
    tau = P.kp[0] * dq - P.kd[0] * r["qd"][0]             # the motor's push (the MIT law)
    assert r["tau"][0] == pytest.approx(min(tau, 27.0), abs=0.05)
    m, d = w.m, w.d                                       # (adding the box recompiled the model)
    box = m.body("obj_wall").id
    load = 0.0                                            # the box's torque about joint 1, from the contact forces
    for i in range(d.ncon):
        c = d.contact[i]
        b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        if box not in (b1, b2):
            continue
        f = np.zeros(6)
        mujoco.mj_contactForce(m, d, i, f)
        fw = c.frame.reshape(3, 3).T @ f[:3]              # world frame: the force geom1 exerts on geom2
        if b1 != box:
            fw = -fw                                      # the force on the arm
        load += float(np.dot(np.cross(c.pos - anchor, fw), axis))
    assert load < -1.0                                    # the hand is on the box, which pushes back
    assert -load == pytest.approx(r["tau"][0], abs=m.dof_frictionloss[w.arm_v[0]] + 0.2)
    assert dq == pytest.approx(-load / P.kp[0], abs=(0.3 + 0.2) / P.kp[0])    # deflection = tau / kp
    w.remove_object("wall")
    w.arm_op("disable", {})


def test_gripper_mit_law_moves_the_jaws(w):
    w.arm_op("enable", {})
    q0 = q_start(w)
    r = run(w, 1.0, q0, bias, g=-2.0)
    friction = w.cfg["components"]["arm"]["dynamic"]["gripper"]["coulomb"]
    assert r["g_q"] == pytest.approx(-2.0, abs=friction / P.g_kp + 0.01)
    assert r["g_q"] > -2.0                               # short of the goal by the dry friction (kp 1.0 N m/rad)
    r = run(w, 1.0, q0, bias, g=0.0)
    assert r["g_q"] == pytest.approx(0.0, abs=friction / P.g_kp + 0.01)
    w.arm_op("gripper_motor", {"on": False})             # damping: zero torque
    w.step(5)
    assert w.d.actuator_force[w.finger_a] == 0.0
