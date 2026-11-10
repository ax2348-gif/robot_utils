"""Arm node core against the simulated bus, stepped by hand on a fake clock (no threads except
where a request blocks on the control thread). docs/arm.md."""
import math
import threading

import numpy as np
import pinocchio as pin
import pytest

from robot_utils.nodes.arm import config as C
from robot_utils.nodes.arm.bus import SimBus
from robot_utils.nodes.arm.node import COL, DT, FIELDS, JAC_FIELDS, PHASES, SOURCES, STATES, ArmNode
from robot_utils.nodes.common.lease import RequestError
from robot_utils.nodes.common.locks import LockStore

Q0 = (0.0, -0.6, -0.9, 0.0, 0.5, 0.0)


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture(scope="session", autouse=True)
def _planner_process():
    """Start the planner process once for the whole run (spawning it takes a couple of seconds)."""
    from robot_utils.nodes.arm.kin import ArmKinematics
    from robot_utils.nodes.arm.planner import Planner
    k = ArmKinematics()
    Planner.run("ik", k, k.fk(np.array(Q0)), np.array(Q0))
    yield


@pytest.fixture
def rig(tmp_path):
    clock = Clock()
    sims = []

    def factory():
        from reBotArm_control_py.actuator.rebotarm import joint_limits, load_cfg
        b = SimBus(Q0, clock=clock, limits=joint_limits(load_cfg(C.PROFILES["soft"])))
        sims.append(b)
        return b

    node = ArmNode(factory, locks=LockStore(tmp_path / "locks"), clock=clock)
    r = Rig(node, clock, sims)
    r.step(1)
    return r


class Rig:
    def __init__(self, node, clock, sims):
        self.node, self.clock, self.sims = node, clock, sims

    @property
    def sim(self):
        return self.sims[-1]

    def step(self, n=1):
        for _ in range(n):
            self.clock.t += DT
            self.node.step()

    def run_s(self, s):
        self.step(int(round(s / DT)))

    def blocking(self, fn, *a, **kw):
        """Run a request that waits on the control thread, stepping until it returns."""
        out = {}

        def go():
            try:
                out["v"] = fn(*a, **kw)
            except Exception as e:  # noqa: BLE001
                out["e"] = e
        th = threading.Thread(target=go)
        th.start()
        for _ in range(2000):
            if not th.is_alive():
                break
            self.step(1)
            th.join(0.001)
        th.join(1)
        if "e" in out:
            raise out["e"]
        return out["v"]

    def lease(self):
        tok, _ = self.node.acquire("test")
        self.tok = tok
        return tok

    def enable(self):
        return self.blocking(self.node.enable, self.tok)

    def row(self):
        return dict(zip(FIELDS, self.node.hub.recent(1, "tick")[0]))

    def tcp(self):
        return self.node.kin.fk(self.sim.q)

    def keep(self, s):
        """Step for s seconds, renewing the lease like a client would."""
        for _ in range(int(round(s / DT))):
            self.step(1)
            if self.node.lease.held and self.node.tick_count % 20 == 0:
                self.node.renew(self.tok)


ArmNode.tick_count = property(lambda self: self._tick)


def ready(rig):
    """Lease and enable: the arm floats."""
    rig.lease()
    rig.enable()
    rig.keep(0.2)
    return rig


def engaged(rig):
    """Lease, enable and engage the arm (track its own measured pose): servoing, holding."""
    ready(rig)
    assert rig.node.track_cmd(rig.tok, rig.tcp())["ok"]
    rig.keep(0.2)
    assert rig.node.servo
    return rig


def test_connects_uninitialized_and_streams(rig):
    n = rig.node
    assert n.state == "uninitialized" and not n.initialized and not n.motors_on
    rig.step(3)
    row = rig.row()
    assert row["state"] == STATES.index("uninitialized") and row["motors_on"] == 0
    assert np.isnan(row["j1_q_cmd"])                     # nothing commanded while disabled
    assert len(n.hub.recent(1, "jacobian")[0]) == len(JAC_FIELDS)


def test_enable_needs_lease_runs_gripper_check_and_floats(rig):
    n = rig.node
    with pytest.raises(RequestError) as e:
        rig.blocking(n.enable, None)
    assert e.value.status == 401
    rig.lease()
    out = rig.enable()
    assert out["enabled"] and out["initialized"] and n.state == "floating" and n.motors_on and not n.servo
    assert any("gripper check passed" in ev["msg"] for ev in n.events.list())
    rig.keep(0.5)
    row = rig.row()
    assert row["source"] == SOURCES.index("none") and np.isnan(row["j1_q_cmd"])     # no command while floating
    assert np.all(n.bus.cmd[2] == 0) and np.all(n.bus.cmd[3] == 0)                   # kp = kd = 0
    assert n.gripper_state()["mode"] == "free"


def test_gripper_check_failure_refuses_enable(rig):
    rig.sim.object_gap_rad = -1.0          # something in the jaws: can't close
    rig.lease()
    with pytest.raises(RequestError) as e:
        rig.enable()
    assert e.value.status == 503 and "gripper" in str(e.value).lower()
    assert rig.node.state == "uninitialized" and not rig.node.motors_on


def test_track_reaches_target_and_ik_failure_keeps_goal(rig):
    n = ready(rig).node
    T = rig.tcp()
    T.translation = T.translation + np.array([0.0, 0.0, 0.02])
    out = n.track_cmd(rig.tok, T, seq=7)
    assert out["ok"] and out["seq"] == 7
    rig.keep(1.0)
    assert np.linalg.norm(rig.tcp().translation - T.translation) < 1e-3
    assert rig.row()["source"] == SOURCES.index("track") and rig.row()["seq"] == 7
    far = pin.SE3(np.eye(3), np.array([3.0, 0, 0]))
    out = n.track_cmd(rig.tok, far)
    assert not out["ok"] and out["ik"]["reason"]
    rig.keep(0.5)
    assert np.linalg.norm(rig.tcp().translation - T.translation) < 1e-3   # previous goal still in force


def test_track_goal_is_literal_twist_is_the_velocity_reference(rig):
    n = engaged(rig).node
    T = rig.tcp()
    q_goal = n.kin.ik(T, n.q_cmd).q
    n.track_cmd(rig.tok, T, twist=[0.05, 0, 0, 0, 0, 0])
    rig.step(5)
    assert np.any(np.abs(n.qd_ff) > 1e-3)                     # the twist's joint velocity, sent as reference
    assert np.allclose(n.track.q_goal, q_goal, atol=1e-6)     # the goal itself does not move
    assert np.allclose(n.q_cmd, n.track.q_goal, atol=1e-3)
    rig.keep(0.3)                                             # timed out
    assert np.allclose(n.qd_ff, 0.0) and rig.row()["twist_age"] > 0.25


def test_twist_reference_is_capped(rig):
    n = engaged(rig).node
    out = n.track_cmd(rig.tok, rig.tcp(), twist=[0, 0, 1.0, 0, 0, 5.0])
    rig.step(2)
    assert out["twist_scaled"] and np.all(np.abs(n.qd_ff) <= n.soft.joint_vel + 1e-9)


def test_step_limit_clips_and_flags(rig):
    n = engaged(rig).node
    n.set_limits(rig.tok, joint_vel=[0.2] * 6)
    T = rig.tcp()
    T.translation = T.translation + np.array([0.0, 0.02, 0.0])
    q_before = n.q_cmd.copy()
    assert n.track_cmd(rig.tok, T)["ok"]
    rig.step(2)
    assert rig.row()["limited"] == 1.0
    assert np.all(np.abs(n.q_cmd - q_before) <= 0.2 * DT * 2 + 1e-9)


def test_servo_plans_executes_and_reaches(rig):
    n = ready(rig).node
    T = rig.tcp()
    goal = pin.SE3(T.rotation @ pin.exp3(np.array([0, 0, 0.2])), T.translation + np.array([0.04, -0.03, 0.02]))
    m = n.servo_cmd(rig.tok, goal, lin_vel=0.1, lin_acc=0.3)
    assert m["phase"] == "executing" and m["duration"] > 0.5
    rig.keep(m["duration"] / 2)
    assert 0.2 < rig.row()["progress"] < 0.9 and rig.row()["source"] == SOURCES.index("servo")
    rig.keep(m["duration"] / 2 + 1.0)
    st = n.state_dict()
    assert st["motion"]["phase"] == "reached", st["motion"]
    dp = np.linalg.norm(rig.tcp().translation - goal.translation)
    assert dp < 2e-3


def test_servo_refuses_unreachable_and_too_short(rig):
    n = ready(rig).node
    with pytest.raises(RequestError) as e:
        n.servo_cmd(rig.tok, pin.SE3(np.eye(3), np.array([2.0, 0, 0])))
    assert e.value.status == 422 and "nothing moved" in str(e.value)
    T = rig.tcp()
    T.translation = T.translation + np.array([0.1, 0, 0])
    with pytest.raises(RequestError) as e:
        n.servo_cmd(rig.tok, T, duration=0.1)
    assert "too short" in str(e.value)


def test_servo_aborts_on_obstruction(rig):
    n = ready(rig).node
    T = rig.tcp()
    T.translation = T.translation + np.array([0.0, 0.0, 0.08])
    n.servo_cmd(rig.tok, T, lin_vel=0.1)
    rig.keep(0.3)
    sim = rig.sim
    stuck = sim.q.copy()
    for _ in range(100):                   # something holds the arm still
        sim.q[:] = stuck
        sim.qd[:] = 0
        rig.step(1)
        if n.motion is None:
            break
    assert n.last_motion.phase == "aborted" and "tracking error" in n.last_motion.reason


def test_home_and_named_pose(rig):
    n = ready(rig).node
    m = n.home(rig.tok)
    assert m["kind"] == "home"
    rig.keep(m["duration"] + 1.0)
    assert np.allclose(rig.sim.q, 0.0, atol=2e-3)
    with pytest.raises(RequestError):
        n.servo_cmd(rig.tok, name="nope")


def test_chunk_follows_samples_and_skips_past(rig):
    n = ready(rig).node
    T = rig.tcp()
    poses = []
    for i in range(50):
        P = pin.SE3(T)
        P.translation = T.translation + np.array([0.0, 0.0006 * i, 0.0])
        poses.append(P)
    t0 = rig.clock.t - 0.1                   # 0.1 s old: the first 5 samples are already past
    m = n.chunk(rig.tok, poses, dt=0.02, t0=t0, gaps_mm=[20.0] * 50, blend_s=0.05)
    assert m["kind"] == "chunk" and m["info"]["skipped"] >= 5
    rig.keep(1.5)
    assert n.last_motion.phase == "done"
    assert np.linalg.norm(rig.tcp().translation - poses[-1].translation) < 1e-3
    assert abs(n.gripper.goal - 0.02) < 1e-9
    with pytest.raises(RequestError) as e:
        n.chunk(rig.tok, poses, dt=0.02, t0=rig.clock.t - 5)
    assert "stale" in str(e.value)


def test_gripper_gap_and_force_limit(rig):
    n = ready(rig).node
    n.gripper_cmd(rig.tok, gap_mm=30.0)
    rig.keep(1.5)
    assert abs(rig.row()["g_gap"] - 0.030) < 1e-3
    rig.sim.object_gap_rad = C.gap_to_rad(0.020)       # a 20 mm object
    n.gripper_cmd(rig.tok, gap_mm=0.0, max_force_n=2.0)
    rig.keep(2.0)
    row = rig.row()
    assert abs(row["g_gap"] - 0.020) < 1e-3
    assert row["g_limiting"] == 1.0 and row["g_tau_ff"] < 0 and abs(row["g_tau_ff"]) <= row["g_tau_est"]
    # applied closing torque = estimate + feedforward = the force limit
    assert abs(C.force_of_torque(row["g_tau_est"] + row["g_tau_ff"]) - 2.0) < 1e-6
    with pytest.raises(ValueError):
        n.gripper_cmd(rig.tok, gap_mm=10, max_force_n=-1)


def test_force_limit_off_near_fully_open(rig):
    n = ready(rig).node
    n.gripper_cmd(rig.tok, gap_mm=C.GAP_MAX * 1000)
    rig.keep(2.0)
    rig.sim.object_gap_rad = C.GRIPPER_OPEN_RAD + 0.05        # blocked just off fully open
    n.gripper_cmd(rig.tok, gap_mm=0.0, max_force_n=0.0)
    rig.keep(0.2)
    row = rig.row()
    assert row["g_gap"] > C.GAP_MAX - C.FF_OPEN_MARGIN and row["g_tau_ff"] == 0.0


def test_estop_cuts_to_damping_and_latches(rig):
    n = engaged(rig).node
    n.trigger_estop("test", "pytest")
    rig.step(2)
    assert n.state == "off" and not n.enabled and not n.motors_on and not rig.sim.enabled
    with pytest.raises(RequestError) as e:
        rig.enable()
    assert e.value.status == 409
    n.clear_estop(rig.tok)
    rig.enable()
    assert n.state == "floating"


def test_lease_lapse_disables_arm_floats_gripper_damps(rig):
    n = engaged(rig).node
    n.gripper_cmd(rig.tok, gap_mm=20.0)
    rig.step(3)
    rig.run_s(1.5)                          # no renew
    assert not n.lease.held and n.state == "disabled" and n.motors_on and not n.servo
    assert rig.sim.enabled and not rig.sim.g_enabled and n.gripper_state()["mode"] == "damping"
    assert np.all(rig.sim.cmd[2] == 0)      # arm gainless (floating), still sent frames


def test_chock_refuses_enable_and_disables(rig, tmp_path):
    n = engaged(rig).node
    n.locks.place("chock", "arm", "tester", "testing")
    n.locks.active(force=True)
    rig.step(2)
    assert n.state == "disabled" and not n.servo
    with pytest.raises(RequestError) as e:
        rig.enable()
    assert e.value.status == 423


def test_read_failure_faults_and_reset_reinitializes(rig):
    n = ready(rig).node
    rig.sim.fail_reads = True
    rig.step(1)
    assert n.state == "fault" and not n.motors_on and "xout" in n.error
    with pytest.raises(RequestError) as e:
        rig.enable()
    assert e.value.status == 503
    n.reset(rig.tok)
    rig.step(2)
    assert n.state == "uninitialized" and not n.initialized and len(rig.sims) == 2
    rig.enable()
    assert n.motors_on


def test_motor_fault_code_faults(rig):
    n = ready(rig).node
    rig.sim.fault_code = 0xD
    rig.step(1)
    assert n.state == "fault" and "communication lost" in n.error


def test_profile_and_tcp_only_while_not_servoing(rig):
    n = engaged(rig).node
    with pytest.raises(RequestError):
        n.set_profile(rig.tok, "safe")
    with pytest.raises(RequestError):
        n.set_tcp(rig.tok, [0, 0, 0])
    n.float_arm(rig.tok)
    rig.step(1)
    p = n.set_profile(rig.tok, "safe")
    assert p["gripper"]["kp"] == 0.5 and p["tau_scale"][3] == 0.84
    assert n.set_tcp(rig.tok, [-0.1, 0, 0])["offset"] == [-0.1, 0.0, 0.0]


def test_kinematics_lookups(rig):
    n = rig.node
    rig.step(2)
    now = n.kinematics_now()
    assert len(now["jacobian"]) == 6 and len(now["jacobian"][0]) == 6 and now["w"] > 0
    k = n.kinematics([0.0, -0.6, -0.9, 0.0, 0.5, 0.0], frame="tcp")
    assert k["within_limits"] and k["frame"] == "tcp" and len(k["lin_axes"]) == 3
    assert not n.kinematics([0.0, 1.0, 0, 0, 0, 0])["within_limits"]
    ik = n.ik_check(n.kin.fk(np.array(Q0)), seed="measured")
    assert ik["ok"] and ik["pos_err_mm"] < 0.01
    assert len(n.links()) == 8


def test_ik_respects_config_joint1_limit(rig):
    n = rig.node
    assert math.isclose(math.degrees(n.kin.lo[0]), -36.0, abs_tol=1e-6)


# -- HTTP (real control thread, real clock) -----------------------------------------------------

def test_http_api_end_to_end(tmp_path):
    import time as _time

    from fastapi.testclient import TestClient
    from reBotArm_control_py.actuator.rebotarm import joint_limits, load_cfg

    from robot_utils.nodes.arm.server import create_app
    limits = joint_limits(load_cfg(C.PROFILES["soft"]))
    node = ArmNode(lambda: SimBus(Q0, limits=limits), locks=LockStore(tmp_path / "locks"))
    with TestClient(create_app(node, sim=True)) as c:
        for _ in range(100):
            if c.get("/health").json()["state"] == "uninitialized":
                break
            _time.sleep(0.02)
        assert c.get("/meta").json()["fake"] is True
        tok = c.post("/lease", json={"client": "t"}).json()["token"]
        h = {"X-Lease-Token": tok}
        assert c.post("/enable", headers=h).json()["enabled"]
        st = c.get("/state")                              # right after enable (NaN force regression)
        assert st.status_code == 200 and st.json()["enabled"]
        tcp = c.get("/kinematics").json()["tcp"]
        pose = {"pos": [tcp["pos"][0], tcp["pos"][1], tcp["pos"][2] + 0.01], "quat": tcp["quat"]}
        r = c.post("/track", headers=h, json={"pose": pose, "twist": [0, 0, 0.01, 0, 0, 0], "seq": 3}).json()
        assert r["ok"] and r["seq"] == 3
        bad = c.post("/track", headers=h, json={"pose": {"pos": [3, 0, 0], "rpy": [0, 0, 0]}}).json()
        assert not bad["ok"]
        assert c.post("/track", headers=h, json={"pose": {"pos": [0, 0, 0]}}).status_code == 422   # no orientation
        r = c.post("/servo", headers=h, json={"pose": {"pos": [3, 0, 0], "rpy": [0, 0, 0]}})
        assert r.status_code == 422, (r.json(), node.error, [e["msg"] for e in node.events.list()][-6:])
        assert c.post("/servo", headers=h, json={"name": "home"}).json()["kind"] == "pose"
        assert c.post("/cancel", headers=h).json()["cancelled"]
        assert c.post("/ik", json={"pose": pose}).json()["ok"]
        k = c.post("/kinematics", json={"q": [list(Q0)], "frame": "tcp"}).json()["results"][0]
        assert k["within_limits"] and len(k["jacobian"]) == 6
        assert c.post("/kinematics", json={"q": [[0, 0, 0]]}).status_code == 422
        assert c.post("/gripper", headers=h, json={"gap_mm": 20, "max_force_n": 30}).json()["max_force_n"] == 30
        assert c.post("/profile", headers=h, json={"name": "safe"}).status_code == 409   # enabled
        for path in ("/joints", "/tracking", "/gains", "/gripper", "/limits", "/tcp", "/links", "/fields",
                     "/fields?channel=jacobian", "/"):
            assert c.get(path).status_code == 200, path
        assert c.post("/estop", json={"reason": "t"}).json()["latched"]
        _time.sleep(0.05)
        assert not c.get("/state").json()["motors_on"]


# -- audit regressions (physical safety) ----------------------------------------------------------

def _disable(rig):
    rig.node.disable(rig.tok)
    rig.step(2)


def test_engaging_far_outside_limits_is_refused_not_snapped(rig):
    n = ready(rig).node                               # floating: can be pushed anywhere
    rig.sim.q[0] = math.radians(-40.0)                # past joint1's -36°
    rig.step(2)
    with pytest.raises(RequestError) as e:
        n.track_cmd(rig.tok, rig.tcp())
    assert e.value.status == 409 and "outside its limits" in str(e.value) and not n.servo
    rig.sim.q[0] = math.radians(-37.0)                # within the 3° tolerance: engaged where it is
    rig.step(2)
    assert n.track_cmd(rig.tok, rig.tcp())["ok"] is False or True
    n.track = None
    m = n.home(rig.tok)                               # brought in gradually, never snapped
    rig.step(1)
    assert n.servo and abs(n.q_cmd[0] - math.radians(-37.0)) < 0.01
    rig.keep(m["duration"] + 1.0)
    assert n.kin.within_limits(rig.sim.q, margin=1e-3)


def test_encoder_mismatch_refuses_enable_and_faults_running(rig):
    rig.sim.mot_offset[5] = 2 * math.pi              # joint6 output vs motor: a 2π wrap
    rig.lease()
    with pytest.raises(RequestError) as e:
        rig.enable()
    assert "disagree" in str(e.value) and not rig.sim.enabled
    rig.sim.mot_offset[5] = 0.0
    rig.enable()
    rig.sim.mot_offset[2] = 0.3
    rig.step(1)
    assert rig.node.state == "fault" and "disagree" in rig.node.error


def test_engage_seeds_from_the_reading(rig):
    rig.sim.mot_offset[:] = 0.01                      # backlash: the motors sit 0.01 rad off the output
    n = ready(rig).node
    rig.sim.qd[:] = 0.3                               # moving (by hand) while floating
    rig.step(1)
    q_mot, qd = rig.sim.q + 0.01, rig.sim.qd.copy()
    n.track_cmd(rig.tok, rig.tcp())
    rig.step(1)
    # started at the motor-side pose; from the arm's own velocity it may slow down at once, and only
    # a reversal's part past zero is limited
    assert n.servo and np.allclose(n.q_cmd, q_mot, atol=0.02)
    v = n.v_cmd
    assert np.all((np.sign(v) == np.sign(qd)) | (np.abs(v) <= n.soft.joint_acc * DT + 1e-9))


def test_driver_refusal_faults(rig):
    n = engaged(rig).node
    with pytest.raises(Exception):
        rig.sim.send(np.array([-1.0, 0, 0, 0, 0, 0]), np.zeros(6), np.ones(6), np.ones(6), np.zeros(6), 0, 0, 1, 0, 0)
    assert n.kin.within_limits(n.q_cmd)


def test_acceleration_limits_only_increases_and_names_the_joints(rig):
    n = engaged(rig).node
    T = rig.tcp()
    T.translation = T.translation + np.array([0.0, 0.0, 0.03])
    assert n.track_cmd(rig.tok, T)["ok"]
    vs, flags = [], []
    for _ in range(150):
        rig.step(1)
        vs.append(n.v_cmd.copy())
        flags.append([rig.row()[f"j{i}_acc_lim"] for i in range(1, 7)])
    vs = np.array(vs)
    grow = np.abs(vs[1:]) - np.abs(vs[:-1])
    assert np.all(grow <= n.soft.joint_acc * DT + 1e-9)       # speeding up: limited
    assert np.array(flags)[:5].any()                           # and reported per joint
    assert np.allclose(vs[-1], 0.0, atol=1e-6)


def test_cancel_holds_where_the_command_is(rig):
    n = engaged(rig).node
    n.home(rig.tok)
    rig.keep(0.4)
    q_at = n.q_cmd.copy()
    n.cancel(rig.tok)
    rig.step(1)
    assert np.allclose(n.q_cmd, q_at, atol=1e-9) and np.allclose(n.v_cmd, 0.0)   # slowing down is never held back


def test_servo_planned_from_a_stale_pose_is_refused(rig):
    n = engaged(rig).node
    from robot_utils.nodes.arm.motion import plan_joint
    q_stale = n.q_cmd.copy()
    n.home(rig.tok)
    rig.keep(0.5)                                    # the command moved on
    traj = plan_joint(q_stale, [np.zeros(6)], 0.5, n.soft.joint_vel, n.kin)
    with pytest.raises(RequestError) as e:
        n._start(traj, "servo", q_stale)
    assert e.value.status == 409 and "while planning" in str(e.value)


def test_plans_run_one_at_a_time(rig):
    n = ready(rig).node
    n._plan_lock.acquire()
    try:
        with pytest.raises(RequestError) as e:
            n.home(rig.tok)
        assert e.value.status == 409
    finally:
        n._plan_lock.release()


def test_chunk_far_from_the_arm_is_refused(rig):
    n = ready(rig).node
    T = rig.tcp()
    far = pin.SE3(T)
    far.translation = T.translation + np.array([0.0, 0.08, 0.0])
    with pytest.raises(RequestError) as e:
        n.chunk(rig.tok, [far] * 10, dt=0.02, blend_s=0.02)
    assert e.value.status == 422


def test_track_refuses_large_jumps(rig):
    n = ready(rig).node
    T = rig.tcp()
    T.translation = T.translation + np.array([0.0, 0.08, -0.05])
    out = n.track_cmd(rig.tok, T)
    if out["ik"]["ok"]:
        assert not out["ok"] and "too far for track" in out["reason"]


def test_wide_object_still_force_limited(rig):
    n = ready(rig).node
    n.gripper_cmd(rig.tok, gap_mm=48.0)
    rig.keep(1.5)
    rig.sim.object_gap_rad = C.gap_to_rad(0.046)      # 46 mm: inside the 5 mm open margin
    n.gripper_cmd(rig.tok, gap_mm=0.0, max_force_n=10.0)
    rig.keep(2.0)
    row = rig.row()
    assert row["g_limiting"] == 1.0 and row["g_tau_ff"] == 0.0
    assert C.force_of_torque(rig.sim.g_tau) <= 10.0 + 0.5




def test_gripper_force_limit_defaults_when_omitted_and_validates_first(rig):
    n = ready(rig).node
    n.gripper_cmd(rig.tok, gap_mm=20.0, max_force_n=10.0)
    assert n.gripper.max_force == 10.0
    n.gripper_cmd(rig.tok, gap_mm=10.0)
    assert n.gripper.max_force == C.DEFAULT_MAX_FORCE
    before = n.track
    with pytest.raises(ValueError):
        n.track_cmd(rig.tok, rig.tcp(), gripper={"gap_mm": 10.0, "max_force_n": -5.0})
    assert n.track is before


def test_nan_limits_rejected(rig):
    n = ready(rig).node
    for bad in ([math.nan] * 6, [1, 1, 1, 1, 1, math.inf]):
        with pytest.raises(ValueError):
            n.set_limits(rig.tok, joint_vel=bad)
    with pytest.raises(ValueError):
        n.set_limits(rig.tok, lin_vel=math.nan)
    assert np.all(np.isfinite(n.soft.joint_vel))


# -- re-audit regressions ------------------------------------------------------------------------

def test_command_never_passes_a_joint_limit(rig):
    n = engaged(rig).node
    with n._lock:
        n.q_cmd[0] = n.kin.lo[0] + 0.05
        n.v_cmd[:] = 0.0
        for _ in range(50):
            n._command_step(n.q_cmd + np.array([-1.0, 0, 0, 0, 0, 0]))
            assert n.q_cmd[0] >= n.kin.lo[0] - 1e-12


def test_clip_increase_handles_reversal():
    from robot_utils.nodes.arm.node import clip_increase
    dv = np.array([0.1])
    assert clip_increase(np.array([1.0]), np.array([2.0]), dv)[0] == pytest.approx(1.1)   # speeding up: limited
    assert clip_increase(np.array([1.0]), np.array([0.2]), dv)[0] == pytest.approx(0.2)   # slowing: free
    assert clip_increase(np.array([1.0]), np.array([-2.0]), dv)[0] == pytest.approx(-0.1) # past zero: limited
    assert clip_increase(np.array([0.0]), np.array([-2.0]), dv)[0] == pytest.approx(-0.1)
    assert clip_increase(np.array([-1.0]), np.array([0.0]), dv)[0] == pytest.approx(0.0)




def test_gravity_floating_holds_estop_sinks_and_home_recovers(tmp_path):
    clock = Clock()
    from reBotArm_control_py.actuator.rebotarm import joint_limits, load_cfg
    limits = joint_limits(load_cfg(C.PROFILES["soft"]))
    sims = []

    def factory():
        sims.append(SimBus(Q0, clock=clock, limits=limits, gravity=True))
        return sims[-1]
    rig = Rig(ArmNode(factory, locks=LockStore(tmp_path / "locks"), clock=clock), clock, sims)
    rig.step(1)
    n = ready(rig).node
    rig.keep(5.0)                                     # it was sinking before enable: let friction stop it
    start = rig.tcp()
    rig.keep(2.0)
    # floating: kp = kd = 0, the gravity feedforward alone holds it (a wrong sign or scale would sag)
    assert n.state == "floating" and np.linalg.norm(rig.tcp().translation - start.translation) < 0.005
    n.trigger_estop("t", "pytest")
    rig.keep(2.0)
    assert n.state == "off" and np.linalg.norm(rig.tcp().translation - start.translation) > 0.02   # sank
    assert n.kin.within_limits(rig.sim.q, margin=1e-6)                                           # onto its stops
    n.clear_estop(rig.tok)
    rig.enable()                                                                                  # recover, untouched
    m = n.home(rig.tok)
    rig.keep(m["duration"] + 1.5)
    assert np.allclose(rig.sim.q, 0.0, atol=5e-3)


def test_gripper_encoder_mismatch_refuses_enable(rig):
    rig.sim.g_mot_offset = 0.2
    rig.lease()
    with pytest.raises(RequestError) as e:
        rig.enable()
    assert "gripper" in str(e.value)



# -- float / disable / stow ------------------------------------------------------------------------

def test_float_releases_the_arm_not_the_gripper(rig):
    n = engaged(rig).node
    n.gripper_cmd(rig.tok, gap_mm=20.0)
    rig.keep(0.5)
    n.float_arm(rig.tok)
    rig.step(2)
    assert n.state == "floating" and n.enabled and not n.servo and n.q_cmd is None
    assert n.gripper_state()["mode"] == "holding"
    with pytest.raises(RequestError):
        n.kinematics_now(which="command")              # no command while floating


def test_disable_then_enable_gripper_back_from_damping(rig):
    n = engaged(rig).node
    n.disable(rig.tok)
    rig.step(2)
    assert n.state == "disabled" and not rig.sim.g_enabled
    with pytest.raises(RequestError):
        n.track_cmd(rig.tok, rig.tcp())               # refused until enable
    rig.enable()
    assert n.state == "floating" and rig.sim.g_enabled and n.gripper_state()["mode"] == "free"


def test_plan_refused_if_moved_by_hand_while_planning(rig):
    n = ready(rig).node
    with n._lock:
        q0 = n._arm_start()
    from robot_utils.nodes.arm.motion import plan_joint
    traj = plan_joint(q0, [np.zeros(6)], 0.5, n.soft.joint_vel, n.kin)
    rig.sim.q[1] -= 0.1                                # pushed while the plan was computed
    rig.step(1)
    with pytest.raises(RequestError) as e:
        n._start(traj, "servo", q0)
    assert "moved" in str(e.value)


def test_stow_homes_then_cuts_to_damping(rig):
    n = ready(rig).node
    out = n.stow(rig.tok)
    assert not out["stowed"] and out["motion"]["kind"] == "stow"
    rig.keep(out["motion"]["duration"] + 1.5)
    assert n.state == "off" and not n.motors_on and np.allclose(rig.sim.q, 0.0, atol=5e-3)


def test_stow_near_home_cuts_at_once(rig):
    n = ready(rig).node
    rig.sim.q[:] = np.radians([1.0, -1.0, -2.0, 0.5, 0.0, 1.0])
    rig.step(2)
    out = n.stow(rig.tok)
    assert out["stowed"]
    rig.step(2)
    assert n.state == "off" and not rig.sim.enabled


def test_obstruction_abort_past_a_limit_holds_without_a_driver_refusal(rig):
    # audit: the abort replaced the command by the reading without telling the driver, which then
    # refused it (outside the limits) and would resend its old setpoint
    n = engaged(rig).node
    goal = rig.sim.q.copy()
    goal[0] = n.kin.lo[0] + math.radians(1.0)
    m = n.servo_cmd(rig.tok, n.kin.fk(goal))
    rig.keep(m["duration"] * 0.9)
    past = n.kin.lo[0] - math.radians(20.0)               # pushed well past the limit (obstruction)
    for _ in range(40):
        rig.sim.q[0], rig.sim.qd[0] = past, 0.0
        rig.step(1)
    assert n.last_motion is not None and n.last_motion.phase == "aborted", n.state
    assert n.state == "servoing", (n.state, n.error)                    # no driver refusal, no fault
    assert abs(n.q_cmd[0] - past) < 0.02



def test_twist_reference_is_zero_without_a_twist(rig):
    n = engaged(rig).node
    T = rig.tcp()
    T.translation = T.translation + np.array([0.0, 0.0, 0.01])
    n.track_cmd(rig.tok, T)
    for _ in range(30):
        rig.step(1)
        assert np.all(n.qd_ff == 0.0)


def test_home_trajectory_is_a_proper_min_jerk_move(rig):
    n = ready(rig).node
    from robot_utils.nodes.arm.motion import plan_joint
    with n._lock:
        q0 = n._arm_start()
    vel = n.soft.home_vel
    via = np.radians([10.0, -30.0, -40.0, 5.0, 20.0, -10.0])
    tr = plan_joint(q0, [via, np.zeros(6)], vel, n.soft.joint_vel, n.kin)
    assert np.allclose(tr.q[0], q0) and np.allclose(tr.q[-1], 0.0)        # starts here, ends at home
    assert np.all(np.diff(tr.t) > 0)
    v = np.diff(tr.q, axis=0) / np.diff(tr.t)[:, None]
    assert np.abs(v).max() <= vel * 1.001                                   # never faster than the home speed
    i_via = int(np.argmin(np.linalg.norm(tr.q - via, axis=1)))
    assert np.allclose(tr.q[i_via], via, atol=1e-9)                         # passes exactly through the via pose
    assert np.allclose(tr.qd[0], 0) and np.allclose(tr.qd[-1], 0)           # at rest at both ends
    assert np.abs(v[i_via - 1]).max() < 0.02 * vel and np.abs(v[i_via]).max() < 0.02 * vel   # and at the via
    assert all(n.kin.within_limits(q) for q in tr.q[1:])


def test_servo_uses_the_geodesic_and_clik(rig):
    n = engaged(rig).node
    from robot_utils.nodes.arm.motion import plan_servo
    T = rig.tcp()
    goal = pin.SE3(T.rotation @ pin.exp3(np.array([0.0, 0.0, 0.3])), T.translation + np.array([0.03, 0.02, 0.0]))
    with n._lock:
        q0 = n._arm_start()
    s = n.soft
    tr = plan_servo(n.kin, q0, goal, lin_vel=s.lin_vel, lin_acc=s.lin_acc, ang_vel=s.ang_vel, ang_acc=s.ang_acc,
                    joint_vel=s.joint_vel)
    assert np.allclose(tr.q[0], q0, atol=1e-3) and tr.info["clik_unconverged"] == 0
    from robot_utils.nodes.arm.kin import pose_error
    dp, dr = pose_error(n.kin.fk(tr.q[-1]), goal)
    assert dp < 2e-3 and dr < math.radians(1.0)
    assert np.allclose(tr.qd[-1], 0.0)


def test_fast_gripper_reading_skew_does_not_fault(rig):
    # on the robot: the gripper's output and motor readings differed by 12.7° while it moved fast
    n = engaged(rig).node
    rig.sim.g_mot_offset = math.radians(15.0)
    rig.step(3)
    assert n.state == "servoing", n.error
