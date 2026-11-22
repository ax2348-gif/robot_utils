"""The WBC's lead bound (docs/wbc.md "Lead bound"): each arm joint's next command stays within lead_torque / kp of
the measured joints extrapolated over the arm's delay, so in contact the servo's push kp (q_cmd - q) stays within
lead_torque. The controller harness of test_wbc_controller with a measured arm that lags the command by a delay or
is blocked at a pose; the node with the fake arm for the runtime config, /state and the stream columns."""
import math

import numpy as np
import pinocchio as pin
import pytest

from robot_utils.nodes.wbc.controller import LIMITED, QPController, EEState, Snapshot
from robot_utils.nodes.wbc.model import EESpec, WbcModel, arm_config_limits, fetch_urdf
from robot_utils.nodes.wbc.spec import Config, Hints, make_config
from robot_utils.nodes.wbc.targets import TargetBuffer

from test_wbc_controller import Q0, WHEELS

KP = np.array([72.0, 72.0, 96.0, 18.0, 18.0, 18.0])     # the soft profile's (rebotarm_dm_soft.yaml)


@pytest.fixture(scope="module")
def model():
    urdf, _ = fetch_urdf("local")
    return WbcModel(urdf, (EESpec(arm_limits=arm_config_limits()),))


class ArmSim:
    """The controller on a measured arm: the base locked, the command integrated from the previous command (as the
    node does); the measured arm trails the command by ``delay`` ticks (pure delay: q and qd of the command then),
    or is held at ``blocked`` (an obstacle: q fixed, qd 0) while ``blocked`` is set. ``lead=False``: no measurement
    given (the bound off)."""

    def __init__(self, model, cfg=None, delay=5, lead=True):
        self.m, self.cfg = model, cfg or Config()
        self.ctl = QPController(model, self.cfg, WHEELS)
        self.q, self.v = Q0.copy(), np.zeros(6)
        self.t = 0.0
        self.delay, self.lead = delay, lead
        self.hist = [(Q0.copy(), np.zeros(6))] * (delay + 1)
        self.blocked = None
        self.buf = TargetBuffer("arm", self.cfg.ff_timeout, self.cfg.blend_tau)
        self.hints = Hints(mode="locked")
        self.log = []              # (t, q_cmd, q_meas, out)

    def meas(self):
        if self.blocked is not None:
            return self.blocked.copy(), np.zeros(6)
        return self.hist[-1 - self.delay]

    def tcp(self, q=None):
        return self.m.fk(self.m.config((0.0, 0.0, 0.0), {"arm": self.q if q is None else q}), "arm")

    def step(self):
        qm, qdm = self.meas()
        tg = self.buf.sample(self.t)
        e = EEState(self.q.copy(), self.v.copy(), tg, self.hints.ee("arm", self.cfg), Q0,
                    q_meas=qm.copy() if self.lead else None, qd_meas=qdm.copy() if self.lead else None)
        out = self.ctl.step(Snapshot(self.t, (0.0, 0.0, 0.0), np.zeros(3), {"arm": e}, self.hints))
        assert out.status == "ok", out.reason
        self.q = self.q + out.dq["arm"]
        self.v = out.qd["arm"].copy()
        self.hist = self.hist[1:] + [(self.q.copy(), self.v.copy())]
        self.t += self.cfg.dt
        self.log.append((self.t, self.q.copy(), qm.copy(), out))
        return out

    def run(self, s, fn=None):
        for _ in range(int(round(s / self.cfg.dt))):
            if fn:
                fn(self)
            self.step()


def test_defaults_and_leads(model):
    c = Config()
    assert c.lead_torque == (6.0, 6.0, 6.0, 1.5, 1.5, 1.5) and c.lead_delay == 0.08
    ctl = QPController(model, c, WHEELS)
    assert np.allclose(ctl.kp["arm"], KP)                  # from the arm's default profile file
    assert np.allclose(ctl.lead("arm"), [6 / 72, 6 / 72, 6 / 96, 1.5 / 18, 1.5 / 18, 1.5 / 18])
    assert "lead" in LIMITED and LIMITED.index("lead") == 7    # appended (the stream's enum is append-only)
    for bad in ([6, 6, 6, 1.5, 1.5], [6, 6, 6, 1.5, 1.5, 0], [6, 6, 6, 1.5, 1.5, math.nan], "6"):
        with pytest.raises(ValueError):
            make_config({"lead_torque": bad})
    with pytest.raises(ValueError):
        make_config({"lead_delay": -0.01})
    assert make_config({"lead_delay": 0.0}).lead_delay == 0.0


def contact(model, **cfg):
    """The measured arm blocked at Q0 (in contact) while the target moves 10 cm into the obstacle over 1 s, then
    stays there."""
    s = ArmSim(model, make_config(cfg) if cfg else None)
    s.blocked = Q0.copy()
    T0 = s.tcp()
    d = np.array([0.10, 0.0, -0.03]) / np.linalg.norm([0.10, 0.0, -0.03]) * 0.10

    def move(s_):
        a = min(s_.t / 1.0, 1.0)
        s_.buf.set_reactive(s_.t, T0.translation + a * d, T0.rotation, v=np.r_[d if a < 1 else np.zeros(3), 0, 0, 0])
    s.run(2.0, move)
    return s, T0, d


def test_contact_torque_stays_within_the_budget(model):
    s, T0, d = contact(model)
    lt = np.array(Config().lead_torque)
    tau = np.array([KP * (q - qm) for _, q, qm, _ in s.log])
    assert np.all(np.abs(tau) <= lt + 0.1), np.abs(tau).max(axis=0)   # + v^2 / 2a past a still edge
    assert np.abs(tau).max(axis=0).max() > 0.5 * lt.min()   # it did push (the bound, not a stall)
    # bound: reported, and no resync sawtooth: the command never moves more than the lead from the measured pose
    assert any(o.limited_by == "lead" for *_, o in s.log)
    assert s.log[-1][3].limited_by == "lead"
    dev = np.array([q - Q0 for _, q, _, _ in s.log])
    assert np.all(np.abs(dev).max(axis=0) <= s.ctl.lead("arm") + 0.1 / KP)
    assert np.abs(dev).max() < 0.7 * Config().resync          # the node's resync never fires
    # the command slid along with the target within the bound: the commanded hand moved toward it
    p = s.tcp().translation
    assert (p - T0.translation) @ d / 0.10 > 0.005
    # settled at the bound (no sawtooth): the torque converges monotonically; the last half second within 0.05 Nm
    last = np.abs(tau[-50:])
    assert np.ptp(last, axis=0).max() < 0.05 and np.all(np.abs(np.diff(np.abs(tau[100:]), axis=0)) < 0.05)


def test_contact_budget_follows_lead_torque(model):
    lt = (3.0, 3.0, 3.0, 0.8, 0.8, 0.8)
    s, _, _ = contact(model, lead_torque=lt)
    tau = np.array([KP * (q - qm) for _, q, qm, _ in s.log])
    assert np.all(np.abs(tau) <= np.array(lt) + 0.1), np.abs(tau).max(axis=0)


def free_motion(model, lead, delay=5, **cfg):
    """Fast free motion: the target runs away at 1 m/s (with its twist) and spins about the tool axis at 8 rad/s,
    then comes back, so the joints run at their speed caps; the measured arm trails by ``delay`` ticks."""
    s = ArmSim(model, make_config(cfg) if cfg else None, delay=delay, lead=lead)
    T0 = s.tcp()
    v = np.array([0.0, 1.0, 0.3])
    w = 8.0

    def move(s_):
        t = s_.t
        k = t if t < 0.3 else max(0.6 - t, 0.0)
        sgn = 1.0 if t < 0.3 else (-1.0 if t < 0.6 else 0.0)
        R = T0.rotation @ pin.exp3(np.array([w * k, 0.0, 0.0]))
        s_.buf.set_reactive(t, T0.translation + v * k, R,
                            v=np.r_[sgn * v, sgn * w * (R @ np.array([1.0, 0.0, 0.0]))])
    s.run(1.2, move)
    return s


@pytest.mark.parametrize("delay", [0, 5])
def test_free_motion_at_the_speed_caps_is_not_throttled(model, delay):
    on, off = free_motion(model, True, delay), free_motion(model, False, delay)
    vm = on.ctl.caps["arm"][0]
    qd = np.array([o.qd["arm"] for *_, o in off.log])
    assert np.any(np.abs(qd) >= vm - 1e-6, axis=0).sum() >= 2, np.abs(qd).max(axis=0)   # caps reached
    q_on = np.array([q for _, q, _, _ in on.log])
    q_off = np.array([q for _, q, _, _ in off.log])
    assert np.abs(q_on - q_off).max() < 1e-6, np.abs(q_on - q_off).max(axis=0)
    assert not any(o.limited_by == "lead" for *_, o in on.log)
    lag = np.array([np.abs(q - qm) for _, q, qm, _ in on.log]).max(axis=0)
    if delay:
        assert np.any(lag > on.ctl.lead("arm"))             # the raw lag is beyond the lead: the extrapolation
                                                            # is what keeps it free


def test_release_from_contact_has_no_jump(model):
    """In contact at the bound, the obstacle goes away (the measured arm follows the command again): the command
    keeps moving smoothly (within the joint acceleration limit), no jump."""
    s, T0, d = contact(model)
    n0 = len(s.log)
    s.hist = [(s.blocked.copy(), np.zeros(6))] * (s.delay + 1)
    s.blocked = None
    s.run(2.0)
    a = s.ctl.caps["arm"][1]
    steps = np.array([o.dq["arm"] for *_, o in s.log])
    qd = steps / s.cfg.dt
    assert np.all(np.abs(np.diff(qd, axis=0)) <= a * s.cfg.dt + 1e-5)
    assert np.abs(steps[n0:]).max() <= s.ctl.caps["arm"][0].max() * s.cfg.dt + 1e-9
    # free again: it reaches the target
    assert np.linalg.norm(s.tcp().translation - (T0.translation + d)) < 0.005


def test_sudden_blockage_returns_within_the_joint_box(model):
    """Moving freely, then the arm is blocked where it is measured (the command leads by the lag): outside the
    bound, the command returns toward it no faster than the joint acceleration limit allows, never a jump."""
    s = ArmSim(model)
    T0 = s.tcp()
    v = np.array([0.0, 0.5, 0.0])
    s.run(0.4, lambda s_: s_.buf.set_reactive(s_.t, T0.translation + v * s_.t, T0.rotation, v=np.r_[v, 0, 0, 0]))
    s.blocked = s.meas()[0].copy()
    lead = s.ctl.lead("arm")
    n0 = len(s.log)
    s.run(1.0, lambda s_: s_.buf.set_reactive(s_.t, T0.translation + v * s_.t, T0.rotation, v=np.r_[v, 0, 0, 0]))
    a = s.ctl.caps["arm"][1]
    qd = np.array([o.qd["arm"] for *_, o in s.log])
    assert np.all(np.abs(np.diff(qd, axis=0)) <= a * s.cfg.dt + 1e-5)
    dev = np.array([np.abs(q - s.blocked) for _, q, _, _ in s.log[-50:]])
    assert np.all(dev <= lead + 0.1 / KP)                         # back within the bound
    assert all(o.limited_by in LIMITED for *_, o in s.log[n0:])


def test_limited_by_values_are_in_the_enum(model):
    for fn in (lambda: contact(model)[0], lambda: free_motion(model, True)):
        s = fn()
        assert {o.limited_by for *_, o in s.log} <= set(LIMITED)


# -- the node: kp from the arm, the measured arm into the QP, runtime config, /state, the stream columns ----------

GAINS = [60.0, 60.0, 80.0, 15.0, 15.0, 15.0]


@pytest.fixture
def rig(model):
    import tempfile
    import time
    from helpers import Clock, serve, stop
    from test_wbc_node import Rig
    from wbc_fakes import FakeArm
    from robot_utils.nodes.wbc.node import WbcNode
    d = tempfile.mkdtemp(prefix="wbcl")
    arm = FakeArm(d + "/arm.joint.sock")
    arm.app.get("/gains")(lambda: {"name": "soft", "file": "x.yaml", "kp": GAINS, "kd": [1.0] * 6,
                                   "gripper": {"kp": 1.0, "kd": 0.05}, "tau_scale": [1.0] * 6})
    url, srv, th = serve(arm.app)
    clock = Clock(time.monotonic())
    node = WbcNode(model, arm=url, swerve=None, estimator=None, clock=clock, udp_host="127.0.0.1", udp_port=0,
                   unix_socket=d + "/wbc.target.sock", arm_socket=d + "/arm.joint.sock")
    node.receiver.start()
    r = Rig(node, clock, arm)
    r.step(2)
    yield r
    if node.enabled:
        node.disable("test end")
    node.receiver.close()
    arm.close()
    stop((srv, th))


def test_node_contact_kp_from_the_arm_state_and_columns(rig):
    """The arm's GET /gains kp is used; the fake arm blocked (it stops following the commands) while the target moves
    10 cm on: the joint torque kp (q_cmd - q) stays within lead_torque, no resync, limited_by lead, /state shows the
    bound and the binds; the stream's actual error, lead and per-joint lead torque columns."""
    node, arm = rig.node, rig.arm
    rig.lease_enable()
    kp = np.array(GAINS)
    assert np.allclose(node.ctl.kp["arm"], kp) and "GET /gains" in node.ctl.kp_source
    p0, q0 = rig.cmd_pose()
    rig.target(p0, q0)
    rig.step(3)
    assert node.armed
    arm.kinematic = False                                   # blocked where it is
    q_block = arm.q.copy()
    lt = np.array(node.cfg.lead_torque)
    d = np.array([0.10, 0.0, -0.03])
    d *= 0.10 / np.linalg.norm(d)
    worst = np.zeros(6)
    for i in range(150):
        rig.target(p0 + d * min(i / 100, 1.0), q0)
        rig.step(1)
        worst = np.maximum(worst, np.abs(kp * (node.q_cmd["arm"] - q_block)))
    assert np.all(worst <= lt + 0.1), worst
    assert worst.max() > 0.5 * lt.min()
    st = node.state_dict()
    assert st["arm"]["resyncs"] == 0 and st["limited_by"] == "lead" and st["limited_counts"].get("lead", 0) > 10
    ld = st["arm"]["lead"]
    assert ld["binding"] and ld["binds"] > 10 and ld["kp"] == GAINS and ld["lead_torque"] == list(lt)
    assert np.allclose(ld["bound"], lt / kp) and len(ld["dev"]) == 6
    r = rig.row()
    tau = np.array([r[f"arm_lead_tau{i}"] for i in range(1, 7)])
    assert np.allclose(tau, kp * (node.q_cmd["arm"] - q_block), atol=1e-9)
    assert r["limited_by"] == LIMITED.index("lead")
    # three numbers: QP shortfall (target - commanded) is large (the bound holds the command back), the actual
    # error (target - measured) larger, the lead (commanded - measured) the deflection
    assert r["arm_err_pos"] > 0.03 and r["arm_err_meas_pos"] > r["arm_err_pos"] and r["arm_lead_pos"] > 0.001
    assert all(math.isfinite(r[k]) for k in ("arm_err_meas_rot", "arm_lead_rot"))
    doc = node.field_doc
    assert all(k in doc for k in ("<ee>_err_meas_pos", "<ee>_lead_pos", "<ee>_lead_tau*", "<ee>_err_pos"))
    # released: the arm follows again, no jump in the command
    arm.kinematic = True
    qd_prev = node.v_cmd["arm"].copy()
    for _ in range(100):
        rig.target(p0 + d, q0)
        rig.step(1)
        assert np.all(np.abs(node.v_cmd["arm"] - qd_prev) <= node.ctl.caps["arm"][1] * 0.01 * 3 + 1e-6)
        qd_prev = node.v_cmd["arm"].copy()
    # disabled: the measured columns are NaN
    node.disable("test")
    rig.step(1)
    r = rig.row()
    assert all(math.isnan(r[k]) for k in ("arm_err_meas_pos", "arm_lead_pos", "arm_lead_tau1"))


def test_node_lead_config_at_runtime(rig):
    from fastapi.testclient import TestClient
    from robot_utils.nodes.wbc.server import create_app
    node = rig.node
    with TestClient(create_app(node, start=False)) as c:
        for bad in ({"lead_torque": [6, 6, 6, 1.5, 1.5]}, {"lead_torque": [6, 6, 6, 1.5, 1.5, -1]},
                    {"lead_torque": [6, 6, 6, 1.5, 1.5, 7.0]},          # beyond kp x max_lead (18 x 0.349)
                    {"lead_delay": -0.1}):
            r = c.post("/config", json=bad)
            assert r.status_code == 422, (bad, r.text)
        rig.lease_enable()                                    # enabled: kp is the arm's (GAINS)
        r = c.post("/config", json={"lead_torque": [3, 3, 3, 1, 1, 1], "lead_delay": 0.05})
        assert r.status_code == 200, r.text
        assert node.ctl.cfg.lead_torque == (3.0, 3.0, 3.0, 1.0, 1.0, 1.0) and node.ctl.cfg.lead_delay == 0.05
        assert np.allclose(node.ctl.lead("arm"), np.array([3, 3, 3, 1, 1, 1]) / np.array(GAINS))
        assert c.post("/config", json={"lead_torque": [60 * 0.35, 3, 3, 1, 1, 1]}).status_code == 422
        st = c.get("/state").json()["arm"]["lead"]
        assert st["lead_torque"] == [3.0, 3.0, 3.0, 1.0, 1.0, 1.0] and st["lead_delay"] == 0.05


def test_state_dict_holds_the_control_lock_only_briefly(rig):
    """/state builds its document outside the control lock: a slow build (the event list made slow here) never
    blocks a tick taking the lock."""
    import threading
    import time
    node = rig.node
    rig.lease_enable()
    slow = node.events.list

    def slow_list():
        time.sleep(0.3)
        return slow()
    node.events.list = slow_list
    th = threading.Thread(target=node.state_dict)
    th.start()
    time.sleep(0.05)
    t0 = time.perf_counter()
    with node.lock:
        held = time.perf_counter() - t0
    rig.step(1)
    th.join()
    assert held < 0.02, held
    node.events.list = slow


def test_events_sse_is_throttled_to_2_hz():
    import asyncio
    from robot_utils.nodes.wbc.server import throttled_sse

    class Hub:                                    # a state change every 10 ms
        def state_future(self):
            f = asyncio.get_running_loop().create_future()
            asyncio.get_running_loop().call_later(0.01, f.set_result, None)
            return f
    n = {"calls": 0}

    def state():
        n["calls"] += 1
        return {"n": n["calls"]}

    async def run():
        out = []

        async def take():
            async for chunk in throttled_sse(Hub(), state):
                out.append(chunk)
        try:
            await asyncio.wait_for(take(), 1.6)
        except asyncio.TimeoutError:
            pass
        return out
    out = asyncio.run(run())
    assert 3 <= len(out) <= 5, len(out)            # t = 0, 0.5, 1.0, 1.5 s
    assert n["calls"] == len(out)
