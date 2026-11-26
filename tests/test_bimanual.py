"""Robot profiles and the mirrored bimanual prototype (robots/, docs/plans/bimanual_and_robot_swap.md): the URDF
mirror, the profile's model (joint names, the mirrored mount, limits), the sim world with two arms (each backend
moves its own arm and nothing else), two arm nodes on it, the stack's registry for a robot, the WBC's model and QP
with two end effectors, and the default robot unchanged."""
import functools
import math
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from helpers import waiter

from robot_utils import robots
from robot_utils.robots.mirror import mirror_urdf_text

pin = pytest.importorskip("pinocchio")
wait_for = waiter(15.0, 0.02)
M = np.diag([1.0, -1.0, 1.0])
REBOT_URDF = "rebot_ctl:urdf/DM/urdf/ReBot_Arm_DM_mount_right.urdf"


def mirrored(T: np.ndarray) -> np.ndarray:
    """The mirror image of a pose across the x-z plane (conjugation keeps it a proper rotation)."""
    out = np.eye(4)
    out[:3, :3] = M @ T[:3, :3] @ M
    out[:3, 3] = M @ T[:3, 3]
    return out


def model(robot: str, variant: str = "full") -> dict:
    import tempfile

    from robot_utils.nodes.description.model import Options
    from robot_utils.nodes.description.node import DescriptionNode
    return DescriptionNode(tempfile.mkdtemp(prefix="bm-desc-"), None, robot=robot).model(Options(variant=variant))


# -- profiles -----------------------------------------------------------------------------------------------------

def test_profiles_and_their_arms(monkeypatch):
    assert {"rebot", "rebot_bimanual"} <= set(robots.available())
    monkeypatch.delenv(robots.ENV, raising=False)
    p = robots.load()
    assert p.name == "rebot" and not p.sim_only
    (arm,) = p.arms
    assert (arm.name, arm.prefix, arm.frame, arm.mirror, arm.file) == ("arm", "arm_", "arm", False, REBOT_URDF)
    assert arm.urdf_path() is None                          # rebot_ctl's own model: the arm node as before
    assert p.arm("anything") == arm                         # a one-arm robot's arm serves any instance name
    monkeypatch.setenv(robots.ENV, "rebot_bimanual")
    b = robots.load()
    assert b.name == "rebot_bimanual" and b.sim_only
    left, right = b.arms
    assert (left.name, left.prefix, left.frame, left.mirror) == ("arm_left", "arm_left_", "arm_left", True)
    assert (right.name, right.prefix, right.frame, right.mirror) == ("arm_right", "arm_right_", "arm_right", False)
    assert left.model_joints == [f"arm_left_joint{i}" for i in range(1, 7)] and right.model_ee_link == "arm_right_end_link"
    with pytest.raises(robots.ProfileError):
        b.arm("arm")                                        # two arms: the instance name picks one
    with pytest.raises(robots.ProfileError):
        robots.load("no_such_robot")
    # the bimanual description: the default one with the arm replaced by two (null deletes)
    assert "arm" not in b.description["frames"] and "arm" not in b.description["components"]
    assert b.description["components"]["wrist_camera"]["parent"] == "arm_right_end_link"
    assert b.description["components"]["base"] == p.description["components"]["base"]


# -- the URDF mirror ----------------------------------------------------------------------------------------------

@functools.lru_cache(maxsize=None)
def arm_models():
    from robot_utils.nodes.description.model import resolve_file
    text = resolve_file(REBOT_URDF).read_text("utf-8-sig")
    return pin.buildModelFromXML(text), pin.buildModelFromXML(mirror_urdf_text(text))


def test_mirrored_urdf_is_the_mirror_image_at_the_same_q():
    m, mm = arm_models()
    assert list(m.names) == list(mm.names)
    assert np.allclose(m.lowerPositionLimit, mm.lowerPositionLimit) and np.allclose(m.upperPositionLimit, mm.upperPositionLimit)
    d, dm = m.createData(), mm.createData()
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = rng.uniform(np.maximum(m.lowerPositionLimit, -3), np.minimum(m.upperPositionLimit, 3))
        pin.forwardKinematics(m, d, q)
        pin.forwardKinematics(mm, dm, q)
        pin.updateFramePlacements(m, d)
        pin.updateFramePlacements(mm, dm)
        for name in ("end_link", "link3", "finger_left_link", "finger_right_link"):
            T = d.oMf[m.getFrameId(name)].homogeneous
            Tm = dm.oMf[mm.getFrameId(name)].homogeneous
            assert np.allclose(Tm, mirrored(T), atol=1e-9), name
        # gravity torques are mirror-invariant: the arm's gravity compensation carries over unchanged
        assert np.allclose(pin.computeGeneralizedGravity(m, d, q), pin.computeGeneralizedGravity(mm, dm, q), atol=1e-9)


def test_mirror_flips_rotation_axes_and_keeps_prismatic_ones_in_the_plane():
    src = ('<robot name="r"><link name="a"><inertial><origin xyz="1 2 3" rpy="0.1 0.2 0.3"/><mass value="1"/>'
           '<inertia ixx="1" ixy="0.1" ixz="0.2" iyy="1" iyz="0.3" izz="1"/></inertial>'
           '<visual><origin xyz="0 0.5 0"/><geometry><mesh filename="x.stl" scale="2 3 4"/></geometry></visual></link>'
           '<link name="b"/><link name="c"/>'
           '<joint name="j" type="revolute"><origin xyz="0 1 0" rpy="0.5 0.6 0.7"/><parent link="a"/><child link="b"/>'
           '<axis xyz="0 0 1"/><limit lower="-1" upper="2"/></joint>'
           '<joint name="p" type="prismatic"><parent link="b"/><child link="c"/><axis xyz="0 1 0"/></joint></robot>')
    r = ET.fromstring(mirror_urdf_text(src))
    j = r.find("joint[@name='j']")
    assert j.find("origin").get("xyz") == "0 -1.0 0" and j.find("origin").get("rpy") == "-0.5 0.6 -0.7"
    assert j.find("axis").get("xyz") == "0 0 -1.0" and j.find("limit").get("lower") == "-1"
    assert r.find("joint[@name='p']/axis").get("xyz") == "0 -1.0 0"
    it = r.find("link/inertial/inertia")
    assert (it.get("ixy"), it.get("ixz"), it.get("iyz")) == ("-0.1", "0.2", "-0.3")
    assert r.find("link/visual/geometry/mesh").get("scale") == "2.0 -3.0 4.0"


# -- the bimanual model -------------------------------------------------------------------------------------------

def test_bimanual_model_joints_mount_and_limits():
    m = model("rebot_bimanual")
    joints = {j["name"]: j for j in m["joints"]}
    for side in ("left", "right"):
        assert [f"arm_{side}_joint{i}" for i in range(1, 7)] + [f"arm_{side}_finger_left"] == \
            [n for n in joints if n.startswith(f"arm_{side}_") and not n.endswith("finger_right")]
    assert not any(n.startswith("arm_joint") for n in joints)
    for i in range(1, 7):                                   # the same limits: the same q is the mirrored pose
        L, R = joints[f"arm_left_joint{i}"], joints[f"arm_right_joint{i}"]
        assert (L["lower"], L["upper"]) == (R["lower"], R["upper"])
    assert joints["arm_left_joint1"]["axis"] == [0.0, 0.0, -1.0] and joints["arm_right_joint1"]["axis"] == [0.0, 0.0, 1.0]
    fb = {k: np.array(v) for k, v in m["frames_in_base"].items()}
    assert np.allclose(fb["arm_left_base_link"], mirrored(fb["arm_right_base_link"]), atol=1e-9)
    assert np.allclose(fb["arm_right_base_link"][:3, 3], [0.1159, -0.1459, 0.22225])
    for link in ("link1", "link4", "end_link", "finger_left_link"):        # at q = 0 too: the whole arm mirrored
        assert np.allclose(fb[f"arm_left_{link}"], mirrored(fb[f"arm_right_{link}"]), atol=1e-9), link
    assert m["description_frames"]["arm_left"] == "arm_left_base_link"
    assert [a["name"] for a in m["arms"]] == ["arm_left", "arm_right"]
    left = m["arms"][0]
    assert left["mirror"] and left["joints"][0] == "arm_left_joint1" and left["gripper"] == "arm_left_finger_left"
    assert "wrist_camera_color_optical" in m["frames"]
    assert m["frames"]["wrist_camera_depth_optical"]["parent"] is not None


def test_bimanual_boxes_variant_mirrors_the_boxes():
    m = model("rebot_bimanual", "boxes")
    assert m["options"]["variant"] == "boxes" and len(m["arms"]) == 2


def test_default_model_unchanged():
    m = model("rebot")
    assert [a["name"] for a in m["arms"]] == ["arm"]
    names = [j["name"] for j in m["joints"]]
    assert [f"arm_joint{i}" for i in range(1, 7)] == [n for n in names if n.startswith("arm_joint")]
    assert not any(n.startswith("arm_left") or n.startswith("arm_right") for n in names)
    assert np.allclose(np.array(m["frames_in_base"]["arm_base_link"])[:3, 3], [0.1159, -0.1459, 0.22225])


# -- the world with two arms ----------------------------------------------------------------------------------------

@functools.lru_cache(maxsize=None)
def bimanual_bundle() -> bytes:
    from robot_utils.nodes.sim.node import fetch_bundle
    return fetch_bundle("local", "full", "rebot_bimanual")[0]


def make_bimanual_world(**over):
    from robot_utils.nodes.sim.world import World, load_config, unzip_bundle
    urdf, assets, info = unzip_bundle(bimanual_bundle())
    over.setdefault("scene", {"tabletop": False})
    over.setdefault("time", {"run": False})
    return World(urdf, assets, load_config(overrides=over), {"source": "test", "arms": info["arms"]})


@pytest.fixture(scope="module")
def bworld():
    pytest.importorskip("mujoco")
    w = make_bimanual_world()
    yield w
    w.close()


def _hold(w, arm, goal, seconds=1.0):
    for k in range(int(seconds / w.m.opt.timestep)):
        if k % 10 == 0:
            w.arm_op("send", {"q": list(goal), "qd": [0] * 6, "kp": [30] * 6, "kd": [1] * 6, "tau": [0] * 6,
                              "g_q": -1.0, "g_qd": 0, "g_kp": 1.0, "g_kd": 0.05, "g_tau": 0, "gripper": True}, arm)
        w.step(1)


def test_world_runs_both_arms_each_backend_moves_its_own(bworld):
    w = bworld
    w.reset()
    assert list(w.arm_ix) == ["arm_left", "arm_right"] and w.primary_arm == "arm_left"
    assert w.arm_op("hello", {}, "arm_right")["joints"] == [f"arm_right_joint{i}" for i in range(1, 7)]
    with pytest.raises(Exception):
        w.arm_op("read", {}, "arm")                                   # no arm of that name
    read = lambda a: np.array(w.arm_op("read", {}, a)["q"])          # noqa: E731
    qL0, qR0 = read("arm_left"), read("arm_right")
    assert np.allclose(qL0, qR0, atol=1e-6)                           # the same start pose (mirrored)
    for arm, other in (("arm_left", "arm_right"), ("arm_right", "arm_left")):
        w.reset()
        w.arm_op("enable", {}, arm)
        goal = read(arm) + np.array([0.3, -0.2, 0.1, 0.2, -0.2, 0.3])
        q_other = read(other)
        _hold(w, arm, goal)
        assert np.allclose(read(arm), goal, atol=3e-3), arm
        assert np.allclose(read(other), q_other, atol=1e-3), other    # untouched
        snap = w.snapshot()["arms"]
        assert snap[arm]["enabled"] and not snap[other]["enabled"]
        assert snap[arm]["gap_mm"] > 10 and snap[other]["gap_mm"] < 1   # only this arm's gripper opened
        w.arm_op("disable", {}, arm)


def test_world_arms_are_mirror_images(bworld):
    import mujoco
    w = bworld
    w.reset()
    q = np.array([0.4, -0.5, -0.7, 0.3, 0.2, 0.5])
    for a in ("arm_left", "arm_right"):
        w.d.qpos[w.arm_ix[a].q] = q
    mujoco.mj_forward(w.m, w.d)

    def in_base(body):
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = w.d.xmat[w.m.body(body).id].reshape(3, 3), w.d.xpos[w.m.body(body).id]
        B = np.eye(4)
        B[:3, :3], B[:3, 3] = w.d.xmat[w.base_body].reshape(3, 3), w.d.xpos[w.base_body]
        return np.linalg.inv(B) @ T
    for link in ("end_link", "link3", "finger_left_link"):
        assert np.allclose(in_base(f"arm_left_{link}"), mirrored(in_base(f"arm_right_{link}")), atol=1e-6), link
    # each arm's fingers collide as their jaw boxes
    names = {w.m.geom(i).name for i in range(w.m.ngeom)}
    for a in ("arm_left", "arm_right"):
        assert any(n.startswith(f"{a}_finger_left_link_") and n.endswith("_collision") for n in names), a


def test_world_lost_backend_faults_only_its_arm(bworld):
    w = bworld
    w.reset()
    w.arm_op("enable", {}, "arm_left")
    w.arm_op("enable", {}, "arm_right")
    w.arm_lost("arm_right")
    assert w.arms["arm_right"].fault == 0xD and not w.arms["arm_right"].enabled
    assert w.arms["arm_left"].enabled and w.arms["arm_left"].fault == 0
    w.arm_op("disable", {}, "arm_left")
    w.reset()


# -- two arm nodes on the bimanual sim ----------------------------------------------------------------------------

def test_two_arm_nodes_enable_hold_and_move_their_own_arm(tmp_path):
    from reBotArm_control_py.actuator.rebotarm import joint_limits, load_cfg

    from robot_utils.nodes.arm import config as C
    from robot_utils.nodes.arm.node import ArmNode
    from robot_utils.nodes.common.locks import LockStore
    from robot_utils.nodes.sim.backends.arm import WorldArmBus
    from robot_utils.nodes.sim.node import SimNode
    sim = SimNode(make_bimanual_world(time={"run": True}), "simbm", str(tmp_path / "scenes"))
    sim.start()
    prof = robots.load("rebot_bimanual")
    limits = joint_limits(load_cfg(C.PROFILES["soft"]))
    nodes = {}
    try:
        for spec in prof.arms:
            nodes[spec.name] = ArmNode(functools.partial(WorldArmBus, "simbm", limits=limits, arm=spec.name),
                                       locks=LockStore(tmp_path / f"locks-{spec.name}"), urdf=spec.urdf_path(),
                                       arm_frame=spec.frame, robot=prof.name)
            nodes[spec.name].start()
        toks = {}
        for name, node in nodes.items():
            wait_for(lambda: node.state == "uninitialized", 30)
            toks[name], _ = node.acquire("test")
            assert node.enable(toks[name])["enabled"], name
        assert all(a.enabled for a in sim.world.arms.values())
        stop = threading.Event()

        def renew():
            while not stop.wait(0.1):
                for n, nd in nodes.items():
                    nd.renew(toks[n])
        th = threading.Thread(target=renew, daemon=True)
        th.start()
        try:
            left, right = nodes["arm_left"], nodes["arm_right"]
            time.sleep(0.5)                                   # both hold
            qR = np.array(sim.world.arm_op("read", {}, "arm_right")["q"])
            # the left node's own kinematics are the mirrored arm's: its TCP in its arm frame is where the world has it
            ql = np.array(sim.world.arm_op("read", {}, "arm_left")["q"])
            assert np.allclose(left.kin.fk(ql).homogeneous, mirrored(right.kin.fk(ql).homogeneous), atol=1e-9)
            T = left.kin.fk(ql)
            T.translation = T.translation + np.array([0.0, 0.02, 0.02])    # up and outward (+y on the left)
            assert left.track_cmd(toks["arm_left"], T)["ok"]
            wait_for(lambda: np.linalg.norm(left.kin.fk(np.array(sim.world.arm_op("read", {}, "arm_left")["q"])).translation
                                            - T.translation) < 0.003, 10)
            # the world agrees: the left end link moved up and outward in the robot base frame
            assert np.allclose(np.array(sim.world.arm_op("read", {}, "arm_right")["q"]), qR, atol=2e-3)   # right held
            assert left.arm_in_base()["T"][1, 3] == pytest.approx(0.1459)   # its own description frame (fallback)
            assert right.arm_in_base()["T"][1, 3] == pytest.approx(-0.1459)
        finally:
            stop.set()
            th.join()
        for name, node in nodes.items():
            node.disable(toks[name])
    finally:
        for node in nodes.values():
            node.close()
        sim.close()


# -- the stack's registry for a robot -----------------------------------------------------------------------------

def test_stack_registry_for_the_bimanual_robot(tmp_path):
    import yaml

    from robot_utils.frontend.config import load_config
    from robot_utils.nodes.sim.stack import DEFAULT_NODES, registry_path, robot_registry, start_list
    prof = robots.load("rebot_bimanual")
    path, arms = robot_registry(registry_path(), prof, tmp_path, port=8095, wbc_udp_port=9495, quest="127.0.0.1:9396")
    assert arms == ["arm_left", "arm_right"]
    nodes = {n.name: n for n in load_config(path)}            # a valid registry for the landing page
    assert "arm" not in nodes and {"arm_left", "arm_right", "wbc", "quest_teleop", "agent_interface"} <= set(nodes)
    assert list(nodes["arm_left"].start.args) == ["--sim", "world", "--robot", "rebot_bimanual"]
    for k in ("description", "sim", "agent_interface"):
        assert nodes[k].start.args[-2:] == ("--robot", "rebot_bimanual"), k
    w = list(nodes["wbc"].start.args)
    assert w[w.index("--udp-port") + 1] == "9495" and "--robot" in w
    q = list(nodes["quest_teleop"].start.args)
    assert q[q.index("--ar") + 1] == ":8095" and q[q.index("--quest") + 1] == "127.0.0.1"
    assert q[q.index("--quest-port") + 1] == "9396"
    assert yaml.safe_load(open(registry_path()))["nodes"][2]["name"] == "arm"     # the packaged one untouched
    assert start_list(DEFAULT_NODES, arms, set()).split(",") == \
        ["description", "sim", "arm_left", "arm_right", "swerve", "camera", "wrist_camera", "power", "estimator",
         "pose_servo", "wbc"]


def test_arm_node_cli_refuses_hardware_for_a_sim_only_robot(monkeypatch, capsys):
    import sys

    from robot_utils.nodes.arm import __main__ as arm_main
    monkeypatch.setattr(sys, "argv", ["arm", "--name", "arm_left", "--robot", "rebot_bimanual"])
    with pytest.raises(SystemExit):
        arm_main.main()
    assert "sim only" in capsys.readouterr().err


# -- the WBC's model and QP with two end effectors (the math layer; the node is still one arm) ---------------------

def test_wbc_qp_drives_two_end_effectors_on_the_bimanual_model():
    pytest.importorskip("qpsolvers")
    from robot_utils.nodes.swerve.constants import MODULES
    from robot_utils.nodes.wbc.controller import EEState, QPController, Snapshot
    from robot_utils.nodes.wbc.model import EESpec, WbcModel, arm_config_limits, ee_specs, fetch_urdf
    from robot_utils.nodes.wbc.spec import Config, Hints
    from robot_utils.nodes.wbc.targets import TargetBuffer
    (one,) = ee_specs("rebot")
    assert one == EESpec() and one.arm_limits == arm_config_limits()     # the default robot: the WBC's EE as before
    urdf, _ = fetch_urdf("local", "rebot_bimanual")
    m = WbcModel(urdf, ee_specs("rebot_bimanual"))
    assert set(m.ees) == {"arm_left", "arm_right"}
    assert m.mount_yaw["arm_left"] == pytest.approx(-m.mount_yaw["arm_right"], abs=1e-9)
    assert m.reach["arm_left"] == pytest.approx(m.reach["arm_right"], rel=1e-6)
    q0 = np.array([0.0, -0.6, -0.9, 0.0, 0.5, 0.0])
    cfg = Config()
    ctl = QPController(m, cfg, [(mo.x, mo.y) for mo in MODULES])
    base, q = np.zeros(3), {e: q0.copy() for e in m.ees}
    v = {e: np.zeros(6) for e in m.ees}
    goals, bufs = {}, {}
    for e, d in (("arm_left", [0.0, 0.03, 0.04]), ("arm_right", [0.0, -0.03, -0.02])):
        T0 = m.fk(m.config(base, q), e)
        goals[e] = T0.translation + np.array(d)
        bufs[e] = TargetBuffer(e, cfg.ff_timeout, cfg.blend_tau)
        bufs[e].set_reactive(0.0, goals[e], T0.rotation)
    hints, t = Hints(), 0.0
    for _ in range(300):
        snap = Snapshot(t, tuple(base), np.zeros(3),
                        {e: EEState(q[e].copy(), v[e].copy(), bufs[e].sample(t), hints.ee(e, cfg), q0) for e in m.ees},
                        hints, hold=False, velocity_age=math.inf, hint_age=0.0)
        out = ctl.step(snap)
        assert out.status == "ok", out.status
        for e in m.ees:
            q[e] = q[e] + out.dq[e]
            v[e] = out.qd[e].copy()
        t += cfg.dt
    for e in m.ees:
        err = np.linalg.norm(m.fk(m.config(base, q), e).translation - goals[e])
        assert err < 0.003, (e, err)


# -- the WBC node with two arms (lockstep on a fake clock; tests/wbc_fakes.py's FakeArm per arm) -------------------

@pytest.fixture
def wbc2():
    import json
    import socket
    import tempfile

    from helpers import Clock, serve, stop
    from wbc_fakes import FakeArm

    from robot_utils.nodes.wbc.model import WbcModel, ee_specs, fetch_urdf
    from robot_utils.nodes.wbc.node import WbcNode
    urdf, _ = fetch_urdf("local", "rebot_bimanual")
    model = WbcModel(urdf, ee_specs("rebot_bimanual"))
    d = tempfile.mkdtemp(prefix="wbc2")
    arms, urls, srvs = {}, {}, []
    for e in ("arm_left", "arm_right"):
        arms[e] = FakeArm(f"{d}/{e}.joint.sock")
        url, srv, th = serve(arms[e].app)
        urls[e] = url
        srvs.append((srv, th))
    clock = Clock(time.monotonic())
    node = WbcNode(model, arms=urls, arm_sockets={e: f"{d}/{e}.joint.sock" for e in arms}, swerve=None,
                   estimator=None, clock=clock, udp_host="127.0.0.1", udp_port=0, unix_socket=d + "/wbc.target.sock")
    node.receiver.start()
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    class R:
        seq = 0

        def step(self, n=1):
            for _ in range(n):
                clock.t += 0.01
                node.step()
                time.sleep(0.0015)

        def row(self):
            return dict(zip(node.fields, node.hub.recent(1, "tick")[0]))

        def send(self, ees):
            self.seq += 1
            m = {"session": self.session, "seq": self.seq, "armed": True, "arm_seq": self.aseq, "type": "target",
                 "t": None, "ees": ees}
            udp.sendto(json.dumps(m).encode(), ("127.0.0.1", node.receiver.udp_port))
            time.sleep(0.003)

    r = R()
    r.node, r.arms = node, arms
    yield r
    if node.enabled:
        node.disable("test end")
    node.receiver.close()
    for a in arms.values():
        a.close()
    for s in srvs:
        stop(s)


def test_wbc_node_drives_two_arms(wbc2):
    from robot_utils.nodes.wbc.targets import R_to_quat
    r, node = wbc2, wbc2.node
    assert set(node.arm_targets) == {"arm_left", "arm_right"} and node.primary == "arm_left"
    assert {"arm_left_cmd_x", "arm_right_tgt_x", "arm_right_q_cmd6", "arm_left_lead_tau1"} <= set(node.fields)
    tok, _ = node.acquire("test")
    node.enable(tok)
    for e, a in r.arms.items():                       # each arm leased, enabled and its joint channel open
        assert a.token is not None and a.enabled and a.session is not None, e
    assert set(node.channels) == {"arm_left", "arm_right"} and set(node.arm_lease_ids) == set(node.channels)
    out = node.open_session(tok)
    r.session, r.aseq = out["session"], out["arm_seq"] + 1
    r.step(3)
    row = r.row()
    m = node.model
    joints = lambda: {k: r.arms[k].q for k in r.arms}            # noqa: E731
    T = {e: m.fk(m.config((0, 0, 0), joints()), e) for e in r.arms}
    for e in r.arms:                                  # seeded per arm from its own measured joints
        assert np.allclose([row[f"{e}_cmd_{k}"] for k in "xyz"], T[e].translation, atol=1e-6), e
    assert row["arm_left_cmd_y"] > 0 > row["arm_right_cmd_y"]
    goal = {"arm_left": T["arm_left"].translation + [0.0, 0.03, 0.03],
            "arm_right": T["arm_right"].translation + [0.02, -0.02, 0.0]}
    q = {e: list(map(float, R_to_quat(T[e].rotation))) for e in r.arms}
    r.send({e: {"frame": "world", "pos": list(map(float, T[e].translation)), "quat": q[e]} for e in r.arms})
    r.step(2)                                         # arms at the commanded poses (a far first target never arms)
    assert node.armed, (node.disarm_reason, node.latch_counts)
    for _ in range(150):
        r.send({e: {"frame": "world", "pos": list(map(float, goal[e])), "quat": q[e],
                    "gripper": 0.25 if e == "arm_left" else 0.75} for e in r.arms})
        r.step(1)
    row = r.row()
    assert node.armed and node.state == "tracking", (node.state, node.disarm_reason)
    for e in r.arms:
        tcp = m.fk(m.config((0, 0, 0), joints()), e).translation
        assert np.linalg.norm(tcp - goal[e]) < 0.004, (e, tcp, goal[e])                 # each arm reached its own
        assert np.linalg.norm(np.array([row[f"{e}_tgt_{k}"] for k in "xyz"]) - goal[e]) < 1e-9
    assert abs(r.arms["arm_left"].gap - 0.25 * node.gap_maxes.get("arm_left", node.gap_max)) < 1e-6
    assert abs(r.arms["arm_right"].gap - 0.75 * node.gap_maxes.get("arm_right", node.gap_max)) < 1e-6
    st = node.state_dict()
    assert set(st["arms"]) == {"arm_left", "arm_right"} and st["arm"]["target"] == node.arm_targets["arm_left"]
    assert st["arms"]["arm_right"]["state"] in ("floating", "servoing")
    # one arm's fault stops the session (the latch and the base are shared)
    r.arms["arm_right"].set_state("fault")
    r.step(80)
    assert node.fault is not None and "arm_right" in node.fault, node.fault
    node.disable("test")


def test_collect_default_streams_follow_the_robot():
    from robot_utils.collect.streams import DEFAULT_STREAMS, default_streams
    assert default_streams("rebot") == DEFAULT_STREAMS                         # the single-arm robot unchanged
    s = default_streams("rebot_bimanual")
    assert "arm/stream" not in s and s[2:4] == ("arm_left/stream", "arm_right/stream")


def test_agent_interface_per_arm_routes_and_leases(tmp_path):
    """/arm/* is the first arm; /arms/{name}/* each arm, with its own lease held by the node."""
    import httpx
    from helpers import serve, stop

    from robot_utils.nodes.agent_interface.node import AgentNav
    from robot_utils.nodes.agent_interface.server import create_app
    from robot_utils.nodes.agent_interface.sim import fake_arm_app
    apps = {e: fake_arm_app() for e in ("arm_left", "arm_right")}
    urls, srvs = {}, []
    for e, app in apps.items():
        url, srv, th = serve(app)
        urls[e] = url
        srvs.append((srv, th))
    nav = AgentNav("http://127.0.0.1:9", "http://127.0.0.1:9", None, str(tmp_path / "nav"), require_imu=False,
                   description=None, swerve=None, arms=urls)
    url, srv, th = serve(create_app(nav))
    L, Rt = apps["arm_left"].state.arm, apps["arm_right"].state.arm
    h = {"X-Operator": "human"}
    try:
        assert httpx.get(url + "/arms", timeout=5).json() == {"arms": ["arm_left", "arm_right"], "primary": "arm_left"}
        r = httpx.post(url + "/arms/arm_right/enable", json={}, headers=h, timeout=10)
        assert r.status_code == 200, r.text
        assert Rt["token"] is not None and Rt["enabled"] and L["token"] is None          # only that arm's lease
        r = httpx.post(url + "/arms/arm_right/gripper", json={"gap_mm": 20}, headers=h, timeout=10)
        assert r.status_code == 200, r.text
        assert Rt["calls"][-1][0] == "gripper" and not any(c[0] == "gripper" for c in L["calls"])
        assert httpx.post(url + "/arm/gripper", json={"gap_mm": 20}, headers=h, timeout=10).status_code == 409  # left: not enabled
        assert httpx.post(url + "/arm/enable", json={}, headers=h, timeout=10).status_code == 200    # /arm = arm_left
        assert L["token"] is not None and L["token"] != Rt["token"]
        st = httpx.get(url + "/state", timeout=5).json()
        assert st["arm"]["held"] and set(st["arms"]) == {"arm_left", "arm_right"} and st["arms"]["arm_right"]["held"]
        assert httpx.post(url + "/arms/nope/enable", json={}, headers=h, timeout=10).status_code == 404
        for name in ("arm_left", "arm_right"):
            assert httpx.post(url + f"/arms/{name}/disable", json={}, headers=h, timeout=10).status_code == 200
        assert L["token"] is None and Rt["token"] is None
    finally:
        stop((srv, th))
        for s in srvs:
            stop(s)


def test_wbc_proc_node_enables_and_releases_two_arms():
    """The control-process WBC: both arms' joint channels owned by the control process, /state per arm."""
    import tempfile

    import httpx
    from helpers import serve, stop
    from wbc_fakes import FakeArm

    from robot_utils.nodes.wbc.model import WbcModel, ee_specs, fetch_urdf
    from robot_utils.nodes.wbc.proc_node import ProcNode
    from robot_utils.nodes.wbc.server import create_app
    urdf, _ = fetch_urdf("local", "rebot_bimanual")
    d = tempfile.mkdtemp(prefix="wbcp2")
    arms, urls, srvs = {}, {}, []
    for e in ("arm_left", "arm_right"):
        arms[e] = FakeArm(f"{d}/{e}.joint.sock")
        url, srv, th = serve(arms[e].app)
        urls[e] = url
        srvs.append((srv, th))
    node = ProcNode(WbcModel(urdf, ee_specs("rebot_bimanual")), arms=urls,
                    arm_sockets={e: f"{d}/{e}.joint.sock" for e in arms}, swerve=None, estimator=None,
                    udp_host="127.0.0.1", udp_port=0, unix_socket=d + "/t.sock")
    url, srv, th = serve(create_app(node))
    http = httpx.Client(base_url=url, timeout=10)
    try:
        tok = http.post("/lease", json={"client": "test"}).json()["token"]
        h = {"X-Lease-Token": tok}
        r = http.post("/enable", headers=h)
        assert r.status_code == 200, r.text
        for e, a in arms.items():
            assert a.enabled and a.session is not None, e
        wait_for(lambda: len(arms["arm_left"].commands) > 20 and len(arms["arm_right"].commands) > 20, 5)
        st = http.get("/state").json()
        assert st["enabled"] and set(st["arms"]) == {"arm_left", "arm_right"}
        assert all(st["arms"][e]["state"] in ("floating", "servoing") for e in arms)
        assert http.post("/disable", headers=h).status_code == 200
        wait_for(lambda: not any(a.enabled for a in arms.values()), 5)
    finally:
        http.close()
        stop((srv, th))
        node.close()
        for a in arms.values():
            a.close()
        for s in srvs:
            stop(s)


def test_lenient_timing_ordering_sim_ge_wbc_ge_arm_node():
    """The stall limits in a sim on macOS: the sim's arm watchdog and the WBC's arm_stale outlast what the arm node
    forgives (its lenient ctl_heartbeat), so a stall it rides out faults nothing downstream; strict / Linux unchanged."""
    from robot_utils.nodes.arm.config import LENIENT_TIMING as ARM
    from robot_utils.nodes.sim.world import default_config, resolve_timing
    from robot_utils.nodes.wbc.__main__ import apply_timing
    from robot_utils.nodes.wbc.spec import make_config
    sim = resolve_timing(default_config(), "darwin", ["arm_left", "arm_right"])
    wbc = make_config(apply_timing(None, "auto", "darwin"))
    assert min(sim["arm_watchdogs"].values()) >= ARM["ctl_heartbeat"] + 0.5
    assert wbc.arm_stale >= ARM["ctl_heartbeat"] + 0.5
    assert apply_timing(None, "auto", "linux") is None and apply_timing({"arm_stale": 0.3}, "strict", "darwin") == \
        {"arm_stale": 0.3}
    assert apply_timing({"arm_stale": 5.0}, "lenient", "linux")["arm_stale"] == 5.0      # never lowered


# -- the rigid scene and the goal ghosts, per arm --------------------------------------------------------------------

def test_scene3d_has_both_arms_and_a_goal_ghost_per_arm(bworld, tmp_path):
    """The headset's rigid-scene view: both arms' links are bodies; the sim aids post one goal ghost per EE (each from
    its own <ee>_tgt_* / <ee>_meas_*, on its own end link, coloured by its own error); the index carries both."""
    from types import SimpleNamespace

    from robot_utils.nodes.quest_teleop.sim_aids import AidsConfig, SimAids
    from robot_utils.nodes.sim import rigid
    from robot_utils.nodes.sim.node import SimNode
    w = bworld
    w.reset()
    sim = SimNode(w, "s3bm", str(tmp_path / "scenes"))
    idx, _ = rigid.build(w)
    names = {b["name"] for b in idx["bodies"]}
    for arm in ("arm_left", "arm_right"):
        assert {f"{arm}_link1", f"{arm}_link6", f"{arm}_end_link", f"{arm}_finger_left_link"} <= names, arm
    keys = ("x", "y", "z", "qx", "qy", "qz", "qw")
    meas = {"arm_left": [0.4, 0.2, 0.5, 0, 0, 0, 1], "arm_right": [0.4, -0.2, 0.5, 0, 0, 0, 1]}
    tgt = {"arm_left": [0.41, 0.2, 0.5, 0, 0, 0, 1], "arm_right": [0.45, -0.2, 0.5, 0, 0, 0, 1]}   # 1 cm, 5 cm off
    row = {f"{e}_{kind}_{k}": v for e in meas for kind, src in (("tgt", tgt), ("meas", meas))
           for k, v in zip(keys, src[e])}
    node = SimpleNamespace(cfg=SimpleNamespace(ee="arm_right", hands={"left": "arm_left", "right": "arm_right"}),
                           stream=SimpleNamespace(latest=lambda: (row, 0.0)), wbc_target="wbc",
                           events=SimpleNamespace(add=lambda *a: None), hub=SimpleNamespace(state_changed=lambda: None))
    aids = SimAids(node, "s3bm", AidsConfig(ghost=True, auto_pause=False))
    aids._call = lambda method, path, body=None: sim.put_ghosts(body["ghosts"], body.get("source")) \
        if (method, path) == ("post", "/ghosts") else {}
    aids.links = {"arm_left": "arm_left_end_link", "arm_right": "arm_right_end_link"}
    aids.tcps = {"arm_left": [-0.15, 0.0, 0.0], "arm_right": [-0.15, 0.0, 0.0]}
    aids._ghost()
    gidx, _ = rigid.build(w, ghosts=sim.ghosts.declared())
    ghosts = {g["name"]: g for g in gidx["ghosts"]}
    assert set(ghosts) == {"ee_goal_arm_left", "ee_goal_arm_right"}
    assert ghosts["ee_goal_arm_left"]["body"] == "arm_left_end_link"
    assert ghosts["ee_goal_arm_right"]["body"] == "arm_right_end_link"
    poses = rigid.Poses(gidx)
    aids._ghost()                                     # posted again (the build took longer than the ghosts' ttl)
    f = poses.frame(w, sim.ghosts.live())
    col = {n: f.colors[gidx["dynamic"].index(g["bodies"][0])] for n, g in ghosts.items()}
    assert col["ee_goal_arm_left"][:3] != col["ee_goal_arm_right"][:3]       # each coloured by its own error
    assert col["ee_goal_arm_right"][:3] == [1.0, 0.25, 0.2]                  # 5 cm: fully red


# -- the dynamic arm (MIT law, gravity fit), per arm ---------------------------------------------------------------

@pytest.fixture(scope="module")
def dworld():
    pytest.importorskip("mujoco")
    w = make_bimanual_world(components={"arm": {"fidelity": "dynamic"}})
    yield w
    w.close()


def test_dynamic_both_arms_gravity_fit_and_float_hold(dworld):
    """Each arm has its own motors and its own gravity fit (the mirrored arm's to its mirrored model, as its node
    loads it): the node's float (kp = kd = 0, tau_ff = its own model's gravity x tau_scale) holds each arm, alone."""
    from robot_utils.nodes.arm import config as C
    from robot_utils.nodes.arm.kin import ArmKinematics
    w = dworld
    w.reset()
    P = C.load_profile("soft")
    assert set(w.gravity_fits) == {"arm_left", "arm_right"}
    coulomb = np.array(w.cfg["components"]["arm"]["dynamic"]["coulomb"])
    for name, fit in w.gravity_fits.items():
        assert np.all(np.array(fit["max"]) < coulomb), (name, fit["max"])
        assert set(fit["bodies"]) == {f"{name}_{n}" for n in ("link2", "link3", "link4", "link5", "link6", "end_link")}
    assert w.gravity_fits["arm_left"].get("model", "").startswith("mirrored")
    for name, ix in w.arm_ix.items():                  # each arm's motors are its own actuators
        assert np.allclose(w.m.actuator_forcerange[ix.a, 1], [27.0] * 3 + [7.0] * 3)
    left_com = np.array(w.gravity_fits["arm_left"]["bodies"]["arm_left_link3"]["com"][1])
    right_com = np.array(w.gravity_fits["arm_right"]["bodies"]["arm_right_link3"]["com"][1])
    assert np.allclose(left_com * [1, -1, 1], right_com, atol=2e-3)     # the fitted COMs mirror each other
    kins = {e: ArmKinematics({}, urdf=robots.load("rebot_bimanual").arm(e).urdf_path()) for e in w.arm_ix}
    q0 = np.array([0.3, -1.0, -1.2, 0.3, 0.8, 0.2])
    for arm in w.arm_ix:                               # one arm floats on its own node's feedforward, the other off
        w.reset()
        ix = w.arm_ix[arm]
        w.arm_op("enable", {}, arm)
        w.d.qpos[ix.q] = q0
        w.d.qvel[ix.v] = 0.0
        prev = w.arm_op("read", {}, arm)
        for _ in range(300):
            g = kins[arm].gravity(np.asarray(prev["q"], float)) * P.tau_scale
            w.arm_op("send", {"q": list(q0), "qd": [0.0] * 6, "kp": [0.0] * 6, "kd": [0.0] * 6,
                              "tau": [float(v) for v in g]}, arm)
            prev = w.arm_op("read", {}, arm)
            w.step(10)
        drift = np.degrees(np.abs(np.array(w.arm_op("read", {}, arm)["q"]) - q0))
        assert np.all(drift < 1.0), (arm, drift)
        assert w.arms[arm].enabled and w.arms[arm].fault == 0
        w.arm_op("disable", {}, arm)


def test_dynamic_each_arm_tracks_its_own_target(dworld):
    from robot_utils.nodes.arm import config as C
    w = dworld
    P = C.load_profile("soft")
    for arm, other in (("arm_left", "arm_right"), ("arm_right", "arm_left")):
        w.reset()
        w.arm_op("enable", {}, arm)
        ix, iy = w.arm_ix[arm], w.arm_ix[other]
        q0 = np.array(w.arm_op("read", {}, arm)["q"])
        q_other = w.d.qpos[iy.q].copy()
        goal = q0 + np.array([0.3, -0.2, 0.2, 0.1, 0.2, -0.3])
        for k in range(150):
            q_t = q0 + (goal - q0) * min(1.0, k / 80)
            w.arm_op("send", {"q": list(q_t), "qd": [0.0] * 6, "kp": list(P.kp), "kd": list(P.kd),
                              "tau": [float(v) for v in w.d.qfrc_bias[ix.v]]}, arm)
            w.step(10)
        assert np.allclose(w.arm_op("read", {}, arm)["q"], goal, atol=0.01), arm
        assert not w.arms[other].enabled
        assert np.allclose(w.d.actuator_force[iy.a], 0.0)      # the other arm's motors get no torque (it sinks, off)
        assert q_other is not None
        w.arm_op("disable", {}, arm)
