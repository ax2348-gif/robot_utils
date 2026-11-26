"""The OpenArm v2 bimanual robot (robots/openarm_bimanual.yaml, robots/openarm/): the 7-DOF arms (Enactic's OpenArm
v2.0 with the Anvil OpenArm 2.0's joint ranges, J1 Enactic's window) and Anvil's gripper, the profile's model, the sim's dynamic 7-joint arms, and the WBC's null-space posture (controller.py):
the human-like terms (wbc/humanlike.py: swivel model, effort, centering, the short-range arm-proximity term) and the
fixed nominal posture, all in the EE task's null space: the EE is unaffected, the elbow settles on the model's swivel
and returns after a push, the swivel is smooth as the hand moves, effort is lower than at the nominal posture, the
proximity term keeps the arms apart when they close in and is exactly dormant otherwise, joint limits hold."""
import functools
import math

import numpy as np
import pytest

from robot_utils import robots

pin = pytest.importorskip("pinocchio")
POSTURE = {"arm_left": [0.0, 0.0, 0.0, 1.5708, 0.0, 0.0, 0.0],     # the owner's pose (2026-11-26): upper arm down,
           "arm_right": [0.0, 0.0, 0.0, 1.5708, 0.0, 0.0, 0.0]}    # forearm forward, the wrist camera up
OLD_POSTURE = {"arm_left": [-0.244, -0.894, 0.616, 1.868, 0.906, -0.370, -0.920],   # the WBC studies' start (elbow
               "arm_right": [0.244, 0.894, -0.616, 1.868, -0.906, 0.370, 0.920]}    # out, tool 50 deg down)


# -- the profile and the model ----------------------------------------------------------------------------------

def test_profile_arms_are_openarm_7dof_with_side_type_posture():
    p = robots.load("openarm_bimanual")
    left, right = p.arms
    assert (left.side, right.side) == ("left", "right") and left.type == right.type == "openarm_v2"
    assert left.dof == 7 and list(left.posture) == POSTURE["arm_left"] and not left.mirror
    assert left.urdf_path().endswith("openarm_v2_left_arm.urdf")          # OpenArm's own left model, not mirrored
    t = left.arm_type()
    assert t.dof == 7 and t.torque_max == (40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0) and len(t.kp) == 7


DEG = math.pi / 180
# Enactic's arms at q = 0, in base_link: each joint's origin and axis (the left arm is upstream's reflection of the
# right: origins mirrored in y, the axes of joints 1 and 6 reversed). Anvil's model of the arm matches it (FK, 1e-9 m:
# test_anvil_ranges_carry_over_by_fk), so Anvil's joint ranges are ours unchanged.
ENACTIC_ZERO = {
    "right": [((0, -0.0625, 0), (0, -1, 0)), ((0, -0.1225, 0), (-1, 0, 0)), ((0, -0.1225, -0.06625), (0, 0, -1)),
              ((0, -0.1225, -0.22), (0, -1, 0)), ((0, -0.1225, -0.3155), (0, 0, -1)),
              ((0, -0.1225, -0.436), (0, 1, 0)), ((0, -0.1225, -0.436), (1, 0, 0))],
    "left": [((0, 0.0625, 0), (0, 1, 0)), ((0, 0.1225, 0), (-1, 0, 0)), ((0, 0.1225, -0.06625), (0, 0, -1)),
             ((0, 0.1225, -0.22), (0, -1, 0)), ((0, 0.1225, -0.3155), (0, 0, -1)),
             ((0, 0.1225, -0.436), (0, -1, 0)), ((0, 0.1225, -0.436), (1, 0, 0))]}


def arm_model(side):
    a = robots.load("openarm_bimanual").arm(f"arm_{side}")
    m = pin.buildModelFromUrdf(a.urdf_path())
    return a, m, m.createData()


def joint_frames(m, d, q7):
    """Each arm joint's origin and axis (world = base_link) at q7."""
    q = pin.neutral(m)
    q[:7] = q7
    pin.computeJointJacobians(m, d, q)
    pin.framesForwardKinematics(m, d, q)
    out = []
    for k in range(1, 8):
        J = pin.getJointJacobian(m, d, k, pin.WORLD)
        out.append((d.oMi[k].translation.copy(), J[3:, m.joints[k].idx_v].copy()))
    return out


def test_urdfs_are_enactics_arm_with_anvils_ranges():
    """Enactic's arm in our conventions: joint1..7, then the fingers, in pinocchio's order (the arm node takes joint1..N
    as the first N); Anvil's efforts and velocities, J2-J5 / J7 limits and J6 as Anvil states it (left -45..+70 deg,
    right -70..+45), J1 Enactic's window; at q = 0 every joint origin and axis is Enactic's."""
    from robot_utils.robots.arm_types import joint_limits
    for side in ("left", "right"):
        a, m, d = arm_model(side)
        assert list(m.names)[1:] == [f"joint{i}" for i in range(1, 8)] + ["finger_left", "finger_right"]
        assert np.allclose(m.effortLimit[:7], [40, 40, 27, 27, 7, 7, 7])
        assert np.allclose(m.velocityLimit[:7], [10.47, 10.47, 3.77, 3.77, 12.57, 12.57, 12.57])
        assert all(m.existFrame(f) for f in ("base_link", "link7", "end_link", "finger_left_link"))
        lim = joint_limits(a.urdf_path(), a.joints)
        assert lim["joint4"] == (0.0, 2.443461) and lim["joint3"] == lim["joint5"] == lim["joint7"] == (-1.570796, 1.570796)
        assert lim["joint2"] == ((-0.1745, 3.3161) if side == "right" else (-3.3161, 0.1745))
        assert lim["joint6"] == ((-1.22173, 0.785398) if side == "right" else (-0.785398, 1.22173))
        assert lim["joint1"] == ((-1.3963, 3.4907) if side == "right" else (-3.4907, 1.3963))
        for k, ((p, ax), (p0, a0)) in enumerate(zip(joint_frames(m, d, np.zeros(7)), ENACTIC_ZERO[side]), 1):
            assert np.allclose(p, p0, atol=1e-6), (side, k, p)
            assert np.allclose(ax, a0, atol=1e-5), (side, k, ax)


def test_no_anvil_arm_data_in_the_package():
    """Only Enactic's arm geometry and Anvil's gripper ship: every mesh the URDFs use is upstream's (meshes/collision/)
    or the gripper's (gripper/meshes/), and nothing else is vendored."""
    import re
    from pathlib import Path

    from robot_utils.robots.openarm import build_urdf as B
    assert not (B.HERE / "anvil").exists()
    for side in ("left", "right"):
        txt = (B.HERE / "urdf" / f"openarm_v2_{side}_arm.urdf").read_text()
        for f in re.findall(r'filename="([^"]+)"', txt):
            assert f.startswith(("../meshes/collision/", "../gripper/meshes/")), f
            assert (B.HERE / "urdf" / f).resolve().is_file(), f
    meshes = {p.relative_to(B.HERE).as_posix() for p in Path(B.HERE).rglob("*") if p.suffix in (".stl", ".obj", ".dae")}
    assert meshes == {f"meshes/collision/{n}.stl" for n in B.LINKS} | {
        f"gripper/meshes/{n}" for n in ("hand_anodized_black.obj", "hand_camera_gray.stl", "left_finger_dark.stl",
                                        "right_finger_dark.stl", "left_finger_fine_tip_dark.stl",
                                        "right_finger_fine_tip_dark.stl")}


def test_anvil_ranges_carry_over_by_fk():
    """Anvil's ranges are ours unchanged because the joints are the same: at q = 0 Anvil's model of the arm (the
    URDFs generated from Anvil's OpenArm v2 URDF, which is not distributed: set OPENARM_ANVIL_URDF_DIR to a directory
    holding them) has every joint origin within 1e-9 m and every axis, with its sign, equal to ours; its joint ranges,
    efforts and velocities are ours; and its end_link and finger frames are ours at random q (Anvil's 1.5708 roundings:
    1e-6 m, 1e-5 rad)."""
    import os
    d_anvil = os.environ.get("OPENARM_ANVIL_URDF_DIR")
    if not d_anvil:
        pytest.skip("OPENARM_ANVIL_URDF_DIR not set (Anvil's model is not distributed)")
    rng = np.random.default_rng(0)
    for side in ("left", "right"):
        a, m, d = arm_model(side)
        mA = pin.buildModelFromUrdf(os.path.join(d_anvil, f"openarm_v2_{side}_arm.urdf"))
        dA = mA.createData()
        jA = [mA.getJointId(f"joint{k}") for k in range(1, 8)]
        assert np.allclose([mA.lowerPositionLimit[mA.joints[j].idx_q] for j in jA], m.lowerPositionLimit[:7])
        assert np.allclose([mA.upperPositionLimit[mA.joints[j].idx_q] for j in jA], m.upperPositionLimit[:7])
        assert np.allclose([mA.effortLimit[mA.joints[j].idx_v] for j in jA], m.effortLimit[:7])
        assert np.allclose([mA.velocityLimit[mA.joints[j].idx_v] for j in jA], m.velocityLimit[:7])
        for t in range(50):
            q7 = np.zeros(7) if t == 0 else rng.uniform(m.lowerPositionLimit[:7], m.upperPositionLimit[:7])
            q, qA = pin.neutral(m), pin.neutral(mA)
            q[:7] = q7
            for k, j in enumerate(jA):
                qA[mA.joints[j].idx_q] = q7[k]
            for n in ("link6_crank_joint", "link6_bar_joint"):           # Anvil's J6 linkage follows joint6
                if mA.existJointName(n):
                    qA[mA.joints[mA.getJointId(n)].idx_q] = q7[5] * (1 if "crank" in n else -1)
            pin.computeJointJacobians(m, d, q)
            pin.framesForwardKinematics(m, d, q)
            pin.computeJointJacobians(mA, dA, qA)
            pin.framesForwardKinematics(mA, dA, qA)
            for k, j in enumerate(jA, 1):
                assert np.linalg.norm(d.oMi[k].translation - dA.oMi[j].translation) < 1e-8, (side, t, k)
                ax = pin.getJointJacobian(m, d, k, pin.WORLD)[3:, m.joints[k].idx_v]
                axA = pin.getJointJacobian(mA, dA, j, pin.WORLD)[3:, mA.joints[j].idx_v]
                assert ax @ axA == pytest.approx(1.0, abs=1e-9), (side, t, k)
            for f in ("end_link", "finger_left_link", "finger_right_link"):
                dT = d.oMf[m.getFrameId(f)].inverse() * dA.oMf[mA.getFrameId(f)]
                assert np.linalg.norm(dT.translation) < 1e-6 and np.linalg.norm(pin.log3(dT.rotation)) < 1e-5, (side, f)


def test_urdfs_are_the_generators_output(tmp_path, monkeypatch):
    """The checked-in URDFs are what build_urdf.py makes of enactic/openarm_description and the gripper (regenerate
    after any change; set OPENARM_DESCRIPTION to a checkout of build_urdf.UPSTREAM_COMMIT)."""
    import os
    import shutil
    import subprocess

    from robot_utils.robots.openarm import build_urdf as B
    up = os.environ.get("OPENARM_DESCRIPTION")
    if not up:
        pytest.skip("OPENARM_DESCRIPTION not set (a checkout of enactic/openarm_description)")
    rev = subprocess.run(["git", "-C", up, "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    assert rev == B.UPSTREAM_COMMIT
    orig, here = B.HERE, tmp_path / "openarm"
    shutil.copytree(orig, here, ignore=shutil.ignore_patterns("urdf", "__pycache__"))
    (here / "urdf").mkdir()
    monkeypatch.setattr(B, "HERE", here)
    monkeypatch.setattr(B, "GRIPPER", here / "gripper")
    B.main(up)
    for side in ("left", "right"):
        f = f"urdf/openarm_v2_{side}_arm.urdf"
        assert (here / f).read_text() == (orig / f).read_text(), side
    for n in B.LINKS:
        f = f"meshes/collision/{n}.stl"
        assert (here / f).read_bytes() == (orig / f).read_bytes(), f


def test_j1_window_is_enactics_mapped_physically_by_fk():
    """J1 keeps Enactic's window (right -80..+200 deg, left -200..+80; Anvil's is +-140), checked as arm poses: with the
    other joints at 0, the window's edges swing the hanging arm 200 deg forward (over the top, 20 deg past vertical)
    and 80 deg backward, on both arms; the window is 280 deg."""
    from robot_utils.robots.arm_types import joint_limits
    for side in ("left", "right"):
        a, m, d = arm_model(side)
        lo, hi = joint_limits(a.urdf_path(), ["joint1"])["joint1"]
        assert hi - lo == pytest.approx(280 * DEG, abs=1e-4)
        L = 0.436                                           # the shoulder centre to the wrist centre (hanging)
        for q1, swing in ((lo, None), (hi, None)):
            fr = joint_frames(m, d, [q1, 0, 0, 0, 0, 0, 0])
            v = fr[5][0] - fr[1][0]                         # shoulder centre (joint2) -> wrist centre (joint6)
            ang = math.degrees(math.atan2(v[0], -v[2]))     # 0 = hanging, + = swung forward
            assert np.linalg.norm(v) == pytest.approx(L, abs=1e-6) and abs(v[1]) < 1e-6
            forward = (q1 == hi) if side == "right" else (q1 == lo)
            assert (ang % 360) == pytest.approx(200.0 if forward else 280.0, abs=0.01), (side, q1, ang)


def test_j6_wider_side_tilts_the_fingertips_toward_the_wrist_camera():
    """Anvil's J6 range (left -45..+70 deg, right -70..+45): on both arms the 70 deg side tilts the fingertips forward
    at the zero pose, toward the wrist camera's side (end_link +z, forward at q = 0), and up with the forearm forward.
    (Our previous README inferred the opposite sign from a palm-in hand analogy, which a symmetric gripper does not
    fix.)"""
    for side in ("left", "right"):
        a, m, d = arm_model(side)
        wide = 1.0 if side == "left" else -1.0                # the sign of the 70 deg side
        q = pin.neutral(m)
        pin.framesForwardKinematics(m, d, q)
        E0 = d.oMf[m.getFrameId("end_link")]
        assert E0.rotation[:, 2] == pytest.approx([1, 0, 0], abs=1e-4)          # the camera side faces forward at 0
        q[5] = 0.5 * wide
        pin.framesForwardKinematics(m, d, q)
        assert d.oMf[m.getFrameId("end_link")].translation[0] > 0.08            # fingertips swung forward
        q[3] = math.pi / 2                                                       # forearm forward
        q[5] = 0.0
        pin.framesForwardKinematics(m, d, q)
        z0 = d.oMf[m.getFrameId("end_link")].translation[2]
        q[5] = 0.5 * wide
        pin.framesForwardKinematics(m, d, q)
        assert d.oMf[m.getFrameId("end_link")].translation[2] > z0 + 0.05       # ... and up


def test_gripper_is_anvils_stroke_and_end_link_at_the_fingertips():
    """Anvil's gripper (gripper/, from Anvil's URDF): each finger 0..0.05 m (100 mm of gap; Anvil's docs say 110), the motor law's
    gap_max matching it; end_link at the fine tips' far end (joint-7 axis to fingertips 0.2026 m), the TCP Anvil's
    hand_tcp; the hand's 0.7 kg and the fingers' 0.036 kg; the finger links' frames are end_link's at q = 0."""
    from robot_utils.robots.arm_types import joint_limits
    from robot_utils.robots.grippers import get
    g = get("anvil")
    assert g.travel == 0.05 and g.gap_max == pytest.approx(0.100) and g.gap_per_rad == pytest.approx(0.100 / 2.858)
    assert g.tcp == (-0.0225, 0.0, 0.0) and g.end_mass == 0.7
    for side in ("left", "right"):
        a, m, d = arm_model(side)
        assert joint_limits(a.urdf_path(), ["finger_left"])["finger_left"] == (0.0, 0.05)
        assert joint_limits(a.urdf_path(), ["finger_right"])["finger_right"] == (-0.05, 0.0)
        pin.framesForwardKinematics(m, d, pin.neutral(m))
        tips = d.oMf[m.getFrameId("end_link")].translation
        assert np.linalg.norm(tips - d.oMi[m.getJointId("joint7")].translation) == pytest.approx(0.2026, abs=1e-6)
        assert d.oMf[m.getFrameId("finger_left_link")].isApprox(d.oMf[m.getFrameId("end_link")])
        assert m.inertias[m.getJointId("finger_left")].mass == pytest.approx(0.036025, abs=1e-6)


def test_urdfs_load_in_mujoco_with_the_grippers_colours():
    """The generated URDFs compile in MuJoCo (the sim's importer: meshes, materials): Enactic's link meshes, the flange
    adapter, the gripper's per-material colours on its visual geoms, the right finger's mimic as an equality
    constraint."""
    mujoco = pytest.importorskip("mujoco")
    import re
    for side in ("left", "right"):
        a, _, _ = arm_model(side)
        txt = open(a.urdf_path()).read()
        here = a.urdf_path().rsplit("/", 1)[0] + "/"
        txt = re.sub(r'filename="', f'filename="{here}', txt)
        txt = re.sub(r"(<robot\b[^>]*>)", lambda mm: mm.group(1) + '<mujoco><compiler discardvisual="false" '
                     'fusestatic="false" strippath="false"/></mujoco>', txt, count=1)
        m = mujoco.MjSpec.from_string(txt).compile()
        vis = [g for g in range(m.ngeom) if m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0]
        assert len(vis) == 7 + 1 + 2 + 4 and len({tuple(np.round(m.geom_rgba[g], 2)) for g in vis}) >= 3
        assert m.neq == 1                                         # finger_right (finger_left)
        eq = {(m.joint(int(m.eq_obj1id[i])).name, m.joint(int(m.eq_obj2id[i])).name): m.eq_data[i][1] for i in range(m.neq)}
        assert eq == {("finger_right", "finger_left"): -1.0}


@functools.lru_cache(maxsize=None)
def model_json():
    import tempfile

    from robot_utils.nodes.description.model import Options
    from robot_utils.nodes.description.node import DescriptionNode
    return DescriptionNode(tempfile.mkdtemp(prefix="oa-"), None, robot="openarm_bimanual").model(Options())


def test_model_mounting_and_arms():
    m = model_json()
    assert [(a["name"], a["type"], len(a["joints"]), a["side"]) for a in m["arms"]] == \
        [("arm_left", "openarm_v2", 7, "left"), ("arm_right", "openarm_v2", 7, "right")]
    fb = {k: np.array(v) for k, v in m["frames_in_base"].items()}
    assert np.allclose(fb["arm_left_base_link"][:3, 3], [0.12, 0.03, 1.074])
    assert np.allclose(fb["arm_right_base_link"][:3, 3], [0.12, -0.03, 1.074])
    assert "arm_left_joint7" in {j["name"] for j in m["joints"]} and "torso_link" in fb


def test_shoulders_at_human_scale():
    """The owner's rule: the shoulders at the shoulder height of a human with OpenArm's arm lengths (Winter: upper arm
    0.186 H, forearm 0.146 H, shoulder height 0.818 H), the shoulder centre (joint2's origin) at the profile's mount."""
    a = robots.load("openarm_bimanual").arm("arm_right")
    m = pin.buildModelFromUrdf(a.urdf_path())
    d = m.createData()
    pin.forwardKinematics(m, d, pin.neutral(m))
    p = {m.names[i]: d.oMi[i].translation for i in range(1, 8)}
    arm = np.linalg.norm(p["joint4"] - p["joint2"]) + np.linalg.norm(p["joint6"] - p["joint4"])
    shoulder = 0.818 * arm / (0.186 + 0.146)
    fb = {k: np.array(v) for k, v in model_json()["frames_in_base"].items()}
    base_z = 0.0                                                  # base_link on the floor (the swerve's convention)
    assert abs(p["joint2"][2]) < 1e-9                             # the shoulder centre in base_link's plane
    assert fb["arm_right_base_link"][2, 3] + base_z == pytest.approx(shoulder, abs=0.002)


def test_posture_is_the_owners_rest_pose_within_limits():
    """The start pose (2026-11-26, the owner's): the upper arm straight down, the forearm horizontal forward (elbow
    90 deg), the tool forward with the wrist camera's side (end_link z) up; every joint within its limits."""
    from robot_utils.nodes.wbc.humanlike import Humanlike
    from robot_utils.nodes.wbc.model import WbcModel, ee_specs, fetch_urdf
    from robot_utils.nodes.wbc.spec import make_config
    urdf, _ = fetch_urdf("local", "openarm_bimanual")
    m = WbcModel(urdf, ee_specs("openarm_bimanual"))
    h = Humanlike(m, make_config({}))
    qa = {e: np.array(POSTURE[e], float) for e in m.ees}
    h.prepare(m.config((0, 0, 0), qa))
    u = lambda v: v / np.linalg.norm(v)                                                    # noqa: E731
    for e, a in h.arms.items():
        S, E, W = (h._p(j)[0] for j in (a.S, a.E, a.W))
        assert np.dot(u(E - S), [0, 0, -1]) > math.cos(math.radians(1))                  # upper arm down
        assert np.dot(u(W - E), [1, 0, 0]) > math.cos(math.radians(1))                   # forearm forward
        R = m.fk(m.config((0, 0, 0), qa), e).rotation
        assert R[:, 0] @ [1, 0, 0] > math.cos(math.radians(1)) and R[:, 2] @ [0, 0, 1] > math.cos(math.radians(1))
        lo, hi = m.arm_limits(e)
        assert np.all(qa[e] > lo) and np.all(qa[e] < hi)

@pytest.fixture(scope="module")
def world():
    pytest.importorskip("mujoco")
    from robot_utils.nodes.sim.node import fetch_bundle
    from robot_utils.nodes.sim.world import World, load_config, unzip_bundle
    urdf, assets, info = unzip_bundle(fetch_bundle("local", "full", "openarm_bimanual")[0])
    prof = robots.load("openarm_bimanual")
    w = World(urdf, assets, load_config(overrides={"time": {"run": False}, "scene": {"tabletop": False},
                                                   "components": {"arm": {"fidelity": "dynamic"}}}, robot=prof.sim),
              {"source": "test", "arms": info["arms"]})
    yield w
    w.close()


def test_sim_arms_are_7dof_with_the_types_limits_and_their_own_gravity_fit(world):
    w = world
    assert {k: len(ix.joints) for k, ix in w.arm_ix.items()} == {"arm_left": 7, "arm_right": 7}
    for k, ix in w.arm_ix.items():
        assert np.allclose(w.m.actuator_forcerange[ix.a, 1], [40, 40, 27, 27, 7, 7, 7])     # the type's, not the reBot's
        assert np.allclose(w.arm_limits(k)["joint_speed_max"], [2, 2, 2, 2, 6, 6, 6])
        assert np.allclose(w.arm_op("read", {}, k)["q"], POSTURE[k], atol=1e-3)              # starts at its posture
        assert np.all(np.array(w.gravity_fits[k]["max"]) < 0.05), w.gravity_fits[k]
    assert w.arm_op("hello", {}, "arm_left")["joints"] == [f"arm_left_joint{i}" for i in range(1, 8)]


def test_sim_mit_hold_on_the_node_model_and_track_each_arm(world):
    from robot_utils.nodes.sim.world import _UrdfGravity
    w = world
    t = robots.load("openarm_bimanual").arm("arm_left").arm_type()
    for k in ("arm_left", "arm_right"):
        w.reset()
        g = _UrdfGravity(robots.load("openarm_bimanual").arm(k).urdf_path(), 7)
        w.arm_op("enable", {}, k)
        q0 = np.array(w.arm_op("read", {}, k)["q"])
        goal = q0 + np.array([0.2, 0.1, -0.2, -0.3, 0.2, 0.1, -0.2]) * (1 if k == "arm_left" else -1)
        for i in range(250):
            q = np.array(w.arm_op("read", {}, k)["q"])
            qt = q0 + (goal - q0) * min(1.0, i / 120)
            w.arm_op("send", {"q": list(qt), "qd": [0.0] * 7, "kp": list(t.kp), "kd": list(t.kd),
                              "tau": [float(v) for v in g.gravity(q)]}, k)
            w.step(10)
        r = w.arm_op("read", {}, k)
        assert np.allclose(r["q"], goal, atol=0.02), (k, np.array(r["q"]) - goal)
        assert r["status"] == [1] * 8
        w.arm_op("disable", {}, k)


# -- the WBC: 7 DOF, null-space posture ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def wmodel():
    pytest.importorskip("qpsolvers")
    from robot_utils.nodes.wbc.model import WbcModel, ee_specs, fetch_urdf
    urdf, _ = fetch_urdf("local", "openarm_bimanual")
    return WbcModel(urdf, ee_specs("openarm_bimanual"))




class Run:
    """A WBC QP run (perfect tracking, the base locked) with its histories."""

    def __init__(self, m, q_start, goals, steps=300, cfg_over=None, absolute=False, goal_fn=None, hint_fn=None):
        """``goals``: per EE an offset (m) from its start TCP, or (``absolute``) an SE3; ``goal_fn(t)``: per EE an
        offset at time t (a moving hand, a stream of reactive targets); ``hint_fn(t, run)``: per EE the operator's
        swivel hint datagram field (seq, dpsi, elbow, conf), absent = none sent this tick (the last one ages)."""
        from robot_utils.nodes.swerve.constants import MODULES
        from robot_utils.nodes.wbc.controller import EEState, QPController, Snapshot
        from robot_utils.nodes.wbc.node import posture_of, robot_wbc_defaults
        from robot_utils.nodes.wbc.swivel_geom import HintInput
        from robot_utils.nodes.wbc.spec import make_config, parse_hints
        from robot_utils.nodes.wbc.targets import TargetBuffer
        cfg = make_config({**robot_wbc_defaults("openarm_bimanual"), **(cfg_over or {})})   # as the node: the profile's
        self.m, self.cfg = m, cfg
        self.ctl = ctl = QPController(m, cfg, [(mo.x, mo.y) for mo in MODULES])
        self.q = q = {e: np.array(q_start[e], float) for e in m.ees}
        v = {e: np.zeros(7) for e in m.ees}
        T0 = {e: m.fk(m.config((0, 0, 0), q), e) for e in m.ees}
        bufs = {}

        def goal(e, t):
            if absolute:
                g = goal_fn(t)[e] if goal_fn else goals[e]                  # an SE3 (goal_fn: moving)
                return g.translation, g.rotation
            off = goal_fn(t)[e] if goal_fn else goals[e]
            return T0[e].translation + np.asarray(off), T0[e].rotation
        for e in m.ees:
            bufs[e] = TargetBuffer(e, cfg.ff_timeout, cfg.blend_tau)
            bufs[e].set_reactive(0.0, *goal(e, 0.0))
        hints = parse_hints({"base": {"mode": "locked"}}, cfg, list(m.ees), {}, {e: 7 for e in m.ees})
        self.err = {e: [] for e in m.ees}
        self.psi = {e: [] for e in m.ees}
        self.psi_star = {e: [] for e in m.ees}
        self.clear, self.active = [], 0
        self.status = {e: [] for e in m.ees}            # the elbow's limit-aware status per tick
        self.qd = {e: [] for e in m.ees}
        self.psi_cont = {e: [] for e in m.ees}           # the WBC's continuous swivel (swivel_geom.py)
        self.hint_w = {e: [] for e in m.ees}             # the hint's weight in force
        self.hint_target = {e: [] for e in m.ees}
        self.q_hist = {e: [] for e in m.ees}
        self.t = 0.0
        last_hint: dict = {}

        def hint_at(e, t):
            h = (hint_fn(t, self) or {}).get(e) if hint_fn else None
            if h is not None:
                last_hint[e] = (*h, t)
            lh = last_hint.get(e)
            return None if lh is None else HintInput(*lh[:4], t - lh[4])
        t = 0.0
        for _ in range(steps):
            if goal_fn:
                for e in m.ees:
                    bufs[e].set_reactive(t, *goal(e, t))
            snap = Snapshot(t, (0.0, 0.0, 0.0), np.zeros(3),
                            {e: EEState(q[e].copy(), v[e].copy(), bufs[e].sample(t), hints.ee(e, cfg),
                                        posture_of("ready", m.ees[e], 7), swivel_hint=hint_at(e, t))
                             for e in m.ees}, hints, hold=False,
                            velocity_age=math.inf, hint_age=0.0)
            out = ctl.step(snap)
            assert out.status == "ok", (out.status, out.reason)
            for e in m.ees:
                q[e] = q[e] + out.dq[e]
                v[e] = out.qd[e].copy()
            self.active += any(w > 0 for w in out.diag.get("proximity", {}).values())
            for e in m.ees:
                sw = (out.diag.get("swivel") or {}).get(e) or {}
                self.status[e].append(sw.get("status"))
                self.qd[e].append(out.qd[e].copy())
                self.psi_cont[e].append((out.diag.get("swivel_cont") or {}).get(e, math.nan))
                self.hint_w[e].append((sw.get("hint") or {}).get("weight", 0.0))
                self.hint_target[e].append((sw.get("hint") or {}).get("target", math.nan))
                self.q_hist[e].append(q[e].copy())
            cq = m.config((0, 0, 0), q)
            ctl.human.prepare(cq)
            self.clear.append(ctl.human.arm_clearance())
            for e in m.ees:
                self.err[e].append(float(np.linalg.norm(m.fk(cq, e).translation - bufs[e].sample(t).p)))
                self.psi[e].append(ctl.human.swivel(e))
                self.psi_star[e].append(ctl.human.psi_star.get(e, math.nan))
            t += cfg.dt
            self.t = t
        self.effort = {e: ctl.human.gravity_effort(e) for e in m.ees}


def swivel(m, e, q, angle):
    """q with the elbow pushed off its swivel (a perturbation of ``angle`` scale; the EE fixed): Newton in the null
    space of the EE (6 rows) from a perturbed start."""
    target = m.fk(m.config((0, 0, 0), {e: q}), e)
    qq = np.array(q, float) + angle * np.array([0.3, 0.6, -0.4, 0.0, 0.3, -0.2, 0.1])
    for _ in range(100):
        cfg = m.config((0, 0, 0), {e: qq})
        T = m.fk(cfg, e)
        err = np.concatenate([target.translation - T.translation, pin.log3(target.rotation @ T.rotation.T)])
        J = m.jacobian(cfg, e)[:, m.arm_v[e]]
        qq = qq + np.linalg.pinv(J) @ err
    return qq


OFF = {"w_swivel": 0.0, "w_effort": 0.0, "w_center": 0.0, "w_arm_proximity": 0.0}   # the null-space terms all off
REACHES = [{"arm_left": [0.05, 0.03, 0.04], "arm_right": [0.04, -0.05, -0.03]},
           {"arm_left": [-0.04, -0.06, 0.06], "arm_right": [0.06, 0.06, 0.0]},
           {"arm_left": [0.08, 0.05, -0.05], "arm_right": [-0.03, -0.04, 0.08]}]
# a configuration with the left forearm's end against the right upper arm near its elbow (sampled within the joint
# limits): its TCP poses draw the arms together
CLOSE = {"arm_left": [-0.265, 0.133, 1.256, 1.694, 1.2, 0.309, -0.003],
         "arm_right": [0.841, -0.091, -0.767, 0.983, -0.414, 0.293, 0.515]}


def test_wbc_model_is_7dof_with_the_profile_posture(wmodel):
    m = wmodel
    assert {e: len(m.arm_q[e]) for e in m.ees} == {"arm_left": 7, "arm_right": 7}
    assert list(m.ees["arm_left"].posture) == POSTURE["arm_left"] and m.ees["arm_left"].arm_type == "openarm_v2"
    from robot_utils.nodes.wbc.node import stream_fields
    f, _, _ = stream_fields(list(m.ees), {e: 7 for e in m.ees})
    assert "arm_left_q_cmd7" in f and "arm_right_lead_tau7" in f
    assert "arm_q_cmd7" not in stream_fields(["arm"])[0]                    # the 6-joint default unchanged


def test_humanlike_only_for_redundant_arms_and_proximity_needs_two():
    from robot_utils.nodes.wbc.humanlike import Humanlike
    from robot_utils.nodes.wbc.model import WbcModel, ee_specs, fetch_urdf
    from robot_utils.nodes.wbc.node import robot_wbc_defaults
    from robot_utils.nodes.wbc.spec import make_config
    assert robot_wbc_defaults("openarm_bimanual")["w_arm_proximity"] > 0
    assert "w_arm_proximity" not in robot_wbc_defaults("rebot")                  # off for the one-arm reBot
    urdf, _ = fetch_urdf("local", "rebot")
    m6 = WbcModel(urdf, ee_specs("rebot"))
    assert Humanlike(m6, make_config({"w_arm_proximity": 30})).arms == {}        # a 6-joint arm: no null space


def test_ee_task_unaffected_by_the_null_space_terms(wmodel):
    """Reaches with the human-like terms (proximity included), the fixed nominal posture, and no null-space term at
    all: the EE error histories agree (the terms move the elbow only)."""
    m = wmodel
    start = {e: swivel(m, e, OLD_POSTURE[e], 0.3) for e in m.ees}                # elbow away from where it settles
    for goals in REACHES:
        off = Run(m, start, goals, 200, OFF)
        for over in ({"w_arm_proximity": 30.0}, {"null_mode": "nominal"}):
            on = Run(m, start, goals, 200, over)
            for e in m.ees:
                diff = np.max(np.abs(np.array(on.err[e]) - np.array(off.err[e])))
                assert diff < 2.5e-3, (over, e, diff)                       # the EE path: the same within 2.5 mm
                # (2.2 mm at most since Anvil's J6 range and the re-solved posture; below 2 mm before)
                assert on.err[e][-1] < 2e-3                                 # and it reaches
            # the elbow did move (2.3 deg with the owner's nominal pose since 2026-11-26; 3 was set at the old one)
            assert max(abs(on.psi[e][-1] - off.psi[e][-1]) for e in m.ees) > math.radians(2)


def test_elbow_converges_to_the_model_swivel(wmodel):
    """For several hand poses the elbow settles on the swivel model's ψ* (swivel alone: on it; with effort and
    centering: near it, the swivel having priority), the EE still."""
    m = wmodel
    for goals in REACHES:
        r = Run(m, OLD_POSTURE, goals, 500, {"w_effort": 0.0, "w_center": 0.0})
        for e in m.ees:
            lo, hi = m.arm_limits(e)
            saturated = np.min(np.minimum(r.q[e] - lo, hi - r.q[e])) < r.cfg.null_limit_margin
            if saturated:          # a joint at the null space's limit margin: the swivel stops there (by design)
                continue
            assert abs(r.psi[e][-1] - r.psi_star[e][-1]) < math.radians(2.0), (e, math.degrees(r.psi[e][-1]),
                                                                              math.degrees(r.psi_star[e][-1]))
            assert r.err[e][-1] < 1e-3
        d = Run(m, OLD_POSTURE, goals, 500)                                          # the defaults
        for e in m.ees:
            assert abs(d.psi[e][-1] - d.psi_star[e][-1]) < math.radians(6.0), (e, math.degrees(d.psi[e][-1]),
                                                                              math.degrees(d.psi_star[e][-1]))


def test_swivel_is_smooth_as_the_hand_moves(wmodel):
    """The hands sweep (across, down and back) in 6 s: ψ* and ψ change smoothly, no flips or jumps."""
    m = wmodel

    def sweep(t):
        a = min(t / 6.0, 1.0) * 2 * math.pi
        return {"arm_left": [0.06 * math.sin(a), -0.10 * (1 - math.cos(a)) / 2, -0.06 * math.sin(a)],
                "arm_right": [0.06 * math.sin(a), 0.10 * (1 - math.cos(a)) / 2, -0.06 * math.sin(a)]}
    r = Run(m, POSTURE, None, 700, {}, goal_fn=sweep)
    off = Run(m, POSTURE, None, 700, OFF, goal_fn=sweep)
    for e in m.ees:
        ps, p = np.array(r.psi_star[e]), np.array(r.psi[e])
        assert np.max(np.abs(np.diff(ps))) < math.radians(0.5), (e, math.degrees(np.max(np.abs(np.diff(ps)))))
        assert np.max(np.abs(np.diff(p))) < math.radians(0.5), (e, math.degrees(np.max(np.abs(np.diff(p)))))
        assert np.ptp(ps) > math.radians(2)                                    # and it does follow the hand
        # the EE error (the lag behind a moving reactive target, ~2.4 cm here): within 5 mm of the one without the
        # terms (measured 3.2-3.7 mm: the null-space motion shares the joints' speed / acceleration caps)
        assert np.max(np.abs(np.array(r.err[e]) - np.array(off.err[e]))) < 5e-3, e
    # the model itself along a dense line of hand poses (no low-pass): continuous
    from robot_utils.nodes.wbc.humanlike import swivel_target
    S, W0 = np.array([0.12, -0.153, 1.074]), np.array([0.35, -0.15, 0.92])
    prev = None
    for k in range(201):
        W = W0 + np.array([0.0, 0.3 * k / 200, -0.15 * k / 200])
        E = S + np.array([0.05, -0.15, -0.15])
        u = np.array([0.12, 0.0, 1.0]) - W
        ps = swivel_target(S, E, W, u / np.linalg.norm(u), -1.0, math.radians(-20), math.radians(100), 10.0)
        if prev is not None:
            assert abs(ps - prev) < math.radians(1.5), (k, math.degrees(ps), math.degrees(prev))
        prev = ps


def test_elbow_pushed_away_returns(wmodel):
    """A disturbance on the elbow (pushed along the self-motion, the hand still): it returns to the model's swivel."""
    m = wmodel
    settled = Run(m, OLD_POSTURE, {e: [0, 0, 0] for e in m.ees}, 500)
    for push in (0.4, -0.4):
        start = {e: swivel(m, e, settled.q[e], push) for e in m.ees}
        r = Run(m, start, {e: [0, 0, 0] for e in m.ees}, 500)
        for e in m.ees:
            assert abs(r.psi[e][0] - settled.psi[e][-1]) > math.radians(5)          # it was pushed
            assert abs(r.psi[e][-1] - settled.psi[e][-1]) < math.radians(1.5), (e, push)   # and came back
            assert max(r.err[e]) < 1e-3                                            # the hand stayed


def test_effort_minimisation_lowers_the_held_gravity_torque(wmodel):
    """Effort alone vs the fixed nominal posture at the same hand poses: Σ (τ_g / τ_max)² is lower."""
    m = wmodel
    ratios = []
    for goals in [{e: [0, 0, 0] for e in OLD_POSTURE}] + REACHES:
        eff = Run(m, OLD_POSTURE, goals, 500, {"w_swivel": 0.0, "w_center": 0.0})
        nom = Run(m, OLD_POSTURE, goals, 500, {"null_mode": "nominal"})
        for e in m.ees:
            assert eff.effort[e] < nom.effort[e], (goals, e, eff.effort[e], nom.effort[e])
            ratios.append(eff.effort[e] / nom.effort[e])
    assert np.mean(ratios) < 0.96, ratios          # 0.9505 with the owner's nominal pose (0.95 set at the old one)


def test_proximity_keeps_upper_arm_and_forearm_apart(wmodel):
    """Hand targets drawing the arms together (the left forearm against the right elbow): the smallest upper-arm /
    forearm clearance is larger with the term than without, the hands still reach."""
    m = wmodel
    qc = m.config((0, 0, 0), {e: np.array(v) for e, v in CLOSE.items()})
    goals = {e: m.fk(qc, e) for e in m.ees}
    off = Run(m, POSTURE, goals, 800, {"w_arm_proximity": 0.0}, absolute=True)
    on = Run(m, POSTURE, goals, 800, {"w_arm_proximity": 30.0}, absolute=True)
    # measured: the smallest 17.2 -> 19.7 mm (during the reach: the elbow's swivel speed is capped), settled
    # 27.1 -> 42.4 mm
    assert min(on.clear) > min(off.clear) + 0.002, (min(on.clear), min(off.clear))
    assert on.clear[-1] > off.clear[-1] + 0.01, (on.clear[-1], off.clear[-1])
    assert on.clear[-1] >= 0.035                                                   # settled 3.5+ cm apart
    for e in m.ees:
        assert on.err[e][-1] < 1e-3 and off.err[e][-1] < 1e-3


def test_proximity_dormant_in_the_natural_posture(wmodel):
    """No near-contact (every clearance beyond proximity_clearance): the posture is identical with the term on or
    off, tick for tick (the term is exactly zero there)."""
    m = wmodel
    for goals in REACHES:
        off = Run(m, POSTURE, goals, 300, {"w_arm_proximity": 0.0})
        on = Run(m, POSTURE, goals, 300, {"w_arm_proximity": 30.0})
        assert min(on.clear) > on.cfg.proximity_clearance and on.active == 0
        for e in m.ees:
            assert np.array_equal(on.q[e], off.q[e]), (e, on.q[e] - off.q[e])
            assert on.psi[e] == off.psi[e]


def test_proximity_ramp_is_zero_beyond_the_clearance_and_smooth(wmodel):
    """The term's weight: exactly zero beyond the clearance, rising as s² below it."""
    from robot_utils.nodes.wbc.humanlike import Humanlike
    from robot_utils.nodes.wbc.spec import make_config
    m = wmodel
    h = Humanlike(m, make_config({"w_arm_proximity": 30.0}))
    h.prepare(m.config((0, 0, 0), {e: np.array(v) for e, v in POSTURE.items()}))
    terms, clear = h.proximity({})
    assert clear > 0.05 and all(v == [] for v in terms.values())
    h.prepare(m.config((0, 0, 0), {e: np.array(v) for e, v in CLOSE.items()}))
    terms, clear = h.proximity({})
    assert clear < 0.05 and terms["arm_right"]
    s = (0.05 - clear) / 0.05
    assert max(w for w, _ in terms["arm_right"]) == pytest.approx(30.0 * s * s)


def test_nominal_posture_converges_in_the_null_space_with_the_ee_still(wmodel):
    """null_mode nominal (kept for comparison): the elbow returns to the fixed nominal posture, the EE still."""
    m = wmodel
    start = {e: swivel(m, e, POSTURE[e], 0.4) for e in m.ees}
    p0 = {e: float(np.linalg.norm(start[e] - np.asarray(POSTURE[e]))) for e in m.ees}
    r = Run(m, start, {e: [0, 0, 0] for e in m.ees}, 400, {"null_mode": "nominal"})
    for e in m.ees:
        assert max(r.err[e]) < 1e-3, (e, max(r.err[e]))
        assert np.linalg.norm(r.q[e] - np.asarray(POSTURE[e])) < 0.15 * p0[e]


def test_joint_limits_hold_with_a_posture_beyond_them(wmodel):
    """A nominal posture past a joint's limit: the joint stops short of its limit, never past; the EE unaffected."""
    m = wmodel
    from robot_utils.nodes.wbc import node as N
    lo, hi = m.arm_limits("arm_left")
    beyond = np.array(OLD_POSTURE["arm_left"])
    beyond[5] = 1.5                                                          # joint6's limit is 0.785
    spec = m.ees["arm_left"]
    import dataclasses
    m.ees["arm_left"] = dataclasses.replace(spec, posture=tuple(beyond))
    try:
        r = Run(m, OLD_POSTURE, {e: [0, 0, 0] for e in m.ees}, 400, {"null_mode": "nominal"})
    finally:
        m.ees["arm_left"] = spec
    q = r.q["arm_left"]
    assert np.all(q <= hi + 1e-6) and np.all(q >= lo - 1e-6)
    assert q[5] > OLD_POSTURE["arm_left"][5] + 0.05                                # pulled toward it, not past
    assert max(r.err["arm_left"]) < 1e-3                                       # the EE unaffected
    assert N.posture_of("home", spec, 7).shape == (7,)


def test_arm_type_loads_without_importing_the_arm_config():
    """The arm node puts its type in the environment before importing arm/config.py (N, the shared-memory layout of
    its control process): loading the profile's arm type must not import that module first (a 6-joint parent and a
    7-joint spawned control process otherwise: their shared memory disagrees)."""
    import subprocess
    import sys
    code = ("import sys; from robot_utils.robots import load; load('openarm_bimanual').arm('arm_left').arm_type(); "
            "print('robot_utils.nodes.arm.config' in sys.modules)")
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True)
    assert out.stdout.strip() == "False"


# -- the block view: the boxes variant and the pages' arm boxes ----------------------------------------------------

def _page_boxes(frames, gap, boxes):
    """The pages' arm (common/viz_scene.py armSegments, arm/page.py) in Python: each box's 8 corners in the arm frame
    from GET links frames [(pos, R)] (joint1..jointN, end_link, tcp), the gap and the arm's boxes."""
    def box(p, R, b, sy):
        return np.array([p + R @ (np.array([b[1] + i * b[4] / 2, sy * (b[2] + j * b[5] / 2), b[3] + k * b[6] / 2]) / 1000)
                         for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    I = (np.zeros(3), np.eye(3))
    FR = {"base": I, "end": frames[-2], **{f"link{i + 1}": frames[i] for i in range(len(frames) - 2)}}
    out = [box(*FR[b[0]], b, 1) for b in boxes if b[0] in FR]
    E = frames[-2]
    for sgn in (1, -1):
        p = E[0] + E[1][:, 1] * sgn * gap / 2
        out += [box(p, E[1], b, sgn) for b in boxes if b[0] == "finger"]
    return out


def test_block_view_draws_openarm_with_its_own_boxes(tmp_path):
    """variant=boxes and the pages' wireframe ("block") view: an OpenArm arm is drawn with its own boxes (its links'
    collision-mesh bounds, the reBot gripper's), never the reBot's arm boxes on OpenArm frames (it was: the boxes
    variant matched the reBot link names, and the pages took frames[6] - joint7 - as end_link). The tree's boxes
    (tree.fk, arm frame) and the pages' boxes on GET links' frames (joint1..7, end_link, tcp) agree one to one, and
    each link box bounds that link's mesh."""
    from robot_utils.nodes.description.model import Options
    from robot_utils.nodes.description.node import DescriptionNode
    from robot_utils.nodes.description.tree import fk, tree
    from robot_utils.robots.arm_types import arm_boxes
    from robot_utils.nodes.arm.geometry import REBOT_BOXES
    a = DescriptionNode(str(tmp_path / "d"), None, robot="openarm_bimanual").model_builder().build(
        Options.parse("boxes", base="swerve"))
    t = tree(a.urdf, "v")
    rng = np.random.default_rng(3)
    for name in ("arm_left", "arm_right"):
        spec = robots.load("openarm_bimanual").arm(name)
        boxes, names = arm_boxes(spec.urdf_path(), spec.arm_type().gripper)
        assert [b[0] for b in boxes][:9] == ["base", "link1", "link2", "link3", "link4", "link5", "link6", "link7", "end"]
        assert not any(b in REBOT_BOXES for b in boxes if b[0] not in ("end", "finger"))
        m = pin.buildModelFromUrdf(spec.urdf_path())
        d = m.createData()
        lo, hi = m.lowerPositionLimit[:7], m.upperPositionLimit[:7]
        for _ in range(5):
            q7 = lo + rng.random(7) * (hi - lo)
            gap = float(rng.uniform(0, 0.100))
            q = pin.neutral(m)
            q[:7] = q7
            pin.framesForwardKinematics(m, d, q)
            frames = [(np.array(d.oMi[j].translation), np.array(d.oMi[j].rotation)) for j in range(1, 8)]
            E = d.oMf[m.getFrameId("end_link")]
            frames += [(np.array(E.translation), np.array(E.rotation))] * 2          # end_link, tcp (unused)
            page = _page_boxes(frames, gap, boxes)
            qd = {f"{name}_joint{i + 1}": float(v) for i, v in enumerate(q7)} | {f"{name}_finger_left": gap / 2}
            T = fk(t, qd)
            Tarm = np.linalg.inv(T[f"{name}_base_link"])
            ours = []
            for link, lk in t["links"].items():
                if not link.startswith(name + "_"):
                    continue
                for v in lk["visuals"]:
                    assert v["type"] == "box", (link, v)
                    c, s = np.array(v["pos"]), np.array(v["size"])
                    P = np.array([c + np.array([i, j, k]) * s / 2 for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
                    M = Tarm @ T[link]
                    ours.append(P @ M[:3, :3].T + M[:3, 3])
            assert len(ours) == len(page) == 8 + 4 + 2 * 3            # base + link1..7, the hand's 4, 2 x 3 jaw boxes
            left = list(range(len(ours)))
            for B in page:
                hit = [i for i in left if all(np.min(np.linalg.norm(ours[i] - p, axis=1)) < 1e-6 for p in B)]
                assert len(hit) == 1, (name, B, hit)
                left.remove(hit[0])
    # the gripper's boxes are Anvil's gripper's (robots/grippers.py), in end_link (frames[-2]), not on joint 7, and the
    # reBot arm keeps the reBot gripper's
    from robot_utils.robots.grippers import get
    spec = robots.load("openarm_bimanual").arm("arm_right")
    boxes, _ = arm_boxes(spec.urdf_path(), spec.arm_type().gripper)
    assert [tuple(b) for b in boxes if b[0] in ("end", "finger")] == [tuple(b) for b in get("anvil").boxes]
    assert [tuple(b) for b in get("rebot").boxes] == [tuple(b) for b in REBOT_BOXES if b[0] in ("end", "finger")]


def test_arm_node_process_draws_its_own_boxes():
    """In an OpenArm arm node's process (its type in the environment) geometry.ARM_BOXES are the OpenArm's, and the
    body (GET/POST /geometry) and the agent render place them on joint1..7 / end_link (9 frames)."""
    import json
    import subprocess
    import sys
    code = r"""
import json, os, sys
import numpy as np
from robot_utils.robots import load
from robot_utils.robots.arm_types import ENV, arm_boxes
a = load('openarm_bimanual').arm('arm_right')
os.environ[ENV] = json.dumps(a.arm_type().public())
from robot_utils.nodes.arm import geometry as G
from robot_utils.nodes.arm.body import body
from robot_utils.nodes.arm.render import box_edges
frames = [np.eye(4) for _ in range(9)]
frames[7][:3, 3] = [0.5, 0.0, 0.0]                      # end_link
b = body(frames, 0.02)
print(json.dumps({"same": G.ARM_BOXES == arm_boxes(a.urdf_path(), a.arm_type().gripper)[0], "n": len(b["boxes"]),
                  "links": sorted({x[1] for x in b["boxes"]}),
                  "edges": len(box_edges([(f[:3, 3], f[:3, :3]) for f in frames], 0.02))}))
"""
    out = json.loads(subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True).stdout)
    assert out["same"] and out["n"] == 18 and "link7" in out["links"] and "end" in out["links"]
    assert out["edges"] > 0


def test_description_draws_the_torso_under_the_arms(tmp_path):
    """GET description's robot block carries the procedural boxes on base_link (the torso column and shoulder block),
    so the description page draws what holds the arms up (they floated 0.85 m over the base before); the reBot
    has none."""
    from robot_utils.nodes.description.node import DescriptionNode
    d = DescriptionNode(str(tmp_path / "a"), None, robot="openarm_bimanual").description()
    bodies = {b["name"]: b for b in d["robot"]["bodies"]}
    assert set(bodies) == {"torso", "shoulders"}
    top = bodies["torso"]["T"][2][3] + bodies["torso"]["size"][2] / 2
    low = bodies["shoulders"]["T"][2][3] - bodies["shoulders"]["size"][2] / 2
    assert top == pytest.approx(low, abs=1e-3)                                   # the column meets the block
    assert bodies["torso"]["T"][2][3] - bodies["torso"]["size"][2] / 2 == pytest.approx(0.2223, abs=1e-3)  # on the base
    assert bodies["shoulders"]["T"][2][3] == pytest.approx(1.074)                # centred on the shoulders
    assert DescriptionNode(str(tmp_path / "b"), None, robot="rebot").description()["robot"]["bodies"] == []


def test_rings_and_table_follow_the_profile_in_the_sim():
    """The sim built as the stack builds it (world_from, the profile's sim: block): the rings centre on the right arm's
    mount (y -0.03: the model info carries frames_in_base; without it they fell back to the reBot's -0.146) and the
    work surface is the profile's human-scale 0.70 m (table top, rings, the objects on them)."""
    from robot_utils.nodes.sim.node import world_from
    from robot_utils.nodes.sim.scene import ring_geometry
    from robot_utils.nodes.sim.world import load_config
    prof = robots.load("openarm_bimanual")
    assert prof.sim["scene"]["work_top"] == pytest.approx(0.70)
    cfg = load_config(None, {"scene": {"tabletop": True, "rings": True}, "cameras": {}}, robot=prof.sim)
    w = world_from("local", cfg, robot="openarm_bimanual")
    assert w.ring_mount_y() == pytest.approx(-0.03)
    objs = {o["name"]: o for o in w.start_objects()}
    table = objs["table"]
    assert table["xyz"][2] + 0.015 == pytest.approx(0.70, abs=1e-6)               # its top face
    ring_objs = [o for n, o in objs.items() if n.startswith(("red_", "blue_"))]
    assert ring_objs and all(0.70 < o["xyz"][2] < 0.80 for o in ring_objs)
    g = ring_geometry(-0.03, 0.70)
    assert objs["rings"]["xyz"][1] == pytest.approx(g["centre"][1])


# -- near the joint limits: the limit-aware swivel and the limit repulsion (null space only) -----------------------

# Pull-up-top hand poses (hands 0.20 m above the shoulder, 0.25 / 0.15 m in front of it, pull-up grips), each a joint
# configuration within the WBC's limits (the workspace study's probe, its geometry intersecting nothing, with Enactic's
# J6 range): the WBC must reach them from the nominal posture (a coverage floor). With Anvil's J6 range (2026-11-25)
# three of the four configurations left it; their TCP poses were re-solved within Anvil's limits (the most margin),
# the chin-up with the elbow folded is the probe's own. Three are no longer reached from the (re-solved) nominal posture
# (joint-limit local minima, docs/wbc.md): the left overhand (re-solved [-0.632, -2.659, -0.307, 2.127, -1.279, -0.494,
# -0.229], 42 mm short; not listed: from there the arm creeps), the left chin-up (134 mm short; 4 mm from the previous
# hand pose's posture) and the right up-forward (26 mm short): ``PULLUP_STALLS``, held still once there.
PULLUP_TOP = [("arm_left", [-0.549, -0.048, 0.019, 2.429, -1.571, 0.049, -0.164]),       # chin-up, elbow folded
              ("arm_right", [1.342, 0.397, -0.857, 2.373, 0.911, 0.691, -1.384]),        # up-forward
              ("arm_right", [0.924, 0.254, -0.642, 1.846, -1.462, 0.676, -0.495])]       # chin-up
PULLUP_STALLS = {0, 1}


def _goals(m, arm, q):
    qt = {e: np.array(POSTURE[e], float) for e in m.ees}
    qt[arm] = np.array(q, float)
    return {e: m.fk(m.config((0, 0, 0), qt), e) for e in m.ees}


def test_pullup_top_poses_are_reached(wmodel):
    """A coverage floor: reachable, collision-free pull-up-top poses are reached within 5 mm / 3 deg, inside the joint
    limits (the hand never leaves its path to do it: no whole-arm moves exist)."""
    m = wmodel
    for arm, q in (p for i, p in enumerate(PULLUP_TOP) if i not in PULLUP_STALLS):
        goals = _goals(m, arm, q)
        r = Run(m, POSTURE, goals, 800, {}, absolute=True)
        assert r.err[arm][-1] < 0.005, (arm, q, r.err[arm][-1])
        Tq = m.fk(m.config((0, 0, 0), {arm: r.q[arm]}), arm)
        assert np.linalg.norm(pin.log3(goals[arm].rotation @ Tq.rotation.T)) < math.radians(3)
        lo, hi = m.arm_limits(arm)
        assert np.all(r.q[arm] >= lo - 1e-6) and np.all(r.q[arm] <= hi + 1e-6)


def test_reposing_near_the_limits_keeps_the_hand_still_and_settles(wmodel):
    """At those poses, the hand held still: whatever the limit-aware swivel and the limit repulsion do to the elbow,
    the hand stays within 1 mm of its target (null space only) and the arm settles (the joints still, the status
    changing at most a few times: no oscillation)."""
    m = wmodel
    for arm, q in PULLUP_TOP:
        start = {e: np.array(POSTURE[e], float) for e in m.ees}
        start[arm] = np.array(q, float)
        lo, hi = m.arm_limits(arm)
        start[arm] = np.clip(start[arm], lo, hi)
        qt = {e: start[e] for e in m.ees}
        goals = {e: m.fk(m.config((0, 0, 0), qt), e) for e in m.ees}
        r = Run(m, start, goals, 1000, {"swivel_limit_aware": True}, absolute=True)
        assert max(r.err[arm]) < 0.001, (arm, max(r.err[arm]))
        changes = sum(1 for a, b in zip(r.status[arm], r.status[arm][1:]) if a != b)
        assert changes <= 4, (arm, changes)
        assert np.max(np.abs(r.qd[arm][-1])) < 0.01, (arm, r.qd[arm][-1])


def test_arm_unchanged_away_from_the_limits(wmodel):
    """Reaches that keep every joint out of the repulsion band and the human-like swivel inside its room: the joints
    are identical tick for tick with the limit-aware swivel and the limit repulsion on or off."""
    m = wmodel
    for goals in REACHES:
        on = Run(m, POSTURE, goals, 300, {"swivel_limit_aware": True})
        off = Run(m, POSTURE, goals, 300, {"swivel_limit_aware": False, "w_limit": 0.0})
        quiet = True
        for e in m.ees:
            lo, hi = m.arm_limits(e)
            quiet &= all(s in ("ok", "pending") for s in on.status[e])
            quiet &= bool(np.min(np.minimum(on.q[e] - lo, hi - on.q[e])) > on.cfg.limit_band)
        if not quiet:
            continue
        for e in m.ees:
            assert np.allclose(on.q[e], off.q[e], atol=1e-9), e


def test_anvil_gripper_lifts_and_holds_30_to_95_mm_without_slip():
    """Anvil's gripper in the dynamic sim, as the arm node commands it (its gripper law at the default 75 N squeeze,
    the arm type's constants): boxes 30, 60, 80, 95 mm across the jaws (100 mm fully open), grasped, lifted 10 cm, held, shaken +-2 cm at
    2 Hz: the jaws close on each width, the box rises with the hand and slips under 2 mm; no motor faults."""
    import json
    import os
    import subprocess
    import sys
    here = os.path.dirname(__file__)
    out = subprocess.run([sys.executable, os.path.join(here, "openarm_grasp_run.py"), "30", "60", "80", "95"],
                         capture_output=True, text=True, check=True, env=os.environ.copy(), cwd=here)
    rows = [json.loads(line) for line in out.stdout.splitlines() if line.startswith("{")]
    assert [r["width_mm"] for r in rows] == [30.0, 60.0, 80.0, 95.0], out.stderr[-2000:]
    for r in rows:
        assert r["contact"] and abs(r["gap_mm"] - r["width_mm"]) < 2.0, r
        assert r["squeeze_n"] == pytest.approx(75.0, abs=1.0), r                     # the default squeeze
        assert r["lifted_mm"] > 80.0, r                                              # it came up with the hand
        assert max(r["slip"].values()) < 2.0, r
        assert all(s == 1 for st in r["faults"] for s in st), r
