"""The robot model of the description node (model.py, GET /model*): assembly from components, the frame
tree against /description, URDF loading in pinocchio and MuJoCo, masses, limits, procedural inertias,
capsule fitting and the swerve dimensions against swerve/constants.py. docs/description_node.md."""
import io
import math
import zipfile

import numpy as np
import pytest
from fastapi.testclient import TestClient

from robot_utils.nodes.description.model import (ModelBuilder, ModelError, Options, fit_capsule, load_stl,
                                                 primitive_inertia, primitive_volume, resolve_file)
from robot_utils.nodes.description.node import DescriptionNode
from robot_utils.nodes.description.server import create_app

# The camera node's GET /description (fake camera): color 25 mm right of depth, IMU frame = depth axes
CAMERA_DESC = {"kind": "orbbec", "root": "depth_optical", "device": {"name": "Fake", "serial": "S1", "firmware": "0"},
               "frames": {"depth_optical": {"parent": None},
                          "color_optical": {"parent": "depth_optical", "rotation": np.eye(3).tolist(),
                                            "translation": [0.025, 0.0, 0.0]},
                          "imu": {"parent": "depth_optical", "rotation": [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
                                  "translation": [0.01, 0.002, -0.003]}}}


@pytest.fixture
def node(tmp_path):
    n = DescriptionNode(str(tmp_path / "desc"), None)
    n.cams["camera"] = {"description": CAMERA_DESC, "error": None, "t": 0.0}      # as if polled
    return n


def ground(node, normal=(0.05, -0.9, -0.3), height=0.48):
    nrm = np.asarray(normal, float)
    nrm /= np.linalg.norm(nrm)
    return node.store.save("ground", "S1", {"result": {"normal": nrm.tolist(), "height": height, "rms": 0.001,
                                                       "inliers": 5000}, "summary": "test"}, {})


def test_frame_tree_equals_description_nominal_and_calibrated(node):
    for calibrated in (False, True):
        if calibrated:
            ground(node)
        m = node.model(Options())
        assert m["description_frames"]["base"] == "base_link" and m["description_frames"]["arm"] == "arm_base_link"
        names = {"camera/depth_optical", "camera/color_optical", "camera/imu", "arm", "base"}
        assert names <= set(m["description_frames"])
        for desc, link in m["description_frames"].items():
            want = np.array(node.transform("base", desc)["matrix"])
            got = np.array(m["frames_in_base"][link])
            assert np.allclose(got, want, atol=1e-6), (desc, link, got, want)
        cam = next(c for c in m["components"] if c["name"] == "camera")
        assert ("ground" in cam["mount_from"]) == calibrated
    nominal = node.model(Options(mounts="nominal"))        # the nominal pose, whatever is calibrated
    T = np.array(nominal["frames_in_base"]["camera_depth_optical"])
    from robot_utils.nodes.description.frames import frame_T
    assert np.allclose(T, frame_T(node.store.description()["cameras"]["camera"]["nominal"]), atol=1e-6)


def test_http_model_agrees_with_description_and_transform(node):
    ground(node)
    with TestClient(create_app(node, start=False)) as c:
        d = c.get("/description").json()
        m = c.get("/model").json()
        assert m["model_version"] and m["root"] == "base_link" and m["options"]["variant"] == "full"
        for desc, link in m["description_frames"].items():
            assert np.allclose(m["frames_in_base"][link], d["frames_in_base"][desc], atol=1e-6), desc
        tr = c.get("/transform", params={"from": "arm", "to": "camera/depth_optical"}).json()
        T = np.linalg.inv(np.array(m["frames_in_base"]["arm_base_link"])) @ np.array(m["frames_in_base"]["camera_depth_optical"])
        assert np.allclose(T, tr["matrix"], atol=1e-6)
        r = c.get("/model/urdf", params={"variant": "simple", "base": "swerve"})
        assert r.status_code == 200 and r.headers["x-model-version"] == m["model_version"]
        assert r.text.startswith("<?xml") and "<mesh" not in r.text
        r = c.get("/model/urdf", params={"mesh_uri": "http"})
        assert "http://testserver/model/meshes/arm/meshes/collision/link2.STL" in r.text
        mesh = c.get("/model/meshes/arm/meshes/collision/link2.STL")
        assert mesh.status_code == 200 and mesh.content == resolve_file(
            "rebot_ctl:urdf/DM/meshes/collision/link2.STL").read_bytes()
        assert c.get("/model/meshes/arm/../urdf/ReBot_Arm_DM.urdf").status_code == 404
        assert c.get("/model/meshes/arm/meshes/nothing.stl").status_code == 404
        assert c.get("/model/urdf", params={"variant": "huge"}).status_code == 400
        assert c.get("/model/urdf", params={"variant": "simple,legs=full"}).status_code == 400
        assert c.get("/model/urdf", params={"base": "flying"}).status_code == 400
        z = c.get("/model/bundle.zip", params={"variant": "full,arm=simple"})
        assert z.status_code == 200 and z.headers["x-model-version"] == m["model_version"]
        names = zipfile.ZipFile(io.BytesIO(z.content)).namelist()
        assert "robot.urdf" in names and "model.json" in names and not any(n.startswith("meshes/") for n in names)


def test_model_version_follows_calibrations_and_description(node):
    v0 = node.model(Options())["model_version"]
    assert node.model(Options.parse("simple", base="planar"))["model_version"] == v0     # options don't change it
    cid = ground(node)
    v1 = node.model(Options())["model_version"]
    assert v1 != v0
    node.store.activate("ground", "S1", None)
    assert node.model(Options())["model_version"] == v0
    node.store.activate("ground", "S1", cid)
    assert node.model(Options())["model_version"] == v1
    node.set_nominal("camera", [0.1, 0.0, 0.5], [-1.5707963, 0.0, -1.5707963])
    assert node.model(Options())["model_version"] not in (v0, v1)


def test_urdf_loads_in_pinocchio_with_the_arm_names_limits_and_masses(node):
    import pinocchio as pin
    from reBotArm_control_py.kinematics.robot_model import _resolve_urdf, load_robot_model
    arm = load_robot_model()
    for opts, nq in ((Options(), 8), (Options.parse("simple", base="swerve"), 8 + 8 * 2), (Options(base="planar"), 12)):    # continuous yaw: nq 2
        a = node.model_builder().build(opts)
        m = pin.buildModelFromXML(a.urdf)
        assert m.nq == nq, opts
        for i in range(1, arm.njoints):
            j = m.getJointId("arm_" + arm.names[i])
            assert j < m.njoints, arm.names[i]
            qa, qm = arm.joints[i].idx_q, m.joints[j].idx_q
            assert math.isclose(m.lowerPositionLimit[qm], arm.lowerPositionLimit[qa])
            assert math.isclose(m.upperPositionLimit[qm], arm.upperPositionLimit[qa])
        arm_mass = sum(I.mass for I in arm.inertias)
        wrist = a.links["wrist_camera_depth_optical"]["mass"]        # rides on joint 6 (end_link)
        for i in range(1, arm.njoints):
            mm = m.inertias[m.getJointId("arm_" + arm.names[i])].mass
            assert math.isclose(mm, arm.inertias[i].mass + (wrist if arm.names[i] == "joint6" else 0.0),
                                rel_tol=1e-6), arm.names[i]
        # whole robot: base + arm + cameras (+ the planar base's two tiny links)
        total = sum(I.mass for I in m.inertias)
        assert math.isclose(total, a.mass(), rel_tol=1e-6)
        assert total > arm_mass
        # the same kinematics as the arm node's own model: the TCP in the arm frame agrees
        q = np.array([0.3, -0.8, -1.0, 0.2, 0.4, -0.5])
        da = arm.createData()
        qa = pin.neutral(arm)
        qa[:6] = q
        pin.framesForwardKinematics(arm, da, qa)
        dm = m.createData()
        qm = pin.neutral(m)
        for i in range(6):
            qm[m.joints[m.getJointId(f"arm_joint{i + 1}")].idx_q] = q[i]
        pin.framesForwardKinematics(m, dm, qm)
        T_arm = dm.oMf[m.getFrameId("arm_base_link")].inverse() * dm.oMf[m.getFrameId("arm_end_link")]
        assert np.allclose(T_arm.homogeneous, da.oMf[arm.getFrameId("end_link")].homogeneous, atol=1e-9)
    assert resolve_file(node.store.description()["components"]["arm"]["file"]) == \
        __import__("pathlib").Path(_resolve_urdf()[0]).resolve(), "the model's arm URDF is not the arm node's"


def _mujoco_from_bundle(node, **params):
    mujoco = pytest.importorskip("mujoco")
    data, _ = node.model_bundle(Options.parse(**params))
    z = zipfile.ZipFile(io.BytesIO(data))
    urdf = z.read("robot.urdf").decode()
    assets = {n: z.read(n) for n in z.namelist() if n.startswith("meshes/")}
    urdf = urdf.replace("<robot name=\"robot\">", '<robot name="robot"><mujoco><compiler fusestatic="false" '
                                                 'discardvisual="false" strippath="false"/></mujoco>', 1)
    return mujoco, mujoco.MjModel.from_xml_string(urdf, assets)


@pytest.mark.parametrize("params", [{}, {"variant": "simple", "base": "swerve"}, {"base": "planar"}])
def test_urdf_loads_in_mujoco(node, params):
    mujoco, m = _mujoco_from_bundle(node, **params)
    names = {m.joint(i).name for i in range(m.njnt)}
    assert {f"arm_joint{i}" for i in range(1, 7)} <= names
    if params.get("base") == "swerve":
        assert {f"base_{w}_steer" for w in ("FL", "FR", "RR", "RL")} <= names
    a = node.model_builder().build(Options.parse(**params))
    for link, v in a.links.items():      # links with an inertial (MuJoCo infers the rest from their geometry)
        if v["mass"] > 0:
            assert math.isclose(float(m.body(link).mass[0]), v["mass"], rel_tol=1e-6), link
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    assert np.isfinite(d.qacc).all()


def test_procedural_inertias_match_mujoco():
    mujoco = pytest.importorskip("mujoco")
    shapes = {"box": ({"size": [0.3, 0.2, 0.1]}, 'type="box" size="0.15 0.1 0.05"'),
              "cylinder": ({"radius": 0.05, "length": 0.2}, 'type="cylinder" size="0.05 0.1"'),
              "capsule": ({"radius": 0.04, "length": 0.3}, 'type="capsule" size="0.04 0.15"'),
              "sphere": ({"radius": 0.07}, 'type="sphere" size="0.07"')}
    for kind, (g, mj) in shapes.items():
        m = mujoco.MjModel.from_xml_string(f'<mujoco><worldbody><body><freejoint/><geom {mj} density="1000"/>'
                                           "</body></worldbody></mujoco>")
        mass = float(m.body_mass[1])
        assert math.isclose(mass, 1000 * primitive_volume(kind, g), rel_tol=1e-6), kind
        assert np.allclose(np.sort(np.diag(primitive_inertia(kind, mass, g))), np.sort(m.body_inertia[1]),
                           rtol=1e-6), kind


def test_procedural_components_mass_from_density_and_errors(node):
    d = node.store.description()
    d["components"]["post"] = {"source": "procedural", "type": "cylinder", "parent": "base_link", "radius": 0.02,
                               "length": 0.3, "density": 2700, "mount": {"xyz": [0, 0, 0.2225]},
                               "origin": {"xyz": [0, 0, 0.15]}}
    a = ModelBuilder(d, node.frames()).build()
    assert math.isclose(a.links["post_link"]["mass"], 2700 * math.pi * 0.02 ** 2 * 0.3)
    d["components"]["post"].pop("density")
    with pytest.raises(ModelError, match="mass or density"):
        ModelBuilder(d, node.frames()).build()
    d["components"]["post"].update(mass=1.0, parent="nowhere")
    with pytest.raises(ModelError, match="parent"):
        ModelBuilder(d, node.frames()).build()


def test_simple_capsules_bound_the_collision_meshes(node):
    a = node.model_builder().build(Options.parse("simple"))
    import xml.etree.ElementTree as ET
    from robot_utils.nodes.description.frames import frame_T
    root = ET.fromstring(a.urdf)
    tree = ET.fromstring(resolve_file("rebot_ctl:urdf/DM/urdf/ReBot_Arm_DM_mount_right.urdf").read_text("utf-8-sig"))
    checked = 0
    for lk in tree.findall("link"):
        col = lk.find("collision")
        if col is None:
            continue
        mesh = col.find("geometry/mesh").get("filename")
        P = load_stl(resolve_file("rebot_ctl:urdf/DM/urdf/" + mesh))
        o = col.find("origin")
        T = frame_T({"xyz": [float(x) for x in o.get("xyz").split()], "rpy": [float(x) for x in o.get("rpy").split()]})
        with np.errstate(all="ignore"):
            P = P @ T[:3, :3].T + T[:3, 3]
        ml = root.find(f"link[@name='arm_{lk.get('name')}']")
        spheres = [(np.array([float(x) for x in c.find("origin").get("xyz").split()]),
                    float(c.find("geometry/sphere").get("radius"))) for c in ml.findall("collision")
                   if c.find("geometry/sphere") is not None]
        (p0, r), (p1, _) = spheres if len(spheres) == 2 else (spheres[0], spheres[0])
        ax = p1 - p0
        with np.errstate(all="ignore"):
            t = np.clip(((P - p0) @ ax) / max(ax @ ax, 1e-12), 0, 1) if ax @ ax > 0 else np.zeros(len(P))
            dist = np.linalg.norm(P - (p0 + np.outer(t, ax)), axis=1)
        assert dist.max() <= r * (1 + 1e-6), lk.get("name")
        if lk.find("inertial") is not None:                                                 # full masses kept
            assert ml.find("inertial/mass").get("value") == lk.find("inertial/mass").get("value")
        checked += 1
    assert checked >= 9
    p0, p1, r = fit_capsule(np.array([[0, 0, 0], [1, 0, 0], [0.5, 0.1, 0]]))      # a thin line along x
    assert abs(abs(p1[0] - p0[0]) - (1 - 2 * r)) < 0.2 and r < 0.2


def test_swerve_base_dimensions_equal_swerve_constants(node):
    from robot_utils.nodes.swerve import constants as K
    c = node.store.description()["components"]["base"]
    assert math.isclose(c["frame_size"], K.FRAME, abs_tol=1e-9)
    assert np.allclose(c["frame_z"], [K.FRAME_BOTTOM_Z, K.FRAME_TOP_Z], atol=1e-9)
    assert np.allclose(c["top_rail_z"], [K.TOP_RAIL_BOTTOM_Z, K.TOP_Z], atol=1e-9)
    assert math.isclose(c["wheel_diameter"], K.WHEEL_DIAMETER, abs_tol=1e-9)
    assert math.isclose(c["module_size"], 2 * K.WHEEL_INSET, abs_tol=1e-9)
    assert list(c["modules"]) == [m.name for m in K.MODULES]
    for m in K.MODULES:
        assert np.allclose(c["modules"][m.name], [m.x, m.y], atol=1e-9), m.name
    from robot_utils.nodes.pose_servo.planner import STEER_RATE_MAX, WHEEL_SPEED_MAX
    assert math.isclose(c["steer_velocity"], STEER_RATE_MAX, rel_tol=1e-4)
    assert math.isclose(c["wheel_velocity"], WHEEL_SPEED_MAX / (K.WHEEL_DIAMETER / 2), rel_tol=1e-4)
    # and the generated model puts each steer axis there, the wheel axle one radius above the floor
    m = node.model(Options(base="swerve"))
    for mod in K.MODULES:
        T = np.array(m["frames_in_base"][f"base_{mod.name}_wheel"])
        assert np.allclose(T[:3, 3], [mod.x, mod.y, K.WHEEL_DIAMETER / 2], atol=1e-9)
    assert {j["name"] for j in m["joints"]} >= {"base_FL_steer", "base_FL_wheel_joint"}


@pytest.mark.parametrize("variant,base", [("simple", "swerve"), ("full", "swerve"), ("simple", "fixed"),
                                          ("boxes", "swerve")])
def test_model_tree_reproduces_the_urdf_kinematics(node, variant, base):
    """GET /model/tree (robot_tree.v1): its joints, composed as documented (tree.fk), give pinocchio's forward
    kinematics of the same URDF for random joint values; every link with a URDF visual has visuals."""
    import xml.etree.ElementTree as ET

    import pinocchio as pin

    from robot_utils.nodes.description.tree import fk, tree
    a = node.model_builder().build(Options.parse(variant, base=base))
    t = tree(a.urdf, "v")
    assert t["root"] == "base_link" and t["schema"] == "robot_tree.v1"
    m = pin.buildModelFromXML(a.urdf)
    d = m.createData()
    rng = np.random.default_rng(1)
    movable = [j for j in t["joints"] if j["type"] in ("revolute", "continuous", "prismatic") and "mimic" not in j]
    assert {j["name"] for j in movable} >= {f"arm_joint{i}" for i in range(1, 7)}
    for _ in range(5):
        qd = {}
        for j in movable:
            lo, hi = j.get("lower", -math.pi), j.get("upper", math.pi)
            qd[j["name"]] = float(rng.uniform(lo, hi)) if j["type"] != "continuous" else float(rng.uniform(-3, 3))
        q = pin.neutral(m)
        for j in t["joints"]:
            if j["type"] == "fixed":
                continue
            v = qd.get(j["name"], 0.0)
            if "mimic" in j:
                v = j["mimic"]["multiplier"] * qd.get(j["mimic"]["joint"], 0.0) + j["mimic"]["offset"]
            jid = m.getJointId(j["name"])
            iq = m.joints[jid].idx_q
            if m.joints[jid].nq == 2:                  # continuous: (cos, sin)
                q[iq:iq + 2] = [math.cos(v), math.sin(v)]
            else:
                q[iq] = v
        pin.framesForwardKinematics(m, d, q)
        T = fk(t, qd)
        for link, Tl in T.items():
            f = d.oMf[m.getFrameId(link)]
            assert np.allclose(Tl[:3, 3], f.translation, atol=1e-6), link
            assert np.allclose(Tl[:3, :3], f.rotation, atol=1e-6), link
    with_visual = {lk.get("name") for lk in ET.fromstring(a.urdf).findall("link") if lk.find("visual") is not None}
    assert with_visual and all(t["links"][n]["visuals"] for n in with_visual)


def _page_arm_boxes(frames, gap):
    """The description page's arm (common/viz_scene.py armSegments) in Python: the 8 corners of each ARM_BOXES box,
    in the arm frame, from the arm node's GET links frames [(pos, R)] (joint1..6, end_link, tcp) and the gap."""
    from robot_utils.nodes.arm.geometry import ARM_BOXES

    def box(p, R, b, sy):
        return np.array([p + R @ (np.array([b[1] + i * b[4] / 2, sy * (b[2] + j * b[5] / 2), b[3] + k * b[6] / 2]) / 1000)
                         for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    I = (np.zeros(3), np.eye(3))
    FR = {"base": I, "end": frames[6], **{f"link{i + 1}": frames[i] for i in range(6)}}
    out = [box(*FR[b[0]], b, 1) for b in ARM_BOXES if b[0] in FR]
    g = max(0.0, min(gap, 0.057))
    E = frames[6]
    for sgn in (1, -1):
        p = E[0] + E[1][:, 1] * sgn * g / 2
        out += [box(p, E[1], b, sgn) for b in ARM_BOXES if b[0] == "finger"]
    return out


def test_boxes_variant_draws_the_arm_as_the_page_does(node):
    """variant=boxes: the arm's link visuals are ARM_BOXES, so for random joints and gaps the tree's boxes (posed by
    tree.fk, in the arm frame) are the page's boxes on the arm node's own kinematics (ArmKinematics.frames, which
    GET links serves), one to one; the swerve wheels are boxes on the steer links, no module housings."""
    from robot_utils.nodes.arm.kin import ArmKinematics
    from robot_utils.nodes.description.tree import fk, tree
    a = node.model_builder().build(Options.parse("boxes", base="swerve"))
    t = tree(a.urdf, "v")
    kin = ArmKinematics()
    rng = np.random.default_rng(7)
    for _ in range(10):
        q = np.array([rng.uniform(kin.model.lowerPositionLimit[i], kin.model.upperPositionLimit[i]) for i in range(6)])
        gap = float(rng.uniform(0, 0.057))
        frames = [(np.array(T.translation), np.array(T.rotation)) for T in kin.frames(q)]
        page = _page_arm_boxes(frames, gap)
        qd = {f"arm_{n}": float(v) for n, v in zip(kin.joint_names, q)} | {"arm_finger_left": gap / 2}
        T = fk(t, qd)
        Tarm = np.linalg.inv(T["arm_base_link"])
        ours = []
        for link, lk in t["links"].items():
            if not link.startswith("arm_"):
                continue
            for v in lk["visuals"]:
                assert v["type"] == "box" and v["quat"] == [0.0, 0.0, 0.0, 1.0], (link, v)
                c, s = np.array(v["pos"]), np.array(v["size"])
                P = np.array([c + np.array([i, j, k]) * s / 2 for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
                M = Tarm @ T[link]
                ours.append(P @ M[:3, :3].T + M[:3, 3])
        assert len(ours) == len(page) == 15
        left = list(range(len(ours)))
        for B in page:                               # each page box is exactly one tree box (same 8 corners)
            hit = [i for i in left if all(np.min(np.linalg.norm(ours[i] - p, axis=1)) < 1e-9 for p in B)]
            assert len(hit) == 1, (B, hit)
            left.remove(hit[0])
    for mod in ("FL", "FR", "RR", "RL"):
        assert [v["type"] for v in t["links"][f"base_{mod}_module"]["visuals"]] == ["box"]
        assert t["links"][f"base_{mod}_wheel"]["visuals"] == []
    assert t["links"]["arm_link6"]["visuals"] == []


def test_boxes_variant_options(node):
    """boxes is accepted wherever variants are (whole robot or per component), by /model, /model/urdf and /model/tree;
    the options are in `options`, not in model_version (which follows the description, not the options)."""
    with TestClient(create_app(node, start=False)) as c:
        t = c.get("/model/tree", params={"variant": "boxes"})
        assert t.status_code == 200
        t = t.json()
        assert t["options"]["variant"] == "boxes" and t["options"]["base"] == "swerve"
        s = c.get("/model/tree").json()
        assert s["options"]["variant"] == "simple" and s["model_version"] == t["model_version"]
        assert set(s["links"]) == set(t["links"]) and s["joints"] == t["joints"] and s["movable"] == t["movable"]
        r = c.get("/model/urdf", params={"variant": "boxes"})
        assert r.status_code == 200 and "<mesh" not in r.text and 'name="arm_link2_upper_arm"' in r.text
        m = c.get("/model", params={"variant": "full,arm=boxes"}).json()
        assert {x["name"]: x["variant"] for x in m["components"]}["arm"] == "boxes"
        r = c.get("/model/urdf", params={"variant": "full,arm=boxes"}).text
        assert 'name="arm_link2_upper_arm"' in r and "<mesh" not in r     # the other components are procedural
        assert c.get("/model/urdf", params={"variant": "arm=box"}).status_code == 400
