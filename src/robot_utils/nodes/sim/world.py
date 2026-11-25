"""The MuJoCo sim world: the robot model from the description node plus what URDF lacks (docs/sim.md).

``World`` owns one ``MjModel`` / ``MjData`` under a lock. It adds to the URDF: a free joint on
``base_link``, the floor, actuators (module servos, the arm's kinematic servo, the fingers), cameras,
lights and scene objects. Every physics step it first runs the per-component controllers:

- **base** (the decided hybrid): per module, the steer angle and wheel speed references follow the
  commanded SPARK setpoints within the robot's own limits (limits.py: the SPARKs' output range, ramp and
  current limits from swerve/constants.py); a stiff position servo (steer) and a velocity servo (wheel),
  each torque-capped at what the duty / current limits allow, make the wheels turn; wheel-floor friction moves the free chassis. A
  disabled drive coasts (no torque); a disabled steer holds its angle. Soft wheel contact is the
  modules' compliance.
- **arm** (kinematic fidelity): the joint reference follows the MIT position setpoint within
  arm node's hard joint speed caps (a joint sent kp = 0 holds); a stiff servo plus a gravity / bias feedforward
  (``qfrc_bias`` as a joint torque, so the chassis carries the arm's weight) keeps the joints on it.
  The gripper's reference follows its goal at ``gripper_speed``; the fingers are force-capped.

Time: ``step(n)`` advances by hand (paused); ``run(rtf)`` locks the sim clock to the wall clock in a
thread. The world never touches a node: backends reach it through ``protocol`` sockets (node.py).
"""
from __future__ import annotations

import collections
import copy
import io
import logging
import math
import re
import threading
import time
import zipfile
from importlib import resources
from typing import Optional

import mujoco
import numpy as np
import yaml

log = logging.getLogger("sim.world")

GAP_PER_RAD = 0.016                  # m of jaw gap per rad of the gripper motor (arm/config.py: 2 x pitch radius)
GRIPPER_OPEN_RAD = -3.1              # fully open (arm/config.py); 0 = closed
FINGER_TRAVEL = 0.0285               # m, each finger (the arm URDF's finger_left limit)
ARM_JOINTS = [f"arm_joint{i}" for i in range(1, 7)]
MODULE_JOINT = re.compile(r"^base_(\w+)_steer$")
ROBOT_CONTYPE, ROBOT_CONAFFINITY = 2, 1      # the robot never collides with itself
WORLD_CONTYPE, WORLD_CONAFFINITY = 1, 3      # floor and objects collide with the robot and each other
INTEGRATORS = {"euler": mujoco.mjtIntegrator.mjINT_EULER, "rk4": mujoco.mjtIntegrator.mjINT_RK4,
               "implicit": mujoco.mjtIntegrator.mjINT_IMPLICIT, "implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST}
SHAPES = ("box", "cylinder", "articulated")
JOINT_TYPES = {"slide": mujoco.mjtJoint.mjJNT_SLIDE, "hinge": mujoco.mjtJoint.mjJNT_HINGE}
DRIVE_GAIN = 1000.0                  # N m s/rad (N s/m): an object joint drive's default velocity gain
IMU_BUFFER = 2000                    # samples kept per IMU (2 s at 1 kHz)


class SimError(ValueError):
    """A bad request to the world (HTTP 400 / an ``ok: false`` reply)."""


def _merge(a: dict, b: dict) -> dict:
    out = copy.deepcopy(a)
    for k, v in (b or {}).items():
        out[k] = _merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else copy.deepcopy(v)
    return out


def default_config() -> dict:
    return yaml.safe_load(resources.files(__package__).joinpath("default_sim.yaml").read_text())


def load_config(path: Optional[str] = None, overrides: Optional[dict] = None) -> dict:
    cfg = default_config()
    if path:
        with open(path) as f:
            cfg = _merge(cfg, yaml.safe_load(f) or {})
    return _merge(cfg, overrides or {})


def unzip_bundle(data: bytes) -> tuple[str, dict, dict]:
    """(URDF text, assets {path: bytes}, model.json) from GET /model/bundle.zip."""
    import json
    z = zipfile.ZipFile(io.BytesIO(data))
    assets = {n: z.read(n) for n in z.namelist() if n.startswith("meshes/")}
    info = json.loads(z.read("model.json")) if "model.json" in z.namelist() else {}
    return z.read("robot.urdf").decode(), assets, info


def yaw_quat(yaw: float) -> np.ndarray:
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def rpy_quat(rpy) -> np.ndarray:
    from ..description.frames import rpy_to_matrix
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, rpy_to_matrix(*rpy).reshape(-1))
    return q


def quat_rpy(q) -> list:
    from ..description.frames import matrix_to_rpy
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, float))
    return [float(v) for v in matrix_to_rpy(R.reshape(3, 3))]


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Module:
    """One swerve module's command and servo references (world side)."""

    def __init__(self, name: str):
        self.name = name
        self.steer_on, self.steer_goal = False, 0.0       # commanded module angle (rad, 0 = forward)
        self.drive_on, self.drive_goal = False, 0.0       # commanded wheel speed (m/s)
        self.steer_ref = 0.0                              # unwrapped steer reference (rad)
        self.speed_ref = 0.0                              # wheel speed reference (m/s)


class Arm:
    """The arm's motors as the world models them: enable state, the last MIT frames, references."""

    def __init__(self, q0):
        self.enabled = False
        self.g_enabled = False
        self.fault = 0                    # Damiao status code of the arm motors (0xD: CAN watchdog)
        self.cmd = None                   # (q, qd, kp, kd, tau) arrays of 6
        self.gcmd = None                  # (g_q, g_qd, g_kp, g_kd, g_tau)
        self.q_ref = np.array(q0, float)
        self.g_ref = 0.0
        self.last_frame = -math.inf       # sim time of the last MIT frame
        self.frames = 0


class World:
    def __init__(self, urdf: str, assets: dict, config: Optional[dict] = None, model_info: Optional[dict] = None):
        self.cfg = config or default_config()
        from .limits import refuse_overrides, robot_limits
        bad = refuse_overrides(self.cfg)
        if bad:
            raise SimError(f"the sim config sets {', '.join(bad)}: limits come from the robot config "
                           "(swerve/constants.py, arm/config.py; docs/sim.md)")
        self.limits = robot_limits()
        self.model_info = model_info or {}
        self.lock = threading.RLock()
        self.objects: dict = {}           # name -> spec dict (shape, size, mass, pose at add / reset)
        self.wrenches: list = []          # [body id, until (sim s), wrench 6]
        self.generation = 0               # bumps on every recompile (renderers rebuild)
        self.steps = 0
        self.imu: dict = {}               # camera -> deque of samples (seq, t_host, t_sim, ax, ay, az, gx, gy, gz)
        self.imu_seq = 0
        self.imu_cond = threading.Condition(threading.Lock())
        self.spec = self._build_spec(urdf, assets)
        for o in self.start_objects():
            self._spec_object(self._check_object(o))
        self.m = self.spec.compile()
        self.d = mujoco.MjData(self.m)
        self._index()
        self.modules = [Module(n) for n in self.module_names]
        self.arm = Arm(self.cfg["components"]["arm"]["start_q"])
        self.swerve_last = -math.inf
        tc = self.cfg.get("time") or {}
        self.rtf = float(tc.get("rtf", 1.0))
        self.running = False
        self.reset_seq = 0                 # resets since start (POST /scene/reset; not the initial one)
        self.reset(count=False)
        self._anchor = (0.0, 0.0)          # (wall, sim) when running started / re-anchored
        self.behind = 0                    # re-anchors because the physics could not keep up
        self._rate = (time.monotonic(), 0, 0.0, 0.0)     # (wall, steps, sim t) at the last rate sample, rate
        self.step_rate = 0.0
        self.rtf_measured = 0.0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # -- model -----------------------------------------------------------------------------------

    def _build_spec(self, urdf: str, assets: dict) -> mujoco.MjSpec:
        ext = ('<mujoco><compiler fusestatic="false" discardvisual="false" strippath="false" '
               'balanceinertia="true"/></mujoco>')
        urdf = re.sub(r"(<robot\b[^>]*>)", lambda mm: mm.group(1) + ext, urdf, count=1)
        spec = mujoco.MjSpec.from_string(urdf, assets=assets)
        ph = self.cfg["physics"]
        spec.option.timestep = float(ph["timestep"])
        spec.option.integrator = INTEGRATORS[ph.get("integrator", "implicitfast")]
        spec.option.gravity = [float(v) for v in ph["gravity"]]
        base = spec.body("base_link")
        if base is None:
            raise SimError("the model has no base_link")
        fj = base.add_freejoint()
        fj.name = "base_free"
        # collisions: the robot's geoms against the world only; visuals (contype 0) stay out
        bc = self.cfg["base"]
        wheel_bodies = set()
        for j in spec.joints:
            mm = MODULE_JOINT.match(j.name or "")
            if mm:
                j.armature = float(bc["steer"]["armature"])
                wheel_bodies.add(f"base_{mm.group(1)}_wheel")
            elif (j.name or "").startswith("base_") and j.name.endswith("_wheel_joint"):
                j.armature = float(bc["wheel"]["armature"])
        for g in spec.geoms:
            if g.contype or g.conaffinity:
                g.contype, g.conaffinity = ROBOT_CONTYPE, ROBOT_CONAFFINITY
                if g.parent is not None and g.parent.name in wheel_bodies:
                    wc = bc["wheel_contact"]
                    g.friction = [float(v) for v in wc["friction"]]
                    g.solref = [float(v) for v in wc["solref"]]
                    g.solimp = [float(v) for v in wc["solimp"]][:3] + [0.5, 2.0]
                    g.priority = 1
        self._jaws(spec)
        cc = self.cfg.get("contact") or {}
        spec.option.cone = {"pyramidal": mujoco.mjtCone.mjCONE_PYRAMIDAL,
                            "elliptic": mujoco.mjtCone.mjCONE_ELLIPTIC}[cc.get("cone", "elliptic")]
        spec.option.noslip_iterations = int(cc.get("noslip_iterations", 0))
        # floor, light
        fl = self.cfg["floor"]
        tex = spec.add_texture(name="floor_grid", type=mujoco.mjtTexture.mjTEXTURE_2D,
                               builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, rgb1=[0.34, 0.35, 0.37],
                               rgb2=[0.29, 0.30, 0.32], width=256, height=256)
        mat = spec.add_material(name="floor_grid")
        mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = tex.name
        mat.texrepeat = [float(fl["size"]) * 2, float(fl["size"]) * 2]
        floor = spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE,
                                        size=[float(fl["size"]), float(fl["size"]), 0.1], material="floor_grid")
        floor.contype, floor.conaffinity = WORLD_CONTYPE, WORLD_CONAFFINITY
        floor.friction = [float(v) for v in fl["friction"]]
        spec.worldbody.add_light(name="sun", pos=[0, 0, 4], dir=[0.2, 0.3, -1], type=int(mujoco.mjtLightType.mjLIGHT_DIRECTIONAL),
                                 diffuse=[0.7, 0.7, 0.7], specular=[0.1, 0.1, 0.1])
        # actuators
        for j in list(spec.joints):
            mm = MODULE_JOINT.match(j.name or "")
            if mm:
                s, w = bc["steer"], bc["wheel"]
                L = self.limits
                self._servo(spec, f"steer_{mm.group(1)}", j.name, float(s["kp"]), float(s["kv"]), L["steer_torque_max"])
                self._servo(spec, f"wheel_{mm.group(1)}", f"base_{mm.group(1)}_wheel_joint", 0.0, float(w["kv"]),
                            L["wheel_torque_max"], velocity=True)
        ac = self.cfg["components"]["arm"]
        if ac.get("fidelity", "kinematic") != "kinematic":
            raise SimError(f"arm fidelity {ac.get('fidelity')!r} is not built yet (kinematic only)")
        names = {j.name for j in spec.joints}
        if set(ARM_JOINTS) <= names:
            for jn in ARM_JOINTS:
                self._servo(spec, jn, jn, float(ac["servo"]["kp"]), float(ac["servo"]["kv"]), None)
            if "arm_finger_left" in names:
                self._servo(spec, "arm_finger", "arm_finger_left", 2000.0, 40.0, self.limits["finger_force_max"])
        # cameras: MuJoCo looks along -z with y up; the optical frame looks along +z with y down
        w_max, h_max = 640, 480
        for cam, cc in (self.cfg.get("cameras") or {}).items():
            for stream in ("color", "depth"):
                sc = cc[stream]
                body = spec.body(sc["link"])
                if body is None:
                    raise SimError(f"camera {cam}: the model has no link {sc['link']!r}")
                body.add_camera(name=f"{cam}_{stream}", pos=[0, 0, 0], quat=[0, 1, 0, 0], fovy=float(sc["fovy_deg"]))
                w_max, h_max = max(w_max, int(sc["width"])), max(h_max, int(sc["height"]))
        # IMUs (the head camera's accelerometer + gyro): sensors at a site on the configured link, IMU frame = the link's
        for cam, cc in (self.cfg.get("cameras") or {}).items():
            ic = cc.get("imu")
            if not ic:
                continue
            body = spec.body(ic["link"])
            if body is None:
                raise SimError(f"camera {cam}: the model has no IMU link {ic['link']!r}")
            body.add_site(name=f"{cam}_imu", pos=[0, 0, 0])
            for kind, typ in (("accel", mujoco.mjtSensor.mjSENS_ACCELEROMETER), ("gyro", mujoco.mjtSensor.mjSENS_GYRO)):
                spec.add_sensor(name=f"{cam}_{kind}", type=typ, objtype=mujoco.mjtObj.mjOBJ_SITE, objname=f"{cam}_imu")
        spec.visual.global_.offwidth, spec.visual.global_.offheight = w_max, h_max
        spec.visual.map.znear = 0.01          # x the model extent: near clip ~ cm (depth precision)
        return spec

    def _jaws(self, spec: mujoco.MjSpec) -> None:
        """The fingers collide as the real jaws, in every model variant: the jaw boxes (arm/geometry.py ARM_BOXES:
        the carriage and the prong, bounded from the visual meshes) replace whatever collision geometry the finger
        links have (the full variant's convex hull fills the space between the pads). Visual geoms stay. The
        mimic coupling of the right finger is stiffened (``contact.fingers.mimic_solref``): one pinion drives both."""
        from ..description.model import arm_box_geometry
        fc = (self.cfg.get("contact") or {}).get("fingers") or {}
        boxes = arm_box_geometry()
        for link in ("finger_left_link", "finger_right_link"):
            body = spec.body(f"arm_{link}")
            if body is None:
                continue
            for g in list(body.geoms):
                if g.contype or g.conaffinity:
                    spec.delete(g)
            for name, c, size in boxes.get(link, []):
                g = body.add_geom(name=f"arm_{link}_{name}_collision", type=mujoco.mjtGeom.mjGEOM_BOX,
                                  pos=c, size=[v / 2 for v in size], group=3)
                g.contype, g.conaffinity = ROBOT_CONTYPE, ROBOT_CONAFFINITY
                g.condim = int(fc.get("condim", 3))
                g.friction = [float(v) for v in fc.get("friction", [1.0, 0.005, 0.0001])]
                if fc.get("solref"):
                    g.solref = [float(v) for v in fc["solref"]]
                if fc.get("solimp"):
                    g.solimp = [float(v) for v in fc["solimp"]][:3] + [0.5, 2.0]
        if fc.get("mimic_solref"):
            for e in spec.equalities:
                e.solref = [float(v) for v in fc["mimic_solref"]]

    @staticmethod
    def _servo(spec, name: str, joint: str, kp: float, kv: float, force: Optional[float], velocity: bool = False):
        a = spec.add_actuator(name=name, target=joint, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        if velocity:                       # force = kv (ctrl - qvel)
            a.gainprm[0] = kv
            a.biasprm[:3] = [0.0, 0.0, -kv]
        else:                              # force = kp (ctrl - q) - kv qvel
            a.gainprm[0] = kp
            a.biasprm[:3] = [0.0, -kp, -kv]
        if force is not None:
            a.forcerange = [-force, force]
            a.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
        a.ctrllimited = mujoco.mjtLimited.mjLIMITED_FALSE
        return a

    def _index(self) -> None:
        """Addresses into qpos / qvel / ctrl after every (re)compile."""
        m = self.m
        self.base_body = m.body("base_link").id
        j = m.joint("base_free")
        self.base_q, self.base_v = int(j.qposadr[0]), int(j.dofadr[0])
        names = [m.joint(i).name for i in range(m.njnt)]
        self.module_names = [MODULE_JOINT.match(n).group(1) for n in names if MODULE_JOINT.match(n)]
        self.mod = []
        for n in self.module_names:
            s, w = m.joint(f"base_{n}_steer"), m.joint(f"base_{n}_wheel_joint")
            geom = next(g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body(f"base_{n}_wheel").id
                        and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER)
            self.mod.append({"sq": int(s.qposadr[0]), "sv": int(s.dofadr[0]), "wq": int(w.qposadr[0]),
                             "wv": int(w.dofadr[0]), "sa": m.actuator(f"steer_{n}").id,
                             "wa": m.actuator(f"wheel_{n}").id, "r": float(m.geom_size[geom][0])})
        self.has_arm = all(n in names for n in ARM_JOINTS)
        if self.has_arm:
            self.arm_q = np.array([m.joint(n).qposadr[0] for n in ARM_JOINTS])
            self.arm_v = np.array([m.joint(n).dofadr[0] for n in ARM_JOINTS])
            self.arm_a = np.array([m.actuator(n).id for n in ARM_JOINTS])
            fj = m.joint("arm_finger_left")
            self.finger_q, self.finger_v = int(fj.qposadr[0]), int(fj.dofadr[0])
            self.finger_a = m.actuator("arm_finger").id
        self.imu_adr = {}                   # camera -> (accel adr, gyro adr) in sensordata
        for cam in (self.cfg.get("cameras") or {}):
            try:
                self.imu_adr[cam] = (int(m.sensor(f"{cam}_accel").adr[0]), int(m.sensor(f"{cam}_gyro").adr[0]))
            except KeyError:
                pass
        self.obj_idx = {}
        self.obj_joints = {}                # articulated object -> {link: (qpos adr, dof adr)}
        self.obj_drives = []                # (actuator id, velocity) of every driven object joint
        for name, o in self.objects.items():
            if o["shape"] == "articulated":
                self.obj_joints[name] = {ln["name"]: (int(m.joint(f"obj_{name}/{ln['name']}").qposadr[0]),
                                                      int(m.joint(f"obj_{name}/{ln['name']}").dofadr[0]))
                                         for ln in o["links"] if ln.get("joint")}
                self.obj_drives += [(m.actuator(f"obj_{name}/{ln['name']}").id, ln["joint"]["drive"]["velocity"])
                                    for ln in o["links"] if (ln.get("joint") or {}).get("drive")]
            if o.get("fixed"):
                self.obj_idx[name] = (m.body(f"obj_{name}").id, None, None)
            else:
                jj = m.joint(f"obj_{name}_free")
                self.obj_idx[name] = (m.body(f"obj_{name}").id, int(jj.qposadr[0]), int(jj.dofadr[0]))

    def start_objects(self) -> list:
        """The scene at start: the default tabletop (scene.py) when ``scene.tabletop``, then ``scene.objects``."""
        sc = self.cfg.get("scene") or {}
        out = []
        if sc.get("tabletop"):
            from .scene import tabletop
            out += tabletop()
        if sc.get("cabinet"):
            from .scene import cabinet_scene
            out += cabinet_scene()
        if sc.get("rings"):
            from .scene import rings_scene
            out += rings_scene(**(sc.get("rings_setup") or {}))
        return out + list(sc.get("objects") or [])

    # -- objects -------------------------------------------------------------------------------------

    def _check_object(self, o: dict) -> dict:
        o = dict(o)
        name = str(o.get("name") or "")
        if not re.match(r"^[A-Za-z0-9_-]{1,40}$", name):
            raise SimError("an object needs a name of letters, digits, _ or - (up to 40)")
        shape = o.get("shape", "box")
        if shape not in SHAPES:
            raise SimError(f"shape must be one of {', '.join(SHAPES)}")
        if shape == "articulated":
            o["links"] = self._check_links(name, o.get("links"))
        else:
            o.pop("links", None)
            o.update(self._check_shape(o, shape))
        o["fixed"] = bool(o.get("fixed", False))
        o["mass"] = float(o.get("mass", 0.5))
        if not o["mass"] > 0:
            raise SimError("mass must be > 0 (kg)")
        if not o["fixed"]:
            o.pop("fixed")
        o["xyz"] = [float(v) for v in o.get("xyz", [0.5, 0.0, 0.2])]
        o["rpy"] = [float(v) for v in o.get("rpy", [0.0, 0.0, float(o.get("yaw", 0.0))])]
        o.pop("yaw", None)
        o["rgba"] = [float(v) for v in o.get("rgba", [0.85, 0.55, 0.2, 1.0])]
        oc = (self.cfg.get("contact") or {}).get("objects") or {}
        o["friction"] = [float(v) for v in o.get("friction", oc.get("friction", [0.8, 0.005, 0.0001]))]
        o["name"], o["shape"] = name, shape
        return o

    @staticmethod
    def _check_shape(g: dict, shape: str) -> dict:
        """A primitive's dimensions: box ``size`` (full extents), cylinder ``radius`` + ``length`` (along z)."""
        if shape == "box":
            size = [float(v) for v in g.get("size", [0.1, 0.1, 0.1])]
            if len(size) != 3 or min(size) <= 0:
                raise SimError("a box needs size [x, y, z] > 0 (full extents, m)")
            return {"size": size}
        if shape == "cylinder":
            r, n = float(g.get("radius", 0.05)), float(g.get("length", 0.1))
            if r <= 0 or n <= 0:
                raise SimError("a cylinder needs radius and length > 0 (m)")
            return {"radius": r, "length": n}
        raise SimError(f"a part's shape is box or cylinder, not {shape!r}")

    def _check_links(self, obj: str, links) -> list:
        """An articulated object's links: a tree of rigid parts, each a list of primitives, joined to its parent by
        a slide or hinge joint (or welded). The first link is the root (no parent, no joint): it is the object's
        frame, placed by the object's xyz / rpy (and fixed or free as the object)."""
        if not isinstance(links, list) or not links:
            raise SimError(f"articulated object {obj!r} needs links")
        out, names = [], set()
        for i, ln in enumerate(links):
            ln = dict(ln)
            nm = str(ln.get("name") or "")
            if not re.match(r"^[A-Za-z0-9_-]{1,40}$", nm) or nm in names:
                raise SimError(f"{obj}: link names are unique letters, digits, _ or - (got {nm!r})")
            parent = ln.get("parent")
            if i == 0:
                if parent is not None or ln.get("joint"):
                    raise SimError(f"{obj}: the first link is the root: no parent, no joint")
            elif parent not in names:
                raise SimError(f"{obj}/{nm}: parent must be an earlier link (got {parent!r})")
            j = ln.get("joint")
            if j is not None:
                j = j_in = dict(j)
                if j.get("type") not in JOINT_TYPES:
                    raise SimError(f"{obj}/{nm}: joint type is slide or hinge")
                axis = [float(v) for v in j.get("axis", [1, 0, 0])]
                if len(axis) != 3 or not np.linalg.norm(axis) > 0:
                    raise SimError(f"{obj}/{nm}: joint axis is a non-zero [x, y, z]")
                rng = j.get("range")
                if rng is not None:
                    rng = [float(v) for v in rng]
                    if len(rng) != 2 or not rng[0] < rng[1]:
                        raise SimError(f"{obj}/{nm}: joint range is [lo, hi], lo < hi (m or rad)")
                j = {"type": j["type"], "axis": axis, **({"range": rng} if rng else {}),
                     "pos": [float(v) for v in j.get("pos", [0, 0, 0])],
                     "damping": float(j.get("damping", 2.0)), "frictionloss": float(j.get("frictionloss", 0.0))}
                if j["damping"] < 0 or j["frictionloss"] < 0:
                    raise SimError(f"{obj}/{nm}: damping and frictionloss are >= 0")
                if j_in.get("drive") is not None:
                    j["drive"] = self._check_drive(f"{obj}/{nm}", j_in["drive"])
                if j_in.get("mimic") is not None:          # held at the position of an earlier link's joint
                    lead = next((x for x in out if x["name"] == j_in["mimic"]), None)
                    if lead is None or (lead.get("joint") or {}).get("type") != j["type"]:
                        raise SimError(f"{obj}/{nm}: mimic names an earlier link with a {j['type']} joint")
                    j["mimic"] = lead["name"]
            geoms = []
            for g in ln.get("geoms") or []:
                g = dict(g)
                gs = g.get("shape", "box")
                geoms.append({"shape": gs, **self._check_shape(g, gs),
                              "pos": [float(v) for v in g.get("pos", [0, 0, 0])],
                              "rpy": [float(v) for v in g.get("rpy", [0, 0, 0])],
                              "mass": float(g.get("mass", 0.2)),
                              **({"rgba": [float(v) for v in g["rgba"]]} if g.get("rgba") else {})})
                if not geoms[-1]["mass"] > 0:
                    raise SimError(f"{obj}/{nm}: a part's mass is > 0 (kg)")
            if not geoms:
                raise SimError(f"{obj}/{nm}: a link needs geoms")
            out.append({"name": nm, **({"parent": parent} if parent is not None else {}),
                        "pos": [float(v) for v in ln.get("pos", [0, 0, 0])],
                        "rpy": [float(v) for v in ln.get("rpy", [0, 0, 0])],
                        **({"joint": j} if j else {}), "geoms": geoms})
            names.add(nm)
        return out

    @staticmethod
    def _check_drive(where: str, dr) -> dict:
        """A joint's drive: a velocity servo (MuJoCo velocity actuator) holding ``velocity`` (rad/s or m/s) with up to
        ``max_torque`` (N m or N) at ``gain`` (N m s/rad or N s/m of torque per unit of speed error)."""
        if not isinstance(dr, dict):
            raise SimError(f"{where}: drive is {{velocity, max_torque, gain}}")
        try:
            out = {"velocity": float(dr["velocity"]), "max_torque": float(dr["max_torque"]),
                   "gain": float(dr.get("gain", DRIVE_GAIN))}
        except (KeyError, TypeError, ValueError):
            raise SimError(f"{where}: drive needs a numeric velocity and max_torque") from None
        if not math.isfinite(out["velocity"]) or not out["max_torque"] > 0 or not out["gain"] > 0 \
                or not math.isfinite(out["max_torque"] + out["gain"]):
            raise SimError(f"{where}: drive velocity is finite, max_torque and gain are > 0")
        return out

    def _add_part(self, body, g: dict, rgba) -> None:
        """One primitive of an object on ``body``, colliding with the robot and other objects."""
        if g["shape"] == "box":
            mg = body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[v / 2 for v in g["size"]], mass=g["mass"],
                               rgba=g.get("rgba", rgba))
        else:
            mg = body.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[g["radius"], g["length"] / 2, 0],
                               mass=g["mass"], rgba=g.get("rgba", rgba))
        if "pos" in g:
            mg.pos, mg.quat = g["pos"], rpy_quat(g["rpy"])
        mg.contype, mg.conaffinity, mg.friction = WORLD_CONTYPE, WORLD_CONAFFINITY, g["friction"]
        mg.condim = int(((self.cfg.get("contact") or {}).get("objects") or {}).get("condim", 3))

    def _spec_object(self, o: dict) -> None:
        if o["name"] in self.objects:
            raise SimError(f"object {o['name']!r} exists")
        b = self.spec.worldbody.add_body(name=f"obj_{o['name']}", pos=o["xyz"], quat=rpy_quat(o["rpy"]))
        if not o.get("fixed"):                       # fixed: welded to the world (furniture)
            fj = b.add_freejoint()
            fj.name = f"obj_{o['name']}_free"
        if o["shape"] != "articulated":
            self._add_part(b, {k: o[k] for k in ("shape", "size", "radius", "length", "mass", "friction") if k in o},
                           o["rgba"])
            self.objects[o["name"]] = o
            return
        bodies = {}
        for ln in o["links"]:
            if "parent" not in ln:                  # the root: the object's own body
                body = b
            else:
                body = bodies[ln["parent"]].add_body(name=f"obj_{o['name']}/{ln['name']}", pos=ln["pos"],
                                                     quat=rpy_quat(ln["rpy"]))
            if ln.get("joint"):
                j = ln["joint"]
                mj = body.add_joint(name=f"obj_{o['name']}/{ln['name']}", type=JOINT_TYPES[j["type"]],
                                    axis=j["axis"], pos=j["pos"], damping=j["damping"],
                                    frictionloss=j["frictionloss"])
                if "range" in j:
                    mj.limited, mj.range = mujoco.mjtLimited.mjLIMITED_TRUE, j["range"]
                if "drive" in j:                    # a velocity servo; its ctrl (the velocity) is set by _drive
                    self._servo(self.spec, mj.name, mj.name, 0.0, j["drive"]["gain"], j["drive"]["max_torque"],
                                velocity=True)
                if "mimic" in j:                    # q = q of the leader (a joint equality)
                    e = self.spec.add_equality(type=mujoco.mjtEq.mjEQ_JOINT, name=mj.name, name1=mj.name,
                                               name2=f"obj_{o['name']}/{j['mimic']}")
                    e.data[:5] = [0.0, 1.0, 0.0, 0.0, 0.0]
            for g in ln["geoms"]:
                self._add_part(body, {**g, "friction": o["friction"]}, o["rgba"])
            bodies[ln["name"]] = body
        self.objects[o["name"]] = o

    def _unspec(self, name: str) -> None:
        """Delete an object from the spec: its body tree, its joints' drives and mimics."""
        for a in list(self.spec.actuators) + list(self.spec.equalities):
            if (a.name or "").startswith(f"obj_{name}/"):
                self.spec.delete(a)
        self.spec.delete(self.spec.body(f"obj_{name}"))

    def _drive(self) -> None:
        """The driven object joints' velocity references (after a compile or a reset zeroes ctrl)."""
        for a, v in self.obj_drives:
            self.d.ctrl[a] = v

    def _recompile(self) -> None:
        self.m, self.d = self.spec.recompile(self.m, self.d)
        self._index()
        self._drive()
        mujoco.mj_forward(self.m, self.d)
        self.generation += 1

    def add_object(self, o: dict) -> dict:
        o = self._check_object(o)
        with self.lock:
            self._spec_object(o)
            try:
                self._recompile()
            except Exception:
                self._unspec(o["name"])
                del self.objects[o["name"]]
                raise
            self._place(o["name"], o["xyz"], rpy_quat(o["rpy"]))
            return self.object_state(o["name"])

    def remove_object(self, name: str) -> None:
        with self.lock:
            if name not in self.objects:
                raise KeyError(f"no object {name!r}")
            self._unspec(name)
            del self.objects[name]
            self.wrenches = []
            self._recompile()

    def _place(self, name: str, xyz, quat) -> None:
        b, qa, va = self.obj_idx[name]
        if qa is None:                                # fixed: move the body itself (model and spec, for recompiles)
            self.m.body_pos[b], self.m.body_quat[b] = xyz, quat
            sb = self.spec.body(f"obj_{name}")
            sb.pos, sb.quat = list(xyz), list(quat)
            mujoco.mj_forward(self.m, self.d)
            return
        self.d.qpos[qa:qa + 3] = xyz
        self.d.qpos[qa + 3:qa + 7] = quat
        self.d.qvel[va:va + 6] = 0.0
        mujoco.mj_forward(self.m, self.d)

    def move_object(self, name: str, xyz, rpy=None, yaw=None) -> dict:
        """Teleport an object (privileged): its pose set, its velocity zeroed."""
        with self.lock:
            if name not in self.objects:
                raise KeyError(f"no object {name!r}")
            rpy = rpy if rpy is not None else [0.0, 0.0, float(yaw or 0.0)]
            self._place(name, [float(v) for v in xyz], rpy_quat(rpy))
            return self.object_state(name)

    def object_state(self, name: str) -> dict:
        o = self.objects[name]
        b, qa, va = self.obj_idx[name]
        out = {k: o[k] for k in ("name", "shape", "size", "radius", "length", "mass", "fixed", "rgba", "links")
               if k in o}
        if name in self.obj_joints:
            out["joints"] = {ln: round(float(self.d.qpos[qa_]), 6) for ln, (qa_, _) in self.obj_joints[name].items()}
        if qa is None:
            out["xyz"] = [round(float(v), 6) for v in self.d.xpos[b]]
            out["rpy"] = [round(v, 6) for v in quat_rpy(self.d.xquat[b])]
            out["speed"] = 0.0
            return out
        out["xyz"] = [round(float(v), 6) for v in self.d.qpos[qa:qa + 3]]
        out["rpy"] = [round(v, 6) for v in quat_rpy(self.d.qpos[qa + 3:qa + 7])]
        out["speed"] = round(float(np.linalg.norm(self.d.qvel[va:va + 3])), 6)
        return out

    def scene(self) -> dict:
        with self.lock:
            return {"objects": [self.object_state(n) for n in self.objects]}

    def load_scene(self, objects: list) -> dict:
        """Replace every object (one recompile)."""
        new = [self._check_object(o) for o in objects]
        if len({o["name"] for o in new}) != len(new):
            raise SimError("object names must be unique")
        with self.lock:
            for n in list(self.objects):
                self._unspec(n)
            self.objects = {}
            for o in new:
                self._spec_object(o)
            self.wrenches = []
            self._recompile()
            for o in new:
                self._place(o["name"], o["xyz"], rpy_quat(o["rpy"]))
            return self.scene()

    # -- state reset -----------------------------------------------------------------------------------

    def reset(self, start_scene: bool = False, count: bool = True) -> int:
        """Time 0, the robot at its start pose, objects where they were added, every command dropped.
        ``start_scene``: the scene the world started with (the default tabletop + config objects) instead.
        Every reset increments ``reset_seq`` (in /state and every robot_state.v1 record: the headset re-anchors its
        AR scene on a change; ``count=False`` only for the world's own initial reset); returns it."""
        if start_scene:
            self.load_scene(self.start_objects())
        with self.lock:
            m, d = self.m, self.d
            mujoco.mj_resetData(m, d)
            self._drive()
            st = self.cfg["base"]["start"]
            d.qpos[self.base_q:self.base_q + 3] = [float(v) for v in st["xyz"]]
            d.qpos[self.base_q + 3:self.base_q + 7] = yaw_quat(float(st.get("yaw", 0.0)))
            for i, mod in enumerate(self.modules):
                mod.__init__(mod.name)
            if self.has_arm:
                self.arm = Arm(self.cfg["components"]["arm"]["start_q"])
                d.qpos[self.arm_q] = self.arm.q_ref
            for name, o in self.objects.items():
                b, qa, _ = self.obj_idx[name]
                if qa is None:
                    m.body_pos[b], m.body_quat[b] = o["xyz"], rpy_quat(o["rpy"])
                    sb = self.spec.body(f"obj_{name}")
                    sb.pos, sb.quat = list(o["xyz"]), list(rpy_quat(o["rpy"]))
                    continue
                d.qpos[qa:qa + 3] = o["xyz"]
                d.qpos[qa + 3:qa + 7] = rpy_quat(o["rpy"])
            self.wrenches = []
            mujoco.mj_forward(m, d)
            self._settle()
            if count:
                self.reset_seq += 1
            return self.reset_seq

    def _settle(self) -> None:
        """Let the base sink onto its soft contacts (0.3 s of physics) and restart the clock at 0."""
        for _ in range(int(0.3 / self.m.opt.timestep)):
            self._control()
            mujoco.mj_step(self.m, self.d)
        self.d.time = 0.0
        if self.running:
            self._anchor = (time.monotonic(), 0.0)

    # -- control (every physics step, lock held) -----------------------------------------------------

    def _control(self) -> None:
        m, d = self.m, self.d
        dt = m.opt.timestep
        bc = self.cfg["base"]
        if d.time - self.swerve_last > 0.1:          # no setpoints: the SPARKs' heartbeat is gone
            for mod in self.modules:
                mod.steer_on = mod.drive_on = False
        L = self.limits
        rate, vmax, amax = L["steer_rate_max"], L["wheel_speed_max"], L["wheel_accel_max"]
        for mod, ix in zip(self.modules, self.mod):
            if mod.steer_on:
                err = wrap(mod.steer_goal - wrap(mod.steer_ref))
                mod.steer_ref += max(-rate * dt, min(rate * dt, err))
            d.ctrl[ix["sa"]] = mod.steer_ref
            r = ix["r"]
            if mod.drive_on:
                goal = max(-vmax, min(vmax, mod.drive_goal))
                mod.speed_ref += max(-amax * dt, min(amax * dt, goal - mod.speed_ref))
                m.actuator_gainprm[ix["wa"], 0] = bc["wheel"]["kv"]
                m.actuator_biasprm[ix["wa"], 2] = -bc["wheel"]["kv"]
                d.ctrl[ix["wa"]] = mod.speed_ref / r
            else:                                     # coasting: no torque, the reference follows the wheel
                mod.speed_ref = float(d.qvel[ix["wv"]]) * r
                m.actuator_gainprm[ix["wa"], 0] = 0.0
                m.actuator_biasprm[ix["wa"], 2] = 0.0
        for w in self.wrenches:
            if d.time >= w[1]:
                d.xfrc_applied[w[0]] = 0.0
        self.wrenches = [w for w in self.wrenches if d.time < w[1]]
        for w in self.wrenches:
            d.xfrc_applied[w[0]] = w[2]
        if self.has_arm:
            self._arm_control(dt)

    def _arm_control(self, dt: float) -> None:
        d, a, ac = self.d, self.arm, self.cfg["components"]["arm"]
        if a.enabled and d.time - a.last_frame > float(ac["watchdog"]):
            a.enabled, a.g_enabled, a.fault = False, False, 0xD
        if a.enabled and a.cmd is not None:
            q, _, kp, _, _ = a.cmd
            target = np.where(kp > 0, q, a.q_ref)     # kp = 0 (floating): hold where it is
            vmax = self.limits["joint_speed_max"] * dt
            a.q_ref = a.q_ref + np.clip(target - a.q_ref, -vmax, vmax)
        d.ctrl[self.arm_a] = a.q_ref
        d.qfrc_applied[self.arm_v] = d.qfrc_bias[self.arm_v]      # gravity / bias feedforward
        if a.enabled and a.g_enabled and a.gcmd is not None:
            g_q, _, g_kp, _, g_tau = a.gcmd
            goal = g_q if g_kp > 0 else (0.0 if g_tau > 0 else GRIPPER_OPEN_RAD if g_tau < 0 else a.g_ref)
            gs = float(ac["gripper_speed"]) * dt
            a.g_ref = min(0.0, max(GRIPPER_OPEN_RAD, a.g_ref + max(-gs, min(gs, goal - a.g_ref))))
        d.ctrl[self.finger_a] = min(FINGER_TRAVEL, max(0.0, -a.g_ref * GAP_PER_RAD / 2))

    # -- time ------------------------------------------------------------------------------------------

    def step(self, n: int = 1) -> float:
        """Advance ``n`` physics steps (any time; meant for paused / stepped mode). Returns the sim time."""
        with self.lock:
            for _ in range(int(n)):
                self._control()
                mujoco.mj_step(self.m, self.d)
                self._sample_imu(time.monotonic())
            self.steps += int(n)
            return float(self.d.time)

    def step_for(self, seconds: float) -> float:
        return self.step(max(1, int(round(seconds / self.m.opt.timestep))))

    @property
    def t(self) -> float:
        return float(self.d.time)

    def run(self, rtf: Optional[float] = None) -> None:
        if rtf is not None:
            if not 0 < float(rtf) <= 100:
                raise SimError("rtf must be within (0, 100]")
            self.rtf = float(rtf)
        with self.lock:
            self._anchor = (time.monotonic(), float(self.d.time))
            self.running = True
        if self._thread is None or not self._thread.is_alive():
            self._stop.clear()
            self._thread = threading.Thread(target=self._loop, name="sim-physics", daemon=True)
            self._thread.start()

    def pause(self) -> None:
        with self.lock:                                # the physics thread finishes its batch first
            self.running = False

    def close(self) -> None:
        self.running = False
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)

    def _loop(self) -> None:
        from ..common import realtime
        self.realtime = realtime.make_realtime(realtime.PRIO_ESTIMATE, "sim physics loop")   # macOS: no App Nap
        dt = self.m.opt.timestep
        while not self._stop.is_set():
            if not self.running:
                time.sleep(0.005)
                continue
            now = time.monotonic()
            with self.lock:
                if not self.running:                   # paused while this thread waited for the lock
                    continue
                w0, s0 = self._anchor
                behind = s0 + (now - w0) * self.rtf - float(self.d.time)
                n = int(behind / dt)
                if behind > 0.1 * self.rtf:            # the physics cannot keep up: drop the backlog
                    self.behind += 1
                    self._anchor = (now, float(self.d.time))
                    n = int(0.01 / dt)
                w0, s0 = self._anchor
                for _ in range(min(n, 20)):
                    self._control()
                    mujoco.mj_step(self.m, self.d)
                    self._sample_imu(w0 + (float(self.d.time) - s0) / self.rtf)    # the host time it stands for
                self.steps += min(n, 20)
            self._measure(now)
            if n < 20:
                time.sleep(0.0005)

    def _measure(self, now: float) -> None:
        w, s, t, _ = self._rate
        if now - w >= 1.0:
            self.step_rate = (self.steps - s) / (now - w)
            self.rtf_measured = (float(self.d.time) - t) / (now - w)
            self._rate = (now, self.steps, float(self.d.time), self.step_rate)

    # -- IMU ---------------------------------------------------------------------------------------------

    def _sample_imu(self, t_host: float) -> None:
        """After a physics step (lock held): one sample per IMU (MuJoCo's accelerometer: specific force, so
        +9.81 m/s^2 up at rest; gyro: angular velocity; both in the IMU link's frame)."""
        if not self.imu_adr:
            return
        sd = self.d.sensordata
        t = float(self.d.time)
        self.imu_seq += 1
        for cam, (a, g) in self.imu_adr.items():
            buf = self.imu.get(cam)
            if buf is None:
                buf = self.imu[cam] = collections.deque(maxlen=IMU_BUFFER)
            buf.append((self.imu_seq, t_host, t, float(sd[a]), float(sd[a + 1]), float(sd[a + 2]),
                        float(sd[g]), float(sd[g + 1]), float(sd[g + 2])))
        if self.imu_seq % 5 == 0:
            with self.imu_cond:
                self.imu_cond.notify_all()

    def imu_samples(self, cam: str, after: int = 0, wait: float = 0.0) -> list:
        """Samples of ``cam``'s IMU with seq > ``after`` (oldest first), waiting up to ``wait`` s for one."""
        if cam not in self.imu_adr:
            raise SimError(f"camera {cam!r} has no IMU in the sim config")
        end = time.monotonic() + wait
        while True:
            with self.lock:
                buf = self.imu.get(cam) or ()
                out = [s for s in buf if s[0] > after] if buf and buf[-1][0] > after else []
            if out or time.monotonic() >= end:
                return out
            with self.imu_cond:
                self.imu_cond.wait(max(0.0, min(0.05, end - time.monotonic())))

    # -- arm backend -------------------------------------------------------------------------------------

    def arm_op(self, op: str, a: dict) -> dict:
        if not self.has_arm:
            raise SimError("the model has no arm")
        with self.lock:
            arm = self.arm
            if op == "hello":
                return {"joints": ARM_JOINTS, "fidelity": "kinematic",
                        "reply_latency": float(self.cfg["components"]["arm"]["reply_latency"])}
            if op == "enable":
                arm.enabled = arm.g_enabled = True
                arm.fault = 0
                arm.q_ref = self.d.qpos[self.arm_q].copy()
                arm.last_frame = float(self.d.time)
                arm.cmd = arm.gcmd = None
            elif op == "disable":
                arm.enabled = arm.g_enabled = False
                arm.cmd = arm.gcmd = None
            elif op == "gripper_motor":
                arm.g_enabled = bool(a["on"]) and arm.enabled
                if not arm.g_enabled:
                    arm.gcmd = None
            elif op == "check_gripper":
                arm.g_ref = 0.0
                self.d.qpos[self.finger_q] = 0.0
                self.d.ctrl[self.finger_a] = 0.0
            elif op == "send":
                if arm.enabled:
                    arm.cmd = tuple(np.asarray(a[k], float) for k in ("q", "qd", "kp", "kd", "tau"))
                    if a.get("gripper"):
                        arm.gcmd = tuple(float(a[k]) for k in ("g_q", "g_qd", "g_kp", "g_kd", "g_tau"))
                    arm.last_frame = float(self.d.time)
                    arm.frames += 1
                return {"t": float(self.d.time)}
            elif op != "read":
                raise SimError(f"unknown arm op {op!r}")
            return self._arm_reading()

    def _arm_reading(self) -> dict:
        d, arm = self.d, self.arm
        tau = d.actuator_force[self.arm_a] + d.qfrc_applied[self.arm_v]
        f = float(d.qpos[self.finger_q])
        status = [arm.fault or (1 if arm.enabled else 0)] * 6 + [1 if arm.g_enabled else 0]
        return {"t": float(d.time), "q": d.qpos[self.arm_q].tolist(), "qd": d.qvel[self.arm_v].tolist(),
                "tau": tau.tolist(), "status": status, "g_q": -2 * f / GAP_PER_RAD,
                "g_qd": -2 * float(d.qvel[self.finger_v]) / GAP_PER_RAD,
                "g_tau": float(d.actuator_force[self.finger_a]) * GAP_PER_RAD / 2, "q_ref": arm.q_ref.tolist(),
                "g_ref": arm.g_ref}

    def arm_lost(self) -> None:
        """The arm backend's connection closed: like a lost bus, the motors stop being commanded."""
        with self.lock:
            if self.has_arm and self.arm.enabled:
                self.arm.enabled = self.arm.g_enabled = False
                self.arm.fault = 0xD

    # -- swerve backend --------------------------------------------------------------------------------

    def swerve_op(self, op: str, a: dict) -> dict:
        with self.lock:
            if op == "hello":
                return {"modules": self.module_names, "wheel_radius": [ix["r"] for ix in self.mod]}
            if op != "modules":
                raise SimError(f"unknown swerve op {op!r}")
            byname = {m.name: m for m in self.modules}
            for name, (on, angle) in (a.get("steer") or {}).items():
                mod = byname[name]
                mod.steer_on, mod.steer_goal = bool(on), float(angle)
            for name, (on, speed) in (a.get("drive") or {}).items():
                mod = byname[name]
                mod.drive_on, mod.drive_goal = bool(on), float(speed)
            self.swerve_last = float(self.d.time)
            return {"t": float(self.d.time), **self.module_states()}

    def module_states(self) -> dict:
        d = self.d
        out = {}
        for mod, ix in zip(self.modules, self.mod):
            out[mod.name] = {"steer": wrap(float(d.qpos[ix["sq"]])), "steer_rate": float(d.qvel[ix["sv"]]),
                             "wheel": float(d.qpos[ix["wq"]]), "speed": float(d.qvel[ix["wv"]]) * ix["r"]}
        return {"modules": out}

    # -- privileged physics ------------------------------------------------------------------------------

    def apply_wrench(self, body: str, force, torque, duration: float) -> dict:
        """Add a wrench (world frame, at the body's centre of mass) for ``duration`` s of sim time."""
        with self.lock:
            name = f"obj_{body}" if body in self.objects else ("base_link" if body == "base" else body)
            try:
                bid = self.m.body(name).id
            except KeyError:
                raise KeyError(f"no body {body!r}") from None
            w = np.concatenate([np.asarray(force, float), np.asarray(torque, float)])
            self.wrenches.append([bid, float(self.d.time) + float(duration), w])
            return {"body": name, "until": float(self.d.time) + float(duration)}

    def teleport_base(self, xyz=None, yaw: Optional[float] = None, rpy=None) -> dict:
        with self.lock:
            q = self.d.qpos
            if xyz is not None:
                q[self.base_q:self.base_q + 3] = [float(v) for v in xyz]
            if rpy is not None:
                q[self.base_q + 3:self.base_q + 7] = rpy_quat(rpy)
            elif yaw is not None:
                q[self.base_q + 3:self.base_q + 7] = yaw_quat(float(yaw))
            self.d.qvel[self.base_v:self.base_v + 6] = 0.0
            mujoco.mj_forward(self.m, self.d)
            return self.base_state()

    # -- state ---------------------------------------------------------------------------------------

    def base_state(self) -> dict:
        d = self.d
        q = d.qpos[self.base_q:self.base_q + 7]
        v = d.qvel[self.base_v:self.base_v + 6]      # world-frame linear velocity, body-frame angular
        rpy = quat_rpy(q[3:])
        c, s = math.cos(rpy[2]), math.sin(rpy[2])
        return {"xyz": [round(float(x), 6) for x in q[:3]], "rpy": [round(x, 6) for x in rpy],
                "twist": [round(float(c * v[0] + s * v[1]), 6), round(float(-s * v[0] + c * v[1]), 6), round(float(v[5]), 6)]}

    def snapshot(self) -> dict:
        with self.lock:
            out = {"t": round(float(self.d.time), 6), "running": self.running, "rtf": self.rtf,
                   "reset_seq": self.reset_seq,
                   "rtf_measured": round(self.rtf_measured, 3), "step_rate": round(self.step_rate, 1),
                   "steps": self.steps, "behind": self.behind, "base": self.base_state(),
                   "modules": [{"name": mod.name, "steer_on": mod.steer_on, "drive_on": mod.drive_on,
                                "steer_goal": round(mod.steer_goal, 6), "speed_goal": round(mod.drive_goal, 6),
                                **{k: round(v, 6) for k, v in self.module_states()["modules"][mod.name].items()}}
                               for mod in self.modules],
                   "objects": [self.object_state(n) for n in self.objects]}
            if self.has_arm:
                a = self.arm
                out["arm"] = {"enabled": a.enabled, "gripper_enabled": a.g_enabled, "fault": a.fault,
                              "q": [round(float(x), 6) for x in self.d.qpos[self.arm_q]],
                              "q_ref": [round(float(x), 6) for x in a.q_ref],
                              "gap_mm": round(2 * float(self.d.qpos[self.finger_q]) * 1000, 3), "frames": a.frames}
            return out
