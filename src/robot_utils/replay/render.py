"""The operator's view of a rigid scene, rendered offscreen with MuJoCo (no other renderer needed).

``SceneModel`` turns one rigid scene (docs/scene3d.md: bodies, geoms, meshes) into an ``MjSpec``: every dynamic body
a mocap body (its pose and colour override set per frame), every static one fixed at its pose, the geoms the
headset draws (``scene_view.h``: role ``auto`` = the robot's and the ghosts' visual geoms, everything else's
collision geoms; planes left out), plus two controller-like shapes on mocap bodies and a light rig. Meshes are
given one vertex per triangle corner with the face normal, so they shade flat. The light is the headset's flat
style (``mesh_renderer.h``): ``0.45 + 0.55 |n . L|`` with ``L`` fixed in LOCAL, built from two opposite directional
lights on a mocap body turned with the placement.

``ViewRenderer`` draws one frame from an eye: the camera is the eye pose mapped into the sim world through the
placement (``ar.SimInLocal.camera``), with the eye's own asymmetric frustum (OpenXR FOV tangents, near 0.04 m as
on the headset). The controllers are drawn on top of the scene by default (a second pass, composited where it
covers), or depth-tested in the scene (``controllers="depth"``).
"""
from __future__ import annotations

import math
from typing import Optional

import mujoco
import numpy as np

from robot_utils.scene3d.format import Frame, mesh_arrays

from . import ar

L_LOCAL = np.array([0.3, 0.9, 0.4]) / np.linalg.norm([0.3, 0.9, 0.4])   # mesh_renderer.h's light, LOCAL axes
AMBIENT, DIFFUSE = 0.45, 0.55
NEAR_M, FAR_M = 0.04, 40.0                                               # demo_renderer.h
SCENE_GROUP, CONTROLLER_GROUP = 0, 3
CONTROLLER_RGBA = {"left": (0.45, 0.72, 0.95, 1.0), "right": (0.95, 0.64, 0.35, 1.0)}
DEFAULT_BG = (0.12, 0.12, 0.13)


def _wxyz(q_xyzw) -> list:
    x, y, z, w = (float(v) for v in q_xyzw)
    return [w, x, y, z]


def _quat_of(r: np.ndarray) -> np.ndarray:
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(r, float).reshape(-1))
    return q


def _wants(geom_role: str, want: str) -> bool:
    return geom_role == want or geom_role == "both"


def select_geoms(body: dict, role: str) -> list[dict]:
    """The geoms of a body the headset draws (``SceneView::rebuild``): planes never."""
    want = role if role in ("visual", "collision") else ("visual" if body.get("kind") in ("robot", "ghost")
                                                          else "collision")
    geoms = body.get("geoms") or []
    have = any(_wants(g.get("role", "both"), want) for g in geoms)
    return [g for g in geoms if g.get("type") != "plane" and (not have or _wants(g.get("role", "both"), want))]


def _controller_geoms(body, scale: float, rgba) -> None:
    """A Touch-like controller in its grip frame (OpenXR: -Z forward out of the fist, +Y up): a handle along the
    fist's axis and a head with the buttons above and ahead of it. Sizes in metres of the operator's world (divided
    by the AR scale: the controllers live in the sim world's units here)."""
    s = 1.0 / scale
    kw = dict(contype=0, conaffinity=0, group=CONTROLLER_GROUP, rgba=list(rgba), material="flat")
    tilt = _quat_of(np.array([[1, 0, 0], [0, math.cos(0.35), -math.sin(0.35)], [0, math.sin(0.35), math.cos(0.35)]]))
    body.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.018 * s, 0.04 * s, 0], pos=[0, -0.005 * s, 0.015 * s],
                  quat=tilt, **kw)
    body.add_geom(type=mujoco.mjtGeom.mjGEOM_ELLIPSOID, size=[0.034 * s, 0.018 * s, 0.036 * s],
                  pos=[0, 0.022 * s, -0.045 * s], **kw)
    body.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.006 * s, 0.004 * s, 0], pos=[0, 0.04 * s, -0.04 * s],
                  quat=_quat_of(np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])),
                  **{**kw, "rgba": [0.15, 0.15, 0.15, 1.0]})


class SceneModel:
    """One scene (``index``, ``blob``) as a MuJoCo model, for ``scale`` (the controllers' size) and an image of at
    most ``width`` x ``height``."""

    def __init__(self, index: dict, blob: bytes, *, role: str = "auto", scale: float = 1.0,
                 bg=DEFAULT_BG, width: int = 960, height: int = 960):
        self.index, self.role, self.scale = index, role, scale
        spec = mujoco.MjSpec()
        spec.modelname = f"scene_{index.get('scene_id', '')}"
        spec.compiler.inertiafromgeom = mujoco.mjtInertiaFromGeom.mjINERTIAFROMGEOM_FALSE
        spec.visual.headlight.active = 0
        spec.visual.global_.offwidth = max(int(width), 64)
        spec.visual.global_.offheight = max(int(height), 64)
        spec.add_texture(name="bg", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_FLAT,
                         rgb1=list(bg), rgb2=list(bg), width=8, height=8)
        mat = spec.add_material(name="flat")
        mat.specular, mat.shininess, mat.reflectance = 0.0, 0.0, 0.0
        mat.rgba = [0.5, 0.5, 0.5, 1.0]
        rig = spec.worldbody.add_body(name="light_rig", mocap=True, mass=1.0, inertia=[1e-3] * 3)
        for sgn in (1.0, -1.0):
            rig.add_light(type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, dir=list(sgn * L_LOCAL),
                          ambient=[AMBIENT / 2] * 3, diffuse=[DIFFUSE] * 3, specular=[0, 0, 0], castshadow=False)
        meshes: dict[int, str] = {}
        self.body_geoms: dict[int, list[str]] = {}      # scene body id -> its MuJoCo geom names
        self.mocap_body: dict[int, str] = {}            # dynamic scene body id -> MuJoCo body name
        self.skipped = 0
        dynamic = set(index.get("dynamic", []))
        for b in index.get("bodies", []):
            geoms = select_geoms(b, role)
            name = f"b{b['id']}"
            pose = b.get("pose") or [0, 0, 0, 0, 0, 0, 1]
            if b["id"] in dynamic:
                body = spec.worldbody.add_body(name=name, mocap=True, pos=pose[:3], quat=_wxyz(pose[3:]),
                                               mass=1.0, inertia=[1e-3] * 3)
                self.mocap_body[b["id"]] = name
            else:
                body = spec.worldbody.add_body(name=name, pos=pose[:3], quat=_wxyz(pose[3:]), mass=1.0,
                                               inertia=[1e-3] * 3)
            names = []
            for k, g in enumerate(geoms):
                kw = dict(name=f"{name}_g{k}", pos=list(g.get("pos", (0, 0, 0))), quat=_wxyz(g.get("quat", (0, 0, 0, 1))),
                          rgba=list(g.get("rgba", (0.7, 0.7, 0.7, 1.0))), contype=0, conaffinity=0, group=SCENE_GROUP,
                          material="flat")
                t = g["type"]
                if t == "mesh":
                    mi = int(g["mesh"])
                    if mi not in meshes:
                        meshes[mi] = self._add_mesh(spec, index, blob, mi)
                    if meshes[mi] is None:
                        self.skipped += 1
                        continue
                    body.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=meshes[mi], **kw)
                elif t == "box":
                    body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=list(g["half_extents"]), **kw)
                elif t == "sphere":
                    body.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[g["radius"], 0, 0], **kw)
                elif t in ("capsule", "cylinder"):
                    gt = mujoco.mjtGeom.mjGEOM_CAPSULE if t == "capsule" else mujoco.mjtGeom.mjGEOM_CYLINDER
                    body.add_geom(type=gt, size=[g["radius"], g["half_length"], 0], **kw)
                elif t == "ellipsoid":
                    body.add_geom(type=mujoco.mjtGeom.mjGEOM_ELLIPSOID, size=list(g["radii"]), **kw)
                else:
                    self.skipped += 1
                    continue
                names.append(kw["name"])
            self.body_geoms[b["id"]] = names
        for side in ("left", "right"):
            _controller_geoms(spec.worldbody.add_body(name=f"controller_{side}", mocap=True, mass=1.0,
                                                      inertia=[1e-3] * 3), scale, CONTROLLER_RGBA[side])
        self.m = spec.compile()
        self.d = mujoco.MjData(self.m)
        m = self.m
        gid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)          # noqa: E731
        mid = lambda n: int(m.body_mocapid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)])   # noqa: E731
        self.geom_ids = {b: np.array([gid(n) for n in ns], dtype=int) for b, ns in self.body_geoms.items()}
        self.slot_mocap = np.array([mid(self.mocap_body[b]) if b in self.mocap_body else -1
                                    for b in index.get("dynamic", [])], dtype=int)
        self.slot_geoms = [self.geom_ids.get(b, np.zeros(0, int)) for b in index.get("dynamic", [])]
        self.rig = mid("light_rig")
        self.controller = {s: mid(f"controller_{s}") for s in ("left", "right")}
        self.controller_geoms = {s: np.array([i for i in range(m.ngeom)
                                              if m.geom_bodyid[i] == mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY,
                                                                                       f"controller_{s}")])
                                 for s in ("left", "right")}
        self.base_rgba = m.geom_rgba.copy()

    @staticmethod
    def _add_mesh(spec, index: dict, blob: bytes, i: int) -> Optional[str]:
        v, t, _ = mesh_arrays(index, blob, i)
        if len(t) == 0:
            return None
        tri = v[t.astype(np.int64)].astype(float)                    # (n, 3, 3): one vertex per corner
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        area = np.linalg.norm(n, axis=1)
        keep = area > 1e-14
        if not keep.any():
            return None
        tri, n = tri[keep], n[keep] / area[keep, None]
        name = f"mesh{i}"
        me = spec.add_mesh(name=name)
        me.uservert = tri.reshape(-1).tolist()
        me.userface = list(range(3 * len(tri)))
        me.usernormal = n.reshape(-1).tolist()
        me.userfacenormal = np.repeat(np.arange(len(n)), 3).tolist()
        me.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL
        return name

    def apply(self, frame: Optional[Frame], placement: ar.SimInLocal, hands: dict) -> None:
        """The frame's dynamic poses and colour overrides, the controllers (grip poses in LOCAL, mapped into the
        sim world; hidden when untracked) and the light rig (LOCAL's axes in the sim world)."""
        m, d = self.m, self.d
        m.geom_rgba[:] = self.base_rgba
        if frame is not None:
            p = np.asarray(frame.poses, float)
            n = min(len(p), len(self.slot_mocap))
            ok = self.slot_mocap[:n] >= 0
            ids = self.slot_mocap[:n][ok]
            d.mocap_pos[ids] = p[:n][ok, :3]
            d.mocap_quat[ids] = p[:n][ok][:, [6, 3, 4, 5]]
            for slot, rgba in frame.colors.items():
                if slot < len(self.slot_geoms) and len(self.slot_geoms[slot]):
                    m.geom_rgba[self.slot_geoms[slot]] = rgba
        for side, mid in self.controller.items():
            h = hands.get(side)
            if h is None:
                m.geom_rgba[self.controller_geoms[side], 3] = 0.0
                continue
            cp, cr = placement.camera(*h)
            d.mocap_pos[mid] = cp
            d.mocap_quat[mid] = _quat_of(cr)
        d.mocap_quat[self.rig] = _quat_of(placement.r.T)
        mujoco.mj_forward(m, d)


class ViewRenderer:
    """Renders ``SceneModel`` frames from an eye at ``width`` x ``height``."""

    def __init__(self, width: int, height: int, controllers: str = "overlay"):
        if controllers not in ("overlay", "depth", "off"):
            raise ValueError("controllers: overlay, depth or off")
        self.width, self.height, self.controllers = int(width), int(height), controllers
        self._renderers: dict[int, mujoco.Renderer] = {}

    def _renderer(self, model: SceneModel) -> mujoco.Renderer:
        r = self._renderers.get(id(model))
        if r is None:
            r = mujoco.Renderer(model.m, self.height, self.width, max_geom=model.m.ngeom + 64)
            self._renderers = {id(model): r}           # one scene at a time: drop the previous one's context
        return r

    def _camera(self, r: mujoco.Renderer, cam_p, cam_r, eye: ar.Eye, scale: float) -> None:
        tl, tr, tu, td = eye.tangents()
        n, f = NEAR_M / scale, FAR_M / scale
        fwd, up = cam_r @ np.array([0.0, 0.0, -1.0]), cam_r @ np.array([0.0, 1.0, 0.0])
        sc = r.scene
        for i in (0, 1):
            c = sc.camera[i]
            c.pos[:] = cam_p
            c.forward[:] = fwd
            c.up[:] = up
            c.orthographic = 0
            c.frustum_near, c.frustum_far = n, f
            c.frustum_bottom, c.frustum_top = n * td, n * tu
            c.frustum_center, c.frustum_width = n * (tr + tl) / 2, n * (tr - tl) / 2
        sc.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        sc.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
        sc.flags[mujoco.mjtRndFlag.mjRND_FOG] = 0
        sc.flags[mujoco.mjtRndFlag.mjRND_SKYBOX] = 1

    def _pass(self, model: SceneModel, groups, cam, eye: ar.Eye, scale: float, segment: bool = False) -> np.ndarray:
        r = self._renderer(model)
        opt = mujoco.MjvOption()
        opt.geomgroup[:] = 0
        for g in groups:
            opt.geomgroup[g] = 1
        if segment:
            r.enable_segmentation_rendering()
        try:
            r.update_scene(model.d, scene_option=opt)
            self._camera(r, cam[0], cam[1], eye, scale)
            return r.render().copy()
        finally:
            if segment:
                r.disable_segmentation_rendering()

    def render(self, model: SceneModel, frame: Optional[Frame], placement: ar.SimInLocal, eye_local: tuple,
               eye: ar.Eye, hands: dict) -> np.ndarray:
        """(height, width, 3) uint8: the scene from ``eye_local`` (an eye pose in LOCAL) through ``placement``."""
        hands = hands if self.controllers != "off" else {}
        model.apply(frame, placement, hands)
        cam = placement.camera(*eye_local)
        s = placement.scale
        if self.controllers == "depth":
            return self._pass(model, (SCENE_GROUP, CONTROLLER_GROUP), cam, eye, s)
        img = self._pass(model, (SCENE_GROUP,), cam, eye, s)
        if any(h is not None for h in hands.values()):
            top = self._pass(model, (CONTROLLER_GROUP,), cam, eye, s)
            seg = self._pass(model, (CONTROLLER_GROUP,), cam, eye, s, segment=True)
            mask = seg[..., 0] >= 0
            img[mask] = top[mask]
        return img
