"""The sim world as a rigid scene (rigid_scene.v1, docs/scene3d.md): MuJoCo's bodies, geoms and meshes, and every
moving body's pose.

- A body is **static** when MuJoCo welds it to the world (``body_weldid == 0``: the world body and fixed scene
  objects); everything else is streamed, robot links and articulated parts alike.
- **Roles**: a geom with ``contype == conaffinity == 0`` is ``visual``; a colliding geom is ``collision``, or ``both``
  on a body without visual geoms (scene objects: what is drawn is what collides).
- **What the physics touches**: MuJoCo collides a mesh as its **convex hull**, so a colliding mesh geom is exported
  as its hull (MuJoCo's own, ``mesh_graph``) with role ``collision``, and as the mesh itself with role ``visual``.
  The robot's fingers are the known case where the two differ (docs/sim.md, "Contacts").
- **Simplification**: visual meshes by vertex clustering at ``cell`` m (viz.cluster). No vertex moves further than
  the cell's diagonal, so ``error`` = cell * sqrt(3) bounds how far the drawn surface's vertices are from the
  model's (a bound on vertex displacement; a measured surface distance is the planned replacement). Hulls are
  exported as MuJoCo computed them (error 0).
- **Ghosts** (viz.Ghosts): copies of a body subtree (e.g. the gripper) at a goal pose, in the poster's colour:
  scene bodies of kind ``ghost`` whose poses and colours come with every frame.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

from ...scene3d import Frame, SceneBuilder
from ...scene3d.pose import compose, inverse
from .viz import GEOM_TYPES, body_kind, cluster

DEFAULT_CELL = 0.002


def _xyzw(q) -> list:
    return [float(q[1]), float(q[2]), float(q[3]), float(q[0])]


def _hull(m: mujoco.MjModel, mid: int):
    """MuJoCo's convex hull of mesh ``mid``: (vertices, triangles), or None if it has none."""
    adr = int(m.mesh_graphadr[mid])
    if adr < 0:
        return None
    g = m.mesh_graph[adr:]
    nv, nf = int(g[0]), int(g[1])
    # layout: numvert, numface, vert_edgeadr[nv], vert_globalid[nv], edge_localid[nv + 3 nf], face_globalid[3 nf]
    faces = g[2 + 3 * nv + 3 * nf:2 + 3 * nv + 6 * nf].reshape(-1, 3)
    used, inv = np.unique(faces.reshape(-1), return_inverse=True)
    a = m.mesh_vertadr[mid]
    return m.mesh_vert[a + used].astype(np.float64), inv.reshape(-1, 3)


def _dims(t: str, size) -> dict:
    s = [float(x) for x in size]
    if t == "box":
        return {"half_extents": s[:3]}
    if t == "sphere":
        return {"radius": s[0]}
    if t in ("capsule", "cylinder"):
        return {"radius": s[0], "half_length": s[1]}
    if t == "ellipsoid":
        return {"radii": s[:3]}
    return {}


def build(world, cell: float = DEFAULT_CELL, crease_deg: float = 30.0, ghosts: tuple = ()) -> tuple[dict, bytes]:
    """(index, blob) of the world now. Cached per (model generation, cell, static poses, ghosts): fixed objects can
    be moved without a recompile, and a static body's pose is part of the scene. ``ghosts``: (name, body, subtree)
    per declared ghost (viz.Ghosts.declared): each adds copies of the body (and its subtree), kind ``ghost``, named
    ``ghost:<name>/<body>``, after the world's bodies; ``index["ghosts"]`` maps them back."""
    with world.lock:
        m, d = world.m, world.d
        static = [b for b in range(m.nbody) if int(m.body_weldid[b]) == 0]
        key = (world.generation, round(float(cell), 6), float(crease_deg), tuple(ghosts),
               np.round(np.concatenate([d.xpos[static].reshape(-1), d.xquat[static].reshape(-1)]), 6).tobytes())
        cached = getattr(world, "_rigid_cache", None)
        if cached is not None and cached[0] == key:
            return cached[1], cached[2]
        sb = SceneBuilder("sim", crease_deg)
        visual = np.array([m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0 for g in range(m.ngeom)])
        has_visual = {int(m.geom_bodyid[g]) for g in range(m.ngeom) if visual[g]}
        meshes: dict[tuple, int] = {}

        def mesh(mid: int, hull: bool) -> int:
            k = (mid, hull)
            if k not in meshes:
                name = m.mesh(mid).name
                if hull and (h := _hull(m, mid)) is not None:
                    meshes[k] = sb.add_mesh(f"{name}.hull", h[0], h[1], error=0.0)
                else:
                    a, n = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
                    fa, fn = m.mesh_faceadr[mid], m.mesh_facenum[mid]
                    v, f = cluster(m.mesh_vert[a:a + n].astype(np.float64),
                                   m.mesh_face[fa:fa + fn].astype(np.int64), cell)
                    meshes[k] = sb.add_mesh(name, v, f, error=cell * math.sqrt(3) if cell > 0 else 0.0)
            return meshes[k]

        def add_geoms(bid: int, b: int) -> None:
            """MuJoCo body ``b``'s geoms onto scene body ``bid``."""
            for g in np.flatnonzero(m.geom_bodyid == b):
                gt = mujoco.mjtGeom(m.geom_type[g])
                t = "plane" if gt == mujoco.mjtGeom.mjGEOM_PLANE else GEOM_TYPES.get(gt)
                if t is None:
                    continue
                rgba = m.geom_rgba[g] if m.geom_matid[g] < 0 else m.mat_rgba[m.geom_matid[g]]
                common = {"pos": m.geom_pos[g], "quat": _xyzw(m.geom_quat[g]), "rgba": rgba}
                role = "visual" if visual[g] else ("collision" if b in has_visual else "both")
                if t == "plane":
                    sb.add_geom(bid, "plane", role=role, half_extents=[float(x) for x in m.geom_size[g][:2]], **common)
                elif t == "mesh":
                    mid = int(m.geom_dataid[g])
                    if role == "visual":
                        sb.add_geom(bid, "mesh", role="visual", mesh=mesh(mid, False), **common)
                    else:
                        sb.add_geom(bid, "mesh", role="collision", mesh=mesh(mid, True), **common)
                        if role == "both":
                            sb.add_geom(bid, "mesh", role="visual", mesh=mesh(mid, False), **common)
                else:
                    sb.add_geom(bid, t, role=role, **_dims(t, m.geom_size[g]), **common)

        for b in range(m.nbody):
            name = m.body(b).name
            kind = body_kind(name)
            joint = None
            if m.body_jntnum[b] == 1:
                j = int(m.body_jntadr[b])
                jt = mujoco.mjtJoint(m.jnt_type[j])
                if jt in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                    joint = {"name": m.joint(j).name, "type": "hinge" if jt == mujoco.mjtJoint.mjJNT_HINGE else "slide",
                             **({"range": [float(x) for x in m.jnt_range[j]]} if m.jnt_limited[j] else {})}
            bid = sb.add_body(name[4:] if kind == "object" else name, kind, list(d.xpos[b]) + _xyzw(d.xquat[b]),
                              int(m.body_weldid[b]) != 0, parent=int(m.body_parentid[b]) if b else -1, joint=joint)
            add_geoms(bid, b)
        gmap = []
        for gname, root, subtree in ghosts:
            r = m.body(root).id
            src = [b for b in range(m.nbody) if b == r or (subtree and _descends(m, b, r))]
            ids = []
            for b in src:
                ids.append(sb.add_body(f"ghost:{gname}/{m.body(b).name}", "ghost",
                                       list(d.xpos[b]) + _xyzw(d.xquat[b]), True))
                add_geoms(ids[-1], b)
            gmap.append({"name": gname, "body": root, "subtree": bool(subtree), "bodies": ids, "sources": src})
        index, blob = sb.build()
        index["generation"] = world.generation
        index["cell"] = float(cell)
        index["model_version"] = world.model_info.get("model_version")
        index["ghosts"] = gmap
    world._rigid_cache = (key, index, blob)
    return index, blob


def _descends(m, b: int, root: int) -> bool:
    while b > 0:
        b = int(m.body_parentid[b])
        if b == root:
            return True
    return False


class Poses:
    """Frames of one scene: the dynamic bodies' poses from the world now (scene body ids below m.nbody are MuJoCo's
    body ids: the builder adds every body in order), the ghosts' from their posts."""

    def __init__(self, index: dict):
        self.index = index
        self.dynamic = list(index["dynamic"])
        self.slot = {b: k for k, b in enumerate(self.dynamic)}
        self.ghosts = index.get("ghosts") or []
        ghost_ids = {i for g in self.ghosts for i in g["bodies"]}
        self.world_slots = [k for k, b in enumerate(self.dynamic) if b not in ghost_ids]
        self.world_ids = np.array([self.dynamic[k] for k in self.world_slots], dtype=int)
        self.seq = 0

    def frame(self, world, live: dict | None = None) -> Frame:
        """``live``: name -> ghost (viz.Ghosts.live); a declared ghost that is not live is hidden (alpha 0)."""
        live = live or {}
        with world.lock:
            d = world.d
            p = np.zeros((len(self.dynamic), 7))
            if len(self.world_ids):
                p[self.world_slots] = np.concatenate([d.xpos[self.world_ids], d.xquat[self.world_ids][:, [1, 2, 3, 0]]],
                                                     axis=1)
            colors = {}
            for g in self.ghosts:
                gl = live.get(g["name"])
                xs = [np.concatenate([d.xpos[b], _xyzw(d.xquat[b])]) for b in g["sources"]]
                if gl is None:
                    for i, x in zip(g["bodies"], xs):
                        p[self.slot[i]] = x
                        colors[self.slot[i]] = [0.0, 0.0, 0.0, 0.0]
                    continue
                root = np.concatenate([d.xpos[g["sources"][0]], _xyzw(d.xquat[g["sources"][0]])])
                if gl["mode"] == "pose":
                    G = compose(gl["value"], inverse(root))
                else:                                   # relative: the goal in the root body's own frame
                    G = compose(compose(root, gl["value"]), inverse(root))
                for i, x in zip(g["bodies"], xs):
                    p[self.slot[i]] = compose(G, x)
                    colors[self.slot[i]] = list(gl["rgba"])
            t, rs = float(d.time), int(world.reset_seq)
        self.seq += 1
        return Frame(self.seq, t, rs, self.index["scene_id"], p, colors)
