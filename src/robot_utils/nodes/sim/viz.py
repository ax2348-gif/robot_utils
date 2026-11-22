"""What the sim node's 3D view draws (docs/sim.md, "3D view and markers"): the world's geometry once, then
body poses many times a second, plus markers that other nodes and clients post.

- ``model(world, cell)``: the drawable geometry of the compiled model: per body (robot links, scene objects)
  its geoms (box, sphere, capsule, cylinder, ellipsoid, mesh) with their local pose and colour, and the
  meshes as one binary blob (float32 vertices, uint16 / uint32 triangle indices). The robot's visual geoms
  are drawn (a link without any falls back to its collision geoms); the floor is not (the page draws a grid).
  Meshes are simplified by vertex clustering (``cell`` m, 0 = as compiled): the arm's CAD meshes are
  ~216 k vertices, 2 mm cells keep ~38 k (~1 MB) and look the same at page scale.
- ``frame(world)``: every body's world pose now (``xpos``, ``xquat`` w x y z), the sim clock and the base pose.
- ``Markers``: named frames, points, lines and text posted over HTTP (e.g. the WBC's EE targets), each in a
  frame (``world``, ``base`` or any model body / object) with an optional lifetime; the page resolves the frame
  with the body poses it already has.
"""
from __future__ import annotations

import math
import re
import threading
import time
from typing import Optional

import mujoco
import numpy as np

GEOM_TYPES = {mujoco.mjtGeom.mjGEOM_BOX: "box", mujoco.mjtGeom.mjGEOM_SPHERE: "sphere",
              mujoco.mjtGeom.mjGEOM_CAPSULE: "capsule", mujoco.mjtGeom.mjGEOM_CYLINDER: "cylinder",
              mujoco.mjtGeom.mjGEOM_ELLIPSOID: "ellipsoid", mujoco.mjtGeom.mjGEOM_MESH: "mesh"}
DEFAULT_CELL = 0.002


def cluster(v: np.ndarray, f: np.ndarray, cell: float) -> tuple[np.ndarray, np.ndarray]:
    """Vertex-clustering simplification: vertices merged per ``cell``-sized voxel (their mean), degenerate and
    duplicate triangles dropped, winding kept."""
    if cell <= 0 or len(v) == 0:
        return v, f
    key = np.floor(v / cell).astype(np.int64)
    _, inv, counts = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    nv = np.stack([np.bincount(inv, weights=v[:, i], minlength=counts.size) for i in range(3)], 1) / counts[:, None]
    nf = inv[f]
    nf = nf[(nf[:, 0] != nf[:, 1]) & (nf[:, 1] != nf[:, 2]) & (nf[:, 0] != nf[:, 2])]
    if len(nf):
        _, idx = np.unique(np.sort(nf, 1), axis=0, return_index=True)
        nf = nf[np.sort(idx)]
    return nv, nf


def _r(a, n: int = 6) -> list:
    return [round(float(x), n) for x in np.asarray(a).reshape(-1)]


def body_kind(name: str) -> str:
    return "world" if name == "world" else ("object" if name.startswith("obj_") else "robot")


def model(world, cell: float = DEFAULT_CELL) -> tuple[dict, bytes]:
    """(the geometry index as JSON-able dict, the mesh blob). Cached per (model generation, cell)."""
    key = (world.generation, round(float(cell), 6))
    cached = getattr(world, "_viz_cache", None)
    if cached is not None and cached[0] == key:
        return cached[1], cached[2]
    with world.lock:
        m = world.m
        bodies = []
        for b in range(m.nbody):
            name = m.body(b).name
            kind = body_kind(name)
            bodies.append({"name": name[4:] if kind == "object" else name, "body": name, "kind": kind,
                           "parent": int(m.body_parentid[b])})
        visual = np.array([m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0 for g in range(m.ngeom)])
        has_visual = {int(m.geom_bodyid[g]) for g in range(m.ngeom) if visual[g]}
        geoms, used = [], {}
        for g in range(m.ngeom):
            b = int(m.geom_bodyid[g])
            t = GEOM_TYPES.get(mujoco.mjtGeom(m.geom_type[g]))
            if t is None or b == 0:
                continue
            if bodies[b]["kind"] == "robot" and b in has_visual and not visual[g]:
                continue
            rgba = m.geom_rgba[g] if m.geom_matid[g] < 0 else m.mat_rgba[m.geom_matid[g]]
            out = {"body": b, "type": t, "size": _r(m.geom_size[g]), "pos": _r(m.geom_pos[g]),
                   "quat": _r(m.geom_quat[g]), "rgba": _r(rgba, 3), "visual": bool(visual[g])}
            if t == "mesh":
                out["mesh"] = used.setdefault(int(m.geom_dataid[g]), len(used))
            geoms.append(out)
        meshes, parts, off = [], [], 0
        for mid, _ in sorted(used.items(), key=lambda kv: kv[1]):
            a, n = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
            fa, fn = m.mesh_faceadr[mid], m.mesh_facenum[mid]
            v, f = cluster(m.mesh_vert[a:a + n].astype(np.float64), m.mesh_face[fa:fa + fn].astype(np.int64), cell)
            vb = v.astype("<f4").tobytes()
            idx = "u16" if len(v) < 65536 else "u32"
            fb = f.astype("<u2" if idx == "u16" else "<u4").tobytes()
            pad = (-len(fb)) % 4
            meshes.append({"name": m.mesh(mid).name, "nvert": int(len(v)), "nface": int(len(f)),
                           "vert_offset": off, "face_offset": off + len(vb), "index": idx})
            parts += [vb, fb, b"\0" * pad]
            off += len(vb) + len(fb) + pad
        blob = b"".join(parts)
        info = {"generation": world.generation, "model_version": world.model_info.get("model_version"),
                "cell": float(cell), "bodies": bodies, "geoms": geoms, "meshes": meshes, "blob_bytes": len(blob),
                "base_body": "base_link"}
    world._viz_cache = (key, info, blob)
    return info, blob


def frame(world) -> dict:
    """Every body's world pose now (flat lists: xpos 3 per body, xquat 4 per body, w x y z)."""
    with world.lock:
        d = world.d
        pos = np.round(d.xpos, 5).reshape(-1).tolist()
        quat = np.round(d.xquat, 5).reshape(-1).tolist()
        b = world.base_state()
        return {"t": round(float(d.time), 4), "generation": world.generation, "running": world.running,
                "rtf": world.rtf, "rtf_measured": round(world.rtf_measured, 3), "pos": pos, "quat": quat,
                "base": {"xyz": b["xyz"], "yaw": b["rpy"][2], "twist": b["twist"]}}


# -- robot_state.v1 ------------------------------------------------------------------------------------------

STATE_SCHEMA = "robot_state.v1"


def _xyzw(q) -> list:
    return [round(float(q[1]), 6), round(float(q[2]), 6), round(float(q[3]), 6), round(float(q[0]), 6)]


def robot_state(world) -> dict:
    """The robot's full state in the description's terms: base_link's world pose and every movable joint by its
    model (URDF) name, plus the scene's objects. A client draws the robot from GET /model/tree with these."""
    with world.lock:
        m, d = world.m, world.d
        q = d.qpos[world.base_q:world.base_q + 7]
        joints = {}
        for j in range(m.njnt):
            if int(m.jnt_type[j]) in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
                joints[m.joint(j).name] = round(float(d.qpos[m.jnt_qposadr[j]]), 6)
        objects = {}
        for name, o in world.objects.items():
            b = world.obj_idx[name][0]
            geom = {"type": o["shape"], **({"size": o["size"]} if o["shape"] == "box"
                                           else {"radius": o["radius"], "length": o["length"]}),
                    "rgba": o.get("rgba")}
            objects[name] = {"pose": [round(float(v), 6) for v in d.xpos[b]] + _xyzw(d.xquat[b]), "geom": geom,
                             "fixed": bool(o.get("fixed"))}
        return {"t": round(float(d.time), 4), "model_version": world.model_info.get("model_version"),
                "base": [round(float(v), 6) for v in q[:3]] + _xyzw(q[3:7]), "joints": joints, "objects": objects,
                "sim": {"running": world.running, "rtf": world.rtf}}


# -- markers -------------------------------------------------------------------------------------------------

NAME = re.compile(r"^[A-Za-z0-9_.:/-]{1,80}$")
KINDS = ("frame", "point", "line", "text")
MAX_MARKERS = 500
MAX_POINTS = 2000


class Markers:
    """Named markers in the world, posted by any client: kind frame (axes at pos + quat), point (a dot at pos),
    line (a polyline through points), text (a label at pos). Coordinates are in ``frame``: ``world``,
    ``base`` (base_link) or a model body / object name. ``ttl`` s (wall clock) removes a marker unless
    re-posted (a live target posted at 10 Hz with ttl 0.5 disappears when its sender stops)."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.items: dict = {}        # name -> (marker dict, expires monotonic or inf)
        self.version = 0

    @staticmethod
    def check(mk: dict, frames) -> dict:
        mk = dict(mk)
        name = mk.get("name")
        if not isinstance(name, str) or not NAME.match(name):
            raise ValueError("a marker needs a name of letters, digits and _ . : / - (up to 80)")
        if mk.get("kind", "frame") not in KINDS:
            raise ValueError(f"marker {name}: kind must be one of {', '.join(KINDS)}")
        mk["kind"] = mk.get("kind", "frame")
        fr = mk.get("frame") or "world"
        if fr not in ("world", "base") and fr not in frames:
            raise ValueError(f"marker {name}: unknown frame {fr!r} (world, base or a body name)")
        mk["frame"] = fr
        if mk["kind"] == "line":
            pts = mk.get("points")
            if not pts or len(pts) > MAX_POINTS or any(len(p) != 3 for p in pts):
                raise ValueError(f"marker {name}: a line needs 1..{MAX_POINTS} points [x, y, z]")
            mk["points"] = [[float(v) for v in p] for p in pts]
            if not all(math.isfinite(v) for p in mk["points"] for v in p):
                raise ValueError(f"marker {name}: points must be finite")
        else:
            pos = mk.get("pos")
            if pos is None or len(pos) != 3 or not all(math.isfinite(float(v)) for v in pos):
                raise ValueError(f"marker {name}: pos must be three finite numbers")
            mk["pos"] = [float(v) for v in pos]
        q = mk.get("quat")
        if q is not None:
            q = [float(v) for v in q]
            n = math.sqrt(sum(v * v for v in q))
            if len(q) != 4 or not math.isfinite(n) or n < 1e-9:
                raise ValueError(f"marker {name}: quat must be [x, y, z, w], non-zero")
            mk["quat"] = [v / n for v in q]
        return mk

    def put(self, markers: list, frames, source: Optional[str] = None, replace: bool = False) -> list:
        checked = [self.check(mk, frames) for mk in markers]
        now = self.clock()
        with self.lock:
            items = {k: v for k, v in self.items.items() if v[1] > now and not (replace and v[0].get("source") == source)}
            for mk in checked:
                if source is not None:
                    mk["source"] = source
                ttl = mk.get("ttl")
                items[mk["name"]] = (mk, now + float(ttl) if ttl else math.inf)
            if len(items) > MAX_MARKERS:
                raise ValueError(f"too many markers (max {MAX_MARKERS}); DELETE /markers to clear")
            self.items = items
            self.version += 1
        return [mk["name"] for mk in checked]

    def remove(self, name: Optional[str] = None, source: Optional[str] = None) -> int:
        with self.lock:
            before = len(self.items)
            if name is not None:
                if name not in self.items:
                    raise KeyError(f"no marker {name!r}")
                del self.items[name]
            else:
                self.items = {k: v for k, v in self.items.items()
                              if source is not None and v[0].get("source") != source}
            self.version += 1
            return before - len(self.items)

    def list(self) -> list:
        now = self.clock()
        with self.lock:
            gone = [k for k, (_, exp) in self.items.items() if exp <= now]
            for k in gone:
                del self.items[k]
            if gone:
                self.version += 1
            return [mk for mk, _ in self.items.values()]
