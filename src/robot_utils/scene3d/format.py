"""rigid_scene.v1: a world as rigid bodies (docs/scene3d.md).

**Scene** (sent once, cached by ``scene_id``): a JSON index plus one binary blob.

- ``bodies``: ``{id, name, kind, parent, dynamic, pose, geoms, joint?}``. ``pose`` is the body's world pose when
  the scene was made ([x, y, z, qx, qy, qz, qw]); a static body (``dynamic: false``) never moves, so that is its
  pose for good. ``parent`` and ``joint`` are informational (cues, labels): a viewer places every body from its
  own pose and never runs kinematics.
- each geom: ``{type, pos, quat, rgba, role}`` and its dimensions: box ``half_extents`` [3], sphere ``radius``,
  capsule / cylinder ``radius`` + ``half_length`` (along local z), ellipsoid ``radii`` [3], plane
  ``half_extents`` [2] (0: unbounded), mesh ``mesh`` (an index into ``meshes``). ``role``: ``collision`` (what
  the physics touches), ``visual`` (drawn only) or ``both``.
- ``meshes``: ``{name, nvert, ntri, nedge, vert_offset, tri_offset, edge_offset, index, bounds, error}``: in the
  blob, float32 xyz vertices, then ``index`` (u16 | u32) triangles, then outline edges (vertex pairs: boundary,
  non-manifold and crease edges, ``crease_deg``), each section 4-byte aligned. ``error``: the largest distance
  (m) from the simulated surface the mesh may be off by after simplification, when known.
- ``dynamic``: the ids of the bodies that move, in the order of the pose stream.

**Frame** (streamed): the dynamic bodies' world poses. Binary, little-endian, 32-byte header ``RSF1``, u32 seq,
f64 t (the publisher's clock), u32 reset_seq, u32 n, the first 8 bytes of ``scene_id`` (hex-decoded); then n x 7
float32 (x, y, z, qx, qy, qz, qw). A viewer whose scene_id differs fetches the scene again.

Conventions: z up, metres, quaternions x, y, z, w (as robot_state.v1).
"""
from __future__ import annotations

import hashlib
import json
import struct
from dataclasses import dataclass
from typing import Optional

import numpy as np

SCHEMA = "rigid_scene.v1"
FRAME_MAGIC = b"RSF1"
_HEADER = struct.Struct("<4sIdII8s")
CONVENTIONS = {"up": "+z", "units": "m", "quat": "xyzw"}
GEOM_TYPES = ("box", "sphere", "capsule", "cylinder", "ellipsoid", "plane", "mesh")
ROLES = ("collision", "visual", "both")


def _r(a, n: int = 6) -> list:
    return [round(float(x), n) for x in np.asarray(a, dtype=float).reshape(-1)]


def outline_edges(v: np.ndarray, f: np.ndarray, crease_deg: float = 30.0) -> np.ndarray:
    """The edges a line drawing keeps: boundary (one face), non-manifold (3+ faces) and crease edges (the two faces'
    normals more than ``crease_deg`` apart). (k, 2) vertex index pairs, each once, smaller index first."""
    f = np.asarray(f, dtype=np.int64).reshape(-1, 3)
    v = np.asarray(v, dtype=float).reshape(-1, 3)
    if len(f) == 0:
        return np.zeros((0, 2), np.int64)
    n = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    area = np.linalg.norm(n, axis=1)
    keep = area > 1e-14
    f, n = f[keep], n[keep] / area[keep, None]
    if len(f) == 0:
        return np.zeros((0, 2), np.int64)
    e = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
    face = np.tile(np.arange(len(f)), 3)
    uniq, inv, counts = np.unique(e, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    out = counts != 2                                    # boundary or non-manifold
    two = np.flatnonzero(counts == 2)
    if len(two):
        order = np.argsort(inv, kind="stable")
        starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
        fa, fb = face[order[starts[two]]], face[order[starts[two] + 1]]
        cos = np.einsum("ij,ij->i", n[fa], n[fb])
        out[two] = cos < np.cos(np.radians(crease_deg))
    return uniq[out]


def scene_hash(index: dict, blob: bytes) -> str:
    """16 hex digits over the index (without its scene_id) and the blob: equal scenes, equal ids."""
    body = {k: v for k, v in index.items() if k != "scene_id"}
    h = hashlib.sha256(json.dumps(body, sort_keys=True, separators=(",", ":")).encode())
    h.update(blob)
    return h.hexdigest()[:16]


class SceneBuilder:
    """Collects bodies and meshes, then ``build()`` -> (index, blob) with its scene_id."""

    def __init__(self, source: str, crease_deg: float = 30.0):
        self.source = source
        self.crease_deg = float(crease_deg)
        self.bodies: list[dict] = []
        self.meshes: list[dict] = []
        self._parts: list[bytes] = []
        self._off = 0

    def _append(self, data: bytes) -> int:
        at = self._off
        pad = (-len(data)) % 4
        self._parts += [data, b"\0" * pad]
        self._off += len(data) + pad
        return at

    def add_mesh(self, name: str, vertices, triangles, error: Optional[float] = None,
                 edges: Optional[np.ndarray] = None) -> int:
        """A triangle mesh in its own frame (geoms place it); returns its index. ``edges``: outline edges when the
        caller has them (else found here at ``crease_deg``)."""
        v = np.asarray(vertices, dtype=float).reshape(-1, 3)
        t = np.asarray(triangles, dtype=np.int64).reshape(-1, 3)
        e = outline_edges(v, t, self.crease_deg) if edges is None else np.asarray(edges, np.int64).reshape(-1, 2)
        idx = "u16" if len(v) < 65536 else "u32"
        it = "<u2" if idx == "u16" else "<u4"
        vo = self._append(v.astype("<f4").tobytes())
        to = self._append(t.astype(it).tobytes())
        eo = self._append(e.astype(it).tobytes())
        lo, hi = (v.min(0), v.max(0)) if len(v) else (np.zeros(3), np.zeros(3))
        self.meshes.append({"name": name, "nvert": int(len(v)), "ntri": int(len(t)), "nedge": int(len(e)),
                            "vert_offset": vo, "tri_offset": to, "edge_offset": eo, "index": idx,
                            "bounds": [_r(lo), _r(hi)], "error": None if error is None else float(error)})
        return len(self.meshes) - 1

    def add_body(self, name: str, kind: str, pose, dynamic: bool, parent: int = -1, geoms: Optional[list] = None,
                 joint: Optional[dict] = None) -> int:
        b = {"id": len(self.bodies), "name": name, "kind": kind, "parent": int(parent), "dynamic": bool(dynamic),
             "pose": _r(pose), "geoms": []}
        if joint:
            b["joint"] = joint
        self.bodies.append(b)
        for g in geoms or []:
            self.add_geom(b["id"], **g)
        return b["id"]

    def add_geom(self, body: int, type: str, pos=(0, 0, 0), quat=(0, 0, 0, 1), rgba=(0.7, 0.7, 0.7, 1.0),
                 role: str = "both", **dims) -> dict:
        if type not in GEOM_TYPES:
            raise ValueError(f"geom type {type!r}: one of {', '.join(GEOM_TYPES)}")
        if role not in ROLES:
            raise ValueError(f"geom role {role!r}: one of {', '.join(ROLES)}")
        g = {"type": type, "pos": _r(pos), "quat": _r(quat), "rgba": _r(rgba, 3), "role": role}
        need = {"box": ("half_extents",), "sphere": ("radius",), "capsule": ("radius", "half_length"),
                "cylinder": ("radius", "half_length"), "ellipsoid": ("radii",), "plane": ("half_extents",),
                "mesh": ("mesh",)}[type]
        for k in need:
            if k not in dims:
                raise ValueError(f"a {type} geom needs {k}")
            val = dims[k]
            g[k] = int(val) if k == "mesh" else (_r(val) if np.ndim(val) else round(float(val), 6))
        if type == "mesh" and not 0 <= g["mesh"] < len(self.meshes):
            raise ValueError(f"mesh {g['mesh']} not added")
        self.bodies[body]["geoms"].append(g)
        return g

    def build(self) -> tuple[dict, bytes]:
        blob = b"".join(self._parts)
        index = {"schema": SCHEMA, "source": self.source, "conventions": CONVENTIONS, "crease_deg": self.crease_deg,
                 "bodies": self.bodies, "dynamic": [b["id"] for b in self.bodies if b["dynamic"]],
                 "meshes": self.meshes, "blob_bytes": len(blob)}
        index["scene_id"] = scene_hash(index, blob)
        return index, blob


@dataclass
class Frame:
    seq: int
    t: float
    reset_seq: int
    scene_id: str
    poses: np.ndarray            # (n, 7): x, y, z, qx, qy, qz, qw

    def to_json(self) -> dict:
        return {"seq": self.seq, "t": round(self.t, 4), "reset_seq": self.reset_seq, "scene_id": self.scene_id,
                "poses": [_r(p) for p in self.poses]}


def encode_frame(f: Frame) -> bytes:
    p = np.asarray(f.poses, dtype="<f4").reshape(-1, 7)
    return _HEADER.pack(FRAME_MAGIC, f.seq & 0xFFFFFFFF, float(f.t), f.reset_seq & 0xFFFFFFFF, len(p),
                        bytes.fromhex(f.scene_id[:16].ljust(16, "0"))) + p.tobytes()


def decode_frame(data: bytes) -> tuple[Frame, int]:
    """(the frame, bytes used) from the start of ``data``; ValueError if it is not a whole frame."""
    if len(data) < _HEADER.size:
        raise ValueError("short frame header")
    magic, seq, t, reset_seq, n, sid = _HEADER.unpack_from(data)
    if magic != FRAME_MAGIC:
        raise ValueError(f"bad frame magic {magic!r}")
    end = _HEADER.size + n * 28
    if len(data) < end:
        raise ValueError("short frame body")
    poses = np.frombuffer(data, dtype="<f4", count=n * 7, offset=_HEADER.size).reshape(n, 7).astype(float)
    return Frame(seq, t, reset_seq, sid.hex(), poses), end


def frame_bytes(n: int) -> int:
    return _HEADER.size + 28 * n


def mesh_arrays(index: dict, blob: bytes, i: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(vertices (n, 3), triangles (m, 3), edges (k, 2)) of mesh ``i`` from a scene's blob."""
    m = index["meshes"][i]
    it = "<u2" if m["index"] == "u16" else "<u4"
    v = np.frombuffer(blob, "<f4", m["nvert"] * 3, m["vert_offset"]).reshape(-1, 3)
    t = np.frombuffer(blob, it, m["ntri"] * 3, m["tri_offset"]).reshape(-1, 3)
    e = np.frombuffer(blob, it, m["nedge"] * 2, m["edge_offset"]).reshape(-1, 2)
    return v, t, e
