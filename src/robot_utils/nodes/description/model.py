"""The robot model: components assembled into one URDF (docs/description_node.md, "Robot model").

The description YAML's ``components:`` map names each part of the robot and where it comes from:

- ``source: urdf``: a URDF file with its meshes (``file: <package>:<path>``), every link and joint
  prefixed (``prefix:``). Its ``simple`` variant replaces each link's geometry with one bounding
  capsule fitted to the link's collision meshes (``fit_capsule``), keeping the full model's masses and
  inertias.
- ``source: procedural``: generated from dimensions: ``box``, ``cylinder``, ``capsule``, ``sphere``,
  ``swerve_base`` (frame + four modules, each a steer joint and a wheel joint) and ``camera`` (a box
  body with the Orbbec frame names ``<name>_depth_optical``, ``<name>_color_optical``, ...). Mass is
  given (``mass``) or ``density`` x volume; the inertia is the primitive's.

Each component hangs from a ``parent`` link at a ``mount`` pose: explicit (``xyz``/``rpy``), a frame of
the description (``{frame: arm}``) or a camera's composed pose (``{camera: camera}``: the active
calibrations, or the nominal pose with ``mounts=nominal``). The one component without a parent is the
root, ``base_link`` (the description's ``base`` frame).

Pure: no HTTP, no node state. The node hands in its description, frames and active calibrations.
"""
from __future__ import annotations

import copy
import hashlib
import io
import json
import math
import os
import struct
import threading
import xml.etree.ElementTree as ET
import zipfile
from dataclasses import dataclass, field
from pathlib import Path, PurePosixPath
from typing import Optional

import numpy as np

from .frames import FrameTree, frame_T, make_T, matrix_to_rpy

VARIANTS = ("full", "simple")
BASES = ("fixed", "planar", "swerve")
MOUNTS = ("nominal", "calibrated")
MESH_URIS = ("package", "http", "relative", "file")
PROCEDURAL = ("box", "cylinder", "capsule", "sphere", "swerve_base", "camera")
MESH_SUFFIXES = (".stl", ".dae", ".obj")
ROOT_LINK = "base_link"
TINY_MASS = 1e-4          # kg, the planar base's massless x / y links (simulators need some)


class ModelError(ValueError):
    """The components or the options are invalid (HTTP 400 / 404)."""


# -- geometry helpers -----------------------------------------------------------------------------

def load_stl(path) -> np.ndarray:
    """Vertices (N x 3, float) of an STL file, binary or ASCII; repeated vertices are kept."""
    data = Path(path).read_bytes()
    if len(data) >= 84:
        n = struct.unpack_from("<I", data, 80)[0]
        if 84 + 50 * n == len(data):
            rec = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                                count=n, offset=84)
            return rec["v"].reshape(-1, 3).astype(float)
    pts = [[float(x) for x in line.split()[1:4]] for line in data.decode("ascii", "replace").splitlines()
           if line.strip().startswith("vertex")]
    if not pts:
        raise ModelError(f"{path}: not an STL file")
    return np.asarray(pts, dtype=float)


def fit_capsule(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    """A capsule bounding every point: (segment start, segment end, radius).

    The axis is the points' principal direction through their centroid; the radius the largest
    distance from it; the segment then as short as keeps each point inside a hemispherical cap
    (a point at distance d from the axis needs the segment end within sqrt(r^2 - d^2) of it)."""
    with np.errstate(all="ignore"):     # macOS Accelerate flags spurious FP errors in large matmuls
        return _fit_capsule(np.asarray(points, dtype=float).reshape(-1, 3))


def _fit_capsule(p: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    c = p.mean(axis=0)
    _, _, vt = np.linalg.svd(p - c, full_matrices=False)
    axis = vt[0] / np.linalg.norm(vt[0])
    t = (p - c) @ axis
    radial = p - c - np.outer(t, axis)
    d = np.linalg.norm(radial, axis=1)
    r = float(d.max()) * (1 + 1e-9) + 1e-9
    h = np.sqrt(np.maximum(r * r - d * d, 0.0))
    lo, hi = float(np.min(t + h)), float(np.max(t - h))
    if lo > hi:                       # shorter than its diameter: a sphere, grown to hold the ends
        mid = 0.5 * (lo + hi)
        lo = hi = mid
        r = float(np.max(np.linalg.norm(p - (c + mid * axis), axis=1))) * (1 + 1e-9) + 1e-9
    return c + lo * axis, c + hi * axis, r


def axis_rotation(z: np.ndarray) -> np.ndarray:
    """A rotation whose z axis is ``z`` (for a URDF cylinder along a direction)."""
    z = np.asarray(z, dtype=float)
    z = z / np.linalg.norm(z)
    a = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(a, z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def primitive_volume(kind: str, g: dict) -> float:
    if kind == "box":
        x, y, z = g["size"]
        return x * y * z
    if kind == "cylinder":
        return math.pi * g["radius"] ** 2 * g["length"]
    if kind == "sphere":
        return 4 / 3 * math.pi * g["radius"] ** 3
    if kind == "capsule":
        r = g["radius"]
        return math.pi * r * r * g["length"] + 4 / 3 * math.pi * r ** 3
    raise ModelError(f"unknown primitive {kind!r}")


def primitive_inertia(kind: str, m: float, g: dict) -> np.ndarray:
    """3x3 inertia (kg m^2) about the centre, primitive axes (cylinder / capsule along z)."""
    if kind == "box":
        x, y, z = g["size"]
        return np.diag([m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12])
    if kind == "cylinder":
        r, h = g["radius"], g["length"]
        i = m * (3 * r * r + h * h) / 12
        return np.diag([i, i, m * r * r / 2])
    if kind == "sphere":
        return np.eye(3) * (2 / 5 * m * g["radius"] ** 2)
    if kind == "capsule":
        r, h = g["radius"], g["length"]
        vc, vs = math.pi * r * r * h, 4 / 3 * math.pi * r ** 3
        mc, ms = m * vc / (vc + vs), m * vs / (vc + vs)
        ixx = mc * (h * h / 12 + r * r / 4) + ms * (2 * r * r / 5 + h * h / 4 + 3 * h * r / 8)
        return np.diag([ixx, ixx, mc * r * r / 2 + ms * 2 * r * r / 5])
    raise ModelError(f"unknown primitive {kind!r}")


def combine_inertias(parts: list) -> tuple[float, np.ndarray, np.ndarray]:
    """(mass, centre of mass, inertia about it) of parts [(m, com, I about com, frame axes)]."""
    m = sum(p[0] for p in parts)
    if m <= 0:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    com = sum(p[0] * np.asarray(p[1]) for p in parts) / m
    I = np.zeros((3, 3))
    for mi, ci, Ii in parts:
        d = np.asarray(ci) - com
        I += Ii + mi * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return m, com, I


# -- file hashes (model_version) ------------------------------------------------------------------

_hash_cache: dict = {}
_hash_lock = threading.Lock()


def file_hash(path: Path) -> str:
    st = path.stat()
    key = (str(path), st.st_mtime_ns, st.st_size)
    with _hash_lock:
        h = _hash_cache.get(key)
    if h is None:
        h = hashlib.sha256(path.read_bytes()).hexdigest()
        with _hash_lock:
            _hash_cache[key] = h
    return h


_capsule_cache: dict = {}


def _mesh_points(path: Path, scale) -> np.ndarray:
    st = path.stat()
    key = (str(path), st.st_mtime_ns, st.st_size)
    pts = _capsule_cache.get(key)
    if pts is None:
        if path.suffix.lower() != ".stl":
            raise ModelError(f"{path.name}: only STL meshes can be fitted with capsules")
        pts = load_stl(path)
        _capsule_cache[key] = pts
    return pts * np.asarray(scale, dtype=float)


# -- packages ---------------------------------------------------------------------------------------

def package_root(name: str) -> Path:
    """Where a ``<package>:`` file reference points. ``rebot_ctl``: the reBot arm project (the arm
    node's own rebot_ctl install); ``robot_utils``: this package."""
    if name == "rebot_ctl":
        import reBotArm_control_py
        return Path(reBotArm_control_py.__file__).resolve().parents[1]
    if name == "robot_utils":
        return Path(__file__).resolve().parents[2]
    raise ModelError(f"unknown package {name!r} (have rebot_ctl, robot_utils)")


def resolve_file(ref: str) -> Path:
    if ":" in ref and not ref.startswith("/"):
        pkg, rel = ref.split(":", 1)
        p = package_root(pkg) / rel
    else:
        p = Path(os.path.expanduser(ref))
    if not p.is_file():
        raise ModelError(f"component file not found: {ref} ({p})")
    return p.resolve()


# -- options ----------------------------------------------------------------------------------------

@dataclass
class Options:
    """How to assemble: ``variant`` full | simple, per component overridable (``variant="simple,arm=full"``);
    ``base`` fixed | planar | swerve; ``mounts`` nominal | calibrated; ``mesh_uri`` package | http |
    relative | file (``http_base``: the URL the http meshes are under)."""
    variant: str = "full"
    overrides: dict = field(default_factory=dict)
    base: str = "fixed"
    mounts: str = "calibrated"
    mesh_uri: str = "package"
    http_base: str = ""

    @classmethod
    def parse(cls, variant: str = "full", base: str = "fixed", mounts: str = "calibrated",
              mesh_uri: str = "package", http_base: str = "") -> "Options":
        o = cls(base=base, mounts=mounts, mesh_uri=mesh_uri, http_base=http_base.rstrip("/"))
        for part in (p.strip() for p in (variant or "full").split(",") if p.strip()):
            if "=" in part:
                k, v = (s.strip() for s in part.split("=", 1))
                if v not in VARIANTS:
                    raise ModelError(f"variant of {k!r} must be one of {', '.join(VARIANTS)}, got {v!r}")
                o.overrides[k] = v
            elif part in VARIANTS:
                o.variant = part
            else:
                raise ModelError(f"variant must be full | simple, optionally per component (arm=full), got {part!r}")
        for name, val, allowed in (("base", base, BASES), ("mounts", mounts, MOUNTS), ("mesh_uri", mesh_uri, MESH_URIS)):
            if val not in allowed:
                raise ModelError(f"{name} must be one of {', '.join(allowed)}, got {val!r}")
        if mesh_uri == "http" and not o.http_base:
            raise ModelError("mesh_uri=http needs the node's URL")
        return o

    def variant_of(self, comp: str) -> str:
        return self.overrides.get(comp, self.variant)

    def public(self) -> dict:
        v = ",".join([self.variant] + [f"{k}={x}" for k, x in self.overrides.items()])
        return {"variant": v, "base": self.base, "mounts": self.mounts, "mesh_uri": self.mesh_uri}


# -- URDF building ----------------------------------------------------------------------------------

def _f(x: float) -> str:
    return f"{float(x):.9g}"


def _vec(v) -> str:
    return " ".join(_f(x) for x in v)


def origin_el(parent: ET.Element, T: np.ndarray) -> ET.Element:
    r, p, y = matrix_to_rpy(T[:3, :3])
    return ET.SubElement(parent, "origin", xyz=_vec(T[:3, 3]), rpy=_vec((r, p, y)))


def inertial_el(link: ET.Element, m: float, com, I: np.ndarray) -> None:
    el = ET.SubElement(link, "inertial")
    ET.SubElement(el, "origin", xyz=_vec(com), rpy="0 0 0")
    ET.SubElement(el, "mass", value=_f(m))
    ET.SubElement(el, "inertia", ixx=_f(I[0, 0]), ixy=_f(I[0, 1]), ixz=_f(I[0, 2]), iyy=_f(I[1, 1]),
                  iyz=_f(I[1, 2]), izz=_f(I[2, 2]))


def geom_el(link: ET.Element, tag: str, kind: str, g: dict, T: np.ndarray, material: Optional[str] = None,
            name: Optional[str] = None) -> None:
    el = ET.SubElement(link, tag, **({"name": name} if name else {}))
    origin_el(el, T)
    geo = ET.SubElement(el, "geometry")
    if kind == "box":
        ET.SubElement(geo, "box", size=_vec(g["size"]))
    elif kind == "cylinder":
        ET.SubElement(geo, "cylinder", radius=_f(g["radius"]), length=_f(g["length"]))
    elif kind == "sphere":
        ET.SubElement(geo, "sphere", radius=_f(g["radius"]))
    else:
        raise ModelError(f"no URDF geometry {kind!r}")
    if material and tag == "visual":
        ET.SubElement(el, "material", name=material)


def capsule_els(link: ET.Element, p0, p1, r: float, material: str, name: str) -> None:
    """A capsule as standard URDF: a cylinder between p0 and p1 plus a sphere at each end (URDF has
    no capsule), as visual and collision."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    length = float(np.linalg.norm(p1 - p0))
    for tag in ("visual", "collision"):
        if length > 1e-9:
            geom_el(link, tag, "cylinder", {"radius": r, "length": length},
                    make_T(axis_rotation(p1 - p0), (p0 + p1) / 2), material, f"{name}_body" if tag == "visual" else None)
        for i, p in enumerate((p0, p1) if length > 1e-9 else (p0,)):
            geom_el(link, tag, "sphere", {"radius": r}, make_T(np.eye(3), p), material,
                    f"{name}_end{i}" if tag == "visual" else None)


def fixed_joint(robot: ET.Element, name: str, parent: str, child: str, T: np.ndarray, jtype: str = "fixed",
                axis=None, limit: Optional[dict] = None) -> ET.Element:
    j = ET.SubElement(robot, "joint", name=name, type=jtype)
    origin_el(j, T)
    ET.SubElement(j, "parent", link=parent)
    ET.SubElement(j, "child", link=child)
    if axis is not None:
        ET.SubElement(j, "axis", xyz=_vec(axis))
    if limit is not None:
        ET.SubElement(j, "limit", **{k: _f(v) for k, v in limit.items()})
    return j


MATERIALS = {"ru_chassis": "0.55 0.57 0.6 1", "ru_module": "0.25 0.27 0.3 1", "ru_wheel": "0.08 0.08 0.08 1",
             "ru_camera": "0.12 0.12 0.14 1", "ru_part": "0.6 0.6 0.62 1", "ru_capsule": "0.7 0.72 0.74 1"}


# -- the assembly -----------------------------------------------------------------------------------

@dataclass
class Assembly:
    urdf: str
    root: str
    links: dict                  # link -> {component, parent, joint, type, T (4x4 in parent), axis, mass}
    joints: list                 # every joint: {name, type, parent, child, axis, lower, upper, velocity, effort, component}
    components: list             # public component info
    meshes: dict                 # (component, relative path) -> absolute Path
    mesh_uris: dict              # (component, relative path) -> the URI written in the URDF
    options: Options
    desc_frames: dict            # description frame name -> model link (the frames both have)

    def T_root(self, link: str) -> np.ndarray:
        T = np.eye(4)
        while link is not None:
            info = self.links[link]
            if info["parent"] is None:
                break
            T = info["T"] @ T
            link = info["parent"]
        return T

    def T(self, a: str, b: str) -> np.ndarray:
        """Pose of link b in link a at zero joint positions."""
        return np.linalg.inv(self.T_root(a)) @ self.T_root(b)

    def mass(self) -> float:
        return float(sum(v["mass"] for v in self.links.values()))


class ModelBuilder:
    """Assemble one URDF from the description's components.

    ``description``: the merged description YAML; ``frames``: the node's frame map (camera poses
    composed from the active calibrations, camera-internal frames from the camera nodes); ``active``:
    the active calibrations map (for the version only)."""

    def __init__(self, description: dict, frames: dict, active: Optional[dict] = None):
        self.desc = description
        self.frames_cal = frames
        self.active = active or {}
        self.components = copy.deepcopy(description.get("components") or {})
        if not self.components:
            raise ModelError("the description has no components")
        roots = [n for n, c in self.components.items() if not c.get("parent")]
        if len(roots) != 1:
            raise ModelError(f"exactly one component must have no parent (the root), got {roots}")
        self.root_component = roots[0]

    # -- frames ----------------------------------------------------------------------------------

    def frames(self, mounts: str) -> dict:
        """The description's frames with camera poses nominal or calibrated."""
        fr = {k: dict(v) for k, v in self.frames_cal.items()}
        if mounts == "nominal":
            for name, cam in (self.desc.get("cameras") or {}).items():
                if f"{name}/depth_optical" in fr:
                    fr[f"{name}/depth_optical"] = {"parent": "base", **cam["nominal"], "source": "nominal"}
        return fr

    def desc_map(self) -> dict:
        """Description frame -> model link, for frames the model has (camera-internal frames included)."""
        out = {}
        for name, c in self.components.items():
            link = self._root_link(name, c)
            if c.get("frame"):
                out[c["frame"]] = link
            cam = (c.get("mount") or {}).get("camera") if c.get("type") == "camera" else None
            if cam:
                out[f"{cam}/depth_optical"] = f"{name}_depth_optical"
                for f, v in self.frames_cal.items():
                    if f.startswith(f"{cam}/") and f != f"{cam}/depth_optical":
                        out[f] = f"{name}_{f.split('/', 1)[1]}"
        return out

    def _root_link(self, name: str, c: dict) -> str:
        if not c.get("parent"):
            return ROOT_LINK
        if c.get("link"):
            return c["link"]
        if c.get("source") == "urdf":
            return c.get("prefix", f"{name}_") + self._urdf_root(c)
        if c.get("type") == "camera":
            return f"{name}_depth_optical"
        return f"{name}_link"

    def _urdf_root(self, c: dict) -> str:
        tree = ET.fromstring(resolve_file(c["file"]).read_text(encoding="utf-8-sig"))
        children = {j.find("child").get("link") for j in tree.findall("joint")}
        roots = [lk.get("name") for lk in tree.findall("link") if lk.get("name") not in children]
        if len(roots) != 1:
            raise ModelError(f"{c['file']}: expected one root link, got {roots}")
        return roots[0]

    def _mount(self, name: str, c: dict, frames: dict, desc_of: dict) -> tuple[np.ndarray, str]:
        m = c.get("mount") or {}
        parent = c["parent"]
        if "frame" in m or "camera" in m:
            target = m["frame"] if "frame" in m else f"{m['camera']}/depth_optical"
            pdesc = desc_of.get(parent)
            if pdesc is None:
                raise ModelError(f"component {name!r}: its parent {parent!r} is not a description frame, so "
                                 f"its mount cannot come from {target!r}")
            if target not in frames:
                raise ModelError(f"component {name!r}: no description frame {target!r}")
            T = FrameTree(frames).T(pdesc, target)
            src = frames[target].get("source")
            how = f"frame {target}" if "frame" in m else (
                f"camera {m['camera']}: " + ("nominal" if src == "nominal" or not isinstance(src, dict) else
                                            "nominal refined by " + (", ".join(k for k in ("ground", "yaw") if src.get(k))
                                                                     or "nothing (no active calibration)")))
            return T, how
        return frame_T(m), "explicit"

    # -- assembly --------------------------------------------------------------------------------

    def build(self, opts: Optional[Options] = None) -> Assembly:
        opts = opts or Options()
        for k in opts.overrides:
            if k not in self.components:
                raise ModelError(f"variant override for unknown component {k!r}")
        frames = self.frames(opts.mounts)
        robot = ET.Element("robot", name=self.desc.get("name", "robot"))
        for mname, rgba in MATERIALS.items():
            ET.SubElement(ET.SubElement(robot, "material", name=mname), "color", rgba=rgba)
        a = Assembly("", ROOT_LINK, {}, [], [], {}, {}, opts, {})
        self._robot, self._a = robot, a
        desc_map = self.desc_map()
        desc_of = {v: k for k, v in desc_map.items()}
        placed: set = set()
        order = []
        pending = dict(self.components)
        while pending:                       # parents first
            progress = False
            for name, c in list(pending.items()):
                if not c.get("parent") or c["parent"] in a.links:
                    order.append(name)
                    self._component(name, c, frames, desc_of, opts)
                    placed.add(name)
                    del pending[name]
                    progress = True
            if not progress:
                raise ModelError("components whose parent link does not exist: " +
                                 ", ".join(f"{n} (parent {c['parent']!r})" for n, c in pending.items()))
        if opts.base == "planar":
            self._planar(robot, a)
        a.desc_frames = {d: l for d, l in desc_map.items() if l in a.links}
        ET.indent(robot, space="  ")
        a.urdf = '<?xml version="1.0"?>\n' + ET.tostring(robot, encoding="unicode") + "\n"
        return a

    def _add_link(self, link: str, comp: str, parent: Optional[str], joint: Optional[str], jtype: Optional[str],
                  T: np.ndarray, axis=None, mass: float = 0.0) -> None:
        if link in self._a.links:
            raise ModelError(f"duplicate link {link!r} (component {comp!r})")
        self._a.links[link] = {"component": comp, "parent": parent, "joint": joint, "type": jtype, "T": T,
                               "axis": axis, "mass": mass}

    def _add_joint(self, comp: str, el: ET.Element) -> None:
        lim = el.find("limit")
        ax = el.find("axis")
        g = lambda k: None if lim is None or lim.get(k) is None else float(lim.get(k))   # noqa: E731
        mimic = el.find("mimic")
        self._a.joints.append({"name": el.get("name"), "type": el.get("type"), "parent": el.find("parent").get("link"),
                               "child": el.find("child").get("link"), "component": comp,
                               "axis": None if ax is None else [float(x) for x in ax.get("xyz").split()],
                               "lower": g("lower"), "upper": g("upper"), "velocity": g("velocity"), "effort": g("effort"),
                               "mimic": None if mimic is None else mimic.get("joint")})

    def _mount_joint(self, name: str, c: dict, root: str, frames: dict, desc_of: dict) -> dict:
        if not c.get("parent"):
            return {"parent": None, "how": "root"}
        T, how = self._mount(name, c, frames, desc_of)
        el = fixed_joint(self._robot, f"{name}_mount", c["parent"], root, T)
        self._add_joint(name, el)
        self._a.links[root].update(parent=c["parent"], joint=f"{name}_mount", type="fixed", T=T)
        return {"parent": c["parent"], "T": T, "how": how}

    def _component(self, name: str, c: dict, frames: dict, desc_of: dict, opts: Options) -> None:
        src = c.get("source")
        variant = opts.variant_of(name)
        n_links = len(self._a.links)
        if src == "urdf":
            root = self._urdf(name, c, variant, opts)
        elif src == "procedural":
            kind = c.get("type")
            if kind not in PROCEDURAL:
                raise ModelError(f"component {name!r}: type must be one of {', '.join(PROCEDURAL)}, got {kind!r}")
            root = self._root_link(name, c)
            if kind == "swerve_base":
                self._swerve_base(name, c, root, opts)
            elif kind == "camera":
                self._camera(name, c, root, frames)
            else:
                self._primitive(name, c, root, kind)
        else:
            raise ModelError(f"component {name!r}: source must be urdf or procedural, got {src!r}")
        mount = self._mount_joint(name, c, root, frames, desc_of)
        links = list(self._a.links)[n_links:]
        info = {"name": name, "source": src, "type": c.get("type"), "file": c.get("file"), "root_link": root,
                "parent": mount["parent"], "variant": variant, "links": links,
                "joints": [j["name"] for j in self._a.joints if j["component"] == name and j["type"] != "fixed"],
                "mass": round(sum(self._a.links[l]["mass"] for l in links), 6),
                "mount_from": mount["how"], "doc": c.get("doc")}
        if mount["parent"] is not None:
            r, p, y = matrix_to_rpy(mount["T"][:3, :3])
            info["mount"] = {"xyz": np.round(mount["T"][:3, 3], 6).tolist(), "rpy": [round(v, 7) for v in (r, p, y)]}
        self._a.components.append(info)

    # -- urdf components --------------------------------------------------------------------------

    def _urdf(self, name: str, c: dict, variant: str, opts: Options) -> str:
        path = resolve_file(c["file"])
        prefix = c.get("prefix", f"{name}_")
        tree = ET.fromstring(path.read_text(encoding="utf-8-sig"))
        mesh_root = Path(c["mesh_root"]).resolve() if c.get("mesh_root") else path.parent.parent
        joints = {j.find("child").get("link"): j for j in tree.findall("joint")}
        root = None
        for el in tree:
            if el.tag == "material":
                if self._robot.find(f"material[@name='{el.get('name')}']") is None:
                    self._robot.append(copy.deepcopy(el))
            elif el.tag == "link":
                lk = copy.deepcopy(el)
                lname = el.get("name")
                lk.set("name", prefix + lname)
                for mesh in lk.iter("mesh"):
                    mesh.set("filename", self._mesh_uri(name, mesh.get("filename"), path, mesh_root, opts))
                if variant == "simple":
                    self._simplify(name, lk, el, path, mesh_root)
                self._robot.append(lk)
                if lname not in joints:
                    root = prefix + lname
                inert = el.find("inertial/mass")
                self._add_link(prefix + lname, name, None, None, None, np.eye(4),
                               mass=0.0 if inert is None else float(inert.get("value")))
        for el in tree.findall("joint"):
            j = copy.deepcopy(el)
            j.set("name", prefix + el.get("name"))
            for tag in ("parent", "child"):
                j.find(tag).set("link", prefix + el.find(tag).get("link"))
            if j.find("mimic") is not None:
                j.find("mimic").set("joint", prefix + j.find("mimic").get("joint"))
            self._robot.append(j)
            self._add_joint(name, j)
            o = el.find("origin")
            T = make_T(np.eye(3), (0, 0, 0)) if o is None else frame_T(
                {"xyz": [float(x) for x in (o.get("xyz") or "0 0 0").split()],
                 "rpy": [float(x) for x in (o.get("rpy") or "0 0 0").split()]})
            ax = el.find("axis")
            self._a.links[j.find("child").get("link")].update(
                parent=j.find("parent").get("link"), joint=j.get("name"), type=j.get("type"), T=T,
                axis=None if ax is None else [float(x) for x in ax.get("xyz").split()])
        if root is None:
            raise ModelError(f"{c['file']}: no root link")
        return root

    def _mesh_path(self, ref: str, urdf: Path, mesh_root: Path) -> Path:
        if ref.startswith("package://"):
            ref = ref[len("package://"):].split("/", 1)[1]
            p = mesh_root / ref
        elif ref.startswith("file://"):
            p = Path(ref[len("file://"):])
        else:
            p = (urdf.parent / ref) if not ref.startswith("meshes/") else (urdf.parent.parent / ref)
        p = p.resolve()
        if not p.is_file():
            raise ModelError(f"mesh not found: {ref} ({p})")
        return p

    def _mesh_uri(self, comp: str, ref: str, urdf: Path, mesh_root: Path, opts: Options) -> str:
        p = self._mesh_path(ref, urdf, mesh_root)
        try:
            rel = p.relative_to(mesh_root).as_posix()
        except ValueError:
            raise ModelError(f"mesh {ref} is outside its component's mesh root {mesh_root}") from None
        self._a.meshes[(comp, rel)] = p
        uri = {"package": f"package://{comp}/{rel}", "http": f"{opts.http_base}/model/meshes/{comp}/{rel}",
               "relative": f"meshes/{comp}/{rel}", "file": f"file://{p}"}[opts.mesh_uri]
        self._a.mesh_uris[(comp, rel)] = uri
        return uri

    def _simplify(self, comp: str, lk: ET.Element, orig: ET.Element, urdf: Path, mesh_root: Path) -> None:
        """Replace a link's geometry with one capsule bounding its collision meshes (link frame)."""
        pts = []
        for col in orig.findall("collision"):
            mesh = col.find("geometry/mesh")
            if mesh is None:
                continue
            o = col.find("origin")
            T = np.eye(4) if o is None else frame_T({"xyz": [float(x) for x in (o.get("xyz") or "0 0 0").split()],
                                                    "rpy": [float(x) for x in (o.get("rpy") or "0 0 0").split()]})
            scale = [float(x) for x in (mesh.get("scale") or "1 1 1").split()]
            P = _mesh_points(self._mesh_path(mesh.get("filename"), urdf, mesh_root), scale)
            with np.errstate(all="ignore"):     # spurious FP flags from Accelerate's matmul (macOS)
                pts.append(P @ T[:3, :3].T + T[:3, 3])
        for tag in ("visual", "collision"):
            for e in lk.findall(tag):
                lk.remove(e)
        if pts:
            p0, p1, r = fit_capsule(np.vstack(pts))
            capsule_els(lk, p0, p1, r, "ru_capsule", f"{lk.get('name')}_capsule")

    # -- procedural components ----------------------------------------------------------------------

    def _mass(self, name: str, c: dict, volume: float, key: str = "mass", density_key: str = "density") -> float:
        if c.get(key) is not None:
            m = float(c[key])
        elif c.get(density_key) is not None:
            m = float(c[density_key]) * volume
        else:
            raise ModelError(f"component {name!r}: give {key} or {density_key}")
        if not m > 0:
            raise ModelError(f"component {name!r}: mass must be positive")
        return m

    def _primitive(self, name: str, c: dict, root: str, kind: str) -> None:
        g = {k: c[k] for k in ("size", "radius", "length") if k in c}
        try:
            vol = primitive_volume(kind, g)
        except KeyError as e:
            raise ModelError(f"component {name!r}: {kind} needs {e.args[0]}") from None
        m = self._mass(name, c, vol)
        T = frame_T(c.get("origin") or {})
        lk = ET.SubElement(self._robot, "link", name=root)
        I = T[:3, :3] @ primitive_inertia(kind, m, g) @ T[:3, :3].T
        inertial_el(lk, m, T[:3, 3], I)
        mat = c.get("material", "ru_part")
        if kind == "capsule":
            h = g["length"] / 2
            capsule_els(lk, T[:3, :3] @ [0, 0, -h] + T[:3, 3], T[:3, :3] @ [0, 0, h] + T[:3, 3], g["radius"], mat, root)
        else:
            for tag in ("visual", "collision"):
                geom_el(lk, tag, kind, g, T, mat)
        self._add_link(root, name, None, None, None, np.eye(4), mass=m)

    def _camera(self, name: str, c: dict, root: str, frames: dict) -> None:
        size = c.get("size", [0.1, 0.03, 0.03])
        g = {"size": size}
        m = self._mass(name, c, primitive_volume("box", g))
        body = frame_T(c.get("body_origin") or {"xyz": [0.0, 0.0, -size[2] / 2]})
        lk = ET.SubElement(self._robot, "link", name=root)
        inertial_el(lk, m, body[:3, 3], body[:3, :3] @ primitive_inertia("box", m, g) @ body[:3, :3].T)
        for tag in ("visual", "collision"):
            geom_el(lk, tag, "box", g, body, "ru_camera")
        self._add_link(root, name, None, None, None, np.eye(4), mass=m)
        cam = (c.get("mount") or {}).get("camera")
        internal = {}
        if cam:   # the camera node's factory frames (color_optical, imu, ...), when it was reached
            for f, v in frames.items():
                if f.startswith(f"{cam}/") and f != f"{cam}/depth_optical" and v.get("parent", "").startswith(f"{cam}/"):
                    internal[f.split("/", 1)[1]] = (v["parent"].split("/", 1)[1], frame_T(v))
        defaults = c.get("frames") or {"color_optical": {"xyz": [0.0, 0.0, 0.0]}}
        for f, v in defaults.items():
            internal.setdefault(f, ("depth_optical", frame_T(v)))
        todo = dict(internal)
        while todo:
            moved = False
            for f, (parent, T) in list(todo.items()):
                plink = f"{name}_{parent}"
                if plink in self._a.links:
                    ET.SubElement(self._robot, "link", name=f"{name}_{f}")
                    self._add_link(f"{name}_{f}", name, None, None, None, np.eye(4))
                    el = fixed_joint(self._robot, f"{name}_{f}_joint", plink, f"{name}_{f}", T)
                    self._add_joint(name, el)
                    self._a.links[f"{name}_{f}"].update(parent=plink, joint=el.get("name"), type="fixed", T=T)
                    del todo[f]
                    moved = True
            if not moved:
                raise ModelError(f"camera {name!r}: frames with unknown parents: {sorted(todo)}")

    def _swerve_base(self, name: str, c: dict, root: str, opts: Options) -> None:
        """The chassis (lower frame tubes and top rails as two boxes) and four modules, each a steer
        joint (vertical, at the module's centre) and a wheel joint (its axle, y when the module points
        forward). Movable only with base=swerve; fixed otherwise."""
        try:
            S = float(c["frame_size"])
            z0, z1 = (float(v) for v in c["frame_z"])
            r0, r1 = (float(v) for v in c["top_rail_z"])
            R = float(c["wheel_diameter"]) / 2
            W = float(c.get("wheel_width", 0.0381))
            modules = c["modules"]
        except KeyError as e:
            raise ModelError(f"component {name!r}: swerve_base needs {e.args[0]}") from None
        boxes = [({"size": [S, S, z1 - z0]}, make_T(np.eye(3), (0, 0, (z0 + z1) / 2))),
                 ({"size": [S, S, r1 - r0]}, make_T(np.eye(3), (0, 0, (r0 + r1) / 2)))]
        mc = self._mass(name, c, 0.0, "chassis_mass", "_none")
        vols = [primitive_volume("box", g) for g, _ in boxes]
        parts = [(mc * v / sum(vols), T[:3, 3], primitive_inertia("box", mc * v / sum(vols), g)) for (g, T), v in zip(boxes, vols)]
        m, com, I = combine_inertias(parts)
        lk = ET.SubElement(self._robot, "link", name=root)
        inertial_el(lk, m, com, I)
        for g, T in boxes:
            for tag in ("visual", "collision"):
                geom_el(lk, tag, "box", g, T, "ru_chassis")
        self._add_link(root, name, None, None, None, np.eye(4), mass=m)
        movable = opts.base == "swerve"
        mm = self._mass(name, c, 0.0, "module_mass", "_none")
        mw = self._mass(name, c, 0.0, "wheel_mass", "_none")
        housing = {"size": [float(c.get("module_size", 0.1006)), float(c.get("module_size", 0.1006)), 0.04]}
        housing_T = make_T(np.eye(3), (0, 0, z0 - R + 0.02))
        wheel = {"radius": R, "length": W}
        wheel_T = make_T(np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]]), (0, 0, 0))   # cylinder z -> y
        for mod, (x, y) in modules.items():
            mod_link, wheel_link = f"{name}_{mod}_module", f"{name}_{mod}_wheel"
            ml = ET.SubElement(self._robot, "link", name=mod_link)
            inertial_el(ml, mm, housing_T[:3, 3], primitive_inertia("box", mm, housing))
            geom_el(ml, "visual", "box", housing, housing_T, "ru_module")
            T = make_T(np.eye(3), (float(x), float(y), R))
            el = fixed_joint(self._robot, f"{name}_{mod}_steer", root, mod_link, T,
                             "continuous" if movable else "fixed", axis=(0, 0, 1) if movable else None,
                             limit={"effort": float(c.get("steer_effort", 45.0)), "velocity": float(c.get("steer_velocity", 12.57))}
                             if movable else None)
            self._add_link(mod_link, name, root, el.get("name"), el.get("type"), T, (0, 0, 1) if movable else None, mm)
            self._add_joint(name, el)
            wl = ET.SubElement(self._robot, "link", name=wheel_link)
            inertial_el(wl, mw, (0, 0, 0), wheel_T[:3, :3] @ primitive_inertia("cylinder", mw, wheel) @ wheel_T[:3, :3].T)
            for tag in ("visual", "collision"):
                geom_el(wl, tag, "cylinder", wheel, wheel_T, "ru_wheel")
            el = fixed_joint(self._robot, f"{name}_{mod}_wheel_joint", mod_link, wheel_link, np.eye(4),
                             "continuous" if movable else "fixed", axis=(0, 1, 0) if movable else None,
                             limit={"effort": float(c.get("wheel_effort", 18.3)), "velocity": float(c.get("wheel_velocity", 140.0))}
                             if movable else None)
            self._add_link(wheel_link, name, mod_link, el.get("name"), el.get("type"), np.eye(4),
                           (0, 1, 0) if movable else None, mw)
            self._add_joint(name, el)

    def _planar(self, robot: ET.Element, a: Assembly) -> None:
        """base=planar: odom -> x (prismatic) -> y (prismatic) -> yaw (continuous) -> base_link."""
        chain = [("odom", None), ("base_x_link", ("base_x", "prismatic", (1, 0, 0))),
                 ("base_y_link", ("base_y", "prismatic", (0, 1, 0))), (ROOT_LINK, ("base_yaw", "continuous", (0, 0, 1)))]
        prev = None
        for link, j in chain:
            if link != ROOT_LINK:
                lk = ET.Element("link", name=link)
                if link != "odom":
                    inertial_el(lk, TINY_MASS, (0, 0, 0), np.eye(3) * 1e-8)
                robot.insert(len(MATERIALS), lk)
                a.links[link] = {"component": "base", "parent": None, "joint": None, "type": None, "T": np.eye(4),
                                 "axis": None, "mass": 0.0 if link == "odom" else TINY_MASS}
            if j is not None:
                el = fixed_joint(robot, j[0], prev, link, np.eye(4), j[1], axis=j[2],
                                 limit={"lower": -1000.0, "upper": 1000.0, "effort": 1000.0, "velocity": 10.0}
                                 if j[1] == "prismatic" else {"effort": 1000.0, "velocity": 10.0})
                self._add_joint("base", el)
                a.links[link].update(parent=prev, joint=j[0], type=j[1], T=np.eye(4), axis=list(j[2]))
            prev = link
        a.root = "odom"

    # -- version ----------------------------------------------------------------------------------

    def files(self) -> list[Path]:
        out = []
        a = self.build(Options(mesh_uri="relative"))
        for c in self.components.values():
            if c.get("source") == "urdf":
                out.append(resolve_file(c["file"]))
        out += sorted(set(a.meshes.values()))
        return out

    def version(self) -> str:
        """A hash of the description (components, frames, cameras), the active calibrations, the camera
        nodes' factory frames and every component file (URDF and meshes)."""
        cam_frames = {k: v for k, v in self.frames_cal.items() if v.get("source") == "camera node (factory)"}
        active = {k: v for k, v in self.active.items() if v}        # a camera with none active = no entry
        blob = json.dumps({"description": self.desc, "active": active, "camera_frames": cam_frames,
                           "files": {str(p): file_hash(p) for p in self.files()}},
                          sort_keys=True, default=str)
        return hashlib.sha256(blob.encode()).hexdigest()[:16]


# -- public views -----------------------------------------------------------------------------------

def _pose(T: np.ndarray) -> dict:
    r, p, y = matrix_to_rpy(T[:3, :3])
    return {"xyz": [round(float(v), 9) for v in T[:3, 3]], "rpy": [round(float(v), 9) for v in (r, p, y)]}


def model_public(a: Assembly, version: str) -> dict:
    """GET /model: components, the frame tree (links, each with its parent and joint) and the joints."""
    frames = {}
    for link, v in a.links.items():
        f = {"parent": v["parent"], "joint": v["joint"], "type": v["type"], "component": v["component"],
             "mass": round(float(v["mass"]), 9)}
        if v["parent"] is not None:
            f.update(_pose(v["T"]))
            if v["axis"] is not None:
                f["axis"] = [float(x) for x in v["axis"]]
        frames[link] = f
    base = ROOT_LINK if ROOT_LINK in a.links else a.root
    in_base = {link: np.round(a.T(base, link), 9).tolist() for link in a.links}
    joints = [{k: j[k] for k in ("name", "type", "parent", "child", "component", "axis", "lower", "upper",
                                 "velocity", "effort", "mimic")}
              for j in a.joints if j["type"] != "fixed"]
    return {"model_version": version, "root": a.root, "base_link": base, "options": a.options.public(),
            "mass": round(a.mass(), 6), "components": a.components, "frames": frames, "frames_in_base": in_base,
            "joints": joints, "description_frames": a.desc_frames,
            "units": {"length": "m", "angle": "rad", "mass": "kg", "inertia": "kg m^2"},
            "conventions": {"base_link": "the description's base frame: floor level, centre of the wheels; x forward, "
                                         "y left, z up",
                            "transform": "a frame's pose in its parent, p_parent = R p_child + t at zero joint "
                                         "positions; rpy = fixed-axis X, Y, Z (URDF), rad",
                            "names": "links of a component are prefixed with its name; a description frame "
                                     "<camera>/<frame> is the link <component>_<frame>"}}


def bundle(a: Assembly, version: str, model_json: dict) -> bytes:
    """A zip: robot.urdf (meshes relative: meshes/<component>/<path>), the meshes it uses, model.json."""
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("robot.urdf", a.urdf)
        z.writestr("model.json", json.dumps(model_json, indent=1))
        used = {k for k, uri in a.mesh_uris.items() if uri in a.urdf}
        for (comp, rel) in sorted(used):
            z.write(a.meshes[(comp, rel)], str(PurePosixPath("meshes", comp, rel)))
    return buf.getvalue()
