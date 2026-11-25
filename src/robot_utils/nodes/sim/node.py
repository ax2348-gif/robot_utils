"""The sim node: the world, its cameras and the backend sockets (docs/sim.md).

No lease and no e-stop (decided): it substitutes for the hardware, it doesn't command it. The nodes
that command (arm, swerve, cameras) keep their own leases and e-stops and reach the world through
their backends over ``protocol`` sockets: ``arm.sock``, ``swerve.sock``, ``camera-<camera>.sock``.
"""
from __future__ import annotations

import logging
import os
import tempfile
import time
from typing import Optional

import yaml

from ..common.hub import RowHub
from ..common.server import EventLog
from .protocol import IpcServer, backend_socket
from . import viz
from .limits import describe as describe_limits
from .render import Cameras
from .world import SimError, World, unzip_bundle

log = logging.getLogger("sim")


def fetch_bundle(source: str, variant: str = "full") -> tuple[bytes, str]:
    """GET /model/bundle.zip (base=swerve) from ``source``: a description node (instance name or URL), a
    bundle .zip file, or ``local`` (assembled here from the shipped description, nominal camera poses).
    Returns (zip bytes, where it came from)."""
    params = {"variant": variant, "base": "swerve", "mounts": "calibrated"}
    if source == "local":
        from ..description.model import Options
        from ..description.node import DescriptionNode
        data, v = DescriptionNode(tempfile.mkdtemp(prefix="sim-desc-"), None).model_bundle(
            Options.parse(variant, base="swerve"))
        return data, f"local description (model {v})"
    if source.endswith(".zip") and os.path.exists(source):
        with open(source, "rb") as f:
            return f.read(), source
    from ..common.client import NodeClient
    with NodeClient(source, timeout=30.0) as c:
        r = c.http.get("/model/bundle.zip", params=params)
        if r.status_code != 200:
            raise RuntimeError(f"{source}: GET /model/bundle.zip: {r.status_code} {r.text[:200]}")
        return r.content, f"description node {source} (model {r.headers.get('x-model-version')})"


def description_of(source: str) -> Optional[dict]:
    """The description node's GET /description (None for a bundle or ``local``)."""
    if source == "local" or source.endswith(".zip"):
        return None
    from ..common.client import NodeClient
    with NodeClient(source, timeout=10.0) as c:
        return c.get("/description")


def apply_camera_serials(config: dict, description: Optional[dict]) -> dict:
    """Make each sim camera report the robot's serial for that camera name (the description's ``last_serial``,
    from the robot's snapshot), so the description node applies that camera's active calibrations unchanged.
    Without one the sim config's serial stays (no calibration matches it: nominal mounts). Returns {name: serial}."""
    from .snapshot import camera_serials
    cams = config.get("cameras") or {}
    serials = camera_serials(description or {}, cams)
    for name, s in serials.items():
        cams[name]["serial"] = s
    return serials


def world_from(source: str, config: dict) -> World:
    data, how = fetch_bundle(source, (config.get("model") or {}).get("variant", "full"))
    urdf, assets, info = unzip_bundle(data)
    try:
        desc = description_of(source)
    except Exception as e:  # noqa: BLE001 - the model came; the serials fall back to the config's
        log.warning("description %s: GET /description failed (%s): sim camera serials from the config", source, e)
        desc = None
    serials = apply_camera_serials(config, desc)
    mounts = {name: c.get("from") for name, c in ((desc or {}).get("cameras") or {}).items()}
    log.info("camera serials %s; camera mounts %s", serials, mounts or "nominal (no description node)")
    return World(urdf, assets, config, {"source": how, "model_version": info.get("model_version"),
                                        "camera_serials": serials, "camera_mounts": mounts})


class SimNode:
    def __init__(self, world: World, name: str = "sim", scene_dir: Optional[str] = None):
        self.world = world
        self.name = name
        self.cameras = Cameras(world, world.cfg.get("cameras") or {})
        self.scene_dir = scene_dir or os.path.expanduser("~/.local/share/robot_utils/sim")
        self.hub = RowHub(1)
        self.events = EventLog(on_add=self.hub.state_changed)
        self.servers: dict = {}
        self.t_start = time.monotonic()
        self.markers = viz.Markers()
        self.ghosts = viz.Ghosts()
        self.view = {"style": "hidden_line", "role": "auto"}   # how headsets draw /scene3d (GET/POST /scene3d/view)
        self._ghost_lines = None             # (scene_id, rigid.Poses, compat.StateLines) for robot_state's ghosts

    # -- lifecycle ---------------------------------------------------------------------------------

    def start(self) -> None:
        w = self.world
        self.servers["arm"] = IpcServer(backend_socket(self.name, "arm"), self._arm, on_close=self._arm_closed)
        self.servers["swerve"] = IpcServer(backend_socket(self.name, "swerve"), self._swerve)
        for cam in self.cameras.cams:
            self.servers[f"camera-{cam}"] = IpcServer(backend_socket(self.name, f"camera-{cam}"),
                                                      lambda h, p, c, cam=cam: self._camera(cam, h, p, c))
        if (w.cfg.get("time") or {}).get("run", True):
            w.run()
        self.events.add(f"sim world {self.name!r} started ({w.model_info.get('source', 'model')}; "
                        f"{'running' if w.running else 'paused'}); backend sockets in "
                        f"{backend_socket(self.name, 'arm').parent}")

    def close(self) -> None:
        for s in self.servers.values():
            s.close()
        self.cameras.close()
        self.world.close()

    # -- backends ------------------------------------------------------------------------------------

    def _arm(self, h: dict, payload: bytes, conn):
        op = h.get("op")
        if op == "enable":
            conn.state["armed"] = True
        out = self.world.arm_op(op, h)
        if op in ("enable", "disable"):
            self.events.add(f"arm motors {'on' if op == 'enable' else 'off'} (arm backend)")
        return out

    def _arm_closed(self, conn) -> None:
        if conn.state.get("armed"):
            self.world.arm_lost()
            self.events.add("arm backend disconnected: arm motors off (status 0xD)", "warn")

    def _swerve(self, h: dict, payload: bytes, conn):
        return self.world.swerve_op(h.get("op"), h)

    def _camera(self, cam: str, h: dict, payload: bytes, conn):
        op = h.get("op")
        if op == "info":
            return self.cameras.info(cam)
        if op == "start":
            self.cameras.start(cam, h.get("streams") or [])
            return {"streams": sorted(self.cameras.cam(cam).streams)}
        if op == "stop":
            self.cameras.stop(cam)
            return {}
        if op == "frames":
            metas, data = self.cameras.frames(cam, h.get("after") or {}, float(h.get("wait", 0.05)))
            return {"frames": metas}, data
        if op == "imu":
            rows = self.world.imu_samples(cam, int(h.get("after") or 0), min(1.0, float(h.get("wait", 0.05))))
            return {"samples": rows}
        raise SimError(f"unknown camera op {op!r}")

    # -- scene files -----------------------------------------------------------------------------------

    def save_scene(self, name: str) -> dict:
        from .world import SimError as E
        if not name.replace("-", "").replace("_", "").isalnum():
            raise E("a scene name is letters, digits, - and _")
        sc = self.world.scene()
        os.makedirs(self.scene_dir, exist_ok=True)
        path = os.path.join(self.scene_dir, f"{name}.yaml")
        objs = [{k: v for k, v in o.items() if k not in ("speed", "joints")} for o in sc["objects"]]
        with open(path, "w") as f:
            yaml.safe_dump({"objects": objs}, f, sort_keys=False)
        self.events.add(f"scene saved: {path} ({len(objs)} objects)")
        return {"name": name, "path": path, "objects": objs}

    def load_scene(self, name: Optional[str] = None, objects: Optional[list] = None) -> dict:
        if objects is None:
            path = os.path.join(self.scene_dir, f"{name}.yaml")
            if not os.path.exists(path):
                raise KeyError(f"no saved scene {name!r}")
            with open(path) as f:
                objects = (yaml.safe_load(f) or {}).get("objects") or []
        out = self.world.load_scene(objects)
        self.events.add(f"scene loaded{f' {name!r}' if name else ''}: {len(objects)} objects")
        return out

    # -- 3D view -----------------------------------------------------------------------------------------

    def frames(self) -> set:
        """Names a marker's frame may be: model bodies and scene objects (besides world / base)."""
        with self.world.lock:
            m = self.world.m
            return {m.body(i).name for i in range(m.nbody)} | set(self.world.objects)

    def put_markers(self, markers: list, source: Optional[str] = None, replace: bool = False) -> dict:
        names = self.markers.put(markers, self.frames(), source, replace)
        return {"names": names, "count": len(self.markers.list())}

    def viz_frame(self, markers_version: Optional[int] = None) -> dict:
        """Body poses now; the markers too unless the caller already has this version of them."""
        out = viz.frame(self.world)
        mk = self.markers.list()
        out["markers_version"] = self.markers.version
        if markers_version != self.markers.version:
            out["markers"] = mk
        return out

    VIEW_STYLES = ("hidden_line", "wire", "flat_edges", "flat")
    VIEW_ROLES = ("auto", "visual", "collision")

    def set_view(self, style: Optional[str] = None, role: Optional[str] = None, next_style: bool = False) -> dict:
        if next_style:
            i = self.VIEW_STYLES.index(self.view["style"])
            style = self.VIEW_STYLES[(i + 1) % len(self.VIEW_STYLES)]
        if style is not None:
            if style not in self.VIEW_STYLES:
                raise ValueError(f"style: one of {', '.join(self.VIEW_STYLES)}")
            self.view["style"] = style
        if role is not None:
            if role not in self.VIEW_ROLES:
                raise ValueError(f"role: one of {', '.join(self.VIEW_ROLES)}")
            self.view["role"] = role
        self.events.add(f"headset view: {self.view['style']}, {self.view['role']} geometry")
        self.hub.state_changed()
        return dict(self.view)

    def put_ghosts(self, ghosts: list, source: Optional[str] = None, replace: bool = False) -> dict:
        with self.world.lock:
            m = self.world.m
            bodies = {m.body(i).name for i in range(1, m.nbody)}
        if replace:
            self.ghosts.remove(source=source)
        names = self.ghosts.put(ghosts, bodies, source)
        return {"names": names, "count": len(self.ghosts.list())}

    def ghost_objects(self) -> dict:
        """The live ghosts as robot_state.v1 objects (their bodies' primitives; meshes as boxes), so today's AR view
        draws them: ``ghost:<name>/<body>``."""
        decl = self.ghosts.declared()
        if not decl:
            return {}
        from ...scene3d import compat
        from . import rigid
        index, _ = rigid.build(self.world, ghosts=decl)
        if self._ghost_lines is None or self._ghost_lines[0] != index["scene_id"]:
            self._ghost_lines = (index["scene_id"], rigid.Poses(index), compat.StateLines(index))
        _, poses, lines = self._ghost_lines
        line = lines.line(poses.frame(self.world, self.ghosts.live()))
        return {k: {**v, "fixed": False} for k, v in line["objects"].items() if k.startswith("ghost:")}

    def robot_state(self) -> dict:
        """One robot_state.v1 record (docs/sim.md, "Robot state stream")."""
        s = viz.robot_state(self.world)
        s["objects"].update(self.ghost_objects())
        sim = s.pop("sim")
        self._state_seq = getattr(self, "_state_seq", 0) + 1
        return {"schema": viz.STATE_SCHEMA, "source": "sim", "seq": self._state_seq, "t": s["t"], "clock": "sim",
                "reset_seq": self.world.reset_seq,
                "t_host": round(time.monotonic(), 4), "model_version": s["model_version"], "frame": "world",
                "base": s["base"], "joints": s["joints"], "objects": s["objects"], "overlays": self.overlays(sim)}

    def overlays(self, sim: Optional[dict] = None) -> dict:
        """robot_state.v1's overlays: the live markers, the sim's run state, the arm motors as the world sees them."""
        w = self.world
        arm = w.arm if w.has_arm else None
        return {"markers": self.markers.list(), "sim": sim or {"running": w.running, "rtf": w.rtf},
                **({"arm": {"enabled": arm.enabled, "fault": arm.fault}} if arm is not None else {})}

    # -- state -------------------------------------------------------------------------------------------

    def state(self) -> dict:
        w = self.world
        snap = w.snapshot()
        cams = {}
        for name, c in self.cameras.cams.items():
            active = time.monotonic() < c.active_until and bool(c.streams)
            cams[name] = {"active": active, "streams": sorted(c.streams), "rendered": dict(c.rendered),
                          "ms_per_frame": {s: round(1000 * c.render_s[s] / c.rendered[s], 2) if c.rendered[s] else None
                                           for s in c.rendered}}
        return {"state": "running" if snap["running"] else "paused", "error": self.cameras.error,
                "model": {"source": w.model_info.get("source"), "model_version": w.model_info.get("model_version"),
                          "camera_serials": w.model_info.get("camera_serials") or {},
                          "camera_mounts": w.model_info.get("camera_mounts") or {}},
                "limits": describe_limits(w.limits),
                "fidelity": {"base": "hybrid (kinematic modules, dynamic chassis)",
                             "arm": w.cfg["components"]["arm"].get("fidelity", "kinematic")},
                **snap, "cameras": cams,
                "backends": {k: {"socket": str(s.path), "clients": s.clients, "requests": s.requests}
                             for k, s in self.servers.items()},
                "view": dict(self.view),
                "events": self.events.list()}
