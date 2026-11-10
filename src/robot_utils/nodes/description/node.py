"""Description node core: the robot's frame tree, its calibrations, and calibration jobs.

The pose of a camera's ``depth_optical`` in ``base`` is composed from:
- the **nominal** pose (default file, or measured and set with ``set_nominal``): x, y always, and
  everything else until a calibration provides it;
- the active **ground** calibration: the floor normal (camera frame) <-> base z, and the height z;
- the active **yaw** calibration: each motion direction (camera frame) <-> its base vector.
Rotation = Wahba over those pairs, with the nominal axes as a very weak prior (so a ground-only
calibration keeps the nominal heading). The camera's internal frames (color_optical, imu) are
taken from the camera node's ``GET /description`` and hang under ``<camera>/depth_optical``.
"""
from __future__ import annotations

import logging
import math
import threading
import time
import traceback
from typing import Callable, Optional

import numpy as np

from ..common.client import NodeClient
from ..common.estop import EStop
from ..common.hub import RowHub
from ..common.server import EventLog
from .frames import FrameTree, describe_T, frame_T, make_T, matrix_to_rpy
from .routines import Aborted, GroundParams, HttpCameraIO, HttpSwerveIO, YawParams, run_ground, run_yaw
from .solvers import angle_deg, solve_rotation
from .store import Store

log = logging.getLogger("description")

W_GROUND, W_LINE, W_PRIOR = 10.0, 1.0, 1e-3


def target(cam: dict) -> str:
    """Where a camera node is: ``url`` if given (remote / debugging), else its instance name ``node``."""
    return cam.get("url") or cam.get("node") or "camera"

# A run is always saved; it only becomes active (with activate=true) if it passes these.
QUALITY = {"ground_rms_max": 0.02, "ground_inliers_min": 2000,
           "yaw_linearity_max": 0.3, "yaw_sign_corr_min": 0.5, "yaw_angle_error_max_deg": 3.0}


def ground_quality(meta: dict) -> list[str]:
    r, q, out = meta["result"], QUALITY, []
    if r["rms"] > q["ground_rms_max"]:
        out.append(f"plane rms {r['rms'] * 1000:.1f} mm > {q['ground_rms_max'] * 1000:.0f} mm")
    if r["inliers"] < q["ground_inliers_min"]:
        out.append(f"{r['inliers']} inliers < {q['ground_inliers_min']}")
    return out


def yaw_quality(meta: dict) -> list[str]:
    q, out = QUALITY, []
    for i, v in enumerate(meta["vectors"]):
        if v["linearity"] > q["yaw_linearity_max"]:
            out.append(f"vector {i}: linearity {v['linearity']:.2f} > {q['yaw_linearity_max']} (not a clean line)")
        if (v["sign_corr"] or 0) < q["yaw_sign_corr_min"]:
            out.append(f"vector {i}: sign correlation {v['sign_corr'] or 0:.2f} < {q['yaw_sign_corr_min']}")
    for k, c in meta["checks"].items():
        if k.startswith("angle_") and abs(c["error_deg"]) > q["yaw_angle_error_max_deg"]:
            out.append(f"{k}: measured {c['measured_deg']:.1f}° vs {c['expected_deg']:.1f}° expected")
    return out
EZ = np.array([0.0, 0.0, 1.0])


def compose_camera(nominal: dict, ground: Optional[dict], yaw: Optional[dict]) -> tuple[np.ndarray, dict]:
    """4x4 pose of depth_optical in base, and how it was obtained."""
    Tn = frame_T(nominal)
    Rn = Tn[:3, :3]
    pairs = [(Rn.T @ np.array(e), e, W_PRIOR) for e in ([1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0])]
    used = {"nominal": True}
    if ground:
        pairs.append((np.asarray(ground["result"]["normal"]), EZ, W_GROUND))
        used["ground"] = ground["id"]
    if yaw:
        for v in yaw["vectors"]:
            pairs.append((np.asarray(v["direction_depth"]), np.array(v["vector"] + [0.0]), W_LINE))
        used["yaw"] = yaw["id"]
    if len(pairs) > 3:
        R, info = solve_rotation(pairs)
        used["residual_deg"] = info["residual_deg"][3:]
    else:
        R = Rn
    t = Tn[:3, 3].copy()
    if ground:
        t[2] = float(ground["result"]["height"])
    return make_T(R, t), used


class Job:
    def __init__(self, jid: int, kind: str, params: dict):
        self.id, self.kind, self.params = jid, kind, params
        self.state = "running"          # running | done | failed | aborted
        self.progress: list[tuple[float, str]] = []
        self.result: Optional[dict] = None
        self.error: Optional[str] = None
        self.t_start = time.monotonic()
        self.t_end: Optional[float] = None
        self.abort_reason: Optional[str] = None
        self.raw_path = None            # an aborted run's recording (Path), if any

    def info(self) -> dict:
        return {"id": self.id, "kind": self.kind, "state": self.state, "params": self.params,
                "progress": [m for _, m in self.progress[-20:]], "result": self.result, "error": self.error,
                "raw": f"jobs/{self.id}/raw.npz" if self.raw_path is not None and self.raw_path.exists() else None,
                "elapsed": round((self.t_end or time.monotonic()) - self.t_start, 2)}


class DescriptionNode:
    def __init__(self, data_dir: str, swerve_url: Optional[str] = "swerve",
                 camera_io: Optional[Callable[[str], object]] = None,
                 swerve_io: Optional[Callable[[str], object]] = None):
        self.store = Store(data_dir)
        self.swerve_url = swerve_url
        self.camera_io = camera_io or HttpCameraIO
        self.swerve_io = swerve_io or HttpSwerveIO
        self.hub = RowHub(1)                   # used for state change notifications (SSE)
        self.events = EventLog(on_add=self.hub.state_changed)
        self.estop = EStop(on_trigger=self._on_estop)
        self.lock = threading.Lock()
        self.jobs: dict[int, Job] = {}
        self.job: Optional[Job] = None         # the running one
        self._next_id = 1
        self.cams: dict[str, dict] = {}        # camera name -> {"description", "error", "t"}
        self._stop = threading.Event()
        self._poller = threading.Thread(target=self._poll_cameras, name="camera-poll", daemon=True)

    # -- lifecycle -----------------------------------------------------------------------------

    def start(self):
        self.events.add(f"description node started (data: {self.store.dir})")
        self._poller.start()

    def close(self):
        self._stop.set()
        self.abort("node shutting down")

    def _poll_cameras(self):
        while not self._stop.is_set():
            for name, cam in (self.store.description().get("cameras") or {}).items():
                try:
                    with NodeClient(target(cam), timeout=2.0) as c:
                        d = c.get("/description")
                    with self.lock:
                        prev = self.cams.get(name, {}).get("description")
                        self.cams[name] = {"description": d, "error": None, "t": time.monotonic()}
                    if prev is None or (prev.get("device") or {}).get("serial") != (d.get("device") or {}).get("serial"):
                        self.events.add(f"camera {name!r}: serial {(d.get('device') or {}).get('serial')}")
                        self.hub.state_changed()
                except Exception as e:
                    with self.lock:
                        c = self.cams.setdefault(name, {"description": None, "t": None})
                        c["error"] = f"{type(e).__name__}: {e}"
            self._stop.wait(5.0)

    # -- description ---------------------------------------------------------------------------

    def serial(self, name: str) -> Optional[str]:
        with self.lock:
            d = (self.cams.get(name) or {}).get("description")
        if d and d.get("device"):
            return d["device"]["serial"]
        return (self.store.description().get("cameras", {}).get(name) or {}).get("last_serial")

    def active_calibrations(self, serial: Optional[str]) -> dict:
        out = {}
        for kind, cid in (self.store.active().get(serial or "", {}) or {}).items():
            try:
                out[kind] = self.store.get(kind, serial, cid)
            except KeyError:
                pass
        return out

    def camera_pose(self, name: str) -> tuple[np.ndarray, dict]:
        cam = self.store.description()["cameras"][name]
        act = self.active_calibrations(self.serial(name))
        return compose_camera(cam["nominal"], act.get("ground"), act.get("yaw"))

    def frames(self) -> dict:
        """The full frame map (base, links, each camera's depth_optical and its internal frames)."""
        d = self.store.description()
        frames = {k: dict(v) for k, v in (d.get("frames") or {}).items()}
        for name in d.get("cameras") or {}:
            T, used = self.camera_pose(name)
            r, p, y = matrix_to_rpy(T[:3, :3])
            frames[f"{name}/depth_optical"] = {"parent": "base", "xyz": T[:3, 3].tolist(), "rpy": [r, p, y],
                                               "source": used}
            with self.lock:
                cd = (self.cams.get(name) or {}).get("description")
            for fname, f in ((cd or {}).get("frames") or {}).items():
                if f.get("parent"):
                    frames[f"{name}/{fname}"] = {"parent": f"{name}/{f['parent']}", "rotation": f["rotation"],
                                                 "translation": f["translation"], "source": "camera node (factory)"}
        return frames

    def description(self) -> dict:
        d = self.store.description()
        cams = {}
        for name, cam in (d.get("cameras") or {}).items():
            T, used = self.camera_pose(name)
            with self.lock:
                c = dict(self.cams.get(name) or {})
            cams[name] = {"node": target(cam), "serial": self.serial(name), "nominal": cam["nominal"],
                          "pose_in_base": describe_T(T), "from": used, "reachable": c.get("error") is None and bool(c.get("description")),
                          "error": c.get("error"), "intrinsics": (c.get("description") or {}).get("intrinsics")}
        frames = self.frames()
        tree = FrameTree(frames)
        in_base = {}
        for f in frames:
            try:
                in_base[f] = np.round(tree.T("base", f), 9).tolist()
            except (KeyError, ValueError):        # a frame not under base (bad override): skip
                pass
        from ..swerve.constants import FRAME, MODULES
        robot = {"frame_size": FRAME, "modules": [{"name": m.name, "x": m.x, "y": m.y} for m in MODULES]}
        return {"frames": frames, "frames_in_base": in_base, "robot": robot, "cameras": cams,
                "conventions": {"base": "floor level, centre of the wheels; x forward, y left, z up",
                                "optical": "x right, y down, z out of the lens",
                                "transform": "p_parent = R p_child + t; rpy = fixed-axis X, Y, Z (URDF), rad"}}

    def transform(self, a: str, b: str) -> dict:
        return describe_T(FrameTree(self.frames()).T(a, b))

    def set_nominal(self, name: str, xyz: list, rpy: list) -> dict:
        if name not in (self.store.description().get("cameras") or {}):
            raise KeyError(f"unknown camera {name!r}")
        self.store.set_override(["cameras", name, "nominal"], {"xyz": [float(v) for v in xyz], "rpy": [float(v) for v in rpy]})
        self.events.add(f"camera {name!r}: nominal pose set to xyz {xyz}, rpy {rpy}")
        return self.description()["cameras"][name]

    # -- calibrations --------------------------------------------------------------------------

    def activate(self, kind: str, serial: str, cid: Optional[str]):
        self.store.activate(kind, serial, cid)
        self.events.add(f"{kind} calibration {cid} active for camera {serial}" if cid else
                        f"no active {kind} calibration for camera {serial}")

    def _start_job(self, kind: str, params: dict, fn) -> dict:
        with self.lock:
            if self.job is not None and self.job.state == "running":
                raise RuntimeError(f"job {self.job.id} ({self.job.kind}) is still running")
            if kind == "yaw" and self.estop.active:
                raise RuntimeError("the description node's e-stop is latched; clear it first")
            job = Job(self._next_id, kind, params)
            self._next_id += 1
            self.jobs[job.id] = job
            self.job = job
            for old in sorted(self.jobs)[:-20]:
                del self.jobs[old]

        def progress(m):
            job.progress.append((time.monotonic(), m))
            self.hub.state_changed()

        def run():
            try:
                job.result = fn(job, progress)
                job.state = "done"
                self.events.add(f"job {job.id} ({kind}) done: {job.result.get('summary', '')}")
            except Aborted as e:
                if e.raw:               # saved before the state flips, so a final state has its recording
                    try:
                        job.raw_path = self.store.save_aborted(kind, job.id, e.raw)
                    except Exception:
                        log.exception("could not save the aborted run's recording")
                job.state, job.error = "aborted", str(e)
                self.events.add(f"job {job.id} ({kind}) aborted: {e}"
                                + (f" (recording: {job.raw_path.name})" if job.raw_path else ""), "warn")
            except Exception as e:
                job.state, job.error = "failed", f"{type(e).__name__}: {e}"
                log.error("job %s failed\n%s", job.id, traceback.format_exc())
                self.events.add(f"job {job.id} ({kind}) failed: {job.error}", "error")
            finally:
                job.t_end = time.monotonic()
                self.hub.state_changed()

        threading.Thread(target=run, name=f"job-{job.id}", daemon=True).start()
        self.events.add(f"job {job.id} ({kind}) started")
        return job.info()

    def _camera(self, name: str):
        cams = self.store.description().get("cameras") or {}
        if name not in cams:
            raise KeyError(f"unknown camera {name!r}")
        return self.camera_io(target(cams[name]))

    def start_ground(self, camera: str, params: GroundParams, activate: bool = True) -> dict:
        def fn(job, progress):
            cam = self._camera(camera)
            meta, raw = run_ground(cam, params, progress)
            serial = meta["camera"]["device"]["serial"]
            self.store.set_override(["cameras", camera, "last_serial"], serial)
            bad = ground_quality(meta)
            meta["quality"] = {"ok": not bad, "problems": bad}
            cid = self.store.save("ground", serial, {**meta, "camera_name": camera}, raw, activate=activate and not bad)
            return {"id": cid, "serial": serial, "summary": meta["summary"], "activated": activate and not bad,
                    "quality": meta["quality"], **self._ground_checks(camera, meta)}
        return self._start_job("ground", {"camera": camera, **params.__dict__}, fn)

    def _ground_checks(self, camera: str, meta: dict) -> dict:
        out = {"result": meta["result"]}
        act = self.active_calibrations(meta["camera"]["device"]["serial"]).get("yaw")
        if act and act.get("floor_normal_depth"):
            out["angle_to_yaw_floor_normal_deg"] = angle_deg(meta["result"]["normal"], act["floor_normal_depth"])
        return out

    def start_yaw(self, camera: str, params: YawParams, activate: bool = True) -> dict:
        from .routines import validate_yaw
        validate_yaw(params)                  # 400 before anything moves
        if not self.swerve_url:
            raise RuntimeError("no swerve node configured (--swerve)")

        def fn(job, progress):
            cam = self._camera(camera)
            swerve = self.swerve_io(self.swerve_url)
            meta, raw = run_yaw(cam, swerve, params, should_abort=lambda: job.abort_reason, progress=progress)
            self.store.set_override(["cameras", camera, "last_serial"], meta["camera"]["device"]["serial"])
            return self._save_yaw(camera, meta, raw, activate)
        return self._start_job("yaw", {"camera": camera, **{k: v for k, v in params.__dict__.items()}}, fn)

    def _save_yaw(self, camera: str, meta: dict, raw: dict, activate: bool) -> dict:
        serial = meta["camera"]["device"]["serial"]
        ground = self.active_calibrations(serial).get("ground")
        if ground and meta.get("floor_normal_depth"):
            meta["checks"]["floor_normal_vs_ground_deg"] = angle_deg(meta["floor_normal_depth"], ground["result"]["normal"])
        bad = yaw_quality(meta)
        meta["quality"] = {"ok": not bad, "problems": bad}
        cid = self.store.save("yaw", serial, {**meta, "camera_name": camera}, raw, activate=activate and not bad)
        return {"id": cid, "serial": serial, "summary": meta["summary"], "activated": activate and not bad,
                "quality": meta["quality"], "checks": meta["checks"], "accel_lowpass_hz": meta.get("accel_lowpass_hz"),
                "vectors": [{k: v.get(k) for k in ("vector", "direction_depth", "linearity", "linearity_unfiltered",
                                                   "sign_corr", "amplitude", "vibration_rms", "yaw_rate_max_dps", "n")}
                            for v in meta["vectors"]]}

    def resolve_yaw(self, serial: str, cid: str, accel_lowpass_hz: Optional[float] = None,
                    activate: bool = True) -> dict:
        """Solve a stored yaw run again from its raw data (current solver; nothing moves): a new record."""
        from .routines import depth_to_imu_rotation, solve_yaw
        old = self.store.get("yaw", serial, cid)
        path = self.store.raw_path("yaw", serial, cid)
        if not path.exists():
            raise KeyError(f"yaw calibration {cid!r} has no raw data")
        with np.load(path) as z:
            raw = {k: z[k] for k in z.files}
        I = {k[4:]: v for k, v in raw.items() if k.startswith("imu_")}
        T = {k[7:]: v for k, v in raw.items() if k.startswith("swerve_")}
        prm = {k: v for k, v in old["params"].items() if k in YawParams.__dataclass_fields__}
        prm["accel_lowpass_hz"] = accel_lowpass_hz
        p = YawParams(**prm)
        vecs = [[float(x) for x in v] for v in p.vectors]
        windows = [(vecs[i], float(ts), float(te)) for i, (ts, te) in enumerate(raw["windows"])]
        desc = old["camera"]
        meta, raw2 = solve_yaw(I, T, windows, vecs, p, desc, depth_to_imu_rotation(desc), old.get("gravity_up_imu"))
        meta["resolved_from"] = cid
        meta["summary"] += f" (re-solved from {cid})"
        res = self._save_yaw(old.get("camera_name", "camera"), meta, raw2, activate)
        self.events.add(f"yaw {cid} re-solved as {res['id']}: {meta['summary']}"
                        + ("" if res["activated"] else " (not activated: " + "; ".join(res["quality"]["problems"]) + ")"))
        return res

    def abort(self, reason: str = "abort requested") -> Optional[dict]:
        with self.lock:
            job = self.job
        if job is None or job.state != "running":
            return None
        job.abort_reason = reason
        return job.info()

    def _on_estop(self, info):
        self.abort(f"e-stop: {info.reason}")
        self.events.add(f"E-STOP ({info.source}): {info.reason}", "error")
        if self.swerve_url:
            try:                               # belt and braces: stop the base directly too
                with NodeClient(self.swerve_url, timeout=1.0) as c:
                    c.post("/estop", {"reason": f"via description node: {info.reason}", "source": "description node"})
            except Exception as e:
                self.events.add(f"could not forward the e-stop to the swerve node: {e}", "error")

    # -- state ---------------------------------------------------------------------------------

    def state(self) -> dict:
        with self.lock:
            job = self.job
            cams = {k: {"reachable": v.get("error") is None and bool(v.get("description")), "error": v.get("error"),
                        "serial": ((v.get("description") or {}).get("device") or {}).get("serial")}
                    for k, v in self.cams.items()}
        e = self.estop.info
        running = job is not None and job.state == "running"
        from ..common.locks import LockStore, lock_state
        if not hasattr(self, "_locks"):
            self._locks = LockStore()
        return {"state": "calibrating" if running else "ready",
                "error": None, "job": None if job is None else job.info(),
                "estop": {"latched": e is not None, "reason": e and e.reason, "source": e and e.source, "t": e and e.t},
                "lease": {"held": False, "holder": None},
                "cameras": cams, "active": self.store.active(), "events": self.events.list(),
                # the yaw routine drives through the swerve: its chocks apply here too
                **lock_state(self._locks, chock={"swerve", "description"}, cover={"camera"})}
