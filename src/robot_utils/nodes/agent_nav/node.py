"""agent_nav core: the agent <-> robot boundary (no web code).

Principles (docs/agent_nav.md):
- Perception and planning calls are READ-ONLY: they return data, proposals and overlay images.
- Motion happens only when the agent (or a human on the page) explicitly issues a goal / stop.
  No call ingests perception and moves the base on its own.
- Stopping is the only automatic action allowed: the stop reflex brakes (pose_servo /stop) when
  depth shows an obstacle inside the braking distance ahead of the motion.
- Every call is logged as a dialogue turn (caller -> robot reply), with an optional comment.

Peers (instance names or URLs): pose_servo (motion), 3d_perception (registered colour + depth),
estimator (pose stream, for the pose at each snapshot's capture time).
"""
from __future__ import annotations

import base64
import bisect
import io
import json
import logging
import math
import os
import queue
import struct
import threading
import time
from collections import OrderedDict, deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from ..common.client import NodeClient, NodeError
from ..common.lease import RequestError
from ..common.server import EventLog
from ..estimator.node import Upstream
from . import geometry as G

log = logging.getLogger("agent_nav")

DATA_DIR = os.path.expanduser("~/.local/share/robot_utils/agent_nav")
KEEP_SNAPSHOTS = 30          # full-resolution snapshots kept in memory (thumbnails are stored)
KEEP_IMAGES = 200            # rendered overlay images kept in memory
KEEP_LOG = 1000              # dialogue turns kept in memory (all are in the session's log.jsonl)
POSE_HISTORY_S = 30.0
REFLEX_HZ = 6.0
REFLEX_POINTS = 25           # obstacle points ahead needed to trigger the reflex
REFLEX_LATENCY = 0.35        # s: depth age + reaction, added to the braking distance
REFLEX_MARGIN = 0.12         # m: default gap left in front of what stops the reflex (beyond braking + latency)
REFLEX_MARGIN_MAX = 1.0      # m: a goal may set its margin anywhere in (0, this]


def stopping_distance(v: float, accel: float, jerk: float, tick_s: float = 1.0 / REFLEX_HZ) -> float:
    """m travelled from the moment an obstacle enters the zone until the base stands still:
    detection (up to one reflex tick) + latency (depth age, reaction) at speed v, then pose_servo's
    jerk-limited braking (v²/2a, plus v·a/2j while the deceleration ramps up). The reflex margin
    is added on top of this, so a small margin still stops before what it sees."""
    return v * (tick_s + REFLEX_LATENCY) + v * v / (2 * accel) + v * accel / (2 * jerk)
MAX_AVATAR = 64 * 1024       # bytes (data URI)
CHECK_SNAPSHOTS = 6          # path checks also use this many recent snapshots (coverage, obstacles)
STALE_MOVE_M = 0.10          # a snapshot is stale once the robot moved this far / turned this much since
STALE_TURN_RAD = 0.35
STALE_AGE_S = 300.0

# Safety profiles (server side). An agent may lower values, never raise them; the human picks the profile.
PROFILES = {
    "cautious": {"max_speed": 0.15, "max_omega": 0.5, "max_accel": 0.2, "max_alpha": 0.8, "max_jerk": 0.8,
                 "max_ang_jerk": 3.0, "max_distance": 2.0, "max_rotation": 3.2},
    "normal": {"max_speed": 0.3, "max_omega": 0.8, "max_accel": 0.5, "max_alpha": 1.5, "max_jerk": 2.0,
               "max_ang_jerk": 6.0, "max_distance": 3.0, "max_rotation": 3.2},
}
GOAL_OPTIONS = ("mode", "pos_tol", "heading_tol", "on_reach", "hold_band", "speed", "omega", "accel", "alpha",
                "jerk", "ang_jerk", "min_wheel_speed", "stall_s")


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Snapshot:
    sid: str
    t: float                     # capture time (colour), this host's clock
    pose: Optional[tuple]        # base pose in the world at capture (estimator, interpolated)
    reset_id: Optional[float]
    jpeg: bytes
    depth: np.ndarray
    cam: G.Camera
    T_bc: Optional[np.ndarray]
    mode: str
    moving: bool
    _cloud: Optional[G.Cloud] = None

    @property
    def cloud(self) -> G.Cloud:
        if self._cloud is None:
            self._cloud = G.Cloud(self.depth, self.cam, self.T_bc, self.pose)
        return self._cloud


@dataclass
class GoalRec:
    gid: int                     # pose_servo goal id
    request: dict
    resolved: dict
    check: dict
    issued_by: str
    t: float
    result: Optional[dict] = None
    done: threading.Event = field(default_factory=threading.Event)


class AgentNav:
    def __init__(self, pose_servo: str = "pose_servo", perception: str = "3d_perception",
                 estimator: str = "estimator", data_dir: str = DATA_DIR, profile: str = "cautious",
                 require_imu: bool = True, description: Optional[str] = "description"):
        self.targets = {"pose_servo": pose_servo, "perception": perception, "estimator": estimator,
                        "description": description}
        self.data_dir = data_dir
        self.events = EventLog()
        self.lock = threading.RLock()
        self.on_change = None                   # set by the server: wakes /events subscribers
        self.stop_ev = threading.Event()
        self.q: queue.Queue = queue.Queue(maxsize=5000)
        self.est = Upstream("estimator", estimator, "/stream", self.q, self.events) if estimator else None
        self.poses: deque = deque()             # (t, x, y, theta, reset_id)
        self.profile = profile
        self.require_imu = require_imu          # passed to pose_servo (False only for simulation)
        self.session: Optional[dict] = None
        self.turns: deque = deque(maxlen=KEEP_LOG)
        self.n_turn = 0
        self.snapshots: OrderedDict[str, Snapshot] = OrderedDict()
        self.images: OrderedDict[str, tuple[bytes, str]] = OrderedDict()
        self.queries: dict[str, dict] = {}
        self.goals: dict[int, GoalRec] = {}
        self.memory = {"notes": [], "places": {}}
        self.latch: Optional[dict] = None       # human disable / e-stop: the agent cannot re-enable
        self.enabled_by: Optional[str] = None
        self.enabled_at = 0.0                   # monotonic s when our last /enable returned (watcher ignores older polls)
        self.caps: Optional[dict] = None
        self.reflex = {"enabled": True, "armed": False, "last": None, "blind": None, "stops": 0,
                       "margin": REFLEX_MARGIN, "tick_errors": 0}
        self._reflex_tick_s = 1.0 / REFLEX_HZ      # measured interval between reflex checks (EMA)
        self._n_snap = self._n_img = self._n_q = 0
        self._ps_status = None
        self._threads = [threading.Thread(target=self._read_poses, name="poses", daemon=True),
                         threading.Thread(target=self._watch, name="watch", daemon=True),
                         threading.Thread(target=self._reflex_loop, name="reflex", daemon=True)]

    # -- clients ------------------------------------------------------------------------------

    def _client(self, key: str, timeout: float = 5.0) -> NodeClient:
        t = self.targets.get(key)
        if not t:
            raise RequestError(503, f"no {key} node configured")
        return NodeClient(t, timeout=timeout)

    def _call(self, key: str, method: str, path: str, body=None, timeout: float = 5.0, **params):
        try:
            with self._client(key, timeout) as c:
                return c.get(path, **params) if method == "GET" else c.post(path, body if body is not None else {}, **params)
        except NodeError as e:
            raise RequestError(e.status if e.status < 500 else 502, f"{key}: {e.detail}")
        except RequestError:
            raise
        except Exception as e:
            raise RequestError(503, f"{key} unreachable: {type(e).__name__}: {e}")

    # -- lifecycle ----------------------------------------------------------------------------

    def start(self):
        if self.est is not None:
            self.est.start()
        for t in self._threads:
            t.start()
        self.events.add("agent_nav started")

    def close(self):
        self.stop_ev.set()
        if self.est is not None:
            self.est.stop.set()

    # -- poses --------------------------------------------------------------------------------

    def _read_poses(self):
        while not self.stop_ev.is_set():
            try:
                _, rows, offset = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            names = rows.dtype.names
            with self.lock:
                for r in rows:
                    rid = float(r["reset_id"]) if "reset_id" in names else 0.0
                    if self.poses and self.poses[-1][4] != rid:
                        self.poses.clear()
                        self._robot_turn("estimator reset: the world frame moved; snapshot-relative goals and "
                                         "places from before are no longer valid", level="warn")
                    self.poses.append((float(r["t"]) - offset, float(r["x"]), float(r["y"]), float(r["theta"]), rid))
                while self.poses and self.poses[-1][0] - self.poses[0][0] > POSE_HISTORY_S:
                    self.poses.popleft()

    def pose_at(self, t: float) -> Optional[tuple]:
        """(x, y, theta, reset_id) interpolated at time t (None if not covered within 0.2 s)."""
        with self.lock:
            P = list(self.poses)
        if not P:
            return None
        ts = [p[0] for p in P]
        i = bisect.bisect_left(ts, t)
        if i == 0:
            return P[0][1:] if ts[0] - t < 0.2 else None
        if i == len(P):
            return P[-1][1:] if t - ts[-1] < 0.2 else None
        a, b = P[i - 1], P[i]
        f = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
        return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * (b[3] - a[3]), b[4])

    def pose_now(self) -> Optional[tuple]:
        with self.lock:
            p = self.poses[-1] if self.poses else None
        if p is None or time.monotonic() - p[0] > 0.5:
            return None
        return p[1:]

    # -- dialogue log ---------------------------------------------------------------------------

    def _session_dir(self) -> Optional[str]:
        if self.session is None:
            return None
        d = os.path.join(self.data_dir, self.session["id"])
        os.makedirs(os.path.join(d, "thumbs"), exist_ok=True)
        return d

    def turn(self, actor: str, kind: str, request: dict, reply: dict, comment: str = "",
             images: Optional[list] = None, level: str = "info") -> dict:
        """Log one exchange (caller request + robot reply); thumbnails of its images are stored."""
        with self.lock:
            self.n_turn += 1
            t = {"n": self.n_turn, "session": (self.session or {}).get("id"), "t": time.monotonic(), "wall": time.time(),
                 "actor": actor, "kind": kind,
                 "request": request, "reply": reply, "comment": comment or "", "images": images or [],
                 "level": level}
            d = self._session_dir()
            if d:
                thumbs = []
                for name in t["images"]:
                    img = self.images.get(name)
                    if img is not None and img[1] == "image/jpeg" or (img is not None and img[1] == "image/png"):
                        fn = f"{self.n_turn:05d}_{name.split('.')[0]}.jpg"
                        try:
                            with open(os.path.join(d, "thumbs", fn), "wb") as f:
                                f.write(G.thumbnail(_as_jpeg(img), 320))
                            thumbs.append(fn)
                        except Exception as e:      # never let logging break a call
                            log.warning("thumbnail: %s", e)
                t["thumbs"] = thumbs
                with open(os.path.join(d, "log.jsonl"), "a") as f:
                    f.write(json.dumps(t, default=_json_default) + "\n")
            self.turns.append(t)
        if self.on_change:
            self.on_change()
        return t

    def _robot_turn(self, msg: str, level: str = "info", **data):
        self.turn("robot", "event", {}, {"message": msg, **data}, level=level)

    def store_image(self, data: bytes, kind: str, ext: str) -> str:
        with self.lock:
            self._n_img += 1
            name = f"{kind}-{self._n_img}.{ext}"
            self.images[name] = (data, "image/png" if ext == "png" else "image/jpeg")
            while len(self.images) > KEEP_IMAGES:
                self.images.popitem(last=False)
            return name

    # -- session / ownership ----------------------------------------------------------------------

    def open_session(self, agent: str, avatar: str = "", comment: str = "", actor: str = "agent") -> dict:
        if avatar.startswith("data:") and len(avatar) > MAX_AVATAR:
            raise ValueError(f"avatar data URI larger than {MAX_AVATAR} bytes")
        if not avatar.startswith("data:") and len(avatar) > 16:
            raise ValueError("avatar: an emoji / short text (<= 16 chars) or a data:image URI")
        with self.lock:
            if self.session is not None:
                self.close_session("replaced by a new session", actor)
            sid = time.strftime("%Y%m%d-%H%M%S")
            self.session = {"id": sid, "agent": agent or "agent", "avatar": avatar or "🤖", "started": time.time(),
                            "profile": self.profile}
            self.memory = {"notes": [], "places": {}}
            self.goals.clear()
            d = self._session_dir()
            with open(os.path.join(d, "session.json"), "w") as f:
                json.dump(self.session, f)
        out = {"session": self.session, "profile": self.profile, "caps": PROFILES[self.profile],
               "guide": "guide.md", "robot": "GET robot (body, camera pose and field of view, blind zone, diagram)"}
        try:
            c = self.describe_robot(actor, log_turn=False)["camera"]
            out["camera_summary"] = (f"camera {c.get('height_m')} m up, pitched {c.get('pitch_down_deg')}° down, "
                                     f"fov {c.get('hfov_deg')}°x{c.get('vfov_deg')}°, nearest visible floor "
                                     f"{c.get('nearest_visible_floor_m')} m from the base centre")
        except Exception:
            pass
        self.turn(actor, "session", {"agent": agent, "avatar": avatar[:16]}, out, comment)
        return out

    def close_session(self, reason: str = "closed", actor: str = "agent", comment: str = "") -> dict:
        with self.lock:
            s = self.session
        if s is None:
            return {"closed": False}
        if self.enabled_by is not None:
            self.disable(actor, comment="session closed")
        self.turn(actor, "session_end", {"reason": reason}, {"closed": s["id"]}, comment)
        with self.lock:
            try:
                with open(os.path.join(self.data_dir, s["id"], "session.json"), "w") as f:
                    json.dump({**s, "ended": time.time(), "turns": sum(1 for t in self.turns if t.get("session") == s["id"])}, f)
            except OSError as e:
                log.warning("session.json: %s", e)
            self.session = None
        return {"closed": s["id"]}

    def _need_session(self, actor: str):
        if actor == "agent" and self.session is None:
            raise RequestError(409, "open a session first: POST /session {agent, avatar}")

    def enable(self, actor: str, caps: Optional[dict] = None, comment: str = "") -> dict:
        self._need_session(actor)
        if self.latch is not None and actor != "human":
            raise RequestError(423, f"held by the human operator ({self.latch['why']}); ask them to release it")
        base = PROFILES[self.profile]
        cfg = dict(base)
        for k, v in (caps or {}).items():
            if k not in base:
                raise ValueError(f"unknown cap {k!r}; caps: {sorted(base)}")
            if v is None:
                continue
            if not (0 < float(v) <= base[k]):
                raise ValueError(f"{k} must be within (0, {base[k]}] (profile {self.profile!r}; only the human can raise it)")
            cfg[k] = float(v)
        name = (self.session or {}).get("agent", "human") if actor == "agent" else "human"
        ps = self._call("pose_servo", "GET", "/state")
        adopted = None
        if ps.get("enabled"):
            holder = ps.get("holder") or ""
            if not holder.startswith("agent_nav"):
                raise RequestError(409, f"pose_servo is enabled by {holder!r} (not agent_nav): they must disable it first")
            if ps.get("status") not in ("idle", "holding"):
                raise RequestError(409, f"pose_servo is {ps.get('status')} under {holder!r}: stop it first")
            self._call("pose_servo", "POST", "/disable", {})     # idle: re-enable with this caller's caps
            adopted = holder
        out = self._call("pose_servo", "POST", "/enable", {"client": f"agent_nav: {name}",
                                                           "config": {**cfg, "require_imu": self.require_imu}}, timeout=8)
        with self.lock:
            self.enabled_by, self.caps = actor, cfg
            self.enabled_at = time.monotonic()
            if actor == "human":
                self.latch = None
        reply = {"enabled": True, "profile": self.profile, "caps": cfg}
        if adopted:
            reply["adopted"] = f"pose_servo was already enabled by {adopted!r} (idle): re-enabled with these caps"
        self.turn(actor, "enable", {"caps": caps or {}}, reply, comment)
        return {**reply, "pose_servo": out.get("config")}

    def disable(self, actor: str, comment: str = "") -> dict:
        try:
            self._call("pose_servo", "POST", "/disable", {})
        finally:
            with self.lock:
                self.enabled_by = None
                if actor == "human":
                    self.latch = {"why": "disabled on the page", "t": time.time()}
        self.turn(actor, "disable", {}, {"enabled": False, "latched": self.latch is not None}, comment)
        return {"enabled": False, "latched": self.latch is not None}

    def release(self, comment: str = "") -> dict:
        with self.lock:
            was, self.latch = self.latch, None
        self.turn("human", "release", {}, {"released": was is not None}, comment)
        return {"released": was is not None}

    def stop(self, actor: str, comment: str = "") -> dict:
        out = self._call("pose_servo", "POST", "/stop", {})
        self.turn(actor, "stop", {}, out, comment)
        return out

    def estop(self, actor: str, reason: str = "", comment: str = "") -> dict:
        out = self._call("pose_servo", "POST", "/estop", {"reason": f"agent_nav ({actor}): {reason}", "source": actor})
        with self.lock:
            self.enabled_by = None
            if actor == "human":
                self.latch = {"why": "e-stop from the page", "t": time.time()}
        self.turn(actor, "estop", {"reason": reason}, out, comment, level="warn")
        return out

    # -- robot description (read-only) --------------------------------------------------------------

    def describe_robot(self, actor: str = "agent", comment: str = "", log_turn: bool = True) -> dict:
        """The body, the camera (where it is, where it points, what it sees), frames and limits."""
        from ..swerve.constants import MODULES
        from ..pose_servo.planner import STEER_RATE_MAX, WHEEL_SPEED_MAX
        cal = self._call("perception", "GET", "/calibration")
        desc, desc_err = None, None
        try:
            desc = self._call("description", "GET", "/description", timeout=3)
        except RequestError as e:
            desc_err = str(e)
        wheels = (desc or {}).get("robot", {}).get("modules") or [{"name": m.name, "x": m.x, "y": m.y} for m in MODULES]
        T_bc = np.array(cal["base_to_color"]) if cal.get("base_to_color") else None
        camera = {"calibrated": T_bc is not None}
        img = None
        if T_bc is not None:
            cam = G.Camera.from_color(cal["color"], cal["color"]["width"] // 2, cal["color"]["height"] // 2)
            facts = G.camera_facts(cam, T_bc)
            img = self.store_image(G.robot_diagram(cam, T_bc, wheels, facts), "robot", "png")
            camera.update({k: v for k, v in facts.items() if not k.startswith("_")})
            camera["base_to_color"] = [[round(float(v), 6) for v in row] for row in T_bc]
        dc = ((desc or {}).get("cameras") or {}).get("camera") or {}
        if dc:
            camera["calibration"] = {"source": dc.get("from"), "serial": dc.get("serial")}
        caps = self.caps or PROFILES[self.profile]
        out = {
            "frames": {
                "base": "floor level at the centre of the wheels; x forward, y left, z up (m); angles CCW from x",
                "world": "the estimator's frame: the base pose at its last reset; drifts slowly (odometry + IMU heading)",
                "image": f"colour pixels (u right, v down), {cal['color']['width']}x{cal['color']['height']}, origin top left",
                "color_optical": "x right, y down, z forward (m), at the camera",
                "snapshot:<id>": "the base frame at that snapshot's capture time",
            },
            "body": {"footprint_m": [round(2 * G.HALF, 3), round(2 * G.HALF, 3)], "shape": "square, centred on the base origin",
                     "drive": "swerve (holonomic): it can translate in any direction while turning, and turn in place",
                     "wheels": [{"name": w["name"], "x": round(w["x"], 4), "y": round(w["y"], 4)} for w in wheels],
                     "turn_in_place_sweep_radius_m": round(math.hypot(G.HALF, G.HALF), 3),
                     "height_note": "low base (~0.17 m) with a camera mast; the mast is the tallest part"},
            "camera": camera,
            "limits": {"profile": self.profile, "caps": caps, "wheel_speed_max_m_s": WHEEL_SPEED_MAX,
                       "steer_rate_max_rad_s": round(STEER_RATE_MAX, 3)},
            "sensing_notes": [
                "only the forward camera: nothing beside or behind the robot is seen; turn to look",
                "depth is weak on glass, mirrors, black / shiny surfaces and thin legs or cables",
                "the stop reflex sees only what the camera sees, beyond the blind zone",
            ],
            "diagram": f"images/{img}" if img else None,
        }
        if desc_err:
            out["warnings"] = [f"description node unavailable ({desc_err}); body from built-in constants"]
        if log_turn:
            self.turn(actor, "robot", {}, {k: out[k] for k in ("body", "camera", "diagram")}, comment, [img] if img else [])
        return out

    # -- perception (read-only) -------------------------------------------------------------------

    def look(self, actor: str, scale: Optional[float] = None, grid: bool = False, settle: bool = False,
             comment: str = "") -> dict:
        self._need_session(actor)
        if settle:
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline and self._servo_moving():
                time.sleep(0.1)
        cal = self._call("perception", "GET", "/calibration")
        restore = None
        if scale is not None:
            if not (0.25 <= scale <= 1.0):
                raise ValueError("scale must be within [0.25, 1.0] (1.0 = depth at 1280x720)")
            cur = self._call("perception", "GET", "/settings").get("scale")
            if cur is not None and abs(cur - scale) > 1e-6:
                self._call("perception", "POST", "/settings", {"scale": scale})
                restore = cur
        try:
            h, payload = self._frame(cal, want_w=None if scale is None else int(round(cal["color"]["width"] * scale)))
        finally:
            if restore is not None:
                try:
                    self._call("perception", "POST", "/settings", {"scale": restore})
                except RequestError as e:
                    self.events.add(f"could not restore 3d_perception scale {restore}: {e}", "warn")
        c, d = h["color"], h["depth"]
        jpeg = payload[c["offset"]: c["offset"] + c["len"]]
        if d.get("offset") is None:
            raise RequestError(503, f"3d_perception gave no depth (mode {h.get('mode')})")
        depth = np.frombuffer(payload, "<u2", d["width"] * d["height"], d["offset"]).reshape(d["height"], d["width"]).copy()
        cam = G.Camera.from_color(cal["color"], d["width"], d["height"])
        T_bc = np.array(cal["base_to_color"]) if cal.get("base_to_color") else None
        p = self.pose_at(h["t_color"])
        moving = self._servo_moving()
        with self.lock:
            self._n_snap += 1
            sid = f"s{self._n_snap}"
            snap = Snapshot(sid, h["t_color"], p[:3] if p else None, p[3] if p else None, jpeg, depth, cam, T_bc,
                            h.get("mode", "?"), moving)
            self.snapshots[sid] = snap
            while len(self.snapshots) > KEEP_SNAPSHOTS:
                self.snapshots.popitem(last=False)
        img = self.store_image(G.overlay_image(jpeg, grid=True, label=sid) if grid else jpeg, sid, "jpg")
        bev = self.store_image(G.bev_image(snap.cloud, label=f"{sid} BEV (base frame, 0.5 m grid)"), sid + "_bev", "png")
        warnings = []
        if T_bc is None:
            warnings.append("camera pose in base not calibrated: no base/world coordinates")
        if p is None:
            warnings.append("no estimator pose at capture: no world coordinates, no snapshot-frame goals")
        if moving:
            warnings.append("the robot was moving (motion-compensated registration; geometry less exact)")
        out = {"snapshot": sid, "image": f"images/{img}", "bev": f"images/{bev}",
               "image_size": [cam.color_w, cam.color_h], "depth_size": [cam.width, cam.height],
               "pose_world": None if p is None else [round(v, 4) for v in p[:3]], "mode": snap.mode,
               "depth_fraction": round(float((depth > 0).mean()), 3), "moving": moving, "warnings": warnings,
               "grid_image": f"images/{sid}/grid.jpg"}
        self.turn(actor, "look", {"scale": scale, "grid": grid, "settle": settle}, out, comment, [img, bev])
        return out

    def _frame(self, cal, want_w=None):
        deadline = time.monotonic() + 6
        while True:
            try:
                with self._client("perception", 8) as c:
                    for h, payload, _ in c.records("/frame", max_age=0.3, timeout=3):
                        if h.get("type") == "fused":
                            if want_w is None or h["depth"]["width"] == want_w:
                                return h, payload
                            break
            except NodeError as e:
                raise RequestError(502, f"3d_perception: {e.detail}")
            except Exception as e:
                raise RequestError(503, f"3d_perception unreachable: {type(e).__name__}: {e}")
            if time.monotonic() > deadline:
                raise RequestError(504, "3d_perception did not deliver a frame at the requested scale")
            time.sleep(0.1)

    def snapshot(self, sid: Optional[str]) -> Snapshot:
        with self.lock:
            if sid is None:
                if not self.snapshots:
                    raise RequestError(409, "no snapshot yet: POST /look")
                return next(reversed(self.snapshots.values()))
            s = self.snapshots.get(sid)
        if s is None:
            raise RequestError(404, f"snapshot {sid!r} unknown or expired (the last {KEEP_SNAPSHOTS} are kept)")
        return s

    def grid_image(self, sid: str) -> bytes:
        s = self.snapshot(sid)
        return G.overlay_image(s.jpeg, grid=True, label=sid)

    def query(self, actor: str, kind: str, snapshot: Optional[str], select: Optional[dict], exclude: list,
              comment: str = "", points: Optional[list] = None) -> dict:
        self._need_session(actor)
        s = self.snapshot(snapshot)
        if select is None and not (kind == "line" and points):
            raise ValueError("give a selection (or, for a line, 'points')")
        if select is not None:
            G.check_shape(select, s.cam)
        for e in exclude or []:
            G.check_shape(e, s.cam)
        mask = G.selection_mask(select, exclude, s.cam) if select is not None else None
        plane = line = None
        if kind == "region":
            res = G.region_stats(s.cloud, mask)
        elif kind == "plane":
            res = G.fit_plane(s.cloud, mask)
            plane = res
        elif kind == "line":
            res = G.fit_line(s.cloud, mask, points_px=points if select is None else None)
            line = res
        else:
            raise ValueError(kind)
        inl = res.pop("_inlier_mask", None)
        res.pop("_ends_base", None)
        with self.lock:
            self._n_q += 1
            qid = f"q{self._n_q}"
        shapes = ([(select, G.SEL_COLOR)] if select is not None else
                  [({"point": p}, G.SEL_COLOR) for p in points or []]) + [(e, G.WARN_COLOR) for e in exclude or []]
        ov = self.store_image(G.overlay_image(s.jpeg, shapes=shapes, plane=plane, cam=s.cam, T_bc=s.T_bc, line=line,
                                              label=f"{qid} {kind} on {s.sid}"), qid, "jpg")
        bev = self.store_image(G.bev_image(s.cloud, selection=mask, plane=plane, line=line,
                                           label=f"{qid} {kind}: selection outlined (magenta)"), qid + "_bev", "png")
        out = {"query": qid, "kind": kind, "snapshot": s.sid, **res, "overlay": f"images/{ov}", "bev": f"images/{bev}"}
        with self.lock:
            self.queries[qid] = {"snapshot": s.sid, "kind": kind, "select": select, "exclude": exclude, "result": res}
        self.turn(actor, f"query_{kind}", {"snapshot": s.sid, "select": select, "exclude": exclude,
                                            **({"points": points} if points else {})}, out, comment,
                  [ov, bev], level="warn" if (res.get("warning") or res.get("warnings")) else "info")
        return out

    # -- planning (read-only) ---------------------------------------------------------------------

    def resolve(self, frame: str, x: float, y: float, theta: float) -> dict:
        """A goal in some frame -> world pose + the literal turn from the current pose."""
        now = self.pose_now()
        if now is None:
            raise RequestError(503, "no current estimator pose")
        cx, cy, cth, crid = now
        if frame == "local":
            c, s = math.cos(cth), math.sin(cth)
            g = (cx + c * x - s * y, cy + s * x + c * y, cth + theta)
        elif frame == "global":
            g = (x, y, cth + wrap(theta - cth))
        elif frame.startswith("snapshot:"):
            snap = self.snapshot(frame.split(":", 1)[1])
            if snap.pose is None:
                raise RequestError(409, f"{snap.sid} has no pose (no estimator at capture)")
            if snap.reset_id != crid:
                raise RequestError(409, f"the estimator was reset since {snap.sid}: its frame is gone")
            px, py, pth = snap.pose
            c, s = math.cos(pth), math.sin(pth)
            g = (px + c * x - s * y, py + s * x + c * y, pth + theta)
        elif frame.startswith("place:"):
            name = frame.split(":", 1)[1]
            pl = self.memory["places"].get(name)
            if pl is None:
                raise RequestError(404, f"no place {name!r}")
            if pl["reset_id"] != crid:
                raise RequestError(409, f"place {name!r} was saved before an estimator reset")
            g = (pl["x"], pl["y"], cth + wrap(pl["theta"] - cth) if pl.get("theta") is not None else cth)
        else:
            raise ValueError("frame: local | global | snapshot:<id> | place:<name>")
        lc, ls = math.cos(cth), math.sin(cth)
        dx, dy = g[0] - cx, g[1] - cy
        local = (lc * dx + ls * dy, -ls * dx + lc * dy, g[2] - cth)
        return {"world": [round(v, 4) for v in g], "local": [round(v, 4) for v in local],
                "from_world": [round(v, 4) for v in (cx, cy, cth)], "distance": round(math.hypot(dx, dy), 4)}

    def _check(self, resolved: dict, snapshot: Optional[str]) -> tuple[dict, Optional[Snapshot]]:
        """Path check against the given (or latest) snapshot plus the recent ones in the same world
        frame (coverage and obstacles from all of them). STALE only when the robot moved since that
        snapshot, or it is minutes old: agents take their time between a look and a goal."""
        try:
            s = self.snapshot(snapshot)
        except RequestError:
            return {"checked": False, "warnings": ["UNCHECKED: no snapshot to check the path against (POST /look)"]}, None
        with self.lock:
            recent = [o for o in list(self.snapshots.values())[-CHECK_SNAPSHOTS:]
                      if o is not s and o.pose is not None and o.reset_id == s.reset_id]
        chk = G.path_check(s.cloud, tuple(resolved["from_world"]), tuple(resolved["world"]),
                           others=[o.cloud for o in recent])
        chk["against_snapshot"] = s.sid
        chk["also_used"] = [o.sid for o in recent]
        age = time.monotonic() - s.t
        chk["snapshot_age_s"] = round(age, 1)
        now = self.pose_now()
        if s.pose is not None and now is not None:
            moved = math.hypot(now[0] - s.pose[0], now[1] - s.pose[1])
            turned = abs(now[2] - s.pose[2])
            chk["moved_since_snapshot"] = {"m": round(moved, 3), "rad": round(turned, 3)}
            if moved > STALE_MOVE_M or turned > STALE_TURN_RAD:
                chk.setdefault("warnings", []).append(
                    f"STALE: the robot moved {moved:.2f} m / {turned:.2f} rad since {s.sid}; look again before trusting it")
        if age > STALE_AGE_S:
            chk.setdefault("warnings", []).append(f"STALE: {s.sid} is {age / 60:.0f} min old; the scene may have changed")
        return chk, s

    def _check_images(self, chk: dict, s: Optional[Snapshot], resolved: dict, tag: str) -> list:
        if s is None or not chk.get("checked"):
            return []
        px, py, pth = s.pose
        gx, gy, gth = resolved["world"]
        c, sn = math.cos(pth), math.sin(pth)
        gb = (c * (gx - px) + sn * (gy - py), -sn * (gx - px) + c * (gy - py), gth - pth)
        bev = self.store_image(G.bev_image(s.cloud, path=chk, goal_base=gb,
                                           label=f"{tag}: path check on {s.sid} (green clear, red blocked, grey unseen)"),
                               tag + "_bev", "png")
        return [bev]

    def plan_check(self, actor: str, frame: str, x: float, y: float, theta: float, snapshot: Optional[str] = None,
                   comment: str = "") -> dict:
        self._need_session(actor)
        r = self.resolve(frame, x, y, theta)
        chk, s = self._check(r, snapshot)
        imgs = self._check_images(chk, s, r, f"check{self.n_turn + 1}")
        out = {"goal": r, "check": _public(chk), "bev": f"images/{imgs[0]}" if imgs else None}
        self.turn(actor, "plan_check", {"frame": frame, "x": x, "y": y, "theta": theta}, out, comment, imgs,
                  level="warn" if chk.get("warnings") else "info")
        return out

    def plan_approach(self, actor: str, query: Optional[str], standoff: float, face: bool = True,
                      comment: str = "") -> dict:
        """A proposed goal in front of a fitted plane (or a region's nearest point). Moves nothing."""
        self._need_session(actor)
        q = self.queries.get(query or "")
        if q is None:
            raise RequestError(404, f"query {query!r} unknown: run /query/plane or /query/region first")
        res = q["result"]
        s = self.snapshot(q["snapshot"])
        if s.pose is None:
            raise RequestError(409, f"{s.sid} has no pose")
        if q["kind"] == "plane":
            cx, cy, _ = res["base"]["centre"]
            nx, ny, _ = res["base"]["normal"]
            nn = math.hypot(nx, ny) or 1.0
            nx, ny = nx / nn, ny / nn
            gx, gy = cx + standoff * nx, cy + standoff * ny
            gth = math.atan2(-ny, -nx) if face else 0.0
        else:
            if "base" not in res:
                raise RequestError(409, "the region has no base-frame points")
            tx, ty, _ = res["base"]["nearest_xy"]
            d = math.hypot(tx, ty)
            if d > standoff:
                gx, gy = tx * (1 - standoff / d), ty * (1 - standoff / d)
            else:
                gx = gy = 0.0
            gth = math.atan2(ty, tx) if face else 0.0
        r = self.resolve(f"snapshot:{s.sid}", gx, gy, gth)
        chk, s2 = self._check(r, s.sid)
        imgs = self._check_images(chk, s2, r, f"approach{self.n_turn + 1}")
        warnings = list(chk.get("warnings", []))
        if res.get("warning"):
            warnings.insert(0, "the source query warned: " + res["warning"])
        out = {"proposal": {"frame": f"snapshot:{s.sid}", "x": round(gx, 4), "y": round(gy, 4),
                            "theta": round(gth, 4)}, "goal": r, "check": _public(chk), "warnings": warnings,
               "bev": f"images/{imgs[0]}" if imgs else None,
               "note": "a proposal only: nothing moves until you POST /goal"}
        self.turn(actor, "plan_approach", {"query": query, "standoff": standoff, "face": face}, out, comment, imgs,
                  level="warn" if warnings else "info")
        return out

    # -- motion (explicit only) ---------------------------------------------------------------------

    def goal(self, actor: str, frame: str, x: float, y: float, theta: float, options: dict,
             snapshot: Optional[str] = None, comment: str = "") -> dict:
        self._need_session(actor)
        if self.latch is not None and actor != "human":
            raise RequestError(423, f"held by the human operator ({self.latch['why']})")
        options = dict(options)
        margin = options.pop("reflex_margin_m", None)          # agent_nav's own (the reflex); not pose_servo's
        if margin is not None and not (0 < float(margin) <= REFLEX_MARGIN_MAX):
            raise ValueError(f"reflex_margin_m must be > 0 and <= {REFLEX_MARGIN_MAX:g} m")
        margin = REFLEX_MARGIN if margin is None else float(margin)
        bad = set(options) - set(GOAL_OPTIONS)
        if bad:
            raise ValueError(f"unknown goal options {sorted(bad)}")
        r = self.resolve(frame, x, y, theta)
        chk, s = self._check(r, snapshot)
        lx, ly, lth = r["local"]
        body = {"frame": "local", "x": lx, "y": ly, "theta": lth, **{k: v for k, v in options.items() if v is not None}}
        ps = self._call("pose_servo", "POST", "/goal", body)
        rec = GoalRec(ps["goal_id"], {"frame": frame, "x": x, "y": y, "theta": theta, **options,
                                      "reflex_margin_m": margin}, r, chk, actor, time.monotonic())
        with self.lock:
            self.goals[rec.gid] = rec
            self.reflex["armed"] = True
            self.reflex["margin"] = margin                     # for this goal (and until the next one)
        imgs = self._check_images(chk, s, r, f"goal{rec.gid}")
        out = {"goal_id": rec.gid, "status": "moving", "goal": r, "check": _public(chk),
               "warnings": chk.get("warnings", []), "bev": f"images/{imgs[0]}" if imgs else None,
               "wait": f"goals/{rec.gid}/wait", "reflex_margin_m": margin}
        self.turn(actor, "goal", rec.request, out, comment, imgs, level="warn" if chk.get("warnings") else "info")
        return out

    def goal_state(self, gid: int) -> dict:
        rec = self.goals.get(gid)
        if rec is None:
            raise RequestError(404, f"goal {gid} unknown in this session")
        st = self._ps_status or {}
        out = {"goal_id": gid, "done": rec.done.is_set(), "result": rec.result, "goal": rec.resolved,
               "issued_by": rec.issued_by, "age_s": round(time.monotonic() - rec.t, 2)}
        if not rec.done.is_set():
            out["progress"] = {k: st.get("latest", {}).get(k) for k in ("err_pos", "err_theta", "vx", "vy", "omega")}
        return out

    def _servo_moving(self) -> bool:
        st = self._ps_status
        return bool(st and st.get("status") in ("moving", "braking"))

    # -- watcher: goal results, external changes ------------------------------------------------------

    def _watch(self):
        last_status = None
        while not self.stop_ev.is_set():
            time.sleep(0.1)
            if not self.targets.get("pose_servo"):
                continue
            t_poll = time.monotonic()
            try:
                with self._client("pose_servo", 2) as c:
                    st = c.get("/state")
            except Exception:
                st = None
            self._ps_status = st
            if st is None:
                continue
            g = st.get("goal") or {}
            rec = self.goals.get(g.get("goal_id"))
            if rec is not None and not rec.done.is_set() and g.get("result") is not None:
                rec.result = {"result": g["result"], "reason": g.get("reason"),
                              "pose_world": [st["latest"].get(k) for k in ("x", "y", "theta")] if st.get("latest") else None,
                              "err_pos": (st.get("latest") or {}).get("err_pos"),
                              "duration_s": round(time.monotonic() - rec.t, 2)}
                rec.done.set()
                self._robot_turn(f"goal {rec.gid} {g['result']}" + (f": {g.get('reason')}" if g.get("reason") else ""),
                                 level="info" if g["result"] == "reached" else "warn", goal_id=rec.gid, **rec.result)
            # a poll that started before our latest enable returned may still say "disabled": not an outside disable
            if (self.enabled_by is not None and st.get("status") == "disabled" and last_status not in (None, "disabled")
                    and t_poll > self.enabled_at):
                why = st.get("disabled_reason")
                with self.lock:
                    self.enabled_by = None
                self._robot_turn(f"pose_servo disabled outside agent_nav: {why}", level="warn")
                for rec in self.goals.values():
                    if not rec.done.is_set():
                        rec.result = {"result": "aborted", "reason": f"pose_servo disabled: {why}"}
                        rec.done.set()
            last_status = st.get("status")

    # -- the stop reflex (the only automatic action: it can only stop) ------------------------------------

    def _reflex_loop(self):
        prev = None
        while not self.stop_ev.is_set():
            time.sleep(1 / REFLEX_HZ)
            if not (self.reflex["enabled"] and self._servo_moving() and self.reflex["armed"]):
                prev = None
                continue
            now = time.monotonic()
            if prev is not None:      # the real interval between checks (sleep + the last check's own time)
                self._reflex_tick_s = max(1.0 / REFLEX_HZ, 0.7 * self._reflex_tick_s + 0.3 * (now - prev))
            prev = now
            try:
                self._reflex_tick()
            except Exception as e:
                self.reflex["last"] = f"error: {type(e).__name__}: {e}"
                self.reflex["tick_errors"] += 1

    def _reflex_tick(self):
        st = self._ps_status or {}
        L = st.get("latest") or {}
        vx, vy = L.get("vx") or 0.0, L.get("vy") or 0.0
        v = math.hypot(vx, vy)
        if v < 0.03:
            return
        caps = self.caps or PROFILES[self.profile]
        margin = self.reflex.get("margin", REFLEX_MARGIN)   # per goal (POST /goal reflex_margin_m)
        tick = self._reflex_tick_s
        dist = stopping_distance(v, caps["max_accel"], caps["max_jerk"], tick) + margin
        ang = math.atan2(vy, vx)
        if abs(wrap(ang)) > math.radians(40):
            self.reflex["blind"] = f"moving at {math.degrees(ang):.0f}° (outside the camera's view): reflex blind"
            return
        self.reflex["blind"] = None
        cal = self._call("perception", "GET", "/calibration", timeout=2)
        h, payload = self._frame(cal)
        d = h["depth"]
        depth = np.frombuffer(payload, "<u2", d["width"] * d["height"], d["offset"]).reshape(d["height"], d["width"])
        cam = G.Camera.from_color(cal["color"], d["width"], d["height"])
        if not cal.get("base_to_color"):
            return
        cl = G.Cloud(depth, cam, np.array(cal["base_to_color"]), None)
        P = cl.p_b                                     # (H, W, 3); NaN where there is no depth
        with np.errstate(invalid="ignore"):
            c, s = math.cos(ang), math.sin(ang)
            a = c * P[..., 0] + s * P[..., 1]          # along the motion
            b = -s * P[..., 0] + c * P[..., 1]
            h_ = G.HALF + 0.03
            hit = ((P[..., 2] > G.OBST_Z[0]) & (P[..., 2] < G.OBST_Z[1])
                   & (a > G.HALF) & (a < G.HALF + dist) & (np.abs(b) < h_))   # (H, W) on the depth grid
        n = int(hit.sum())
        self.reflex["last"] = {"t": time.time(), "v": round(v, 3), "lookahead_m": round(dist, 3),
                               "margin_m": round(margin, 3), "tick_s": round(tick, 3), "points": n}
        if n >= REFLEX_POINTS:
            near = float(a[hit].min() - G.HALF)
            try:
                self._call("pose_servo", "POST", "/stop", {})
            finally:
                self.reflex["stops"] += 1
                self.reflex["armed"] = False          # re-armed by the next goal
                self._reflex_event(n, near, dist, v, margin, hit, P, h, payload)

    def _reflex_event(self, n, near, dist, v, margin, hit, P, h, payload):
        """Log a reflex stop (after the stop was sent): what the reflex counted, and its mask."""
        H = P[hit]
        pct = lambda k: [round(float(q), 3) for q in np.percentile(H[:, k], [5, 50, 95])]
        reply = {"message": f"Reflex stop: {n} depth points in the stopping zone ({dist:.2f} m ahead at {v:.2f} m/s, "
                            f"margin {margin:.2f} m), nearest {near:.2f} m ahead of the base. They may include false "
                            "positives; see the mask. The goal has ended; a new goal continues.",
                 "points": n, "distance_m": round(near, 3), "lookahead_m": round(dist, 3), "margin_m": round(margin, 3),
                 "speed_m_s": round(v, 3), "tick_s": self.reflex["last"]["tick_s"],
                 "tick_errors": self.reflex["tick_errors"],
                 "points_base": {"x": pct(0), "y": pct(1), "z": pct(2)}}
        imgs = []
        try:
            c = h["color"]
            jpeg = payload[c["offset"]: c["offset"] + c["len"]]
            name = self.store_image(G.mask_image(jpeg, hit, label=f"reflex: {n} points in the stopping zone"),
                                    "reflex", "jpg")
            reply["mask"] = f"images/{name}"
            imgs = [name]
        except Exception as e:      # the stop is already sent; a missing image must not hide the event
            reply["mask_error"] = f"{type(e).__name__}: {e}"
        self.turn("robot", "event", {}, reply, images=imgs, level="warn")

    # -- memory ---------------------------------------------------------------------------------

    def note(self, actor: str, text: str, comment: str = "") -> dict:
        with self.lock:
            self.memory["notes"].append({"t": time.time(), "by": actor, "text": text})
        self.turn(actor, "note", {"text": text}, {"notes": len(self.memory["notes"])}, comment)
        return {"notes": len(self.memory["notes"])}

    def place(self, actor: str, name: str, frame: str, x: float, y: float, theta: Optional[float],
              comment: str = "") -> dict:
        r = self.resolve(frame, x, y, theta or 0.0)
        now = self.pose_now()
        p = {"x": r["world"][0], "y": r["world"][1], "theta": r["world"][2] if theta is not None else None,
             "reset_id": now[3] if now else None, "t": time.time()}
        with self.lock:
            self.memory["places"][name] = p
        self.turn(actor, "place", {"name": name, "frame": frame, "x": x, "y": y, "theta": theta}, {"place": name, **p}, comment)
        return {"place": name, **p}

    # -- stored sessions ---------------------------------------------------------------------------

    def _sdir(self, sid: str) -> str:
        if not sid or "/" in sid or sid.startswith("."):
            raise RequestError(404, "no such session")
        d = os.path.join(self.data_dir, sid)
        if not os.path.isfile(os.path.join(d, "session.json")):
            raise RequestError(404, f"no session {sid!r}")
        return d

    def list_sessions(self) -> list:
        out = []
        if os.path.isdir(self.data_dir):
            for sid in sorted(os.listdir(self.data_dir), reverse=True):
                f = os.path.join(self.data_dir, sid, "session.json")
                if not os.path.isfile(f):
                    continue
                try:
                    with open(f) as fh:
                        s = json.load(fh)
                except (OSError, ValueError):
                    continue
                lf = os.path.join(self.data_dir, sid, "log.jsonl")
                s["bytes"] = sum(os.path.getsize(os.path.join(dp, fn)) for dp, _, fns in os.walk(os.path.join(self.data_dir, sid)) for fn in fns)
                if "turns" not in s and os.path.isfile(lf):
                    with open(lf) as fh:
                        s["turns"] = sum(1 for _ in fh)
                s["live"] = self.session is not None and self.session["id"] == sid
                out.append(s)
        return out

    def read_session(self, sid: str) -> dict:
        d = self._sdir(sid)
        with open(os.path.join(d, "session.json")) as f:
            s = json.load(f)
        turns = []
        lf = os.path.join(d, "log.jsonl")
        if os.path.isfile(lf):
            with open(lf) as f:
                for line in f:
                    try:
                        turns.append(json.loads(line))
                    except ValueError:
                        pass
        return {"session": s, "turns": turns}

    def export_session(self, sid: str) -> bytes:
        """A zip: session.json, log.jsonl, thumbs/*.jpg (the stored transcript)."""
        import zipfile
        d = self._sdir(sid)
        b = io.BytesIO()
        with zipfile.ZipFile(b, "w", zipfile.ZIP_DEFLATED) as z:
            for dp, _, fns in os.walk(d):
                for fn in sorted(fns):
                    p = os.path.join(dp, fn)
                    z.write(p, os.path.join(f"agent_nav_{sid}", os.path.relpath(p, d)))
        return b.getvalue()

    def delete_session(self, sid: str) -> dict:
        import shutil
        d = self._sdir(sid)
        if self.session is not None and self.session["id"] == sid:
            raise RequestError(409, "end the live session before deleting it")
        shutil.rmtree(d)
        self.events.add(f"session {sid} deleted")
        return {"deleted": sid}

    # -- state ------------------------------------------------------------------------------------

    def safety(self) -> dict:
        """Locks that apply (chocks on the base, a camera cover) and the swerve's e-stop, for pages."""
        from ..common.locks import LockStore, lock_state
        if not hasattr(self, "_locks"):
            self._locks = LockStore()
        sw = (self._ps_status or {}).get("swerve") or {}
        return {**lock_state(self._locks, chock={"swerve", "pose_servo", "agent_nav"}, cover={"camera"}),
                "estop": {"latched": bool(sw.get("estop")), "source": "swerve" if sw.get("estop") else None, "reason": None}}

    def state(self) -> dict:
        now = self.pose_now()
        st = self._ps_status or {}
        safety = self.safety()
        with self.lock:
            return {
                **safety,
                "session": self.session, "profile": self.profile, "profiles": PROFILES,
                "enabled_by": self.enabled_by, "caps": self.caps, "latch": self.latch,
                "pose_world": None if now is None else [round(v, 4) for v in now[:3]],
                "reset_id": None if now is None else now[3],
                "pose_servo": {k: st.get(k) for k in ("status", "enabled", "holder", "disabled_reason", "goal")} if st else None,
                "reflex": dict(self.reflex),
                "snapshots": list(self.snapshots.keys())[-10:],
                "goals": {g.gid: {"done": g.done.is_set(), "result": g.result and g.result.get("result")}
                          for g in list(self.goals.values())[-10:]},
                "memory": self.memory, "turns": self.n_turn,
                "inputs": {"estimator": self.est.status() if self.est else None},
                "events": self.events.list()[-20:],
            }

    def set_profile(self, name: str) -> dict:
        if name not in PROFILES:
            raise ValueError(f"profile must be one of {sorted(PROFILES)}")
        with self.lock:
            self.profile = name
        self.turn("human", "profile", {"profile": name}, {"profile": name, "caps": PROFILES[name],
                  "note": "applies at the next enable"})
        return {"profile": name, "caps": PROFILES[name]}


def _public(d: dict) -> dict:
    return {k: v for k, v in d.items() if not k.startswith("_")}


def _json_default(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


def _as_jpeg(img: tuple) -> bytes:
    data, mime = img
    if mime == "image/jpeg":
        return data
    from PIL import Image
    b = io.BytesIO()
    Image.open(io.BytesIO(data)).convert("RGB").save(b, "JPEG", quality=80)
    return b.getvalue()
