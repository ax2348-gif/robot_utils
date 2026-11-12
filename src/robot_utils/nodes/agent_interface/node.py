"""agent core: the agent <-> robot boundary (no web code).

Principles (docs/agent_interface.md):
- Perception and planning calls are READ-ONLY: they return data, proposals and overlay images.
- Motion happens only when the agent (or a human on the page) explicitly issues a goal / stop.
  No call ingests perception and moves the base on its own.
- Stopping is the only automatic action allowed: the stop reflex brakes (pose_servo /stop) when
  depth shows an obstacle inside the braking distance ahead of the motion.
- Every call is logged as a dialogue turn (caller -> robot reply), with an optional comment.

Peers (instance names or URLs): pose_servo (motion), 3d_perception (registered colour + depth),
estimator (pose stream, for the pose at each snapshot's capture time), arm (forwarded; this node holds
its lease for the session), swerve (read only: who holds its lease, for the base anomaly guard).

Events for the agent (`since_last_call` on its replies) are the robot turns of the transcript: goal
ends, reflex stops, arm jams, base movement anomalies.
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

log = logging.getLogger("agent_interface")

DATA_DIR = os.path.expanduser("~/.local/share/robot_utils/agent_interface")
KEEP_SNAPSHOTS = 30          # full-resolution snapshots kept in memory (thumbnails are stored)
KEEP_IMAGES = 200            # rendered overlay images kept in memory
KEEP_LOG = 1000              # dialogue turns kept in memory (all are in the session's log.jsonl)
POSE_HISTORY_S = 30.0
REFLEX_HZ = 6.0
REFLEX_POINTS = 25           # obstacle points ahead needed to trigger the reflex
REFLEX_LATENCY = 0.35        # s: depth age + reaction, added to the braking distance
REFLEX_MARGIN = 0.12         # m: default gap left in front of what stops the reflex (beyond braking + latency)
REFLEX_MARGIN_MAX = 1.0      # m: a goal may set its margin anywhere in (0, this]
ALIGN_DEADZONE_M = 0.02      # defensive alignment: horizontal arm travel under this changes nothing
ALIGN_HYSTERESIS_DEG = 15.0  # nor a direction within this of the current alignment (an axis: no sign)
ALIGN_WAIT_S = 1.0           # wait at most this for the wheels to align before the arm moves
ANOMALY_M = 0.01             # base anomaly: moved this far ...
ANOMALY_DEG = 1.0            # ... or turned this much from where an idle stretch began
ANOMALY_QUIET_S = 1.0        # an idle stretch starts this long after the last goal / stop (braking settles)
ARM_POLL_S = 0.25            # lease renewal and jam polling


def stopping_distance(v: float, accel: float, jerk: float, tick_s: float = 1.0 / REFLEX_HZ) -> float:
    """m travelled from the moment an obstacle enters the zone until the base stands still:
    detection (up to one reflex tick) + latency (depth age, reaction) at speed v, then pose_servo's
    jerk-limited braking (v²/2a, plus v·a/2j while the deceleration ramps up). The reflex margin
    is added on top of this, so a small margin still stops before what it sees."""
    return v * (tick_s + REFLEX_LATENCY) + v * v / (2 * accel) + v * accel / (2 * jerk)
MAX_AVATAR = 64 * 1024       # bytes (data URI)
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
    T_bc_desc: Optional[np.ndarray] = None      # the camera pose as described (T_bc: with the session's correction)
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
                 require_imu: bool = True, description: Optional[str] = "description", arm: Optional[str] = "arm",
                 swerve: Optional[str] = "swerve"):
        self.targets = {"pose_servo": pose_servo, "perception": perception, "estimator": estimator,
                        "description": description, "arm": arm, "swerve": swerve}
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
        self.reflex = {"force": True, "armed": False, "last": None, "blind": None, "stops": 0,
                       "margin": REFLEX_MARGIN, "tick_errors": 0}
        self.alignment = {"force": False, "deadzone_m": ALIGN_DEADZONE_M, "hysteresis_deg": ALIGN_HYSTERESIS_DEG,
                          "last": None}
        self.arm = {"token": None, "lease_id": None, "heartbeat": 1.0, "enabled": False, "jams_seen": None,
                    "error": None}
        self.anomaly = {"checking": False, "ref": None, "count": 0, "last": None}
        self._base_busy_t = 0.0                 # monotonic s the base was last driven (goal, braking, holding, stop)
        self._swerve_lease = (0.0, None)        # (monotonic s read, holder or None)
        self._ev_seen = 0                       # the agent's events cursor (turn number at its last reply)
        self._reflex_tick_s = 1.0 / REFLEX_HZ      # measured interval between reflex checks (EMA)
        self.geom_corr = _zero_corr()           # session placement corrections (display / perception only)
        self._n_snap = self._n_img = self._n_q = 0
        self._ps_status = None
        self._threads = [threading.Thread(target=self._read_poses, name="poses", daemon=True),
                         threading.Thread(target=self._watch, name="watch", daemon=True),
                         threading.Thread(target=self._reflex_loop, name="reflex", daemon=True),
                         threading.Thread(target=self._arm_loop, name="arm", daemon=True)]

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
        self.events.add("agent_interface started")

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
                                         "places from before are no longer valid", level="warn",
                                         source="estimator")
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

    def _robot_turn(self, msg: str, level: str = "info", source: str = "agent_interface", **data):
        self.turn("robot", "event", {}, {"message": msg, "source": source, **data}, level=level)

    def since_last_call(self) -> list:
        """The robot's events (robot turns of this session) since the agent's previous reply, oldest first; moves the
        agent's cursor. Only agents' replies carry them (the page reads the transcript)."""
        with self.lock:
            sid = (self.session or {}).get("id")
            new = [t for t in self.turns if t["n"] > self._ev_seen and t["actor"] == "robot" and t.get("session") == sid]
            self._ev_seen = self.n_turn
        now = time.monotonic()
        out = []
        for t in new:
            r = dict(t["reply"])
            msg, src = r.pop("message", ""), r.pop("source", "agent_interface")
            out.append({"n": t["n"], "wall": round(t["wall"], 3), "age_s": round(now - t["t"], 2), "source": src,
                        "level": t["level"], "message": msg, "data": r})
        return out

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
            self.geom_corr = _zero_corr()
            self.goals.clear()
            self.snapshots.clear()          # snapshots and queries belong to one session: a new session's checks
            self.queries.clear()            # never use an earlier session's views (the arm may have moved since)
            self._n_snap = self._n_q = self._n_zoom = 0     # and its ids start again at s1, q1, z1
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
            self.snapshots.clear()
            self.queries.clear()
        return {"closed": s["id"]}

    def _need_session(self, actor: str):
        if actor == "agent" and self.session is None:
            raise RequestError(409, "open a session first: POST /session {agent, avatar}")

    def enable(self, actor: str, caps: Optional[dict] = None, comment: str = "", base: bool = True,
               arm: bool = True) -> dict:
        """Enable the base (pose_servo + the swerve lease, with the caps) and the arm (this node takes its lease and
        enables it: it floats until a move engages it). `base` / `arm` false: leave that part as it is."""
        self._need_session(actor)
        if self.latch is not None and actor != "human":
            raise RequestError(423, f"held by the human operator ({self.latch['why']}); ask them to release it")
        if not (base or arm):
            raise ValueError("nothing to enable: base and arm both false")
        name = (self.session or {}).get("agent", "human") if actor == "agent" else "human"
        reply = {"enabled": True}
        if base:
            reply.update(self._enable_base(actor, caps, name))
        if arm:
            try:
                out = self._arm_enable(name)
                reply["arm"] = {"enabled": True, "state": out.get("state"), "lease_id": self.arm["lease_id"]}
            except RequestError as e:
                reply["arm"] = {"enabled": False, "error": str(e)}
        if actor == "human":
            with self.lock:
                self.latch = None
        level = "warn" if arm and not reply["arm"]["enabled"] else "info"
        self.turn(actor, "enable", {"caps": caps or {}, "base": base, "arm": arm}, reply, comment, level=level)
        return reply

    def _enable_base(self, actor: str, caps: Optional[dict], name: str) -> dict:
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
        ps = self._call("pose_servo", "GET", "/state")
        adopted = None
        if ps.get("enabled"):
            holder = ps.get("holder") or ""
            if not holder.startswith("agent_interface: "):
                raise RequestError(409, f"pose_servo is enabled by {holder!r} (not agent_interface): they must disable it first")
            if ps.get("status") not in ("idle", "holding"):
                raise RequestError(409, f"pose_servo is {ps.get('status')} under {holder!r}: stop it first")
            self._call("pose_servo", "POST", "/disable", {})     # idle: re-enable with this caller's caps
            adopted = holder
        out = self._call("pose_servo", "POST", "/enable", {"client": f"agent_interface: {name}",
                                                           "config": {**cfg, "require_imu": self.require_imu}}, timeout=8)
        with self.lock:
            self.enabled_by, self.caps = actor, cfg
            self.enabled_at = time.monotonic()
        reply = {"base": True, "profile": self.profile, "caps": cfg}
        if adopted:
            reply["adopted"] = f"pose_servo was already enabled by {adopted!r} (idle): re-enabled with these caps"
        reply["pose_servo"] = out.get("config")
        return reply

    def disable(self, actor: str, comment: str = "") -> dict:
        """Disable the base (pose_servo, no braking) and the arm (disabled, lease released)."""
        arm_err = None
        try:
            self._call("pose_servo", "POST", "/disable", {})
        finally:
            with self.lock:
                self.enabled_by = None
                if actor == "human":
                    self.latch = {"why": "disabled on the page", "t": time.time()}
            arm_err = self._arm_release()
        reply = {"enabled": False, "latched": self.latch is not None}
        if arm_err:
            reply["arm_error"] = arm_err
        self.turn(actor, "disable", {}, reply, comment, level="warn" if arm_err else "info")
        return reply

    def release(self, comment: str = "") -> dict:
        with self.lock:
            was, self.latch = self.latch, None
        self.turn("human", "release", {}, {"released": was is not None}, comment)
        return {"released": was is not None}

    def stop(self, actor: str, comment: str = "") -> dict:
        self._base_busy_t = time.monotonic()
        out = self._call("pose_servo", "POST", "/stop", {})
        self.turn(actor, "stop", {}, out, comment)
        return out

    def estop(self, actor: str, reason: str = "", comment: str = "") -> dict:
        out = self._call("pose_servo", "POST", "/estop", {"reason": f"agent ({actor}): {reason}", "source": actor})
        with self.lock:
            self.enabled_by = None
            if actor == "human":
                self.latch = {"why": "e-stop from the page", "t": time.time()}
        if self.targets.get("arm"):            # the arm too (anyone may; it latches there until its holder clears it)
            try:
                a = self._arm_call("POST", "/estop", {"reason": f"agent_interface ({actor}): {reason}", "source": actor},
                                   timeout=2)
                out = {**out, "arm": {"latched": bool(a.get("latched"))}}
            except RequestError as e:
                out = {**out, "arm": {"latched": False, "error": str(e)}}
            self._arm_forget()
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
                     "height_note": "low base (top surface 0.22 m) with a camera mast (the tallest part); the arm "
                                    "stands on the front-right corner and its gripper can reach past the front: the arm "
                                    "node's GET /geometry gives where it is, and /plan/check draws it"},
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

    def look(self, actor: str, scale: Optional[float] = None, grid=False, settle: bool = False,
             comment: str = "") -> dict:
        """grid: False | True (100 px) | {"step": px, "values": x|y|z|range|xyz}."""
        gspec = None if not grid else ({} if grid is True else dict(grid))
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
        T_desc = np.array(cal["base_to_color"]) if cal.get("base_to_color") else None
        T_bc = self.corrected("camera", T_desc)
        p = self.pose_at(h["t_color"])
        moving = self._servo_moving()
        with self.lock:
            self._n_snap += 1
            sid = f"s{self._n_snap}"
            snap = Snapshot(sid, h["t_color"], p[:3] if p else None, p[3] if p else None, jpeg, depth, cam, T_bc,
                            h.get("mode", "?"), moving, T_desc)
            self.snapshots[sid] = snap
            while len(self.snapshots) > KEEP_SNAPSHOTS:
                self.snapshots.popitem(last=False)
        ginfo = None
        if gspec is not None:
            gj, ginfo = G.view_image(jpeg, step=gspec.get("step") or 100, values=gspec.get("values"), cloud=snap.cloud,
                                     label=sid)
            img = self.store_image(gj, sid, "jpg")
        else:
            img = self.store_image(jpeg, sid, "jpg")
        bev = self.store_image(G.bev_image(snap.cloud, label=f"{sid} BEV (base frame, 0.5 m grid)"), sid + "_bev", "png")
        warnings = []
        if T_bc is None:
            warnings.append("camera pose in base not calibrated: no base/world coordinates")
        if p is None:
            warnings.append("no estimator pose at capture: no world coordinates, no snapshot-frame goals")
        if moving:
            warnings.append("the robot was moving (motion-compensated registration; geometry less exact)")
        if self._corr_active("camera"):
            warnings.append("session camera correction active (GET /session/geometry): base/world coordinates use it")
        out = {"snapshot": sid, "image": f"images/{img}", "bev": f"images/{bev}",
               "image_size": [cam.color_w, cam.color_h], "depth_size": [cam.width, cam.height],
               "pose_world": None if p is None else [round(v, 4) for v in p[:3]], "mode": snap.mode,
               "depth_fraction": round(float((depth > 0).mean()), 3), "moving": moving, "warnings": warnings,
               "grid_image": f"images/{sid}/grid.jpg"}
        if ginfo and ginfo.get("values"):
            out["grid"] = ginfo
        self.turn(actor, "look", {"scale": scale, "grid": grid, "settle": settle}, out, comment, [img, bev])
        return out

    def zoom(self, actor: str, snapshot: Optional[str], box: Optional[list], step: Optional[int] = None,
             values: Optional[str] = None, comment: str = "") -> dict:
        """A held snapshot (no new capture), or the crop ``box`` of it enlarged, with a grid labelled in the
        full image's pixels and optionally depth values (base frame) at its intersections."""
        self._need_session(actor)
        s = self.snapshot(snapshot)
        if box is not None and len(box) != 4:
            raise ValueError("box: [u0, v0, u1, v1] in colour pixels")
        with self.lock:
            self._n_zoom = getattr(self, "_n_zoom", 0) + 1
            zid = f"z{self._n_zoom}"
        jpg, info = G.view_image(s.jpeg, box=box, step=step, values=values, cloud=s.cloud,
                                 label=f"{zid} on {s.sid}" + (f" · {values} (m, base) at the dots" if values else ""))
        img = self.store_image(jpg, zid, "jpg")
        out = {"zoom": zid, "snapshot": s.sid, "image": f"images/{img}", **info,
               "note": "grid labels are the full image's pixel coordinates: use them as they are in queries on "
                       f"{s.sid}" + ("; samples: base-frame points (median of the depth around each intersection, "
                                     "null where there is none)" if values else "")
                       + ("; values use the session camera correction" if values and self._corr_active("camera") else "")}
        self.turn(actor, "zoom", {"snapshot": s.sid, "box": box, "step": step, "values": values},
                  {k: v for k, v in out.items() if k != "samples"}, comment, [img])
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
                    raise RequestError(409, "no snapshot in this session yet: POST /look")
                return next(reversed(self.snapshots.values()))
            s = self.snapshots.get(sid)
        if s is None:
            raise RequestError(404, f"snapshot {sid!r} unknown, expired (the last {KEEP_SNAPSHOTS} are kept) or from another session")
        return s

    def grid_image(self, sid: str) -> bytes:
        s = self.snapshot(sid)
        return G.view_image(s.jpeg, step=100, label=sid)[0]

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

    def _check(self, resolved: dict, snapshot: Optional[str],
                also_use: Optional[list] = None) -> tuple[dict, Optional[Snapshot]]:
        """Path check against the given (or latest) snapshot alone, or combined with the snapshots
        ``also_use`` names (same world frame; coverage and obstacles from all of them). Never others by
        default: an older view may hold the arm, or the scene, where it no longer is. STALE only when the
        robot moved since that snapshot, or it is minutes old: agents take their time between a look and a
        goal."""
        try:
            s = self.snapshot(snapshot)
        except RequestError:
            return {"checked": False, "warnings": ["UNCHECKED: no snapshot to check the path against (POST /look)"]}, None
        recent = []
        if also_use:
            for sid in also_use:
                o = self.snapshot(sid)
                if o is s:
                    continue
                if o.pose is None or o.reset_id != s.reset_id:
                    raise RequestError(409, f"{sid} is not in {s.sid}'s world frame (no pose, or the estimator was "
                                            "reset since): it can't be combined")
                recent.append(o)
        chk = G.path_check(s.cloud, tuple(resolved["from_world"]), tuple(resolved["world"]),
                           others=[o.cloud for o in recent])
        chk["against_snapshot"] = s.sid
        chk["also_used"] = [o.sid for o in recent]
        by_cloud = {id(o.cloud): o for o in [s, *recent]}
        chk["_masks"] = [(by_cloud[id(c)], m) for c, m in chk.get("_masks", []) if id(c) in by_cloud]
        if chk.get("checked") and chk.get("obstacle_points_in_path"):
            chk["obstacles_by_snapshot"] = {o.sid: int(m.sum()) for o, m in chk["_masks"] if m.any()}
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
        out = [bev]
        masks = {}
        for o, m in chk.get("_masks", []):
            if m.any():
                img = self.store_image(G.mask_image(o.jpeg, m, label=f"{tag}: {int(m.sum())} points of {o.sid} counted as "
                                                                       "obstacles in the path (red)"),
                                       f"{tag}_mask_{o.sid}", "jpg")
                masks[o.sid] = f"images/{img}"
                out.append(img)
        if masks:
            chk["obstacle_masks"] = masks
        chk.pop("_masks", None)                     # pixel masks: only for these images (they hold the snapshots)
        return out

    def plan_check(self, actor: str, frame: str, x: float, y: float, theta: float, snapshot: Optional[str] = None,
                   comment: str = "", arm: Optional[dict] = None, view: bool = True,
                   also_use: Optional[list] = None) -> dict:
        self._need_session(actor)
        r = self.resolve(frame, x, y, theta)
        chk, s = self._check(r, snapshot, also_use)
        tag = f"check{self.n_turn + 1}"
        imgs = self._check_images(chk, s, r, tag)
        out = {"goal": r, "check": _public(chk), "bev": f"images/{imgs[0]}" if imgs else None}
        if view or arm is not None:
            robot, vimg = self._robot_view(s, r, arm, tag)
            out["robot"] = robot
            if vimg:
                imgs.append(vimg)
        req = {"frame": frame, "x": x, "y": y, "theta": theta, **({"arm": arm} if arm is not None else {}),
               **({"also_use": also_use} if also_use is not None else {})}
        self.turn(actor, "plan_check", req, out, comment, imgs,
                  level="warn" if chk.get("warnings") or (out.get("robot") or {}).get("warnings") else "info")
        return out

    # -- session geometry: placement corrections for display and perception (never motion) ----------

    def corrected(self, which: str, T: Optional[np.ndarray]) -> Optional[np.ndarray]:
        """A frame's pose in base (4x4) with the session's correction: its origin moved by ``xyz`` (m, base
        axes) and turned by ``rpy_deg`` about that origin (base axes)."""
        if T is None or not self._corr_active(which):
            return T
        c = self.geom_corr[which]
        R = _rpy(*np.radians(c["rpy_deg"]))
        out = np.array(T, float).copy()
        out[:3, :3] = R @ out[:3, :3]
        out[:3, 3] = out[:3, 3] + np.asarray(c["xyz"], float)
        return out

    def _corr_active(self, which: str) -> bool:
        c = self.geom_corr[which]
        return any(abs(v) > 0 for v in c["xyz"]) or any(abs(v) > 0 for v in c["rpy_deg"])

    def session_geometry(self, actor: str = "agent", log_turn: bool = False, comment: str = "") -> dict:
        """The placements as described, the session's corrections, and the effective placements."""
        desc = {"camera": None, "arm": None}
        errs = []
        try:
            cal = self._call("perception", "GET", "/calibration", timeout=3)
            if cal.get("base_to_color"):
                desc["camera"] = np.array(cal["base_to_color"], float)
        except RequestError as e:
            errs.append(f"camera pose unavailable: {e}")
        try:
            desc["arm"] = np.array(self._call("arm", "GET", "/geometry", timeout=4)["frames"]["arm_in_base"]["matrix"], float)
        except RequestError as e:
            errs.append(f"arm mount unavailable: {e}")
        pose = lambda T: None if T is None else {"xyz": [round(float(v), 5) for v in T[:3, 3]],
                                                 "rpy_deg": [round(float(v), 3) for v in np.degrees(_matrix_rpy(T[:3, :3]))]}
        out = {
            "frames": {"camera": "the colour camera's optical frame (x right, y down, z out of the lens) in base",
                       "arm": "the arm frame (the arm's URDF base_link, axes parallel to base) in base"},
            "described": {k: pose(v) for k, v in desc.items()},
            "corrections": self.geom_corr,
            "effective": {k: pose(self.corrected(k, v)) for k, v in desc.items()},
            "limits": {"xyz_m": CORR_MAX_M, "rpy_deg": CORR_MAX_DEG},
            "applies_to": "look, zoom grid values, queries (base and world coordinates) and the robot drawn by "
                          "plan/check, for this session only",
            "never_applies_to": "the stop reflex, goals and arm commands (motion nodes use the description)",
        }
        if errs:
            out["warnings"] = errs
        if log_turn:
            self.turn(actor, "geometry", {}, out, comment)
        return out

    def set_session_geometry(self, actor: str, camera: Optional[dict], arm: Optional[dict], comment: str = "") -> dict:
        """Replace the session's placement corrections (each {xyz, rpy_deg}; omitted = unchanged; {} = none)."""
        self._need_session(actor)
        new = {k: dict(v) for k, v in self.geom_corr.items()}
        for name, c in (("camera", camera), ("arm", arm)):
            if c is None:
                continue
            xyz = [float(v) for v in c.get("xyz", [0, 0, 0])]
            rpy = [float(v) for v in c.get("rpy_deg", [0, 0, 0])]
            if len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(v) for v in xyz + rpy):
                raise ValueError(f"{name}: xyz and rpy_deg are 3 finite numbers each")
            if max(abs(v) for v in xyz) > CORR_MAX_M or max(abs(v) for v in rpy) > CORR_MAX_DEG:
                raise ValueError(f"{name}: corrections are limited to {CORR_MAX_M} m and {CORR_MAX_DEG}° per axis "
                                 "(a larger error is a calibration for the operator)")
            new[name] = {"xyz": xyz, "rpy_deg": rpy}
        with self.lock:
            self.geom_corr = new
            for s in self.snapshots.values():          # held snapshots: re-derive their points with the correction
                s.T_bc = self.corrected("camera", s.T_bc_desc)
                s._cloud = None
            d = self._session_dir()
            if d:
                with open(os.path.join(d, "geometry.json"), "w") as f:
                    json.dump({"t": time.time(), "corrections": new}, f)
        out = self.session_geometry(actor)
        self.turn(actor, "geometry_set", {"camera": camera, "arm": arm}, out, comment, level="warn")
        return out

    def _arm_geometry(self, arm: Optional[dict]) -> dict:
        """The arm node's geometry: now (arm None) or at a TCP pose / joint angles (arm dict)."""
        if arm is None:
            return self._call("arm", "GET", "/geometry", timeout=5)
        body = {k: arm[k] for k in ("pose", "q", "gap_mm", "max_rot_deg") if arm.get(k) is not None}
        body["seed"] = "measured"
        return self._call("arm", "POST", "/geometry", body, timeout=8)

    def _robot_view(self, s: Optional[Snapshot], resolved: dict, arm: Optional[dict], tag: str) -> tuple[dict, Optional[str]]:
        """The whole robot (base + arm) at the goal pose, drawn into the snapshot, with the fingertips
        and the depth points inside the arm's boxes. Imagination only: nothing moves."""
        out: dict = {"warnings": []}
        W = out["warnings"]
        if s is None or s.T_bc is None or s.pose is None:
            W.append("no calibrated snapshot with a pose: the robot is not drawn (POST /look)")
            return out, None
        px, py, pth = s.pose
        gx, gy, gth = resolved["world"]
        c, sn = math.cos(pth), math.sin(pth)
        gb = (c * (gx - px) + sn * (gy - py), -sn * (gx - px) + c * (gy - py), wrap(gth - pth))
        Rg = np.array([[math.cos(gb[2]), -math.sin(gb[2]), 0], [math.sin(gb[2]), math.cos(gb[2]), 0], [0, 0, 1.0]])
        tg = np.array([gb[0], gb[1], 0.0])
        out["goal_in_snapshot_base"] = [round(v, 4) for v in gb]
        boxes = [(G.place(b, Rg, tg), G.BODY_COLOR) for b in G.body_boxes()]
        pts = []
        try:
            geo = self._arm_geometry(arm)
        except RequestError as e:
            if e.status < 500:
                raise                                   # a bad arm request: the agent's to fix
            geo = None
            W.append(f"arm node unavailable ({e}): base only")
        if geo is not None:
            T_desc_ab = np.asarray(geo["frames"]["arm_in_base"]["matrix"], float)  # arm frame -> base (description)
            R_desc = T_desc_ab[:3, :3]
            T_ab = self.corrected("arm", T_desc_ab)                                # with the session's correction
            to_snap = lambda p_arm: Rg @ (T_ab[:3, :3] @ np.asarray(p_arm) + T_ab[:3, 3]) + tg
            arm_boxes = []
            for b in geo["boxes"]:
                axes_arm = R_desc.T @ np.array(b["axes_base"]).T                     # columns = box axes, arm frame
                bb = {"name": b["name"], "center": to_snap(b["center"]["arm"]), "axes": Rg @ T_ab[:3, :3] @ axes_arm,
                      "size": np.array(b["size_m"])}
                arm_boxes.append(bb)
                boxes.append((bb, G.ARM_COLOR))
            ft = geo["fingertips"]
            tips = {"left": ft["left"]["arm"], "right": ft["right"]["arm"], "center": ft["center"]["arm"]["pos"]}
            P_tips = {k: to_snap(v) for k, v in tips.items()}
            pts += [(P_tips["left"], "L", G.TIP_COLOR), (P_tips["right"], "R", G.TIP_COLOR)]
            wx = lambda p: list(s.cloud.base_to_world_xy(p[0], p[1])) + [p[2]]
            back = lambda p: Rg.T @ (np.asarray(p) - tg)                               # snapshot base -> goal base
            out["arm"] = {"which": geo["which"], "q": geo["q"], "gap_mm": geo["gap_mm"], "ik": geo.get("ik"),
                          "arm_in_base": {**geo["frames"]["arm_in_base"], "effective": [[round(float(v), 6) for v in row] for row in T_ab],
                                          "session_correction": self.geom_corr["arm"] if self._corr_active("arm") else None}}
            if self._corr_active("arm"):
                W.append("session arm-mount correction active (GET /session/geometry): the arm is drawn with it; the arm "
                         "node's own replies (and every motion command) use the description")
            out["fingertips"] = {k: {"base": [round(float(v), 4) for v in back(p)],
                                     "snapshot_base": [round(float(v), 4) for v in p],
                                     "world": [round(float(v), 4) for v in wx(p)]} for k, p in P_tips.items()}
            out["past_footprint_m"] = geo["extent_base"]["past_footprint_m"]
            ik = geo.get("ik")
            if ik is not None and not ik.get("exact", ik.get("ok")):
                W.append("the arm pose is not exactly reachable: drawn at the nearest reachable configuration (`arm.ik`)")
            P = s.cloud.p_b
            if P is not None:
                P = P.reshape(-1, 3)
                P = P[np.isfinite(P).all(1)]
                hits = [{"box": b["name"], "points": n} for b in arm_boxes if (n := G.points_in_box(P, b)) > 0]
                out["depth_points_in_arm"] = hits
                if hits:
                    W.append("depth points inside the arm's boxes at this pose: an object would be in the way, or it is "
                             "the arm itself where it is now (" + ", ".join(f"{h['box']}: {h['points']}" for h in hits) + ")")
        jpg, info = G.robot_view_image(s.jpeg, s.cam, s.T_bc, boxes, pts,
                                       label=f"{tag}: the robot at the goal, drawn into {s.sid} (cyan base, orange arm)")
        out.update(info)
        if info["boxes_partly_out_of_view"]:
            W.append(f"{info['boxes_partly_out_of_view']} of {len(boxes)} parts are partly outside {s.sid}'s view")
        img = self.store_image(jpg, tag + "_robot", "jpg")
        out["view"] = f"images/{img}"
        return out, img

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
             snapshot: Optional[str] = None, comment: str = "", also_use: Optional[list] = None) -> dict:
        self._need_session(actor)
        if self.latch is not None and actor != "human":
            raise RequestError(423, f"held by the human operator ({self.latch['why']})")
        options = dict(options)
        margin = options.pop("reflex_margin_m", None)          # agent's own (the reflex); not pose_servo's
        if margin is not None and not (0 < float(margin) <= REFLEX_MARGIN_MAX):
            raise ValueError(f"reflex_margin_m must be > 0 and <= {REFLEX_MARGIN_MAX:g} m")
        margin = REFLEX_MARGIN if margin is None else float(margin)
        want = options.pop("reflex", None)                     # the agent's opt-in (the operator may force it on)
        reflex_on = bool(self.reflex["force"] or want)
        reflex_why = ("forced on by the operator" + (" (your reflex: false is overridden)" if want is False else "")
                      if self.reflex["force"] else "on: requested" if want else "off: not requested (reflex: true turns it on)")
        bad = set(options) - set(GOAL_OPTIONS)
        if bad:
            raise ValueError(f"unknown goal options {sorted(bad)}")
        r = self.resolve(frame, x, y, theta)
        chk, s = self._check(r, snapshot, also_use)
        lx, ly, lth = r["local"]
        body = {"frame": "local", "x": lx, "y": ly, "theta": lth, **{k: v for k, v in options.items() if v is not None}}
        self._base_busy_t = time.monotonic()
        ps = self._call("pose_servo", "POST", "/goal", body)
        rec = GoalRec(ps["goal_id"], {"frame": frame, "x": x, "y": y, "theta": theta, **options,
                                      "reflex": want, "reflex_margin_m": margin}, r, chk, actor, time.monotonic())
        with self.lock:
            self.goals[rec.gid] = rec
            self.reflex["armed"] = reflex_on
            self.reflex["margin"] = margin                     # for this goal (and until the next one)
        imgs = self._check_images(chk, s, r, f"goal{rec.gid}")
        out = {"goal_id": rec.gid, "status": "moving", "goal": r, "check": _public(chk),
               "warnings": chk.get("warnings", []), "bev": f"images/{imgs[0]}" if imgs else None,
               "wait": f"goals/{rec.gid}/wait", "reflex_margin_m": margin,
               "reflex": {"on": reflex_on, "why": reflex_why}}
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
                                 level="info" if g["result"] == "reached" else "warn", source="base", goal_id=rec.gid,
                                 **rec.result)
            # a poll that started before our latest enable returned may still say "disabled": not an outside disable
            if (self.enabled_by is not None and st.get("status") == "disabled" and last_status not in (None, "disabled")
                    and t_poll > self.enabled_at):
                why = st.get("disabled_reason")
                with self.lock:
                    self.enabled_by = None
                self._robot_turn(f"pose_servo disabled outside agent_interface: {why}", level="warn", source="base")
                for rec in self.goals.values():
                    if not rec.done.is_set():
                        rec.result = {"result": "aborted", "reason": f"pose_servo disabled: {why}"}
                        rec.done.set()
            last_status = st.get("status")
            try:
                self._anomaly_check(st)
            except Exception as e:          # the watcher must keep running
                self.anomaly["checking"] = False
                log.warning("anomaly check: %s", e)

    # -- the stop reflex (the only automatic action: it can only stop) ------------------------------------

    def _reflex_loop(self):
        prev = None
        while not self.stop_ev.is_set():
            time.sleep(1 / REFLEX_HZ)
            if not (self._servo_moving() and self.reflex["armed"]):
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
        reply = {"source": "reflex", "message": f"Reflex stop: {n} depth points in the stopping zone ({dist:.2f} m ahead at {v:.2f} m/s, "
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

    # -- the arm (forwarded to the arm node; this node holds its lease for the session) -------------------

    def _arm_call(self, method: str, path: str, body=None, timeout: float = 5.0, lease: bool = False):
        """One arm node request. A refusal keeps the arm's status and its machine-readable fields (`code`, `retry`,
        `nearest`, ...); `lease`: with this node's arm lease token (409 if the arm isn't enabled through here)."""
        headers = {}
        if lease:
            tok = self.arm["token"]
            if tok is None:
                raise RequestError(409, "the arm is not enabled through agent_interface: POST /enable (or /arm/enable)")
            headers["X-Lease-Token"] = tok
        try:
            with self._client("arm", timeout) as c:
                r = c.http.request(method, path, json=body if method == "POST" else None, headers=headers)
        except RequestError:
            raise
        except Exception as e:
            raise RequestError(503, f"arm unreachable: {type(e).__name__}: {e}")
        try:
            data = r.json() if r.content else None
        except ValueError:
            data = r.text
        if r.status_code >= 400:
            d = data if isinstance(data, dict) else {"detail": data}
            detail = d.get("detail", "")
            extra = {k: v for k, v in d.items() if k != "detail"}
            if not isinstance(detail, str):          # FastAPI validation errors: a list
                extra["errors"], detail = detail, "invalid request"
            raise RequestError(r.status_code if r.status_code < 500 else 502, f"arm: {detail}", extra)
        return data

    def _arm_enable(self, name: str) -> dict:
        if self.arm["token"] is None:
            got = self._arm_call("POST", "/lease", {"client": f"agent_interface: {name}"})
            with self.lock:
                self.arm.update(token=got["token"], lease_id=got["lease_id"], heartbeat=float(got.get("heartbeat") or 1.0),
                                error=None, jams_seen=None)
        out = self._arm_call("POST", "/enable", lease=True, timeout=40)      # the first: the gripper check (seconds)
        with self.lock:
            self.arm["enabled"] = True
        return out

    def _arm_forget(self) -> None:
        with self.lock:
            self.arm.update(token=None, lease_id=None, enabled=False, jams_seen=None)

    def _arm_release(self) -> Optional[str]:
        """Disable the arm and release its lease (None, or why it failed; the lease is forgotten either way)."""
        if self.arm["token"] is None:
            return None
        err = None
        for path, method in (("/disable", "POST"), ("/lease", "DELETE")):
            try:
                self._arm_call(method, path, lease=True)
            except RequestError as e:
                err = err or str(e)
        self._arm_forget()
        return err

    def _arm_loop(self):
        """Keep the arm lease while it is held here; report new arm jams as robot turns (`since_last_call`)."""
        last_renew = 0.0
        while not self.stop_ev.is_set():
            time.sleep(ARM_POLL_S)
            if not self.targets.get("arm") or self.arm["token"] is None:
                continue
            now = time.monotonic()
            if now - last_renew >= self.arm["heartbeat"] / 3:
                try:
                    self._arm_call("POST", "/lease/renew", lease=True, timeout=2)
                    last_renew = now
                except RequestError as e:
                    if e.status in (401, 403, 409):
                        self._arm_forget()
                        self.arm["error"] = str(e)
                        self._robot_turn(f"arm lease lost ({e}): the arm is no longer enabled through agent_interface; "
                                         "POST /arm/enable to take it again", level="warn", source="arm")
                        continue
            try:
                self._jam_events(self._arm_call("GET", "/jam", timeout=2))
            except RequestError:
                pass

    def _jam_events(self, j: dict) -> None:
        count = int(j.get("count") or 0)
        seen = self.arm["jams_seen"]
        self.arm["jams_seen"] = count
        if seen is None or count <= seen:
            return                                   # the first read is the baseline: jams from before aren't ours
        for jam in (j.get("recent") or []):
            if jam.get("count", 0) <= seen:
                continue
            what = (f"arm jam: joint{jam['joint']} residual {jam['residual_nm']:+.1f} Nm (band {jam['band_nm']:g}) "
                    f"for {jam['held_s']:.1f} s while static")
            if jam.get("response") == "hold":
                what += (": the arm stopped pushing and holds where it is (its move ended `jammed`). Back off along "
                         "the opposite direction; don't retry into it.")
            else:
                what += " (reported only: the arm keeps its command)."
            self._robot_turn(what, level="warn", source="arm", jam=jam)

    def _need_arm_motion(self, actor: str) -> None:
        self._need_session(actor)
        if self.latch is not None and actor != "human":
            raise RequestError(423, f"held by the human operator ({self.latch['why']})")

    def arm_enable(self, actor: str, comment: str = "") -> dict:
        self._need_arm_motion(actor)
        name = (self.session or {}).get("agent", "human") if actor == "agent" else "human"
        out = self._arm_enable(name)
        self.turn(actor, "arm_enable", {}, out, comment)
        return out

    def arm_disable(self, actor: str, comment: str = "") -> dict:
        err = self._arm_release()
        reply = {"enabled": False}
        if err:
            reply["error"] = err
        self.turn(actor, "arm_disable", {}, reply, comment, level="warn" if err else "info")
        return reply

    def arm_forward(self, actor: str, kind: str, method: str, path: str, body: Optional[dict] = None,
                    comment: str = "", lease: bool = True, timeout: float = 15.0) -> dict:
        """A forwarded arm call as a transcript turn; the reply is the arm node's, unchanged."""
        if lease:
            self._need_arm_motion(actor)
        try:
            out = self._arm_call(method, path, body, lease=lease, timeout=timeout)
        except RequestError as e:
            self.turn(actor, kind, body or {}, {"error": str(e), "status": e.status, **e.extra}, comment, level="warn")
            e.logged = True
            raise
        self.turn(actor, kind, body or {}, out if isinstance(out, dict) else {"reply": out}, comment)
        return out

    def arm_servo(self, actor: str, body: dict, align_wheels: bool = True, comment: str = "") -> dict:
        """POST /arm/servo: defensive wheel alignment first (unless off and not forced), then the arm's planned move."""
        self._need_arm_motion(actor)
        if self.arm["token"] is None:
            raise RequestError(409, "the arm is not enabled through agent_interface: POST /enable (or /arm/enable)")
        align = self._defend(body.get("pose"), align_wheels)
        req = {**body, "align_wheels": align_wheels}
        try:
            out = self._arm_call("POST", "/servo", body, lease=True, timeout=15)
        except RequestError as e:
            self.turn(actor, "arm_servo", req, {"error": str(e), "status": e.status, **e.extra, "align": align},
                      comment, level="warn")
            e.logged = True
            raise
        reply = {"align": align, **out}
        self.turn(actor, "arm_servo", req, reply, comment)
        return reply

    # -- defensive wheel alignment ------------------------------------------------------------------------

    def _defend(self, pose: Optional[dict], requested: bool) -> dict:
        """Set the wheels (drives coasting) along the arm move's horizontal direction, so a push from the arm rolls
        the base instead of dragging the tyres sideways. Skipped (and said) when it can't or needn't."""
        a = self.alignment
        res = {"requested": requested, "forced": bool(a["force"] and not requested), "done": False}

        def skip(why):
            res["why"] = why
            a["last"] = {"t": time.time(), **res}
            return res
        if not (requested or a["force"]):
            return skip("off: align_wheels false (the operator doesn't force it)")
        if pose is None:
            return skip("a named pose: no target position to align for")
        if self.enabled_by is None:
            return skip("the base is not enabled through agent_interface (POST /enable with base)")
        ps = self._call("pose_servo", "GET", "/state", timeout=2)
        if ps.get("status") != "idle":
            return skip(f"the base is {ps.get('status')}: only an idle base is aligned")
        tcp = self._arm_call("GET", "/kinematics", timeout=3)["tcp"]["pos"]     # arm frame: axes parallel to the base's
        dx, dy = pose["pos"][0] - tcp[0], pose["pos"][1] - tcp[1]
        d = math.hypot(dx, dy)
        res["travel_m"] = round(d, 3)
        if d < a["deadzone_m"]:
            return skip(f"horizontal travel {d:.3f} m < the deadzone {a['deadzone_m']:g} m (mostly vertical)")
        axis = math.degrees(math.atan2(dy, dx)) % 180.0
        res["axis_deg"] = round(axis, 1)
        cur = (ps.get("align") or {}).get("twist")
        if cur is not None and abs(cur[2]) < 1e-6:
            cur_axis = math.degrees(math.atan2(cur[1], cur[0])) % 180.0
            diff = abs(axis - cur_axis) % 180.0
            diff = min(diff, 180.0 - diff)
            if diff < a["hysteresis_deg"]:
                res.update(done=True, kept=True, axis_deg=round(cur_axis, 1), aligned=bool(ps["align"].get("aligned")),
                           why=f"within {a['hysteresis_deg']:g}° of the current alignment: kept")
                a["last"] = {"t": time.time(), **res}
                return res
        th = math.radians(axis)
        self._call("pose_servo", "POST", "/align", {"twist": [math.cos(th), math.sin(th), 0.0]}, timeout=2)
        t0 = time.monotonic()
        st = {}
        while time.monotonic() - t0 < ALIGN_WAIT_S:
            st = self._call("pose_servo", "GET", "/state", timeout=2).get("align") or {}
            if st.get("aligned"):
                break
            time.sleep(0.05)
        res.update(done=True, aligned=bool(st.get("aligned")), wait_s=round(time.monotonic() - t0, 2),
                   steer_err_deg=st.get("steer_err_deg"))
        if not res["aligned"]:
            res["why"] = f"not aligned within {ALIGN_WAIT_S:g} s: the arm moves anyway"
        a["last"] = {"t": time.time(), **res}
        return res

    # -- base movement anomalies (something moved the base while nothing drove it) ----------------------

    def _swerve_holder(self) -> tuple[bool, Optional[str]]:
        """(known, holder) of the swerve lease; read at most once a second."""
        t, h = self._swerve_lease
        if time.monotonic() - t < 1.0:
            return h
        try:
            info = self._call("swerve", "GET", "/lease", timeout=1)
            h = (True, None if info is None else info.get("client"))
        except RequestError:
            h = (False, None)
        self._swerve_lease = (time.monotonic(), h)
        return h

    def _anomaly_check(self, st: dict) -> None:
        """Watcher (10 Hz). Guarded so it can't fire while the base is driven: a session is open, no goal runs,
        pose_servo isn't moving / braking / holding, >= ANOMALY_QUIET_S since that, and the swerve lease is
        pose_servo's or nobody's. Each idle stretch has its own reference pose."""
        a = self.anomaly
        now = time.monotonic()
        status = st.get("status")
        running = any(not g.done.is_set() for g in self.goals.values())
        if running or status in ("moving", "braking", "holding"):
            self._base_busy_t = now
        quiet = self.session is not None and not running and now - self._base_busy_t >= ANOMALY_QUIET_S
        if quiet and not st.get("enabled"):          # pose_servo off: someone else may drive (teleop)
            known, holder = self._swerve_holder()
            quiet = known and holder is None
        pose = self.pose_now() if quiet else None
        if pose is None:
            a["checking"], a["ref"] = False, None
            return
        a["checking"] = True
        ref = a["ref"]
        if ref is None or ref[3] != pose[3]:          # a new idle stretch (or an estimator reset): a new reference
            a["ref"] = pose
            return
        d = math.hypot(pose[0] - ref[0], pose[1] - ref[1])
        dth = math.degrees(wrap(pose[2] - ref[2]))
        if d < ANOMALY_M and abs(dth) < ANOMALY_DEG:
            return
        a["ref"] = pose
        a["count"] += 1
        a["last"] = {"t": time.time(), "moved_m": round(d, 4), "turned_deg": round(dth, 2)}
        align = st.get("align") or {}
        self._robot_turn(
            f"base moved {d * 100:.1f} cm / {dth:+.1f}° while nothing drove it (pose_servo {status}"
            + (", wheels aligned for the arm, drives coasting" if align.get("twist") else "")
            + "): something pushed it (the arm against an obstacle?)", level="warn", source="base",
            moved_m=round(d, 4), turned_deg=round(dth, 2), from_world=[round(v, 4) for v in ref[:3]],
            to_world=[round(v, 4) for v in pose[:3]])

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
                    z.write(p, os.path.join(f"agent_{sid}", os.path.relpath(p, d)))
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
        return {**lock_state(self._locks, chock={"swerve", "pose_servo", "arm", "agent_interface"}, cover={"camera"}),
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
                "alignment": dict(self.alignment),
                "arm": {"held": self.arm["token"] is not None, "lease_id": self.arm["lease_id"],
                        "enabled": self.arm["enabled"], "jams_seen": self.arm["jams_seen"], "error": self.arm["error"]},
                "anomaly": {"checking": self.anomaly["checking"], "count": self.anomaly["count"],
                            "last": self.anomaly["last"]},
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


CORR_MAX_M = 0.05          # m per axis: session placement corrections
CORR_MAX_DEG = 3.0         # degrees per axis


def _zero_corr() -> dict:
    return {"camera": {"xyz": [0.0, 0.0, 0.0], "rpy_deg": [0.0, 0.0, 0.0]},
            "arm": {"xyz": [0.0, 0.0, 0.0], "rpy_deg": [0.0, 0.0, 0.0]}}


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    """R = Rz(y) Ry(p) Rx(r) (fixed axes X, Y, Z, as URDF)."""
    from ..description.frames import rpy_to_matrix
    return rpy_to_matrix(r, p, y)


def _matrix_rpy(R: np.ndarray):
    from ..description.frames import matrix_to_rpy
    return matrix_to_rpy(R)


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
