"""A synthetic scene and a fake 3d_perception node for tests and offline trials (no hardware).

The scene is defined in the WORLD frame (floor, vertical rectangles such as a whiteboard,
axis-aligned boxes); frames are rendered from the robot's current pose (``pose_fn``) through the
Gemini 336L's real calibration (colour 1280x720, registered depth grid 640x360). The fake node
serves ``/calibration``, ``/frame`` (fused record: colour JPEG + depth) and ``/state`` like
3d_perception.
"""
from __future__ import annotations

import io
import json
import math
import struct
import time
from dataclasses import dataclass, field
from typing import Callable

import numpy as np
from PIL import Image

from .geometry import Camera, pose_matrix, rays

# the robot's live calibration (2026-11-08)
CALIBRATION = {
    "ready": True,
    "color": {"fx": 609.517578125, "fy": 609.6476440429688, "cx": 640.5364990234375, "cy": 354.30474853515625,
              "width": 1280, "height": 720,
              "distortion": {"k1": -0.03221883252263069, "k2": 0.03856267035007477, "k3": -0.013476075604557991,
                             "k4": 0.0, "k5": 0.0, "k6": 0.0, "p1": -0.000343605614034459, "p2": -0.00011862569954246283}},
    "depth_grid": {"fx": 304.7587890625, "fy": 304.8238220214844, "cx": 320.01824951171875, "cy": 176.90237426757812,
                   "width": 640, "height": 360,
                   "distortion": {"k1": -0.03221883252263069, "k2": 0.03856267035007477, "k3": -0.013476075604557991,
                                  "k4": 0.0, "k5": 0.0, "k6": 0.0, "p1": -0.000343605614034459, "p2": -0.00011862569954246283}},
    "base_to_color": [[-0.067749053, -0.385963038, 0.920023222, -0.001966901],
                      [-0.997690655, 0.021794742, -0.064324815, -0.023613158],
                      [0.004775844, -0.922256268, -0.386548733, 1.12712125], [0.0, 0.0, 0.0, 1.0]],
    "base_calibration": {"calibrated": True},
}


@dataclass
class Rect:
    """Vertical rectangle: centre (x, y) on the floor plan, facing heading (its normal), width, z range."""
    x: float
    y: float
    facing: float
    width: float
    z0: float
    z1: float
    color: tuple = (235, 235, 235)


@dataclass
class Box:
    x0: float
    y0: float
    x1: float
    y1: float
    z1: float
    color: tuple = (190, 140, 80)


@dataclass
class Scene:
    rects: list = field(default_factory=list)
    boxes: list = field(default_factory=list)

    @np.errstate(all="ignore")
    def render(self, pose: tuple) -> tuple[np.ndarray, bytes]:
        """(depth mm uint16 (360, 640), colour JPEG 1280x720) seen from base pose (x, y, theta)."""
        cam = Camera.from_calibration(CALIBRATION)
        T_bc = np.array(CALIBRATION["base_to_color"])
        T_wb = pose_matrix(*pose)
        T_wc = T_wb @ T_bc
        R, o = T_wc[:3, :3], T_wc[:3, 3]
        d = rays(cam) @ R.T                                   # world ray per grid pixel (scaled by z_c)
        H, W = cam.height, cam.width
        best = np.full((H, W), np.inf)                        # parameter t = colour-frame depth z
        col = np.zeros((H, W, 3), np.uint8)
        col[:] = (60, 60, 64)
        # floor z = 0
        with np.errstate(divide="ignore", invalid="ignore"):
            t = -o[2] / d[..., 2]
        m = (t > 0) & (t < best)
        best[m] = t[m]
        px, py = o[0] + d[..., 0] * t, o[1] + d[..., 1] * t
        chk = ((np.floor(px * 2) + np.floor(py * 2)) % 2 == 0)
        fl = np.where(chk[..., None], np.array([150, 150, 155], np.uint8), np.array([135, 135, 140], np.uint8))
        col[m] = fl[m]
        for r in self.rects:
            n = np.array([math.cos(r.facing), math.sin(r.facing), 0.0])
            e = np.array([-n[1], n[0], 0.0])
            c = np.array([r.x, r.y, 0.0])
            with np.errstate(divide="ignore", invalid="ignore"):
                t = ((c - o) @ n) / (d @ n)
            p = o + d * t[..., None]
            s = (p - c) @ e
            m = (t > 0) & (t < best) & (np.abs(s) <= r.width / 2) & (p[..., 2] >= r.z0) & (p[..., 2] <= r.z1)
            best[m] = t[m]
            col[m] = r.color
        for b in self.boxes:
            lo = np.array([b.x0, b.y0, 0.0])
            hi = np.array([b.x1, b.y1, b.z1])
            with np.errstate(divide="ignore", invalid="ignore"):
                t1, t2 = (lo - o) / d, (hi - o) / d
            tmin = np.nanmax(np.minimum(t1, t2), -1)
            tmax = np.nanmin(np.maximum(t1, t2), -1)
            m = (tmax >= tmin) & (tmin > 0) & (tmin < best)
            best[m] = tmin[m]
            col[m] = b.color
        depth = np.where(np.isfinite(best) & (best < 8.0), best * 1000, 0).astype(np.uint16)
        img = Image.fromarray(col).resize((cam.color_w, cam.color_h), Image.NEAREST)
        buf = io.BytesIO()
        img.save(buf, "JPEG", quality=85)
        return depth, buf.getvalue()


def demo_scene() -> Scene:
    """A whiteboard 3 m ahead facing the origin, a box to the left, a low box (cardboard-like) ahead-right."""
    return Scene(rects=[Rect(3.0, 0.3, math.pi, 1.9, 0.75, 1.75)],
                 boxes=[Box(1.6, 0.9, 2.0, 1.3, 0.6), Box(1.2, -0.9, 1.5, -0.6, 0.35, (170, 120, 60))])


def fake_perception_app(scene: Scene, pose_fn: Callable[[], tuple]):
    """FastAPI app mimicking 3d_perception's /calibration, /frame, /state, /time."""
    from fastapi import FastAPI
    from fastapi.responses import Response

    app = FastAPI(title="fake 3d_perception")
    seq = {"n": 0}

    @app.get("/calibration")
    async def calibration():
        return CALIBRATION

    @app.get("/time")
    async def t():
        return {"mono": time.monotonic(), "wall": time.time(), "epoch_minus_mono": time.time() - time.monotonic()}

    @app.get("/state")
    async def state():
        return {"state": "running", "mode": "full"}

    @app.get("/frame")
    async def frame(color: bool = True, depth: bool = True, max_age: float = 0.5, timeout: float = 3.0):
        seq["n"] += 1
        t_c = time.monotonic()
        dm, jpg = scene.render(pose_fn())
        db = dm.tobytes()
        payload = (jpg if color else b"") + (db if depth else b"")
        hdr = {"type": "fused", "seq": seq["n"], "t_color": t_c, "t_depth": t_c - 0.1, "mode": "full",
               "color": {"format": "MJPG", "width": 1280, "height": 720, "offset": 0 if color else None, "len": len(jpg) if color else 0},
               "depth": {"width": 640, "height": 360, "dtype": "<u2", "units": "mm", "frame": "color_optical",
                         "offset": (len(jpg) if color else 0) if depth else None, "len": len(db) if depth else 0,
                         "valid": float((dm > 0).mean())},
               "scale": 0.5}
        from ..common.protocol import pack_record
        return Response(pack_record(hdr, payload), media_type="application/octet-stream")

    return app


def fake_arm_app(tcp=(0.25, 0.0, 0.2)):
    """FastAPI app standing in for the arm node (no pinocchio): the lease, enable / disable, servo / home / gripper /
    cancel (recorded, nothing moves), /kinematics (a fixed TCP), /ik, /geometry, /state and /jam. ``app.state.arm``
    holds what it saw (``calls``) and lets a test add jams (``add_jam``)."""
    import secrets

    from fastapi import FastAPI, Header, HTTPException
    from fastapi.responses import JSONResponse

    app = FastAPI(title="fake arm")
    st = {"token": None, "lease_id": 0, "enabled": False, "calls": [], "jams": [], "tcp": list(tcp), "motion": 0}
    app.state.arm = st

    def holder(tok):
        if st["token"] is None or tok != st["token"]:
            raise HTTPException(403, "not the lease holder")

    def add_jam(joint=2, residual=5.1, band=4.0, response="hold"):
        st["jams"].append({"t": time.monotonic(), "wall": time.time(), "joint": joint, "residual_nm": residual,
                           "band_nm": band, "held_s": 1.0, "response": response, "source": "hold",
                           "residuals_nm": [0.0] * 6, "count": len(st["jams"]) + 1})
    st["add_jam"] = add_jam

    @app.post("/lease")
    async def lease(body: dict):
        if st["token"] is not None:
            raise HTTPException(409, "lease held")
        st["token"], st["lease_id"] = secrets.token_hex(8), st["lease_id"] + 1
        return {"token": st["token"], "client": body.get("client"), "lease_id": st["lease_id"], "heartbeat": 1.0}

    @app.post("/lease/renew")
    async def renew(x_lease_token: str = Header(None)):
        holder(x_lease_token)
        return {"lease_id": st["lease_id"], "expires_in": 1.0}

    @app.delete("/lease")
    async def release(x_lease_token: str = Header(None)):
        holder(x_lease_token)
        st["token"], st["enabled"] = None, False
        return {"released": True}

    @app.post("/enable")
    async def enable(x_lease_token: str = Header(None)):
        holder(x_lease_token)
        st["enabled"] = True
        return {"enabled": True, "state": "floating"}

    @app.post("/disable")
    async def disable(x_lease_token: str = Header(None)):
        holder(x_lease_token)
        st["enabled"] = False
        return {"enabled": False, "state": "disabled"}

    @app.post("/estop")
    async def estop(body: dict = None):
        st["token"], st["enabled"] = None, False
        return {"latched": True, "reason": (body or {}).get("reason", ""), "source": (body or {}).get("source", "")}

    def motion(kind, body, tok):
        holder(tok)
        st["calls"].append((kind, body))
        st["motion"] += 1
        return {"id": st["motion"], "kind": kind, "duration": 1.0, "info": {}}

    @app.post("/servo")
    async def servo(body: dict, x_lease_token: str = Header(None)):
        if body.get("pose") and body["pose"]["pos"][0] > 1.0:
            return JSONResponse(status_code=422, content={"detail": "unreachable", "code": "ik_failed", "retry": None})
        return motion("servo", body, x_lease_token)

    @app.post("/home")
    async def home(x_lease_token: str = Header(None)):
        return motion("home", {}, x_lease_token)

    @app.post("/gripper")
    async def gripper(body: dict, x_lease_token: str = Header(None)):
        holder(x_lease_token)
        st["calls"].append(("gripper", body))
        return {"goal_mm": body.get("gap_mm"), "max_force_n": body.get("max_force_n") or 20.0}

    @app.post("/cancel")
    async def cancel(x_lease_token: str = Header(None)):
        holder(x_lease_token)
        return {"cancelled": True}

    @app.get("/kinematics")
    async def kinematics():
        return {"tcp": {"pos": st["tcp"], "quat": [0.0, 0.0, 0.0, 1.0]}}

    @app.post("/ik")
    async def ik(body: dict):
        return {"ok": True, "exact": True, "q": [0.0] * 6, "nearest": body["pose"]}

    def geo(which, q, gap_mm=40.0):
        """The shape of the arm node's geometry: the arm frame on the front-right mount, one jaw box, fingertips."""
        xyz = [0.1159, -0.1459, 0.22225]
        T = [[1.0, 0.0, 0.0, xyz[0]], [0.0, 1.0, 0.0, xyz[1]], [0.0, 0.0, 1.0, xyz[2]], [0.0, 0.0, 0.0, 1.0]]
        p = st["tcp"]
        return {"which": which, "q": q, "gap_mm": gap_mm, "ik": {"ok": True, "exact": True} if which == "ik" else None,
                "frames": {"arm_in_base": {"matrix": T, "xyz": xyz, "source": "fake"}},
                "boxes": [{"name": "jaw", "link": "finger", "center": {"arm": p, "base": [0, 0, 0]},
                           "size_m": [0.03, 0.03, 0.03], "axes_base": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}],
                "fingertips": {"left": {"arm": [p[0], p[1] + 0.02, p[2]]}, "right": {"arm": [p[0], p[1] - 0.02, p[2]]},
                               "center": {"arm": {"pos": p}}},
                "extent_base": {"past_footprint_m": {"front": -0.1, "rear": -0.3, "left": -0.3, "right": -0.05}}}

    @app.get("/geometry")
    async def geometry():
        return geo("measured", [0.0] * 6)

    @app.post("/geometry")
    async def geometry_at(body: dict):
        return geo("ik" if body.get("pose") else "q", body.get("q") or [0.0] * 6, body.get("gap_mm") or 40.0)

    @app.get("/state")
    async def state():
        return {"state": "floating" if st["enabled"] else "disabled", "motion": None,
                "jam": {"count": len(st["jams"]), "active": None}}

    @app.get("/jam")
    async def jam():
        return {"config": {}, "active": st["jams"][-1] if st["jams"] else None, "count": len(st["jams"]),
                "recent": st["jams"][-20:]}

    return app
