"""The arm node's read-only lookups (no lease; nothing moves): IK checks, kinematics, geometry, link frames,
and the arm frame's pose in the robot base frame. Each takes the kinematics, the latest reading and (for IK
seeds) the command in force; the node passes its own."""
from __future__ import annotations

import base64
import math
import time
from typing import Callable, Optional

import numpy as np
import pinocchio as pin

from ..common.lease import RequestError
from . import config as C
from .config import N
from .kin import ArmKinematics, pose_public
from .planner import Planner
from .render import VIEW_DEFAULTS

AIB_CACHE_S = 10.0     # s arm_in_base keeps the description node's answer


def need_kin(kin: Optional[ArmKinematics]) -> ArmKinematics:
    if kin is None:
        raise RequestError(503, "the arm has not connected yet (no model)")
    return kin


def ik_check(kin, reading, q_cmd, pose: pin.SE3, seed: str = "command", max_rot_deg: Optional[float] = None,
             render: Optional[dict] = None) -> dict:
    """IK seed only (never taken as where the arm is): "command" = ``q_cmd`` (the command while servoing,
    else None), else the measured pose; "measured" = the measured pose. With ``render`` (view options,
    render.VIEW_DEFAULTS) the answer also carries ``image``: a PNG data URL of the query."""
    if render is not None:
        out, png = ik_image(kin, reading, q_cmd, pose, seed, max_rot_deg, render)
        return {**out, "image": "data:image/png;base64," + base64.b64encode(png).decode("ascii"),
                "view": {**VIEW_DEFAULTS, **{k: v for k, v in render.items() if v is not None}}}
    kin = need_kin(kin)
    r = reading
    if seed == "measured" or q_cmd is None:
        if r is None:
            raise RequestError(503, "no reading yet")
        q0 = r.q
    else:
        q0 = q_cmd
    rot_tol = None if max_rot_deg is None else math.radians(max_rot_deg)
    out = Planner.run("ik", kin, pose, np.asarray(q0, float).copy(), rot_tol=rot_tol)   # in the planner process
    return out.public()


def ik_image(kin, reading, q_cmd, pose: pin.SE3, seed: str = "command", max_rot_deg: Optional[float] = None,
             view: Optional[dict] = None) -> tuple[dict, bytes]:
    """The IK check and its picture (render.py), both in the planner process: (IK answer, PNG).
    Shows the measured arm (and gripper gap), so it needs a reading."""
    kin = need_kin(kin)
    r = reading
    if r is None:
        raise RequestError(503, "no reading yet")
    q0 = r.q if (seed == "measured" or q_cmd is None) else q_cmd
    gap = C.rad_to_gap(r.g_q) if math.isfinite(r.g_q) else 0.0
    rot_tol = None if max_rot_deg is None else math.radians(max_rot_deg)
    out, png = Planner.run("ik_render", kin, pose, np.asarray(q0, float).copy(), np.asarray(r.q, float).copy(),
                           min(max(gap, 0.0), C.GAP_MAX), dict(view or {}), rot_tol=rot_tol)
    return out.public(), png


def kinematics(kin, q, frame: str = "arm") -> dict:
    frame = "arm" if frame == "base" else frame      # "base": the old name of the arm frame
    kin = need_kin(kin)
    q = np.asarray(q, float)
    if q.shape != (N,) or not np.all(np.isfinite(q)):
        raise ValueError(f"a configuration is {N} finite joint angles (rad)")
    T = kin.fk(q)
    J = kin.jacobian(q, frame)
    return {"q": q.tolist(), "within_limits": kin.within_limits(q), "tcp": pose_public(T),
            "frame": frame, "jacobian": J.round(9).tolist(), **kin.manipulability(J)}


def kinematics_batch(kin, qs: list, frame: str = "arm") -> list:
    """Lookups at many configurations, in the planner process."""
    kin = need_kin(kin)
    arr = np.asarray(qs, float)
    if arr.ndim != 2 or arr.shape[1] != N or not np.all(np.isfinite(arr)):
        raise ValueError(f"each configuration is {N} finite joint angles (rad)")
    return Planner.run("kinematics", kin, [list(map(float, q)) for q in arr], frame="arm" if frame == "base" else frame)


def arm_in_base(description_node: str, cache: Optional[dict], frame: str = "arm",
                robot: Optional[str] = None) -> tuple[dict, dict]:
    """The arm frame's pose in the robot base frame (p_base = T p_arm), from the description node's
    ``frame`` (the arm's; cached AIB_CACHE_S); the robot profile's (``robot``, robots/) shipped default if that
    node is unreachable. Says which. Returns (the answer, the cache to keep)."""
    now = time.monotonic()
    c = cache
    if c and now - c["t"] < AIB_CACHE_S:
        return c["v"], c
    try:
        from ..common.client import NodeClient
        with NodeClient(description_node, timeout=1.5) as cl:
            d = cl.get("/transform", **{"from": "base", "to": frame})
        T, src = np.asarray(d["matrix"], float), f"description node ({description_node!r}): frame {frame!r}"
    except Exception as e:
        from ...robots import load as load_robot
        from ..description.frames import FrameTree
        T = FrameTree(load_robot(robot).description["frames"]).T("base", frame)
        src = f"shipped default (description node unreachable: {type(e).__name__})"
    v = {"T": T, "source": src}
    return v, {"t": now, "v": v}


def geometry(kin, reading, ik: Callable[..., dict], aib: Callable[[], dict], q=None, pose: Optional[pin.SE3] = None,
             seed: str = "measured", max_rot_deg: Optional[float] = None, gap_mm: Optional[float] = None) -> dict:
    """Gripper and finger geometry, and every box of the arm, as numbers in both the arm and the robot
    base frame: at the measured configuration, at ``q``, or at the IK solution of a TCP ``pose``.
    ``ik(pose, seed, max_rot_deg)``: the node's IK check; ``aib()``: its arm_in_base."""
    from ..swerve.constants import FRAME
    from .body import GRASP_DEPTH, JAW_HEIGHT_ROOT, JAW_HEIGHT_TIP, JAW_LENGTH, JAW_THICKNESS, body
    kin = need_kin(kin)
    r = reading
    ik_out = None
    if pose is not None:
        ik_out = ik(pose, seed, max_rot_deg)
        qv, which = np.asarray(ik_out["q"], float), "ik"
    elif q is not None:
        qv, which = np.asarray(q, float), "q"
        if qv.shape != (N,) or not np.all(np.isfinite(qv)):
            raise ValueError(f"a configuration is {N} finite joint angles (rad)")
    else:
        if r is None:
            raise RequestError(503, "no reading yet")
        qv, which = np.asarray(r.q, float), "measured"
    if gap_mm is not None:
        gap, gap_src = float(gap_mm) / 1000, "given"
    elif r is not None and math.isfinite(r.g_q):
        gap, gap_src = C.rad_to_gap(r.g_q), "measured"
    else:
        gap, gap_src = 0.0, "unknown (no reading): 0"
    F = [T.homogeneous for T in kin.frames(qv)]
    B = body(F, gap)
    a = aib()
    Tb = a["T"]
    rnd = lambda v: [round(float(x), 5) for x in v]
    pt = lambda p: {"arm": rnd(p), "base": rnd(Tb[:3, :3] @ p + Tb[:3, 3])}
    vec = lambda v: {"arm": rnd(v), "base": rnd(Tb[:3, :3] @ v)}
    pose2 = lambda T: {"arm": pose_public(pin.SE3(T)), "base": pose_public(pin.SE3(Tb @ T))}
    jaws = {k: {"tip": pt(j["tip"]), "inner_face_center": pt(j["inner_face_center"]),
                "inner_face_root": pt(j["inner_face_root"]), "outer_face_tip": pt(j["outer_face_tip"]),
                "inward": vec(j["inward"])} for k, j in B["jaws"].items()}
    boxes, allc, grip = [], [], []
    for name, link, corners, bp in B["boxes"]:
        cb = corners @ Tb[:3, :3].T + Tb[:3, 3]
        allc.append(cb)
        if link in ("end", "finger"):
            grip.append(cb)
        boxes.append({"name": name, "link": link, "center": pt(bp["center"]), "size_m": rnd(bp["size"]),
                      "axes_base": [rnd(Tb[:3, :3] @ bp["axes"][:, k]) for k in range(3)]})
    allc, grip = np.vstack(allc), np.vstack(grip)
    h = FRAME / 2
    ext = lambda P: {"min": rnd(P.min(0)), "max": rnd(P.max(0))}
    past = {"front": allc[:, 0].max() - h, "rear": -h - allc[:, 0].min(), "left": allc[:, 1].max() - h,
            "right": -h - allc[:, 1].min()}
    return {
        "which": which, "q": rnd(qv), "within_limits": kin.within_limits(qv), "ik": ik_out,
        "gap_mm": round(B["gap"] * 1000, 2), "gap_source": gap_src,
        "frames": {
            "arm_in_base": {"matrix": [rnd(row) for row in Tb], "xyz": rnd(Tb[:3, 3]), "source": a["source"]},
            "note": "Every point is given in both frames: p_base = R p_arm + t (arm_in_base). The arm frame is "
                    "the arm's URDF base_link: the underside of its mount plate on joint 1's axis, axes parallel to "
                    "the robot base. Motion commands (/servo, /track, /chunk, /ik) take TCP poses in the arm frame.",
        },
        "tcp": {**pose2(B["tcp"]), "offset_in_end_link": kin.tcp_offset,
                "note": "the point motion commands target, near the wrist (tcp_offset behind the fingertips)"},
        "fingertips": {"left": pt(B["jaws"]["left"]["tip"]), "right": pt(B["jaws"]["right"]["tip"]),
                       "center": pose2(B["end_link"]),
                       "note": "left/right: each jaw's tip on its inner (gripping) face, mid height; center: end_link, "
                               "midway between them. end_link's x axis is the approach direction (out between the "
                               "jaws), y the jaw travel (left jaw at +y), z the jaws' height"},
        "axes": {"approach": vec(B["approach"]), "jaw_travel": vec(B["jaw_axis"]), "up": vec(B["up"])},
        "jaws": jaws,
        "jaw_shape_m": {"grasp_depth": round(GRASP_DEPTH, 4), "length": round(JAW_LENGTH, 4),
                        "thickness": round(JAW_THICKNESS, 4), "height_root": JAW_HEIGHT_ROOT, "height_tip": JAW_HEIGHT_TIP,
                        "note": "approximate (from the meshes). Each jaw is a prong from its tip back along -approach "
                                "to inner_face_root, where the jaw carriage begins: grasp_depth is how far an object "
                                "can sit between the jaws, measured from the tips. The inner faces grip; each jaw is "
                                "`thickness` thick outward from its inner face and tapers in height from root to tip"},
        "extent_base": {"arm": ext(allc), "gripper": ext(grip),
                        "past_footprint_m": {k: round(float(v), 4) for k, v in past.items()},
                        "note": f"axis-aligned bounds of every box in the robot base frame; past_footprint_m: how far "
                                f"the arm reaches beyond each side of the {FRAME:.4f} m square frame (negative: inside)"},
        "boxes": boxes,
    }


def links(kin, reading, q=None) -> list:
    """The joint frames, end_link and tcp at ``q`` (default measured), for drawing."""
    kin = need_kin(kin)
    if q is None:
        if reading is None:
            raise RequestError(503, "no reading yet")
        q = reading.q
    return [pose_public(T) for T in kin.frames(q)]
