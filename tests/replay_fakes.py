"""A synthetic recorded sim teleop session for the replay tests, written in the collect layout (docs/collect.md):
``session.json``, ``quest_teleop.oplog.000.bin`` (headset records with head, controllers and, unless ``logged`` is
False, the ``robot_ar`` block a headset logs), ``sim.scene3d.000.bin`` (RSF1 frames as ``scene3d`` records) and
``scene3d/<id>.json|.blob`` + ``scenes.jsonl``.

The scene: a white marker sphere (an object) moving along sim y, a gripper ghost (kind ``ghost``) with a green colour
override, a static table. The operator: head 1.6 m above a LOCAL floor at 0, turning and stepping; the right
controller in front of the face, the left one untracked half the time. The sim resets once (``reset_seq`` 0 -> 1) at
``reset_at``: the placement re-anchors at the head then.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from robot_utils.nodes.common.protocol import pack_record
from robot_utils.replay import ar
from robot_utils.scene3d.format import Frame, SceneBuilder, encode_frame

MARKER_R = 0.03
WHITE = (1.0, 1.0, 1.0, 1.0)
GHOST_RGBA = (0.1, 0.9, 0.2, 1.0)


def scene():
    b = SceneBuilder("test")
    b.add_body("marker", "object", [0, 0, 1.2, 0, 0, 0, 1], True,
               geoms=[{"type": "sphere", "radius": MARKER_R, "rgba": WHITE, "role": "both"}])
    b.add_body("ghost:goal/arm_end_link", "ghost", [0.3, 0.3, 1.0, 0, 0, 0, 1], True,
               geoms=[{"type": "box", "half_extents": [0.03, 0.03, 0.03], "rgba": [0.5, 0.5, 0.5, 1], "role": "visual"}])
    b.add_body("table", "object", [0, 3.0, 0.4, 0, 0, 0, 1], False,
               geoms=[{"type": "box", "half_extents": [0.3, 0.3, 0.4], "rgba": [0.5, 0.3, 0.55, 1]},
                      {"type": "plane", "half_extents": [0, 0], "rgba": [1, 1, 1, 1]}])
    return b.build()


def marker_at(t: float) -> np.ndarray:
    return np.array([0.0, 0.15 * math.sin(t * 2.0), 1.2])


def head_at(t: float):
    """Head (VIEW) in LOCAL: 1.6 m up, turning +-0.25 rad, stepping."""
    yaw = 0.25 * math.sin(t * 1.3)
    r = np.array([[math.cos(yaw), 0, math.sin(yaw)], [0, 1, 0], [-math.sin(yaw), 0, math.cos(yaw)]])
    pitch = -0.15
    r = r @ np.array([[1, 0, 0], [0, math.cos(pitch), -math.sin(pitch)], [0, math.sin(pitch), math.cos(pitch)]])
    return np.array([0.05 * t, 1.6, -0.02 * t]), r


def right_at(t: float):
    hp, hr = head_at(t)
    return hp + hr @ np.array([0.08, -0.25, -0.45]), hr


def write_session(d: Path, *, seconds: float = 2.0, rate: float = 50.0, scene_hz: float = 30.0,
                  logged: bool = True, reset_at: float = 1.0, offset=(0.0, 0.0, 0.0), yaw: float = 0.0,
                  scale: float = 1.0, floor: float = 0.0, t0: float = 1000.0) -> dict:
    d = Path(d)
    d.mkdir(parents=True, exist_ok=True)
    index, blob = scene()
    sid = index["scene_id"]
    (d / "scene3d").mkdir()
    (d / "scene3d" / f"{sid}.json").write_text(json.dumps(index))
    (d / "scene3d" / f"{sid}.blob").write_bytes(blob)
    (d / "scene3d" / "scenes.jsonl").write_text(json.dumps({"scene_id": sid, "t_rx": t0, "view": {
        "style": "flat", "role": "auto"}, "files": [f"{sid}.json", f"{sid}.blob"]}) + "\n")
    streams = [{"spec": "quest_teleop/oplog/stream", "name": "quest_teleop.oplog", "url": "", "params": {}},
               {"spec": "sim/scene3d/stream", "name": "sim.scene3d", "url": "", "params": {"format": "bin"},
                "framing": "rsf1"}]
    (d / "session.json").write_text(json.dumps({"robot": "test", "streams": streams}))
    reset = lambda t: 0 if t < t0 + reset_at else 1        # noqa: E731
    # scene frames
    with open(d / "sim.scene3d.000.bin", "wb") as f:
        n = int(seconds * scene_hz) + 1
        for k in range(n):
            t = t0 + k / scene_hz
            poses = [list(marker_at(t - t0)) + [0, 0, 0, 1], [0.3, 0.3, 1.0, 0, 0, 0, 1]]
            fr = Frame(k + 1, t - t0, reset(t), sid, np.array(poses), {1: list(GHOST_RGBA)})
            raw = encode_frame(fr)
            f.write(pack_record({"type": "scene3d", "t_rx": t, "seq": k + 1, "t_sim": t - t0, "reset_seq": reset(t),
                                 "scene_id": sid, "n": 2, "m": 1}, raw))
    # headset records; the placement as the headset makes it
    pl = ar.Placement(offset=np.asarray(offset, float), yaw=yaw, scale=scale)
    views = [{"eye": k, "fov": list(v.fov), "pose": ar.matrix16(v.p, v.r)} for k, v in ar.default_views().items()]
    truth = []
    with open(d / "quest_teleop.oplog.000.bin", "wb") as f:
        n = int(seconds * rate) + 1
        for k in range(n):
            t = t0 + k / rate
            hp, hr = head_at(t - t0)
            rp, rr = right_at(t - t0)
            left_tracked = k % 40 < 20
            pl.update("s1", ar.flu(hp, hr), reset(t), False, floor)
            frame = {"schema": "teleop.quest_state.v1", "session_id": "s1", "frame_seq": k,
                     "head": {"position_valid": True, "orientation_valid": True, "position_tracked": True,
                              "orientation_tracked": True, "pose": ar.matrix16(hp, hr)},
                     "hands": {"right": {"active": True, "position_tracked": True, "orientation_tracked": True,
                                         "pose": ar.matrix16(rp, rr)},
                               "left": {"active": left_tracked, "position_tracked": left_tracked,
                                        "orientation_tracked": left_tracked,
                                        "pose": ar.matrix16(hp + hr @ np.array([-0.2, -0.3, -0.4]), hr)
                                        if left_tracked else None}}}
            if logged:
                frame["robot_ar"] = pl.block("LOCAL_FLOOR", views)
            f.write(pack_record({"type": "headset", "t_rx": t, "frame": frame}))
            p, r = pl.sim_in_local()
            truth.append({"t": t, "sim_in_local": (p, r), "scale": pl.scale, "placements": pl.placements})
    return {"scene_id": sid, "truth": truth, "t0": t0}
