"""A recorded sim teleop session or episode (docs/collect.md), read for a replay: the headset frames of the operator
stream, the rigid scene's pose frames and the scenes they index.

Accepted: a session directory (``session.json``; ``<name>.NNN.bin`` per stream; optionally one of its episodes by
id, from ``episodes.jsonl``) or an episode directory (``episode.json``; ``<name>.bin``). Times are the robot's
monotonic clock (the records' ``t_rx``).

Body tracking (the frames' optional ``body`` block, quest_env BODY_TRACKING.md) is not drawn: a replay counts the
frames that carry one (``Recording.body_frames``) and leaves the block in the recording, where
``tools/quest_teleop/body.py`` parses it.
"""
from __future__ import annotations

import bisect
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

from robot_utils.collect.episodes import read_episodes
from robot_utils.collect.records import iter_raw
from robot_utils.collect.scenes import SCENE_DIR, read_scenes
from robot_utils.collect.streams import rotated_files
from robot_utils.scene3d.format import Frame, decode_frame

from . import ar


@dataclass
class HeadsetSample:
    t: float
    session: Optional[str]
    head: Optional[tuple]                    # (p, R) VIEW in LOCAL, None when untracked
    hands: dict                              # side -> (p, R) grip in LOCAL, or None
    robot_ar: Optional[dict]                 # the record's block (shared between equal consecutive blocks)


@dataclass
class Recording:
    path: Path
    headset: list[HeadsetSample]
    scene_t: list[float]
    scene_raw: list[bytes]
    scene_dir: Path
    scenes_meta: dict = field(default_factory=dict)       # scene_id -> scenes.jsonl line
    window: Optional[tuple[float, float]] = None       # what to render (an episode); everything is loaded, so a
    label: str = ""                                    # reconstructed placement sees the session's history
    body_frames: int = 0                               # headset frames carrying a body block (not drawn)
    _scenes: dict = field(default_factory=dict)

    # -- lookups ----------------------------------------------------------------------------------------------

    def headset_at(self, t: float) -> Optional[HeadsetSample]:
        i = bisect.bisect_right(self._ht, t) - 1
        return self.headset[i] if i >= 0 else None

    def scene_frame_at(self, t: float) -> Optional[Frame]:
        i = bisect.bisect_right(self.scene_t, t) - 1
        return decode_frame(self.scene_raw[i])[0] if i >= 0 else None

    def scene(self, scene_id: str) -> Optional[tuple[dict, bytes]]:
        """(index, blob) of a saved scene, or None (not saved: missed, or recorded without the scene saver)."""
        if scene_id not in self._scenes:
            j, b = self.scene_dir / f"{scene_id}.json", self.scene_dir / f"{scene_id}.blob"
            self._scenes[scene_id] = (json.loads(j.read_text()), b.read_bytes()) if j.exists() and b.exists() \
                else None
        return self._scenes[scene_id]

    def view(self, scene_id: str) -> dict:
        """The headset view setting (style, role) when the scene was saved; {} if unknown."""
        return (self.scenes_meta.get(scene_id) or {}).get("view") or {}

    def __post_init__(self):
        self._ht = [h.t for h in self.headset]

    @property
    def has_logged_placement(self) -> bool:
        return any(h.robot_ar is not None for h in self.headset)


def _stream_files(d: Path, kind: str) -> list[Path]:
    """The files of the operator stream (``kind`` "oplog") or the scene pose stream ("scene3d") in a session or
    an episode directory."""
    names = []
    meta = d / "session.json"
    if meta.exists():
        for s in json.loads(meta.read_text()).get("streams", []):
            spec = s.get("spec", "")
            if (kind == "oplog" and spec.split("?")[0].endswith("/oplog/stream")) or \
                    (kind == "scene3d" and spec.split("?")[0].endswith("/scene3d/stream")):
                names.append(s["name"])
        files = [f for n in names for f in rotated_files(d, n)]
        if files:
            return files
    suffix = ".oplog" if kind == "oplog" else ".scene3d"
    return sorted(p for p in d.glob(f"*{suffix}.bin")) + sorted(p for p in d.glob(f"*{suffix}.[0-9][0-9][0-9].bin"))


def _same_block(a: Optional[dict], b: Optional[dict]) -> bool:
    if a is None or b is None:
        return a is b
    return a.get("placements") == b.get("placements") and a.get("sim_in_local") == b.get("sim_in_local") and \
        a.get("scale") == b.get("scale") and a.get("placed") == b.get("placed") and a.get("views") == b.get("views")


def load(path, episode: Optional[str] = None) -> Recording:
    d = Path(path)
    window = None
    label = d.name
    if episode is not None:
        eps = [e for e in read_episodes(d / "episodes.jsonl") if e["id"] == episode]
        if not eps:
            raise ValueError(f"no episode {episode!r} in {d / 'episodes.jsonl'}")
        e = eps[0]
        if e.get("t_start") is None or e.get("t_stop") is None:
            raise ValueError(f"episode {episode} has no start / stop time")
        window = (float(e["t_start"]), float(e["t_stop"]))
        label = f"{d.name}/{episode}"
    elif (d / "episode.json").exists():
        w = json.loads((d / "episode.json").read_text()).get("window")
        window = (float(w[0]), float(w[1])) if w else None
    oplog = _stream_files(d, "oplog")
    if not oplog:
        raise ValueError(f"{d}: no operator stream (quest_teleop.oplog) recorded")
    headset: list[HeadsetSample] = []
    body_frames = 0
    prev_block = None
    for _, h, _ in iter_raw(oplog):
        if h.get("type") != "headset" or not isinstance(h.get("frame"), dict):
            continue
        t = h.get("t_rx")
        if t is None:
            continue
        f = h["frame"]
        body_frames += isinstance(f.get("body"), dict)
        block = f.get("robot_ar") if isinstance(f.get("robot_ar"), dict) else None
        if _same_block(block, prev_block):
            block = prev_block
        prev_block = block
        headset.append(HeadsetSample(float(t), f.get("session_id"), ar.head_pose(f),
                                     {s: ar.hand_pose(f, s) for s in ("left", "right")}, block))
    headset.sort(key=lambda x: x.t)
    scene_t, scene_raw = [], []
    for _, h, payload in iter_raw(_stream_files(d, "scene3d")):
        if h.get("type") != "scene3d":
            continue
        t = h.get("t_rx")
        if t is None:
            continue
        scene_t.append(float(t))
        scene_raw.append(payload)
    order = np.argsort(scene_t, kind="stable")
    scene_t = [scene_t[i] for i in order]
    scene_raw = [scene_raw[i] for i in order]
    meta = {x["scene_id"]: x for x in read_scenes(d) if "missed" not in x}
    return Recording(d, headset, scene_t, scene_raw, d / SCENE_DIR, meta, window, label, body_frames)
