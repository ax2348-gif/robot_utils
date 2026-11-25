"""The recorded rigid scene (docs/scene3d.md, docs/collect.md "The rigid scene") for the browser's 3D view.

A sim recording holds the pose stream (``sim.scene3d``: RSF1 frames, each in a ``scene3d`` record dated by the
recorder's arrival time ``t_rx``, the time base of every other stream) and each scene it names once
(``scene3d/<scene_id>.json`` + ``.blob``, ``scenes.jsonl``; a cut episode has its own copy). ``SceneTrack`` serves
them to the page:

- ``scenes()``: per scene its id, when it was first named (``t_rx``), the headset view then, whether it was saved;
- ``index(scene_id)``: the scene index as recorded, each body with ``draw``: the indices of its geoms a viewer
  draws (``scene3d.view.drawn_geoms``: the headset's rule, ``auto`` = the robot's and ghosts' visual geoms, the
  rest's collision geoms, planes never), so the page applies exactly the rule the headset and the offscreen replay
  use;
- ``blob(scene_id)``: the mesh blob, as recorded;
- ``frames(t0, t1, hz)``: the pose frames in a window, at most ``hz`` per second (the first of each 1/hz bucket),
  packed for the page (``pack_frames``).

The frames are the recorded RSF1 bytes, untouched: the page reads each frame's scene id (a new one switches the
geometry), its poses and its colour overrides (ghosts in their colour, alpha 0 hidden) itself.
"""
from __future__ import annotations

import json
import re
import struct
from pathlib import Path
from typing import Optional

import numpy as np

from ...scene3d.view import drawn_geoms
from ..scenes import SCENE_DIR, read_scenes
from .index import KINDS, StreamIndex

MAX_FRAMES = 20_000
_SID = re.compile(r"^[0-9a-f]{16}$")


class SceneTrack:
    def __init__(self, path: Path, stream: StreamIndex):
        self.path = Path(path)
        self.stream = stream
        self.dir = self.path / SCENE_DIR
        lines = read_scenes(self.path)
        self.meta = {x["scene_id"]: x for x in lines if "scene_id" in x}
        self._index: dict[str, dict] = {}

    @property
    def name(self) -> str:
        return self.stream.name

    def scenes(self) -> list[dict]:
        out = []
        for sid, x in self.meta.items():
            out.append({"scene_id": sid, "t_rx": x.get("t_rx"), "view": x.get("view"), "missed": x.get("missed"),
                        "saved": self._files(sid) is not None})
        return out

    def summary(self) -> dict:
        a, b = self.stream.t_range
        return {"stream": self.name, "frames": int(self.stream.n_records), "t0": a, "t1": b, "scenes": self.scenes()}

    def _files(self, sid: str) -> Optional[tuple[Path, Path]]:
        if not _SID.match(sid or ""):
            return None
        j, b = self.dir / f"{sid}.json", self.dir / f"{sid}.blob"
        return (j, b) if j.is_file() and b.is_file() else None

    def role(self, sid: str) -> str:
        """The geometry role the headset drew this scene with (``scenes.jsonl`` view), ``auto`` if not known."""
        v = (self.meta.get(sid) or {}).get("view") or {}
        r = v.get("role")
        return r if r in ("auto", "visual", "collision") else "auto"

    def index(self, sid: str) -> Optional[dict]:
        if sid not in self._index:
            f = self._files(sid)
            if f is None:
                return None
            idx = json.loads(f[0].read_text())
            role = self.role(sid)
            for b in idx.get("bodies", []):
                b["draw"] = drawn_geoms(b, role)
            idx["draw_role"] = role
            self._index[sid] = idx
        return self._index[sid]

    def blob(self, sid: str) -> Optional[bytes]:
        f = self._files(sid)
        return f[1].read_bytes() if f else None

    def frame_indices(self, t0: float, t1: float, hz: float = 0.0) -> np.ndarray:
        ix = self.stream.window(t0, t1, "scene3d")
        if not len(ix):
            return ix
        t = np.asarray(self.stream.a["t0"])[ix]
        if hz and hz > 0:
            bucket = np.floor(t * hz).astype(np.int64)
            keep = np.ones(len(ix), bool)
            keep[1:] = bucket[1:] != bucket[:-1]
            ix, t = ix[keep], t[keep]
        if len(ix) > MAX_FRAMES:
            ix = ix[np.linspace(0, len(ix) - 1, MAX_FRAMES).astype(np.int64)]
        return ix

    def frames(self, t0: float, t1: float, hz: float = 0.0) -> bytes:
        ix = self.frame_indices(t0, t1, hz)
        return pack_frames(np.asarray(self.stream.a["t0"])[ix], [self.stream.payload(int(i)) for i in ix])


def pack_frames(t: np.ndarray, payloads: list[bytes]) -> bytes:
    """``u32 BE`` header length, the JSON header ``{n}`` (padded so what follows is 8-aligned), ``n`` float64 ``t_rx``,
    ``n`` u32 byte offsets into the frame block, then the frame block: the RSF1 frames back to back (each a multiple
    of 4 bytes, so the page reads their float32 poses in place)."""
    n = len(payloads)
    hdr = json.dumps({"n": n}).encode()
    hdr += b" " * ((-(4 + len(hdr))) % 8)
    offs = np.zeros(n, "<u4")
    if n:
        offs[1:] = np.cumsum([len(p) for p in payloads[:-1]])
    return struct.pack(">I", len(hdr)) + hdr + np.asarray(t, "<f8").tobytes() + offs.tobytes() + b"".join(payloads)


def find_track(path: Path, raw: dict) -> Optional[SceneTrack]:
    """The scene track of a recording: its stream of ``scene3d`` records (the sim's ``<sim>/scene3d/stream``)."""
    for idx in raw.values():
        if idx.kind == "scene3d" or idx.types.get("scene3d"):
            if np.any(np.asarray(idx.a["kind"]) == KINDS["scene3d"]):
                return SceneTrack(path, idx)
    return None
