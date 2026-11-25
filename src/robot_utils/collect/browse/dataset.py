"""What the browser opens: a data root (sessions below it), a session, or a cut episode.

- ``Recording``: one session directory (``session.json``, rotated ``<name>.NNN.bin``) or one episode directory
  (``episode.json``, ``<name>.bin``): its streams (``StreamIndex`` each; the operator stream also as the
  ``oplog.headset`` / ``oplog.sent`` tables), its episodes and events, its time range, and the recorded rigid scene
  (``scene``: scene.py) when it has one.
- ``Library``: what a path holds: ``items()`` lists every session (with its episodes and any cut episode
  directories) and loose episode directories under the path (one level of nesting below a root, as
  ``collect run --out`` lays them out); ``open(key)`` opens one ``Recording`` (cached).
"""
from __future__ import annotations

import json
import threading
from pathlib import Path
from typing import Optional

import numpy as np

from ..episodes import read_episodes
from ..streams import rotated_files
from .index import StreamIndex
from .oplog import OplogTables
from .scene import SceneTrack, find_track


def _read_json(p: Path) -> dict:
    try:
        return json.loads(p.read_text())
    except (OSError, ValueError):
        return {}


def _jsonl(p: Path) -> list[dict]:
    out = []
    try:
        with open(p) as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        out.append(json.loads(line))
                    except ValueError:
                        pass
    except OSError:
        pass
    return out


def kind_of(path: Path) -> Optional[str]:
    if (path / "episode.json").exists():
        return "episode"
    if (path / "session.json").exists():
        return "session"
    return None


class Recording:
    def __init__(self, path: Path, cache: Optional[Path] = None):
        self.path = Path(path)
        self.kind = kind_of(self.path)
        if self.kind is None:
            raise ValueError(f"{path}: neither a session (session.json) nor an episode (episode.json)")
        self.meta = _read_json(self.path / ("episode.json" if self.kind == "episode" else "session.json"))
        self.streams: dict[str, object] = {}
        self.oplog: Optional[OplogTables] = None
        self.raw: dict[str, StreamIndex] = {}
        for name, files in self._stream_files().items():
            if not files:
                continue
            idx = StreamIndex.open(name, files, cache)
            self.raw[name] = idx
            if idx.kind == "oplog":
                self.oplog = OplogTables.open(idx, cache)
                self.streams["oplog.headset"] = self.oplog.headset
                self.streams["oplog.sent"] = self.oplog.sent
            else:
                self.streams[name] = idx
        self.scene: Optional[SceneTrack] = find_track(self.path, self.raw)

    def _stream_files(self) -> dict[str, list[Path]]:
        if self.kind == "episode":
            names = list((self.meta.get("streams") or {}).keys()) or sorted(
                p.name[:-4] for p in self.path.glob("*.bin"))
            return {n: [self.path / f"{n}.bin"] for n in names if (self.path / f"{n}.bin").exists()}
        names = [s["name"] for s in self.meta.get("streams") or [] if "name" in s]
        if not names:
            names = sorted({p.name.rsplit(".", 2)[0] for p in self.path.glob("*.[0-9][0-9][0-9].bin")})
        return {n: rotated_files(self.path, n) for n in names}

    # -- what's in it ----------------------------------------------------------------------------

    @property
    def t_range(self) -> tuple[float, float]:
        lo, hi = np.inf, -np.inf
        for s in self.streams.values():
            a, b = s.t_range
            if np.isfinite(a):
                lo, hi = min(lo, a), max(hi, b)
        if self.kind == "episode" and self.meta.get("window"):
            w = self.meta["window"]
            lo, hi = max(lo, w[0]) if np.isfinite(lo) else w[0], min(hi, w[1]) if np.isfinite(hi) else w[1]
        return (float(lo), float(hi)) if np.isfinite(lo) else (0.0, 0.0)

    def episodes(self) -> list[dict]:
        if self.kind == "episode":
            m = self.meta
            return [{k: m.get(k) for k in ("id", "task", "t_start", "t_stop", "outcome", "reason", "wall")}]
        return read_episodes(self.path / "episodes.jsonl")

    def cut_episodes(self) -> list[str]:
        d = self.path / "episodes"
        return sorted(p.name for p in d.iterdir() if (p / "episode.json").exists()) if d.is_dir() else []

    def events(self) -> list[dict]:
        if self.kind == "session":
            ev = [e for e in _jsonl(self.path / "events.jsonl") if "seq" in e]
            if ev:
                return ev
        return list(self.oplog.events) if self.oplog else []

    def gaps(self) -> dict[str, list]:
        if self.kind == "episode":
            return self.meta.get("gaps") or {}
        return {n: _jsonl(self.path / f"{n}.gaps.jsonl") for n in self.raw if (self.path / f"{n}.gaps.jsonl").exists()}

    def summary(self) -> dict:
        t0, t1 = self.t_range
        streams = {}
        for name, s in self.streams.items():
            a, b = s.t_range
            info = {"kind": getattr(s, "kind", "rows"), "records": int(s.n_records), "t0": a, "t1": b,
                    "dropped": int(getattr(s, "dropped", 0))}
            if info["kind"] == "frames":
                idx: StreamIndex = s
                frames = idx.a["kind"] == 2
                info["frames"] = int(frames.sum())
                info["jpeg"] = bool(np.any(idx.a["jpeg"][frames])) if frames.any() else False
                w = idx.a["width"][frames]
                h = idx.a["height"][frames]
                info["size"] = [int(w.max()) if len(w) else 0, int(h.max()) if len(h) else 0]
                if frames.sum() > 1:
                    tf = idx.a["t0"][frames]
                    info["fps"] = float((len(tf) - 1) / max(1e-9, tf[-1] - tf[0]))
                info["bytes"] = int(idx.a["plen"][frames].sum())
            elif info["kind"] != "scene3d":
                info["fields"] = [[f, u] for f, u in s.fields()]
            streams[name] = info
        return {"path": str(self.path), "kind": self.kind, "meta": self.meta, "t0": t0, "t1": t1,
                "streams": streams, "episodes": self.episodes(), "cut_episodes": self.cut_episodes()
                if self.kind == "session" else [], "events": self.events(), "gaps": self.gaps(),
                "anchors": self.oplog.anchors() if self.oplog else [],
                "scene3d": self.scene.summary() if self.scene else None}


class Library:
    def __init__(self, path: Path, cache: Optional[Path] = None):
        self.path = Path(path).expanduser().resolve()
        self.cache = cache
        self._open: dict[str, Recording] = {}
        self._lock = threading.Lock()

    def _candidates(self) -> list[Path]:
        p = self.path
        if kind_of(p):
            out = [p]
            if kind_of(p) == "session" and (p / "episodes").is_dir():
                out += [e for e in sorted((p / "episodes").iterdir()) if kind_of(e) == "episode"]
            return out
        out = []
        for c in sorted(p.iterdir()) if p.is_dir() else []:
            if not c.is_dir():
                continue
            k = kind_of(c)
            if k:
                out.append(c)
                if k == "session" and (c / "episodes").is_dir():
                    out += [e for e in sorted((c / "episodes").iterdir()) if kind_of(e) == "episode"]
        return out

    def key(self, p: Path) -> str:
        try:
            rel = p.resolve().relative_to(self.path)
            return str(rel) if str(rel) != "." else "."
        except ValueError:
            return str(p)

    def items(self) -> list[dict]:
        out = []
        for c in self._candidates():
            k = kind_of(c)
            m = _read_json(c / ("episode.json" if k == "episode" else "session.json"))
            item = {"key": self.key(c), "kind": k, "name": c.name}
            if k == "session":
                eps = read_episodes(c / "episodes.jsonl")
                item.update(task=m.get("task"), started_wall=m.get("started_wall"), robot=m.get("robot"),
                            episodes=[{kk: e.get(kk) for kk in ("id", "task", "t_start", "t_stop", "outcome", "reason")}
                                      for e in eps])
            else:
                item.update(task=m.get("task"), outcome=m.get("outcome"), reason=m.get("reason"),
                            t_start=m.get("t_start"), t_stop=m.get("t_stop"), id=m.get("id"))
            out.append(item)
        return out

    def open(self, key: str) -> Recording:
        with self._lock:
            r = self._open.get(key)
            if r is None:
                p = self.path if key == "." else (self.path / key)
                p = p.resolve()
                if self.path not in (p, *p.parents):
                    raise KeyError(key)
                r = Recording(p, self.cache)
                self._open[key] = r
            return r
