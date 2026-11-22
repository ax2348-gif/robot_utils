"""Cut a session into episodes by Pi time.

For every episode of ``episodes.jsonl`` and every recorded stream: the records whose Pi time
(``records.py``: per record type) falls in ``[t_start - pre_roll, t_stop + post_roll]`` go to
``<out>/<id>/<stream>.bin`` (a rows record that straddles an edge is cut to its rows inside; each
file starts with the schema records its rows need, so it decodes on its own; pings are left out),
and ``<out>/<id>/episode.json`` describes the episode (task, outcome, times, per-stream counts,
dropped, the gaps that intersect the window, the time field used).
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from .episodes import read_episodes
from .records import header_time, iter_raw, rows_times, slice_rows_record, time_field_name
from .streams import rotated_files


def read_gaps(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def gap_intersects(g: dict, lo: float, hi: float) -> bool:
    """A gap spans Pi time (pi_t_last, pi_t_first); an unknown end is open."""
    a = g.get("pi_t_last")
    b = (g.get("reconnect") or {}).get("pi_t_first")
    a = -np.inf if a is None else a
    b = np.inf if b is None else b
    return a < hi and b > lo


class _EpisodeOut:
    def __init__(self, ep: dict, lo: float, hi: float, d: Path):
        self.ep, self.lo, self.hi, self.dir = ep, lo, hi, d
        self.file = None
        self.schema_written: dict = {}
        self.records = 0
        self.rows = 0
        self.dropped = 0
        self.first_t: Optional[float] = None
        self.last_t: Optional[float] = None
        self.time_fields: dict[str, str] = {}     # record type -> the field that dated it
        self.types: dict[str, int] = {}

    def write(self, path: Path, raw: bytes):
        if self.file is None:
            self.dir.mkdir(parents=True, exist_ok=True)
            self.file = open(path, "wb")
        self.file.write(raw)

    def seen(self, t0: float, t1: float):
        self.first_t = t0 if self.first_t is None else min(self.first_t, t0)
        self.last_t = t1 if self.last_t is None else max(self.last_t, t1)

    def close(self):
        if self.file is not None:
            self.file.close()


def cut_session(session: str | Path, out: Optional[str | Path] = None, pre_roll: float = 0.0,
                post_roll: float = 0.0, only: Optional[Sequence[str]] = None) -> list[dict]:
    """Write the episode directories; returns the episode.json contents."""
    session = Path(session)
    out = Path(out) if out is not None else session / "episodes"
    meta = json.loads((session / "session.json").read_text())
    episodes = [e for e in read_episodes(session / "episodes.jsonl") if only is None or e["id"] in only]
    episodes = [e for e in episodes if e.get("t_start") is not None and e.get("t_stop") is not None]
    windows = [(e, e["t_start"] - pre_roll, e["t_stop"] + post_roll) for e in episodes]
    results = {e["id"]: {"id": e["id"], "task": e["task"], "outcome": e["outcome"], "reason": e["reason"],
                         "t_start": e["t_start"], "t_stop": e["t_stop"], "wall": e["wall"],
                         "pre_roll": pre_roll, "post_roll": post_roll, "window": [lo, hi],
                         "event_seqs": e.get("events", []), "reconciled": e.get("reconciled", False),
                         "session": str(session.resolve()), "robot": meta.get("robot"),
                         "streams": {}, "gaps": {}}
               for e, lo, hi in windows}
    for s in meta["streams"]:
        name = s["name"]
        outs = [_EpisodeOut(e, lo, hi, out / e["id"]) for e, lo, hi in windows]
        _cut_stream(rotated_files(session, name), outs, name)
        gaps = read_gaps(session / f"{name}.gaps.jsonl")
        for o in outs:
            o.close()
            r = results[o.ep["id"]]
            r["streams"][name] = {"spec": s["spec"], "file": f"{name}.bin" if o.file else None,
                                  "records": o.records, "rows": o.rows, "by_type": o.types,
                                  "dropped": o.dropped, "time_fields": o.time_fields,
                                  "first_t": o.first_t, "last_t": o.last_t}
            r["gaps"][name] = [g for g in gaps if gap_intersects(g, o.lo, o.hi)]
    for r in results.values():
        d = out / r["id"]
        d.mkdir(parents=True, exist_ok=True)
        tmp = d / "episode.json.tmp"
        tmp.write_text(json.dumps(r, indent=1))
        os.replace(tmp, d / "episode.json")
    return list(results.values())


def _cut_stream(files, outs: list[_EpisodeOut], name: str):
    if not outs:
        return
    schemas: dict = {}      # id -> (raw, fields)
    for raw, h, payload in iter_raw(files):
        kind = h.get("type")
        if kind == "schema":
            schemas[h.get("schema")] = (raw, h.get("fields"))
            continue
        if kind == "ping":
            continue
        fields = h.get("fields")
        schema_raw = None
        if fields is None and kind == "rows":
            sid = h.get("schema")
            if sid not in schemas:
                continue
            schema_raw, fields = schemas[sid]
        if fields:
            ts = rows_times(payload, fields, h.get("dtype", "<f8"))
            if ts is None or not len(ts):
                continue
            tf = time_field_name(h, fields)
            for o in outs:
                mask = (ts >= o.lo) & (ts <= o.hi)
                if not mask.any():
                    continue
                if schema_raw is not None and o.schema_written.get(h.get("schema")) is not schema_raw:
                    o.write(o.dir / f"{name}.bin", schema_raw)
                    o.schema_written[h.get("schema")] = schema_raw
                o.write(o.dir / f"{name}.bin", raw if mask.all() else slice_rows_record(h, payload, fields, mask))
                o.records += 1
                o.rows += int(mask.sum())
                o.dropped += int(h.get("dropped") or 0)
                o.time_fields.setdefault(kind, tf)
                o.types[kind] = o.types.get(kind, 0) + 1
                o.seen(float(ts[mask].min()), float(ts[mask].max()))
            continue
        t = header_time(h)
        if t is None:
            continue
        for o in outs:
            if o.lo <= t <= o.hi:
                o.write(o.dir / f"{name}.bin", raw)
                o.records += 1
                o.dropped += int(h.get("dropped") or 0)
                o.types[kind] = o.types.get(kind, 0) + 1
                if kind == "frame" and h.get("t_sensor") is None:
                    o.time_fields.setdefault("frame (no t_sensor)", "t_capture")
                else:
                    o.time_fields.setdefault(kind, time_field_name(h))
                o.seen(t, t)
