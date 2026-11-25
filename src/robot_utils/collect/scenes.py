"""The rigid scenes a recorded pose stream refers to (docs/scene3d.md), saved once per ``scene_id``.

A pose frame (``<sim>/scene3d/stream``) carries only the dynamic bodies' poses and the id of the scene they index;
the scene itself (bodies, geoms, meshes) is ``GET <sim>/scene3d`` + ``GET <sim>/scene3d/blob``. ``SceneSaver``
watches the recorded frames and, at each ``scene_id`` it has not saved yet (the first frame, an object added or
removed, a ghost declared, a fixed object moved), fetches the scene with the stream's own ``cell`` /
``crease_deg`` and writes::

    <session>/scene3d/<scene_id>.json    the index, as served
    <session>/scene3d/<scene_id>.blob    the mesh blob (?scene_id=: never another scene's)
    <session>/scene3d/scenes.jsonl       one line per scene: scene_id, t_rx (the first frame that named it),
                                         wall, view (GET /scene3d/view: the headset's style and geometry role
                                         then), files; or {scene_id, missed: why} when it could not be had

A scene the sim has already moved past when the fetch runs is gone (the sim serves only its current one): that
id is noted as missed, and the scene that was served instead is saved under its own id.
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import time
from pathlib import Path
from typing import Optional

import httpx

from .streams import StreamSpec

log = logging.getLogger("collect")

SCENE_DIR = "scene3d"
QUERY_KEYS = ("cell", "crease_deg")         # the stream's parameters that shape the scene


def _write(path: Path, data: bytes) -> None:
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "wb") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


class SceneSaver:
    def __init__(self, spec: StreamSpec, robot: str, session: Path, *, attempts: int = 3):
        node_path = spec.path[:-len("/stream")] if spec.path.endswith("/stream") else spec.path
        self.base = f"{robot.rstrip('/')}/nodes/{node_path}"          # .../nodes/sim/scene3d
        self.params = {k: v for k, v in spec.params.items() if k in QUERY_KEYS}
        self.dir = Path(session) / SCENE_DIR
        self.attempts = attempts
        self.saved: dict[str, dict] = {}        # scene_id -> its scenes.jsonl line
        self.missed: dict[str, str] = {}
        self.last_error: Optional[str] = None
        self._pending: dict[str, asyncio.Task] = {}
        self._client: Optional[httpx.AsyncClient] = None

    def attach(self, client: httpx.AsyncClient) -> None:
        self._client = client

    # -- the recorder's hook (its event loop) ----------------------------------------------------------------

    def on_record(self, h: dict) -> None:
        sid = h.get("scene_id") if h.get("type") == "scene3d" else None
        if not sid or sid in self.saved or sid in self._pending or self._client is None:
            return
        self._pending[sid] = asyncio.get_running_loop().create_task(self._fetch(sid, h.get("t_rx")))

    # -- fetching ---------------------------------------------------------------------------------------------

    async def _fetch(self, sid: str, t_rx: Optional[float]) -> None:
        try:
            delay = 0.3
            why = "not fetched"
            for _ in range(self.attempts):
                try:
                    got = await self._get_scene(sid)
                except (httpx.HTTPError, OSError, ValueError) as e:
                    why = f"{type(e).__name__}: {e}"
                    self.last_error = why
                    await asyncio.sleep(delay)
                    delay *= 2
                    continue
                if got is None:
                    why = "the sim had moved on to another scene"
                    break
                return
            self.missed[sid] = why
            self._line({"scene_id": sid, "t_rx": t_rx, "wall": time.time(), "missed": why})
            log.warning("scene %s not saved: %s", sid, why)
        finally:
            self._pending.pop(sid, None)

    async def _get_scene(self, sid: str) -> Optional[str]:
        """Fetch and save the current scene; the id saved, or None when it is not ``sid`` (saved anyway)."""
        c = self._client
        r = await c.get(self.base, params=self.params, timeout=30.0)
        r.raise_for_status()
        index_bytes = r.content
        index = json.loads(index_bytes)
        cur = index.get("scene_id")
        if not cur:
            raise ValueError("the scene index has no scene_id")
        if cur not in self.saved:
            b = await c.get(self.base + "/blob", params={**self.params, "scene_id": cur}, timeout=60.0)
            if b.status_code == 409:      # moved on between the two requests
                return None
            b.raise_for_status()
            view = None
            try:
                v = await c.get(self.base + "/view", timeout=5.0)
                if v.status_code < 400:
                    view = v.json()
            except (httpx.HTTPError, ValueError):
                pass
            self.dir.mkdir(parents=True, exist_ok=True)
            _write(self.dir / f"{cur}.json", index_bytes)
            _write(self.dir / f"{cur}.blob", b.content)
            line = {"scene_id": cur, "t_rx": time.monotonic(), "wall": time.time(), "view": view,
                    "files": [f"{cur}.json", f"{cur}.blob"], "bytes": len(index_bytes) + len(b.content),
                    "requested": sid}
            self.saved[cur] = line
            self._line(line)
            log.info("scene %s saved (%d bodies, %.1f MB)", cur, len(index.get("bodies", [])),
                     line["bytes"] / 1e6)
        return cur if cur == sid else None

    def _line(self, line: dict) -> None:
        self.dir.mkdir(parents=True, exist_ok=True)
        with open(self.dir / "scenes.jsonl", "a") as f:
            f.write(json.dumps(line) + "\n")
            f.flush()
            os.fsync(f.fileno())

    async def drain(self, timeout: float = 10.0) -> None:
        """Wait for the fetches in flight (session end)."""
        tasks = list(self._pending.values())
        if tasks:
            await asyncio.wait(tasks, timeout=timeout)

    def info(self) -> dict:
        return {"saved": sorted(self.saved), "missed": dict(self.missed), "pending": sorted(self._pending),
                "error": self.last_error}


def read_scenes(session: Path) -> list[dict]:
    """``scene3d/scenes.jsonl`` of a session or an episode directory (empty if none)."""
    p = Path(session) / SCENE_DIR / "scenes.jsonl"
    if not p.exists():
        return []
    return [json.loads(x) for x in p.read_text().splitlines() if x.strip()]
