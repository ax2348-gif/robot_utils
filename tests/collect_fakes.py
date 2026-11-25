"""A fake robot frontend for the collect script's tests: the node <-> script contract of
docs/plans/data_collection_v1.md on the paths the script uses (``/nodes/<node>/...``).

- ``/nodes/<arm|swerve|wbc>/stream``: a schema record, then rows (``t_tick``, ``seq``, ``x``) at ``rows_hz``,
  every pending tick in one record (so a record can carry several rows); ``/nodes/estimator/stream`` the same
  with ``t`` (the estimator's time column).
- ``/nodes/<camera|wrist_camera>/stream?streams=color``: ``frame`` records (MJPG-looking payloads, ``t_sensor``,
  ``t_capture``, ``seq``, ``dropped``) at ``cam_fps``.
- ``/nodes/quest_teleop/oplog/stream``: ``headset`` (``t_rx``, ``frame``), ``sent`` (``t_send``, ``datagram``)
  and ``event`` records in order of occurrence.
- ``/nodes/quest_teleop``: ``POST /task``, ``/start``, ``/stop``; ``GET /state`` (``episode``: the open one or
  null); ``GET /eventlog?since_seq=N&limit=M`` (JSON lines, at most M, default 1000), ``GET /eventlog/files``.

- ``/nodes/sim/scene3d`` (+ ``/blob``, ``/view``, ``/stream?format=bin``): a small rigid scene (``FakeScene``:
  a robot body, a static table, optionally a cube) and its RSF1 pose frames at ``scene_hz``; ``new_scene()``
  adds the cube (a new scene_id), ``sim_reset()`` bumps reset_seq.

Test controls: ``emit`` / ``start_episode`` / ``stop_episode`` / ``abort_episode`` (events go to the log and,
unless their seq is in ``skip_on_stream``, to every open operator stream), ``drop(key)`` (every open
connection of that stream ends after half a record), ``refuse`` (keys whose new connections get 503),
``pruned_below`` (the log no longer has the seqs below it).
Pi time is this process's ``time.monotonic()``.
"""
import asyncio
import itertools
import json
import os
import threading
import time

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse, PlainTextResponse, Response, StreamingResponse

from robot_utils.nodes.common.protocol import pack_record, rows_record, schema_record
from robot_utils.scene3d.format import Frame, SceneBuilder, encode_frame

ROW_NODES = ("arm", "swerve", "wbc", "estimator")
CAM_NODES = ("camera", "wrist_camera")


class FakeScene:
    """A rigid scene: a robot body (box) moving along x, a static table, and with ``cube`` a cube (an object)."""

    def __init__(self, cube=False):
        b = SceneBuilder("fake")
        tri = b.add_mesh("wedge", [[0, 0, 0], [0.2, 0, 0], [0, 0.2, 0], [0, 0, 0.2]],
                         [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]])
        b.add_body("base", "robot", [0, 0, 0.1, 0, 0, 0, 1], True,
                   geoms=[{"type": "box", "half_extents": [0.3, 0.25, 0.1], "rgba": [0.6, 0.6, 0.7, 1],
                           "role": "both"},
                          {"type": "mesh", "mesh": tri, "pos": [0, 0, 0.1], "role": "visual"}])
        b.add_body("table", "object", [1.0, 0, 0.35, 0, 0, 0, 1], False,
                   geoms=[{"type": "box", "half_extents": [0.4, 0.6, 0.35], "rgba": [0.5, 0.4, 0.3, 1]}])
        if cube:
            b.add_body("cube", "object", [1.0, 0, 0.75, 0, 0, 0, 1], True,
                       geoms=[{"type": "box", "half_extents": [0.025] * 3, "rgba": [0.9, 0.1, 0.1, 1]}])
        self.index, self.blob = b.build()
        self.id = self.index["scene_id"]

    def frame(self, seq, t, reset_seq) -> bytes:
        n = len(self.index["dynamic"])
        poses = [[0.1 * (seq % 50) / 50, 0, 0.1, 0, 0, 0, 1]] + [[1.0, 0, 0.75, 0, 0, 0, 1]] * (n - 1)
        return encode_frame(Frame(seq, t, reset_seq, self.id, poses))


class FakeRobot:
    def __init__(self, rows_hz=100.0, cam_fps=20.0, headset_hz=50.0, frame_bytes=2000, prior_events=3,
                 scene_hz=50.0):
        self.rows_hz = rows_hz
        self.cam_fps = cam_fps
        self.headset_hz = headset_hz
        self.frame_bytes = frame_bytes
        self.lock = threading.Lock()
        self.events: list[dict] = []
        self.seq = 41                         # the log continues from earlier sessions
        self.skip_on_stream: set[int] = set()
        self.refuse: set[str] = set()
        self.gen: dict[str, int] = {}
        self.calls: list[tuple[str, object]] = []
        self.task = None
        self.teleop = False
        self.start_status = 200
        self.open_episode = None
        self.open_info = None
        self.counter = itertools.count(1)
        self.connections: dict[str, int] = {}
        self.eventlog_calls: list[tuple[int, int]] = []
        self.pruned_below = 0
        self.scene_hz = scene_hz
        self.scene = FakeScene()
        self.reset_seq = 0
        self.scene_requests: list[str] = []
        for _ in range(prior_events):
            self.emit("button", button="Y", pressed=True)
        self.app = self._build()

    # -- events --------------------------------------------------------------------------------

    def emit(self, kind, episode=None, **detail) -> dict:
        with self.lock:
            self.seq += 1
            ev = {"seq": self.seq, "t": time.monotonic(), "wall": time.time(), "kind": kind,
                  "episode": episode, "detail": detail}
            self.events.append(ev)
        return ev

    def start_episode(self) -> str:
        self.emit("button", button="A", pressed=True)
        if self.open_episode is not None:                      # A while open: stop (kept), then start
            self.emit("episode_stop", self.open_episode)
        eid = f"20261121-{next(self.counter):04d}"
        self.open_episode = eid
        ev = self.emit("episode_start", eid, task=self.task)
        self.open_info = {"id": eid, "task": self.task, "t_start": ev["t"], "start_seq": ev["seq"]}
        return eid

    def stop_episode(self) -> dict:
        self.emit("button", button="B", pressed=False)
        ev = self.emit("episode_stop", self.open_episode)
        self.open_episode = None
        return ev

    def abort_episode(self, reason) -> dict:
        ev = self.emit("episode_abort", self.open_episode, reason=reason)
        self.open_episode = None
        return ev

    def new_scene(self) -> str:
        self.scene = FakeScene(cube=True)
        return self.scene.id

    def sim_reset(self) -> int:
        self.reset_seq += 1
        return self.reset_seq

    def _scene_frames(self):
        async def body():
            seq = 0
            period = 1.0 / self.scene_hz
            t0 = time.monotonic()
            while True:
                seq += 1
                f = self.scene.frame(seq, time.monotonic() - t0, self.reset_seq)
                yield f[:20]                                 # frames arrive split across chunks too
                yield f[20:]
                await asyncio.sleep(period)
        return body

    def drop(self, key):
        """End every open connection of stream ``key`` (``arm``, ``camera``, ``oplog``...) mid-record."""
        with self.lock:
            self.gen[key] = self.gen.get(key, 0) + 1

    # -- app -----------------------------------------------------------------------------------

    def _stream(self, key, body):
        if key in self.refuse:
            raise HTTPException(503, f"{key} refused (test)")
        self.connections[key] = self.connections.get(key, 0) + 1
        gen0 = self.gen.get(key, 0)

        async def wrapped():
            async for chunk in body():
                if self.gen.get(key, 0) != gen0:
                    yield chunk[:max(1, len(chunk) // 2)]      # half a record, then the connection ends
                    return
                yield chunk
        return StreamingResponse(wrapped(), media_type="application/octet-stream")

    def _rows(self, node):
        tcol = "t" if node == "estimator" else "t_tick"

        async def body():
            yield schema_record(1, [tcol, "seq", "x"], ["s", "", "m"])
            period = 1.0 / self.rows_hz
            t_next = time.monotonic()
            seq = 0
            while True:
                now = time.monotonic()
                rows = []
                while t_next <= now:
                    rows.append((t_next, seq, seq * 0.01))
                    seq += 1
                    t_next += period
                if rows:
                    yield rows_record(rows, 1, dropped=0, t_send=now)
                await asyncio.sleep(0.005)
        return body

    def _frames(self, node, stream):
        async def body():
            seq = 0
            period = 1.0 / self.cam_fps
            while True:
                now = time.monotonic()
                payload = b"\xff\xd8" + os.urandom(self.frame_bytes - 4) + b"\xff\xd9"
                seq += 1
                yield pack_record({"type": "frame", "stream": stream, "seq": seq, "dropped": 0,
                                   "format": "MJPG", "encoding": "jpeg", "width": 64, "height": 48,
                                   "t_sensor": now - 0.03, "t_capture": now, "t_send": time.monotonic()}, payload)
                await asyncio.sleep(period)
        return body

    def _oplog(self):
        async def body():
            with self.lock:
                cursor = len(self.events)
            period = 1.0 / self.headset_hz
            t_next = time.monotonic()
            n = 0
            while True:
                out = []
                with self.lock:
                    new = self.events[cursor:]
                    cursor = len(self.events)
                now = time.monotonic()
                while t_next <= now:
                    n += 1
                    out.append((t_next, {"type": "headset", "t_rx": t_next,
                                         "frame": {"schema": "teleop.quest_state.v1", "n": n}}))
                    out.append((t_next + 1e-4, {"type": "sent", "t_send": t_next + 1e-4,
                                                "datagram": {"n": n, "targets": []}}))
                    t_next += period
                for ev in new:
                    if ev["seq"] not in self.skip_on_stream:
                        out.append((ev["t"], {"type": "event", **ev}))
                out.sort(key=lambda x: x[0])
                if out:
                    yield b"".join(pack_record(h) for _, h in out)
                await asyncio.sleep(0.005)
        return body

    def _build(self):
        app = FastAPI()

        @app.get("/nodes/quest_teleop/oplog/stream")
        def oplog():
            return self._stream("oplog", self._oplog())

        @app.get("/nodes/description/description")
        def description():
            if "description" in self.refuse:
                return JSONResponse({"detail": "down"}, status_code=503)
            return {"frames": {"base": {}}, "cameras": {"camera": {"serial": "FAKE"}}, "conventions": {}}

        @app.get("/nodes/description/model")
        def model():
            return {"model_version": "fake0001", "options": {"mounts": "calibrated"}, "joints": []}

        @app.get("/nodes/description/model/urdf")
        def urdf():
            return Response('<robot name="fake"/>', media_type="application/xml")

        @app.post("/nodes/quest_teleop/task")
        async def task(request: Request):
            body = await request.json()
            self.calls.append(("task", body))
            self.task = body["text"]
            return {"task": self.task}

        @app.post("/nodes/quest_teleop/start")
        def start():
            self.calls.append(("start", None))
            if self.start_status != 200:
                raise HTTPException(self.start_status, "WBC not ready (test)")
            self.teleop = True
            self.emit("teleop_start")
            return {"teleop": True}

        @app.post("/nodes/quest_teleop/stop")
        def stop():
            self.calls.append(("stop", None))
            if self.open_episode is not None:
                self.abort_episode("teleop stopped")
            self.teleop = False
            self.emit("teleop_stop", reason="request")
            return {"teleop": False}

        @app.get("/nodes/quest_teleop/state")
        def state():
            return {"teleop": self.teleop, "episode": self.open_info if self.open_episode is not None else None}

        @app.get("/nodes/quest_teleop/eventlog")
        def eventlog(since_seq: int = -1, limit: int = 1000):
            self.eventlog_calls.append((since_seq, limit))
            with self.lock:
                lines = [json.dumps(e) for e in self.events
                         if e["seq"] > since_seq and e["seq"] >= self.pruned_below][:limit]
            return PlainTextResponse("".join(line + "\n" for line in lines), media_type="application/x-ndjson")

        @app.get("/nodes/quest_teleop/eventlog/files")
        def eventlog_files():
            with self.lock:
                evs = list(self.events)
            if not evs:
                return []
            return [{"name": "events-20261121.jsonl", "bytes": sum(len(json.dumps(e)) + 1 for e in evs),
                     "first_seq": evs[0]["seq"], "last_seq": evs[-1]["seq"]}]

        @app.get("/nodes/sim/scene3d")
        def scene3d(cell: float = 0.002, crease_deg: float = 30.0):
            self.scene_requests.append("index")
            return Response(json.dumps(self.scene.index), media_type="application/json")

        @app.get("/nodes/sim/scene3d/blob")
        def scene3d_blob(scene_id: str = None):
            self.scene_requests.append("blob")
            if scene_id is not None and scene_id != self.scene.id:
                raise HTTPException(409, "scene moved on")
            return Response(self.scene.blob, media_type="application/octet-stream")

        @app.get("/nodes/sim/scene3d/view")
        def scene3d_view():
            return {"style": "flat", "role": "auto"}

        @app.get("/nodes/sim/scene3d/stream")
        def scene3d_stream(format: str = "jsonl", hz: float = 30.0):
            assert format == "bin"
            return self._stream("scene3d", self._scene_frames())

        @app.get("/nodes/{node}/stream")
        def stream(node: str, streams: str = "color", mode: str = "jpeg"):
            if node in ROW_NODES:
                return self._stream(node, self._rows(node))
            if node in CAM_NODES:
                return self._stream(node, self._frames(node, streams))
            raise HTTPException(404, f"no node {node}")

        return app
