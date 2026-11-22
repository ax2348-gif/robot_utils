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
from fastapi.responses import PlainTextResponse, StreamingResponse

from robot_utils.nodes.common.protocol import pack_record, rows_record, schema_record

ROW_NODES = ("arm", "swerve", "wbc", "estimator")
CAM_NODES = ("camera", "wrist_camera")


class FakeRobot:
    def __init__(self, rows_hz=100.0, cam_fps=20.0, headset_hz=50.0, frame_bytes=2000, prior_events=3):
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

        @app.get("/nodes/{node}/stream")
        def stream(node: str, streams: str = "color", mode: str = "jpeg"):
            if node in ROW_NODES:
                return self._stream(node, self._rows(node))
            if node in CAM_NODES:
                return self._stream(node, self._frames(node, streams))
            raise HTTPException(404, f"no node {node}")

        return app
