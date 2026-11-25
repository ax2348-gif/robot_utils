"""A collection session: set the task, (optionally) start teleop, record every stream
continuously, keep the episode index from the operator stream's events, stop cleanly.

    from robot_utils.collect.session import Session
    s = Session("http://botcore", "data", task="pick the cup", start_teleop=True)
    asyncio.run(s.run())            # until SIGINT / SIGTERM (or s.request_stop())
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, Sequence

import httpx

from .episodes import EpisodeIndex
from .records import event_line
from .scenes import SceneSaver
from .streams import DEFAULT_STREAMS, StreamRecorder, StreamSpec

log = logging.getLogger("collect")

TELEOP = "quest_teleop"
DESCRIPTION = "description"


def parse_event_lines(r: httpx.Response) -> list[dict]:
    """``GET /eventlog`` body: JSON lines (the contract); a JSON array is accepted too."""
    text = r.text.strip()
    if not text:
        return []
    if text.startswith("["):
        return list(json.loads(text))
    return [json.loads(line) for line in text.splitlines() if line.strip()]


class Session:
    def __init__(self, robot: str, out: str | Path, task: str, streams: Sequence[str] = DEFAULT_STREAMS, *,
                 start_teleop: bool = False, stop_teleop: bool = False, rotate_bytes: int = 512 << 20,
                 read_timeout: float = 10.0, status_every: float = 5.0, settle: float = 1.0,
                 session_name: Optional[str] = None, status_stream=None, event_page: int = 1000,
                 teleop: str = TELEOP, batch_ms: float = 0.0, set_task: bool = True,
                 baseline_seq: Optional[int] = None, transport: Optional[httpx.AsyncBaseTransport] = None,
                 status_json: bool = False):
        """``set_task``: POST the task to the node first (``collect run``); False when the caller is the node itself
        (the task is already its own). ``baseline_seq``: index the events after this seq (default: the Pi log's
        newest at the start). ``transport``: the HTTP transport (the quest_teleop node's own recorder reaches the
        nodes over their unix sockets: ``robot_utils.collect.local.NodeSocketTransport``). ``status_json``: each
        status line is ``stats()`` as one JSON line (for a supervising process) instead of the text line."""
        self.robot = robot.rstrip("/")
        self.teleop = teleop                      # the quest_teleop node instance (its /task, /start, oplog)
        self.oplog_spec = f"{teleop}/oplog/stream"
        self.task = task
        self.specs = [StreamSpec.parse(s) for s in streams]
        if batch_ms > 0:                          # recording only: rows in batches (same rows, same timestamps)
            for sp in self.specs:
                if sp.is_rows_stream() and "batch_ms" not in sp.params:
                    sp.params["batch_ms"] = f"{batch_ms:g}"
        names = [s.name for s in self.specs]
        if len(set(names)) != len(names):
            raise ValueError(f"two streams map to the same file name: {names}")
        self.set_task = set_task
        self.status_json = status_json
        self.baseline_seq = baseline_seq
        self.transport = transport
        self.scenes: list[SceneSaver] = []
        self.start_teleop = start_teleop
        self.stop_teleop = stop_teleop
        self.rotate_bytes = rotate_bytes
        self.read_timeout = read_timeout
        self.status_every = status_every
        self.settle = settle
        self.event_page = event_page
        self.status_stream = status_stream if status_stream is not None else sys.stderr
        name = session_name or datetime.now().strftime("%Y%m%d-%H%M%S")
        self.dir = Path(out) / name
        self.dir.mkdir(parents=True, exist_ok=False)
        self.index: Optional[EpisodeIndex] = None
        self.recorders: list[StreamRecorder] = []
        self._stop: Optional[asyncio.Event] = None
        self._reconcile: Optional[asyncio.Event] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._t0 = time.monotonic()
        self.meta: dict = {}

    # -- control -------------------------------------------------------------------------------

    def request_stop(self):
        """Thread-safe: end the session (what Ctrl-C does)."""
        if self._loop is not None and self._stop is not None:
            self._loop.call_soon_threadsafe(self._stop.set)

    def _teleop_url(self, path: str) -> str:
        return f"{self.robot}/nodes/{self.teleop}{path}"

    async def _post(self, client: httpx.AsyncClient, path: str, body=None):
        r = await client.post(self._teleop_url(path), json=body, timeout=10.0)
        if r.status_code >= 400:
            try:
                detail = r.json().get("detail", r.text)
            except ValueError:
                detail = r.text
            raise RuntimeError(f"POST {path}: {r.status_code} {detail}")
        return r.json() if r.content else None

    async def _save_description(self, client: httpx.AsyncClient) -> None:
        """The robot as it is now, saved unconditionally at the start (a recording without it can't be replayed
        or used with the right mounts): ``description.json`` (GET description/description: frames, cameras with
        their active calibrations, conventions; what the browser reads), ``model.json`` (GET description/model:
        model_version, components, joints) and ``model.urdf`` (the assembled URDF). No description node, no
        recording."""
        base = f"{self.robot}/nodes/{DESCRIPTION}"
        last = None
        for attempt in range(3):
            try:
                got = {}
                for path, fname in (("/description", "description.json"), ("/model", "model.json"),
                                    ("/model/urdf", "model.urdf")):
                    r = await client.get(base + path, timeout=10.0)
                    r.raise_for_status()
                    got[fname] = r.content
                break
            except (httpx.HTTPError, OSError) as e:
                last = e
                await asyncio.sleep(1.0)
        else:
            raise RuntimeError(f"cannot record without the robot description ({base}): {last}")
        for fname, data in got.items():
            (self.dir / fname).write_bytes(data)
        model = json.loads(got["model.json"])
        self.meta["description"] = {"node": base, "model_version": model.get("model_version"),
                                    "options": model.get("options"),
                                    "files": sorted(got)}

    async def _baseline_seq(self, client: httpx.AsyncClient) -> Optional[int]:
        """The newest seq in the Pi event log now (events after it are this session's)."""
        try:
            r = await client.get(self._teleop_url("/eventlog/files"), timeout=5.0)
            r.raise_for_status()
            seqs = [f.get("last_seq") for f in r.json() if f.get("last_seq") is not None]
            return max(seqs) if seqs else -1
        except Exception as e:     # noqa: BLE001
            log.warning("event log baseline unavailable (%s): the first streamed event starts the index", e)
            return None

    async def _fetch_events(self, client: httpx.AsyncClient, since: int) -> list[dict]:
        """Every event after ``since``, paged: ``limit`` at a time from the last seq until a page comes
        back short (the contract)."""
        out: list[dict] = []
        while True:
            r = await client.get(self._teleop_url("/eventlog"), params={"since_seq": since, "limit": self.event_page},
                                 timeout=10.0)
            r.raise_for_status()
            page = parse_event_lines(r)
            out += page
            if len(page) < self.event_page:
                return out
            since = max(int(e["seq"]) for e in page)

    async def _open_episode(self, client: httpx.AsyncClient) -> Optional[dict]:
        """The node's open episode now (``/state`` ``episode``: {id, task, t_start, start_seq}), or None."""
        try:
            r = await client.get(self._teleop_url("/state"), timeout=5.0)
            r.raise_for_status()
            return r.json().get("episode")
        except Exception as e:     # noqa: BLE001
            log.warning("teleop /state unavailable (%s): an episode open now would not be indexed", e)
            return None

    async def reconcile(self, client: httpx.AsyncClient) -> bool:
        """Fetch the events after the newest applied seq from the Pi log and apply them; skip the
        seqs the log does not have. True on success."""
        idx = self.index
        if idx.applied_seq is None:
            return True      # no baseline and no event yet: nothing to reconcile against
        since = idx.applied_seq
        try:
            events = await self._fetch_events(client, since)
        except Exception as e:     # noqa: BLE001
            log.warning("event log fetch failed (%s); will retry", e)
            return False
        top = since
        for ev in events:
            if int(ev["seq"]) > since:
                idx.add(ev, "eventlog")
                top = max(top, int(ev["seq"]))
        idx.skip_missing(up_to=top)
        if idx.has_hole():
            self._reconcile.set()
        return True

    async def _reconcile_loop(self, client: httpx.AsyncClient):
        delay = 0.5
        while True:
            await self._reconcile.wait()
            self._reconcile.clear()
            if await self.reconcile(client):
                delay = 0.5
            else:
                await asyncio.sleep(delay)
                delay = min(delay * 2, 5.0)
                self._reconcile.set()

    def _on_oplog_record(self, h: dict):
        if h.get("type") == "event":
            if self.index.add(event_line(h), "stream"):
                self._reconcile.set()

    def _on_oplog_connect(self):
        self._reconcile.set()      # whatever the dropped connection cost: from the Pi log

    # -- status --------------------------------------------------------------------------------

    def status_line(self) -> str:
        el = time.monotonic() - self._t0
        h, rem = divmod(int(el), 3600)
        m, s = divmod(rem, 60)
        parts = [f"[collect {h:02d}:{m:02d}:{s:02d}]"]
        idx = self.index
        oplog = self._oplog()
        if idx is not None and idx.open is not None:
            now = oplog.stats.last_pi_t if oplog is not None else None
            dur = f" {now - idx.open['t_start']:.1f} s" if now is not None and idx.open["t_start"] is not None else ""
            parts.append(f"in episode {idx.open['id']}{dur}")
        else:
            parts.append("recording")
        if idx is not None:
            kept = sum(e["outcome"] == "kept" for e in idx.finished)
            parts.append(f"{kept} kept {len(idx.finished) - kept} aborted")
        rates = []
        for r in self.recorders:
            st = r.stats
            mark = "" if st.connected else "!"
            gaps = f" gaps {st.gaps}" if st.gaps else ""
            drop = f" dropped {st.dropped}" if st.dropped else ""
            rates.append(f"{mark}{r.spec.name} {st.rate():.0f}/s{gaps}{drop}")
        parts.append(", ".join(rates))
        if self.scenes:
            parts.append(f"{sum(len(s.saved) for s in self.scenes)} scenes"
                         + (f" ({sum(len(s.missed) for s in self.scenes)} missed)" if any(s.missed for s in self.scenes)
                            else ""))
        parts.append(f"{sum(r.stats.bytes for r in self.recorders) / 1e6:.0f} MB")
        return " | ".join(parts)

    def stats(self) -> dict:
        """What the session has recorded so far (the quest_teleop node's /state ``recording``)."""
        idx = self.index
        return {"dir": str(self.dir), "streams": {r.spec.name: {"connected": r.stats.connected,
                                                                "rate": round(r.stats.rate(), 1),
                                                                "records": r.stats.records, "gaps": r.stats.gaps,
                                                                "dropped": r.stats.dropped}
                                                  for r in self.recorders},
                "bytes": sum(r.stats.bytes for r in self.recorders),
                "episodes": None if idx is None else {
                    "kept": sum(e["outcome"] == "kept" for e in idx.finished),
                    "aborted": sum(e["outcome"] != "kept" for e in idx.finished),
                    "open": idx.open["id"] if idx.open else None},
                "scenes": [s.info() for s in self.scenes], "status": self.status_line()}

    def _print_status(self):
        line = json.dumps({"status": self.stats()}) if self.status_json else self.status_line()
        print(line, file=self.status_stream, flush=True)

    async def _status_loop(self):
        while True:
            await asyncio.sleep(self.status_every)
            self._print_status()

    def _oplog(self) -> Optional[StreamRecorder]:
        for r in self.recorders:
            if r.spec.spec == self.oplog_spec or r.spec.path == self.oplog_spec:
                return r
        return None

    # -- meta ----------------------------------------------------------------------------------

    def _write_meta(self):
        tmp = self.dir / "session.json.tmp"
        with open(tmp, "w") as f:
            json.dump(self.meta, f, indent=1)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.dir / "session.json")

    # -- run -----------------------------------------------------------------------------------

    async def run(self, install_signals: bool = True) -> Path:
        self._loop = asyncio.get_running_loop()
        self._stop = asyncio.Event()
        self._reconcile = asyncio.Event()
        if install_signals:
            for sig in (signal.SIGINT, signal.SIGTERM):
                try:
                    self._loop.add_signal_handler(sig, self._on_signal, sig)
                except (NotImplementedError, RuntimeError):
                    pass
        self.meta = {"robot": self.robot, "task": self.task, "started_wall": time.time(),
                     "started_mono": time.monotonic(), "stopped_wall": None, "stop_reason": None,
                     "rotate_bytes": self.rotate_bytes, "teleop_started": False,
                     "streams": [s.to_json(self.robot) for s in self.specs]}
        self._write_meta()
        limits = httpx.Limits(max_connections=len(self.specs) + 8, max_keepalive_connections=8)
        async with httpx.AsyncClient(limits=limits, transport=self.transport) as client:
            await self._save_description(client)
            self._write_meta()
            if self.set_task:
                await self._post(client, "/task", {"text": self.task})
            baseline = self.baseline_seq if self.baseline_seq is not None else await self._baseline_seq(client)
            open_ep = await self._open_episode(client)
            if open_ep is not None and open_ep.get("start_seq") is not None:
                # index it from its start: the events since then come from the Pi log
                start = int(open_ep["start_seq"]) - 1
                baseline = start if baseline is None else min(baseline, start)
            self.meta["eventlog_baseline_seq"] = baseline
            self.meta["open_episode_at_start"] = open_ep
            self.index = EpisodeIndex(self.dir / "episodes.jsonl", self.dir / "events.jsonl", baseline,
                                      on_episode=self._on_episode)
            if open_ep is not None:
                await self.reconcile(client)
                if self.index.open is None and not any(e["id"] == open_ep["id"] for e in self.index.finished):
                    self.index.open_from_state(open_ep)      # the log no longer has its start
            for spec in self.specs:
                is_oplog = spec.path == self.oplog_spec
                on_record = self._on_oplog_record if is_oplog else None
                if spec.framing == "rsf1":           # the rigid scene: saved once per scene_id
                    saver = SceneSaver(spec, self.robot, self.dir)
                    saver.attach(client)
                    self.scenes.append(saver)
                    on_record = saver.on_record
                self.recorders.append(StreamRecorder(
                    spec, self.robot, self.dir, rotate_bytes=self.rotate_bytes, read_timeout=self.read_timeout,
                    on_record=on_record, on_connect=self._on_oplog_connect if is_oplog else None))
            tasks = [asyncio.create_task(r.run(client), name=r.spec.name) for r in self.recorders]
            helpers = [asyncio.create_task(self._reconcile_loop(client)),
                       asyncio.create_task(self._status_loop())]
            print(f"collect: session {self.dir}", file=self.status_stream, flush=True)
            try:
                if self.start_teleop:
                    await self._post(client, "/start")
                    self.meta["teleop_started"] = True
                    self._write_meta()
                await self._stop.wait()
                self.meta["stop_reason"] = self.meta.get("stop_reason") or "requested"
            except Exception as e:
                self.meta["stop_reason"] = f"error: {e}"
                raise
            finally:
                await self._shutdown(client, tasks, helpers)
        return self.dir

    def _on_signal(self, sig):
        self.meta["stop_reason"] = signal.Signals(sig).name
        self._stop.set()
        self._loop.remove_signal_handler(sig)     # a second one is the default: kill

    def _on_episode(self, ep: dict):
        print(f"collect: episode {ep['id']} {ep['outcome']}" + (f" ({ep['reason']})" if ep["reason"] else ""),
              file=self.status_stream, flush=True)

    async def _shutdown(self, client, tasks, helpers):
        if self.stop_teleop:
            try:
                await self._post(client, "/stop")
            except Exception as e:     # noqa: BLE001
                log.warning("teleop stop failed: %s", e)
        if self.settle > 0:
            await asyncio.sleep(self.settle)       # the last events and frames arrive
        for t in helpers:
            t.cancel()
        try:
            await asyncio.wait_for(self.reconcile(client), 5.0)
        except (asyncio.TimeoutError, Exception) as e:     # noqa: BLE001
            log.warning("final event log reconcile failed: %s", e)
        self.index.skip_missing(why="unreconciled at session end")
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, *helpers, return_exceptions=True)
        for s in self.scenes:
            await s.drain()
        for r in self.recorders:
            r.close()
        oplog = self._oplog()
        t_end = oplog.stats.last_pi_t if oplog is not None and oplog.stats.last_pi_t is not None else None
        if t_end is None:
            ts = [r.stats.last_pi_t for r in self.recorders if r.stats.last_pi_t is not None]
            t_end = max(ts) if ts else self.index.last_event_t
        self.index.close_open("session ended", t_end, time.time())
        self.meta.update({"stopped_wall": time.time(), "stopped_mono": time.monotonic(),
                          "episodes": len(self.index.finished), "skipped_event_seqs": self.index.skipped,
                          "last_event_seq": self.index.applied_seq,
                          "files": {r.spec.name: r.files for r in self.recorders},
                          "scenes": [s.info() for s in self.scenes],
                          "stats": {r.spec.name: {"records": r.stats.records, "rows": r.stats.rows,
                                                  "bytes": r.stats.bytes, "dropped": r.stats.dropped,
                                                  "gaps": r.stats.gaps} for r in self.recorders}})
        self._write_meta()
        self._print_status()
        print(f"collect: stopped ({self.meta['stop_reason']}); {len(self.index.finished)} episodes in "
              f"{self.dir / 'episodes.jsonl'}", file=self.status_stream, flush=True)
