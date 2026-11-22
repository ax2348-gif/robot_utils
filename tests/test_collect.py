"""The collect script (robot_utils.collect) against a fake robot (tests/collect_fakes.py): continuous recording,
the episode index from events, reconciliation from the Pi event log, cutting by Pi time, rotation, Ctrl-C."""
import asyncio
import io
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from collect_fakes import FakeRobot
from helpers import serve, stop, waiter
from robot_utils.collect.cut import cut_session
from robot_utils.collect.episodes import EpisodeIndex, read_episodes
from robot_utils.collect.records import event_line, header_time
from robot_utils.collect.session import Session
from robot_utils.collect.streams import StreamSpec, rotated_files
from robot_utils.nodes.common.protocol import iter_records, iter_rows

wait = waiter(10.0, 0.02)
STREAMS = ["arm/stream", "estimator/stream", "camera/color", "quest_teleop/oplog/stream"]


@pytest.fixture
def robot():
    fake = FakeRobot()
    url, srv, th = serve(fake.app)
    fake.url = url
    yield fake
    stop((srv, th))


class Running:
    """A Session on a thread (its own event loop), stopped with ``stop()``."""

    def __init__(self, robot, tmp_path, streams=STREAMS, **kw):
        kw.setdefault("settle", 0.3)
        kw.setdefault("status_every", 0.5)
        self.out = io.StringIO()
        self.session = Session(robot.url, tmp_path, "pick the cup", streams, session_name="s1",
                               status_stream=self.out, **kw)
        self.dir = self.session.dir
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        wait(lambda: self.session.recorders and all(r.stats.records > 2 for r in self.session.recorders))

    def _run(self):
        try:
            asyncio.run(self.session.run(install_signals=False))
        except Exception as e:     # noqa: BLE001
            self.error = e

    def stop(self):
        self.session.request_stop()
        self.thread.join(20)
        assert not self.thread.is_alive()
        assert self.error is None, self.error
        return self.dir


def records(session_dir, name):
    return list(iter_records(_cat(rotated_files(session_dir, name))))


def _cat(paths):
    return io.BytesIO(b"".join(Path(p).read_bytes() for p in paths))


def event_seqs(session_dir):
    lines = [json.loads(x) for x in (session_dir / "events.jsonl").read_text().splitlines()]
    return [x["seq"] for x in lines if "seq" in x], lines


def episode_seqs(robot, eid):
    evs = [e for e in robot.events if e["episode"] == eid]
    return evs[0]["seq"], evs[-1]["seq"]


# -- units ---------------------------------------------------------------------------------------

def test_stream_specs():
    s = StreamSpec.parse("camera/color")
    assert (s.name, s.path, s.params) == ("camera.color", "camera/stream", {"streams": "color", "mode": "jpeg"})
    s = StreamSpec.parse("quest_teleop/oplog/stream")
    assert (s.name, s.path, s.params) == ("quest_teleop.oplog", "quest_teleop/oplog/stream", {})
    assert StreamSpec.parse("arm/stream").url("http://bot/") == "http://bot/nodes/arm/stream"
    s = StreamSpec.parse("camera/stream?streams=depth&mode=raw")
    assert s.path == "camera/stream" and s.params == {"streams": "depth", "mode": "raw"}
    with pytest.raises(ValueError):
        StreamSpec.parse("arm")


def test_index_orders_and_skips(tmp_path):
    idx = EpisodeIndex(tmp_path / "e.jsonl", tmp_path / "ev.jsonl", applied_seq=9)

    def ev(seq, kind, ep=None, **d):
        return {"seq": seq, "t": float(seq), "wall": 1e9 + seq, "kind": kind, "episode": ep, "detail": d}
    assert not idx.add(ev(10, "episode_start", "E1", task="t"), "stream")
    assert idx.add(ev(12, "episode_stop", "E1"), "stream")          # 11 missing: hold, ask for a reconcile
    assert idx.open is not None and not idx.finished
    assert not idx.add(ev(11, "hold", "E1", reason="x"), "eventlog")   # the log fills it: applied in order
    assert [e["outcome"] for e in idx.finished] == ["kept"] and idx.finished[0]["events"] == [10, 11, 12]
    assert idx.finished[0]["reconciled"]
    idx.add(ev(13, "episode_start", "E2", task="t"), "stream")
    assert idx.add(ev(15, "episode_abort", "E2", reason="estop"), "stream")
    assert idx.skip_missing(up_to=15) == [14]                       # the log does not have 14 either
    assert idx.finished[-1]["outcome"] == "aborted" and idx.finished[-1]["reason"] == "estop"
    assert not idx.add(ev(15, "episode_abort", "E2", reason="estop"), "stream")   # duplicate: ignored
    assert len(read_episodes(tmp_path / "e.jsonl")) == 2


# -- recording -----------------------------------------------------------------------------------

def test_records_streams_and_indexes_episodes(robot, tmp_path):
    run = Running(robot, tmp_path, start_teleop=True)
    assert ("task", {"text": "pick the cup"}) in robot.calls and ("start", None) in robot.calls
    e1 = robot.start_episode()
    time.sleep(0.3)
    robot.stop_episode()
    e2 = robot.start_episode()
    time.sleep(0.2)
    robot.abort_episode("estop")
    wait(lambda: len(run.session.index.finished) == 2)
    time.sleep(0.3)
    d = run.stop()

    eps = read_episodes(d / "episodes.jsonl")
    assert [(e["id"], e["outcome"], e["reason"], e["task"]) for e in eps] == \
        [(e1, "kept", None, "pick the cup"), (e2, "aborted", "estop", "pick the cup")]
    for e in eps:
        start, stop_ = episode_seqs(robot, e["id"])
        assert (e["seq_start"], e["seq_stop"]) == (start, stop_)
        assert e["t_start"] < e["t_stop"] and not e["reconciled"]
    seqs, _ = event_seqs(d)
    assert seqs == list(range(seqs[0], robot.seq + 1)) and seqs[0] == 45   # after the 3 prior events (42..44)

    # every stream: raw records, rows contiguous (nothing lost), frames in order
    for name in ("arm", "estimator"):
        rows = np.concatenate([r["seq"] for _, r in iter_rows(records(d, name))])
        assert len(rows) > 50 and np.all(np.diff(rows) == 1)
    frames = [h for h, _ in records(d, "camera.color") if h["type"] == "frame"]
    assert len(frames) > 10 and all(p == q - 1 for p, q in zip([f["seq"] for f in frames], [f["seq"] for f in frames][1:]))
    kinds = {h["type"] for h, _ in records(d, "quest_teleop.oplog")}
    assert kinds == {"headset", "sent", "event"}
    meta = json.loads((d / "session.json").read_text())
    assert meta["teleop_started"] and meta["episodes"] == 2 and meta["stopped_wall"]
    assert not list(d.glob("*.gaps.jsonl"))
    assert "kept" in run.out.getvalue() and "[collect" in run.out.getvalue()


def test_dropped_operator_stream_recovered_from_eventlog(robot, tmp_path):
    run = Running(robot, tmp_path)
    e1 = robot.start_episode()
    time.sleep(0.2)
    robot.refuse.add("oplog")
    robot.drop("oplog")
    wait(lambda: not run.session._oplog().stats.connected)
    robot.emit("hold", e1, reason="tracking lost")          # all of these are missed by the stream
    robot.stop_episode()
    e2 = robot.start_episode()
    time.sleep(0.6)                                         # a few refused reconnects
    robot.refuse.discard("oplog")
    wait(lambda: run.session._oplog().stats.connected)
    wait(lambda: len(run.session.index.finished) == 1 and run.session.index.open is not None)
    robot.stop_episode()
    wait(lambda: len(run.session.index.finished) == 2)
    d = run.stop()

    eps = read_episodes(d / "episodes.jsonl")
    assert [(e["id"], e["outcome"]) for e in eps] == [(e1, "kept"), (e2, "kept")]
    s1, p1 = episode_seqs(robot, e1)
    assert (eps[0]["seq_start"], eps[0]["seq_stop"]) == (s1, p1)
    assert eps[0]["events"] == [e["seq"] for e in robot.events if s1 <= e["seq"] <= p1]
    assert eps[0]["reconciled"] and eps[1]["reconciled"]
    assert eps[0]["t_stop"] == next(e["t"] for e in robot.events if e["seq"] == p1)   # the Pi's own time
    seqs, lines = event_seqs(d)
    assert seqs == list(range(45, robot.seq + 1))
    assert {x["via"] for x in lines if x["seq"] in (p1, p1 + 1)} == {"eventlog"}
    gaps = [json.loads(x) for x in (d / "quest_teleop.oplog.gaps.jsonl").read_text().splitlines()]
    assert len(gaps) == 1
    g = gaps[0]
    assert g["partial_bytes"] > 0 and g["reconnect"]["attempts"] >= 2 and "503" in " ".join(g["retry_errors"])
    assert g["pi_t_last"] < g["reconnect"]["pi_t_first"] and g["wall"] < g["reconnect"]["wall"]


def test_event_skipped_on_stream_is_fetched(robot, tmp_path):
    run = Running(robot, tmp_path)
    e1 = robot.start_episode()
    robot.skip_on_stream.add(robot.seq + 2)                 # the episode_stop after the B button
    robot.stop_episode()
    robot.emit("disabled", reason="X")                      # the next streamed event reveals the gap
    wait(lambda: len(run.session.index.finished) == 1)
    d = run.stop()
    ep = read_episodes(d / "episodes.jsonl")[0]
    assert (ep["id"], ep["outcome"], ep["seq_stop"]) == (e1, "kept", robot.seq - 1) and ep["reconciled"]
    assert any(s >= 45 for s in robot.eventlog_calls)


def test_open_episode_closed_at_session_end(robot, tmp_path):
    run = Running(robot, tmp_path)
    e1 = robot.start_episode()
    time.sleep(0.3)
    d = run.stop()
    ep = read_episodes(d / "episodes.jsonl")[0]
    assert (ep["id"], ep["outcome"], ep["reason"], ep["seq_stop"]) == (e1, "aborted", "session ended", None)
    assert ep["t_stop"] - ep["t_start"] > 0.3


def test_stop_teleop_on_exit(robot, tmp_path):
    run = Running(robot, tmp_path, start_teleop=True, stop_teleop=True)
    e1 = robot.start_episode()
    time.sleep(0.2)
    d = run.stop()
    assert ("stop", None) in robot.calls
    ep = read_episodes(d / "episodes.jsonl")[0]
    assert (ep["id"], ep["outcome"], ep["reason"]) == (e1, "aborted", "teleop stopped")   # the node's own abort
    _, lines = event_seqs(d)
    assert lines[-1]["kind"] == "teleop_stop"


def test_start_teleop_refused(robot, tmp_path):
    robot.start_status = 409
    s = Session(robot.url, tmp_path, "t", ["arm/stream"], start_teleop=True, settle=0, session_name="s",
                status_stream=io.StringIO())
    with pytest.raises(RuntimeError, match="409"):
        asyncio.run(s.run(install_signals=False))
    assert json.loads((s.dir / "session.json").read_text())["stop_reason"].startswith("error")


# -- cutting -------------------------------------------------------------------------------------

def test_cut_by_pi_time(robot, tmp_path):
    run = Running(robot, tmp_path)
    time.sleep(0.3)
    e1 = robot.start_episode()
    time.sleep(0.3)
    robot.drop("camera")                                     # a camera gap inside the episode
    time.sleep(0.3)
    robot.stop_episode()
    time.sleep(0.3)
    e2 = robot.start_episode()
    time.sleep(0.3)
    robot.stop_episode()
    wait(lambda: len(run.session.index.finished) == 2)
    time.sleep(0.2)
    d = run.stop()

    pre, post = 0.1, 0.05
    eps = cut_session(d, pre_roll=pre, post_roll=post)
    assert [e["id"] for e in eps] == [e1, e2]
    index = {e["id"]: e for e in read_episodes(d / "episodes.jsonl")}
    for ep in eps:
        lo, hi = index[ep["id"]]["t_start"] - pre, index[ep["id"]]["t_stop"] + post
        assert ep["window"] == [lo, hi]
        on_disk = json.loads((d / "episodes" / ep["id"] / "episode.json").read_text())
        assert on_disk == ep
        for name in ("arm", "estimator"):
            src = np.concatenate([r[r.dtype.names[0]] for _, r in iter_rows(records(d, name))])
            want = int(((src >= lo) & (src <= hi)).sum())
            got = list(iter_rows(iter_records(open(d / "episodes" / ep["id"] / f"{name}.bin", "rb"))))
            t = np.concatenate([r[r.dtype.names[0]] for _, r in got])
            assert len(t) == want == ep["streams"][name]["rows"] and t.min() >= lo and t.max() <= hi
            assert ep["streams"][name]["time_fields"] == {"rows": "t" if name == "estimator" else "t_tick"}
        for name in ("camera.color", "quest_teleop.oplog"):
            src = [header_time(h) for h, _ in records(d, name) if h["type"] != "ping"]
            want = sum(lo <= t <= hi for t in src)
            got = list(iter_records(open(d / "episodes" / ep["id"] / f"{name}.bin", "rb")))
            assert len(got) == want == ep["streams"][name]["records"]
            assert all(lo <= header_time(h) <= hi for h, _ in got)
        assert ep["streams"]["camera.color"]["time_fields"] == {"frame": "t_sensor"}
        assert ep["streams"]["quest_teleop.oplog"]["time_fields"] == {"headset": "t_rx", "sent": "t_send",
                                                                     "event": "t"}
        ev = [h["seq"] for h, _ in iter_records(open(d / "episodes" / ep["id"] / "quest_teleop.oplog.bin", "rb"))
              if h["type"] == "event"]
        assert set(index[ep["id"]]["events"]) <= set(ev) and ev[-1] == index[ep["id"]]["seq_stop"]
        assert ev[0] == index[ep["id"]]["seq_start"] - 1        # the A press, just before: in the pre-roll
    assert len(eps[0]["gaps"]["camera.color"]) == 1 and eps[1]["gaps"]["camera.color"] == []
    assert eps[0]["gaps"]["arm"] == []


def test_rotation(robot, tmp_path):
    run = Running(robot, tmp_path, streams=["arm/stream", "camera/color", "quest_teleop/oplog/stream"],
                  rotate_bytes=6 << 10)
    e1 = robot.start_episode()
    time.sleep(1.0)
    robot.stop_episode()
    wait(lambda: len(run.session.index.finished) == 1)
    d = run.stop()
    for name in ("arm", "camera.color"):
        files = rotated_files(d, name)
        assert len(files) >= 3
        assert all(f.stat().st_size <= (6 << 10) + 4096 for f in files[:-1])
    for f in rotated_files(d, "arm"):                         # each file decodes on its own
        first = next(iter_records(open(f, "rb")))[0]
        assert first["type"] == "schema"
        assert sum(len(r) for _, r in iter_rows(iter_records(open(f, "rb")))) > 0
    seqs = np.concatenate([r["seq"] for _, r in iter_rows(records(d, "arm"))])
    assert np.all(np.diff(seqs) == 1)                         # nothing lost or doubled across the files
    meta = json.loads((d / "session.json").read_text())
    assert meta["files"]["arm"] == [f.name for f in rotated_files(d, "arm")]
    ep = cut_session(d)[0]
    assert ep["id"] == e1 and ep["streams"]["arm"]["rows"] > 50


def test_ctrl_c_stops_cleanly(robot, tmp_path):
    src = Path(__file__).resolve().parents[1] / "src"
    env = {**os.environ, "PYTHONPATH": f"{src}{os.pathsep}{os.environ.get('PYTHONPATH', '')}"}
    p = subprocess.Popen([sys.executable, "-m", "robot_utils.collect", "run", "--robot", robot.url,
                          "--out", str(tmp_path), "--task", "fold", "--name", "s", "--settle", "0.2",
                          "--status-every", "0.3", "--streams", "arm/stream,quest_teleop/oplog/stream"],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, text=True)
    try:
        d = tmp_path / "s"
        wait(lambda: (d / "quest_teleop.oplog.000.bin").exists() and robot.connections.get("oplog"))
        time.sleep(0.3)
        e1 = robot.start_episode()
        time.sleep(0.5)
        p.send_signal(signal.SIGINT)
        out, _ = p.communicate(timeout=20)
    finally:
        if p.poll() is None:
            p.kill()
    assert p.returncode == 0, out
    assert "[collect" in out and "stopped (SIGINT)" in out
    meta = json.loads((d / "session.json").read_text())
    assert meta["stop_reason"] == "SIGINT" and meta["stopped_wall"]
    ep = read_episodes(d / "episodes.jsonl")[0]
    assert (ep["id"], ep["outcome"], ep["reason"]) == (e1, "aborted", "session ended")
    for name in ("arm", "quest_teleop.oplog"):              # whole records only: no truncated tail
        assert len(records(d, name)) > 10


def test_event_record_shapes():
    line = {"seq": 7, "t": 12.5, "wall": 1e9, "kind": "hold", "episode": None, "detail": {"reason": "x"}}
    flat = {"type": "event", "len": 0, **line}
    nested = {"type": "event", "len": 0, "event": line}
    assert event_line(flat) == line and event_line(nested) == line
    assert header_time(flat) == header_time(nested) == 12.5
