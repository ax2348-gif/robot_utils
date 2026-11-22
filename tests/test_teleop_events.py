"""The teleop node's event log (src/robot_utils/nodes/quest_teleop/eventlog.py; the contract in
docs/plans/data_collection_v1.md): one JSONL file per day, each line flushed and fsynced at write, seq gap-free and
continued across restarts, the episode counter per day continued too, pruning oldest-first at the size cap, nothing
dropped silently, ``GET /eventlog?since_seq=N&limit=M`` across files and ``GET /eventlog/files``."""
import json
import os
import threading
import time

import httpx
import pytest

from helpers import serve, stop
from robot_utils.nodes.quest_teleop import eventlog as E
from robot_utils.nodes.quest_teleop.node import QuestTeleopNode
from robot_utils.nodes.quest_teleop.server import create_app

DAY = 86400.0


class Wall:
    def __init__(self, t=1_800_000_000.0):
        self.t = t

    def __call__(self):
        return self.t


def lines(d):
    out = []
    for n in sorted(os.listdir(d)):
        with open(os.path.join(d, n)) as f:
            out += [json.loads(ln) for ln in f if ln.strip()]
    return out


def test_every_line_is_flushed_and_fsynced_at_write(tmp_path, monkeypatch):
    synced = []
    real = os.fsync

    def fsync(fd):
        synced.append(os.fstat(fd).st_size)          # the line is in the file (flushed) when it is synced
        real(fd)
    monkeypatch.setattr(os, "fsync", fsync)
    log = E.TeleopEventLog(str(tmp_path))
    for i in range(5):
        log.emit("button", {"button": "A", "pressed": bool(i % 2)})
        assert log.flush(2.0)
    assert len(synced) == 5 and synced == sorted(synced) and len(set(synced)) == 5
    got = lines(tmp_path)
    assert [ln["seq"] for ln in got] == [1, 2, 3, 4, 5]
    assert set(got[0]) == {"seq", "t", "wall", "kind", "episode", "detail"}
    assert got[0]["kind"] == "button" and got[0]["detail"] == {"button": "A", "pressed": False}
    assert got[0]["episode"] is None
    log.close()


def test_the_line_has_the_contract_shape_and_unknown_kinds_are_refused(tmp_path):
    t = {"v": 12.5}
    log = E.TeleopEventLog(str(tmp_path), clock=lambda: t["v"], wall=Wall())
    ln = log.emit("episode_start", {"task": "pick"}, "20270115-0001")
    assert ln == {"seq": 1, "t": 12.5, "wall": 1_800_000_000.0, "kind": "episode_start", "episode": "20270115-0001",
                  "detail": {"task": "pick"}}
    with pytest.raises(ValueError):
        log.emit("recording")
    log.close()
    assert lines(tmp_path) == [ln]
    assert set(E.KINDS) == {"button", "armed", "disarmed", "enabled", "disabled", "episode_start", "episode_stop",
                            "episode_abort", "estop", "fault", "hold", "stale", "teleop_start", "teleop_stop",
                            "sim_reset"}


def test_seq_and_the_episode_counter_continue_across_a_restart(tmp_path):
    wall = Wall()
    log = E.TeleopEventLog(str(tmp_path), wall=wall)
    e1 = log.next_episode()
    log.emit("episode_start", {"task": ""}, e1)
    e2 = log.next_episode()
    log.emit("episode_start", {"task": ""}, e2)
    log.emit("teleop_stop", {"reason": "requested"})
    log.close()
    day = E.day_of(wall.t)
    assert (e1, e2) == (f"{day}-0001", f"{day}-0002")
    # a crash cut the last line: it is skipped, the next line starts on its own line
    with open(tmp_path / f"events-{day}.jsonl", "ab") as f:
        f.write(b'{"seq": 4, "t": 1, "wal')
    log = E.TeleopEventLog(str(tmp_path), wall=wall)
    assert log.seq == 3
    assert log.emit("teleop_start")["seq"] == 4
    assert log.next_episode() == f"{day}-0003"
    log.close()
    got = []
    for raw in open(tmp_path / f"events-{day}.jsonl"):
        try:
            got.append(json.loads(raw)["seq"])
        except ValueError:
            pass
    assert got == [1, 2, 3, 4]
    # the next day: a new file, seq continues, episodes number from 1
    wall.t += DAY
    log = E.TeleopEventLog(str(tmp_path), wall=wall)
    assert log.emit("teleop_start")["seq"] == 5
    assert log.next_episode() == f"{E.day_of(wall.t)}-0001"
    log.close()
    # a newest file without a valid line: the seq comes from the file before
    (tmp_path / f"events-{E.day_of(wall.t + DAY)}.jsonl").write_bytes(b"garbage\n")
    log = E.TeleopEventLog(str(tmp_path), wall=wall)
    assert log.seq == 5
    log.close()


def test_pruned_oldest_first_at_the_size_cap(tmp_path):
    wall = Wall()
    log = E.TeleopEventLog(str(tmp_path), cap_bytes=3000, wall=wall)
    for day in range(6):
        for _ in range(10):                          # ~100 bytes per line: ~1 kB per day
            log.emit("hold", {"reason": "x" * 20})
        assert log.flush(2.0)
        wall.t += DAY
    log.close()
    names = sorted(os.listdir(tmp_path))
    assert sum(os.path.getsize(tmp_path / n) for n in names) <= 3000
    assert log.pruned == 6 - len(names) and log.pruned >= 2
    assert names[-1] == f"events-{E.day_of(wall.t - DAY)}.jsonl"   # the newest kept, the oldest gone
    seqs = [ln["seq"] for ln in lines(tmp_path)]
    assert seqs == list(range(seqs[0], 61)) and seqs[0] > 1
    # never the file being written, even when it alone is over the cap
    log = E.TeleopEventLog(str(tmp_path), cap_bytes=200, wall=wall)
    for _ in range(10):
        log.emit("hold", {"reason": "y" * 20})
    log.close()
    assert os.listdir(tmp_path) == [f"events-{E.day_of(wall.t)}.jsonl"]


def test_since_seq_across_files_with_limit_and_the_files_list(tmp_path):
    wall = Wall()
    log = E.TeleopEventLog(str(tmp_path), wall=wall)
    for day in range(3):
        for _ in range(4):
            log.emit("button", {"button": "B", "pressed": True})
        wall.t += DAY
    got = [json.loads(b) for b in log.since(2)]
    assert [g["seq"] for g in got] == list(range(3, 13))
    assert [json.loads(b)["seq"] for b in log.since(3, limit=5)] == [4, 5, 6, 7, 8]
    assert list(log.since(12)) == []
    files = log.files()
    assert [(f["first_seq"], f["last_seq"]) for f in files] == [(1, 4), (5, 8), (9, 12)]
    assert all(f["bytes"] > 0 and f["name"].startswith("events-") for f in files)
    log.close()


def test_without_a_data_dir_the_log_is_in_memory(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    log = E.TeleopEventLog(None)
    for _ in range(3):
        log.emit("armed", {"arm_seq": 1})
    assert [json.loads(b)["seq"] for b in log.since(1)] == [2, 3]
    assert log.files() == [] and os.listdir(tmp_path) == []
    log.close()


def test_never_dropped_silently_and_close_writes_everything(tmp_path, monkeypatch, caplog):
    gate = threading.Event()
    real = os.fsync

    def slow(fd):
        gate.wait(5)
        real(fd)
    monkeypatch.setattr(os, "fsync", slow)
    log = E.TeleopEventLog(str(tmp_path), queue_size=3)
    t0 = time.monotonic()
    for _ in range(10):                              # the writer is stuck on the first: the queue overflows
        log.emit("button", {"button": "X", "pressed": True})
    assert time.monotonic() - t0 < 0.5               # emit never blocks
    assert log.dropped >= 5 and "queue is full" in log.last_error
    assert any("not written" in r.message for r in caplog.records)
    assert log.info()["dropped"] == log.dropped
    gate.set()
    log.close()                                     # everything that was queued is on disk before it returns
    assert len(lines(tmp_path)) == 10 - log.dropped


def test_write_errors_are_counted(tmp_path, monkeypatch):
    def broken(fd):
        raise OSError(5, "I/O error")
    monkeypatch.setattr(os, "fsync", broken)
    monkeypatch.setattr(E.time, "sleep", lambda s: None)
    log = E.TeleopEventLog(str(tmp_path))
    log.emit("fault", {"reason": "test"})
    assert log.flush(2.0)
    assert log.write_errors == 3 and log.dropped == 1 and "I/O error" in log.last_error
    log.close()


def test_the_eventlog_endpoints(tmp_path):
    node = QuestTeleopNode("http://127.0.0.1:9", quest_host="127.0.0.1", quest_port=1, ar="off",
                           data_dir=str(tmp_path / "events"))
    url, srv, th = serve(create_app(node, start=False))
    try:
        for i in range(30):
            node.emit("button", {"button": "Y", "pressed": bool(i % 2)})
        with httpx.Client(base_url=url, timeout=5) as c:
            r = c.get("/eventlog", params={"since_seq": 5})
            assert r.headers["content-type"].startswith("application/x-ndjson")
            got = [json.loads(ln) for ln in r.text.splitlines()]
            assert [g["seq"] for g in got] == list(range(6, 31))
            page = [json.loads(ln)["seq"] for ln in c.get("/eventlog", params={"since_seq": 0, "limit": 10}).text
                    .splitlines()]
            assert page == list(range(1, 11))
            # the default limit is 1000
            for _ in range(1000):
                node.emit("hold", {"reason": "h"})
            assert len(c.get("/eventlog").text.splitlines()) == 1000
            files = c.get("/eventlog/files").json()
            assert files == [{"name": f"events-{E.day_of(time.time())}.jsonl", "bytes": files[0]["bytes"],
                              "first_seq": 1, "last_seq": 1030}]
            s = c.get("/state").json()
            assert s["eventlog"]["seq"] == 1030 and s["last_event"]["seq"] == 1030
            assert s["config"]["data_dir"] == str(tmp_path / "events")
    finally:
        stop((srv, th))
        node.eventlog.close()
