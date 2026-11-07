"""robot_utils.nodes.common: framing, rows, lease, e-stop, hub (no hardware, no server)."""
import asyncio
import io

import numpy as np
import pytest

from robot_utils.nodes.common.estop import EStop
from robot_utils.nodes.common.hub import RowHub
from robot_utils.nodes.common.lease import Lease, LeaseError
from robot_utils.nodes.common.protocol import (RecordReader, RowsDecoder, iter_records, iter_rows,
                                               pack_record, rows_record, schema_record)


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def test_records_survive_any_chunking():
    data = (pack_record({"type": "ping"}) + pack_record({"type": "x", "k": 1}, b"abc")
            + pack_record({"type": "y"}, bytes(range(256)) * 10))
    for size in (1, 3, 7, 1000):
        r, out = RecordReader(), []
        for i in range(0, len(data), size):
            out += r.feed(data[i:i + size])
        assert [h["type"] for h, _ in out] == ["ping", "x", "y"]
        assert out[1] == ({"type": "x", "k": 1, "len": 3}, b"abc")
        assert r.pending == 0


def test_schema_and_rows_roundtrip_through_a_file():
    a = np.arange(12.0).reshape(4, 3)
    blob = (schema_record(1, ["t", "x", "y"], ["s", "m", "m"]) + pack_record({"type": "ping"})
            + rows_record(a[:3], 1, dropped=0) + rows_record(a[3:], 1))
    got = list(iter_rows(iter_records(io.BytesIO(blob))))
    assert [len(r) for _, r in got] == [3, 1]
    assert got[0][1]["x"].tolist() == [1.0, 4.0, 7.0]
    assert got[1][1]["t"][0] == 9.0


def test_rows_with_inline_fields_and_unknown_schema():
    dec = RowsDecoder()
    rec = rows_record(np.ones((2, 2)), None, fields=["a", "b"])
    h, p = RecordReader().feed(rec)[0]
    assert dec.decode(h, p)["b"].tolist() == [1.0, 1.0]
    h, p = RecordReader().feed(rows_record(np.ones((1, 2)), 7))[0]
    with pytest.raises(ValueError):
        dec.decode(h, p)


def test_lease_acquire_renew_release_and_conflict():
    ended = []
    clock = Clock()
    lease = Lease(heartbeat=1.0, on_end=ended.append, clock=clock)
    tok, info = lease.acquire("alice")
    with pytest.raises(LeaseError) as e:
        lease.acquire("bob")
    assert e.value.status == 409
    for bad, status in ((None, 401), ("nope", 403)):
        with pytest.raises(LeaseError) as e:
            lease.check(bad)
        assert e.value.status == status
    clock.t += 0.9
    lease.renew(tok)
    clock.t += 0.9                       # 1.8 s after acquire, 0.9 after renew: still held
    assert not lease.expire_if_due() and lease.check(tok).client == "alice"
    lease.release(tok)
    assert ended == ["released"] and not lease.held
    with pytest.raises(LeaseError):
        lease.check(tok)


def test_lease_lapses_once():
    ended = []
    clock = Clock()
    lease = Lease(heartbeat=1.0, on_end=ended.append, clock=clock)
    tok, _ = lease.acquire("alice")
    clock.t += 1.01
    with pytest.raises(LeaseError):
        lease.check(tok)
    assert lease.expire_if_due() and not lease.expire_if_due()
    assert ended == ["lapsed"]
    tok2, info2 = lease.acquire("bob")   # free again
    assert tok2 != tok and info2.lease_id == 2


def test_estop_latches_and_reports_first_trigger():
    hits = []
    e = EStop(on_trigger=hits.append)
    e.trigger("button", "viewer")
    e.trigger("again", "other")
    assert e.active and e.info.reason == "button" and len(hits) == 1
    e.clear()
    assert not e.active


def test_hub_delivers_every_row_and_counts_drops():
    async def main():
        hub = RowHub(columns=2)
        hub.attach(asyncio.get_running_loop())
        fast, slow = hub.subscribe(100), hub.subscribe(3)
        for i in range(5):
            hub.publish_rows([[i, i]])
        await asyncio.sleep(0.01)
        rows, dropped = fast.take()
        assert rows[:, 0].tolist() == [0, 1, 2, 3, 4] and dropped == 0
        rows, dropped = slow.take()
        assert rows[:, 0].tolist() == [2, 3, 4] and dropped == 2
        assert hub.recent(2)[:, 0].tolist() == [3, 4]
        with pytest.raises(ValueError):
            hub.publish_rows([[1, 2, 3]])
    asyncio.run(main())


def test_hub_named_channels_are_independent():
    async def main():
        hub = RowHub({"raw": 3, "state": 2})
        hub.attach(asyncio.get_running_loop())
        raw, state = hub.subscribe(10, "raw"), hub.subscribe(10, "state")
        hub.publish_rows([[1, 2, 3]], "raw")
        hub.publish_rows([[4, 5]], "state")
        await asyncio.sleep(0.01)
        assert raw.take()[0].tolist() == [[1, 2, 3]] and state.take()[0].tolist() == [[4, 5]]
        assert hub.recent(5, "state").shape == (1, 2) and hub.subscribers == 2
        with pytest.raises(KeyError):
            hub.publish_rows([[1, 2]], "nope")
        with pytest.raises(ValueError):
            hub.publish_rows([[1, 2]], "raw")
    asyncio.run(main())
