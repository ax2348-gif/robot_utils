"""rows_stream(max_hz=...): overviews get at most max_hz rows per second, each the latest row."""
import asyncio

import numpy as np

from robot_utils.nodes.common.hub import RowHub
from robot_utils.nodes.common.protocol import RecordReader
from robot_utils.nodes.common.server import rows_stream


def collect(max_hz, seconds=0.6, rate_hz=200):
    hub = RowHub(2)
    reader, rows = RecordReader(), []

    async def produce():
        i = 0
        while True:
            hub.publish_rows(np.array([[i, i * 2.0]]))
            i += 1
            await asyncio.sleep(1 / rate_hz)

    async def consume():
        gen = rows_stream(hub, ["i", "x"], ["", ""], max_hz=max_hz)
        async for chunk in gen:
            for h, body in reader.feed(chunk):
                if h["type"] == "rows":
                    rows.append(np.frombuffer(body, "<f8").reshape(h["n"], 2))

    async def main():
        hub.attach(asyncio.get_running_loop())     # nodes do this at startup
        tasks = [asyncio.create_task(produce()), asyncio.create_task(consume())]
        await asyncio.sleep(seconds)
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

    asyncio.run(main())
    return rows


def test_max_hz_sends_one_latest_row_per_interval():
    recs = collect(max_hz=10)
    assert 3 <= len(recs) <= 8, len(recs)
    assert all(r.shape[0] == 1 for r in recs)
    ids = [r[0, 0] for r in recs]
    assert ids == sorted(ids) and ids[-1] - ids[0] > 40        # latest rows, not the first ones


def test_default_sends_every_row():
    recs = collect(max_hz=0, seconds=0.3)
    ids = np.concatenate(recs)[:, 0]
    assert len(ids) > 30 and np.all(np.diff(ids) == 1)
