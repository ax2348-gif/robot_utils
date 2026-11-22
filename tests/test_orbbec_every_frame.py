"""Every-frame mode of the Orbbec node's /stream (``every=true``), for recording clients.

    PYTHONPATH=src python -m pytest tests/test_orbbec_every_frame.py -q

Most tests drive the server's record generators directly against a Hub on a private event loop, so
"a slow client" is exact (the test simply does not pull while frames are published). The frames
are published from a separate thread, like the camera owner thread. One end-to-end test runs the
real server on the FakeBackend.
"""
import asyncio
import gc
import json
import struct
import threading
import time
import weakref

import numpy as np
import pytest

pytest.importorskip("fastapi")
pytest.importorskip("pydantic")

from robot_utils.nodes.orbbec.hub import Frame, Hub  # noqa: E402

KB = 1000


def _parse(rec: bytes):
    n = struct.unpack(">I", rec[:4])[0]
    h = json.loads(rec[4:4 + n])
    return h, rec[4 + n:]


class _Seq:
    def __init__(self):
        self.seq = {}

    def frame(self, stream="color", size=100 * KB):
        self.seq[stream] = self.seq.get(stream, 0) + 1
        s = self.seq[stream]
        if stream == "depth":
            data = np.full((1, size // 2), s, np.uint16)
            fr = Frame(stream, "Y16", size // 2, 1, data, 0.0, None, 0, 0)
        else:
            data = s.to_bytes(4, "big") + bytes(size - 4)          # MJPG-looking, seq in the first bytes
            fr = Frame(stream, "MJPG", 640, 480, data, 0.0, None, 0, 0)
        fr.seq = s
        fr.t_publish = time.monotonic()
        return fr


def _publish_from_thread(hub, frames_lists):
    """Publish like the owner thread does; returns the seconds the publishing thread spent."""
    took = []

    def run():
        t0 = time.perf_counter()
        for fl in frames_lists:
            hub.publish_frames(fl)
        took.append(time.perf_counter() - t0)
    th = threading.Thread(target=run)
    th.start()
    th.join(5)
    assert not th.is_alive()
    return took[0]


@pytest.fixture
def node():
    from robot_utils.nodes.orbbec.fake_backend import FakeBackend
    from robot_utils.nodes.orbbec.models import Settings
    from robot_utils.nodes.orbbec.owner import Owner
    from robot_utils.nodes.orbbec.server import create_app
    hub = Hub()
    owner = Owner(FakeBackend(), hub, Settings(), calibration_file=None)    # never started: no device thread
    app = create_app(owner, hub)
    return hub, app


def run(coro):
    return asyncio.run(asyncio.wait_for(coro, 20))


async def _start(hub, gen):
    """Attach the hub to this loop and start the generator (it subscribes on its first step)."""
    hub.attach(asyncio.get_running_loop())
    task = asyncio.ensure_future(gen.__anext__())
    for _ in range(3):
        await asyncio.sleep(0)
    return task


async def _settle():
    for _ in range(5):          # let call_soon_threadsafe callbacks (the dispatch) run
        await asyncio.sleep(0)


def test_slow_client_gets_every_frame_in_order_up_to_the_bound(node):
    hub, app = node
    gen = app.state.every_frame_records({"color": "jpeg"}, False, 10, 10**9)
    s = _Seq()

    async def main():
        first = await _start(hub, gen)
        _publish_from_thread(hub, [[s.frame()]])
        h, body = _parse(await first)
        assert (h["seq"], h["dropped"], body[:4]) == (1, 0, (1).to_bytes(4, "big"))
        # the client stalls (does not pull) while 10 more frames arrive: exactly the bound
        _publish_from_thread(hub, [[s.frame()] for _ in range(10)])
        await _settle()
        got = [_parse(await gen.__anext__())[0] for _ in range(10)]
        assert [g["seq"] for g in got] == list(range(2, 12)) and all(g["dropped"] == 0 for g in got)
        await gen.aclose()
    run(main())


def test_past_the_bound_the_oldest_are_dropped_and_counted_exactly(node):
    hub, app = node
    gen = app.state.every_frame_records({"color": "jpeg", "depth": "raw"}, False, 10, 10**9)
    s = _Seq()

    async def main():
        first = await _start(hub, gen)
        _publish_from_thread(hub, [[s.frame("color")]])
        assert _parse(await first)[0]["seq"] == 1
        # stalled through 20 framesets (40 frames) with room for 10: the 30 oldest go
        _publish_from_thread(hub, [[s.frame("color"), s.frame("depth", 2000)] for _ in range(20)])
        await _settle()
        got = [_parse(await gen.__anext__()) for _ in range(10)]
        hs = [h for h, _ in got]
        assert [(h["stream"], h["seq"]) for h in hs] == [p for k in range(17, 22) for p in (("color", k), ("depth", k - 1))]
        # color seq 2..16 (15) and depth seq 1..15 (15) dropped: each count rides its stream's next record
        assert [h["dropped"] for h in hs] == [15, 15] + [0] * 8
        assert np.frombuffer(got[1][1], np.uint16)[0] == 16 and hs[1]["dtype"] == "<u2"   # raw depth payload
        # and the next frames after catching up carry no stale drop count
        _publish_from_thread(hub, [[s.frame("color")]])
        await _settle()
        h, _ = _parse(await gen.__anext__())
        assert (h["seq"], h["dropped"]) == (22, 0)
        await gen.aclose()
    run(main())


def test_fast_client_gets_all_frames(node):
    hub, app = node
    gen = app.state.every_frame_records({"color": "jpeg"}, False, 3, 10**9)
    s = _Seq()

    async def main():
        first = await _start(hub, gen)
        _publish_from_thread(hub, [[s.frame()]])
        seqs = [_parse(await first)[0]["seq"]]
        for _ in range(199):
            _publish_from_thread(hub, [[s.frame()]])
            h, _ = _parse(await gen.__anext__())
            seqs.append(h["seq"])
            assert h["dropped"] == 0
        assert seqs == list(range(1, 201))
        await gen.aclose()
    run(main())


def test_latest_frame_wins_stays_the_default(node):
    hub, app = node
    gen = app.state.frame_records({"color": "jpeg"}, False)
    s = _Seq()

    async def main():
        first = await _start(hub, gen)
        _publish_from_thread(hub, [[s.frame()]])
        assert _parse(await first)[0]["seq"] == 1
        assert not hub._frame_subs                     # a live client costs no queue
        _publish_from_thread(hub, [[s.frame()] for _ in range(25)])
        await _settle()
        h, _ = _parse(await gen.__anext__())
        assert (h["seq"], h["dropped"]) == (26, 24)    # skipped to the newest, skip counted
        await gen.aclose()
    run(main())


def test_owner_never_blocks_on_a_stalled_client_and_the_queue_is_bounded(node):
    hub, app = node
    gen = app.state.every_frame_records({"color": "jpeg"}, False, 60, 10**9)
    s = _Seq()

    async def main():
        first = await _start(hub, gen)
        _publish_from_thread(hub, [[s.frame()]])
        await first
        sub = next(iter(hub._frame_subs))
        # the client never reads again; 3000 frames (100 s at 30 fps) from the owner thread
        frames = [[s.frame()] for _ in range(3000)]
        took = _publish_from_thread(hub, frames)
        assert took < 0.5, took                       # ~a few us per publish; nothing waits for the client
        await _settle()
        assert len(sub.queue) == 60 and sub.peak == 60 and sub.bytes == 60 * 100 * KB
        assert sub.dropped == {"color": 2940}
        await gen.aclose()
    run(main())


def test_byte_bound(node):
    hub, app = node
    gen = app.state.every_frame_records({"color": "jpeg"}, False, 1000, 350 * KB)
    s = _Seq()

    async def main():
        first = await _start(hub, gen)
        _publish_from_thread(hub, [[s.frame()]])
        await first
        sub = next(iter(hub._frame_subs))
        _publish_from_thread(hub, [[s.frame()] for _ in range(20)])
        await _settle()
        assert len(sub.queue) == 3 and sub.bytes == 300 * KB
        h, _ = _parse(await gen.__anext__())
        assert (h["seq"], h["dropped"]) == (19, 17)
        await gen.aclose()
    run(main())


def test_disconnect_frees_the_queue_and_the_frames(node):
    hub, app = node
    gen = app.state.every_frame_records({"color": "jpeg"}, False, 60, 10**9)
    s = _Seq()

    async def main():
        first = await _start(hub, gen)
        _publish_from_thread(hub, [[s.frame()]])
        await first
        assert hub.clients == 1 and len(hub._frame_subs) == 1
        queued = [[s.frame()] for _ in range(30)]
        refs = [weakref.ref(fl[0]) for fl in queued]
        _publish_from_thread(hub, queued)
        await _settle()
        sub = next(iter(hub._frame_subs))
        assert len(sub.queue) == 30
        del queued
        await gen.aclose()                            # what Starlette does when the client goes away
        assert not hub._frame_subs and hub.clients == 0 and len(sub.queue) == 0 and sub.bytes == 0
        hub._latest.clear()
        gc.collect()
        assert all(r() is None for r in refs)         # nothing else kept the frames alive
    run(main())


def test_encodes_are_shared_once_per_frame(node):
    """Depth in jpeg mode needs an encode: two every-frame clients at different lags and a live
    client share one encode per frame (the cache), not one per client."""
    hub, app = node
    a = app.state.every_frame_records({"depth": "jpeg"}, False, 60, 10**9)
    b = app.state.every_frame_records({"depth": "jpeg"}, False, 60, 10**9)
    s = _Seq()

    async def main():
        ta = await _start(hub, a)
        tb = asyncio.ensure_future(b.__anext__())
        await _settle()
        _publish_from_thread(hub, [[s.frame("depth", 640 * 2 * 4)]])
        await ta
        await tb
        _publish_from_thread(hub, [[s.frame("depth", 640 * 2 * 4)] for _ in range(9)])
        await _settle()
        ha = [_parse(await a.__anext__())[0]["seq"] for _ in range(9)]
        hb = [_parse(await b.__anext__())[0]["seq"] for _ in range(9)]
        assert ha == hb == list(range(2, 11))
        await a.aclose()
        await b.aclose()
    run(main())
    assert app.state.encode_cache.encodes == 10


# -- end to end over HTTP on the synthetic camera -----------------------------------------------

def test_every_frame_over_http(tmp_path):
    uvicorn = pytest.importorskip("uvicorn")
    pytest.importorskip("httpx")
    import socket
    from robot_utils.nodes.orbbec.client import OrbbecClient
    from robot_utils.nodes.orbbec.fake_backend import FakeBackend
    from robot_utils.nodes.orbbec.models import Settings
    from robot_utils.nodes.orbbec.owner import Owner
    from robot_utils.nodes.orbbec.server import create_app
    hub = Hub()
    app = create_app(Owner(FakeBackend(), hub, Settings(), startup_s=1.5,
                           calibration_file=str(tmp_path / "imu.json")), hub)
    with socket.socket() as so:
        so.bind(("127.0.0.1", 0))
        port = so.getsockname()[1]
    server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning",
                                           timeout_graceful_shutdown=1))
    th = threading.Thread(target=server.run, daemon=True)
    th.start()
    deadline = time.time() + 10
    while not server.started and time.time() < deadline:
        time.sleep(0.05)
    try:
        with OrbbecClient(f"http://127.0.0.1:{port}") as c:
            seqs = {"color": [], "depth": []}
            for i, fr in enumerate(c.frames("color,depth:raw", every=True, decode_jpeg=False)):
                seqs[fr.stream].append((fr.header["seq"], fr.header["dropped"]))
                if i == 2:
                    time.sleep(0.8)        # a stall shorter than the backlog: nothing lost
                if min(len(v) for v in seqs.values()) >= 15:
                    break
            for v in seqs.values():
                ks = [k for k, _ in v]
                assert ks == list(range(ks[0], ks[0] + len(ks))) and all(d == 0 for _, d in v)
            r = c.http.get("/stream", params={"streams": "color", "every": "true", "max_backlog": 0})
            assert r.status_code == 422
    finally:
        server.should_exit = True
        th.join(timeout=5)
    assert not hub._frame_subs
