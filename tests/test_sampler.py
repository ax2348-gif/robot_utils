"""common/sampler: the on-demand stack sampler and its /debug/profile route."""
import threading
import time

from fastapi import FastAPI
from fastapi.testclient import TestClient

from robot_utils.nodes.common import sampler


def _busy(stop):
    while not stop.is_set():
        sum(i * i for i in range(2000))


def _sleepy(stop):
    while not stop.is_set():
        time.sleep(0.01)


def test_profile_attributes_cpu_and_sleep_to_named_threads():
    stop = threading.Event()
    ths = [threading.Thread(target=_busy, args=(stop,), name="spinner", daemon=True),
           threading.Thread(target=_sleepy, args=(stop,), name="napper", daemon=True)]
    for t in ths:
        t.start()
    try:
        out = sampler.profile(seconds=1.0, hz=100)
    finally:
        stop.set()
    by = {t["name"]: t for t in out["threads"]}
    assert out["samples"] >= 50 and out["sampler_late_ms"]["p50"] is not None
    spin, nap = by["spinner"], by["napper"]
    assert spin["native_id"] and spin["samples"] >= 50
    assert any("_busy" in s["stack"] for s in spin["runnable_stacks"] + spin["sleeping_stacks"])
    assert any("_sleepy" in s["stack"] for s in nap["sleeping_stacks"])
    if spin["cpu_pct"] is not None:                    # Linux (/proc): the spinner burns CPU, the napper doesn't
        assert spin["cpu_pct"] > 20 and spin["runnable_pct"] > nap["runnable_pct"]
        assert (nap["cpu_pct"] or 0) < spin["cpu_pct"]


def test_profile_route_one_at_a_time_and_bounds():
    app = FastAPI()
    sampler.add_profile_route(app)
    sampler.add_profile_route(app)                     # idempotent
    assert sum(getattr(r, "path", "") == "/debug/profile" for r in app.routes) == 1
    c = TestClient(app)
    r = c.get("/debug/profile", params={"seconds": 0.5, "hz": 50, "folded": True})
    assert r.status_code == 200 and "folded_runnable" in r.json() and r.json()["hz"] == 50
    assert c.get("/debug/profile", params={"seconds": 999}).status_code == 422
    assert sampler._busy.acquire(blocking=False)
    try:
        assert c.get("/debug/profile", params={"seconds": 0.5}).status_code == 409
    finally:
        sampler._busy.release()
