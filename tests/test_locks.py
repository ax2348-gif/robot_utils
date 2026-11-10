"""Lockout / tagout: the lock store, chocks on the swerve node, covers on the camera, the landing
page's placement API and the admin CLI. Lock files live in a per-test directory (conftest)."""
import json
import socket
import threading
import time

import httpx
import pytest
import uvicorn

from robot_utils.nodes.common.lease import RequestError
from robot_utils.nodes.common.locks import LockStore


def wait_for(pred, timeout=5.0, interval=0.02):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        v = pred()
        if v:
            return v
        time.sleep(interval)
    raise AssertionError("condition not met in time")


# -- the store --------------------------------------------------------------------------------------

def test_store_place_match_persist_remove(tmp_locks_dir):
    s = LockStore()
    lock, created = s.place("chock", "swerve", "avery", "battery swap")
    assert created and lock.id.startswith("chock-") and (tmp_locks_dir / f"{lock.id}.json").exists()
    s.place("chock", "all", "bob", "")
    s.place("cover", "camera", "carol", "meeting")
    assert len(s.matching("chock", {"swerve"})) == 2              # its own + "all"
    assert len(s.matching("chock", {"arm"})) == 1                  # only "all"
    assert len(s.matching("cover", {"camera", "cam2"})) == 1
    fresh = LockStore()                                            # a new process sees the same locks
    assert {l.by for l in fresh.active()} == {"avery", "bob", "carol"}
    fresh.remove(lock.id)
    assert len(LockStore().matching("chock", {"swerve"})) == 1
    with pytest.raises(RequestError):
        fresh.remove(lock.id)
    with pytest.raises(RequestError):
        fresh.remove("../etc/passwd")
    assert len(fresh.remove_all("chock")) == 1 and len(fresh.active(force=True)) == 1
    (tmp_locks_dir / "junk.json").write_text("{not json")          # foreign / broken files are ignored
    assert len(LockStore().active()) == 1


def test_web_placement_is_deduplicated_rate_limited_and_capped():
    s = LockStore()
    a, created = s.place("chock", "all", "web user", "x", source="web 1.2.3.4", now=1000.0)
    b, created2 = s.place("chock", "all", "web user", "x", source="web 5.6.7.8", now=1001.0)
    assert created and not created2 and a.id == b.id               # the same lock, not a second one
    s.place("cover", "camera", "web user", "", source="web 1.2.3.4", now=1002.0)   # a different lock: at once
    cam = [l for l in s.active(force=True) if l.kind == "cover"][0]
    s.remove(cam.id)
    with pytest.raises(RequestError) as e:                         # the same lock again within 30 s
        s.place("cover", "camera", "web user", "", source="web 1.2.3.4", now=1010.0)
    assert e.value.status == 429
    s.place("cover", "camera", "web user", "", source="web 1.2.3.4", now=1040.0)
    for i, t in enumerate(("t1", "t2", "t3")):
        s.place("chock", t, "agent", "", source=f"web 10.0.0.{i}", now=1100.0 + i)
    with pytest.raises(RequestError) as e:                         # 5 web locks active: capped
        s.place("chock", "t4", "agent", "", source="web 10.0.0.9", now=1200.0)
    assert e.value.status == 429
    s.place("chock", "t4", "admin", "", source="ssh")              # SSH is never limited
    with pytest.raises(RequestError):
        s.place("bogus", "x", "a", "")


# -- chocks: the swerve node -----------------------------------------------------------------------------

def test_chock_stops_and_blocks_the_swerve():
    from robot_utils.nodes.swerve.node import SwerveNode
    from robot_utils.nodes.swerve.sim import start_sim, stop_sim
    sims, iface = start_sim()
    node = SwerveNode(iface)
    node.start()
    try:
        wait_for(lambda: node.status == "running", 10)
        token, _ = node.acquire("t")
        node.enable(token)
        node.command(token, (0.2, 0, 0))
        wait_for(lambda: node._applied is not None, 2)
        lock, _ = LockStore().place("chock", "swerve", "admin", "test")
        wait_for(lambda: not node.enabled, 1.0)                     # stopped and disabled within ticks
        assert node._applied is None
        with pytest.raises(RequestError) as e:
            node.enable(token)
        assert e.value.status == 423 and "chocked" in str(e.value)
        st = node.state()
        assert st["chocked"] and st["locks"][0]["id"] == lock.id
        LockStore().remove(lock.id)
        time.sleep(0.3)
        node.enable(token)                                          # free again
        assert node.enabled and not node.state()["chocked"]
    finally:
        node.close()
        stop_sim(sims, iface)


# -- covers: the camera node -----------------------------------------------------------------------------

def test_cover_swaps_the_camera_to_procedural_frames_and_keeps_the_imu(tmp_path):
    from robot_utils.nodes.orbbec.fake_backend import FakeBackend
    from robot_utils.nodes.orbbec.hub import Hub
    from robot_utils.nodes.orbbec.models import Settings
    from robot_utils.nodes.orbbec.owner import Owner
    hub = Hub()
    seen = []
    orig = hub.publish_frames
    hub.publish_frames = lambda frames: (seen.extend(frames), orig(frames))
    backend = FakeBackend()
    owner = Owner(backend, hub, Settings(streams={"color": True, "depth": True}), startup_s=1.5,
                  calibration_file=str(tmp_path / "imu.json"))
    owner.start()
    try:
        wait_for(lambda: owner.status()["state"] == "streaming", 10)
        assert not seen[-1].covered
        lock, _ = LockStore().place("cover", "camera", "admin", "meeting")
        wait_for(lambda: owner.status()["state"] == "covered", 3)
        assert not backend.running                                  # no capture at all
        n0 = len(seen)
        wait_for(lambda: len([f for f in seen[n0:] if f.stream == "color"]) >= 3, 3)
        new = seen[n0:]
        assert all(f.covered for f in new) and {f.stream for f in new} == {"color", "depth"}
        col = next(f for f in new if f.stream == "color")
        assert col.data[:2] == b"\xff\xd8" and (col.w, col.h) == (1280, 720) and col.header()["covered"]
        dep = next(f for f in new if f.stream == "depth")
        assert dep.data.dtype.name == "uint16" and dep.data.shape == (480, 848) and 500 < dep.data.mean() < 6000
        st = owner.status()
        assert st["covered"] and st["locks"][0]["id"] == lock.id
        assert st["imu"]["running"] and st["imu"]["samples"] > 0     # the IMU keeps running
        s0 = st["imu"]["samples"]
        wait_for(lambda: owner.status()["imu"]["samples"] > s0, 2)
        LockStore().remove(lock.id)
        wait_for(lambda: owner.status()["state"] == "streaming", 5)
        assert backend.running and not owner.status()["covered"]
        n1 = len(seen)
        wait_for(lambda: len(seen) > n1 + 2, 3)
        assert not any(f.covered for f in seen[n1 + 2:])
    finally:
        owner.close()


def test_3d_perception_passes_the_covered_flag(tmp_path):
    from robot_utils.nodes.common.client import NodeClient
    from robot_utils.nodes.orbbec.fake_backend import FakeBackend
    from robot_utils.nodes.orbbec.hub import Hub
    from robot_utils.nodes.orbbec.models import Settings
    from robot_utils.nodes.orbbec.owner import Owner
    from robot_utils.nodes.orbbec.server import create_app as camera_app
    from robot_utils.nodes.perception3d.node import Params, PerceptionNode
    from robot_utils.nodes.perception3d.server import create_app as p_app

    def serve(app):
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            port = s.getsockname()[1]
        srv = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning", timeout_graceful_shutdown=1))
        t = threading.Thread(target=srv.run, daemon=True)
        t.start()
        wait_for(lambda: srv.started, 10)
        return f"http://127.0.0.1:{port}", srv, t

    LockStore().place("cover", "camera", "admin", "test")
    hub = Hub()
    cam, *c_srv = serve(camera_app(Owner(FakeBackend(), hub, Settings(streams={"color": True, "depth": True}),
                                          startup_s=1.5, calibration_file=str(tmp_path / "imu.json")), hub))
    node = PerceptionNode(cam, None, None, Params(idle_s=0))
    url, *p_srv = serve(p_app(node))
    try:
        wait_for(lambda: httpx.get(cam + "/state").json()["state"] == "covered", 10)
        wait_for(lambda: httpx.get(url + "/state").json()["state"] == "running", 20)
        with NodeClient(url) as c:
            for h, _, _ in c.records("/stream"):
                if h["type"] == "fused":
                    break
        assert h["covered"] is True
        assert httpx.get(url + "/state").json()["covered"] is True
    finally:
        for srv, t in (p_srv, c_srv):
            srv.should_exit = True
            t.join(10)


# -- the landing page API and the admin CLI ------------------------------------------------------------

def test_landing_page_places_but_never_removes():
    from fastapi.testclient import TestClient

    from robot_utils.frontend.app import create_app
    from robot_utils.frontend.config import NodeEntry
    nodes = [NodeEntry("swerve", kind="swerve"), NodeEntry("camera", kind="camera"), NodeEntry("estimator")]
    with TestClient(create_app(nodes)) as c:
        g = c.get("/api/locks").json()
        assert set(g["targets"]["chock"]) == {"all", "swerve", "arm"} and "camera" in g["targets"]["cover"]
        assert c.post("/api/locks", json={"kind": "chock", "target": "estimator"}).status_code == 422
        r = c.post("/api/locks", json={"kind": "chock", "target": "all", "reason": "agent went wild"},
                   headers={"X-Forwarded-For": "100.65.75.106"})
        assert r.status_code == 201 and r.json()["lock"]["source"] == "web 100.65.75.106"
        lid = r.json()["lock"]["id"]
        again = c.post("/api/locks", json={"kind": "chock", "target": "all"}, headers={"X-Forwarded-For": "1.1.1.1"})
        assert again.status_code == 200 and not again.json()["created"]           # reused
        cover = c.post("/api/locks", json={"kind": "cover", "target": "camera"}, headers={"X-Forwarded-For": "100.65.75.106"})
        assert cover.status_code == 201                            # a different lock right after: allowed
        LockStore().remove(cover.json()["lock"]["id"])             # (admin, over SSH)
        spam = c.post("/api/locks", json={"kind": "cover", "target": "camera"}, headers={"X-Forwarded-For": "100.65.75.106"})
        assert spam.status_code == 429                             # the same lock again within 30 s
        d = c.delete(f"/api/locks/{lid}")
        assert d.status_code == 403 and "ssh" in d.json()["error"].lower()
        assert [l["id"] for l in c.get("/api/locks").json()["locks"]] == [lid]


def test_admin_cli(capsys):
    from robot_utils.admin import locks as cli
    assert cli.main(["chock", "swerve", "--reason", "battery", "--as", "avery"]) == 0
    assert cli.main(["cover", "camera"]) == 0
    assert cli.main(["status"]) == 0
    out = capsys.readouterr().out
    assert "2 lock(s)" in out and "battery" in out and "avery" in out
    lid = LockStore().matching("chock", {"swerve"})[0].id
    assert cli.main(["remove", lid]) == 0 and "1 lock(s) left" in capsys.readouterr().out
    assert cli.main(["remove", "nope"]) == 2
    assert cli.main(["remove", "--all"]) == 0 and not LockStore().active(force=True)
