"""3d_perception node end to end: fake camera node (+ stub estimator and description) and real servers."""
import io
import time

import httpx
import numpy as np
import pytest

from helpers import serve, stop, wait_for
from perception3d_stubs import camera, stub_description, stub_estimator  # noqa: F401  (camera: a fixture)
from robot_utils.nodes.common.client import NodeClient
from robot_utils.nodes.perception3d.node import Params, PerceptionNode
from robot_utils.nodes.perception3d.server import create_app


def test_rotation_only_with_the_fake_camera(camera):
    node = PerceptionNode(camera, None, None, Params(idle_s=0))
    url, srv, t = serve(create_app(node))
    try:
        st = wait_for(lambda: (s := httpx.get(url + "/state").json())["state"] == "running" and s, 20)
        # the fake IMU's records are 'initializing' for ~1.3 s: then the IMU compensates
        st = wait_for(lambda: (s := httpx.get(url + "/state").json())["mode"] == "rotation" and s, 10)
        assert st["inputs"]["frames"]["connected"] and st["inputs"]["imu"]["connected"] and "pose" not in st["inputs"]
        assert st["timing"]["compute_ms"]["median"] > 0 and st["rate_hz"] > 5
        with NodeClient(url) as c:
            for h, p, _ in c.records("/stream"):
                if h["type"] == "fused":
                    break
            cal = c.get("/calibration")
            assert cal["ready"] and cal["color"]["width"] == 1280 and cal["depth_grid"]["width"] == 640
            assert h["mode"] == "rotation" and len(h["motion"]) == 4 and h["t_depth"] is not None
            assert len(p) == h["color"]["len"] + h["depth"]["len"] and p[:2] == b"\xff\xd8"   # JPEG first
            dep = np.frombuffer(p, "<u2", offset=h["depth"]["offset"]).reshape(h["depth"]["height"], h["depth"]["width"])
            assert dep.shape == (360, 640) and 0.5 < h["depth"]["valid"] <= 1 and dep.max() < 4000
            # point query: depth along the clicked pixel's ray, colour camera frame
            q = c.get("/point", u=640, v=500)
            assert q["source"] in ("pixel", "window_median") and 400 < q["depth_mm"] < 4000
            assert q["point"][2] == pytest.approx(q["depth_mm"] / 1000) and q["point_base"] is None
            assert c.get("/point", u=0.5, v=0.7, norm=True)["depth_mm"] is not None
            # single frame, images
            r = c.http.get("/frame")
            assert r.status_code == 200 and r.content[4:200].find(b'"type":"fused"') >= 0
            assert c.http.get("/color.jpg").content[:2] == b"\xff\xd8"
            png = c.http.get("/depth.png").content
            assert png[:8] == b"\x89PNG\r\n\x1a\n"
            # settings: compensation off -> mode none; grid size follows scale
            c.post("/settings", {"compensation": "none", "scale": 0.25})
            st = wait_for(lambda: (s := c.get("/state"))["mode"] == "none" and s, 5)
            assert c.get("/calibration")["depth_grid"]["width"] == 320
            meta = c.get("/meta")
            assert meta["kind"] == "3d_perception" and meta["actuated"] is False
            page = c.http.get("/").text
            assert 'src="common/ui.js"' in page and 'RU.stream("stream"' in page
            assert 'api("/' not in page and 'stream("/' not in page and 'sse("/' not in page   # relative URLs only
    finally:
        stop((srv, t))


def test_full_compensation_with_estimator_and_description(camera):
    est_url, est_srv, est_t = serve(stub_estimator())
    desc_url, desc_srv, desc_t = serve(stub_description())
    node = PerceptionNode(camera, est_url, desc_url, Params(idle_s=0))
    url, srv, t = serve(create_app(node))
    try:
        st = wait_for(lambda: (s := httpx.get(url + "/state").json())["mode"] == "full" and s, 20)
        assert st["sources"]["description"]["calibrated"] and st["inputs"]["pose"]["connected"]
        latest = st["latest"]
        # the stub base moves 0.2 m/s forward: translation ≈ 0.2 m/s x |dt|
        if latest["dt_color_depth"]:
            assert latest["translation_mm"] == pytest.approx(200 * abs(latest["dt_color_depth"]), abs=2)
        q = httpx.get(url + "/point", params={"u": 640, "v": 500}).json()
        assert q["point_base"] is not None and len(q["point_base"]) == 3
        cal = httpx.get(url + "/calibration").json()
        assert cal["base_to_color"] is not None and cal["base_calibration"]["calibrated"]
    finally:
        stop((srv, t), (est_srv, est_t), (desc_srv, desc_t))


def test_uncalibrated_description_falls_back_to_rotation(camera):
    est_url, est_srv, est_t = serve(stub_estimator())
    desc_url, desc_srv, desc_t = serve(stub_description(calibrated=False))
    node = PerceptionNode(camera, est_url, desc_url, Params(idle_s=0))
    url, srv, t = serve(create_app(node))
    try:
        st = wait_for(lambda: (s := httpx.get(url + "/state").json())["mode"] == "rotation" and s, 20)
        assert st["sources"]["description"]["ok"] and not st["sources"]["description"]["calibrated"]
        httpx.post(url + "/settings", json={"allow_nominal": True})
        wait_for(lambda: httpx.get(url + "/state").json()["mode"] == "full", 5)
    finally:
        stop((srv, t), (est_srv, est_t), (desc_srv, desc_t))


def test_idle_and_wake(camera):
    node = PerceptionNode(camera, None, None, Params(idle_s=0.5))
    url, srv, t = serve(create_app(node))
    try:
        wait_for(lambda: httpx.get(url + "/state").json()["state"] == "idle", 10)
        time.sleep(1.0)                                     # the last frame is now older than max_age
        st = httpx.get(url + "/state").json()
        assert st["state"] == "idle" and st["inputs"]["frames"]["paused"] and st["inputs"]["imu"]["paused"]
        seq = st["seq"]
        q = httpx.get(url + "/point", params={"u": 640, "v": 500}, timeout=10).json()   # wakes it, waits for a frame
        assert q["seq"] > seq and q["age_s"] < 0.6 and q["depth_mm"] is not None and q["mode"] == "rotation"
    finally:
        stop((srv, t))
