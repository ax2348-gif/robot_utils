"""Estimator node end to end: simulated swerve node + fake camera node + estimator, real servers."""
import socket
import threading
import time

import httpx
import pytest
import uvicorn

from robot_utils.nodes.common.client import NodeClient
from robot_utils.nodes.estimator.fusion import FIELDS
from robot_utils.nodes.estimator.node import EstimatorNode
from robot_utils.nodes.estimator.server import create_app as estimator_app


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.02)
    raise AssertionError("condition not met in time")


def serve(app):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning",
                                           timeout_graceful_shutdown=1))
    t = threading.Thread(target=server.run, daemon=True)
    t.start()
    wait_for(lambda: server.started, 10)
    return f"http://127.0.0.1:{port}", server, t


@pytest.fixture
def nodes(tmp_path):
    from robot_utils.nodes.orbbec.fake_backend import FakeBackend
    from robot_utils.nodes.orbbec.hub import Hub
    from robot_utils.nodes.orbbec.models import Settings
    from robot_utils.nodes.orbbec.owner import Owner
    from robot_utils.nodes.orbbec.server import create_app as camera_app
    from robot_utils.nodes.swerve.node import SwerveNode
    from robot_utils.nodes.swerve.server import create_app as swerve_app
    from robot_utils.nodes.swerve.sim import start_sim, stop_sim

    sims, iface = start_sim()
    sw = SwerveNode(iface)
    sw_url, sw_srv, sw_t = serve(swerve_app(sw))
    wait_for(lambda: sw.status == "running", 10)
    hub = Hub()
    cam_url, cam_srv, cam_t = serve(camera_app(Owner(FakeBackend(), hub, Settings(streams={}), startup_s=1.5,
                                                     calibration_file=str(tmp_path / "imu.json")), hub))
    est = EstimatorNode(sw_url, cam_url)
    est_url, est_srv, est_t = serve(estimator_app(est))
    yield {"est": est_url, "swerve": sw_url, "camera": cam_url, "node": est}
    for srv, t in ((est_srv, est_t), (cam_srv, cam_t), (sw_srv, sw_t)):
        srv.should_exit = True
        t.join(10)
    stop_sim(sims, iface)


def test_fuses_and_tracks_a_drive(nodes):
    st = wait_for(lambda: (s := httpx.get(nodes["est"] + "/state").json())["state"] == "running" and s, 15)
    wait_for(lambda: httpx.get(nodes["est"] + "/state").json()["mode"].startswith("fused"), 10)
    # steady state: the camera's IMU startup calibration (no heading yet) has left the 5 s window
    wait_for(lambda: (httpx.get(nodes["est"] + "/state").json()["imu_coverage"] or 0) > 0.95, 15)
    httpx.post(nodes["est"] + "/reset", json={})
    from robot_utils.nodes.swerve.client import SwerveClient
    with SwerveClient(nodes["swerve"]) as base, base.control("test"):
        base.drive_for((0.4, 0.0, 0.0), seconds=1.5)
    time.sleep(0.5)
    st = httpx.get(nodes["est"] + "/state").json()
    p = st["pose"]
    assert 0.3 < p["x"] < 0.7 and abs(p["y"]) < 0.02, p        # ~0.6 m minus the drive's ramp
    assert st["imu_coverage"] > 0.95 and st["rate_hz"] > 80
    assert st["inputs"]["swerve"]["connected"] and st["inputs"]["imu"]["connected"]


def test_stream_meta_page_and_reset(nodes):
    wait_for(lambda: httpx.get(nodes["est"] + "/state").json()["state"] == "running", 15)
    with NodeClient(nodes["est"]) as c:
        h, rows = next(c.rows("/stream"))
        assert list(rows.dtype.names) == FIELDS and h["kind"] == "pose"
        meta = c.get("/meta")
        assert meta["name"] == "estimator" and meta["actuated"] is False and meta["footprint"]["half_length"] > 0
        page = c.http.get("/").text
        assert 'src="common/ui.js"' in page and 'RU.rows("stream"' in page
        assert 'api("/' not in page and 'rows("/' not in page and 'sse("/' not in page   # relative URLs only
        c.post("/reset", {"x": 1.0, "y": 2.0, "theta": 0.5})
        p = wait_for(lambda: (q := c.get("/state")["pose"]) and abs(q["x"] - 1.0) < 1e-6 and q, 3)
        assert p["y"] == pytest.approx(2.0) and p["theta"] == pytest.approx(0.5, abs=1e-3)


def test_runs_without_the_camera(nodes):
    """Odometry only when the IMU upstream is missing; reported as such."""
    est = EstimatorNode(nodes["swerve"], None)
    url, srv, t = serve(estimator_app(est))
    try:
        st = wait_for(lambda: (s := httpx.get(url + "/state").json())["state"] == "running" and s, 10)
        assert st["mode"].startswith("odometry only") and "imu" not in st["inputs"]
    finally:
        srv.should_exit = True
        t.join(10)
