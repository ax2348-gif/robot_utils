"""Description node end to end: fake camera node + simulated swerve node + description node (real servers)."""
import socket
import threading
import time

import httpx
import pytest
import uvicorn
import yaml

from robot_utils.nodes.description.node import DescriptionNode, compose_camera
from robot_utils.nodes.description.server import create_app


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        v = predicate()
        if v:
            return v
        time.sleep(0.05)
    raise AssertionError("condition not met in time")


def serve(app):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning", timeout_graceful_shutdown=1))
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
    cam_url, cam_srv, cam_t = serve(camera_app(Owner(FakeBackend(), hub, Settings(), startup_s=1.5,
                                                     calibration_file=str(tmp_path / "imu.json")), hub))
    data = tmp_path / "desc"
    data.mkdir()
    (data / "description.yaml").write_text(yaml.safe_dump({"cameras": {"camera": {"url": cam_url}}}))
    node = DescriptionNode(str(data), sw_url)
    url, srv, t = serve(create_app(node))
    wait_for(lambda: httpx.get(url + "/description").json()["cameras"]["camera"]["reachable"], 15)
    yield {"url": url, "swerve": sw_url, "camera": cam_url, "node": node, "data": data}
    for s, th in ((srv, t), (cam_srv, cam_t), (sw_srv, sw_t)):
        s.should_exit = True
        th.join(10)
    stop_sim(sims, iface)


def job_done(url, jid, timeout=30):
    return wait_for(lambda: (j := httpx.get(f"{url}/jobs/{jid}").json())["state"] != "running" and j, timeout)


def test_meta_page_description_transform(nodes):
    u = nodes["url"]
    meta = httpx.get(u + "/meta").json()
    assert meta["name"] == "description" and meta["actuated"] is True and meta["endpoints"]["estop"] == "estop"
    page = httpx.get(u + "/").text
    assert 'src="common/ui.js"' in page and 'api("/' not in page and 'sse("/' not in page
    d = httpx.get(u + "/description").json()
    assert d["cameras"]["camera"]["serial"] == "FAKE0001"
    assert {"base", "camera/depth_optical", "camera/color_optical"} <= set(d["frames"])
    t = httpx.get(u + "/transform", params={"from": "base", "to": "camera/color_optical"}).json()
    assert len(t["xyz"]) == 3 and "rpy_deg" in t
    r = httpx.post(u + "/cameras/camera/nominal", json={"xyz": [0.1, 0.0, 0.4], "rpy": [-1.5707963, 0, -1.5707963]})
    assert r.status_code == 200 and r.json()["pose_in_base"]["xyz"] == pytest.approx([0.1, 0.0, 0.4])
    assert httpx.get(u + "/cameras/camera/depth.jpg").headers["content-type"] == "image/jpeg"


def test_yaw_refused_without_confirm_then_runs_on_the_sim(nodes):
    u = nodes["url"]
    r = httpx.post(u + "/calibrate/yaw", json={"amplitude": 0.05})
    assert r.status_code == 400 and "confirm" in r.text
    r = httpx.post(u + "/calibrate/yaw", json={"amplitude": 0.04, "frequency": 2.0, "cycles": 2, "confirm": True})
    assert r.status_code == 200, r.text
    j = job_done(u, r.json()["id"], 40)
    assert j["state"] == "done", j
    cals = httpx.get(u + "/calibrations", params={"type": "yaw"}).json()
    # the fake IMU does not feel the simulated motion: saved, but refused by the quality gate
    assert len(cals) == 1 and not cals[0]["active"] and cals[0]["has_raw"] and not cals[0]["quality"]["ok"]
    assert j["result"]["activated"] is False and j["result"]["quality"]["problems"]
    st = httpx.get(nodes["swerve"] + "/state").json()
    assert not st["lease"]["held"] and not st["enabled"] and not st["estop"]["latched"]
    # re-solve from the stored raw data (nothing moves): a new record, linked to the old one
    c = cals[0]
    r = httpx.post(u + f"/calibrations/yaw/{c['serial']}/{c['id']}/resolve", json={"accel_lowpass_hz": 3.0, "activate": False})
    assert r.status_code == 200, r.text
    assert r.json()["accel_lowpass_hz"] == 3.0 and r.json()["id"] != c["id"] and "re-solved" in r.json()["summary"]
    assert httpx.post(u + f"/calibrations/yaw/{c['serial']}/nope/resolve").status_code == 404


def test_estop_aborts_a_running_yaw_job(nodes):
    u = nodes["url"]
    r = httpx.post(u + "/calibrate/yaw", json={"amplitude": 0.04, "frequency": 1.0, "cycles": 10, "confirm": True})
    jid = r.json()["id"]
    wait_for(lambda: httpx.get(nodes["swerve"] + "/state").json()["lease"]["held"], 10)
    assert httpx.post(u + "/estop", json={"reason": "test"}).json()["latched"]
    j = job_done(u, jid, 10)
    assert j["state"] == "aborted"
    sw = httpx.get(nodes["swerve"] + "/state").json()
    assert sw["estop"]["latched"] and not sw["lease"]["held"]
    assert httpx.post(u + "/calibrate/yaw", json={"amplitude": 0.04, "confirm": True}).status_code == 409   # latched
    httpx.post(u + "/estop/clear")


def test_ground_job_and_fifo(nodes, monkeypatch):
    u, node = nodes["url"], nodes["node"]
    r = httpx.post(u + "/calibrate/ground", json={"frames": 2, "polygons": [[[0, 0.5], [1, 0.5], [1, 1], [0, 1]]]})
    j = job_done(u, r.json()["id"])
    assert j["state"] in ("done", "failed"), j       # the fake camera's depth is a ramp, not a floor
    for i in range(7):                                 # FIFO: 5 kept + the active one
        node.store.save("ground", "FAKE0001", {"summary": f"s{i}", "result": {"normal": [0, -1, 0], "height": 0.5}},
                        {}, activate=(i == 0))
        time.sleep(1.05)
    cals = node.store.list("ground", "FAKE0001")
    active = [c for c in cals if c["active"]]
    assert len(active) == 1 and len(cals) <= 6 and any(c["summary"] == "s0" for c in cals)


def test_compose_uses_ground_and_keeps_nominal_heading():
    import numpy as np
    nominal = {"xyz": [0.1, 0.2, 0.5], "rpy": [-1.5707963, 0.0, -1.5707963]}
    T0, _ = compose_camera(nominal, None, None)
    ground = {"id": "g", "result": {"normal": (T0[:3, :3].T @ np.array([0, 0, 1.0])).tolist(), "height": 0.61}}
    T, used = compose_camera(nominal, ground, None)
    assert T[:3, 3] == pytest.approx([0.1, 0.2, 0.61]) and np.allclose(T[:3, :3], T0[:3, :3], atol=1e-6)
    assert used["ground"] == "g"
