"""Pose servo end to end: simulated swerve node + estimator (odometry only: the fake camera's IMU
never rotates) + pose servo, real servers. Constraints are re-checked on the twists the servo
reports as accepted by the swerve node."""
import math
import socket
import threading
import time

import httpx
import pytest
import uvicorn

from robot_utils.nodes.common.client import NodeClient
from robot_utils.nodes.estimator.node import EstimatorNode
from robot_utils.nodes.estimator.server import create_app as estimator_app
from robot_utils.nodes.pose_servo.node import FIELDS, PERIOD, PoseServoNode
from robot_utils.nodes.pose_servo.planner import Caps, Cmd, check_step, within
from robot_utils.nodes.pose_servo.server import create_app as servo_app

NO_IMU = {"require_imu": False}


def wait_for(predicate, timeout=5.0, interval=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(interval)
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


def stop(srv, t):
    srv.should_exit = True
    t.join(10)


class Recorder:
    """Collects the pose servo's stream rows in a thread."""

    def __init__(self, url):
        self.rows, self.stop = [], threading.Event()
        self.t = threading.Thread(target=self._run, args=(url,), daemon=True)
        self.t.start()

    def _run(self, url):
        try:
            with NodeClient(url) as c:
                for _, rows in c.rows("/stream"):
                    for r in rows:
                        self.rows.append({n: float(r[n]) for n in FIELDS})
                    if self.stop.is_set():
                        return
        except Exception:        # the server went away at teardown
            pass


@pytest.fixture
def stack():
    from robot_utils.nodes.swerve.node import SwerveNode
    from robot_utils.nodes.swerve.server import create_app as swerve_app
    from robot_utils.nodes.swerve.sim import start_sim, stop_sim

    sims, iface = start_sim()
    sw = SwerveNode(iface)
    sw_url, sw_srv, sw_t = serve(swerve_app(sw))
    wait_for(lambda: sw.status == "running", 10)
    est = EstimatorNode(sw_url, None)
    est_url, est_srv, est_t = serve(estimator_app(est))
    node = PoseServoNode(sw_url, est_url)
    url, srv, t = serve(servo_app(node))
    wait_for(lambda: httpx.get(url + "/state").json()["estimator"]["health"] in
             ("ok", "estimator heading is not IMU-fused (require_imu)"), 15)
    out = {"url": url, "swerve": sw_url, "est": est_url, "node": node, "est_server": (est_srv, est_t)}
    yield out
    stop(srv, t)
    if not out.get("est_stopped"):
        stop(est_srv, est_t)
    stop(sw_srv, sw_t)
    stop_sim(sims, iface)


def post(url, path, body=None):
    return httpx.post(url + path, json=body if body is not None else {}, timeout=5)


def state(stack):
    return httpx.get(stack["url"] + "/state").json()


def vx(stack):
    v = (state(stack)["latest"] or {}).get("vx")
    return 0.0 if v is None else v


def wait_result(stack, timeout=30):
    return wait_for(lambda: (g := state(stack)["goal"]) and g["result"] is not None and g, timeout, 0.05)


def check_sent(rows, caps: Caps):
    """Every accepted twist in order: the same checks as the planner's limiter, recomputed."""
    prev, n_sent, bad = Cmd(), 0, []
    for r in rows:
        if r["sent"] != 1.0:
            if r["status"] in (0.0, 1.0):
                prev = Cmd()            # idle / disabled: the swerve holds still
            continue
        c = Cmd(r["vx"], r["vy"], r["omega"], r["ax"], r["ay"], r["alpha"])
        v = within(check_step(prev, c, PERIOD), caps)
        if v:
            bad.append((r["t"], v))
        prev, n_sent = c, n_sent + 1
    return n_sent, bad


def test_enable_goal_reached_and_constraints_hold(stack):
    url = stack["url"]
    r = post(url, "/enable", {"client": "test", "config": NO_IMU})
    assert r.status_code == 200, r.text
    sw = httpx.get(stack["swerve"] + "/state").json()
    assert sw["lease"]["holder"].startswith("pose_servo") and sw["enabled"]
    assert sw["limits"]["soft"] == {"max_speed": 0.3, "max_omega": 0.8}
    rec = Recorder(url)
    p0 = httpx.get(stack["est"] + "/pose").json()
    g = post(url, "/goal", {"x": 0.4, "y": 0.2, "theta": 0.6})        # local by default
    assert g.status_code == 200, g.text
    res = wait_result(stack)
    assert res["result"] == "reached", res
    time.sleep(0.3)
    p = httpx.get(stack["est"] + "/pose").json()
    c, s = math.cos(p0["theta"]), math.sin(p0["theta"])
    gx, gy = p0["x"] + c * 0.4 - s * 0.2, p0["y"] + s * 0.4 + c * 0.2
    assert math.hypot(p["x"] - gx, p["y"] - gy) < 0.03, (p, gx, gy)
    assert abs(p["theta"] - (p0["theta"] + 0.6)) < 0.05
    rec.stop.set()
    n, bad = check_sent(rec.rows, Caps())
    assert n > 50 and not bad, bad[:5]
    st = state(stack)
    assert st["status"] == "idle" and st["violations"] == 0
    assert st["loop_hz"] > 40


def test_requires_imu_by_default_and_rejects_bad_goals(stack):
    url = stack["url"]
    assert post(url, "/goal", {"x": 0.1}).status_code == 409                   # not enabled
    assert post(url, "/enable", {"client": "t"}).status_code == 200
    r = post(url, "/goal", {"x": 0.1})
    assert r.status_code == 503 and "IMU" in r.json()["detail"]
    assert post(url, "/enable", {}).status_code == 409                           # already enabled
    assert post(url, "/disable").status_code == 200
    assert post(url, "/enable", {"config": {"require_imu": False, "max_distance": 1.0}}).status_code == 200
    r = post(url, "/goal", {"x": 1.2})
    assert r.status_code == 422 and "max_distance" in r.json()["detail"]
    assert post(url, "/goal", {"theta": 3.5}).status_code == 422                 # local, literal > max_rotation (pi)
    r = post(url, "/goal", {"x": 0.1, "speed": 0.5})
    assert r.status_code == 422 and "max_speed" in r.json()["detail"]
    assert post(url, "/goal", {"x": 0.1, "bogus": 1}).status_code == 422
    assert post(url, "/config", {"max_speed": 2.5}).status_code == 422
    # global: shortest turn (theta = current + 2 pi - 0.1 is a -0.1 rad turn)
    th = state(stack)["latest"]["theta"]
    r = post(url, "/goal", {"frame": "global", "x": state(stack)["latest"]["x"], "y": state(stack)["latest"]["y"],
                             "theta": th + 2 * math.pi - 0.1})
    assert r.status_code == 200 and r.json()["rotation"] == pytest.approx(-0.1, abs=0.01)
    assert wait_result(stack)["result"] == "reached"


def test_goal_replaced_mid_motion_and_stop(stack):
    url = stack["url"]
    post(url, "/enable", {"client": "t", "config": NO_IMU})
    rec = Recorder(url)
    post(url, "/goal", {"x": 0.8})
    wait_for(lambda: vx(stack) > 0.2, 10)
    r = post(url, "/goal", {"x": -0.3, "y": 0.3, "theta": -0.5})                # behind: brakes, reverses
    assert r.status_code == 200
    first = r.json()["goal_id"]
    wait_for(lambda: vx(stack) < 0, 10)
    post(url, "/goal", {"x": 0.8})
    wait_for(lambda: vx(stack) > 0.15, 10)
    assert post(url, "/stop").status_code == 200
    res = wait_result(stack)
    assert res["result"] == "stopped" and res["goal_id"] == first + 1
    wait_for(lambda: state(stack)["status"] == "idle", 5)
    rec.stop.set()
    n, bad = check_sent(rec.rows, Caps())
    assert n > 50 and not bad, bad[:5]
    assert state(stack)["enabled"]


def test_estimator_reset_aborts_and_brakes(stack):
    url = stack["url"]
    post(url, "/enable", {"client": "t", "config": NO_IMU})
    rec = Recorder(url)
    post(url, "/goal", {"x": 1.0})
    wait_for(lambda: vx(stack) > 0.2, 10)
    post(stack["est"], "/reset", {})
    res = wait_result(stack, 5)
    assert res["result"] == "aborted" and "reset" in res["reason"], res
    wait_for(lambda: state(stack)["status"] == "idle", 5)
    rec.stop.set()
    n, bad = check_sent(rec.rows, Caps())
    assert not bad, bad[:5]
    braking = [r for r in rec.rows if r["status"] == 4.0]
    assert braking and braking[-1]["vx"] < braking[0]["vx"]                     # braked, did not jump to 0
    assert state(stack)["enabled"]


def test_estimator_lost_aborts(stack):
    url = stack["url"]
    post(url, "/enable", {"client": "t", "config": NO_IMU})
    post(url, "/goal", {"x": 1.0})
    wait_for(lambda: vx(stack) > 0.1, 10)
    stop(*stack["est_server"])
    stack["est_stopped"] = True
    res = wait_result(stack, 5)
    assert res["result"] == "aborted" and "stale" in res["reason"], res
    wait_for(lambda: state(stack)["status"] == "idle", 5)
    assert post(url, "/goal", {"x": 0.1}).status_code == 503


def test_disable_is_immediate_and_releases(stack):
    url = stack["url"]
    post(url, "/enable", {"client": "t", "config": NO_IMU})
    post(url, "/goal", {"x": 1.0})
    wait_for(lambda: vx(stack) > 0.2, 10)
    assert post(url, "/disable").status_code == 200
    st = state(stack)
    assert st["status"] == "disabled" and st["goal"]["result"] == "aborted"
    sw = wait_for(lambda: (s := httpx.get(stack["swerve"] + "/state").json()) and not s["lease"]["held"] and s, 3)
    assert not sw["enabled"]
    # someone else can take the swerve now
    assert httpx.post(stack["swerve"] + "/lease", json={"client": "other"}).status_code == 200


def test_swerve_lease_taken_away_disables(stack):
    url = stack["url"]
    post(url, "/enable", {"client": "t", "config": NO_IMU})
    time.sleep(0.4)
    httpx.post(stack["swerve"] + "/estop", json={"reason": "test", "source": "pytest"})
    wait_for(lambda: state(stack)["status"] == "disabled", 3)
    assert "e-stop" in state(stack)["disabled_reason"]


def test_estop_forwards(stack):
    url = stack["url"]
    post(url, "/enable", {"client": "t", "config": NO_IMU})
    r = post(url, "/estop", {"reason": "button"})
    assert r.status_code == 200 and r.json()["forwarded"]
    sw = httpx.get(stack["swerve"] + "/state").json()
    assert sw["estop"]["latched"] and "pose_servo" in sw["estop"]["reason"]
    assert state(stack)["status"] == "disabled"
    assert post(url, "/enable", {}).status_code == 409                           # swerve e-stop latched


def test_page_and_meta(stack):
    with NodeClient(stack["url"]) as c:
        meta = c.get("/meta")
        assert meta["name"] == "pose_servo" and meta["actuated"] is True
        page = c.http.get("/").text
        assert 'src="common/ui.js"' in page and 'api("/' not in page and 'rows("/' not in page
        cfg = c.get("/config")
        assert cfg["config"]["max_distance"] == 3.0 and cfg["hard"]["steer_rate"] == pytest.approx(4 * math.pi)


def test_static_friction_options(stack):
    url = stack["url"]
    r = post(url, "/enable", {"client": "t", "config": {"require_imu": False, "min_wheel_speed": 0.1, "stall_s": 5}})
    assert r.status_code == 200 and r.json()["config"]["min_wheel_speed"] == 0.1
    assert post(url, "/config", {"min_wheel_speed": 0.5}).status_code == 422
    r = post(url, "/goal", {"theta": 0.3, "min_wheel_speed": 0.0, "stall_s": 0})
    assert r.status_code == 200 and r.json()["motion"]["min_wheel_speed"] == 0.0
    assert wait_result(stack)["result"] == "reached"
    with NodeClient(stack["url"]) as c:
        assert "boost" in c.get("/fields")["fields"]
