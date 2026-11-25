"""The sim world (nodes/sim/world.py, render.py, protocol.py, server.py) without any node attached: the base
hybrid, the kinematic arm, time, scene, privileged controls, rendering, the IPC protocol, the HTTP API.
docs/sim.md."""
import math
import socket
import struct
import time

import numpy as np
import pytest

from sim_helpers import gl_available, make_world, needs_gl

from robot_utils.nodes.sim.protocol import IpcClient, IpcError, IpcServer, recv_msg, send_msg
from robot_utils.nodes.sim.world import SimError


@pytest.fixture(scope="module")
def world():
    w = make_world()
    yield w
    w.close()


@pytest.fixture
def w(world):
    world.reset()
    world.load_scene([])
    return world


def drive(w, seconds, angle, speed, on=True):
    for k in range(int(round(seconds / w.m.opt.timestep))):
        if k % 10 == 0:
            w.swerve_op("modules", {"steer": {m: [on, angle] for m in w.module_names},
                                    "drive": {m: [on, speed] for m in w.module_names}})
        w.step(1)


def test_base_rests_on_four_loaded_wheels(w):
    b = w.base_state()
    assert abs(b["xyz"][2]) < 0.003 and max(abs(v) for v in b["rpy"][:2]) < 0.01
    import mujoco
    forces = {}
    for i in range(w.d.ncon):
        c = w.d.contact[i]
        bodies = {w.m.body(w.m.geom_bodyid[g]).name for g in (c.geom1, c.geom2)}
        f = np.zeros(6)
        mujoco.mj_contactForce(w.m, w.d, i, f)
        for n in bodies:
            if n.endswith("_wheel"):
                forces[n] = forces.get(n, 0.0) + f[0]
    assert len(forces) == 4
    total = sum(forces.values())
    assert math.isclose(total, float(w.m.body_subtreemass[w.base_body]) * 9.81, rel_tol=0.03)
    assert min(forces.values()) > 0.1 * total            # every wheel carries a share


def test_base_drives_forward_sideways_and_turns(w):
    drive(w, 2.0, 0.0, 0.3)
    b = w.base_state()
    assert b["twist"][0] == pytest.approx(0.3, abs=0.01) and abs(b["twist"][1]) < 0.01
    assert 0.5 < b["xyz"][0] < 0.62                      # the SPARK ramp (~10 m/s^2), then 0.3 m/s
    drive(w, 1.5, 0.0, 0.0)
    assert abs(w.base_state()["twist"][0]) < 0.01
    x0 = w.base_state()["xyz"]
    drive(w, 2.0, math.pi / 2, 0.2)                      # wheels turn 90 deg at the steer rate cap while driving
    b = w.base_state()
    assert b["xyz"][1] - x0[1] > 0.2 and abs(b["xyz"][0] - x0[0]) < 0.05
    # turning in place: each wheel tangential
    for k in range(3000):
        if k % 10 == 0:
            w.swerve_op("modules", {"steer": {m.name: [True, math.atan2(m.x, -m.y)] for m in _modules()},
                                    "drive": {m.name: [True, 0.15] for m in _modules()}})
        w.step(1)
    omega = w.base_state()["twist"][2]
    assert omega == pytest.approx(0.15 / math.hypot(0.165608, 0.165608), rel=0.05)


def _modules():
    from robot_utils.nodes.swerve.constants import MODULES
    return MODULES


def test_steer_rate_wheel_speed_and_accel_are_the_robots(w):
    from robot_utils.nodes.sim.limits import swerve_limits
    L = swerve_limits()
    w.swerve_op("modules", {"steer": {m: [True, 1.0] for m in w.module_names}, "drive": {}})
    w.step(50)
    st = w.module_states()["modules"]["FL"]
    assert st["steer"] <= L["steer_rate_max"] * 0.05 + 0.01          # ~7.4 rad/s: STEER_MAX_OUTPUT x free rate
    drive(w, 0.1, 0.0, 3.0)
    assert w.module_states()["modules"]["FL"]["speed"] <= L["wheel_accel_max"] * 0.1 + 0.05   # the SPARK ramp
    drive(w, 1.5, 0.0, 3.0)                                            # 3 m/s asked: the duty cap's ~1.87 m/s
    assert w.module_states()["modules"]["FL"]["speed"] == pytest.approx(L["wheel_speed_max"], abs=0.05)


def test_sim_limits_are_the_robot_configs(w, tmp_path):
    """Every actuator limit of the world comes from the robot codebase (swerve/constants.py as setup_sparks.py
    configures the SPARKs, arm/config.py as the arm node loads it), computed here independently."""
    from robot_utils.nodes.arm import config as C
    from robot_utils.nodes.sim.limits import robot_limits
    from robot_utils.nodes.sim.world import SimError, World, load_config
    from robot_utils.nodes.swerve import constants as K
    from robot_utils.nodes.swerve import setup_sparks
    L = w.limits
    assert all(np.all(np.asarray(L[k]) == np.asarray(v)) for k, v in robot_limits().items()) and set(L) == set(robot_limits())
    assert L["wheel_speed_max"] == pytest.approx(K.DRIVE_MAX_OUTPUT * K.DRIVE_FREE_SPEED)          # ~1.87 m/s
    assert L["wheel_accel_max"] == pytest.approx(K.DRIVE_FREE_SPEED / K.DRIVE_RAMP)
    assert L["steer_rate_max"] == pytest.approx(K.STEER_MAX_OUTPUT * K.NEO550_FREE_RPM / 60 * 2 * math.pi
                                                / K.STEER_REDUCTION)
    assert 1.1 < L["steer_rate_max"] / (2 * math.pi) < 1.3                    # "about 1.2 wheel rotations/s"
    assert L["wheel_torque_max"] == pytest.approx(
        min(K.DRIVE_MAX_OUTPUT * K.VORTEX_STALL_TORQUE, K.VORTEX_STALL_TORQUE / K.VORTEX_STALL_CURRENT
            * K.DRIVE_CURRENT_LIMIT) * K.DRIVE_REDUCTION)
    assert L["steer_torque_max"] == pytest.approx(
        min(K.STEER_MAX_OUTPUT * K.NEO550_STALL_TORQUE, K.NEO550_STALL_TORQUE / K.NEO550_STALL_CURRENT
            * K.STEER_CURRENT_LIMIT) * K.STEER_REDUCTION)
    assert np.allclose(L["joint_speed_max"], C.load_settings().hard.joint_vel)
    assert L["finger_force_max"] == C.HARD_MAX_FORCE
    # the SPARK configuration these assume is the one setup_sparks applies
    src = open(setup_sparks.__file__).read()
    assert "output_range(-c.DRIVE_MAX_OUTPUT, c.DRIVE_MAX_OUTPUT)" in src and "closed_loop_ramp_rate(c.DRIVE_RAMP)" in src
    assert "output_range(-c.STEER_MAX_OUTPUT, c.STEER_MAX_OUTPUT)" in src
    # the actuators carry those torque caps
    m = w.m
    assert m.actuator_forcerange[m.actuator("wheel_FL").id][1] == pytest.approx(L["wheel_torque_max"])
    assert m.actuator_forcerange[m.actuator("steer_FL").id][1] == pytest.approx(L["steer_torque_max"])
    # an arm settings file's joint_vel_max (as the arm node loads it) reaches the world
    from robot_utils.nodes.sim.limits import arm_limits
    f = tmp_path / "arm.yaml"
    f.write_text("joint_vel_max: [1, 1, 1, 3, 3, 3]\n")
    assert arm_limits(C.load_settings(f))["joint_speed_max"].tolist() == [1, 1, 1, 3, 3, 3]
    # a sim config cannot set its own limits
    with pytest.raises(SimError, match="limits come from the robot config"):
        World("<robot name='x'/>", {}, load_config(overrides={"base": {"wheel_speed_max": 3.0}}))


def test_disabled_modules_coast_and_heartbeat_loss_disables(w):
    drive(w, 1.5, 0.0, 0.3)
    w.step(200)                                          # no setpoints for 0.2 s: like a lost heartbeat
    assert not any(m.drive_on for m in w.modules)
    v1 = w.base_state()["twist"][0]
    w.step(300)
    assert 0.0 < w.base_state()["twist"][0] <= v1 + 1e-3     # coasting: no drive torque, slows by friction only


def test_pushing_moves_the_base_and_the_wheels_stall_at_their_torque_cap(w):
    w.apply_wrench("base", [0, 300.0, 0], [0, 0, 0], 0.5)    # sideways, wheels pointing forward and powered
    drive(w, 0.5, 0.0, 0.0)
    assert w.base_state()["xyz"][1] > 0.005                  # slides: friction, not an unbounded holding force
    w.teleport_base([0, 0, 0], yaw=0.0)
    assert w.base_state()["xyz"][:2] == [0.0, 0.0]


def test_arm_kinematic_follows_q_within_speed_limit_and_watchdog(w):
    q0 = np.array(w.arm_op("read", {})["q"])
    w.arm_op("enable", {})
    goal = q0 + np.array([0.6, -0.3, 0.2, 0.0, 0.3, -0.5])

    def send():
        w.arm_op("send", {"q": goal.tolist(), "qd": [0] * 6, "kp": [30] * 6, "kd": [1] * 6, "tau": [0] * 6,
                          "gripper": True, "g_q": -2.0, "g_qd": 0, "g_kp": 5, "g_kd": 0.1, "g_tau": 0})
    send()
    w.step(50)
    r = w.arm_op("read", {})
    assert abs(r["q"][0] - q0[0]) <= 6.0 * 0.05 + 0.005       # joint_speed_max 6 rad/s
    for _ in range(40):
        send()
        w.step(20)
    r = w.arm_op("read", {})
    assert np.allclose(r["q"], goal, atol=2e-3)
    assert r["g_q"] == pytest.approx(-2.0, abs=0.02)
    assert np.allclose(r["tau"], w.d.qfrc_bias[w.arm_v] + w.d.actuator_force[w.arm_a], atol=1e-6)
    assert r["status"][:6] == [1] * 6
    w.step(150)                                               # no frames for > 0.1 s: CAN watchdog
    r = w.arm_op("read", {})
    assert r["status"][:6] == [0xD] * 6 and not w.arm.enabled
    q_hold = r["q"]
    w.step(200)
    assert np.allclose(w.arm_op("read", {})["q"], q_hold, atol=2e-3)      # holds where it is


def test_timing_is_the_robots_unless_lenient_and_auto_is_macos_only():
    from robot_utils.nodes.sim.world import default_config, resolve_timing
    cfg = default_config()
    assert resolve_timing(cfg, "linux") == {"lenient": False, "why": "auto: not macOS", "arm_watchdog": 0.1,
                                            "swerve_heartbeat": 0.1}
    mac = resolve_timing(cfg, "darwin")
    assert mac["lenient"] and mac["arm_watchdog"] == 0.5 and mac["swerve_heartbeat"] == 0.5
    assert not resolve_timing({**cfg, "timing": {"lenient": False}}, "darwin")["lenient"]
    w = make_world(timing={"lenient": True})
    try:
        w.arm_op("enable", {})
        w.step(300)                                           # 0.3 s without frames: forgiven
        assert w.arm_op("read", {})["status"][:6] == [1] * 6
        w.step(250)                                           # past 0.5 s: the watchdog still trips
        assert w.arm_op("read", {})["status"][:6] == [0xD] * 6
        assert w.snapshot()["timing"]["lenient"] is True
    finally:
        w.close()


def test_time_step_run_pause_rtf(w):
    t0 = w.t
    w.step(100)
    assert w.t == pytest.approx(t0 + 0.1)
    w.run(1.0)
    time.sleep(0.6)
    w.pause()
    t1 = w.t
    assert 0.3 < t1 - t0 - 0.1 < 0.9
    time.sleep(0.1)
    assert w.t == t1
    with pytest.raises(SimError):
        w.run(0)


def test_scene_objects_add_move_remove_load_and_reset(w):
    w.add_object({"name": "crate", "shape": "box", "size": [0.2, 0.2, 0.2], "mass": 2.0, "xyz": [1.0, 0, 0.3]})
    w.step(500)
    o = w.object_state("crate")
    assert o["xyz"][2] == pytest.approx(0.1, abs=0.005)        # fell onto the floor
    w.move_object("crate", [1.5, 0.5, 0.1])
    assert w.object_state("crate")["xyz"][:2] == [1.5, 0.5]
    with pytest.raises(SimError):
        w.add_object({"name": "crate"})
    with pytest.raises(SimError):
        w.add_object({"name": "bad", "shape": "cone"})
    w.add_object({"name": "can", "shape": "cylinder", "radius": 0.03, "length": 0.12, "xyz": [0.8, 0.3, 0.06]})
    assert {o["name"] for o in w.scene()["objects"]} == {"crate", "can"}
    w.remove_object("crate")
    with pytest.raises(KeyError):
        w.remove_object("crate")
    w.load_scene([{"name": "a", "xyz": [2, 0, 0.05]}, {"name": "b", "xyz": [2, 1, 0.05]}])
    assert [o["name"] for o in w.scene()["objects"]] == ["a", "b"]
    w.move_object("a", [3, 0, 0.05])
    drive(w, 0.5, 0.0, 0.3)
    w.reset()
    assert w.t == 0.0 and w.object_state("a")["xyz"][0] == pytest.approx(2.0, abs=1e-3)
    assert abs(w.base_state()["xyz"][0]) < 1e-3


def test_driving_into_an_object_pushes_it(w):
    w.load_scene([{"name": "box", "size": [0.3, 0.3, 0.3], "mass": 1.0, "xyz": [0.5, 0.0, 0.15],
                   "friction": [0.3, 0.005, 0.0001]}])
    drive(w, 4.0, 0.0, 0.25)
    assert w.object_state("box")["xyz"][0] > 0.6


def test_ipc_protocol_roundtrip_errors_and_framing(tmp_sock_dir):
    import os
    path = os.path.join(tmp_sock_dir, "t.sock")
    closed = []

    def handler(h, payload, conn):
        if h["op"] == "echo":
            return {"args": h["x"]}, payload[::-1]
        raise SimError("nope")
    srv = IpcServer(path, handler, on_close=closed.append)
    c = IpcClient(path)
    r, data = c.call("echo", b"abc", x=[1, 2.5])
    assert r == {"ok": True, "args": [1, 2.5], "len": 3} and data == b"cba"
    with pytest.raises(IpcError, match="nope"):
        c.call("other")
    s = socket.socket(socket.AF_UNIX)
    s.connect(path)
    send_msg(s, {"op": "echo", "x": 1})
    h, p = recv_msg(s.makefile("rb"))
    assert h["ok"] and p == b""
    raw = b'{"op":"echo","x":0,"len":2}'
    s.sendall(struct.pack(">I", len(raw)) + raw + b"hi")              # the wire format by hand
    h, p = recv_msg(s.makefile("rb"))
    assert p == b"ih"
    s.close()
    c.close()
    deadline = time.time() + 2
    while len(closed) < 2 and time.time() < deadline:
        time.sleep(0.01)
    assert len(closed) == 2
    srv.close()
    with pytest.raises(IpcError, match="not reachable"):
        IpcClient(path).call("echo", x=1)


@needs_gl
def test_cameras_render_color_and_depth_with_delays(w):
    from robot_utils.nodes.sim.render import Cameras
    cams = Cameras(w, w.cfg["cameras"])
    try:
        info = cams.info("camera")
        assert info["intrinsics"]["color"]["width"] == 640 and info["intrinsics"]["depth"]["height"] == 400
        assert np.allclose(np.array(info["depth_to_color"]["rotation"]), np.eye(3), atol=1e-6)
        cams.start("camera", ["color", "depth"])
        got = {}
        deadline = time.time() + 10
        while len(got) < 2 and time.time() < deadline:
            w.step(40)
            metas, data = cams.frames("camera", {}, 0.2)
            off = 0
            for m in metas:
                got[m["stream"]] = (m, data[off:off + m["len"]])
                off += m["len"]
        mc, jpg = got["color"]
        assert jpg[:2] == b"\xff\xd8" and mc["t_capture"] - mc["t_sensor"] == pytest.approx(0.030)
        md, raw = got["depth"]
        d = np.frombuffer(raw, "<u2").reshape(md["height"], md["width"])
        # the head camera at 0.5 m looking forward sees the floor in the lower image half
        lower = d[int(md["height"] * 0.9):, md["width"] // 2]
        assert (lower > 0).all() and 500 < np.median(lower) < 3000
        assert md["t_capture"] - md["t_sensor"] == pytest.approx(0.017)
        time.sleep(0.3)
        n0 = cams.cams["camera"].rendered["color"]
        time.sleep(0.3)                                    # paused: no sim time, no new frames
        assert cams.cams["camera"].rendered["color"] == n0
        assert cams.snapshot_jpeg("wrist_camera")[:2] == b"\xff\xd8"
    finally:
        cams.close()


def test_gl_check_runs():
    assert gl_available() in (True, False)


def test_http_api(tmp_sock_dir, tmp_path):
    from fastapi.testclient import TestClient
    from robot_utils.nodes.sim.node import SimNode
    from robot_utils.nodes.sim.server import create_app
    w = make_world()
    node = SimNode(w, "simtest", str(tmp_path / "scenes"))
    with TestClient(create_app(node)) as c:
        st = c.get("/state").json()
        assert st["state"] == "paused" and set(st["backends"]) >= {"arm", "swerve", "camera-camera"}
        assert c.get("/meta").json()["actuated"] is False
        assert c.post("/step", json={"seconds": 0.25}).json()["t"] == pytest.approx(0.25)
        o = c.post("/scene/objects", json={"name": "b1", "size": [0.1, 0.1, 0.1], "xyz": [1, 0, 0.05]}).json()
        assert o["name"] == "b1"
        assert c.post("/scene/objects", json={"name": "b1"}).status_code == 400
        assert c.put("/scene/objects/b1", json={"xyz": [1, 1, 0.05]}).json()["xyz"][:2] == [1, 1]
        assert c.put("/scene/objects/zz", json={"xyz": [1, 1, 0.05]}).status_code == 404
        saved = c.post("/scene/save", json={"name": "one"}).json()
        assert saved["objects"][0]["name"] == "b1"
        c.delete("/scene/objects/b1")
        assert c.get("/scene").json()["objects"] == []
        assert [o["name"] for o in c.post("/scene/load", json={"name": "one"}).json()["objects"]] == ["b1"]
        assert c.post("/physics/wrench", json={"body": "base", "force": [50, 0, 0], "duration": 0.2}).status_code == 200
        c.post("/step", json={"n": 300})
        assert c.get("/state").json()["base"]["xyz"][0] > 0.001
        assert c.post("/physics/teleport", json={"xyz": [2, 0, 0], "yaw": 1.0}).json()["xyz"][0] == 2.0
        assert c.post("/run", json={"rtf": 2.0}).json()["running"] is True
        assert c.post("/pause").json()["running"] is False
        assert c.post("/scene/reset").json()["t"] == 0.0
        assert c.get("/").status_code == 200 and "Sim world" in c.get("/").text
        if gl_available():
            r = c.get("/cameras/camera/depth.jpg")
            assert r.status_code == 200 and r.content[:2] == b"\xff\xd8"
        assert c.get("/cameras/nope/color.jpg").status_code == 404
