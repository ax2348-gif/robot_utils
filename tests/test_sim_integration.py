"""The real nodes on the sim world, in-process: the arm node on WorldArmBus, the swerve node on world-coupled
SPARKs (its full CAN path), the orbbec Owner on WorldCameraBackend. The world runs in real time with its
backend sockets, as `python -m robot_utils.nodes.sim` would. docs/sim.md."""
import math
import threading
import time

import numpy as np
import pytest

from helpers import waiter
from sim_helpers import make_world, needs_gl

from robot_utils.nodes.sim.node import SimNode

SIM = "simit"
wait_for = waiter(10.0, 0.02)


@pytest.fixture
def sim(tmp_path):
    w = make_world(time={"run": True}, components={"arm": {"start_q": [0.0, -0.6, -0.9, 0.0, 0.5, 0.0]}})
    node = SimNode(w, SIM, str(tmp_path / "scenes"))
    node.start()
    assert w.running
    yield node
    node.close()


def test_arm_node_on_the_world(sim, tmp_path):
    from reBotArm_control_py.actuator.rebotarm import joint_limits, load_cfg
    from robot_utils.nodes.arm import config as C
    from robot_utils.nodes.arm.node import ArmNode
    from robot_utils.nodes.common.locks import LockStore
    from robot_utils.nodes.sim.backends.arm import WorldArmBus
    buses = []

    def factory():
        b = WorldArmBus(SIM, limits=joint_limits(load_cfg(C.PROFILES["soft"])))
        buses.append(b)
        return b

    node = ArmNode(factory, locks=LockStore(tmp_path / "locks"))
    node.start()
    try:
        wait_for(lambda: node.state == "uninitialized", 20)
        tok, _ = node.acquire("test")
        out = node.enable(tok)
        assert out["enabled"] and sim.world.arm.enabled
        assert node.reading is not None and node.bus.reply_latency == pytest.approx(0.007)
        q_start = np.array(node.reading.q)
        T = node.kin.fk(q_start)
        T.translation = T.translation + np.array([0.0, 0.0, 0.02])
        out = node.track_cmd(tok, T)
        assert out["ok"], out
        stop = threading.Event()

        def renew():
            while not stop.wait(0.1):
                node.renew(tok)
        th = threading.Thread(target=renew, daemon=True)
        th.start()
        try:
            wait_for(lambda: np.linalg.norm(node.kin.fk(np.array(sim.world.arm_op("read", {})["q"])).translation
                                            - T.translation) < 0.002, 10)
            r = node.reading                                   # the node's own (pipelined) reading agrees
            assert np.linalg.norm(node.kin.fk(r.q).translation - T.translation) < 0.004
            assert r.t_meas is not None and r.t_meas < r.t and r.t_meas_mot <= r.t
            assert r.t - r.t_meas < 0.1                        # about one tick old (pipelined)
            from robot_utils.nodes.arm.node import COL, DT
            rows = node.hub.recent(50, "tick")                 # the row's t_meas columns: one tick behind t_tick
            lag = rows[:, COL["t_tick"]] - rows[:, COL["t_meas"]]
            assert np.all(np.isfinite(lag)) and 0.5 * DT < np.median(lag) < 1.6 * DT, np.median(lag)
            assert np.all(rows[:, COL["t_meas_mot"]] >= rows[:, COL["t_meas"]] - 1e-9)
            # the gripper follows the node's gap goal
            node.gripper_cmd(tok, gap_mm=30.0)
            wait_for(lambda: abs(sim.world.snapshot()["arm"]["gap_mm"] - 30.0) < 1.0, 10)
        finally:
            stop.set()
            th.join()
        node.disable(tok)                                  # floats: the motors stay on, gainless
    finally:
        node.close()
    assert not sim.world.arm.enabled and sim.world.arm.fault == 0      # closed cleanly: motors off, no lost bus


def test_arm_bus_lost_connection_faults_the_motors(sim):
    from robot_utils.nodes.sim.backends.arm import WorldArmBus
    b = WorldArmBus(SIM)
    b.connect()
    b.enable()
    r1 = b.read()
    r2 = b.read()                                          # answers the previous read's request
    assert r2.t_meas > r1.t_meas and list(r2.status[:6]) == [1] * 6
    b.client.close()                                       # the process dies: the socket closes
    wait_for(lambda: sim.world.arm.fault == 0xD, 3)


def test_swerve_node_on_world_coupled_sparks(sim):
    from robot_utils.nodes.sim.backends.swerve import start_world_sim, stop_world_sim
    from robot_utils.nodes.swerve.node import FIELDS, SwerveNode
    sims, iface, coupler = start_world_sim(SIM)
    node = SwerveNode(iface)
    node.start()
    try:
        assert node.status == "running"
        tok, _ = node.acquire("test")
        node.enable(tok)
        x0 = sim.world.base_state()["xyz"]
        stop = threading.Event()

        def hold(twist):
            while not stop.wait(0.05):
                node.command(tok, twist)
        th = threading.Thread(target=hold, args=((0.2, 0.0, 0.0),), daemon=True)
        th.start()
        try:
            wait_for(lambda: abs(sim.world.base_state()["twist"][0] - 0.2) < 0.02, 10)
            time.sleep(0.5)
            b = sim.world.base_state()
            assert b["twist"][0] == pytest.approx(0.2, abs=0.02) and abs(b["twist"][1]) < 0.02
            row = dict(zip(FIELDS, node.hub.recent(1, "tick")[0]))
            for m in ("FL", "FR", "RR", "RL"):                 # what the node reads back over CAN
                assert row[f"{m}_drive_vel"] == pytest.approx(0.2, abs=0.03), m
                assert abs(math.remainder(row[f"{m}_steer_angle"], math.pi)) < 0.05, m
        finally:
            stop.set()
            th.join()
        moved = sim.world.base_state()["xyz"][0] - x0[0]
        assert moved > 0.1
        # sideways: the modules steer through 90 degrees first, then the base goes left
        stop.clear()
        th = threading.Thread(target=hold, args=((0.0, 0.15, 0.0),), daemon=True)
        th.start()
        try:
            y0 = sim.world.base_state()["xyz"][1]
            wait_for(lambda: sim.world.base_state()["xyz"][1] - y0 > 0.1, 10)
        finally:
            stop.set()
            th.join()
        node.command(tok, (0.0, 0.0, 0.0))
        wait_for(lambda: abs(sim.world.base_state()["twist"][1]) < 0.01, 5)
    finally:
        node.close()
        stop_world_sim(sims, iface, coupler)
    wait_for(lambda: not any(m.drive_on for m in sim.world.modules), 3)       # heartbeat gone: disabled


@needs_gl
def test_orbbec_owner_on_the_world_camera(sim):
    from robot_utils.nodes.orbbec.hub import Hub
    from robot_utils.nodes.orbbec.models import Settings
    from robot_utils.nodes.orbbec.owner import Owner
    from robot_utils.nodes.sim.backends.camera import WorldCameraBackend
    hub = Hub()
    seen = []
    orig = hub.publish_frames
    hub.publish_frames = lambda frames: (seen.extend(frames), orig(frames))
    o = Owner(WorldCameraBackend(SIM, "camera"), hub, Settings(), startup_s=5, calibration_file=None)
    o.start()
    try:
        wait_for(lambda: o.state == "streaming", 15)
        wait_for(lambda: {f.stream for f in seen} >= {"color", "depth"} and len(seen) > 20, 15)
        c = next(f for f in reversed(seen) if f.stream == "color")
        d = next(f for f in reversed(seen) if f.stream == "depth")
        assert c.fmt == "MJPG" and c.data[:2] == b"\xff\xd8" and (c.w, c.h) == (640, 480)
        assert d.fmt == "Y16" and d.data.shape == (400, 640) and d.data.dtype == np.uint16
        assert c.t_capture - c.t_sensor == pytest.approx(0.030, abs=1e-6)
        assert d.t_capture - d.t_sensor == pytest.approx(0.017, abs=1e-6)
        assert c.t_owner >= c.t_capture - 1e-3                 # handed over no earlier than t_capture
        assert (d.data[-20:, 300:340] > 0).all()                # the floor
        cal = o.calibration
        assert cal["color"]["width"] == 640 and cal["depth"]["height"] == 400
        n = len([f for f in seen if f.stream == "color"])
        time.sleep(1.0)
        rate = (len([f for f in seen if f.stream == "color"]) - n) / 1.0
        assert rate > 10                                        # 30 fps configured; rendering keeps up mostly
    finally:
        o.close()
