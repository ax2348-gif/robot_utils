"""The WBC on the MuJoCo world, in real time, with the real nodes: the arm node on ``WorldArmBus`` (its HTTP API
and its internal joint channel), the swerve node on world-coupled SPARKs (its full CAN path), the estimator on the
swerve stream, and the WBC. docs/wbc.md, docs/sim.md."""
import json
import math
import os
import socket
import tempfile
import time

import httpx
import numpy as np
import pytest

from helpers import serve, stop, waiter
from sim_helpers import make_world

SIM = "simwbc"
Q0 = (0.0, -0.6, -0.9, 0.0, 0.5, 0.0)
wait_for = waiter(20.0, 0.02)


@pytest.fixture
def world(tmp_path):
    from robot_utils.nodes.sim.node import SimNode
    w = make_world(time={"run": True}, components={"arm": {"start_q": list(Q0)}})
    node = SimNode(w, SIM, str(tmp_path / "scenes"))
    node.start()
    yield node
    node.close()


def test_whole_body_reach_in_the_sim_world(world):
    from robot_utils.nodes.estimator.node import EstimatorNode
    from robot_utils.nodes.estimator.server import create_app as estimator_app
    from robot_utils.nodes.sim.backends.swerve import start_world_sim, stop_world_sim
    from robot_utils.nodes.swerve.node import SwerveNode
    from robot_utils.nodes.swerve.server import create_app as swerve_app
    from robot_utils.nodes.wbc.model import EESpec, WbcModel, arm_config_limits, fetch_urdf
    from robot_utils.nodes.wbc.node import WbcNode
    from robot_utils.nodes.wbc.server import create_app as wbc_app
    sims, iface, coupler = start_world_sim(SIM)
    sw = SwerveNode(iface)
    sw_url, sw_srv, sw_t = serve(swerve_app(sw))
    est = EstimatorNode(sw_url, None)
    est_url, est_srv, est_t = serve(estimator_app(est))
    from reBotArm_control_py.actuator.rebotarm import joint_limits, load_cfg
    from robot_utils.nodes.arm import config as C
    from robot_utils.nodes.arm.node import ArmNode
    from robot_utils.nodes.arm.server import create_app as arm_app
    from robot_utils.nodes.common.locks import LockStore
    from robot_utils.nodes.sim.backends.arm import WorldArmBus
    d = tempfile.mkdtemp(prefix="wbcs")                     # short: macOS limits socket paths to ~104 bytes
    limits = joint_limits(load_cfg(C.PROFILES["soft"]))
    arm = ArmNode(lambda: WorldArmBus(SIM, limits=limits), locks=LockStore(os.path.join(d, "locks")),
                  joint_socket=os.path.join(d, "arm.joint.sock"))
    arm_url, arm_srv, arm_t = serve(arm_app(arm))
    urdf, _ = fetch_urdf("local")
    node = WbcNode(WbcModel(urdf, (EESpec(arm_limits=arm_config_limits()),)), arm=arm_url, swerve=sw_url,
                   estimator=est_url, udp_host="127.0.0.1", udp_port=0, unix_socket=d + "/t.sock",
                   arm_socket=d + "/arm.joint.sock",
                   # one process (one GIL) for the 1 kHz world and every node, often beside other test runs: the
                   # arm's state packets arrive less regularly than from its own process on the Pi
                   config={"arm_stale": 0.3})
    url, srv, t = serve(wbc_app(node))
    try:
        wait_for(lambda: arm.state == "uninitialized")
        wait_for(lambda: sw.status == "running" and node.estimator_health(time.monotonic()) is None)
        tok = httpx.post(url + "/lease", json={"client": "sim"}).json()["token"]
        h = {"X-Lease-Token": tok}
        r = httpx.post(url + "/enable", headers=h, timeout=15)
        assert r.status_code == 200, r.text
        js = arm.state_dict()["joint_channel"]["session"]
        assert js and js["client"] == "wbc", js                  # the real joint channel, opened by the WBC
        s = httpx.post(url + "/session", json={}, headers=h).json()
        latest = lambda: httpx.get(url + "/state").json()["latest"]   # noqa: E731
        wait_for(lambda: latest() and latest()["state"] == 2)
        L = latest()
        p0 = np.array([L[f"arm_cmd_{k}"] for k in "xyz"])
        q0 = [L[f"arm_cmd_q{k}"] for k in "xyzw"]
        b0 = world.world.base_state()
        goal = p0 + [0.40, 0.10, 0.0]
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        seq, errs = 0, []
        t_end = time.monotonic() + 15.0
        while time.monotonic() < t_end:
            seq += 1
            udp.sendto(json.dumps({"type": "target", "session": s["session"], "seq": seq, "t": None,
                                   "armed": True, "arm_seq": s.get("arm_seq", 0) + 1,     # armed (one press)
                                   "ees": {"arm": {"pos": list(goal), "quat": q0}}}).encode(),
                       ("127.0.0.1", s["udp_port"]))
            time.sleep(0.02)
            if seq % 25 == 0:
                L = latest()
                meas = np.array([L[f"arm_meas_{k}"] for k in "xyz"])
                errs.append(float(np.linalg.norm(meas - goal)))
                if errs[-1] < 0.01 and abs(L["base_vx"]) < 1e-9 and seq > 100:
                    break
        L = latest()
        st = httpx.get(url + "/state").json()
        assert st["fault"] is None, (st["events"][-3:], [e["msg"] for e in arm.events.list()[-4:]])
        b1 = world.world.base_state()
        moved = math.hypot(b1["xyz"][0] - b0["xyz"][0], b1["xyz"][1] - b0["xyz"][1])
        assert moved > 0.10, moved                               # the base drove in the world
        assert abs(math.hypot(L["base_x"], L["base_y"]) - moved) < 0.05   # the estimator agrees with the world
        assert errs[-1] < 0.02, (errs[-3:], st["state"], st["hold_reason"], st["limited_by"], {k: L[k] for k in ("arm_err_pos", "base_vx", "deadband", "hold", "status", "input_age")}, st["events"][-4:])                            # the measured EE (world arm at the estimated base)
        assert st["loop_hz"] > 80
        js = arm.state_dict()["joint_channel"]["session"]       # the arm took the WBC's goals: none dropped
        assert js["counts"]["goals"] > 200 and not {k: v for k, v in js["dropped"].items() if v}, js
        assert st["arm"]["resyncs"] == 0 and st["arm"]["limits"]["max_lead"] > 0
        # the world's own TCP: the WBC model at the world's base pose and arm joints
        m = node.model
        c, s_ = math.cos(-b0["rpy"][2]), math.sin(-b0["rpy"][2])   # world displacement into the start frame
        dx, dy = b1["xyz"][0] - b0["xyz"][0], b1["xyz"][1] - b0["xyz"][1]
        T = m.fk(m.config((c * dx - s_ * dy, s_ * dx + c * dy, b1["rpy"][2] - b0["rpy"][2]), {"arm": np.array(world.world.arm_op("read", {})["q"])}), "arm")
        assert np.linalg.norm(T.translation - goal) < 0.04, (T.translation, goal)
        assert httpx.post(url + "/disable", headers=h).status_code == 200
        wait_for(lambda: arm.lease.info is None)               # the WBC gave the arm lease back
    finally:
        stop((srv, t), (arm_srv, arm_t), (est_srv, est_t), (sw_srv, sw_t))
        sw.close()
        stop_world_sim(sims, iface, coupler)
