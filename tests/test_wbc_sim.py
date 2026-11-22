"""The WBC on the MuJoCo world, in real time: the real swerve node on world-coupled SPARKs (its full CAN path), the
real estimator on the swerve stream, and the WBC. The arm side is a joint-channel peer that drives the world's
arm through ``WorldArmBus`` (MIT frames, pipelined reads) — standing in for the arm node's own joint channel, which
is being built on a parallel branch: switch this test to the real ``ArmNode`` once that is merged.
docs/wbc.md, docs/sim.md."""
import json
import math
import socket
import tempfile
import time

import httpx
import numpy as np
import pytest

from helpers import serve, stop, waiter
from sim_helpers import make_world
from wbc_fakes import Q0, FakeArm

SIM = "simwbc"
wait_for = waiter(15.0, 0.02)


class WorldArmPeer(FakeArm):
    """The fake arm's HTTP and joint channel, with the joints of the sim world: measured q from the world (one
    tick old, pipelined), commands sent to it as MIT frames (holding the measured pose until servoing)."""

    def __init__(self, path):
        from reBotArm_control_py.actuator.rebotarm import joint_limits, load_cfg
        from robot_utils.nodes.arm import config as C
        from robot_utils.nodes.sim.backends.arm import WorldArmBus
        self.bus = WorldArmBus(SIM, limits=joint_limits(load_cfg(C.PROFILES["soft"])))
        self.bus.connect()
        self.bus.enable()
        self.hold = None
        super().__init__(path)
        self.kinematic = False

    def _tx(self):
        kp, kd = np.full(6, 40.0), np.full(6, 1.0)
        while not self.stop.is_set():
            time.sleep(self.period)
            with self.lock:
                try:
                    r = self.bus.read()
                    self.q = np.array(r.q)
                    if self.hold is None:
                        self.hold = self.q.copy()
                    goal = self.q_cmd if self.state == "servoing" and np.all(np.isfinite(self.q_cmd)) else self.hold
                    qd = self.qd_cmd if self.state == "servoing" else np.zeros(6)
                    self.bus.send(goal, qd, kp, kd, np.zeros(6), 0.0, 0.0, 0.0, 0.0, 0.0, gripper=False)
                except Exception:  # noqa: BLE001 - the world went away at teardown
                    pass
                self.tick += 1
                self._send_state()

    def close(self):
        super().close()
        try:
            self.bus.close()
        except Exception:  # noqa: BLE001
            pass


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
    d = tempfile.mkdtemp(prefix="wbcs")
    arm = WorldArmPeer(d + "/arm.joint.sock")
    arm_url, arm_srv, arm_t = serve(arm.app)
    urdf, _ = fetch_urdf("local")
    node = WbcNode(WbcModel(urdf, (EESpec(arm_limits=arm_config_limits()),)), arm=arm_url, swerve=sw_url,
                   estimator=est_url, udp_host="127.0.0.1", udp_port=0, unix_socket=d + "/t.sock",
                   # everything shares one process (and the GIL) with the 1 kHz world here: the arm peer's
                   # states arrive less regularly than from a separate arm node
                   config={"arm_stale": 0.5, "resync": 0.3},
                   arm_socket=d + "/arm.joint.sock")
    url, srv, t = serve(wbc_app(node))
    try:
        wait_for(lambda: sw.status == "running" and node.estimator_health(time.monotonic()) is None)
        tok = httpx.post(url + "/lease", json={"client": "sim"}).json()["token"]
        h = {"X-Lease-Token": tok}
        r = httpx.post(url + "/enable", headers=h, timeout=10)
        assert r.status_code == 200, r.text
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
        assert st["fault"] is None, st["events"][-5:]
        b1 = world.world.base_state()
        moved = math.hypot(b1["xyz"][0] - b0["xyz"][0], b1["xyz"][1] - b0["xyz"][1])
        assert moved > 0.15, moved                               # the base drove in the world
        assert abs(math.hypot(L["base_x"], L["base_y"]) - moved) < 0.05   # the estimator agrees with the world
        assert errs[-1] < 0.02, (errs[-3:], st["state"], st["hold_reason"], st["limited_by"], {k: L[k] for k in ("arm_err_pos", "base_vx", "deadband", "hold", "status", "input_age")}, st["events"][-4:])                            # the measured EE (world arm at the estimated base)
        assert st["loop_hz"] > 80
        # the world's own TCP: the WBC model at the world's base pose and arm joints
        m = node.model
        c, s_ = math.cos(-b0["rpy"][2]), math.sin(-b0["rpy"][2])   # world displacement into the start frame
        dx, dy = b1["xyz"][0] - b0["xyz"][0], b1["xyz"][1] - b0["xyz"][1]
        T = m.fk(m.config((c * dx - s_ * dy, s_ * dx + c * dy, b1["rpy"][2] - b0["rpy"][2]), {"arm": arm.q}), "arm")
        assert np.linalg.norm(T.translation - goal) < 0.04, (T.translation, goal)
        assert httpx.post(url + "/disable", headers=h).status_code == 200
    finally:
        stop((srv, t), (arm_srv, arm_t), (est_srv, est_t), (sw_srv, sw_t))
        arm.close()
        sw.close()
        stop_world_sim(sims, iface, coupler)
