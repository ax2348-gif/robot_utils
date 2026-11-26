"""The engaged EE target and the base hint use the same (this frame's) stick offset in every frame (no one-frame lag
of the target behind the hint while the sticks move); the capture frame keeps the offset at zero, so engaging never
moves the robot even with a stick deflected. Pure (tools/quest_teleop).

The robot is a fixed stream row (it never follows the hint), so this is the untethered offset (``tether`` off: the
raw stick integral, the EE rigid on the hint). Tethered, the same-frame property is
test_quest_stick_tether.py::test_tethered_target_and_hint_share_this_frames_offset."""
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from quest_teleop import frames as F  # noqa: E402
from quest_teleop.body_frozen import Config, RobotState, TeleopCore  # noqa: E402
from quest_teleop.fake_headset import Headset, HeadsetState  # noqa: E402
from quest_teleop.quest_stream import Frame, QuestGate  # noqa: E402

IDLE = HeadsetState()
GRIP = replace(IDLE, right=replace(IDLE.right, grip=1.0))


def stick(state, left=(0.0, 0.0), right_x=0.0):
    return replace(state, left=replace(state.left, stick=left), right=replace(state.right, stick=(right_x, 0.0)))


class Rig:
    def __init__(self):
        self.hs, self.gate = Headset(), QuestGate(rate_window=0.0, recover_s=0.0)
        self.core = TeleopCore(Config(tether=False))
        self.t = 100.0
        self.rob = RobotState(0.0, np.array([1.5, 2.1, 0.8]), F.rot_axis((1, 1, 0), 0.7), 1.0, 2.0, 0.3, 0.0)

    def frame(self, s):
        self.t += 0.01
        d = self.hs.frame(s, self.t)
        self.rob.t_recv = self.t
        self.core.on_frame(d, self.gate.check(Frame(d, self.t, 1)), self.rob, self.t)
        return self.core.command(self.t, self.gate.fresh(self.t), self.rob)


def hint_frame(c):
    t = c["base"]["target"]
    return F.PlanarFrame(t["pose"]["x"], t["pose"]["y"], t["heading"]["yaw"])


def test_target_and_hint_share_the_current_offset_in_every_frame():
    rig = Rig()
    for _ in range(5):
        rig.frame(IDLE)
    c = rig.frame(stick(GRIP, (0.5, 1.0), right_x=-1.0))       # engaged with the sticks already deflected
    assert rig.core.eng is not None and not np.any(rig.core.offset)                # the capture frame: zero
    np.testing.assert_allclose(c["ees"]["arm"]["pos"], rig.rob.ee_p, atol=1e-12)  # engaging never moves it
    np.testing.assert_allclose([c["base"]["target"]["pose"]["x"], c["base"]["target"]["pose"]["y"]], [1.0, 2.0],
                               atol=1e-12)
    rel0 = F.PlanarFrame(1.0, 2.0, 0.3).to_local(rig.rob.ee_p, rig.rob.ee_r)
    moved = 0.0
    for k in range(60):                                          # the sticks moving (varying), standing still
        c = rig.frame(stick(GRIP, (0.5 * np.cos(k / 9), 1.0), right_x=-np.sin(k / 7)))
        e = c["ees"]["arm"]
        rel = hint_frame(c).to_local(np.array(e["pos"]), F.matrix_from_quat(e["quat"]))
        np.testing.assert_allclose(rel[0], rel0[0], atol=1e-9)  # the same rigid offset, this very frame
        np.testing.assert_allclose(rel[1], rel0[1], atol=1e-9)
        moved = np.linalg.norm(np.array(e["pos"]) - rig.rob.ee_p)
    assert moved > 0.2


def test_the_engaged_stick_ramps_at_the_bases_limits_and_reaches_full_speed():
    """stick_accel / stick_alpha (open_wbc sets them from the WBC's max_accel / max_alpha): a full stick held from the
    capture ramps the offset's rate at those limits (no step the base cannot follow), never clipped: it reaches the
    full stick velocity; the hint's feedforward is the shaped rate; on release it ramps down the same way."""
    rig = Rig()
    rig.core.cfg.stick_accel, rig.core.cfg.stick_alpha = 1.0, 1.5
    for _ in range(5):
        rig.frame(IDLE)
    rig.frame(GRIP)                                              # captured
    speeds = []
    for _ in range(120):                                         # 1.2 s of full stick forward
        c = rig.frame(stick(GRIP, (1.0, 0.0), right_x=1.0))
        speeds.append(np.hypot(c["base"]["target"]["pose"]["vx"], c["base"]["target"]["pose"]["vy"]))
    speeds = np.array(speeds)
    assert np.all(np.diff(speeds) <= 1.0 * 0.01 + 1e-6)          # never faster than 1 m/s^2
    assert speeds[10] < 0.15                                     # 0.1 s in: about 0.1 m/s, not the stick's 0.8
    assert abs(speeds[-1] - rig.core.cfg.stick_max_speed) < 1e-6 # it gets there: never clipped
    assert abs(rig.core.v_shaped[2]) <= rig.core.cfg.stick_max_omega + 1e-9
    off = []
    for _ in range(100):                                         # released: ramps down, the offset stops moving
        rig.frame(GRIP)
        off.append(rig.core.offset[:2].copy())
    assert np.linalg.norm(off[-1] - off[-2]) < 1e-12 and abs(rig.core.v_shaped[0]) < 1e-12
