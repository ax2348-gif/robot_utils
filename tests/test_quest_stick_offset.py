"""The engaged EE target and the base hint use the same (this frame's) stick offset in every frame (no one-frame lag
of the target behind the hint while the sticks move); the capture frame keeps the offset at zero, so engaging never
moves the robot even with a stick deflected. Pure (tools/quest_teleop)."""
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
        self.hs, self.gate, self.core = Headset(), QuestGate(rate_window=0.0, recover_s=0.0), TeleopCore(Config())
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
