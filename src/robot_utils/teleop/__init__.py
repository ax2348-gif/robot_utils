"""Generic, callback-based teleoperation from the Quest (see ``robot_utils.quest``).

A robot adapter subclasses ``TeleopApp`` and implements coarse callbacks (``start``,
``engage``, ``active_step``, ``disengage``, ``hold_step``, ``stop``, plus optional
``on_fault`` / ``emergency_stop`` / ``wants_active``); ``TeleopRunner`` runs them at a
fixed rate and owns everything that must be right for every robot: input freshness and
validity, release-before-engage clutching, fault policy, overruns and the signal e-stop.

    from robot_utils.quest import QuestClient
    from robot_utils.teleop import PoseClutch, TeleopApp, TeleopRunner

    class MyRobot(TeleopApp):
        def start(self, ctx): ...                 # connect, enable holding
        def active_step(self, ctx, inputs):
            target = self.clutch.update(inputs.hands["right"], measure=self.measure_tool_pose)
            ...                                   # follow target, map inputs.hands["right"].trigger
        def hold_step(self, ctx, inputs): ...     # hold still; inputs.twist is zero unless live
        def stop(self, ctx, result): ...          # disable, disconnect

    with QuestClient(host, port) as quest:
        result = TeleopRunner(MyRobot(), quest).run()

See docs/teleop.md for the full contract.
"""
from .app import (ESTOP, FAULT, REQUESTED, Context, Fault, FaultAction, Inputs, Overrun,
                  RunResult, Stats, TeleopApp)
from .clutch import ENGAGED, INVALID, NEEDS_RELEASE, NO_INPUT, RELEASED, HandClutch, HandState
from .estop import EStop
from .monitor import Monitor, MonitorStats
from .pose import Pose, PoseClutch, clamp_rotation, rotation_from_vector, rotation_vector
from .runner import FrameSource, TeleopRunner

__all__ = [
    "TeleopApp", "TeleopRunner", "Context", "Inputs", "Fault", "FaultAction", "Overrun",
    "RunResult", "Stats", "REQUESTED", "ESTOP", "FAULT", "FrameSource",
    "HandClutch", "HandState", "NO_INPUT", "INVALID", "RELEASED", "NEEDS_RELEASE", "ENGAGED",
    "EStop", "Monitor", "MonitorStats", "Pose", "PoseClutch", "rotation_vector", "rotation_from_vector", "clamp_rotation",
]
