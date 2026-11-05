#!/usr/bin/env python3
"""Quest teleop of the reBot arm on robot_utils.teleop: an example robot adapter.

This is rebot_ctl's tools/quest_teleop.py rewritten as a ``TeleopApp``. Everything
reBot-specific lives here (MIT mode, gravity feed-forward, IK, the joint slew limit, the
gripper mapping); the runner supplies the loop, the input contract, the clutch, faults and
the e-stop.

Mapping of the generic inputs:
    hands[--hand] pose delta  -> TCP target (anchored on the measured TCP), solved by IK
    hands[--hand].trigger     -> gripper position (0 open .. 1 closed)
    twist                     -> unused (no base)

Behaviour: motors stay disabled until the first engagement; then they hold the last
command under gravity feed-forward while released. A bus read error skips the tick
(IGNORE); three in a row stop the run. On Ctrl-C/e-stop the loop stops commanding and the
motors' 100 ms CAN watchdog disables them; stop() then disconnects.

    python examples/rebot_quest_teleop.py [hardware_yaml] --host 100.72.77.83
"""
import argparse
import logging

import numpy as np
import pinocchio as pin
from motorbridge import CallError

from reBotArm_control_py.actuator import RebotArm, load_cfg
from reBotArm_control_py.dynamics import compute_generalized_gravity
from reBotArm_control_py.kinematics import (compute_fk, get_arm_joint_count,
                                            get_end_effector_frame_id, load_robot_model,
                                            pad_q_for_model)
from reBotArm_control_py.kinematics.inverse_kinematics import IKParams, solve_ik

from robot_utils.quest import DEFAULT_PORT, QuestClient
from robot_utils.teleop import FaultAction, Pose, PoseClutch, TeleopApp, TeleopRunner

TCP_OFFSET = pin.SE3(np.eye(3), np.array([-0.15, 0.0, 0.0]))   # tool point on end_link
MAX_JOINT_SPEED = 2.5        # rad/s: slew limit on the joint command
GRIPPER_OPEN_RAD = -3.1      # gripper position when open (0 = closed); None = leave it alone


class RebotArmTeleop(TeleopApp):
    rate_hz = 100.0   # well above the motors' 100 ms CAN watchdog

    def __init__(self, hardware_yaml, hand):
        self.hand = hand
        self.robot = RebotArm(hardware_yaml)
        self.model = load_robot_model(load_cfg(hardware_yaml).get("urdf_path"))
        ee_id = get_end_effector_frame_id(self.model)
        ee = self.model.frames[ee_id]
        self.tcp_id = self.model.addFrame(pin.Frame("tcp", ee.parentJoint, ee_id, ee.placement * TCP_OFFSET,
                                                    pin.FrameType.OP_FRAME))
        self.data = self.model.createData()
        self.n = get_arm_joint_count(self.model, self.tcp_id)
        self.ik = IKParams(max_iter=30)
        self.clutch = PoseClutch()   # operator frame (x fwd, y left, z up) == arm base frame
        self.groups = [self.robot.arm] + ([self.robot.gripper] if GRIPPER_OPEN_RAD is not None else [])
        self.enabled = False
        self.q_cmd = None

    # -- helpers -------------------------------------------------------------

    def measure(self):
        return pad_q_for_model(self.model, self.robot.arm.get_positions(position_source="output"), self.n)

    def tcp(self, q):
        p, r = compute_fk(self.model, q, "tcp")[:2]
        return Pose(np.asarray(p), np.asarray(r))

    def send_arm(self, q_meas):
        tau = compute_generalized_gravity(self.model, q_meas, self.data)[:self.n]
        self.robot.arm.send_mit(self.q_cmd, tau=tau)

    # -- callbacks -----------------------------------------------------------

    def start(self, ctx):
        self.robot.connect()
        for jc in self.robot._all_joints:   # clear faults left over (e.g. a tripped watchdog)
            self.robot._motor_map[jc.name].clear_error()
        for g in self.groups:
            g.mode_mit()
        self.robot.disable_all()
        print(f"Grip the {self.hand} controller to drive the arm; Ctrl-C to quit.")

    def engage(self, ctx, inputs):
        if not self.enabled:
            for g in self.groups:
                g.enable()
            self.enabled = True
        self.q_cmd = self.measure()[:self.n].copy()   # follow from where the arm is: no jump

    def active_step(self, ctx, inputs):
        hand = inputs.hands[self.hand]
        q = self.measure()
        target = self.clutch.update(hand, lambda: self.tcp(q))
        if target is None:        # this hand is not the one engaged (the other hand is)
            self.hold_step(ctx, inputs)
            return
        result = solve_ik(self.model, self.data, self.tcp_id, pin.SE3(target.rotation, target.position),
                          self.q_cmd, self.ik, controlled_joints=self.n)
        step = MAX_JOINT_SPEED * ctx.period_s
        self.q_cmd = self.q_cmd + np.clip(result.q[:self.n] - self.q_cmd, -step, step)
        self.send_arm(q)
        if GRIPPER_OPEN_RAD is not None and hand.trigger is not None:
            self.robot.gripper.send_mit(np.array([GRIPPER_OPEN_RAD * (1.0 - hand.trigger)]))

    def disengage(self, ctx, inputs):
        self.clutch.reset()       # hold_step keeps the last command

    def hold_step(self, ctx, inputs):
        if self.enabled and self.q_cmd is not None:
            self.send_arm(self.measure())

    def wants_active(self, ctx, inputs):
        return inputs.hands[self.hand].engaged

    def on_fault(self, ctx, fault):
        if isinstance(fault.error, CallError) and fault.phase in ("active_step", "hold_step"):
            return FaultAction.IGNORE   # one lost read; the runner stops after 3 in a row
        return FaultAction.STOP

    def stop(self, ctx, result):
        print(f"\nstopping ({result.kind}: {result.reason}); disabling and disconnecting")
        self.robot.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("hardware_yaml", nargs="?", default=None)
    parser.add_argument("--host", default="100.72.77.83")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")

    app = RebotArmTeleop(args.hardware_yaml, args.hand)
    with QuestClient(args.host, args.port) as quest:
        result = TeleopRunner(app, quest).run()
    print(f"{result.kind}: {result.reason}; {result.stats}")
    raise SystemExit(0 if result.ok or result.kind == "estop" else 1)


if __name__ == "__main__":
    main()
