#!/usr/bin/env python3
"""Quest teleop of a simulated reBot arm in MeshCat, on robot_utils.teleop. No hardware.

The same adapter shape as examples/rebot_quest_teleop.py, with the arm replaced by its
kinematic model (the commanded joints are the "measured" ones):

    hands[--hand] pose delta  -> TCP target (anchored on the model's TCP), solved by IK
    hands[--hand].trigger     -> finger joints (0 open .. 1 closed), whenever the hand is valid
    twist                     -> drives a virtual mobile base under the arm (--no-drive: off)

Target frame policy (--base-frame): ``live`` keeps the target fixed relative to the moving
base; ``frozen`` keeps it fixed in the world from the moment of engagement, which is what
the Quest's body-frozen (walking) scheme expects. ``auto`` (default) follows the scheme
the Quest reports.

The steps only compute; drawing and the status line run on a ``Monitor`` thread at
--viz-hz, so a slow MeshCat round trip (e.g. while a browser connects) cannot stall control.

Frames (red X / green Y / blue Z): base, end_link (small), ee = TCP; while engaged also
anchor, target and a yellow anchor->target line. The target ball is green while engaged.

    python examples/rebot_quest_teleop_meshcat.py --host 100.72.77.83 --port 9392 [--hand right]

Open the MeshCat URL it prints. Ctrl-C to quit.
"""
import argparse
import logging
from dataclasses import dataclass
from typing import Optional

import meshcat
import meshcat.geometry as mcg
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

from reBotArm_control_py.kinematics import (compute_fk, get_arm_joint_count,
                                            get_end_effector_frame_id)
from reBotArm_control_py.kinematics.inverse_kinematics import IKParams, solve_ik
from reBotArm_control_py.kinematics.robot_model import _resolve_urdf

from robot_utils.quest import QuestClient
from robot_utils.teleop import Monitor, Pose, PoseClutch, TeleopApp, TeleopRunner

TCP_OFFSET = pin.SE3(np.eye(3), np.array([-0.15, 0.0, 0.0]))   # tool point on end_link
MAX_JOINT_SPEED = 2.5        # rad/s: slew limit on the joint command
FINGERS = ("finger_left", "finger_right")
FINGER_OPEN = 0.0285         # finger_left upper limit; finger_right mirrors it
ROOT = "pinocchio"           # MeshcatVisualizer's root node; frames go under it to move with the base


def planar(x, y, yaw) -> Pose:
    return Pose(np.array([x, y, 0.0]), pin.rpy.rpyToMatrix(0.0, 0.0, yaw))


@dataclass(frozen=True)
class Snapshot:
    """What the monitor draws; copies only, taken on the control thread."""
    q: np.ndarray
    base: Pose
    ee: Pose
    end_link: Pose
    anchor: Optional[Pose]
    target: Optional[Pose]
    input_status: str
    hand_status: str
    policy: str
    trigger: float


class SimArmTeleop(TeleopApp):
    rate_hz = 100.0

    def __init__(self, hand, urdf_path=None, drive=True, base_frame="auto", viz_hz=30.0):
        self.hand, self.drive, self.base_frame, self.viz_hz = hand, drive, base_frame, viz_hz
        urdf, package_dirs = _resolve_urdf(urdf_path)
        self.model = pin.buildModelFromUrdf(urdf)
        ee_id = get_end_effector_frame_id(self.model)
        ee = self.model.frames[ee_id]
        self.tcp_id = self.model.addFrame(pin.Frame("tcp", ee.parentJoint, ee_id, ee.placement * TCP_OFFSET,
                                                    pin.FrameType.OP_FRAME))
        self.data = self.model.createData()   # control thread only; after addFrame
        self.visual = pin.buildGeomFromUrdf(self.model, urdf, pin.GeometryType.VISUAL,
                                            package_dirs=package_dirs)
        self.n = get_arm_joint_count(self.model, self.tcp_id)
        self.fingers = [self.model.joints[self.model.getJointId(j)].idx_q for j in FINGERS]
        self.ik = IKParams(max_iter=30)
        self.clutch = PoseClutch()   # operator frame (x fwd, y left, z up) == arm base frame
        self.q = pin.neutral(self.model)
        self.trigger = 0.0
        self.base = np.zeros(3)      # x, y, yaw of the virtual base in the world
        self.monitor = None

    # -- control-thread helpers ----------------------------------------------

    def fk(self, frame=None):
        p, r = compute_fk(self.model, self.q, frame)[:2] if frame else compute_fk(self.model, self.q)[:2]
        return Pose(np.array(p), np.array(r))

    def base_pose(self):
        return planar(*self.base)

    def policy(self, inputs):
        if self.base_frame != "auto":
            return self.base_frame
        intent = inputs.frame.intent if inputs.frame is not None else None
        return "frozen" if intent is not None and intent.scheme == "body-frozen" else "live"

    def step_common(self, ctx, inputs, target):
        h = inputs.hands[self.hand]
        if h.trigger is not None:          # independent of the clutch; held while invalid
            self.trigger = h.trigger
        self.q[self.fingers[0]] = FINGER_OPEN * (1 - self.trigger)
        self.q[self.fingers[1]] = -FINGER_OPEN * (1 - self.trigger)
        if self.drive:                     # twist is zero unless input is live
            x, y, yaw = self.base
            fwd, left, wz = inputs.twist * ctx.period_s
            self.base = np.array([x + fwd * np.cos(yaw) - left * np.sin(yaw),
                                  y + fwd * np.sin(yaw) + left * np.cos(yaw), yaw + wz])
        self.monitor.publish(Snapshot(
            q=self.q.copy(), base=self.base_pose(), ee=self.fk("tcp"), end_link=self.fk(),
            anchor=self.clutch.anchor, target=target, input_status=inputs.status,
            hand_status=h.status, policy=self.policy(inputs), trigger=self.trigger))

    # -- callbacks -----------------------------------------------------------

    def start(self, ctx):
        self.vis = meshcat.Visualizer()
        # the viewer gets its own Data: it runs on the monitor thread
        self.viewer = MeshcatVisualizer(self.model, collision_model=None, visual_model=self.visual,
                                        data=self.model.createData(), visual_data=self.visual.createData())
        self.viewer.initViewer(self.vis, loadModel=False)
        self.viewer.loadViewerModel(rootNodeName=ROOT)
        self.vis["/Cameras/default/rotated/<object>"].set_property("position", [0.6, 0.5, 0.45])
        self.drawn, self.props = set(), {}
        self.pose("base", Pose.identity(), 0.15)
        self.vis[ROOT]["frames/target/ball"].set_object(mcg.Sphere(0.012),
                                                        mcg.MeshLambertMaterial(color=0x888888))
        self.monitor = Monitor(self.render, rate_hz=self.viz_hz, name="meshcat").start()
        print(f"MeshCat: {self.vis.url()}\nGrip the {self.hand} controller to drive the arm; Ctrl-C to quit.")

    def active_step(self, ctx, inputs):
        frozen = self.policy(inputs) == "frozen"
        target = self.clutch.update(inputs.hands[self.hand], lambda: self.fk("tcp"),
                                    frame=self.base_pose() if frozen else None)
        if target is not None:
            result = solve_ik(self.model, self.data, self.tcp_id, pin.SE3(target.rotation, target.position),
                              self.q[:self.n], self.ik, controlled_joints=self.n)
            step = MAX_JOINT_SPEED * ctx.period_s
            # unreachable targets are approached as far as possible; re-grip to recover
            self.q[:self.n] += np.clip(result.q[:self.n] - self.q[:self.n], -step, step)
        self.step_common(ctx, inputs, target)

    def disengage(self, ctx, inputs):
        self.clutch.reset()

    def hold_step(self, ctx, inputs):
        self.step_common(ctx, inputs, None)     # arm joints stay where they are

    def wants_active(self, ctx, inputs):
        return inputs.hands[self.hand].engaged

    def stop(self, ctx, result):
        if self.monitor is not None:
            self.monitor.close()
            print(f"\nmonitor: {self.monitor.stats}")
        print(f"stopping ({result.kind}: {result.reason})")

    # -- monitor thread ------------------------------------------------------

    def pose(self, name, pose, scale=0.08):
        node = self.vis[ROOT]["frames"][name]
        if name not in self.drawn:
            node.set_object(mcg.triad(scale))
            self.drawn.add(name)
        node.set_transform(pose.matrix())

    def prop(self, name, key, value):
        if self.props.get((name, key)) != value:   # every MeshCat call is a round trip
            self.vis[ROOT]["frames"][name].set_property(key, value)
            self.props[(name, key)] = value

    def render(self, s: Snapshot):
        self.viewer.display(s.q)
        self.vis[ROOT].set_transform(s.base.matrix())
        self.pose("ee", s.ee)
        self.pose("end_link", s.end_link, 0.03)
        engaged = s.target is not None
        self.prop("anchor", "visible", engaged)
        self.prop("delta", "visible", engaged)
        self.prop("target/ball", "color", [0, 0.8, 0.2, 1] if engaged else [0.5, 0.5, 0.5, 1])
        if engaged:
            self.pose("anchor", s.anchor, 0.05)
            self.pose("target", s.target)
            self.vis[ROOT]["frames/delta"].set_object(mcg.Line(
                mcg.PointsGeometry(np.array([s.anchor.position, s.target.position], dtype=np.float32).T),
                mcg.LineBasicMaterial(color=0xFFCC00)))
            err = np.linalg.norm(s.target.position - s.ee.position)
        else:
            self.pose("target", s.ee)
            err = 0.0
        print(f"\r[{s.input_status:9} {s.hand_status:13} {s.policy:6}] err={err * 1e3:5.1f}mm "
              f"trigger={s.trigger:.2f} q(deg)={np.array2string(np.degrees(s.q[:self.n]), precision=0, suppress_small=True)}   ",
              end="", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="100.72.77.83")
    parser.add_argument("--port", type=int, default=9392)
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--urdf", default=None, help="URDF (default: the hardware config's)")
    parser.add_argument("--no-drive", dest="drive", action="store_false", help="ignore the twist (fixed base)")
    parser.add_argument("--base-frame", choices=("auto", "live", "frozen"), default="auto")
    parser.add_argument("--viz-hz", type=float, default=30.0)
    args = parser.parse_args()
    logging.basicConfig(level=logging.WARNING, format="\n%(levelname)s %(name)s: %(message)s")

    app = SimArmTeleop(args.hand, args.urdf, args.drive, args.base_frame, args.viz_hz)
    with QuestClient(args.host, args.port) as quest:
        result = TeleopRunner(app, quest).run()
    print(f"{result.kind}: {result.reason}; {result.stats}")
    raise SystemExit(0 if result.ok or result.kind == "estop" else 1)


if __name__ == "__main__":
    main()
