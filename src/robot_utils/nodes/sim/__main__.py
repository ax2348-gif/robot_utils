"""python -m robot_utils.nodes.sim [--model description|URL|bundle.zip|local] [--robot NAME] [--config FILE] [--paused]
                                [--rtf X]

Runs the MuJoCo sim world (docs/sim.md): the robot model from the description node (or assembled locally),
the floor, scene objects and rendered cameras. The arm, swerve and camera nodes attach to it through their
backends over Unix sockets in <runtime dir>/robot/sim/<name>/. Serves its page and API on
<runtime dir>/robot/<name>.sock = http://<host>/nodes/<name>/; --port is for debugging. Never on the robot.
"""
import argparse
import logging
import os

from ..common.serve import add_serve_args, serve


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--model", default="description",
                   help="description node (instance name or URL), a bundle .zip, or 'local' (default: description)")
    p.add_argument("--config", default=os.environ.get("ROBOT_SIM_CONFIG") or None,
                   help="sim config YAML merged over default_sim.yaml (default: $ROBOT_SIM_CONFIG if set)")
    p.add_argument("--robot", default=None,
                   help="robot profile (robots/): its sim config, and the model with --model local (default $ROBOT_UTILS_ROBOT, "
                        "else rebot); with a description node the model and its arms are that node's")
    p.add_argument("--variant", default=None, help="model variant: full | simple (default: the config's)")
    p.add_argument("--paused", action="store_true", help="start paused (advance with POST /step)")
    p.add_argument("--rtf", type=float, default=None, help="real-time factor (default 1)")
    p.add_argument("--scene-dir", default=None,
                   help="where POST /scene/save writes (default ~/.local/share/robot_utils/sim)")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "sim")
    a = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("httpx").setLevel(logging.WARNING)
    from .node import SimNode, world_from
    from .server import create_app
    from .world import load_config
    from ...robots import load as load_robot
    robot = load_robot(a.robot)
    over: dict = {"time": {}}
    if a.paused:
        over["time"]["run"] = False
    if a.rtf is not None:
        over["time"]["rtf"] = a.rtf
    if a.variant:
        over["model"] = {"variant": a.variant}
    world = world_from(a.model, load_config(a.config, over, robot.sim), robot.name)
    node = SimNode(world, a.name, a.scene_dir)
    serve(create_app(node), a, banner=f"sim node {a.name!r} (robot {robot.name}; model {world.model_info.get('source')}; "
                                      f"arms {', '.join(world.arm_ix) or 'none'})")


if __name__ == "__main__":
    main()
