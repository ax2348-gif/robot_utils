"""python -m robot_utils.nodes.orbbec [--name camera] [--serial SN] [--streams color,depth] [--fake | --sim world[=NAME|DIR]] [--port 8101]

``--sim world`` takes the images from the MuJoCo sim world (the sim node ``sim``; ``world=NAME`` another instance,
``world=DIR`` its socket directory; ``--sim-camera`` picks the world's camera, default the instance name; docs/sim.md).
Serves on the unix socket <runtime dir>/robot/<name>.sock, i.e. http://<robot>/nodes/<name>/
through the robot's port-80 proxy; --port adds a TCP port for debugging (docs/nodes.md)."""
import argparse
import logging

from ..common.serve import add_serve_args, serve


def main(argv=None):
    from .models import CAMERA_MODES, MODE_DEFAULT_STREAMS, STREAM_NAMES, Settings, ImuSettings
    p = argparse.ArgumentParser(prog="python -m robot_utils.nodes.orbbec",
                                description="HTTP node for the Orbbec depth camera (docs/orbbec_node.md).")
    p.add_argument("--serial", default=None,
                   help="open only the camera with this serial number (default: the first one the SDK lists, "
                        "which is arbitrary with several cameras attached)")
    p.add_argument("--camera-mode", default="depth", choices=list(CAMERA_MODES),
                   help="depth (default): color + depth + the gray stereo pair; dual_color: both stereo sensors in "
                        "colour (color = left, color_right = right), no depth, on devices that offer it (Gemini 305)")
    p.add_argument("--streams", default=None, help="video streams on at start: " + ",".join(STREAM_NAMES)
                   + " (default: the camera mode's, color,depth or color,color_right)")
    p.add_argument("--no-imu", action="store_true", help="start with the IMU off")
    p.add_argument("--depth-filter", default="hardware", choices=["hardware", "software", "both", "off", "device"],
                   help="depth noise filter set on the device at start (default hardware: the camera's own, no host "
                        "CPU; software: the SDK's, on the host; device: leave as is)")
    p.add_argument("--fake", action="store_true", help="synthetic camera (no SDK / hardware needed)")
    p.add_argument("--sim", default=None, metavar="world[=NAME|DIR]",
                   help="images rendered by the sim world (docs/sim.md); no SDK / hardware needed")
    p.add_argument("--sim-camera", default=None, help="with --sim: the world's camera (default: --name)")
    p.add_argument("--imu-calibration-file", default=None,
                   help="where IMU calibrations are kept, per camera serial (default "
                        "~/.config/robot_utils/orbbec_imu.json; none with --fake); 'none' disables")
    p.add_argument("--encode-workers", type=int, default=3, help="threads for JPEG/zlib encoding")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "camera")
    a = p.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    streams = [s for s in a.streams.split(",") if s] if a.streams is not None else list(MODE_DEFAULT_STREAMS[a.camera_mode])
    bad = set(streams) - set(STREAM_NAMES)
    if bad:
        p.error(f"unknown stream(s) {', '.join(sorted(bad))}")

    from .hub import Hub
    from .owner import Owner
    from .server import create_app
    if a.sim:
        from ..sim.backends import world_socket
        from ..sim.backends.camera import WorldCameraBackend
        cam = a.sim_camera or a.name
        try:
            backend = WorldCameraBackend(camera=cam, socket_path=world_socket(a.sim, f"camera-{cam}"))
        except ValueError as e:
            p.error(str(e))
    elif a.fake:
        from .fake_backend import FakeBackend
        backend = FakeBackend(dual=True)
    else:
        from .sdk_backend import SdkBackend
        backend = SdkBackend(serial=a.serial)
    settings = Settings(camera_mode=a.camera_mode, streams={n: n in streams for n in STREAM_NAMES},
                        imu=ImuSettings(enabled=not a.no_imu), depth_filter=a.depth_filter)
    hub = Hub()
    from .owner import IMU_CALIBRATION_FILE
    cal_file = a.imu_calibration_file or (None if a.fake or a.sim else IMU_CALIBRATION_FILE)
    if cal_file and cal_file.lower() == "none":
        cal_file = None
    app = create_app(Owner(backend, hub, settings, calibration_file=cal_file), hub, encode_workers=a.encode_workers)
    # exactly one worker: the process owns the camera. Streams never end, so keep the graceful
    # shutdown short.
    serve(app, a, banner=f"Orbbec camera node {a.name!r}", log_level="debug" if a.verbose else "info",
          access_log=a.verbose, timeout_keep_alive=60, timeout_graceful_shutdown=2)


if __name__ == "__main__":
    main()
