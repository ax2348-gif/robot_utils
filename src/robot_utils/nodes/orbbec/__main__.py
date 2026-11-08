"""python -m robot_utils.nodes.orbbec [--name camera] [--streams color,depth] [--fake] [--port 8101]

Serves on the unix socket <runtime dir>/robot/<name>.sock, i.e. http://<robot>/nodes/<name>/
through the robot's port-80 proxy; --port adds a TCP port for debugging (docs/nodes.md)."""
import argparse
import logging

from ..common.serve import add_serve_args, serve


def main(argv=None):
    from .models import STREAM_NAMES, Settings, ImuSettings
    p = argparse.ArgumentParser(prog="python -m robot_utils.nodes.orbbec",
                                description="HTTP node for the Orbbec depth camera (docs/orbbec_node.md).")
    p.add_argument("--streams", default="color,depth", help="video streams on at start: " + ",".join(STREAM_NAMES))
    p.add_argument("--no-imu", action="store_true", help="start with the IMU off")
    p.add_argument("--fake", action="store_true", help="synthetic camera (no SDK / hardware needed)")
    p.add_argument("--imu-calibration-file", default=None,
                   help="where IMU calibrations are kept, per camera serial (default "
                        "~/.config/robot_utils/orbbec_imu.json; none with --fake); 'none' disables")
    p.add_argument("--encode-workers", type=int, default=3, help="threads for JPEG/zlib encoding")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "camera")
    a = p.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    streams = [s for s in a.streams.split(",") if s]
    bad = set(streams) - set(STREAM_NAMES)
    if bad:
        p.error(f"unknown stream(s) {', '.join(sorted(bad))}")

    from .hub import Hub
    from .owner import Owner
    from .server import create_app
    if a.fake:
        from .fake_backend import FakeBackend
        backend = FakeBackend()
    else:
        from .sdk_backend import SdkBackend
        backend = SdkBackend()
    settings = Settings(streams={n: n in streams for n in STREAM_NAMES}, imu=ImuSettings(enabled=not a.no_imu))
    hub = Hub()
    from .owner import IMU_CALIBRATION_FILE
    cal_file = a.imu_calibration_file or (None if a.fake else IMU_CALIBRATION_FILE)
    if cal_file and cal_file.lower() == "none":
        cal_file = None
    app = create_app(Owner(backend, hub, settings, calibration_file=cal_file), hub, encode_workers=a.encode_workers)
    # exactly one worker: the process owns the camera. Streams never end, so keep the graceful
    # shutdown short.
    serve(app, a, banner=f"Orbbec camera node {a.name!r}", log_level="debug" if a.verbose else "info",
          access_log=a.verbose, timeout_keep_alive=60, timeout_graceful_shutdown=2)


if __name__ == "__main__":
    main()
