"""python -m robot_utils.nodes.orbbec [--host 0.0.0.0] [--port 8000] [--streams color,depth] [--fake]"""
import argparse
import logging
import socket
import subprocess


def lan_addresses():
    try:
        addrs = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=2).stdout.split()
    except Exception:
        addrs = []
    return [a for a in addrs if ":" not in a] + [socket.gethostname()]


def main(argv=None):
    from .models import STREAM_NAMES, Settings, ImuSettings
    p = argparse.ArgumentParser(prog="python -m robot_utils.nodes.orbbec",
                                description="HTTP node for the Orbbec depth camera (docs/orbbec_node.md).")
    p.add_argument("--host", default="0.0.0.0", help="address to bind (default: all interfaces)")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--streams", default="color,depth", help="video streams on at start: " + ",".join(STREAM_NAMES))
    p.add_argument("--no-imu", action="store_true", help="start with the IMU off")
    p.add_argument("--fake", action="store_true", help="synthetic camera (no SDK / hardware needed)")
    p.add_argument("--imu-calibration-file", default=None,
                   help="where IMU calibrations are kept, per camera serial (default "
                        "~/.config/robot_utils/orbbec_imu.json; none with --fake); 'none' disables")
    p.add_argument("--encode-workers", type=int, default=3, help="threads for JPEG/zlib encoding")
    p.add_argument("-v", "--verbose", action="store_true")
    a = p.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    streams = [s for s in a.streams.split(",") if s]
    bad = set(streams) - set(STREAM_NAMES)
    if bad:
        p.error(f"unknown stream(s) {', '.join(sorted(bad))}")

    import uvicorn
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
    hosts = lan_addresses() if a.host in ("0.0.0.0", "") else [a.host]
    for h in hosts:
        print(f"Orbbec node: http://{h}:{a.port}/   (API docs: http://{h}:{a.port}/docs)", flush=True)
    # exactly one worker: the process owns the camera. Streams never end, so keep the graceful
    # shutdown short.
    uvicorn.run(app, host=a.host, port=a.port, workers=1, log_level="debug" if a.verbose else "info",
                access_log=a.verbose, timeout_keep_alive=60, timeout_graceful_shutdown=2)


if __name__ == "__main__":
    main()
