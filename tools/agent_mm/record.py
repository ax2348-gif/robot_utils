#!/usr/bin/env python3
"""Record an agent episode's high-rate data on the client (the robot framework's recording model).

  python3 record.py RUN_DIR [--arm-hz 50] [--est-hz 100] [--swerve-hz 100] [--cam-hz 1] [--cameras camera,wrist_camera] [--no-cam]
  (run in the background for the whole episode; Ctrl-C / SIGTERM stops it cleanly)

Writes into RUN_DIR/streams/:
  arm.bin, estimator.bin,  raw bytes of GET /stream: the framework's recording format. Read back with
  swerve.bin               robot_utils.nodes.common.protocol.iter_rows(iter_records(open(f, "rb"))).
                           swerve.bin has every module's per-motor frame times (t_steer, t_drive, t_*_s0), so
                           per-motor reply gaps can be checked; --swerve-hz 0 leaves it out.
                           A reconnect appends a new schema record, which is still valid.
  cam/NNNNNN_depth.png     lossless 16-bit depth (mm), polled at --cam-hz
  cam/NNNNNN_color.jpg     colour frame taken right after
  cam/frames.jsonl         one line per frame: wall time + the camera's X-Frame-Header (t_capture, seq, ...)
  cam_<name>/ ...          the same for each other camera in --cameras (e.g. cam_wrist_camera/), with
                           calibration_<name>.json; the head camera keeps cam/ and calibration.json
Restarting into the same RUN_DIR continues it: streams append, frame numbers continue, meta.json
gains a "restarts" entry.
  calibration.json, arm_fields.json, estimator_fields.json, swerve_fields.json, meta.json
Depth + calibration (+ estimator pose at t_capture) gives ego point clouds offline.
Read-only: it never commands the robot.
"""
import argparse, json, os, signal, threading, time, urllib.request

HOST = os.environ.get("ROBOT_NODES", "http://botcore-desktop.local/nodes")   # robot LAN (mDNS), not Tailscale
stop = threading.Event()


def get_json(path):
    with urllib.request.urlopen(f"{HOST}/{path}", timeout=10) as r:
        return json.load(r)


def stream_to_file(node, params, out, stats):
    """Append the raw stream bytes; reconnect on any error until stopped."""
    url = f"{HOST}/{node}/stream?" + "&".join(f"{k}={v}" for k, v in params.items())
    with open(out, "ab") as f:
        while not stop.is_set():
            try:
                with urllib.request.urlopen(url, timeout=10) as r:
                    while not stop.is_set():
                        chunk = r.read1(65536)
                        if not chunk:
                            break
                        f.write(chunk)
                        stats[node] = stats.get(node, 0) + len(chunk)
            except Exception as e:
                stats[node + "_errors"] = stats.get(node + "_errors", 0) + 1
                stats[node + "_last_error"] = repr(e)
            f.flush()
            stop.wait(1.0)


def cam_dir_of(out, cam):
    return os.path.join(out, "cam" if cam == "camera" else f"cam_{cam}")


def poll_camera(cam, cam_dir, hz, stats):
    os.makedirs(cam_dir, exist_ok=True)
    index = os.path.join(cam_dir, "frames.jsonl")
    n = sum(1 for _ in open(index)) if os.path.exists(index) else 0   # a restart continues the numbering
    idx = open(index, "a")
    last_seq, key = None, "cam" if cam == "camera" else cam
    while not stop.is_set():
        t0 = time.time()
        try:
            with urllib.request.urlopen(f"{HOST}/{cam}/snapshot/depth?format=png", timeout=10) as r:
                hdr = json.loads(r.headers.get("x-frame-header") or "{}")
                png = r.read()
            if hdr.get("seq") != last_seq:          # skip a frame we already have
                last_seq = hdr.get("seq")
                open(os.path.join(cam_dir, f"{n:06d}_depth.png"), "wb").write(png)
                with urllib.request.urlopen(f"{HOST}/{cam}/snapshot/color?format=jpg", timeout=10) as r:
                    chdr = json.loads(r.headers.get("x-frame-header") or "{}")
                    open(os.path.join(cam_dir, f"{n:06d}_color.jpg"), "wb").write(r.read())
                idx.write(json.dumps({"n": n, "t_wall": t0, "depth": hdr, "color": chdr}) + "\n")
                idx.flush()
                n += 1
                stats[key + "_frames"] = n
        except Exception as e:
            stats[key + "_errors"] = stats.get(key + "_errors", 0) + 1
            stats[key + "_last_error"] = repr(e)
        stop.wait(max(0.0, 1.0 / hz - (time.time() - t0)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--arm-hz", type=float, default=50)
    ap.add_argument("--est-hz", type=float, default=100)
    ap.add_argument("--swerve-hz", type=float, default=100, help="swerve /stream rate (its loop is 100 Hz); 0 = do not record it")
    ap.add_argument("--cam-hz", type=float, default=1.0)
    ap.add_argument("--cameras", default="camera,wrist_camera",
                    help="camera nodes to poll at --cam-hz (comma separated); `camera` (head) -> cam/, others -> cam_<name>/")
    ap.add_argument("--no-cam", action="store_true")
    a = ap.parse_args()
    out = os.path.join(a.run_dir, "streams")
    os.makedirs(out, exist_ok=True)
    cams = [] if a.no_cam else [c for c in a.cameras.split(",") if c]
    for name, path in [("calibration" if c == "camera" else f"calibration_{c}", f"{c}/calibration") for c in cams] + [
            ("arm_fields", "arm/fields"), ("estimator_fields", "estimator/fields")] + (
            [("swerve_fields", "swerve/fields")] if a.swerve_hz > 0 else []):
        try:
            json.dump(get_json(path), open(os.path.join(out, name + ".json"), "w"), indent=1)
        except Exception as e:
            print(f"{path}: {e}", flush=True)
    t = get_json("estimator/time")
    run = {"started_wall": time.time(), "estimator_epoch_minus_mono": t["epoch_minus_mono"], "args": vars(a)}
    meta_path = os.path.join(out, "meta.json")
    if os.path.exists(meta_path):                    # a restart: keep the first start, list the restarts
        meta = json.load(open(meta_path))
        meta.setdefault("restarts", []).append(run)
    else:
        meta = run
    json.dump(meta, open(meta_path, "w"), indent=1)

    stats = {}
    threads = [threading.Thread(target=stream_to_file, args=("arm", {"max_hz": a.arm_hz}, os.path.join(out, "arm.bin"), stats)),
               threading.Thread(target=stream_to_file, args=("estimator", {"max_hz": a.est_hz}, os.path.join(out, "estimator.bin"), stats))]
    if a.swerve_hz > 0:
        threads.append(threading.Thread(target=stream_to_file, args=("swerve", {"max_hz": a.swerve_hz},
                                                                      os.path.join(out, "swerve.bin"), stats)))
    for c in cams:
        threads.append(threading.Thread(target=poll_camera, args=(c, cam_dir_of(out, c), a.cam_hz, stats)))
    for th in threads:
        th.daemon = True
        th.start()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        while not stop.is_set():
            stop.wait(10)
            print(time.strftime("%H:%M:%S"), json.dumps(stats), flush=True)
    except KeyboardInterrupt:
        stop.set()
    print("stopped", json.dumps(stats), flush=True)


if __name__ == "__main__":
    main()
