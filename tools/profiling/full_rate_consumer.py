#!/usr/bin/env python3
"""One efficient remote consumer of every robot stream at full rate (run on the Mac, like a recorder would):

  python3 full_rate_consumer.py SECONDS [DEPTH: raw|zlib] [OUT.json]

Opens each source stream once, through the robot's nginx proxy over the network, in its cheapest form:
  camera        stream?streams=color,depth&mode=raw[&compress=zlib]   colour = the camera's MJPG, depth Y16
  wrist_camera  stream?streams=color,depth&mode=raw[&compress=zlib]
  camera        imu/stream?raw=true                                    every 1 kHz sample
  arm, swerve, estimator, pose_servo   stream                          every row (no max_hz)
and reads them as fast as they come, parsing only the record headers (robot_utils' RecordReader). Per stream it
reports records and frames per second, MB/s, the frames the server skipped for this client (the camera stream is
latest-frame: `dropped` counts frames it could not deliver in time), sequence gaps, the largest gap between
arrivals, and for camera frames the delivery latency (arrival - t_capture, with each camera node's clock synced
through GET /time). Discards the data. Read-only.
"""
import json, os, statistics, sys, threading, time, urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src"))
from robot_utils.nodes.common.protocol import RecordReader  # noqa: E402

HOST = os.environ.get("ROBOT_NODES", "http://botcore-desktop.local/nodes")   # robot LAN (mDNS), not Tailscale
SEC = float(sys.argv[1]) if len(sys.argv) > 1 else 60
DEPTH = sys.argv[2] if len(sys.argv) > 2 else "zlib"
OUT = sys.argv[3] if len(sys.argv) > 3 else None
CZ = "&compress=zlib" if DEPTH == "zlib" else ""
STREAMS = {
    "camera": f"camera/stream?streams=color,depth&mode=raw{CZ}",
    "wrist_camera": f"wrist_camera/stream?streams=color,depth&mode=raw{CZ}",
    "camera_imu": "camera/imu/stream?raw=true",
    "arm": "arm/stream", "swerve": "swerve/stream", "estimator": "estimator/stream", "pose_servo": "pose_servo/stream",
}
stop = threading.Event()
stats: dict = {}


def clock_offset(node):
    """Pi monotonic - local monotonic, from the best of 15 GET /time round trips (NTP style)."""
    best = None
    for _ in range(15):
        t0 = time.monotonic()
        with urllib.request.urlopen(f"{HOST}/{node}/time", timeout=5) as r:
            mono = json.loads(r.read())["mono"]
        t1 = time.monotonic()
        if best is None or t1 - t0 < best[0]:
            best = (t1 - t0, mono - (t0 + t1) / 2)
    return best[1], best[0]


def consume(name, path, offset):
    s = stats[name] = {"bytes": 0, "records": 0, "frames": {}, "dropped": {}, "gaps": {}, "lat": {}, "max_gap_s": 0.0,
                       "errors": 0, "rows": 0}
    last_seq, last_arrival = {}, None
    while not stop.is_set():
        try:
            with urllib.request.urlopen(f"{HOST}/{path}", timeout=10) as r:
                rr = RecordReader()
                while not stop.is_set():
                    chunk = r.read1(1 << 20)
                    if not chunk:
                        break
                    now = time.monotonic()
                    s["bytes"] += len(chunk)
                    if last_arrival is not None:
                        s["max_gap_s"] = max(s["max_gap_s"], now - last_arrival)
                    last_arrival = now
                    for h, payload in rr.feed(chunk):
                        s["records"] += 1
                        typ = h.get("type")
                        if typ == "frame":
                            st = h.get("stream", "?")
                            s["frames"][st] = s["frames"].get(st, 0) + 1
                            s["dropped"][st] = s["dropped"].get(st, 0) + int(h.get("dropped") or 0)
                            q = h.get("seq")
                            if q is not None and st in last_seq and q > last_seq[st] + 1:
                                s["gaps"][st] = s["gaps"].get(st, 0) + (q - last_seq[st] - 1)
                            if q is not None:
                                last_seq[st] = q
                            if offset is not None and h.get("t_capture") is not None:
                                s["lat"].setdefault(st, []).append(now + offset - h["t_capture"])
                        elif typ == "rows":
                            s["rows"] += int(h.get("n") or h.get("count") or 0) or 1
        except Exception as e:  # noqa: BLE001
            s["errors"] += 1
            s["last_error"] = f"{type(e).__name__}: {e}"[:120]
            stop.wait(1.0)


def pct(a, q):
    a = sorted(a)
    return round(1e3 * a[min(len(a) - 1, int(q / 100 * len(a)))], 1) if a else None


def main():
    offsets = {}
    for node in ("camera", "wrist_camera"):
        off, rtt = clock_offset(node)
        offsets[node] = off
        print(f"{node} clock offset {off:.4f} s (best round trip {rtt * 1e3:.1f} ms)", flush=True)
    ths = [threading.Thread(target=consume, args=(n, p, offsets.get(n if n in offsets else None)), daemon=True)
           for n, p in STREAMS.items()]
    t0 = time.monotonic()
    for t in ths:
        t.start()
    while time.monotonic() - t0 < SEC:
        time.sleep(1)
    stop.set()
    dt = time.monotonic() - t0
    report = {"seconds": round(dt, 1), "depth": DEPTH, "streams": {}}
    total = 0
    for n, s in stats.items():
        total += s["bytes"]
        r = {"MB_s": round(s["bytes"] / dt / 1e6, 2), "records_s": round(s["records"] / dt, 1),
             "max_arrival_gap_ms": round(1e3 * s["max_gap_s"], 1), "errors": s["errors"]}
        if s["frames"]:
            r["fps"] = {st: round(c / dt, 1) for st, c in s["frames"].items()}
            r["server_skipped"] = s["dropped"]
            r["seq_gaps"] = s["gaps"]
            r["latency_ms_p50_p95_p99_max"] = {st: [pct(v, 50), pct(v, 95), pct(v, 99), pct(v, 100)]
                                               for st, v in s["lat"].items()}
        if s.get("last_error"):
            r["last_error"] = s["last_error"]
        report["streams"][n] = r
    report["total_MB_s"] = round(total / dt / 1e6, 2)
    print(json.dumps(report, indent=1), flush=True)
    if OUT:
        json.dump(report, open(OUT, "w"), indent=1)


if __name__ == "__main__":
    main()
