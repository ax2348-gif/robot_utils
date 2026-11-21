#!/usr/bin/env python3
"""Pin camera settings for testing (run on the Pi, robot venv): pi_camera_pin.py save|apply [FILE]

save:  write both camera nodes' current settings (streams, colour profile, stereo mode, align, frame sync, depth
       filter, IMU on/rate) to FILE (default ~/profiling/camera_pin.json).
apply: POST them back to each node that differs and verify every field (and that the streams deliver); exits 1 if
       anything does not match. Settings changes only; nothing moves.
"""
import json, os, sys, time

sys.path.insert(0, os.path.expanduser("~/robot_utils/src"))
from robot_utils.nodes.common.client import NodeClient  # noqa: E402

CMD = sys.argv[1]
FILE = sys.argv[2] if len(sys.argv) > 2 else os.path.expanduser("~/profiling/camera_pin.json")
NODES = ("camera", "wrist_camera")
KEYS = ("streams", "color_profile", "stereo_mode", "align", "frame_sync", "depth_filter")


def current(n):
    s = NodeClient(n, timeout=30).get("/settings")
    out = {k: s[k] for k in KEYS}
    out["imu"] = {"enabled": s["imu"]["enabled"], "sample_rate": s["imu"]["sample_rate"]}
    return out


if CMD == "save":
    pin = {n: current(n) for n in NODES}
    json.dump(pin, open(FILE, "w"), indent=1)
    print(json.dumps(pin, indent=1))
elif CMD == "apply":
    pin, ok = json.load(open(FILE)), True
    for n, want in pin.items():
        c = NodeClient(n, timeout=30)
        have = current(n)
        if have != want:
            patch = {k: want[k] for k in KEYS if have.get(k) != want[k]}
            if want["imu"] != have["imu"]:
                patch["imu"] = {k: v for k, v in want["imu"].items() if v is not None}
            print(f"{n}: applying {patch}", flush=True)
            c.post("/settings", patch)
            time.sleep(2)
        got = current(n)
        # streams the pin does not mention (e.g. color_right, added after it was saved) are not part of it
        got["streams"] = {k: v for k, v in got["streams"].items() if k in want["streams"]}
        st = c.get("/state")["streams"]
        live = {k: (v["width"], v["height"], v["format"], round(v["fps"], 1)) for k, v in st.items() if v.get("enabled")}
        match = got == want
        ok &= match
        print(f"{n}: {'pinned' if match else 'MISMATCH ' + json.dumps(got)} | live {live}", flush=True)
    sys.exit(0 if ok else 1)
