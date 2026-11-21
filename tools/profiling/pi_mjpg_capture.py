#!/usr/bin/env python3
"""MJPG vs uncompressed colour from one camera node (on the Pi, robot venv): pi_mjpg_capture.py NODE OUT_DIR [N]

Sets the node's colour profile to each of PROFILES in turn, waits for it to settle, then saves N raw colour frames
(mode=raw: the camera's own MJPG bytes untouched, or the uncompressed pixels as captured) with their headers. Restores
the previous colour profile. Also saves the colour image controls (GET /properties). Analysis happens elsewhere.
"""
import json, os, sys, time, urllib.request

sys.path.insert(0, os.path.expanduser("~/robot_utils/src"))
from robot_utils.nodes.common.protocol import RecordReader  # noqa: E402

NODE, OUT = sys.argv[1], sys.argv[2]
N = int(sys.argv[3]) if len(sys.argv) > 3 else 10
PROFILES = ["1280x800@30 MJPG", "1280x800@30 YUYV"]
BASE = f"http://127.0.0.1/nodes/{NODE}"
os.makedirs(OUT, exist_ok=True)


def api(path, body=None):
    req = urllib.request.Request(BASE + path, data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"}, method="GET" if body is None else "POST")
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read())


def grab(tag, n):
    rr, got = RecordReader(), 0
    with urllib.request.urlopen(f"{BASE}/stream?streams=color&mode=raw", timeout=10) as r:
        skip = 15                                    # let exposure / white balance settle after the restart
        while got < n:
            for h, payload in rr.feed(r.read1(1 << 20)):
                if h.get("type") != "frame" or h.get("stream") != "color":
                    continue
                if skip:
                    skip -= 1
                    continue
                ext = "jpg" if h.get("format") == "MJPG" or h.get("encoding") == "jpeg" else "raw"
                with open(os.path.join(OUT, f"{tag}_{got:02d}.{ext}"), "wb") as f:
                    f.write(bytes(payload))
                with open(os.path.join(OUT, f"{tag}_{got:02d}.json"), "w") as f:
                    json.dump(h, f)
                got += 1
                if got >= n:
                    break


prev = api("/state")["settings"]["color_profile"]
try:
    for prof in PROFILES:
        api("/settings", {"color_profile": prof})
        time.sleep(2)
        grab(prof.split()[1].lower(), N)
        print("captured", prof, flush=True)
    with open(os.path.join(OUT, "properties.json"), "w") as f:
        json.dump(api("/properties?all=true"), f, indent=1)
finally:
    api("/settings", {"color_profile": prev})
    print("restored", prev, flush=True)
