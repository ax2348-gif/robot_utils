"""On the Pi: per camera stream, t_capture - t_sensor (SDK receipt on the host minus the device's own timestamp
mapped to host time), t_publish - t_capture (node), and the frame interval, from 15 s of raw frame headers."""
import json, os, statistics, sys, threading, time, urllib.request
sys.path.insert(0, os.path.expanduser("~/robot_utils/src"))
from robot_utils.nodes.common.protocol import RecordReader
SEC = 15
out = {}
def go(node, streams):
    rr = RecordReader(); d = {}
    t_end = time.monotonic() + SEC
    with urllib.request.urlopen(f"http://127.0.0.1/nodes/{node}/stream?streams={streams}&mode=raw", timeout=10) as r:
        while time.monotonic() < t_end:
            for h, _ in rr.feed(r.read1(1 << 20)):
                if h.get("type") != "frame":
                    continue
                s = d.setdefault(h["stream"], {"lag": [], "pub": [], "ts": []})
                if h.get("t_sensor") is not None:
                    s["lag"].append(h["t_capture"] - h["t_sensor"])
                if h.get("t_publish") is not None:
                    s["pub"].append(h["t_publish"] - h["t_capture"])
                s["ts"].append(h.get("t_sensor") or h["t_capture"])
    out[node] = d
ths = [threading.Thread(target=go, args=a) for a in (("camera", "color,depth"), ("wrist_camera", "color"))]
[t.start() for t in ths]; [t.join() for t in ths]
q = lambda a, p: round(1e3 * sorted(a)[min(len(a) - 1, int(p / 100 * len(a)))], 1) if a else None
for node, d in out.items():
    for st, s in d.items():
        iv = [b - a for a, b in zip(s["ts"], s["ts"][1:])]
        print(f"{node:13s} {st:6s} n={len(s['ts']):4d}  capture-sensor ms p5/p50/p95/max {q(s['lag'],5)} {q(s['lag'],50)} {q(s['lag'],95)} {q(s['lag'],100)}"
              f"  | node capture->publish p50/p95 {q(s['pub'],50)} {q(s['pub'],95)}  | frame interval p50 {q(iv,50)}")
