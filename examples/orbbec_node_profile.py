#!/usr/bin/env python3
"""Latency profile of the Orbbec node: capture -> client receive, per frame, over a settings matrix.

    PYTHONPATH=src python examples/orbbec_node_profile.py --url http://100.80.153.120:8000 \
        --duration 12 --out results_tailscale.json

For every case it (1) applies the settings (POST /settings round trip; time until the first frame
captured with the new settings arrives), (2) syncs the clock (GET /time), (3) streams for
``--duration`` s with 1 or 2 clients and reports per stream: frames received vs camera fps,
dropped, latency median / p95 / p99 and the median breakdown

    sensor->capture  t_capture - t_sensor   device timestamp (SDK-mapped to host) to SDK receipt:
                                            exposure/readout end + on-camera processing + USB
    capture->owner   t_owner - t_capture    waiting in the SDK pipeline (frame aggregation / sync)
    owner->publish   t_publish - t_owner    copy, D2C alignment
    publish->encoded t_encoded - t_publish  JPEG / zlib encode (0 for passthrough)
    encoded->send    t_send - t_encoded     waiting for the client's previous send to drain
    send->recv       t_recv - t_send        network + client read (client clock mapped to server)

plus the Pi's CPU (node process, % of one core) from GET /state. IMU cases report per-sample
latency and achieved rate. Writes JSON and prints a markdown table.
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from robot_utils.nodes.orbbec.client import OrbbecClient  # noqa: E402

DEF_COLOR, DEF_STEREO = "1280x720@10 MJPG", "848x480@10"
ALL = {"color": True, "depth": True, "ir_left": True, "ir_right": True}
ONLY = lambda *n: {k: k in n for k in ALL}  # noqa: E731
BASE = {"color_profile": DEF_COLOR, "stereo_mode": DEF_STEREO, "align": False, "frame_sync": False,
        "display": {"jpeg_quality": 80}, "imu": {"enabled": False}}


def case(name, streams_on, query, mode="jpeg", compress=None, clients=1, **settings):
    s = json.loads(json.dumps(BASE))
    s["streams"] = streams_on
    for k, v in settings.items():
        if k == "jpeg_quality":
            s["display"]["jpeg_quality"] = v
        else:
            s[k] = v
    return {"name": name, "settings": s, "query": query, "mode": mode, "compress": compress, "clients": clients}


CASES = [
    case("color 720p10", ONLY("color"), "color"),
    case("depth 848x480@10 jpeg", ONLY("depth"), "depth"),
    case("depth 848x480@10 raw", ONLY("depth"), "depth", "raw"),
    case("depth 848x480@10 raw+zlib", ONLY("depth"), "depth", "raw", "zlib"),
    case("color+depth defaults jpeg", ONLY("color", "depth"), "color,depth"),
    case("color+depth defaults raw", ONLY("color", "depth"), "color,depth", "raw"),
    case("color+depth defaults jpeg q50", ONLY("color", "depth"), "color,depth", jpeg_quality=50),
    case("color+depth defaults jpeg q95", ONLY("color", "depth"), "color,depth", jpeg_quality=95),
    case("color+depth defaults jpeg 2 clients", ONLY("color", "depth"), "color,depth", clients=2),
    case("color+depth defaults raw 2 clients", ONLY("color", "depth"), "color,depth", "raw", clients=2),
    case("color+depth defaults frame_sync jpeg", ONLY("color", "depth"), "color,depth", frame_sync=True),
    case("color+depth aligned jpeg", ONLY("color", "depth"), "color,depth", align=True),
    case("color+depth aligned raw", ONLY("color", "depth"), "color,depth", "raw", align=True),
    case("all four defaults jpeg", ALL, "color,depth,ir_left,ir_right"),
    case("all four defaults raw", ALL, "color,depth,ir_left,ir_right", "raw"),
    case("color+depth 30fps jpeg", ONLY("color", "depth"), "color,depth",
         color_profile="640x480@30 MJPG", stereo_mode="640x400@30"),
    case("color+depth 30fps raw", ONLY("color", "depth"), "color,depth", "raw",
         color_profile="640x480@30 MJPG", stereo_mode="640x400@30"),
    case("all four 30fps jpeg", ALL, "color,depth,ir_left,ir_right",
         color_profile="640x480@30 MJPG", stereo_mode="640x400@30"),
    case("depth 480x270@60 jpeg", ONLY("depth"), "depth", stereo_mode="480x270@60"),
    case("depth 480x270@60 raw", ONLY("depth"), "depth", "raw", stereo_mode="480x270@60"),
]
IMU_CASES = [("imu 200Hz", "200Hz"), ("imu 1kHz", "1kHz")]


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float("nan")


def ms(x):
    return None if x is None or x != x else round(x * 1000, 1)


class CpuSampler(threading.Thread):
    def __init__(self, url):
        super().__init__(daemon=True)
        self.url, self.vals, self.stop, self.state = url, [], threading.Event(), None

    def run(self):
        with OrbbecClient(self.url) as c:
            while not self.stop.wait(1.5):
                try:
                    st = c.state()
                    self.state = st
                    self.vals.append((st["cpu"]["process_pct"], st["cpu"]["load1"]))
                except Exception:
                    pass


def run_clients(url, c, duration, warmup=1.5):
    """Each client: own connection + own clock sync; returns list of per-frame rows."""
    rows, lock, errors = [], threading.Lock(), []

    def one(k):
        try:
            with OrbbecClient(url) as cl:
                cl.sync()
                t_start = time.monotonic() + warmup
                t_end = t_start + duration
                for fr in cl.frames(c["query"], mode=c["mode"], compress=c["compress"], decode_jpeg=False):
                    if fr.t_recv >= t_start:
                        h = fr.header
                        with lock:
                            rows.append((k, h["stream"], h["seq"], h["t_capture"], h.get("t_sensor"), h["t_owner"],
                                         h["t_publish"], h["t_encoded"], h["t_send"], fr.t_recv_server, len(fr.payload),
                                         h.get("dropped", 0)))
                    if fr.t_recv > t_end:
                        break
        except Exception as e:
            errors.append(repr(e))

    ths = [threading.Thread(target=one, args=(k,), daemon=True) for k in range(c["clients"])]
    for t in ths:
        t.start()
    for t in ths:
        t.join(duration + warmup + 30)
    return rows, errors


def summarize(rows, duration, nclients, state):
    out = {}
    for s in sorted({r[1] for r in rows}):
        rs = [r for r in rows if r[1] == s]
        a = np.array([[np.nan if v is None else v for v in r[3:10]] for r in rs], dtype=np.float64)
        tc, tsen, tow, tpub, tenc, tsend, trecv = a.T
        lat = trecv - tc
        cam_fps = state["streams"][s]["fps"] if state else None
        out[s] = {
            "frames": len(rs), "fps_cam": cam_fps,
            "fps_recv": round((len(rs) / nclients - 1) / max(np.max(tc) - np.min(tc), 1e-6), 2),
            "dropped": int(sum(r[11] for r in rs)),
            "lat_med": ms(pct(lat, 50)), "lat_p95": ms(pct(lat, 95)), "lat_p99": ms(pct(lat, 99)),
            "lat_min": ms(float(np.min(lat))),
            "sensor_to_capture": ms(float(np.nanmedian(tc - tsen))) if np.isfinite(tsen).any() else None,
            "capture_to_owner": ms(float(np.median(tow - tc))),
            "owner_to_publish": ms(float(np.median(tpub - tow))),
            "publish_to_encoded": ms(float(np.median(tenc - tpub))),
            "encoded_to_send": ms(float(np.median(tsend - tenc))),
            "send_to_recv": ms(float(np.median(trecv - tsend))),
            "kbytes": round(statistics.mean(r[10] for r in rs) / 1024, 1),
            "mbit_s": round(sum(r[10] for r in rs) * 8 / duration / nclients / 1e6, 2),
        }
    return out


def run_imu(url, rate, duration, warmup=1.0):
    with OrbbecClient(url) as c:
        c.update_settings({"streams": ONLY(), "imu": {"enabled": True, "sample_rate": rate, "batch_ms": 5}})
        time.sleep(0.5)
        c.sync()
        lat, last_lat, sen, n, dropped, seqs = [], [], [], 0, 0, []
        t_start = time.monotonic() + warmup
        for b in c.imu():
            if b.t_recv < t_start:
                continue
            s = b.samples
            lat.extend(b.t_recv_server - s.t_capture)
            last_lat.append(b.t_recv_server - s.t_capture[-1])
            sen.extend(s.t_capture - s.t_sensor)
            n += len(s)
            dropped += b.header.get("dropped", 0)
            seqs.extend(s.seq)
            if b.t_recv > t_start + duration:
                break
        st = c.state()
        seqs = np.array(seqs)
        return {"samples": n, "rate_recv": round(n / duration, 1), "rate_server": st["imu"]["rate_hz"],
                "batches": len(last_lat), "dropped": dropped, "seq_gaps": int(np.sum(np.diff(seqs) - 1)) if n > 1 else 0,
                "device_gaps_total": st["imu"]["gaps"],
                "lat_med": ms(pct(lat, 50)), "lat_p95": ms(pct(lat, 95)), "lat_p99": ms(pct(lat, 99)),
                "newest_in_batch_lat_med": ms(pct(last_lat, 50)),
                "sensor_to_capture": ms(float(np.nanmedian(sen))) if np.isfinite(sen).any() else None,
                "cpu": st["cpu"]}


def sync_quality(url, bursts=5):
    with OrbbecClient(url) as c:
        ests = [c.sync(16) for _ in range(bursts)]
    return {"uncertainty_ms": [round(e.uncertainty * 1e3, 3) for e in ests],
            "rtt_min_ms": [round(e.rtt_min * 1e3, 3) for e in ests],
            "rtt_median_ms": [round(e.rtt_median * 1e3, 3) for e in ests],
            "offset_spread_ms": round((max(e.offset for e in ests) - min(e.offset for e in ests)) * 1e3, 3)}


def settings_timing(url, c):
    """POST round trip, and POST start -> first frame captured with the new settings received."""
    with OrbbecClient(url) as cl:
        cl.sync()
        stream = c["query"].split(",")[0]
        t0 = time.monotonic()
        r = cl.update_settings(c["settings"])
        t1 = time.monotonic()
        first = None
        if r["restarted"]:
            for fr in cl.frames(stream, mode="jpeg", decode_jpeg=False, read_timeout=15):
                if fr.header["t_capture"] >= r["t_applied"]:
                    first = fr.t_recv_server - cl.to_server(t0)
                    break
        return {"post_rtt_ms": round((t1 - t0) * 1e3, 1), "restarted": r["restarted"],
                "first_new_frame_ms": ms(first)}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--url", default="http://localhost:8000")
    p.add_argument("--label", default=None, help="name of this client location (for the table)")
    p.add_argument("--duration", type=float, default=12.0)
    p.add_argument("--only", default="", help="comma-separated substrings; run only matching cases")
    p.add_argument("--skip", default="", help="comma-separated substrings of cases to skip")
    p.add_argument("--no-imu", action="store_true")
    p.add_argument("--out", default=None)
    a = p.parse_args()
    label = a.label or a.url
    sel = [c for c in CASES if (not a.only or any(k in c["name"] for k in a.only.split(",")))
           and not any(k and k in c["name"] for k in a.skip.split(","))]
    results = {"url": a.url, "label": label, "started": time.time(), "sync": sync_quality(a.url), "cases": [],
               "imu": []}
    print(f"## {label}\nclock sync: {results['sync']}", flush=True)
    restore = OrbbecClient(a.url).settings()
    try:
        for c in sel:
            st = settings_timing(a.url, c)
            time.sleep(1.0)
            cpu = CpuSampler(a.url)
            cpu.start()
            rows, errors = run_clients(a.url, c, a.duration)
            cpu.stop.set()
            cpu.join(3)
            summ = summarize(rows, a.duration, c["clients"], cpu.state)
            cpu_pct = round(statistics.mean(v[0] for v in cpu.vals), 1) if cpu.vals else None
            res = {"case": c["name"], "mode": c["mode"], "compress": c["compress"], "clients": c["clients"],
                   "settings_change": st, "streams": summ, "cpu_pct": cpu_pct, "errors": errors}
            results["cases"].append(res)
            for s, v in summ.items():
                print(f"| {c['name']} | {s} | {v['fps_cam']} | {v['fps_recv']} | {v['dropped']} | {v['lat_med']} | "
                      f"{v['lat_p95']} | {v['lat_p99']} | {v['sensor_to_capture']} | {v['capture_to_owner']} | "
                      f"{v['owner_to_publish']} | {v['publish_to_encoded']} | {v['encoded_to_send']} | "
                      f"{v['send_to_recv']} | {v['kbytes']} | {v['mbit_s']} | {cpu_pct} |", flush=True)
            print(f"|   settings: {st} errors: {errors}", flush=True)
        if not a.no_imu:
            for name, rate in IMU_CASES:
                r = run_imu(a.url, rate, a.duration)
                r["case"] = name
                results["imu"].append(r)
                print(f"| {name} | {r}", flush=True)
    finally:
        try:
            with OrbbecClient(a.url) as c:
                c.update_settings({k: restore[k] for k in ("streams", "color_profile", "stereo_mode", "align",
                                                           "frame_sync", "display", "imu")})
        except Exception as e:
            print("could not restore settings:", e)
    if a.out:
        with open(a.out, "w") as f:
            json.dump(results, f, indent=1)


if __name__ == "__main__":
    main()
