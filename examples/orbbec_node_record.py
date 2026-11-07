#!/usr/bin/env python3
"""Record the Orbbec node's streams on THIS machine (the Pi has no disk to spare).

    PYTHONPATH=src python examples/orbbec_node_record.py --url http://100.80.153.120:8101 \
        --out rec_001 --streams color,depth --mode raw --duration 30
    PYTHONPATH=src python examples/orbbec_node_record.py --read rec_001      # summary + export

Writes <out>/video.rec, <out>/imu.rec (the wire records; each header gains t_recv = receive time
on the server clock, t_recv_client and sync_uncertainty) and <out>/meta.json (settings, catalog,
clock-sync history). Depth stays lossless (uint16, x depth_scale = mm), color stays the camera's
own MJPEG bytes. ``--read`` prints a summary and, with --export, writes depth PNGs / color JPEGs and
a timestamps CSV.
"""
import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from robot_utils.nodes.orbbec.client import read_recording, record  # noqa: E402


def summarize(out_dir, export=False):
    rows = []
    exp = os.path.join(out_dir, "export")
    if export:
        os.makedirs(exp, exist_ok=True)
    for h, data in read_recording(os.path.join(out_dir, "video.rec"), decode_jpeg=False):
        rows.append((h["stream"], h["seq"], h["t_sensor"], h["t_capture"], h.get("t_recv"), h["encoding"]))
        if export:
            name = f"{h['stream']}_{h['seq']:06d}"
            if isinstance(data, (bytes, bytearray)):            # MJPEG / JPEG: write as is
                open(os.path.join(exp, name + ".jpg"), "wb").write(data)
            else:                                               # raw depth/IR -> 16-bit/8-bit PNG
                import cv2
                cv2.imwrite(os.path.join(exp, name + ".png"), data)
    if export:
        with open(os.path.join(exp, "frames.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["stream", "seq", "t_sensor", "t_capture", "t_recv", "encoding"])
            w.writerows(rows)
    for s in sorted({r[0] for r in rows}):
        tc = np.array([r[3] for r in rows if r[0] == s])
        lat = np.array([r[4] - r[3] for r in rows if r[0] == s and r[4] is not None])
        print(f"{s:9s} {len(tc):5d} frames  {((len(tc) - 1) / (tc[-1] - tc[0])) if len(tc) > 1 else 0:5.1f} fps"
              f"  capture->recv median {np.median(lat) * 1e3 if lat.size else float('nan'):.1f} ms")
    raw_path = os.path.join(out_dir, "imu_raw.rec")
    if os.path.exists(raw_path):
        batches = [d for _, d in read_recording(raw_path)]
        if batches:
            s = np.concatenate(batches).view(np.recarray)
            span = s.t_capture[-1] - s.t_capture[0]
            print(f"imu raw   {len(s):5d} samples {len(s) / span if span else 0:6.1f} Hz  seq gaps "
                  f"{int(np.sum(np.diff(s.seq) - 1))}  mean accel {s.ax.mean():.2f} {s.ay.mean():.2f} {s.az.mean():.2f} m/s^2")
            if export:
                np.save(os.path.join(exp, "imu_raw.npy"), s)
    imu_path = os.path.join(out_dir, "imu.rec")
    if os.path.exists(imu_path):
        batches = [d for _, d in read_recording(imu_path)]
        if batches:
            s = np.concatenate(batches).view(np.recarray)
            span = s.t_end[-1] - s.t_start[0]
            print(f"imu state {len(s):5d} records {len(s) / span if span else 0:6.1f} Hz  last orientation "
                  f"(w,x,y,z) {s.qw[-1]:.4f} {s.qx[-1]:.4f} {s.qy[-1]:.4f} {s.qz[-1]:.4f}")
            if export:
                np.save(os.path.join(exp, "imu_state.npy"), s)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--url", default="http://localhost:8101")
    p.add_argument("--out", default="orbbec_rec")
    p.add_argument("--streams", default="color,depth")
    p.add_argument("--mode", default="raw", choices=["raw", "jpeg"])
    p.add_argument("--duration", type=float, default=None, help="seconds (default: until Ctrl-C)")
    p.add_argument("--no-imu", action="store_true")
    p.add_argument("--read", metavar="DIR", help="summarize an existing recording instead")
    p.add_argument("--export", action="store_true", help="with --read: write PNG/JPEG files + frames.csv")
    a = p.parse_args()
    if a.read:
        summarize(a.read, a.export)
        return
    record(a.url, a.out, streams=a.streams, mode=a.mode, duration=a.duration, imu=not a.no_imu)
    summarize(a.out)


if __name__ == "__main__":
    main()
