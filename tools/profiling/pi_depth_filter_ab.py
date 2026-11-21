#!/usr/bin/env python3
"""Head camera depth: SDK software noise filter vs the camera's hardware filter (run on the Pi, robot venv).

  ~/robot_utils/.venv/bin/python pi_depth_filter_ab.py [N_FRAMES]

For each configuration of (OB_PROP_DEPTH_SOFT_FILTER_BOOL, OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL) it
captures N fused frames from 3d_perception's /frame (the depth the stop reflex uses), measures the camera
node's CPU meanwhile, and reports: valid fraction, flicker, per-pixel temporal noise, speckle islands, holes /
depth change vs the original configuration, and the reflex's own straight-ahead obstacle count
(agent_interface geometry: points in OBST_Z within the base's width, HALF .. HALF+lookahead ahead; it stops at
>= REFLEX_POINTS). Mean depth images go to ./filter_ab/. Device properties only; nothing moves. The original
values are restored at the end, and checked.
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.expanduser("~/robot_utils/src"))
import cv2  # noqa: E402
import numpy as np  # noqa: E402

from robot_utils.nodes.agent_interface import geometry as G  # noqa: E402
from robot_utils.nodes.common.client import NodeClient  # noqa: E402

N = int(sys.argv[1]) if len(sys.argv) > 1 else 40
SOFT, HW = "OB_PROP_DEPTH_SOFT_FILTER_BOOL", "OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL"
THR = "OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT"
REFLEX_POINTS = 25
LOOKAHEAD = (0.5, 1.0, 1.5)
SPECKLE_PX = 50
OUT = "filter_ab"
TCK = os.sysconf("SC_CLK_TCK")
cam = NodeClient("camera", timeout=15)
per = NodeClient("3d_perception", timeout=15)


def props():
    return {p["id"]: p["value"] for p in cam.get("/properties", all="true")}


def set_prop(pid, v):
    cam.post("/properties", {"id": pid, "value": v})


def cam_pid():
    for p in os.listdir("/proc"):
        if p.isdigit():
            try:
                cmd = open(f"/proc/{p}/cmdline", "rb").read().split(b"\0")
            except OSError:
                continue
            if b"robot_utils.nodes.orbbec" in cmd and b"camera" in cmd and b"wrist_camera" not in cmd:
                return int(p)


def cpu(p):
    f = open(f"/proc/{p}/stat").read().rsplit(")", 1)[1].split()
    return (int(f[11]) + int(f[12])) / TCK


def capture(n):
    frames, color, hdr = [], None, None
    p = cam_pid()
    c0, t0 = cpu(p), time.monotonic()
    while len(frames) < n:
        for h, payload, _ in per.records("/frame", max_age=0.3, timeout=5):
            if h.get("type") != "fused":
                continue
            d = h["depth"]
            frames.append(np.frombuffer(payload, "<u2", d["width"] * d["height"], d["offset"])
                          .reshape(d["height"], d["width"]).copy())
            hdr = h
            if color is None and "color" in h:
                c = h["color"]
                color = payload[c["offset"]: c["offset"] + c["len"]]
            if len(frames) >= n:
                break
    cpu_pct = 100 * (cpu(p) - c0) / (time.monotonic() - t0)
    return np.stack(frames), color, hdr, cpu_pct


def reflex_counts(F, cal):
    h, w = F.shape[1:]
    camm = G.Camera.from_color(cal["color"], w, h)
    T = np.array(cal["base_to_color"])
    out = {la: [] for la in LOOKAHEAD}
    wide = []
    masks = []
    for depth in F:
        P = G.Cloud(depth, camm, T, None).p_b
        with np.errstate(invalid="ignore"):
            a, b, z = P[..., 0], P[..., 1], P[..., 2]
            band = (z > G.OBST_Z[0]) & (z < G.OBST_Z[1])
            for la in LOOKAHEAD:
                out[la].append(int((band & (a > G.HALF) & (a < G.HALF + la) & (np.abs(b) < G.HALF + 0.03)).sum()))
            m3 = band & (a > G.HALF) & (a < 3.0) & (np.abs(b) < 1.0)
            wide.append(int(m3.sum()))
            masks.append(m3)
    res = {f"ahead_{la}m": {"mean": round(float(np.mean(v)), 1), "min": int(min(v)), "max": int(max(v)),
                            "frames_stop": f"{sum(x >= REFLEX_POINTS for x in v)}/{len(v)}"} for la, v in out.items()}
    res["obstacle_pts_3m"] = round(float(np.mean(wide)))
    return res, np.mean(masks, 0)


def metrics(F, ref=None):
    valid = F > 0
    vfrac = valid.mean(0)                                  # per pixel: fraction of frames valid
    Fn = np.where(valid, F.astype(float), np.nan)
    stable = vfrac >= 0.9
    with np.errstate(invalid="ignore"), np.errstate(all="ignore"):
        mean = np.nanmean(Fn, 0)
        std = np.nanstd(Fn, 0)
    s = std[stable]
    rel = (std / mean)[stable]
    speck = []
    for v in valid:
        n, _, stats, _ = cv2.connectedComponentsWithStats(v.astype(np.uint8), connectivity=8)
        speck.append(int((stats[1:, cv2.CC_STAT_AREA] < SPECKLE_PX).sum()))
    m = {"valid_pct": round(100 * float(valid.mean()), 2),
         "flicker_pct": round(100 * float(((vfrac > 0) & (vfrac < 1)).mean()), 2),
         "noise_mm_median": round(float(np.median(s)), 2), "noise_mm_p90": round(float(np.percentile(s, 90)), 2),
         "noise_rel_median_pct": round(100 * float(np.median(rel)), 3),
         "speckles_per_frame": round(float(np.mean(speck)), 1)}
    if ref is not None:
        rv, rmean = ref
        both = (rv >= 0.9) & stable
        m["holes_vs_orig_pct"] = round(100 * float(((rv >= 0.9) & (vfrac < 0.5)).mean()), 2)
        m["filled_vs_orig_pct"] = round(100 * float(((rv < 0.5) & (vfrac >= 0.9)).mean()), 2)
        m["depth_change_mm_median"] = round(float(np.median(np.abs(mean[both] - rmean[both]))), 2)
        m["depth_change_mm_p95"] = round(float(np.percentile(np.abs(mean[both] - rmean[both]), 95)), 2)
    return m, (vfrac, mean)


def save_mean(name, mean, color):
    os.makedirs(OUT, exist_ok=True)
    m = np.nan_to_num(mean, nan=0)
    v = m[m > 0]
    lo, hi = (np.percentile(v, 2), np.percentile(v, 98)) if v.size else (0, 1)
    img = np.clip((m - lo) / max(hi - lo, 1) * 255, 0, 255).astype(np.uint8)
    img = cv2.applyColorMap(img, cv2.COLORMAP_TURBO)
    img[m == 0] = 0
    cv2.imwrite(f"{OUT}/{name}.png", img)
    if color is not None and not os.path.exists(f"{OUT}/color.jpg"):
        open(f"{OUT}/color.jpg", "wb").write(color)


def main():
    p0 = props()
    orig = {THR: p0[THR], SOFT: p0[SOFT], HW: p0[HW]}
    print("original:", orig, flush=True)
    cal = per.get("/calibration")
    thr0 = p0[THR]
    configs = [("soft_only", {SOFT: True, HW: False}), ("neither", {SOFT: False, HW: False})]
    configs += [(f"hw_{t:.2f}", {SOFT: False, HW: True, THR: t}) for t in (0.10, 0.20, 0.40, 0.60)]
    results, ref, ref_obst = {}, None, None
    try:
        for name, cfg in configs:
            for k, v in cfg.items():
                set_prop(k, v)
            time.sleep(2.0)
            now = props()
            got = {k: (round(now[k], 3) if isinstance(now[k], float) else now[k]) for k in cfg}
            F, color, hdr, cpu_pct = capture(N)
            m, r = metrics(F, ref)
            if ref is None:
                ref = r
            m["camera_cpu_pct"] = round(cpu_pct, 1)
            m["reflex"], obst = reflex_counts(F, cal)
            if ref_obst is None:
                ref_obst = obst >= 0.9                    # real obstacle pixels: in the band in >= 90% of frames
                m["reflex"]["obstacle_pixels_ref"] = int(ref_obst.sum())
            elif ref_obst.any():
                m["reflex"]["obstacle_kept_pct"] = round(100 * float((obst[ref_obst] >= 0.5).mean()), 1)
                m["reflex"]["new_obstacle_px"] = int(((obst >= 0.5) & ~ref_obst & (obst > 0)).sum())
            m["props_read_back"] = got
            m["frame"] = f"{F.shape[2]}x{F.shape[1]}, {len(F)} frames"
            results[name] = m
            save_mean(name, r[1], color)
            print(name, json.dumps(m), flush=True)
    finally:
        for k, v in orig.items():
            set_prop(k, v)
        time.sleep(1.0)
        back = props()
        ok = all(back[k] == v for k, v in orig.items())
        print("restored:", "OK" if ok else f"MISMATCH {[(k, back[k]) for k in orig]}", flush=True)
    json.dump(results, open(f"{OUT}/results.json", "w"), indent=1)


if __name__ == "__main__":
    main()
