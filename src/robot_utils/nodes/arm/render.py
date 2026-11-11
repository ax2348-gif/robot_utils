"""An agent-friendly picture of an IK query (``POST /ik`` with ``render``; ``POST /ik/render``).

A plain, high-contrast wireframe PNG for vision models deciding how to move the arm: the arm at its
measured pose (dark grey boxes, geometry.ARM_BOXES as on the page), the IK solution as a ghost arm
(blue when IK succeeds, orange when it fails: then it is the nearest reachable configuration), the
requested target frame, the ``nearest`` reachable TCP pose when it differs, joints at a limit
(red rings), the arm base frame's labelled axes, reach rings on the floor, a scale bar and short
annotations. Runs in the planner process (planner.py), never on the control thread.

View: orthographic, looking at the drawing's centre from the direction (azimuth, elevation) in the
arm base frame. ``azimuth_deg`` 0 looks from +x toward the base (+y to the right), 90 from +y
(+x to the left), 180 from -x, -90 from -y; ``elevation_deg`` 0 is level, 90 looks straight down
(VIEW_DOC, which the API docs carry). The drawing is fitted to the image; ``zoom`` > 1 magnifies
about its centre.

Pillow only (no matplotlib): the edges are projected and drawn at 2x, then downsampled (antialiased).
"""
from __future__ import annotations

import io
import math
from typing import Optional, Sequence

import numpy as np

from .geometry import ARM_BOXES

VIEW_DEFAULTS = {"azimuth_deg": 45.0, "elevation_deg": 25.0, "zoom": 1.0, "width": 640, "height": 480}
VIEW_DOC = ("Orthographic view of the arm base frame (x, y on the floor, z up). `azimuth_deg`: the direction the "
            "camera looks FROM, about +z: 0 = from +x looking toward the base (+y appears to the right), 90 = from +y "
            "(+x to the left), 180 = from -x, -90 = from -y. `elevation_deg`: 0 = level, 90 = straight down from above "
            "(at azimuth 0, +y right and -x up the image), negative = from below the floor. The drawing (both arms, target "
            "and nearest pose) is fitted to the image; `zoom` > 1 magnifies about its centre (parts may leave the "
            "image). Defaults: azimuth 45, elevation 25, zoom 1, 640 x 480 px.")

SS = 2                                     # supersampling factor
INK, GHOST_OK, GHOST_FAIL, LIMIT = (34, 34, 34), (21, 101, 192), (211, 84, 0), (198, 40, 40)
AXC = [(211, 47, 47), (46, 125, 50), (25, 90, 200)]       # x y z
FLOOR, MUTED, TARGET = (205, 210, 216), (110, 116, 124), (0, 0, 0)
EDGES = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)]


def _font(px: int):
    from PIL import ImageFont
    try:
        return ImageFont.load_default(size=px)
    except TypeError:                       # Pillow without FreeType sizes
        return ImageFont.load_default()


def box_edges(frames: Sequence, gap: float) -> list:
    """World-space segments of the arm's boxes: [(kind, a, b)], kind = "base" | "link" | "jaw".
    ``frames``: 8 (pos, R) pairs as /links gives them (joint1..joint6 frames, end_link, tcp)."""
    I = (np.zeros(3), np.eye(3))
    F = {"base": I, "end": frames[6], **{f"link{i + 1}": frames[i] for i in range(6)}}
    out = []

    def add(T, b, sy, kind):
        p, R = np.asarray(T[0], float), np.asarray(T[1], float)
        Q = [p + R @ (np.array([b[1] + i * b[4] / 2, sy * (b[2] + j * b[5] / 2), b[3] + k * b[6] / 2]) / 1000)
             for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]
        out.extend((kind, Q[a], Q[c]) for a, c in EDGES)

    for b in ARM_BOXES:
        if b[0] in F:
            add(F[b[0]], b, 1, "base" if b[0] == "base" else "link")
    E = frames[6]
    for sgn in (1, -1):
        Tj = (np.asarray(E[0], float) + np.asarray(E[1], float) @ np.array([0, sgn * gap / 2, 0]), E[1])
        for b in ARM_BOXES:
            if b[0] == "finger":
                add(Tj, b, sgn, "jaw")
    return out


def _nice(x: float) -> float:
    e = 10 ** math.floor(math.log10(x))
    return max(m * e for m in (1, 2, 5, 10) if m * e <= x)


def render_ik(meas: Sequence, gap: float, sol: Sequence, target, nearest, info: dict, joint_names: Sequence[str],
              view: Optional[dict] = None) -> bytes:
    """PNG bytes. ``meas`` / ``sol``: the 8 (pos, R) frames at the measured / IK configuration;
    ``target`` / ``nearest``: (pos, R) TCP poses; ``info``: IKOut.public()."""
    from PIL import Image, ImageDraw
    v = {**VIEW_DEFAULTS, **{k: x for k, x in (view or {}).items() if x is not None}}
    W, H = int(v["width"]), int(v["height"])
    az, el = math.radians(v["azimuth_deg"]), math.radians(v["elevation_deg"])
    e = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    r = np.array([-math.sin(az), math.cos(az), 0.0])
    u = np.cross(e, r)

    ok, exact = bool(info.get("ok")), bool(info.get("exact"))
    ghost = GHOST_OK if ok else GHOST_FAIL
    segs_m, segs_s = box_edges(meas, gap), box_edges(sol, gap)
    tp, tR = np.asarray(target[0], float), np.asarray(target[1], float)
    np_, nR = np.asarray(nearest[0], float), np.asarray(nearest[1], float)
    differs = info.get("pos_err_mm", 0) > 1.0 or info.get("rot_err_deg", 0) > 1.0

    # fit: everything that matters (not the floor rings) inside the area between the text bands
    pts = [s[1] for s in segs_m + segs_s] + [s[2] for s in segs_m + segs_s] + [tp, np_, tp + tR @ [0.06, 0, 0]]
    P2 = np.array([[p @ r, p @ u] for p in pts])
    lo, hi = P2.min(0), P2.max(0)
    fs = max(11, round(min(W, H) / 36))           # font px (output)
    lh = round(fs * 1.35)
    top, bot, side = 4 * lh + 8, lh + 12, 16
    sc = min((W - 2 * side) / max(hi[0] - lo[0], 1e-3), (H - top - bot) / max(hi[1] - lo[1], 1e-3)) * float(v["zoom"])
    cx, cy = (lo + hi) / 2
    ox, oy = W / 2, top + (H - top - bot) / 2

    def P(q):
        q = np.asarray(q, float)
        return ((ox + (q @ r - cx) * sc) * SS, (oy - (q @ u - cy) * sc) * SS)

    img = Image.new("RGB", (W * SS, H * SS), (255, 255, 255))
    g = ImageDraw.Draw(img)
    font, small = _font(fs * SS), _font(max(10, fs - 2) * SS)

    def line(a, b, col, w=1.0, dash=None):
        A, B = np.array(P(a)), np.array(P(b))
        wpx = max(1, round(w * SS))
        if not dash:
            g.line([tuple(A), tuple(B)], fill=col, width=wpx)
            return
        L = float(np.linalg.norm(B - A))
        if L < 1e-6:
            return
        d, gp = dash[0] * SS, dash[1] * SS
        t = 0.0
        while t < L:
            t2 = min(t + d, L)
            g.line([tuple(A + (B - A) * t / L), tuple(A + (B - A) * t2 / L)], fill=col, width=wpx)
            t = t2 + gp

    def text(q_or_xy, s, col, f=None, dx=6, dy=-6, world=True, anchor="ls"):
        x, y = P(q_or_xy) if world else (q_or_xy[0] * SS, q_or_xy[1] * SS)
        g.text((x + dx * SS, y + dy * SS), s, fill=col, font=f or font, anchor=anchor, stroke_width=3 * SS // 2, stroke_fill=(255, 255, 255))

    def ring(q, rad, col, w=1.5):
        x, y = P(q)
        R_ = rad * SS
        g.ellipse([x - R_, y - R_, x + R_, y + R_], outline=col, width=max(1, round(w * SS)))

    def triad(p, R, L, w, dash=None, cols=AXC):
        for k in range(3):
            line(p, p + R[:, k] * L, cols[k], w, dash)

    # floor: reach rings 0.2 m apart, labelled
    for R0 in (0.2, 0.4, 0.6):
        ring_pts = [np.array([R0 * math.cos(t), R0 * math.sin(t), 0.0]) for t in np.linspace(0, 2 * math.pi, 97)]
        for a, b in zip(ring_pts[:-1], ring_pts[1:]):
            line(a, b, FLOOR, 1)
        q = np.array([r[0], r[1], 0.0]) * R0 / max(math.hypot(r[0], r[1]), 1e-9)   # to the right of the base
        if abs(el) > math.radians(10) and top + lh < P(q)[1] / SS < H - bot - fs - 6:
            text(q, f"{R0:.1f} m", MUTED, small, 3, 12)
    # floor drop lines under the target and the measured TCP
    for q in (tp, np.asarray(meas[7][0], float)):
        line(q, [q[0], q[1], 0], FLOOR, 1, (3, 3))
        line([q[0] - 0.01, q[1], 0], [q[0] + 0.01, q[1], 0], MUTED, 1)
        line([q[0], q[1] - 0.01, 0], [q[0], q[1] + 0.01, 0], MUTED, 1)
    # base frame axes, labelled
    for k, name in enumerate(("+x", "+y", "+z")):
        d = np.zeros(3); d[k] = 0.15
        line(np.zeros(3), d, AXC[k], 2.5)
        text(d, name, AXC[k], font, 4, 4)

    # the measured arm, then the IK solution as a ghost
    for kind, a, b in segs_m:
        line(a, b, INK, 1.6 if kind == "jaw" else 1.2)
    for kind, a, b in segs_s:
        if kind != "base":                          # the base doesn't move
            line(a, b, ghost, 1.6 if kind == "jaw" else 1.2, (6, 4))
    for k in range(6):
        ring(meas[k][0], 3, INK, 1.5)
        ring(sol[k][0], 3, ghost, 1.5)
    at = set(info.get("joints_at_limit") or [])
    for k, n in enumerate(joint_names[:6]):
        if n in at:
            ring(sol[k][0], 9, LIMIT, 2.5)
            text(sol[k][0], f"{n} at limit", LIMIT, small, 10, 14)

    # measured TCP, target, nearest
    mp, mR = np.asarray(meas[7][0], float), np.asarray(meas[7][1], float)
    triad(mp, mR, 0.04, 1.5)
    text(mp, "tcp now", INK, small, -8, -8, anchor="rs")
    if differs:
        triad(np_, nR, 0.05, 2, (5, 3))
        line(tp, np_, LIMIT, 1.5, (2, 3))
        text(np_, "nearest", ghost, small, 8, 16)
    triad(tp, tR, 0.06, 3)
    ring(tp, 5, TARGET, 2)
    text(tp, "target (TCP)", TARGET, font, 9, -9)

    # annotations (top band)
    if ok:
        head = "IK ok, exact" if exact else f"IK ok (orientation within tolerance, {info.get('rot_err_deg', 0):.1f}° off)"
    else:
        rs = info.get("reason") or ""
        why = ("outside the joint limits" if "joint limits" in rs else "out of reach" if "out of reach" in rs
               else "orientation not reachable" if "orientation" in rs else "no solution")
        head = f"IK FAILED: {why}; orange = nearest reachable"
    lines = [
        (head, (46, 125, 50) if ok else LIMIT),
        (f"error {info.get('pos_err_mm', 0):.1f} mm, {info.get('rot_err_deg', 0):.1f}°   joints at limit: "
         f"{', '.join(sorted(at)) or 'none'}", INK),
        (f"target  x {tp[0]:.3f}  y {tp[1]:.3f}  z {tp[2]:.3f} m" +
         (f"   nearest  x {np_[0]:.3f}  y {np_[1]:.3f}  z {np_[2]:.3f}" if differs else ""), INK),
        (f"view: azimuth {v['azimuth_deg']:g}°, elevation {v['elevation_deg']:g}°, zoom {float(v['zoom']):g} "
         f"(azimuth 0 = from +x)", MUTED),
    ]
    for i, (s, col) in enumerate(lines):
        text((8, 6 + (i + 1) * lh - 4), s, col, font if i < 3 else small, 0, 0, world=False)
    # legend (bottom band) and scale bar (bottom right)
    y0 = H - 8
    x = 8
    for label, col, dash in (("measured arm", INK, None), ("IK solution" if ok else "nearest reachable (IK failed)", ghost, (6, 4)),
                             ("x y z", None, None)):
        if col:
            A = (x * SS, (y0 - fs // 3) * SS)
            for t in range(0, 22, 10 if dash else 22):
                g.line([(A[0] + t * SS, A[1]), (A[0] + min(t + (6 if dash else 22), 22) * SS, A[1])], fill=col, width=2 * SS)
            x += 26
            g.text((x * SS, y0 * SS), label, fill=col, font=small, anchor="ls")
            x += round(g.textlength(label, font=small) / SS) + 16
        else:
            for k, c in enumerate("xyz"):
                g.text((x * SS, y0 * SS), c, fill=AXC[k], font=small, anchor="ls")
                x += round(g.textlength(c + " ", font=small) / SS)
    L = _nice(W * 0.18 / sc)
    bx1, bx0 = W - 12, W - 12 - L * sc
    yb = H - 10
    g.line([(bx0 * SS, yb * SS), (bx1 * SS, yb * SS)], fill=INK, width=2 * SS)
    for xb in (bx0, bx1):
        g.line([(xb * SS, (yb - 5) * SS), (xb * SS, (yb + 1) * SS)], fill=INK, width=2 * SS)
    g.text(((bx0 + bx1) / 2 * SS, (yb - 6) * SS), f"{L * 100:g} cm" if L < 1 else f"{L:g} m", fill=INK, font=small, anchor="ms")

    out = io.BytesIO()
    img.resize((W, H), Image.LANCZOS).save(out, "PNG", optimize=True)
    return out.getvalue()
