"""Scene components for ``RU.viz`` (appended to ``common/viz.js`` by viz_assets.py): the robot in its
world, in 3D. Same contract as viz_assets.py (plain data in, ``update`` schedules one
redraw, colours from theme tokens), kept in its own module so it composes with the other components.

``RU.viz.cameraFloor(desc, {range})``: from the description node's ``GET description``, each camera's
  optical centre and the outline of its depth image on the floor (z = 0), in the base frame:
  ``[{name, origin: [x, y, z], ring: [[x, y], ...], axis: [x, y] | null}]``. Rays above the horizon
  or beyond ``range`` (m, 3) stop at the range along their heading.

``RU.viz.Scene(el, opts)``: the robot in its world, in 3D (orthographic wire-frame), centred on the
  robot: a floor grid fixed to the world, the estimator's trails on the floor (fused solid, odometry
  only dashed), the robot frame and wheels at their steering angles at the estimated pose, each
  camera's depth frustum and floor footprint, and the velocity. A top view (``setView("top")``,
  world +x up) gives the same picture as a 2D map. Drag to orbit, wheel to zoom, double-click for
  the default 3D view. opts: ``half`` (m shown each side at zoom 1, 1.6), ``frustum`` (m, 0.35),
  ``range`` (m of floor footprint, 2.5), ``trailMax`` (points, 20000), ``staleAfter`` (s, 1).
  ``push(pose)``: an estimator row ({t, x, y, theta, x_odom, y_odom}); kept in the trail when the
  robot moved 5 mm / 0.01 rad or 0.25 s passed; a jump > 0.5 m or a gap > 1 s breaks it.
  ``update({desc, modules: {FL: {angle, speed}, ...}, velocity: [vx, vy] (world m/s), driving,
  cloud, arm})``; ``cloud`` from ``cloudFromFrame`` (null to hide); ``arm``: ``{frames, gap, enabled, at}``
  (the arm node's ``GET links`` frames, gripper gap m, enabled, ``performance.now()`` of the reading),
  drawn on the description's ``arm`` frame (the mount alone without readings; grey after 1 s).

``RU.viz.armSegments(T, frames, gap)``: the arm as wireframe edges ``[[a, b], ...]`` in base, from
  ``RU.viz.ARM_BOXES`` (arm/geometry.py): T = the arm frame in base (4x4), frames = ``GET links``
  frames (null: the mount plate and column only).

``RU.viz.cloudFromFrame(header, payload, calib, {maxPoints})`` (async): a 3d_perception fused frame
  record (``GET frame``) and its ``GET calibration`` -> coloured points in the base frame
  ``{xyz, rgb, n, t}`` (about ``maxPoints``, 8000), or null without ``base_to_color``.
"""

import json as _json

from ..arm.geometry import ARM_BOXES as _ARM_BOXES

SCENE_JS = r"""
(function(){
"use strict";
const RU = window.RU = window.RU || {};
const viz = RU.viz = RU.viz || {};
const fin = v => v !== null && v !== undefined && isFinite(v);
function tok(n, f){ const v = RU.token ? RU.token(n, "") : getComputedStyle(document.documentElement).getPropertyValue(n); return (v && String(v).trim()) || f; }
function colours(){
  const dark = window.matchMedia && matchMedia("(prefers-color-scheme: dark)").matches;
  return {ink: tok("--ink", dark ? "#e3e8ed" : "#15202b"), muted: tok("--muted", dark ? "#98a3ae" : "#5b6672"),
          rule: tok("--rule", dark ? "#2e363f" : "#d3dae1"), signal: tok("--signal", dark ? "#78aeff" : "#1f5fbf"),
          signalBg: tok("--signal-bg", dark ? "#16263d" : "#e3ecfa"), live: tok("--live", dark ? "#4cc88a" : "#157a45"),
          stop: tok("--stop", dark ? "#ff5b50" : "#c41e17"), plot1: tok("--plot-1", dark ? "#78aeff" : "#1f5fbf"),
          font: tok("--font", "system-ui"), mono: tok("--mono", "ui-monospace, monospace")};
}
function inject(id, css){ if (document.getElementById(id)) return; const s = document.createElement("style"); s.id = id; s.textContent = css; document.head.appendChild(s); }

// canvas plumbing shared by the components: container or <canvas>, dpr, resize, one redraw per frame
class Surface {
  constructor(el, cls){
    if (el.tagName === "CANVAS") this.cv = el;
    else { el.classList.add(cls); this.cv = document.createElement("canvas"); el.appendChild(this.cv); }
    this.g = this.cv.getContext("2d"); this.raf = 0; this.col = colours(); this.colAt = performance.now();
    this.ro = window.ResizeObserver ? new ResizeObserver(() => this.schedule()) : null;
    if (this.ro) this.ro.observe(this.cv);
  }
  schedule(){ if (!this.raf) this.raf = requestAnimationFrame(() => { this.raf = 0; this.draw(); }); }
  frame(){
    if (performance.now() - this.colAt > 500){ this.col = colours(); this.colAt = performance.now(); }   // theme changes
    const dpr = window.devicePixelRatio || 1, W = this.cv.clientWidth, H = this.cv.clientHeight;
    if (this.cv.width !== Math.round(W * dpr) || this.cv.height !== Math.round(H * dpr)){ this.cv.width = Math.round(W * dpr); this.cv.height = Math.round(H * dpr); }
    this.g.setTransform(dpr, 0, 0, dpr, 0, 0); this.g.clearRect(0, 0, W, H);
    return [W, H];
  }
  destroy(){ if (this.ro) this.ro.disconnect(); cancelAnimationFrame(this.raf); }
}

// -- geometry ------------------------------------------------------------------------------------
const tf = (T, p) => [0, 1, 2].map(i => T[i][0] * p[0] + T[i][1] * p[1] + T[i][2] * p[2] + T[i][3]);

// -- the arm: ARM_BOXES (arm/geometry.py: [frame, centre xyz, size xyz] mm, axis-aligned in that frame) ------
viz.ARM_BOXES = __ARM_BOXES__;
const qToR = ([x, y, z, w]) => [[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)], [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                                [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]];
// The arm as wireframe edges [[a, b], ...] in base. T: the arm frame in base (4x4, the description's
// frames_in_base.arm); frames: the arm node's GET links frames ({pos, quat} in the arm frame: joint1-6,
// end_link, tcp), or null for the mount alone; gap: the gripper gap (m).
function armSegments(T, frames, gap){
  const out = [];
  const box = (F, b, sy) => { const Q = [];
    for (const i of [-1, 1]) for (const j of [-1, 1]) for (const k of [-1, 1]){
      const v = [(b[1] + i * b[4] / 2) / 1000, sy * (b[2] + j * b[5] / 2) / 1000, (b[3] + k * b[6] / 2) / 1000];
      Q.push(tf(T, [0, 1, 2].map(r => F.p[r] + F.R[r][0] * v[0] + F.R[r][1] * v[1] + F.R[r][2] * v[2]))); }
    for (const [a, c] of [[0,1],[2,3],[4,5],[6,7],[0,2],[1,3],[4,6],[5,7],[0,4],[1,5],[2,6],[3,7]]) out.push([Q[a], Q[c]]); };
  const I = {p: [0, 0, 0], R: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]};
  for (const b of viz.ARM_BOXES) if (b[0] === "base") box(I, b, 1);
  if (!frames || frames.length < 7) return out;
  const L = frames.map(f => ({p: f.pos, R: qToR(f.quat)})), E = L[6];
  const FR = {link1: L[0], link2: L[1], link3: L[2], link4: L[3], link5: L[4], link6: L[5], end: E};
  for (const b of viz.ARM_BOXES) if (FR[b[0]]) box(FR[b[0]], b, 1);
  const g = fin(gap) ? Math.max(0, Math.min(gap, 0.057)) : 0;     // each jaw moves gap/2 along end_link y
  for (const sgn of [1, -1]){ const d = [0, sgn * g / 2, 0];
    const F = {p: [0, 1, 2].map(r => E.p[r] + E.R[r][1] * d[1]), R: E.R};
    for (const b of viz.ARM_BOXES) if (b[0] === "finger") box(F, b, sgn); }
  return out;
}
viz.armSegments = armSegments;
const rot = (T, d) => [0, 1, 2].map(i => T[i][0] * d[0] + T[i][1] * d[1] + T[i][2] * d[2]);
function cameraFloor(desc, opts){
  const R = (opts && opts.range) || 3, out = [];
  if (!desc || !desc.cameras) return out;
  const FB = desc.frames_in_base || {};
  for (const [name, c] of Object.entries(desc.cameras)){
    const T = FB[name + "/depth_optical"] || (c.pose_in_base && c.pose_in_base.matrix), K = (c.intrinsics || {}).depth;
    if (!T) continue;
    const o = tf(T, [0, 0, 0]);
    const hit = d => { const w = rot(T, d); if (w[2] >= -1e-6) return null; const t = -o[2] / w[2]; return [o[0] + w[0] * t, o[1] + w[1] * t]; };
    const hitR = d => { const p = hit(d), w = rot(T, d), hz = Math.hypot(w[0], w[1]);
      if (p && Math.hypot(p[0] - o[0], p[1] - o[1]) <= R) return p;
      return hz < 1e-6 ? null : [o[0] + w[0] / hz * R, o[1] + w[1] / hz * R]; };
    const ring = [];
    if (K){
      const cs = [[0, 0], [K.width, 0], [K.width, K.height], [0, K.height]].map(([u, v]) => [(u - K.cx) / K.fx, (v - K.cy) / K.fy, 1]);
      for (let e = 0; e < 4; e++) for (let s = 0; s < 12; s++){
        const a = cs[e], b = cs[(e + 1) % 4], p = hitR([a[0] + (b[0] - a[0]) * s / 12, a[1] + (b[1] - a[1]) * s / 12, 1]); if (p) ring.push(p); }
    }
    out.push({name, origin: o, ring, axis: hit([0, 0, 1]), T, K});
  }
  return out;
}
viz.cameraFloor = cameraFloor;

// a 3d_perception fused frame (RU.RecordReader record) + its GET calibration -> coloured points in the
// base frame: {xyz: Float32Array, rgb: Uint8ClampedArray, n, t}. Depth is z in the colour optical
// frame on calib.depth_grid; the colour JPEG is resampled onto the same grid. null without a pose.
async function cloudFromFrame(header, payload, calib, opts){
  const T = calib && calib.base_to_color, K = calib && calib.depth_grid, H = header || {};
  if (!T || !K || !H.depth || !H.depth.len || !H.color || !H.color.len) return null;
  const W = H.depth.width, Hh = H.depth.height, max = (opts && opts.maxPoints) || 8000;
  const dep = new Uint16Array(payload.buffer.slice(payload.byteOffset + H.depth.offset, payload.byteOffset + H.depth.offset + H.depth.len));
  const img = await createImageBitmap(new Blob([payload.subarray(H.color.offset, H.color.offset + H.color.len)], {type: "image/jpeg"}));
  const cv = (typeof OffscreenCanvas !== "undefined") ? new OffscreenCanvas(W, Hh) : Object.assign(document.createElement("canvas"), {width: W, height: Hh});
  const g = cv.getContext("2d"); g.drawImage(img, 0, 0, W, Hh); img.close && img.close();
  const px = g.getImageData(0, 0, W, Hh).data;
  const fx = K.fx * W / K.width, fy = K.fy * Hh / K.height, cx = K.cx * W / K.width, cy = K.cy * Hh / K.height;
  const step = Math.max(1, Math.ceil(Math.sqrt(W * Hh / max)));
  const xyz = new Float32Array(Math.ceil(W / step) * Math.ceil(Hh / step) * 3), rgb = new Uint8ClampedArray(xyz.length);
  let n = 0;
  for (let v = 0; v < Hh; v += step) for (let u = 0; u < W; u += step){
    const z = dep[v * W + u] / 1000; if (!z) continue;
    const x = (u - cx) / fx * z, y = (v - cy) / fy * z;
    xyz[3 * n] = T[0][0] * x + T[0][1] * y + T[0][2] * z + T[0][3];
    xyz[3 * n + 1] = T[1][0] * x + T[1][1] * y + T[1][2] * z + T[1][3];
    xyz[3 * n + 2] = T[2][0] * x + T[2][1] * y + T[2][2] * z + T[2][3];
    const k = 4 * (v * W + u); rgb[3 * n] = px[k]; rgb[3 * n + 1] = px[k + 1]; rgb[3 * n + 2] = px[k + 2]; n++;
  }
  // group by colour (4 bits a channel) so drawing sets fillStyle once per group, not per point
  const key = new Uint16Array(n), order = new Uint32Array(n);
  for (let i = 0; i < n; i++){ key[i] = (rgb[3 * i] >> 4) << 8 | (rgb[3 * i + 1] >> 4) << 4 | rgb[3 * i + 2] >> 4; order[i] = i; }
  order.sort((a, b) => key[a] - key[b]);
  const groups = [];
  for (let i = 0; i < n; ){ let j = i; const k = key[order[i]]; while (j < n && key[order[j]] === k) j++;
    groups.push({style: `rgb(${(k >> 8) * 17},${((k >> 4) & 15) * 17},${(k & 15) * 17})`, a: i, b: j}); i = j; }
  return {xyz, rgb, n, order, groups, t: H.t_color};
}
viz.cloudFromFrame = cloudFromFrame;

// -- Scene: the robot in its world, in 3D ---------------------------------------------------------
const SCENE_DEFAULTS = {frustum: 0.35, range: 4, half: 1.6, trailMax: 20000, staleAfter: 1};
const VIEWS = {iso: {az: -2.35, el: 0.6}, top: {az: Math.PI, el: Math.PI / 2 - 1e-4}};
class Scene extends Surface {
  constructor(el, opts){
    super(el, "ru-scene");
    inject("ru-viz-scene-css", ".ru-scene{position:relative}.ru-scene canvas{display:block;width:100%;height:100%;touch-action:none;cursor:grab}");
    this.o = Object.assign({}, SCENE_DEFAULTS, opts || {});
    this.d = {}; this.view = Object.assign({zoom: 1}, VIEWS.iso);
    this.trail = []; this.last = null; this.kept = null; this.at = 0;
    let drag = null;
    this.cv.addEventListener("pointerdown", e => { this.cv.setPointerCapture(e.pointerId); drag = {x: e.clientX, y: e.clientY, az: this.view.az, el: this.view.el}; });
    this.cv.addEventListener("pointermove", e => { if (!drag) return;
      this.view.az = drag.az - (e.clientX - drag.x) * 0.01;
      this.view.el = Math.max(0.05, Math.min(Math.PI / 2 - 1e-4, drag.el + (e.clientY - drag.y) * 0.01)); this.schedule(); });
    const end = () => drag = null; this.cv.addEventListener("pointerup", end); this.cv.addEventListener("pointercancel", end);
    this.cv.addEventListener("wheel", e => { e.preventDefault(); this.view.zoom = Math.max(0.1, Math.min(10, this.view.zoom * Math.exp(-e.deltaY * 0.0015))); this.schedule(); }, {passive: false});
    this.cv.addEventListener("dblclick", () => this.setView("iso"));
    this.timer = setInterval(() => this.schedule(), 500);        // greys out when the streams stop
  }
  setView(name){ Object.assign(this.view, VIEWS[name] || VIEWS.iso, {zoom: 1}); this.schedule(); }
  update(d){ Object.assign(this.d, d || {}); if (d && d.desc) this.fov = cameraFloor(d.desc, {range: this.o.range}); this.schedule(); }
  push(p){
    if (!p || !fin(p.x) || !fin(p.y)) return;
    const l = this.last;
    if (l && (Math.hypot(p.x - l.x, p.y - l.y) > 0.5 || p.t - l.t > 1)){ this.trail.push(null); this.kept = null; }
    const k = this.kept;
    if (!k || Math.hypot(p.x - k.x, p.y - k.y) >= 0.005 || Math.abs(p.theta - k.theta) >= 0.01 || p.t - k.t >= 0.25){
      this.trail.push([p.x, p.y, p.x_odom, p.y_odom]); this.kept = p;
      if (this.trail.length > this.o.trailMax) this.trail.splice(0, this.trail.length - this.o.trailMax);
    }
    this.last = p; this.at = performance.now(); this.schedule();
  }
  clear(){ this.trail = []; this.kept = null; this.schedule(); }
  destroy(){ super.destroy(); clearInterval(this.timer); }
  draw(){
    const [W, H] = this.frame(), g = this.g, C = this.col, D = this.d, desc = D.desc, p = this.last;
    if (W < 10 || H < 10) return;
    const live = p && (performance.now() - this.at) / 1000 < this.o.staleAfter;
    const px = p ? p.x : 0, py = p ? p.y : 0, th = p && fin(p.theta) ? p.theta : 0, ct = Math.cos(th), st = Math.sin(th);
    const {az, el} = this.view, e = [Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)];
    const r = [-Math.sin(az), Math.cos(az), 0], u = [e[1] * r[2] - e[2] * r[1], e[2] * r[0] - e[0] * r[2], e[0] * r[1] - e[1] * r[0]];
    const s = Math.min(W, H) / (2 * this.o.half) * this.view.zoom, lift = el > 1.4 ? 0 : 0.1 * H;
    // world point -> screen, centred on the robot
    const P = q => { const x = q[0] - px, y = q[1] - py, z = q[2] || 0;
      return [W / 2 + (x * r[0] + y * r[1] + z * r[2]) * s, H / 2 + lift - (x * u[0] + y * u[1] + z * u[2]) * s]; };
    const Bw = q => [px + ct * q[0] - st * q[1], py + st * q[0] + ct * q[1], q[2] || 0];   // base -> world
    const line = (a, b, col, w, dash) => { const A = P(a), B2 = P(b); g.strokeStyle = col; g.lineWidth = w || 1; g.setLineDash(dash || []); g.beginPath(); g.moveTo(A[0], A[1]); g.lineTo(B2[0], B2[1]); g.stroke(); };
    const poly = (pts, close) => { g.beginPath(); pts.forEach((q, i) => { const a = P(q); i ? g.lineTo(a[0], a[1]) : g.moveTo(a[0], a[1]); }); if (close) g.closePath(); };
    // floor grid fixed to the world (0.5 m; 1 m bold), around the robot; the world origin as a cross
    const R0 = Math.max(W, H) / s * 0.9, step = s > 140 ? 0.5 : s > 30 ? 1 : 5;
    g.setLineDash([]);
    for (let v = Math.floor((px - R0) / step) * step; v <= px + R0; v += step){ const bold = Math.abs(v - Math.round(v)) < 1e-6;
      line([v, py - R0, 0], [v, py + R0, 0], C.rule, bold ? 1 : 0.5); }
    for (let v = Math.floor((py - R0) / step) * step; v <= py + R0; v += step){ const bold = Math.abs(v - Math.round(v)) < 1e-6;
      line([px - R0, v, 0], [px + R0, v, 0], C.rule, bold ? 1 : 0.5); }
    line([-0.15, 0, 0], [0.15, 0, 0], C.muted, 1.5); line([0, -0.15, 0], [0, 0.15, 0], C.muted, 1.5);
    // trails on the floor: odometry only (dashed), fused (solid)
    const path = (ix, iy) => { g.beginPath(); let pen = false;
      for (const q of this.trail){ if (!q || !fin(q[ix])){ pen = false; continue; } const a = P([q[ix], q[iy], 0]); if (pen) g.lineTo(a[0], a[1]); else { g.moveTo(a[0], a[1]); pen = true; } } g.stroke(); };
    g.lineJoin = "round"; g.setLineDash([5, 4]); g.strokeStyle = C.muted; g.lineWidth = 1.2; path(2, 3);
    g.setLineDash([]); g.strokeStyle = live ? C.plot1 : C.muted; g.lineWidth = 2; path(0, 1);
    // what the camera sees on the floor
    for (const f of this.fov || []){
      if (f.ring.length > 2){ poly(f.ring.map(q => Bw([q[0], q[1], 0])), true);
        g.globalAlpha = 0.5; g.fillStyle = C.signalBg; g.fill(); g.globalAlpha = 1; g.setLineDash([3, 3]); g.strokeStyle = C.signal; g.lineWidth = 1; g.stroke(); g.setLineDash([]); }
    }
    // the coloured point cloud (base frame, placed at the current pose), 2 px squares
    const cl = D.cloud;
    if (cl && cl.n){
      const sz = Math.max(1.5, Math.min(4, s / 220));
      for (const grp of cl.groups){
        g.fillStyle = grp.style;
        for (let j = grp.a; j < grp.b; j++){ const i = cl.order[j];
          const a = P(Bw([cl.xyz[3 * i], cl.xyz[3 * i + 1], cl.xyz[3 * i + 2]])); g.fillRect(a[0] - sz / 2, a[1] - sz / 2, sz, sz); }
      }
    }
    // the robot: the 2x1 frame tubes and the 1x1 top rails solid, the interior volume between them as
    // dotted corner posts, front marker, wheels at their steering angles, and the arm on its mount (live
    // from the arm node's links; grey when they stop)
    const rb = (desc && desc.robot) || {frame_size: 0.4318, modules: [{name: "FL", x: 0.1656, y: 0.1656}, {name: "FR", x: 0.1656, y: -0.1656}, {name: "RR", x: -0.1656, y: -0.1656}, {name: "RL", x: -0.1656, y: 0.1656}]};
    const h = rb.frame_size / 2, ink = live || !p ? C.ink : C.muted;
    const box = (c, sz, col, w, dash) => { const Q = []; for (const i of [-1, 1]) for (const j of [-1, 1]) for (const k of [-1, 1]) Q.push(Bw([c[0] + i * sz[0] / 2, c[1] + j * sz[1] / 2, c[2] + k * sz[2] / 2]));
      [[0,1],[2,3],[4,5],[6,7],[0,2],[1,3],[4,6],[5,7],[0,4],[1,5],[2,6],[3,7]].forEach(([a, b]) => line(Q[a], Q[b], col, w, dash)); };
    const [fz0, fz1] = rb.frame_z || [0.02, 0.12];
    box([0, 0, (fz0 + fz1) / 2], [rb.frame_size, rb.frame_size, fz1 - fz0], ink, 1.5);
    const tr = rb.top_rail_z;
    if (tr){ box([0, 0, (tr[0] + tr[1]) / 2], [rb.frame_size, rb.frame_size, tr[1] - tr[0]], ink, 1.5);
      for (const i of [-1, 1]) for (const j of [-1, 1]) line(Bw([i * h, j * h, fz1]), Bw([i * h, j * h, tr[0]]), ink, 1, [2, 3]); }
    line(Bw([h, 0, fz1]), Bw([h + 0.07, 0, fz1]), ink, 2.5);
    const Ta = (desc && desc.frames_in_base || {}).arm;
    if (Ta){ const A = D.arm || {}, armLive = A.frames && performance.now() - (A.at || 0) < 1000;
      const col = armLive ? (A.enabled ? C.live : ink) : C.muted;
      g.lineJoin = "round"; for (const [a, b] of armSegments(Ta, A.frames, A.gap)) line(Bw(a), Bw(b), col, 1.25); }
    for (const m of rb.modules || []){
      const sm = (D.modules || {})[m.name] || {}, a = fin(sm.angle) ? sm.angle : 0, Rw = 0.0381, ca = Math.cos(a), sa = Math.sin(a);
      const col = fin(sm.angle) && live ? (D.driving ? C.live : ink) : C.muted, ring = [];
      for (let k = 0; k <= 16; k++){ const t = k / 16 * 2 * Math.PI; ring.push(Bw([m.x + ca * Rw * Math.cos(t), m.y + sa * Rw * Math.cos(t), Rw + Rw * Math.sin(t)])); }
      for (let k = 0; k < 16; k++) line(ring[k], ring[k + 1], col, 1.5);
      line(Bw([m.x, m.y, Rw]), Bw([m.x + ca * Rw * 1.7, m.y + sa * Rw * 1.7, Rw]), col, 1);
    }
    // coordinate frames (x red, y green, z blue, as on the description page): world origin, base,
    // and every camera frame the description gives (depth / colour optical, IMU)
    const AXC = ["#ef4444", "#22c55e", "#3b82f6"];
    const axes = (T, len, w) => { const o = Bw(tf(T, [0, 0, 0])); for (let k = 0; k < 3; k++){ const d = [0, 0, 0]; d[k] = len; line(o, Bw(tf(T, d)), AXC[k], w); } };
    for (let k = 0; k < 3; k++){ const d = [0, 0, 0]; d[k] = 0.25; line([0, 0, 0], d, AXC[k], 1.5); }          // world origin
    axes([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]], 0.2, 2.5);                                        // base
    const FB = (desc && desc.frames_in_base) || {};
    for (const [name, T] of Object.entries(FB)) if (name !== "base") axes(T, name.endsWith("/imu") ? 0.04 : 0.07, 1.5);
    // cameras: frustum, optical axis to the floor, drop line
    for (const f of this.fov || []){
      const T = f.T, o = Bw(f.origin), z = this.o.frustum;
      line(o, [o[0], o[1], 0], C.muted, 1, [3, 3]);
      if (f.K){ const K = f.K, F = [[0, 0], [K.width, 0], [K.width, K.height], [0, K.height]].map(([uu, vv]) => Bw(tf(T, [(uu - K.cx) / K.fx * z, (vv - K.cy) / K.fy * z, z])));
        F.forEach((q, i) => { line(o, q, C.signal, 1); line(q, F[(i + 1) % 4], C.signal, 1.5); }); }
      if (f.axis) line(o, Bw([f.axis[0], f.axis[1], 0]), C.signal, 1, [6, 3]);
      const a = P(o); g.fillStyle = C.signal; g.beginPath(); g.arc(a[0], a[1], 3, 0, 2 * Math.PI); g.fill();
    }
    // velocity (world frame) on the floor: 1 m/s = 0.5 m of arrow
    const v = D.velocity;
    if (live && v && fin(v[0]) && fin(v[1]) && Math.hypot(v[0], v[1]) > 0.02){
      const a = P([px, py, 0.13]), b = P([px + v[0] * 0.5, py + v[1] * 0.5, 0.13]), ang = Math.atan2(b[1] - a[1], b[0] - a[0]);
      g.setLineDash([]); g.strokeStyle = C.live; g.lineWidth = 2.5; g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]);
      g.moveTo(b[0] - 9 * Math.cos(ang - 0.45), b[1] - 9 * Math.sin(ang - 0.45)); g.lineTo(b[0], b[1]); g.lineTo(b[0] - 9 * Math.cos(ang + 0.45), b[1] - 9 * Math.sin(ang + 0.45)); g.stroke();
    }
    g.setLineDash([]); g.font = "12px " + C.font; g.fillStyle = C.muted;
    if (!p) g.fillText("waiting for the estimator", 10, 18); else if (!live) g.fillText("estimator stream stopped", 10, 18);
    if (!desc) g.fillText("waiting for the description", 10, 34);
  }
}
viz.Scene = Scene;
})();
""".replace("__ARM_BOXES__", _json.dumps(_ARM_BOXES))
