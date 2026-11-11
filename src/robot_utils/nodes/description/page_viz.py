"""Robot/camera visualisation for the description page: one wire-frame scene (floor grid, robot
frame and wheels, base axes, each camera's body, depth frustum, optical / colour / IMU frames,
where the image and the optical axis meet the floor, and the nominal pose as a ghost) drawn by
one orthographic renderer into an orbitable 3D view and top / side / front views.

Uses only GET description (``frames_in_base``, ``robot``, ``cameras[*].intrinsics`` / ``nominal``).
Inlined into page.PAGE (no CDN)."""

VIZ_CSS = r"""
.views{display:grid;grid-template-columns:minmax(0,2fr) minmax(0,1fr);gap:8px}
.views .v3{grid-row:span 3}
.views .vw{position:relative;min-width:0}
.views canvas{width:100%;display:block;background:var(--bg);border-radius:6px;touch-action:none}
.views .v3 canvas{height:470px;cursor:grab}
.views .v2 canvas{height:143px;cursor:move}
.views .cap{position:absolute;left:6px;top:4px;font-size:11px;color:var(--muted);pointer-events:none}
.views .ro{position:absolute;left:6px;right:6px;top:20px;font:11px ui-monospace,monospace;color:var(--muted);pointer-events:none;white-space:pre-wrap}
.views .help{position:absolute;right:6px;bottom:4px;font-size:11px;color:var(--muted);pointer-events:none}
.vopts{display:flex;flex-wrap:wrap;gap:4px 14px;font-size:12px;margin:6px 0}
.vopts label{display:flex;gap:4px;align-items:center}
@media (max-width:820px){.views{grid-template-columns:1fr}.views .v3{grid-row:auto}.views .v3 canvas{height:360px}.views .help{display:none}}
"""

VIZ_HTML = r"""
<div class="vopts">
  <label><input type="checkbox" id="o_frustum" checked>frustum</label>
  <label><input type="checkbox" id="o_floor" checked>image on the floor</label>
  <label><input type="checkbox" id="o_sub" checked>colour / IMU frames</label>
  <label><input type="checkbox" id="o_nominal" checked>nominal pose (ghost)</label>
  <label><input type="checkbox" id="o_labels" checked>labels</label>
  <label>fit <select id="o_fit"><option value="robot">robot</option><option value="view">robot + camera view</option></select></label>
  <label>frustum <input type="number" id="o_flen" value="0.4" min="0.05" max="3" step="0.05" style="width:56px"> m</label>
  <label>floor range <input type="number" id="o_range" value="4" min="0.5" max="12" step="0.5" style="width:56px"> m</label>
</div>
<div class="views">
  <div class="vw v3"><canvas id="v3d"></canvas><div class="cap">3D</div><div class="ro" id="vro"></div>
    <div class="help">drag: orbit · shift-drag: pan · wheel: zoom · double-click: reset</div></div>
  <div class="vw v2"><canvas id="vtop"></canvas><div class="cap">top (x up, y left)</div></div>
  <div class="vw v2"><canvas id="vside"></canvas><div class="cap">side, from the right (x right, z up)</div></div>
  <div class="vw v2"><canvas id="vfront"></canvas><div class="cap">front, from ahead (y right, z up)</div></div>
</div>
"""

VIZ_JS = r"""
// ===================== robot / camera scene =====================
const V3 = {
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]], sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  mul: (a, s) => [a[0] * s, a[1] * s, a[2] * s], dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  norm: a => Math.hypot(a[0], a[1], a[2]),
};
const AX = ["#ef4444", "#22c55e", "#3b82f6"];       // x, y, z
const tf = (T, p) => [0, 1, 2].map(i => T[i][0] * p[0] + T[i][1] * p[1] + T[i][2] * p[2] + T[i][3]);
const rot = (T, d) => [0, 1, 2].map(i => T[i][0] * d[0] + T[i][1] * d[1] + T[i][2] * d[2]);
function rpyT(xyz, rpy){   // URDF fixed-axis XYZ: R = Rz(y) Ry(p) Rx(r)
  const [r, p, y] = rpy, cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
  return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
          [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
          [-sp, cp * sr, cp * cr, xyz[2]], [0, 0, 0, 1]];
}
const opt = id => document.getElementById(id);
const BODY = {size: [0.124, 0.029, 0.026], center: [0.012, 0, -0.016]};   // Gemini 336L, approx, in depth_optical

function buildScene(){
  // -> {lines: [{a, b, c, w, dash}], fills: [{pts, c}], labels: [{p, t, c}], fit: {robot: [], view: []}, info}
  const S = {lines: [], fills: [], labels: [], fit: {robot: [], view: []}, info: []};
  if (!DESC) return S;
  const css = n => getComputedStyle(document.body).getPropertyValue(n).trim() || "#888";
  const grid = css("--line"), muted = css("--muted"), fg = css("--fg") || "#ddd";
  const L = (a, b, c, w, dash) => S.lines.push({a, b, c, w: w || 1, dash});
  const lab = (p, t, c) => { if (opt("o_labels").checked) S.labels.push({p, t, c: c || muted}); };
  const fitR = (...ps) => { S.fit.robot.push(...ps); S.fit.view.push(...ps); };
  const axes = (T, len, w, names, dash) => [0, 1, 2].forEach(k => {
    const d = [0, 0, 0]; d[k] = len[k] ?? len; const e = tf(T, d); L(tf(T, [0, 0, 0]), e, AX[k], w, dash);
    if (names) lab(e, names[k], AX[k]); });
  const box = (T, c, s, col, w, dash) => {      // box of size s centred at c, in frame T
    const P = []; for (const i of [-1, 1]) for (const j of [-1, 1]) for (const k of [-1, 1])
      P.push(tf(T, [c[0] + i * s[0] / 2, c[1] + j * s[1] / 2, c[2] + k * s[2] / 2]));
    [[0,1],[2,3],[4,5],[6,7],[0,2],[1,3],[4,6],[5,7],[0,4],[1,5],[2,6],[3,7]].forEach(([a, b]) => L(P[a], P[b], col, w, dash));
    return P; };
  // robot: the 2x1 frame tubes (2.75-4.75 in) and the 1x1 top rails (7.75-8.75 in) solid, the interior
  // volume between them as dotted corner posts, wheels (3 in MAXSwerve), base axes, and the arm at the
  // arm frame (live links, else its mount)
  const rb = DESC.robot || {frame_size: 0.43, modules: []}, h = rb.frame_size / 2, I4 = rpyT([0, 0, 0], [0, 0, 0]);
  const [fz0, fz1] = rb.frame_z || [0.02, 0.12], tr = rb.top_rail_z;
  fitR(...box(I4, [0, 0, (fz0 + fz1) / 2], [rb.frame_size, rb.frame_size, fz1 - fz0], muted, 1.5));
  if (tr){ fitR(...box(I4, [0, 0, (tr[0] + tr[1]) / 2], [rb.frame_size, rb.frame_size, tr[1] - tr[0]], muted, 1.5));
    for (const i of [-1, 1]) for (const j of [-1, 1]) L([i * h, j * h, fz1], [i * h, j * h, tr[0]], muted, 1, [2, 3]); }
  const Tarm = (DESC.frames_in_base || {}).arm;
  if (Tarm){ const live = ARM && performance.now() - ARM.at < 1500;
    for (const [a, b] of RU.viz.armSegments(Tarm, live ? ARM.frames : null, live ? ARM.gap : 0)) L(a, b, live ? fg : muted, 1.25);
    fitR(tf(Tarm, [0, 0, 0])); axes(Tarm, 0.08, 2, null); lab(tf(Tarm, [0.12, 0, 0.02]), "arm", fg); }
  for (const m of rb.modules){ box(rpyT([m.x, m.y, 0.0381], [0, 0, 0]), [0, 0, 0], [0.0762, 0.03, 0.0762], fg, 1); lab([m.x, m.y, 0.09], m.name); }
  L([h, 0, fz1], [h + 0.06, 0, fz1], AX[0], 2);             // front marker
  axes(I4, 0.2, 2.5, ["x", "y", "z"]); lab([0, 0, -0.03], "base");
  // floor grid: 0.1 m inside ±1 m, 0.5 m beyond; grows with the camera view
  const cams = Object.entries(DESC.cameras || {}), FB = DESC.frames_in_base || {};
  let ext = 1;
  const camPts = [];
  for (const [name, c] of cams){
    const T = FB[name + "/depth_optical"] || c.pose_in_base.matrix, K = (c.intrinsics || {}).depth;
    const o = tf(T, [0, 0, 0]); fitR(o, [o[0], o[1], 0]);
    // body, optical axes, drop line to the floor, height
    box(T, BODY.center, BODY.size, fg, 1.5);
    axes(T, [0.08, 0.08, 0.16], 2.5, ["x", "y", "z"]);
    lab(V3.add(o, [0, 0, 0.08]), name, fg);
    L(o, [o[0], o[1], 0], muted, 1, [4, 4]); lab([o[0], o[1], o[2] / 2], `${o[2].toFixed(3)} m`);
    if (opt("o_sub").checked){
      for (const [sub, len] of [["color_optical", 0.05], ["imu", 0.04]]){
        const Ts = FB[`${name}/${sub}`]; if (!Ts) continue;
        axes(Ts, len, 1.5); lab(tf(Ts, sub === "imu" ? [0, len * 1.6, -0.03] : [len * 1.2, len * 1.2, 0]), sub.replace("_optical", ""));
      } }
    // frustum (depth intrinsics), its rays to the floor, the optical axis' floor point
    const z = Math.max(0.05, +opt("o_flen").value || 0.4);
    const corners = K ? [[0, 0], [K.width, 0], [K.width, K.height], [0, K.height]].map(([u, v]) => [(u - K.cx) / K.fx, (v - K.cy) / K.fy, 1]) : null;
    if (corners && opt("o_frustum").checked){
      const F = corners.map(r => tf(T, V3.mul(r, z))); F.forEach((p, i) => { L(o, p, "#60a5fa", 1); L(p, F[(i + 1) % 4], "#60a5fa", 1.5); });
      L(F[0], V3.mul(V3.add(F[0], F[1]), 0.5), "#60a5fa", 3);       // thick top edge: image up
      fitR(...F); }
    const R = Math.max(0.5, +opt("o_range").value || 4);
    const hit = d => { const dw = rot(T, d); if (dw[2] >= -1e-6) return null; const t = -o[2] / dw[2];
      return V3.add(o, V3.mul(dw, t)); };
    const hitR = d => {   // floor point, or (above the horizon / beyond R) the range limit along its heading
      const p = hit(d), dw = rot(T, d), hz = Math.hypot(dw[0], dw[1]);
      if (p && Math.hypot(p[0] - o[0], p[1] - o[1]) <= R) return p;
      return hz < 1e-6 ? null : [o[0] + dw[0] / hz * R, o[1] + dw[1] / hz * R, 0]; };
    const ax = hit([0, 0, 1]), dz = rot(T, [0, 0, 1]);
    const down = Math.asin(-dz[2]) * 180 / Math.PI, head = Math.atan2(dz[1], dz[0]) * 180 / Math.PI;
    S.info.push(`${name}: height ${o[2].toFixed(3)} m · optical axis ${down.toFixed(1)}° below horizontal, ` +
      `heading ${head >= 0 ? "+" : ""}${head.toFixed(1)}° (left +)` + (ax ? ` · meets the floor ${V3.norm([ax[0] - o[0], ax[1] - o[1], 0]).toFixed(2)} m out` : ""));
    if (corners && opt("o_floor").checked){
      // image border sampled along each edge, intersected with z = 0 (rays above the horizon are cut)
      const ring = [];
      for (let e = 0; e < 4; e++) for (let s = 0; s < 16; s++){
        const a = corners[e], b = corners[(e + 1) % 4], r = V3.add(a, V3.mul(V3.sub(b, a), s / 16)), p = hitR(r); if (p) ring.push(p); }
      if (ring.length > 2){ S.fills.push({pts: ring, c: "#3b82f622"}); ring.forEach((p, i) => L(p, ring[(i + 1) % ring.length], "#60a5fa", 1, [3, 3]));
        corners.forEach(r => { const p = hit(r); if (p && V3.norm(V3.sub(p, o)) < 3 * R) L(o, p, "#60a5fa55", 1, [2, 4]); });
        S.fit.view.push(...ring); camPts.push(...ring); }
      if (ax){ L(o, ax, "#3b82f6", 1, [6, 3]); L(V3.add(ax, [-0.04, 0, 0]), V3.add(ax, [0.04, 0, 0]), "#3b82f6", 2);
        L(V3.add(ax, [0, -0.04, 0]), V3.add(ax, [0, 0.04, 0]), "#3b82f6", 2); lab(ax, "optical axis"); S.fit.view.push(ax); }
    }
    // nominal pose, dashed grey: what the calibrations changed
    if (opt("o_nominal").checked && c.nominal && (c.from.ground || c.from.yaw)){
      const Tn = rpyT(c.nominal.xyz, c.nominal.rpy), on = tf(Tn, [0, 0, 0]);
      box(Tn, BODY.center, BODY.size, muted, 1, [3, 3]); axes(Tn, [0.06, 0.06, 0.12], 1, null, [3, 3]);
      if (corners && opt("o_frustum").checked){ const F = corners.map(r => tf(Tn, V3.mul(r, z * 0.6)));
        F.forEach((p, i) => { L(on, p, muted, 1, [3, 3]); L(p, F[(i + 1) % 4], muted, 1, [3, 3]); }); }
      lab(V3.add(on, [0, 0, -0.04]), "nominal", muted); fitR(on);
    }
  }
  for (const p of S.fit.view) ext = Math.max(ext, Math.abs(p[0]), Math.abs(p[1]));
  ext = Math.min(Math.ceil(ext * 2) / 2, 8);
  const G = [];
  for (let v = -ext; v <= ext + 1e-9; v += 0.1){
    const major = Math.abs(v - Math.round(v * 2) / 2) < 1e-6; if (Math.abs(v) > 1 + 1e-9 && !major) continue;
    G.push({a: [v, -ext, 0], b: [v, ext, 0], c: grid, w: major ? 1 : 0.5}, {a: [-ext, v, 0], b: [ext, v, 0], c: grid, w: major ? 1 : 0.5}); }
  S.lines.unshift(...G);
  return S;
}

// ---------- orthographic views ----------
// view: {r, u} screen right / up as base-frame unit vectors; e = toward the viewer (depth order)
const VIEWS = {
  vtop:   {r: [0, -1, 0], u: [1, 0, 0]},
  vside:  {r: [1, 0, 0],  u: [0, 0, 1]},
  vfront: {r: [0, 1, 0],  u: [0, 0, 1]},
};
const ORBIT0 = {az: -2.35, el: 0.5, zoom: 1, pan: [0, 0]};
const VS = {v3d: {...ORBIT0, pan: [0, 0]}, vtop: {zoom: 1, pan: [0, 0]}, vside: {zoom: 1, pan: [0, 0]}, vfront: {zoom: 1, pan: [0, 0]}};
let SCENE = null;
function orbitBasis(st){
  const e = [Math.cos(st.el) * Math.cos(st.az), Math.cos(st.el) * Math.sin(st.az), Math.sin(st.el)];
  const r = [-Math.sin(st.az), Math.cos(st.az), 0];
  return {r, u: V3.cross(e, r), e};
}
function renderView(id){
  const cv = opt(id); if (!cv || !SCENE) return;
  const dpr = devicePixelRatio || 1, W = cv.clientWidth, H = cv.clientHeight;
  if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)){ cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
  const g = cv.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
  const st = VS[id], B = id === "v3d" ? orbitBasis(st) : VIEWS[id];
  const pts = SCENE.fit[opt("o_fit").value] || SCENE.fit.robot;
  if (!pts.length) return;
  // fit: 2D views to the projected box; the 3D view to the bounding sphere (no rescaling while orbiting)
  let cx, cy, s;
  if (id === "v3d"){
    const lo = [0, 1, 2].map(k => Math.min(...pts.map(p => p[k]))), hi = [0, 1, 2].map(k => Math.max(...pts.map(p => p[k])));
    const c = V3.mul(V3.add(lo, hi), 0.5), rad = Math.max(0.2, ...pts.map(p => V3.norm(V3.sub(p, c)))) * 1.05;
    s = Math.min(W, H) / (2 * rad); cx = V3.dot(c, B.r); cy = V3.dot(c, B.u);
  } else {
    const xs = pts.map(p => V3.dot(p, B.r)), ys = pts.map(p => V3.dot(p, B.u));
    const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
    s = Math.min(W / Math.max(0.2, (x1 - x0) * 1.15), (H - 14) / Math.max(0.2, (y1 - y0) * 1.15)); cx = (x0 + x1) / 2; cy = (y0 + y1) / 2;
  }
  s *= st.zoom;
  const P = p => [W / 2 + (V3.dot(p, B.r) - cx) * s + st.pan[0], H / 2 + 7 - (V3.dot(p, B.u) - cy) * s + st.pan[1]];
  st._P = P; st._s = s;
  for (const f of SCENE.fills){ g.fillStyle = f.c; g.beginPath(); f.pts.forEach((p, i) => i ? g.lineTo(...P(p)) : g.moveTo(...P(p))); g.closePath(); g.fill(); }
  for (const l of SCENE.lines){ g.strokeStyle = l.c; g.lineWidth = l.w; g.setLineDash(l.dash || []);
    const a = P(l.a), b = P(l.b); g.beginPath(); g.moveTo(...a); g.lineTo(...b); g.stroke(); }
  g.setLineDash([]); g.font = "11px system-ui";
  if (id === "v3d" || s > 250) for (const t of SCENE.labels){ const a = P(t.p); g.fillStyle = t.c; g.fillText(t.t, a[0] + 4, a[1] - 3); }
  // scale bar
  const m = Math.pow(10, Math.floor(Math.log10(120 / s))), bar = [1, 2, 5].map(k => k * m).filter(v => v * s <= 120).pop() || m;
  g.strokeStyle = g.fillStyle = getComputedStyle(document.body).getPropertyValue("--muted").trim(); g.lineWidth = 2;
  g.beginPath(); g.moveTo(W - 10 - bar * s, H - 22); g.lineTo(W - 10, H - 22); g.stroke();
  g.fillText(bar >= 1 ? `${bar} m` : `${Math.round(bar * 100)} cm`, W - 10 - bar * s, H - 27);
}
function drawViews(){
  SCENE = buildScene();
  for (const id of ["v3d", "vtop", "vside", "vfront"]) renderView(id);
  opt("vro").textContent = SCENE.info.join("\n");
}
function bindView(id){
  const cv = opt(id), st = VS[id]; let drag = null;
  cv.addEventListener("pointerdown", e => { cv.setPointerCapture(e.pointerId); drag = {x: e.clientX, y: e.clientY, az: st.az, el: st.el, pan: [...st.pan], panMode: id !== "v3d" || e.shiftKey || e.button === 2}; });
  cv.addEventListener("pointermove", e => { if (!drag) return; const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
    if (drag.panMode) st.pan = [drag.pan[0] + dx, drag.pan[1] + dy];
    else { st.az = drag.az - dx * 0.01; st.el = Math.max(-1.55, Math.min(1.55, drag.el + dy * 0.01)); }
    renderView(id); });
  cv.addEventListener("pointerup", () => drag = null); cv.addEventListener("pointercancel", () => drag = null);
  cv.addEventListener("contextmenu", e => e.preventDefault());
  cv.addEventListener("wheel", e => { e.preventDefault(); const k = Math.exp(-e.deltaY * 0.0015), r = cv.getBoundingClientRect();
    const mx = e.clientX - r.left - r.width / 2, my = e.clientY - r.top - r.height / 2;      // zoom about the cursor
    st.pan = [mx - (mx - st.pan[0]) * k, my - (my - st.pan[1]) * k]; st.zoom *= k; renderView(id); }, {passive: false});
  cv.addEventListener("dblclick", () => { Object.assign(st, id === "v3d" ? {...ORBIT0, pan: [0, 0]} : {zoom: 1, pan: [0, 0]}); renderView(id); });
}
["v3d", "vtop", "vside", "vfront"].forEach(bindView);
["o_frustum", "o_floor", "o_sub", "o_nominal", "o_labels", "o_fit", "o_flen", "o_range"].forEach(i => opt(i).addEventListener("input", drawViews));
if (window.ResizeObserver) new ResizeObserver(() => drawViews()).observe(document.querySelector(".views"));
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", drawViews);   // colours come from the theme
"""
