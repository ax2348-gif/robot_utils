"""The arm node's browser page (served at the node root; docs/arm.md "Companion page"). Relative URLs only.

Data paths (kept few, docs/arm.md "State"): one ``stream?max_hz=30`` subscription for telemetry,
``events`` (SSE) for status, ``links`` at 10 Hz for drawing, ``kinematics`` at 5 Hz (measured: the
velocity ellipsoid) and, only while the arm is servoing, 2.5 Hz ``which=command`` for the commanded
ghost frame. The arm, its readouts and the servo form use measured values only. Polls pause while the
tab is hidden. The lease renews through the shared widget only: a forgotten, throttled tab may let it
lapse, which disables (the arm floats).

Jog: each of the four controls is a ``track`` client. Each hold starts from the measured TCP pose;
while held the page integrates the target from there and POSTs it with the matching twist at 25 Hz
(one request in flight at most); on release, pointer cancel, lost capture, window blur or a hidden tab
it stops sending, and the twist feedforward times out (0.25 s) into a hold. A pose the node refuses
is not integrated past.
"""

import json

from .geometry import ARM_BOXES

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Arm</title>
<link rel="stylesheet" href="common/ui.css">
<style>
.ru-wrap{max-width:1320px}
.layout{display:grid;grid-template-columns:minmax(0,1fr) 400px;grid-template-rows:auto 1fr;grid-template-areas:"view side" "status side";gap:0 12px;align-items:start}
.layout>.a-view{grid-area:view} .layout>.ru-side{grid-area:side} .layout>.a-status{grid-area:status}
@media (max-width:980px){.layout{grid-template-columns:minmax(0,1fr);grid-template-rows:none;grid-template-areas:"view" "side" "status"}}
.ctl{display:flex;flex-wrap:wrap;gap:6px;align-items:center}
.sub{display:flex;justify-content:space-between;align-items:baseline;gap:8px;flex-wrap:wrap;margin:0 0 6px}
/* the view */
#view{display:block;width:100%;height:500px;touch-action:none;cursor:grab;border-radius:calc(var(--radius) - 2px)}
#view:active{cursor:grabbing}
@media (max-width:980px){#view{height:360px}}
.legend{display:flex;flex-wrap:wrap;gap:4px 14px;font-size:12px;margin-top:6px}
.legend i{display:inline-block;width:18px;height:0;border-top:2px solid;vertical-align:middle;margin-right:5px}
#modeline{margin:0 0 10px;gap:10px}
.manip{display:grid;grid-template-columns:repeat(3,auto) 1fr;gap:2px 18px;align-items:baseline;margin-top:8px;padding-top:8px;border-top:1px solid var(--rule)}
.manip .v{font-size:18px;font-variant-numeric:tabular-nums}
.manip .lab{font-size:12px}
.manip .msg{justify-self:end;text-align:right;font-size:13px}
/* jog: hollow, wireframe controls */
.jog{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.jg{display:flex;flex-direction:column;align-items:center;gap:4px}
.jg .pair{display:flex;gap:8px;align-items:center}
svg.stick{display:block;touch-action:none;user-select:none;-webkit-user-select:none;cursor:grab;color:var(--muted);outline:none}
svg.stick.xy{width:128px;height:128px} svg.stick.v{width:40px;height:128px} svg.stick.h{width:176px;height:40px}
svg.stick .ring{fill:none;stroke:var(--muted);stroke-width:1.25}
svg.stick .faint{fill:none;stroke:var(--rule);stroke-width:1}
svg.stick .knob{fill:var(--panel);stroke:var(--signal);stroke-width:2}
svg.stick text{fill:var(--muted);font:17px var(--font)}
svg.stick.on .ring,svg.stick.on .knob{stroke:var(--live)}
svg.stick:focus-visible .ring{stroke:var(--signal);stroke-width:2}
@media (max-width:520px){.jog{grid-template-columns:minmax(0,1fr);gap:14px}}
.jog.off svg.stick{cursor:not-allowed} .jog.off svg.stick .knob{stroke:var(--muted)}
.speeds{display:flex;gap:12px;flex-wrap:wrap;align-items:center;margin-top:8px}
/* forms */
.form{display:grid;grid-template-columns:auto minmax(0,1fr) auto minmax(0,1fr);gap:4px 8px;align-items:center}
.form input,.form select{width:100%;box-sizing:border-box;min-width:0}
input[type=number]{font:inherit;font-variant-numeric:tabular-nums}
.speeds input{width:64px}
.hint{font-size:12px}
.bar{height:4px;border-radius:2px;background:var(--rule);overflow:hidden;margin:6px 0 2px}
.bar>div{height:100%;width:0;background:var(--signal)}
.named{display:flex;flex-wrap:wrap;gap:6px;margin-top:8px}
.kv{display:grid;grid-template-columns:auto 1fr;gap:2px 12px;font-variant-numeric:tabular-nums}
.kv span:nth-child(odd){color:var(--muted)}
.tables{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1.3fr);gap:16px}
@media (max-width:700px){.tables{grid-template-columns:minmax(0,1fr)}}
canvas.ru-plot{height:110px}
#events{max-height:170px;overflow:auto;font-size:12px}
</style><script src="../../themes/theme.js"></script>
</head><body data-page="arm"><div id="band"></div><div class="ru-wrap">
<div class="ru-row ru-head">
  <h2 style="margin:6px 0">Arm <span id="state" class="ru-pill">…</span></h2>
  <div class="ru-row"><div id="estop"></div><a class="ru-muted" href="docs">API docs</a></div>
</div>
<div class="ru-card ru-row ru-spread" data-panel="control" data-role="control">
  <div id="lease"></div>
  <div class="ctl">
    <button class="ru live" id="enable" title="Motors on: the arm floats (movable by hand) until a command engages it">Enable</button>
    <button class="ru" id="float" title="Stop servoing: the arm floats, gravity compensated, movable by hand; the gripper is untouched">Float</button>
    <button class="ru" id="disable" title="The arm floats, the gripper goes into damping, commands are refused until Enable">Disable</button>
    <button class="ru" id="home" title="Slow joint-space move to the home pose">Home</button>
    <button class="ru" id="stow" title="Safe homing, then motors off (damping) at home">Stow</button>
    <button class="ru" id="cancel" title="End the planned move or the track and hold where the arm is commanded">Hold</button>
    <button class="ru hazard" id="clear" hidden>Clear e-stop</button>
    <button class="ru hazard" id="reset" hidden title="Reconnect after a fault (ends uninitialized)">Reset</button>
  </div>
</div>
<div class="ru-row" id="modeline"><span class="ru-pill" id="mode">…</span><span id="modetext"></span></div>
<div id="err" class="ru-err"></div>
<div class="layout ru-body">
  <div class="ru-card a-view" data-panel="view" data-role="view">
    <div class="sub"><b>View</b>
      <span class="ctl"><button class="ru" data-v="iso">3D</button><button class="ru" data-v="top">Top</button><button class="ru" data-v="side">Side</button><button class="ru" data-v="front">Front</button></span></div>
    <canvas id="view" role="img" aria-label="The arm in its base frame: links and gripper as wireframe boxes, joint axes, TCP frames and the velocity ellipsoid"></canvas>
    <div class="legend ru-muted">
      <span><i style="border-color:var(--ink)"></i>measured</span>
      <span><i style="border-top-style:dashed;border-color:var(--muted)"></i>commanded TCP</span>
      <span><i style="border-top-style:dashed;border-color:var(--signal)"></i>goal</span>
      <span><i style="border-top-style:dotted;border-color:var(--ink)"></i>servo target (not sent)</span>
      <span><i style="border-color:var(--plot-3)"></i>TCP velocity ellipsoid</span>
      <span id="viewnote"></span>
    </div>
    <div class="manip" aria-live="polite">
      <span class="lab ru-muted">w</span><span class="lab ru-muted">σ<sub>min</sub></span><span class="lab ru-muted">w<sub>lin</sub></span><span></span>
      <span class="v" id="m_w">–</span><span class="v" id="m_s">–</span><span class="v" id="m_wl">–</span><span class="msg" id="m_msg"></span>
    </div>
  </div>

  <div class="ru-side">
    <div class="ru-card" data-panel="jog" data-role="control">
      <div class="sub"><b>Jog</b><span class="hint ru-muted" id="jognote">hold to move; let go to stop</span></div>
      <div class="jog off" id="jog">
        <div class="jg"><span class="hint ru-muted">position, base frame</span>
          <div class="pair"><div id="j_xy"></div><div id="j_z"></div></div></div>
        <div class="jg"><span class="hint ru-muted">rotation, TCP axes</span>
          <div id="j_py"></div><div id="j_roll"></div></div>
      </div>
      <div class="speeds"><label>linear <input type="number" id="j_lin" value="30" min="1" step="5"> mm/s</label>
        <label>angular <input type="number" id="j_ang" value="15" min="1" step="5"> °/s</label></div>
    </div>

    <div class="ru-card" data-panel="servo" data-role="control">
      <div class="sub"><b>Move to a pose</b><span class="hint ru-muted" id="s_src">follows the measured TCP until you edit</span></div>
      <div class="form">
        <span>x</span><input id="s_x" type="number" step="5"><span>roll</span><input id="s_r" type="number" step="5">
        <span>y</span><input id="s_y" type="number" step="5"><span>pitch</span><input id="s_p" type="number" step="5">
        <span>z</span><input id="s_z" type="number" step="5"><span>yaw</span><input id="s_yw" type="number" step="5">
        <span title="mm/s">speed</span><input id="s_lv" type="number" step="10" min="0"><span title="°/s">turn</span><input id="s_av" type="number" step="5" min="0">
        <span title="mm/s²">accel</span><input id="s_la" type="number" step="50" min="0"><span title="°/s²">ang. accel</span><input id="s_aa" type="number" step="10" min="0">
      </div>
      <div class="hint ru-muted">mm and degrees, arm base frame; blank speeds use the soft limits</div>
      <div class="ctl" style="margin-top:8px"><button class="ru primary" id="go">Go</button><button class="ru" id="s_cancel">Cancel</button>
        <button class="ru" id="s_follow">Use current pose</button></div>
      <div class="bar"><div id="s_bar"></div></div>
      <div class="ru-num" id="s_phase">no move yet</div>
      <div class="named" id="named"></div>
    </div>

    <div class="ru-card" data-panel="gripper" data-role="control">
      <div class="sub"><b>Gripper</b><span class="ru-num" id="g_meas">–</span></div>
      <div class="form">
        <span>gap</span><input id="g_gap" type="number" step="1" min="0" value="20"><span>max force</span><input id="g_force" type="number" step="5" min="0" placeholder="50">
      </div>
      <div class="ctl" style="margin-top:8px"><button class="ru" id="g_set">Set gripper</button><button class="ru" id="g_open">Open</button><button class="ru" id="g_close">Close</button>
        <span class="hint ru-muted">mm, N (blank force: 50 N)</span></div>
    </div>

    <div class="ru-card" data-panel="settings" data-role="config">
      <div class="sub"><b>Settings</b><span class="hint ru-muted" id="set_note"></span></div>
      <div class="form">
        <span>gains</span><select id="c_prof"></select><span></span><button class="ru" id="c_prof_set">Switch</button>
        <span>TCP x</span><input id="c_tx" type="number" step="5"><span>TCP y</span><input id="c_ty" type="number" step="5">
        <span>TCP z</span><input id="c_tz" type="number" step="5"><span></span><button class="ru" id="c_tcp_set">Set TCP</button>
      </div>
      <div class="hint ru-muted" style="margin:4px 0 8px">gains change only while nothing servos, the TCP offset (mm in end_link) while the arm doesn't</div>
      <div class="form" id="limform"></div>
      <div class="ctl" style="margin-top:8px"><button class="ru" id="c_lim_set">Set limits</button><span class="hint ru-muted" id="hardlim"></span></div>
    </div>
  </div>

  <div class="ru-card a-status" data-panel="status" data-role="telemetry">
    <div class="sub"><b>Tracking</b><span class="hint ru-muted">TCP error, mm, last 10 s</span></div>
    <canvas class="ru-plot" id="plot"></canvas>
    <div class="tables" style="margin-top:8px">
      <div class="kv" id="kv"></div>
      <table class="ru-table" id="joints"></table>
    </div>
  </div>
</div>
<div class="ru-card" data-panel="events" data-role="log"><b>Events</b><div id="events"></div></div>
</div>
<script src="common/ui.js"></script>
<script>
"use strict";
const $ = s => document.querySelector(s), D = 180 / Math.PI;
const fin = v => v !== null && v !== undefined && isFinite(v);
const f = (v, k) => { if (!fin(v)) return "–"; const s = Number(v).toFixed(k); return /^-0(\.0*)?$/.test(s) ? s.slice(1) : s; };
const lease = RU.leaseWidget($("#lease"), {onChange: () => render()});
const estop = RU.estopButton($("#estop"));
const band = RU.stateBand($("#band"));
const err = m => { $("#err").textContent = m || ""; };
const call = async (path, body) => { try { err(""); return await RU.api(path, {method: "POST", body, headers: lease.headers()}); } catch(e){ err(e.message); return null; } };

// -- rotations: R row-major 3x3, quaternions [x, y, z, w], rpy fixed X-Y-Z (R = Rz Ry Rx) ------------
function qToR(q){ const [x, y, z, w] = q;
  return [[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)], [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)], [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]]; }
function rToQ(R){ const t = R[0][0] + R[1][1] + R[2][2]; let x, y, z, w;
  if (t > 0){ const s = Math.sqrt(t + 1) * 2; w = s / 4; x = (R[2][1] - R[1][2]) / s; y = (R[0][2] - R[2][0]) / s; z = (R[1][0] - R[0][1]) / s; }
  else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]){ const s = Math.sqrt(1 + R[0][0] - R[1][1] - R[2][2]) * 2; w = (R[2][1] - R[1][2]) / s; x = s / 4; y = (R[0][1] + R[1][0]) / s; z = (R[0][2] + R[2][0]) / s; }
  else if (R[1][1] > R[2][2]){ const s = Math.sqrt(1 + R[1][1] - R[0][0] - R[2][2]) * 2; w = (R[0][2] - R[2][0]) / s; x = (R[0][1] + R[1][0]) / s; y = s / 4; z = (R[1][2] + R[2][1]) / s; }
  else { const s = Math.sqrt(1 + R[2][2] - R[0][0] - R[1][1]) * 2; w = (R[1][0] - R[0][1]) / s; x = (R[0][2] + R[2][0]) / s; y = (R[1][2] + R[2][1]) / s; z = s / 4; }
  const n = Math.hypot(x, y, z, w); return [x / n, y / n, z / n, w / n]; }
const mm = (A, B) => A.map((r, i) => [0, 1, 2].map(j => r[0] * B[0][j] + r[1] * B[1][j] + r[2] * B[2][j]));
const mv = (A, v) => A.map(r => r[0] * v[0] + r[1] * v[1] + r[2] * v[2]);
function expR(w){ const th = Math.hypot(w[0], w[1], w[2]); if (th < 1e-12) return [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  const k = w.map(v => v / th), c = Math.cos(th), s = Math.sin(th), C = 1 - c, [x, y, z] = k;
  return [[c + x*x*C, x*y*C - z*s, x*z*C + y*s], [y*x*C + z*s, c + y*y*C, y*z*C - x*s], [z*x*C - y*s, z*y*C + x*s, c + z*z*C]]; }
const rToRpy = R => [Math.atan2(R[2][1], R[2][2]), Math.atan2(-R[2][0], Math.hypot(R[0][0], R[1][0])), Math.atan2(R[1][0], R[0][0])];
function rpyToR(r, p, y){ const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
  return [[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr], [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr], [-sp, cp*sr, cp*cr]]; }
const col = (R, j) => [R[0][j], R[1][j], R[2][j]];

// -- the view: orthographic wireframe of the arm in its base frame --------------------------------------
const VIEWS = {iso: {az: Math.PI - 0.7, el: 0.5}, top: {az: Math.PI, el: Math.PI / 2 - 1e-4}, side: {az: -Math.PI / 2, el: 0.02}, front: {az: 0, el: 0.02}};
const AXC = ["#ef4444", "#22c55e", "#3b82f6"];        // x y z, as on the description page
const SIG = {warn: 0.02, crit: 0.007};                  // sigma_min thresholds (6x6 TCP Jacobian)
const ARM_BOXES = __ARM_BOXES__;                       // [frame, centre xyz, size xyz] mm (geometry.py ARM_BOXES)
class ArmView {
  constructor(cv){
    this.cv = cv; this.g = cv.getContext("2d"); this.d = {}; this.raf = 0; this.view = Object.assign({zoom: 1}, VIEWS.iso);
    let drag = null;
    cv.addEventListener("pointerdown", e => { cv.setPointerCapture(e.pointerId); drag = {x: e.clientX, y: e.clientY, az: this.view.az, el: this.view.el}; });
    cv.addEventListener("pointermove", e => { if (!drag) return;
      this.view.az = drag.az - (e.clientX - drag.x) * 0.01;
      this.view.el = Math.max(-0.2, Math.min(Math.PI / 2 - 1e-4, drag.el + (e.clientY - drag.y) * 0.01)); this.schedule(); });
    const end = () => drag = null; cv.addEventListener("pointerup", end); cv.addEventListener("pointercancel", end);
    cv.addEventListener("wheel", e => { e.preventDefault(); this.view.zoom = Math.max(0.3, Math.min(6, this.view.zoom * Math.exp(-e.deltaY * 0.0015))); this.schedule(); }, {passive: false});
    cv.addEventListener("dblclick", () => this.setView("iso"));
    if (window.ResizeObserver) new ResizeObserver(() => this.schedule()).observe(cv);
    setInterval(() => this.schedule(), 500);           // greys out when the data stops
  }
  setView(n){ Object.assign(this.view, VIEWS[n] || VIEWS.iso, {zoom: 1}); this.schedule(); }
  update(d){ Object.assign(this.d, d); this.schedule(); }
  schedule(){ if (!this.raf) this.raf = requestAnimationFrame(() => { this.raf = 0; this.draw(); }); }
  draw(){
    const cv = this.cv, g = this.g, dpr = window.devicePixelRatio || 1, W = cv.clientWidth, H = cv.clientHeight;
    if (W < 10 || H < 10) return;
    if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)){ cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
    g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
    const C = {ink: RU.token("--ink", "#15202b"), muted: RU.token("--muted", "#5b6672"), rule: RU.token("--rule", "#d3dae1"), signal: RU.token("--signal", "#1f5fbf"),
               live: RU.token("--live", "#157a45"), stop: RU.token("--stop", "#c41e17"), caution: RU.token("--caution", "#a86400"), ell: RU.token("--plot-3", "#0e7490"),
               panel: RU.token("--panel", "#fff"), font: RU.token("--font", "system-ui")};
    const D_ = this.d, now = performance.now();
    const links = D_.links && now - D_.linksAt < 1000 ? D_.links : null;
    const fresh = !!links, frames = D_.links;
    const {az, el, zoom} = this.view, e = [Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)];
    const r = [-Math.sin(az), Math.cos(az), 0], u = [e[1] * r[2] - e[2] * r[1], e[2] * r[0] - e[0] * r[2], e[0] * r[1] - e[1] * r[0]];
    const c0 = [0.12, 0, 0.2], s = Math.min(W, H) / (2 * 0.34) * zoom;
    const P = q => { const x = q[0] - c0[0], y = q[1] - c0[1], z = q[2] - c0[2]; return [W / 2 + (x * r[0] + y * r[1] + z * r[2]) * s, H / 2 - (x * u[0] + y * u[1] + z * u[2]) * s]; };
    const seg = (a, b, colr, w, dash) => { const A = P(a), B = P(b); g.strokeStyle = colr; g.lineWidth = w || 1; g.setLineDash(dash || []); g.beginPath(); g.moveTo(A[0], A[1]); g.lineTo(B[0], B[1]); g.stroke(); };
    const path = (pts, colr, w, dash, close) => { g.strokeStyle = colr; g.lineWidth = w || 1; g.setLineDash(dash || []); g.beginPath();
      pts.forEach((q, i) => { const a = P(q); i ? g.lineTo(a[0], a[1]) : g.moveTo(a[0], a[1]); }); if (close) g.closePath(); g.stroke(); };
    const text = (q, str, colr, dx, dy) => { const a = P(q); g.setLineDash([]); g.font = "12px " + C.font; g.fillStyle = colr; g.fillText(str, a[0] + (dx || 6), a[1] + (dy || -6)); };
    const add = (a, b, k) => [a[0] + b[0] * (k === undefined ? 1 : k), a[1] + b[1] * (k === undefined ? 1 : k), a[2] + b[2] * (k === undefined ? 1 : k)];
    const tp = (T, v) => add(T.p, mv(T.R, v));
    const triad = (T, len, w, dash, colrs) => { for (let k = 0; k < 3; k++){ const d = [0, 0, 0]; d[k] = len; seg(T.p, tp(T, d), colrs ? colrs[k] : AXC[k], w, dash); } };
    g.lineJoin = "round"; g.lineCap = "round";
    // reach rings on the base plane (0.2 m apart) and the base frame
    for (const R0 of [0.2, 0.4, 0.6]){ const pts = []; for (let k = 0; k <= 72; k++){ const t = k / 72 * 2 * Math.PI; pts.push([R0 * Math.cos(t), R0 * Math.sin(t), 0]); }
      path(pts, C.rule, 1); text([R0, 0, 0], R0.toFixed(1) + " m", C.muted, 4, 14); }
    triad({p: [0, 0, 0], R: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}, 0.08, 1.5);
    // the arm as wireframe boxes (ARM_BOXES, mm, from the URDF meshes): each box's 12 edges in its frame
    const box = (Tf, b, colr, w, sy) => { const Q = [];
      for (const i of [-1, 1]) for (const j of [-1, 1]) for (const k of [-1, 1]) Q.push(P(tp(Tf, [(b[1] + i * b[4] / 2) / 1000, sy * (b[2] + j * b[5] / 2) / 1000, (b[3] + k * b[6] / 2) / 1000])));
      g.strokeStyle = colr; g.lineWidth = w; g.setLineDash([]); g.beginPath();
      for (const [a, c] of [[0, 1], [2, 3], [4, 5], [6, 7], [0, 2], [1, 3], [4, 6], [5, 7], [0, 4], [1, 5], [2, 6], [3, 7]]){ g.moveTo(Q[a][0], Q[a][1]); g.lineTo(Q[c][0], Q[c][1]); }
      g.stroke(); };
    const I3 = {p: [0, 0, 0], R: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]};
    const ink = fresh ? C.ink : C.muted;
    for (const b of ARM_BOXES) if (b[0] === "base") box(I3, b, ink, 1.25, 1);
    if (!frames){ g.font = "13px " + C.font; g.fillStyle = C.muted; g.fillText("waiting for the arm's joint readings", 12, 22); return; }
    const T = frames.map(fr => ({p: fr.pos, R: qToR(fr.quat)}));
    const E = T[6], tcp = T[7], gap = fin(D_.gap) ? Math.max(0, Math.min(D_.gap, 0.057)) : 0;
    const FR = {link1: T[0], link2: T[1], link3: T[2], link4: T[3], link5: T[4], link6: T[5], end: E};
    // shadow on the base plane: where the arm is over the floor
    const chain = [[0, 0, 0], T[0].p, T[1].p, T[2].p, T[3].p, T[4].p, T[5].p, E.p];
    path(chain.map(q => [q[0], q[1], 0]), C.rule, 1.5, [4, 4]);
    // links and the gripper's motor block and rail
    for (const b of ARM_BOXES) if (FR[b[0]]) box(FR[b[0]], b, ink, 1.25, 1);
    // the jaws at the measured gap: each moves gap/2 along end_link y (the right one mirrored)
    for (const sgn of [1, -1]){ const Tj = {p: tp(E, [0, sgn * gap / 2, 0]), R: E.R};
      for (const b of ARM_BOXES) if (b[0] === "finger") box(Tj, b, ink, 1.5, sgn); }
    // joint axes and joints
    for (let k = 0; k < 6; k++){ const z = col(T[k].R, 2); seg(add(T[k].p, z, -0.05), add(T[k].p, z, 0.05), C.muted, 1.25, [3, 3]); }
    g.setLineDash([]);
    for (let k = 0; k < 6; k++){ const a = P(T[k].p); g.beginPath(); g.arc(a[0], a[1], 3.5, 0, 2 * Math.PI); g.fillStyle = C.panel; g.fill(); g.strokeStyle = ink; g.lineWidth = 1.5; g.stroke(); }
    // the velocity ellipsoid at the TCP: principal sections, semi-axes = singular values x 0.25 m per (m/s per rad/s)
    const K = D_.kin;
    if (K && K.lin_axes && now - D_.kinAt < 2000){
      const sm = K.sigma_min, ec = sm < SIG.crit ? C.stop : sm < SIG.warn ? C.caution : C.ell, k = 0.25;
      const ax = K.lin_axes.map((a, i) => a.map(v => v * K.lin_sigma[i] * k)), o = tcp.p;
      for (const [i, j] of [[0, 1], [1, 2], [0, 2]]){ const pts = []; for (let n = 0; n <= 64; n++){ const t = n / 64 * 2 * Math.PI; pts.push(add(add(o, ax[i], Math.cos(t)), ax[j], Math.sin(t))); }
        path(pts, ec, i === 0 && j === 1 ? 1.5 : 1, i === 0 && j === 1 ? [] : [3, 3]); }
      for (const a of ax) seg(add(o, a, -1), add(o, a), ec, 0.75, [2, 3]);
    }
    // TCP: measured (solid), commanded (dashed ghost), goal (dashed, signal), servo form target (dotted)
    const Cm = D_.cmd;
    if (Cm && fin(Cm.p[0])){ triad(Cm, 0.045, 1.5, [4, 3], [C.muted, C.muted, C.muted]); }
    const Gl = D_.goal;
    if (Gl){ seg(tcp.p, Gl.p, C.signal, 1, [2, 4]); triad(Gl, 0.06, 1.75, [6, 3], [C.signal, C.signal, C.signal]);
      seg(Gl.p, tp(Gl, [0.06, 0, 0]), C.signal, 2.5, [6, 3]); text(Gl.p, D_.goalLabel || "goal", C.signal, 8, 14); }
    const Tg = D_.target;
    if (Tg){ triad(Tg, 0.05, 1.5, [1, 3], [C.ink, C.ink, C.ink]); text(Tg.p, "target", C.ink, 8, 14); }
    triad(tcp, 0.05, 2.25);
    text(tcp.p, "tcp", ink, -26, -6);
    if (!fresh){ g.font = "13px " + C.font; g.fillStyle = C.muted; g.fillText("no reading for over 1 s: showing the last one", 12, 22); }
  }
}

// -- hold-to-move controls: SVG, pointer-captured, spring back to centre on release ----------------------
// kind "xy" (2 axes), "v" (vertical), "h" (horizontal). onMove(x, y) with x right +, y up +, in [-1, 1].
function Stick(el, kind, labels, onMove){
  const S = {xy: [200, 200], v: [60, 200], h: [275, 60]}[kind];
  const t = (x, y, s, a) => `<text x="${x}" y="${y}" text-anchor="${a || "middle"}" dominant-baseline="middle">${s}</text>`;
  let body;
  if (kind === "xy") body = `<circle class="ring" cx="100" cy="100" r="92"/><circle class="faint" cx="100" cy="100" r="46" stroke-dasharray="3 4"/>
    <line class="faint" x1="8" y1="100" x2="192" y2="100"/><line class="faint" x1="100" y1="8" x2="100" y2="192"/>
    ${t(100, 22, labels[0])}${t(100, 180, labels[1])}${t(22, 100, labels[2])}${t(178, 100, labels[3])}<circle class="knob" cx="100" cy="100" r="17"/>`;
  else if (kind === "v") body = `<rect class="ring" x="12" y="6" width="36" height="188" rx="18"/><line class="faint" x1="18" y1="100" x2="42" y2="100"/>
    ${t(30, 24, labels[0])}${t(30, 178, labels[1])}<circle class="knob" cx="30" cy="100" r="14"/>`;
  else body = `<rect class="ring" x="6" y="12" width="263" height="36" rx="18"/><line class="faint" x1="137.5" y1="18" x2="137.5" y2="42"/>
    ${t(40, 30, labels[0])}${t(235, 30, labels[1])}<circle class="knob" cx="137.5" cy="30" r="14"/>`;
  el.innerHTML = `<svg class="stick ${kind}" viewBox="0 0 ${S[0]} ${S[1]}" tabindex="0" role="slider" aria-label="${labels.join(", ")}">${body}</svg>`;
  const svg = el.querySelector("svg"), knob = svg.querySelector(".knob"), Rm = kind === "xy" ? 72 : kind === "v" ? 72 : 100;
  const cx = S[0] / 2, cy = S[1] / 2;
  let id = null;
  const shape = v => { const a = Math.abs(v); return a < 0.06 ? 0 : Math.sign(v) * Math.pow((a - 0.06) / 0.94, 1.6); };
  function set(x, y){ knob.setAttribute("cx", cx + x * Rm); knob.setAttribute("cy", cy - y * Rm); onMove(shape(x), shape(y)); }
  function at(e){ const b = svg.getBoundingClientRect(); let x = ((e.clientX - b.left) / b.width * S[0] - cx) / Rm, y = -((e.clientY - b.top) / b.height * S[1] - cy) / Rm;
    if (kind === "v") x = 0; if (kind === "h") y = 0;
    const n = Math.hypot(x, y); if (n > 1){ x /= n; y /= n; } set(x, y); }
  function release(){ id = null; svg.classList.remove("on"); set(0, 0); }
  svg.addEventListener("pointerdown", e => { if (!ready()) return; id = e.pointerId; svg.setPointerCapture(id); svg.classList.add("on"); at(e); e.preventDefault(); });
  svg.addEventListener("pointermove", e => { if (id === e.pointerId) at(e); });
  for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) svg.addEventListener(type, release);
  // keyboard: arrows hold while pressed (focus the control first)
  const keys = new Set(), keyMove = () => { const x = (keys.has("ArrowRight") ? 1 : 0) - (keys.has("ArrowLeft") ? 1 : 0), y = (keys.has("ArrowUp") ? 1 : 0) - (keys.has("ArrowDown") ? 1 : 0);
    svg.classList.toggle("on", !!(x || y)); set(kind === "v" ? 0 : x * 0.6, kind === "h" ? 0 : y * 0.6); };
  svg.addEventListener("keydown", e => { if (!e.key.startsWith("Arrow")) return; e.preventDefault(); if (!ready()) return; keys.add(e.key); keyMove(); });
  svg.addEventListener("keyup", e => { if (keys.delete(e.key)) keyMove(); });
  svg.addEventListener("blur", () => { keys.clear(); release(); });
  return {release(){ keys.clear(); release(); }};
}

// -- state ---------------------------------------------------------------------------------------------
let st = null, last = null, lastAt = 0, kinCmd = null;
const view = new ArmView($("#view"));
document.querySelectorAll("[data-v]").forEach(b => b.onclick = () => view.setView(b.dataset.v));
const hasLease = () => !!lease.token;
const ready = () => !!(st && hasLease() && st.enabled && !(st.estop && st.estop.latched));

// -- jog engine: one track client for all four controls -------------------------------------------------
const J = {xy: [0, 0], z: 0, py: [0, 0], roll: 0};
const sticks = [
  Stick($("#j_xy"), "xy", ["+x", "−x", "+y", "−y"], (x, y) => { J.xy = [x, y]; }),
  Stick($("#j_z"), "v", ["+z", "−z"], (x, y) => { J.z = y; }),
  Stick($("#j_py"), "xy", ["+pitch", "−pitch", "−yaw", "+yaw"], (x, y) => { J.py = [x, y]; }),
  Stick($("#j_roll"), "h", ["−roll", "+roll"], (x, y) => { J.roll = x; }),
];
const held = () => !!(J.xy[0] || J.xy[1] || J.z || J.py[0] || J.py[1] || J.roll);
function releaseAll(){ sticks.forEach(s => s.release()); }
window.addEventListener("blur", releaseAll);
document.addEventListener("visibilitychange", () => { if (document.hidden) releaseAll(); });
let jp = null, jSeq = 0, jBusy = false, jT = 0;
const jogNote = m => { $("#jognote").textContent = m || (ready() ? "hold to move; let go to stop" : "take the lease and enable to jog"); };
async function jogBase(){
  // each hold starts from the MEASURED TCP pose (never a command); the target is integrated from there
  const k = await RU.api("kinematics");
  return {p: k.tcp.pos.slice(), R: qToR(k.tcp.quat)};
}
function jogSpeeds(){
  const L = st && st.limits ? st.limits.soft : {lin_vel: 0.15, ang_vel: 1.0};
  return [Math.min(Math.max(+$("#j_lin").value || 0, 0) / 1000, L.lin_vel), Math.min(Math.max(+$("#j_ang").value || 0, 0) / D, L.ang_vel)];
}
setInterval(async () => {
  if (!held()){ jp = null; return; }          // released: nothing more is sent; the twist times out into a hold
  if (!ready()){ jp = null; return; }         // never resume from a pose taken before a disable / e-stop
  if (jBusy) return;
  jBusy = true;
  try {
    if (!jp){ jp = await jogBase(); jT = performance.now(); }
    const now = performance.now(), dt = Math.min(0.1, (now - jT) / 1000); jT = now;
    const [L, A] = jogSpeeds();
    const v = [J.xy[1] * L, -J.xy[0] * L, J.z * L];
    const wt = [J.roll * A, J.py[1] * A, J.py[0] * A];               // about the TCP's x (roll), y (pitch), z (yaw)
    const next = {p: jp.p.map((x, i) => x + v[i] * dt), R: mm(jp.R, expR(wt.map(x => x * dt)))};
    if (!held()) return;
    const seq = ++jSeq;
    const r = await RU.api("track", {body: {pose: {pos: next.p, quat: rToQ(next.R)}, twist: v.concat(mv(next.R, wt)), twist_frame: "base", seq}, headers: lease.headers()});
    if (r.ok){ jp = next;
      jogNote(r.limited ? "the step limit is clipping: the arm lags the jog" : r.twist_scaled ? "jog speed capped at the soft limit" : "moving"); }
    else { jp = null;                          // refused: restart from the measured pose, never from a stale target
      jogNote(r.reason ? `not moved: ${r.reason}` : r.ik ? `out of reach here (IK ${f(r.ik.pos_err_mm, 1)} mm, ${f(r.ik.rot_err_deg, 1)}° off): back off` : "not moved"); }
  } catch(e){ err(e.message); jp = null; }
  finally { jBusy = false; }
}, 40);

// -- servo panel ---------------------------------------------------------------------------------------
const SF = ["#s_x", "#s_y", "#s_z", "#s_r", "#s_p", "#s_yw"];
let sEdited = false, sShownAt = 0;
SF.forEach(id => $(id).addEventListener("input", () => { sEdited = true; srcNote(); }));
const srcNote = () => { $("#s_src").textContent = sEdited ? "edited: the dotted frame is the target" : "follows the measured TCP until you edit"; };
function servoFollow(force){
  if (sEdited || !last || !fin(last.x)) return;
  if (!force && performance.now() - sShownAt < 500) return;
  if (!force && SF.some(id => document.activeElement === $(id))) return;
  sShownAt = performance.now();
  const rpy = rToRpy(qToR([last.qx, last.qy, last.qz, last.qw]));
  [last.x * 1000, last.y * 1000, last.z * 1000].forEach((v, i) => $(SF[i]).value = v.toFixed(1));
  rpy.forEach((v, i) => $(SF[3 + i]).value = (Math.abs(v * D) < 0.05 ? 0 : v * D).toFixed(1));
}
function servoTarget(){
  const v = SF.map(id => $(id).value === "" ? NaN : Number($(id).value));
  if (!v.every(isFinite)) return null;
  return {pos: v.slice(0, 3).map(x => x / 1000), rpy: v.slice(3).map(x => x / D)};
}
$("#s_follow").onclick = () => { sEdited = false; srcNote(); servoFollow(true); };
$("#go").onclick = async () => {
  const t = servoTarget(); if (!t){ err("fill in x, y, z, roll, pitch and yaw"); return; }
  const b = {pose: t}, num = (id, k) => { const v = $(id).value; if (v !== "" && Number(v) > 0) b[k] = Number(v); };
  num("#s_lv", "lin_vel"); num("#s_la", "lin_acc"); num("#s_av", "ang_vel"); num("#s_aa", "ang_acc");
  if (b.lin_vel) b.lin_vel /= 1000; if (b.lin_acc) b.lin_acc /= 1000; if (b.ang_vel) b.ang_vel /= D; if (b.ang_acc) b.ang_acc /= D;
  $("#go").textContent = "Planning…"; $("#go").dataset.busy = "1"; render();
  const r = await call("servo", b);
  $("#go").textContent = "Go"; delete $("#go").dataset.busy; render();
  if (r){ sEdited = false; srcNote(); }
};
$("#s_cancel").onclick = () => call("cancel");
$("#cancel").onclick = () => call("cancel");
$("#home").onclick = () => call("home");

// -- gripper -------------------------------------------------------------------------------------------
// max force applies to this command only; blank = the node's default (50 N)
const grip = gap => { const b = {gap_mm: gap}, F = $("#g_force").value; if (F !== "") b.max_force_n = Number(F); return call("gripper", b); };
$("#g_set").onclick = () => grip(Number($("#g_gap").value));
$("#g_open").onclick = () => { const m = st ? st.gripper.gap_max_mm : 49.6; $("#g_gap").value = m; grip(m); };
$("#g_close").onclick = () => { $("#g_gap").value = 0; grip(0); };

// -- control row ---------------------------------------------------------------------------------------
let enabling = false;
$("#enable").onclick = async () => {
  enabling = true; render();
  $("#enable").textContent = st && !st.initialized ? "Checking the gripper…" : "Enabling…";
  await call("enable");
  enabling = false; $("#enable").textContent = "Enable"; render();
};
$("#disable").onclick = () => { releaseAll(); jp = null; return call("disable"); };
// Float / Disable win over a held control: it is released, and jogging needs a fresh press
$("#float").onclick = () => { releaseAll(); jp = null; return call("float"); };
$("#stow").onclick = async () => { const r = await call("stow"); if (r && r.stowed) $("#s_phase").textContent = "stowed: already at home, motors off"; };
$("#clear").onclick = () => call("estop/clear");
$("#reset").onclick = () => call("reset");

// -- settings ------------------------------------------------------------------------------------------
// soft limits in page units; the joint ones are 6 values each (comma separated)
const LIM = [["lin_vel", "speed", "mm/s", 1000], ["lin_acc", "accel", "mm/s²", 1000], ["ang_vel", "turn", "°/s", D], ["ang_acc", "ang. accel", "°/s²", D],
             ["home_vel", "home", "°/s", D], ["", "", "", 0],
             ["joint_vel", "joint speed", "°/s, joints 1-6", D, true], ["joint_acc", "joint accel", "°/s², joints 1-6", D, true]];
$("#limform").innerHTML = LIM.map(([k, lab, u, , vec]) => !k ? "<span></span><span></span>" : vec
  ? `<span title="${u}">${lab}</span><input id="l_${k}" type="text" inputmode="decimal" title="${u}" style="grid-column:2/-1">`
  : `<span title="${u}">${lab}</span><input id="l_${k}" type="number" step="any" min="0" title="${u}">`).join("");
const limGet = (k, sc, vec) => { const v = $("#l_" + k).value.trim(); if (v === "") return undefined;
  if (!vec) return Number(v) / sc;
  const a = v.split(/[,\s]+/).filter(Boolean).map(Number); if (a.length !== 6 || !a.every(isFinite)) throw new Error(`${k}: give 6 numbers`); return a.map(x => x / sc); };
const limShow = (v, sc, vec) => vec ? v.map(x => +(x * sc).toFixed(0)).join(", ") : +(v * sc).toFixed(1);
let limDirty = false, tcpDirty = false, profShown = null;
const limEdited = new Set();      // send only what was edited: shown values are rounded
$("#limform").addEventListener("input", e => { limDirty = true; limEdited.add(e.target.id.slice(2)); });
["#c_tx", "#c_ty", "#c_tz"].forEach(id => $(id).addEventListener("input", () => tcpDirty = true));
$("#c_lim_set").onclick = async () => { const b = {};
  try { for (const [k, , , sc, vec] of LIM) if (k && limEdited.has(k)){ const v = limGet(k, sc, vec); if (v !== undefined) b[k] = v; } } catch(e){ err(e.message); return; }
  if (await call("limits", b)){ limDirty = false; limEdited.clear(); } };
$("#c_prof_set").onclick = () => call("profile", {name: $("#c_prof").value});
$("#c_tcp_set").onclick = async () => { if (await call("tcp", {offset: ["#c_tx", "#c_ty", "#c_tz"].map(id => Number($(id).value) / 1000)})) tcpDirty = false; };

// what each state means for someone standing next to the arm: [pill, sentence]
const MODES = {
  connecting: ["connecting", "Opening the bus; motors off."],
  uninitialized: ["motors off", "Connected, motors off. Enable checks the gripper once, then the arm floats."],
  initializing: ["checking the gripper", "Closing the gripper to check it."],
  off: ["off: damping", "Motors off, joints in damping: the arm resists fast motion and settles under its weight."],
  disabled: ["floating, disabled", "The arm floats (weightless, movable by hand); the gripper is in damping; commands are refused until Enable."],
  floating: ["floating", "The arm floats, weightless: you can move it by hand. A jog, servo or Home engages servoing from where it is."],
  servoing: ["servoing", "The arm holds its pose and follows commands. Float to move it by hand."],
  fault: ["fault", "Motors off after a fault. Reset reconnects."],
};
function modeLine(s){
  const m = MODES[s.state] || [s.state || "…", ""], latched = !!(s.estop && s.estop.latched);
  $("#mode").textContent = latched ? "e-stop: off" : m[0];
  $("#mode").className = "ru-pill " + (latched || s.state === "fault" ? "ru-err" : s.state === "servoing" ? "ru-ok" : ["floating", "disabled"].includes(s.state) ? "ru-warn" : "");
  $("#modetext").textContent = latched ? "E-stop latched: motors off (damping). The holder clears it, then Enable." : m[1];
}

function render(){
  const s = st || {}, latched = !!(s.estop && s.estop.latched), has = hasLease(), en = !!s.enabled;
  const busy = ["connecting", "initializing", "fault"].includes(s.state);
  $("#enable").disabled = enabling || !has || latched || !!s.chocked || en || busy;
  $("#enable").title = s.chocked ? "Chocked: an admin removes the chock over SSH" : latched ? "Clear the e-stop first" : !has ? "Take the lease first" : "";
  $("#disable").disabled = !has || !en;
  $("#float").disabled = !has || !en || !s.servo || latched;
  $("#stow").disabled = !ready();
  $("#clear").hidden = !latched; $("#clear").disabled = !has;
  $("#reset").hidden = s.state !== "fault"; $("#reset").disabled = !has;
  const can = ready();
  for (const id of ["#home", "#go", "#s_cancel", "#g_set", "#g_open", "#g_close"]) $(id).disabled = !can;
  $("#cancel").disabled = !can || !s.servo;
  if ($("#go").dataset.busy) $("#go").disabled = true;
  document.querySelectorAll("#named button").forEach(b => b.disabled = !can);
  $("#jog").classList.toggle("off", !can);
  if (!can) releaseAll();
  jogNote(held() ? $("#jognote").textContent : "");
  $("#c_prof_set").disabled = !has || !!s.servo || !!s.gripper_engaged;
  $("#c_tcp_set").disabled = !has || !!s.servo;
  $("#c_lim_set").disabled = !has;
  $("#set_note").textContent = !has ? "take the lease to change" : s.servo || s.gripper_engaged ? "Float or Disable to change gains" + (s.servo ? " or TCP" : "") : "";
  modeLine(s);
}

let wasServo = false;
RU.sse("events", x => {
  st = x;
  // the arm stopped servoing (float, disable, e-stop, fault, stow): it may sink or be moved by hand, so
  // a held control restarts from the measured pose
  if ((wasServo && !st.servo) || !ready()) jp = null;
  wasServo = !!st.servo;
  lease.update(st); estop.update(st);
  const latched = st.estop && st.estop.latched;
  band.update(st, {live: !!st.motors_on, liveText: MODES[st.state] ? MODES[st.state][1] : ""});
  $("#state").textContent = st.chocked ? "chocked" : latched ? "e-stop latched" : st.state + (st.sim ? ", sim" : "") + (st.loop_hz ? ", " + st.loop_hz + " Hz" : "");
  $("#state").className = "ru-pill " + (st.chocked || latched || st.state === "fault" ? "ru-err" : st.state === "servoing" ? "ru-ok" : ["connecting", "initializing", "floating", "disabled"].includes(st.state) ? "ru-warn" : "");
  if (st.error) err(st.error);
  // named poses
  const names = Object.keys(st.named_poses || {});
  if ($("#named").dataset.k !== names.join(",")){ $("#named").dataset.k = names.join(",");
    $("#named").innerHTML = names.length ? '<span class="hint ru-muted">named poses</span>' + names.map(n => `<button class="ru" data-pose="${n}">${n}</button>`).join("") : "";
    $("#named").querySelectorAll("[data-pose]").forEach(b => b.onclick = () => call("servo", {name: b.dataset.pose})); }
  // motion
  const m = st.motion;
  if (m){ const goal = m.phase === "executing" || m.phase === "settling";
    $("#s_phase").textContent = `${m.kind}${m.name ? " " + m.name : ""} #${m.id}: ${m.phase.replace("_", " ")}` + (goal ? ` ${Math.round(m.progress * 100)} % of ${f(m.duration, 1)} s` : "") + (m.reason ? `: ${m.reason}` : "");
    $("#s_phase").className = "ru-num " + (["aborted", "not_reached"].includes(m.phase) ? "ru-warn" : m.phase === "reached" || m.phase === "done" ? "ru-ok" : "");
    $("#s_bar").style.width = (Math.max(0, Math.min(1, m.progress)) * 100) + "%"; }
  // goal ghost: the planned move's goal while it runs, else the track goal
  let G = null, gl = "";
  if (m && (m.phase === "executing" || m.phase === "settling") && m.goal){ G = m.goal; gl = m.kind === "servo" ? "servo goal" : (m.name || m.kind) + " goal"; }
  else if (st.source === "track" && st.track && st.track.goal){ G = st.track.goal; gl = "track goal"; }
  view.update({goal: G ? {p: G.pos, R: qToR(G.quat)} : null, goalLabel: gl});
  // gripper, settings
  const gr = st.gripper || {};
  $("#g_meas").textContent = `${gr.mode || "–"}, ${f(gr.gap_mm, 1)} mm` + (fin(gr.force_n) ? `, ${f(gr.force_n, 1)} N` : "") + (gr.limiting ? ", force limited" : "");
  $("#g_force").placeholder = f(gr.default_max_force_n, 0);
  $("#g_gap").max = gr.gap_max_mm; $("#g_force").max = 120;
  const pr = st.profiles || [];
  if (profShown !== pr.join(",") + st.profile){ profShown = pr.join(",") + st.profile; $("#c_prof").innerHTML = pr.map(p => `<option ${p === st.profile ? "selected" : ""}>${p}</option>`).join(""); }
  if (!tcpDirty && st.tcp_offset) ["#c_tx", "#c_ty", "#c_tz"].forEach((id, i) => { if (document.activeElement !== $(id)) $(id).value = (st.tcp_offset[i] * 1000).toFixed(1); });
  const L = st.limits;
  if (L){ if (!limDirty) for (const [k, , , sc, vec] of LIM) if (k && L.soft[k] !== undefined && document.activeElement !== $("#l_" + k)) $("#l_" + k).value = limShow(L.soft[k], sc, vec);
    $("#hardlim").textContent = `hard: ${f(L.hard.lin_vel * 1000, 0)} mm/s, ${f(L.hard.ang_vel * D, 0)} °/s; joints ${L.hard.joint_vel.map(v => f(v * D, 0)).join("/")} °/s` +
      (L.hard.joint_acc ? `, ${L.hard.joint_acc.map(v => f(v * D, 0)).join("/")} °/s²` : "");
    $("#s_lv").placeholder = f(L.soft.lin_vel * 1000, 0); $("#s_la").placeholder = f(L.soft.lin_acc * 1000, 0);
    $("#s_av").placeholder = f(L.soft.ang_vel * D, 0); $("#s_aa").placeholder = f(L.soft.ang_acc * D, 0);
    $("#j_lin").max = f(L.soft.lin_vel * 1000, 0); $("#j_ang").max = f(L.soft.ang_vel * D, 0); }
  $("#events").innerHTML = st.events.slice().reverse().map(e => `<div class="${e.level === "error" ? "ru-ev-error" : (e.level === "warn" || e.level === "warning") ? "ru-ev-warn" : ""}">${new Date(e.wall * 1000).toLocaleTimeString()} ${e.msg}</div>`).join("");
  render(); kv();
});

// -- telemetry: one stream, thinned to 30 rows/s -----------------------------------------------------------
const plot = new RU.Plot($("#plot"), {seconds: 10, series: [{name: "measured vs command"}, {name: "command vs goal"}]});
RU.rows("stream?max_hz=30", d => {
  for (let i = 0; i < d.n; i++){ const g = n => d.raw[i * d.fields.length + d.idx[n]];
    plot.push(g("t_tick"), [g("err_pos") * 1000, g("goal_err_pos") * 1000]); }
  const row = {}; for (const n of d.fields) row[n] = d.last(n);
  last = row; lastAt = performance.now();
  view.update({gap: row.g_gap});
  if (row.servo === 1 && fin(row.cmd_x) && kinCmd) view.update({cmd: {p: [row.cmd_x, row.cmd_y, row.cmd_z], R: qToR(kinCmd.tcp.quat)}});
  else view.update({cmd: null});
  const sm = row.sigma_min;
  $("#m_w").textContent = f(row.w, 4); $("#m_s").textContent = f(sm, 4); $("#m_wl").textContent = f(row.w_lin, 4);
  const lvl = !fin(sm) ? "" : sm < SIG.crit ? "ru-err" : sm < SIG.warn ? "ru-warn" : "";
  $("#m_s").className = "v " + lvl;
  $("#m_msg").className = "msg " + (lvl || "ru-muted");
  $("#m_msg").textContent = !fin(sm) ? "" : sm < SIG.crit ? "at a singularity: some directions need very large joint speeds"
    : sm < SIG.warn ? `near a singularity (σmin under ${SIG.warn}): motion in some directions is slow or jerky` : "clear of singularities";
  servoFollow(false);
});
// the servo form's target, drawn while edited
setInterval(() => { const t = sEdited ? servoTarget() : null; view.update({target: t ? {p: t.pos, R: rpyToR(...t.rpy)} : null}); }, 200);

function kv(){
  const s = st || {}, r = last || {};
  const age = lastAt ? (performance.now() - lastAt) / 1000 : null;
  const rows = [
    ["servo", `arm ${r.servo === 1 ? "servoing" : "not servoing"}, gripper ${r.g_servo === 1 ? "holding" : (s.gripper || {}).mode || "–"}`],
    ["source", (s.source || "–") + (s.source === "track" && s.track ? `, seq ${s.track.seq}, ${f(s.track.age, 2)} s ago` : "")],
    ["tracking", `${f(r.err_pos * 1000, 1)} mm, ${f(r.err_rot * D, 2)}° (measured vs command)`],
    ["lag to goal", `${f(r.goal_err_pos * 1000, 1)} mm, ${f(r.goal_err_rot * D, 2)}°` + (r.limited ? ", step limit clipping" : "")],
    ["TCP", `${f(r.x * 1000, 1)}, ${f(r.y * 1000, 1)}, ${f(r.z * 1000, 1)} mm`],
    ["gains", `${s.profile || "–"}; TCP offset ${s.tcp_offset ? s.tcp_offset.map(v => f(v * 1000, 0)).join(", ") + " mm" : "–"}`],
    ["loop", `${f(s.loop_hz, 1)} Hz, tick ${f(r.tick_dt * 1000, 1)} ms, overruns ${fin(s.overruns) ? s.overruns : "–"}`],
    ["stream", age === null ? "waiting" : age > 1 ? `stopped ${f(age, 0)} s ago` : "live, 30 rows/s"],
  ];
  $("#kv").innerHTML = rows.map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join("");
  if (last){ const J6 = [1, 2, 3, 4, 5, 6], g = (i, n) => r[`j${i}_${n}`];
    $("#joints").innerHTML = "<tr><th>joint</th><th>q °</th><th>cmd °</th><th>err °</th><th>τ Nm</th><th>τ<sub>g</sub></th><th>°C</th><th title='the acceleration limit held back this joint&#39;s speed increase'>acc lim</th></tr>" +
      J6.map(i => `<tr><td>${i}</td><td>${f(g(i, "q") * D, 1)}</td><td>${f(g(i, "q_cmd") * D, 1)}</td><td>${f(g(i, "e_mot") * D, 2)}</td><td>${f(g(i, "tau"), 2)}</td><td>${f(g(i, "tau_g"), 2)}</td><td>${f(g(i, "t_mos"), 0)}</td><td>${g(i, "acc_lim") === 1 ? "held" : ""}</td></tr>`).join(""); }
}
setInterval(kv, 250);

// -- drawing data: links at 10 Hz, kinematics at 5 Hz (measured) and 2.5 Hz while enabled (commanded) ---------
let pollN = 0;
async function poll(){
  const t0 = performance.now();
  if (!document.hidden){
    pollN++;
    const jobs = [RU.api("links").then(x => view.update({links: x.frames, linksAt: performance.now()}))];
    if (pollN % 2 === 0) jobs.push(RU.api("kinematics").then(k => view.update({kin: k, kinAt: performance.now()})));
    else if (pollN % 4 === 1 && st && st.servo) jobs.push(RU.api("kinematics?which=command").then(k => { kinCmd = k; }, () => { kinCmd = null; }));
    try { await Promise.all(jobs); $("#viewnote").textContent = ""; } catch(e){ $("#viewnote").textContent = "drawing: " + e.message; }
  }
  setTimeout(poll, Math.max(20, 100 - (performance.now() - t0)));
}
poll();
function frame(){ plot.draw(); requestAnimationFrame(frame); }
requestAnimationFrame(frame);
render();
</script></body></html>
""".replace("__ARM_BOXES__", json.dumps(ARM_BOXES))
