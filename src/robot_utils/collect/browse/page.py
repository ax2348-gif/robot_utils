"""The browser page (one HTML string): library, the replay stage (robot 3D, cameras, operator 3D), transport,
timeline, plots and the export panel. Plain JS, no external libraries; the shared toolkit and ``RU.viz``
(``common/ui.css``, ``common/viz.js``: the arm wireframe) come from the node assets. docs/browse.md describes it.
"""

PAGE_HTML = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Episode browser</title>
<link rel="stylesheet" href="common/ui.css">
<script src="common/viz.js"></script>
<style>
:root{--side:250px}
body{margin:0;height:100vh;display:flex;flex-direction:column;overflow:hidden}
header{display:flex;align-items:center;gap:14px;padding:8px 14px;border-bottom:1px solid var(--rule)}
header h1{font-size:15px;margin:0;font-weight:600}
header .src{color:var(--muted);font-size:12px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;flex:1}
button,select,input{font:inherit;font-size:13px;color:var(--ink);background:var(--panel);border:1px solid var(--rule);border-radius:var(--radius);padding:3px 8px}
button{cursor:pointer} button:hover{border-color:var(--muted)} button.on{border-color:var(--signal);color:var(--signal)}
button:disabled{opacity:.5;cursor:default}
.wrap{flex:1;display:flex;min-height:0}
aside{width:var(--side);border-right:1px solid var(--rule);overflow:auto;padding:8px 0;flex:none}
aside.hidden{display:none}
aside .it{padding:4px 12px;cursor:pointer;font-size:13px;line-height:1.35}
aside .it:hover{background:var(--signal-bg)} aside .it.sel{color:var(--signal)}
aside .ses{margin-top:6px;font-weight:600}
aside .ep{padding-left:22px;font-variant-numeric:tabular-nums}
aside .tag{font-size:11px;color:var(--muted)} aside .tag.ab{color:var(--caution)}
main{flex:1;min-width:0;display:flex;flex-direction:column;overflow:auto}
.stage{display:grid;grid-template-columns:minmax(0,1.75fr) minmax(0,1fr);gap:10px;padding:10px 12px 0;height:clamp(420px,64vh,820px);flex:none}
.col{display:flex;flex-direction:column;gap:10px;min-width:0;min-height:0}
.stage>.col,#cams{height:100%} #cams{flex:2} #rightcol>.panel{flex:1}
#cams>.panel{flex:1;min-height:0}
.panel{position:relative;border:1px solid var(--rule);border-radius:var(--radius);background:var(--panel);overflow:hidden}
.panel>canvas{display:block;width:100%;height:100%}
.panel .lab{position:absolute;left:8px;top:5px;font-size:11px;color:var(--muted);pointer-events:none;letter-spacing:.02em}
.panel .tools{position:absolute;right:6px;top:4px;display:flex;gap:4px}
.panel .tools button{font-size:11px;padding:1px 6px}
#robot{flex:1;min-height:0}
#operator{min-height:0}
.cam .lab{background:var(--panel);padding:0 5px;border-radius:3px;opacity:.85}
.transport{display:flex;align-items:center;gap:8px;padding:8px 12px;flex-wrap:wrap}
.transport .time{font:13px var(--mono);font-variant-numeric:tabular-nums;min-width:200px}
.transport .sp{flex:1}
#timeline{height:64px;margin:0 12px;border:1px solid var(--rule);border-radius:var(--radius);background:var(--panel);cursor:pointer;position:relative}
#timeline canvas{width:100%;height:100%;display:block}
.plots{padding:10px 12px 16px;display:flex;flex-direction:column;gap:6px}
.plot{height:118px}
.plot .lab{pointer-events:auto}
.plot .legend{position:absolute;left:8px;top:4px;font-size:11px;display:flex;gap:10px;flex-wrap:wrap;right:70px}
.plot .legend span{white-space:nowrap}
.addplot{display:flex;gap:6px;align-items:center;flex-wrap:wrap;font-size:13px}
.addplot input{width:220px}
#fieldlist{max-height:160px;overflow:auto;border:1px solid var(--rule);border-radius:var(--radius);display:none;font:12px var(--mono);background:var(--panel)}
#fieldlist div{padding:2px 8px;cursor:pointer} #fieldlist div:hover{background:var(--signal-bg)}
#exp{position:fixed;right:14px;top:52px;width:360px;max-height:calc(100vh - 70px);overflow:auto;background:var(--panel);border:1px solid var(--rule);border-radius:var(--radius);padding:12px 14px;display:none;box-shadow:0 6px 24px rgba(0,0,0,.12)}
#exp h2{font-size:14px;margin:0 0 8px}
#exp label{display:flex;justify-content:space-between;align-items:center;gap:8px;margin:5px 0;font-size:13px}
#exp .row{display:flex;gap:6px;flex-wrap:wrap;align-items:center;margin:5px 0;font-size:13px}
#exp .est{font:12px var(--mono);margin:8px 0;padding:6px 8px;border:1px solid var(--rule);border-radius:var(--radius)}
#exp progress{width:100%}
#exp input[type=number]{width:80px}
.muted{color:var(--muted)} .warn{color:var(--caution)} .err{color:var(--stop)}
.hidden{display:none!important}
#views{position:fixed;right:14px;top:52px;background:var(--panel);border:1px solid var(--rule);border-radius:var(--radius);padding:8px 12px;display:none;font-size:13px}
#views label{display:block;margin:3px 0}
#tip{position:fixed;pointer-events:none;background:var(--panel);border:1px solid var(--rule);border-radius:4px;padding:2px 6px;font-size:12px;display:none;z-index:9}
@media (max-width:900px){.stage{grid-template-columns:1fr;height:auto}.stage>.col,#cams{height:auto}#robot,.cam,#operator{aspect-ratio:4/3}}
</style></head>
<body data-page="browse">
<header class="ru-head">
  <button id="sidebtn" title="library">&#9776;</button>
  <h1 id="title">Episode browser</h1>
  <div class="src" id="src"></div>
  <button id="viewbtn">View</button>
  <button id="expbtn">Export video</button>
</header>
<div class="wrap">
<aside id="side"></aside>
<main>
  <div class="stage">
    <div class="col">
      <div class="panel" id="robot" data-panel="robot"><canvas></canvas><div class="lab">robot · world</div>
        <div class="tools"><button data-v="follow" class="on" title="keep the robot centred">follow</button><button data-v="iso">3D</button><button data-v="top">top</button><button data-v="side">side</button></div></div>
    </div>
    <div class="col" id="rightcol">
      <div id="cams" class="col"></div>
      <div class="panel" id="operator" data-panel="operator"><canvas></canvas><div class="lab">operator · headset frame (FLU)</div>
        <div class="tools"><button data-v="iso">3D</button><button data-v="top">top</button><button data-v="side">side</button></div></div>
    </div>
  </div>
  <div class="transport">
    <button id="start" title="to start (Home)">&#x23EE;</button>
    <button id="back" title="step back (&larr;; shift: 1 s)">&#x25C0;&#x25C0;</button>
    <button id="play" title="play / pause (space)" style="min-width:56px">play</button>
    <button id="fwd" title="step forward (&rarr;; shift: 1 s)">&#x25B6;&#x25B6;</button>
    <select id="rate" title="replay rate ([ and ])"></select>
    <label class="muted" style="font-size:13px"><input type="checkbox" id="loop" checked> loop</label>
    <span class="time" id="time">-</span>
    <span class="sp"></span>
    <button id="inbtn" title="mark in (I)">in</button><button id="outbtn" title="mark out (O)">out</button><button id="clearbtn" title="clear in / out">&times;</button>
    <span class="muted" id="inout" style="font-size:12px"></span>
  </div>
  <div id="timeline"><canvas></canvas></div>
  <div class="plots" id="plots"></div>
  <div style="padding:0 12px 18px" class="addplot">
    <span class="muted">add a plot:</span>
    <select id="addstream"></select>
    <input id="addfield" placeholder="field (type to filter, several with commas)">
    <button id="addbtn">add</button>
  </div>
  <div style="padding:0 12px 18px"><div id="fieldlist"></div></div>
</main>
</div>
<div id="views"></div>
<div id="exp">
  <h2>Export video</h2>
  <div class="row muted" id="expffmpeg"></div>
  <label>range <select id="xrange"><option value="view">episode / view</option><option value="inout">in &ndash; out</option><option value="all">whole recording</option></select></label>
  <label>source <select id="xsource"></select></label>
  <label>replay rate <select id="xrate"></select></label>
  <label>resolution <select id="xres"><option>640x360</option><option>960x540</option><option selected>1280x720</option><option>1920x1080</option></select></label>
  <label>fps <select id="xfps"><option>15</option><option>24</option><option selected>30</option><option>60</option></select></label>
  <label>preset <select id="xpreset"></select></label>
  <label>size <span><select id="xmode"><option value="quality">by quality</option><option value="size">target size</option></select></span></label>
  <label id="xqrow">quality (CRF; lower = better) <input type="number" id="xcrf" min="10" max="40" step="1"></label>
  <label id="xsrow" class="hidden">target (MB) <input type="number" id="xmb" min="0.2" step="0.5" value="20"></label>
  <label id="xprow" class="hidden">two-pass (exact size) <input type="checkbox" id="xtwo" checked></label>
  <div class="est" id="xest">-</div>
  <div class="row"><button id="xprobe" title="encode a few seconds of real frames with these settings">measure</button><button id="xgo">export</button><button id="xcancel" class="hidden">cancel</button><span class="sp"></span><button id="xclose">close</button></div>
  <progress id="xprog" max="1" value="0" class="hidden"></progress>
  <div class="row" id="xstatus" style="font-size:12px"></div>
</div>
<div id="tip"></div>
<script>
"use strict";
const $ = s => document.querySelector(s);
const fin = v => v !== null && v !== undefined && Number.isFinite(v);
const tok = (n, f) => (getComputedStyle(document.documentElement).getPropertyValue(n) || "").trim() || f;
let C = {};
function colours(){ C = {ink: tok("--ink", "#15202b"), muted: tok("--muted", "#5b6672"), rule: tok("--rule", "#d3dae1"),
  signal: tok("--signal", "#1f5fbf"), live: tok("--live", "#157a45"), stop: tok("--stop", "#c41e17"), caution: tok("--caution", "#a86400"),
  panel: tok("--panel", "#fff"), ground: tok("--ground", "#eef1f4"), sigbg: tok("--signal-bg", "#e3ecfa"),
  p: [1, 2, 3, 4, 5].map(i => tok("--plot-" + i, "#1f5fbf")), font: tok("--font", "system-ui"), mono: tok("--mono", "monospace")}; }
colours(); setInterval(colours, 2000);
const api = async (p, opt) => { const r = await fetch(p, opt); if (!r.ok){ let d = r.statusText; try { d = (await r.json()).detail || d; } catch(e){} throw new Error(d); } return r; };
const qs = o => Object.entries(o).map(([k, v]) => encodeURIComponent(k) + "=" + encodeURIComponent(v)).join("&");
const RATES = [0.05, 0.1, 0.25, 0.5, 1, 2, 4, 8, 16];

// ---- state ---------------------------------------------------------------------------------------
const S = {key: null, rec: null, model: null, t: 0, t0: 0, t1: 1, view: [0, 1], playing: false, rate: 1, inT: null, outT: null,
           ee: "arm", lastFrame: 0, dirty: true, modules: []};

// ---- rows cache: per stream, 4 s chunks of the fields anything needs --------------------------------
const CH = 4;
class Rows {
  constructor(name){ this.name = name; this.fields = []; this.chunks = new Map(); this.pending = new Map(); this.gen = 0; }
  need(fs){ let add = false; for (const f of fs) if (!this.fields.includes(f)){ this.fields.push(f); add = true; }
    if (add){ this.gen++; this.chunks.clear(); this.pending.clear(); S.dirty = true; } }
  has(f){ return this.fields.includes(f); }
  async load(k){
    const key = k, g = this.gen;
    if (this.chunks.has(key) || this.pending.has(key) || !this.fields.length) return this.pending.get(key);
    const p = (async () => {
      const r = await api("api/rows?" + qs({key: S.key, stream: this.name, fields: this.fields.join(","), t0: k * CH, t1: (k + 1) * CH}));
      const buf = await r.arrayBuffer(), dv = new DataView(buf), hl = dv.getUint32(0, false);
      const h = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 4, hl)));
      const n = h.n, t = new Float64Array(buf, 4 + hl, n), v = new Float32Array(buf, 4 + hl + 8 * n, n * h.fields.length);
      const cols = {}; h.fields.forEach((f, j) => { const a = new Float32Array(n); for (let i = 0; i < n; i++) a[i] = v[i * h.fields.length + j]; cols[f] = a; });
      if (g === this.gen){ this.chunks.set(key, {t, cols, at: performance.now()}); this.trim(); S.dirty = true; }
      this.pending.delete(key);
    })().catch(e => { this.pending.delete(key); console.warn(this.name, e); });
    this.pending.set(key, p); return p;
  }
  trim(){ if (this.chunks.size <= 90) return; const ks = [...this.chunks.entries()].sort((a, b) => a[1].at - b[1].at);
    for (let i = 0; i < ks.length - 90; i++) this.chunks.delete(ks[i][0]); }
  ensure(a, b){ const ps = []; for (let k = Math.floor(a / CH); k <= Math.floor(b / CH); k++) ps.push(this.load(k)); return Promise.all(ps); }
  // the value at t: linear between neighbours (hold = previous sample) when they are close, NaN past 0.5 s
  at(f, t, hold){
    const c = this.chunks.get(Math.floor(t / CH)); if (!c || !c.cols[f]) return NaN;
    const T = c.t, n = T.length; if (!n) return NaN;
    let lo = 0, hi = n; while (lo < hi){ const m = (lo + hi) >> 1; if (T[m] <= t) lo = m + 1; else hi = m; }
    let i = lo - 1, a = c.cols[f];
    if (i < 0){ const pc = this.chunks.get(Math.floor(t / CH) - 1); if (pc && pc.t.length && t - pc.t[pc.t.length - 1] < 0.5) return pc.cols[f][pc.t.length - 1]; return NaN; }
    if (t - T[i] > 0.5) return NaN;
    if (hold || i + 1 >= n || T[i + 1] - T[i] > 0.3) return a[i];
    const u = (t - T[i]) / (T[i + 1] - T[i]); return a[i] + (a[i + 1] - a[i]) * u;
  }
  // [t[], v[]] of a field over [a, b] (for plots and trails)
  series(f, a, b, step){
    const ts = [], vs = [];
    for (let k = Math.floor(a / CH); k <= Math.floor(b / CH); k++){ const c = this.chunks.get(k); if (!c || !c.cols[f]) continue;
      const T = c.t, V = c.cols[f], s = step || 1;
      for (let i = 0; i < T.length; i += s) if (T[i] >= a && T[i] <= b){ ts.push(T[i]); vs.push(V[i]); } }
    return [ts, vs];
  }
}
const R = {};
const rows = name => R[name] || (R[name] = new Rows(name));
const have = name => S.rec && S.rec.streams[name];

// ---- camera frames ------------------------------------------------------------------------------
class Frames {
  constructor(name){ this.name = name; this.lists = new Map(); this.pend = new Map(); this.img = new Map(); this.ipend = new Map(); }
  async list(k){ if (this.lists.has(k) || this.pend.has(k)) return this.pend.get(k);
    const p = api("api/frames?" + qs({key: S.key, stream: this.name, t0: k * CH, t1: (k + 1) * CH - 1e-9})).then(r => r.json())
      .then(d => { this.lists.set(k, d); this.pend.delete(k); S.dirty = true; }).catch(() => this.pend.delete(k));
    this.pend.set(k, p); return p; }
  // the record index of the frame shown at t (the newest at or before t, within 1 s)
  index(t){ for (let k = Math.floor(t / CH), d = 0; d < 2; d++, k--){ const L = this.lists.get(k); if (!L){ if (d === 0) this.list(k); continue; }
      let lo = 0, hi = L.t.length; while (lo < hi){ const m = (lo + hi) >> 1; if (L.t[m] <= t) lo = m + 1; else hi = m; }
      if (lo > 0) return t - L.t[lo - 1] < 1.0 ? L.i[lo - 1] : -1; }
    return -1; }
  next(t, n){ const out = []; let k = Math.floor(t / CH);
    for (let d = 0; d < 3 && out.length < n; d++, k++){ const L = this.lists.get(k); if (!L){ this.list(k); continue; }
      for (let j = 0; j < L.t.length && out.length < n; j++) if (L.t[j] > t) out.push(L.i[j]); }
    return out; }
  async bitmap(i){ if (this.img.has(i)) return this.img.get(i); if (this.ipend.has(i)) return this.ipend.get(i);
    const p = api("api/frame?" + qs({key: S.key, stream: this.name, i})).then(r => r.blob()).then(b => createImageBitmap(b))
      .then(bm => { this.img.set(i, bm); this.ipend.delete(i); this.trim(); S.dirty = true; return bm; }).catch(() => { this.ipend.delete(i); return null; });
    this.ipend.set(i, p); return p; }
  trim(){ if (this.img.size <= 150) return; const ks = [...this.img.keys()]; for (let j = 0; j < ks.length - 150; j++){ const b = this.img.get(ks[j]); if (b && b.close) b.close(); this.img.delete(ks[j]); } }
  async ensure(t){ await this.list(Math.floor(t / CH)); await this.list(Math.floor(t / CH) - 1); const i = this.index(t); if (i >= 0) await this.bitmap(i); }
}
const F = {};

// ---- geometry ------------------------------------------------------------------------------------
const qToR = ([x, y, z, w]) => [[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)], [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)], [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]];
function rToQ(m){ const tr = m[0][0] + m[1][1] + m[2][2]; let q;
  if (tr > 0){ const s = Math.sqrt(tr + 1) * 2; q = [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, s / 4]; }
  else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]){ const s = Math.sqrt(1 + m[0][0] - m[1][1] - m[2][2]) * 2; q = [s / 4, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s]; }
  else if (m[1][1] > m[2][2]){ const s = Math.sqrt(1 + m[1][1] - m[0][0] - m[2][2]) * 2; q = [(m[0][1] + m[1][0]) / s, s / 4, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s]; }
  else { const s = Math.sqrt(1 + m[2][2] - m[0][0] - m[1][1]) * 2; q = [(m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, s / 4, (m[1][0] - m[0][1]) / s]; }
  return q; }
const mm4 = (A, B) => A.map((r, i) => [0, 1, 2, 3].map(j => r[0] * B[0][j] + r[1] * B[1][j] + r[2] * B[2][j] + r[3] * B[3][j]));
function axisRot(a, q){ const n = Math.hypot(...a), x = a[0] / n, y = a[1] / n, z = a[2] / n, c = Math.cos(q), s = Math.sin(q), t = 1 - c;
  return [[t*x*x + c, t*x*y - s*z, t*x*z + s*y, 0], [t*x*y + s*z, t*y*y + c, t*y*z - s*x, 0], [t*x*z - s*y, t*y*z + s*x, t*z*z + c, 0], [0, 0, 0, 1]]; }
// the arm's frames (joint1-6, end_link, tcp) in its base frame, as {pos, quat} for RU.viz.armSegments
function armFK(q){ const ch = S.model && S.model.chain; if (!ch || q.some(v => !fin(v))) return null;
  let T = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]; const out = [];
  ch.joints.forEach((j, k) => { T = mm4(mm4(T, j.T), axisRot(j.axis, q[k])); out.push(T); });
  out.push(mm4(T, ch.end)); out.push(mm4(T, ch.tcp));
  return out.map(M => ({pos: [M[0][3], M[1][3], M[2][3]], quat: rToQ(M), M})); }
const tfp = (T, p) => [0, 1, 2].map(i => T[i][0] * p[0] + T[i][1] * p[1] + T[i][2] * p[2] + T[i][3]);
const nq = q => { const n = Math.hypot(...q); return n > 0.5 ? q.map(v => v / n) : null; };

// an orthographic wireframe camera (as RU.viz.Scene): az / el / zoom, centred on a focus point
class View {
  constructor(canvas, opts){ this.cv = canvas; this.o = Object.assign({half: 1.4, az: -2.35, el: 0.55}, opts || {}); this.reset();
    let drag = null;
    canvas.addEventListener("pointerdown", e => { canvas.setPointerCapture(e.pointerId); drag = {x: e.clientX, y: e.clientY, az: this.az, el: this.el, pan: e.shiftKey, f: this.pan.slice()}; });
    canvas.addEventListener("pointermove", e => { if (!drag) return;
      if (drag.pan){ const s = this.scale(); const dx = (e.clientX - drag.x) / s, dy = (e.clientY - drag.y) / s;
        const r = [-Math.sin(this.az), Math.cos(this.az)]; this.pan = [drag.f[0] - dx * r[0], drag.f[1] - dx * r[1], drag.f[2] + dy]; }
      else { this.az = drag.az - (e.clientX - drag.x) * 0.01; this.el = Math.max(-0.2, Math.min(Math.PI / 2 - 1e-4, drag.el + (e.clientY - drag.y) * 0.01)); }
      S.dirty = true; });
    const end = () => drag = null; canvas.addEventListener("pointerup", end); canvas.addEventListener("pointercancel", end);
    canvas.addEventListener("wheel", e => { e.preventDefault(); this.zoom = Math.max(0.1, Math.min(20, this.zoom * Math.exp(-e.deltaY * 0.0015))); S.dirty = true; }, {passive: false});
    canvas.addEventListener("dblclick", () => { this.reset(); S.dirty = true; });
  }
  reset(){ this.az = this.o.az; this.el = this.o.el; this.zoom = 1; this.pan = [0, 0, 0]; }
  set(name){ if (name === "top"){ this.az = Math.PI; this.el = Math.PI / 2 - 1e-4; } else if (name === "side"){ this.az = -Math.PI / 2; this.el = 0.02; } else { this.az = this.o.az; this.el = this.o.el; } this.zoom = 1; this.pan = [0, 0, 0]; S.dirty = true; }
  scale(){ return Math.min(this.W, this.H) / (2 * this.o.half) * this.zoom; }
  // a projector for a W x H target, centred on `focus` (+ pan)
  proj(W, H, focus){ this.W = W; this.H = H; const {az, el} = this, e = [Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)];
    const r = [-Math.sin(az), Math.cos(az), 0], u = [e[1] * r[2] - e[2] * r[1], e[2] * r[0] - e[0] * r[2], e[0] * r[1] - e[1] * r[0]];
    const s = this.scale(), f = [focus[0] + this.pan[0], focus[1] + this.pan[1], focus[2] + this.pan[2]];
    return q => { const x = q[0] - f[0], y = q[1] - f[1], z = (q[2] || 0) - f[2]; return [W / 2 + (x * r[0] + y * r[1] + z * r[2]) * s, H / 2 - (x * u[0] + y * u[1] + z * u[2]) * s]; };
  }
}
function pen(g, P){ return {
  line(a, b, col, w, dash){ const A = P(a), B = P(b); g.strokeStyle = col; g.lineWidth = w || 1; g.setLineDash(dash || []); g.beginPath(); g.moveTo(A[0], A[1]); g.lineTo(B[0], B[1]); g.stroke(); },
  path(pts, col, w, dash){ g.strokeStyle = col; g.lineWidth = w || 1; g.setLineDash(dash || []); g.beginPath(); let on = false;
    for (const q of pts){ if (!q){ on = false; continue; } const a = P(q); if (on) g.lineTo(a[0], a[1]); else { g.moveTo(a[0], a[1]); on = true; } } g.stroke(); },
  dot(p, col, r){ const a = P(p); g.fillStyle = col; g.beginPath(); g.arc(a[0], a[1], r || 3, 0, 2 * Math.PI); g.fill(); },
  ring(p, col, r, w){ const a = P(p); g.strokeStyle = col; g.lineWidth = w || 1.2; g.setLineDash([]); g.beginPath(); g.arc(a[0], a[1], r || 5, 0, 2 * Math.PI); g.stroke(); },
  text(p, s, col, dx, dy){ const a = P(p); g.fillStyle = col; g.fillText(s, a[0] + (dx || 0), a[1] + (dy || 0)); },
}; }
const AX = ["#d9473f", "#2f9e57", "#3b6fd6"];
function triad(D, p, Rm, len, w, col, dash){ for (let k = 0; k < 3; k++){ const d = [Rm[0][k] * len, Rm[1][k] * len, Rm[2][k] * len]; D.line(p, [p[0] + d[0], p[1] + d[1], p[2] + d[2]], col || AX[k], w || 1.5, dash); } }
function grid(D, cx, cy, z, R0, step){ for (let v = Math.floor((cx - R0) / step) * step; v <= cx + R0 + 1e-9; v += step){ const b = Math.abs(v - Math.round(v)) < 1e-6; D.line([v, cy - R0, z], [v, cy + R0, z], C.rule, b ? 1 : 0.5); }
  for (let v = Math.floor((cy - R0) / step) * step; v <= cy + R0 + 1e-9; v += step){ const b = Math.abs(v - Math.round(v)) < 1e-6; D.line([cx - R0, v, z], [cx + R0, v, z], C.rule, b ? 1 : 0.5); } }
function fitCanvas(cv){ const dpr = window.devicePixelRatio || 1, W = cv.clientWidth, H = cv.clientHeight;
  if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)){ cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
  const g = cv.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); return [g, W, H]; }

// ---- what the 3D views read -----------------------------------------------------------------------
const POSE = ["x", "y", "z", "qx", "qy", "qz", "qw"];
function pose(st, prefix, t){ if (!have(st)) return null; const r = rows(st), v = POSE.map(k => r.at(prefix + k, t));
  if (v.some(x => !fin(x))) return null; const q = nq(v.slice(3)); return q ? {p: v.slice(0, 3), q, R: qToR(q)} : null; }
function needs3D(){
  const ee = S.ee;
  if (have("wbc")) rows("wbc").need(["base_x", "base_y", "base_yaw", "armed", "state", "limited_by", "base_mode", `${ee}_err_pos`, `${ee}_err_meas_pos`, `${ee}_gripper`]
    .concat(...["tgt", "cmd", "meas"].map(k => POSE.map(p => `${ee}_${k}_${p}`))).concat([1, 2, 3, 4, 5, 6].map(i => `${ee}_q_cmd${i}`)));
  if (have("arm")) rows("arm").need([1, 2, 3, 4, 5, 6].map(i => `j${i}_q`).concat(["g_gap"]));
  if (have("estimator")) rows("estimator").need(["x", "y", "theta"]);
  if (have("swerve")) rows("swerve").need([].concat(...S.modules.map(m => [`${m.name}_steer_angle`, `${m.name}_drive_vel`])));
  if (have("oplog.sent")) rows("oplog.sent").need(["armed", "base_mode", "hint_x", "hint_y", "hint_yaw"]);
  if (have("oplog.headset")) rows("oplog.headset").need(["head_valid"].concat(...["head", "left", "right"].map(h => POSE.map(p => `${h}_${p}`)),
    ...["left", "right"].map(h => ["valid", "grip", "trigger", "engaged", "engagement_id", "btn_primary", "btn_secondary"].map(k => `${h}_${k}`))));
}
const WBC_STATES = ["disabled", "enabling", "idle", "tracking", "holding", "fault"];
const LIMITS = ["", "reach", "manipulability", "fence", "joint limit", "base limit", "base band", "lead"];
let trail = null;      // the base path over the whole recording (decimated)

function basePose(t){
  if (have("wbc")){ const w = rows("wbc"), x = w.at("base_x", t), y = w.at("base_y", t), th = w.at("base_yaw", t); if (fin(x) && fin(y) && fin(th)) return [x, y, th, "wbc"]; }
  if (have("estimator")){ const e = rows("estimator"), x = e.at("x", t), y = e.at("y", t), th = e.at("theta", t); if (fin(x) && fin(y)) return [x, y, fin(th) ? th : 0, "estimator"]; }
  return null;
}
function anchorAt(t){ const A = (S.rec && S.rec.anchors) || []; let best = null;
  for (const a of A) if (a.accepted && a.t_start <= t && t <= a.t_end + 0.05) best = a; return best; }

// ---- the robot view -------------------------------------------------------------------------------
function drawRobot(g, W, H, t, view, follow, compact){
  g.clearRect(0, 0, W, H); g.fillStyle = C.panel; g.fillRect(0, 0, W, H);
  const bp = basePose(t), desc = S.model && S.model.description, rb = (desc && desc.robot) || {frame_size: 0.4318, modules: S.modules};
  const focus = bp && follow ? [bp[0], bp[1], 0.35] : (view.focus || [0, 0, 0.35]);
  if (bp && follow) view.focus = focus;
  const P = view.proj(W, H, focus), D = pen(g, P);
  g.lineJoin = "round"; g.font = "11px " + C.font;
  const s = view.scale(), R0 = Math.max(W, H) / s * 0.9, step = s > 140 ? 0.5 : s > 30 ? 1 : 5;
  grid(D, Math.round(focus[0]), Math.round(focus[1]), 0, R0, step);
  for (let k = 0; k < 3; k++){ const d = [0, 0, 0]; d[k] = 0.2; D.line([0, 0, 0], d, AX[k], 1.2); }
  // the base path: the whole recording faint, the last 5 s solid
  if (trail) D.path(trail.map(p => p ? [p[1], p[2], 0] : null), C.rule, 1.5);
  if (bp){ const w = have("wbc") ? rows("wbc") : rows("estimator"), fx = have("wbc") ? "base_x" : "x", fy = have("wbc") ? "base_y" : "y";
    const [ts, xs] = w.series(fx, t - 5, t, 2), [, ys] = w.series(fy, t - 5, t, 2); D.path(ts.map((_, i) => [xs[i], ys[i], 0]), C.muted, 1.5); }
  const ee = S.ee, W8 = have("wbc") ? rows("wbc") : null;
  // the base hint target (body-frozen, engaged): a dashed footprint
  if (have("oplog.sent")){ const o = rows("oplog.sent"); if (o.at("base_mode", t, true) === 3){ const hx = o.at("hint_x", t, true), hy = o.at("hint_y", t, true), hth = o.at("hint_yaw", t, true);
    if (fin(hx) && fin(hy) && fin(hth)){ const c = Math.cos(hth), sn = Math.sin(hth), h = rb.frame_size / 2, B = q => [hx + c * q[0] - sn * q[1], hy + sn * q[0] + c * q[1], 0.005];
      D.path([[-h, -h], [h, -h], [h, h], [-h, h], [-h, -h]].map(B), C.signal, 1, [5, 4]); D.line(B([h, 0]), B([h + 0.12, 0]), C.signal, 1.5, [5, 4]); } } }
  // the clutch anchors on the robot side while engaged: the frozen platform frame and the EE anchor
  const an = anchorAt(t);
  if (an && an.robot_frame){ const [ax, ay, ath] = an.robot_frame, c = Math.cos(ath), sn = Math.sin(ath);
    D.line([ax, ay, 0.01], [ax + 0.3 * c, ay + 0.3 * sn, 0.01], C.muted, 1.2, [3, 3]); D.line([ax, ay, 0.01], [ax - 0.3 * sn, ay + 0.3 * c, 0.01], C.muted, 1.2, [3, 3]);
    if (an.ee_anchor){ const p = an.ee_anchor.slice(0, 3); triad(D, p, qToR(an.ee_anchor.slice(3)), 0.05, 1, C.muted, [2, 2]); D.text(p, "anchor", C.muted, 6, 12); } }
  if (!bp){ g.fillStyle = C.muted; g.fillText("no base pose (wbc / estimator) at this time", 10, H - 10); return; }
  const [px, py, th] = bp, ct = Math.cos(th), st = Math.sin(th), Bw = q => [px + ct * q[0] - st * q[1], py + st * q[0] + ct * q[1], q[2] || 0];
  const armed = W8 ? W8.at("armed", t, true) > 0.5 : false, wstate = W8 ? W8.at("state", t, true) : NaN;
  const ink = C.ink, h = rb.frame_size / 2;
  const box = (c, sz, col, w, dash) => { const Q = []; for (const i of [-1, 1]) for (const j of [-1, 1]) for (const k of [-1, 1]) Q.push(Bw([c[0] + i * sz[0] / 2, c[1] + j * sz[1] / 2, c[2] + k * sz[2] / 2]));
    [[0,1],[2,3],[4,5],[6,7],[0,2],[1,3],[4,6],[5,7],[0,4],[1,5],[2,6],[3,7]].forEach(([a, b]) => D.line(Q[a], Q[b], col, w, dash)); };
  const [fz0, fz1] = rb.frame_z || [0.02, 0.12]; box([0, 0, (fz0 + fz1) / 2], [rb.frame_size, rb.frame_size, fz1 - fz0], ink, 1.4);
  const tr = rb.top_rail_z; if (tr){ box([0, 0, (tr[0] + tr[1]) / 2], [rb.frame_size, rb.frame_size, tr[1] - tr[0]], ink, 1.4);
    for (const i of [-1, 1]) for (const j of [-1, 1]) D.line(Bw([i * h, j * h, fz1]), Bw([i * h, j * h, tr[0]]), ink, 1, [2, 3]); }
  D.line(Bw([h, 0, fz1]), Bw([h + 0.07, 0, fz1]), ink, 2.5);
  const sw = have("swerve") ? rows("swerve") : null;
  for (const m of rb.modules || S.modules){ const a0 = sw ? sw.at(`${m.name}_steer_angle`, t) : NaN, a = fin(a0) ? a0 : 0, Rw = 0.0381, ca = Math.cos(a), sa = Math.sin(a), ring = [];
    for (let k = 0; k <= 16; k++){ const u = k / 16 * 2 * Math.PI; ring.push(Bw([m.x + ca * Rw * Math.cos(u), m.y + sa * Rw * Math.cos(u), Rw + Rw * Math.sin(u)])); }
    D.path(ring, fin(a0) ? ink : C.muted, 1.3); }
  // the arm: measured joints solid; the WBC's commanded joints as a faint ghost
  const Ta = desc && desc.frames_in_base && desc.frames_in_base.arm;
  if (Ta && RU.viz && RU.viz.armSegments){
    if (W8 && view.ghost !== false){ const qc = [1, 2, 3, 4, 5, 6].map(i => W8.at(`${ee}_q_cmd${i}`, t)); const fc = armFK(qc);
      if (fc) for (const [a, b] of RU.viz.armSegments(Ta, fc, NaN)) D.line(Bw(a), Bw(b), C.muted, 0.7); }
    const A = have("arm") ? rows("arm") : null, q = A ? [1, 2, 3, 4, 5, 6].map(i => A.at(`j${i}_q`, t)) : null, fr = q ? armFK(q) : null;
    for (const [a, b] of RU.viz.armSegments(Ta, fr, A ? A.at("g_gap", t) : NaN)) D.line(Bw(a), Bw(b), fr ? (armed ? C.live : ink) : C.muted, 1.25);
  }
  // EE references (world): target, command, measured, with their recent paths
  const roles = [["tgt", C.signal, "target"], ["cmd", C.p[1], "command"], ["meas", ink, "measured"]];
  if (W8) for (const [k, col] of roles){ const [ts, xs] = W8.series(`${ee}_${k}_x`, t - 2, t, 2), [, ys] = W8.series(`${ee}_${k}_y`, t - 2, t, 2), [, zs] = W8.series(`${ee}_${k}_z`, t - 2, t, 2);
    g.globalAlpha = 0.55; D.path(ts.map((_, i) => fin(xs[i]) ? [xs[i], ys[i], zs[i]] : null), col, 1); g.globalAlpha = 1; }
  for (const [k, col] of roles){ const P7 = pose("wbc", `${ee}_${k}_`, t); if (!P7) continue;
    triad(D, P7.p, P7.R, k === "tgt" ? 0.07 : 0.045, k === "tgt" ? 1.6 : 1.1, col); if (k === "tgt") D.ring(P7.p, col, 6, 1.4); else D.dot(P7.p, col, k === "meas" ? 2.5 : 2); }
  // labels
  g.setLineDash([]); g.font = "11px " + C.font; let y = H - 10;
  const lines = [];
  if (W8){ const lim = W8.at("limited_by", t, true), e1 = W8.at(`${ee}_err_pos`, t), e2 = W8.at(`${ee}_err_meas_pos`, t);
    lines.push((armed ? "ARMED" : "disarmed") + " · " + (WBC_STATES[wstate] || "-") + (fin(lim) && lim > 0 ? " · limited: " + (LIMITS[lim] || lim) : ""));
    lines.push(`cmd-tgt ${fin(e1) ? (e1 * 1000).toFixed(0) + " mm" : "-"} · meas-tgt ${fin(e2) ? (e2 * 1000).toFixed(0) + " mm" : "-"}`); }
  if (!compact) lines.push("base from " + bp[3]);
  for (const s of lines.reverse()){ g.fillStyle = s.startsWith("ARMED") ? C.live : C.muted; g.fillText(s, 10, y); y -= 14; }
  let lx = W - 10; g.textAlign = "right";
  for (const [, col, name] of roles.slice().reverse()){ g.fillStyle = col; g.fillText("— " + name, lx, H - 10); lx -= g.measureText("— " + name).width + 10; }
  g.textAlign = "left";
}

// ---- the operator view: headset, controllers, clutch anchors (headset LOCAL in FLU axes) ---------------
const DEV = {head: [[-0.09, -0.05, -0.04], [0.09, 0.05, 0.05]], hand: [[-0.025, -0.03, -0.06], [0.025, 0.02, 0.06]]};
function devBox(D, p, Rm, kind, col, w, dash){ const [lo, hi] = DEV[kind], Q = [];
  for (const i of [0, 1]) for (const j of [0, 1]) for (const k of [0, 1]){ const v = [i ? hi[0] : lo[0], j ? hi[1] : lo[1], k ? hi[2] : lo[2]]; Q.push([0, 1, 2].map(r => p[r] + Rm[r][0] * v[0] + Rm[r][1] * v[1] + Rm[r][2] * v[2])); }
  [[0,1],[2,3],[4,5],[6,7],[0,2],[1,3],[4,6],[5,7],[0,4],[1,5],[2,6],[3,7]].forEach(([a, b]) => D.line(Q[a], Q[b], col, w, dash));
  const f = [0, 1, 2].map(r => p[r] - Rm[r][2] * (kind === "head" ? 0.16 : 0.1)); D.line(p, f, col, w, dash); }   // OpenXR -z = forward
function drawOperator(g, W, H, t, view){
  g.clearRect(0, 0, W, H); g.fillStyle = C.panel; g.fillRect(0, 0, W, H); g.font = "11px " + C.font;
  if (!have("oplog.headset")){ g.fillStyle = C.muted; g.fillText("no operator stream in this recording", 10, H - 10); return; }
  const o = rows("oplog.headset"), head = pose("oplog.headset", "head_", t);
  // follows the operator: centred a little ahead of and below the head (they walk with the robot)
  const hy = head ? Math.atan2(-head.R[1][2], -head.R[0][2]) : 0;           // OpenXR -z = forward
  const focus = head ? [head.p[0] + 0.25 * Math.cos(hy), head.p[1] + 0.25 * Math.sin(hy), head.p[2] - 0.45] : (view.focus || [0.3, 0, -0.5]);
  view.focus = focus;
  const P = view.proj(W, H, focus), D = pen(g, P), floor = (view.floor = view.floor || (head ? head.p[2] - 1.55 : -1.55));
  grid(D, Math.round(focus[0] * 2) / 2, Math.round(focus[1] * 2) / 2, floor, 2.5, 0.5);
  triad(D, [0, 0, floor], [[1,0,0],[0,1,0],[0,0,1]], 0.2, 1.2);
  const [ts, xs] = o.series("head_x", t - 4, t, 2), [, ys] = o.series("head_y", t - 4, t, 2);
  D.path(ts.map((_, i) => fin(xs[i]) ? [xs[i], ys[i], floor] : null), C.muted, 1.2);
  if (head){ devBox(D, head.p, head.R, "head", C.ink, 1.4); D.line(head.p, [head.p[0], head.p[1], floor], C.rule, 1, [2, 3]); }
  // the engagement in force: the operator frame and the hand anchor captured at its start, and the delta
  const an = anchorAt(t);
  if (an && an.operator){ const [ox, oy, oz] = an.operator.origin, yaw = an.operator.yaw, c = Math.cos(yaw), sn = Math.sin(yaw);
    triad(D, [ox, oy, oz], [[c, -sn, 0], [sn, c, 0], [0, 0, 1]], 0.25, 1.2, null, [4, 3]); D.text([ox, oy, oz], "operator frame", C.muted, 6, -6);
    if (an.hand_world){ const p = an.hand_world.slice(0, 3); devBox(D, p, qToR(an.hand_world.slice(3)), "hand", C.muted, 1, [3, 2]); D.text(p, "anchor", C.muted, 8, 12);
      const cur = pose("oplog.headset", an.hand + "_", t); if (cur) D.line(p, cur.p, C.signal, 1.4, [5, 3]); } }
  const lines = [];
  for (const hand of ["left", "right"]){ const P7 = pose("oplog.headset", hand + "_", t); const eng = o.at(hand + "_engaged", t, true) > 0.5;
    const grip = o.at(hand + "_grip", t), trig = o.at(hand + "_trigger", t);
    if (P7){ devBox(D, P7.p, P7.R, "hand", eng ? C.signal : C.ink, eng ? 2 : 1.2); D.text(P7.p, hand[0].toUpperCase(), C.muted, -14, -6); }
    const b1 = o.at(hand + "_btn_primary", t, true) > 0.5, b2 = o.at(hand + "_btn_secondary", t, true) > 0.5, names = hand === "left" ? ["X", "Y"] : ["A", "B"];
    lines.push(`${hand[0].toUpperCase()}  grip ${fin(grip) ? grip.toFixed(2) : "-"}  trigger ${fin(trig) ? trig.toFixed(2) : "-"}${eng ? "  engaged #" + o.at(hand + "_engagement_id", t, true) : ""}${b1 ? "  " + names[0] : ""}${b2 ? "  " + names[1] : ""}`); }
  if (have("oplog.sent")){ const s = rows("oplog.sent"), ar = s.at("armed", t, true), bm = s.at("base_mode", t, true);
    lines.push(`sent: ${ar > 0.5 ? "armed" : "disarmed"} · base ${["free", "locked", "commanded", "hint"][bm] || "-"}`); }
  let y = H - 10; g.setLineDash([]);
  for (const s of lines.reverse()){ g.fillStyle = s.includes("engaged") ? C.signal : C.muted; g.fillText(s, 10, y); y -= 14; }
}

// ---- cameras ------------------------------------------------------------------------------------------
function drawCamera(g, W, H, t, name, label){
  g.fillStyle = C.panel; g.fillRect(0, 0, W, H);
  const fr = F[name]; if (!fr) return;
  const i = fr.index(t), bm = i >= 0 ? fr.img.get(i) : null;
  if (i >= 0 && !bm){ fr.bitmap(i); }
  const show = bm || fr.lastShown;
  if (show){ const s = Math.min(W / show.width, H / show.height), w = show.width * s, h = show.height * s; g.drawImage(show, (W - w) / 2, (H - h) / 2, w, h); if (bm) fr.lastShown = bm; }
  else { g.fillStyle = C.muted; g.font = "11px " + C.font; g.fillText(i < 0 ? "no frame at this time" : "loading", 10, H - 10); }
  if (label){ g.font = "11px " + C.font; const tw = g.measureText(label).width; g.fillStyle = "rgba(0,0,0,.45)"; g.fillRect(6, 4, tw + 10, 16); g.fillStyle = "#fff"; g.fillText(label, 11, 16); }
}

// ---- plots -----------------------------------------------------------------------------------------------
const plots = [];      // {el, cv, series: [{stream, field, unit}], win}
let plotWin = 10;
function defaultPlots(){
  const ee = S.ee, out = [];
  if (have("wbc")) out.push([["wbc", `${ee}_err_pos`], ["wbc", `${ee}_err_meas_pos`]]);
  if (have("wbc")) out.push([["wbc", `${ee}_tgt_z`], ["wbc", `${ee}_cmd_z`], ["wbc", `${ee}_meas_z`]]);
  if (have("wbc")) out.push([["wbc", "base_vx"], ["wbc", "base_vy"], ["wbc", "base_w"]]);
  if (have("arm")) out.push([1, 2, 3, 4, 5, 6].map(i => ["arm", `j${i}_q`]));
  if (have("arm")) out.push([["arm", "g_gap"], ["arm", "g_gap_cmd"]]);
  if (have("oplog.headset")) out.push([["oplog.headset", "right_grip"], ["oplog.headset", "right_trigger"], ["oplog.headset", "right_engaged"]]);
  return out;
}
function unitOf(st, f){ const s = S.rec.streams[st]; const u = s && s.fields ? (s.fields.find(x => x[0] === f) || [])[1] : ""; return u || ""; }
function addPlot(series){
  series = series.filter(([st, f]) => have(st) && (S.rec.streams[st].fields || []).some(x => x[0] === f));
  if (!series.length) return;
  const el = document.createElement("div"); el.className = "panel plot"; el.dataset.panel = "plot";
  el.innerHTML = '<canvas></canvas><div class="legend"></div><div class="tools"><button data-a="up" title="move up">&uarr;</button><button data-a="x" title="remove">&times;</button></div>';
  const p = {el, cv: el.querySelector("canvas"), series: series.map(([stream, field]) => ({stream, field, unit: unitOf(stream, field)}))};
  el.querySelector(".legend").innerHTML = p.series.map((s, k) => `<span style="color:${C.p[k % 5]}">— ${s.field}${s.unit ? " [" + s.unit + "]" : ""} <span class="muted">${s.stream}</span></span>`).join("");
  el.querySelector('[data-a="x"]').onclick = () => { plots.splice(plots.indexOf(p), 1); el.remove(); savePlots(); };
  el.querySelector('[data-a="up"]').onclick = () => { const i = plots.indexOf(p); if (i > 0){ plots.splice(i, 1); plots.splice(i - 1, 0, p); el.parentNode.insertBefore(el, plots[i].el); savePlots(); } };
  for (const s of p.series) rows(s.stream).need([s.field]);
  p.cv.addEventListener("wheel", e => { e.preventDefault(); plotWin = Math.max(1, Math.min(600, plotWin * Math.exp(e.deltaY * 0.0015))); S.dirty = true; }, {passive: false});
  p.cv.addEventListener("pointerdown", e => { const r = p.cv.getBoundingClientRect(); const x0 = e.clientX; const t0 = S.t; p.cv.setPointerCapture(e.pointerId);
    const mv = ev => seek(t0 - (ev.clientX - x0) / r.width * plotWin); p.cv.addEventListener("pointermove", mv);
    p.cv.addEventListener("pointerup", () => p.cv.removeEventListener("pointermove", mv), {once: true}); });
  $("#plots").appendChild(el); plots.push(p); savePlots(); S.dirty = true;
}
function savePlots(){ try { localStorage.setItem("browse.plots", JSON.stringify(plots.map(p => p.series.map(s => [s.stream, s.field])))); } catch(e){} }
function drawPlot(g, W, H, t, p, win, opts){
  opts = opts || {}; g.fillStyle = C.panel; g.fillRect(0, 0, W, H);
  const a = t - win / 2, b = t + win / 2, top = opts.legend ? 18 : 20, bot = 14, X = v => (v - a) / win * W;
  const data = p.series.map(s => rows(s.stream).series(s.field, a, b, Math.max(1, Math.floor(win * 100 / W / 2))));
  let lo = Infinity, hi = -Infinity; for (const [, v] of data) for (const x of v) if (fin(x)){ lo = Math.min(lo, x); hi = Math.max(hi, x); }
  if (!fin(lo)){ lo = 0; hi = 1; } if (hi - lo < 1e-9){ hi += 0.5; lo -= 0.5; } const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;
  const Y = v => top + (1 - (v - lo) / (hi - lo)) * (H - top - bot);
  // armed (from the WBC) as a faint band along the bottom; events as ticks
  if (have("wbc") && rows("wbc").has("armed")){ const [ts, vs] = rows("wbc").series("armed", a, b, 5); g.fillStyle = C.sigbg;
    for (let i = 0; i + 1 < ts.length; i++) if (vs[i] > 0.5) g.fillRect(X(ts[i]), H - bot + 3, Math.max(1, X(ts[i + 1]) - X(ts[i]) + 0.5), 4); }
  g.strokeStyle = C.rule; g.lineWidth = 1; g.setLineDash([]);
  for (const e of (S.rec.events || [])) if (e.t >= a && e.t <= b && /episode|estop|fault/.test(e.kind)){ g.beginPath(); g.moveTo(X(e.t), top); g.lineTo(X(e.t), H - bot); g.stroke(); }
  const zero = Y(0); if (zero > top && zero < H - bot){ g.strokeStyle = C.rule; g.beginPath(); g.moveTo(0, zero); g.lineTo(W, zero); g.stroke(); }
  data.forEach(([ts, vs], k) => { g.strokeStyle = C.p[k % 5]; g.lineWidth = 1.3; g.beginPath(); let on = false;
    for (let i = 0; i < ts.length; i++){ if (!fin(vs[i]) || (i && ts[i] - ts[i - 1] > 0.5)){ on = false; if (!fin(vs[i])) continue; }
      const x = X(ts[i]), y = Y(vs[i]); if (on) g.lineTo(x, y); else { g.moveTo(x, y); on = true; } } g.stroke(); });
  g.strokeStyle = C.ink; g.lineWidth = 1; g.beginPath(); g.moveTo(W / 2, 0); g.lineTo(W / 2, H); g.stroke();
  g.fillStyle = C.muted; g.font = "10px " + C.mono; g.textAlign = "right"; g.fillText(fmtNum(hi), W - 4, top + 8); g.fillText(fmtNum(lo), W - 4, H - bot - 2);
  g.textAlign = "left"; g.fillText(`${win.toFixed(win < 10 ? 1 : 0)} s`, 4, H - 2);
  let y = top + 10;                                         // the values at the playhead, beside it
  p.series.forEach((s, k) => { const v = rows(s.stream).at(s.field, t); g.fillStyle = C.p[k % 5]; g.fillText(fin(v) ? fmtNum(v) : "-", W / 2 + 5, y); y += 12; });
  if (opts.legend){ let x = 6; g.font = "11px " + C.font; p.series.forEach((s, k) => { g.fillStyle = C.p[k % 5]; const txt = "— " + s.field; g.fillText(txt, x, 13); x += g.measureText(txt).width + 10; }); }
}
const fmtNum = v => { const a = Math.abs(v); return a >= 1000 ? v.toFixed(0) : a >= 10 ? v.toFixed(1) : a >= 1 ? v.toFixed(2) : v.toFixed(3); };

// ---- timeline ------------------------------------------------------------------------------------------
const EVT = {episode_start: "start", episode_stop: "kept", episode_abort: "aborted", estop: "e-stop", fault: "fault", sim_reset: "reset", armed: "armed", disarmed: "disarmed"};
let armedTrack = null;
function drawTimeline(){
  const cv = $("#timeline canvas"), [g, W, H] = fitCanvas(cv);
  g.fillStyle = C.panel; g.fillRect(0, 0, W, H);
  if (!S.rec) return;
  const [a, b] = [S.t0, S.t1], X = v => (v - a) / Math.max(1e-9, b - a) * W;
  // the shown range (an episode or in / out) as a bracket; episodes as spans
  for (const e of S.rec.episodes || []){ if (!fin(e.t_start) || !fin(e.t_stop)) continue; const x0 = X(e.t_start), x1 = X(e.t_stop);
    g.strokeStyle = e.outcome === "kept" ? C.live : C.caution; g.lineWidth = 1.5; g.beginPath(); g.moveTo(x0, 30); g.lineTo(x0, 22); g.lineTo(x1, 22); g.lineTo(x1, 30); g.stroke();
    g.fillStyle = C.muted; g.font = "10px " + C.font; g.fillText(e.id + (e.outcome !== "kept" ? " · " + (e.outcome || "") : ""), x0 + 3, 18); }
  if (armedTrack){ g.fillStyle = C.sigbg; const [ts, vs] = armedTrack; for (let i = 0; i + 1 < ts.length; i++) if (vs[i] > 0.5) g.fillRect(X(ts[i]), 36, Math.max(1, X(ts[i + 1]) - X(ts[i]) + 0.5), 6); }
  g.strokeStyle = C.rule; g.lineWidth = 1;
  for (const e of S.rec.events || []){ if (!fin(e.t) || e.t < a || e.t > b) continue; const x = X(e.t), big = /episode|estop|fault/.test(e.kind);
    g.strokeStyle = /estop|fault|abort/.test(e.kind) ? C.stop : big ? C.ink : C.rule; g.beginPath(); g.moveTo(x, big ? 32 : 44); g.lineTo(x, 50); g.stroke(); }
  if (fin(S.inT) || fin(S.outT)){ const x0 = X(fin(S.inT) ? S.inT : a), x1 = X(fin(S.outT) ? S.outT : b); g.strokeStyle = C.signal; g.lineWidth = 2; g.beginPath(); g.moveTo(x0, 54); g.lineTo(x1, 54); g.stroke(); }
  // ticks
  g.fillStyle = C.muted; g.font = "10px " + C.mono; const span = b - a, st = [1, 2, 5, 10, 20, 30, 60, 120, 300, 600].find(s => span / s < 12) || 1200;
  for (let v = Math.ceil((a - S.zero) / st) * st; v + S.zero <= b; v += st){ const x = X(v + S.zero); g.fillRect(x, H - 6, 1, 6); g.fillText(fmtT(v), x + 2, H - 1); }
  const x = X(S.t); g.strokeStyle = C.signal; g.lineWidth = 2; g.beginPath(); g.moveTo(x, 0); g.lineTo(x, H); g.stroke();
}
const fmtT = v => { const s = v < 0 ? "-" : ""; v = Math.abs(v); const m = Math.floor(v / 60); return s + (m ? m + ":" + (v - 60 * m).toFixed(0).padStart(2, "0") : v.toFixed(v < 10 ? 1 : 0)); };
(function(){ const el = $("#timeline"); let drag = false;
  const go = e => { const r = el.getBoundingClientRect(); seek(S.t0 + (e.clientX - r.left) / r.width * (S.t1 - S.t0)); };
  el.addEventListener("pointerdown", e => { drag = true; el.setPointerCapture(e.pointerId); go(e); });
  el.addEventListener("pointermove", e => { if (drag) go(e); const r = el.getBoundingClientRect(), t = S.t0 + (e.clientX - r.left) / r.width * (S.t1 - S.t0);
    const near = (S.rec && S.rec.events || []).filter(ev => Math.abs(ev.t - t) < (S.t1 - S.t0) / r.width * 4);
    const tip = $("#tip"); if (near.length){ tip.style.display = "block"; tip.style.left = e.clientX + 12 + "px"; tip.style.top = e.clientY + 12 + "px";
      tip.textContent = near.map(ev => `${fmtT(ev.t - S.zero)} ${ev.kind}${ev.detail && Object.keys(ev.detail).length ? " " + JSON.stringify(ev.detail) : ""}`).join("  |  "); } else tip.style.display = "none"; });
  el.addEventListener("pointerleave", () => $("#tip").style.display = "none");
  el.addEventListener("pointerup", () => drag = false); })();

// ---- transport -----------------------------------------------------------------------------------------
function seek(t){ S.t = Math.max(S.t0, Math.min(S.t1, t)); S.dirty = true; }
function setRate(r){ S.rate = r; $("#rate").value = String(r); }
RATES.forEach(r => { const o = document.createElement("option"); o.value = String(r); o.textContent = r + "×"; $("#rate").appendChild(o); });
RATES.forEach(r => { const o = document.createElement("option"); o.value = String(r); o.textContent = r + "×"; $("#xrate").appendChild(o); });
$("#rate").value = "1"; $("#xrate").value = "1";
$("#rate").onchange = e => setRate(+e.target.value);
$("#play").onclick = () => { if (!S.playing && S.t >= S.t1 - 1e-6) S.t = S.t0; S.playing = !S.playing; S.lastFrame = performance.now(); $("#play").textContent = S.playing ? "pause" : "play"; };
const step = dir => { S.playing = false; $("#play").textContent = "play"; seek(S.t + dir * frameStep()); };
function frameStep(){ // one camera frame if a camera is shown, else 10 ms
  const c = Object.values(S.rec.streams).find(s => s.kind === "frames" && s.fps); return c ? 1 / c.fps : 0.01; }
$("#back").onclick = () => step(-1); $("#fwd").onclick = () => step(1); $("#start").onclick = () => seek(S.t0);
$("#inbtn").onclick = () => { S.inT = S.t; showInOut(); }; $("#outbtn").onclick = () => { S.outT = S.t; showInOut(); };
$("#clearbtn").onclick = () => { S.inT = S.outT = null; showInOut(); };
function showInOut(){ $("#inout").textContent = (fin(S.inT) || fin(S.outT)) ? `in ${fin(S.inT) ? fmtT(S.inT - S.zero) : "-"} · out ${fin(S.outT) ? fmtT(S.outT - S.zero) : "-"}` : ""; S.dirty = true; }
document.addEventListener("keydown", e => {
  if (["INPUT", "SELECT", "TEXTAREA"].includes(e.target.tagName)) return;
  if (e.key === " "){ e.preventDefault(); $("#play").click(); }
  else if (e.key === "ArrowLeft"){ e.preventDefault(); e.shiftKey ? seek(S.t - 1) : step(-1); }
  else if (e.key === "ArrowRight"){ e.preventDefault(); e.shiftKey ? seek(S.t + 1) : step(1); }
  else if (e.key === "[" || e.key === "]"){ const i = RATES.indexOf(S.rate), j = Math.max(0, Math.min(RATES.length - 1, (i < 0 ? 4 : i) + (e.key === "]" ? 1 : -1))); setRate(RATES[j]); }
  else if (e.key === "Home") seek(S.t0); else if (e.key === "End") seek(S.t1);
  else if (e.key === "i") $("#inbtn").click(); else if (e.key === "o") $("#outbtn").click();
});

// ---- views ------------------------------------------------------------------------------------------------
const robotView = new View($("#robot canvas"), {half: 0.95}), opView = new View($("#operator canvas"), {half: 0.6, az: -2.6, el: 0.45});
let follow = true;
document.querySelectorAll("#robot .tools button").forEach(b => b.onclick = () => { const v = b.dataset.v;
  if (v === "follow"){ follow = !follow; b.classList.toggle("on", follow); if (!follow) robotView.focus = robotView.focus || [0, 0, 0.35]; S.dirty = true; } else robotView.set(v); });
document.querySelectorAll("#operator .tools button").forEach(b => b.onclick = () => { opView.set(b.dataset.v); opView.focus = null; });
const vis = {robot: true, operator: true, plots: true};
function buildViews(){
  const el = $("#views"); el.innerHTML = "";
  const opt = (k, label) => { const l = document.createElement("label"); l.innerHTML = `<input type="checkbox" ${vis[k] !== false ? "checked" : ""}> ${label}`;
    l.querySelector("input").onchange = e => { vis[k] = e.target.checked; applyVis(); }; el.appendChild(l); };
  opt("robot", "robot 3D"); opt("operator", "operator 3D");
  for (const n of camNames()) opt("cam:" + n, n);
  opt("plots", "plots"); opt("ghost", "commanded arm (ghost)");
}
function applyVis(){ $("#robot").classList.toggle("hidden", vis.robot === false); $("#operator").classList.toggle("hidden", vis.operator === false || !have("oplog.headset"));
  document.querySelectorAll(".cam").forEach(c => c.classList.toggle("hidden", vis["cam:" + c.dataset.name] === false));
  $("#plots").classList.toggle("hidden", vis.plots === false); robotView.ghost = vis.ghost !== false; S.dirty = true; }
$("#viewbtn").onclick = () => { const v = $("#views"); v.style.display = v.style.display === "block" ? "none" : "block"; $("#exp").style.display = "none"; };
$("#sidebtn").onclick = () => $("#side").classList.toggle("hidden");
const camNames = () => S.rec ? Object.entries(S.rec.streams).filter(([, s]) => s.kind === "frames" && s.jpeg).map(([n]) => n) : [];

// ---- the render loop ------------------------------------------------------------------------------------------
let lastPrefetch = 0;
function tick(now){
  requestAnimationFrame(tick);
  if (!S.rec) return;
  if (S.playing){ const dt = Math.min(0.1, (now - S.lastFrame) / 1000); S.lastFrame = now; let t = S.t + dt * S.rate;
    if (t >= S.t1){ if ($("#loop").checked) t = S.t0; else { t = S.t1; S.playing = false; $("#play").textContent = "play"; } }
    S.t = t; S.dirty = true; }
  if (now - lastPrefetch > 120){ lastPrefetch = now; prefetch(); }
  if (!S.dirty) return; S.dirty = false;
  $("#time").textContent = `${fmtT(S.t - S.zero).padStart(6)} s  / ${fmtT(S.t1 - S.zero)}   t=${S.t.toFixed(3)}`;
  if (vis.robot !== false){ const [g, W, H] = fitCanvas($("#robot canvas")); drawRobot(g, W, H, S.t, robotView, follow); }
  if (vis.operator !== false && have("oplog.headset")){ const [g, W, H] = fitCanvas($("#operator canvas")); drawOperator(g, W, H, S.t, opView); }
  document.querySelectorAll(".cam").forEach(el => { if (el.classList.contains("hidden")) return; const [g, W, H] = fitCanvas(el.querySelector("canvas")); drawCamera(g, W, H, S.t, el.dataset.name); });
  if (vis.plots !== false) for (const p of plots){ const [g, W, H] = fitCanvas(p.cv); drawPlot(g, W, H, S.t, p, plotWin); }
  drawTimeline();
}
function prefetch(){
  const ahead = S.playing ? Math.max(2, 3 * S.rate) : 2, t = S.t;
  for (const r of Object.values(R)) if (r.fields.length) r.ensure(t - Math.max(plotWin / 2, 2) - 1, t + Math.max(plotWin / 2, ahead));
  for (const fr of Object.values(F)){ fr.list(Math.floor(t / CH)); fr.list(Math.floor(t / CH) + 1); fr.list(Math.floor(t / CH) - 1);
    const n = S.playing ? Math.min(30, Math.ceil(12 * Math.max(1, S.rate))) : 3; for (const i of fr.next(t, n)) fr.bitmap(i);
    if (!S.playing){ const i = fr.index(t); if (i >= 0) fr.bitmap(i); } }
}
requestAnimationFrame(tick);
new ResizeObserver(() => S.dirty = true).observe(document.body);

// ---- library and opening ---------------------------------------------------------------------------------------
let LIB = [];
async function loadLibrary(){
  const d = await (await api("api/library")).json(); LIB = d.items; const side = $("#side"); side.innerHTML = "";
  const add = (cls, html, fn, key) => { const el = document.createElement("div"); el.className = "it " + cls; el.innerHTML = html; el.onclick = fn; el.dataset.key = key || ""; side.appendChild(el); return el; };
  for (const it of LIB){
    if (it.kind === "session"){ add("ses", `${it.name} <span class="tag">${it.task || ""}</span>`, () => openRec(it.key, null), it.key);
      for (const e of it.episodes || []){ const d = fin(e.t_stop) && fin(e.t_start) ? (e.t_stop - e.t_start).toFixed(1) + " s" : "";
        add("ep", `${e.id} <span class="tag ${e.outcome === "kept" ? "" : "ab"}">${e.outcome || ""}${e.reason ? " · " + e.reason : ""} ${d}</span>`, () => openRec(it.key, e), it.key + "#" + e.id); } }
    else add("ep", `${it.name} <span class="tag ${it.outcome === "kept" ? "" : "ab"}">cut · ${it.outcome || ""}</span>`, () => openRec(it.key, null), it.key);
  }
  if (!LIB.length) side.innerHTML = '<div class="it muted">no sessions or episodes here</div>';
  const h = decodeURIComponent(location.hash.slice(1)); const [k, ep] = h ? h.split("#") : [null, null];
  const item = LIB.find(i => i.key === k) || LIB[0];
  if (item){ const e = ep && item.episodes ? item.episodes.find(x => x.id === ep) : (item.kind === "session" && item.episodes && item.episodes.length && !h ? item.episodes[0] : null); openRec(item.key, e); }
}
async function openRec(key, ep){
  S.playing = false; $("#play").textContent = "play";
  if (key !== S.key){
    for (const k in R) delete R[k]; for (const k in F){ for (const b of F[k].img.values()) if (b && b.close) b.close(); delete F[k]; }
    S.key = key; S.rec = null;
    const [rec, model] = await Promise.all([api("api/recording?" + qs({key})).then(r => r.json()), api("api/model?" + qs({key})).then(r => r.json())]);
    S.rec = rec; S.model = model;
    const wbc = rec.streams.wbc; const m = wbc && (wbc.fields || []).map(f => f[0].match(/^(.+)_tgt_x$/)).find(Boolean); S.ee = m ? m[1] : "arm";
    S.modules = (model.description && model.description.robot && model.description.robot.modules) || [{name: "FL", x: 0.1656, y: 0.1656}, {name: "FR", x: 0.1656, y: -0.1656}, {name: "RR", x: -0.1656, y: -0.1656}, {name: "RL", x: -0.1656, y: 0.1656}];
    $("#src").textContent = `${rec.path} · model: ${model.description_source}${model.chain ? "" : " · " + model.chain_source}`;
    const cams = $("#cams"); cams.innerHTML = "";
    for (const n of camNames()){ F[n] = new Frames(n); const el = document.createElement("div"); el.className = "panel cam"; el.dataset.name = n; el.dataset.panel = "camera";
      el.innerHTML = `<canvas></canvas><div class="lab">${n} · ${(rec.streams[n].fps || 0).toFixed(1)} fps${rec.streams[n].dropped ? " · " + rec.streams[n].dropped + " dropped" : ""}</div>`; cams.appendChild(el); }
    needs3D();
    $("#plots").innerHTML = ""; plots.length = 0;
    let saved = null; try { saved = JSON.parse(localStorage.getItem("browse.plots") || "null"); } catch(e){}
    for (const s of (saved && saved.length ? saved : defaultPlots())) addPlot(s);
    if (!plots.length) for (const s of defaultPlots()) addPlot(s);
    fillAddPlot(); buildViews(); applyVis(); fillExportSources();
    // the base path and the armed track over the whole recording (decimated)
    trail = null; armedTrack = null;
    const span = rec.t1 - rec.t0, stride = Math.max(1, Math.round(span * 100 / 3000));
    const bs = have("wbc") ? ["wbc", "base_x", "base_y"] : have("estimator") ? ["estimator", "x", "y"] : null;
    if (bs) rowsOnce(bs[0], [bs[1], bs[2]], rec.t0, rec.t1, stride).then(([t, v]) => { trail = []; for (let i = 0; i < t.length; i++){ const x = v[i * 2], y = v[i * 2 + 1]; trail.push(fin(x) && fin(y) ? [t[i], x, y] : null); } S.dirty = true; });
    if (have("wbc")) rowsOnce("wbc", ["armed"], rec.t0, rec.t1, stride).then(([t, v]) => { armedTrack = [t, v]; S.dirty = true; });
    $("#title").textContent = rec.kind === "episode" ? `Episode ${rec.meta.id || ""}` : `Session ${rec.path.split("/").pop()}`;
  }
  const rec = S.rec;
  if (ep && fin(ep.t_start)){ const pad = 1.0; S.t0 = Math.max(rec.t0, ep.t_start - pad); S.t1 = Math.min(rec.t1, (fin(ep.t_stop) ? ep.t_stop : rec.t1) + pad); S.zero = ep.t_start;
    $("#title").textContent = `Episode ${ep.id}`; $("#src").title = ep.task || ""; }
  else { S.t0 = rec.t0; S.t1 = rec.t1; S.zero = rec.kind === "episode" && fin(rec.meta.t_start) ? rec.meta.t_start : rec.t0; }
  S.ep = ep; S.t = S.t0; S.inT = S.outT = null; showInOut();
  document.querySelectorAll("aside .it").forEach(el => el.classList.toggle("sel", el.dataset.key === key + (ep ? "#" + ep.id : "") || (!ep && el.dataset.key === key)));
  history.replaceState(null, "", "#" + encodeURIComponent(key + (ep ? "#" + ep.id : "")));
  robotView.focus = null; opView.focus = null; opView.floor = null; S.dirty = true;
}
async function rowsOnce(stream, fields, t0, t1, stride){
  const r = await api("api/rows?" + qs({key: S.key, stream, fields: fields.join(","), t0, t1, stride})), buf = await r.arrayBuffer(), dv = new DataView(buf), hl = dv.getUint32(0, false);
  const h = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 4, hl))); return [new Float64Array(buf, 4 + hl, h.n), new Float32Array(buf, 4 + hl + 8 * h.n, h.n * fields.length)]; }
function fillAddPlot(){
  const sel = $("#addstream"); sel.innerHTML = "";
  for (const [n, s] of Object.entries(S.rec.streams)) if (s.fields){ const o = document.createElement("option"); o.value = n; o.textContent = n; sel.appendChild(o); }
  const list = $("#fieldlist"), inp = $("#addfield");
  const show = () => { const s = S.rec.streams[sel.value]; if (!s){ list.style.display = "none"; return; } const q = inp.value.split(",").pop().trim().toLowerCase();
    const fs = s.fields.filter(f => f[0].toLowerCase().includes(q)).slice(0, 300); list.innerHTML = "";
    for (const [f, u] of fs){ const d = document.createElement("div"); d.textContent = f + (u ? "  [" + u + "]" : ""); d.onclick = () => { const parts = inp.value.split(","); parts[parts.length - 1] = f; inp.value = parts.join(",") + ","; inp.focus(); show(); }; list.appendChild(d); }
    list.style.display = fs.length ? "block" : "none"; };
  inp.onfocus = show; inp.oninput = show; sel.onchange = show;
  $("#addbtn").onclick = () => { const fs = inp.value.split(",").map(x => x.trim()).filter(Boolean); if (fs.length){ addPlot(fs.map(f => [sel.value, f])); inp.value = ""; list.style.display = "none"; } };
}

// ---- export ---------------------------------------------------------------------------------------------------
let XI = null, xjob = null, xbpp = null;
$("#expbtn").onclick = async () => { const e = $("#exp"); e.style.display = e.style.display === "block" ? "none" : "block"; $("#views").style.display = "none";
  if (!XI){ XI = await (await api("api/export/info")).json();
    $("#expffmpeg").innerHTML = XI.available ? `ffmpeg: ${XI.ffmpeg.split("/").pop()} · ${XI.codecs.join(", ")}${XI.hw.length ? " · " + XI.hw.join(", ") : ""} · to ${XI.out_dir}` : '<span class="err">no ffmpeg found: export is off (docs/browse.md)</span>';
    for (const [k, p] of Object.entries(XI.presets)){ const o = document.createElement("option"); o.value = k; o.textContent = p.label; $("#xpreset").appendChild(o); }
    $("#xpreset").value = "balanced"; $("#xcrf").value = XI.presets.balanced.crf; $("#xgo").disabled = $("#xprobe").disabled = !XI.available; }
  xEstimate(); };
$("#xclose").onclick = () => $("#exp").style.display = "none";
function fillExportSources(){ const s = $("#xsource"); s.innerHTML = '<option value="stage">replay (what is shown)</option>';
  for (const n of camNames()){ const o = document.createElement("option"); o.value = "cam:" + n; o.textContent = n + " only (no re-render, fast)"; s.appendChild(o); } }
$("#xpreset").onchange = () => { $("#xcrf").value = XI.presets[$("#xpreset").value].crf; xbpp = null; xEstimate(); };
$("#xmode").onchange = () => { const sz = $("#xmode").value === "size"; $("#xqrow").classList.toggle("hidden", sz); $("#xsrow").classList.toggle("hidden", !sz); $("#xprow").classList.toggle("hidden", !sz); xEstimate(); };
["#xrange", "#xrate", "#xres", "#xfps", "#xcrf", "#xmb", "#xtwo", "#xsource"].forEach(s => $(s).addEventListener("change", () => { if (s !== "#xmb" && s !== "#xtwo" && s !== "#xrange" && s !== "#xrate") xbpp = null; xEstimate(); }));
function xRange(){ const m = $("#xrange").value; if (m === "inout" && (fin(S.inT) || fin(S.outT))) return [fin(S.inT) ? S.inT : S.t0, fin(S.outT) ? S.outT : S.t1];
  if (m === "all") return [S.rec.t0, S.rec.t1]; return [S.t0, S.t1]; }
function xSettings(){ const [w, h] = $("#xres").value.split("x").map(Number), s = {width: w, height: h, fps: +$("#xfps").value, preset: $("#xpreset").value, crf: +$("#xcrf").value};
  if ($("#xmode").value === "size"){ s.target_mb = +$("#xmb").value; s.two_pass = $("#xtwo").checked; } return s; }
async function xEstimate(){
  if (!XI) return; const [a, b] = xRange(), rate = +$("#xrate").value, dur = (b - a) / rate, s = xSettings();
  const e = await (await api("api/export/estimate", {method: "POST", headers: {"content-type": "application/json"}, body: JSON.stringify({settings: s, duration: dur, bpp: s.target_mb ? null : xbpp})})).json();
  const mb = e.bytes / 1e6;
  $("#xest").innerHTML = `${fmtT(b - a)} s of data at ${rate}× = ${dur.toFixed(1)} s of video, ${Math.round(dur * s.fps)} frames<br>`
    + (s.target_mb ? `target ${mb.toFixed(1)} MB = ${(e.bitrate / 1e6).toFixed(2)} Mbit/s (${e.bpp.toFixed(3)} bit/px)` : `≈ ${mb.toFixed(1)} MB (${(e.bitrate / 1e6).toFixed(2)} Mbit/s; ${e.how === "measured" ? "measured" : "model, press measure"})`)
    + (e.warning ? `<br><span class="warn">${e.warning}</span>` : "");
}
// render the replay at time t into an export canvas: robot 3D left, cameras and the operator right, plots below
const xcv = document.createElement("canvas");
const xRobot = new View(document.createElement("canvas"), {half: 0.95}), xOp = new View(document.createElement("canvas"), {half: 0.6, az: -2.6, el: 0.45});
function composeLayout(W, H){
  const cams = camNames().filter(n => vis["cam:" + n] !== false), op = vis.operator !== false && have("oplog.headset"), pl = vis.plots !== false ? plots.slice(0, 3) : [];
  const head = 24, plotH = pl.length ? Math.round(H * 0.24) : 0, mainH = H - head - plotH, rightW = (cams.length || op) ? Math.round(W * 0.36) : 0, L = [];
  L.push(["robot", 0, head, W - rightW, mainH]);
  const n = cams.length + (op ? 1 : 0); let y = head;
  for (const c of cams){ const h = Math.round(mainH / n); L.push(["cam:" + c, W - rightW, y, rightW, h]); y += h; }
  if (op) L.push(["operator", W - rightW, y, rightW, head + mainH - y]);
  pl.forEach((p, i) => L.push(["plot", 0, head + mainH + i * Math.round(plotH / pl.length), W, Math.round(plotH / pl.length), p]));
  return L;
}
function renderExportFrame(t, W, H, rate){
  xcv.width = W; xcv.height = H; const g = xcv.getContext("2d"); g.fillStyle = C.panel; g.fillRect(0, 0, W, H);
  const sc = Math.max(1, H / 720);
  for (const [k, x, y, w, h, p] of composeLayout(W, H)){
    g.save(); g.translate(x, y); g.beginPath(); g.rect(0, 0, w, h); g.clip(); g.scale(sc, sc);
    const ww = w / sc, hh = h / sc;
    if (k === "robot"){ Object.assign(xRobot, {az: robotView.az, el: robotView.el, zoom: robotView.zoom, pan: robotView.pan, ghost: robotView.ghost}); drawRobot(g, ww, hh, t, xRobot, follow, true); }
    else if (k === "operator"){ Object.assign(xOp, {az: opView.az, el: opView.el, zoom: opView.zoom, pan: opView.pan, focus: opView.focus, floor: opView.floor}); drawOperator(g, ww, hh, t, xOp); }
    else if (k.startsWith("cam:")) drawCamera(g, ww, hh, t, k.slice(4), k.slice(4));
    else if (k === "plot") drawPlot(g, ww, hh, t, p, plotWin, {legend: true});
    g.restore(); g.strokeStyle = C.rule; g.lineWidth = 1; g.strokeRect(x + 0.5, y + 0.5, w - 1, h - 1);
  }
  g.fillStyle = C.ink; g.font = `${Math.round(13 * sc)}px ${C.font}`;
  const ep = S.ep ? S.ep.id + (S.ep.task ? " · " + S.ep.task : "") : S.rec.path.split("/").pop();
  g.fillText(ep, 8 * sc, 17 * sc); g.textAlign = "right"; g.font = `${Math.round(13 * sc)}px ${C.mono}`;
  g.fillText(`${fmtT(t - S.zero)} s · ${rate}×`, W - 8 * sc, 17 * sc); g.textAlign = "left";
}
async function ensureAt(t){
  const ps = [];
  for (const r of Object.values(R)) if (r.fields.length) ps.push(r.ensure(t - plotWin / 2 - 2.1, t + plotWin / 2 + 0.1));
  for (const n of camNames()) if (vis["cam:" + n] !== false) ps.push(F[n].ensure(t));
  await Promise.all(ps);
  for (const fr of Object.values(F)){ const i = fr.index(t); if (i >= 0 && !fr.img.has(i)) await fr.bitmap(i); }
}
// the JPEG of the export canvas. toDataURL (synchronous) rather than toBlob: browsers throttle toBlob's callback
// to about once a second in a background tab, which would stall a long export the moment the tab is hidden
function blob(){ const s = atob(xcv.toDataURL("image/jpeg", 0.92).split(",")[1]), b = new Uint8Array(s.length);
  for (let i = 0; i < s.length; i++) b[i] = s.charCodeAt(i); return b; }
async function runExport(probe){
  const [a, b] = xRange(), rate = +$("#xrate").value, s = xSettings(), src = $("#xsource").value, fps = s.fps;
  let times = []; const n = Math.floor((b - a) * fps / rate) + 1; for (let k = 0; k < n; k++) times.push(a + k * rate / fps);
  if (probe){ const seg = Math.max(1, Math.round(fps * 1.0)), out = []; for (const c of [0.15, 0.5, 0.85]){ const k0 = Math.floor(c * Math.max(0, n - seg)); out.push(...times.slice(k0, k0 + seg)); } times = out; }
  const name = (S.ep ? S.ep.id : S.rec.path.split("/").pop()) + (src === "stage" ? "" : "-" + src.slice(4)) + `-${rate}x`;
  const title = S.ep ? `${S.ep.id} ${S.ep.task || ""}` : name;
  $("#xprog").classList.remove("hidden"); $("#xprog").value = 0; $("#xstatus").textContent = probe ? "measuring" : "rendering"; $("#xgo").disabled = $("#xprobe").disabled = true;
  try {
    let job;
    if (src.startsWith("cam:")){
      job = await (await api("api/export/camera", {method: "POST", headers: {"content-type": "application/json"}, body: JSON.stringify({key: S.key, stream: src.slice(4), t0: a, t1: b, rate, settings: s, name, title, probe})})).json();
    } else {
      job = await (await api("api/export", {method: "POST", headers: {"content-type": "application/json"}, body: JSON.stringify({settings: s, frames: times.length, name, title, probe})})).json();
      xjob = job.id; $("#xcancel").classList.remove("hidden"); const t0 = performance.now(); let inflight = [];
      for (let k = 0; k < times.length; k++){
        if (xjob === null) throw new Error("cancelled");
        await ensureAt(times[k]); renderExportFrame(times[k], s.width, s.height, rate);
        const bl = blob(); inflight.push(api(`api/export/${job.id}/frame`, {method: "POST", body: bl}));
        if (inflight.length >= 3) await inflight.shift();
        if (k % 5 === 0){ $("#xprog").value = 0.5 * k / times.length; const el = (performance.now() - t0) / 1000;
          $("#xstatus").textContent = `${probe ? "measuring" : "rendering"} ${k}/${times.length} · ${(k / Math.max(el, 1e-3)).toFixed(1)} frames/s`; }
      }
      await Promise.all(inflight);
      await api(`api/export/${job.id}/finish`, {method: "POST"});
    }
    let st; do { await new Promise(r => setTimeout(r, 300)); st = await (await api(`api/export/${job.id}`)).json(); $("#xprog").value = st.progress; if (st.phase) $("#xstatus").textContent = st.phase; } while (st.state === "encoding" || st.state === "receiving");
    if (st.state !== "done") throw new Error(st.error || st.state);
    const r = st.result;
    if (probe){ xbpp = r.bpp; await xEstimate(); $("#xstatus").textContent = `measured ${r.bpp.toFixed(4)} bit/px on ${st.frames} frames (encode ${(st.frames / r.seconds).toFixed(0)} frames/s)`; }
    else $("#xstatus").innerHTML = `done: <a href="api/export/${job.id}/file" download>${st.out.split("/").pop()}</a> · ${(r.bytes / 1e6).toFixed(2)} MB${r.target_bytes ? ` (target ${(r.target_bytes / 1e6).toFixed(1)} MB, ${(100 * r.target_error).toFixed(1)} %)` : ""} · ${r.passes} pass${r.passes > 1 ? "es" : ""} · encode ${r.seconds.toFixed(1)} s<br><span class="muted">${st.out}</span>`;
  } catch(e){ $("#xstatus").innerHTML = `<span class="err">${e.message}</span>`; }
  finally { xjob = null; $("#xcancel").classList.add("hidden"); $("#xgo").disabled = $("#xprobe").disabled = !XI.available; S.dirty = true; }
}
$("#xgo").onclick = () => runExport(false); $("#xprobe").onclick = () => runExport(true);
$("#xcancel").onclick = async () => { const j = xjob; xjob = null; if (j) await api(`api/export/${j}/cancel`, {method: "POST"}); };
window.addEventListener("hashchange", () => {});
loadLibrary().catch(e => { $("#src").textContent = "error: " + e.message; });
</script></body></html>
"""
