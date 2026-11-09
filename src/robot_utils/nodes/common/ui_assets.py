"""The shared browser toolkit, served by every node at ``common/ui.js`` and ``common/ui.css``.

Kept as Python strings so it ships with the package without package-data configuration.
Pages load it with RELATIVE URLs (``<script src="common/ui.js">``) so they work both on the
node's own port and behind a reverse proxy under ``/nodes/<name>/``. API (``window.RU``):

  RU.url(path)                        relative URL ("state" -> "./state")
  RU.api(path, {method, body, headers})   JSON call; throws Error(detail) on 4xx/5xx
  new RU.TimeSync(); await ts.sync(16)    NTP-style against "time": ts.offset, ts.uncertainty,
                                          ts.toServer(performance.now()/1000), ts.serverNow()
  new RU.RecordReader().feed(Uint8Array)  -> [{header, payload: Uint8Array}] (node framing)
  new RU.RowsDecoder().decode(h, p)       -> {fields, n, col(name) -> Float64Array} or null
  RU.stream(path, onRecord, {signal})     reads a binary record stream; reconnects
  RU.rows(path, onRows, {signal})         schema + rows records -> onRows(decoded, header)
  RU.sse(path, onJson)                    EventSource of JSON states; returns the EventSource
  RU.leaseWidget(el, {client, onChange})  acquire / release / auto-renew; .token, .headers()
  RU.estopButton(el, {path, headers})     POST "estop"; shows the latched state from "state"
  RU.stateBand(el).update(state, {live})  the page's state band: e-stop > chocked > covered > live
  RU.token("--signal", fallback)          a theme token's value (for canvases)
  new RU.Plot(canvas, {seconds, series:[{name, color}], ymin, ymax}); plot.push(t, values)
"""

UI_CSS = r"""
/* -- tokens (themes override these; docs/frontend.md "Themes") ----------------------------------
   Semantic colours: stop = e-stop / latched / chocked; caution = covers, warnings, lease held by
   someone else; live = motors enabled / healthy; signal = interactive and primary data. */
@font-face{font-family:"Plex";src:url("fonts/ibm-plex-sans-latin-wght-normal.woff2") format("woff2");font-weight:100 700;font-display:swap}
:root{color-scheme:light dark;
  --ground:#EEF1F4;--panel:#FFFFFF;--ink:#15202B;--muted:#5B6672;--rule:#D3DAE1;
  --signal:#1F5FBF;--stop:#C41E17;--caution:#A86400;--live:#157A45;
  --stop-bg:#FBE4E2;--caution-bg:#FCEFD9;--live-bg:#E2F3EA;--signal-bg:#E3ECFA;
  --stop-fill:#C41E17;--on-stop:#FFFFFF;   /* e-stop buttons and the e-stop / chocked bands: solid fill + its text (>= 4.5:1) */
  --caution-fill:#F2B33D;--on-caution:#1B1200;   /* the covered band */
  --plot-1:#1F5FBF;--plot-2:#C2410C;--plot-3:#0E7490;--plot-4:#9D174D;--plot-5:#4D7C0F;
  --font:"Plex",system-ui,-apple-system,"Segoe UI",Roboto,Cantarell,sans-serif;
  --mono:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;
  --radius:6px;
  /* older names, kept as aliases */
  --bg:var(--ground);--fg:var(--ink);--card:var(--panel);--line:var(--rule);--acc:var(--signal);
  --ok:var(--live);--err:var(--stop);--warn:var(--caution)}
@media (prefers-color-scheme:dark){:root{
  --ground:#161A1F;--panel:#1E242B;--ink:#E3E8ED;--muted:#98A3AE;--rule:#2E363F;
  --signal:#78AEFF;--stop:#FF5B50;--caution:#F2B33D;--live:#4CC88A;
  --stop-bg:#3A1714;--caution-bg:#33270F;--live-bg:#12301F;--signal-bg:#16263D;
  --stop-fill:#D42A20;--on-stop:#FFFFFF;
  --plot-1:#78AEFF;--plot-2:#FB923C;--plot-3:#22D3EE;--plot-4:#F472B6;--plot-5:#A3E635}}
body{margin:0;font:14px/1.45 var(--font);background:var(--ground);color:var(--ink)}
.ru-wrap{max-width:1100px;margin:0 auto;padding:16px}
.ru-card{background:var(--panel);border:1px solid var(--rule);border-radius:var(--radius);padding:12px 14px;margin:0 0 12px}
.ru-row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
/* -- layout contract (docs/frontend.md "Layouts"): hooks themes may target to rearrange a page ------
   body[data-page="<kind>"]   which page this is (swerve, pose_servo, camera, landing, ...)
   .ru-head                   the header row: title, status, e-stop. Must stay visible (tests/test_visual_safety.py)
   .ru-body                   the main grid of panels; .ru-side its side column, where a page has one
   [data-panel="<name>"]      one panel, unique within its page (map, jog, events, ...)
   [data-role="..."]          what it holds: view | control | telemetry | config | log
   #band                      the state band. Must stay visible. */
.ru-row.ru-head,.ru-row.ru-spread{justify-content:space-between}
.ru-muted{color:var(--muted)} .ru-ok{color:var(--live)} .ru-err{color:var(--stop)} .ru-warn{color:var(--caution)}
.ru-num{font-variant-numeric:tabular-nums}
code,pre,.ru-mono{font-family:var(--mono)}
:focus-visible{outline:2px solid var(--signal);outline-offset:2px}
/* buttons: neutral by default; .primary (main action), .hazard (stops / irreversible), .live (starts motion) */
button.ru,.ru-btn{font:inherit;min-height:32px;padding:5px 12px;border-radius:var(--radius);border:1px solid var(--rule);background:var(--panel);color:var(--ink);cursor:pointer}
button.ru:hover{border-color:var(--signal)} button.ru:disabled{opacity:.45;cursor:not-allowed}
button.ru.primary{background:var(--signal);border-color:var(--signal);color:#fff}
button.ru.hazard{border-color:var(--stop);color:var(--stop)} button.ru.live{border-color:var(--live);color:var(--live)}
button.ru-estop{min-height:44px;min-width:110px;background:var(--stop-fill);color:var(--on-stop);border:0;font-weight:700;font-size:16px;padding:8px 18px;letter-spacing:.04em}
button.ru-estop.latched{background:#7f1d1d;box-shadow:inset 0 0 0 3px #fff3}
.ru-pill{display:inline-block;padding:1px 8px;border-radius:999px;border:1px solid var(--rule);font-size:12px}
.ru-pill.ru-ok{border-color:var(--live);background:var(--live-bg)} .ru-pill.ru-err{border-color:var(--stop);background:var(--stop-bg)}
.ru-pill.ru-warn{border-color:var(--caution);background:var(--caution-bg)}
canvas.ru-plot{width:100%;height:160px;display:block}
table.ru-table{border-collapse:collapse;width:100%} .ru-table td,.ru-table th{padding:3px 6px;border-bottom:1px solid var(--rule);text-align:right;font-variant-numeric:tabular-nums}
.ru-table th:first-child,.ru-table td:first-child,.ru-table .t{text-align:left}
.ru-ev-warn{color:var(--caution)} .ru-ev-error{color:var(--stop)}
/* the state band: one per page, pinned to the top (RU.stateBand). Priority: e-stop > chocked > covered > live > nominal */
.ru-band{position:sticky;top:0;z-index:20;margin:0 0 10px;font-size:14px;line-height:1.3}
.ru-band .in{max-width:1100px;margin:0 auto;padding:8px 16px;display:flex;gap:10px;align-items:baseline;flex-wrap:wrap}
.ru-band b{font-weight:700;letter-spacing:.02em} .ru-band small{opacity:.85}
.ru-band.nominal{height:3px;background:var(--rule)} .ru-band.nominal .in{display:none}
.ru-band.live{background:var(--live-bg);color:var(--live);box-shadow:inset 0 -3px 0 var(--live)}
.ru-band.estop{background:var(--stop-fill);color:var(--on-stop)}
.ru-band.chocked{color:var(--on-stop);background:var(--stop-fill) repeating-linear-gradient(135deg,var(--stop-fill) 0 14px,#7f1d1d 14px 28px)}
.ru-band.covered{color:var(--on-caution);background:var(--caution-fill) repeating-linear-gradient(135deg,var(--caution-fill) 0 14px,#d99a2b 14px 28px)}
"""

UI_JS = r"""
(function(){
"use strict";
const RU = {};
RU.url = p => (p.startsWith("./") || p.startsWith("../") || /^[a-z]+:/i.test(p)) ? p : "./" + p.replace(/^\/+/, "");

RU.api = async function(path, opts){
  opts = opts || {};
  const init = {method: opts.method || (opts.body !== undefined ? "POST" : "GET"), headers: Object.assign({}, opts.headers || {}), cache: "no-store"};
  if (opts.body !== undefined){ init.headers["Content-Type"] = "application/json"; init.body = JSON.stringify(opts.body); }
  const r = await fetch(RU.url(path), init);
  const text = await r.text();
  let data = null; try { data = text ? JSON.parse(text) : null; } catch(e){ data = text; }
  if (!r.ok){ const d = data && data.detail !== undefined ? data.detail : text; throw new Error(r.status + ": " + (typeof d === "string" ? d : JSON.stringify(d))); }
  return data;
};

// -- time: client clock = performance.now()/1000 (monotonic); server clock = its time.monotonic()
RU.TimeSync = class {
  constructor(){ this.offset = null; this.uncertainty = null; this.rtt = null; }
  async sync(n){
    n = n || 16; await fetch(RU.url("time"), {cache: "no-store"});
    const s = [];
    for (let i = 0; i < n; i++){
      const t0 = performance.now()/1000;
      const j = await (await fetch(RU.url("time"), {cache: "no-store"})).json();
      const t1 = performance.now()/1000;
      s.push({t0, t1, ts: j.mono, rtt: t1 - t0});
    }
    s.sort((a, b) => a.rtt - b.rtt);
    const best = s.slice(0, 8);
    let lo = -Infinity, hi = Infinity;
    for (const x of best){ lo = Math.max(lo, x.ts - x.t1); hi = Math.min(hi, x.ts - x.t0); }
    if (lo <= hi){ this.offset = (lo + hi)/2; this.uncertainty = (hi - lo)/2; }
    else { const b = best[0]; this.offset = b.ts - (b.t0 + b.t1)/2; this.uncertainty = b.rtt/2; }
    this.rtt = s[0].rtt;
    return this;
  }
  toServer(tClient){ return tClient + this.offset; }
  serverNow(){ return performance.now()/1000 + this.offset; }
};

// -- binary records: [u32 BE header length][JSON header][header.len bytes payload]
RU.RecordReader = class {
  constructor(){ this.buf = new Uint8Array(0); this.dec = new TextDecoder(); }
  feed(chunk){
    const nb = new Uint8Array(this.buf.length + chunk.length); nb.set(this.buf); nb.set(chunk, this.buf.length); this.buf = nb;
    const out = [];
    for (;;){
      if (this.buf.length < 4) break;
      const hl = new DataView(this.buf.buffer, this.buf.byteOffset, 4).getUint32(0);
      if (this.buf.length < 4 + hl) break;
      const header = JSON.parse(this.dec.decode(this.buf.subarray(4, 4 + hl)));
      const len = header.len || 0;
      if (this.buf.length < 4 + hl + len) break;
      out.push({header, payload: this.buf.slice(4 + hl, 4 + hl + len)});
      this.buf = this.buf.slice(4 + hl + len);
    }
    return out;
  }
};

RU.RowsDecoder = class {
  constructor(){ this.schemas = {}; }
  decode(h, p){
    if (h.type === "schema"){ this.schemas[h.schema] = h; return null; }
    let fields = h.fields;
    if (!fields && h.type === "rows"){ const s = this.schemas[h.schema]; if (!s) return null; fields = s.fields; }
    if (!fields) return null;
    const a = new Float64Array(p.buffer.slice(p.byteOffset, p.byteOffset + p.byteLength));   // little-endian hosts
    const m = fields.length, n = a.length / m, idx = {};
    fields.forEach((f, i) => idx[f] = i);
    return {fields, n, raw: a, idx,
            col(name){ const j = idx[name]; const c = new Float64Array(n); for (let i = 0; i < n; i++) c[i] = a[i*m + j]; return c; },
            last(name){ return a[(n - 1)*m + idx[name]]; }};
  }
};

RU.stream = async function(path, onRecord, opts){
  opts = opts || {};
  for (;;){
    if (opts.signal && opts.signal.aborted) return;
    try {
      const r = await fetch(RU.url(path), {cache: "no-store", signal: opts.signal});
      if (!r.ok) throw new Error(r.status);
      const rd = r.body.getReader(), reader = new RU.RecordReader();
      for (;;){
        const {value, done} = await rd.read(); if (done) break;
        for (const rec of reader.feed(value)) onRecord(rec.header, rec.payload);
      }
    } catch (e) { if (opts.signal && opts.signal.aborted) return; }
    await new Promise(res => setTimeout(res, 1000));
  }
};

RU.rows = function(path, onRows, opts){
  const dec = new RU.RowsDecoder();
  return RU.stream(path, (h, p) => { const d = dec.decode(h, p); if (d) onRows(d, h); }, opts);
};

RU.sse = function(path, onJson){
  const es = new EventSource(RU.url(path));
  es.onmessage = ev => { try { onJson(JSON.parse(ev.data)); } catch(e){} };
  return es;
};

// -- lease widget: token kept in sessionStorage per node path; renews every 300 ms while held
RU.leaseWidget = function(el, opts){
  opts = opts || {};
  const key = "ru-lease:" + location.pathname;
  const w = {token: sessionStorage.getItem(key) || null, holder: null,
             headers(){ return this.token ? {"X-Lease-Token": this.token} : {}; }};
  el.innerHTML = '<span class="ru-pill" data-k="who">lease: ?</span> <input data-k="name" size="12" placeholder="your name"> ' +
                 '<button class="ru" data-k="acq">Take control</button> <button class="ru" data-k="rel">Release</button>';
  const $ = k => el.querySelector('[data-k="' + k + '"]');
  $("name").value = opts.client || localStorage.getItem("ru-client") || "";
  const set = (tok) => { w.token = tok; if (tok) sessionStorage.setItem(key, tok); else sessionStorage.removeItem(key); render(); if (opts.onChange) opts.onChange(w); };
  function render(){
    $("who").textContent = w.token ? "you hold the lease" : (w.holder ? "held by " + w.holder : "lease free");
    $("who").className = "ru-pill " + (w.token ? "ru-ok" : (w.holder ? "ru-warn" : ""));
    $("acq").disabled = !!w.token; $("rel").disabled = !w.token;
  }
  $("acq").onclick = async () => {
    const name = $("name").value || "browser"; localStorage.setItem("ru-client", name);
    try { set((await RU.api("lease", {body: {client: name}})).token); } catch(e){ alert(e.message); }
  };
  $("rel").onclick = async () => { try { await RU.api("lease", {method: "DELETE", headers: w.headers()}); } catch(e){} set(null); };
  setInterval(async () => {
    if (!w.token) return;
    try { await RU.api("lease/renew", {method: "POST", headers: w.headers()}); } catch(e){ set(null); }
  }, 300);
  w.update = st => { const l = st && st.lease; w.holder = l && l.held ? l.holder : null; if (l && !l.held && w.token) set(null); render(); };
  render();
  return w;
};

RU.estopButton = function(el, opts){
  opts = opts || {};
  el.innerHTML = '<button class="ru ru-estop">E-STOP</button> <span class="ru-muted" data-k="s"></span>';
  const b = el.querySelector("button"), s = el.querySelector('[data-k="s"]');
  b.onclick = async () => { try { await RU.api(opts.path || "estop", {method: "POST", headers: opts.headers || {}, body: opts.body}); } catch(e){ s.textContent = e.message; } };
  return {update(st){ const e = st && st.estop; const on = !!(e && e.latched);
    b.classList.toggle("latched", on); s.textContent = on ? "E-stop latched" + (e.source || e.reason ? " (" + [e.source, e.reason].filter(Boolean).join(": ") + ")" : "") : ""; s.className = on ? "ru-err" : "ru-muted"; }};
};

// -- tokens: CSS custom properties of the current theme (for canvases)
RU.token = function(name, fallback){
  const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  return v || fallback || "";
};

// -- the state band (one per page; CSS .ru-band). update(st, {live}) with a node's /state:
//    st.estop.latched, st.locks (chocks / covers that apply: common/locks.py), and live = motors on.
RU.stateBand = function(el){
  el.className = "ru-band nominal";
  el.setAttribute("role", "status");
  el.innerHTML = '<div class="in"><b data-k="t"></b><span data-k="r"></span><small data-k="m"></small></div>';
  const q = k => el.querySelector('[data-k="' + k + '"]');
  const ago = s => s < 3600 ? Math.round(s / 60) + " min ago" : (s / 3600).toFixed(1) + " h ago";
  return {update(st, extra){
    st = st || {}; extra = extra || {};
    const locks = st.locks || [], e = st.estop || {};
    const ch = locks.filter(l => l.kind === "chock"), cv = locks.filter(l => l.kind === "cover");
    let cls = "nominal", t = "", r = "", m = "";
    const tag = l => [l.reason ? "“" + l.reason + "”" : "no reason given", "by " + l.by, ago(l.age_s || 0)];
    if (e.latched){ cls = "estop"; t = "E-STOP latched"; r = [e.source, e.reason].filter(Boolean).join(": "); m = "the holder clears it on the page"; }
    else if (ch.length){ cls = "chocked"; t = "CHOCKED: can't be enabled"; const x = tag(ch[0]); r = x[0]; m = x.slice(1).join(", ") + (ch.length > 1 ? ` (+${ch.length - 1} more)` : "") + " · removed by an admin over SSH"; }
    else if (cv.length){ cls = "covered"; t = "Camera covered: no images captured"; const x = tag(cv[0]); r = x[0]; m = x.slice(1).join(", "); }
    else if (extra.live){ cls = "live"; t = "Motors live"; r = extra.liveText || ""; }
    el.className = "ru-band " + cls; q("t").textContent = t; q("r").textContent = r; q("m").textContent = m;
    return cls;
  }};
};

// -- tiny rolling line plot (time in s on x). Series colours: a token name ("--plot-2"), a CSS colour,
//    or by default the theme's --plot-1..5; series 2+ are dashed so they don't rely on colour alone.
RU.Plot = class {
  constructor(canvas, o){ this.c = canvas; this.o = Object.assign({seconds: 5, series: [], ymin: null, ymax: null}, o || {}); this.data = this.o.series.map(() => []); this.t = []; }
  push(t, values){ this.t.push(t); values.forEach((v, i) => this.data[i].push(v)); const t0 = t - this.o.seconds;
    while (this.t.length && this.t[0] < t0){ this.t.shift(); this.data.forEach(d => d.shift()); } }
  color(s, i){ const c = s.color; if (c && c.startsWith("--")) return RU.token(c, "#1F5FBF");
    return c || RU.token("--plot-" + (i % 5 + 1), "#1F5FBF"); }
  draw(){
    const c = this.c, dpr = window.devicePixelRatio || 1, W = c.clientWidth, H = c.clientHeight;
    if (c.width !== W*dpr){ c.width = W*dpr; c.height = H*dpr; }
    const g = c.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
    if (this.t.length < 2) return;
    const top = 16, ff = RU.token("--font", "system-ui");            // a legend strip above the data
    let lo = this.o.ymin, hi = this.o.ymax;
    if (lo === null || hi === null){ let a = Infinity, b = -Infinity; this.data.forEach(d => d.forEach(v => { if (isFinite(v)){ a = Math.min(a, v); b = Math.max(b, v); } }));
      if (!isFinite(a)){ a = -1; b = 1; } if (a === b){ a -= 1; b += 1; } lo = lo === null ? a : lo; hi = hi === null ? b : hi; }
    const t1 = this.t[this.t.length - 1], t0 = t1 - this.o.seconds, X = t => (t - t0)/(t1 - t0)*W, Y = v => H - (v - lo)/(hi - lo)*(H - top - 6) - 4;
    g.strokeStyle = RU.token("--rule", "#ccc"); g.lineWidth = 1; g.setLineDash([]); g.beginPath(); g.moveTo(0, Y(0)); g.lineTo(W, Y(0)); g.stroke();
    const dashes = [[], [6, 3], [2, 3], [8, 3, 2, 3], [4, 4]];
    this.o.series.forEach((s, i) => { g.strokeStyle = this.color(s, i); g.lineWidth = 1.6; g.setLineDash(dashes[i % dashes.length]); g.beginPath(); let pen = false;
      this.data[i].forEach((v, k) => { if (!isFinite(v)){ pen = false; return; } const x = X(this.t[k]), y = Y(v); if (pen) g.lineTo(x, y); else { g.moveTo(x, y); pen = true; } }); g.stroke(); });
    g.setLineDash([]);
    g.fillStyle = RU.token("--muted", "#888"); g.font = "11px " + ff; g.textBaseline = "top";
    g.fillText(hi.toFixed(2), 2, top); g.textBaseline = "alphabetic"; g.fillText(lo.toFixed(2), 2, H - 2);
    let x = W; g.textBaseline = "top";                                // legend, right-aligned in the strip
    for (let i = this.o.series.length - 1; i >= 0; i--){ const s = this.o.series[i], w = g.measureText(s.name).width;
      x -= w; g.fillStyle = this.color(s, i); g.fillText(s.name, x, 1);
      g.strokeStyle = g.fillStyle; g.lineWidth = 2; g.setLineDash(dashes[i % dashes.length]); g.beginPath(); g.moveTo(x - 18, 7); g.lineTo(x - 4, 7); g.stroke(); g.setLineDash([]);
      x -= 26; }
    g.textBaseline = "alphabetic";
  }
};

window.RU = RU;
})();
"""
