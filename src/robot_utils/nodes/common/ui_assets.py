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
  RU.estopButton(el)                      POST "estop"; shows the latched state from "state"
  new RU.Plot(canvas, {seconds, series:[{name, color}], ymin, ymax}); plot.push(t, values)
"""

UI_CSS = r"""
:root{--bg:#fafafa;--fg:#1d1f23;--muted:#6b7280;--card:#fff;--line:#e5e7eb;--acc:#2563eb;--ok:#15803d;--err:#b91c1c;--warn:#b45309}
@media (prefers-color-scheme:dark){:root{--bg:#111316;--fg:#e8eaed;--muted:#9aa0a6;--card:#1b1e23;--line:#2c3036;--acc:#60a5fa;--ok:#4ade80;--err:#f87171;--warn:#fbbf24}}
body{margin:0;font:14px/1.45 system-ui,-apple-system,Segoe UI,Roboto,sans-serif;background:var(--bg);color:var(--fg)}
.ru-wrap{max-width:1100px;margin:0 auto;padding:16px}
.ru-card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 14px;margin:0 0 12px}
.ru-row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
.ru-muted{color:var(--muted)} .ru-ok{color:var(--ok)} .ru-err{color:var(--err)} .ru-warn{color:var(--warn)}
button.ru,.ru-btn{font:inherit;padding:6px 12px;border-radius:8px;border:1px solid var(--line);background:var(--card);color:var(--fg);cursor:pointer}
button.ru:hover{border-color:var(--acc)} button.ru:disabled{opacity:.5;cursor:default}
button.ru-estop{background:#dc2626;color:#fff;border:0;font-weight:700;padding:10px 18px;letter-spacing:.04em}
button.ru-estop.latched{background:#7f1d1d}
.ru-pill{display:inline-block;padding:1px 8px;border-radius:999px;border:1px solid var(--line);font-size:12px}
canvas.ru-plot{width:100%;height:160px;display:block}
table.ru-table{border-collapse:collapse;width:100%} .ru-table td,.ru-table th{padding:3px 6px;border-bottom:1px solid var(--line);text-align:right;font-variant-numeric:tabular-nums}
.ru-table th:first-child,.ru-table td:first-child{text-align:left}
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

RU.estopButton = function(el){
  el.innerHTML = '<button class="ru ru-estop">E-STOP</button> <span class="ru-muted" data-k="s"></span>';
  const b = el.querySelector("button"), s = el.querySelector('[data-k="s"]');
  b.onclick = async () => { try { await RU.api("estop", {method: "POST"}); } catch(e){ s.textContent = e.message; } };
  return {update(st){ const e = st && st.estop; const on = !!(e && e.latched);
    b.classList.toggle("latched", on); s.textContent = on ? "LATCHED (" + (e.source || "") + ": " + (e.reason || "") + ")" : ""; s.className = on ? "ru-err" : "ru-muted"; }};
};

// -- tiny rolling line plot (time in s on x)
RU.Plot = class {
  constructor(canvas, o){ this.c = canvas; this.o = Object.assign({seconds: 5, series: [], ymin: null, ymax: null}, o || {}); this.data = this.o.series.map(() => []); this.t = []; }
  push(t, values){ this.t.push(t); values.forEach((v, i) => this.data[i].push(v)); const t0 = t - this.o.seconds;
    while (this.t.length && this.t[0] < t0){ this.t.shift(); this.data.forEach(d => d.shift()); } }
  draw(){
    const c = this.c, dpr = window.devicePixelRatio || 1, W = c.clientWidth, H = c.clientHeight;
    if (c.width !== W*dpr){ c.width = W*dpr; c.height = H*dpr; }
    const g = c.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
    if (this.t.length < 2) return;
    let lo = this.o.ymin, hi = this.o.ymax;
    if (lo === null || hi === null){ let a = Infinity, b = -Infinity; this.data.forEach(d => d.forEach(v => { if (isFinite(v)){ a = Math.min(a, v); b = Math.max(b, v); } }));
      if (!isFinite(a)){ a = -1; b = 1; } if (a === b){ a -= 1; b += 1; } lo = lo === null ? a : lo; hi = hi === null ? b : hi; }
    const t1 = this.t[this.t.length - 1], t0 = t1 - this.o.seconds, X = t => (t - t0)/(t1 - t0)*W, Y = v => H - (v - lo)/(hi - lo)*(H - 8) - 4;
    g.strokeStyle = getComputedStyle(document.body).getPropertyValue("--line") || "#ccc"; g.beginPath(); g.moveTo(0, Y(0)); g.lineTo(W, Y(0)); g.stroke();
    this.o.series.forEach((s, i) => { g.strokeStyle = s.color || "#2563eb"; g.lineWidth = 1.5; g.beginPath(); let pen = false;
      this.data[i].forEach((v, k) => { if (!isFinite(v)){ pen = false; return; } const x = X(this.t[k]), y = Y(v); if (pen) g.lineTo(x, y); else { g.moveTo(x, y); pen = true; } }); g.stroke(); });
    g.fillStyle = getComputedStyle(document.body).getPropertyValue("--muted") || "#888"; g.font = "11px system-ui";
    g.fillText(hi.toFixed(2), 2, 10); g.fillText(lo.toFixed(2), 2, H - 2);
    let x = 50; this.o.series.forEach(s => { g.fillStyle = s.color || "#2563eb"; g.fillText(s.name, x, 10); x += g.measureText(s.name).width + 12; });
  }
};

window.RU = RU;
})();
"""
