"""The power node's page (served at the node root). Relative URLs only.

Sources as cards (RU.viz.Battery for every source with a battery), a live power-flow pill in the
header (input, pack net with its sign, derived load; caution while the pack drains on input), the
24 h history as page-local canvas charts (power flow with the pack's net power filled against zero,
cells, charge, voltage, current; anomaly spans marked; one shared crosshair), the watched
conditions as spans, low-voltage settings, and the shutdown control: a hazard button opens a confirm step that says
what will happen; a pending shutdown shows as a banner with its countdown and a Cancel button,
also on load. User-supplied text (reasons, names) is always inserted as text, never as HTML.
"""

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Power</title>
<link rel="stylesheet" href="common/ui.css">
<style>
.ru-wrap{max-width:1200px}
.layout{display:grid;grid-template-columns:minmax(0,1fr) 360px;gap:12px}
@media (max-width:900px){.layout{grid-template-columns:1fr}}
.sources{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:10px;margin-top:8px}
.src{border:1px solid var(--rule);border-radius:var(--radius);padding:10px 12px}
.src h3{margin:0 0 6px;font-size:15px;display:flex;gap:6px;align-items:center;flex-wrap:wrap}
.kv{display:grid;grid-template-columns:auto 1fr;gap:2px 10px;font-variant-numeric:tabular-nums;margin-top:6px}
.kv span:nth-child(odd){color:var(--muted)}
canvas.chart{display:block;width:100%;height:96px;touch-action:pan-y;cursor:crosshair}
canvas.chart.flow{height:200px}
canvas.chart.cell{height:104px}
canvas.chart.spread{height:76px}
.plots b{display:block;margin-top:10px}
.hread{font-size:12px;min-height:18px;margin:4px 0 0;color:var(--muted)}
.hsum{font-size:12px;margin:2px 0}
.hsum .warn{color:var(--caution)}
.hkey{font-size:12px;margin-top:8px}
.sw{display:inline-block;width:16px;height:9px;vertical-align:baseline;border:1px solid var(--muted);margin:0 2px}
.sw.hatch{border-color:var(--plot-5);background:repeating-linear-gradient(135deg,var(--plot-5) 0 1px,transparent 1px 4px)}
.sw.tint{border-color:var(--plot-5);background:color-mix(in srgb,var(--plot-5) 14%,transparent)}
.sw.mark{height:5px;border-color:var(--caution);background:repeating-linear-gradient(135deg,var(--caution) 0 1px,transparent 1px 3px)}
.cond{border-left:2px solid var(--caution);padding:1px 0 1px 6px;margin:2px 0}
.cond.over{border-left-color:var(--rule);color:var(--muted)}
#events,#audit{max-height:220px;overflow:auto;font-size:12px}
#audit div,#events div{padding:1px 0}
label.f{display:block;margin:8px 0 2px;color:var(--muted)}
input.f{font:inherit;width:100%;box-sizing:border-box;padding:6px 8px;border:1px solid var(--rule);border-radius:var(--radius);background:var(--ground);color:var(--ink)}
button.ru.big{min-height:44px;font-weight:600;width:100%}
.steps{margin:6px 0 6px 18px;padding:0}
.steps li{margin:2px 0}
.warnbox{border-left:4px solid var(--stop);background:var(--stop-bg);padding:6px 10px;margin:8px 0;color:var(--ink)}
.pending{position:sticky;top:8px;z-index:15;box-shadow:0 2px 10px #0004;   /* stays in view while scrolling */
  background:var(--stop-fill);color:var(--on-stop);border-radius:var(--radius);padding:10px 14px;margin:0 0 12px;
  display:flex;gap:12px;align-items:center;flex-wrap:wrap;justify-content:space-between}
.pending[hidden]{display:none}
.pending .count{font-size:28px;font-weight:700;font-variant-numeric:tabular-nums}
.pending button{min-height:48px;min-width:150px;font:inherit;font-weight:700;font-size:16px;border:0;border-radius:var(--radius);
  background:var(--on-stop);color:var(--stop-fill);cursor:pointer;padding:8px 18px}
.pending button:disabled{opacity:.6;cursor:not-allowed}
.row2{display:grid;grid-template-columns:1fr 1fr;gap:8px}
</style><script src="../../themes/theme.js"></script>
</head><body data-page="power"><div id="band"></div><div class="ru-wrap">
<div class="ru-row ru-head">
  <h2 style="margin:6px 0;display:flex;gap:8px;align-items:center;flex-wrap:wrap">Power <span id="hdr-batt" style="width:120px"></span>
    <span id="state" class="ru-pill">…</span>
    <span id="flow" class="ru-pill ru-num" hidden title="USB-C input, pack net (+ charging, − draining) and the load derived from them (input − pack net, includes converter losses)"></span></h2>
  <div class="ru-row"><span id="fake" class="ru-pill ru-warn" hidden>simulated UPS (--fake)</span><a class="ru-muted" href="docs">API docs</a></div>
</div>
<div id="pending" class="pending" role="alert" hidden>
  <div><div><b>SHUTDOWN</b> in <span class="count" id="p-count">–</span></div>
    <div id="p-who"></div><div id="p-phase"></div></div>
  <button id="p-cancel">Cancel shutdown</button>
</div>
<div id="err" class="ru-err"></div>
<div class="layout ru-body">
  <div>
    <div class="ru-card" data-panel="sources" data-role="telemetry"><b>Sources</b>
      <span class="ru-muted">what powers what</span><div class="sources" id="sources"></div></div>
    <div class="ru-card plots" data-panel="history" data-role="telemetry">
      <div class="ru-row ru-spread"><b style="margin:0">History</b>
        <label class="ru-muted">window <select id="win"><option value="3600">1 h</option><option value="21600">6 h</option>
          <option value="86400" selected>24 h</option></select></label></div>
      <span class="ru-muted" id="h-note">UPS, one point per 30 s; now at the right edge</span>
      <div class="hread ru-num" id="h-read"></div>
      <b>Power flow</b> <span class="ru-muted">W · above zero the pack charges, below it drains ·
        load = input − pack net (derived, includes converter losses)</span>
      <canvas class="chart flow" id="ch-flow"></canvas>
      <div class="hsum ru-num" id="h-energy"></div>
      <div class="hsum" id="h-anom"></div>
      <b>Cells</b> <span class="ru-muted">V, lowest cell and spread (max − min)</span>
      <canvas class="chart cell" id="ch-cell"></canvas>
      <canvas class="chart spread" id="ch-spread"></canvas>
      <b>Charge</b><canvas class="chart" id="ch-pct"></canvas>
      <b>Pack voltage</b><canvas class="chart" id="ch-v"></canvas>
      <b>Current</b> <span class="ru-muted">A, + charging</span><canvas class="chart" id="ch-a"></canvas>
      <div class="ru-muted hkey"><span class="sw hatch"></span> pack draining · <span class="sw tint"></span> pack charging ·
        <span class="sw mark"></span> marked span: draining on input (input present, pack net below −0.05 A),
        a low cell (&lt; 3.30 V) or cells &gt; 0.20 V apart</div>
    </div>
  </div>
  <div class="ru-side">
    <div class="ru-card" data-panel="shutdown" data-role="control"><b>Shut down the Pi</b>
      <p class="ru-muted" style="margin:4px 0 8px">Remote power-off, with a countdown anyone can cancel.</p>
      <button class="ru hazard big" id="sd-open">Shut down the Pi…</button>
      <div id="sd-confirm" hidden>
        <p style="margin:10px 0 2px"><b>This will:</b></p>
        <ol class="steps">
          <li>count down for the time below; anyone can cancel it on this page or over SSH;</li>
          <li><b>e-stop</b> every actuated node (swerve, arm, …);</li>
          <li>tell the UPS to cut power about 30 s later, and shut the Pi down.</li>
        </ol>
        <div class="warnbox" id="sd-after">The robot is then unreachable for everyone until it is powered on again.</div>
        <div class="row2"><div><label class="f" for="sd-delay">Countdown (s)</label>
          <input class="f" id="sd-delay" type="number" min="10" max="300" step="1" value="30"></div>
          <div><label class="f" for="sd-by">Your name</label><input class="f" id="sd-by" maxlength="80" placeholder="name"></div></div>
        <label class="f" for="sd-reason">Reason (required, shown to everyone)</label>
        <input class="f" id="sd-reason" maxlength="200" placeholder="e.g. end of day, moving the robot">
        <label class="f" for="sd-type">Type <code>shutdown</code> to confirm</label>
        <input class="f" id="sd-type" autocomplete="off" spellcheck="false">
        <div class="ru-row" style="margin-top:10px"><button class="ru hazard" id="sd-go" disabled>Schedule shutdown</button>
          <button class="ru" id="sd-back">Back</button></div>
      </div>
      <div id="sd-msg" class="ru-err"></div>
      <div id="sd-last" class="ru-muted" style="margin-top:8px"></div>
    </div>
    <div class="ru-card" data-panel="low_voltage" data-role="config"><b>Low-voltage shutdown</b> <span id="lv-status" class="ru-pill">…</span>
      <p class="ru-muted" style="margin:4px 0">When the UPS pack stays below the threshold for the hold time while not
        charging, the node schedules a shutdown (10 s countdown, cancellable). Stale or missing readings never trigger it.</p>
      <p class="ru-warn" style="margin:4px 0">Leave this off until the old ups-dashboard service is retired: it does the
        same job (docs/power.md, Cutover).</p>
      <label><input type="checkbox" id="lv-on"> enabled</label>
      <div class="row2"><div><label class="f" for="lv-v">Threshold (V)</label><input class="f" id="lv-v" type="number" min="11" max="16" step="0.1"></div>
        <div><label class="f" for="lv-hold">Hold (s)</label><input class="f" id="lv-hold" type="number" min="5" max="120" step="1"></div></div>
      <div class="ru-muted" style="font-size:12px;margin-top:2px">11-16 V; 13.2 V = 3.3 V per cell (4 cells)</div>
      <div class="ru-row" style="margin-top:8px"><button class="ru" id="lv-save">Save</button><span id="lv-msg" class="ru-muted"></span></div>
    </div>
  </div>
</div>
<div class="ru-card" data-panel="events" data-role="log"><b>Events</b>
  <div id="conds"></div><div id="events"></div>
  <details style="margin-top:8px"><summary class="ru-muted">Audit log: shutdown requests, cancellations, outcomes, settings (kept on the robot, survives reboots)</summary><div id="audit"></div></details>
</div>
</div>
<script src="common/ui.js"></script>
<script src="common/viz.js"></script>
<script>
const $ = s => document.querySelector(s);
const el = (tag, props, ...kids) => { const e = Object.assign(document.createElement(tag), props || {}); e.append(...kids); return e; };
const f = (v, k, u) => v === null || v === undefined || !isFinite(v) ? "–" : (+v).toFixed(k) + (u ? " " + u : "");
const mins = m => m === null || m === undefined ? "–" : m >= 90 ? (m / 60).toFixed(1) + " h" : Math.round(m) + " min";
const tod = w => w ? new Date(w * 1000).toLocaleTimeString() : "–";
const store = {get(k){ try { return localStorage.getItem(k) || ""; } catch(e){ return ""; } },
               set(k, v){ try { localStorage.setItem(k, v); } catch(e){} }};
const band = RU.stateBand($("#band"));
const hdrBatt = new RU.viz.Battery($("#hdr-batt"), {compact: true, label: "UPS battery"});
let last = null;

// ---------------- sources -------------------------------------------------------------------------
const cards = {};
function kv(box, rows){ box.replaceChildren(...rows.flatMap(([k, v]) => [el("span", {textContent: k}), el("span", {textContent: v})])); }
function card(s){
  let c = cards[s.id];
  if (!c){
    const head = el("h3", {}), body = el("div", {}), list = el("div", {className: "kv"});
    const root = el("div", {className: "src"}, head, body, list);
    c = cards[s.id] = {root, head, body, list, gauge: null};
    if (s.telemetry && s.percent !== undefined){ const g = el("div"); body.append(g); c.gauge = new RU.viz.Battery(g, {label: s.id + " battery"}); }
  }
  const okTxt = !s.telemetry ? "no telemetry" : s.ok ? "ok" : s.stale ? "stale" : "no reading";
  c.head.replaceChildren(el("span", {textContent: s.id === "ups" ? "UPS (Pi)" : s.id === "main" ? "Main" : s.id}),
    el("span", {className: "ru-pill", textContent: s.type}), el("span", {className: "ru-muted", textContent: "powers: " + s.role}),
    el("span", {className: "ru-pill " + (!s.telemetry ? "" : s.ok ? "ru-ok" : "ru-err"), textContent: okTxt}));
  if (!s.telemetry){
    c.body.replaceChildren(el("div", {textContent: s.type === "tether" ? "Tether, no telemetry" : "No telemetry"}),
                           el("div", {className: "ru-muted", textContent: s.note || ""}));
    kv(c.list, []);
    return c.root;
  }
  if (c.gauge) c.gauge.update(gaugeData(s, !!(last && last.low_voltage && last.low_voltage.low)));
  const rows = [["state", s.draining_on_input ? `draining on input (charger bit: ${s.charge_state})` : s.charge_state || "–"]];
  if (s.input) rows.push(["input (USB-C)", s.input.present ? `present · ${f(s.input.v, 2, "V")} · ${f(s.input.a, 2, "A")} · ${f(s.input.w, 1, "W")}` : "absent"]);
  rows.push(["pack", `${f(s.pack_v, 3, "V")} · ${sg(s.current_a, 3)} A · ${sg(s.power_w, 1)} W`]);
  if (s.input && s.power_w !== null && s.power_w !== undefined)
    rows.push(["load (derived)", `≈${f((s.input.present ? s.input.w : 0) - s.power_w, 1, "W")} = input − pack net (includes converter losses)`]);
  if (s.cells_v) rows.push(["cells", s.cells_v.map(v => f(v, 3)).join(" ") + " V"]);
  rows.push(["runtime", s.runtime_min !== null && s.runtime_min !== undefined ? `~${mins(s.runtime_min)} (${s.runtime_source})` : s.charging ? "on input" : "–"]);
  if (s.time_to_full_min !== null && s.time_to_full_min !== undefined) rows.push(["time to full", mins(s.time_to_full_min)]);
  if (s.remaining_mah !== undefined) rows.push(["remaining", f(s.remaining_mah, 0, "mAh")]);
  if (s.charger_state) rows.push(["charger", s.charger_state + (s.fast_charge ? " (fast)" : "")]);
  if (s.links) rows.push(["links", `gauge ${s.links.fuel_gauge ? "ok" : "ERROR"} · charger ${s.links.charger ? "ok" : "ERROR"}`]);
  if (s.control) rows.push(["auto power-on", s.control.auto_power_on ? "on" : "off"]);
  if (s.firmware) rows.push(["firmware", "V" + s.firmware]);
  if (s.power_off_pending) rows.push(["HAT", "POWER-OFF ARMED"]);
  rows.push(["reading", s.age_s === null ? "none yet" : s.age_s + " s ago" + (s.error ? " · " + s.error : "")]);
  kv(c.list, rows);
  return c.root;
}
// the gauge shows a charging bolt from charge_state; while the pack drains on input it is not charging
function gaugeData(s, low){ const d = Object.assign(RU.viz.Battery.fromSource(s), {low});
  if (s.draining_on_input) d.charging = false; return d; }
function flowText(u){
  const inW = u.input && u.input.present ? u.input.w : 0, load = inW - u.power_w;
  return `in ${f(inW, 1, "W")} · pack ${sg(u.power_w, 1)} W` + (u.draining_on_input ? " draining on input" : u.power_w < 0 ? " draining" : u.power_w > 0 ? " charging" : "") +
         ` · load ≈${f(load, 1, "W")}`;
}
function renderSources(list){
  const box = $("#sources"), want = list.map(card);
  if (want.length !== box.children.length || want.some((n, i) => box.children[i] !== n)) box.replaceChildren(...want);
}

// ---------------- history ---------------------------------------------------------------------------
// Page-local charts (RU.Plot has no zero fill, spans or time axis): thin lines on a shared time axis, the
// pack's net power filled against zero (faint tint = charging, hatching = draining), anomaly spans as a
// hatched caution strip at the top with dotted edges, and one crosshair shared by every chart.
const sg = (v, k) => v === null || v === undefined || !isFinite(v) ? "–" : (v > 0 ? "+" : v < 0 ? "−" : "") + Math.abs(v).toFixed(k);
const dur = s => s < 3600 ? Math.max(1, Math.round(s / 60)) + " min" : Math.floor(s / 3600) + " h " + String(Math.round(s % 3600 / 60)).padStart(2, "0") + " min";
const hm = w => new Date(w * 1000).toLocaleTimeString([], {hour: "2-digit", minute: "2-digit"});
const DEF_LIMITS = {drain_a: -0.05, cell_low_v: 3.30, spread_v: 0.20};
let hist = null, view = null, hoverT = null;
const charts = [];
function drawAll(){ if (view) for (const ch of charts) ch.draw(view); readout(); }

class Chart {
  // o: {series: [{key, name, color, dash, width}], zero, fill (key filled against zero), refs: [{v(lim), name}],
  //     ymin, ymax, digits, marks: [span kinds]}
  constructor(canvas, o){
    this.c = canvas; this.o = o; this.geo = null; charts.push(this);
    const move = e => { if (!this.geo) return; const r = this.c.getBoundingClientRect(); hoverT = this.geo.tOf(e.clientX - r.left); drawAll(); };
    canvas.addEventListener("pointermove", move); canvas.addEventListener("pointerdown", move);
    canvas.addEventListener("pointerleave", () => { hoverT = null; drawAll(); });
  }
  draw(v){
    const c = this.c, o = this.o, dpr = window.devicePixelRatio || 1, W = c.clientWidth, H = c.clientHeight;
    if (!W || !H) return;
    if (c.width !== Math.round(W * dpr) || c.height !== Math.round(H * dpr)){ c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
    const g = c.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
    const tk = (n, d) => RU.token(n, d), col = s => s && s.startsWith("--") ? tk(s, "#1F5FBF") : s;
    const muted = tk("--muted", "#5B6672"), rule = tk("--rule", "#D3DAE1"), ink = tk("--ink", "#15202B"), caution = tk("--caution", "#A86400");
    g.font = "11px " + tk("--font", "system-ui"); g.lineCap = "butt";
    const L = 40, R = 6, TOP = 15, STRIP = 5, PT = TOP + STRIP + 5, PB = H - 15;
    const {rows, t0, t1, iv} = v, X = t => L + (t - t0) / (t1 - t0) * (W - L - R);
    this.geo = {tOf: x => t0 + (x - L) / (W - L - R) * (t1 - t0)};
    const cols = o.series.map(s => v.col(s.key)), lim = v.limits;
    let lo = Infinity, hi = -Infinity;
    for (const r of rows) for (const i of cols){ const y = r[i]; if (y !== null && isFinite(y)){ lo = Math.min(lo, y); hi = Math.max(hi, y); } }
    if (!isFinite(lo)){ lo = 0; hi = 1; }
    if (o.zero){ lo = Math.min(lo, 0); hi = Math.max(hi, 0); }
    for (const rf of o.refs || []){ const y = rf.v(lim); lo = Math.min(lo, y); hi = Math.max(hi, y); }
    if (hi - lo < 1e-6){ lo -= 0.5; hi += 0.5; }
    const pad = (hi - lo) * 0.08;
    lo = o.ymin !== undefined ? o.ymin : lo - pad; hi = o.ymax !== undefined ? o.ymax : hi + pad;
    const Y = y => PB - (y - lo) / (hi - lo) * (PB - PT);
    // baseline and time ticks (local clock; a few per window, no grid)
    g.strokeStyle = rule; g.lineWidth = 1; g.beginPath(); g.moveTo(L, PB + .5); g.lineTo(W - R, PB + .5); g.stroke();
    const span = t1 - t0, step = [600, 1800, 3600, 7200, 10800, 21600, 43200].find(st => st / span * (W - L - R) >= 70) || 43200;
    const tz = -new Date().getTimezoneOffset() * 60, w0 = v.wallOf(t0), w1 = v.wallOf(t1);
    g.fillStyle = muted; g.textBaseline = "alphabetic"; g.textAlign = "center";
    for (let w = Math.ceil((w0 + tz) / step) * step - tz; w <= w1; w += step){
      const x = Math.round(X(v.tOf(w))) + .5; g.beginPath(); g.moveTo(x, PB); g.lineTo(x, PB + 3); g.stroke();
      if (x > L + 14 && x < W - R - 14) g.fillText(hm(w), x, H - 2);
    }
    // anomaly spans: hatched strip with an outline at the top, dotted edges down through the plot
    g.textAlign = "left";
    for (const kind of o.marks || []) for (const sp of v.spans[kind] || []){
      const x0 = Math.max(L, X(sp.t0)), x1 = Math.min(W - R, Math.max(X(sp.t1), x0 + 2));
      if (x1 <= L || x0 >= W - R) continue;
      g.save(); g.beginPath(); g.rect(x0, TOP, x1 - x0, STRIP); g.clip();
      g.strokeStyle = caution; g.lineWidth = 1; g.beginPath();
      for (let x = x0 - STRIP; x < x1; x += 3){ g.moveTo(x, TOP + STRIP); g.lineTo(x + STRIP, TOP); }
      g.stroke(); g.restore();
      g.strokeStyle = caution; g.lineWidth = 1; g.strokeRect(x0 + .5, TOP + .5, x1 - x0 - 1, STRIP - 1);
      g.save(); g.globalAlpha = .6; g.setLineDash([1, 3]); g.beginPath();
      for (const x of [x0, sp.ongoing ? null : x1]) if (x !== null){ g.moveTo(Math.round(x) + .5, TOP + STRIP); g.lineTo(Math.round(x) + .5, PB); }
      g.stroke(); g.restore();
      if (o.spanLabel){                                       // the longest label that fits inside the span
        const fit = o.spanLabel(kind, sp, v).find(t => g.measureText(t).width < x1 - x0 - 6);
        if (fit){ g.fillStyle = caution; g.textBaseline = "top"; g.fillText(fit, x0 + 3, PT - 2); } }
    }
    // runs of rows without a gap, for lines and the fill
    const runs = i => { const out = []; let cur = [], pt = null;
      for (const r of rows){ const y = r[i];
        if (y === null || !isFinite(y) || (pt !== null && r[v.it] - pt > 3 * iv)){ if (cur.length) out.push(cur); cur = []; }
        if (y !== null && isFinite(y)) cur.push([X(r[v.it]), Y(y)]);
        pt = r[v.it]; }
      if (cur.length) out.push(cur); return out; };
    const y0 = Math.max(PT, Math.min(PB, Y(0)));
    if (o.fill){
      const s = o.series.find(x => x.key === o.fill), color = col(s.color);
      for (const run of runs(v.col(o.fill))){
        if (run.length < 2) continue;
        const area = new Path2D(); area.moveTo(run[0][0], y0); for (const [x, y] of run) area.lineTo(x, y); area.lineTo(run[run.length - 1][0], y0); area.closePath();
        const xa = run[0][0], xb = run[run.length - 1][0];
        g.save(); g.clip(area); g.beginPath(); g.rect(xa, PT - 2, xb - xa, y0 - PT + 2); g.clip();
        g.globalAlpha = .13; g.fillStyle = color; g.fillRect(xa, PT - 2, xb - xa, y0 - PT + 2); g.restore();      // charging: faint tint
        g.save(); g.clip(area); g.beginPath(); g.rect(xa, y0, xb - xa, PB - y0 + 2); g.clip();
        g.globalAlpha = .55; g.strokeStyle = color; g.lineWidth = 1; g.beginPath();                               // draining: hatching
        for (let x = xa - (PB - y0); x < xb; x += 5){ g.moveTo(x, PB); g.lineTo(x + (PB - y0), y0); }
        g.stroke(); g.restore();
      }
    }
    if (o.zero && lo < 0 && hi > 0){ g.strokeStyle = ink; g.globalAlpha = .5; g.lineWidth = 1; g.beginPath(); g.moveTo(L, Math.round(y0) + .5); g.lineTo(W - R, Math.round(y0) + .5); g.stroke(); g.globalAlpha = 1; }
    for (const rf of o.refs || []){ const y = Math.round(Y(rf.v(lim))) + .5;
      g.strokeStyle = caution; g.lineWidth = 1; g.setLineDash([4, 3]); g.beginPath(); g.moveTo(L, y); g.lineTo(W - R, y); g.stroke(); g.setLineDash([]);
      g.fillStyle = caution; g.textBaseline = "bottom"; g.fillText(rf.name(lim), L + 4, y - 1); }
    o.series.forEach((s, k) => {
      g.strokeStyle = col(s.color); g.lineWidth = s.width || 1.4; g.setLineDash(s.dash || []); g.lineJoin = "round";
      for (const run of runs(cols[k])){ g.beginPath(); run.forEach(([x, y], j) => j ? g.lineTo(x, y) : g.moveTo(x, y)); g.stroke(); }
    });
    g.setLineDash([]);
    // y labels in the gutter, legend right-aligned in the top strip
    const d = o.digits === undefined ? 1 : o.digits;
    g.fillStyle = muted; g.textAlign = "right";
    g.textBaseline = "top"; g.fillText(hi.toFixed(d), L - 5, PT - 3);
    g.textBaseline = "bottom"; g.fillText(lo.toFixed(d), L - 5, PB + 3);
    if (o.zero && lo < 0 && hi > 0 && y0 - PT > 12 && PB - y0 > 12){ g.textBaseline = "middle"; g.fillText("0", L - 5, y0); }
    g.textAlign = "left"; g.textBaseline = "middle";
    let x = W - R;
    for (let k = o.series.length - 1; k >= 0; k--){ const s = o.series[k], w = g.measureText(s.name).width;
      x -= w; g.fillStyle = col(s.color); g.fillText(s.name, x, 6);
      g.strokeStyle = g.fillStyle; g.lineWidth = 2; g.setLineDash(s.dash || []); g.beginPath(); g.moveTo(x - 18, 6.5); g.lineTo(x - 4, 6.5); g.stroke(); g.setLineDash([]);
      x -= 30; }
    // crosshair
    if (hoverT !== null && hoverT >= t0 && hoverT <= t1){ const hx = Math.round(X(hoverT)) + .5;
      g.strokeStyle = ink; g.globalAlpha = .45; g.lineWidth = 1; g.beginPath(); g.moveTo(hx, PT - 4); g.lineTo(hx, PB); g.stroke(); g.globalAlpha = 1;
      const r = v.at(hoverT);
      if (r) for (let k = 0; k < cols.length; k++){ const y = r[cols[k]]; if (y === null || !isFinite(y)) continue;
        g.fillStyle = col(o.series[k].color); g.beginPath(); g.arc(X(r[v.it]), Y(y), 2.5, 0, 7); g.fill(); }
    }
  }
}

const limTxt = (k, u) => lim => (lim[k]).toFixed(2) + " " + u;
const meanIn = (v, key, sp) => { const i = v.col(key); let a = 0, n = 0;
  for (const r of v.rows) if (r[v.it] >= sp.t0 && r[v.it] <= sp.t1 && r[i] !== null && isFinite(r[i])){ a += r[i]; n++; } return n ? a / n : NaN; };
new Chart($("#ch-flow"), {zero: true, fill: "battery_w", digits: 1, marks: ["drain"],
  spanLabel: (k, sp, v) => { const p = sg(meanIn(v, "battery_w", sp), 1) + " W";
    return [`draining on input: pack ${p}, input ${f(meanIn(v, "input_w", sp), 1, "W")}`, `draining on input: pack ${p}`, `pack ${p}`, p]; }, series: [
  {key: "input_w", name: "input", color: "--plot-1", width: 1.5},
  {key: "battery_w", name: "pack net", color: "--plot-5", width: 1.8},
  {key: "load_w", name: "load (derived)", color: "--muted", dash: [5, 3], width: 1.3}]});
new Chart($("#ch-cell"), {digits: 2, marks: ["cell_low"], series: [{key: "cell_min_v", name: "lowest cell", color: "--plot-3"}],
  refs: [{v: lim => lim.cell_low_v, name: lim => "low " + lim.cell_low_v.toFixed(2) + " V"}]});
new Chart($("#ch-spread"), {digits: 2, ymin: 0, marks: ["spread"], series: [{key: "cell_spread_v", name: "spread", color: "--plot-4"}],
  refs: [{v: lim => lim.spread_v, name: lim => "unbalanced > " + lim.spread_v.toFixed(2) + " V"}]});
new Chart($("#ch-pct"), {digits: 0, ymin: 0, ymax: 100, marks: ["drain"], series: [{key: "percent", name: "%", color: "--plot-1"}]});
new Chart($("#ch-v"), {digits: 2, marks: ["drain"], series: [{key: "pack_v", name: "V", color: "--plot-2"}]});
new Chart($("#ch-a"), {digits: 2, zero: true, marks: ["drain"], series: [{key: "current_a", name: "A", color: "--plot-3"}]});

// spans of rows where pred holds (rows are HISTORY_S apart; a gap of > 3 intervals ends a span)
function spansOf(rows, it, iw, iv, now, wallNow, pred){
  const out = []; let cur = null, prev = null;
  for (const r of rows){
    const gap = prev !== null && r[it] - prev[it] > 3 * iv, on = pred(r);
    if (cur && (gap || !on)){ if (gap){ cur.t1 = prev[it] + iv; cur.w1 = prev[iw] + iv; } else { cur.t1 = r[it]; cur.w1 = r[iw]; } out.push(cur); cur = null; }
    if (on && !cur) cur = {t0: r[it], w0: r[iw]};
    prev = r;
  }
  if (cur){ cur.t1 = now; cur.w1 = wallNow; cur.ongoing = prev[it] >= now - 3 * iv; if (!cur.ongoing){ cur.t1 = prev[it] + iv; cur.w1 = prev[iw] + iv; } out.push(cur); }
  return out;
}
function buildView(){
  const win = +$("#win").value, I = n => hist.fields.indexOf(n), it = I("t"), iw = I("wall"), iv = hist.interval_s;
  const t1 = hist.now, t0 = t1 - win, rows = hist.rows.filter(r => r[it] >= t0 - iv);
  const last = hist.rows[hist.rows.length - 1], off = last ? last[iw] - last[it] : Date.now() / 1000 - t1;
  const lim = Object.assign({}, DEF_LIMITS, hist.limits || {}), v0 = (r, k) => { const x = r[I(k)]; return x === null || x === undefined ? NaN : x; };
  const wallNow = t1 + off;
  const sp = pred => spansOf(rows, it, iw, iv, t1, wallNow, pred);
  return {rows, t0, t1, iv, it, limits: lim, col: I, wallOf: t => t + off, tOf: w => w - off,
    at: t => { let best = null; for (const r of rows){ if (best === null || Math.abs(r[it] - t) < Math.abs(best[it] - t)) best = r; } return best && Math.abs(best[it] - t) <= 2 * iv ? best : null; },
    spans: {drain: sp(r => v0(r, "input_present") >= 0.5 && v0(r, "current_a") < lim.drain_a),
            cell_low: sp(r => v0(r, "cell_min_v") < lim.cell_low_v),
            spread: sp(r => v0(r, "cell_spread_v") > lim.spread_v)}};
}
function readout(){
  const box = $("#h-read");
  if (!view || !hist){ box.textContent = ""; return; }
  const r = hoverT !== null ? view.at(hoverT) : view.rows[view.rows.length - 1];
  if (!r){ box.textContent = hoverT !== null ? "no reading here" : ""; return; }
  const g = k => r[view.col(k)], bw = g("battery_w");
  box.textContent = `${hoverT !== null ? "at " : "latest "}${hm(r[view.col("wall")])} · input ${f(g("input_w"), 1, "W")} · pack ${sg(bw, 1)} W` +
    `${bw < 0 && g("input_present") >= 0.5 && g("current_a") < view.limits.drain_a ? " (draining on input)" : bw < 0 ? " (draining)" : bw > 0 ? " (charging)" : ""}` +
    ` · load ≈${f(g("load_w"), 1, "W")} · ${f(g("percent"), 0, "%")} · ${f(g("pack_v"), 2, "V")} · lowest cell ${f(g("cell_min_v"), 2, "V")} · spread ${f(g("cell_spread_v"), 2, "V")}`;
}
function summarize(){
  // energy in / out of the pack over the window (rows 30 s apart; a gap counts one interval)
  const {rows, it, iv} = view, ib = view.col("battery_w"), ii = view.col("input_w"), il = view.col("load_w");
  let into = 0, out = 0, inWh = 0, loadWh = 0, secs = 0;
  for (let k = 0; k < rows.length; k++){ const r = rows[k], dt = k + 1 < rows.length ? Math.min(rows[k + 1][it] - r[it], iv) : iv;
    if (r[it] < view.t0) continue;
    const b = r[ib]; if (b !== null && isFinite(b)){ if (b > 0) into += b * dt; else out += b * dt; }
    if (r[ii] !== null && isFinite(r[ii])) inWh += r[ii] * dt;
    if (r[il] !== null && isFinite(r[il])){ loadWh += r[il] * dt; secs += dt; } }
  const wh = x => (x / 3600).toFixed(1);
  $("#h-energy").replaceChildren(
    el("span", {className: "ru-muted", textContent: "in this window: "}),
    el("span", {textContent: `into the pack +${wh(into)} Wh · out of it −${wh(-out)} Wh · net ${sg((into + out) / 3600, 1)} Wh`,
                className: into + out < 0 ? "warn" : ""}),
    el("span", {className: "ru-muted", textContent: secs ? ` · input ${wh(inWh)} Wh · load ≈${wh(loadWh)} Wh (avg ${f(loadWh / secs, 1, "W")}, derived)` : ""}));
  const names = {drain: "draining on input", cell_low: "low cell", spread: "cells unbalanced"}, parts = [];
  for (const [k, list] of Object.entries(view.spans)){ if (!list.length) continue;
    const total = list.reduce((a, s) => a + (s.w1 - s.w0), 0);
    parts.push(`${names[k]}: ` + list.slice(-3).map(s => s.ongoing ? `since ${hm(s.w0)} (ongoing, ${dur(s.w1 - s.w0)})` : `${hm(s.w0)}–${hm(s.w1)} (${dur(s.w1 - s.w0)})`).join(", ") +
      (list.length > 3 ? ` (+${list.length - 3} earlier)` : "") + (list.length > 1 ? `; ${dur(total)} in all` : "")); }
  $("#h-anom").replaceChildren(parts.length ? el("span", {className: "warn", textContent: parts.join(" · ")})
                                            : el("span", {className: "ru-muted", textContent: "no anomaly spans in this window"}));
}
function drawHistory(){
  if (!hist) return;
  view = buildView();
  $("#h-note").textContent = `${hist.source.toUpperCase()}, one point per ${hist.interval_s} s; ${view.rows.length} points; now at the right edge; hover or touch a chart for values`;
  summarize(); drawAll();
}
async function loadHistory(){
  try { hist = await RU.api("history?source=ups&seconds=86400"); drawHistory(); }
  catch(e){ $("#h-note").textContent = "history: " + e.message; }
}
$("#win").onchange = drawHistory;
window.addEventListener("resize", drawAll);
if (window.matchMedia) matchMedia("(prefers-color-scheme: dark)").addEventListener("change", drawAll);   // canvases read the theme's tokens
new MutationObserver(drawAll).observe(document.documentElement, {attributes: true, attributeFilter: ["data-theme", "class", "style"]});
loadHistory(); setInterval(loadHistory, 30000);

// ---------------- shutdown -------------------------------------------------------------------------
let deadline = null;            // local performance.now()/1000 at which the countdown ends
function renderShutdown(sd){
  const p = $("#pending");
  p.hidden = !sd.pending;
  $("#sd-open").disabled = !!sd.pending;
  if (sd.pending){
    if (sd.phase === "countdown") deadline = performance.now() / 1000 + sd.in_s; else deadline = null;
    $("#p-who").textContent = `requested by ${sd.by} (${sd.client}, ${sd.source}) at ${tod(sd.requested_at)}: “${sd.reason}”`;
    $("#p-phase").textContent = sd.phase === "countdown" ? "then: e-stop all, UPS power-off, systemctl poweroff" :
      sd.phase === "executing" ? "e-stopping the robot…" : sd.phase === "powering_off" ? "powering off…" : sd.phase;
    $("#p-cancel").disabled = sd.phase !== "countdown";
    $("#sd-confirm").hidden = true;
  } else deadline = null;
  const l = sd.last;
  $("#sd-last").textContent = !l ? "" : l.phase === "cancelled" ?
    `Last: requested by ${l.by} at ${tod(l.requested_at)} (“${l.reason}”), cancelled by ${l.cancelled_by} at ${tod(l.cancelled_at)}.` :
    `Last: requested by ${l.by} at ${tod(l.requested_at)} (“${l.reason}”): ${l.phase}${l.error ? " · " + l.error : ""}.`;
  tick();
}
function tick(){
  if (deadline !== null) $("#p-count").textContent = Math.max(0, deadline - performance.now() / 1000).toFixed(0) + " s";
  else if (last && last.shutdown.pending) $("#p-count").textContent = "now";
}
setInterval(tick, 250);
function afterText(){
  const u = last && last.sources.find(s => s.type === "ups");
  const auto = u && u.control && u.control.auto_power_on, input = u && u.input_present;
  return "The robot is then unreachable for everyone until it is powered on again. " + (u ? (auto && input ?
    "USB-C input is present and the HAT's auto power-on is on, so the HAT may power the Pi back up after it cuts power." :
    input ? "The HAT's auto power-on is off: someone must power it on by hand." :
    "There is no USB-C input: it stays off until input power returns or someone power-cycles it.") : "");
}
const sdValid = () => $("#sd-type").value === "shutdown" && $("#sd-reason").value.trim().length > 0;
for (const id of ["#sd-type", "#sd-reason"]) $(id).addEventListener("input", () => $("#sd-go").disabled = !sdValid());
$("#sd-open").onclick = () => { $("#sd-confirm").hidden = false; $("#sd-msg").textContent = ""; $("#sd-after").textContent = afterText();
  $("#sd-by").value = $("#sd-by").value || store.get("ru-client"); $("#sd-type").value = ""; $("#sd-go").disabled = true; $("#sd-reason").focus(); };
$("#sd-back").onclick = () => { $("#sd-confirm").hidden = true; $("#sd-type").value = ""; };
$("#sd-confirm").addEventListener("keydown", e => { if (e.key === "Escape") $("#sd-back").click(); });
$("#sd-go").onclick = async () => {
  if (!sdValid()) return;
  const by = $("#sd-by").value.trim() || "browser"; store.set("ru-client", by);
  $("#sd-go").disabled = true;
  try {
    const sd = await RU.api("shutdown", {body: {confirm: $("#sd-type").value, reason: $("#sd-reason").value.trim(), by,
                                                delay_s: +$("#sd-delay").value}});
    $("#sd-msg").textContent = ""; $("#sd-type").value = ""; $("#sd-reason").value = ""; renderShutdown(sd);
  } catch(e){ $("#sd-msg").textContent = e.message; $("#sd-go").disabled = !sdValid(); }
};
$("#p-cancel").onclick = async () => {
  $("#p-cancel").disabled = true;
  try { renderShutdown(await RU.api("shutdown/cancel", {body: {by: store.get("ru-client") || "browser"}})); }
  catch(e){ $("#p-phase").textContent = "cancel failed: " + e.message; $("#p-cancel").disabled = false; }
};

// ---------------- low voltage ------------------------------------------------------------------------
let lvFilled = false;
function renderLow(lv){
  $("#lv-status").textContent = lv.status;
  $("#lv-status").className = "ru-pill " + (!lv.enabled ? "" : lv.low ? "ru-err" : lv.status === "armed" ? "ru-ok" : "ru-warn");
  if (!lvFilled){ $("#lv-on").checked = lv.enabled; $("#lv-v").value = lv.volts; $("#lv-hold").value = lv.hold_s; lvFilled = true; }
}
async function saveLow(confirm){
  const body = {enabled: $("#lv-on").checked, volts: +$("#lv-v").value, hold_s: +$("#lv-hold").value, confirm: !!confirm,
                by: store.get("ru-client") || "browser"};
  try { const lv = await RU.api("low_voltage", {body}); lvFilled = false; renderLow(lv); $("#lv-msg").textContent = "saved"; }
  catch(e){
    if (!confirm && e.message.startsWith("409") && window.confirm(e.message.replace(/^409: /, "") + "\n\nEnable anyway?")) return saveLow(true);
    $("#lv-msg").textContent = e.message;
  }
}
$("#lv-save").onclick = () => saveLow(false);

// ---------------- log --------------------------------------------------------------------------------
function renderEvents(evs){
  $("#events").replaceChildren(...evs.slice().reverse().map(e =>
    el("div", {className: e.level === "error" ? "ru-ev-error" : e.level !== "info" ? "ru-ev-warn" : "", textContent: `${tod(e.wall)} ${e.msg}`})));
}
// conditions the node watches (input lost, draining on input, low cell, unbalanced), as spans, newest first
function renderConds(list){
  const now = Date.now() / 1000;
  $("#conds").replaceChildren(...list.slice().reverse().map(c => el("div", {className: "cond" + (c.active ? "" : " over"),
    textContent: `${c.label} · ` + (c.active ? `since ${hm(c.start)} (ongoing, ${dur(now - c.start)})` : `${hm(c.start)}–${hm(c.end)} (${dur(c.end - c.start)})`) +
                 (c.detail ? ` · at start: ${c.detail}` : "")})));
}
async function loadAudit(){
  try {
    const d = await RU.api("log?n=50");
    $("#audit").replaceChildren(...d.log.slice().reverse().map(r => el("div", {textContent:
      `${r.time} ${r.event}` + (r.by ? ` by ${r.by}` : "") + (r.client ? ` [${r.client}]` : "") + (r.reason ? `: ${r.reason}` : "") +
      (r.detail ? ` (${r.detail})` : "") + (r.error ? ` (${r.error})` : "")})));
  } catch(e){}
}
loadAudit(); setInterval(loadAudit, 10000);

// ---------------- state --------------------------------------------------------------------------------
let lastPhase = null;
RU.sse("events", st => {
  last = st; band.update(st);
  const sd = st.shutdown;
  $("#state").textContent = st.state;
  $("#state").className = "ru-pill " + (sd.pending ? "ru-err" : st.error ? "ru-err" : /stale|unavailable|draining/.test(st.state) ? "ru-warn" : "ru-ok");
  $("#fake").hidden = !st.fake;
  $("#err").textContent = st.error || "";
  const ups = st.sources.find(s => s.type === "ups");
  if (ups) hdrBatt.update(gaugeData(ups, !!st.low_voltage.low));
  const fl = $("#flow"), live = ups && ups.ok && ups.power_w !== undefined && ups.power_w !== null;
  fl.hidden = !live;
  if (live){ fl.textContent = flowText(ups); fl.className = "ru-pill ru-num" + (ups.draining_on_input ? " ru-warn" : ""); }
  renderSources(st.sources); renderShutdown(sd); renderLow(st.low_voltage); renderEvents(st.events); renderConds(st.conditions || []);
  if (sd.phase !== lastPhase){ lastPhase = sd.phase; loadAudit(); }
});
</script></body></html>
"""
