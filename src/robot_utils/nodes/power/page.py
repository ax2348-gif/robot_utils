"""The power node's page (served at the node root). Relative URLs only.

Sources as cards (RU.viz.Battery for every source with a battery), the 24 h history (RU.Plot),
low-voltage settings, and the shutdown control: a hazard button opens a confirm step that says
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
canvas.ru-plot{width:100%;height:120px}
.plots b{display:block;margin-top:8px}
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
    <span id="state" class="ru-pill">…</span></h2>
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
      <b>Charge</b><canvas class="ru-plot" id="ph-pct"></canvas>
      <b>Pack voltage</b><canvas class="ru-plot" id="ph-v"></canvas>
      <b>Current</b> <span class="ru-muted">A, + charging</span><canvas class="ru-plot" id="ph-a"></canvas>
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
<div class="ru-card" data-panel="events" data-role="log"><b>Events</b><div id="events"></div>
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
  if (c.gauge) c.gauge.update(Object.assign(RU.viz.Battery.fromSource(s), {low: !!(last && last.low_voltage && last.low_voltage.low)}));
  const rows = [["state", s.charge_state || "–"]];
  if (s.input) rows.push(["input (USB-C)", s.input.present ? `present · ${f(s.input.v, 2, "V")} · ${f(s.input.a, 2, "A")}` : "absent"]);
  rows.push(["pack", `${f(s.pack_v, 3, "V")} · ${s.current_a > 0 ? "+" : ""}${f(s.current_a, 3, "A")} · ${f(s.power_w, 1, "W")}`]);
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
function renderSources(list){
  const box = $("#sources"), want = list.map(card);
  if (want.length !== box.children.length || want.some((n, i) => box.children[i] !== n)) box.replaceChildren(...want);
}

// ---------------- history ---------------------------------------------------------------------------
const plots = {
  pct: new RU.Plot($("#ph-pct"), {seconds: 86400, series: [{name: "%"}], ymin: 0, ymax: 100}),
  v: new RU.Plot($("#ph-v"), {seconds: 86400, series: [{name: "V", color: "--plot-2"}]}),
  a: new RU.Plot($("#ph-a"), {seconds: 86400, series: [{name: "A", color: "--plot-3"}]}),
};
let hist = null;
function drawHistory(){
  const win = +$("#win").value;
  for (const p of Object.values(plots)){ p.o.seconds = win; p.t = []; p.data = p.o.series.map(() => []); }
  if (hist){
    const I = n => hist.fields.indexOf(n), it = I("t"), ip = I("percent"), iv = I("pack_v"), ia = I("current_a");
    for (const r of hist.rows){ plots.pct.push(r[it], [r[ip]]); plots.v.push(r[it], [r[iv]]); plots.a.push(r[it], [r[ia]]); }
    $("#h-note").textContent = `${hist.source.toUpperCase()}, one point per ${hist.interval_s} s; ${hist.rows.length} points; now at the right edge`;
  }
  for (const p of Object.values(plots)) p.draw();
}
async function loadHistory(){
  try { hist = await RU.api("history?source=ups&seconds=86400"); drawHistory(); }
  catch(e){ $("#h-note").textContent = "history: " + e.message; }
}
$("#win").onchange = drawHistory;
window.addEventListener("resize", () => { for (const p of Object.values(plots)) p.draw(); });
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
  $("#state").className = "ru-pill " + (sd.pending ? "ru-err" : st.error ? "ru-err" : /stale|unavailable/.test(st.state) ? "ru-warn" : "ru-ok");
  $("#fake").hidden = !st.fake;
  $("#err").textContent = st.error || "";
  const ups = st.sources.find(s => s.type === "ups");
  if (ups) hdrBatt.update(Object.assign(RU.viz.Battery.fromSource(ups), {low: !!st.low_voltage.low}));
  renderSources(st.sources); renderShutdown(sd); renderLow(st.low_voltage); renderEvents(st.events);
  if (sd.phase !== lastPhase){ lastPhase = sd.phase; loadAudit(); }
});
</script></body></html>
"""
