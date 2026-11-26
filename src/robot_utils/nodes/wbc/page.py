"""The WBC node's page (served at the node root). Relative URLs only."""

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Whole-body control</title>
<link rel="stylesheet" href="common/ui.css">
<style>
.ru-wrap{max-width:1300px}
.layout{display:grid;grid-template-columns:minmax(0,1fr) 380px;gap:12px}
@media (max-width:900px){.layout{grid-template-columns:1fr}}
.kv{display:grid;grid-template-columns:auto 1fr;gap:2px 10px;font-variant-numeric:tabular-nums}
.kv span:nth-child(odd){color:var(--muted)}
canvas.ru-plot{width:100%;height:110px}
#events{max-height:180px;overflow:auto;font-size:12px}
.ctl{display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin:6px 0}
table.ee{border-collapse:collapse;font-variant-numeric:tabular-nums;width:100%}
table.ee td,table.ee th{padding:1px 6px;text-align:right;border-bottom:1px solid var(--line)}
table.ee th:first-child,table.ee td:first-child{text-align:left;color:var(--muted);font-weight:normal}
.cfg{display:grid;grid-template-columns:repeat(auto-fill,minmax(230px,1fr));gap:2px 14px;font-size:12px}
.cfg label{display:flex;justify-content:space-between;gap:6px;align-items:center}
.cfg label span{color:var(--muted);overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.cfg input{width:92px;font:inherit;font-variant-numeric:tabular-nums}
.cfg input.dirty{outline:1px solid var(--accent, currentColor)}
.cfg h4{grid-column:1/-1;margin:8px 0 2px;font-size:12px;font-weight:600}
</style><script src="../../themes/theme.js"></script>
</head><body data-page="wbc"><div id="band"></div><div class="ru-wrap">
<div class="ru-row ru-head">
  <h2 style="margin:6px 0">Whole-body control <span id="status" class="ru-pill">…</span></h2>
  <div class="ru-row"><span id="estop"></span><a class="ru-muted" href="docs">API docs</a></div>
</div>
<div id="err" class="ru-err"></div>
<div class="layout ru-body">
  <div class="ru-card" data-panel="plots" data-role="view">
    <b>End effector</b> <span class="ru-muted">commanded vs measured, world frame</span>
    <table class="ee" id="ee"></table>
    <b>Tracking error, position</b> <span class="ru-muted">m, 20 s · QP shortfall (target − commanded), actual (target − measured), lead (commanded − measured)</span><canvas class="ru-plot" id="pe"></canvas>
    <b>Tracking error, orientation</b> <span class="ru-muted">rad, the same three</span><canvas class="ru-plot" id="pr"></canvas>
    <b>Base twist</b> <span class="ru-muted">vx, vy m/s · w rad/s (body)</span><canvas class="ru-plot" id="pb"></canvas>
    <b>Solve time</b> <span class="ru-muted">ms per tick</span><canvas class="ru-plot" id="ps"></canvas>
  </div>
  <div class="ru-side">
    <div class="ru-card" data-panel="control" data-role="control"><b>Control</b>
      <div class="ctl" id="leasew"></div>
      <div class="ctl"><button class="ru live" id="enable">Enable</button><button class="ru" id="disable">Disable</button></div>
      <div class="ctl"><span class="ru-muted">base mode</span>
        <select id="mode"><option>free</option><option>locked</option><option>hint</option><option>commanded</option></select>
        <button class="ru" id="setmode">Set</button></div>
      <div class="ru-muted">Targets arrive as datagrams (POST /session). Enable takes the arm and swerve leases.</div>
    </div>
    <div class="ru-card" data-panel="state" data-role="telemetry"><b>State</b><div class="kv" id="kv"></div></div>
  </div>
</div>
<div class="ru-card" data-panel="config" data-role="config"><b>Config</b>
  <span class="ru-muted">applies at once (no lease); structural keys only while disabled; hard limits: <span id="hard">–</span></span>
  <div class="ctl"><button class="ru" id="apply">Apply changed</button><button class="ru" id="revert">Revert edits</button>
    <span class="ru-muted" id="cfgmsg"></span></div>
  <div class="cfg" id="cfg"></div>
</div>
<div class="ru-card" data-panel="events" data-role="log"><b>Events</b><div id="events"></div></div>
</div>
<script src="common/ui.js"></script>
<script>
const $ = s => document.querySelector(s);
const err = m => $("#err").textContent = m || "";
const lease = RU.leaseWidget($("#leasew"));
const call = async (p, body) => { try { err(""); return await RU.api(p, {method: "POST", body: body || {}, headers: lease.headers()}); } catch(e){ err(e.message); } };
const es = RU.estopButton($("#estop"));
const band = RU.stateBand($("#band"));
$("#enable").onclick = () => call("enable");
$("#disable").onclick = () => call("disable");
$("#setmode").onclick = () => call("hints", {base: {mode: $("#mode").value}});
// config: every key, grouped by prefix-free sections in declaration order
const GROUPS = [["Base caps", /^(max_speed|max_omega|max_accel|max_alpha|wheel_speed)$/],
  ["Base references and bands", /^(w_base|band_|k_band|w_slack|k_base_pose|k_heading|base_yaw_radius)/],
  ["EE tracking", /^(k_pos|k_rot|max_err|w_position|w_orientation|lm_|damping|w_arm|posture|k_posture|tie_|w_manip)/],
  ["Arm limits and yielding", /^(joint_|limit_gain|reach|xi_|w_min|w_influence|resync|lead_)/],
  ["Timeouts and inputs", /./]];
let cfgCur = {};
function cfgRender(r){
  cfgCur = r.config;
  $("#hard").textContent = Object.keys(r.hard || {}).length ? Object.entries(r.hard).map(([k, v]) => `${k} ${v}`).join(", ") : "(read at enable)";
  const keys = Object.keys(r.config), used = new Set(), parts = [];
  for (const [name, re] of GROUPS){
    const ks = keys.filter(k => !used.has(k) && re.test(k)); ks.forEach(k => used.add(k));
    if (!ks.length) continue;
    parts.push(`<h4>${name}</h4>`);
    for (const k of ks){ const v = r.config[k], d = r.defaults[k];
      const val = Array.isArray(v) ? v.join(",") : (v === null ? "" : v);
      const st = r.structural.includes(k) ? " (disabled only)" : "";
      parts.push(`<label title="${k}${st}; default ${d === null ? "–" : d}"><span>${k}${st ? " *" : ""}</span><input data-k="${k}" value="${val}"></label>`); }
  }
  $("#cfg").innerHTML = parts.join("");
  $("#cfg").querySelectorAll("input").forEach(i => i.oninput = () => i.classList.toggle("dirty", i.value !== String(Array.isArray(cfgCur[i.dataset.k]) ? cfgCur[i.dataset.k].join(",") : (cfgCur[i.dataset.k] ?? ""))));
}
const parseVal = (k, txt) => { const cur = cfgCur[k];
  if (txt.trim() === "") return null;
  if (Array.isArray(cur) || txt.includes(",")) return txt.split(",").map(Number);
  if (typeof cur === "boolean") return txt === "true";
  const n = Number(txt); return isFinite(n) ? n : txt; };
$("#apply").onclick = async () => {
  const ch = {}; $("#cfg").querySelectorAll("input.dirty").forEach(i => ch[i.dataset.k] = parseVal(i.dataset.k, i.value));
  if (!Object.keys(ch).length){ $("#cfgmsg").textContent = "nothing changed"; return; }
  try { err(""); cfgRender(await RU.api("config", {method: "POST", body: ch})); $("#cfgmsg").textContent = "applied: " + Object.keys(ch).join(", "); }
  catch(e){ err(e.message); }
};
$("#revert").onclick = () => RU.api("config").then(cfgRender);
RU.api("config").then(cfgRender).catch(e => err(e.message));
// the end effectors from the stream's columns (<ee>_cmd_x): one ("arm") or one per arm (arm_left, arm_right)
let EES = ["arm"], plots = null, last = null;
const errSeries = () => EES.length > 1 ? EES.flatMap(e => [{name: e + " QP"}, {name: e + " actual"}, {name: e + " lead"}])
                                       : [{name: "QP shortfall"}, {name: "actual"}, {name: "lead"}];
const makePlots = () => ({
  e: new RU.Plot($("#pe"), {seconds: 20, series: errSeries()}),
  r: new RU.Plot($("#pr"), {seconds: 20, series: errSeries()}),
  b: new RU.Plot($("#pb"), {seconds: 20, series: [{name: "vx"}, {name: "vy"}, {name: "w"}]}),
  s: new RU.Plot($("#ps"), {seconds: 20, series: [{name: "solve ms"}]}),
});
RU.rows("stream?batch_ms=100", d => {
  if (!plots){ const e = d.fields.filter(f => f.endsWith("_cmd_x")).map(f => f.slice(0, -6)); if (e.length) EES = e; plots = makePlots(); }
  for (let i = 0; i < d.n; i++){
    const p = {}; for (const f of d.fields) p[f] = d.col(f)[i];
    plots.e.push(p.t_tick, EES.flatMap(e => [p[e + "_err_pos"], p[e + "_err_meas_pos"], p[e + "_lead_pos"]]));
    plots.r.push(p.t_tick, EES.flatMap(e => [p[e + "_err_rot"], p[e + "_err_meas_rot"], p[e + "_lead_rot"]]));
    plots.b.push(p.t_tick, [p.base_vx, p.base_vy, p.base_w]);
    plots.s.push(p.t_tick, [p.solve_ms]);
    last = p;
  }
  if (!last) return;
  const f = v => v === undefined || !isFinite(v) ? "–" : v.toFixed(4);
  $("#ee").innerHTML = EES.map(e => (EES.length > 1 ? `<tr><th colspan="4">${e}</th></tr>` : "") +
    `<tr><th></th><th>commanded</th><th>measured</th><th>target</th></tr>` +
    ["x", "y", "z", "qx", "qy", "qz", "qw"].map(k => `<tr><td>${k}</td><td>${f(last[e + "_cmd_" + k])}</td><td>${f(last[e + "_meas_" + k])}</td><td>${f(last[e + "_tgt_" + k])}</td></tr>`).join("")).join("");
});
// per joint |kp (q_cmd - q_meas)| against its budget lead_torque (Nm); "bound" while the lead bound binds
const leadText = (a, L, e) => { const ld = a && a.lead; if (!ld) return "–";
  const t = [1, 2, 3, 4, 5, 6].map(i => L[(e || "arm") + "_lead_tau" + i]);
  if (t.some(v => v === null || v === undefined)) return `budget ${ld.lead_torque.join(" / ")} Nm`;
  return t.map((v, i) => `${Math.abs(v).toFixed(1)}/${ld.lead_torque[i]}`).join("  ") + ` Nm${ld.binding ? " (bound)" : ""}; binds ${ld.binds}`; };
RU.sse("events", s => {
  lease.update(s);
  es.update({estop: s.estop && s.estop.latched ? s.estop : null});
  band.update(s, {live: ["tracking", "holding", "idle"].includes(s.state), liveText: s.state});
  $("#status").textContent = s.state; $("#status").className = "ru-pill " + ({tracking: "ru-ok", idle: "ru-ok", holding: "ru-warn", fault: "ru-err"}[s.state] || "");
  const L = s.latest || {}, g = (v, n) => v === null || v === undefined ? "–" : Number(v).toFixed(n);
  const kv = [
    ["enabled", s.enabled ? `yes (${s.holder})` : `no${s.disabled_reason ? " — " + s.disabled_reason : ""}`],
    ["armed", s.armed ? `yes (arm_seq ${s.latch.arm_seq})` : `no${s.disarm_reason ? " — " + s.disarm_reason : ""}`],
    ["fault", s.fault || "–"],
    ["holding", s.hold_reason || "–"],
    ["base mode", s.hints.base.mode],
    ["limited by", s.limited_by],
    ["solve", `${g(L.solve_ms, 3)} ms, ${g(L.solves, 0)} QP(s), status ${L.status}`],
    ...EES.flatMap(e => { const x = k => L[e + "_" + k], arm = (s.arms && s.arms[e]) || (e === EES[0] ? s.arm : null), pre = EES.length > 1 ? e + " " : "";
      return [[pre + "EE error", `QP shortfall ${g(x("err_pos"), 4)} m, ${g(x("err_rot"), 3)} rad; actual ${g(x("err_meas_pos"), 4)} m, ${g(x("err_meas_rot"), 3)} rad; lead ${g(x("lead_pos"), 4)} m, ${g(x("lead_rot"), 3)} rad`],
              [pre + "joint lead", leadText(arm, L, e)]]; }),
    ["base", `${g(L.base_x, 3)}, ${g(L.base_y, 3)} m, ${g(L.base_yaw, 3)} rad (reset ${s.estimator.reset_id})`],
    ["estimator", `${s.estimator.health}, age ${g(s.estimator.age, 3)} s`],
    ...Object.entries(s.arms || {arm: s.arm}).map(([e, a]) => [e, `${a.state || "–"}, age ${g(a.age, 3)} s, resyncs ${a.resyncs}`]),
    ["session", `${s.session.session || "–"} udp ${s.session.udp_port}, accepted ${s.session.accepted}, input age ${g(s.session.input_age, 3)} s`],
    ["dropped", JSON.stringify(s.session.dropped)],
    ["lease", s.lease.held ? `${s.lease.holder} (${g(s.lease.expires_in, 2)} s)` : "free"],
    ["loop", `${g(s.loop_hz, 1)} Hz; tick ${g(s.tick.period_ms, 1)} ms (max ${g(s.tick.max_period_ms, 1)}), compute ${g(s.tick.compute_ms, 2)} ms (max ${g(s.tick.max_compute_ms, 2)}); overruns ${s.tick.overruns}, late ${s.tick.late}`],
  ];
  $("#kv").innerHTML = kv.map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join("");
  $("#events").innerHTML = s.events.slice().reverse().map(e => `<div class="${e.level === "error" ? "ru-ev-error" : e.level === "warn" ? "ru-ev-warn" : ""}">${new Date(e.wall * 1000).toLocaleTimeString()} ${e.msg}</div>`).join("");
});
function draw(){ Object.values(plots || {}).forEach(p => p.draw()); requestAnimationFrame(draw); }
requestAnimationFrame(draw);
</script></body></html>
"""
