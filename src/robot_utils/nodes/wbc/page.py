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
    <b>Tracking error</b> <span class="ru-muted">position m · orientation rad (×0.1), 20 s</span><canvas class="ru-plot" id="pe"></canvas>
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
const plots = {
  e: new RU.Plot($("#pe"), {seconds: 20, series: [{name: "pos"}, {name: "rot×0.1"}]}),
  b: new RU.Plot($("#pb"), {seconds: 20, series: [{name: "vx"}, {name: "vy"}, {name: "w"}]}),
  s: new RU.Plot($("#ps"), {seconds: 20, series: [{name: "solve ms"}]}),
};
let last = null;
RU.rows("stream?batch_ms=100", d => {
  for (let i = 0; i < d.n; i++){
    const p = {}; for (const f of d.fields) p[f] = d.col(f)[i];
    plots.e.push(p.t_tick, [p.arm_err_pos, 0.1 * p.arm_err_rot]);
    plots.b.push(p.t_tick, [p.base_vx, p.base_vy, p.base_w]);
    plots.s.push(p.t_tick, [p.solve_ms]);
    last = p;
  }
  if (!last) return;
  const f = v => v === undefined || !isFinite(v) ? "–" : v.toFixed(4);
  const rows = ["x", "y", "z", "qx", "qy", "qz", "qw"].map(k => `<tr><td>${k}</td><td>${f(last["arm_cmd_" + k])}</td><td>${f(last["arm_meas_" + k])}</td><td>${f(last["arm_tgt_" + k])}</td></tr>`);
  $("#ee").innerHTML = `<tr><th></th><th>commanded</th><th>measured</th><th>target</th></tr>` + rows.join("");
});
RU.sse("events", s => {
  lease.update(s);
  es.update({estop: s.estop && s.estop.latched ? s.estop : null});
  band.update(s, {live: ["tracking", "holding", "idle"].includes(s.state), liveText: s.state});
  $("#status").textContent = s.state; $("#status").className = "ru-pill " + ({tracking: "ru-ok", idle: "ru-ok", holding: "ru-warn", fault: "ru-err"}[s.state] || "");
  const L = s.latest || {}, g = (v, n) => v === null || v === undefined ? "–" : Number(v).toFixed(n);
  const kv = [
    ["enabled", s.enabled ? `yes (${s.holder})` : `no${s.disabled_reason ? " — " + s.disabled_reason : ""}`],
    ["fault", s.fault || "–"],
    ["holding", s.hold_reason || "–"],
    ["base mode", s.hints.base.mode],
    ["limited by", s.limited_by],
    ["solve", `${g(L.solve_ms, 3)} ms, ${g(L.solves, 0)} QP(s), status ${L.status}`],
    ["EE error", `${g(L.arm_err_pos, 4)} m, ${g(L.arm_err_rot, 4)} rad`],
    ["base", `${g(L.base_x, 3)}, ${g(L.base_y, 3)} m, ${g(L.base_yaw, 3)} rad (reset ${s.estimator.reset_id})`],
    ["estimator", `${s.estimator.health}, age ${g(s.estimator.age, 3)} s`],
    ["arm", `${s.arm.state || "–"}, age ${g(s.arm.age, 3)} s, resyncs ${s.arm.resyncs}`],
    ["session", `${s.session.session || "–"} udp ${s.session.udp_port}, accepted ${s.session.accepted}, input age ${g(s.session.input_age, 3)} s`],
    ["dropped", JSON.stringify(s.session.dropped)],
    ["lease", s.lease.held ? `${s.lease.holder} (${g(s.lease.expires_in, 2)} s)` : "free"],
    ["loop", `${g(s.loop_hz, 1)} Hz`],
  ];
  $("#kv").innerHTML = kv.map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join("");
  $("#events").innerHTML = s.events.slice().reverse().map(e => `<div class="${e.level === "error" ? "ru-ev-error" : e.level === "warn" ? "ru-ev-warn" : ""}">${new Date(e.wall * 1000).toLocaleTimeString()} ${e.msg}</div>`).join("");
});
function draw(){ Object.values(plots).forEach(p => p.draw()); requestAnimationFrame(draw); }
requestAnimationFrame(draw);
</script></body></html>
"""
