"""The pose servo node's page (served at the node root). Relative URLs only."""

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pose servo</title>
<link rel="stylesheet" href="common/ui.css">
<style>
.ru-wrap{max-width:1300px}
.layout{display:grid;grid-template-columns:minmax(0,1fr) 360px;gap:12px}
@media (max-width:900px){.layout{grid-template-columns:1fr}}
#map{width:100%;height:520px;display:block;border-radius:6px;background:var(--bg);cursor:crosshair;touch-action:none}
.kv{display:grid;grid-template-columns:auto 1fr;gap:2px 10px;font-variant-numeric:tabular-nums}
.kv span:nth-child(odd){color:var(--muted)}
.form{display:grid;grid-template-columns:auto 1fr auto 1fr;gap:4px 8px;align-items:center}
.form input,.form select{width:100%;box-sizing:border-box}
canvas.ru-plot{width:100%;height:100px}
#events{max-height:160px;overflow:auto;font-size:12px}
.ctl{display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin:6px 0}
</style><script src="../../themes/theme.js"></script>
</head><body data-page="pose_servo"><div id="band"></div><div class="ru-wrap">
<div class="ru-row ru-head">
  <h2 style="margin:6px 0">Pose servo <span id="status" class="ru-pill">…</span></h2>
  <div class="ru-row"><span id="estop"></span><a class="ru-muted" href="docs">API docs</a></div>
</div>
<div id="err" class="ru-err"></div>
<div class="layout ru-body">
  <div class="ru-card" data-panel="map" data-role="view">
    <b>Map</b> <span class="ru-muted">(estimator world frame, grid 0.5 m; click to place a global goal; trail kept in this page)</span>
    <canvas id="map"></canvas>
    <div class="ctl">
      <button class="ru" id="center" title="Back to the default scale (the view always follows the robot)">Reset zoom</button><button class="ru" id="zin">+</button><button class="ru" id="zout">−</button>
      <button class="ru" id="clear">Clear trail</button>
      <span class="ru-muted">the view follows the robot; robot in blue, goal with its acceptance circle</span>
    </div>
    <b>Twist sent</b> <span class="ru-muted">vx, vy m/s · omega rad/s, 20 s</span><canvas class="ru-plot" id="pv"></canvas>
    <b>Twist rate</b> <span class="ru-muted">|a| m/s² · alpha rad/s²</span><canvas class="ru-plot" id="pa"></canvas>
    <b>Error</b> <span class="ru-muted">distance m · heading rad</span><canvas class="ru-plot" id="pe"></canvas>
  </div>
  <div class="ru-side">
    <div class="ru-card" data-panel="control" data-role="control"><b>Control</b>
      <div class="ctl"><input id="client" size="10" placeholder="your name">
        <button class="ru live" id="enable">Enable</button><button class="ru" id="disable">Disable</button>
        <button class="ru" id="stop">Stop (brake)</button></div>
      <div class="ru-muted">Enable takes the swerve lease (with the caps below); Disable stops at once and releases it.</div>
    </div>
    <div class="ru-card" data-panel="goal" data-role="control"><b>Goal</b>
      <div class="form">
        <span>frame</span><select id="g_frame"><option>local</option><option>global</option></select>
        <span>mode</span><select id="g_mode"><option value="">(config)</option><option>synchronized</option><option>independent</option></select>
        <span>x m</span><input id="g_x" type="number" step="0.05" value="0">
        <span>y m</span><input id="g_y" type="number" step="0.05" value="0">
        <span>θ deg</span><input id="g_th" type="number" step="5" value="0">
        <span>on reach</span><select id="g_reach"><option value="">(config)</option><option>terminate</option><option>hold</option></select>
        <span>speed</span><input id="g_speed" type="number" step="0.05" placeholder="cap">
        <span>accel</span><input id="g_accel" type="number" step="0.05" placeholder="cap">
      </div>
      <div class="ctl"><button class="ru primary" id="go">Go</button><span class="ru-muted" id="goalinfo"></span></div>
    </div>
    <div class="ru-card" data-panel="config" data-role="config"><b>Config</b> <span class="ru-muted">(per enable; changeable when idle)</span>
      <div class="form" id="cfg"></div>
      <div class="ctl"><button class="ru" id="apply">Apply</button><span class="ru-muted" id="hard"></span></div>
    </div>
    <div class="ru-card" data-panel="state" data-role="telemetry"><b>State</b><div class="kv" id="kv"></div></div>
  </div>
</div>
<div class="ru-card" data-panel="events" data-role="log"><b>Events</b><div id="events"></div></div>
</div>
<script src="common/ui.js"></script>
<script>
const $ = s => document.querySelector(s), D = 180 / Math.PI;
const err = m => $("#err").textContent = m || "";
const call = async (p, body) => { try { err(""); return await RU.api(p, body === undefined ? {method: "POST"} : {body}); } catch(e){ err(e.message); } };
$("#client").value = localStorage.getItem("ru-client") || "";
const es = RU.estopButton($("#estop"));
const band = RU.stateBand($("#band"));
$("#enable").onclick = () => { localStorage.setItem("ru-client", $("#client").value); call("enable", {client: $("#client").value || "browser", config: cfgValues()}); };
$("#disable").onclick = () => call("disable");
$("#stop").onclick = () => call("stop");
const num = id => { const v = $(id).value; return v === "" ? null : Number(v); };
$("#go").onclick = async () => {
  const b = {frame: $("#g_frame").value, x: num("#g_x") || 0, y: num("#g_y") || 0, theta: (num("#g_th") || 0) / D};
  if ($("#g_mode").value) b.mode = $("#g_mode").value;
  if ($("#g_reach").value) b.on_reach = $("#g_reach").value;
  if (num("#g_speed") !== null) b.speed = num("#g_speed");
  if (num("#g_accel") !== null) b.accel = num("#g_accel");
  const r = await call("goal", b);
  if (r) $("#goalinfo").textContent = `goal ${r.goal_id}: ${r.distance.toFixed(3)} m, ${(r.rotation * D).toFixed(1)}°`;
};
// config form
const CFG = [["max_speed","m/s"],["max_omega","rad/s"],["max_accel","m/s²"],["max_alpha","rad/s²"],["max_jerk","m/s³"],
             ["max_ang_jerk","rad/s³"],["max_distance","m"],["max_rotation","rad"],["pos_tol","m"],["heading_tol","rad"],["hold_band","×"],["min_wheel_speed","m/s"],["stall_s","s"]];
$("#cfg").innerHTML = CFG.map(([k,u]) => `<span title="${u}">${k}</span><input id="c_${k}" type="number" step="any">`).join("") +
  `<span>mode</span><select id="c_mode"><option>synchronized</option><option>independent</option></select>` +
  `<span>on_reach</span><select id="c_on_reach"><option>terminate</option><option>hold</option></select>` +
  `<span>require_imu</span><select id="c_require_imu"><option value="true">true</option><option value="false">false</option></select>`;
let cfgDirty = false; $("#cfg").addEventListener("input", () => cfgDirty = true);
function cfgValues(){ const o = {}; for (const [k] of CFG){ const v = num("#c_" + k); if (v !== null) o[k] = v; }
  o.mode = $("#c_mode").value; o.on_reach = $("#c_on_reach").value; o.require_imu = $("#c_require_imu").value === "true"; return o; }
function cfgShow(c){ if (cfgDirty) return; for (const [k] of CFG) $("#c_" + k).value = c[k];
  $("#c_mode").value = c.mode; $("#c_on_reach").value = c.on_reach; $("#c_require_imu").value = String(c.require_imu); }
$("#apply").onclick = async () => { const r = await call("config", cfgValues()); if (r){ cfgDirty = false; cfgShow(r.config); } };
RU.api("config").then(r => { cfgShow(r.config); const h = r.hard;
  $("#hard").textContent = `hard: wheels ≤ ${h.wheel_speed} m/s, steering ≤ ${h.steer_rate.toFixed(2)} rad/s (wheels ≥ ${h.steer_min_speed} m/s)`; });

// stream: trail, plots
const plots = {
  v: new RU.Plot($("#pv"), {seconds: 20, series: [{name: "vx"}, {name: "vy"}, {name: "omega"}]}),
  a: new RU.Plot($("#pa"), {seconds: 20, series: [{name: "|a|"}, {name: "alpha"}]}),
  e: new RU.Plot($("#pe"), {seconds: 20, series: [{name: "dist"}, {name: "heading"}]}),
};
let last = null, trail = [], goal = null;
RU.rows("stream?batch_ms=100", d => {
  for (let i = 0; i < d.n; i++){
    const p = {}; for (const f of d.fields) p[f] = d.col(f)[i];
    if (isFinite(p.x)){ const k = trail[trail.length - 1]; if (!k || Math.hypot(p.x - k[0], p.y - k[1]) > 0.005) trail.push([p.x, p.y]); if (trail.length > 20000) trail.shift(); }
    plots.v.push(p.t, [p.vx, p.vy, p.omega]);
    plots.a.push(p.t, [Math.hypot(p.ax, p.ay), p.alpha]);
    plots.e.push(p.t, [p.err_pos, p.err_theta]);
    last = p;
  }
});
let st = null;
RU.sse("events", s => {
  st = s; const est = {estop: s.swerve && s.swerve.estop ? {latched: true, source: "swerve", reason: ""} : null};
  es.update(est);
  band.update({...s, estop: est.estop || {}}, {live: ["moving", "holding", "braking"].includes(s.status), liveText: s.status});
  $("#enable").disabled = !!s.chocked;
  $("#enable").title = s.chocked ? "Chocked: an admin removes the chock over SSH" : "";
  $("#status").textContent = s.status; $("#status").className = "ru-pill " + ({moving: "ru-ok", holding: "ru-ok", braking: "ru-warn", disabled: ""}[s.status] || "");
  if (!s.enabled) cfgShow(s.config);
  const g = s.goal, L = s.latest || {}, f = (v, n) => v === null || v === undefined ? "–" : Number(v).toFixed(n);
  goal = g && g.result === null ? g : null;
  const rows = [
    ["enabled", s.enabled ? `yes (${s.holder}, swerve lease ${s.swerve_lease_id})` : `no${s.disabled_reason ? " — " + s.disabled_reason : ""}`],
    ["estimator", `${s.estimator.health}, age ${f(s.estimator.age, 3)} s, reset ${s.estimator.reset_id}`],
    ["pose", `${f(L.x, 3)}, ${f(L.y, 3)} m, ${f(L.theta === null ? null : L.theta * D, 1)}°`],
    ["goal", g ? `${g.goal_id} (${g.frame}) → ${f(g.x, 3)}, ${f(g.y, 3)}, ${f(g.theta * D, 1)}°` : "–"],
    ["result", g ? (g.result === null ? "in progress" : g.result + (g.reason ? " — " + g.reason : "")) : "–"],
    ["error", `${f(L.err_pos, 3)} m, ${f(L.err_theta === null ? null : L.err_theta * D, 1)}°`],
    ["twist", `${f(L.vx, 3)}, ${f(L.vy, 3)} m/s, ${f(L.omega, 3)} rad/s`],
    ["sync scale", `${f(L.scale_lin, 2)} lin / ${f(L.scale_rot, 2)} rot; ttg ${f(L.ttg_lin, 2)} / ${f(L.ttg_rot, 2)} s`],
    ["limiter", `kept ${f(L.kept, 2)}; wheel ${f(L.wheel_max, 3)} m/s; steer ${f(L.steer_rate, 2)} rad/s; boost ${f(L.boost, 2)} m/s`],
    ["violations", String(s.violations)],
    ["loop", `${f(s.loop_hz, 1)} Hz`],
  ];
  $("#kv").innerHTML = rows.map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join("");
  $("#events").innerHTML = s.events.slice().reverse().map(e => `<div class="${e.level === "error" ? "ru-ev-error" : e.level === "warn" ? "ru-ev-warn" : ""}">${new Date(e.wall * 1000).toLocaleTimeString()} ${e.msg}</div>`).join("");
});

// map: world x up, y left; follows the robot
const cv = $("#map"); let scale = 150;
function w2s(x, y){ const c = last && isFinite(last.x) ? last : {x: 0, y: 0}; return [cv.clientWidth / 2 - (y - c.y) * scale, cv.clientHeight / 2 - (x - c.x) * scale]; }
function s2w(sx, sy){ const c = last && isFinite(last.x) ? last : {x: 0, y: 0}; return [c.x + (cv.clientHeight / 2 - sy) / scale, c.y + (cv.clientWidth / 2 - sx) / scale]; }
cv.addEventListener("click", e => { const r = cv.getBoundingClientRect(), [x, y] = s2w(e.clientX - r.left, e.clientY - r.top);
  $("#g_frame").value = "global"; $("#g_x").value = x.toFixed(3); $("#g_y").value = y.toFixed(3); });
cv.addEventListener("wheel", e => { e.preventDefault(); scale = Math.max(20, Math.min(2000, scale * Math.exp(-e.deltaY * 0.0015))); }, {passive: false});
$("#zin").onclick = () => scale = Math.min(2000, scale * 1.5);
$("#zout").onclick = () => scale = Math.max(20, scale / 1.5);
$("#center").onclick = () => scale = 150;
$("#clear").onclick = () => trail = [];
function robot(g, x, y, th, color){
  const pts = [[0.25, 0], [-0.2, 0.18], [-0.2, -0.18]].map(([a, b]) => w2s(x + a * Math.cos(th) - b * Math.sin(th), y + a * Math.sin(th) + b * Math.cos(th)));
  g.strokeStyle = color; g.lineWidth = 2; g.beginPath(); g.moveTo(...pts[0]); g.lineTo(...pts[1]); g.lineTo(...pts[2]); g.closePath(); g.stroke();
}
function draw(){
  const dpr = window.devicePixelRatio || 1, W = cv.clientWidth, H = cv.clientHeight;
  if (cv.width !== Math.round(W * dpr)){ cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
  const g = cv.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
  const line = getComputedStyle(document.body).getPropertyValue("--line") || "#ddd";
  const [a, b] = s2w(W, H), [c, d] = s2w(0, 0); g.strokeStyle = line; g.lineWidth = 1;
  for (let x = Math.floor(a / 0.5) * 0.5; x <= c; x += 0.5){ const [s0, t0] = w2s(x, d), [s1, t1] = w2s(x, b); g.beginPath(); g.moveTo(s0, t0); g.lineTo(s1, t1); g.stroke(); }
  for (let y = Math.floor(b / 0.5) * 0.5; y <= d; y += 0.5){ const [s0, t0] = w2s(a, y), [s1, t1] = w2s(c, y); g.beginPath(); g.moveTo(s0, t0); g.lineTo(s1, t1); g.stroke(); }
  g.strokeStyle = RU.token("--muted", "#9ca3af"); g.lineWidth = 1.5; g.beginPath(); trail.forEach((p, i) => { const [x, y] = w2s(p[0], p[1]); i ? g.lineTo(x, y) : g.moveTo(x, y); }); g.stroke();
  if (goal){ robot(g, goal.x, goal.y, goal.theta, RU.token("--live", "#157A45")); const [x, y] = w2s(goal.x, goal.y);
    g.strokeStyle = RU.token("--live", "#157A45"); g.lineWidth = 1; g.beginPath(); g.arc(x, y, Math.max(2, goal.motion.pos_tol * scale), 0, 2 * Math.PI); g.stroke();
    if (last && isFinite(last.x)){ const [p, q] = w2s(last.x, last.y); g.setLineDash([4, 4]); g.beginPath(); g.moveTo(p, q); g.lineTo(x, y); g.stroke(); g.setLineDash([]); } }
  if (last && isFinite(last.x)) robot(g, last.x, last.y, last.theta, RU.token("--signal", "#1F5FBF"));
  Object.values(plots).forEach(p => p.draw());
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);
</script></body></html>
"""
