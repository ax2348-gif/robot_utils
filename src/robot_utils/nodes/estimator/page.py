"""The estimator node's page (served at the node root). Relative URLs only.

The trajectory is accumulated here, in the browser, from the pose stream: the node keeps no
history. Reloading the page starts a new trail.
"""

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>State estimator</title>
<link rel="stylesheet" href="common/ui.css">
<style>
.ru-wrap{max-width:1300px}
.layout{display:grid;grid-template-columns:minmax(0,1fr) 330px;gap:12px}
@media (max-width:900px){.layout{grid-template-columns:1fr}}
#map{width:100%;height:560px;display:block;border-radius:6px;background:var(--bg);cursor:grab;touch-action:none}
#map.drag{cursor:grabbing}
.ctl{display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin-top:8px}
.kv{display:grid;grid-template-columns:auto 1fr;gap:2px 10px;font-variant-numeric:tabular-nums}
.kv span:nth-child(odd){color:var(--muted)}
canvas.ru-plot{width:100%;height:110px}
#events{max-height:150px;overflow:auto;font-size:12px}
.sw{display:inline-block;width:18px;height:3px;vertical-align:middle;margin-right:4px}
</style></head><body><div class="ru-wrap">
<div class="ru-row" style="justify-content:space-between">
  <h2 style="margin:6px 0">State estimator <span id="state" class="ru-pill">…</span></h2>
  <div class="ru-row"><span id="mode" class="ru-muted"></span><a class="ru-muted" href="docs">API docs</a></div>
</div>
<div id="err" class="ru-err"></div>
<div class="layout">
  <div class="ru-card">
    <b>Trajectory</b> <span class="ru-muted">(kept in this page only; grid 1 m)</span>
    <span class="ru-muted" style="float:right"><span class="sw" style="background:#2563eb"></span>fused
      <span class="sw" style="background:#9ca3af"></span>odometry only</span>
    <canvas id="map"></canvas>
    <div class="ctl">
      <label><input type="checkbox" id="follow" checked> follow robot</label>
      <label><input type="checkbox" id="headup"> heading up</label>
      <label><input type="checkbox" id="showodom" checked> odometry-only trail</label>
      <button class="ru" id="center" title="Centre the view on the robot">Recentre</button>
      <button class="ru" id="fit" title="Fit the whole trail">Fit trail</button>
      <button class="ru" id="origin" title="Centre on the world origin">Origin</button>
      <button class="ru" id="zin">+</button><button class="ru" id="zout">−</button>
      <button class="ru" id="clear" title="Forget the trail (the pose is unchanged)">Clear trail</button>
      <button class="ru" id="reset" title="Set the pose to the origin and clear the trail">Reset pose</button>
      <span class="ru-muted" id="scale"></span>
    </div>
    <div class="ru-muted">Drag to pan (turns off follow), wheel to zoom about the cursor, double-click to recentre.</div>
  </div>
  <div>
    <div class="ru-card"><b>Pose</b><div class="kv" id="pose"></div></div>
    <div class="ru-card"><b>Inputs</b><div class="kv" id="inputs"></div></div>
    <div class="ru-card"><b>Heading</b> <span class="ru-muted">deg, 20 s</span><canvas class="ru-plot" id="ph"></canvas>
      <b>Yaw rate</b> <span class="ru-muted">rad/s: IMU vs wheels (difference = slip)</span><canvas class="ru-plot" id="pw"></canvas>
      <b>Speed</b> <span class="ru-muted">m/s</span><canvas class="ru-plot" id="pv"></canvas></div>
  </div>
</div>
<div class="ru-card"><b>Events</b><div id="events"></div></div>
</div>
<script src="common/ui.js"></script>
<script>
const $ = s => document.querySelector(s), D = 180 / Math.PI;
const err = m => $("#err").textContent = m || "";
const ts = new RU.TimeSync(); ts.sync().then(() => setInterval(() => ts.sync(), 30000)).catch(() => {});
let FOOT = {half_length: 0.3, half_width: 0.3};
RU.api("meta").then(m => { if (m.footprint) FOOT = m.footprint; }).catch(() => {});

// ---------------- client-side trajectory -----------------------------------------------------
// trail points [x, y, xo, yo]; null = break (pose reset / gap). Decimated: a point is kept when the
// robot moved >= 5 mm or turned, or 0.25 s passed; capped (oldest dropped).
const MAX_PTS = 50000;
let trail = [], last = null, lastKept = null;
function keep(p){
  const k = lastKept;
  if (k && Math.hypot(p.x - k.x, p.y - k.y) < 0.005 && Math.abs(p.theta - k.theta) < 0.01 && p.t - k.t < 0.25) return;
  trail.push([p.x, p.y, p.x_odom, p.y_odom]); lastKept = p;
  if (trail.length > MAX_PTS) trail.splice(0, trail.length - MAX_PTS);
}
const plots = {
  h: new RU.Plot($("#ph"), {seconds: 20, series: [{name: "fused", color: "#2563eb"}, {name: "odometry", color: "#9ca3af"}]}),
  w: new RU.Plot($("#pw"), {seconds: 20, series: [{name: "IMU", color: "#2563eb"}, {name: "wheels", color: "#f59e0b"}]}),
  v: new RU.Plot($("#pv"), {seconds: 20, series: [{name: "speed", color: "#16a34a"}]}),
};
RU.rows("stream", d => {
  const c = n => d.col(n), F = d.fields;
  for (let i = 0; i < d.n; i++){
    const p = {}; for (const f of F) p[f] = c(f)[i];
    if (last && (Math.hypot(p.x - last.x, p.y - last.y) > 0.5 || p.t - last.t > 1.0)) { trail.push(null); lastKept = null; }  // reset or gap
    keep(p);
    plots.h.push(p.t, [p.theta * D, p.theta_odom * D]);
    plots.w.push(p.t, [p.omega_imu, p.omega_wheel]);
    plots.v.push(p.t, [p.speed]);
    last = p;
  }
});

// ---------------- view: world (x fwd, y left) -> screen; pan / zoom / follow / heading-up --------
const cv = $("#map");
let view = {cx: 0, cy: 0, scale: 120};          // centre in world m, px per m
function rot(){ return $("#headup").checked && last ? Math.PI / 2 - last.theta : 0; }  // heading up: robot +x points up
function w2s(x, y){ const W = cv.clientWidth, H = cv.clientHeight, a = rot(), dx = x - view.cx, dy = y - view.cy;
  const rx = dx * Math.cos(a) - dy * Math.sin(a), ry = dx * Math.sin(a) + dy * Math.cos(a);
  return [W / 2 - ry * view.scale, H / 2 - rx * view.scale]; }        // default: world +x up, +y left
function s2w(sx, sy){ const W = cv.clientWidth, H = cv.clientHeight, a = -rot();
  const ry = (W / 2 - sx) / view.scale, rx = (H / 2 - sy) / view.scale;
  return [view.cx + rx * Math.cos(a) - ry * Math.sin(a), view.cy + rx * Math.sin(a) + ry * Math.cos(a)]; }
function recentre(){ if (last){ view.cx = last.x; view.cy = last.y; } }
function fit(){
  let a = Infinity, b = -Infinity, c = Infinity, e = -Infinity;
  for (const p of trail) if (p){ a = Math.min(a, p[0]); b = Math.max(b, p[0]); c = Math.min(c, p[1]); e = Math.max(e, p[1]);
    if ($("#showodom").checked){ a = Math.min(a, p[2]); b = Math.max(b, p[2]); c = Math.min(c, p[3]); e = Math.max(e, p[3]); } }
  if (!isFinite(a)) return; $("#follow").checked = false; $("#headup").checked = false;
  view.cx = (a + b) / 2; view.cy = (c + e) / 2;
  view.scale = Math.max(5, Math.min(2000, 0.85 * Math.min(cv.clientHeight / Math.max(b - a, 0.5), cv.clientWidth / Math.max(e - c, 0.5))));
}
function zoomAt(f, sx, sy){ const before = s2w(sx, sy); view.scale = Math.max(5, Math.min(3000, view.scale * f));
  const after = s2w(sx, sy); view.cx += before[0] - after[0]; view.cy += before[1] - after[1]; }
let drag = null;
cv.addEventListener("pointerdown", e => { drag = {x: e.clientX, y: e.clientY, cx: view.cx, cy: view.cy}; cv.setPointerCapture(e.pointerId); cv.classList.add("drag"); });
cv.addEventListener("pointermove", e => { if (!drag) return; const r = cv.getBoundingClientRect();
  if (Math.hypot(e.clientX - drag.x, e.clientY - drag.y) > 3) $("#follow").checked = false;
  view.cx = drag.cx; view.cy = drag.cy; const a = s2w(drag.x - r.left, drag.y - r.top), b = s2w(e.clientX - r.left, e.clientY - r.top);
  view.cx += a[0] - b[0]; view.cy += a[1] - b[1]; });
cv.addEventListener("pointerup", () => { drag = null; cv.classList.remove("drag"); });
cv.addEventListener("wheel", e => { e.preventDefault(); const r = cv.getBoundingClientRect();
  zoomAt(Math.exp(-e.deltaY * 0.0015), e.clientX - r.left, e.clientY - r.top); }, {passive: false});
cv.addEventListener("dblclick", () => { $("#follow").checked = true; recentre(); });
$("#center").onclick = () => { $("#follow").checked = true; recentre(); };
$("#fit").onclick = fit;
$("#origin").onclick = () => { $("#follow").checked = false; view.cx = 0; view.cy = 0; };
$("#zin").onclick = () => zoomAt(1.5, cv.clientWidth / 2, cv.clientHeight / 2);
$("#zout").onclick = () => zoomAt(1 / 1.5, cv.clientWidth / 2, cv.clientHeight / 2);
$("#clear").onclick = () => { trail = []; lastKept = null; };
$("#reset").onclick = async () => { try { err(""); await RU.api("reset", {body: {}}); trail = []; lastKept = null; last = null; view.cx = view.cy = 0; }
  catch(e){ err(e.message); } };

function draw(){
  const dpr = window.devicePixelRatio || 1, W = cv.clientWidth, H = cv.clientHeight;
  if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)){ cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
  const g = cv.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
  if ($("#follow").checked && last){ view.cx = last.x; view.cy = last.y; }
  const css = n => getComputedStyle(document.body).getPropertyValue(n).trim();
  // grid (1 m, 10 m bold) over the visible world box
  const corners = [s2w(0, 0), s2w(W, 0), s2w(0, H), s2w(W, H)];
  const xs = corners.map(c => c[0]), ys = corners.map(c => c[1]);
  const step = view.scale < 12 ? 10 : 1;
  g.lineWidth = 1;
  for (const [axis, lo, hi] of [[0, Math.min(...xs), Math.max(...xs)], [1, Math.min(...ys), Math.max(...ys)]]){
    for (let v = Math.floor(lo / step) * step; v <= hi; v += step){
      const a = axis ? w2s(Math.min(...xs), v) : w2s(v, Math.min(...ys)), b = axis ? w2s(Math.max(...xs), v) : w2s(v, Math.max(...ys));
      g.strokeStyle = Math.abs(v) < 1e-9 ? css("--muted") : (Math.round(v) % 10 === 0 ? css("--muted") + "88" : css("--line"));
      g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke(); } }
  // origin axes (x red, y green)
  const o = w2s(0, 0), ox = w2s(0.5, 0), oy = w2s(0, 0.5);
  g.lineWidth = 2; g.strokeStyle = "#dc2626"; g.beginPath(); g.moveTo(o[0], o[1]); g.lineTo(ox[0], ox[1]); g.stroke();
  g.strokeStyle = "#16a34a"; g.beginPath(); g.moveTo(o[0], o[1]); g.lineTo(oy[0], oy[1]); g.stroke();
  // trails
  const path = (ix, iy, col, w) => { g.strokeStyle = col; g.lineWidth = w; g.beginPath(); let pen = false;
    for (const p of trail){ if (!p){ pen = false; continue; } const s = w2s(p[ix], p[iy]); if (pen) g.lineTo(s[0], s[1]); else { g.moveTo(s[0], s[1]); pen = true; } } g.stroke(); };
  if ($("#showodom").checked){ g.setLineDash([5, 4]); path(2, 3, "#9ca3af", 1.5); g.setLineDash([]); }
  path(0, 1, "#2563eb", 2);
  // robot footprint + heading arrow
  if (last){
    const c = Math.cos(last.theta), s = Math.sin(last.theta), L = FOOT.half_length, Wd = FOOT.half_width;
    const P = (bx, by) => w2s(last.x + c * bx - s * by, last.y + s * bx + c * by);
    const pts = [P(L, Wd), P(L, -Wd), P(-L, -Wd), P(-L, Wd)];
    g.fillStyle = "#2563eb33"; g.strokeStyle = "#2563eb"; g.lineWidth = 2; g.beginPath(); pts.forEach((p, i) => i ? g.lineTo(p[0], p[1]) : g.moveTo(p[0], p[1])); g.closePath(); g.fill(); g.stroke();
    const tip = P(L + 0.15, 0), base = P(0, 0); g.beginPath(); g.moveTo(base[0], base[1]); g.lineTo(tip[0], tip[1]); g.stroke();
    const h1 = P(L + 0.05, 0.06), h2 = P(L + 0.05, -0.06); g.beginPath(); g.moveTo(h1[0], h1[1]); g.lineTo(tip[0], tip[1]); g.lineTo(h2[0], h2[1]); g.stroke();
  }
  $("#scale").textContent = `${view.scale.toFixed(0)} px/m · ${trail.filter(Boolean).length} pts`;
  for (const p of Object.values(plots)) p.draw();
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);

// ---------------- state ----------------------------------------------------------------------
const f = (v, k) => v === null || v === undefined || !isFinite(v) ? "–" : (+v).toFixed(k);
const kv = (el, rows) => el.innerHTML = rows.map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join("");
RU.sse("events", st => {
  $("#state").textContent = st.state + (st.rate_hz ? " · " + st.rate_hz + " Hz" : "");
  $("#state").className = "ru-pill " + (st.state === "running" ? "ru-ok" : "ru-warn");
  $("#mode").textContent = st.mode; err(st.error);
  const p = st.pose || {}, lag = p.t && ts.offset !== null ? (ts.serverNow() - p.t) * 1000 : null;
  kv($("#pose"), [["x, y", `${f(p.x, 3)}, ${f(p.y, 3)} m`], ["heading", `${f(p.theta * D, 2)}° (wrapped ${f(((p.theta * D + 540) % 360) - 180, 1)}°)`],
    ["velocity", `${f(p.vx, 3)}, ${f(p.vy, 3)} m/s · ${f(p.omega, 3)} rad/s`], ["distance", f(p.distance, 3) + " m"],
    ["odometry only", `${f(p.x_odom, 3)}, ${f(p.y_odom, 3)} m, ${f(p.theta_odom * D, 2)}°`],
    ["IMU used", st.imu_coverage === null ? "–" : (st.imu_coverage * 100).toFixed(1) + " % of ticks"],
    ["IMU lead", f(p.imu_lead * 1000, 1) + " ms"], ["IMU still", st.imu_still ? "yes" : "no"],
    ["pose age", lag === null ? "–" : lag.toFixed(0) + " ms"]]);
  kv($("#inputs"), Object.entries(st.inputs).flatMap(([k, u]) => [[k, `<span class="${u.connected ? "ru-ok" : "ru-err"}">${u.connected ? "connected" : "down"}</span> ${u.rate_hz ? u.rate_hz + " Hz" : ""}`],
    ["", `<span class="ru-muted">${u.url}${u.error ? " · " + u.error : ""}${u.clock_uncertainty !== null ? " · clock ±" + (u.clock_uncertainty * 1000).toFixed(2) + " ms" : ""}</span>`]]));
  $("#events").innerHTML = st.events.slice().reverse().map(e => `<div class="${e.level !== "info" ? "ru-err" : ""}">${new Date(e.wall * 1000).toLocaleTimeString()} ${e.msg}</div>`).join("");
});
</script></body></html>
"""
