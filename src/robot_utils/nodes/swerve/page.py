"""The swerve node's browser page (served at the node root). Relative URLs only."""

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Swerve base</title>
<link rel="stylesheet" href="common/ui.css">
<style>
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:12px}
.pad{display:grid;grid-template-columns:repeat(3,56px);gap:6px;justify-content:start}
.pad button{height:44px} .pad .on{border-color:var(--acc);outline:2px solid var(--acc)}
input[type=number]{width:70px}
#events{max-height:160px;overflow:auto;font-size:12px}
</style></head><body><div class="ru-wrap">
<div class="ru-row" style="justify-content:space-between">
  <h2 style="margin:6px 0">Swerve base <span id="state" class="ru-pill">…</span></h2>
  <div id="estop"></div>
</div>
<div class="ru-card ru-row" style="justify-content:space-between">
  <div id="lease"></div>
  <div class="ru-row"><button class="ru" id="enable">Enable</button><button class="ru" id="disable">Disable</button>
    <button class="ru" id="clear">Clear e-stop</button><a class="ru-muted" href="docs">API docs</a></div>
</div>
<div id="err" class="ru-err"></div>
<div class="grid">
  <div class="ru-card">
    <b>Jog</b> <span class="ru-muted">(hold; keys W A S D, Q E rotate, space stops)</span>
    <div class="ru-row" style="margin:8px 0">speed <input type="number" id="spd" value="0.2" step="0.05" min="0" max="2"> m/s
      turn <input type="number" id="trn" value="0.5" step="0.1" min="0" max="4"> rad/s</div>
    <div class="pad">
      <button class="ru" data-j="q">⟲</button><button class="ru" data-j="w">↑</button><button class="ru" data-j="e">⟳</button>
      <button class="ru" data-j="a">←</button><button class="ru" data-j=" ">■</button><button class="ru" data-j="d">→</button>
      <span></span><button class="ru" data-j="s">↓</button><span></span>
    </div>
    <div class="ru-row" style="margin-top:10px">soft limits <input type="number" id="lv" step="0.1" min="0" max="2"> m/s
      <input type="number" id="lw" step="0.1" min="0" max="4"> rad/s <button class="ru" id="setlim">Set</button></div>
    <div class="ru-muted" id="lim"></div>
  </div>
  <div class="ru-card">
    <b>Drive velocity</b> <span class="ru-muted">m/s, last 5 s</span>
    <canvas class="ru-plot" id="plot"></canvas>
    <div class="ru-muted" id="lat"></div>
  </div>
</div>
<div class="ru-card"><table class="ru-table" id="mods"></table></div>
<div class="ru-card"><b>Events</b><div id="events"></div></div>
</div>
<script src="common/ui.js"></script>
<script>
const $ = s => document.querySelector(s);
const lease = RU.leaseWidget($("#lease"), {});
const estop = RU.estopButton($("#estop"));
const ts = new RU.TimeSync();
const MODS = ["FL", "FR", "RR", "RL"], COLORS = ["#2563eb", "#16a34a", "#dc2626", "#9333ea"];
const plot = new RU.Plot($("#plot"), {seconds: 5, series: MODS.map((m, i) => ({name: m, color: COLORS[i]})).concat([{name: "cmd vx", color: "#888"}])});
const err = m => $("#err").textContent = m || "";
const call = async (path, body) => { try { err(""); return await RU.api(path, {method: "POST", body, headers: lease.headers()}); } catch(e){ err(e.message); } };
$("#enable").onclick = () => call("enable");
$("#disable").onclick = () => call("disable");
$("#clear").onclick = () => call("estop/clear");
$("#setlim").onclick = () => call("limits", {max_speed: +$("#lv").value, max_omega: +$("#lw").value});

// jog: while a key/button is held, send the twist at 20 Hz (the node stops 0.25 s after the last one)
const held = new Set();
function twist(){ const v = +$("#spd").value, w = +$("#trn").value; let x = 0, y = 0, r = 0;
  if (held.has("w")) x += v; if (held.has("s")) x -= v; if (held.has("a")) y += v; if (held.has("d")) y -= v;
  if (held.has("q")) r += w; if (held.has("e")) r -= w; return [x, y, r]; }
setInterval(() => { if (held.size && lease.token && !held.has(" ")) call("command", {twist: twist()}); }, 50);
function press(k, on){ if (!"wasdqe ".includes(k)) return; on ? held.add(k) : held.delete(k);
  document.querySelectorAll("[data-j]").forEach(b => b.classList.toggle("on", held.has(b.dataset.j))); }
document.addEventListener("keydown", e => { if (e.target.tagName === "INPUT") return; press(e.key.toLowerCase(), true); if (e.key === " ") e.preventDefault(); });
document.addEventListener("keyup", e => press(e.key.toLowerCase(), false));
document.querySelectorAll("[data-j]").forEach(b => { const k = b.dataset.j;
  b.onpointerdown = () => press(k, true); b.onpointerup = b.onpointerleave = () => press(k, false); });
window.addEventListener("blur", () => { held.clear(); press("x", false); });

// state (SSE)
RU.sse("events", st => {
  lease.update(st); estop.update(st);
  $("#state").textContent = st.state + (st.enabled ? " · enabled" : "") + (st.loop_hz ? " · " + st.loop_hz + " Hz" : "");
  $("#state").className = "ru-pill " + (st.state === "running" ? "ru-ok" : "ru-err");
  if (st.error) err(st.error);
  const l = st.limits; $("#lim").textContent = `hard ${l.hard.max_speed} m/s, ${l.hard.max_omega} rad/s · soft ${l.soft.max_speed} m/s, ${l.soft.max_omega} rad/s`;
  if (!$("#lv").value) { $("#lv").value = l.soft.max_speed; $("#lw").value = l.soft.max_omega; }
  $("#events").innerHTML = st.events.slice().reverse().map(e => `<div class="${e.level === "error" ? "ru-err" : ""}">${new Date(e.wall*1000).toLocaleTimeString()} ${e.msg}</div>`).join("");
});

// telemetry (every 100 Hz tick; plotted and tabulated at screen rate)
let last = null;
ts.sync().then(() => setInterval(() => ts.sync(), 30000));
RU.rows("stream", (d) => { const n = d.n;
  for (let i = 0; i < n; i++){ const g = name => d.raw[i*d.fields.length + d.idx[name]];
    plot.push(g("t_tick"), MODS.map(m => g(m + "_drive_vel")).concat([g("cmd_vx")])); }
  last = d; });
const f = (v, k) => (v === null || !isFinite(v)) ? "–" : v.toFixed(k);
function frame(){
  plot.draw();
  if (last){ const g = name => last.last(name);
    const rows = [["", ...MODS], ["angle (°)", ...MODS.map(m => f(g(m + "_steer_angle")*57.2958, 1))],
      ["angle sp (°)", ...MODS.map(m => f(g(m + "_steer_sp")*57.2958, 1))], ["drive (m/s)", ...MODS.map(m => f(g(m + "_drive_vel"), 3))],
      ["drive sp (m/s)", ...MODS.map(m => f(g(m + "_drive_sp"), 3))], ["travel (m)", ...MODS.map(m => f(g(m + "_drive_pos"), 3))],
      ["steer A", ...MODS.map(m => f(g(m + "_steer_current"), 2))], ["drive A", ...MODS.map(m => f(g(m + "_drive_current"), 2))]];
    $("#mods").innerHTML = rows.map((r, i) => "<tr>" + r.map(c => i ? `<td>${c}</td>` : `<th>${c}</th>`).join("") + "</tr>").join("");
    if (ts.offset !== null) $("#lat").textContent = `tick → screen ${((ts.serverNow() - g("t_tick"))*1000).toFixed(1)} ms (clock sync ±${(ts.uncertainty*1000).toFixed(2)} ms)`;
  }
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
</script></body></html>
"""
