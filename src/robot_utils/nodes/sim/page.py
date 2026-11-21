"""The sim node's page (served at the node root). Relative URLs only."""

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Sim world</title>
<link rel="stylesheet" href="common/ui.css">
<style>
.ru-wrap{max-width:1300px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(360px,1fr));gap:12px}
.kv{display:grid;grid-template-columns:auto 1fr;gap:2px 10px;font-variant-numeric:tabular-nums;font-size:13px}
.kv span:nth-child(odd){color:var(--muted)}
.cams{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:8px}
.cams img{width:100%;display:block;border-radius:6px;background:#000;min-height:120px}
input[type=number]{width:72px}
.priv{border-left:3px solid var(--caution);padding-left:8px}
#events{max-height:150px;overflow:auto;font-size:12px}
table.ru-table td,table.ru-table th{font-size:12px}
</style><script src="../../themes/theme.js"></script>
</head><body data-page="sim"><div id="band"></div><div class="ru-wrap">
<div class="ru-row ru-head">
  <h2 style="margin:6px 0">Sim world <span id="state" class="ru-pill">…</span></h2>
  <div class="ru-row"><span class="ru-muted">no lease, no e-stop here: the nodes attached to it have theirs</span>
    <a class="ru-muted" href="docs">API docs</a></div>
</div>
<div id="err" class="ru-err"></div>
<div class="grid ru-body">
  <div class="ru-card"><b>Time</b>
    <div class="kv" id="clock"></div>
    <div class="ru-row" style="margin-top:6px"><button class="ru" id="run">Run</button><button class="ru" id="pause">Pause</button>
      rtf <input type="number" id="rtf" value="1" step="0.1" min="0.05" max="100">
      <button class="ru" id="step">Step</button><input type="number" id="stepms" value="100" min="1"> ms
      <button class="ru" id="reset">Reset</button></div>
  </div>
  <div class="ru-card"><b>Robot</b><div class="kv" id="robot"></div>
    <table class="ru-table" id="mods"></table></div>
  <div class="ru-card"><b>Scene</b>
    <table class="ru-table" id="objs"></table>
    <div class="ru-row" style="margin-top:6px">name <input id="oname" value="box1" style="width:80px">
      <select id="oshape"><option>box</option><option>cylinder</option></select>
      x <input type="number" id="ox" value="0.6" step="0.05"> y <input type="number" id="oy" value="0" step="0.05">
      <button class="ru" id="add">Add</button></div>
    <div class="priv" style="margin-top:8px"><b>Sim-only physics</b>
      <div class="ru-row">push base x <input type="number" id="fx" value="20"> N y <input type="number" id="fy" value="0"> N for 0.2 s
        <button class="ru" id="push">Push</button><button class="ru" id="home">Teleport base home</button></div></div>
  </div>
  <div class="ru-card"><b>Cameras</b> <span class="ru-muted">(rendered on request)</span>
    <div class="cams" id="cams"></div><button class="ru" id="snap">Refresh images</button></div>
  <div class="ru-card"><b>Events</b><div id="events"></div></div>
</div></div>
<script src="common/ui.js"></script>
<script>
const $ = s => document.querySelector(s);
const f3 = v => (v ?? 0).toFixed(3);
async function api(method, path, body) {
  const r = await fetch(path, {method, headers: {"Content-Type": "application/json"}, body: body === undefined ? undefined : JSON.stringify(body)});
  const j = await r.json().catch(() => ({}));
  $("#err").textContent = r.ok ? "" : (j.detail ? JSON.stringify(j.detail) : r.status);
  return j;
}
function kv(el, rows) { el.innerHTML = rows.map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join(""); }
let cams = [];
function render(s) {
  $("#state").textContent = s.state;
  kv($("#clock"), [["sim time", s.t.toFixed(3) + " s"], ["rtf", `${s.rtf} (measured ${s.rtf_measured})`],
                    ["steps/s", s.step_rate], ["fell behind", s.behind], ["model", s.model.source || ""]]);
  const b = s.base;
  kv($("#robot"), [["base xyz", b.xyz.map(f3).join(", ")], ["yaw", (b.rpy[2] * 180 / Math.PI).toFixed(1) + "°"],
                    ["twist", b.twist.map(f3).join(", ")],
                    ["arm", s.arm ? `${s.arm.enabled ? "on" : "off"}${s.arm.fault ? " fault 0x" + s.arm.fault.toString(16) : ""}, gap ${s.arm.gap_mm.toFixed(1)} mm` : "none"],
                    ["fidelity", `base ${s.fidelity.base}; arm ${s.fidelity.arm}`]]);
  $("#mods").innerHTML = "<tr><th>module</th><th>steer °</th><th>goal °</th><th>m/s</th><th>goal</th></tr>" +
    s.modules.map(m => `<tr><td>${m.name}${m.steer_on ? "" : " (off)"}</td><td>${(m.steer * 57.3).toFixed(1)}</td><td>${(m.steer_goal * 57.3).toFixed(1)}</td><td>${f3(m.speed)}</td><td>${f3(m.speed_goal)}</td></tr>`).join("");
  $("#objs").innerHTML = "<tr><th>object</th><th>xyz</th><th></th></tr>" + s.objects.map(o =>
    `<tr><td>${o.name}</td><td>${o.xyz.map(f3).join(", ")}</td><td><button class="ru" data-del="${o.name}">remove</button></td></tr>`).join("");
  const names = Object.keys(s.cameras);
  if (names.join() !== cams.join()) {
    cams = names;
    $("#cams").innerHTML = names.map(n => `<div><div class="ru-muted">${n}</div><img data-cam="${n}" data-s="color" alt="${n} colour"><img data-cam="${n}" data-s="depth" alt="${n} depth"></div>`).join("");
    snap();
  }
  $("#events").innerHTML = s.events.slice().reverse().map(e => `<div>${new Date(e.wall * 1000).toLocaleTimeString()} ${e.msg}</div>`).join("");
}
function snap() { document.querySelectorAll("img[data-cam]").forEach(i => i.src = `cameras/${i.dataset.cam}/${i.dataset.s}.jpg?t=${Date.now()}`); }
$("#run").onclick = () => api("POST", "run", {rtf: +$("#rtf").value});
$("#pause").onclick = () => api("POST", "pause");
$("#step").onclick = () => api("POST", "step", {seconds: +$("#stepms").value / 1000}).then(snap);
$("#reset").onclick = () => api("POST", "scene/reset").then(snap);
$("#snap").onclick = snap;
$("#add").onclick = () => api("POST", "scene/objects", {name: $("#oname").value, shape: $("#oshape").value,
  xyz: [+$("#ox").value, +$("#oy").value, 0.1], size: [0.1, 0.1, 0.1], radius: 0.04, length: 0.15});
$("#objs").onclick = e => { const n = e.target.dataset.del; if (n) api("DELETE", `scene/objects/${encodeURIComponent(n)}`); };
$("#push").onclick = () => api("POST", "physics/wrench", {body: "base", force: [+$("#fx").value, +$("#fy").value, 0], duration: 0.2});
$("#home").onclick = () => api("POST", "physics/teleport", {xyz: [0, 0, 0], yaw: 0});
const es = new EventSource("events");
es.onmessage = e => render(JSON.parse(e.data));
</script></body></html>
"""
