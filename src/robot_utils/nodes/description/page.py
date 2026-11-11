"""The description node's page (served at the node root). Relative URLs only.
The robot/camera views are in page_viz.py."""
from .page_viz import VIZ_CSS, VIZ_HTML, VIZ_JS

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot description</title>
<link rel="stylesheet" href="common/ui.css">
<style>
.ru-wrap{max-width:1300px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(380px,1fr));gap:12px}
canvas.view{width:100%;height:300px;display:block;background:var(--bg);border-radius:6px}
.kv{display:grid;grid-template-columns:auto 1fr;gap:2px 10px;font-variant-numeric:tabular-nums;font-size:13px}
.kv span:nth-child(odd){color:var(--muted)}
#poly{position:relative;line-height:0}#poly img{width:100%;display:block;cursor:crosshair;border-radius:6px;min-height:120px;background:#000}
#poly svg{position:absolute;inset:0;width:100%;height:100%;pointer-events:none}
input[type=number]{width:72px}
.job{font:12px ui-monospace,monospace;white-space:pre-wrap;max-height:180px;overflow:auto}
#abort{font-size:16px;font-weight:700;color:var(--stop);border:2px solid var(--stop);background:transparent;padding:6px 16px;min-height:40px;border-radius:var(--radius)}
#cals td{text-align:left}
.caution{border-left:4px solid var(--caution);background:var(--caution-bg);padding:6px 10px;border-radius:var(--radius);margin:6px 0}
table.ru-table td,table.ru-table th{font-size:12px}
#events{max-height:150px;overflow:auto;font-size:12px}
.camhead{display:flex;flex-wrap:wrap;gap:6px 24px;align-items:flex-start}
/*VIZ_CSS*/
</style><script src="../../themes/theme.js"></script>
</head><body data-page="description"><div id="band"></div><div class="ru-wrap">
<div class="ru-row ru-head">
  <h2 style="margin:6px 0">Robot description <span id="state" class="ru-pill">…</span></h2>
  <div class="ru-row"><div id="estop"></div><button class="ru" id="clear" hidden title="Clears this node's e-stop (the swerve node's is separate)">Clear e-stop</button><a class="ru-muted" href="docs">API docs</a></div>
</div>
<div id="err" class="ru-err"></div>
<div class="ru-card" data-panel="cameras" data-role="view">
  <div class="camhead">
    <div><b>Robot and cameras</b> <select id="cam"></select> <span id="reach" class="ru-pill"></span>
      <div class="kv" id="pose" style="margin-top:4px"></div></div>
    <details><summary>Measured / nominal pose (x, y are never calibrated)</summary>
      <div class="ru-row">xyz m <input type="number" step="0.001" id="nx"><input type="number" step="0.001" id="ny"><input type="number" step="0.001" id="nz"></div>
      <div class="ru-row">rpy ° <input type="number" step="0.1" id="nr"><input type="number" step="0.1" id="np"><input type="number" step="0.1" id="nyw">
        <button class="ru" id="setnom">Save</button></div></details>
  </div>
  /*VIZ_HTML*/
</div>
<div class="grid ru-body">
  <div class="ru-card" data-panel="ground" data-role="control"><b>Ground plane</b> <span class="ru-muted">(never moves) roll, pitch, height</span>
    <div class="ru-muted">Click on the depth image to outline floor regions; "Close" ends a polygon. No polygon = automatic (largest near-horizontal plane, using gravity).</div>
    <div id="poly"><img id="depth" alt="depth image"><svg id="svg"></svg></div>
    <div class="ru-row" style="margin-top:6px"><button class="ru" id="close">Close polygon</button><button class="ru" id="undo">Undo point</button>
      <button class="ru" id="clr">Clear</button><button class="ru" id="refresh">Refresh image</button>
      frames <input type="number" id="gframes" value="10" min="1" max="60"><button class="ru" id="ground">Run ground calibration</button></div>
  </div>
  <div class="ru-card" data-panel="yaw" data-role="control"><b>Camera yaw</b> <div class="caution"><b>Drives the base</b>: back and forth along the given vectors, no rotation. Keep the area clear.</div>
    <div class="ru-row">vectors <input id="vecs" value="1,0; 0,1" style="width:140px"> <span class="ru-muted">base frame [vx, vy]; …</span></div>
    <div class="ru-row">amplitude <input type="number" id="amp" value="0.08" step="0.01" min="0.01" max="0.15"> m
      frequency <input type="number" id="freq" value="1.0" step="0.1" min="0.3" max="2"> Hz cycles <input type="number" id="cyc" value="5" min="2" max="20"></div>
    <div class="ru-muted" id="yawinfo"></div>
    <label class="ru-row"><input type="checkbox" id="confirm"> clear space around the robot, tether slack, e-stop at hand</label>
    <div class="ru-row"><button class="ru live" id="yaw">Run yaw calibration</button><button id="abort" hidden>Abort calibration</button></div>
    <div class="ru-muted">Aborts by itself on yaw rate &gt; 15°/s (0.1 s mean), heading drift &gt; 5°, or a lost stream (and then latches the swerve e-stop).</div>
  </div>
</div>
<div class="ru-card" data-panel="job" data-role="telemetry"><b>Job</b> <span id="jobstate" class="ru-pill"></span> <a id="jobraw" class="ru-muted" hidden>recording (.npz)</a><div class="job" id="job">no job yet</div></div>
<div class="ru-card" data-panel="calibrations" data-role="config"><b>Calibrations</b> <span class="ru-muted">(newest first; 5 kept per type and camera, plus the active one)</span>
  <table class="ru-table" id="cals"></table></div>
<div class="ru-card" data-panel="events" data-role="log"><b>Events</b><div id="events"></div></div>
</div>
<script src="common/ui.js"></script>
<script src="common/viz.js"></script>
<script>
const $ = s => document.querySelector(s), D = 180 / Math.PI;
const err = m => $("#err").textContent = m || "";
const estop = RU.estopButton($("#estop"));
let DESC = null, ST = null, polys = [], cur = [], ARM = null;   // ARM: the arm's live links (below)
const cam = () => $("#cam").value || "camera";
async function call(path, body, method){ try { err(""); return await RU.api(path, {method: method || "POST", body}); } catch(e){ err(e.message); throw e; } }

// ---------- description + drawing ----------
async function loadDesc(){
  try { DESC = await RU.api("description"); } catch(e){ err(e.message); return; }
  const sel = $("#cam"), names = Object.keys(DESC.cameras);
  if (sel.options.length !== names.length){ sel.innerHTML = names.map(n => `<option>${n}</option>`).join(""); }
  const c = DESC.cameras[cam()]; if (!c) return;
  const p = c.pose_in_base;
  $("#reach").textContent = c.reachable ? "reachable · SN " + c.serial : "unreachable"; $("#reach").className = "ru-pill " + (c.reachable ? "ru-ok" : "ru-err");
  const from = c.from, src = ["nominal"].concat(from.ground ? ["ground " + from.ground] : [], from.yaw ? ["yaw " + from.yaw] : []).join(" + ");
  $("#pose").innerHTML = [["xyz (m)", p.xyz.map(v => v.toFixed(4)).join(", ")], ["rpy (°)", p.rpy_deg.map(v => v.toFixed(2)).join(", ")],
    ["from", src], ["residuals (°)", from.residual_deg ? from.residual_deg.map(v => v.toFixed(3)).join(", ") : "–"]]
    .map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join("");
  if (document.activeElement.tagName !== "INPUT"){ const n = c.nominal;
    ["nx","ny","nz"].forEach((id, i) => $("#" + id).value = (+n.xyz[i]).toFixed(3));
    ["nr","np","nyw"].forEach((id, i) => $("#" + id).value = (n.rpy[i] * D).toFixed(1)); }
  drawViews();
}
$("#cam").onchange = loadDesc;
/*VIZ_JS*/
$("#setnom").onclick = () => call(`cameras/${cam()}/nominal`, {xyz: ["nx","ny","nz"].map(i => +$("#" + i).value), rpy: ["nr","np","nyw"].map(i => +$("#" + i).value / D)}).then(loadDesc);

// ---------- ground polygons on the depth image ----------
function refreshImg(){ $("#depth").src = `cameras/${cam()}/depth.jpg?t=` + Date.now(); }
$("#depth").onerror = () => err("no depth image (is the camera's depth stream on?)");
function drawPolys(){
  const all = polys.concat(cur.length ? [cur] : []);
  $("#svg").innerHTML = all.map((p, i) => { const open = i === polys.length && cur.length;
    const pts = p.map(([x, y]) => `${x * 100},${y * 100}`).join(" ");
    return `<${open ? "polyline" : "polygon"} points="${pts}" fill="${open ? "none" : "#2563eb33"}" stroke="#60a5fa" stroke-width="0.6" vector-effect="non-scaling-stroke" style="stroke-width:2px"/>`; }).join("")
    + cur.map(([x, y]) => `<circle cx="${x * 100}" cy="${y * 100}" r="0.8" fill="#60a5fa"/>`).join("");
  $("#svg").setAttribute("viewBox", "0 0 100 100"); $("#svg").setAttribute("preserveAspectRatio", "none");
}
$("#depth").onclick = e => { const r = e.target.getBoundingClientRect(); cur.push([(e.clientX - r.left) / r.width, (e.clientY - r.top) / r.height]); drawPolys(); };
$("#close").onclick = () => { if (cur.length >= 3) polys.push(cur); cur = []; drawPolys(); };
$("#undo").onclick = () => { cur.pop(); drawPolys(); };
$("#clr").onclick = () => { polys = []; cur = []; drawPolys(); };
$("#refresh").onclick = refreshImg;
$("#ground").onclick = () => { if (cur.length >= 3){ polys.push(cur); cur = []; drawPolys(); }
  call("calibrate/ground", {camera: cam(), polygons: polys.length ? polys : null, frames: +$("#gframes").value}); };

// ---------- yaw ----------
function yawParams(){ return {camera: cam(), vectors: $("#vecs").value.split(";").map(s => s.split(",").map(Number)).filter(v => v.length === 2),
  amplitude: +$("#amp").value, frequency: +$("#freq").value, cycles: +$("#cyc").value, confirm: $("#confirm").checked}; }
function yawInfo(){ const p = yawParams(), peak = p.amplitude * 2 * Math.PI * p.frequency;
  $("#yawinfo").textContent = `peak speed ${peak.toFixed(2)} m/s (max 0.8) · swing 0–${(2 * p.amplitude).toFixed(2)} m · ~${(p.vectors.length * (p.cycles / p.frequency + 2)).toFixed(0)} s`; }
["#vecs", "#amp", "#freq", "#cyc"].forEach(s => $(s).oninput = yawInfo); yawInfo();
$("#yaw").onclick = () => call("calibrate/yaw", yawParams());
$("#abort").onclick = () => call("jobs/abort", {});
$("#clear").onclick = () => call("estop/clear", {});

// ---------- calibrations ----------
async function loadCals(){
  let cals; try { cals = await RU.api("calibrations"); } catch(e){ return; }
  $("#cals").innerHTML = "<tr><th>type</th><th>id</th><th>camera</th><th>summary</th><th></th></tr>" + cals.map(c =>
    `<tr><td>${c.type}</td><td>${c.id}${c.active ? ' <span class="ru-pill ru-ok">active</span>' : ""}</td><td>${c.serial}</td><td>${c.summary || ""}${c.quality && !c.quality.ok ? '<div class="ru-err">not activated: ' + c.quality.problems.join("; ") + '</div>' : ""}</td>
     <td><button class="ru" data-act='${JSON.stringify({type: c.type, serial: c.serial, id: c.active ? null : c.id})}'>${c.active ? "Deactivate" : "Activate"}</button>
     ${c.type === "yaw" && c.has_raw ? `<button class="ru" title="solve again from the raw data with the current solver (nothing moves); saves a new record" data-resolve='${JSON.stringify({serial: c.serial, id: c.id})}'>Re-solve</button>` : ""}
     <a href="calibrations/${c.type}/${c.serial}/${c.id}" target="_blank">json</a> ${c.has_raw ? `<a href="calibrations/${c.type}/${c.serial}/${c.id}/raw.npz">raw</a>` : ""}</td></tr>`).join("");
  document.querySelectorAll("[data-act]").forEach(b => b.onclick = async () => { const a = JSON.parse(b.dataset.act);
    await call(`calibrations/${a.type}/activate`, {serial: a.serial, id: a.id}); loadCals(); loadDesc(); });
  document.querySelectorAll("[data-resolve]").forEach(b => b.onclick = async () => { const a = JSON.parse(b.dataset.resolve);
    b.disabled = true; try { await call(`calibrations/yaw/${a.serial}/${a.id}/resolve`, {}); } finally { b.disabled = false; }
    loadCals(); loadDesc(); });
}

// ---------- state ----------
let lastJob = null;
const band = RU.stateBand($("#band"));
RU.sse("events", st => { ST = st; estop.update(st);
  const running = !!(st.job && st.job.state === "running");
  band.update(st, {live: running, liveText: running ? "calibration driving the base" : ""});
  $("#abort").hidden = !running;
  $("#clear").hidden = !(st.estop && st.estop.latched);
  $("#state").textContent = st.chocked ? "chocked" : st.state; $("#state").className = "ru-pill " + (st.chocked ? "ru-err" : st.state === "ready" ? "ru-ok" : "ru-warn");
  const j = st.job;
  if (j){ $("#jobstate").textContent = `#${j.id} ${j.kind}: ${j.state} (${j.elapsed} s)`;
    $("#jobstate").className = "ru-pill " + ({done: "ru-ok", running: "ru-warn"}[j.state] || "ru-err");
    $("#job").textContent = j.progress.join("\n") + (j.error ? "\n" + j.error : "") + (j.result ? "\n" + JSON.stringify(j.result, null, 1) : "");
    $("#jobraw").hidden = !j.raw; if (j.raw) $("#jobraw").href = j.raw;
    if (lastJob && lastJob !== j.state && j.state !== "running"){ loadCals(); loadDesc(); } lastJob = j.state; }
  $("#yaw").disabled = st.state !== "ready" || (st.estop && st.estop.latched) || !!st.chocked;
  $("#yaw").title = st.chocked ? "Chocked: an admin removes the chock over SSH" : (st.estop && st.estop.latched) ? "Clear the e-stop first" : "";
  $("#events").innerHTML = st.events.slice().reverse().map(e => `<div class="${e.level === "error" ? "ru-ev-error" : e.level !== "info" ? "ru-ev-warn" : ""}">${new Date(e.wall * 1000).toLocaleTimeString()} ${e.msg}</div>`).join("");
});
loadDesc().then(refreshImg); loadCals(); setInterval(loadDesc, 5000);
// the arm, live: its link frames from the arm node (../arm/, the robot's proxy) at 5 Hz and the gripper gap;
// read only. Without it the 3D view shows the mount alone.
(async () => { for (;;){ try { const [l, gr] = await Promise.all([RU.api("../arm/links"), RU.api("../arm/gripper")]);
    ARM = {frames: l.frames, gap: (gr.gap_mm ?? NaN) / 1000, at: performance.now()}; drawViews(); } catch (_) {}
  await new Promise(r => setTimeout(r, 200)); } })();
</script></body></html>
"""
PAGE = PAGE.replace("/*VIZ_CSS*/", VIZ_CSS).replace("/*VIZ_HTML*/", VIZ_HTML).replace("/*VIZ_JS*/", VIZ_JS)
