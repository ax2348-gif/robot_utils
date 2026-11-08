"""The description node's page (served at the node root). Relative URLs only."""

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
#abort{font-size:18px;font-weight:700;color:var(--err);border:3px solid var(--err);background:transparent;padding:8px 18px}
table.ru-table td,table.ru-table th{font-size:12px}
#events{max-height:150px;overflow:auto;font-size:12px}
</style></head><body><div class="ru-wrap">
<div class="ru-row" style="justify-content:space-between">
  <h2 style="margin:6px 0">Robot description <span id="state" class="ru-pill">…</span></h2>
  <div class="ru-row"><div id="estop"></div><button class="ru" id="clear">Clear e-stop</button><a class="ru-muted" href="docs">API docs</a></div>
</div>
<div id="err" class="ru-err"></div>
<div class="grid">
  <div class="ru-card"><b>Robot</b> <span class="ru-muted">top view (x up, y left) · grid 0.1 m</span>
    <canvas class="view" id="top"></canvas>
    <b>Side</b> <span class="ru-muted">(x right, z up)</span><canvas class="view" id="side" style="height:200px"></canvas></div>
  <div class="ru-card"><b>Camera</b> <select id="cam"></select> <span id="reach" class="ru-pill"></span>
    <div class="kv" id="pose"></div>
    <details style="margin-top:8px"><summary>Measured / nominal pose (x, y are never calibrated)</summary>
      <div class="ru-row">xyz m <input type="number" step="0.001" id="nx"><input type="number" step="0.001" id="ny"><input type="number" step="0.001" id="nz"></div>
      <div class="ru-row">rpy ° <input type="number" step="0.1" id="nr"><input type="number" step="0.1" id="np"><input type="number" step="0.1" id="nyw">
        <button class="ru" id="setnom">Save</button></div></details>
  </div>
</div>
<div class="grid">
  <div class="ru-card"><b>Ground plane</b> <span class="ru-muted">(never moves) roll, pitch, height</span>
    <div class="ru-muted">Click on the depth image to outline floor regions; "Close" ends a polygon. No polygon = automatic (largest near-horizontal plane, using gravity).</div>
    <div id="poly"><img id="depth" alt="depth image"><svg id="svg"></svg></div>
    <div class="ru-row" style="margin-top:6px"><button class="ru" id="close">Close polygon</button><button class="ru" id="undo">Undo point</button>
      <button class="ru" id="clr">Clear</button><button class="ru" id="refresh">Refresh image</button>
      frames <input type="number" id="gframes" value="10" min="1" max="60"><button class="ru" id="ground">Run ground calibration</button></div>
  </div>
  <div class="ru-card"><b>Camera yaw</b> <span class="ru-err">drives the base</span> <span class="ru-muted">(back and forth, no rotation)</span>
    <div class="ru-row">vectors <input id="vecs" value="1,0; 0,1" style="width:140px"> <span class="ru-muted">base frame [vx, vy]; …</span></div>
    <div class="ru-row">amplitude <input type="number" id="amp" value="0.08" step="0.01" min="0.01" max="0.15"> m
      frequency <input type="number" id="freq" value="1.0" step="0.1" min="0.3" max="2"> Hz cycles <input type="number" id="cyc" value="5" min="2" max="20"></div>
    <div class="ru-muted" id="yawinfo"></div>
    <label class="ru-row"><input type="checkbox" id="confirm"> clear space around the robot, tether slack, e-stop at hand</label>
    <div class="ru-row"><button class="ru" id="yaw">Run yaw calibration</button><button id="abort">ABORT</button></div>
    <div class="ru-muted">Aborts by itself on yaw rate &gt; 15°/s (0.1 s mean), heading drift &gt; 5°, or a lost stream (and then latches the swerve e-stop).</div>
  </div>
</div>
<div class="ru-card"><b>Job</b> <span id="jobstate" class="ru-pill"></span> <a id="jobraw" class="ru-muted" hidden>recording (.npz)</a><div class="job" id="job">no job yet</div></div>
<div class="ru-card"><b>Calibrations</b> <span class="ru-muted">(newest first; 5 kept per type and camera, plus the active one)</span>
  <table class="ru-table" id="cals"></table></div>
<div class="ru-card"><b>Events</b><div id="events"></div></div>
</div>
<script src="common/ui.js"></script>
<script>
const $ = s => document.querySelector(s), D = 180 / Math.PI;
const err = m => $("#err").textContent = m || "";
const estop = RU.estopButton($("#estop"));
let DESC = null, ST = null, polys = [], cur = [];
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
  draw();
}
function draw(){
  if (!DESC) return; const c = DESC.cameras[cam()]; if (!c) return;
  const T = c.pose_in_base.matrix, R = [0,1,2].map(i => T[i].slice(0, 3)), t = [T[0][3], T[1][3], T[2][3]];
  const axis = (k, L) => [t[0] + R[0][k] * L, t[1] + R[1][k] * L, t[2] + R[2][k] * L];
  const H = 0.1656 + 0.04;   // half footprint (m), approx
  for (const [id, proj] of [["top", p => [-p[1], p[0]]], ["side", p => [p[0], p[2]]]]){
    const cv = $("#" + id), dpr = devicePixelRatio || 1, W = cv.clientWidth, Hh = cv.clientHeight;
    cv.width = W * dpr; cv.height = Hh * dpr; const g = cv.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, Hh);
    const s = Math.min(W, Hh) / 1.2, cx = W / 2, cy = id === "top" ? Hh / 2 : Hh * 0.85;
    const P = p => { const q = proj(p); return [cx + q[0] * s, cy - q[1] * s]; };
    const css = n => getComputedStyle(document.body).getPropertyValue(n).trim();
    g.strokeStyle = css("--line"); g.lineWidth = 1;
    for (let v = -0.6; v <= 0.6001; v += 0.1){ let a = P(id === "top" ? [v, -0.6, 0] : [v, 0, 0]), b = P(id === "top" ? [v, 0.6, 0] : [v, 0, 0.9]); g.beginPath(); g.moveTo(...a); g.lineTo(...b); g.stroke();
      a = P(id === "top" ? [-0.6, v, 0] : [-0.6, 0, v + 0.6]); b = P(id === "top" ? [0.6, v, 0] : [0.6, 0, v + 0.6]); g.beginPath(); g.moveTo(...a); g.lineTo(...b); g.stroke(); }
    g.lineWidth = 2; g.strokeStyle = css("--muted");
    const box = id === "top" ? [[H, H, 0], [H, -H, 0], [-H, -H, 0], [-H, H, 0]] : [[H, 0, 0], [H, 0, 0.12], [-H, 0, 0.12], [-H, 0, 0]];
    g.beginPath(); box.forEach((p, i) => i ? g.lineTo(...P(p)) : g.moveTo(...P(p))); g.closePath(); g.stroke();
    const o = P([0, 0, 0]); g.strokeStyle = "#dc2626"; g.beginPath(); g.moveTo(...o); g.lineTo(...P([0.15, 0, 0])); g.stroke();
    g.strokeStyle = "#16a34a"; g.beginPath(); g.moveTo(...o); g.lineTo(...P(id === "top" ? [0, 0.15, 0] : [0, 0, 0.15])); g.stroke();
    // camera: optical axes (x red, y green, z blue = view direction) and a small frustum
    const pc = P(t); g.lineWidth = 2.5;
    [["#dc2626", 0], ["#16a34a", 1], ["#2563eb", 2]].forEach(([col, k]) => { g.strokeStyle = col; g.beginPath(); g.moveTo(...pc); g.lineTo(...P(axis(k, k === 2 ? 0.25 : 0.08))); g.stroke(); });
    g.fillStyle = "#2563eb"; g.beginPath(); g.arc(pc[0], pc[1], 4, 0, 7); g.fill();
    g.fillStyle = css("--muted"); g.font = "11px system-ui"; g.fillText(id === "top" ? "camera (blue = view direction)" : `height ${t[2].toFixed(3)} m`, 6, 14);
  }
}
$("#cam").onchange = loadDesc; addEventListener("resize", draw);
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
     <a href="calibrations/${c.type}/${c.serial}/${c.id}" target="_blank">json</a> ${c.has_raw ? `<a href="calibrations/${c.type}/${c.serial}/${c.id}/raw.npz">raw</a>` : ""}</td></tr>`).join("");
  document.querySelectorAll("[data-act]").forEach(b => b.onclick = async () => { const a = JSON.parse(b.dataset.act);
    await call(`calibrations/${a.type}/activate`, {serial: a.serial, id: a.id}); loadCals(); loadDesc(); });
}

// ---------- state ----------
let lastJob = null;
RU.sse("events", st => { ST = st; estop.update(st);
  $("#state").textContent = st.state; $("#state").className = "ru-pill " + (st.state === "ready" ? "ru-ok" : "ru-warn");
  const j = st.job;
  if (j){ $("#jobstate").textContent = `#${j.id} ${j.kind}: ${j.state} (${j.elapsed} s)`;
    $("#jobstate").className = "ru-pill " + ({done: "ru-ok", running: "ru-warn"}[j.state] || "ru-err");
    $("#job").textContent = j.progress.join("\n") + (j.error ? "\n" + j.error : "") + (j.result ? "\n" + JSON.stringify(j.result, null, 1) : "");
    $("#jobraw").hidden = !j.raw; if (j.raw) $("#jobraw").href = j.raw;
    if (lastJob && lastJob !== j.state && j.state !== "running"){ loadCals(); loadDesc(); } lastJob = j.state; }
  $("#yaw").disabled = st.state !== "ready" || (st.estop && st.estop.latched);
  $("#events").innerHTML = st.events.slice().reverse().map(e => `<div class="${e.level !== "info" ? "ru-err" : ""}">${new Date(e.wall * 1000).toLocaleTimeString()} ${e.msg}</div>`).join("");
});
loadDesc().then(refreshImg); loadCals(); setInterval(loadDesc, 5000);
</script></body></html>
"""
