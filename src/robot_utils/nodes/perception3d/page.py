"""The 3d_perception node's page (served at the node root). Relative URLs only.

Shows the colour image with the registered depth over it (RGB / blend / depth), a hover readout of
depth and the 3D point (computed in the browser from the frame and /calibration, no requests), and
a click that asks the node's GET /point (what a click-to-servo tool will do).
"""

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>3D perception</title>
<link rel="stylesheet" href="common/ui.css">
<style>
.ru-wrap{max-width:1400px}
.layout{display:grid;grid-template-columns:minmax(0,1fr) 340px;gap:12px}
@media (max-width:1000px){.layout{grid-template-columns:1fr}}
#view{width:100%;display:block;border-radius:6px;background:#000;cursor:crosshair}
.ctl{display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin-top:8px}
.kv{display:grid;grid-template-columns:auto 1fr;gap:2px 10px;font-variant-numeric:tabular-nums}
.kv span:nth-child(odd){color:var(--muted)}
#readout,#clicked{font-variant-numeric:tabular-nums;min-height:1.4em}
#clicked{white-space:pre-wrap;font-size:12px}
#events{max-height:160px;overflow:auto;font-size:12px}
#bar{height:10px;border-radius:3px;margin-top:4px}
.mode-full{color:var(--ok)} .mode-rotation,.mode-base{color:var(--warn)} .mode-none,.mode-no_depth{color:var(--err)}
</style><script src="../../themes/theme.js"></script>
</head><body><div id="band"></div><div class="ru-wrap">
<div class="ru-row" style="justify-content:space-between">
  <h2 style="margin:6px 0">3D perception <span id="state" class="ru-pill">…</span> <span id="mode" class="ru-pill"></span></h2>
  <div class="ru-row"><span id="lat" class="ru-muted"></span><a class="ru-muted" href="docs">API docs</a></div>
</div>
<div id="err" class="ru-err"></div>
<div class="layout">
  <div class="ru-card">
    <canvas id="view" width="1280" height="720"></canvas>
    <div id="readout" class="ru-muted">hover the image for depth and the 3D point (colour camera frame); click to query the node</div>
    <div class="ctl">
      <span>view
        <label><input type="radio" name="vm" value="rgb"> RGB</label>
        <label><input type="radio" name="vm" value="blend" checked> blend</label>
        <label><input type="radio" name="vm" value="depth"> depth</label></span>
      <label>opacity <input type="range" id="alpha" min="0" max="1" step="0.05" value="0.55"></label>
      <label><input type="checkbox" id="auto" checked> auto range</label>
      <label>min <input id="lo" type="number" step="0.1" value="0.3" style="width:4.5em"> m</label>
      <label>max <input id="hi" type="number" step="0.1" value="4" style="width:4.5em"> m</label>
    </div>
    <div class="ctl">
      <label>compensation <select id="comp"><option value="auto">auto (best available)</option>
        <option value="rotation">IMU rotation only</option><option value="none">none</option></select></label>
      <label>depth grid <select id="scale"><option value="0.25">0.25 × colour</option><option value="0.5">0.5 × colour</option>
        <option value="1">1 × colour (2x2 splat)</option></select></label>
      <label>depth stride <select id="stride"><option>1</option><option>2</option><option>4</option></select></label>
      <span class="ru-muted">(settings are the node's: they apply to every client)</span>
    </div>
    <div id="bar"></div><div class="ru-row" style="justify-content:space-between"><span id="blo" class="ru-muted"></span><span id="bhi" class="ru-muted"></span></div>
    <div id="clicked" class="ru-muted"></div>
  </div>
  <div>
    <div class="ru-card"><b>Latest frame</b><div class="kv" id="frame"></div></div>
    <div class="ru-card"><b>Timing</b> <span class="ru-muted">last 5 s</span><div class="kv" id="timing"></div></div>
    <div class="ru-card"><b>Inputs</b><div class="kv" id="inputs"></div></div>
  </div>
</div>
<div class="ru-card"><b>Events</b><div id="events"></div></div>
</div>
<script src="common/ui.js"></script>
<script>
const $ = s => document.querySelector(s);
const err = m => $("#err").textContent = m || "";
const ts = new RU.TimeSync(); ts.sync().then(() => setInterval(() => ts.sync(), 30000)).catch(() => {});
const fmt = (v, d = 1) => v == null ? "–" : (+v).toFixed(d);

// turbo colormap (polynomial approximation), 256 entries
const LUT = new Uint8Array(256 * 3);
for (let i = 0; i < 256; i++){ const x = i / 255;
  const r = 0.13572138 + x*(4.61539260 + x*(-42.66032258 + x*(132.13108234 + x*(-152.94239396 + x*59.28637943))));
  const g = 0.09140261 + x*(2.19418839 + x*(4.84296658 + x*(-14.18503333 + x*(4.27729857 + x*2.82956604))));
  const b = 0.10667330 + x*(12.64194608 + x*(-60.58204836 + x*(110.36276771 + x*(-89.90310912 + x*27.34824973))));
  LUT[3*i] = Math.max(0, Math.min(255, r*255)); LUT[3*i+1] = Math.max(0, Math.min(255, g*255)); LUT[3*i+2] = Math.max(0, Math.min(255, b*255)); }
$("#bar").style.background = "linear-gradient(90deg," + [0, 64, 128, 192, 255].map(i => `rgb(${LUT[3*i]},${LUT[3*i+1]},${LUT[3*i+2]})`).join(",") + ")";

let CAL = null;
async function loadCal(){ try { const c = await RU.api("calibration"); if (c.ready) CAL = c; } catch(e){} }
loadCal(); setInterval(loadCal, 5000);

const view = $("#view"), vctx = view.getContext("2d");
const dcan = document.createElement("canvas"), dctx = dcan.getContext("2d");
let cur = null, busy = false, next = null, range = [0.3, 4], marker = null, hover = null;

function depthRange(dep){
  if (!$("#auto").checked) return [+$("#lo").value, +$("#hi").value];
  const s = []; for (let i = 0; i < dep.length; i += 7) if (dep[i]) s.push(dep[i]);
  if (s.length < 10) return range;
  s.sort((a, b) => a - b);
  return [s[Math.floor(s.length * 0.02)] / 1000, s[Math.floor(s.length * 0.98)] / 1000];
}

async function show(h, p){
  const c = h.color, d = h.depth;
  const bmp = c.len ? await createImageBitmap(new Blob([p.subarray(c.offset, c.offset + c.len)], {type: "image/jpeg"})) : null;
  const dep = d.len ? new Uint16Array(p.buffer.slice(p.byteOffset + d.offset, p.byteOffset + d.offset + d.len)) : null;
  cur = {h, dep, bmp};
  draw();
  const now = ts.offset == null ? null : ts.serverNow();
  $("#lat").textContent = now == null ? "" : `capture → display ${((now - h.t_color) * 1000).toFixed(0)} ms`;
  const m = $("#mode"); m.textContent = h.mode; m.className = "ru-pill mode-" + h.mode;
  $("#frame").innerHTML = [["seq", h.seq + (h.dropped ? ` (${h.dropped} skipped here)` : "")], ["mode", h.mode],
    ["colour − depth time", fmt(h.dt_color_depth * 1000, 1) + " ms"],
    ["compensated", `${fmt(h.rotation_deg, 2)}° / ${fmt(h.translation_mm, 1)} mm`],
    ["depth grid", `${d.width}×${d.height} (valid ${fmt(d.valid * 100, 0)} %)`],
    ["compute / CPU", `${fmt(h.compute_ms)} / ${fmt(h.cpu_ms)} ms`], ["t_color → published", fmt(h.latency_ms) + " ms"]]
    .map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join("");
}

function draw(){
  if (!cur) return;
  const {h, dep, bmp} = cur, d = h.depth, mode = document.querySelector("input[name=vm]:checked").value;
  if (view.width !== h.color.width || view.height !== h.color.height){ view.width = h.color.width; view.height = h.color.height; }
  vctx.globalAlpha = 1;
  if (bmp && mode !== "depth") vctx.drawImage(bmp, 0, 0, view.width, view.height);
  else { vctx.fillStyle = "#000"; vctx.fillRect(0, 0, view.width, view.height); }
  if (dep && mode !== "rgb"){
    range = depthRange(dep);
    $("#blo").textContent = range[0].toFixed(2) + " m"; $("#bhi").textContent = range[1].toFixed(2) + " m";
    if (dcan.width !== d.width || dcan.height !== d.height){ dcan.width = d.width; dcan.height = d.height; }
    const img = dctx.createImageData(d.width, d.height), px = img.data;
    const lo = range[0] * 1000, k = 255 / Math.max(1, range[1] * 1000 - lo);
    for (let i = 0; i < dep.length; i++){
      const z = dep[i]; if (!z) continue;
      const j = Math.max(0, Math.min(255, (z - lo) * k)) | 0;
      px[4*i] = LUT[3*j]; px[4*i+1] = LUT[3*j+1]; px[4*i+2] = LUT[3*j+2]; px[4*i+3] = 255;
    }
    dctx.putImageData(img, 0, 0);
    vctx.imageSmoothingEnabled = false;
    vctx.globalAlpha = mode === "depth" ? 1 : +$("#alpha").value;
    vctx.drawImage(dcan, 0, 0, view.width, view.height);
    vctx.globalAlpha = 1;
  }
  if (marker){ vctx.strokeStyle = "#fff"; vctx.lineWidth = 2; vctx.beginPath(); vctx.arc(marker[0], marker[1], 9, 0, 7); vctx.stroke();
    vctx.strokeStyle = "#000"; vctx.beginPath(); vctx.arc(marker[0], marker[1], 11, 0, 7); vctx.stroke(); }
  if (hover) readout(hover[0], hover[1]);
}

RU.stream("stream", (h, p) => {
  if (h.type !== "fused") return;
  next = [h, p];
  if (busy) return;                       // decoding the previous one: keep only the newest
  busy = true;
  (async () => { while (next){ const [a, b] = next; next = null; try { await show(a, b); } catch(e){ err(e.message); } } busy = false; })();
});

// colour pixel -> undistorted ray (OpenCV model, fixed-point iteration as the node does)
function ray(u, v){
  const c = CAL.color, D = c.distortion || {};
  const k1 = D.k1||0, k2 = D.k2||0, k3 = D.k3||0, k4 = D.k4||0, k5 = D.k5||0, k6 = D.k6||0, p1 = D.p1||0, p2 = D.p2||0;
  const xd = (u - c.cx) / c.fx, yd = (v - c.cy) / c.fy; let x = xd, y = yd;
  for (let i = 0; i < 8; i++){
    const r2 = x*x + y*y, ic = (1 + r2*(k4 + r2*(k5 + r2*k6))) / (1 + r2*(k1 + r2*(k2 + r2*k3)));
    x = (xd - (2*p1*x*y + p2*(r2 + 2*x*x))) * ic; y = (yd - (p1*(r2 + 2*y*y) + 2*p2*x*y)) * ic;
  }
  return [x, y];
}
function pixelAt(ev){
  const r = view.getBoundingClientRect();
  return [(ev.clientX - r.left) / r.width * view.width - 0.5, (ev.clientY - r.top) / r.height * view.height - 0.5];
}
function readout(u, v){
  if (!cur || !cur.dep){ $("#readout").textContent = `u ${u.toFixed(0)}, v ${v.toFixed(0)}: no depth`; return; }
  const d = cur.h.depth, sx = d.width / view.width, sy = d.height / view.height;
  const iu = Math.round((u + 0.5) * sx - 0.5), iv = Math.round((v + 0.5) * sy - 0.5);
  const z = (iu >= 0 && iu < d.width && iv >= 0 && iv < d.height) ? cur.dep[iv * d.width + iu] : 0;
  let s = `u ${u.toFixed(0)}, v ${v.toFixed(0)}: `;
  if (!z) s += "no depth here";
  else { s += `depth ${(z / 1000).toFixed(3)} m`;
    if (CAL){ const [x, y] = ray(u, v), m = z / 1000; s += `  point (colour frame) x ${(x*m).toFixed(3)}  y ${(y*m).toFixed(3)}  z ${m.toFixed(3)} m`; } }
  $("#readout").textContent = s;
}
view.addEventListener("mousemove", ev => { hover = pixelAt(ev); readout(hover[0], hover[1]); });
view.addEventListener("mouseleave", () => { hover = null; });
view.addEventListener("click", async ev => {
  const [u, v] = pixelAt(ev); marker = [u, v]; draw();
  try {
    const r = await RU.api(`point?u=${u.toFixed(1)}&v=${v.toFixed(1)}`);
    const f = a => a ? "[" + a.map(x => x.toFixed(3)).join(", ") + "]" : "null";
    $("#clicked").textContent = `GET point?u=${u.toFixed(1)}&v=${v.toFixed(1)} → depth ${r.depth_mm} mm (${r.source}), ` +
      `point ${f(r.point)} m (colour frame), point_base ${f(r.point_base)} m, frame ${r.seq} (${r.mode}, ${fmt(r.age_s * 1000, 0)} ms old)`;
  } catch(e){ $("#clicked").textContent = "point: " + e.message; }
});
document.querySelectorAll("input[name=vm],#alpha,#auto,#lo,#hi").forEach(el => el.addEventListener("input", draw));

async function setting(body){ try { await RU.api("settings", {method: "POST", body}); err(""); } catch(e){ err(e.message); } }
$("#comp").onchange = () => setting({compensation: $("#comp").value});
$("#scale").onchange = () => { const s = +$("#scale").value; setting({scale: s, splat: s >= 1 ? 2 : 1}); };
$("#stride").onchange = () => setting({stride: +$("#stride").value});

let first = true;
const band = RU.stateBand($("#band"));
RU.sse("events", st => {
  // the camera node flags covered frames (common/locks.py): these are procedural, not images
  band.update({locks: st.covered ? [{kind: "cover", reason: "frames are procedural; 3D points are not real",
                                     by: "the camera node (see its page)", age_s: 0}] : []});
  const p = $("#state"); p.textContent = st.state; p.className = "ru-pill " + (st.state === "running" ? "ru-ok" : st.state === "idle" ? "" : "ru-warn");
  err(st.error);
  if (first || document.activeElement.tagName !== "SELECT"){
    $("#comp").value = st.params.compensation; $("#scale").value = String(st.params.scale); $("#stride").value = String(st.params.stride); first = false; }
  const t = st.timing;
  $("#timing").innerHTML = [["fused / colour rate", `${fmt(st.rate_hz)} / ${fmt(st.color_rate_hz)} Hz`],
    ["compute (median / p95)", `${fmt(t.compute_ms.median)} / ${fmt(t.compute_ms.p95)} ms`],
    ["CPU (median / p95)", `${fmt(t.cpu_ms.median)} / ${fmt(t.cpu_ms.p95)} ms`],
    ["t_color → published", `${fmt(t.latency_ms.median)} / ${fmt(t.latency_ms.p95)} ms`],
    ["wait for IMU / pose", `${fmt(t.wait_ms.median)} / ${fmt(t.wait_ms.p95)} ms`],
    ["colour − depth time", `${fmt(t.dt_color_depth_ms.median)} / ${fmt(t.dt_color_depth_ms.p95)} ms`],
    ["modes", Object.entries(st.modes).map(([k, v]) => `${k} ${v}`).join(", ") || "–"],
    ["skipped colour frames", st.skipped], ["stream clients", st.clients]]
    .map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join("");
  const s = st.sources, rows = Object.entries(st.inputs).map(([k, i]) =>
    [k, (i.connected ? `<span class="ru-ok">${fmt(i.rate_hz)} Hz</span>` : `<span class="ru-err">${i.error || "down"}</span>`) +
        ` <span class="ru-muted">${i.target}${i.path}</span>`]);
  rows.push(["camera calibration", s.camera_calibration.ok ? '<span class="ru-ok">ok</span>' : `<span class="ru-err">${s.camera_calibration.error || "none"}</span>`]);
  rows.push(["camera pose in base", s.description.ok ? (s.description.calibrated ? '<span class="ru-ok">calibrated</span>' : '<span class="ru-warn">nominal only</span>') +
      ` <span class="ru-muted">${Object.entries(s.description.from || {}).filter(([k, v]) => v && k !== "residual_deg").map(([k, v]) => k + " " + v).join(", ")}</span>`
      : `<span class="ru-warn">${s.description.error || "none"}</span>`]);
  $("#inputs").innerHTML = rows.map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join("");
  $("#events").innerHTML = st.events.slice().reverse().map(e => `<div class="${e.level === "info" ? "" : "ru-" + (e.level === "error" ? "err" : "warn")}">` +
    `${new Date(e.wall * 1000).toLocaleTimeString()} ${e.msg.replace(/</g, "&lt;")}</div>`).join("");
});
</script></body></html>
"""
