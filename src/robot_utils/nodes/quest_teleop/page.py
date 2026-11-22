"""The Quest teleop node's page (served at the node root). Relative URLs only."""

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Quest teleop</title>
<meta name="description" content="Whole-body teleop from the Quest headset through the WBC: start / stop, headset up / down, link health, top-down view.">
<link rel="stylesheet" href="common/ui.css">
<style>
.ru-wrap{max-width:1300px}
.layout{display:grid;grid-template-columns:minmax(0,1fr) 380px;gap:12px}
@media (max-width:900px){.layout{grid-template-columns:1fr}}
.layout>*{min-width:0}
#reason,#events,.kv span{overflow-wrap:anywhere}
#view{width:100%;height:500px;display:block;touch-action:none}
@media (max-width:600px){#view{height:360px}}
.kv{display:grid;grid-template-columns:auto 1fr;gap:2px 10px;font-variant-numeric:tabular-nums}
.kv span:nth-child(odd){color:var(--muted)}
.ctl{display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin:6px 0}
.steps{display:flex;flex-wrap:wrap;gap:4px;margin:8px 0 4px}
.steps span{padding:1px 8px;border:1px solid transparent;border-radius:999px;color:var(--muted);font-size:13px}
.steps span.on{border-color:var(--ink);color:var(--ink)}
.steps span.on.good{border-color:var(--live);color:var(--live)}
.steps span.on.bad{border-color:var(--caution);color:var(--caution)}
#reason{min-height:1.3em}
.hands{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:6px}
.hand{display:flex;gap:10px;align-items:center}
.clutch{width:26px;height:26px;border-radius:50%;border:1.5px solid var(--rule);flex:none}
.clutch.valid{border-color:var(--muted)}
.clutch.engaged{border:3px solid var(--live)}
.grip{position:relative;width:100%;height:8px;border:1px solid var(--rule);border-radius:4px;margin-top:4px}
.grip i{position:absolute;top:-4px;bottom:-4px;width:2px;background:var(--ink);left:0}
.legend{display:flex;flex-wrap:wrap;gap:4px 14px;font-size:12px;color:var(--muted);margin-top:4px}
.legend b{font-weight:600}
canvas.ru-plot{width:100%;height:70px}
#ctlout{max-height:180px;overflow:auto;font:12px/1.4 var(--mono);white-space:pre-wrap;margin:6px 0 0;padding:6px 8px;border:1px solid var(--rule);border-radius:var(--radius)}
#ctlout:empty{display:none}
#events{max-height:160px;overflow:auto;font-size:12px}
.lim{border-left:2px solid var(--rule);padding-left:8px;margin:6px 0}
.lim.on{border-left-color:var(--caution)}
</style><script src="../../themes/theme.js"></script>
</head><body data-page="quest_teleop"><div id="band"></div><div class="ru-wrap">
<div class="ru-row ru-head">
  <h2 style="margin:6px 0">Quest teleop <span id="status" class="ru-pill">…</span></h2>
  <div class="ru-row"><span id="estop"></span><a class="ru-muted" href="docs">API docs</a></div>
</div>
<div id="err" class="ru-err"></div>
<div class="layout ru-body">
  <div class="ru-card" data-panel="view" data-role="view">
    <b>Top-down</b> <span class="ru-muted">world frame, x up, follows the base; rings 0.5 m</span>
    <canvas id="view"></canvas>
    <div class="legend">
      <span><b style="color:var(--ink)">▭</b> base</span>
      <span><b style="color:var(--signal)">┅</b> base hint (pose + heading)</span>
      <span><b style="color:var(--plot-2)">○</b> EE commanded</span>
      <span><b style="color:var(--plot-2)">+</b> EE target (WBC)</span>
      <span><b style="color:var(--muted)">◦</b> operator's head (engaged)</span>
    </div>
  </div>
  <div class="ru-side">
    <div class="ru-card" data-panel="teleop" data-role="control"><b>Teleop</b>
      <div class="ctl"><button class="ru live" id="start">Start</button><button class="ru" id="stop">Stop</button>
        <span class="ru-muted" id="sending"></span></div>
      <div class="steps" id="steps"></div>
      <div id="armed"></div>
      <div class="ru-muted" id="reason"></div>
      <div class="ru-muted" style="font-size:13px">Start takes the WBC lease and enables it (arm + base); nothing moves until <b>Y</b> (left controller) arms it. <b>X</b> disarms and disables (the arm floats, the base coasts); Y re-enables. Armed, the grip clutches the arm after a release.</div>
    </div>
    <div class="ru-card" data-panel="headset" data-role="control"><b>Headset</b>
      <div class="kv" id="link"></div>
      <canvas class="ru-plot" id="prate"></canvas>
      <div class="ctl"><button class="ru" id="qup">Quest up</button><button class="ru" id="qdown">Quest down</button>
        <span class="ru-muted" id="ctlstate"></span></div>
      <div class="ru-muted" style="font-size:13px">Quest up starts the headset's XR telemetry: only while someone is wearing it (on an unworn headset it blanks the display). Never started automatically.</div>
      <pre id="ctlout"></pre>
    </div>
    <div class="ru-card" data-panel="hands" data-role="telemetry"><b>Hands</b> <span class="ru-muted">clutch ring · trigger (gripper)</span>
      <div class="hands" id="hands"></div>
    </div>
    <div class="ru-card" data-panel="wbc" data-role="telemetry"><b>WBC</b>
      <div class="lim" id="lim"></div>
      <div class="kv" id="wbc"></div>
    </div>
  </div>
</div>
<div class="ru-card" data-panel="events" data-role="log"><b>Events</b><div id="events"></div></div>
</div>
<script src="common/ui.js"></script>
<script>
const $ = s => document.querySelector(s);
const err = m => $("#err").textContent = m || "";
const call = async p => { try { err(""); return await RU.api(p, {method: "POST"}); } catch(e){ err(e.message); } };
const es = RU.estopButton($("#estop"));
const band = RU.stateBand($("#band"));
$("#start").onclick = () => call("start");
$("#stop").onclick = () => call("stop");
$("#qup").onclick = () => call("quest/up");
$("#qdown").onclick = () => call("quest/down");
const esc = s => String(s === null || s === undefined ? "" : s).replace(/[&<>"]/g, c => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;"}[c]));
const f = (v, n) => v === null || v === undefined || !isFinite(v) ? "–" : Number(v).toFixed(n);
const STEPS = [["released", "released", "good"], ["engaged", "engaged", "good"], ["needs_release", "needs release", "bad"],
               ["stale", "stale", "bad"], ["refused", "refused", "bad"]];
const LIMITED = {none: "", reach: "reach", manipulability: "manipulability", fence: "fence", joint_limit: "joint limit", base_limit: "base limit"};
const rate = new RU.Plot($("#prate"), {seconds: 30, series: [{name: "frames/s"}], ymin: 0});
let last = null;
RU.rows("stream?batch_ms=100", d => {
  for (let i = 0; i < d.n; i++){
    const p = {}; for (const k of d.fields) p[k] = d.col(k)[i];
    rate.push(p.t, [p.quest_rate]);
    last = p;
  }
});
RU.sse("events", s => {
  const T = s.teleop, H = s.headset, W = s.wbc, C = s.ctl;
  es.update(s);
  const running = T.phase === "running";
  band.update(s, {live: running, liveText: running ? "teleop running: " + s.state.replace("_", " ") : ""});
  $("#status").textContent = s.state.replace("_", " ");
  $("#status").className = "ru-pill " + ({engaged: "ru-ok", released: "ru-ok", stale: "ru-warn", needs_release: "ru-warn", refused: "ru-warn"}[s.state] || "");
  $("#start").disabled = T.phase !== "stopped"; $("#stop").disabled = T.phase !== "running";
  $("#sending").textContent = running ? (T.sending ? `sending · ${T.sent ? T.sent.target : 0} targets` : "silent (holding)") : (T.stop_reason ? "stopped: " + T.stop_reason : "");
  $("#steps").innerHTML = STEPS.map(([k, label, tone]) => `<span class="${s.state === k ? "on " + tone : ""}">${label}</span>`).join("");
  const why = running ? (T.reason || "") : (s.error || "");
  $("#reason").textContent = why ? "last: " + why : (running && T.transport ? "datagrams: " + T.transport.split(" ")[0] + " socket" : "");
  $("#armed").className = "lim" + (running && T.armed ? " on" : "");
  $("#armed").textContent = !running ? "" : T.armed ? `ARMED (press ${T.arm_seq})` : `disarmed${T.disarm_reason ? " — " + T.disarm_reason : ""}${T.disabled_by_x ? " · WBC disabled by X (Y re-enables)" : ""}`;
  $("#reason").title = T.transport || "";
  // headset link
  const age = H.age === null ? "–" : (H.age < 1 ? (H.age * 1e3).toFixed(0) + " ms" : H.age.toFixed(1) + " s");
  const link = [
    ["stream", `${esc(s.config.quest_host)}:${s.config.quest_port} · ${H.connected ? "connected" : "not connected"}`],
    ["rate", `${f(H.rate_hz, 0)} frames/s · age ${age}`],
    ["latency", T.input ? `${T.input.latency_ms === null ? "–" : f(T.input.latency_ms, 0) + " ms"} · ${f(T.input.rate_hz, 0)} Hz (0.5 s)${T.input.fresh ? "" : " · stale: " + esc(T.input.why)}` : "–"],
    ["focus", H.focused === null ? "–" : (H.focused ? "focused" : "not focused")],
    ["battery", esc(H.battery ? H.battery.text : "not reported")],
    ["XR session", esc(H.xr_state || "–")],
    ["session", esc(H.session_id || "–") + (H.scheme ? " · " + esc(H.scheme) : "")],
  ];
  if (!H.connected && H.last_error) link.push(["error", esc(H.last_error)]);
  $("#link").innerHTML = link.map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join("");
  $("#qup").disabled = $("#qdown").disabled = C.busy;
  $("#ctlstate").textContent = C.busy ? `quest ${C.command} running…` : (C.command ? `quest ${C.command}: ${C.rc === 0 ? "ok" : "failed (exit " + C.rc + ")"}${C.timed_out ? ", timed out" : ""}` : `ssh ${s.config.ssh_host}`);
  const out = $("#ctlout"), atEnd = out.scrollTop + out.clientHeight >= out.scrollHeight - 4;
  out.textContent = C.lines.map(l => l.line).join("\n");
  if (atEnd) out.scrollTop = out.scrollHeight;
  // hands
  const hands = H.hands || {};
  $("#hands").innerHTML = ["left", "right"].map(k => { const h = hands[k] || {};
    const g = h.gripper === null || h.gripper === undefined ? null : Math.max(0, Math.min(1, h.gripper));
    return `<div class="hand"><div class="clutch ${h.valid ? "valid" : ""} ${h.engaged ? "engaged" : ""}" title="${h.engaged ? "clutch engaged" : "clutch released"}"></div>
      <div style="flex:1"><div>${k}${k === s.config.hand ? ' <span class="ru-muted">(arm)</span>' : ""}</div>
      <div class="ru-muted" style="font-size:12px">${!h.valid ? "not tracked" : h.engaged ? "engaged" : h.needs_release ? "release first" : "released"}</div>
      <div class="grip" title="trigger ${g === null ? "–" : g.toFixed(2)}"><i style="left:calc(${g === null ? 0 : g * 100}% - 1px);${g === null ? "display:none" : ""}"></i></div></div></div>`; }).join("");
  // WBC
  const lim = W.limited_by && W.limited_by !== "none";
  $("#lim").className = "lim" + (lim ? " on" : "");
  $("#lim").textContent = !W.reachable ? "WBC not reachable" : lim ? "limited: " + (LIMITED[W.limited_by] || W.limited_by) : "not limited";
  const wkv = W.reachable ? [
    ["state", esc(W.state) + (W.fault ? " — " + esc(W.fault) : "") + (W.hold_reason ? " (" + esc(W.hold_reason) + ")" : "")],
    ["base mode", esc(W.base_mode || "–")],
    ["lease", W.lease.held ? esc(W.lease.holder) + (s.lease.ours ? " (this node)" : "") : "free"],
    ["stream age", W.stream_age === null ? "–" : (W.stream_age * 1e3).toFixed(0) + " ms"],
    ["dropped", W.dropped && Object.keys(W.dropped).length ? esc(JSON.stringify(W.dropped)) : "none"],
  ] : [["error", esc(W.error)]];
  if (s.estop.latched) wkv.push(["e-stop", "latched: clear it on the WBC, arm and swerve pages"]);
  $("#wbc").innerHTML = wkv.map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join("");
  $("#events").innerHTML = s.events.slice().reverse().map(e => `<div class="${e.level === "error" ? "ru-ev-error" : e.level === "warn" ? "ru-ev-warn" : ""}">${new Date(e.wall * 1000).toLocaleTimeString()} ${esc(e.msg)}</div>`).join("");
});

// top-down view: world x up, y left, centred on the base
const cv = $("#view"), SCALE_M = 1.6;   // half-height of the view in metres
function footprint(g, w2s, x, y, yaw, dash){
  const L = 0.26, Wd = 0.24, c = Math.cos(yaw), s = Math.sin(yaw);
  const pts = [[L, Wd], [L, -Wd], [-L, -Wd], [-L, Wd]].map(([a, b]) => w2s(x + a * c - b * s, y + a * s + b * c));
  g.setLineDash(dash || []); g.beginPath(); pts.forEach((p, i) => i ? g.lineTo(...p) : g.moveTo(...p)); g.closePath(); g.stroke();
  const [hx, hy] = w2s(x + (L + 0.12) * c, y + (L + 0.12) * s), [ox, oy] = w2s(x, y);
  g.beginPath(); g.moveTo(ox, oy); g.lineTo(hx, hy); g.stroke(); g.setLineDash([]);
}
function draw(){
  const dpr = window.devicePixelRatio || 1, W = cv.clientWidth, H = cv.clientHeight;
  if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)){ cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
  const g = cv.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
  rate.draw();
  const p = last, ok = p && isFinite(p.base_x);
  const cx = ok ? p.base_x : 0, cy = ok ? p.base_y : 0, scale = Math.min(W, H) / (2 * SCALE_M);
  const w2s = (x, y) => [W / 2 - (y - cy) * scale, H / 2 - (x - cx) * scale];
  const ink = RU.token("--ink", "#15202B"), muted = RU.token("--muted", "#5B6672"), rule = RU.token("--rule", "#D3DAE1");
  const sig = RU.token("--signal", "#1F5FBF"), ee = RU.token("--plot-2", "#C2410C");
  g.lineWidth = 1; g.strokeStyle = rule; g.setLineDash([2, 4]);
  for (const r of [0.5, 1.0, 1.5]){ g.beginPath(); g.arc(W / 2, H / 2, r * scale, 0, 2 * Math.PI); g.stroke(); }
  g.setLineDash([]); g.fillStyle = muted; g.font = "11px " + RU.token("--font", "sans-serif");
  if (!ok){ g.fillText("no WBC stream", 10, 18); requestAnimationFrame(draw); return; }
  g.fillText("1 m", W / 2 + 1.0 * scale * 0.707 + 3, H / 2 - 1.0 * scale * 0.707);
  if (isFinite(p.hint_x)){ g.strokeStyle = sig; g.lineWidth = 1.25; footprint(g, w2s, p.hint_x, p.hint_y, p.hint_yaw, [5, 4]); }
  if (isFinite(p.head_x)){ const [hx, hy] = w2s(p.head_x, p.head_y), r = 7;
    g.strokeStyle = muted; g.lineWidth = 1.25; g.beginPath(); g.arc(hx, hy, r, 0, 2 * Math.PI); g.stroke();
    const [nx, ny] = w2s(p.head_x + 0.12 * Math.cos(p.head_yaw), p.head_y + 0.12 * Math.sin(p.head_yaw));
    g.beginPath(); g.moveTo(hx, hy); g.lineTo(nx, ny); g.stroke(); }
  g.strokeStyle = ink; g.lineWidth = 1.5; footprint(g, w2s, p.base_x, p.base_y, p.base_yaw);
  if (isFinite(p.ee_cmd_x)){ const [ex, ey] = w2s(p.ee_cmd_x, p.ee_cmd_y);
    g.strokeStyle = ee; g.lineWidth = 1.5; g.beginPath(); g.arc(ex, ey, 6, 0, 2 * Math.PI); g.stroke();
    if (isFinite(p.ee_tgt_x)){ const [tx, ty] = w2s(p.ee_tgt_x, p.ee_tgt_y);
      g.lineWidth = 1; g.setLineDash([3, 3]); g.beginPath(); g.moveTo(ex, ey); g.lineTo(tx, ty); g.stroke(); g.setLineDash([]);
      g.lineWidth = 1.5; g.beginPath(); g.moveTo(tx - 6, ty); g.lineTo(tx + 6, ty); g.moveTo(tx, ty - 6); g.lineTo(tx, ty + 6); g.stroke(); }
    g.fillStyle = muted; g.fillText(`z ${p.ee_cmd_z.toFixed(2)} m`, ex + 9, ey + 4); }
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);
</script></body></html>
"""
