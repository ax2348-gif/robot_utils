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
.cues{display:grid;grid-template-columns:auto 1fr auto auto;gap:3px 8px;align-items:center;font-size:13px;margin-top:6px}
.cues input[type=number]{width:58px}
.cues .ru-muted{font-size:12px}
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
      <div class="ctl"><button class="ru" id="mode3">Third person</button><button class="ru" id="mode1">First person</button>
        <span class="ru-muted" id="modenote">applies from the next grip</span></div>
      <div class="ctl"><button class="ru" id="headless">Headless left stick: on</button>
        <button class="ru" id="tether" title="Engaged: the stick offset leads the robot's commanded base by at most 5 cm / 0.1 rad (the arm keeps the base's heading), so the arm does not reach ahead while the base accelerates. Applies from the next grip.">Stick tether: on</button>
        <span class="ru-muted" id="tethernote"></span></div>
      <div class="ctl" id="mirrorrow" hidden title="Which arm each hand drives: decided when the first hand engages, locked until both release. Auto mirrors when the robot faces you (headings more than 90° apart).">pairing
        <button class="ru" data-mirror="off">Off</button><button class="ru" data-mirror="auto">Auto</button><button class="ru" data-mirror="straight">Straight</button><button class="ru" data-mirror="mirror">Mirror</button>
        <span class="ru-muted" id="mirrornote"></span></div>
      <div class="ctl">base follows <button class="ru" id="hs-head">Head</button><button class="ru" id="hs-chest">Chest</button><button class="ru" id="hs-hips">Hips</button>
        <span class="ru-muted" id="hsnote"></span></div>
      <div class="ctl" title="Each engaged arm's elbow swivel: the operator's change since the grip (body tracking), sent to the WBC. On applies from the next grip, off at once.">elbow hint <button class="ru" id="eh-on">On</button><button class="ru" id="eh-off">Off</button>
        <span class="ru-muted" id="ehnote"></span></div>
      <div class="ru-muted" id="swivel" style="font-size:13px"></div>
      <div class="ctl" id="simaids" hidden><button class="ru" id="ghost">Goal ghost: on</button>
        <button class="ru" id="autopause">Auto pause: off</button>
        <button class="ru" id="hview" title="Cycle how the headset draws the sim (the sim node's /scene3d/view)">Headset view</button>
        <span class="ru-muted" id="aidsnote"></span></div>
      <div class="ctl">top speed <input type="number" id="vmax" step="0.1" min="0.1" max="2" style="width:64px"> m/s
        <input type="number" id="wmax" step="0.1" min="0.1" max="4" style="width:64px"> rad/s
        <button class="ru" id="setspeed">Set</button></div>
      <div class="ru-muted" id="capsnote"></div>
      <div class="ctl" title="Tiered recovery: a brief dropout (stale or late input, a gap, an unfocused frame, the WBC's own short dropout) holds the robot; once it clears, releasing and re-gripping resumes without Y. Longer than the grace window, or serious (reconnect, estimator reset, fault): a disarm, Y re-arms. X never resumes.">re-grip resume
        <button class="ru" id="rs-on">On</button><button class="ru" id="rs-off">Off</button>
        grace <input type="number" id="rsgrace" step="0.5" min="0" max="30" style="width:56px"> s
        <button class="ru" id="rsset">Set</button></div>
      <div class="ctl" id="rsclasses" style="font-size:13px"></div>
      <div class="ru-muted" id="reg"></div>
      <div class="steps" id="steps"></div>
      <div id="armed"></div>
      <div class="ru-muted" id="reason"></div>
      <div class="ru-muted" style="font-size:13px">Start takes the WBC lease and enables it (arm + base); nothing moves until <b>Y</b> (left controller) arms it. <b>X</b> disarms and disables (the arm floats, the base coasts); Y re-enables. Armed, the grip clutches the arm after a release. A brief dropout <b>holds</b>: release and re-grip to resume (no Y); a long or serious one disarms (Y).</div>
    </div>
    <div class="ru-card" data-panel="episode" data-role="telemetry"><b>Episode</b> <span class="ru-muted">A starts, a B tap stops (right controller)</span>
      <div class="lim" id="epline"></div>
      <div class="kv" id="episode"></div>
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
    <div class="ru-card" data-panel="haptics" data-role="control"><b>Haptics</b> <span class="ru-muted">controller vibration</span>
      <div class="ctl"><button class="ru" id="hapon">Haptics: on</button><span class="ru-muted" id="hapnote"></span></div>
      <div class="cues" id="cues"></div>
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
$("#mode3").onclick = () => call("mode/third");
let headlessOn = true, tetherOn = true;
$("#headless").onclick = () => call("headless/" + (headlessOn ? "off" : "on"));
$("#tether").onclick = () => call("tether/" + (tetherOn ? "off" : "on"));
$("#mode1").onclick = () => call("mode/first");
document.querySelectorAll("[data-mirror]").forEach(b => b.onclick = () => call("mirror/" + b.dataset.mirror));
for (const k of ["head", "chest", "hips"]) $("#hs-" + k).onclick = () => call("hint_source/" + k);
$("#eh-on").onclick = () => call("elbow_hint/on");
$("#eh-off").onclick = () => call("elbow_hint/off");
let ghostOn = true, pauseOn = false;
$("#ghost").onclick = () => call("sim/ghost/" + (ghostOn ? "off" : "on"));
$("#autopause").onclick = () => call("sim/auto_pause/" + (pauseOn ? "off" : "on"));
let simNode = null;
$("#hview").onclick = async () => {
  if (!simNode) return;
  try { err(""); const r = await fetch("/nodes/" + simNode + "/scene3d/view/next", {method: "POST"});
        const v = await r.json(); $("#hview").textContent = "Headset view: " + v.style.replace("_", " "); }
  catch(e){ err("headset view: " + e.message); }
};
$("#setspeed").onclick = async () => {
  try { err(""); await RU.api("top_speed", {method: "POST", body: {max_speed: +$("#vmax").value, max_omega: +$("#wmax").value}}); }
  catch(e){ err(e.message); }
};
$("#rs-on").onclick = () => RU.api("resume", {method: "POST", body: {enabled: true}}).catch(e => err(e.message));
$("#rs-off").onclick = () => RU.api("resume", {method: "POST", body: {enabled: false}}).catch(e => err(e.message));
$("#rsset").onclick = () => RU.api("resume", {method: "POST", body: {grace_s: +$("#rsgrace").value}}).catch(e => err(e.message));
$("#rsclasses").onclick = ev => { const b = ev.target.closest("[data-rsclass]"); if (!b) return;
  RU.api("resume", {method: "POST", body: {classes: {[b.dataset.rsclass]: !b.classList.contains("live")}}}).catch(e => err(e.message)); };
let rsGraceShown = null;
let hapOn = true, cuesBuilt = false;
const hapCfg = body => RU.api("haptics/config", {method: "POST", body}).catch(e => err(e.message));
$("#hapon").onclick = () => hapCfg({enabled: !hapOn});
function buildCues(C) {   // once: the cue rows (their state is refreshed from /state)
  $("#cues").innerHTML = Object.entries(C).map(([k, c]) => `<label title="${esc(c.doc)}"><input type="checkbox" data-cue="${k}"> ${k}</label>
    <span class="ru-muted">${c.kind === "level" ? "continuous" : "pulse"}</span>
    <input type="number" min="0" max="1" step="0.05" data-amp="${k}" title="amplitude 0..1">
    <button class="ru" data-test="${k}" title="play it now on both hands (even when off)">Test</button>`).join("");
  document.querySelectorAll("[data-cue]").forEach(b => b.onchange = () => hapCfg({cues: {[b.dataset.cue]: {enabled: b.checked}}}));
  document.querySelectorAll("[data-amp]").forEach(b => b.onchange = () => hapCfg({cues: {[b.dataset.amp]: {amplitude: +b.value}}}));
  document.querySelectorAll("[data-test]").forEach(b => b.onclick = () => RU.api("haptics/test", {method: "POST", body: {cue: b.dataset.test}}).catch(e => err(e.message)));
  cuesBuilt = true;
}
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
  $("#reg").textContent = s.config.mode !== "third" ? "first person: every grip aligns your forward to the robot's" : (T.registered ? "registered (" + T.registration + "): face where the robot faces and click the right stick to re-register" : "not registered: the first enable (Y) or a right-stick click registers (face where the robot faces)" + (T.registration ? " · " + T.registration : ""));
  headlessOn = !!s.config.headless;
  for (const [id, v] of [["#vmax", s.config.max_speed], ["#wmax", s.config.max_omega]])
    if (document.activeElement !== $(id)) $(id).value = v;
  const WC = T.wbc_caps;
  $("#capsnote").textContent = "the sticks' full deflection; Start sets the WBC's max_speed / max_omega to it and Stop restores them"
    + (WC && Object.keys(WC.asserted).length ? " · now asserted (WBC had " + Object.entries(WC.previous).map(([k, v]) => k + " " + v).join(", ") + ")" : "");
  $("#headless").textContent = "Headless left stick: " + (headlessOn ? "on" : "off");
  $("#headless").classList.toggle("live", headlessOn);
  $("#headless").disabled = s.config.mode !== "third";
  tetherOn = s.config.tether !== false;
  $("#tether").textContent = "Stick tether: " + (tetherOn ? "on" : "off");
  $("#tether").classList.toggle("live", tetherOn);
  const TT = T.tether;
  $("#tethernote").textContent = TT ? "xy " + TT.xy + " · yaw " + TT.yaw + " · lead " + (TT.lead_m * 100).toFixed(1) + " cm, " + TT.lead_rad.toFixed(2) + " rad" : "";
  const A = s.sim_aids;
  $("#simaids").hidden = !A;
  simNode = s.config.sim && !s.config.sim.includes("/") ? s.config.sim : null;
  $("#hview").hidden = !simNode;
  if (A){
    ghostOn = A.ghost; pauseOn = A.auto_pause;
    $("#ghost").textContent = "Goal ghost: " + (ghostOn ? "on" : "off"); $("#ghost").classList.toggle("live", ghostOn);
    $("#autopause").textContent = "Auto pause: " + (pauseOn ? "on" : "off"); $("#autopause").classList.toggle("live", pauseOn);
    $("#aidsnote").textContent = (pauseOn ? (A.paused ? "sim paused (idle)" : "sim time stops when neither clutched nor driving") : "")
      + (A.error ? " · " + A.error : "");
  }
  const P = T.pairing;
  $("#mirrorrow").hidden = !P;
  if (P){
    document.querySelectorAll("[data-mirror]").forEach(b => { b.classList.toggle("live", b.dataset.mirror === P.mode); b.disabled = P.locked; });
    const D = P.decision;
    $("#mirrornote").textContent = (P.pairing === "mirror" ? "mirrored: " : "straight: ")
      + Object.entries(P.hands).map(([h, e]) => h + " → " + e).join(", ")
      + (P.locked ? " · locked until both hands release" : P.mode === "auto" ? " · decided at the next grip" : "")
      + (D && P.mode !== "off" ? " · last grip: " + D.pairing + (D.heading_diff_deg === null ? "" : ", headings " + Math.abs(D.heading_diff_deg).toFixed(0) + "° apart") : "");
  }
  for (const k of ["head", "chest", "hips"]) $("#hs-" + k).classList.toggle("live", s.config.hint_source === k);
  { const hs = T.hint || {};
    $("#hsnote").textContent = !T.hint ? "from the next grip" : (hs.in_force ? "in force: " + hs.in_force : (hs.engaged_source ? "lost" : "from the next grip"))
      + (hs.confidence === null || hs.confidence === undefined ? "" : " · conf " + f(hs.confidence, 2))
      + (hs.why_not && hs.selected !== "head" ? " · " + hs.why_not : ""); }
  $("#eh-on").classList.toggle("live", !!s.config.elbow_hint); $("#eh-off").classList.toggle("live", !s.config.elbow_hint);
  $("#ehnote").textContent = s.config.elbow_hint && s.config.body === "off" ? "needs body tracking (--body)" : "";
  { const SW = T.swivel || {}, deg = x => x === null || x === undefined ? "–" : (x * 180 / Math.PI).toFixed(1) + "°";
    $("#swivel").innerHTML = Object.entries(SW).map(([ee, v]) => `<div>${esc(ee)} <span title="the operator's hand driving it">(${esc(v.hand)})</span>: `
      + (v.valid ? `elbow ${deg(v.elbow)} · conf ${f(v.conf, 2)}` : "input " + esc(v.reason || "not valid"))
      + (v.engaged ? ` · Δψ ${deg(v.dpsi)}${v.straight ? " (arm straight: held)" : ""}` : "")
      + (v.sending ? " · <b>sent</b>" : v.pending ? " · from the next grip" : v.enabled && v.engaged ? " · not sent" : "")
      + (v.robot_swivel === null || v.robot_swivel === undefined ? "" : ` · robot ψ ${deg(v.robot_swivel)}`)
      + (v.robot_swivel_w === null || v.robot_swivel_w === undefined ? "" : ` · hint in force ${f(v.robot_swivel_w, 2)}`)
      + "</div>").join(""); }
  $("#mode3").classList.toggle("live", s.config.mode === "third"); $("#mode1").classList.toggle("live", s.config.mode === "first");
  $("#sending").textContent = running ? (T.sending ? `sending · ${T.sent ? T.sent.target : 0} targets` : "silent (holding)") : (T.stop_reason ? "stopped: " + T.stop_reason : "");
  $("#steps").innerHTML = STEPS.map(([k, label, tone]) => `<span class="${s.state === k ? "on " + tone : ""}">${label}</span>`).join("");
  const why = running ? (T.reason || "") : (s.error || "");
  $("#reason").textContent = why ? "last: " + why : (running && T.transport ? "datagrams: " + T.transport.split(" ")[0] + " socket" : "");
  const TT = T.tier || {}, HD = TT.hold;
  $("#armed").className = "lim" + (running && T.armed ? " on" : "");
  $("#armed").textContent = !running ? "" : T.armed ? `ARMED (press ${T.arm_seq})${TT.resumes ? ` · ${TT.resumes} resumed by re-grip` : ""}`
    : TT.tier === "hold" && HD ? `HOLD — ${HD.cause}: ` + (HD.resumable ? "release and re-grip to resume" : `waiting for it to clear (${f(HD.cause_present_s, 1)} / ${f(HD.grace_s, 1)} s)`)
      + (HD.last_refused ? ` · last re-grip refused: ${HD.last_refused}` : "")
    : `DISARMED — press Y${T.disarm_reason ? " · " + T.disarm_reason : ""}${T.disabled_by_x ? " · WBC disabled by X (Y re-enables)" : ""}`;
  { const R = s.config.resume || {};
    $("#rs-on").classList.toggle("live", !!R.enabled); $("#rs-off").classList.toggle("live", !R.enabled);
    if (R.grace_s !== rsGraceShown && document.activeElement !== $("#rsgrace")) { $("#rsgrace").value = R.grace_s; rsGraceShown = R.grace_s; }
    $("#rsclasses").innerHTML = "holds on: " + Object.entries(R.classes || {}).map(([k, v]) =>
      `<button class="ru${v ? " live" : ""}" data-rsclass="${esc(k)}">${esc(k.replace("_", " "))}</button>`).join(""); }
  $("#reason").title = T.transport || "";
  // headset link
  function bodyText(b) {   // body tracking (quest_env BODY_TRACKING.md): information only
    if (!b || (!b.present && !b.frames)) return "off";
    if (!b.present) return `none for ${b.age === null ? "–" : b.age.toFixed(1) + " s"}`;
    if (!b.available) return "unavailable: " + esc(b.reason || "");
    if (!b.active) return "not active";
    return `${b.tracked}/${b.sent} tracked · ${esc(b.joint_set)} · ${esc(b.fidelity || "fidelity –")} · conf ${f(b.confidence, 2)}`
      + (b.bad ? ` · ${b.bad} bad` : "");
  }
  const age = H.age === null ? "–" : (H.age < 1 ? (H.age * 1e3).toFixed(0) + " ms" : H.age.toFixed(1) + " s");
  const link = [
    ["stream", `${esc(s.config.quest_host)}:${s.config.quest_port} · ${H.connected ? "connected" : "not connected"}`],
    ["rate", `${f(H.rate_hz, 0)} frames/s · age ${age}`],
    ["latency", T.input ? `${T.input.latency_ms === null ? "–" : f(T.input.latency_ms, 0) + " ms"} · ${f(T.input.rate_hz, 0)} Hz (0.5 s)${T.input.fresh ? "" : " · stale: " + esc(T.input.why)}` : "–"],
    ["focus", H.focused === null ? "–" : (H.focused ? "focused" : "not focused")],
    ["battery", esc(H.battery ? H.battery.text : "not reported")],
    ["XR session", esc(H.xr_state || "–")],
    ["body", bodyText(H.body)],
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
  const hands = H.hands || {}, TH = (s.teleop || {}).hands || {};
  $("#hands").innerHTML = ["left", "right"].map(k => { const h = hands[k] || {};
    const g = h.gripper === null || h.gripper === undefined ? null : Math.max(0, Math.min(1, h.gripper));
    return `<div class="hand"><div class="clutch ${h.valid ? "valid" : ""} ${h.engaged ? "engaged" : ""}" title="${h.engaged ? "clutch engaged" : "clutch released"}"></div>
      <div style="flex:1"><div>${k}${s.config.hands ? (s.config.hands[k] ? ` <span class="ru-muted">(${esc(TH[k] ? TH[k].ee : s.config.hands[k])}${TH[k] && TH[k].mirrored ? ", mirrored" : ""}${TH[k] && TH[k].base ? ", base" : ""})</span>` : "") : k === s.config.hand ? ` <span class="ru-muted">(${s.config.ee === "arm" ? "arm" : s.config.ee})</span>` : ""}</div>
      <div class="ru-muted" style="font-size:12px">${!h.valid ? "not tracked" : h.engaged ? "engaged" : h.needs_release ? "release first" : "released"}</div>
      <div class="grip" title="trigger ${g === null ? "–" : g.toFixed(2)}"><i style="left:calc(${g === null ? 0 : g * 100}% - 1px);${g === null ? "display:none" : ""}"></i></div></div></div>`; }).join("");
  // haptics
  { const HP = s.haptics || {}, HS = HP.headset || {};
    if (!cuesBuilt && HP.cues) buildCues(HP.cues);
    hapOn = !!HP.enabled;
    $("#hapon").textContent = "Haptics: " + (hapOn ? "on" : "off"); $("#hapon").classList.toggle("live", hapOn);
    for (const [k, c] of Object.entries(HP.cues || {})) {
      const cb = document.querySelector(`[data-cue="${k}"]`), am = document.querySelector(`[data-amp="${k}"]`);
      if (cb) { cb.checked = c.enabled; cb.disabled = !hapOn; }
      if (am && document.activeElement !== am) am.value = c.amplitude;
    }
    const A = HP.ack_ms, lv = HP.levels || {};
    $("#hapnote").textContent = (HP.readers ? "the headset reads them" + (HS.capable ? "" : " (no ack in its frames)")
                                 : H.connected ? "the headset does not read them (an older build, or --ar off)" : "no headset")
      + (A ? ` · round trip ${f(A.median, 0)} ms (max ${f(A.max, 0)})` : "")
      + (lv.left > 0 || lv.right > 0 ? ` · buzzing: ${["left", "right"].filter(h => lv[h] > 0).map(h => h + " (" + HP.level_cue[h] + ")").join(", ")}` : "");
  }
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
  // episode: the open one, the task, the last outcome and event (the sim also records every teleop run itself: recording.py)
  const E = s.episodes || {}, ev = s.last_event, L = s.eventlog || {};
  $("#epline").className = "lim" + (E.state === "recording" ? " on" : "");
  $("#epline").textContent = E.state === "recording" ? `episode ${E.id} · ${f(E.elapsed, 0)} s` : E.state === "resetting" ? `resetting the sim for ${E.pending}` : "no episode";
  const d = (ev && ev.detail) || {};
  const what = [d.button ? d.button + (d.pressed ? " down" : " up") : "", d.action || "", d.reason || ""].filter(Boolean).join(" ");
  const evText = ev ? `#${ev.seq} ${ev.kind}${ev.episode ? " (" + ev.episode + ")" : ""}${what ? " · " + what : ""}` : "–";
  const ekv = [
    ["task", esc(E.task || "–")],
    ["last", E.last ? esc(`${E.last.id} ${E.last.outcome}${E.last.reason ? ": " + E.last.reason : ""}`) : "–"],
    ["counts", `${E.counts ? E.counts.kept : 0} kept · ${E.counts ? E.counts.aborted : 0} aborted`],
    ["last event", esc(evText)],
    ["event log", `${L.dir ? "on disk" : "memory only"} · seq ${L.seq}` + (L.dropped || L.write_errors ? ` · ${L.dropped} lost, ${L.write_errors} write errors` : "")],
  ];
  if (E.refused && E.state !== "recording") ekv.push(["A refused", esc(E.refused)]);
  const R = s.recording || {};
  if (R.enabled) {   // the sim records every teleop run (recording.py)
    const st = R.streams || {}, down = Object.entries(st).filter(([, v]) => !v.connected).map(([k]) => k);
    const sc = (R.scenes || []).reduce((n, x) => n + (x.saved || []).length, 0);
    const rec = R.active ? `recording · ${f((R.bytes || 0) / 1e6, 0)} MB · ${Object.keys(st).length} streams${down.length ? " (" + down.length + " down)" : ""} · ${sc} scenes`
      : R.dir ? (R.exit_code === 0 ? "finished" : R.exit_code === null ? "starting" : "stopped") : "starts with teleop";
    ekv.push(["recording", esc(rec) + (R.dir ? `<div class="ru-muted" style="font-size:12px">${esc(R.dir)}</div>` : "")]);
    if (R.error) ekv.push(["recorder", esc(R.error)]);
  }
  $("#episode").innerHTML = ekv.map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join("");
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
