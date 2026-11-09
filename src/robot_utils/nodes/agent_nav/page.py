"""agent_nav page: the agent <-> robot dialogue, live view, controls and a command line for humans.
Relative URLs only. Agent-supplied text is rendered as text (never HTML)."""

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Agent navigation</title>
<link rel="stylesheet" href="common/ui.css">
<style>
.ru-wrap{max-width:1500px}
.layout{display:grid;grid-template-columns:minmax(0,1.1fr) minmax(0,1fr);gap:12px;align-items:start}
@media (max-width:1000px){.layout{grid-template-columns:1fr}}
#dialogue{height:calc(100vh - 230px);min-height:420px;overflow:auto;padding:6px;display:flex;flex-direction:column;gap:8px}
.turn{display:flex;gap:8px;align-items:flex-start}
.turn.robot{flex-direction:row-reverse}
.av{flex:0 0 34px;height:34px;border-radius:50%;display:flex;align-items:center;justify-content:center;font-size:20px;background:var(--bg);border:1px solid var(--line);overflow:hidden}
.av img{width:100%;height:100%;object-fit:cover}
.bub{max-width:86%;border:1px solid var(--line);border-radius:10px;padding:6px 9px;background:var(--card,#fff);overflow-wrap:anywhere}
.turn.robot .bub{background:var(--bg)}
.bub.warn{border-color:#ef4444}
.meta{font-size:11px;color:var(--muted)}
.call{font-family:ui-monospace,monospace;font-size:12px;font-weight:600}
.cmt{font-style:italic;margin:2px 0}
.kv{font-family:ui-monospace,monospace;font-size:12px;white-space:pre-wrap;margin:2px 0}
.wl{color:#ef4444;font-size:12.5px;font-weight:600}
.thumbs{display:flex;gap:6px;flex-wrap:wrap;margin-top:4px}
.thumbs img{height:110px;border-radius:4px;cursor:zoom-in;border:1px solid var(--line)}
details summary{cursor:pointer;font-size:11px;color:var(--muted)}
#cmd{flex:1;font-family:ui-monospace,monospace}
.live img{width:100%;border-radius:6px;display:block;background:#111}
.row2{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.ctl{display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin:4px 0}
#zoom{position:fixed;inset:0;background:rgba(0,0,0,.85);display:none;align-items:center;justify-content:center;z-index:9}
#zoom img{max-width:96vw;max-height:94vh}
.sbar{display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin:4px 0 6px}
.sbar .info{flex:1;min-width:180px;font-size:13px}
.banner{background:#f59e0b22;border:1px solid #f59e0b;border-radius:6px;padding:4px 8px;margin-bottom:6px;font-size:13px;display:none}
#sessions{display:none;max-height:300px;overflow:auto;margin-bottom:6px}
#sessions table{width:100%;border-collapse:collapse;font-size:13px}
#sessions td,#sessions th{padding:3px 6px;border-bottom:1px solid var(--line);text-align:left}
</style></head><body><div class="ru-wrap">
<div class="ru-row" style="justify-content:space-between">
  <h2 style="margin:6px 0">Agent navigation <span id="who" class="ru-pill">no session</span></h2>
  <div class="ru-row"><span id="status" class="ru-muted"></span><a class="ru-muted" href="guide.md">guide.md</a><a class="ru-muted" href="docs">API docs</a></div>
</div>
<div id="err" class="ru-err"></div>
<div class="layout">
  <div class="ru-card">
    <div class="sbar"><span class="info" id="sinfo">no session</span>
      <button class="ru" id="snew" title="Open a session as the human operator">New session</button>
      <button class="ru" id="send_s" title="End the live session (disables motion)">End session</button>
      <button class="ru" id="sclear" title="Hide the turns shown so far (nothing is deleted)">Clear view</button>
      <a class="ru" id="sexport" href="#" title="Download the session: log.jsonl, thumbnails, metadata">Export</a>
      <button class="ru" id="slist">Sessions ▾</button></div>
    <div id="sessions" class="ru-card"></div>
    <div class="banner" id="banner"></div>
    <div id="dialogue"></div>
    <div class="ctl"><input id="cmd" placeholder="command as a human, e.g. look grid · region box 400 20 700 160 · goal 0.5 0 90d · help">
      <button class="ru" id="send">Send</button></div>
  </div>
  <div>
    <div class="ru-card">
      <div class="ctl">
        <button class="ru" id="enable">Enable</button><button class="ru" id="disable">Disable</button>
        <button class="ru" id="stop">Stop</button><button class="ru" id="release">Release hold</button>
        <button class="ru ru-estop" id="estop">E-STOP</button>
        <label>profile <select id="profile"></select></label>
        <label><input type="checkbox" id="reflex" checked> stop reflex</label>
      </div>
      <div class="kv" id="state"></div>
    </div>
    <div class="ru-card live"><b>Latest view</b> <span class="ru-muted" id="snapinfo"></span>
      <div class="row2"><img id="liveimg" alt=""><img id="livebev" alt=""></div>
    </div>
  </div>
</div>
</div>
<div id="zoom"><img alt=""></div>
<script src="common/ui.js"></script>
<script>
const $ = s => document.querySelector(s);
const err = m => $("#err").textContent = m || "";
const H = {"X-Operator": "human"};
const human = async (path, body) => { try { err(""); return await RU.api(path, {method: "POST", headers: H, body: body || {}}); } catch(e){ err(e.message); } };
$("#enable").onclick = () => human("enable");
$("#disable").onclick = () => human("disable");
$("#stop").onclick = () => human("stop");
$("#release").onclick = () => human("release");
$("#estop").onclick = () => human("estop", {reason: "page button"});
$("#reflex").onchange = e => human("reflex?enabled=" + e.target.checked);
$("#profile").onchange = e => human("profile/" + e.target.value);
async function sendCmd(){ const t = $("#cmd").value.trim(); if (!t) return;
  try { err(""); await RU.api("chat", {method: "POST", headers: H, body: {text: t}}); $("#cmd").value = ""; } catch(e){ err(e.message); } }
$("#send").onclick = sendCmd; $("#cmd").addEventListener("keydown", e => { if (e.key === "Enter") sendCmd(); });
$("#zoom").onclick = () => $("#zoom").style.display = "none";
function zoom(src){ $("#zoom img").src = src; $("#zoom").style.display = "flex"; }

let session = null, last = 0, viewing = null, hideBefore = 0;
function fmtTime(w){ return w ? new Date(w * 1000).toLocaleString() : "–"; }
function sessionInfo(){
  const s = session;
  $("#sinfo").textContent = s ? `live: ${s.avatar && !s.avatar.startsWith("data:") ? s.avatar + " " : ""}${s.agent} · ${s.id} · started ${new Date(s.started * 1000).toLocaleTimeString()}` : "no live session";
  const ex = $("#sexport"), sid = viewing ? viewing.id : s && s.id;
  ex.href = sid ? "sessions/" + sid + "/export.zip" : "#"; ex.style.opacity = sid ? 1 : 0.4;
}
function resetDialogue(){ $("#dialogue").innerHTML = ""; }
$("#snew").onclick = async () => { const name = prompt("Session name (who is driving)?", "operator"); if (name === null) return;
  await human("session", {agent: name || "operator", avatar: "🧑"}); };
$("#send_s").onclick = async () => { if (!confirm("End the live session? (motion is disabled)")) return;
  try { err(""); await RU.api("session", {method: "DELETE", headers: H}); } catch(e){ err(e.message); } };
$("#sclear").onclick = () => { hideBefore = last; resetDialogue(); };
$("#slist").onclick = () => { const p = $("#sessions"); p.style.display = p.style.display === "block" ? "none" : "block"; if (p.style.display === "block") listSessions(); };
async function listSessions(){
  const rows = await RU.api("sessions"), p = $("#sessions"); p.innerHTML = "";
  const t = el("table"), hr = el("tr"); ["", "session", "agent", "started", "turns", "size", ""].forEach(h => hr.appendChild(el("th", "", h))); t.appendChild(hr);
  for (const s of rows){ const r = el("tr");
    r.appendChild(el("td", "", s.live ? "● live" : ""));
    r.appendChild(el("td", "", s.id)); r.appendChild(el("td", "", (s.avatar && !s.avatar.startsWith("data:") ? s.avatar + " " : "") + s.agent));
    r.appendChild(el("td", "", fmtTime(s.started))); r.appendChild(el("td", "", s.turns ?? "?")); r.appendChild(el("td", "", ((s.bytes || 0) / 1024).toFixed(0) + " KB"));
    const a = el("td"); const v = el("button", "ru", s.live ? "Live" : "View"); v.onclick = () => s.live ? backToLive() : viewSession(s.id); a.appendChild(v);
    const x = el("a", "ru", "Export"); x.href = "sessions/" + s.id + "/export.zip"; a.appendChild(x);
    if (!s.live){ const d = el("button", "ru", "Delete"); d.onclick = async () => { if (!confirm("Delete session " + s.id + " permanently (transcript and thumbnails)? This cannot be undone.")) return;
      try { await RU.api("sessions/" + s.id, {method: "DELETE", headers: H}); listSessions(); } catch(e){ err(e.message); } }; a.appendChild(d); }
    r.appendChild(a); t.appendChild(r); }
  if (!rows.length) p.appendChild(el("div", "ru-muted", "no stored sessions")); else p.appendChild(t);
}
async function viewSession(sid){
  const r = await RU.api("sessions/" + sid); viewing = r.session; resetDialogue();
  const b = $("#banner"); b.style.display = "block"; b.textContent = "";
  b.appendChild(el("span", "", `Viewing stored session ${sid} (${r.turns.length} turns, thumbnails only). `));
  const back = el("a", "", "Back to live"); back.href = "#"; back.onclick = e => { e.preventDefault(); backToLive(); }; b.appendChild(back);
  const keep = session; session = r.session; r.turns.forEach(t => renderTurn(t, true)); session = keep; sessionInfo();
  $("#sessions").style.display = "none";
}
function backToLive(){ viewing = null; $("#banner").style.display = "none"; resetDialogue(); last = 0; $("#sessions").style.display = "none"; sessionInfo(); pull(); }
function el(tag, cls, text){ const e = document.createElement(tag); if (cls) e.className = cls; if (text !== undefined) e.textContent = text; return e; }
function avatar(actor){
  const a = el("div", "av");
  if (actor === "robot") a.textContent = "🛞";
  else if (actor === "human") a.textContent = "🧑";
  else { const v = (session && session.avatar) || "🤖";
    if (v.startsWith("data:image/")) { const i = document.createElement("img"); i.src = v; a.appendChild(i); } else a.textContent = v; }
  return a;
}
const SUM = {look: ["snapshot", "pose_world", "depth_fraction", "mode", "moving"],
  query_region: ["query", "with_depth", "base", "world", "range_m", "touches_image_edge"],
  query_line: ["query", "length_m", "tilt_from_horizontal_deg", "rms_m", "inlier_fraction", "base", "world"],
  query_plane: ["query", "orientation", "width_m", "height_m", "rms_m", "inlier_fraction", "base", "world", "touches_image_edge"],
  plan_check: ["goal", "check"], plan_approach: ["proposal", "goal"], goal: ["goal_id", "goal", "check"],
  enable: ["profile", "caps"], session: ["session", "profile"]};
function short(o, keys){ const out = {}; for (const k of keys) if (o[k] !== undefined) out[k] = o[k]; return out; }
function fmt(o){ return JSON.stringify(o, (k, v) => v === null || v === false && k === "grid" ? undefined : typeof v === "number" ? Math.round(v * 1000) / 1000 : v, 1)
  .replace(/\n\s*/g, " ").replace(/^\{ ?|\}$/g, ""); }
function renderTurn(t, stored){
  const box = $("#dialogue");
  const caller = t.actor === "robot" ? null : t.actor;
  const sid = t.session || (session && session.id);
  const imgs = (t.thumbs && t.thumbs.length && sid) ? t.thumbs.map(f => ["sessions/" + sid + "/thumbs/" + f, null]) : [];
  if (!stored) (t.images || []).forEach((n, i) => { if (imgs[i]) imgs[i][1] = "images/" + n; else imgs.push(["images/" + n, "images/" + n]); });
  const time = new Date(t.wall * 1000).toLocaleTimeString();
  if (caller){                                            // the call
    const row = el("div", "turn " + caller), b = el("div", "bub");
    row.appendChild(avatar(caller));
    b.appendChild(el("div", "meta", (caller === "human" ? "human" : (session && session.agent) || "agent") + " · " + time + " · #" + t.n));
    b.appendChild(el("div", "call", t.kind + (Object.keys(t.request || {}).length ? " " + fmt(t.request) : "")));
    if (t.comment) b.appendChild(el("div", "cmt", "“" + t.comment + "”"));
    row.appendChild(b); box.appendChild(row);
  }
  const r = t.reply || {}, row = el("div", "turn robot"), b = el("div", "bub" + (t.level === "warn" ? " warn" : ""));
  row.appendChild(avatar("robot"));
  b.appendChild(el("div", "meta", "robot · " + time + (caller ? "" : " · #" + t.n)));
  if (r.message) b.appendChild(el("div", "", r.message));
  if (r.error) b.appendChild(el("div", "wl", "✗ " + r.error));
  const warns = (r.warnings || []).concat(r.check && r.check.warnings && !r.warnings ? r.check.warnings : []).concat(r.warning ? [r.warning] : []);
  warns.forEach(w => b.appendChild(el("div", "wl", "⚠ " + w)));
  if (r.help) b.appendChild(el("div", "kv", r.help));
  const keys = SUM[t.kind];
  if (keys && !r.error) b.appendChild(el("div", "kv", fmt(short(r, keys))));
  if (imgs.length){ const th = el("div", "thumbs");
    imgs.forEach(([src, full]) => { const i = document.createElement("img"); i.src = src; i.loading = "lazy";
      i.onclick = () => zoom(full || src); th.appendChild(i); }); b.appendChild(th); }
  const d = document.createElement("details"); d.appendChild(el("summary", "", "full reply"));
  d.appendChild(el("div", "kv", JSON.stringify(r, null, 1))); b.appendChild(d);
  row.appendChild(b); box.appendChild(row);
  if (stored) return;
  if (t.kind === "look" && r.image){ $("#liveimg").src = r.image; $("#livebev").src = r.bev;
    $("#snapinfo").textContent = r.snapshot + " · pose " + fmt(r.pose_world || "–"); }
  else if (r.overlay){ $("#liveimg").src = r.overlay; if (r.bev) $("#livebev").src = r.bev; }
  else if (r.bev){ $("#livebev").src = r.bev; }
}
async function pull(){
  if (viewing) return;
  const r = await RU.api("log?since=" + last);
  const sid = r.session ? r.session.id : null;
  if ((session ? session.id : null) !== sid && sid){ session = r.session; resetDialogue(); }
  else if (!sid) session = null;
  sessionInfo();
  const box = $("#dialogue"), atBottom = box.scrollHeight - box.scrollTop - box.clientHeight < 60;
  for (const t of r.turns){
    if (t.n <= last) continue;
    last = t.n;
    // the live view shows the live session's turns (and, without a session, the unattached ones)
    if (t.n <= hideBefore || (sid ? t.session !== sid : t.session)) continue;
    renderTurn(t);
  }
  if (atBottom) box.scrollTop = box.scrollHeight;
}
RU.api("state").then(s => { const p = $("#profile");
  for (const n of Object.keys(s.profiles)){ const o = el("option", "", n); o.value = n; p.appendChild(o); } p.value = s.profile;
  $("#reflex").checked = s.reflex.enabled; });
RU.sse("events", s => {
  $("#who").textContent = s.session ? (s.session.avatar && !s.session.avatar.startsWith("data:") ? s.session.avatar + " " : "") + s.session.agent : "no session";
  const L = s.latest || {}, f = v => v === null || v === undefined ? "–" : Number(v).toFixed(3);
  $("#status").textContent = `servo ${s.servo || "?"}${s.enabled_by ? " · enabled by " + s.enabled_by : ""}${s.latch ? " · HELD by human" : ""}`;
  $("#state").textContent = `pose ${s.pose_world ? s.pose_world.map(f).join(", ") : "–"}   twist ${f(L.vx)}, ${f(L.vy)}, ${f(L.omega)}` +
    (s.reflex ? `\n⚠ ${s.reflex}` : "");
  if (s.turns > last || ((s.session && s.session.id) !== (session && session.id))) pull();
});
pull();
</script></body></html>
"""
