"""The sim node's page (served at the node root). Relative URLs only.

The 3D view uses three.js from a pinned CDN version (THREE_VERSION, cdn.jsdelivr.net): the sim runs on
development machines and erid, never on the robot, so the browser has the internet. Without it the view says so
and the rest of the page works. Geometry from ``viz/model`` + ``viz/meshes.bin``, poses and markers from the
``viz/stream`` SSE (30 Hz); the status from ``events``.
"""

THREE_VERSION = "0.170.0"

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Sim world</title>
<link rel="stylesheet" href="common/ui.css">
<script type="importmap">{"imports": {"three": "https://cdn.jsdelivr.net/npm/three@__THREE__/build/three.module.js",
  "three/addons/": "https://cdn.jsdelivr.net/npm/three@__THREE__/examples/jsm/"}}</script>
<style>
.ru-wrap{max-width:1440px}
.top{display:grid;grid-template-columns:minmax(0,1fr) 360px;gap:12px;align-items:start}
@media (max-width:980px){.top{grid-template-columns:1fr}}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:12px}
.kv{display:grid;grid-template-columns:auto 1fr;gap:2px 10px;font-variant-numeric:tabular-nums;font-size:13px}
.kv span:nth-child(odd){color:var(--muted)}
#view{position:relative;height:min(68vh,640px);min-height:320px;border:1px solid var(--rule);border-radius:var(--radius);overflow:hidden;background:var(--ground)}
#view canvas{display:block}
#view .bar{position:absolute;top:8px;left:8px;right:8px;display:flex;gap:6px;flex-wrap:wrap;align-items:center;pointer-events:none}
#view .bar > *{pointer-events:auto}
#view .bar button.ru{min-height:26px;padding:2px 9px;font-size:12px;background:var(--panel)}
#view .bar button.ru.on{border-color:var(--signal);color:var(--signal)}
#view .note{position:absolute;left:10px;bottom:8px;font-size:12px;color:var(--muted);font-variant-numeric:tabular-nums;pointer-events:none}
#view .msg{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;color:var(--muted);text-align:center;padding:20px}
.lbl{font:11px/1.2 var(--mono);color:var(--ink);background:color-mix(in srgb,var(--panel) 80%,transparent);padding:1px 4px;border-radius:3px;white-space:nowrap}
.clock{font:600 22px/1.2 var(--mono);font-variant-numeric:tabular-nums}
.cams{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:8px}
.cams img{width:100%;display:block;border-radius:4px;background:#000;aspect-ratio:4/3;object-fit:contain}
input[type=number]{width:66px}
.priv{border:1px dashed var(--caution)}
.priv .tag{display:inline-block;font-size:11px;font-weight:700;letter-spacing:.06em;color:var(--caution);border:1px solid var(--caution);border-radius:3px;padding:0 5px;margin-left:6px}
#events{max-height:160px;overflow:auto;font-size:12px}
table.ru-table td,table.ru-table th{font-size:12px}
.sw{display:inline-block;width:10px;height:10px;border-radius:2px;vertical-align:middle;margin-right:4px;border:1px solid var(--rule)}
</style><script src="../../themes/theme.js"></script>
</head><body data-page="sim"><div id="band"></div><div class="ru-wrap">
<div class="ru-row ru-head">
  <h2 style="margin:6px 0">Sim world <span id="state" class="ru-pill">…</span></h2>
  <div class="ru-row"><span class="ru-muted">simulation: no lease, no e-stop here (the nodes attached to it have theirs)</span>
    <a class="ru-muted" href="docs">API docs</a></div>
</div>
<div id="err" class="ru-err"></div>
<div class="top ru-body">
  <div data-panel="view" data-role="view">
    <div id="view"><div class="msg" id="viewmsg">loading the 3D view…</div>
      <div class="bar">
        <button class="ru" id="v3d" title="Orbit view (double-click the view)">3D</button>
        <button class="ru" id="vtop" title="Top view">Top</button>
        <button class="ru on" id="vfollow" title="Keep the robot centred">Follow</button>
        <button class="ru" id="vlines" title="Edges only">Lines</button>
        <button class="ru on" id="vcoll" title="Camera frustums and the base trail">Frustums + trail</button>
        <button class="ru" id="vclear" title="Clear the base trail">Clear trail</button>
      </div>
      <div class="note" id="vnote"></div></div>
  </div>
  <div class="ru-side">
    <div class="ru-card" data-panel="time" data-role="control"><b>Time</b>
      <div class="ru-row ru-spread"><span class="clock" id="clock">–</span><span id="rtfm" class="ru-muted ru-num"></span></div>
      <div class="ru-row" style="margin-top:6px"><button class="ru primary" id="run">Run</button><button class="ru" id="pause">Pause</button>
        rtf <input type="number" id="rtf" value="1" step="0.1" min="0.05" max="100"></div>
      <div class="ru-row" style="margin-top:6px"><button class="ru" id="step">Step</button><input type="number" id="stepms" value="100" min="1"> ms
        <button class="ru" id="reset" title="Time 0, robot at its start pose, objects back, commands dropped">Reset world</button></div>
      <div class="kv" id="clockkv" style="margin-top:6px"></div>
      <div class="ru-row" style="margin-top:6px">headset view <button class="ru" id="hview"
        title="Cycle how the headset draws the scene: flat, flat + edges, hidden-line, wire">–</button>
        <button class="ru" id="hrole" title="Robot geometry: the URDF's visual meshes, or what collides">–</button></div>
    </div>
    <div class="ru-card" data-panel="robot" data-role="telemetry"><b>Robot</b><div class="kv" id="robot"></div>
      <table class="ru-table" id="mods"></table></div>
  </div>
</div>
<div class="grid">
  <div class="ru-card" data-panel="scene" data-role="control"><b>Scene</b>
    <table class="ru-table" id="objs"></table>
    <div class="ru-row" style="margin-top:8px">name <input id="oname" value="box1" style="width:70px">
      <select id="oshape"><option>box</option><option>cylinder</option></select>
      size <input type="number" id="osize" value="0.1" step="0.01" min="0.01" title="m: box edge / cylinder diameter and length">
      kg <input type="number" id="omass" value="0.5" step="0.1" min="0.01"></div>
    <div class="ru-row" style="margin-top:4px">x <input type="number" id="ox" value="0.6" step="0.05"> y <input type="number" id="oy" value="0" step="0.05">
      z <input type="number" id="oz" value="0.1" step="0.05">
      <button class="ru" id="add">Add</button><button class="ru" id="move" title="Teleport the named object to x, y, z">Move</button></div>
    <div class="ru-row" style="margin-top:4px"><span class="ru-muted">scene file</span><input id="sname" value="scene1" style="width:80px">
      <button class="ru" id="ssave">Save</button><button class="ru" id="sload">Load</button>
      <button class="ru" id="sstart" title="Reset the world to the scene it started with (the default tabletop)">Start scene</button></div>
  </div>
  <div class="ru-card priv" data-panel="physics" data-role="control"><b>Privileged physics</b><span class="tag">SIM-ONLY</span>
    <div class="ru-muted" style="font-size:12px;margin:2px 0 6px">Never exposed by a hardware node.</div>
    <div class="ru-row">push <select id="pbody"><option value="base">base</option></select>
      x <input type="number" id="fx" value="20"> y <input type="number" id="fy" value="0"> z <input type="number" id="fz" value="0"> N,
      <input type="number" id="fdur" value="0.2" step="0.1" min="0.01"> s <button class="ru" id="push">Push</button></div>
    <div class="ru-row" style="margin-top:6px">teleport base x <input type="number" id="tx" value="0" step="0.1"> y <input type="number" id="ty" value="0" step="0.1">
      yaw° <input type="number" id="tyaw" value="0" step="15"> <button class="ru" id="tele">Teleport</button></div>
  </div>
  <div class="ru-card" data-panel="markers" data-role="telemetry"><b>Markers</b> <span class="ru-muted">(POST markers: other nodes and clients draw here)</span>
    <table class="ru-table" id="mks"></table>
    <div class="ru-row" style="margin-top:6px"><button class="ru" id="mkclear">Clear all</button>
      <button class="ru" id="mkdemo" title="A frame 10 cm in front of the gripper, for 30 s">Demo marker</button></div>
  </div>
</div>
<div class="ru-card" data-panel="cameras" data-role="view"><div class="ru-row ru-spread"><b>Cameras</b>
  <div class="ru-row"><button class="ru" id="snap">Refresh</button>
  <label class="ru-muted"><input type="checkbox" id="camlive"> every <input type="number" id="camperiod" value="2" min="0.2" step="0.2" style="width:50px"> s</label></div></div>
  <div class="ru-muted" style="font-size:12px;margin-bottom:6px">Rendered on request (each refresh costs the physics thread a little time: rtf drops to ~0.9 at 1 s); the camera nodes stream their own.</div>
  <div class="cams" id="cams"></div></div>
<div class="ru-card" data-panel="events" data-role="log"><b>Events</b><div id="events"></div></div>
</div>
<script src="common/ui.js"></script>
<script>
const $ = s => document.querySelector(s);
const f3 = v => (v ?? 0).toFixed(3);
const esc = s => String(s).replace(/[&<>"]/g, c => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;"})[c]);
async function api(method, path, body) {
  const r = await fetch(path, {method, headers: {"Content-Type": "application/json"}, body: body === undefined ? undefined : JSON.stringify(body)});
  const j = await r.json().catch(() => ({}));
  $("#err").textContent = r.ok ? "" : (j.detail ? (typeof j.detail === "string" ? j.detail : JSON.stringify(j.detail)) : r.status);
  return j;
}
function kv(el, rows) { el.innerHTML = rows.map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join(""); }
let cams = [], objNames = "";
function render(s) {
  if (s.view){
    $("#hview").textContent = s.view.style.replace("_", " ");
    hrole = s.view.role;
    $("#hrole").textContent = {auto: "robot meshes", visual: "visual everywhere", collision: "collision everywhere"}[hrole];
  }
  $("#state").textContent = s.state;
  $("#state").className = "ru-pill " + (s.state === "running" ? "ru-ok" : "ru-warn");
  kv($("#clockkv"), [["rtf", `${s.rtf} (measured ${s.rtf_measured})`], ["steps/s", s.step_rate], ["fell behind", s.behind],
                     ["model", esc(s.model.source || "")],
                     ["cameras", Object.entries(s.model.camera_serials || {}).map(([n, v]) => {
                        const m = (s.model.camera_mounts || {})[n];
                        const how = m && (m.ground || m.yaw) ? "calibrated" + (m.ground ? " ground " + esc(m.ground) : "") + (m.yaw ? " yaw " + esc(m.yaw) : "") : '<span class="ru-warn">nominal</span>';
                        return `${esc(n)} ${esc(v)}: ${how}`; }).join("<br>") || "–"], ["fidelity", `base ${esc(s.fidelity.base)}; arm ${esc(s.fidelity.arm)}`]]);
  const b = s.base, a = s.arm;
  kv($("#robot"), [["base xyz", b.xyz.map(f3).join(", ") + " m"], ["yaw", (b.rpy[2] * 180 / Math.PI).toFixed(1) + "°"],
                    ["twist", b.twist.map(f3).join(", ")],
                    ["arm", a ? `${a.enabled ? '<span class="ru-ok">motors on</span>' : "off"}${a.fault ? ' <span class="ru-err">fault 0x' + a.fault.toString(16) + "</span>" : ""}, gap ${a.gap_mm.toFixed(1)} mm` : "none"],
                    ["arm q", a ? a.q.map(v => (v * 180 / Math.PI).toFixed(0)).join(" ") + " °" : "–"],
                    ["backends", Object.entries(s.backends).map(([k, v]) => `${k} ${v.clients ? '<span class="ru-ok">' + v.clients + "</span>" : '<span class="ru-muted">0</span>'}`).join(" · ")]]);
  $("#mods").innerHTML = "<tr><th>module</th><th>steer °</th><th>goal °</th><th>m/s</th><th>goal</th></tr>" +
    s.modules.map(m => `<tr><td>${m.name}${m.steer_on || m.drive_on ? "" : ' <span class="ru-muted">off</span>'}</td><td>${(m.steer * 57.3).toFixed(1)}</td><td>${(m.steer_goal * 57.3).toFixed(1)}</td><td>${f3(m.speed)}</td><td>${f3(m.speed_goal)}</td></tr>`).join("");
  $("#objs").innerHTML = "<tr><th>object</th><th>shape</th><th>xyz m</th><th></th></tr>" + (s.objects.length ? s.objects.map(o =>
    `<tr><td>${esc(o.name)}</td><td>${o.shape} ${o.mass} kg</td><td>${o.xyz.map(f3).join(", ")}</td><td><button class="ru" data-del="${esc(o.name)}">remove</button></td></tr>`).join("")
    : '<tr><td class="ru-muted" colspan="4">no objects</td></tr>');
  const on = s.objects.map(o => o.name).join("\n");
  if (on !== objNames) {
    objNames = on; const sel = $("#pbody"), cur = sel.value;
    sel.innerHTML = '<option value="base">base</option>' + s.objects.map(o => `<option>${esc(o.name)}</option>`).join("");
    sel.value = [...sel.options].some(o => o.value === cur) ? cur : "base";
  }
  const names = Object.keys(s.cameras);
  if (names.join() !== cams.join()) {
    cams = names;
    $("#cams").innerHTML = names.map(n => `<div><div class="ru-muted">${esc(n)} colour</div><img data-cam="${esc(n)}" data-s="color" alt="${esc(n)} colour"></div><div><div class="ru-muted">${esc(n)} depth</div><img data-cam="${esc(n)}" data-s="depth" alt="${esc(n)} depth"></div>`).join("");
    snap();
  }
  $("#events").innerHTML = s.events.slice().reverse().map(e => `<div class="${e.level === "warn" ? "ru-ev-warn" : e.level === "error" ? "ru-ev-error" : ""}">${new Date(e.wall * 1000).toLocaleTimeString()} ${esc(e.msg)}</div>`).join("");
}
let snapping = false;
function snap() {
  if (snapping) return;
  const imgs = [...document.querySelectorAll("img[data-cam]")];
  if (!imgs.length) return;
  snapping = true; let left = imgs.length;
  const done = () => { if (--left <= 0) snapping = false; };
  imgs.forEach(i => { i.onload = i.onerror = done; i.src = `cameras/${i.dataset.cam}/${i.dataset.s}.jpg?t=${Date.now()}`; });
}
(function camLoop(){
  const p = Math.max(0.2, +$("#camperiod").value || 1);
  if ($("#camlive").checked && !document.hidden) snap();
  setTimeout(camLoop, p * 1000);
})();
$("#snap").onclick = snap;
$("#run").onclick = () => api("POST", "run", {rtf: +$("#rtf").value});
$("#pause").onclick = () => api("POST", "pause");
$("#step").onclick = () => api("POST", "step", {seconds: +$("#stepms").value / 1000}).then(snap);
$("#reset").onclick = () => api("POST", "scene/reset").then(snap);
let hrole = "auto";
$("#hview").onclick = () => api("POST", "scene3d/view/next");
$("#hrole").onclick = () => api("POST", "scene3d/view", {role: {auto: "collision", collision: "visual", visual: "auto"}[hrole]});
function objSpec() {
  const sz = +$("#osize").value, shape = $("#oshape").value;
  return {name: $("#oname").value, shape, mass: +$("#omass").value, xyz: [+$("#ox").value, +$("#oy").value, +$("#oz").value],
          ...(shape === "box" ? {size: [sz, sz, sz]} : {radius: sz / 2, length: sz})};
}
$("#add").onclick = () => api("POST", "scene/objects", objSpec());
$("#move").onclick = () => api("PUT", `scene/objects/${encodeURIComponent($("#oname").value)}`, {xyz: [+$("#ox").value, +$("#oy").value, +$("#oz").value]});
$("#ssave").onclick = () => api("POST", "scene/save", {name: $("#sname").value});
$("#sstart").onclick = () => api("POST", "scene/reset", {start: true}).then(snap);
$("#sload").onclick = () => api("POST", "scene/load", {name: $("#sname").value});
$("#objs").onclick = e => { const n = e.target.dataset.del; if (n) api("DELETE", `scene/objects/${encodeURIComponent(n)}`); };
$("#push").onclick = () => api("POST", "physics/wrench", {body: $("#pbody").value, force: [+$("#fx").value, +$("#fy").value, +$("#fz").value], duration: +$("#fdur").value});
$("#tele").onclick = () => api("POST", "physics/teleport", {xyz: [+$("#tx").value, +$("#ty").value, 0], yaw: +$("#tyaw").value * Math.PI / 180});
$("#mkclear").onclick = () => api("DELETE", "markers");
$("#mkdemo").onclick = () => api("POST", "markers", {source: "page", markers: [{name: "demo/ee", kind: "frame", frame: "arm_end_link",
  pos: [0, 0, 0.1], size: 0.08, label: "demo", ttl: 30}]});
window.simRenderMarkers = list => {
  $("#mks").innerHTML = "<tr><th>marker</th><th>kind</th><th>frame</th><th>source</th></tr>" + (list.length ? list.map(m =>
    `<tr><td><span class="sw" style="background:${esc(m.color || "var(--signal)")}"></span>${esc(m.name)}</td><td>${m.kind}</td><td>${esc(m.frame)}</td><td>${esc(m.source || "")}</td></tr>`).join("")
    : '<tr><td class="ru-muted" colspan="4">none</td></tr>');
};
window.simRenderMarkers([]);
const es = new EventSource("events");
es.onmessage = e => render(JSON.parse(e.data));
</script>
<script type="module">
// -- the 3D view (three.js, Z up). Bodies as groups posed from viz/stream; geoms as their children.
const view = document.getElementById("view"), msg = document.getElementById("viewmsg"), note = document.getElementById("vnote");
const tok = (n, f) => (window.RU && RU.token ? RU.token(n, f) : f) || f;
async function main() {
  let THREE, OrbitControls, CSS2DRenderer, CSS2DObject;
  try {
    THREE = await import("three");
    ({OrbitControls} = await import("three/addons/controls/OrbitControls.js"));
    ({CSS2DRenderer, CSS2DObject} = await import("three/addons/renderers/CSS2DRenderer.js"));
  } catch (e) {
    msg.textContent = "The 3D view needs three.js from cdn.jsdelivr.net (" + e.message + "). The rest of the page works.";
    return;
  }
  THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
  const renderer = new THREE.WebGLRenderer({antialias: true});
  renderer.setPixelRatio(window.devicePixelRatio || 1);
  view.insertBefore(renderer.domElement, view.firstChild);
  const labels = new CSS2DRenderer();
  Object.assign(labels.domElement.style, {position: "absolute", top: "0", left: "0", pointerEvents: "none"});
  view.insertBefore(labels.domElement, renderer.domElement.nextSibling);
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(45, 1, 0.02, 200);
  camera.position.set(-1.5, -1.35, 1.15);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0.1, 0, 0.35); controls.enableDamping = true; controls.update();
  scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f96, 1.6));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6); sun.position.set(2, -3, 5); scene.add(sun);
  const world = new THREE.Group(); scene.add(world);
  const col = () => ({ground: tok("--ground", "#eef1f4"), ink: tok("--ink", "#15202b"), muted: tok("--muted", "#5b6672"),
                      rule: tok("--rule", "#d3dae1"), signal: tok("--signal", "#1f5fbf"), caution: tok("--caution", "#a86400")});
  let C = col();

  // floor: 1 m major lines over +-10 m, 0.1 m minor lines over +-2 m around the robot (re-centred as it drives)
  function gridLines(half, step, colour, opacity) {
    const pts = [];
    for (let v = -half; v <= half + 1e-9; v += step) pts.push(-half, v, 0, half, v, 0, v, -half, 0, v, half, 0);
    const g = new THREE.BufferGeometry(); g.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
    return new THREE.LineSegments(g, new THREE.LineBasicMaterial({color: colour, transparent: true, opacity, depthWrite: false}));
  }
  let major, minor;
  function buildGrid() {
    if (major) { scene.remove(major, minor); }
    major = gridLines(10, 1, C.muted, 0.35); minor = gridLines(2, 0.1, C.rule, 0.6);
    major.position.z = 0.0005; minor.position.z = 0.0002; scene.add(major, minor);
    scene.background = new THREE.Color(C.ground);
  }
  buildGrid();
  const axes = new THREE.AxesHelper(0.3); axes.position.z = 0.001; scene.add(axes);    // world origin

  let model = null, groups = [], byName = {}, lines = false, extras = true, follow = true;
  const fills = [], edges = [];
  const trailMax = 20000, trailPos = new Float32Array(trailMax * 3); let trailN = 0, trailLast = null;
  const trailGeom = new THREE.BufferGeometry(); trailGeom.setAttribute("position", new THREE.BufferAttribute(trailPos, 3));
  const trail = new THREE.Line(trailGeom, new THREE.LineBasicMaterial({color: C.signal, transparent: true, opacity: 0.7}));
  trail.frustumCulled = false;      // its bounding sphere would be the empty buffer's
  scene.add(trail);

  function material(rgba) {
    return new THREE.MeshStandardMaterial({color: new THREE.Color(rgba[0], rgba[1], rgba[2]), roughness: 0.85, metalness: 0.0,
      flatShading: true, transparent: rgba[3] < 1, opacity: rgba[3], polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1});
  }
  function geomGeometry(g, meshGeoms) {
    const s = g.size;
    switch (g.type) {
      case "box": return new THREE.BoxGeometry(2 * s[0], 2 * s[1], 2 * s[2]);
      case "sphere": return new THREE.SphereGeometry(s[0], 20, 14);
      case "ellipsoid": { const e = new THREE.SphereGeometry(1, 20, 14); e.scale(s[0], s[1], s[2]); return e; }
      case "cylinder": { const c = new THREE.CylinderGeometry(s[0], s[0], 2 * s[1], 28); c.rotateX(Math.PI / 2); return c; }
      case "capsule": { const c = new THREE.CapsuleGeometry(s[0], 2 * s[1], 6, 16); c.rotateX(Math.PI / 2); return c; }
      case "mesh": return meshGeoms[g.mesh];
    }
    return null;
  }
  const frustumOf = (len, fovy, aspect) => {
    const h = Math.tan(fovy * Math.PI / 360) * len, w = h * aspect, c = [[-w, -h], [w, -h], [w, h], [-w, h]];
    const p = []; c.forEach(([x, y], i) => { const [x2, y2] = c[(i + 1) % 4]; p.push(0, 0, 0, x, y, len, x, y, len, x2, y2, len); });
    const g = new THREE.BufferGeometry(); g.setAttribute("position", new THREE.Float32BufferAttribute(p, 3));
    return new THREE.LineSegments(g, new THREE.LineBasicMaterial({color: C.muted, transparent: true, opacity: 0.8}));
  };
  const frustums = [];

  async function loadModel() {
    msg.textContent = "loading the world's geometry…"; msg.hidden = false;
    const info = await (await fetch("viz/model", {cache: "no-store"})).json();
    const r = await fetch(`viz/meshes.bin?generation=${info.generation}&cell=${info.cell}`, {cache: "no-store"});
    if (!r.ok) throw new Error("viz/meshes.bin: " + r.status);
    const buf = await r.arrayBuffer();
    const meshGeoms = info.meshes.map(m => {
      const g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.BufferAttribute(new Float32Array(buf, m.vert_offset, m.nvert * 3), 3));
      g.setIndex(new THREE.BufferAttribute(m.index === "u16" ? new Uint16Array(buf, m.face_offset, m.nface * 3) : new Uint32Array(buf, m.face_offset, m.nface * 3), 1));
      g.computeVertexNormals(); return g;
    });
    world.clear(); fills.length = 0; edges.length = 0; frustums.length = 0;
    groups = info.bodies.map(b => { const gr = new THREE.Group(); gr.name = b.body; world.add(gr); return gr; });
    byName = {}; info.bodies.forEach((b, i) => { byName[b.body] = groups[i]; byName[b.name] = groups[i]; });
    byName.base = byName[info.base_body]; byName.world = world;
    for (const g of info.geoms) {
      const geo = geomGeometry(g, meshGeoms); if (!geo) continue;
      const mesh = new THREE.Mesh(geo, material(g.rgba));
      mesh.position.set(...g.pos); mesh.quaternion.set(g.quat[1], g.quat[2], g.quat[3], g.quat[0]);
      mesh.userData.alpha = g.rgba[3]; groups[g.body].add(mesh); fills.push(mesh);
    }
    for (const n of ["camera_color_optical", "wrist_camera_color_optical"]) {
      if (byName[n]) { const f = frustumOf(0.18, 65, 4 / 3); byName[n].add(f); frustums.push(f); }
    }
    model = info; edgesBuilt = false; applyStyle(); markersShown = -1;
    msg.hidden = true;
  }
  let edgesBuilt = false;
  function applyStyle() {
    if (lines && !edgesBuilt) {
      for (const m of fills) {
        const e = new THREE.LineSegments(new THREE.EdgesGeometry(m.geometry, 30), new THREE.LineBasicMaterial({color: C.ink}));
        e.position.copy(m.position); e.quaternion.copy(m.quaternion); m.parent.add(e); edges.push(e);
      }
      edgesBuilt = true;
    }
    for (const m of fills) { const a = m.userData.alpha; m.material.transparent = lines || a < 1; m.material.opacity = lines ? 0.12 * a : a; m.material.depthWrite = !lines; m.material.needsUpdate = true; }
    for (const e of edges) e.visible = lines;
    for (const f of frustums) f.visible = extras;
    trail.visible = extras;
  }

  // markers (POST markers): frame = axes (+ a dot in its colour), point, line, text; parented to their frame's body
  const markerRoot = new THREE.Group(); scene.add(markerRoot);
  let markerObjs = [], markersShown = -1, markerList = [];
  function buildMarkers(list) {
    for (const o of markerObjs) { o.removeFromParent(); o.traverse(x => { if (x.geometry) x.geometry.dispose(); if (x.element) x.element.remove(); }); }
    markerObjs = [];
    for (const m of list) {
      const parent = byName[m.frame] || world, colour = new THREE.Color(m.color || C.signal);
      const o = new THREE.Group();
      if (m.pos) o.position.set(...m.pos);
      if (m.quat) o.quaternion.set(m.quat[0], m.quat[1], m.quat[2], m.quat[3]);
      if (m.kind === "frame") {
        // axes as thin rods (WebGL lines are 1 px), x red, y green, z blue; drawn over the robot
        const L = m.size || 0.1, r = Math.max(0.0015, L * 0.03);
        [[0xd9402e, [0, 0, -Math.PI / 2]], [0x3aa655, [0, 0, 0]], [0x3d6fe0, [Math.PI / 2, 0, 0]]].forEach(([c, rot]) => {
          const rod = new THREE.Mesh(new THREE.CylinderGeometry(r, r, L, 8).translate(0, L / 2, 0),
                                     new THREE.MeshBasicMaterial({color: c, depthTest: false, transparent: true, opacity: 0.95}));
          rod.rotation.set(...rot); rod.renderOrder = 10; o.add(rod);
        });
        const dot = new THREE.Mesh(new THREE.SphereGeometry(r * 2.4, 14, 10), new THREE.MeshBasicMaterial({color: colour, depthTest: false}));
        dot.renderOrder = 11; o.add(dot);
      } else if (m.kind === "point") {
        const dot = new THREE.Mesh(new THREE.SphereGeometry((m.size || 0.02) / 2, 14, 10), new THREE.MeshBasicMaterial({color: colour}));
        o.add(dot);
      } else if (m.kind === "line") {
        const g = new THREE.BufferGeometry(); g.setAttribute("position", new THREE.Float32BufferAttribute(m.points.flat(), 3));
        o.add(new THREE.Line(g, new THREE.LineBasicMaterial({color: colour})));
      }
      const text = m.kind === "text" ? (m.label || m.name) : m.label;
      if (text) { const d = document.createElement("div"); d.className = "lbl"; d.textContent = text; const l = new CSS2DObject(d); l.position.set(0, 0, 0.02); o.add(l); }
      parent.add(o); markerObjs.push(o);
    }
    if (window.simRenderMarkers) window.simRenderMarkers(list);
  }

  // views
  const vbtn = id => document.getElementById(id);
  function setView(kind) {
    const t = controls.target;
    if (kind === "top") camera.position.set(t.x - 0.001, t.y, t.z + 3.5);
    else camera.position.set(t.x - 1.6, t.y - 1.45, t.z + 0.8);
    controls.update();
  }
  vbtn("v3d").onclick = () => setView("3d");
  vbtn("vtop").onclick = () => setView("top");
  renderer.domElement.addEventListener("dblclick", () => setView("3d"));
  vbtn("vfollow").onclick = e => { follow = !follow; e.target.classList.toggle("on", follow); };
  vbtn("vlines").onclick = e => { lines = !lines; e.target.classList.toggle("on", lines); applyStyle(); };
  vbtn("vcoll").onclick = e => { extras = !extras; e.target.classList.toggle("on", extras); applyStyle(); };
  vbtn("vclear").onclick = () => { trailN = 0; trailLast = null; trailGeom.setDrawRange(0, 0); };

  // the pose stream
  let frame = null, nFrames = 0, rateAt = performance.now(), rate = 0, loading = false, lastBase = null;
  function onFrame(f) {
    nFrames++;
    if (!model || f.generation !== model.generation) {
      if (!loading) { loading = true; loadModel().catch(e => { msg.textContent = "3D view: " + e.message; msg.hidden = false; }).finally(() => { loading = false; }); }
      if (f.markers) markerList = f.markers;
      return;
    }
    frame = f;
    if (f.markers) { markerList = f.markers; markersShown = -1; }
    if (markersShown !== f.markers_version) { buildMarkers(markerList); markersShown = f.markers_version; }
    const P = f.pos, Q = f.quat;
    for (let i = 0; i < groups.length; i++) {
      groups[i].position.set(P[3 * i], P[3 * i + 1], P[3 * i + 2]);
      groups[i].quaternion.set(Q[4 * i + 1], Q[4 * i + 2], Q[4 * i + 3], Q[4 * i]);
    }
    const b = f.base.xyz;
    if (!lastBase) lastBase = [0, 0, 0];          // first frame: bring the robot (wherever it is) into view
    if (follow) {
      const dx = b[0] - lastBase[0], dy = b[1] - lastBase[1];
      controls.target.x += dx; controls.target.y += dy; camera.position.x += dx; camera.position.y += dy;
    }
    lastBase = b;
    minor.position.x = Math.round(b[0]); minor.position.y = Math.round(b[1]);
    if (!trailLast || Math.hypot(b[0] - trailLast[0], b[1] - trailLast[1]) > 0.005) {
      if (trailLast && Math.hypot(b[0] - trailLast[0], b[1] - trailLast[1]) > 0.5) trailN = 0;   // teleport: new trail
      if (trailN >= trailMax) { trailPos.copyWithin(0, 3 * (trailMax / 2)); trailN = trailMax / 2; }
      trailPos.set([b[0], b[1], 0.003], 3 * trailN); trailN++; trailLast = b;
      trailGeom.setDrawRange(0, trailN); trailGeom.attributes.position.needsUpdate = true;
    }
    const clock = document.getElementById("clock");
    if (clock) clock.textContent = f.t.toFixed(2) + " s";
    const rm = document.getElementById("rtfm");
    if (rm) rm.textContent = f.running ? `× ${f.rtf_measured.toFixed(2)} (target ${f.rtf})` : "paused";
  }
  const es = new EventSource("viz/stream?hz=30");
  es.onmessage = e => { try { onFrame(JSON.parse(e.data)); } catch (err) { console.error(err); } };
  es.onerror = () => { note.textContent = "pose stream: reconnecting…"; };

  // theme changes (the theme switcher or the OS): re-read the tokens
  let themeKey = "";
  function size() {
    const w = view.clientWidth, h = view.clientHeight;
    renderer.setSize(w, h, false); renderer.domElement.style.width = w + "px"; renderer.domElement.style.height = h + "px";
    labels.setSize(w, h); camera.aspect = w / Math.max(1, h); camera.updateProjectionMatrix();
  }
  new ResizeObserver(size).observe(view); size();
  function loop() {
    requestAnimationFrame(loop);
    const now = performance.now();
    if (now - rateAt > 1000) {
      rate = nFrames * 1000 / (now - rateAt); nFrames = 0; rateAt = now;
      const k = [tok("--ground", ""), tok("--ink", ""), tok("--signal", "")].join();
      if (k !== themeKey) { themeKey = k; C = col(); buildGrid(); trail.material.color.set(C.signal); }
      note.textContent = frame ? `poses ${rate.toFixed(0)} Hz · base ${frame.base.xyz[0].toFixed(2)}, ${frame.base.xyz[1].toFixed(2)} m, ${(frame.base.yaw * 180 / Math.PI).toFixed(0)}° · ${model ? model.meshes.length + " meshes" : ""}` : "waiting for poses…";
    }
    controls.update();
    renderer.render(scene, camera); labels.render(scene, camera);
  }
  loop();
  window.simView = {THREE, scene, camera, controls, markers: () => markerObjs, model: () => model};   // debugging
}
main();
</script></body></html>
""".replace("__THREE__", THREE_VERSION)
