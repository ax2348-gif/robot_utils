"""The quick-look page served at / (inline HTML/JS, no CDN). Uses only the public API."""

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Orbbec node</title>
<link rel="stylesheet" href="common/ui.css">
<style>
/* colours come from the shared token sheet (common/ui.css; themes restyle it) */
:root{--dim:var(--muted)}
*{box-sizing:border-box}body{margin:0;background:var(--ground);color:var(--ink);font:13px/1.4 var(--font)}
header{display:flex;gap:12px;align-items:center;padding:8px 14px;border-bottom:1px solid var(--line);flex-wrap:wrap}
header h1{font-size:15px;margin:0}#dev{color:var(--dim)}header a{color:var(--acc)}
.pill{padding:2px 8px;border-radius:10px;background:var(--line);font-weight:600}
.pill.streaming{background:var(--live-bg);color:var(--live)}.pill.err{background:var(--stop-bg);color:var(--stop)}.pill.busy{background:var(--caution-bg);color:var(--caution)}
.pill.covered{background:var(--caution-bg);color:var(--caution)}
#banner{display:none;background:var(--stop-bg);color:var(--stop);padding:6px 14px;border-bottom:1px solid var(--stop)}
#main{display:flex;align-items:flex-start}
aside{width:300px;flex:none;padding:10px;border-right:1px solid var(--line);min-height:calc(100vh - 42px)}
section.box{background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:8px 10px;margin-bottom:10px}
section.box h2{font-size:13px;font-weight:600;color:var(--ink);margin:0 0 6px}
label.row,div.row{display:flex;align-items:center;gap:6px;margin:4px 0}
label.row span.k,div.row span.k{flex:1;color:var(--dim)}
select,input[type=number]{background:var(--panel);color:var(--ink);border:1px solid var(--line);border-radius:4px;padding:2px 4px;font:inherit}
input[type=number]{width:74px}select{max-width:170px}input[type=range]{width:100%}
input[type=color]{width:40px;height:22px;border:none;background:none;padding:0}
.bar{height:10px;border-radius:3px;margin:4px 0}
.small{font-size:11px;color:var(--dim)}.mono{font-variant-numeric:tabular-nums;font-family:ui-monospace,monospace;font-size:11px}
#tiles{flex:1;display:grid;grid-template-columns:repeat(auto-fit,minmax(380px,1fr));gap:10px;padding:10px;align-content:start}
.tile{background:var(--panel);border:1px solid var(--line);border-radius:6px;overflow:hidden}
.tile .hd{display:flex;gap:8px;align-items:center;padding:4px 8px;border-bottom:1px solid var(--line);flex-wrap:wrap}
.tile .hd b{font-size:13px}.tile .hd .st{color:var(--dim);flex:1;font-variant-numeric:tabular-nums}
.tile .hd a{color:var(--acc);text-decoration:none;font-size:12px}
.tile .hd{cursor:zoom-in;user-select:none}.tile.max .hd{cursor:zoom-out}
#tiles.has-max{grid-template-columns:1fr}#tiles.has-max .tile:not(.max){display:none}
.tile.max .wrap{width:fit-content;max-width:100%;margin:0 auto}
.tile.max .wrap img{width:auto;max-width:100%;max-height:calc(100vh - 120px)}
.tile.max.viz canvas{height:calc(100vh - 140px)!important}
.wrap{position:relative;background:#000;line-height:0}
.wrap img{width:100%;height:auto;display:block;min-height:120px;cursor:crosshair}
.ro{position:absolute;left:6px;top:6px;background:#000a;padding:2px 6px;border-radius:4px;line-height:1.4;font-variant-numeric:tabular-nums;pointer-events:none;white-space:pre}
.ro:empty{display:none}
.nof{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;color:var(--dim);line-height:1.4}
.pin{position:absolute;width:14px;height:14px;margin:-7px 0 0 -7px;border:2px solid #fff;border-radius:50%;box-shadow:0 0 0 1px #000;pointer-events:none}
.cb{display:flex;align-items:center;gap:6px;padding:4px 8px}.cb .bar{flex:1;margin:0}
#log div{margin:2px 0}#log .e{color:var(--err)}
@media (max-width:760px){#main{flex-direction:column}aside{width:100%;min-height:0;border-right:none}#tiles{width:100%;grid-template-columns:1fr}}
</style><script src="../../themes/theme.js"></script>
</head><body data-page="camera"><div id="band"></div>
<header class="ru-head"><h1>Orbbec node</h1><span id="dev">connecting...</span><span id="pill" class="pill">...</span>
<span id="sync" class="small"></span><span id="msg" class="small"></span><a href="docs" class="small">API docs</a></header>
<div id="banner"></div>
<div id="main" class="ru-body">
<aside class="ru-side">
<section class="box" data-panel="streams" data-role="control"><h2>Streams</h2><div id="streams"></div>
<div class="small">Settings are global: every client sees the same camera.</div></section>
<section class="box" data-panel="profiles" data-role="config"><h2>Profiles</h2>
<label class="row"><span class="k">Color</span><select id="color_profile"></select></label>
<label class="row"><span class="k">Depth / IR</span><select id="stereo_mode"></select></label>
<label class="row"><span class="k">Align depth to color</span><input type="checkbox" id="align"></label>
<label class="row"><span class="k">Frame sync</span><input type="checkbox" id="frame_sync"></label>
<div class="small">Depth and IR share one mode. Aligned depth has the color resolution. Frame sync pairs color+depth (adds latency).</div></section>
<section class="box" data-panel="depth-display" data-role="config"><h2>Depth display</h2>
<label class="row"><span class="k">Colormap</span><select id="colormap"></select></label>
<div class="bar" id="cmapbar"></div>
<label class="row"><span class="k">Invert</span><input type="checkbox" id="invert"></label>
<label class="row"><span class="k">Range</span><select id="range_mode"><option value="auto">auto (percentiles)</option><option value="manual">manual (mm)</option></select></label>
<div class="row" id="pctrow"><span class="k">Percentiles</span><input type="number" id="pct_lo" min="0" max="100" step="0.5"><input type="number" id="pct_hi" min="0" max="100" step="0.5"></div>
<div class="row" id="mmrow"><span class="k">Min / max mm</span><input type="number" id="min_mm" min="0" step="50"><input type="number" id="max_mm" min="0" step="50"></div>
<label class="row"><span class="k">Invalid pixels</span><input type="color" id="invalid_color"></label>
<label class="row"><span class="k">JPEG quality</span><input type="number" id="jpeg_quality" min="30" max="95" step="5"></label>
<div class="small">Hover depth to read mm; click to pin a point.</div></section>
<section class="box" data-panel="imu" data-role="telemetry"><h2>IMU</h2>
<label class="row"><span class="k">Enabled</span><input type="checkbox" id="imu_enabled"></label>
<label class="row"><span class="k">Sample rate</span><select id="imu_rate"></select></label>
<div id="imu" class="mono">-</div>
<div class="row"><button id="imu_cal" title="Keep the camera still">Calibrate (keep still)</button>
<button id="imu_reset" title="Roll/pitch from gravity, yaw 0">Reset orientation</button></div></section>
<section class="box" data-panel="device" data-role="config"><h2>Device controls <a href="#" id="propsref" class="small" style="float:right">refresh</a></h2><div id="props" class="small">-</div></section>
<section class="box" data-panel="log" data-role="log"><h2>Log</h2><div id="log" class="small"></div></section>
</aside>
<div id="tiles" data-panel="tiles" data-role="view"></div>
</div>
<script src="common/ui.js"></script>
<script>
"use strict";
const $=s=>document.querySelector(s), ORDER=["color","depth","ir_left","ir_right"];
let S=null, CAT=null, catId=null, setVer=-1, tiles={};
const sleep=ms=>new Promise(r=>setTimeout(r,ms)), dec=new TextDecoder();
function el(tag,attrs,...kids){const e=document.createElement(tag);for(const[k,v]of Object.entries(attrs||{})){if(k==="class")e.className=v;else if(k.startsWith("on"))e[k]=v;else e.setAttribute(k,v)}for(const c of kids)e.append(c);return e}
async function api(path,body){
  const r=await fetch(path,body===undefined?{cache:"no-store"}:{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body)});
  const j=await r.json().catch(()=>({detail:r.statusText}));
  if(!r.ok)throw new Error(typeof j.detail==="string"?j.detail:JSON.stringify(j.detail));return j}
let msgT=0;function flash(t,bad){const m=$("#msg");m.textContent=t;m.style.color=bad?"var(--err)":"var(--dim)";clearTimeout(msgT);if(t)msgT=setTimeout(()=>m.textContent="",bad?8000:2500)}
async function post(patch){try{const r=await api("settings",patch);flash(r.restarted?"applied (pipeline restarted)":"applied")}catch(e){flash(e.message,true)}}

// --- clock sync (NTP-style): server_mono = performance.now()/1000 + OFF, within +-UNC ---
let OFF=null, UNC=null;
async function sync(n=12){
  const xs=[];await fetch("time",{cache:"no-store"});
  for(let i=0;i<n;i++){const t0=performance.now()/1000;const j=await (await fetch("time",{cache:"no-store"})).json();const t1=performance.now()/1000;xs.push({t0,t1,ts:j.mono})}
  xs.sort((a,b)=>(a.t1-a.t0)-(b.t1-b.t0));const best=xs.slice(0,6);
  let lo=-Infinity,hi=Infinity;for(const s of best){lo=Math.max(lo,s.ts-s.t1);hi=Math.min(hi,s.ts-s.t0)}
  if(lo<=hi){OFF=(lo+hi)/2;UNC=(hi-lo)/2}else{const b=xs[0];OFF=b.ts-(b.t0+b.t1)/2;UNC=(b.t1-b.t0)/2}
  $("#sync").textContent=`clock sync ±${(UNC*1000).toFixed(1)} ms (rtt ${((xs[0].t1-xs[0].t0)*1000).toFixed(1)} ms)`}
const serverNow=()=>performance.now()/1000+OFF;

function buildControls(){
  const st=$("#streams");st.innerHTML="";
  for(const n of ORDER){
    const ok=CAT.available.includes(n);
    const cb=el("input",{type:"checkbox",id:"s_"+n});cb.disabled=!ok;
    cb.onchange=()=>post({streams:{[n]:cb.checked}});
    st.append(el("label",{class:"row"},el("span",{class:"k"},S.streams[n].label+(ok?"":" (n/a)")),el("span",{class:"small",id:"fps_"+n}),cb))}
  const fill=(id,list)=>{const s=$("#"+id);s.innerHTML="";for(const p of list)s.append(el("option",{value:p.key},p.key+(p.default?" (default)":"")))};
  fill("color_profile",CAT.color);fill("stereo_mode",CAT.stereo);
  const ir=$("#imu_rate");ir.innerHTML="";for(const r of CAT.imu.sample_rates)ir.append(el("option",{value:r},r));
  const cm=$("#colormap");cm.innerHTML="";for(const k of Object.keys(CAT.colormaps))cm.append(el("option",{value:k},k));
  for(const id of ["color_profile","stereo_mode"])$("#"+id).onchange=e=>post({[id]:e.target.value});
  for(const id of ["align","frame_sync"])$("#"+id).onchange=e=>post({[id]:e.target.checked});
  for(const id of ["colormap","range_mode","invalid_color"])$("#"+id).onchange=e=>post({display:{[id]:e.target.value}});
  $("#invert").onchange=e=>post({display:{invert:e.target.checked}});
  for(const id of ["pct_lo","pct_hi","min_mm","max_mm","jpeg_quality"])$("#"+id).onchange=e=>post({display:{[id]:Number(e.target.value)}});
  $("#imu_enabled").onchange=e=>post({imu:{enabled:e.target.checked}});
  ir.onchange=e=>post({imu:{sample_rate:e.target.value}});
  $("#imu_cal").onclick=async()=>{flash("calibrating, keep the camera still...");try{const c=await api("imu/calibrate",{});
    flash("calibrated: bias "+c.gyro_bias.map(v=>v.toFixed(5)).join(" ")+" rad/s, |g| "+c.g_ref.toFixed(3))}catch(e){flash(e.message,true)}};
  $("#imu_reset").onclick=async()=>{try{await api("imu/reset",{});flash("orientation reset")}catch(e){flash(e.message,true)}};
}
function grad(name){const c=CAT.colormaps[name]||[];return "linear-gradient(to right,"+c.join(",")+")"}
function syncControls(){
  const s=S.settings,d=s.display;
  for(const n of ORDER){const cb=$("#s_"+n);if(cb)cb.checked=!!s.streams[n]}
  for(const id of ["color_profile","stereo_mode"]){const e=$("#"+id);if(document.activeElement!==e&&s[id]!=null)e.value=s[id]}
  for(const id of ["colormap","range_mode","invalid_color","pct_lo","pct_hi","min_mm","max_mm","jpeg_quality"]){
    const e=$("#"+id);if(e&&document.activeElement!==e&&d[id]!=null)e.value=d[id]}
  $("#align").checked=s.align;$("#frame_sync").checked=s.frame_sync;$("#invert").checked=d.invert;
  $("#align").disabled=!(s.streams.color&&s.streams.depth);
  $("#imu_enabled").checked=s.imu.enabled;if(s.imu.sample_rate)$("#imu_rate").value=s.imu.sample_rate;
  $("#pctrow").style.display=d.range_mode==="auto"?"":"none";$("#mmrow").style.display=d.range_mode==="manual"?"":"none";
  $("#cmapbar").style.background=grad(d.colormap);$("#cmapbar").style.transform=d.invert?"scaleX(-1)":"";
  for(const t of Object.values(tiles))if(t.bar){t.bar.style.background=grad(d.colormap);t.bar.style.transform=d.invert?"scaleX(-1)":""}
}
function renderStatus(){
  const d=S.device||{};$("#dev").textContent=d.name?`${d.name}, SN ${d.serial}, firmware ${d.firmware}, ${d.connection}`:"no camera";
  const p=$("#pill");p.textContent=S.state;p.className="pill "+(S.state==="streaming"?"streaming":S.state==="covered"?"covered":S.state==="idle"?"":["starting","restarting"].includes(S.state)?"busy":"err");
  if(window.RU&&RU.stateBand){(window._band=window._band||RU.stateBand($("#band"))).update(S)}
  const b=$("#banner");b.style.display=S.error?"block":"none";b.textContent=S.error||"";
  $("#log").innerHTML="";for(const e of S.events.slice().reverse()){$("#log").append(el("div",{class:e.error?"e":""},new Date(e.wall*1000).toLocaleTimeString()+" "+e.msg))}
  for(const n of ORDER){const f=$("#fps_"+n),x=S.streams[n];if(f)f.textContent=x.fps?x.fps.toFixed(1)+" fps":""}
  const m=S.imu,l=m.last;
  $("#imu").textContent=!m.available?"no IMU":!m.running?(m.error||"off"):
    `${m.rate_hz.toFixed(0)} Hz  gaps ${m.gaps}\naccel m/s²  ${l?l.accel.map(v=>v.toFixed(2).padStart(6)).join(" "):"-"}\ngyro rad/s  ${l?l.gyro.map(v=>v.toFixed(3).padStart(6)).join(" "):"-"}\ntemp ${l?l.temperature.toFixed(1):"-"} °C`
    +(m.filter?`\nroll/pitch/yaw ° ${m.filter.rpy_deg.map(v=>v.toFixed(1).padStart(7)).join(" ")}\nbias rad/s ${m.filter.gyro_bias.map(v=>v.toFixed(5).padStart(9)).join(" ")}\n${m.filter.initializing?"initializing":m.filter.calibrating?"calibrating":m.filter.calibrated?"calibrated ("+m.filter.calibration.source+")":"NOT calibrated"}${m.filter.still?", still":""}`:"");
  $("#imu").style.whiteSpace="pre";
}
function makeTile(n){
  const img=el("img",{alt:n}),ro=el("div",{class:"ro"}),nof=el("div",{class:"nof"},"waiting for frames..."),pin=el("div",{class:"pin"});pin.style.display="none";
  const st=el("span",{class:"st"}),links=el("span",{});
  links.append(el("a",{href:"snapshot/"+n+"?format=jpg",target:"_blank",title:"current frame as JPEG"},"jpg"));
  if(n!=="color"){links.append(" ",el("a",{href:"snapshot/"+n+"?format=png",target:"_blank",title:"lossless 16-bit (depth: mm x depth_scale) PNG"},"png"))}
  links.append(" ",el("a",{href:"mjpeg/"+n,target:"_blank",title:"MJPEG stream of this sensor alone"},"mjpeg"));
  const wrap=el("div",{class:"wrap"},img,nof,pin,ro);
  const t={n,img,ro,nof,pin,st,url:null,frames:[],lat:[],meta:null,busy:false,pinxy:null};
  const box=el("div",{class:"tile"},el("div",{class:"hd"},el("b",{},S.streams[n].label),st,links),wrap);
  if(n==="depth"){t.lo=el("span",{class:"small"});t.hi=el("span",{class:"small"});t.bar=el("div",{class:"bar"});box.append(el("div",{class:"cb"},t.lo,t.bar,t.hi))}
  const xy=e=>[e.offsetX/img.clientWidth,e.offsetY/img.clientHeight];
  img.onmousemove=e=>{t.hover=xy(e);hover(t)};
  img.onmouseleave=()=>{t.hover=null;t.ro.textContent=pinText(t)};
  img.onclick=e=>{if(n!=="depth")return;t.pinxy=t.pinxy?null:xy(e);t.pinval=null;placePin(t)};
  t.box=box;return t}
// any pane (image tiles, orientation, point cloud): click its title bar to maximize, again to restore
$("#tiles").addEventListener("click",e=>{const hd=e.target.closest(".tile > .hd");
  if(!hd||e.target.closest("a,button,input,select,label"))return;
  const box=hd.parentElement,on=!box.classList.contains("max");
  for(const o of $("#tiles").querySelectorAll(".tile.max"))o.classList.remove("max");
  box.classList.toggle("max",on);$("#tiles").classList.toggle("has-max",on)});
new MutationObserver(()=>{if(!$("#tiles .tile.max"))$("#tiles").classList.remove("has-max")}).observe($("#tiles"),{childList:true});
function placePin(t){t.pin.style.display=t.pinxy?"":"none";if(t.pinxy){t.pin.style.left=(t.pinxy[0]*100)+"%";t.pin.style.top=(t.pinxy[1]*100)+"%"}}
const fmtmm=r=>r&&r.mm!=null?`${r.mm} mm (5x5 median ${r.median5??"-"})`:"no depth";
function pinText(t){return t.pinxy&&t.pinval?"pin: "+fmtmm(t.pinval):""}
async function hover(t){
  if(!t.hover||!t.meta)return;
  const m=t.meta,u=Math.floor(t.hover[0]*m.width),v=Math.floor(t.hover[1]*m.height);
  if(t.n!=="depth"){t.ro.textContent=`(${u}, ${v})`;return}
  if(t.busy)return;t.busy=true;
  try{const r=await api(`depth_at?x=${t.hover[0]}&y=${t.hover[1]}`);if(t.hover)t.ro.textContent=`(${r.u}, ${r.v}) ${fmtmm(r)}`}catch(e){}
  t.busy=false}
setInterval(async()=>{const t=tiles.depth;if(!t||!t.pinxy)return;
  try{t.pinval=await api(`depth_at?x=${t.pinxy[0]}&y=${t.pinxy[1]}`);if(!t.hover)t.ro.textContent=pinText(t)}catch(e){}},500);
const med=a=>{if(!a.length)return null;const s=a.slice().sort((x,y)=>x-y);return s[s.length>>1]};
function renderTiles(){
  const want=ORDER.filter(n=>S.settings.streams[n]&&S.streams[n].available);
  for(const n of Object.keys(tiles))if(!want.includes(n)){tiles[n].box.remove();if(tiles[n].url)URL.revokeObjectURL(tiles[n].url);delete tiles[n]}
  for(const n of want){if(!tiles[n]){tiles[n]=makeTile(n);tiles[n].box.firstChild.title="click to maximize / restore";syncControls()}}
  const box=$("#tiles");for(const n of want)box.append(tiles[n].box);
  for(const n of want){const t=tiles[n],x=S.streams[n],now=performance.now();
    t.frames=t.frames.filter(f=>now-f<2000);const dfps=t.frames.length>1?(t.frames.length-1)*1000/(t.frames[t.frames.length-1]-t.frames[0]):0;
    const lat=med(t.lat.map(z=>z[0])),sen=med(t.lat.map(z=>z[1]).filter(z=>z!=null));t.lat=t.lat.slice(-30);
    t.st.textContent=x.width?`${x.width}x${x.height} ${x.format}${t.meta&&t.meta.aligned?" (aligned)":""} | cam ${x.fps.toFixed(1)} fps | shown ${dfps.toFixed(1)} fps | capture→display ${lat!=null?(lat*1000).toFixed(0):"-"} ms`+(sen!=null?` (+${(sen*1000).toFixed(0)} sensor→host)`:""):"no frames";
    if(!x.fps&&S.state!=="streaming"){t.nof.style.display="";t.nof.textContent=S.state}}
}
function onFrame(h,body){
  const t=tiles[h.stream];if(!t)return;
  t.meta=h;t.frames.push(performance.now());t.nof.style.display="none";
  const url=URL.createObjectURL(new Blob([body],{type:"image/jpeg"}));
  const old=t.url;t.url=url;
  t.img.onload=()=>{if(old)URL.revokeObjectURL(old);if(OFF!=null)t.lat.push([serverNow()-h.t_capture,h.t_sensor!=null?h.t_capture-h.t_sensor:null])};
  t.img.src=url;
  if(t.lo&&h.lo_mm!=null){t.lo.textContent=Math.round(h.lo_mm)+" mm";t.hi.textContent=Math.round(h.hi_mm)+" mm  (valid "+Math.round(h.valid*100)+"%)"}
}
async function live(){
  for(;;){
    try{
      const r=await fetch("stream?streams="+ORDER.join(",")+"&mode=jpeg",{cache:"no-store"});const rd=r.body.getReader();let buf=new Uint8Array(0);
      for(;;){
        const{value,done}=await rd.read();if(done)break;
        const nb=new Uint8Array(buf.length+value.length);nb.set(buf);nb.set(value,buf.length);buf=nb;
        for(;;){
          if(buf.length<4)break;
          const hl=new DataView(buf.buffer,buf.byteOffset,4).getUint32(0);if(buf.length<4+hl)break;
          const h=JSON.parse(dec.decode(buf.subarray(4,4+hl)));if(buf.length<4+hl+h.len)break;
          const body=buf.slice(4+hl,4+hl+h.len);buf=buf.slice(4+hl+h.len);
          if(h.type==="frame")onFrame(h,body)}}
    }catch(e){}
    await sleep(1000)}
}
async function loadProps(){let props;try{props=await api("properties")}catch(e){$("#props").textContent=e.message;return}renderProps(props)}
function renderProps(props){
  const box=$("#props");box.innerHTML="";if(!props.length){box.textContent="none reported";return}
  let group=null;
  for(const p of props){
    if(p.group!==group){group=p.group;box.append(el("div",{style:"margin-top:6px;font-weight:600;color:var(--fg)"},group))}
    const set=async v=>{try{renderProps(await api("properties",{id:p.id,value:v}));flash(p.label+" set")}catch(e){flash(p.label+": "+e.message,true);loadProps()}};
    if(p.type==="bool"){const cb=el("input",{type:"checkbox"});cb.checked=p.value;cb.onchange=()=>set(cb.checked);box.append(el("label",{class:"row"},el("span",{class:"k"},p.label),cb))}
    else if(p.type==="int"||p.type==="float"){
      const num=el("input",{type:"number",min:p.min,max:p.max,step:p.step||1});num.value=p.value;
      const rng=el("input",{type:"range",min:p.min,max:p.max,step:p.step||1});rng.value=p.value;
      rng.oninput=()=>num.value=rng.value;rng.onchange=()=>set(Number(rng.value));num.onchange=()=>set(Number(num.value));
      box.append(el("div",{class:"row"},el("span",{class:"k",title:p.id},`${p.label} [${p.min}..${p.max}]`),num),rng)}
    else box.append(el("div",{class:"row"},el("span",{class:"k"},p.label),el("span",{class:"small"},"error: "+p.error)))}
}
$("#propsref").onclick=e=>{e.preventDefault();loadProps()};
async function onState(st){
  S=st;
  if(S.catalog_id!==catId){try{CAT=await api("catalog");catId=S.catalog_id;buildControls();loadProps();setVer=-1}catch(e){return}}
  if(S.settings_version!==setVer){setVer=S.settings_version;syncControls()}
  renderStatus();renderTiles()}
function events(){
  const es=new EventSource("events");
  es.addEventListener("state",e=>onState(JSON.parse(e.data)));
  es.onerror=()=>{const p=$("#pill");p.textContent="server unreachable";p.className="pill err"};
}
(async()=>{try{await onState(await api("state"))}catch(e){}events();sync().catch(()=>{});setInterval(()=>sync().catch(()=>{}),30000);live()})();
</script></body></html>
"""

# Visualisations (IMU attitude + strip charts, WebGL2 point cloud) live in page_viz.py.
from .page_viz import VIZ_CSS, VIZ_HTML, VIZ_JS  # noqa: E402

for _old, _new in (("</style>", VIZ_CSS + "</style>"),
                   ('<section class="box" data-panel="device"', VIZ_HTML + '<section class="box" data-panel="device"'),
                   ("</script></body>", "</script><script>\n\"use strict\";\n" + VIZ_JS + "</script></body>")):
    assert PAGE.count(_old) == 1, _old
    PAGE = PAGE.replace(_old, _new)
