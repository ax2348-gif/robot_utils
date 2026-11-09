"""Visualisations for the node page: IMU state (attitude view + strip charts) and a WebGL2 point
cloud. Inlined into page.PAGE (no CDN); uses only the public API and the page's globals
(S, OFF, serverNow, tiles, api, el, $)."""

VIZ_CSS = r"""
.viz canvas{display:block;width:100%;background:#0b0e12}
.viz .ro{position:absolute}
.viz .row2{display:flex;gap:0}
.viz .row2>div{position:relative;flex:1;min-width:0}
.viz .legend span{margin-right:10px}
.viz .help{position:absolute;right:6px;bottom:4px;color:var(--dim);font-size:11px;pointer-events:none}
"""

VIZ_HTML = r"""
<section class="box" data-panel="visualisation" data-role="config"><h2>Visualisation</h2>
<label class="row"><span class="k">Orientation (IMU filter)</span><input type="checkbox" id="v_att" checked></label>
<label class="row"><span class="k">Point cloud</span><input type="checkbox" id="v_pc"></label>
<label class="row"><span class="k">Cloud colour</span><select id="pc_color"><option value="0">camera colour</option><option value="1">height</option><option value="2">distance</option></select></label>
<label class="row"><span class="k">Level with IMU (gravity)</span><input type="checkbox" id="pc_level" checked></label>
<label class="row"><span class="k">Pixel stride</span><select id="pc_stride"><option>1</option><option selected>2</option><option>4</option></select></label>
<label class="row"><span class="k">Max range m</span><input type="number" id="pc_max" value="6" min="0.5" max="20" step="0.5"></label>
<label class="row"><span class="k">Point size</span><input type="number" id="pc_size" value="2" min="1" max="8" step="1"></label>
<div class="small">Point cloud uses the raw depth stream (~8 MB/s at 848x480@10). Colour comes from the colour tile (no
alignment needed: points are projected into the colour camera with the factory extrinsics). Drag to orbit, wheel to zoom,
double-click to reset.</div></section>
"""

VIZ_JS = r"""
// ===================== shared helpers =====================
async function readStream(url, onRecord, signal){
  // Same framing as live(): [u32 BE header length][JSON header][payload of header.len bytes]
  const r=await fetch(url,{cache:"no-store",signal});const rd=r.body.getReader();let buf=new Uint8Array(0);
  for(;;){
    const{value,done}=await rd.read();if(done)return;
    const nb=new Uint8Array(buf.length+value.length);nb.set(buf);nb.set(value,buf.length);buf=nb;
    for(;;){
      if(buf.length<4)break;
      const hl=new DataView(buf.buffer,buf.byteOffset,4).getUint32(0);if(buf.length<4+hl)break;
      const h=JSON.parse(dec.decode(buf.subarray(4,4+hl)));const n=h.len||0;if(buf.length<4+hl+n)break;
      const body=buf.slice(4+hl,4+hl+n);buf=buf.slice(4+hl+n);
      if(h.type!=="ping")onRecord(h,body)}}
}
function qToR(q){const[w,x,y,z]=q;return[[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]]}
function rpy(q){const[w,x,y,z]=q,d=180/Math.PI;
  return[Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))*d,Math.asin(Math.max(-1,Math.min(1,2*(w*y-z*x))))*d,Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))*d]}
const mv=(R,p)=>[R[0][0]*p[0]+R[0][1]*p[1]+R[0][2]*p[2],R[1][0]*p[0]+R[1][1]*p[1]+R[1][2]*p[2],R[2][0]*p[0]+R[2][1]*p[1]+R[2][2]*p[2]];
function fitCanvas(c){const r=window.devicePixelRatio||1,w=Math.round(c.clientWidth*r),h=Math.round(c.clientHeight*r);if(c.width!==w||c.height!==h){c.width=w;c.height=h}return r}

// ===================== IMU state: attitude view + strip charts =====================
const ATT={rec:[],q:null,last:null,fields:null,az:-0.9,el:0.45,tile:null,ctl:null,dirty:true};
function makeAttTile(){
  const c3=el("canvas",{style:"height:260px;cursor:grab"}),cc=el("canvas",{style:"height:260px"});
  const ro=el("div",{class:"ro mono"}),st=el("span",{class:"st"});
  const box=el("div",{class:"tile viz"},el("div",{class:"hd"},el("b",{},"Orientation (IMU filter, 100 Hz)"),st),
    el("div",{class:"row2"},el("div",{},c3,ro,el("div",{class:"help"},"drag to orbit")),el("div",{},cc)));
  let drag=null;
  c3.onmousedown=e=>{drag=[e.clientX,e.clientY,ATT.az,ATT.el];c3.style.cursor="grabbing"};
  window.addEventListener("mouseup",()=>{drag=null;c3.style.cursor="grab"});
  window.addEventListener("mousemove",e=>{if(!drag)return;ATT.az=drag[2]-(e.clientX-drag[0])*0.01;
    ATT.el=Math.max(-1.5,Math.min(1.5,drag[3]+(e.clientY-drag[1])*0.01));ATT.dirty=true});
  ATT.tile={box,c3,cc,ro,st};return ATT.tile}
function attStart(){
  if(ATT.ctl)return;ATT.ctl=new AbortController();const sig=ATT.ctl.signal;
  (async()=>{while(!sig.aborted){try{await readStream("imu/stream",(h,body)=>{
      if(h.type!=="imu"||h.kind!=="state")return;ATT.fields=h.fields;
      const a=new Float64Array(body.buffer,0,body.byteLength/8),nf=h.fields.length,F={};h.fields.forEach((f,i)=>F[f]=i);
      for(let k=0;k<h.n;k++){const r=a.subarray(k*nf,(k+1)*nf);const q=[r[F.qw],r[F.qx],r[F.qy],r[F.qz]];
        ATT.q=q;ATT.last={t:r[F.t_end],q,flags:r[F.flags],used:r[F.accel_used],bias:[r[F.bgx],r[F.bgy],r[F.bgz]],
          g:[r[F.gx],r[F.gy],r[F.gz]],a:[r[F.ax],r[F.ay],r[F.az]],temp:r[F.temperature]};
        const e=rpy(q),wn=Math.hypot(r[F.gx],r[F.gy],r[F.gz])*180/Math.PI,an=Math.hypot(r[F.ax],r[F.ay],r[F.az]);
        ATT.rec.push([r[F.t_end],e[0],e[1],e[2],wn,an,r[F.flags]])}
      const t=ATT.rec.length?ATT.rec[ATT.rec.length-1][0]:0;while(ATT.rec.length&&ATT.rec[0][0]<t-10)ATT.rec.shift();
      ATT.dirty=true},sig)}catch(e){}if(!sig.aborted)await sleep(1000)}})()}
function attStop(){if(ATT.ctl){ATT.ctl.abort();ATT.ctl=null}}
function drawAtt(){
  const T=ATT.tile;if(!T||!ATT.dirty)return;ATT.dirty=false;
  // --- 3D attitude (orthographic): world z up; camera body = IMU frame (x right, y down, z out of the lens)
  const c=T.c3,g=c.getContext("2d"),dpr=fitCanvas(c),W=c.width,H=c.height;g.clearRect(0,0,W,H);
  const e=[Math.cos(ATT.el)*Math.cos(ATT.az),Math.cos(ATT.el)*Math.sin(ATT.az),Math.sin(ATT.el)];
  const r=[-Math.sin(ATT.az),Math.cos(ATT.az),0],u=[e[1]*r[2]-e[2]*r[1],e[2]*r[0]-e[0]*r[2],e[0]*r[1]-e[1]*r[0]];
  const s=Math.min(W,H)*0.33,P=p=>[W/2+(p[0]*r[0]+p[1]*r[1]+p[2]*r[2])*s,H/2-(p[0]*u[0]+p[1]*u[1]+p[2]*u[2])*s];
  const line=(a,b,col,lw)=>{const A=P(a),B=P(b);g.strokeStyle=col;g.lineWidth=(lw||1)*dpr;g.beginPath();g.moveTo(A[0],A[1]);g.lineTo(B[0],B[1]);g.stroke()};
  for(let i=-2;i<=2;i++){line([i*0.5,-1,-0.8],[i*0.5,1,-0.8],"#24303c");line([-1,i*0.5,-0.8],[1,i*0.5,-0.8],"#24303c")}
  line([0,0,-0.8],[0.5,0,-0.8],"#a33",2);line([0,0,-0.8],[0,0.5,-0.8],"#3a3",2);line([0,0,-0.8],[0,0,-0.3],"#36c",2);
  g.fillStyle="#8a96a3";g.font=(11*dpr)+"px system-ui";const L=(p,t)=>{const A=P(p);g.fillText(t,A[0]+3,A[1])};
  L([0.55,0,-0.8],"X");L([0,0.55,-0.8],"Y");L([0,0,-0.25],"Z up");
  if(ATT.q){
    const R=qToR(ATT.q),B=p=>mv(R,p);
    const bx=0.6,by=0.12,bz=0.15;const cor=[];for(const sx of[-1,1])for(const sy of[-1,1])for(const sz of[-1,1])cor.push(B([sx*bx,sy*by,sz*bz]));
    const edges=[[0,1],[2,3],[4,5],[6,7],[0,2],[1,3],[4,6],[5,7],[0,4],[1,5],[2,6],[3,7]];
    for(const[a,b]of edges)line(cor[a],cor[b],"#dde3ea",1.5);
    // lens + viewing frustum along +z (out of the lens)
    const f=[[-0.35,-0.2,0.8],[0.35,-0.2,0.8],[0.35,0.2,0.8],[-0.35,0.2,0.8]].map(B),o=B([0,0,bz]);
    for(const p of f)line(o,p,"#4ea1ff",1);for(let i=0;i<4;i++)line(f[i],f[(i+1)%4],"#4ea1ff",1);
    line(B([0,0,0]),B([0.45,0,0]),"#ff6b6b",2);line(B([0,0,0]),B([0,0.45,0]),"#4cd28a",2);line(B([0,0,0]),B([0,0,0.45]),"#4ea1ff",2);
    L(B([0.5,0,0]),"x");L(B([0,0.5,0]),"y");L(B([0,0,0.5]),"z (lens)");
    const l=ATT.last,E=rpy(ATT.q),fl=l.flags|0;
    T.ro.textContent=`roll ${E[0].toFixed(2)}°  pitch ${E[1].toFixed(2)}°  yaw ${E[2].toFixed(2)}°\n`+
      `${fl&4?"initializing":fl&1?"calibrated":"NOT calibrated"}${fl&2?", still":""}${fl&8?", GAP":""}  accel used ${(l.used*100).toFixed(0)}%\n`+
      `bias °/s ${l.bias.map(v=>(v*180/Math.PI).toFixed(3)).join(" ")}  ${l.temp.toFixed(1)} °C`;
    const age=OFF!=null?(serverNow()-l.t)*1000:null;T.st.textContent=age!=null?`sensor → display ${age.toFixed(0)} ms`:""}
  else T.ro.textContent="waiting for IMU filter records...";
  // --- strip charts, last 10 s
  const cc=T.cc,h=cc.getContext("2d");fitCanvas(cc);const CW=cc.width,CH=cc.height;h.clearRect(0,0,CW,CH);
  const R=ATT.rec;if(R.length<2)return;const t1=R[R.length-1][0],t0=t1-10;
  const panels=[{n:"roll / pitch °",k:[1,2],c:["#ff6b6b","#4cd28a"]},{n:"yaw °",k:[3],c:["#4ea1ff"]},
                {n:"|ω| °/s",k:[4],c:["#f5c04e"]},{n:"|a| m/s²",k:[5],c:["#c792ea"]}];
  const ph=CH/panels.length;h.font=(10*dpr)+"px system-ui";
  panels.forEach((p,i)=>{
    const y0=i*ph,pad=4*dpr;let lo=Infinity,hi=-Infinity;for(const z of R)for(const k of p.k){lo=Math.min(lo,z[k]);hi=Math.max(hi,z[k])}
    const span=Math.max(hi-lo,p.k[0]===4?0.2:p.k[0]===5?0.02:0.05),mid=(hi+lo)/2;lo=mid-span*0.6;hi=mid+span*0.6;
    h.fillStyle=i%2?"#0d1116":"#0b0e12";h.fillRect(0,y0,CW,ph);
    h.fillStyle="#1d4d3355";let s0=null;for(const z of R){const st=(z[6]|0)&2;if(st&&s0==null)s0=z[0];if((!st||z===R[R.length-1])&&s0!=null){
      h.fillRect((s0-t0)/10*CW,y0,(z[0]-s0)/10*CW,ph);s0=null}}
    p.k.forEach((k,j)=>{h.strokeStyle=p.c[j];h.lineWidth=1.2*dpr;h.beginPath();
      R.forEach((z,m)=>{const x=(z[0]-t0)/10*CW,y=y0+pad+(1-(z[k]-lo)/(hi-lo))*(ph-2*pad);m?h.lineTo(x,y):h.moveTo(x,y)});h.stroke()});
    h.fillStyle="#8a96a3";h.fillText(`${p.n}  ${R[R.length-1].filter((_,k)=>p.k.includes(k)).map(v=>v.toFixed(p.k[0]>=4?3:2)).join(" / ")}  [${lo.toFixed(2)} .. ${hi.toFixed(2)}]`,4*dpr,y0+11*dpr)});
  h.fillStyle="#8a96a3";h.fillText("10 s  (green band = still)",CW-120*dpr,CH-4*dpr);
}

// ===================== point cloud (WebGL2) =====================
const PC={ctl:null,tile:null,gl:null,prog:null,depthTex:null,colorTex:null,frame:null,newFrame:false,colorSeq:-1,
          cal:null,calVer:-1,az:0,el:0.25,zoom:1,med:2.0,medT:0,hr:[-1.5,1.5],n:0,fps:[],lat:null};
const PC_VS=`#version 300 es
precision highp float;precision highp int;precision highp usampler2D;
uniform usampler2D depthTex;uniform sampler2D colorTex;
uniform ivec2 dsize;uniform int stride;uniform vec4 dK;uniform float dscale;uniform mat4 model;uniform mat4 viewProj;
uniform mat3 Rdc;uniform vec3 tdc;uniform vec4 cK;uniform vec2 csize;uniform int colorMode;uniform float maxRange;uniform float psize;uniform int hasColor;uniform vec2 hrange;
out vec3 vcol;
vec3 turbo(float x){x=clamp(x,0.,1.);vec4 v4=vec4(1.,x,x*x,x*x*x);vec2 v2=v4.zw*v4.z;
  return vec3(dot(v4,vec4(0.13572138,4.61539260,-42.66032258,132.13108234))+dot(v2,vec2(-152.94239396,59.28637943)),
              dot(v4,vec4(0.09140261,2.19418839,4.84296658,-14.18503333))+dot(v2,vec2(4.27729857,2.82956604)),
              dot(v4,vec4(0.10667330,12.64194608,-60.58204836,110.36276771))+dot(v2,vec2(-89.90310912,27.34824973)));}
void main(){
  int cols=(dsize.x+stride-1)/stride;int u=(gl_VertexID%cols)*stride;int v=(gl_VertexID/cols)*stride;
  uint raw=texelFetch(depthTex,ivec2(u,v),0).r;float z=float(raw)*dscale;
  if(raw==0u||z>maxRange||v>=dsize.y){gl_Position=vec4(2.,2.,2.,1.);gl_PointSize=0.;vcol=vec3(0.);return;}
  vec3 p=vec3((float(u)-dK.z)/dK.x*z,(float(v)-dK.w)/dK.y*z,z);
  vec4 w=model*vec4(p,1.);gl_Position=viewProj*w;gl_PointSize=psize;
  if(colorMode==0&&hasColor==1){vec3 pc=Rdc*p+tdc;vec2 uv=vec2(cK.x*pc.x/pc.z+cK.z,cK.y*pc.y/pc.z+cK.w)/csize;
    vcol=(pc.z>0.&&uv.x>=0.&&uv.y>=0.&&uv.x<1.&&uv.y<1.)?texture(colorTex,uv).rgb:vec3(.35);}
  else if(colorMode==1)vcol=turbo((w.z-hrange.x)/max(hrange.y-hrange.x,0.01));
  else vcol=turbo(z/maxRange);
}`;
const PC_FS=`#version 300 es
precision mediump float;in vec3 vcol;out vec4 o;void main(){o=vec4(vcol,1.);}`;
function makePcTile(){
  const c=el("canvas",{style:"height:420px;cursor:grab"}),ro=el("div",{class:"ro mono"}),st=el("span",{class:"st"});
  const box=el("div",{class:"tile viz"},el("div",{class:"hd"},el("b",{},"Point cloud (raw depth, WebGL2)"),st),
    el("div",{class:"wrap"},c,ro,el("div",{class:"help"},"drag: orbit · wheel: zoom · double-click: reset")));
  let drag=null;
  c.onmousedown=e=>{drag=[e.clientX,e.clientY,PC.az,PC.el];c.style.cursor="grabbing"};
  window.addEventListener("mouseup",()=>{drag=null;c.style.cursor="grab"});
  window.addEventListener("mousemove",e=>{if(!drag)return;PC.az=drag[2]-(e.clientX-drag[0])*0.008;PC.el=Math.max(-1.4,Math.min(1.4,drag[3]+(e.clientY-drag[1])*0.008))});
  c.onwheel=e=>{e.preventDefault();PC.zoom=Math.max(0.05,Math.min(10,PC.zoom*Math.exp(e.deltaY*0.001)))};
  c.ondblclick=()=>{PC.az=0;PC.el=0.25;PC.zoom=1};
  PC.tile={box,c,ro,st};
  const gl=c.getContext("webgl2",{antialias:false});
  if(!gl){ro.textContent="WebGL2 is not available in this browser";return PC.tile}
  const sh=(t,src)=>{const s=gl.createShader(t);gl.shaderSource(s,src);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw new Error(gl.getShaderInfoLog(s));return s};
  try{const p=gl.createProgram();gl.attachShader(p,sh(gl.VERTEX_SHADER,PC_VS));gl.attachShader(p,sh(gl.FRAGMENT_SHADER,PC_FS));gl.linkProgram(p);
    if(!gl.getProgramParameter(p,gl.LINK_STATUS))throw new Error(gl.getProgramInfoLog(p));PC.prog=p}catch(e){ro.textContent="shader error: "+e.message;return PC.tile}
  PC.gl=gl;PC.vao=gl.createVertexArray();
  const tex=()=>{const t=gl.createTexture();gl.bindTexture(gl.TEXTURE_2D,t);for(const[k,v]of[[gl.TEXTURE_MIN_FILTER,gl.NEAREST],[gl.TEXTURE_MAG_FILTER,gl.NEAREST],[gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE],[gl.TEXTURE_WRAP_T,gl.CLAMP_TO_EDGE]])gl.texParameteri(gl.TEXTURE_2D,k,v);return t};
  PC.depthTex=tex();PC.colorTex=tex();
  gl.bindTexture(gl.TEXTURE_2D,PC.colorTex);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.LINEAR);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.LINEAR);
  return PC.tile}
function pcStart(){
  if(PC.ctl)return;PC.ctl=new AbortController();const sig=PC.ctl.signal;
  (async()=>{while(!sig.aborted){try{await readStream("stream?streams=depth&mode=raw",(h,body)=>{
      if(h.type!=="frame"||h.stream!=="depth")return;PC.frame={h,body};PC.newFrame=true;
      const now=performance.now();PC.fps.push(now);PC.fps=PC.fps.filter(t=>now-t<2000)},sig)}catch(e){}if(!sig.aborted)await sleep(1000)}})()}
function pcStop(){if(PC.ctl){PC.ctl.abort();PC.ctl=null}}
async function pcCalibration(){if(!S||PC.calVer===S.settings_version)return;PC.calVer=S.settings_version;try{PC.cal=await api("calibration")}catch(e){PC.cal=null}}
function m4mul(a,b){const o=new Float32Array(16);for(let i=0;i<4;i++)for(let j=0;j<4;j++){let s=0;for(let k=0;k<4;k++)s+=a[k*4+j]*b[i*4+k];o[i*4+j]=s}return o}
function persp(fov,asp,n,f){const t=1/Math.tan(fov/2),o=new Float32Array(16);o[0]=t/asp;o[5]=t;o[10]=(f+n)/(n-f);o[11]=-1;o[14]=2*f*n/(n-f);return o}
function lookAt(eye,c,up){const z=[eye[0]-c[0],eye[1]-c[1],eye[2]-c[2]],nz=Math.hypot(...z);z.forEach((v,i)=>z[i]=v/nz);
  const x=[up[1]*z[2]-up[2]*z[1],up[2]*z[0]-up[0]*z[2],up[0]*z[1]-up[1]*z[0]],nx=Math.hypot(...x);x.forEach((v,i)=>x[i]=v/nx);
  const y=[z[1]*x[2]-z[2]*x[1],z[2]*x[0]-z[0]*x[2],z[0]*x[1]-z[1]*x[0]];const d=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
  return new Float32Array([x[0],y[0],z[0],0,x[1],y[1],z[1],0,x[2],y[2],z[2],0,-d(x,eye),-d(y,eye),-d(z,eye),1])}
function drawPc(){
  const gl=PC.gl,T=PC.tile;if(!gl||!PC.frame)return;
  const{h,body}=PC.frame,w=h.width,ht=h.height;
  if(PC.newFrame){PC.newFrame=false;gl.bindTexture(gl.TEXTURE_2D,PC.depthTex);gl.pixelStorei(gl.UNPACK_ALIGNMENT,2);
    gl.texImage2D(gl.TEXTURE_2D,0,gl.R16UI,w,ht,0,gl.RED_INTEGER,gl.UNSIGNED_SHORT,new Uint16Array(body.buffer,0,w*ht))}
  const ct=tiles.color;let hasColor=0;
  if(ct&&ct.img.complete&&ct.img.naturalWidth){hasColor=1;if(ct.meta&&ct.meta.seq!==PC.colorSeq){PC.colorSeq=ct.meta.seq;
    gl.bindTexture(gl.TEXTURE_2D,PC.colorTex);gl.pixelStorei(gl.UNPACK_ALIGNMENT,4);gl.texImage2D(gl.TEXTURE_2D,0,gl.RGBA,gl.RGBA,gl.UNSIGNED_BYTE,ct.img)}}
  const cal=PC.cal||{},aligned=!!h.aligned||cal.aligned_depth;
  let dK=aligned?cal.color:cal.depth;if(!dK){T.ro.textContent="waiting for /calibration...";return}
  const sx=w/dK.width,sy=ht/dK.height;const K=[dK.fx*sx,dK.fy*sy,dK.cx*sx,dK.cy*sy];
  const cK=cal.color||{fx:1,fy:1,cx:0,cy:0,width:1,height:1};
  const ex=aligned?{rotation:[[1,0,0],[0,1,0],[0,0,1]],translation_mm:[0,0,0]}:(cal.depth_to_color||{rotation:[[1,0,0],[0,1,0],[0,0,1]],translation_mm:[0,0,0]});
  // model: depth frame (x right, y down, z forward) -> world (z up).
  let M;const level=$("#pc_level").checked&&ATT.q;
  if(level){ // world <- IMU (filter q) <- depth (inverse of accel_to_depth; rotation only, the offset is ~16 mm)
    const Rq=qToR(ATT.q),Ead=(cal.imu&&cal.imu.accel_to_depth)?cal.imu.accel_to_depth.rotation:[[1,0,0],[0,1,0],[0,0,1]];
    const Rda=[[Ead[0][0],Ead[1][0],Ead[2][0]],[Ead[0][1],Ead[1][1],Ead[2][1]],[Ead[0][2],Ead[1][2],Ead[2][2]]];
    const R=[0,1,2].map(i=>[0,1,2].map(j=>Rq[i][0]*Rda[0][j]+Rq[i][1]*Rda[1][j]+Rq[i][2]*Rda[2][j]));
    M=new Float32Array([R[0][0],R[1][0],R[2][0],0,R[0][1],R[1][1],R[2][1],0,R[0][2],R[1][2],R[2][2],0,0,0,0,1])}
  else M=new Float32Array([1,0,0,0, 0,0,-1,0, 0,1,0,0, 0,0,0,1]);   // x right, z forward -> y forward, -y down -> z up
  const c=T.c,dpr=fitCanvas(c);gl.viewport(0,0,c.width,c.height);gl.clearColor(0.043,0.055,0.07,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.enable(gl.DEPTH_TEST);
  // Orbit around the cloud: target = median depth along the optical axis; the default eye sits
  // behind the camera looking along its axis (drag adds azimuth/elevation, wheel scales distance).
  const R3=[[M[0],M[4],M[8]],[M[1],M[5],M[9]],[M[2],M[6],M[10]]],fwd=mv(R3,[0,0,1]),camUp=mv(R3,[0,-1,0]);
  const now=performance.now();if(now-PC.medT>1000){PC.medT=now;const d=new Uint16Array(body.buffer,0,w*ht),s=[],hz=[],sc=(h.depth_scale||1)*0.001;
    for(let i=0;i<d.length;i+=97)if(d[i]){const z=d[i]*sc;s.push(z);const u=i%w,v=(i/w)|0;
      hz.push(R3[2][0]*(u-K[2])/K[0]*z+R3[2][1]*(v-K[3])/K[1]*z+R3[2][2]*z)}
    if(s.length){s.sort((a,b)=>a-b);hz.sort((a,b)=>a-b);PC.med=s[s.length>>1];PC.hr=[hz[Math.floor(hz.length*0.02)],hz[Math.floor(hz.length*0.98)]]}}
  const tgt=fwd.map(v=>v*PC.med),dist=PC.med*1.35*PC.zoom;
  const az0=Math.atan2(-fwd[1],-fwd[0]),el0=Math.asin(Math.max(-1,Math.min(1,-fwd[2])));
  const az=az0+PC.az,el=Math.max(-1.55,Math.min(1.55,el0+PC.el));
  const dir=[Math.cos(el)*Math.cos(az),Math.cos(el)*Math.sin(az),Math.sin(el)],eye=dir.map((v,i)=>tgt[i]+v*dist);
  const up=Math.abs(dir[2])>0.9?camUp:[0,0,1];
  const VP=m4mul(persp(1.0,c.width/c.height,0.05,80),lookAt(eye,tgt,up));
  gl.useProgram(PC.prog);gl.bindVertexArray(PC.vao);const U=n=>gl.getUniformLocation(PC.prog,n);
  gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,PC.depthTex);gl.uniform1i(U("depthTex"),0);
  gl.activeTexture(gl.TEXTURE1);gl.bindTexture(gl.TEXTURE_2D,PC.colorTex);gl.uniform1i(U("colorTex"),1);
  const stride=Number($("#pc_stride").value);gl.uniform2i(U("dsize"),w,ht);gl.uniform1i(U("stride"),stride);gl.uniform4f(U("dK"),...K);
  gl.uniform1f(U("dscale"),(h.depth_scale||1)*0.001);gl.uniformMatrix4fv(U("model"),false,M);gl.uniformMatrix4fv(U("viewProj"),false,VP);
  const Rr=ex.rotation;gl.uniformMatrix3fv(U("Rdc"),false,new Float32Array([Rr[0][0],Rr[1][0],Rr[2][0],Rr[0][1],Rr[1][1],Rr[2][1],Rr[0][2],Rr[1][2],Rr[2][2]]));
  gl.uniform3f(U("tdc"),...ex.translation_mm.map(v=>v*0.001));gl.uniform4f(U("cK"),cK.fx,cK.fy,cK.cx,cK.cy);gl.uniform2f(U("csize"),cK.width,cK.height);
  gl.uniform1i(U("colorMode"),Number($("#pc_color").value));gl.uniform2f(U("hrange"),PC.hr[0],PC.hr[1]);gl.uniform1i(U("hasColor"),hasColor);
  gl.uniform1f(U("maxRange"),Number($("#pc_max").value));gl.uniform1f(U("psize"),Number($("#pc_size").value)*dpr);
  const n=Math.ceil(w/stride)*Math.ceil(ht/stride);gl.drawArrays(gl.POINTS,0,n);
  const fps=PC.fps.length>1?(PC.fps.length-1)*1000/(PC.fps[PC.fps.length-1]-PC.fps[0]):0,lat=OFF!=null?(serverNow()-h.t_capture)*1000:null;
  T.st.textContent=`${w}x${ht}${aligned?" aligned":""} · ${n.toLocaleString()} points · ${fps.toFixed(1)} fps · capture→now ${lat!=null?lat.toFixed(0):"-"} ms`;
  T.ro.textContent=(level?"gravity-levelled (IMU), z up":"camera frame, z up = −y")+(Number($("#pc_color").value)===0&&!hasColor?"\n(enable the color stream for camera colour)":"");
}

// ===================== wiring =====================
function vizSync(){
  const box=$("#tiles");
  if($("#v_att").checked){if(!ATT.tile)makeAttTile();if(!ATT.tile.box.isConnected)box.prepend(ATT.tile.box);attStart()}
  else{attStop();if(ATT.tile)ATT.tile.box.remove()}
  if($("#v_pc").checked){if(!PC.tile)makePcTile();if(!PC.tile.box.isConnected)box.prepend(PC.tile.box);pcStart();pcCalibration()}
  else{pcStop();if(PC.tile)PC.tile.box.remove()}
}
for(const id of["v_att","v_pc"])$("#"+id).onchange=vizSync;
function vizLoop(){try{if(ATT.tile&&ATT.tile.box.isConnected)drawAtt();if(PC.tile&&PC.tile.box.isConnected){pcCalibration();drawPc()}}catch(e){console.error(e)}requestAnimationFrame(vizLoop)}
setTimeout(()=>{vizSync();requestAnimationFrame(vizLoop)},300);
"""
