"""Shared visualisation components, served by every node at ``common/viz.js``.

Kept as a Python string, like ui_assets.py, so it ships with the package. Pages load it after the
toolkit with a RELATIVE URL::

    <script src="common/ui.js"></script>
    <script src="common/viz.js"></script>

Every component lives on ``window.RU.viz`` and follows one pattern, so tool pages, the landing page
and aggregating nodes can reuse them without knowing where the data came from:

* ``new RU.viz.X(el, opts)``: ``el`` is a container (the component creates its canvas and caption
  inside) or a ``<canvas>`` (drawn into as is). ``opts`` carries geometry / scales; every option
  has a default.
* ``c.update(data)``: plain data only (numbers, NaN / null for "unknown"); components never open
  streams or read row records themselves. ``update`` is cheap: it stores the data and schedules one
  redraw on the next animation frame, so it may be called at any rate (e.g. 100 Hz).
* ``c.draw()``: redraw now (normally not needed). ``c.destroy()``: stop observing / timers.
* Helpers that turn a node's telemetry into ``data`` are static: ``RU.viz.X.fromRow(get)`` where
  ``get(name)`` returns the newest value of a field (``n => d.last(n)`` with ``RU.rows``).
* Canvas, devicePixelRatio aware, resized with a ResizeObserver; fixed aspect set in CSS.
* Colours come from CSS custom properties read on ``document.documentElement`` (``RU.token(name,
  fallback)`` from ui.js when present), refreshed twice a second, so themes restyle components:
  ``--signal`` measured / interactive, ``--ink`` structure, ``--muted`` ghosted / secondary,
  ``--live`` motion while enabled, ``--stop`` faulted / stale / e-stop, ``--rule`` faint lines.
  Fallbacks differ for light and dark mode.
* The component injects its own small stylesheet once (no ``common/viz.css`` to include).

Components
----------
``RU.viz.Swerve(el, opts)``: to-scale top view of a swerve base (x forward is up, y left is left).
  opts: ``modules`` [{name, x, y}] (m, wheel positions; default: this robot, constants.py),
  ``frame`` (m, square side), ``tube_w`` (m, frame tube width seen from above), ``corner`` (m,
  frame edge taken by each corner module, per side), ``plate`` ([[x, y], ...] m, the module's
  top-plate outline about its steering axis, drawn as is for a module at +x +y and mirrored for
  the others), ``wheel_d`` / ``wheel_w`` (m),
  ``half`` (m shown each side of the centre, 0.40), ``speedScale`` (m of arrow per m/s, 0.3),
  ``staleAfter`` (s without update() before everything greys out, 0.6), ``caption`` (true: legend
  and status line under the canvas).
  data: ``{FL: {angle, speed, angle_sp, speed_sp, drive_a, steer_a, stale}, FR: ..., twist:
  [vx, vy, omega], enabled, driving, estop}``; angle rad (0 = forward, CCW +), speed m/s (negative
  = reversed), ``*_sp`` the setpoints (NaN when nothing is commanded), currents in A, ``stale``
  true when that module's status frames are old; ``twist`` the commanded body twist (NaN when
  stopped); ``enabled`` the node's enable flag, ``driving`` motors enabled this tick.
  Wireframe style: everything is stroked, nothing filled. Draws the frame tubes and module plates,
  the wheel at its measured angle with a velocity vector from the leading edge, the setpoint
  as a dashed ghost, the twist as an arrow and rotation arc at the centre, and drive / steering
  currents beside each corner.
  ``RU.viz.Swerve.fromRow(get, {modules, staleAfter})``: data from the swerve node's stream fields
  (``<M>_steer_angle``, ``<M>_drive_vel``, ``<M>_steer_sp``, ``<M>_drive_sp``, ``<M>_drive_current``,
  ``<M>_steer_current``, ``<M>_t_steer`` / ``t_drive`` vs ``t_tick`` for staleness, ``cmd_*``,
  ``enabled``, ``motors_enabled``, ``estop``).
``RU.viz.Battery(el, opts)``: a battery gauge: level fill, charging bolt, input plug, per-cell bars.
  opts: ``low`` (%, 25: caution colour at or below), ``critical`` (%, 10: stop colour at or below),
  ``cellMin`` / ``cellMax`` (V, 3.0 / 4.2: the cell bars' range), ``staleAfter`` (s without update()
  before it greys out, 10), ``caption`` (true: status line under the canvas), ``label`` (aria label),
  ``compact`` (false; true: a ~24 px tall strip for tiles and overviews: no caption, no cells, the
  percentage drawn inside the gauge, the charging bolt and an input plug kept).
  data: ``{percent, charging, full, input, volts, current, cells: [V, ...], runtime_min, stale, low}``;
  percent 0-100 (null = unknown), ``current`` A (+ = charging), ``input`` true when external power is
  present, ``stale`` true when the reading is old, ``low`` true to force the stop colour (e.g. a
  low-voltage alarm). Colours: ``--live`` healthy, ``--caution`` low, ``--stop`` critical / low
  alarm, ``--signal`` charging, ``--muted`` stale or unknown.
  ``RU.viz.Battery.fromSource(src)``: data from one entry of the power node's ``/state`` ``sources``
  (``percent``, ``pack_v``, ``current_a``, ``cells_v``, ``charge_state``, ``input_present``,
  ``runtime_min``, ``ok``, ``stale``); every battery source shares those fields (power/sources.py).
"""

VIZ_JS = r"""
(function(){
"use strict";
const RU = window.RU = window.RU || {};
const viz = RU.viz = RU.viz || {};

// -- shared helpers ------------------------------------------------------------------------------
const darkMQ = window.matchMedia ? matchMedia("(prefers-color-scheme: dark)") : {matches: false};
function isDark(){ const t = document.documentElement.dataset.theme;
  return t === "dark" || (darkMQ.matches && t !== "light"); }
function token(name, light, dark){
  const fb = isDark() ? (dark || light) : light;
  if (typeof RU.token === "function"){ const v = RU.token(name, ""); if (v) return String(v).trim(); }
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fb;
}
function palette(){
  const fg = token("--fg", "#1d1f23", "#e8eaed");
  return {fg, signal: token("--signal", "#1F5FBF", "#6EA0F0"), ink: token("--ink", fg, fg),
          muted: token("--muted", "#6b7280", "#9aa0a6"), live: token("--live", "#157A45", "#3FBF7A"),
          stop: token("--stop", "#C41E17", "#F0625C"), rule: token("--rule", "#c9ccd1", "#3a3f46"),
          panel: token("--panel", "#FFFFFF", "#1E242B")};
}
function injectCss(id, css){
  if (document.getElementById(id)) return;
  const s = document.createElement("style"); s.id = id; s.textContent = css; document.head.appendChild(s);
}
const fin = v => v !== null && v !== undefined && isFinite(v);
viz.palette = palette;

// -- Swerve: top-down module diagram ---------------------------------------------------------------
const IN = 0.0254;
// MAXSwerve top plate about the steering axis (inches, module at +x +y), traced from REV's CAD
// (REV-21-3005, part P02): a 3.5 in square with the axis at its centre, two 1 in tabs along the
// outer edges reaching 3.49 in inward (they bolt onto the 2x1 tubes), and an r 1.904 in arc on the
// inner corner.
const PLATE_IN = (() => {
  const P = [[1.75, 1.75], [-3.49, 1.75], [-3.49, 0.75], [-1.75, 0.75]];
  const a0 = Math.atan2(0.75, -1.75), a1 = Math.atan2(-1.75, 0.75) + 2 * Math.PI, r = 1.904;
  for (let i = 1; i < 16; i++){ const a = a0 + (a1 - a0) * i / 16; P.push([r * Math.cos(a), r * Math.sin(a)]); }
  return P.concat([[0.75, -1.75], [0.75, -3.49], [1.75, -3.49]]);
})();
// This robot: 17 in square of 2x1 tube (10 in tubes between 3.5 in corner modules). Wheel positions
// are the kinematic ones from swerve/constants.py (6.52 in from the centre; the CAD and the 3.5 in
// corners put the steering axes at 6.75 in).
const SWERVE_DEFAULTS = {
  frame: 17 * IN, tube_w: 1 * IN, corner: 3.5 * IN, wheel_d: 3 * IN, wheel_w: 1.2 * IN,
  plate: PLATE_IN.map(([x, y]) => [x * IN, y * IN]),
  modules: [{name: "FL", x: 0.165608, y: 0.165608}, {name: "FR", x: 0.165608, y: -0.165608},
            {name: "RR", x: -0.165608, y: -0.165608}, {name: "RL", x: -0.165608, y: 0.165608}],
  half: 0.40, speedScale: 0.3, maxArrow: 0.19, staleAfter: 0.6, caption: true,
};
const SWERVE_CSS = `
.ru-swerve{position:relative;max-width:480px;margin:6px auto 0}
.ru-swerve canvas{display:block;width:100%;aspect-ratio:1/1;touch-action:none}
.ru-swerve .cap{font-size:11px;color:var(--muted,#6b7280);display:flex;justify-content:space-between;gap:8px;flex-wrap:wrap}
.ru-swerve .key{display:inline-flex;align-items:center;gap:4px;margin-right:8px}
.ru-swerve .key i{display:inline-block;width:14px;height:0;border-top:2px solid}`;

class Swerve {
  constructor(el, opts){
    this.o = Object.assign({}, SWERVE_DEFAULTS, opts || {});
    injectCss("ru-viz-swerve-css", SWERVE_CSS);
    if (el.tagName === "CANVAS"){ this.cv = el; this.note = null; }
    else {
      el.classList.add("ru-swerve");
      el.innerHTML = '<canvas role="img" aria-label="Top view of the swerve base: wheel angles, speeds and setpoints"></canvas>' +
        (this.o.caption ? '<div class="cap"><span><span class="key"><i style="border-color:var(--signal,#1F5FBF)"></i>measured</span>' +
        '<span class="key"><i style="border-top-style:dashed;border-color:var(--muted,#6b7280)"></i>setpoint</span>' +
        '<span class="key"><i style="border-color:var(--live,#157A45)"></i>command</span></span><span data-k="note"></span></div>' : "");
      this.cv = el.querySelector("canvas"); this.note = el.querySelector('[data-k="note"]');
    }
    this.ctx = this.cv.getContext("2d");
    this.data = null; this.at = 0; this.W = 0; this.col = null; this.colAt = 0; this.raf = 0; this.timer = 0;
    this.ro = window.ResizeObserver ? new ResizeObserver(() => this.schedule()) : null;
    if (this.ro) this.ro.observe(this.cv);
    this.schedule();
  }
  update(data){
    this.data = data; this.at = performance.now(); this.schedule();
    clearTimeout(this.timer);        // grey out when updates stop
    this.timer = setTimeout(() => this.schedule(), this.o.staleAfter * 1000 + 50);
  }
  schedule(){ if (!this.raf) this.raf = requestAnimationFrame(() => { this.raf = 0; this.draw(); }); }
  destroy(){ if (this.ro) this.ro.disconnect(); clearTimeout(this.timer); cancelAnimationFrame(this.raf); }

  // robot (x forward, y left, m) -> screen px (x up, y left)
  S(){ return this.W / (2 * this.o.half); }
  P(x, y){ const s = this.S(); return [this.W / 2 - y * s, this.W / 2 - x * s]; }

  arrow(x0, y0, dx, dy, colour, width, dash){
    const ctx = this.ctx, max = this.o.maxArrow;
    let L = Math.hypot(dx, dy);
    if (!(L > 1e-4)) return;
    const clip = L > max;
    if (clip){ dx *= max / L; dy *= max / L; L = max; }
    const [a, b] = this.P(x0, y0), [c, d] = this.P(x0 + dx, y0 + dy);
    const h = Math.min(8, Math.max(4, L * this.S() * 0.4)), ang = Math.atan2(d - b, c - a);
    ctx.strokeStyle = ctx.fillStyle = colour; ctx.lineWidth = width; ctx.setLineDash(dash || []);
    ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c - Math.cos(ang) * h * 0.6, d - Math.sin(ang) * h * 0.6); ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath(); ctx.moveTo(c, d);
    ctx.lineTo(c - h * Math.cos(ang - 0.45), d - h * Math.sin(ang - 0.45));
    ctx.lineTo(c - h * Math.cos(ang + 0.45), d - h * Math.sin(ang + 0.45)); ctx.closePath(); ctx.fill();
    if (clip){ const nx = -Math.sin(ang) * h, ny = Math.cos(ang) * h; ctx.lineWidth = 2;   // bar: clipped
      ctx.beginPath(); ctx.moveTo(c + nx, d + ny); ctx.lineTo(c - nx, d - ny); ctx.stroke(); }
  }
  wheelArrow(m, th, v, colour, width, dash){      // from the wheel's leading edge
    const c = Math.cos(th), s = Math.sin(th), o = Math.sign(v) * this.o.wheel_d / 2, k = this.o.speedScale;
    this.arrow(m.x + c * o, m.y + s * o, c * v * k, s * v * k, colour, width, dash);
  }
  wheel(x, y, th, stroke, dash){                   // th: rad, 0 = forward, CCW +; outline only
    const ctx = this.ctx, [px, py] = this.P(x, y), s = this.S(), l = this.o.wheel_d * s, w = Math.max(5, this.o.wheel_w * s);
    ctx.save(); ctx.translate(px, py); ctx.rotate(-th);     // canvas rotation is clockwise-positive
    ctx.strokeStyle = stroke; ctx.lineWidth = dash ? 1.25 : 2; ctx.setLineDash(dash || []);
    ctx.beginPath(); ctx.rect(-w / 2, -l / 2, w, l); ctx.stroke(); ctx.setLineDash([]);
    if (!dash){   // a chevron at the wheel's forward end, so 0 and 180 degrees differ at standstill
      const c = Math.min(w * 0.32, l * 0.2);
      ctx.lineWidth = 1.5; ctx.beginPath();
      ctx.moveTo(-c, -l / 2 + c * 1.9); ctx.lineTo(0, -l / 2 + c * 0.9); ctx.lineTo(c, -l / 2 + c * 1.9); ctx.stroke();
    }
    ctx.restore();
  }
  text(str, x, y, colour, align, size, bold){
    const ctx = this.ctx;
    ctx.font = `${bold ? "600 " : ""}${size || 11}px ui-monospace,SFMono-Regular,Menlo,monospace`;
    ctx.fillStyle = colour; ctx.textAlign = align || "center"; ctx.textBaseline = "middle"; ctx.fillText(str, x, y);
  }

  draw(){
    const cv = this.cv, ctx = this.ctx, o = this.o, dpr = window.devicePixelRatio || 1, w0 = cv.clientWidth;
    if (!w0) return;
    if (cv.width !== Math.round(w0 * dpr)){ cv.width = Math.round(w0 * dpr); cv.height = Math.round(w0 * dpr); }
    this.W = w0; ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const now = performance.now();
    if (!this.col || now - this.colAt > 500){ this.col = palette(); this.colAt = now; }
    const col = this.col, W = this.W, s = this.S(), D = this.data || {};
    ctx.clearRect(0, 0, W, W);
    const live = !!this.data && (now - this.at) / 1000 < o.staleAfter;
    const driving = live && !!D.driving, estop = live && !!D.estop, small = W < 340;

    // frame: four tubes between the corner modules, the module plates, front marker, centre cross
    const F = o.frame / 2, [fx, fy] = this.P(F, F), T = o.tube_w, C = F - o.corner;
    ctx.strokeStyle = estop ? col.stop : col.ink; ctx.lineWidth = 1.25;
    const rect = (x0, y0, x1, y1) => { const [a, b] = this.P(x1, y1), [c, d] = this.P(x0, y0); ctx.strokeRect(a, b, c - a, d - b); };
    rect(F - T, -C, F, C); rect(-F, -C, -F + T, C); rect(-C, F - T, C, F); rect(-C, -F, C, -F + T);
    for (const m of o.modules){
      const sx = m.x < 0 ? -1 : 1, sy = m.y < 0 ? -1 : 1;
      ctx.beginPath();
      o.plate.forEach(([u, v], i) => { const [px, py] = this.P(m.x + sx * u, m.y + sy * v); i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
      ctx.closePath(); ctx.stroke();
      const [ax, ay] = this.P(m.x, m.y);                     // steering axis
      ctx.beginPath(); ctx.arc(ax, ay, 1.5, 0, 2 * Math.PI); ctx.stroke();
    }
    const [tx, ty] = this.P(F, 0);
    ctx.beginPath(); ctx.moveTo(tx, ty - 7); ctx.lineTo(tx - 6, ty - 1); ctx.lineTo(tx + 6, ty - 1); ctx.closePath(); ctx.stroke();
    this.text("front", tx, ty - 15, col.muted, "center", 10);
    const [cx, cy] = this.P(0, 0);
    ctx.strokeStyle = col.muted; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(cx - 4, cy); ctx.lineTo(cx + 4, cy); ctx.moveTo(cx, cy - 4); ctx.lineTo(cx, cy + 4); ctx.stroke();

    // modules
    let staleN = 0;
    for (const m of o.modules){
      const M = D[m.name] || {}, stale = live && !!M.stale;
      if (stale) staleN++;
      if (live && fin(M.angle_sp)){
        this.wheel(m.x, m.y, M.angle_sp, col.muted, [3, 2]);
        if (fin(M.speed_sp)) this.wheelArrow(m, M.angle_sp, M.speed_sp, col.muted, 1.5, [4, 3]);
      }
      if (fin(M.angle)){
        this.wheel(m.x, m.y, M.angle, !live ? col.muted : stale ? col.stop : col.signal);
        if (live && fin(M.speed)) this.wheelArrow(m, M.angle, M.speed, stale ? col.stop : driving ? col.live : col.signal, 2.25);
      } else {
        const [px, py] = this.P(m.x, m.y); this.text("?", px, py, live ? col.stop : col.muted, "center", 13, true);
      }
      // label beside the frame: name, drive and steering current
      const left = m.y > 0, ly = this.P(m.x, m.y)[1], lx = left ? fx - 6 : fx + 2 * F * s + 6;
      const al = left ? "right" : "left", fs = small ? 10 : 11, lh = fs + 3;
      const A = a => fin(a) ? a.toFixed(Math.abs(a) < 10 ? 1 : 0) : "–";
      this.text(m.name + (stale ? " stale" : ""), lx, ly - lh, stale ? col.stop : col.fg, al, fs, true);
      this.text("d " + A(M.drive_a) + " A", lx, ly, col.muted, al, fs);
      this.text("s " + A(M.steer_a) + " A", lx, ly + lh, col.muted, al, fs);
    }

    // commanded body twist: translation arrow + rotation arc (NaN when stopped)
    const tw = D.twist || [], vx = tw[0], vy = tw[1], w = tw[2], tc = driving ? col.live : col.muted;
    if (live && fin(vx) && fin(vy)) this.arrow(0, 0, vx * o.speedScale, vy * o.speedScale, tc, 3);
    if (live && fin(w) && Math.abs(w) > 1e-3){
      const r = 0.05 * s, sweep = Math.sign(w) * Math.min(0.35 + Math.abs(w) * 1.2, 1.7 * Math.PI);
      const a0 = -Math.PI / 2, a1 = a0 - sweep;          // canvas angles: CCW on screen is negative
      ctx.strokeStyle = ctx.fillStyle = tc; ctx.lineWidth = 2.5;
      ctx.beginPath(); ctx.arc(cx, cy, r, a0, a1, w > 0); ctx.stroke();
      const ex = cx + r * Math.cos(a1), ey = cy + r * Math.sin(a1), t = a1 + (w > 0 ? -Math.PI / 2 : Math.PI / 2), h = 7;
      ctx.beginPath(); ctx.moveTo(ex + Math.cos(t) * h, ey + Math.sin(t) * h);
      ctx.lineTo(ex + Math.cos(t + 2.4) * h * 0.8, ey + Math.sin(t + 2.4) * h * 0.8);
      ctx.lineTo(ex + Math.cos(t - 2.4) * h * 0.8, ey + Math.sin(t - 2.4) * h * 0.8); ctx.closePath(); ctx.fill();
    }

    // scale bar (bottom left), twist readout (bottom right)
    const bx = 8, by = W - 10, bl = 0.5 * o.speedScale * s;
    ctx.strokeStyle = col.muted; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(bx, by - 3); ctx.lineTo(bx, by); ctx.lineTo(bx + bl, by); ctx.lineTo(bx + bl, by - 3); ctx.stroke();
    this.text("0.5 m/s", bx + bl + 5, by - 1, col.muted, "left", 10);
    const wt = fin(w) ? w.toFixed(2) : "–";
    this.text(!(live && fin(vx)) ? "no command" : small ? `cmd ${vx.toFixed(2)} ${vy.toFixed(2)} ${wt}` :
              `cmd ${vx.toFixed(2)}, ${vy.toFixed(2)} m/s ${wt} rad/s`,
              W - 6, by - 1, col.muted, "right", 10);

    const status = !this.data ? "waiting for data" : !live ? "data stale" : estop ? "E-STOP" :
                   staleN ? staleN + " module(s) stale" : driving ? "driving" : D.enabled ? "enabled, idle" : "disabled";
    if (estop || staleN || !live) this.text(status, W / 2, 12, estop || staleN ? col.stop : col.muted, "center", 12, true);
    this.status = status;
    if (this.note && this.note.textContent !== status) this.note.textContent = status;
  }

  static fromRow(get, opts){
    opts = opts || {};
    const mods = (opts.modules || SWERVE_DEFAULTS.modules).map(m => m.name || m), stale = opts.staleAfter || 0.5;
    const t = get("t_tick"), d = {twist: [get("cmd_vx"), get("cmd_vy"), get("cmd_omega")],
      enabled: get("enabled") > 0.5, driving: get("motors_enabled") > 0.5, estop: get("estop") > 0.5};
    for (const N of mods){
      const age = Math.max(t - get(N + "_t_steer"), t - get(N + "_t_drive"));
      d[N] = {angle: get(N + "_steer_angle"), speed: get(N + "_drive_vel"), angle_sp: get(N + "_steer_sp"),
              speed_sp: get(N + "_drive_sp"), drive_a: get(N + "_drive_current"), steer_a: get(N + "_steer_current"),
              stale: !(age < stale)};
    }
    return d;
  }
}
Swerve.defaults = SWERVE_DEFAULTS;
viz.Swerve = Swerve;

// -- Battery: level gauge ----------------------------------------------------------------------------
const BATTERY_DEFAULTS = {low: 25, critical: 10, cellMin: 3.0, cellMax: 4.2, staleAfter: 10, caption: true,
                          label: "Battery level", compact: false};
const BATTERY_CSS = `
.ru-battery{position:relative;max-width:420px}
.ru-battery canvas{display:block;width:100%;aspect-ratio:7/2}
.ru-battery .cap{font-size:12px;color:var(--muted,#6b7280);min-height:1.2em}
.ru-battery.compact{max-width:140px}
.ru-battery.compact canvas{aspect-ratio:auto;height:24px}`;

class Battery {
  constructor(el, opts){
    this.o = Object.assign({}, BATTERY_DEFAULTS, opts || {});
    injectCss("ru-viz-battery-css", BATTERY_CSS);
    if (el.tagName === "CANVAS"){ this.cv = el; this.note = null; }
    else {
      el.classList.add("ru-battery"); el.classList.toggle("compact", !!this.o.compact);
      el.innerHTML = '<canvas role="img"></canvas>' + (this.o.caption && !this.o.compact ? '<div class="cap" data-k="note"></div>' : "");
      this.cv = el.querySelector("canvas"); this.note = el.querySelector('[data-k="note"]');
    }
    this.cv.setAttribute("aria-label", this.o.label);
    this.ctx = this.cv.getContext("2d");
    this.data = null; this.at = 0; this.col = null; this.colAt = 0; this.raf = 0; this.timer = 0;
    this.ro = window.ResizeObserver ? new ResizeObserver(() => this.schedule()) : null;
    if (this.ro) this.ro.observe(this.cv);
    this.schedule();
  }
  update(data){
    this.data = data; this.at = performance.now(); this.schedule();
    clearTimeout(this.timer);
    this.timer = setTimeout(() => this.schedule(), this.o.staleAfter * 1000 + 50);
  }
  schedule(){ if (!this.raf) this.raf = requestAnimationFrame(() => { this.raf = 0; this.draw(); }); }
  destroy(){ if (this.ro) this.ro.disconnect(); clearTimeout(this.timer); cancelAnimationFrame(this.raf); }

  // the level colour and a one-line status, from data (also used for the caption / aria label)
  assess(){
    const D = this.data || {}, o = this.o, col = this.col || palette();
    const live = !!this.data && (performance.now() - this.at) / 1000 < o.staleAfter && !D.stale;
    const p = fin(D.percent) ? Math.max(0, Math.min(100, D.percent)) : null;
    let c = col.muted, status;
    if (!this.data) status = "waiting for data";
    else if (!live) status = "stale reading";
    else if (p === null) status = "level unknown";
    else {
      const caution = token("--caution", "#A86400", "#F2B33D");
      c = D.low || p <= o.critical ? col.stop : p <= o.low ? caution : D.charging ? col.signal : col.live;
      status = D.full ? "full" : D.charging ? "charging" : fin(D.current) && D.current < -0.02 ? "discharging" : D.input ? "on input" : "idle";
      if (D.low) status += ", LOW";
      if (fin(D.runtime_min) && !D.charging) status += ` · ~${D.runtime_min >= 90 ? (D.runtime_min / 60).toFixed(1) + " h" : Math.round(D.runtime_min) + " min"} left`;
    }
    return {live, p, c, status};
  }

  draw(){
    const cv = this.cv, ctx = this.ctx, o = this.o, dpr = window.devicePixelRatio || 1, W = cv.clientWidth, H = cv.clientHeight;
    if (!W || !H) return;
    if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)){ cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, W, H);
    const now = performance.now();
    if (!this.col || now - this.colAt > 500){ this.col = palette(); this.colAt = now; }
    const col = this.col, D = this.data || {}, {live, p, c, status} = this.assess();
    if (o.compact){ this.drawCompact(W, H, col, D, live, p, c); this.finish(p, status); return; }
    const cells = live && Array.isArray(D.cells) ? D.cells : [];
    const cellH = cells.length ? Math.max(10, H * 0.2) : 0, gap = cells.length ? 6 : 0;
    const bw = Math.min(W * 0.55, (H - cellH - gap - 4) * 2.6), bh = H - cellH - gap - 4, x0 = 2, y0 = 2;
    const nubW = Math.max(4, bw * 0.05), nubH = bh * 0.4, r = Math.min(6, bh * 0.12);
    // body + terminal nub (outline), then the level fill inset
    ctx.lineWidth = 2; ctx.strokeStyle = live ? col.ink : col.muted; ctx.setLineDash(live ? [] : [4, 3]);
    ctx.beginPath(); if (ctx.roundRect) ctx.roundRect(x0, y0, bw - nubW, bh, r); else ctx.rect(x0, y0, bw - nubW, bh); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = live ? col.ink : col.muted; ctx.fillRect(x0 + bw - nubW + 1, y0 + (bh - nubH) / 2, nubW - 1, nubH);
    const ins = 4, fw = (bw - nubW - 2 * ins) * (p === null ? 0 : p / 100);
    if (fw > 0){ ctx.fillStyle = c; ctx.globalAlpha = live ? 0.85 : 0.35; ctx.fillRect(x0 + ins, y0 + ins, fw, bh - 2 * ins); ctx.globalAlpha = 1; }
    // tick marks at 25 / 50 / 75 %
    ctx.strokeStyle = col.rule; ctx.lineWidth = 1;
    for (const f of [0.25, 0.5, 0.75]){ const x = x0 + ins + (bw - nubW - 2 * ins) * f; ctx.beginPath(); ctx.moveTo(x, y0 + bh - ins - 5); ctx.lineTo(x, y0 + bh - ins); ctx.stroke(); }
    if (live && D.charging) this.bolt(x0 + (bw - nubW) / 2, y0 + bh / 2, bh * 0.36, col);
    // readout right of the body
    const tx = x0 + bw + 12, big = Math.max(16, Math.min(34, bh * 0.55)), small = Math.max(11, Math.min(13, bh * 0.22));
    ctx.textAlign = "left"; ctx.textBaseline = "alphabetic";
    ctx.fillStyle = live ? col.fg : col.muted; ctx.font = `600 ${big}px ${token("--font", "system-ui")}`;
    ctx.fillText(p === null ? "–" : Math.round(p) + "%", tx, y0 + bh * 0.55);
    ctx.font = `${small}px ${token("--mono", "ui-monospace,monospace")}`; ctx.fillStyle = col.muted;
    const bits = [fin(D.volts) ? D.volts.toFixed(2) + " V" : null, fin(D.current) ? (D.current > 0 ? "+" : "") + D.current.toFixed(2) + " A" : null].filter(Boolean);
    ctx.fillText(bits.join("  "), tx, y0 + bh * 0.55 + small + 4);
    if (D.input !== undefined && D.input !== null){ const iy = y0 + bh * 0.55 + 2 * small + 8;
      if (D.input && live){ this.plug(tx, iy - small * 0.35, small, col.fg); ctx.fillStyle = col.fg; ctx.fillText("input", tx + small + 4, iy); }
      else { ctx.fillStyle = col.muted; ctx.fillText("no input", tx, iy); } }
    // cells: one bar each, cellMin..cellMax, under the body
    if (cells.length){
      const n = cells.length, cy = y0 + bh + gap, cw = (bw - nubW) / n, lo = Math.min(...cells.filter(fin)), hi = Math.max(...cells.filter(fin));
      ctx.font = `${Math.max(9, Math.min(11, cellH * 0.6))}px ${token("--mono", "ui-monospace,monospace")}`; ctx.textAlign = "center"; ctx.textBaseline = "middle";
      cells.forEach((v, i) => {
        const x = x0 + i * cw + 1, w = cw - 3, f = fin(v) ? Math.max(0, Math.min(1, (v - o.cellMin) / (o.cellMax - o.cellMin))) : 0;
        ctx.strokeStyle = col.rule; ctx.lineWidth = 1; ctx.strokeRect(x, cy, w, cellH);
        ctx.fillStyle = c; ctx.globalAlpha = 0.25; ctx.fillRect(x, cy, w * f, cellH); ctx.globalAlpha = 1;
        const odd = n > 1 && hi - lo > 0.05 && (v === lo || v === hi);       // imbalance: mark the extremes
        ctx.fillStyle = odd ? col.fg : col.muted; ctx.fillText(fin(v) ? v.toFixed(2) : "–", x + w / 2, cy + cellH / 2 + 0.5);
      });
    }
    this.finish(p, status);
  }

  bolt(cx, cy, s, col){                // charging bolt, outlined so it reads on any fill
    const ctx = this.ctx, pts = [[0.15, -1], [-0.45, 0.12], [-0.02, 0.12], [-0.15, 1], [0.45, -0.12], [0.02, -0.12]];
    ctx.beginPath(); pts.forEach(([u, v], i) => { const X = cx + u * s, Y = cy + v * s; i ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y); }); ctx.closePath();
    ctx.fillStyle = col.panel; ctx.fill(); ctx.strokeStyle = col.ink; ctx.lineWidth = 1.5; ctx.stroke();
  }

  plug(x, cy, h, colour){              // input power present: a two-pin plug
    const ctx = this.ctx, w = h * 0.7;
    ctx.strokeStyle = ctx.fillStyle = colour; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(x, cy); ctx.lineTo(x + w * 0.3, cy); ctx.stroke();                       // cord
    ctx.strokeRect(x + w * 0.3, cy - h * 0.3, w * 0.4, h * 0.6);                                         // body
    ctx.beginPath(); ctx.moveTo(x + w * 0.7, cy - h * 0.18); ctx.lineTo(x + w, cy - h * 0.18);           // pins
    ctx.moveTo(x + w * 0.7, cy + h * 0.18); ctx.lineTo(x + w, cy + h * 0.18); ctx.stroke();
  }

  drawCompact(W, H, col, D, live, p, c){
    const ctx = this.ctx, plugW = D.input && live ? H * 0.8 : 0, nubW = 3, x0 = 1, y0 = 2, bh = H - 4;
    const bw = W - plugW - nubW - 3;
    ctx.lineWidth = 1.5; ctx.strokeStyle = live ? col.ink : col.muted; ctx.setLineDash(live ? [] : [3, 2]);
    ctx.beginPath(); if (ctx.roundRect) ctx.roundRect(x0, y0, bw, bh, 3); else ctx.rect(x0, y0, bw, bh); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = live ? col.ink : col.muted; ctx.fillRect(x0 + bw, y0 + bh * 0.3, nubW, bh * 0.4);
    const fw = (bw - 4) * (p === null ? 0 : p / 100);
    if (fw > 0){ ctx.fillStyle = c; ctx.globalAlpha = live ? 0.85 : 0.35; ctx.fillRect(x0 + 2, y0 + 2, fw, bh - 4); ctx.globalAlpha = 1; }
    const txt = p === null ? "–" : Math.round(p) + "%", fs = Math.max(10, Math.min(14, bh * 0.62));
    ctx.font = `600 ${fs}px ${token("--font", "system-ui")}`; ctx.textAlign = "center"; ctx.textBaseline = "middle";
    const tx = x0 + bw / 2 - (live && D.charging ? bh * 0.3 : 0), ty = y0 + bh / 2 + 0.5;
    ctx.lineWidth = 3; ctx.strokeStyle = col.panel; ctx.strokeText(txt, tx, ty);        // halo: readable on any fill
    ctx.fillStyle = live ? col.fg : col.muted; ctx.fillText(txt, tx, ty);
    if (live && D.charging) this.bolt(tx + ctx.measureText(txt).width / 2 + bh * 0.3, ty, bh * 0.3, col);
    if (plugW) this.plug(x0 + bw + nubW + 3, y0 + bh / 2, bh * 0.8, col.ink);
  }

  finish(p, status){
    const o = this.o;
    this.status = status;
    const aria = o.label + ": " + (p === null ? "" : Math.round(p) + "%, ") + status;
    if (this.cv.getAttribute("aria-label") !== aria) this.cv.setAttribute("aria-label", aria);
    if (this.note && this.note.textContent !== status) this.note.textContent = status;
  }

  static fromSource(s){
    s = s || {};
    return {percent: s.percent, volts: s.pack_v, current: s.current_a, cells: s.cells_v,
            charging: s.charge_state === "charging", full: s.charge_state === "full", input: s.input_present,
            runtime_min: s.runtime_min, stale: s.ok === false || !!s.stale, low: !!s.low};
  }
}
Battery.defaults = BATTERY_DEFAULTS;
viz.Battery = Battery;
})();
"""
