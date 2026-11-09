"""Visual safety checks for pages under a theme: what a stylesheet must never break.

The page contracts (tests/test_pages_contract.py) prove the e-stop is *wired*; these prove it can be
*seen*. They run in a real browser (Playwright) against a local simulated stack
(``robot_utils.dev.preview``), once per page x theme x colour scheme x viewport width:

- the e-stop button (where the page has one) is visible without scrolling, not covered, not
  translucent, at least 44 x 44 px, filled solidly, with text contrast >= 4.5:1;
- the state band is at the top and visible, and each of its states (e-stop, chocked, covered,
  live) renders with readable text (>= 4.5:1 against its fill);
- the header row (``.ru-head``) is visible;
- body text and muted text in panels are >= 4.5:1 against what is behind them (nearest solid
  background colour; background images such as grids are not sampled, so the theme author still
  checks those by eye);
- the theme actually applied (``data-theme`` on the root).

``CHECK_JS`` is evaluated in the page and returns a list of problems (empty = pass).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


@dataclass(frozen=True)
class Page:
    name: str
    path: str
    estop: Optional[str] = None       # selector of the e-stop button, if the page has one
    band: bool = True                 # has the shared state band (#band, RU.stateBand)
    head: bool = True                 # has the header row (.ru-head)
    wait_for: str = "body"


PAGES = [
    Page("landing", "/", estop="#estop", band=False),
    Page("swerve", "/nodes/swerve/", estop="#estop button.ru-estop"),
    Page("pose_servo", "/nodes/pose_servo/", estop="#estop button.ru-estop"),
    Page("description", "/nodes/description/", estop="#estop button.ru-estop"),
    Page("agent_nav", "/nodes/agent_nav/", estop="#estop button.ru-estop"),
    Page("estimator", "/nodes/estimator/"),
    Page("camera", "/nodes/camera/"),
    Page("3d_perception", "/nodes/3d_perception/"),
    Page("swerve_teleop", "/tools/swerve_teleop.html", estop="#estop", band=False, head=False),
    Page("remote_teleop", "/tools/remote_teleop.html", estop="#estop", band=False, head=False),
]

CHECK_JS = r"""
async (args) => {
  const problems = [];
  const bad = (what) => problems.push(what);
  const parse = (c) => { const m = c.match(/rgba?\(([^)]+)\)/); if (!m) return null;
    const p = m[1].split(/[ ,\/]+/).filter(Boolean).map(Number); return {r: p[0], g: p[1], b: p[2], a: p.length > 3 ? p[3] : 1}; };
  const lum = (c) => { const f = v => { v /= 255; return v <= 0.03928 ? v / 12.92 : Math.pow((v + 0.055) / 1.055, 2.4); };
    return 0.2126 * f(c.r) + 0.7152 * f(c.g) + 0.0722 * f(c.b); };
  const ratio = (a, b) => { const x = lum(a), y = lum(b); return (Math.max(x, y) + 0.05) / (Math.min(x, y) + 0.05); };
  const blend = (top, under) => ({r: top.r * top.a + under.r * (1 - top.a), g: top.g * top.a + under.g * (1 - top.a),
                                  b: top.b * top.a + under.b * (1 - top.a), a: 1});
  // the colour behind an element: its own and its ancestors' background colours, composited
  const behind = (el) => { const stack = []; for (let e = el; e; e = e.parentElement){
      const c = parse(getComputedStyle(e).backgroundColor); if (c && c.a > 0){ stack.push(c); if (c.a >= 1) break; } }
    let out = {r: 255, g: 255, b: 255, a: 1};
    if (getComputedStyle(document.documentElement).colorScheme.includes("dark") && !stack.some(c => c.a >= 1)) out = {r: 0, g: 0, b: 0, a: 1};
    for (let i = stack.length - 1; i >= 0; i--) out = blend(stack[i], out);
    return out; };
  // a gradient (hatched bands, glass sheens) behind the text: every colour stop counts, worst case wins
  // Only layers that cover the box count (hatches, sheens); small tiled decorations such as corner
  // ticks or a dot grid (a background-size, or a radial gradient) are lines, not what text sits on.
  const layers = (v) => { const out = []; let d = 0, cur = "";
    for (const ch of v){ if (ch === "(") d++; if (ch === ")") d--; if (ch === "," && !d){ out.push(cur.trim()); cur = ""; } else cur += ch; }
    if (cur.trim()) out.push(cur.trim()); return out; };
  const stops = (el) => { for (let e = el; e; e = e.parentElement){
      const st = getComputedStyle(e), img = st.backgroundImage;
      if (img && img !== "none"){
        const imgs = layers(img), sizes = layers(st.backgroundSize);
        const cs = [];
        imgs.forEach((g, i) => { const size = sizes[i % sizes.length] || "auto";
          const covers = /^(auto|cover|100%( 100%)?|auto auto)$/.test(size);
          if (!covers || !/^(repeating-)?linear-gradient/.test(g)) return;
          for (const m of g.match(/rgba?\([^)]+\)/g) || []){ const c = parse(m); if (c && c.a > 0) cs.push(c); } });
        if (cs.length) return cs.map(c => blend(c, behind(e))); }
      const c = parse(st.backgroundColor); if (c && c.a >= 1) return []; }
    return []; };
  const textContrast = (el) => { const fg = parse(getComputedStyle(el).color); if (!fg) return 21;
    const st = stops(el), bgs = st.length ? st : [behind(el)];   // a covering gradient is what the text sits on
    return Math.min(...bgs.map(bg => ratio(blend(fg, bg), bg))); };
  const hiddenBy = (el) => { for (let e = el; e && e !== document.documentElement; e = e.parentElement){
      const s = getComputedStyle(e);
      if (s.display === "none") return "display:none on " + (e.id ? "#" + e.id : e.tagName.toLowerCase());
      if (s.visibility === "hidden" || s.visibility === "collapse") return "visibility:hidden";
      if (parseFloat(s.opacity) < 0.95) return "opacity " + s.opacity + " on " + (e.id ? "#" + e.id : e.tagName.toLowerCase()); }
    return null; };
  const W = innerWidth, H = innerHeight;
  window.scrollTo(0, 0);

  if (args.theme && args.theme !== "builtin" && document.documentElement.getAttribute("data-theme") !== args.theme)
    bad(`theme ${args.theme} did not apply (data-theme=${document.documentElement.getAttribute("data-theme")})`);

  if (args.estop){
    const el = document.querySelector(args.estop);
    if (!el) bad(`e-stop ${args.estop} not found`);
    else {
      const r = el.getBoundingClientRect(), h = hiddenBy(el);
      if (h) bad(`e-stop hidden: ${h}`);
      if (r.width < 44 || r.height < 44) bad(`e-stop too small: ${r.width.toFixed(0)}x${r.height.toFixed(0)} px (min 44x44)`);
      if (r.top < 0 || r.left < 0 || r.bottom > H || r.right > W) bad(`e-stop not fully on screen without scrolling (${r.left.toFixed(0)},${r.top.toFixed(0)} ${r.width.toFixed(0)}x${r.height.toFixed(0)} in ${W}x${H})`);
      else { const hit = document.elementFromPoint(r.left + r.width / 2, r.top + r.height / 2);
        if (!hit || !(hit === el || el.contains(hit))) bad(`e-stop covered by ${hit ? (hit.id ? "#" + hit.id : hit.tagName.toLowerCase() + "." + hit.className) : "nothing"}`); }
      const fill = parse(getComputedStyle(el).backgroundColor);
      if (!fill || fill.a < 1) bad(`e-stop fill not solid (${getComputedStyle(el).backgroundColor})`);
      const s = getComputedStyle(el);
      if (s.backdropFilter && s.backdropFilter !== "none") bad("e-stop uses backdrop-filter");
      const c = textContrast(el); if (c < 4.5) bad(`e-stop text contrast ${c.toFixed(2)} (min 4.5)`);
      if (fill && fill.a >= 1 && !(fill.r > fill.g * 1.6 && fill.r > fill.b * 1.6 && fill.r > 120)) bad(`e-stop fill is not red-family (${s.backgroundColor})`);
    }
  }

  const head = args.head ? document.querySelector(".ru-head") : null;
  if (args.head && !head) bad("no .ru-head");
  else if (head) { const h = hiddenBy(head), r = head.getBoundingClientRect();
    if (h) bad(`header hidden: ${h}`); if (r.height < 16) bad(`header too short (${r.height.toFixed(0)} px)`);
    if (r.top > H * 0.5) bad("header not near the top"); }

  if (args.band){
    const band = document.querySelector("#band");
    if (!band || !band.classList.contains("ru-band")) bad("no state band (#band.ru-band)");
    else { const r = band.getBoundingClientRect(), h = hiddenBy(band);
      if (h) bad(`band hidden: ${h}`); if (r.top > 8) bad(`band not at the top (top ${r.top.toFixed(0)} px)`);
      if (r.height < 3) bad(`band thinner than 3 px (${r.height.toFixed(1)})`); }
    if (window.RU && RU.stateBand){
      const probe = document.createElement("div"); document.body.prepend(probe);
      const b = RU.stateBand(probe);
      const states = {estop: [{estop: {latched: true, source: "probe"}}],
                      chocked: [{locks: [{kind: "chock", by: "probe", reason: "probe", age_s: 1}]}],
                      covered: [{locks: [{kind: "cover", by: "probe", reason: "probe", age_s: 1}]}],
                      live: [{}, {live: true, liveText: "probe"}]};
      for (const [want, a] of Object.entries(states)){
        const got = b.update(a[0], a[1]);
        if (got !== want){ bad(`band state ${want}: got ${got}`); continue; }
        const r = probe.getBoundingClientRect(), h = hiddenBy(probe);
        if (h) bad(`band ${want} hidden: ${h}`);
        if (r.height < 16) bad(`band ${want} too thin (${r.height.toFixed(0)} px)`);
        const t = probe.querySelector("b");
        if (t){ const c = textContrast(t); if (c < 4.5) bad(`band ${want} text contrast ${c.toFixed(2)} (min 4.5)`); }
      }
      probe.remove();
    }
  }

  const sample = (sel, n, min, what) => { let k = 0;
    for (const el of document.querySelectorAll(sel)){
      if (k >= n) break;
      if (!el.textContent.trim() || hiddenBy(el) || el.closest("#band")) continue;
      k++; const c = textContrast(el);
      if (c < min){ bad(`${what} contrast ${c.toFixed(2)} (min ${min}): "${el.textContent.trim().slice(0, 40)}"`); break; } } };
  sample(".ru-card b, .ru-card td, .ru-card th, .tile .title, section.box h2, .box .k, h1, h2", 12, 4.5, "text");
  sample(".ru-muted, .tile .kind, .small", 8, 4.5, "muted text");
  return problems;
}
"""


@dataclass
class Result:
    page: str
    theme: str
    scheme: str
    width: int
    problems: list = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.problems

    def label(self) -> str:
        return f"{self.page:14} {self.theme:14} {self.scheme:5} {self.width:5}"


def check_page(pw_page, page: Page, theme: str) -> list:
    return pw_page.evaluate(CHECK_JS, {"estop": page.estop, "band": page.band, "head": page.head, "theme": theme})
