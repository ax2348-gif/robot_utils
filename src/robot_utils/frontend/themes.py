"""Page themes: CSS-only restyling of every page, with a robot-wide default and a per-viewer choice.

- A theme is a CSS file named ``<name>.css``, either in the package (``frontend/themes/``) or added
  by an admin on the robot (``~/.config/robot_utils/themes/``; a file there overrides a packaged one
  of the same name). Its first comment may give ``title:`` and ``description:`` lines. Rules should
  be scoped to ``:root[data-theme="<name>"]`` so they win over each page's own variables.
- The built-in look (no theme) is the default and the fallback: if no theme is chosen, the theme
  file is missing, or the loader can't be reached (a node opened on its debug port), pages look
  as they always did.
- The robot-wide default is set by an admin over SSH (``python -m robot_utils.admin.theme``) and
  stored in ``~/.config/robot_utils/frontend.json``. Each viewer may choose another theme, or the
  built-in look, on the landing page; that choice lives in their browser (all pages share the
  origin, so it applies everywhere) and affects no one else.
- Themes change only how pages look. Markup, ids and handlers are identical under every theme,
  so the page contracts (tests/test_pages_contract.py) hold for all of them.

Every page includes the loader in its head by a relative path (``../../themes/theme.js`` from a node
page, ``themes/theme.js`` from the landing page), so on a node's own debug port it simply isn't
found and the page keeps its built-in look.
"""
from __future__ import annotations

import json
import os
import re
from importlib import resources
from pathlib import Path
from typing import Optional

NAME = re.compile(r"^[a-z0-9][a-z0-9_-]{0,31}$")
CONFIG_DIR = Path(os.environ.get("XDG_CONFIG_HOME", os.path.expanduser("~/.config"))) / "robot_utils"
SETTINGS_FILE = CONFIG_DIR / "frontend.json"
USER_THEMES = CONFIG_DIR / "themes"
MAX_CSS = 512 * 1024


def _packaged_dir() -> Optional[Path]:
    try:
        p = Path(str(resources.files("robot_utils.frontend").joinpath("themes")))
        return p if p.is_dir() else None
    except Exception:
        return None


def _meta(css: str) -> dict:
    m = re.match(r"\s*/\*(.*?)\*/", css, re.S)
    out = {}
    if m:
        for line in m.group(1).splitlines():
            k, _, v = line.strip(" *\t").partition(":")
            if k.strip().lower() in ("title", "description") and v.strip():
                out[k.strip().lower()] = v.strip()
    return out


class Themes:
    def __init__(self, user_dir: Path = USER_THEMES, settings: Path = SETTINGS_FILE,
                 packaged: Optional[Path] = None):
        self.user_dir, self.settings = user_dir, settings
        self.packaged = packaged if packaged is not None else _packaged_dir()

    def files(self) -> dict[str, Path]:
        out = {}
        for d in (self.packaged, self.user_dir):          # user themes override packaged ones
            if d and d.is_dir():
                for f in sorted(d.glob("*.css")):
                    if NAME.match(f.stem) and f.stat().st_size <= MAX_CSS:
                        out[f.stem] = f
        return out

    def list(self) -> list[dict]:
        out = []
        for name, f in self.files().items():
            meta = _meta(f.read_text(errors="replace")[:4000])
            out.append({"name": name, "title": meta.get("title", name), "description": meta.get("description", ""),
                        "source": "robot" if f.parent == self.user_dir else "repo"})
        return out

    def css(self, name: str) -> Optional[str]:
        if not NAME.match(name or ""):
            return None
        f = self.files().get(name)
        return f.read_text(errors="replace") if f else None

    def default(self) -> Optional[str]:
        try:
            d = json.loads(self.settings.read_text())
        except (OSError, ValueError):
            return None
        name = d.get("default_theme")
        return name if isinstance(name, str) and name in self.files() else None

    def set_default(self, name: Optional[str]) -> Optional[str]:
        if name is not None and name not in self.files():
            raise ValueError(f"no theme {name!r}; available: {sorted(self.files())}")
        try:
            d = json.loads(self.settings.read_text())
            if not isinstance(d, dict):
                d = {}
        except (OSError, ValueError):
            d = {}
        if name is None:
            d.pop("default_theme", None)
        else:
            d["default_theme"] = name
        self.settings.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.settings.with_suffix(".tmp")
        tmp.write_text(json.dumps(d, indent=1) + "\n")
        os.replace(tmp, self.settings)
        return name

    def index(self) -> dict:
        return {"default": self.default(), "themes": self.list(),
                "builtin": "the original look (always available; used when no theme applies)"}


# The loader every page includes in its <head>. It applies the cached choice at once (no flash on
# later visits), then reconciles with the robot default; it never throws, and without it pages keep
# their built-in look. The viewer's choice: localStorage "ru-theme" = absent (follow the robot
# default) | "builtin" | a theme name.
LOADER_JS = r"""(function(){
"use strict";
var KEY = "ru-theme", CACHE = "ru-theme-applied";
// where the themes are: next to this script (pages load it by a relative path, so this works behind
// any prefix); /themes/ if the script's own URL is unknown
var BASE = "/themes/";
try { if (document.currentScript && document.currentScript.src) BASE = new URL(".", document.currentScript.src).href; } catch(e){}
function get(k){ try { return localStorage.getItem(k); } catch(e){ return null; } }
function put(k, v){ try { if (v === null) localStorage.removeItem(k); else localStorage.setItem(k, v); } catch(e){} }
function apply(name){
  var root = document.documentElement, link = document.getElementById("ru-theme-css");
  if (!name || name === "builtin"){ if (link) link.parentNode.removeChild(link); root.removeAttribute("data-theme"); return; }
  if (!link){ link = document.createElement("link"); link.rel = "stylesheet"; link.id = "ru-theme-css"; }
  link.href = BASE + encodeURIComponent(name) + ".css";
  (document.head || root).appendChild(link);           // after the page's own styles
  root.setAttribute("data-theme", name);
}
var state = {choice: get(KEY), index: null};
function resolve(){
  var d = state.index, c = state.choice;
  if (!d) return get(CACHE);
  var names = {}; (d.themes || []).forEach(function(t){ names[t.name] = 1; });
  if (c === "builtin") return "builtin";
  if (c && names[c]) return c;
  return d.default && names[d.default] ? d.default : "builtin";
}
function refresh(){ var n = resolve(); if (state.index) put(CACHE, n); apply(n); }
apply(get(CACHE));
document.addEventListener("DOMContentLoaded", function(){ var n = get(CACHE); if (n) apply(n); });
fetch(BASE, {cache: "no-store"}).then(function(r){ return r.ok ? r.json() : null; })
  .then(function(d){ if (d){ state.index = d; refresh(); } }).catch(function(){});
window.RUTheme = {
  choice: function(){ return state.choice; },           // null = follow the robot default
  applied: function(){ return resolve() || "builtin"; },
  index: function(){ return state.index; },
  set: function(v){ state.choice = v || null; put(KEY, state.choice); refresh(); }
};
})();
"""
