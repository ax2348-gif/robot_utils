"""Page themes (frontend/themes.py): registry, robot default, routes, the loader, every page wired."""
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from robot_utils.frontend.themes import LOADER_JS, Themes

ROOT = Path(__file__).resolve().parents[1] / "src" / "robot_utils"
PACKAGED = ROOT / "frontend" / "themes"
NODE = shutil.which("node") or next((str(p) for p in [Path(sys.executable).parent / "node"] if p.exists()), None)


@pytest.fixture
def themes(tmp_path):
    return Themes(user_dir=tmp_path / "themes", settings=tmp_path / "frontend.json", packaged=PACKAGED)


def test_registry_default_and_admin_overrides(themes, tmp_path):
    packaged = {t["name"]: t for t in themes.list()}
    assert "contrast" in packaged and themes.default() is None
    assert packaged["contrast"]["title"] == "High contrast" and packaged["contrast"]["source"] == "repo"
    assert all(t["source"] == "repo" and t["title"] != t["name"] and t["description"] for t in packaged.values())
    themes.set_default("contrast")
    assert themes.default() == "contrast" and json.loads((tmp_path / "frontend.json").read_text())["default_theme"] == "contrast"
    with pytest.raises(ValueError):
        themes.set_default("nope")
    themes.set_default(None)
    assert themes.default() is None
    # an admin-added theme on the robot, and one overriding a packaged theme
    (tmp_path / "themes").mkdir()
    (tmp_path / "themes" / "night.css").write_text("/*\ntitle: Night\n*/\n")
    (tmp_path / "themes" / "contrast.css").write_text("/*\ntitle: Local contrast\n*/\n")
    names = {t["name"]: t for t in themes.list()}
    assert names["night"]["source"] == "robot" and names["contrast"]["title"] == "Local contrast"
    for bad in ("../frontend", "CON", "a/b", "", "x" * 40):
        assert themes.css(bad) is None
    # a default pointing at a theme that disappeared falls back to the built-in look
    themes.set_default("night")
    (tmp_path / "themes" / "night.css").unlink()
    assert themes.default() is None


def test_routes(tmp_path):
    from fastapi.testclient import TestClient

    from robot_utils.frontend.app import create_app
    t = Themes(user_dir=tmp_path / "t", settings=tmp_path / "f.json", packaged=PACKAGED)
    t.set_default("contrast")
    with TestClient(create_app([], themes=t)) as c:
        idx = c.get("/themes/").json()
        assert idx["default"] == "contrast" and "contrast" in [x["name"] for x in idx["themes"]]
        js = c.get("/themes/theme.js")
        assert js.status_code == 200 and "text/javascript" in js.headers["content-type"] and "RUTheme" in js.text
        css = c.get("/themes/contrast.css")
        assert css.status_code == 200 and css.headers["content-type"].startswith("text/css")
        assert c.get("/themes/nope.css").status_code == 404
        assert c.get("/themes/..%2Fpage.css").status_code == 404


PAGE_FILES = ["nodes/swerve/page.py", "nodes/pose_servo/page.py", "nodes/agent_nav/page.py", "nodes/description/page.py",
              "nodes/estimator/page.py", "nodes/perception3d/page.py", "nodes/orbbec/page.py",
              "frontend/page.html", "frontend/tools/swerve_teleop.html", "frontend/tools/remote_teleop.html"]


@pytest.mark.parametrize("f", PAGE_FILES)
def test_every_page_loads_the_theme_loader_in_its_head(f):
    s = (ROOT / f).read_text()
    head = s[: s.index("</head>")]
    want = {"frontend/page.html": "themes/theme.js"}.get(f, "../themes/theme.js" if "tools/" in f else "../../themes/theme.js")
    assert f'<script src="{want}"></script>' in head      # relative: works behind any prefix


@pytest.mark.parametrize("css", sorted(PACKAGED.glob("*.css")), ids=lambda p: p.name)
def test_packaged_themes_scope_every_rule(css):
    """Every rule is scoped to :root[data-theme="<name>"], so a theme never leaks into another."""
    text = re.sub(r"/\*.*?\*/", "", css.read_text(), flags=re.S)
    selectors = [s.strip() for s in re.findall(r"([^{}]+)\{", text) if not s.strip().startswith("@")]
    want = f':root[data-theme="{css.stem}"]'
    for sel in selectors:
        for part in sel.split(","):
            assert part.strip().startswith(want), f"{css.name}: unscoped selector {part.strip()!r}"


def test_admin_cli(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr("robot_utils.admin.theme.Themes",
                        lambda: Themes(user_dir=tmp_path / "t", settings=tmp_path / "f.json", packaged=PACKAGED))
    from robot_utils.admin import theme as cli
    assert cli.main(["list"]) == 0 and "built-in look" in capsys.readouterr().out
    assert cli.main(["default", "contrast"]) == 0
    assert json.loads((tmp_path / "f.json").read_text())["default_theme"] == "contrast"
    assert cli.main(["default", "nope"]) == 2
    assert cli.main(["default", "--clear"]) == 0
    assert "default_theme" not in json.loads((tmp_path / "f.json").read_text())


HARNESS = r"""
const store = {}; let fetched = null; const listeners = {};
const root = {attrs: {}, setAttribute(k, v){ this.attrs[k] = v; }, removeAttribute(k){ delete this.attrs[k]; }};
const head = {kids: [], appendChild(n){ this.kids = this.kids.filter(k => k !== n); this.kids.push(n); n.parentNode = this; },
              removeChild(n){ this.kids = this.kids.filter(k => k !== n); }};
global.localStorage = {getItem: k => (k in store ? store[k] : null), setItem: (k, v) => { store[k] = String(v); },
                       removeItem: k => { delete store[k]; }};
global.document = {documentElement: root, head,
  getElementById: id => head.kids.find(k => k.id === id) || null,
  createElement: () => ({}), addEventListener: (t, f) => { listeners[t] = f; }};
global.window = global;
const INDEX = JSON.parse(process.argv[2]); const PRESET = JSON.parse(process.argv[3]);
Object.assign(store, PRESET);
global.fetch = () => Promise.resolve({ok: INDEX !== null, json: () => Promise.resolve(INDEX)});
require(process.argv[4]);
setTimeout(() => {
  const link = head.kids.find(k => k.id === "ru-theme-css");
  const out = {theme: root.attrs["data-theme"] || null, href: link ? link.href : null, cached: store["ru-theme-applied"] || null};
  RUTheme.set("builtin");
  out.after_builtin = root.attrs["data-theme"] || null; out.links_after = head.kids.length;
  console.log(JSON.stringify(out));
}, 50);
"""


@pytest.mark.skipif(NODE is None, reason="Node.js not installed")
@pytest.mark.parametrize("index,preset,theme", [
    ({"default": "contrast", "themes": [{"name": "contrast"}]}, {}, "contrast"),            # robot default applies
    ({"default": "contrast", "themes": [{"name": "contrast"}]}, {"ru-theme": "builtin"}, None),   # viewer opts out
    ({"default": None, "themes": [{"name": "contrast"}]}, {"ru-theme": "contrast"}, "contrast"),  # viewer opts in
    ({"default": "gone", "themes": []}, {"ru-theme": "gone"}, None),                        # missing -> built-in
    (None, {}, None),                                                                        # loader can't reach /themes/
])
def test_loader_behaviour(tmp_path, index, preset, theme):
    loader = tmp_path / "loader.js"
    loader.write_text(LOADER_JS)
    harness = tmp_path / "h.js"
    harness.write_text(HARNESS)
    r = subprocess.run([NODE, str(harness), json.dumps(index), json.dumps(preset), str(loader)],
                       capture_output=True, text=True, timeout=20)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout)
    assert out["theme"] == theme
    assert (out["href"] == f"/themes/{theme}.css") if theme else out["href"] is None
    assert out["after_builtin"] is None and out["links_after"] == 0      # the viewer can always go back
