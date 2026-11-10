"""Layout contract (static) and visual safety under every packaged theme (browser; dev/visual_safety.py).

The static layout-contract tests always run. The browser tests (every page under every packaged
theme, light and dark, two widths: several minutes) are opt-in: run them after a theme or shared
style change with ``RU_VISUAL=1``. They start the whole robot in simulation in a temp folder
(dev/preview.py), need Playwright with Chromium (``pip install playwright && python -m playwright
install chromium``) and never touch the real robot.
"""
import os
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1] / "src" / "robot_utils"
ROLES = {"view", "control", "telemetry", "config", "log"}


def page_html(f: str) -> str:
    if f.endswith(".py"):
        import importlib
        mod = "robot_utils." + f[:-3].replace("/", ".")
        return importlib.import_module(mod).PAGE
    return (ROOT / f).read_text()


LAYOUT_PAGES = {"nodes/swerve/page.py": "swerve", "nodes/pose_servo/page.py": "pose_servo",
                "nodes/agent_nav/page.py": "agent_nav", "nodes/description/page.py": "description",
                "nodes/estimator/page.py": "estimator", "nodes/perception3d/page.py": "perception3d",
                "nodes/orbbec/page.py": "camera", "nodes/power/page.py": "power", "nodes/arm/page.py": "arm",
                "frontend/page.html": "landing"}


@pytest.mark.parametrize("f,kind", LAYOUT_PAGES.items())
def test_layout_contract(f, kind):
    """The hooks layouts and themes may target (docs/frontend.md "Layouts") exist on every page."""
    s = page_html(f)
    assert f'<body data-page="{kind}">' in s
    assert len(re.findall(r'class="[^"]*\bru-head\b', s)) == 1
    assert len(re.findall(r'class="[^"]*\bru-body\b', s)) == 1
    panels = re.findall(r'data-panel="([^"]+)" data-role="([^"]+)"', s)
    assert panels, "no data-panel"
    names = [p for p, _ in panels]
    assert len(names) == len(set(names)), f"duplicate data-panel names: {names}"
    assert {r for _, r in panels} <= ROLES
    assert len(re.findall(r"data-panel=", s)) == len(panels), "data-panel without data-role right after it"
    if kind != "landing":
        assert '<div id="band"></div>' in s


@pytest.mark.parametrize("tool", ["swerve_teleop", "remote_teleop"])
def test_tools_name_their_page(tool):
    """The teleop tools share element ids (#estop, #state, #take ...); themes tell them apart by page."""
    assert f'<body data-page="{tool}"' in (ROOT / "frontend" / "tools" / f"{tool}.html").read_text()


def test_estop_lives_in_the_header():
    """Every node page with an e-stop puts it in the header row, so it is above the fold on a phone."""
    for f in ["nodes/swerve/page.py", "nodes/pose_servo/page.py", "nodes/agent_nav/page.py", "nodes/description/page.py",
              "nodes/arm/page.py"]:
        s = page_html(f)
        head = s[s.index("ru-head"):]
        head = head[: head.index('<div id="err"')]
        assert 'id="estop"' in head, f


# -- browser ------------------------------------------------------------------------------------

def _chromium_ok() -> bool:
    try:
        from playwright.sync_api import sync_playwright
        with sync_playwright() as pw:
            pw.chromium.launch().close()
        return True
    except Exception:
        return False


browser = pytest.mark.skipif(not os.environ.get("RU_VISUAL") or not _chromium_ok(),
                             reason="browser visual checks are opt-in (RU_VISUAL=1) and need Playwright + Chromium")

BAD_THEME = """/*
title: Bad (test only)
*/
:root[data-theme="bad"] { --stop-fill: #888888; --on-stop: #999999; --muted: #f0f0f0; }
:root[data-theme="bad"] button.ru-estop { opacity: .5; }
:root[data-theme="bad"] .ru-band { display: none; }
:root[data-theme="bad"] #estop { transform: translateY(2000px); }
"""


@pytest.fixture(scope="module")
def sim(tmp_path_factory):
    from robot_utils.dev.preview import stack
    themes = tmp_path_factory.mktemp("themes")
    (themes / "bad.css").write_text(BAD_THEME)
    with stack(None, themes) as (base, _root):
        yield base


@browser
def test_packaged_themes_pass_visual_safety(sim):
    from robot_utils.dev.preview import run_matrix, theme_names
    themes = [t for t in theme_names(sim) if t != "bad"]
    assert "builtin" in themes and "contrast" in themes
    results = run_matrix(sim, themes, [1280, 390], ["light", "dark"], settle_s=1.0)
    failed = [f"{r.label()}: {'; '.join(r.problems)}" for r in results if not r.ok]
    assert not failed, "\n".join(failed)


@browser
def test_checks_catch_a_bad_theme(sim):
    from robot_utils.dev.preview import run_matrix
    from robot_utils.dev.visual_safety import PAGES
    pages = [p for p in PAGES if p.name in ("swerve", "landing")]
    results = {r.page: " ".join(r.problems) for r in run_matrix(sim, ["bad"], [1280], ["light"], pages, settle_s=1.0)}
    assert "e-stop hidden: opacity" in results["swerve"]
    assert "band hidden: display:none" in results["swerve"]
    assert "e-stop text contrast" in results["swerve"] and "not red-family" in results["swerve"]
    assert "muted text contrast" in results["swerve"]
    assert "not fully on screen" in results["landing"]


def test_preview_refuses_remote_hosts():
    from robot_utils.dev.preview import run_matrix
    with pytest.raises(SystemExit, match="local simulated stack"):
        run_matrix("http://botcore-desktop", ["builtin"], [1280], ["light"])
