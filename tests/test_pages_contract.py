"""Page contracts: behaviour in the browser that must survive any restyling.

Pages are HTML / JavaScript strings, isolated from the nodes' logic, and the robot's own guards
(limits, the 0.25 s command timeout, leases, e-stop latches, chocks) are all server-side. But some
page code still matters for safety: e-stop buttons, teleop stop-on-release / blur / hide handling,
and the lease widget. These tests pin that behaviour, and check that every page's JavaScript parses
(``node --check``; skipped when Node.js is not installed). docs/frontend.md, "Editing pages".
"""
import re
import shutil
import subprocess
import sys
from importlib import import_module
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1] / "src" / "robot_utils"


def node_page(module: str) -> str:
    return import_module(f"robot_utils.nodes.{module}.page").PAGE


PAGES = {
    "landing": lambda: (ROOT / "frontend" / "page.html").read_text(),
    "swerve_teleop_tool": lambda: (ROOT / "frontend" / "tools" / "swerve_teleop.html").read_text(),
    "remote_teleop_tool": lambda: (ROOT / "frontend" / "tools" / "remote_teleop.html").read_text(),
    "swerve": lambda: node_page("swerve"),
    "pose_servo": lambda: node_page("pose_servo"),
    "agent_nav": lambda: node_page("agent_nav"),
    "agent": lambda: node_page("agent"),
    "description": lambda: node_page("description"),
    "camera": lambda: node_page("orbbec"),
    "estimator": lambda: node_page("estimator"),
    "3d_perception": lambda: node_page("perception3d"),
    "power": lambda: node_page("power"),
    "arm": lambda: node_page("arm"),
}


def ui_js() -> str:
    return import_module("robot_utils.nodes.common.ui_assets").UI_JS


def scripts(html: str) -> list[str]:
    """Inline <script> bodies (not <script src=...>)."""
    return re.findall(r"<script(?![^>]*\bsrc=)[^>]*>(.*?)</script>", html, re.S)


def has(pattern: str, text: str) -> bool:
    return re.search(pattern, text, re.S) is not None


# -- every page that can move the robot has a working e-stop ----------------------------------------

@pytest.mark.parametrize("page", ["swerve", "pose_servo", "description", "arm"])
def test_node_pages_with_actuation_use_the_shared_estop_button(page):
    html = PAGES[page]()
    assert has(r'id="estop"', html), "the e-stop element is gone"
    assert has(r'RU\.estopButton\(\$\("#estop"\)\)', html), "the shared e-stop button is no longer wired"
    assert 'src="common/ui.js"' in html


def test_agent_nav_estop_button_posts_estop():
    html = PAGES["agent_nav"]()
    assert has(r'id="estop"', html)
    # the shared e-stop button, sent as the human operator (X-Operator: human)
    assert has(r'RU\.estopButton\(\$\("#estop"\), \{headers: H\b', html)
    assert has(r'const H = \{"X-Operator": "human"\}', html)


@pytest.mark.parametrize("page", ["swerve_teleop_tool", "remote_teleop_tool"])
def test_teleop_tools_estop_works_without_the_lease(page):
    html = PAGES[page]()
    assert has(r'id="estop"', html)
    assert has(r'\$\("estop"\)\.onclick\s*=\s*async\s*\(\)\s*=>\s*\{.*?api\("POST",\s*"estop"', html)


def test_landing_page_estop_all_and_lock_buttons():
    html = PAGES["landing"]()
    assert has(r'<button id="estop"[^>]*>\s*E-STOP ALL', html)
    assert has(r'getElementById\("estop"\)\.addEventListener\("click",\s*estopAll\)', html)
    assert has(r"async function estopAll\(\).*?Promise\.all\(targets\.map\(estopOne\)\)", html)
    assert has(r"async function estopOne\(n\).*?\{ method: \"POST\" \}", html)
    assert has(r'getElementById\("chock-all"\)\.onclick\s*=\s*\(\)\s*=>\s*placeLock\("chock",\s*"all"\)', html)
    assert has(r'getElementById\("cover-cam"\)\.onclick\s*=\s*\(\)\s*=>\s*placeLock\("cover",\s*"camera"\)', html)


def test_shared_toolkit_estop_and_lease_renewal():
    js = ui_js()
    # POSTs "estop" by default (opts.path / opts.headers let a page target its own e-stop, e.g. agent_nav as the human)
    assert has(r'RU\.estopButton = function\(el, opts\)\{.*?b\.onclick = async \(\) => \{ try \{ await RU\.api\(opts\.path \|\| "estop", \{method: "POST"', js)
    assert has(r'setInterval\(async \(\) => \{.*?RU\.api\("lease/renew"', js), "lease auto-renewal is gone"


# -- teleop stops when input stops ------------------------------------------------------------------

def test_swerve_page_keyboard_teleop_stops_on_release_and_blur():
    html = PAGES["swerve"]()
    # commands are sent only while a key is held (the server's 0.25 s timeout then stops the base)
    assert has(r'setInterval\(\(\) => \{ if \(held\.size && lease\.token', html)
    assert has(r'addEventListener\("keyup", e => press\(e\.key\.toLowerCase\(\), false\)\)', html)
    assert has(r'addEventListener\("blur", \(\) => \{ held\.clear\(\)', html)
    assert has(r"onpointerup = b\.onpointerleave = \(\) => press\(k, false\)", html)


def test_arm_page_jog_stops_on_release_blur_and_hide():
    html = PAGES["arm"]()
    # each hold-to-move control springs back to zero on release, cancel or lost capture
    assert has(r'for \(const type of \["pointerup", "pointercancel", "lostpointercapture"\]\) svg\.addEventListener\(type, release\)', html)
    assert has(r"function release\(\)\{ id = null; svg\.classList\.remove\(\"on\"\); set\(0, 0\); \}", html), "releasing must zero the control"
    assert has(r'svg\.addEventListener\("blur", \(\) => \{ keys\.clear\(\); release\(\); \}\)', html)
    assert has(r'window\.addEventListener\("blur", releaseAll\)', html)
    assert has(r'addEventListener\("visibilitychange", \(\) => \{ if \(document\.hidden\) releaseAll\(\); \}\)', html)
    # track posts only while a control is held (the node's twist then times out into a hold)
    assert has(r"setInterval\(async \(\) => \{\s*if \(!held\(\)\)\{ jp = null; return; \}", html)
    assert has(r'RU\.api\("track", \{body: \{pose:', html)
    # each hold starts from the measured pose, and a stale target is dropped when the arm stops servoing,
    # the page can't command, or the node refuses a target
    assert has(r'async function jogBase\(\)\{.*?RU\.api\("kinematics"\);', html)
    assert "jogBase(){\n  // each hold starts from the MEASURED" in html and 'jogBase' in html
    assert not has(r'async function jogBase\(\)\{[^}]*which=command', html)
    assert has(r"if \(!ready\(\)\)\{ jp = null; return; \}", html)
    assert has(r"if \(\(wasServo && !st\.servo\) \|\| !ready\(\)\) jp = null;", html)
    assert has(r"else \{ jp = null;", html), "a refused track target must not be integrated from"


@pytest.mark.parametrize("page", ["swerve_teleop_tool", "remote_teleop_tool"])
def test_teleop_tools_stop_on_release_cancel_and_leaving(page):
    html = PAGES[page]()
    assert has(r'for \(const type of \["pointerup", "pointercancel", "lostpointercapture"\]\) el\.addEventListener\(type, release\)', html)
    assert has(r"function release\(e\) \{.*?onChange\(0, 0\);", html), "releasing a stick must zero it"
    assert has(r'addEventListener\("visibilitychange", \(\) => \{ if \(document\.hidden\) leaving\(false\); \}\)', html)
    assert has(r'addEventListener\("pagehide", \(\) => leaving\(true\)\)', html)
    assert has(r'fetch\(NODE \+ "disable", \{ method: "POST"', html), "leaving must disable the base"


# -- every page's JavaScript parses ---------------------------------------------------------------------

NODE = shutil.which("node") or next((str(p) for p in [Path(sys.executable).parent / "node"] if p.exists()), None)


@pytest.mark.skipif(NODE is None, reason="Node.js not installed (conda install -c conda-forge nodejs)")
@pytest.mark.parametrize("page", sorted(PAGES) + ["common/ui.js"])
def test_javascript_parses(page, tmp_path):
    bodies = [ui_js()] if page == "common/ui.js" else scripts(PAGES[page]())
    assert bodies, f"{page}: no inline script found"
    for i, body in enumerate(bodies):
        f = tmp_path / f"s{i}.js"
        f.write_text(body)
        r = subprocess.run([NODE, "--check", str(f)], capture_output=True, text=True)
        if r.returncode != 0:                 # top-level await etc.: retry as a module
            m = tmp_path / f"s{i}.mjs"
            m.write_text(body)
            r = subprocess.run([NODE, "--check", str(m)], capture_output=True, text=True)
        assert r.returncode == 0, f"{page} script {i} does not parse:\n{r.stderr[-1500:]}"
