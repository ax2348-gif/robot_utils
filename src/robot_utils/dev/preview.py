"""Preview every page under every theme, on a local simulated robot (never the real one).

    python -m robot_utils.dev.preview up     [--port 8090] [--themes DIR]
    python -m robot_utils.dev.preview shots  [--out DIR] [--themes DIR] [--theme a,b] [--widths 1280,390]
    python -m robot_utils.dev.preview check  [--themes DIR] [--theme a,b] [--widths 1280,390]

``up`` starts all nodes in simulation (swerve ``--sim``, camera ``--fake``, the rest pointed at them)
plus the landing page on ``127.0.0.1:<port>``, in a private temp folder: its own sockets, config,
state and locks (``XDG_RUNTIME_DIR``, ``XDG_CONFIG_HOME``, ``XDG_STATE_HOME``, ``ROBOT_LOCKS_DIR``),
so it touches nothing else on this machine and can run next to another copy.

``--themes DIR`` adds the ``*.css`` files in DIR (candidate themes) to the packaged ones; they are
copied in at start and again before each page load, so edits show on reload.

``shots`` writes full-page screenshots to ``<out>/<theme>/<scheme>-<width>/<page>.png`` plus an
``index.html`` contact sheet. ``check`` runs the visual safety checks (dev/visual_safety.py) and
exits 1 on any problem. Both need Playwright with Chromium:

    pip install playwright && python -m playwright install chromium

``--url`` points ``shots`` / ``check`` at a stack that is already up; only local addresses are
accepted, so these tools can never reach the robot.
"""
from __future__ import annotations

import argparse
import html
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request
from contextlib import contextmanager
from pathlib import Path
from urllib.parse import urlparse

from .visual_safety import PAGES, Result, check_page

NODES = [  # (instance name, module, extra args): everything in simulation
    ("swerve", "robot_utils.nodes.swerve", ["--sim"]),
    ("camera", "robot_utils.nodes.orbbec", ["--fake"]),
    ("estimator", "robot_utils.nodes.estimator", []),
    ("description", "robot_utils.nodes.description", []),
    ("pose_servo", "robot_utils.nodes.pose_servo", []),
    ("3d_perception", "robot_utils.nodes.perception3d", ["--allow-nominal"]),
    ("agent_nav", "robot_utils.nodes.agent_nav", ["--no-require-imu"]),
]
LOCAL = {"127.0.0.1", "localhost", "::1"}
SRC = Path(__file__).resolve().parents[2]          # .../src


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def sync_themes(themes_dir: Path | None, dest: Path) -> None:
    dest.mkdir(parents=True, exist_ok=True)
    if themes_dir:
        for f in Path(themes_dir).glob("*.css"):
            shutil.copyfile(f, dest / f.name)


@contextmanager
def stack(port: int | None = None, themes_dir: Path | None = None, quiet: bool = True):
    """The whole robot in simulation, isolated in a temp folder. Yields (base URL, root folder)."""
    root = Path(tempfile.mkdtemp(prefix="rup", dir="/tmp"))   # short: unix socket paths are limited
    port = port or free_port()
    env = dict(os.environ, XDG_RUNTIME_DIR=str(root), XDG_CONFIG_HOME=str(root / "cfg"),
               XDG_STATE_HOME=str(root / "state"), ROBOT_LOCKS_DIR=str(root / "locks"),
               PYTHONPATH=str(SRC) + os.pathsep + os.environ.get("PYTHONPATH", ""))
    sync_themes(themes_dir, root / "cfg" / "robot_utils" / "themes")
    # a registry without start: entries, so the landing page never tries systemd here
    (root / "nodes.yaml").write_text("nodes:\n" + "".join(f"  - {{name: {n}}}\n" for n, _, _ in NODES))
    logs = root / "logs"
    logs.mkdir()
    procs = []

    def spawn(name, args):
        out = open(logs / f"{name}.log", "w")
        procs.append((name, subprocess.Popen([sys.executable, *args], env=env, stdout=out, stderr=subprocess.STDOUT,
                                             start_new_session=True)))
    try:
        for name, module, extra in NODES:
            spawn(name, ["-m", module, "--name", name, *extra])
            time.sleep(0.3)        # swerve / camera first: the others connect to them
        spawn("frontend", ["-m", "robot_utils.frontend", "--config", str(root / "nodes.yaml"), "--no-observe",
                           "--port", str(port), "--host", "127.0.0.1"])
        base = f"http://127.0.0.1:{port}"
        deadline = time.time() + 30
        while True:
            try:
                urllib.request.urlopen(base + "/api/health", timeout=1).read()
                break
            except OSError:
                if time.time() > deadline:
                    raise RuntimeError(f"landing page did not come up; logs in {logs}")
                time.sleep(0.3)
        deadline = time.time() + 20
        missing = []
        while time.time() < deadline:
            missing = [n for n, _, _ in NODES if not (root / "robot" / f"{n}.sock").exists()]
            if not missing:
                break
            time.sleep(0.3)
        for n in missing:
            print(f"preview: node {n} did not start (see {logs / (n + '.log')}); its page will show as down",
                  file=sys.stderr)
        if not quiet:
            print(f"preview: {base}/  (sim stack in {root}; logs in {logs})", flush=True)
        yield base, root
    finally:
        for _, p in procs:
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for _, p in procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
        shutil.rmtree(root, ignore_errors=True)


def theme_names(base: str) -> list[str]:
    d = json.loads(urllib.request.urlopen(base + "/themes/", timeout=5).read())
    return ["builtin"] + [t["name"] for t in d.get("themes", [])]


def run_matrix(base: str, themes: list[str], widths: list[int], schemes: list[str], pages=PAGES,
               shots: Path | None = None, settle_s: float = 1.5, refresh=None) -> list[Result]:
    """Load each page x theme x scheme x width in Chromium; check it, and screenshot it if asked."""
    if urlparse(base).hostname not in LOCAL:
        raise SystemExit(f"refusing {base}: preview tools only talk to a local simulated stack")
    from playwright.sync_api import sync_playwright
    results = []
    with sync_playwright() as pw:
        browser = pw.chromium.launch()
        try:
            for theme in themes:
                for scheme in schemes:
                    for width in widths:
                        height = 900 if width >= 768 else 844
                        ctx = browser.new_context(viewport={"width": width, "height": height}, color_scheme=scheme)
                        ctx.add_init_script(f"try {{ localStorage.setItem('ru-theme', {json.dumps(theme)}); }} catch(e){{}}")
                        page = ctx.new_page()
                        for pg in pages:
                            if refresh:
                                refresh()
                            res = Result(pg.name, theme, scheme, width)
                            try:
                                page.goto(base + pg.path, wait_until="load", timeout=15000)
                                page.wait_for_timeout(settle_s * 1000)
                                res.problems = check_page(page, pg, theme)
                                if shots:
                                    d = shots / theme / f"{scheme}-{width}"
                                    d.mkdir(parents=True, exist_ok=True)
                                    page.screenshot(path=str(d / f"{pg.name}.png"), full_page=True)
                            except Exception as e:  # noqa: BLE001 - reported as a problem, run continues
                                res.problems = [f"load failed: {type(e).__name__}: {str(e).splitlines()[0][:160]}"]
                            results.append(res)
                        ctx.close()
        finally:
            browser.close()
    return results


def contact_sheet(out: Path, results: list[Result]) -> Path:
    rows = {}
    for r in results:
        rows.setdefault((r.theme, r.scheme, r.width), []).append(r)
    parts = ["<!doctype html><meta charset=utf-8><title>Theme preview</title><style>"
             "body{font:14px system-ui;margin:16px;background:#eee}h2{margin:24px 0 8px}"
             ".row{display:flex;gap:12px;overflow-x:auto}figure{margin:0;background:#fff;padding:6px;border-radius:6px}"
             "figure img{width:260px;display:block}figcaption{font-size:12px}.bad{color:#b00;font-weight:600}</style>"]
    for (theme, scheme, width), rs in rows.items():
        parts.append(f"<h2>{html.escape(theme)} · {scheme} · {width}px</h2><div class=row>")
        for r in rs:
            src = f"{theme}/{scheme}-{width}/{r.page}.png"
            msg = "ok" if r.ok else "; ".join(r.problems)
            parts.append(f'<figure><a href="{html.escape(src)}"><img src="{html.escape(src)}" loading=lazy></a>'
                         f'<figcaption>{html.escape(r.page)} <span class="{"" if r.ok else "bad"}">'
                         f'{html.escape(msg)}</span></figcaption></figure>')
        parts.append("</div>")
    f = out / "index.html"
    f.write_text("".join(parts))
    return f


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m robot_utils.dev.preview", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["up", "shots", "check"])
    ap.add_argument("--port", type=int, default=None, help="landing page port for 'up' (default 8090)")
    ap.add_argument("--themes", type=Path, help="folder of candidate theme .css files to add")
    ap.add_argument("--theme", help="comma-separated themes to run (default: builtin + all)")
    ap.add_argument("--widths", default="1280,390")
    ap.add_argument("--schemes", default="light,dark")
    ap.add_argument("--pages", help="comma-separated page names (default: all)")
    ap.add_argument("--out", type=Path, default=Path("preview-out"))
    ap.add_argument("--url", help="use a stack that is already up (local addresses only)")
    a = ap.parse_args(argv)

    if a.cmd == "up":
        with stack(a.port or 8090, a.themes, quiet=False):
            print("preview: Ctrl-C to stop", flush=True)
            try:
                while True:
                    time.sleep(3600)
            except KeyboardInterrupt:
                return 0

    def go(base, root):
        dest = root / "cfg" / "robot_utils" / "themes" if root else None
        themes = a.theme.split(",") if a.theme else theme_names(base)
        pages = [p for p in PAGES if not a.pages or p.name in a.pages.split(",")]
        results = run_matrix(base, themes, [int(w) for w in a.widths.split(",")], a.schemes.split(","), pages,
                             shots=a.out if a.cmd == "shots" else None,
                             refresh=(lambda: sync_themes(a.themes, dest)) if (a.themes and dest) else None)
        failed = [r for r in results if not r.ok]
        for r in results:
            print(("ok   " if r.ok else "FAIL ") + r.label() + ("" if r.ok else "  " + "; ".join(r.problems)))
        if a.cmd == "shots":
            print(f"contact sheet: {contact_sheet(a.out, results).resolve()}")
        print(f"{len(results) - len(failed)}/{len(results)} passed")
        return 1 if failed and a.cmd == "check" else 0

    if a.url:
        return go(a.url.rstrip("/"), None)
    with stack(None, a.themes) as (base, root):
        return go(base, root)


if __name__ == "__main__":
    sys.exit(main())
