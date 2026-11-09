"""Shared visualisation components (common/viz.js): served by every node, parse, and the swerve
diagram's defaults / row adapter match the swerve node."""
import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from robot_utils.nodes.common.server import add_page_routes
from robot_utils.nodes.common.viz_assets import VIZ_JS
from robot_utils.nodes.swerve import constants as c
from robot_utils.nodes.swerve.node import FIELDS
from robot_utils.nodes.swerve.page import PAGE

NODE = shutil.which("node") or next((str(p) for p in [Path(sys.executable).parent / "node"] if p.exists()), None)
needs_node = pytest.mark.skipif(NODE is None, reason="Node.js not installed")


def run_js(tmp_path, body: str) -> str:
    f = tmp_path / "t.js"
    f.write_text("globalThis.window = globalThis;\n" + VIZ_JS + "\n" + body)
    r = subprocess.run([NODE, str(f)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr[-1500:]
    return r.stdout


def test_every_node_serves_viz_js():
    app = FastAPI()
    add_page_routes(app, {"name": "x"})
    r = TestClient(app).get("/common/viz.js")
    assert r.status_code == 200 and "javascript" in r.headers["content-type"] and "RU.viz" in r.text


@needs_node
def test_viz_js_parses(tmp_path):
    f = tmp_path / "viz.js"
    f.write_text(VIZ_JS)
    r = subprocess.run([NODE, "--check", str(f)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr[-1500:]


@needs_node
def test_swerve_defaults_match_constants(tmp_path):
    d = json.loads(run_js(tmp_path, "console.log(JSON.stringify(RU.viz.Swerve.defaults));"))
    assert d["frame"] == pytest.approx(c.FRAME) and d["wheel_d"] == pytest.approx(c.WHEEL_DIAMETER)
    # MAXSwerve plate (REV CAD): 3.5 in square about the steering axis, tabs reaching 3.49 in inward
    xs, ys = [p[0] for p in d["plate"]], [p[1] for p in d["plate"]]
    assert max(xs) == pytest.approx(1.75 * c.INCH) and max(ys) == pytest.approx(1.75 * c.INCH)
    assert min(xs) == pytest.approx(-3.49 * c.INCH) and min(ys) == pytest.approx(-3.49 * c.INCH)
    assert d["corner"] == pytest.approx(3.5 * c.INCH) and d["tube_w"] == pytest.approx(1.0 * c.INCH)
    assert [(m["name"], m["x"], m["y"]) for m in d["modules"]] == [
        (m.name, pytest.approx(m.x, abs=1e-6), pytest.approx(m.y, abs=1e-6)) for m in c.MODULES]


@needs_node
def test_swerve_from_row_reads_real_fields(tmp_path):
    row = {f: i + 0.5 for i, f in enumerate(FIELDS)}
    row.update(t_tick=10.0, enabled=1.0, motors_enabled=0.0, estop=0.0, cmd_vx=None)
    for m in c.MODULES:
        row[f"{m.name}_t_steer"] = row[f"{m.name}_t_drive"] = 9.9
    row["RL_t_drive"] = 8.0                                    # old status frame -> stale
    js = (f"const row = {json.dumps(row)};\n"
          "const get = n => n in row ? (row[n] === null ? NaN : row[n]) : undefined;\n"
          "console.log(JSON.stringify(RU.viz.Swerve.fromRow(get)));")
    d = json.loads(run_js(tmp_path, js))
    assert d["enabled"] is True and d["driving"] is False and d["estop"] is False and d["twist"][0] is None
    for m in c.MODULES:
        got = d[m.name]
        assert got["angle"] == row[f"{m.name}_steer_angle"] and got["speed"] == row[f"{m.name}_drive_vel"]
        assert got["angle_sp"] == row[f"{m.name}_steer_sp"] and got["speed_sp"] == row[f"{m.name}_drive_sp"]
        assert got["drive_a"] == row[f"{m.name}_drive_current"] and got["steer_a"] == row[f"{m.name}_steer_current"]
        assert got["stale"] is (m.name == "RL")


def test_swerve_page_uses_the_component():
    assert '<script src="common/viz.js"></script>' in PAGE and 'id="swv"' in PAGE
    assert "new RU.viz.Swerve(" in PAGE and "RU.viz.Swerve.fromRow(" in PAGE
    assert PAGE.count('RU.rows("stream"') == 1          # no second telemetry connection
