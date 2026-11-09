"""Nodes that start at boot: persistent systemd user units for the nodes.yaml entries with ``boot: true``.

    python -m robot_utils.frontend.boot [--config nodes.yaml] [--dry-run]

Every other node runs as a transient unit the landing page starts (``systemd-run``; gone after a
reboot). A ``boot: true`` node instead gets a unit file ``~/.config/systemd/user/robot-node-<name>.service``,
enabled for ``default.target`` (the user's lingering session, so at boot) with ``Restart=on-failure``.
It has the same unit name, so the landing page's Start / Stop / logs work on it unchanged (the
manager starts a unit that has a file with ``systemctl start``). Stopping it from the page stops it
until the next boot; to stop it starting at boot, drop ``boot: true`` and run this again.

This command (``install.sh`` runs it):
- writes or updates the unit file of every boot node, and enables and starts it; a node already
  running as a transient unit is stopped first and comes back as the persistent unit (a few
  seconds' gap);
- disables and removes the unit files it wrote for nodes that are no longer ``boot: true`` (only
  files carrying its marker line, never anything else in that folder).
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path
from typing import Callable

from .config import NodeEntry, load_config
from .manager import STOP_TIMEOUT_S, unit_name

MARKER = "# written by robot_utils.frontend.boot (nodes.yaml boot: true); edits are overwritten"
UNIT_DIR = Path(os.environ.get("XDG_CONFIG_HOME", os.path.expanduser("~/.config"))) / "systemd" / "user"


def unit_text(node: NodeEntry, python: str, cwd: str) -> str:
    argv = " ".join(_quote(a) for a in node.start.argv(python, node.name))
    return f"""{MARKER}
[Unit]
Description=robot_utils node {node.name} (starts at boot)

[Service]
WorkingDirectory={cwd}
ExecStart={argv}
Environment=PYTHONUNBUFFERED=1
Restart=on-failure
RestartSec=5
TimeoutStopSec={STOP_TIMEOUT_S}
KillMode=mixed

[Install]
WantedBy=default.target
"""


def _quote(a: str) -> str:
    return '"' + a.replace("\\", "\\\\").replace('"', '\\"') + '"' if any(c in a for c in ' "\\') else a


def _show(run, unit: str) -> dict:
    out = run(["systemctl", "--user", "show", unit, "--no-pager", "-p", "Transient", "-p", "ActiveState",
               "-p", "LoadState"], capture_output=True, text=True).stdout
    return dict(line.split("=", 1) for line in out.splitlines() if "=" in line)


def sync(nodes: list[NodeEntry], *, unit_dir: Path = UNIT_DIR, python: str = sys.executable,
         cwd: str | None = None, run: Callable = subprocess.run, dry_run: bool = False) -> list[str]:
    """Bring the unit files in line with nodes.yaml. Returns what it did (one line per action)."""
    cwd = cwd or os.getcwd()
    did: list[str] = []
    want = {unit_name(n): n for n in nodes if n.boot and n.enabled and n.start and n.host is None}
    changed: list[str] = []
    for unit, node in want.items():
        f, text = unit_dir / unit, unit_text(node, python, cwd)
        if f.exists() and f.read_text() == text:
            continue
        did.append(f"write {f}")
        changed.append(unit)
        if not dry_run:
            unit_dir.mkdir(parents=True, exist_ok=True)
            tmp = f.with_suffix(".tmp")
            tmp.write_text(text)
            os.replace(tmp, f)
    stale = [f for f in sorted(unit_dir.glob("robot-node-*.service"))
             if f.name not in want and f.read_text(errors="replace").startswith(MARKER)]
    for f in stale:
        did.append(f"disable and remove {f.name}")
        if not dry_run:
            run(["systemctl", "--user", "disable", "--now", f.name], capture_output=True, text=True)
            f.unlink()
    if dry_run:
        return did + [f"enable and start {u}" for u in want]
    if changed or stale:
        run(["systemctl", "--user", "daemon-reload"], capture_output=True, text=True)
    for unit in want:
        st = _show(run, unit)
        if st.get("Transient") == "yes" and st.get("ActiveState") in ("active", "activating"):
            # the landing page started it before it became a boot node: hand over to the unit file
            did.append(f"stop transient {unit}")
            run(["systemctl", "--user", "stop", unit], capture_output=True, text=True)
            run(["systemctl", "--user", "daemon-reload"], capture_output=True, text=True)
        r = run(["systemctl", "--user", "enable", "--now", unit], capture_output=True, text=True)
        did.append(f"enable --now {unit}" + ("" if r.returncode == 0 else f": FAILED {r.stderr.strip()}"))
    return did


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m robot_utils.frontend.boot", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", help="node registry YAML (default: the packaged nodes.yaml)")
    ap.add_argument("--dry-run", action="store_true", help="say what would change; change nothing")
    a = ap.parse_args(argv)
    did = sync(load_config(a.config), dry_run=a.dry_run)
    for line in did or ["nothing to do"]:
        print(line)
    return 1 if any("FAILED" in line for line in did) else 0


if __name__ == "__main__":
    sys.exit(main())
