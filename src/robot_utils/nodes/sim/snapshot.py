"""python -m robot_utils.nodes.sim.snapshot [--host botcore] [--dir DIR] [--show]

The robot's description state, copied for the sim (docs/sim.md, "The robot's calibrations in the sim"): the
description node's ``description.yaml`` (measured nominal camera poses, each camera's ``last_serial``),
``active.json`` and the ACTIVE calibration directories only, pulled read-only over ssh from the robot's store
(``~/.local/share/robot_utils/description``) into the sim description node's data dir (default
``~/.local/share/robot_utils/sim/description``). The sim stack refreshes it at start when the robot answers and
otherwise keeps the previous snapshot; this command refreshes it by hand (``--show``: only say what is there).

Nothing is ever written on the robot: one ``cat active.json`` and one ``tar c`` of the files named. The sim cameras
then report the robot's real serials (``last_serial``, see ``camera_serials``), so the description node applies the
active calibrations exactly as on the robot.
"""
from __future__ import annotations

import argparse
import io
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time
from pathlib import Path
from typing import Optional

import yaml

ROBOT_DIR = "~/.local/share/robot_utils/description"
SIM_DIR = os.path.expanduser("~/.local/share/robot_utils/sim/description")
INFO = "snapshot.json"
SAFE = re.compile(r"^[A-Za-z0-9_.-]+$")


class SnapshotError(RuntimeError):
    pass


def ssh_command(host: str, timeout: float) -> list[str]:
    return ["ssh", "-o", "BatchMode=yes", "-o", f"ConnectTimeout={max(1, int(timeout))}", host]


def _run(cmd: list[str], remote: str, timeout: float) -> bytes:
    try:
        r = subprocess.run(cmd + [remote], capture_output=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as e:
        raise SnapshotError(f"{' '.join(cmd)}: {type(e).__name__}: {e}") from None
    if r.returncode != 0:
        raise SnapshotError(f"{' '.join(cmd)}: exit {r.returncode}: {r.stderr.decode(errors='replace').strip()[:300]}")
    return r.stdout


def pull(host: str = "botcore", dest: str = SIM_DIR, timeout: float = 10.0, remote_dir: str = ROBOT_DIR,
         command: Optional[list[str]] = None) -> dict:
    """Copy the robot's description.yaml, active.json and active calibrations into ``dest`` (replacing what the
    previous snapshot put there; files the sim's own description node wrote, such as aborted runs, are dropped
    too). ``command``: how to run a shell command on the robot (default ssh to ``host``). Returns the info."""
    cmd = command or ssh_command(host, timeout)
    rdir = remote_dir if remote_dir.startswith("~/") else shlex.quote(remote_dir)
    active = json.loads(_run(cmd, f"cd {rdir} && cat active.json 2>/dev/null || echo '{{}}'", timeout) or b"{}")
    paths = []
    for serial, kinds in active.items():
        for kind, cid in (kinds or {}).items():
            if not all(SAFE.match(str(x)) for x in (serial, kind, cid)):
                raise SnapshotError(f"unexpected entry in active.json: {serial}/{kind}/{cid}")
            paths.append(f"calibrations/{kind}/{serial}/{cid}")
    files = " ".join(shlex.quote(p) for p in paths)
    tar = _run(cmd, f"cd {rdir} && tar cf - $(ls description.yaml active.json 2>/dev/null) {files}", timeout * 3)
    dest_p = Path(dest).expanduser()
    dest_p.parent.mkdir(parents=True, exist_ok=True)
    tmp = Path(tempfile.mkdtemp(prefix=".snapshot-", dir=dest_p.parent))
    try:
        with tarfile.open(fileobj=io.BytesIO(tar)) as tf:
            for m in tf.getmembers():
                if m.name.startswith("/") or ".." in Path(m.name).parts or not (m.isfile() or m.isdir()):
                    raise SnapshotError(f"unexpected path in the robot's archive: {m.name}")
            tf.extractall(tmp, filter="data")
        info = {"host": host, "remote_dir": remote_dir, "pulled_at": time.time(),
                "pulled_at_text": time.strftime("%Y-%m-%d %H:%M:%S"), "active": active,
                "last_serials": last_serials(tmp)}
        (tmp / INFO).write_text(json.dumps(info, indent=1))
        old = dest_p.with_name(dest_p.name + ".previous")
        if old.exists():
            shutil.rmtree(old)
        if dest_p.exists():
            os.replace(dest_p, old)
        os.replace(tmp, dest_p)
        if old.exists():
            shutil.rmtree(old)
        return info
    finally:
        if tmp.exists():
            shutil.rmtree(tmp, ignore_errors=True)


def last_serials(data_dir) -> dict:
    """{camera name: the robot's serial} from a description data dir's description.yaml."""
    p = Path(data_dir).expanduser() / "description.yaml"
    try:
        d = yaml.safe_load(p.read_text()) or {}
    except (OSError, yaml.YAMLError):
        return {}
    return {name: c["last_serial"] for name, c in (d.get("cameras") or {}).items()
            if isinstance(c, dict) and c.get("last_serial")}


def info(data_dir: str = SIM_DIR) -> Optional[dict]:
    p = Path(data_dir).expanduser() / INFO
    try:
        return json.loads(p.read_text())
    except (OSError, ValueError):
        return None


def describe(data_dir: str = SIM_DIR) -> str:
    """One line on what the sim's description node will use."""
    i = info(data_dir)
    if i is None:
        return (f"robot description snapshot: NONE in {data_dir}: camera mounts are NOMINAL "
                "(pull one: python -m robot_utils.nodes.sim.snapshot)")
    act = "; ".join(f"{s}: " + ", ".join(f"{k} {c}" for k, c in sorted(kinds.items())) for s, kinds in
                    sorted((i.get("active") or {}).items())) or "no active calibrations (measured nominal poses only)"
    cams = ", ".join(f"{n}={s}" for n, s in sorted((i.get("last_serials") or {}).items())) or "none"
    return (f"robot description snapshot from {i.get('host')} pulled {i.get('pulled_at_text')}: {act}; "
            f"camera serials {cams}")


def camera_serials(description: dict, cameras: dict) -> dict:
    """{sim camera name: serial to report}: the description node's serial for a camera of the same name (the
    robot's ``last_serial``), else the sim config's. ``description``: the description node's GET /description."""
    out = {}
    for name, cc in (cameras or {}).items():
        s = ((description.get("cameras") or {}).get(name) or {}).get("serial")
        out[name] = s or cc.get("serial") or f"SIM-{name}"
    return out


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="python -m robot_utils.nodes.sim.snapshot", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="botcore", help="the robot's ssh host (default botcore)")
    p.add_argument("--dir", default=SIM_DIR, help=f"the sim description node's data dir (default {SIM_DIR})")
    p.add_argument("--timeout", type=float, default=10.0, help="s for the ssh connection (default 10)")
    p.add_argument("--show", action="store_true", help="only print what the snapshot holds")
    a = p.parse_args(argv)
    if not a.show:
        try:
            pull(a.host, a.dir, a.timeout)
        except SnapshotError as e:
            print(f"pull from {a.host} failed: {e}", file=sys.stderr)
            print(describe(a.dir))
            return 1
    print(describe(a.dir))
    return 0


if __name__ == "__main__":
    sys.exit(main())
