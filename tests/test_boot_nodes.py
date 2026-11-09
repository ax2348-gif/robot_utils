"""Boot nodes (nodes.yaml boot: true): config rules, unit files, sync, and the manager starting them."""
import asyncio
import subprocess

import pytest

from robot_utils.frontend.boot import MARKER, sync, unit_text
from robot_utils.frontend.config import ConfigError, load_config, parse_config
from robot_utils.frontend.manager import SystemdRunner


def nodes(*entries):
    return parse_config({"nodes": list(entries)})


def test_config_boot_needs_start_and_enabled():
    n = nodes({"name": "power", "boot": True, "start": {"module": "robot_utils.nodes.power"}})[0]
    assert n.boot and n.as_dict()["boot"] is True
    assert nodes({"name": "x", "start": {"module": "m"}})[0].boot is False
    with pytest.raises(ConfigError, match="needs a 'start'"):
        nodes({"name": "x", "boot": True})
    with pytest.raises(ConfigError, match="needs a 'start'"):
        nodes({"name": "x", "boot": True, "enabled": False, "start": {"module": "m"}})
    with pytest.raises(ConfigError, match="true or false"):
        nodes({"name": "x", "boot": "yes", "start": {"module": "m"}})


def test_packaged_registry_boots_the_power_node_only():
    boot = [n.name for n in load_config() if n.boot]
    assert boot == ["power"]


def test_unit_text():
    n = nodes({"name": "power", "boot": True, "start": {"module": "robot_utils.nodes.power", "args": ["--bus", "1"]}})[0]
    t = unit_text(n, "/r/.venv/bin/python", "/r")
    assert t.startswith(MARKER)
    assert "ExecStart=/r/.venv/bin/python -m robot_utils.nodes.power --name power --bus 1" in t
    assert "WorkingDirectory=/r" in t and "Restart=on-failure" in t and "WantedBy=default.target" in t


class FakeSystemctl:
    def __init__(self, transient_active=()):
        self.calls, self.transient_active = [], set(transient_active)

    def __call__(self, argv, **kw):
        self.calls.append(argv)
        out = ""
        if argv[:3] == ["systemctl", "--user", "show"]:
            unit = argv[3]
            out = "Transient=yes\nActiveState=active\n" if unit in self.transient_active else "Transient=no\nActiveState=inactive\n"
        if argv[:3] == ["systemctl", "--user", "stop"]:
            self.transient_active.discard(argv[3])
        return subprocess.CompletedProcess(argv, 0, out, "")


def test_sync_writes_enables_hands_over_and_removes(tmp_path):
    reg = nodes({"name": "power", "boot": True, "start": {"module": "robot_utils.nodes.power"}},
                {"name": "swerve", "start": {"module": "robot_utils.nodes.swerve"}})
    (tmp_path / "robot-node-old.service").write_text(MARKER + "\n[Unit]\n")       # ours, no longer boot
    (tmp_path / "robot-node-hand.service").write_text("[Unit]\n# someone's own unit\n")
    run = FakeSystemctl(transient_active={"robot-node-power.service"})
    did = sync(reg, unit_dir=tmp_path, python="/py", cwd="/r", run=run)
    assert (tmp_path / "robot-node-power.service").read_text() == unit_text(reg[0], "/py", "/r")
    assert not (tmp_path / "robot-node-swerve.service").exists()
    assert not (tmp_path / "robot-node-old.service").exists()
    assert (tmp_path / "robot-node-hand.service").exists()                        # never touches others
    flat = [" ".join(c) for c in run.calls]
    assert "systemctl --user stop robot-node-power.service" in flat               # transient handed over
    assert flat.index("systemctl --user stop robot-node-power.service") < flat.index("systemctl --user enable --now robot-node-power.service")
    assert "systemctl --user disable --now robot-node-old.service" in flat
    assert any("enable --now robot-node-power.service" in d for d in did)
    # a second run changes nothing on disk
    run2 = FakeSystemctl()
    did2 = sync(reg, unit_dir=tmp_path, python="/py", cwd="/r", run=run2)
    assert not any(d.startswith("write") for d in did2)
    assert ["systemctl", "--user", "daemon-reload"] not in run2.calls


def test_sync_dry_run_changes_nothing(tmp_path):
    reg = nodes({"name": "power", "boot": True, "start": {"module": "m"}})
    run = FakeSystemctl()
    did = sync(reg, unit_dir=tmp_path, python="/py", cwd="/r", run=run, dry_run=True)
    assert did and not list(tmp_path.iterdir()) and run.calls == []


def test_manager_starts_a_unit_file_with_systemctl():
    calls = []

    def fake(persistent):
        async def run(*argv, timeout=None):
            calls.append(list(argv))
            if argv[:3] == ("systemctl", "--user", "show") and "Transient" in argv:
                return 0, ("LoadState=loaded\nTransient=no\nFragmentPath=/u/robot-node-power.service\n" if persistent
                           else "LoadState=not-found\nTransient=no\nFragmentPath=\n"), ""
            if argv[:3] == ("systemctl", "--user", "show"):
                return 0, "LoadState=loaded\nActiveState=active\nMainPID=42\n", ""
            return 0, "", ""
        return run

    node = nodes({"name": "power", "boot": True, "start": {"module": "robot_utils.nodes.power"}})[0]
    asyncio.run(SystemdRunner(run=fake(True)).start(node, ["py", "-m", "x"], "/r"))
    assert ["systemctl", "--user", "start", "robot-node-power.service"] in calls
    assert not any(c[0] == "systemd-run" for c in calls)
    calls.clear()
    asyncio.run(SystemdRunner(run=fake(False)).start(node, ["py", "-m", "x"], "/r"))
    assert any(c[0] == "systemd-run" for c in calls)
