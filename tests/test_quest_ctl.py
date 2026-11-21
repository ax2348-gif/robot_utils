"""quest_up / quest_down / quest_status (tools/quest_teleop/quest_ctl.py) against a fake SSH runner (the headset's
shell, scripted) and the fake headset's stream. No real SSH, no headset."""
import subprocess
import sys
import threading
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from quest_teleop import quest_ctl  # noqa: E402
from quest_teleop.fake_headset import SCENARIOS, FakeHeadsetServer, play  # noqa: E402
from quest_teleop.quest_ctl import QuestCtl, Result, StreamInfo, probe_stream  # noqa: E402

FORBIDDEN = ("rm ", "reset", "--force", " -f ", "clean", "checkout", "stash", "push", "reboot", "kill")


class FakeQuest:
    """The headset's shell as seen through SSH: answers the commands quest_ctl sends, records them."""

    def __init__(self, *, reachable=True, behind=0, dirty=0, mode="off body-frozen", running=True,
                 pull_rc=0, run_rc=0, battery="87"):
        self.reachable, self.behind, self.dirty, self.mode = reachable, behind, dirty, mode
        self.running, self.pull_rc, self.run_rc, self.battery = running, pull_rc, run_rc, battery
        self.commands: list[str] = []

    def __call__(self, cmd, timeout=60.0):
        self.commands.append(cmd)
        if not self.reachable:
            return Result(255, "", "ssh: connect to host 100.72.77.83 port 8022: Operation timed out")
        if cmd == "echo quest-ok":
            return Result(0, "quest-ok\n")
        if "rev-list" in cmd:
            return Result(0, f"behind={self.behind}\ndirty={' ' * 6}{self.dirty}\n")
        if 'quest" pull' in cmd:
            if self.pull_rc == 0:
                self.behind = 0
            return Result(self.pull_rc, "Updated.\n", "" if self.pull_rc == 0 else "Working tree has local changes")
        if 'quest" run' in cmd:
            if self.run_rc == 0:
                self.running = True
                self.mode = "live" if " -s" in cmd else "off"
                if "--scheme=body-frozen" in cmd:
                    self.mode += " body-frozen"
            return Result(self.run_rc, "", "" if self.run_rc == 0 else "No built telemetry package")
        if 'quest" stop' in cmd:
            self.running = False
            return Result(0, "stopped\n")
        if 'quest" status' in cmd:
            return Result(0, f"telemetry {'running' if self.running else 'stopped'}\n")
        if "demo-mode" in cmd:
            return Result(0, self.mode + "\n")
        if "power_supply" in cmd:
            return Result(0, self.battery + "\n") if self.battery else Result(1, "")
        return Result(127, "", f"unexpected command: {cmd}")

    def ran(self, word):
        return [c for c in self.commands if word in c]


def ctl_for(fq, live=lambda fq: fq.running, scheme="body-frozen"):
    logs = []

    def probe(host, port, seconds):
        if live(fq):
            return StreamInfo(True, 72, 72.0, "1700000000000000001", True, "FOCUSED",
                              "body-frozen" if fq.mode.endswith("body-frozen") else "standard")
        return StreamInfo(False, error="ConnectionRefusedError")

    clock = [0.0]

    def sleep(dt):
        clock[0] += dt

    return QuestCtl(fq, "100.72.77.83", probe=probe, log=logs.append, sleep=sleep, clock=lambda: clock[0]), logs


def assert_safe(fq):
    for c in fq.commands:
        assert not any(w in c for w in FORBIDDEN), c


def test_up_unreachable_says_why_and_does_nothing_else():
    fq = FakeQuest(reachable=False)
    ctl, logs = ctl_for(fq)
    assert ctl.up() == 2
    assert len(fq.commands) == 1 and "asleep" in logs[0] and "Operation timed out" in logs[0]
    assert ctl.down() == 2 and ctl.status() == 2


def test_up_already_running_and_up_to_date_does_not_restart():
    fq = FakeQuest()
    ctl, logs = ctl_for(fq)
    assert ctl.up() == 0
    assert not fq.ran('quest" pull') and not fq.ran('quest" run')
    assert any("not restarting" in s for s in logs)
    assert any(s.startswith("stream: live, 72 Hz, session 1700000000000000001, focused") for s in logs)
    assert "battery: 87%" in logs
    assert_safe(fq)


def test_up_pulls_only_when_behind_then_restarts_to_load_it():
    fq = FakeQuest(behind=3)
    ctl, logs = ctl_for(fq)
    assert ctl.up() == 0
    assert len(fq.ran('quest" pull')) == 1
    assert fq.ran('quest" run --no-viewer --scheme=body-frozen')
    assert any("(updated)" in s for s in logs)
    assert_safe(fq)


def test_up_dirty_tree_is_reported_not_pulled_or_forced():
    fq = FakeQuest(behind=2, dirty=1, running=False, mode="")
    ctl, logs = ctl_for(fq)
    assert ctl.up() == 0
    assert not fq.ran('quest" pull')
    assert any("local change" in s and "not pulling" in s for s in logs)
    assert fq.ran('quest" run')                                 # still starts the current version
    assert_safe(fq)


def test_up_failed_pull_stops_there():
    fq = FakeQuest(behind=1, pull_rc=1)
    ctl, logs = ctl_for(fq)
    assert ctl.up() == 3
    assert not fq.ran('quest" run') and any("quest pull failed" in s for s in logs)


@pytest.mark.parametrize("scheme,demo,want", [("body-frozen", False, "off body-frozen"), ("standard", False, "off"),
                                              ("body-frozen", True, "live body-frozen")])
def test_up_starts_headless_with_the_scheme_when_the_mode_differs(scheme, demo, want):
    fq = FakeQuest(mode="live")
    ctl, logs = ctl_for(fq)
    assert ctl.up(scheme, demo) == 0
    (cmd,) = fq.ran('quest" run')
    assert f"--no-viewer --scheme={scheme}" in cmd and (" -s" in cmd) == demo
    assert fq.mode == want
    assert_safe(fq)


def test_up_waits_for_the_stream_and_times_out_with_advice():
    fq = FakeQuest(running=False, mode="")
    calls = {"n": 0}

    def live(fq_):
        calls["n"] += 1
        return fq_.running and calls["n"] > 4                    # answers after a few probes

    ctl, logs = ctl_for(fq, live)
    assert ctl.up() == 0 and calls["n"] == 5
    fq2 = FakeQuest(running=False, mode="")
    ctl2, logs2 = ctl_for(fq2, lambda fq_: False)
    assert ctl2.up(wait=3) == 5 and any("did not answer within 3 s" in s for s in logs2)


def test_restart_forces_run():
    fq = FakeQuest()
    ctl, _ = ctl_for(fq)
    assert ctl.up(restart=True) == 0 and fq.ran('quest" run')


def test_down_and_status():
    fq = FakeQuest()
    ctl, logs = ctl_for(fq)
    assert ctl.status() == 0
    assert "native: telemetry running" in logs and "mode file: off body-frozen" in logs
    assert ctl.down() == 0 and fq.ran('quest" stop') and not fq.running
    logs.clear()
    fq.battery = ""
    assert ctl.status() == 0
    assert "native: telemetry stopped" in logs and "battery: unknown" in logs
    assert any(s.startswith("stream: not answering") for s in logs)
    assert_safe(fq)


def test_commands_resolve_the_quest_env_root_like_profile_sh():
    fq = FakeQuest()
    ctl, _ = ctl_for(fq)
    ctl.status()
    (cmd,) = fq.ran('quest" status')
    assert cmd.startswith('R=$(cat "$HOME/.config/quest-env/root" 2>/dev/null || echo "$HOME/workspace/quest_env"); ')
    # the composed command is valid sh (parse only)
    assert subprocess.run(["sh", "-n", "-c", cmd]).returncode == 0
    ctl.up()
    (git,) = fq.ran("rev-list")
    assert subprocess.run(["sh", "-n", "-c", git]).returncode == 0


def test_probe_reads_the_fake_headset_stream():
    server = FakeHeadsetServer()
    stop = threading.Event()
    th = threading.Thread(target=play, args=(server, SCENARIOS["idle"]()), kwargs=dict(rate=72, loop=True, stop=stop),
                          daemon=True)
    th.start()
    try:
        info = probe_stream("127.0.0.1", server.port, 1.0)
    finally:
        stop.set()
        th.join(2)
        server.close()
    assert info.live and 55 < info.rate_hz < 90
    assert info.focused is True and info.xr_state == "FOCUSED" and info.scheme == "body-frozen"
    assert info.session_id
    dead = probe_stream("127.0.0.1", server.port, 0.2)
    assert not dead.live and dead.error


def test_ssh_runner_builds_a_batch_mode_command(monkeypatch):
    seen = {}

    def fake_run(argv, **kw):
        seen["argv"] = argv
        return subprocess.CompletedProcess(argv, 0, "quest-ok\n", "")

    monkeypatch.setattr(quest_ctl.subprocess, "run", fake_run)
    r = quest_ctl.SshRunner("quest", 5)("echo quest-ok")
    assert r.rc == 0 and seen["argv"][:6] == ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "quest"]
