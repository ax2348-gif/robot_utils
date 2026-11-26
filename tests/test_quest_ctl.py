"""quest_up / quest_down / quest_status (tools/quest_teleop/quest_ctl.py) against a fake SSH runner (the headset's
shell, scripted) and the fake headset's stream. No real SSH, no headset."""
import json
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
                 pull_rc=0, run_rc=0, battery="87", xr=5):
        self.reachable, self.behind, self.dirty, self.mode = reachable, behind, dirty, mode
        self.running, self.pull_rc, self.run_rc, self.battery = running, pull_rc, run_rc, battery
        self.xr = xr
        self.ar = None                        # the robot AR URL in ~/.config/quest-env/robot-ar (None: no file)
        self.body = None                      # joints= in ~/.config/quest-env/body-tracking (None: no file)
        self.loaded = self.package = None     # sha256 of the running library / of the clone's package (None: unknown)
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
                self.ar = cmd.split("--ar=", 1)[1].split()[0] if "--ar=" in cmd else None
                self.body = cmd.split("--body=", 1)[1].split()[0] if "--body=" in cmd else None
                self.loaded = self.package
            return Result(self.run_rc, "", "" if self.run_rc == 0 else "No built telemetry package")
        if 'quest" stop' in cmd:
            self.running = False
            return Result(0, "stopped\n")
        if 'quest" status' in cmd:              # the real output shape (live check, 2026-11-21)
            return Result(0, json.dumps({"ok": True, "runtime_pid": 5939, "xr_pids": [11444] if self.running else [],
                                         **({"selected": f"/data/native/apps/{self.loaded}/libapp.so"}
                                            if self.loaded else {}),
                                         "last_load": {"pid": 11444, "state": "entry-returned"},
                                         "last_load_process_alive": self.running}) + "\n")
        if "sha256sum" in cmd:
            return Result(0, f"{self.package}  libapp.so\n") if self.package else Result(1, "", "No such file")
        if cmd.startswith("logcat"):
            return Result(0, f"11-21 18:50:44.031 11444 11475 I QuestRuntimeXR: OpenXR session state {self.xr}\n")
        if "body-tracking" in cmd:
            return Result(0, f"joints={self.body}\n") if self.body else Result(1, "")
        if "robot-ar" in cmd:
            return Result(0, f"url={self.ar}\n") if self.ar else Result(1, "")
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
            dev = {"battery_pct": int(fq.battery), "charging": False, "plugged": False, "temperature_c": 31.0} \
                if fq.battery else None
            return StreamInfo(True, 72, 72.0, "1700000000000000001", True, "FOCUSED",
                              "body-frozen" if fq.mode.endswith("body-frozen") else "standard", device=dev)
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
    assert "battery: 87%, 31 C" in logs                    # from the stream's device block
    assert_safe(fq)


def test_up_body_tracking_restarts_only_when_the_setting_changes():
    fq = FakeQuest()
    ctl, logs = ctl_for(fq)
    assert ctl.up(body="core") == 0
    (cmd,) = fq.ran('quest" run')
    assert cmd.endswith("--body=core") and any("body off -> core" in s for s in logs)
    assert ctl.up(body="core") == 0 and len(fq.ran('quest" run')) == 1       # already so: no restart
    assert ctl.up() == 0
    assert "--body" not in fq.ran('quest" run')[-1] and fq.body is None      # off: the option is never sent
    assert any("body core -> off" in s for s in logs)
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
    fq2.xr = 1                                                 # what the live check met: session IDLE, not worn
    ctl2, logs2 = ctl_for(fq2, lambda fq_: False)
    assert ctl2.up(wait=3) == 5
    assert any("did not answer within 3 s" in s and "XR session: IDLE (headset not worn?)" in s for s in logs2)


def test_restart_forces_run():
    fq = FakeQuest()
    ctl, _ = ctl_for(fq)
    assert ctl.up(restart=True) == 0 and fq.ran('quest" run')


def test_down_and_status():
    fq = FakeQuest()
    ctl, logs = ctl_for(fq)
    assert ctl.status() == 0
    assert any(s.startswith('native: {"ok": true') for s in logs) and "mode file: off body-frozen" in logs
    assert "XR session: FOCUSED" in logs
    assert ctl.down() == 0 and fq.ran('quest" stop') and not fq.running
    logs.clear()
    fq.battery = ""
    assert ctl.status() == 0
    assert "XR session: not running" in logs
    assert "battery: unavailable while telemetry is stopped (the SSH shell cannot read it)" in logs
    assert any(s.startswith("stream: not answering") for s in logs)
    assert_safe(fq)


def test_commands_resolve_the_quest_env_root_like_profile_sh():
    fq = FakeQuest()
    ctl, _ = ctl_for(fq)
    ctl.status()
    cmd = fq.ran('quest" status')[0]
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
    assert r.rc == 0 and seen["argv"][:7] == ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "quest"]


def test_up_with_ar_restarts_only_when_the_ar_url_changes():
    fq = FakeQuest()
    ctl, logs = ctl_for(fq)
    assert ctl.up(ar="http://100.65.75.106:8080") == 0
    assert fq.ran("--ar=http://100.65.75.106:8080") and fq.ar == "http://100.65.75.106:8080"
    assert any("AR off -> http://100.65.75.106:8080" in s for s in logs)
    n = len(fq.ran('quest" run'))
    assert ctl.up(ar="http://100.65.75.106:8080") == 0 and len(fq.ran('quest" run')) == n     # same URL: no restart
    assert ctl.up() == 0 and len(fq.ran('quest" run')) == n + 1 and fq.ar is None            # AR off: restart
    assert_safe(fq)


def test_ar_values_resolve_to_the_url_the_headset_opens():
    from quest_teleop.quest_ctl import resolve_ar
    local = lambda host: {"100.72.77.83": "100.65.75.106"}[host]     # noqa: E731
    assert resolve_ar("auto", "100.72.77.83", local) == "http://100.65.75.106:8080"
    assert resolve_ar(":8090", "100.72.77.83", local) == "http://100.65.75.106:8090"
    assert resolve_ar("erid:8080", "100.72.77.83", local) == "http://erid:8080"
    assert resolve_ar("http://botcore-desktop.local/", "100.72.77.83", local) == "http://botcore-desktop.local"


def test_local_address_toward_picks_the_route_without_sending():
    from quest_teleop.quest_ctl import local_address_toward
    assert local_address_toward("127.0.0.1") == "127.0.0.1"


def test_battery_comes_from_the_stream_never_from_ssh():
    """The SSH shell runs in an Android app sandbox (sysfs and the battery service denied): quest_ctl reads the
    battery only from the telemetry stream's device block, and says why when it cannot."""
    from quest_teleop.quest_ctl import QuestCtl
    on = StreamInfo(True, 72, 72.0, "s", True, "FOCUSED", "body-frozen",
                    device={"battery_pct": 54, "charging": True, "plugged": True, "temperature_c": 33.4})
    assert QuestCtl.battery(on) == "54%, charging, 33 C"
    old = StreamInfo(True, 72, 72.0, "s", True, "FOCUSED", "body-frozen")
    assert QuestCtl.battery(old).startswith("not reported")
    assert QuestCtl.battery(StreamInfo(False)).startswith("unavailable while telemetry is stopped")
    fq = FakeQuest()
    ctl, _ = ctl_for(fq)
    ctl.status()
    assert not fq.ran("power_supply") and not fq.ran("dumpsys")


def test_up_restarts_when_the_running_library_is_not_the_clones_package():
    """A branch checked out on the headset (or files copied) leaves the old library running: up reloads it."""
    fq = FakeQuest()
    fq.loaded, fq.package = "a" * 64, "b" * 64
    ctl, logs = ctl_for(fq)
    assert ctl.up() == 0
    assert fq.ran('quest" run') and any("not the clone's package" in s for s in logs)
    n = len(fq.ran('quest" run'))
    assert ctl.up() == 0 and len(fq.ran('quest" run')) == n      # now the same: not restarted again
    assert_safe(fq)
