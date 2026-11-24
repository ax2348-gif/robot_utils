"""tools/quest_teleop/quest_setup.py: the parsers, the ssh config block, the tar, and the wizard against a simulated
headset (adb, the runtime's provider, ssh): a fresh headset is set up in one run, a rerun changes nothing, a
refused key is only replaced on a yes, and a network failure never rewrites the keys."""
import argparse
import io
import sys
import tarfile
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "quest_teleop"))

import quest_setup as Q  # noqa: E402

IP_ADDR = """1: lo: <LOOPBACK,UP> mtu 65536
    inet 127.0.0.1/8 scope host lo
3: wlan0: <BROADCAST,UP> mtu 1500
    inet 192.168.1.23/24 brd 192.168.1.255 scope global wlan0
9: tun0: <POINTOPOINT,UP> mtu 1280
    inet 100.101.102.103/32 scope global tun0
"""


def test_parsers():
    assert Q.parse_adb_devices("List of devices attached\n1WMHH0000\tdevice\nABC\tunauthorized\n\n") == [
        ("1WMHH0000", "device"), ("ABC", "unauthorized")]
    assert Q.tailnet_addresses(IP_ADDR) == ["100.101.102.103"]
    assert Q.tailnet_addresses("utun4: flags=8051\n\tinet 100.64.0.7 --> 100.64.0.7 netmask 0xffffffff\n") == [
        "100.64.0.7"]
    assert Q.tailnet_addresses("inet 100.128.0.1 netmask") == []          # outside 100.64.0.0/10
    assert Q.version_tuple("1.6.0") > Q.version_tuple("1.5.1")


def test_ssh_block_is_prepended_then_replaced_in_place(tmp_path):
    other = "Host *\n    User someone\n\nHost robot\n    HostName 10.0.0.2\n"
    b1 = Q.ssh_block("quest", "100.101.102.103", Path.home() / ".ssh/quest_ed25519", Path.home() / ".ssh/kh")
    c1 = Q.with_block(other, b1)
    assert c1.startswith(Q.BLOCK_BEGIN) and c1.endswith(other)           # before `Host *`: ours wins
    assert "IdentityFile ~/.ssh/quest_ed25519" in c1
    b2 = Q.ssh_block("quest", "100.64.0.9", Path.home() / ".ssh/quest_ed25519", Path.home() / ".ssh/kh")
    c2 = Q.with_block(c1, b2)
    assert c2.count(Q.BLOCK_BEGIN) == 1 and "100.64.0.9" in c2 and "100.101.102.103" not in c2
    assert Q.unmanaged_hosts(c2) == {"*", "robot"}
    assert Q.with_block("", b1) == b1


def test_tar_tree_skips_git_and_keeps_modes(tmp_path):
    (tmp_path / ".git").mkdir()
    (tmp_path / ".git" / "HEAD").write_text("x")
    (tmp_path / "build").mkdir()
    (tmp_path / "build" / "big").write_text("x")
    (tmp_path / "shell").mkdir()
    (tmp_path / "quest").write_text("#!/system/bin/sh\n")
    (tmp_path / "quest").chmod(0o755)
    (tmp_path / "shell" / "profile.sh").write_text("q() { :; }\n")
    (tmp_path / "._quest").write_text("apple")
    names = {}
    with tarfile.open(fileobj=io.BytesIO(Q.tar_tree(tmp_path))) as t:
        for m in t.getmembers():
            names[m.name] = m.mode
    assert set(names) == {"quest", "shell/profile.sh"}
    assert names["quest"] & 0o111


class FakeHeadset:
    """adb + the runtime's content provider + ssh, as the wizard drives them."""

    def __init__(self, keys=None, tailnet_ip=None, runtime="1.6.0", reachable=True):
        self.keys = keys                     # authorised public keys (None: never provisioned)
        self.tailnet_ip = tailnet_ip         # the runtime's configured address
        self.runtime = runtime
        self.tailscale = True
        self.ip = "100.101.102.103"
        self.reachable = reachable           # the tailnet route from the computer
        self.writes = []
        self.installs = []
        self.quest_env = None
        self.started = 0
        self.signed_elsewhere = False        # the installed runtime came from another build (no in-place update)
        self.uninstalls = 0

    def status(self):
        if not self.keys:
            return "Provision authorized_keys before starting SSH."
        if self.tailnet_ip != self.ip:
            return "Waiting for the configured Tailscale address."
        return f"SSH running: quest@{self.ip}, 127.0.0.1 port 8022"

    def adb(self, *args, data=None):
        a = list(args)
        if a[:2] == ["shell", "getprop"]:
            return 0, {"ro.product.manufacturer": "Oculus", "ro.product.model": "Quest 3"}[a[2]]
        if a[:3] == ["shell", "pm", "list"]:
            return 0, f"package:{a[4]}" if a[4] == Q.TAILSCALE_PACKAGE and self.tailscale else ""
        if a[:2] == ["shell", "ip"]:
            return 0, IP_ADDR.replace("100.101.102.103", self.ip)
        if a[:2] == ["shell", "dumpsys"]:
            return 0, f"versionName={self.runtime}" if self.runtime else ""
        if a[:3] == ["shell", "content", "read"]:
            return 0, self.status() if a[-1].endswith("/status") else "ssh-ed25519 AAAAhost"
        if a[:3] == ["shell", "content", "write"]:
            name = a[-1].rsplit("/", 1)[1]
            self.writes.append(name)
            if name == "authorized_keys":
                self.keys = data.decode()
            else:
                self.tailnet_ip = data.decode().strip()
            return 0, ""
        if a[:3] == ["shell", "content", "call"]:
            self.started += 1
            return 0, "Result: Bundle[{}]"
        if a[0] == "uninstall":
            self.uninstalls += 1
            self.runtime, self.keys, self.tailnet_ip, self.signed_elsewhere = None, None, None, False
            return 0, "Success"
        if a[:2] == ["install", "-r"] and self.signed_elsewhere:
            return 1, "Failure [INSTALL_FAILED_UPDATE_INCOMPATIBLE: signatures do not match]"
        if a[0] == "install":
            self.installs.append(a[-1])
            self.runtime = Q.RUNTIME_VERSION
            return 0, "Success"
        raise AssertionError(f"unexpected adb {a}")

    def ssh(self, command, data=None):
        if not self.reachable or self.tailnet_ip != self.ip:
            return 255, "ssh: connect to host port 8022: Operation timed out"
        if not self.keys or "AAAAmine" not in self.keys:
            return 255, "quest@host: Permission denied (publickey)."
        if command == "echo quest-ok":
            return 0, "quest-ok\n"
        if "is-git" in command:
            return 0, ("present\n" + self.quest_env) if self.quest_env else ""
        if "tar -xf" in command:
            assert tarfile.open(fileobj=io.BytesIO(data)).getnames()
            self.quest_env = command.split("echo ")[1].split(" >")[0]
            return 0, "copied\n"
        if "install-profile" in command:
            return 0, "telemetry OK\n"
        raise AssertionError(f"unexpected ssh {command}")


def wizard(tmp_path, monkeypatch, hs, answers=(), **over):
    monkeypatch.setattr(Path, "home", lambda: tmp_path)
    args = dict(check=False, yes=False, serial=None, alias="quest", key=str(tmp_path / ".ssh/quest_ed25519"),
                extra_keys=None, apk=str(tmp_path / "rt.apk"), reinstall_runtime=False, quest_env=None,
                skip_telemetry=True, cache=str(tmp_path / "cache"),
                backup_dir=str(tmp_path / "backups"))
    args.update(over)
    w = Q.Wizard(argparse.Namespace(**args))
    w.interactive = True
    replies = list(answers)
    monkeypatch.setattr("builtins.input", lambda prompt="": replies.pop(0))
    w.find_adb = lambda: "/usr/bin/adb"
    w.adb_cmd = lambda *a, timeout=60, data=None: hs.adb(*a, data=data)
    w.ssh = lambda command, timeout=30, data=None: hs.ssh(command, data)
    w.local_tailnet = lambda: ["100.64.0.5"]
    w.port_open = lambda: hs.reachable
    w.wait_for = lambda what, check, timeout, every=3.0: check()

    def run(argv, timeout=60, data=None):
        import subprocess
        if argv[1:] == ["devices"]:
            return subprocess.CompletedProcess(argv, 0, b"List of devices attached\n1WMHH0000\tdevice\n", b"")
        if argv[0] == "ssh-keygen":
            key = Path(argv[-1])
            key.parent.mkdir(parents=True, exist_ok=True)
            key.write_text("PRIVATE")
            key.with_suffix(".pub").write_text("ssh-ed25519 AAAAmine quest@test\n")
            return subprocess.CompletedProcess(argv, 0, b"", b"")
        if argv[:2] == ["ssh", "-G"]:
            return subprocess.CompletedProcess(argv, 0, b"hostname 100.101.102.103\nport 8022\n", b"")
        raise AssertionError(f"unexpected run {argv}")
    w.run = run
    src = tmp_path / "qe"
    (src / "shell").mkdir(parents=True, exist_ok=True)
    (src / "quest").write_text("#!/system/bin/sh\n")
    w.quest_env_source = lambda: src
    return w


def test_fresh_headset_in_one_run_and_a_rerun_changes_nothing(tmp_path, monkeypatch):
    hs = FakeHeadset(runtime=None)
    assert wizard(tmp_path, monkeypatch, hs).run_all() == 0
    assert hs.installs and hs.writes == ["authorized_keys", "tailnet_ip"] and hs.quest_env == Q.QUEST_ENV_REF
    cfg = (tmp_path / ".ssh/config").read_text()
    assert "HostName 100.101.102.103" in cfg and "Port 8022" in cfg
    before = (list(hs.installs), list(hs.writes), cfg)
    assert wizard(tmp_path, monkeypatch, hs).run_all() == 0
    assert (hs.installs, hs.writes, (tmp_path / ".ssh/config").read_text()) == before


def test_a_changed_tailscale_address_is_rewritten_without_touching_the_keys(tmp_path, monkeypatch):
    hs = FakeHeadset()
    assert wizard(tmp_path, monkeypatch, hs).run_all() == 0
    hs.ip, hs.writes = "100.70.1.2", []                     # the headset got a new tailnet address
    assert wizard(tmp_path, monkeypatch, hs).run_all() == 0
    assert hs.writes == ["tailnet_ip"] and "HostName 100.70.1.2" in (tmp_path / ".ssh/config").read_text()


def test_someone_elses_keys_are_replaced_only_on_a_yes(tmp_path, monkeypatch):
    hs = FakeHeadset(keys="ssh-ed25519 AAAAowner owner@laptop\n", tailnet_ip="100.101.102.103")
    assert wizard(tmp_path, monkeypatch, hs, answers=["n"]).run_all() == 1
    assert hs.writes == [] and "AAAAowner" in hs.keys
    assert wizard(tmp_path, monkeypatch, hs, answers=["y"]).run_all() == 0
    assert hs.writes == ["authorized_keys", "tailnet_ip"] and "AAAAmine" in hs.keys


def test_an_unreachable_tailnet_never_rewrites_the_keys(tmp_path, monkeypatch):
    hs = FakeHeadset(keys="ssh-ed25519 AAAAmine quest@test\n", tailnet_ip="100.101.102.103", reachable=False)
    assert wizard(tmp_path, monkeypatch, hs).run_all() == 1
    assert hs.writes == []


def test_check_only_changes_nothing(tmp_path, monkeypatch):
    hs = FakeHeadset(runtime=None)
    assert wizard(tmp_path, monkeypatch, hs, check=True).run_all() == 1
    assert hs.installs == [] and hs.writes == [] and not (tmp_path / ".ssh/config").exists()


def test_an_unmanaged_host_entry_elsewhere_stops_the_wizard(tmp_path, monkeypatch):
    (tmp_path / ".ssh").mkdir()
    (tmp_path / ".ssh/config").write_text("Host quest\n    HostName 100.99.0.1\n")
    hs = FakeHeadset()
    w = wizard(tmp_path, monkeypatch, hs)
    import subprocess
    inner = w.run
    w.run = lambda argv, timeout=60, data=None: (subprocess.CompletedProcess(
        argv, 0, b"hostname 100.99.0.1\nport 22\n", b"") if argv[:2] == ["ssh", "-G"] else inner(argv, timeout, data))
    assert w.run_all() == 1
    assert (tmp_path / ".ssh/config").read_text() == "Host quest\n    HostName 100.99.0.1\n"


def test_a_runtime_from_another_build_is_migrated_with_its_data(tmp_path, monkeypatch):
    hs = FakeHeadset(keys="ssh-ed25519 AAAAmine quest@test\nssh-ed25519 AAAArobot robot\n",
                     tailnet_ip="100.101.102.103", runtime="1.5.3")
    hs.signed_elsewhere = True
    w = wizard(tmp_path, monkeypatch, hs, answers=["y"])
    steps = []
    w.backup_runtime = lambda: steps.append(("backup", hs.uninstalls, hs.keys)) or tmp_path / "b.tar"
    def restore(backup):
        steps.append(("restore", hs.uninstalls, backup))
        hs.keys, hs.tailnet_ip = steps[0][2], hs.ip           # what restore_runtime puts back
    w.restore_runtime = restore
    assert w.run_all() == 0
    assert [s[:2] for s in steps] == [("backup", 0), ("restore", 1)] and hs.runtime == Q.RUNTIME_VERSION
    assert "AAAArobot" in hs.keys                               # other machines keep their access


def test_declining_the_uninstall_keeps_a_working_runtime_and_carries_on(tmp_path, monkeypatch):
    hs = FakeHeadset(keys="ssh-ed25519 AAAAmine quest@test\n", tailnet_ip="100.101.102.103", runtime="1.5.3")
    hs.signed_elsewhere = True
    w = wizard(tmp_path, monkeypatch, hs, answers=["n", "n"])  # no migration, no uninstall
    w.backup_runtime = lambda: (_ for _ in ()).throw(AssertionError("no backup on a no"))
    assert w.run_all() == 0
    assert hs.uninstalls == 0 and hs.runtime == "1.5.3" and hs.quest_env == Q.QUEST_ENV_REF
    assert ("runtime installed", True, "kept version 1.5.3, another build") in w.results
