#!/usr/bin/env python3
"""Set up a Meta Quest for teleop (robot or simulator): an interactive wizard, safe to rerun.

Plug the headset in over USB-C and run it from the robot_utils checkout:

    python3 tools/quest_teleop/quest_setup.py            # walks you through every step
    python3 tools/quest_teleop/quest_setup.py --check    # only reports what is set up; changes nothing

Each step checks first and only fixes what is missing, so after a failure fix the cause and run it again. What it
sets up (the checklist of what to have ready is in docs/quest_setup.md):

1. adb sees the headset (USB debugging allowed in the headset).
2. Tailscale runs on this computer (a 100.x address).
3. Tailscale on the headset: installed over USB if missing, then signed in (you do that in the headset).
4. The Quest developer runtime (SSH server + native XR host; github.com/ax2348-gif/quest_dev) at the pinned
   version, verified by SHA-256.
5. An SSH key for the headset (``~/.ssh/quest_ed25519``) authorised on it, the runtime listening on the headset's
   Tailscale address, and a ``Host quest`` entry in ``~/.ssh/config`` (a managed block at the top).
6. quest_env (the headset app; github.com/ax2348-gif/quest_env) at the pinned version in the headset's workspace,
   unless the headset already has its own Git clone there.
7. Optionally: telemetry started once and the stream checked (``quest_up``), then stopped.

After that the headset needs no cable: ``tools/quest_teleop/quest_up`` (or **Quest up** on the quest_teleop node's
page) reaches it over Tailscale wherever both are online. Standard library only; macOS and Linux.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import ipaddress
import os
import platform
import re
import shutil
import socket
import subprocess
import sys
import tarfile
import time
import urllib.request
from pathlib import Path
from typing import Callable, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))
from quest_ctl import QuestCtl, SshRunner  # noqa: E402

# -- pins: what a collaborator gets (bump together with the tags) ----------------------------------------------
RUNTIME_PACKAGE = "dev.quest.runtime"
RUNTIME_VERSION = "1.6.0"
RUNTIME_URL = "https://raw.githubusercontent.com/ax2348-gif/quest_dev/collab-v2/artifacts/quest-dev-runtime.apk"
RUNTIME_SHA256 = "edc738e0154f2df6c3ae23c7e26ab5f0f46aa3e5518cf92aba807981ce3e4e9d"
QUEST_ENV_REF = "collab-v3"
QUEST_ENV_GIT = "https://github.com/ax2348-gif/quest_env.git"
QUEST_ENV_URL = f"https://codeload.github.com/ax2348-gif/quest_env/tar.gz/refs/tags/{QUEST_ENV_REF}"
TAILSCALE_PACKAGE = "com.tailscale.ipn"
TAILSCALE_APK_URL = "https://pkgs.tailscale.com/stable/tailscale-android-universal-latest.apk"

TAILNET = ipaddress.ip_network("100.64.0.0/10")
SSH_PORT = 8022
PROVIDER = f"content://{RUNTIME_PACKAGE}.files"
BLOCK_BEGIN = "# >>> robot_utils quest (managed by tools/quest_teleop/quest_setup.py; edits inside are replaced) >>>"
BLOCK_END = "# <<< robot_utils quest <<<"
REF_FILE = ".robot_utils_ref"
HEADSET_ROOT = '"$HOME/workspace/quest_env"'
# the runtime's app data (its home is files/home) and what a migration carries across a reinstall
APP_DATA = '"$HOME/../.."'
RUNTIME_DATA = ("files/home", "files/native", "files/ssh_host_ed25519_key", "files/ssh_host_ed25519_key.pub",
                "files/host_public_key.txt", "files/tailnet_ip", "shared_prefs", "blackbox/data")


class Abort(Exception):
    """A step cannot continue: the message says what to do, then rerun."""


# -- pure helpers (tested in tests/test_quest_setup.py) ---------------------------------------------------------

def parse_adb_devices(out: str) -> list[tuple[str, str]]:
    """``adb devices`` output -> [(serial, state)], state: device | unauthorized | offline | ..."""
    rows = []
    for line in out.splitlines():
        if not line.strip() or line.startswith(("List of devices", "*")):
            continue
        parts = line.split()
        if len(parts) >= 2:
            rows.append((parts[0], parts[1]))
    return rows


def tailnet_addresses(text: str) -> list[str]:
    """Every IPv4 address in 100.64.0.0/10 in ``ip addr`` / ``ifconfig`` output, in order, without duplicates."""
    found = []
    for m in re.finditer(r"\binet\s+(?:addr:)?(\d+\.\d+\.\d+\.\d+)", text):
        a = m.group(1)
        try:
            if ipaddress.ip_address(a) in TAILNET and a not in found:
                found.append(a)
        except ValueError:
            pass
    return found


def version_tuple(v: str) -> tuple:
    return tuple(int(x) for x in re.findall(r"\d+", v)[:3])


def tilde(p) -> str:
    home = str(Path.home())
    return "~" + str(p)[len(home):] if str(p).startswith(home) else str(p)


def ssh_block(alias: str, host: str, key: Path, known_hosts: Path) -> str:
    return "\n".join([
        BLOCK_BEGIN,
        f"Host {alias}",
        f"    HostName {host}",
        f"    Port {SSH_PORT}",
        "    User quest",
        f"    IdentityFile {tilde(key)}",
        "    IdentitiesOnly yes",
        f"    UserKnownHostsFile {tilde(known_hosts)}",
        "    StrictHostKeyChecking accept-new",
        "    ServerAliveInterval 15",
        BLOCK_END, ""])


def unmanaged_hosts(config: str) -> set[str]:
    """Host patterns declared outside our managed block."""
    out, inside = set(), False
    for line in config.splitlines():
        s = line.strip()
        if s == BLOCK_BEGIN:
            inside = True
        elif s == BLOCK_END:
            inside = False
        elif not inside and re.match(r"(?i)host\s", s):
            out.update(s.split()[1:])
    return out


def with_block(config: str, block: str) -> str:
    """``config`` with our managed block replaced, or prepended (ssh takes the first value it finds, so a later
    ``Host *`` cannot override ours)."""
    lines = config.splitlines(keepends=True)
    try:
        b = next(i for i, l in enumerate(lines) if l.strip() == BLOCK_BEGIN)
        e = next(i for i, l in enumerate(lines) if i > b and l.strip() == BLOCK_END)
        return "".join(lines[:b]) + block + "".join(lines[e + 1:])
    except StopIteration:
        return block + ("\n" + config if config.strip() else "")


def tar_tree(src: Path) -> bytes:
    """A ustar archive of ``src`` (modes kept), without .git, build output, caches or macOS metadata."""
    skip_dirs = {".git", "build", "__pycache__", ".pytest_cache"}
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w", format=tarfile.USTAR_FORMAT) as tar:
        for root, dirs, files in os.walk(src):
            dirs[:] = sorted(d for d in dirs if d not in skip_dirs)
            for name in sorted(files):
                if name in (".DS_Store", ".git") or name.startswith("._"):
                    continue
                p = Path(root) / name
                if p.is_symlink():
                    continue
                info = tar.gettarinfo(str(p), arcname=str(p.relative_to(src)))
                info.uid = info.gid = 0
                info.uname = info.gname = ""
                with open(p, "rb") as f:
                    tar.addfile(info, f)
    return buf.getvalue()


# -- the wizard -------------------------------------------------------------------------------------------------

class Wizard:
    def __init__(self, a: argparse.Namespace):
        self.a = a
        self.check_only = a.check
        self.interactive = sys.stdin.isatty() and not a.yes
        self.cache = Path(a.cache).expanduser()
        self.key = Path(a.key).expanduser()
        self.known_hosts = Path.home() / ".ssh" / "known_hosts_quest"
        self.ssh_config = Path.home() / ".ssh" / "config"
        self.backup_dir = Path(a.backup_dir).expanduser()
        self.reinstalled = False
        self.adb: Optional[str] = None
        self.serial: Optional[str] = None
        self.quest_ip: Optional[str] = None
        self.results: list[tuple[str, bool, str]] = []

    # ui
    def say(self, msg: str = "") -> None:
        print(msg, flush=True)

    def step(self, title: str) -> None:
        self.say(f"\n== {title}")

    def ask(self, question: str, default: bool) -> bool:
        if not self.interactive:
            return default
        hint = "[Y/n]" if default else "[y/N]"
        while True:
            r = input(f"   {question} {hint} ").strip().lower()
            if not r:
                return default
            if r in ("y", "yes", "n", "no"):
                return r.startswith("y")

    def wait_for(self, what: str, check: Callable[[], bool], timeout: float, every: float = 3.0) -> bool:
        """Poll ``check`` until it passes or ``timeout`` runs out (Ctrl-C stops waiting)."""
        if check():
            return True
        if self.check_only:
            return False
        self.say(f"   waiting for {what} (up to {timeout:.0f} s; Ctrl-C to stop waiting) ...")
        t_end = time.monotonic() + timeout
        try:
            while time.monotonic() < t_end:
                time.sleep(every)
                if check():
                    return True
        except KeyboardInterrupt:
            self.say("")
        return False

    # processes
    def run(self, argv: list[str], timeout: float = 60, data: Optional[bytes] = None) -> subprocess.CompletedProcess:
        try:
            return subprocess.run(argv, input=data, capture_output=True, timeout=timeout)
        except subprocess.TimeoutExpired:
            return subprocess.CompletedProcess(argv, 124, b"", f"timed out after {timeout:.0f} s".encode())
        except FileNotFoundError:
            return subprocess.CompletedProcess(argv, 127, b"", f"{argv[0]} not found".encode())

    def adb_cmd(self, *args: str, timeout: float = 60, data: Optional[bytes] = None) -> tuple[int, str]:
        argv = [self.adb] + (["-s", self.serial] if self.serial else []) + list(args)
        p = self.run(argv, timeout, data)
        return p.returncode, (p.stdout + p.stderr).decode(errors="replace")

    def key_opts(self) -> list[str]:
        """Offer this wizard's key too: an owner's own Host entry may not name it."""
        return ["-o", f"IdentityFile={self.key}"] if self.key.exists() else []

    def ssh(self, command: str, timeout: float = 30, data: Optional[bytes] = None, plain: bool = False
            ) -> tuple[int, str]:
        p = self.run(["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
                      *([] if plain else self.key_opts()), self.a.alias, command], timeout, data)
        return p.returncode, (p.stdout + p.stderr).decode(errors="replace")

    def download(self, url: str, dest: Path, sha256: Optional[str] = None) -> Path:
        if dest.exists() and (sha256 is None or hashlib.sha256(dest.read_bytes()).hexdigest() == sha256):
            return dest
        self.say(f"   downloading {url}")
        dest.parent.mkdir(parents=True, exist_ok=True)
        tmp = dest.with_suffix(dest.suffix + ".part")
        try:
            with urllib.request.urlopen(url, timeout=60) as r, open(tmp, "wb") as f:
                shutil.copyfileobj(r, f)
        except OSError as e:
            raise Abort(f"download failed ({e}); check the network and rerun") from None
        digest = hashlib.sha256(tmp.read_bytes()).hexdigest()
        if sha256 is not None and digest != sha256:
            tmp.unlink()
            raise Abort(f"{url} has SHA-256 {digest}, expected {sha256}: not installing it. Is this checkout's "
                        "quest_setup.py current (git pull)?")
        tmp.replace(dest)
        return dest

    def record(self, name: str, ok: bool, detail: str = "") -> bool:
        self.results.append((name, ok, detail))
        self.say(f"   {'ok' if ok else 'NOT DONE'}: {name}" + (f" ({detail})" if detail else ""))
        return ok

    # 1. adb
    def find_adb(self) -> Optional[str]:
        cands = [shutil.which("adb")]
        for env in ("ANDROID_HOME", "ANDROID_SDK_ROOT"):
            if os.environ.get(env):
                cands.append(str(Path(os.environ[env]) / "platform-tools" / "adb"))
        cands += [str(Path.home() / "Library/Android/sdk/platform-tools/adb"),
                  str(Path.home() / "Android/Sdk/platform-tools/adb"), "/opt/homebrew/bin/adb", "/usr/local/bin/adb"]
        return next((c for c in cands if c and os.access(c, os.X_OK)), None)

    def step_adb(self) -> bool:
        self.step("1. The headset over USB (adb)")
        self.adb = self.find_adb()
        if not self.adb:
            hint = ("brew install android-platform-tools" if platform.system() == "Darwin" else
                    "sudo apt install adb   (or your distribution's android-tools package)")
            raise Abort(f"adb not found. Install it: {hint}")

        def devices() -> list[tuple[str, str]]:
            p = self.run([self.adb, "devices"], 15)
            return parse_adb_devices(p.stdout.decode(errors="replace"))

        def ready() -> bool:
            rows = devices()
            if self.a.serial:
                rows = [r for r in rows if r[0] == self.a.serial]
            return any(state == "device" for _, state in rows)

        rows = devices()
        if not ready():
            if any(state == "unauthorized" for _, state in rows):
                self.say("   The headset asks for permission: put it on and choose Allow on 'Allow USB debugging?' "
                         "(tick 'Always allow from this computer').")
            else:
                self.say("   No headset found. Plug it in with a USB-C data cable and wake it. If it still does not "
                         "show up, developer mode is not enabled yet: in the Meta Horizon phone app, Devices > your "
                         "headset > Headset settings > Developer mode (needs a Meta developer account), then reboot "
                         "the headset.")
            if not self.wait_for("the headset to appear on adb", ready, 180):
                raise Abort("adb does not see an authorised headset")
        rows = [r for r in devices() if r[1] == "device" and (not self.a.serial or r[0] == self.a.serial)]
        if len(rows) > 1:
            raise Abort("several devices on adb: rerun with --serial " + " | ".join(r[0] for r in rows))
        self.serial = rows[0][0]
        _, maker = self.adb_cmd("shell", "getprop", "ro.product.manufacturer", timeout=15)
        _, model = self.adb_cmd("shell", "getprop", "ro.product.model", timeout=15)
        if maker.strip().lower() not in ("oculus", "meta"):
            self.say(f"   note: this device reports {maker.strip()} {model.strip()}, not a Meta Quest")
        return self.record("headset on adb", True, f"{maker.strip()} {model.strip()}, serial {self.serial}")

    # 2. Tailscale here
    def local_tailnet(self) -> list[str]:
        if platform.system() == "Darwin":
            out = self.run(["ifconfig"], 10).stdout.decode(errors="replace")
        else:
            out = self.run(["ip", "-4", "addr"], 10).stdout.decode(errors="replace")
        return tailnet_addresses(out)

    def step_local_tailscale(self) -> bool:
        self.step("2. Tailscale on this computer")
        if not self.local_tailnet():
            self.say("   No Tailscale address on this computer. Install Tailscale (https://tailscale.com/download), "
                     "sign in, and make sure it shows Connected.")
            if not self.wait_for("a Tailscale address here", lambda: bool(self.local_tailnet()), 300):
                raise Abort("this computer is not on a tailnet")
        return self.record("Tailscale on this computer", True, ", ".join(self.local_tailnet()))

    # 3. Tailscale on the headset
    def installed(self, package: str) -> bool:
        rc, out = self.adb_cmd("shell", "pm", "list", "packages", package, timeout=20)
        return rc == 0 and f"package:{package}" in out.split()

    def headset_tailnet(self) -> list[str]:
        rc, out = self.adb_cmd("shell", "ip", "-4", "addr", timeout=15)
        found = tailnet_addresses(out) if rc == 0 else []
        if not found:
            _, out = self.adb_cmd("shell", "ifconfig", timeout=15)
            found = tailnet_addresses(out)
        return found

    def install_apk(self, apk: Path, package: str, what: str) -> bool:
        """Install or update in place. An APK signed by another build cannot be updated: for the runtime, offer to
        migrate (back up its data, reinstall, restore) when SSH reaches it; else ask before an erasing uninstall.
        False when the owner keeps the installed app."""
        rc, out = self.adb_cmd("install", "-r", str(apk), timeout=300)
        if rc == 0 and "Success" in out:
            return True
        if "INSTALL_FAILED_UPDATE_INCOMPATIBLE" in out:
            self.say(f"   The {what} on the headset was signed by another build, so it cannot be updated in place: "
                     "it has to be uninstalled, which erases its data.")
            if package == RUNTIME_PACKAGE and self.ssh_ok()[0]:
                self.say("   The installed runtime answers over SSH, so its data can be carried over: its home "
                         "(workspace, dotfiles, installed tools), authorised keys, SSH host key (other machines keep "
                         f"trusting it) and settings are backed up to {self.backup_dir}, then restored into the new "
                         "runtime.")
                if self.ask("Back up, reinstall and restore?", True):
                    backup = self.backup_runtime()
                    self.reinstall(apk, package, what)
                    self.restore_runtime(backup)
                    return True
            elif package == RUNTIME_PACKAGE:
                self.say("   It does not answer over SSH from here, so its data cannot be backed up first: "
                         "uninstalling erases its home, workspace, authorised keys and SSH host key (machines that "
                         "knew the headset then see a new host key).")
            if not self.ask(f"Uninstall the {what} and install this one?", False):
                self.say(f"   keeping the installed {what} (it works, but gets no updates from this release)")
                return False
            self.reinstall(apk, package, what)
            return True
        raise Abort(f"installing the {what} failed: {out.strip()[-300:]}")

    def reinstall(self, apk: Path, package: str, what: str) -> None:
        self.reinstalled = self.reinstalled or package == RUNTIME_PACKAGE
        self.adb_cmd("uninstall", package, timeout=60)
        rc, out = self.adb_cmd("install", str(apk), timeout=300)
        if not (rc == 0 and "Success" in out):
            raise Abort(f"installing the {what} failed: {out.strip()[-300:]}")

    # migrating the runtime's data across a reinstall
    def ssh_stream(self, command: str, src=None, dst=None, extra: tuple = (), timeout: float = 3600) -> tuple[int, str]:
        """Run ``command`` over SSH, piping the file ``src`` into it or its output into the file ``dst``; prints
        progress. Returns (exit code, stderr)."""
        argv = ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", "-o", "ServerAliveInterval=15",
                *self.key_opts(), *extra, self.a.alias, command]
        p = subprocess.Popen(argv, stdin=subprocess.PIPE if src else subprocess.DEVNULL,
                             stdout=subprocess.PIPE if dst else subprocess.DEVNULL, stderr=subprocess.PIPE)
        moved, shown, t_end = 0, time.monotonic(), time.monotonic() + timeout
        try:
            pipe_in, pipe_out = (src, p.stdin) if src else (p.stdout, dst)
            while True:
                chunk = pipe_in.read(1 << 20)
                if not chunk:
                    break
                pipe_out.write(chunk)
                moved += len(chunk)
                if time.monotonic() - shown > 5:
                    shown = time.monotonic()
                    self.say(f"   ... {moved / 1e6:.0f} MB")
                if time.monotonic() > t_end:
                    p.kill()
                    break
            if src:
                p.stdin.close()
            p.wait(timeout=600)
        except BaseException:
            p.kill()
            raise
        return p.returncode, p.stderr.read().decode(errors="replace")

    def backup_runtime(self) -> Path:
        files = " ".join(RUNTIME_DATA)
        rc, out = self.ssh(f"cd {APP_DATA} && find {files} ! -type s ! -type p 2>/dev/null | wc -l", 120)
        expected = int(out.split()[-1]) if rc == 0 and out.split() and out.split()[-1].isdigit() else None
        self.backup_dir.mkdir(parents=True, exist_ok=True)
        dest = self.backup_dir / f"runtime-{self.serial or 'quest'}-{time.strftime('%Y%m%d-%H%M%S')}.tar"
        self.say(f"   backing up the runtime's data to {dest} ({expected} entries; sockets are skipped)")
        with open(dest, "wb") as f:
            rc, err = self.ssh_stream(f"cd {APP_DATA} && find {files} ! -type s ! -type p 2>/dev/null | "
                                      "tar -cf - --no-recursion -T -", dst=f)
        if rc != 0:
            raise Abort(f"the backup failed (exit {rc}: {err.strip()[-200:]}); nothing was uninstalled. {dest} is "
                        "incomplete")
        with tarfile.open(dest) as t:
            names = set(t.getnames())
        missing = [n for n in ("files/ssh_host_ed25519_key", "files/home/.ssh/authorized_keys") if n not in names]
        if missing or (expected is not None and len(names) < expected):
            raise Abort(f"the backup looks incomplete ({len(names)} of {expected} entries; missing {missing}); "
                        "nothing was uninstalled")
        self.say(f"   backed up {len(names)} entries, {dest.stat().st_size / 1e6:.0f} MB (kept; delete it once "
                 "the headset works)")
        return dest

    def restore_runtime(self, backup: Path) -> None:
        with tarfile.open(backup) as t:
            keys = t.extractfile("files/home/.ssh/authorized_keys").read()
        self.say("   restoring: the old authorised keys and address first, so this computer can log in")
        self.write_runtime("authorized_keys", keys)
        self.write_runtime("tailnet_ip", self.quest_ip.encode() + b"\n")
        self.adb_cmd("shell", "content", "call", "--uri", PROVIDER, "--method", "start", timeout=30)
        # the fresh install made a new host key; trust exactly that one (read over USB) for the restore
        pinned = self.cache / "migrate_known_hosts"
        extra = ("-o", f"UserKnownHostsFile={pinned}", "-o", "StrictHostKeyChecking=yes",
                 "-o", "HostKeyAlias=quest-migrate")

        def pin() -> bool:
            _, out = self.adb_cmd("shell", "content", "read", "--uri", f"{PROVIDER}/host_public_key", timeout=20)
            key = next((l.strip() for l in out.splitlines() if l.startswith("ssh-")), None)
            if key:
                pinned.parent.mkdir(parents=True, exist_ok=True)
                pinned.write_text("quest-migrate " + " ".join(key.split()[:2]) + "\n")
            return bool(key)

        def reachable() -> bool:
            p = self.run(["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", *self.key_opts(), *extra,
                          self.a.alias, "echo quest-ok"], 20)
            return p.returncode == 0 and b"quest-ok" in p.stdout

        if not (self.wait_for("the new runtime's host key", pin, 60, 2.0)
                and self.wait_for("the new runtime's SSH", reachable, 60, 2.0)):
            raise Abort(f"the new runtime does not answer over SSH yet; its data is safe in {backup}. Rerun the "
                        f"wizard, or restore by hand: docs/quest_setup.md, 'Migrating a runtime'")
        self.say(f"   restoring {backup.name} (everything but the BlackBox container)")
        with open(backup, "rb") as f:
            rc, err = self.ssh_stream(f"cd {APP_DATA} && tar -xpf - --exclude 'blackbox/*'", src=f, extra=extra)
        if rc != 0:
            raise Abort(f"the restore failed (exit {rc}: {err.strip()[-200:]}); the data is safe in {backup}")
        # restart, so SSH serves the restored host key
        self.adb_cmd("shell", "content", "call", "--uri", PROVIDER, "--method", "stop", timeout=30)
        time.sleep(2)
        self.adb_cmd("shell", "content", "call", "--uri", PROVIDER, "--method", "start", timeout=30)
        if not self.wait_for("SSH with the restored host key", lambda: self.ssh_ok()[0], 60, 2.0):
            raise Abort(f"SSH does not answer with the restored host key: {self.ssh_ok()[1][-200:]}; the data is "
                        f"safe in {backup}")
        pinned.unlink(missing_ok=True)
        self.say("   restored: the same host key, keys and home as before")

    def step_headset_tailscale(self) -> bool:
        self.step("3. Tailscale on the headset")
        if not self.installed(TAILSCALE_PACKAGE):
            if self.check_only:
                return self.record("Tailscale app on the headset", False, "not installed")
            self.say("   The Tailscale app is not on the headset. It is installed over USB from tailscale.com's "
                     "Android package.")
            if not self.ask("Download and install it now?", True):
                raise Abort("install Tailscale on the headset (pkgs.tailscale.com/stable, Android APK) and rerun")
            apk = self.download(TAILSCALE_APK_URL, self.cache / "tailscale-android.apk")
            self.install_apk(apk, TAILSCALE_PACKAGE, "Tailscale app")
        ips = self.headset_tailnet()
        if not ips:
            if self.check_only:
                return self.record("Tailscale on the headset", False, "installed, not connected")
            self.adb_cmd("shell", "monkey", "-p", TAILSCALE_PACKAGE, "-c", "android.intent.category.LAUNCHER", "1",
                         timeout=20)
            self.say("   Now in the headset: the Tailscale app should be open (otherwise: Library > Unknown "
                     "sources > Tailscale). Sign in with the same account (tailnet) as this computer, allow the "
                     "VPN connection when asked, and leave it Connected. Optional, for after reboots: in the app's "
                     "settings, keep 'Always-on VPN' / start on boot enabled.")
            if not self.wait_for("the headset's Tailscale address", lambda: bool(self.headset_tailnet()), 600):
                raise Abort("the headset has no Tailscale address yet")
            ips = self.headset_tailnet()
        self.quest_ip = ips[0]
        return self.record("Tailscale on the headset", True, self.quest_ip)

    # 4. the runtime
    def runtime_version(self) -> Optional[str]:
        rc, out = self.adb_cmd("shell", "dumpsys", "package", RUNTIME_PACKAGE, timeout=30)
        m = re.search(r"versionName=(\S+)", out) if rc == 0 else None
        return m.group(1) if m else None

    def step_runtime(self) -> bool:
        self.step("4. Quest developer runtime (SSH server, native XR host)")
        have = self.runtime_version()
        if have and version_tuple(have) >= version_tuple(RUNTIME_VERSION) and not self.a.reinstall_runtime:
            return self.record("runtime installed", True, f"version {have}")
        if self.check_only:
            return self.record("runtime installed", False, f"version {have}" if have else "not installed")
        apk = Path(self.a.apk).expanduser() if self.a.apk else self.download(
            RUNTIME_URL, self.cache / f"quest-dev-runtime-{RUNTIME_VERSION}.apk", RUNTIME_SHA256)
        self.say(f"   installing the runtime {RUNTIME_VERSION}" + (f" (replacing {have})" if have else ""))
        if not self.install_apk(apk, RUNTIME_PACKAGE, "runtime"):
            return self.record("runtime installed", True, f"kept version {have}, another build")
        return self.record("runtime installed", True, f"version {self.runtime_version()}")

    # 5. key, ssh config, provisioning
    def runtime_status(self) -> str:
        """The runtime's status line; waits while its provider does not answer yet (just after an install)."""
        out = ""
        for _ in range(10):
            rc, out = self.adb_cmd("shell", "content", "read", "--uri", f"{PROVIDER}/status", timeout=20)
            if rc == 0 and "rror" not in out:
                return out.strip()
            time.sleep(2)
        raise Abort(f"the runtime does not answer over USB ({out.strip()[-200:]}). Open it once in the headset "
                    "(Library > Unknown sources > Quest developer runtime) and rerun")

    def ensure_key(self) -> None:
        if self.key.exists() and self.key.with_suffix(".pub").exists():
            return
        if self.check_only:
            raise Abort(f"no SSH key at {self.key}")
        self.key.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        p = self.run(["ssh-keygen", "-q", "-t", "ed25519", "-N", "", "-C", f"quest@{socket.gethostname()}",
                      "-f", str(self.key)], 30)
        if p.returncode != 0:
            raise Abort(f"ssh-keygen failed: {p.stderr.decode(errors='replace').strip()}")
        self.say(f"   created the SSH key {self.key} (no passphrase; it only opens the headset)")

    def write_ssh_config(self) -> None:
        text = self.ssh_config.read_text() if self.ssh_config.exists() else ""
        if self.a.alias in unmanaged_hosts(text):
            p = self.run(["ssh", "-G", self.a.alias], 10)
            conf = dict(l.split(" ", 1) for l in p.stdout.decode(errors="replace").splitlines() if " " in l)
            if conf.get("hostname") == self.quest_ip and conf.get("port") == str(SSH_PORT):
                self.say(f"   ~/.ssh/config already has its own 'Host {self.a.alias}' for {self.quest_ip}: using it")
                return
            raise Abort(f"~/.ssh/config already has a 'Host {self.a.alias}' entry (outside this wizard's block) "
                        f"pointing at {conf.get('hostname')}:{conf.get('port')}. Remove or rename it, or rerun with "
                        "--alias NAME (then start the teleop node with --ssh-host NAME)")
        new = with_block(text, ssh_block(self.a.alias, self.quest_ip, self.key, self.known_hosts))
        if new == text:
            return
        if self.check_only:
            raise Abort(f"~/.ssh/config has no current 'Host {self.a.alias}' block")
        self.ssh_config.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        backup = self.ssh_config.with_name("config.before-quest-setup")
        if self.ssh_config.exists() and not backup.exists():
            shutil.copy2(self.ssh_config, backup)
        self.ssh_config.write_text(new)
        os.chmod(self.ssh_config, 0o600)
        self.say(f"   ~/.ssh/config: 'Host {self.a.alias}' -> {self.quest_ip}:{SSH_PORT}")

    def ssh_ok(self, plain: bool = False) -> tuple[bool, str]:
        rc, out = self.ssh("echo quest-ok", 20, plain=plain)
        return rc == 0 and "quest-ok" in out, out.strip()

    def ssh_option(self, name: str) -> str:
        p = self.run(["ssh", "-G", self.a.alias], 10)
        for line in p.stdout.decode(errors="replace").splitlines():
            k, _, v = line.partition(" ")
            if k == name:
                return v
        return ""

    def write_runtime(self, name: str, data: bytes) -> None:
        rc, out = self.adb_cmd("shell", "content", "write", "--uri", f"{PROVIDER}/{name}", data=data, timeout=30)
        if rc != 0 or "rror" in out:
            raise Abort(f"writing {name} to the runtime failed: {out.strip()[-300:]}")

    def provision(self, keys_too: bool) -> None:
        """Write the listen address (always: it is the headset's own Tailscale address) and, with ``keys_too``, the
        authorised keys; then (re)start the SSH service."""
        if keys_too:
            keys = [self.key.with_suffix(".pub").read_text().strip()] + self.extra_keys()
            self.write_runtime("authorized_keys", ("\n".join(keys) + "\n").encode())
        self.write_runtime("tailnet_ip", self.quest_ip.encode() + b"\n")
        self.adb_cmd("shell", "content", "call", "--uri", PROVIDER, "--method", "start", timeout=30)

    def extra_keys(self) -> list[str]:
        """The public key lines of --extra-keys files."""
        out = []
        for name in self.a.extra_keys or []:
            text = Path(name).expanduser().read_text()
            if "PRIVATE KEY" in text:
                raise Abort(f"--extra-keys {name} holds a private key: pass the .pub file")
            out += [l.strip() for l in text.splitlines() if re.match(r"(ssh-|ecdsa-|sk-)", l.strip())]
        return out

    def known_hosts_line(self) -> str:
        _, out = self.adb_cmd("shell", "content", "read", "--uri", f"{PROVIDER}/host_public_key", timeout=20)
        key = next((l.split()[:2] for l in out.splitlines() if l.startswith("ssh-")), None)
        return f"[{self.quest_ip}]:{SSH_PORT} {' '.join(key)}" if key else "(host key unavailable over USB)"

    def ensure_extra_keys(self) -> bool:
        """Add any --extra-keys line the headset does not authorise yet (by key, not comment); never remove."""
        want = self.extra_keys()
        if not want:
            return True
        rc, have = self.ssh("cat ~/.ssh/authorized_keys", 20)
        if rc != 0:
            return False
        blobs = {l.split()[1] for l in have.splitlines() if len(l.split()) > 1}
        missing = [k for k in want if k.split()[1] not in blobs]
        if not missing:
            return True
        if self.check_only:
            self.say(f"   {len(missing)} of the --extra-keys are not authorised yet")
            return False
        rc, out = self.ssh("cat >> ~/.ssh/authorized_keys && echo added", 20, data=("\n".join(missing) + "\n").encode())
        self.say(f"   authorised {len(missing)} more key(s): " + ", ".join(k.split()[2] if len(k.split()) > 2 else
                                                                         k.split()[1][-12:] for k in missing))
        return rc == 0 and "added" in out

    def port_open(self) -> bool:
        try:
            socket.create_connection((self.quest_ip, SSH_PORT), timeout=5).close()
            return True
        except OSError:
            return False

    def step_ssh(self) -> bool:
        self.step("5. SSH to the headset over Tailscale")
        if self.ssh_ok(plain=True)[0]:   # already works (this wizard's entry, or the owner's own)
            ok = self.ensure_extra_keys()
            return self.record(f"ssh {self.a.alias}", ok, f"{self.quest_ip}:{SSH_PORT}" if ok else
                               "could not authorise the --extra-keys")
        self.ensure_key()
        self.write_ssh_config()
        ok, out = self.ssh_ok()
        if not ok and "REMOTE HOST IDENTIFICATION HAS CHANGED" in out:
            if not self.forget_host_key():
                return self.record(f"ssh {self.a.alias}", False, "host key changed")
            ok, out = self.ssh_ok()
        if not ok:
            if self.check_only:
                return self.record(f"ssh {self.a.alias}", False, out[-200:])
            self.provision_for_ssh()
        if not self.ensure_extra_keys():
            return self.record(f"ssh {self.a.alias}", False, "could not authorise the --extra-keys")
        if not self.ssh_ok(plain=True)[0] and not self.add_identity_to_own_entry():
            return self.record(f"ssh {self.a.alias}", False, f"works only with {tilde(self.key)}: add "
                               f"'IdentityFile {tilde(self.key)}' to your own 'Host {self.a.alias}' entry in "
                               "~/.ssh/config (the teleop node uses that entry as it is)")
        return self.record(f"ssh {self.a.alias}", True, f"{self.quest_ip}:{SSH_PORT}")

    def add_identity_to_own_entry(self) -> bool:
        """The owner's own Host entry does not offer this wizard's key: offer to add one IdentityFile line to it."""
        line = f"IdentityFile {tilde(self.key)}"
        text = self.ssh_config.read_text() if self.ssh_config.exists() else ""
        lines = text.splitlines(keepends=True)
        at = next((i for i, l in enumerate(lines) if l.split()[:2] == ["Host", self.a.alias]), None)
        if self.check_only or at is None:
            return False
        self.say(f"   Your own 'Host {self.a.alias}' entry in ~/.ssh/config does not offer {tilde(self.key)}, the "
                 f"key authorised on the headset, so a plain `ssh {self.a.alias}` (as the teleop node runs it) is "
                 "refused.")
        if not self.ask(f"Add '{line}' to that entry (the old file is kept as config.before-quest-setup)?", True):
            return False
        nxt = lines[at + 1] if at + 1 < len(lines) else ""
        indent = (nxt[:len(nxt) - len(nxt.lstrip())] if nxt.strip() else "") or "    "
        lines.insert(at + 1, f"{indent}{line}\n")
        backup = self.ssh_config.with_name("config.before-quest-setup")
        if not backup.exists():
            shutil.copy2(self.ssh_config, backup)
        self.ssh_config.write_text("".join(lines))
        return self.ssh_ok(plain=True)[0]

    def forget_host_key(self) -> bool:
        """The headset's host key changed: forget the old one, in whichever known_hosts file the entry uses."""
        files = self.ssh_option("userknownhostsfile").split() or [str(self.known_hosts)]
        kh = Path(files[0]).expanduser()
        ours = kh == self.known_hosts
        self.say("   The headset's SSH host key differs from the one recorded"
                 + (" (expected: the runtime was just reinstalled)" if self.reinstalled else "")
                 + f". The record is in {tilde(kh)}" + (", this wizard's own file." if ours else
                 " (a copy is kept as known_hosts.old). Other machines that knew the headset see the new key too."))
        if self.check_only or not self.ask("Forget the old host key?", ours or self.reinstalled):
            return False
        self.run(["ssh-keygen", "-R", f"[{self.quest_ip}]:{SSH_PORT}", "-f", str(kh)], 10)
        # record the new key as read over USB (a trusted channel), so even a strict entry accepts it
        _, out = self.adb_cmd("shell", "content", "read", "--uri", f"{PROVIDER}/host_public_key", timeout=20)
        key = next((l.split()[:2] for l in out.splitlines() if l.startswith("ssh-")), None)
        if key:
            with open(kh, "a") as f:
                f.write(f"[{self.quest_ip}]:{SSH_PORT} {' '.join(key)}\n")
        return True

    def provision_for_ssh(self) -> None:
        """SSH fails: provision what is missing (keys and address on a fresh runtime, else the address, or ask
        before replacing someone else's keys), then wait for the listener."""
        status = self.runtime_status()
        fresh = not status or "Provision authorized_keys" in status
        listening_here = status.startswith("SSH running") and self.quest_ip in status
        if fresh:
            self.say("   provisioning the runtime: this computer's key, the headset's Tailscale address")
            self.provision(keys_too=True)
        elif not listening_here:
            # keys are there; the runtime listens elsewhere (an old address) or waits for one
            self.say(f"   the runtime is not listening on {self.quest_ip} ({status.splitlines()[0][:80]}): "
                     "setting its address")
            self.provision(keys_too=False)
        elif not self.port_open():
            raise Abort(f"the runtime listens on {self.quest_ip}:{SSH_PORT}, but this computer cannot reach it: "
                        "are both devices Connected in the same tailnet, and do the tailnet's access rules allow "
                        f"this computer to reach the headset? (`tailscale ping {self.quest_ip}` tells)")
        else:
            self.say("   The headset answers but refuses this computer's key: its runtime already has another "
                     "set of authorised keys. Replacing the set locks out machines that use this headset now, "
                     "unless their public keys are passed with --extra-keys FILE.")
            if not self.ask("Replace the headset's authorised keys with this computer's key?", False):
                raise Abort("not provisioned. Ask the headset's owner to add this public key to the headset's "
                            f"~/.ssh/authorized_keys: {self.key.with_suffix('.pub').read_text().strip()}")
            self.provision(keys_too=True)
        def up() -> bool:
            ok, out = self.ssh_ok()
            return ok or "REMOTE HOST IDENTIFICATION HAS CHANGED" in out
        if not self.wait_for(f"the runtime's SSH listener on {self.quest_ip}", up, 30, 2.0):
            ok, out = self.ssh_ok()
            raise Abort(f"ssh {self.a.alias} still fails ({out[-200:]}); the runtime says: "
                        f"{self.runtime_status()[-300:]}")
        _, host_key = self.adb_cmd("shell", "content", "read", "--uri", f"{PROVIDER}/host_public_key", timeout=20)
        shown = next((l.strip() for l in host_key.splitlines() if l.startswith("Fingerprint")), host_key.strip())
        self.say(f"   the headset's SSH host key: {shown}")
        ok, out = self.ssh_ok()
        if not ok and "REMOTE HOST IDENTIFICATION HAS CHANGED" in out:
            if not self.forget_host_key():
                raise Abort("the old host key is still recorded; rerun and answer yes to forget it")
            ok, out = self.ssh_ok()
        if not ok:
            raise Abort(f"ssh {self.a.alias} still fails: {out[-200:]}")

    # 6. quest_env
    def quest_env_source(self) -> Path:
        if self.a.quest_env:
            src = Path(self.a.quest_env).expanduser()
            if not (src / "quest").is_file():
                raise Abort(f"--quest-env {src} is not a quest_env checkout (no 'quest' script)")
            return src
        dest = self.cache / f"quest_env-{QUEST_ENV_REF}"
        if (dest / "quest").is_file():
            return dest
        archive = self.download(QUEST_ENV_URL, self.cache / f"quest_env-{QUEST_ENV_REF}.tar.gz")
        tmp = self.cache / f"quest_env-{QUEST_ENV_REF}.extract"
        shutil.rmtree(tmp, ignore_errors=True)
        with tarfile.open(archive) as t:
            try:
                t.extractall(tmp, filter="data")
            except TypeError:          # Python without extraction filters (before 3.10.12 / 3.11.4)
                t.extractall(tmp)
        top = next(tmp.iterdir())
        shutil.rmtree(dest, ignore_errors=True)
        top.rename(dest)
        shutil.rmtree(tmp, ignore_errors=True)
        return dest

    def step_quest_env(self) -> bool:
        """quest_env as a Git clone tracking --quest-env-branch (so `quest pull` and quest_up's update check work),
        with the developer tools (git, curl, wget) it ships. A clone that already tracks a branch is left alone."""
        self.step("6. quest_env (the headset app) on the headset")
        probe = (f'R={HEADSET_ROOT}; [ -f "$R/quest" ] && echo present; [ -x "$HOME/bin/git" ] && echo has-git; '
                 '[ -d "$R/.git" ] && [ -x "$HOME/bin/git" ] && git -C "$R" rev-parse --abbrev-ref @{u} >/dev/null '
                 '2>&1 && echo tracking; true')
        state = set(self.ssh(probe, 30)[1].split())
        if "tracking" in state:
            self.say("   the headset has a Git clone of quest_env tracking a branch: left as it is (quest_up pulls it)")
        elif self.check_only:
            return self.record("quest_env on the headset", False,
                               "missing" if "present" not in state else "not a tracking Git clone")
        else:
            if "has-git" not in state or "present" not in state:
                src = self.quest_env_source()
                self.say(f"   copying quest_env ({'from ' + str(src) if self.a.quest_env else QUEST_ENV_REF}) to the "
                         "headset's ~/workspace/quest_env")
                rc, out = self.ssh(f'R={HEADSET_ROOT}; rm -rf "$R.new" && mkdir -p "$R.new" && tar -xf - -C "$R.new" '
                                   f'&& rm -rf "$R" && mv "$R.new" "$R" && echo copied', 300, tar_tree(src))
                if rc != 0 or "copied" not in out:
                    raise Abort(f"copying quest_env failed: {out.strip()[-300:]}")
            if "has-git" not in state:
                self.say("   installing the developer tools (git, curl, wget) from quest_env's package")
                rc, out = self.ssh(f'R={HEADSET_ROOT}; sh "$R/quest" install-devtools && "$HOME/bin/git" --version',
                                   300)
                if rc != 0 or "git version" not in out:
                    raise Abort(f"installing the developer tools failed: {out.strip()[-300:]}")
            branch = self.a.quest_env_branch
            self.say(f"   making it a Git clone of {QUEST_ENV_GIT} tracking '{branch}'")
            rc, out = self.ssh(
                f'R={HEADSET_ROOT}; PATH="$HOME/bin:$PATH"; cd "$R" && rm -f {REF_FILE} && {{ [ -d .git ] || git init -q; }} && '
                f'{{ git remote remove origin 2>/dev/null; true; }} && git remote add origin {QUEST_ENV_GIT} && '
                f'git fetch -q origin {branch} && git checkout -q -f -B {branch} origin/{branch} && git clean -fdq && '
                f'git log --oneline -1', 300)
            if rc != 0:
                raise Abort(f"cloning quest_env failed (does the headset reach github.com?): {out.strip()[-300:]}")
            self.say(f"   at {out.strip().splitlines()[-1][:80] if out.strip() else '?'}")
        profile = "" if self.check_only else 'sh "$R/shell/install-profile.sh" >/dev/null && '
        rc, out = self.ssh(f'R={HEADSET_ROOT}; {profile}sh "$R/quest" verify', 120)
        if rc != 0:
            raise Abort(f"quest_env on the headset does not verify: {out.strip()[-300:]}")
        return self.record("quest_env on the headset", True, "verified")

    # 7. telemetry
    def step_telemetry(self) -> bool:
        self.step("7. Telemetry check (optional)")
        if self.check_only or self.a.skip_telemetry:
            return True
        if not any(n == f"ssh {self.a.alias}" and ok for n, ok, _ in self.results):
            self.say(f"   skipped: needs a working `ssh {self.a.alias}` (step 5)")
            return True
        self.say("   This starts the headset app once and reads its stream. Wake the headset (press the power button "
                 "once; it need not be worn, but the stream only flows while the XR session runs).")
        if not self.ask("Run the check now?", True):
            return True
        ctl = QuestCtl(SshRunner(self.a.alias), self.quest_ip, log=lambda s: self.say("   " + s))
        rc = ctl.up("body-frozen", wait=45)
        ok = rc == 0
        if ok and self.ask("Stop the telemetry again (saves the battery)?", True):
            ctl.down()
        return self.record("telemetry stream", ok, "live" if ok else f"quest_up exit {rc}: see above; rerun when "
                                                                     "the headset is awake and worn")

    def run_all(self) -> int:
        if os.name == "nt":
            self.say("Windows is not supported yet (macOS or Linux).")
            return 2
        self.say("Quest setup: " + ("check only, nothing changes" if self.check_only else
                                    "each step checks first and only fixes what is missing; rerun at any time"))
        steps = [self.step_adb, self.step_local_tailscale, self.step_headset_tailscale, self.step_runtime,
                 self.step_ssh, self.step_quest_env, self.step_telemetry]
        try:
            for s in steps:
                # check only: a later step cannot be judged without its predecessors (no adb, no ssh)
                if not s() and self.check_only and s in (self.step_adb, self.step_headset_tailscale, self.step_ssh):
                    break
        except Abort as e:
            self.say(f"\nSTOPPED: {e}\nFix that and run the wizard again; finished steps are skipped.")
            return 1
        except KeyboardInterrupt:
            self.say("\nStopped. Run the wizard again to continue.")
            return 130
        bad = [r for r in self.results if not r[1]]
        self.say("\n== Summary")
        for name, ok, detail in self.results:
            self.say(f"   [{'x' if ok else ' '}] {name}" + (f": {detail}" if detail else ""))
        if not bad and not self.check_only:
            self.say(f"""
Done: the headset is reachable as `ssh {self.a.alias}` over Tailscale; the cable can go.
Other machines that reach the headset (pass their public keys with --extra-keys) record its host key with:
  {self.known_hosts_line()}
Next (docs/quest_setup.md, "Try it in the simulator"):
  python -m robot_utils.nodes.sim.stack --no-pull --host 0.0.0.0
  open http://localhost:8080/ -> Start the quest_teleop tile -> its page: Quest up, then Start
  put the headset on (neck-worn works too), press Y to arm, hold the right grip to move the arm""")
        return 1 if bad else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="only report what is set up; change nothing")
    ap.add_argument("--yes", action="store_true", help="accept every default answer (non-interactive)")
    ap.add_argument("--serial", help="the headset's adb serial when several devices are connected")
    ap.add_argument("--alias", default="quest", help="ssh host name for the headset (default %(default)s; the "
                                                     "teleop node's --ssh-host)")
    ap.add_argument("--key", default="~/.ssh/quest_ed25519", help="SSH key for the headset (created if missing)")
    ap.add_argument("--extra-keys", action="append", default=[], metavar="FILE",
                    help="public keys of other machines to authorise on the headset (repeatable); added if missing "
                         "on every run, never removed")
    ap.add_argument("--apk", help="a local runtime APK instead of the pinned download")
    ap.add_argument("--reinstall-runtime", action="store_true", help="install the runtime even if it is current")
    ap.add_argument("--quest-env", help="copy this local quest_env checkout instead of the pinned version")
    ap.add_argument("--quest-env-branch", default="collab",
                    help="the quest_env branch the headset's clone tracks (default %(default)s)")
    ap.add_argument("--skip-telemetry", action="store_true", help="skip the final telemetry check")
    ap.add_argument("--cache", default="~/.cache/robot_utils/quest", help="downloads (default %(default)s)")
    ap.add_argument("--backup-dir", default="~/quest-backups",
                    help="where a runtime migration backs up the headset's data (default %(default)s)")
    return Wizard(ap.parse_args(argv)).run_all()


if __name__ == "__main__":
    sys.exit(main())
