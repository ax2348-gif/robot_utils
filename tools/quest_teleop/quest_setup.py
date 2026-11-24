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
RUNTIME_URL = "https://raw.githubusercontent.com/ax2348-gif/quest_dev/collab-v1/artifacts/quest-dev-runtime.apk"
RUNTIME_SHA256 = "683d488a60221dd265ad009b89a34b86d312b7e85d01e5e9dd2bb114f06ba4f0"
QUEST_ENV_REF = "collab-v1"
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


def ssh_block(alias: str, host: str, key: Path, known_hosts: Path) -> str:
    home = str(Path.home())
    tilde = lambda p: "~" + str(p)[len(home):] if str(p).startswith(home) else str(p)   # noqa: E731
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
                if name == ".DS_Store" or name.startswith("._"):
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

    def ssh(self, command: str, timeout: float = 30, data: Optional[bytes] = None) -> tuple[int, str]:
        p = self.run(["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", self.a.alias, command],
                     timeout, data)
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

    def install_apk(self, apk: Path, package: str, what: str) -> None:
        rc, out = self.adb_cmd("install", "-r", str(apk), timeout=300)
        if rc == 0 and "Success" in out:
            return
        if "INSTALL_FAILED_UPDATE_INCOMPATIBLE" in out:
            self.say(f"   The {what} on the headset was signed by another build, so it cannot be updated in place. "
                     "Uninstalling it erases its data (for the runtime: its SSH host key, authorised keys and "
                     "workspace; they are set up again by this wizard).")
            if not self.ask(f"Uninstall the {what} and install this one?", False):
                raise Abort(f"left the existing {what} in place")
            self.adb_cmd("uninstall", package, timeout=60)
            rc, out = self.adb_cmd("install", str(apk), timeout=300)
            if rc == 0 and "Success" in out:
                return
        raise Abort(f"installing the {what} failed: {out.strip()[-300:]}")

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
        self.install_apk(apk, RUNTIME_PACKAGE, "runtime")
        return self.record("runtime installed", True, f"version {self.runtime_version()}")

    # 5. key, ssh config, provisioning
    def runtime_status(self) -> str:
        _, out = self.adb_cmd("shell", "content", "read", "--uri", f"{PROVIDER}/status", timeout=20)
        return out.strip()

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

    def ssh_ok(self) -> tuple[bool, str]:
        rc, out = self.ssh("echo quest-ok", 20)
        return rc == 0 and "quest-ok" in out, out.strip()

    def write_runtime(self, name: str, data: bytes) -> None:
        rc, out = self.adb_cmd("shell", "content", "write", "--uri", f"{PROVIDER}/{name}", data=data, timeout=30)
        if rc != 0 or "rror" in out:
            raise Abort(f"writing {name} to the runtime failed: {out.strip()[-300:]}")

    def provision(self, keys_too: bool) -> None:
        """Write the listen address (always: it is the headset's own Tailscale address) and, with ``keys_too``, the
        authorised keys; then (re)start the SSH service."""
        if keys_too:
            keys = self.key.with_suffix(".pub").read_bytes()
            if self.a.extra_keys:
                keys += b"\n" + Path(self.a.extra_keys).expanduser().read_bytes()
            if b"PRIVATE KEY" in keys:
                raise Abort("--extra-keys must hold public keys only")
            self.write_runtime("authorized_keys", keys.strip() + b"\n")
        self.write_runtime("tailnet_ip", self.quest_ip.encode() + b"\n")
        self.adb_cmd("shell", "content", "call", "--uri", PROVIDER, "--method", "start", timeout=30)

    def port_open(self) -> bool:
        try:
            socket.create_connection((self.quest_ip, SSH_PORT), timeout=5).close()
            return True
        except OSError:
            return False

    def step_ssh(self) -> bool:
        self.step("5. SSH to the headset over Tailscale")
        self.ensure_key()
        self.write_ssh_config()
        ok, out = self.ssh_ok()
        if not ok and "REMOTE HOST IDENTIFICATION HAS CHANGED" in out:
            self.say("   The headset's SSH host key differs from the one recorded (expected after reinstalling the "
                     f"runtime). The record is in {self.known_hosts}, this wizard's own file.")
            if self.check_only or not self.ask("Forget the old host key?", True):
                return self.record("ssh quest", False, "host key changed")
            self.run(["ssh-keygen", "-R", f"[{self.quest_ip}]:{SSH_PORT}", "-f", str(self.known_hosts)], 10)
            ok, out = self.ssh_ok()
        if not ok:
            if self.check_only:
                return self.record(f"ssh {self.a.alias}", False, out[-200:])
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
            if not self.wait_for(f"the runtime's SSH listener on {self.quest_ip}",
                                 lambda: self.ssh_ok()[0], 30, 2.0):
                ok, out = self.ssh_ok()
                raise Abort(f"ssh {self.a.alias} still fails ({out[-200:]}); the runtime says: "
                            f"{self.runtime_status()[-300:]}")
            _, host_key = self.adb_cmd("shell", "content", "read", "--uri", f"{PROVIDER}/host_public_key", timeout=20)
            self.say(f"   the headset's SSH host key: {host_key.strip()}")
        return self.record(f"ssh {self.a.alias}", True, f"{self.quest_ip}:{SSH_PORT}")

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
        self.step("6. quest_env (the headset app) on the headset")
        rc, out = self.ssh(f'R={HEADSET_ROOT}; [ -d "$R/.git" ] && echo is-git; [ -f "$R/quest" ] && echo present; '
                           f'cat "$R/{REF_FILE}" 2>/dev/null; true')
        lines = out.split()
        want = self.a.quest_env and "local" or QUEST_ENV_REF
        if "is-git" in lines:
            self.say("   the headset has its own Git clone of quest_env: left as it is (quest_up pulls it)")
        elif "present" in lines and want in lines and not self.a.quest_env:
            self.say(f"   quest_env {QUEST_ENV_REF} is already there")
        else:
            if self.check_only:
                return self.record("quest_env on the headset", False, "missing or another version")
            src = self.quest_env_source()
            self.say(f"   copying quest_env ({'from ' + str(src) if self.a.quest_env else QUEST_ENV_REF}) to the "
                     "headset's ~/workspace/quest_env")
            rc, out = self.ssh(f'R={HEADSET_ROOT}; rm -rf "$R.new" && mkdir -p "$R.new" && tar -xf - -C "$R.new" && '
                               f'rm -rf "$R" && mv "$R.new" "$R" && echo {want} > "$R/{REF_FILE}" && echo copied',
                               300, tar_tree(src))
            if rc != 0 or "copied" not in out:
                raise Abort(f"copying quest_env failed: {out.strip()[-300:]}")
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
    ap.add_argument("--extra-keys", help="a file of further public keys to authorise (other machines)")
    ap.add_argument("--apk", help="a local runtime APK instead of the pinned download")
    ap.add_argument("--reinstall-runtime", action="store_true", help="install the runtime even if it is current")
    ap.add_argument("--quest-env", help="copy this local quest_env checkout instead of the pinned version")
    ap.add_argument("--skip-telemetry", action="store_true", help="skip the final telemetry check")
    ap.add_argument("--cache", default="~/.cache/robot_utils/quest", help="downloads (default %(default)s)")
    return Wizard(ap.parse_args(argv)).run_all()


if __name__ == "__main__":
    sys.exit(main())
