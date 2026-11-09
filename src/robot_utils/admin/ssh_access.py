"""SSH access from a config file: GitHub accounts and extra keys -> a managed block of authorized_keys.

    python -m robot_utils.admin.ssh_access init              # write a commented config template (never overwrites)
    python -m robot_utils.admin.ssh_access sync --dry-run    # show who would gain / lose which keys
    python -m robot_utils.admin.ssh_access sync              # apply
    python -m robot_utils.admin.ssh_access status            # list keys: managed (by person) and unmanaged

Config (on the machine, not in the repo): ``~/.config/robot_utils/ssh_access.yaml``::

    github: [alice, bob]          # every key on github.com/<user>.keys
    keys:                         # extra keys (full public-key lines, with a comment)
      - ssh-ed25519 AAAA... lab-desktop

Safety (passwords are off, so a bad write could lock everyone out):
- Only the block between the BEGIN / END markers is ever rewritten; every other line of
  authorized_keys is kept byte for byte. Those unmanaged keys are the administrator's fallback.
- Refuses to write unless at least one valid key remains OUTSIDE the managed block.
- Fails closed: any GitHub fetch error, an account with no keys, an invalid config, an invalid
  key, or broken / duplicated markers abort the sync and leave the file untouched.
- Writes atomically (temp file + fsync + rename, mode 600), keeps a timestamped backup (last 10),
  then re-reads the result and validates every key with ssh-keygen; on any failure it restores
  the backup.
- Holds a lock against concurrent runs; refuses a symlinked authorized_keys or ~/.ssh; enforces
  0700 on ~/.ssh.
- Every applied change is appended to ~/.local/state/robot_utils/ssh_access.log (JSON lines).
"""
from __future__ import annotations

import argparse
import fcntl
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import yaml

BEGIN = "# BEGIN robot_utils ssh_access (generated: edit ~/.config/robot_utils/ssh_access.yaml, then sync)"
END = "# END robot_utils ssh_access"
BEGIN_PREFIX = "# BEGIN robot_utils ssh_access"
END_PREFIX = "# END robot_utils ssh_access"
DEFAULT_CONFIG = Path("~/.config/robot_utils/ssh_access.yaml").expanduser()
DEFAULT_KEYS = Path("~/.ssh/authorized_keys").expanduser()
DEFAULT_LOG = Path("~/.local/state/robot_utils/ssh_access.log").expanduser()
KEEP_BACKUPS = 10
GITHUB_USER = re.compile(r"^[A-Za-z0-9](?:[A-Za-z0-9-]{0,37}[A-Za-z0-9])?$")
KEY_TYPES = ("ssh-ed25519", "ssh-rsa", "ecdsa-sha2-nistp256", "ecdsa-sha2-nistp384", "ecdsa-sha2-nistp521",
             "sk-ssh-ed25519@openssh.com", "sk-ecdsa-sha2-nistp256@openssh.com")
TEMPLATE = """\
# Who may SSH in as this user. Apply with:  python -m robot_utils.admin.ssh_access sync --dry-run
#                                      then: python -m robot_utils.admin.ssh_access sync
# Keys outside the managed block of ~/.ssh/authorized_keys (e.g. the administrator's own) are never
# touched and must keep at least one valid key: they are the fallback if this list goes wrong.

github: []        # GitHub usernames; every key on https://github.com/<user>.keys
#  - alice

keys: []          # extra public keys, one full line each, with a comment naming the person / device
#  - ssh-ed25519 AAAAC3Nza... bob-laptop
"""


class AccessError(Exception):
    """Refused: nothing was written."""


@dataclass(frozen=True)
class Key:
    line: str          # "type base64 comment"
    fingerprint: str   # SHA256:...
    owner: str         # "gh:alice" or "config"


# ----------------------------------------------------------------------------------------------
# keys
# ----------------------------------------------------------------------------------------------

def fingerprint(line: str) -> str:
    """SHA256 fingerprint of one public-key line via ssh-keygen; raises AccessError if invalid."""
    parts = line.strip().split()
    if len(parts) < 2 or parts[0] not in KEY_TYPES:
        raise AccessError(f"not a supported public key line: {line[:60]!r}")
    with tempfile.NamedTemporaryFile("w", suffix=".pub", delete=False) as f:
        f.write(line.strip() + "\n")
        name = f.name
    try:
        r = subprocess.run(["ssh-keygen", "-lf", name], capture_output=True, text=True, timeout=10)
    finally:
        os.unlink(name)
    if r.returncode != 0:
        raise AccessError(f"invalid public key {line[:60]!r}: {r.stderr.strip() or r.stdout.strip()}")
    m = re.search(r"(SHA256:\S+)", r.stdout)
    if not m:
        raise AccessError(f"ssh-keygen gave no fingerprint for {line[:60]!r}")
    return m.group(1)


def key_identity(line: str) -> tuple[str, str]:
    """(type, base64) of a key line, ignoring options-free comments: used for de-duplication."""
    p = line.split()
    return (p[0], p[1]) if len(p) >= 2 else ("", line)


def fetch_github(user: str, timeout: float = 30.0, attempts: int = 2) -> list[str]:
    """Keys from github.com/<user>.keys. Retries once on timeouts / 5xx; any final failure raises
    (the sync then aborts with nothing written)."""
    url = f"https://github.com/{user}.keys"
    req = urllib.request.Request(url, headers={"User-Agent": "robot_utils-ssh_access"})
    last = None
    for i in range(attempts):
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                if r.status != 200:
                    raise AccessError(f"{url}: HTTP {r.status}")
                text = r.read(1_000_000).decode("utf-8", "replace")
            return [ln.strip() for ln in text.splitlines() if ln.strip()]
        except urllib.error.HTTPError as e:
            if e.code == 404:
                raise AccessError(f"{url}: HTTP 404 (no such GitHub user?)") from e
            last = AccessError(f"{url}: HTTP {e.code}")
        except (urllib.error.URLError, OSError) as e:
            last = AccessError(f"{url}: {e}")
        if i + 1 < attempts:
            time.sleep(2)
    raise last


# ----------------------------------------------------------------------------------------------
# config
# ----------------------------------------------------------------------------------------------

def load_config(path: Path) -> dict:
    if not path.exists():
        raise AccessError(f"no config at {path} (create one with: python -m robot_utils.admin.ssh_access init)")
    try:
        cfg = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError as e:
        raise AccessError(f"{path}: invalid YAML: {e}") from e
    if not isinstance(cfg, dict):
        raise AccessError(f"{path}: expected a mapping with 'github' and / or 'keys'")
    unknown = set(cfg) - {"github", "keys"}
    if unknown:
        raise AccessError(f"{path}: unknown entries {sorted(unknown)} (allowed: github, keys)")
    gh = cfg.get("github") or []
    ks = cfg.get("keys") or []
    if not isinstance(gh, list) or not all(isinstance(u, str) and GITHUB_USER.match(u) for u in gh):
        raise AccessError(f"{path}: 'github' must be a list of valid GitHub usernames")
    if len(set(u.lower() for u in gh)) != len(gh):
        raise AccessError(f"{path}: a GitHub username is listed twice")
    if not isinstance(ks, list) or not all(isinstance(k, str) for k in ks):
        raise AccessError(f"{path}: 'keys' must be a list of public-key lines")
    return {"github": gh, "keys": ks}


def desired_keys(cfg: dict, fetch: Optional[Callable[[str], list[str]]] = None) -> list[Key]:
    """Every key the config grants, validated. Any failure aborts (fail closed)."""
    fetch = fetch or fetch_github
    out: list[Key] = []
    for user in cfg["github"]:
        lines = fetch(user)
        if not lines:
            raise AccessError(f"github.com/{user}.keys has no keys: remove {user!r} from the config or ask them "
                              "to add one (refusing to silently drop their access)")
        for ln in lines:
            fp = fingerprint(ln)
            t, b = key_identity(ln)
            out.append(Key(f"{t} {b} gh:{user}", fp, f"gh:{user}"))
    for ln in cfg["keys"]:
        fp = fingerprint(ln)
        out.append(Key(" ".join(ln.split()), fp, "config"))
    return out


# ----------------------------------------------------------------------------------------------
# authorized_keys
# ----------------------------------------------------------------------------------------------

@dataclass
class Parsed:
    before: list[str]      # raw lines (with newlines) before the block
    block: list[str]       # raw lines inside the block (without markers)
    after: list[str]
    has_block: bool


def parse(text: str) -> Parsed:
    lines = text.splitlines(keepends=True)
    begins = [i for i, l in enumerate(lines) if l.strip().startswith(BEGIN_PREFIX)]
    ends = [i for i, l in enumerate(lines) if l.strip().startswith(END_PREFIX)]
    if not begins and not ends:
        return Parsed(lines, [], [], False)
    if len(begins) != 1 or len(ends) != 1 or ends[0] < begins[0]:
        raise AccessError("authorized_keys has broken or duplicated managed-block markers; fix it by hand "
                          f"(BEGIN at lines {[i + 1 for i in begins]}, END at {[i + 1 for i in ends]})")
    b, e = begins[0], ends[0]
    return Parsed(lines[:b], lines[b + 1:e], lines[e + 1:], True)


def key_lines(raw: list[str]) -> list[str]:
    return [l.strip() for l in raw if l.strip() and not l.strip().startswith("#")]


def valid_fingerprints(lines: list[str]) -> list[str]:
    """Fingerprints of the lines that are valid keys (lines with options are passed to ssh-keygen whole)."""
    fps = []
    for ln in lines:
        try:
            fps.append(fingerprint(_strip_options(ln)))
        except AccessError:
            pass
    return fps


def _strip_options(line: str) -> str:
    """authorized_keys lines may start with options (from=..., command=...); find the key part."""
    p = line.split()
    for i, tok in enumerate(p):
        if tok in KEY_TYPES:
            return " ".join(p[i:])
    return line


def render(parsed: Parsed, keys: list[Key]) -> str:
    before = list(parsed.before)
    if before and not before[-1].endswith("\n"):
        before[-1] += "\n"
    block = [BEGIN + "\n"] + [k.line + "\n" for k in keys] + [END + "\n"]
    return "".join(before + block + parsed.after)


def check_paths(keys_path: Path) -> None:
    ssh_dir = keys_path.parent
    if ssh_dir.is_symlink() or (keys_path.exists() and keys_path.is_symlink()):
        raise AccessError(f"{keys_path} or its directory is a symlink: refusing to write through it")
    if not ssh_dir.exists():
        raise AccessError(f"{ssh_dir} does not exist")


@dataclass
class Plan:
    text: str
    changed: bool
    added: list[Key]
    removed: list[tuple[str, str]]       # (fingerprint, owner / comment)
    kept: list[Key]
    unmanaged_fps: list[str]
    skipped_duplicates: list[Key]


def plan(keys_path: Path, cfg: dict, fetch: Optional[Callable[[str], list[str]]] = None) -> Plan:
    check_paths(keys_path)
    text = keys_path.read_text() if keys_path.exists() else ""
    parsed = parse(text)
    want = desired_keys(cfg, fetch)
    unmanaged = key_lines(parsed.before) + key_lines(parsed.after)
    unmanaged_fps = valid_fingerprints(unmanaged)
    if not unmanaged_fps:
        raise AccessError("refusing: no valid key outside the managed block. Keep at least one administrator key "
                          "in authorized_keys outside the block (the fallback if this list goes wrong).")
    # de-duplicate: keys already present outside the block stay there; within the config, first wins
    seen = set(unmanaged_fps)
    final, dups = [], []
    for k in want:
        if k.fingerprint in seen:
            dups.append(k)
            continue
        seen.add(k.fingerprint)
        final.append(k)
    old = key_lines(parsed.block)
    old_fps = {}
    for ln in old:
        try:
            old_fps[fingerprint(_strip_options(ln))] = ln.split()[-1] if len(ln.split()) > 2 else "?"
        except AccessError:
            old_fps["invalid:" + ln[:20]] = "invalid line"
    new_fps = {k.fingerprint for k in final}
    added = [k for k in final if k.fingerprint not in old_fps]
    kept = [k for k in final if k.fingerprint in old_fps]
    removed = [(fp, who) for fp, who in old_fps.items() if fp not in new_fps]
    new_text = render(parsed, final)
    return Plan(new_text, new_text != text, added, removed, kept, unmanaged_fps, dups)


def apply(keys_path: Path, p: Plan, log_path: Path = DEFAULT_LOG, now: Optional[float] = None) -> Optional[Path]:
    """Atomic write + backup + verification (restores the backup on any failure). Returns the backup path."""
    if not p.changed:
        return None
    check_paths(keys_path)
    ssh_dir = keys_path.parent
    os.chmod(ssh_dir, 0o700)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(now))
    backup = None
    if keys_path.exists():
        backup = keys_path.with_name(f"{keys_path.name}.bak-{stamp}")
        shutil.copy2(keys_path, backup)
        os.chmod(backup, 0o600)
    fd, tmp = tempfile.mkstemp(dir=ssh_dir, prefix=".authorized_keys.")
    try:
        with os.fdopen(fd, "w") as f:
            f.write(p.text)
            f.flush()
            os.fsync(f.fileno())
        os.chmod(tmp, 0o600)
        os.replace(tmp, keys_path)
    except Exception:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    # verify what is now on disk; restore on any doubt
    try:
        written = keys_path.read_text()
        if written != p.text:
            raise AccessError("verification: the file on disk differs from what was written")
        parsed = parse(written)
        if not valid_fingerprints(key_lines(parsed.before) + key_lines(parsed.after)):
            raise AccessError("verification: no valid key outside the managed block after writing")
        bad = [ln for ln in key_lines(parsed.block) if not valid_fingerprints([ln])]
        if bad:
            raise AccessError(f"verification: invalid lines in the block: {bad[:2]}")
        if oct(os.stat(keys_path).st_mode & 0o777) != oct(0o600):
            raise AccessError("verification: wrong permissions")
    except Exception:
        if backup is not None:
            shutil.copy2(backup, keys_path)
            os.chmod(keys_path, 0o600)
        raise
    _prune_backups(keys_path)
    _log(log_path, {"t": time.time(), "added": [(k.fingerprint, k.owner) for k in p.added],
                    "removed": p.removed, "kept": len(p.kept), "backup": str(backup) if backup else None})
    return backup


def _prune_backups(keys_path: Path) -> None:
    baks = sorted(keys_path.parent.glob(keys_path.name + ".bak-*"))
    for b in baks[:-KEEP_BACKUPS]:
        b.unlink()


def _log(path: Path, rec: dict) -> None:
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "a") as f:
            f.write(json.dumps(rec) + "\n")
    except OSError as e:
        print(f"warning: could not write the audit log {path}: {e}", file=sys.stderr)


class Lock:
    def __init__(self, keys_path: Path):
        self.path = keys_path.parent / ".ssh_access.lock"

    def __enter__(self):
        self.f = open(self.path, "w")
        try:
            fcntl.flock(self.f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise AccessError("another ssh_access sync is running")
        return self

    def __exit__(self, *exc):
        fcntl.flock(self.f, fcntl.LOCK_UN)
        self.f.close()


# ----------------------------------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------------------------------

def _print_plan(p: Plan) -> None:
    by = {}
    for k in p.added:
        by.setdefault(k.owner, {"+": [], "=": []})["+"].append(k.fingerprint)
    for k in p.kept:
        by.setdefault(k.owner, {"+": [], "=": []})["="].append(k.fingerprint)
    print(f"unmanaged keys kept (fallback): {len(p.unmanaged_fps)}")
    for fp in p.unmanaged_fps:
        print(f"    {fp}")
    for owner, d in sorted(by.items()):
        print(f"{owner}: +{len(d['+'])} new, {len(d['='])} unchanged")
        for fp in d["+"]:
            print(f"    + {fp}")
    for fp, who in p.removed:
        print(f"  - {fp}  ({who})  REMOVED")
    for k in p.skipped_duplicates:
        print(f"  = {k.fingerprint} ({k.owner}) already present outside the block: left there")
    if not p.changed:
        print("no change")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["init", "sync", "status"])
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("--authorized-keys", type=Path, default=DEFAULT_KEYS)
    ap.add_argument("--log", type=Path, default=DEFAULT_LOG)
    ap.add_argument("--dry-run", action="store_true", help="sync: show the changes, write nothing")
    a = ap.parse_args(argv)
    try:
        if a.command == "init":
            if a.config.exists():
                print(f"{a.config} exists; not overwriting")
                return 0
            a.config.parent.mkdir(parents=True, exist_ok=True)
            a.config.write_text(TEMPLATE)
            os.chmod(a.config, 0o600)
            print(f"wrote {a.config}")
            return 0
        if a.command == "status":
            check_paths(a.authorized_keys)
            parsed = parse(a.authorized_keys.read_text() if a.authorized_keys.exists() else "")
            print("unmanaged (never touched by sync):")
            for ln in key_lines(parsed.before) + key_lines(parsed.after):
                fps = valid_fingerprints([ln])
                print(f"    {fps[0] if fps else 'INVALID'}  {ln.split()[-1] if len(ln.split()) > 2 else ''}")
            print("managed block:" if parsed.has_block else "managed block: (none yet)")
            for ln in key_lines(parsed.block):
                fps = valid_fingerprints([ln])
                print(f"    {fps[0] if fps else 'INVALID'}  {ln.split()[-1] if len(ln.split()) > 2 else ''}")
            return 0
        with Lock(a.authorized_keys):
            cfg = load_config(a.config)
            p = plan(a.authorized_keys, cfg)
            _print_plan(p)
            if a.dry_run or not p.changed:
                if a.dry_run and p.changed:
                    print("(dry run: nothing written)")
                return 0
            backup = apply(a.authorized_keys, p, a.log)
            print(f"written {a.authorized_keys}" + (f" (backup {backup})" if backup else ""))
            return 0
    except AccessError as e:
        print(f"refused, nothing written: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
