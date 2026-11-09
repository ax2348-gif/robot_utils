"""ssh_access: the managed authorized_keys block. Real keys (ssh-keygen), fake GitHub, every refusal path.

The invariant under test: whatever happens, lines outside the managed block are byte-identical,
at least one of them is a valid key, and a refused sync leaves the file untouched."""
import os
import stat
import subprocess
from pathlib import Path

import pytest

from robot_utils.admin import ssh_access as S


def make_key(tmp: Path, name: str) -> str:
    p = tmp / name
    subprocess.run(["ssh-keygen", "-q", "-t", "ed25519", "-N", "", "-C", name, "-f", str(p)], check=True)
    return (tmp / (name + ".pub")).read_text().strip()


@pytest.fixture
def env(tmp_path):
    keys = {n: make_key(tmp_path, n) for n in ("admin", "admin2", "alice1", "alice2", "bob1", "carol1")}
    ssh = tmp_path / "ssh"
    ssh.mkdir(mode=0o755)
    ak = ssh / "authorized_keys"
    ak.write_text(f"{keys['admin']}\n# a comment the admin wrote\n{keys['admin2']}\n")
    os.chmod(ak, 0o600)
    gh = {"alice": [keys["alice1"].rsplit(" ", 1)[0], keys["alice2"].rsplit(" ", 1)[0]],
          "bob": [keys["bob1"].rsplit(" ", 1)[0]], "empty": []}

    def fetch(user):
        if user == "offline":
            raise S.AccessError("github.com/offline.keys: network unreachable")
        return gh[user]

    cfg = tmp_path / "ssh_access.yaml"
    return {"keys": keys, "ak": ak, "fetch": fetch, "cfg": cfg, "tmp": tmp_path, "log": tmp_path / "log.jsonl"}


def sync(env, cfg_dict, dry=False):
    p = S.plan(env["ak"], cfg_dict, env["fetch"])
    if not dry:
        S.apply(env["ak"], p, env["log"])
    return p


def outside(text):
    pr = S.parse(text)
    return "".join(pr.before + pr.after)


def test_first_sync_adds_a_block_and_keeps_everything_else_byte_identical(env):
    original = env["ak"].read_text()
    p = sync(env, {"github": ["alice", "bob"], "keys": [env["keys"]["carol1"]]})
    text = env["ak"].read_text()
    assert outside(text) == original                                   # untouched, byte for byte
    assert S.BEGIN in text and S.END in text
    assert {k.owner for k in p.added} == {"gh:alice", "gh:bob", "config"} and len(p.added) == 4
    assert "gh:alice" in text and "carol1" in text
    assert stat.S_IMODE(os.stat(env["ak"]).st_mode) == 0o600
    assert stat.S_IMODE(os.stat(env["ak"].parent).st_mode) == 0o700   # ~/.ssh tightened
    assert list(env["ak"].parent.glob("authorized_keys.bak-*"))       # backup kept
    assert env["log"].read_text().count("\n") == 1
    # ssh-keygen accepts the whole file: 2 admin + 4 managed keys
    out = subprocess.run(["ssh-keygen", "-lf", str(env["ak"])], capture_output=True, text=True).stdout
    assert out.count("SHA256:") == 6


def test_resync_is_a_no_op_and_removal_only_touches_the_block(env):
    sync(env, {"github": ["alice", "bob"], "keys": []})
    before_outside = outside(env["ak"].read_text())
    again = sync(env, {"github": ["alice", "bob"], "keys": []})
    assert not again.changed and not again.added and not again.removed
    p = sync(env, {"github": ["alice"], "keys": []})                    # bob removed from the config
    assert len(p.removed) == 1 and "gh:bob" not in env["ak"].read_text()
    assert outside(env["ak"].read_text()) == before_outside
    p = sync(env, {"github": [], "keys": []})                           # everyone removed: block empties
    assert len(p.removed) == 2 and outside(env["ak"].read_text()) == before_outside


def test_dry_run_writes_nothing(env):
    original = env["ak"].read_text()
    p = sync(env, {"github": ["alice"], "keys": []}, dry=True)
    assert p.changed and env["ak"].read_text() == original
    assert not list(env["ak"].parent.glob("authorized_keys.bak-*"))


@pytest.mark.parametrize("cfg,msg", [
    ({"github": ["alice", "offline"], "keys": []}, "network"),          # any fetch failure
    ({"github": ["empty"], "keys": []}, "no keys"),                     # account with no keys
    ({"github": [], "keys": ["ssh-ed25519 AAAAnotakey x"]}, "invalid"), # bad key
    ({"github": [], "keys": ['command="rm -rf /" ssh-ed25519 AAAA x']}, "not a supported"),  # options injection
])
def test_fail_closed_leaves_the_file_untouched(env, cfg, msg):
    sync(env, {"github": ["bob"], "keys": []})                          # an existing block
    snapshot = env["ak"].read_text()
    with pytest.raises(S.AccessError, match=msg):
        sync(env, cfg)
    assert env["ak"].read_text() == snapshot


def test_refuses_without_a_fallback_key_outside_the_block(env):
    env["ak"].write_text("# only comments here\n")
    with pytest.raises(S.AccessError, match="no valid key outside"):
        sync(env, {"github": ["alice"], "keys": []})
    assert env["ak"].read_text() == "# only comments here\n"
    # an invalid line outside does not count as a fallback either
    env["ak"].write_text("ssh-ed25519 AAAAbroken nobody\n")
    with pytest.raises(S.AccessError, match="no valid key outside"):
        sync(env, {"github": ["alice"], "keys": []})


def test_broken_or_duplicated_markers_are_refused(env):
    base = env["ak"].read_text()
    for bad in (base + S.BEGIN + "\n", base + S.END + "\n" + S.BEGIN + "\n",
                base + S.BEGIN + "\n" + S.END + "\n" + S.BEGIN + "\n" + S.END + "\n"):
        env["ak"].write_text(bad)
        with pytest.raises(S.AccessError, match="markers"):
            sync(env, {"github": ["alice"], "keys": []})
        assert env["ak"].read_text() == bad


def test_symlinked_authorized_keys_is_refused(env):
    real = env["tmp"] / "elsewhere"
    real.write_text(env["ak"].read_text())
    env["ak"].unlink()
    env["ak"].symlink_to(real)
    with pytest.raises(S.AccessError, match="symlink"):
        sync(env, {"github": ["alice"], "keys": []})


def test_keys_already_outside_the_block_are_not_duplicated(env):
    p = sync(env, {"github": [], "keys": [env["keys"]["admin"]]})       # the admin key, also in the config
    assert [k.owner for k in p.skipped_duplicates] == ["config"] and not p.added
    assert env["ak"].read_text().count(env["keys"]["admin"].split()[1]) == 1


def test_verification_failure_restores_the_backup(env, monkeypatch):
    original = env["ak"].read_text()
    p = S.plan(env["ak"], {"github": ["alice"], "keys": []}, env["fetch"])
    real = S.valid_fingerprints
    calls = {"n": 0}

    def flaky(lines):                                                    # fail only during post-write verification
        calls["n"] += 1
        return [] if calls["n"] == 1 else real(lines)
    monkeypatch.setattr(S, "valid_fingerprints", flaky)
    with pytest.raises(S.AccessError, match="verification"):
        S.apply(env["ak"], p, env["log"])
    assert env["ak"].read_text() == original


def test_config_validation(env):
    for text, msg in (("github: [alice, alice]\n", "twice"), ("github: ['bad user!']\n", "valid GitHub"),
                      ("users: [alice]\n", "unknown"), ("github: alice\n", "list"), ("- a\n- b\n", "mapping")):
        env["cfg"].write_text(text)
        with pytest.raises(S.AccessError, match=msg):
            S.load_config(env["cfg"])
    env["cfg"].write_text(S.TEMPLATE)
    assert S.load_config(env["cfg"]) == {"github": [], "keys": []}


def test_cli_init_never_overwrites_and_lock_blocks_concurrent_runs(env, capsys):
    assert S.main(["init", "--config", str(env["cfg"])]) == 0
    env["cfg"].write_text("github: []\nkeys: []\n# mine\n")
    assert S.main(["init", "--config", str(env["cfg"])]) == 0
    assert "# mine" in env["cfg"].read_text()
    with S.Lock(env["ak"]):
        with pytest.raises(S.AccessError, match="another"):
            with S.Lock(env["ak"]):
                pass
    # sync through the CLI with an empty config: a no-op that creates an empty block
    assert S.main(["sync", "--config", str(env["cfg"]), "--authorized-keys", str(env["ak"]),
                   "--log", str(env["log"])]) == 0
    assert S.main(["status", "--authorized-keys", str(env["ak"])]) == 0
    # a refused sync exits 2
    env["cfg"].write_text("github: [nonexistent-user-zz]\n")
    snapshot = env["ak"].read_text()
    calls = []

    def no_network(user, timeout=10.0):
        calls.append(user)
        raise S.AccessError("HTTP 404 (no such user?)")
    orig = S.fetch_github
    S.fetch_github = no_network
    try:
        assert S.main(["sync", "--config", str(env["cfg"]), "--authorized-keys", str(env["ak"])]) == 2
    finally:
        S.fetch_github = orig
    assert calls == ["nonexistent-user-zz"] and env["ak"].read_text() == snapshot
