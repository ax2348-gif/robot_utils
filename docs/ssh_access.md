# SSH access from a config file

Password login on the robot is off, so everyone logs in with a key. `robot_utils.admin.ssh_access`
manages who has access from a small config file on the robot. The config is never in the repo.

```bash
cd ~/robot_utils
.venv/bin/python -m robot_utils.admin.ssh_access init              # template at ~/.config/robot_utils/ssh_access.yaml
nano ~/.config/robot_utils/ssh_access.yaml                          # list GitHub usernames / extra keys
.venv/bin/python -m robot_utils.admin.ssh_access sync --dry-run    # who gains / loses which fingerprints
.venv/bin/python -m robot_utils.admin.ssh_access sync              # apply
.venv/bin/python -m robot_utils.admin.ssh_access status            # what is authorized, managed vs not
```

```yaml
github: [alice, bob]            # every key on https://github.com/<user>.keys
keys:
  - ssh-ed25519 AAAA... carol-laptop
```

- **Revoke** someone by removing them from the config, then syncing.
- **Re-sync** to pick up keys people added to GitHub. It also trusts whoever controls those
  accounts.

## How it works

`~/.ssh/authorized_keys` gets one managed block:

```
# BEGIN robot_utils ssh_access (generated: edit ~/.config/robot_utils/ssh_access.yaml, then sync)
ssh-ed25519 AAAA... gh:alice
# END robot_utils ssh_access
```

Everything outside the block is the administrator's and is never touched; your own keys belong
there.

## Lockout protection

- **Only the block is rewritten.** Lines outside it stay byte for byte.
- **A fallback key is required:** the sync refuses unless at least one valid key remains outside
  the block.
- **It fails closed.** Any of these aborts with nothing written:
  - a GitHub fetch error (network down, unknown user);
  - an account with no keys (its access is not silently dropped: remove the person deliberately);
  - an invalid key, or a key line with options (`command=` and the like);
  - an invalid config;
  - broken or duplicated block markers.
- **Writes are safe:** atomic (temporary file, fsync, rename, mode 600), with a timestamped backup
  (`authorized_keys.bak-*`, the last 10 kept).
- **The result is verified:** the file is re-read, and every key is checked with `ssh-keygen`. If
  anything is off, the backup is restored.
- **Guards:** a lock against concurrent syncs, refusing a symlinked `authorized_keys` or `~/.ssh`,
  and enforcing 0700 on `~/.ssh`.
- **Audit:** each applied change goes to `~/.local/state/robot_utils/ssh_access.log` (JSON lines:
  added and removed fingerprints, the backup).

If a sync ever goes wrong anyway:
- **Restore a backup:** `cp ~/.ssh/authorized_keys.bak-<time> ~/.ssh/authorized_keys`.
- **Otherwise:** your unmanaged key still works, and failing that, the robot's console (monitor and
  keyboard).

Keep a session open while trying a new setup.
