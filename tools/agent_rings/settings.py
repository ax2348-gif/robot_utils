"""Write a rings agent's settings (tools/agent_rings/README.md): a file in the harness directory, passed to the CLI
with --settings (flag settings: no workspace trust needed, and out of the agent's reach), with --setting-sources ''
so no user / project settings mix in.

    python settings.py OUT.json --robot-host ::1 --deny PATH [--deny PATH ...]

dontAsk: anything not allowed here is refused without a prompt. Bash runs in the OS sandbox (seatbelt): no reads of
the --deny paths, writes only in the workspace, network only through the sandbox's proxy to the robot host and PyPI.
The Read / Edit tools get matching deny rules (the sandbox covers Bash and its children only).
"""
import argparse
import json
import os

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
p.add_argument("out")
p.add_argument("--robot-host", action="append", default=[], help="host(s) the agent may reach besides PyPI")
p.add_argument("--deny", action="append", default=[], help="a path the agent may not read")
a = p.parse_args()

rules = []
for d in a.deny:
    rules += [f"Read(/{d}/**)", f"Edit(/{d}/**)"]      # //abs: an absolute path; Edit covers every writing tool
settings = {
    "permissions": {
        "defaultMode": "dontAsk",
        "allow": ["Read", "Edit", "Write", "Glob", "Grep", "NotebookEdit", "TodoWrite", "Bash"],
        "deny": rules + ["WebFetch", "WebSearch", "Bash(ssh *)", "Bash(scp *)", "Bash(rsync *)", "Bash(sudo *)",
                         "Bash(git push *)", "Edit(.claude/**)", "Edit(//**/.claude/**)"],
        "denyInsteadOfPrompting": True,
    },
    "sandbox": {
        "enabled": True,
        "failIfUnavailable": True,
        "autoAllowBashIfSandboxed": True,
        "allowUnsandboxedCommands": False,
        "filesystem": {"denyRead": a.deny},
        "network": {"allowedDomains": a.robot_host + ["pypi.org", "files.pythonhosted.org"]},
    },
    "enableAllProjectMcpServers": False,
}
with open(a.out, "w") as f:
    json.dump(settings, f, indent=2)
