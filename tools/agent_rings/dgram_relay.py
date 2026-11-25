"""The agent's datagram door to the WBC (tools/agent_rings/README.md): a unix datagram socket the agent's sandbox may
send to, forwarding every datagram unchanged to the WBC's own target socket (in the stack's runtime dir, out of the
agent's reach). The WBC runs with UDP off, so this is its only target input.

    python dgram_relay.py --listen /tmp/ara/wbc.target.sock --to unix:/tmp/ar/robot/wbc.target.sock --log LOG.jsonl

The log has one JSON line per second with datagrams: count, bytes, the type counts and the first datagram of that
second (truncated), enough to audit what was sent without logging 50 Hz streams in full.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import time


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--listen", required=True)
    p.add_argument("--to", required=True, help="unix:/path of the WBC's target socket")
    p.add_argument("--log", required=True)
    a = p.parse_args()
    if os.path.exists(a.listen):
        os.unlink(a.listen)
    rx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    rx.bind(a.listen)
    os.chmod(a.listen, 0o666)
    tx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    to = a.to[5:] if a.to.startswith("unix:") else a.to
    rx.settimeout(1.0)
    log = open(a.log, "a", buffering=1)
    sec, n, size, types, first, errors = None, 0, 0, {}, None, 0

    def flush():
        if n:
            log.write(json.dumps({"t": sec, "n": n, "bytes": size, "types": types, "errors": errors,
                                  "first": first}, separators=(",", ":")) + "\n")

    while True:
        try:
            data = rx.recv(65536)
        except socket.timeout:
            data = None
        now = int(time.time())
        if now != sec:
            flush()
            sec, n, size, types, first, errors = now, 0, 0, {}, None, 0
        if data is None:
            continue
        n += 1
        size += len(data)
        try:
            kind = json.loads(data).get("type", "?")
        except (ValueError, AttributeError):
            kind = "not_json"
        types[kind] = types.get(kind, 0) + 1
        if first is None:
            first = data[:600].decode("utf-8", "replace")
        try:
            tx.sendto(data, to)
        except OSError:
            errors += 1          # the WBC not up (yet): dropped, as UDP would be


if __name__ == "__main__":
    main()
