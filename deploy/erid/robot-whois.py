#!/usr/bin/env python3
"""Tailscale identity for the robot remote-access proxy (erid). nginx asks it once per request (auth_request) with
the client's address in X-Client-IP; it answers 200 with X-TS-User / X-TS-Name / X-TS-Device (percent-encoded UTF-8,
empty when unknown), which nginx forwards to the robot as X-Robot-User / -Name / -Device. Always 200: identity is
informational, never access control (nginx's allow/deny does that). `tailscale whois` results are cached 10 min.

    python3 robot-whois.py [PORT]    # listens on 127.0.0.1 only (default 8098)
"""
import json, subprocess, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import quote

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8098
TTL, FAIL_TTL = 600.0, 60.0
cache: dict = {}
lock = threading.Lock()


def whois(ip: str) -> dict:
    now = time.monotonic()
    with lock:
        hit = cache.get(ip)
    if hit and now < hit[0]:
        return hit[1]
    info, ttl = {}, FAIL_TTL
    try:
        r = subprocess.run(["tailscale", "whois", "--json", ip], capture_output=True, text=True, timeout=3)
        if r.returncode == 0:
            d = json.loads(r.stdout)
            node, prof = d.get("Node") or {}, d.get("UserProfile") or {}
            device = (node.get("ComputedName") or node.get("Name") or "").split(".")[0]
            info = {"user": prof.get("LoginName") or "", "name": prof.get("DisplayName") or "",
                    # a device shared into this tailnet is anonymised ("device-of-shared-to-user")
                    "device": "" if device.startswith("device-of-shared") else device}
            ttl = TTL
    except (OSError, ValueError, subprocess.TimeoutExpired):
        pass
    with lock:
        cache[ip] = (now + ttl, info)
    return info


class H(BaseHTTPRequestHandler):
    def do_GET(self):
        info = whois(self.headers.get("X-Client-IP", ""))
        self.send_response(200)
        for k in ("user", "name", "device"):
            self.send_header(f"X-TS-{k.capitalize()}", quote(info.get(k, ""), safe="@.-_ "))
        self.send_header("Content-Length", "0")
        self.end_headers()

    def log_message(self, *a):
        pass


if __name__ == "__main__":
    ThreadingHTTPServer(("127.0.0.1", PORT), H).serve_forever()
