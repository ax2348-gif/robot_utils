"""frontend/observe.py: access-log tail, identities, live peers, uplink, rotation, report, endpoints."""
import json
import os
import signal
import time
from pathlib import Path

import pytest

from robot_utils.frontend import observe as O

WHOIS = json.dumps({"Node": {"Name": "averys-macbook-air.tail0fa175.ts.net.", "ComputedName": "averys-macbook-air",
                             "Hostinfo": {"OS": "macOS"}},
                    "UserProfile": {"LoginName": "averyxi42@github", "DisplayName": "Avery Xi"}})
SS_1 = """\
0      0           100.80.153.120:80       100.65.75.106:57604
\t cubic wscale:7,7 rto:204 bytes_acked:1000 bytes_received:300
0      2450000     100.80.153.120:80       100.71.119.7:51476
\t cubic bytes_acked:5000000 bytes_received:900
0      0           100.80.153.120:22       100.65.75.106:50000
\t cubic bytes_acked:9 bytes_received:9
"""
SS_2 = SS_1.replace("bytes_acked:1000 ", "bytes_acked:3000 ").replace("bytes_acked:5000000", "bytes_acked:6000000")


def entry(ts, ip="100.65.75.106", uri="/nodes/camera/state", node="camera", status=200, nbytes=500):
    return json.dumps({"t": "x", "ts": ts, "ip": ip, "method": "GET", "uri": uri, "status": status,
                       "bytes": nbytes, "req_bytes": 80, "dur": 0.01, "node": node, "ua": "t", "conn": 1}) + "\n"


class FakeRunner:
    def __init__(self, outputs=None):
        self.outputs, self.calls = outputs or {}, []

    def __call__(self, argv, timeout):
        self.calls.append(argv)
        key = argv[0]
        v = self.outputs.get(key)
        if isinstance(v, list):
            return v.pop(0) if v else None
        return v


def test_identities_tailscale_lan_and_cache():
    r = FakeRunner({"tailscale": WHOIS})
    clock = [0.0]
    ids = O.Identities(r, clock=lambda: clock[0])
    a = ids.get("100.65.75.106")
    assert a["user"] == "averyxi42@github" and a["device"] == "averys-macbook-air" and a["os"] == "macOS"
    ids.get("100.65.75.106")
    assert len(r.calls) == 1                                        # cached
    clock[0] += O.WHOIS_TTL + 1
    ids.get("100.65.75.106")
    assert len(r.calls) == 2                                        # expired
    lan = ids.get("192.168.8.191")
    assert lan["kind"] == "lan" and len(r.calls) == 2               # never whois for LAN
    unknown = O.Identities(FakeRunner({"tailscale": None})).get("100.71.119.7")
    assert "unknown to whois" in unknown["label"]
    assert O.is_tailscale("fd7a:115c:a1e0::822c:4b6b") and not O.is_tailscale("10.0.0.1")


def test_log_tail_follows_rotation_and_waits_for_complete_lines(tmp_path):
    log = tmp_path / "access.log"
    log.write_text(entry(1.0) + entry(2.0))
    t = O.LogTail(log)
    assert t.poll() == 2
    with open(log, "a") as f:
        f.write(entry(3.0)[:-20])                                    # half a line
    assert t.poll() == 0
    with open(log, "a") as f:
        f.write(entry(3.0)[-20:] + "garbage\n")
    assert t.poll() == 1                                             # the completed line; garbage skipped
    log.rename(tmp_path / "access-old.log")                          # rotation
    log.write_text(entry(4.0))
    assert t.poll() == 1 and [e["ts"] for e in t.recent] == [1.0, 2.0, 3.0, 4.0]


def test_log_tail_warm_start_skips_the_partial_first_line(tmp_path):
    log = tmp_path / "access.log"
    log.write_text("".join(entry(float(i)) for i in range(200)))
    t = O.LogTail(log, warm_bytes=1000)
    n = t.poll()
    assert 0 < n < 200 and all("ip" in e for e in t.recent) and t.recent[-1]["ts"] == 199.0


def test_parse_ss_and_live_rates():
    cur = O.parse_ss(SS_1)
    assert len(cur) == 2                                             # the :22 line is not ours
    r = FakeRunner({"ss": [SS_1, SS_2], "tailscale": WHOIS})
    ob = O.Observer(Path("/nonexistent"), runner=r, netdev=lambda: {})
    ob.sample_peers(now=10.0)
    ob.sample_peers(now=12.0)
    assert ob.peers["100.65.75.106"]["bps"] == pytest.approx(1000.0)
    assert ob.peers["100.71.119.7"]["bps"] == pytest.approx(500000.0) and ob.peers["100.71.119.7"]["queue"] == 2450000
    assert O.peer_ip("[fd7a:115c:a1e0::1]:5000") == "fd7a:115c:a1e0::1"


def test_clients_view_merges_log_and_live(tmp_path):
    log = tmp_path / "access.log"
    now = time.time()
    log.write_text(entry(now - 10) + entry(now - 5, uri="/nodes/agent_nav/look", node="agent_nav")
                   + entry(now - 7200) + entry(now - 3, ip="192.168.8.50", node="", uri="/", status=404))
    r = FakeRunner({"ss": [SS_1, SS_2], "tailscale": WHOIS})
    ob = O.Observer(tmp_path, runner=r, netdev=lambda: {})
    ob.tail.poll()
    ob.sample_peers(now=1.0)
    ob.sample_peers(now=2.0)
    cl = {c["identity"]["ip"]: c for c in ob.clients(now=now)}
    mac = cl["100.65.75.106"]
    assert mac["hour"]["requests"] == 2 and mac["hour"]["nodes"] == {"camera": 1, "agent_nav": 1}   # 2 h old excluded
    assert mac["hour"]["last_uri"] == "/nodes/agent_nav/look" and mac["identity"]["user"] == "averyxi42@github"
    assert cl["192.168.8.50"]["hour"]["errors"] == 1 and cl["192.168.8.50"]["identity"]["kind"] == "lan"
    heavy = cl["100.71.119.7"]
    assert heavy["hour"] is None and heavy["live"]["send_bps"] == 1000000 and heavy["live"]["connections"] == 1
    assert list(cl) and ob.clients(now=now)[0]["identity"]["ip"] == "100.71.119.7"   # heaviest first
    reqs = ob.requests(limit=10, ip="100.65.75.106")
    assert len(reqs) == 3 and reqs[0]["uri"] == "/nodes/agent_nav/look" and "averyxi42" in reqs[0]["who"]


def test_uplink_saturation():
    pings = ["64 bytes from 1.1.1.1: icmp_seq=1 ttl=57 time=420 ms"] * 6
    r = FakeRunner({"ip": "default via 192.168.8.1 dev eth0", "ping": None})
    ob = O.Observer(Path("/x"), runner=r, netdev=lambda: {"eth0": (100, 200)})
    for p in pings:
        r.outputs["ping"] = [p, "64 bytes from 192.168.8.1: icmp_seq=1 ttl=64 time=0.35 ms"]
        ob.sample_uplink()
    v = ob.uplink_view()
    assert v["internet_ms"] == 420 and v["gateway_ms"] == 0.35 and v["saturated"]
    r.outputs["ping"] = None                                          # losses
    ob2 = O.Observer(Path("/x"), runner=FakeRunner({"ip": "", "ping": None}), netdev=lambda: {})
    for _ in range(3):
        ob2.sample_uplink()
    assert ob2.uplink_view()["saturated"] and ob2.uplink_view()["lost_1min"] == 3
    assert O.parse_ping("time=9.87 ms") == 9.87 and O.parse_ping("timeout") is None


def test_rotation_daily_size_retention_and_reopen(tmp_path):
    now = time.time()
    (tmp_path / "nginx.pid").write_text("4242\n")
    (tmp_path / "access.log").write_text(entry(now - 86400 * 1.5))   # starts yesterday
    old = tmp_path / "access-20200101-000000.log"
    old.write_text(entry(1.0))
    os.utime(old, (now - 86400 * 8, now - 86400 * 8))
    sent = []
    acts = O.rotate(tmp_path, now=now, send_signal=lambda pid, sig: sent.append((pid, sig)))
    assert any("rotated access.log" in a for a in acts) and not (tmp_path / "access.log").exists()
    assert sent == [(4242, signal.SIGUSR1)] and not old.exists()
    # today's small log: left alone, no signal
    (tmp_path / "access.log").write_text(entry(now - 60))
    sent.clear()
    assert O.rotate(tmp_path, now=now, send_signal=lambda pid, sig: sent.append(pid)) == [] and not sent
    # too big: rotated even if it is today's
    with open(tmp_path / "access.log", "a") as f:
        f.truncate(O.ROTATE_BYTES + 10)
    assert any("rotated" in a for a in O.rotate(tmp_path, now=now, send_signal=lambda *a: None))


def test_terminal_report(tmp_path):
    now = time.time()
    (tmp_path / "access.log").write_text(entry(now - 100, nbytes=5_000_000) + entry(now - 50, uri="/nodes/camera/stream?x=1")
                                         + entry(now - 30, ip="192.168.8.50", node=""))
    txt = O.report(tmp_path, hours=24, top=10, ids=O.Identities(FakeRunner({"tailscale": WHOIS})), now=now)
    assert "3 requests from 2 clients" in txt and "averyxi42@github" in txt and "5.0 MB sent" in txt
    assert "/nodes/camera/stream" in txt and "?x=1" not in txt and "192.168.8.50" in txt


def test_endpoints():
    import asyncio

    import httpx

    from robot_utils.frontend.app import create_app

    class FakeObserver:
        started = closed = False

        def start(self):
            self.started = True

        def close(self):
            self.closed = True

        def snapshot(self):
            return {"clients": [{"identity": {"ip": "1.2.3.4", "label": "x"}}], "uplink": {}}

        def requests(self, limit, ip, node):
            return [{"limit": limit, "ip": ip, "node": node}]

    async def go(app):
        async with app.router.lifespan_context(app):
            async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://t") as c:
                return (await c.get("/api/observe")), (await c.get("/api/observe/requests", params={"limit": 5000, "ip": "1.2.3.4"}))

    fo = FakeObserver()
    a, b = asyncio.run(go(create_app([], observer=fo)))
    assert a.status_code == 200 and a.json()["clients"][0]["identity"]["ip"] == "1.2.3.4"
    assert b.json() == [{"limit": 1000, "ip": "1.2.3.4", "node": None}] and fo.started and fo.closed
    a, _ = asyncio.run(go(create_app([])))
    assert a.status_code == 503
