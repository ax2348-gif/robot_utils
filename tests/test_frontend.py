"""Frontend tests: no hardware, no network. Fake nodes are httpx.MockTransport handlers."""

from __future__ import annotations

import json
import socket

import httpx
import pytest
from fastapi.testclient import TestClient

from robot_utils.frontend import __main__ as cli
from robot_utils.frontend.app import create_app, estop_path
from robot_utils.frontend.config import ConfigError, NodeEntry, load_config, parse_config


# --- config ---------------------------------------------------------------------------------

def test_packaged_default_config():
    nodes = {n.name: n for n in load_config()}
    assert (nodes["camera"].port, nodes["camera"].enabled) == (8101, True)
    assert (nodes["swerve"].port, nodes["swerve"].enabled) == (8102, True)
    assert (nodes["arm"].port, nodes["arm"].enabled) == (8103, False)
    assert (nodes["estimator"].port, nodes["estimator"].enabled) == (8110, False)
    assert all(n.host is None for n in nodes.values())


def test_config_file(tmp_path):
    p = tmp_path / "nodes.yaml"
    p.write_text("nodes:\n  - {name: a, port: 9001}\n  - {name: b, port: 9002, host: ' pi.local ', enabled: false}\n")
    assert load_config(p) == [NodeEntry("a", 9001), NodeEntry("b", 9002, "pi.local", False)]


@pytest.mark.parametrize(
    "data, msg",
    [
        ({"nodes": [{"name": "a"}]}, "port"),
        ({"nodes": [{"name": "a", "port": 0}]}, "port"),
        ({"nodes": [{"name": "a", "port": True}]}, "port"),
        ({"nodes": [{"name": "a b", "port": 1}]}, "name"),
        ({"nodes": [{"name": "a", "port": 1}, {"name": "a", "port": 2}]}, "duplicate"),
        ({"nodes": [{"name": "a", "port": 1, "prot": 2}]}, "unknown"),
        ({"nodes": [{"name": "a", "port": 1, "enabled": "no"}]}, "enabled"),
        ({"nodes": ["a"]}, "mapping"),
        ([1, 2], "mapping"),
    ],
)
def test_config_errors(data, msg):
    with pytest.raises(ConfigError, match=msg):
        parse_config(data)


def test_config_missing_and_bad_yaml(tmp_path):
    with pytest.raises(ConfigError, match="cannot read"):
        load_config(tmp_path / "nope.yaml")
    bad = tmp_path / "bad.yaml"
    bad.write_text("nodes: [\n")
    with pytest.raises(ConfigError, match="invalid YAML"):
        load_config(bad)
    empty = tmp_path / "empty.yaml"
    empty.write_text("# nothing\n")
    assert load_config(empty) == []


# --- fake nodes -----------------------------------------------------------------------------

def meta(name, actuated, estop="estop"):
    return {"name": name, "title": name.title(), "kind": name, "version": "0.1", "has_page": True,
            "page": "./", "actuated": actuated,
            "endpoints": {"health": "health", "state": "state", "events": "events",
                          "estop": estop if actuated else None}}


class FakeNodes:
    """Routes requests by port: {port: behaviour}. Records every request it sees."""

    def __init__(self, nodes: dict[int, dict]):
        self.nodes = nodes
        self.seen: list[tuple[str, str]] = []

    def __call__(self, request: httpx.Request) -> httpx.Response:
        self.seen.append((request.method, str(request.url)))
        node = self.nodes.get(request.url.port)
        if node is None or node.get("down"):
            raise httpx.ConnectError("refused", request=request)
        path = request.url.path
        if request.method == "GET" and path == "/meta":
            if "meta" not in node:
                return httpx.Response(404)
            return httpx.Response(200, json=node["meta"])
        if request.method == "POST" and path == node.get("estop_path", "/estop"):
            if node.get("estop_timeout"):
                raise httpx.ReadTimeout("slow", request=request)
            return httpx.Response(node.get("estop_status", 200), json=node.get("estop_reply", {"latched": True}))
        return httpx.Response(404)


def client_for(nodes: list[NodeEntry], fake: FakeNodes) -> TestClient:
    return TestClient(create_app(nodes, transport=httpx.MockTransport(fake)))


# --- endpoints ------------------------------------------------------------------------------

def test_landing_page_and_node_list():
    nodes = [NodeEntry("camera", 8101), NodeEntry("arm", 8103, enabled=False), NodeEntry("x", 9000, "other")]
    with client_for(nodes, FakeNodes({})) as c:
        r = c.get("/")
        assert r.status_code == 200 and r.headers["content-type"].startswith("text/html")
        assert "E-STOP ALL" in r.text and "location.hostname" in r.text
        assert "localhost" not in r.text and "https://" not in r.text  # no CDN, no hardwired host
        assert 'fetchJson("api/nodes"' in r.text  # relative URL within the frontend
        r = c.get("/api/nodes")
        assert r.json() == {"nodes": [
            {"name": "camera", "port": 8101, "host": None, "enabled": True, "start": None},
            {"name": "arm", "port": 8103, "host": None, "enabled": False, "start": None},
            {"name": "x", "port": 9000, "host": "other", "enabled": True, "start": None},
        ]}
        assert c.get("/api/health").json() == {"ok": True}


def test_estop_fanout_mixed():
    fake = FakeNodes({
        8101: {"meta": meta("camera", actuated=False)},
        8102: {"meta": meta("swerve", actuated=True)},
        8103: {"meta": meta("arm", actuated=True, estop="stop/now"), "estop_path": "/stop/now"},
        8104: {"down": True},
        8105: {"meta": meta("bad", actuated=True), "estop_status": 500, "estop_reply": {}},
        8106: {"meta": meta("liar", actuated=True), "estop_reply": {"latched": False}},
        8107: {"meta": meta("slow", actuated=True), "estop_timeout": True},
        8108: {"meta": meta("off", actuated=True)},
    })
    nodes = [NodeEntry(n, p) for n, p in [("camera", 8101), ("swerve", 8102), ("arm", 8103), ("gone", 8104),
                                          ("bad", 8105), ("liar", 8106), ("slow", 8107)]]
    nodes.append(NodeEntry("off", 8108, enabled=False))
    with client_for(nodes, fake) as c:
        r = c.post("/api/estop")
    assert r.status_code == 502
    body = r.json()
    assert body["ok"] is False
    status = {x["name"]: (x["status"], x["detail"]) for x in body["results"]}
    assert status == {
        "camera": ("skipped", "not actuated"),
        "swerve": ("latched", ""),
        "arm": ("latched", ""),
        "gone": ("unreachable", "ConnectError"),
        "bad": ("failed", "HTTP 500"),
        "liar": ("failed", "reply did not confirm latched"),
        "slow": ("unreachable", "timed out"),
    }
    posts = [u for m, u in fake.seen if m == "POST"]
    assert "http://127.0.0.1:8103/stop/now" in posts
    assert not any(":8101/" in u or ":8108/" in u for u in posts)  # camera and disabled: never posted


def test_estop_all_latched_is_ok_and_uses_host():
    fake = FakeNodes({8102: {"meta": meta("swerve", actuated=True)}, 8103: {"meta": meta("arm", actuated=True)}})
    nodes = [NodeEntry("swerve", 8102), NodeEntry("arm", 8103, host="arm-pi")]
    with client_for(nodes, fake) as c:
        r = c.post("/api/estop")
    assert r.status_code == 200 and r.json()["ok"] is True
    assert ("POST", "http://arm-pi:8103/estop") in fake.seen
    assert ("POST", "http://127.0.0.1:8102/estop") in fake.seen


def test_estop_without_meta_still_tries_default_path():
    fake = FakeNodes({8102: {}})  # /meta 404, /estop works
    with client_for([NodeEntry("swerve", 8102)], fake) as c:
        r = c.post("/api/estop")
    assert r.json()["results"][0]["status"] == "latched"


def test_estop_nothing_actuated_is_not_ok():
    fake = FakeNodes({8101: {"meta": meta("camera", actuated=False)}})
    with client_for([NodeEntry("camera", 8101)], fake) as c:
        r = c.post("/api/estop")
    assert r.status_code == 502 and r.json()["results"][0]["status"] == "skipped"


@pytest.mark.parametrize(
    "m, expected",
    [
        (None, "estop"),
        ({}, "estop"),
        ({"actuated": True}, "estop"),
        ({"actuated": False}, None),
        ({"actuated": True, "endpoints": {"estop": None}}, "estop"),
        ({"actuated": False, "endpoints": {"estop": "x/estop"}}, "x/estop"),
        ({"endpoints": {"estop": None}}, None),
        ({"endpoints": "junk"}, "estop"),
    ],
)
def test_estop_path(m, expected):
    assert estop_path(m) == expected


# --- CLI ------------------------------------------------------------------------------------

def test_cli_permission_error_points_to_script(monkeypatch, capsys):
    def deny(host, port):
        raise PermissionError(13, "Permission denied")

    monkeypatch.setattr(cli, "bind", deny)
    assert cli.main(["--port", "80"]) == 1
    err = capsys.readouterr().err
    assert "sudo tools/enable_low_ports.sh" in err and "--port 8080" in err


def test_cli_port_in_use(capsys):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        s.listen()
        port = s.getsockname()[1]
        assert cli.main(["--host", "127.0.0.1", "--port", str(port)]) == 1
    assert "already in use" in capsys.readouterr().err


def test_cli_bad_config(tmp_path, capsys):
    p = tmp_path / "n.yaml"
    p.write_text(json.dumps({"nodes": [{"name": "a"}]}))
    assert cli.main(["--config", str(p)]) == 2
    assert "port" in capsys.readouterr().err


# --- proxy and tools ------------------------------------------------------------------------

class EchoNode:
    """Echoes what the proxy forwarded: method, path, query, lease header, body."""

    def __init__(self):
        self.seen = []

    def __call__(self, request: httpx.Request) -> httpx.Response:
        self.seen.append(request)
        if request.url.port != 8102:
            raise httpx.ConnectError("refused", request=request)
        return httpx.Response(201, json={"method": request.method, "path": request.url.path,
                                         "query": str(request.url.query, "ascii"),
                                         "token": request.headers.get("x-lease-token"),
                                         "body": request.content.decode()},
                              headers={"x-internal": "not forwarded"})


def test_proxy_forwards_to_the_node_same_origin():
    nodes = [NodeEntry("swerve", 8102), NodeEntry("camera", 8101), NodeEntry("arm", 8103, enabled=False)]
    echo = EchoNode()
    with TestClient(create_app(nodes, transport=httpx.MockTransport(echo))) as c:
        r = c.post("/nodes/swerve/command?x=1", json={"twist": [0.1, 0, 0]},
                   headers={"X-Lease-Token": "tok", "Cookie": "secret=1"})
        assert r.status_code == 201
        body = r.json()
        assert body["method"] == "POST" and body["path"] == "/command" and body["query"] == "x=1"
        assert body["token"] == "tok" and '"twist"' in body["body"]
        assert "x-internal" not in r.headers
        assert "cookie" not in {k.lower() for k in echo.seen[-1].headers}       # only allow-listed headers
        assert str(echo.seen[-1].url).startswith("http://127.0.0.1:8102/")      # loopback on the robot
        assert c.delete("/nodes/swerve/lease").json()["method"] == "DELETE"
        r = c.get("/nodes/camera/state")                                         # node down
        assert r.status_code == 502 and "unreachable" in r.json()["detail"]
        assert c.get("/nodes/arm/state").status_code == 404                      # disabled node
        assert c.get("/nodes/nope/state").status_code == 404


def test_tools_listed_and_served():
    with client_for([NodeEntry("swerve", 8102)], FakeNodes({})) as c:
        tools = c.get("/api/tools").json()["tools"]
        teleop = next(t for t in tools if t["name"] == "swerve_teleop")
        assert teleop["href"] == "tools/swerve_teleop.html" and teleop["title"] == "Swerve teleop"
        assert teleop["description"]
        r = c.get("/tools/swerve_teleop.html")
        assert r.status_code == 200 and "../nodes/swerve/" in r.text
        assert "localhost" not in r.text and "https://" not in r.text
        assert c.get("/tools/missing.html").status_code == 404
        assert c.get("/tools/..%2Fpage.html").status_code == 404


def test_landing_page_uses_the_proxy():
    with client_for([NodeEntry("swerve", 8102)], FakeNodes({})) as c:
        page = c.get("/").text
        assert '"nodes/" + encodeURIComponent(n.cfg.name)' in page
        assert 'fetchJson("api/tools"' in page
