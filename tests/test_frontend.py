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
    assert (nodes["camera"].node_kind, nodes["camera"].enabled) == ("camera", True)
    assert (nodes["swerve"].node_kind, nodes["swerve"].enabled) == ("swerve", True)
    assert (nodes["arm"].node_kind, nodes["arm"].enabled) == ("arm", True)
    assert (nodes["estimator"].node_kind, nodes["estimator"].enabled) == ("estimator", True)
    assert (nodes["pose_servo"].node_kind, nodes["pose_servo"].enabled) == ("pose_servo", True)
    assert (nodes["3d_perception"].node_kind, nodes["3d_perception"].enabled) == ("3d_perception", True)
    assert (nodes["agent_nav"].node_kind, nodes["agent_nav"].enabled) == ("agent_nav", True)
    assert all(n.host is None for n in nodes.values())
    # sockets, no ports (description keeps TCP 8104 until it moves to the serve helper)
    assert all(n.port is None for name, n in nodes.items() if name != "description")


def test_config_file(tmp_path):
    p = tmp_path / "nodes.yaml"
    p.write_text("nodes:\n  - {name: a, port: 9001}\n  - {name: b, port: 9002, host: ' pi.local ', enabled: false}\n")
    assert load_config(p) == [NodeEntry("a", 9001), NodeEntry("b", 9002, "pi.local", False)]


@pytest.mark.parametrize(
    "data, msg",
    [
        ({"nodes": [{"name": "a", "host": "pi"}]}, "needs a 'port'"),
        ({"nodes": [{"name": "a", "kind": "x y"}]}, "kind"),
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
    """Routes requests by node: {name: behaviour} for local nodes (URL http://<name>/, i.e. its
    unix socket) or {port: behaviour} for remote ones. Records every request it sees."""

    def __init__(self, nodes: dict[int, dict]):
        self.nodes = nodes
        self.seen: list[tuple[str, str]] = []

    def __call__(self, request: httpx.Request) -> httpx.Response:
        self.seen.append((request.method, str(request.url)))
        key = request.url.host if request.url.host in self.nodes else request.url.port
        node = self.nodes.get(key)
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
        assert "E-STOP ALL" in r.text and '"nodes/" + encodeURIComponent(n.cfg.name)' in r.text
        assert "localhost" not in r.text and "https://" not in r.text  # no CDN, no hardwired host
        assert 'fetchJson("api/nodes"' in r.text  # relative URL within the frontend
        r = c.get("/api/nodes")
        assert r.json() == {"nodes": [
            {"name": "camera", "port": 8101, "host": None, "enabled": True, "start": None, "kind": None, "boot": False},
            {"name": "arm", "port": 8103, "host": None, "enabled": False, "start": None, "kind": None, "boot": False},
            {"name": "x", "port": 9000, "host": "other", "enabled": True, "start": None, "kind": None, "boot": False},
        ]}
        assert c.get("/api/health").json() == {"ok": True}


def test_estop_fanout_mixed():
    fake = FakeNodes({
        "camera": {"meta": meta("camera", actuated=False)},
        "swerve": {"meta": meta("swerve", actuated=True)},
        "arm": {"meta": meta("arm", actuated=True, estop="stop/now"), "estop_path": "/stop/now"},
        "gone": {"down": True},
        "bad": {"meta": meta("bad", actuated=True), "estop_status": 500, "estop_reply": {}},
        "liar": {"meta": meta("liar", actuated=True), "estop_reply": {"latched": False}},
        "slow": {"meta": meta("slow", actuated=True), "estop_timeout": True},
        "off": {"meta": meta("off", actuated=True)},
    })
    nodes = [NodeEntry(n) for n in ("camera", "swerve", "arm", "gone", "bad", "liar", "slow")]
    nodes.append(NodeEntry("off", enabled=False))
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
    assert "http://arm/stop/now" in posts                        # local node: its unix socket
    assert not any("//camera/" in u or "//off/" in u for u in posts)  # camera and disabled: never posted


def test_estop_all_latched_is_ok_and_uses_host():
    fake = FakeNodes({"swerve": {"meta": meta("swerve", actuated=True)}, 8103: {"meta": meta("arm", actuated=True)}})
    nodes = [NodeEntry("swerve"), NodeEntry("arm", 8103, host="arm-pi")]
    with client_for(nodes, fake) as c:
        r = c.post("/api/estop")
    assert r.status_code == 200 and r.json()["ok"] is True
    assert ("POST", "http://arm-pi:8103/estop") in fake.seen
    assert ("POST", "http://swerve/estop") in fake.seen


def test_estop_without_meta_still_tries_default_path():
    fake = FakeNodes({"swerve": {}})  # /meta 404, /estop works
    with client_for([NodeEntry("swerve")], fake) as c:
        r = c.post("/api/estop")
    assert r.json()["results"][0]["status"] == "latched"


def test_estop_nothing_actuated_is_not_ok():
    fake = FakeNodes({"camera": {"meta": meta("camera", actuated=False)}})
    with client_for([NodeEntry("camera")], fake) as c:
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

def test_cli_permission_error_points_to_script(monkeypatch, capsys, tmp_sock_dir):
    import robot_utils.nodes.common.serve as serve_mod

    def deny(host, port):
        raise PermissionError(13, "Permission denied")

    monkeypatch.setattr(serve_mod, "bind_tcp", deny)
    with pytest.raises(SystemExit) as e:
        cli.main(["--port", "80"])
    assert "sudo tools/enable_low_ports.sh" in str(e.value) and "--port 8080" in str(e.value)


def test_cli_port_in_use(tmp_sock_dir):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        s.listen()
        port = s.getsockname()[1]
        with pytest.raises(SystemExit) as e:
            cli.main(["--host", "127.0.0.1", "--port", str(port)])
    assert "cannot listen" in str(e.value)


def test_cli_bad_config(tmp_path, capsys):
    p = tmp_path / "n.yaml"
    p.write_text(json.dumps({"nodes": [{"name": "a", "host": "pi"}]}))
    assert cli.main(["--config", str(p)]) == 2
    assert "port" in capsys.readouterr().err


# --- proxy and tools ------------------------------------------------------------------------

class EchoNode:
    """Echoes what the proxy forwarded: method, path, query, lease header, body."""

    def __init__(self):
        self.seen = []

    def __call__(self, request: httpx.Request) -> httpx.Response:
        self.seen.append(request)
        if request.url.host != "swerve":
            raise httpx.ConnectError("refused", request=request)
        return httpx.Response(201, json={"method": request.method, "path": request.url.path,
                                         "query": str(request.url.query, "ascii"),
                                         "token": request.headers.get("x-lease-token"),
                                         "body": request.content.decode()},
                              headers={"x-internal": "not forwarded"})


def test_proxy_forwards_to_the_node_same_origin():
    nodes = [NodeEntry("swerve"), NodeEntry("camera"), NodeEntry("arm", enabled=False)]
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
        assert str(echo.seen[-1].url).startswith("http://swerve/")              # its unix socket
        assert echo.seen[-1].headers["x-forwarded-prefix"] == "/nodes/swerve"
        assert echo.seen[-1].headers["x-forwarded-for"] == "testclient"           # the peer, for identities
        c.get("/nodes/swerve/state", headers={"X-Forwarded-For": "1.2.3.4"})
        assert echo.seen[-1].headers["x-forwarded-for"] == "1.2.3.4, testclient"  # appended, like nginx
        assert c.delete("/nodes/swerve/lease").json()["method"] == "DELETE"
        r = c.get("/nodes/camera/state")                                         # node down
        assert r.status_code == 502 and "not running" in r.json()["detail"]
        assert c.get("/nodes/arm/state").status_code == 404                      # disabled node
        assert c.get("/nodes/nope/state").status_code == 404
        r = c.get("/nodes/swerve", follow_redirects=False)
        assert r.status_code == 301 and r.headers["location"] == "swerve/"


def test_tools_listed_and_served():
    with client_for([NodeEntry("swerve", 8102)], FakeNodes({})) as c:
        tools = c.get("/api/tools").json()["tools"]
        remote = next(t for t in tools if t["name"] == "remote_teleop")
        assert remote["href"] == "tools/remote_teleop.html" and remote["title"] == "Remote teleop"
        page = c.get("/tools/remote_teleop.html").text
        assert "../api/nodes" in page and "mjpeg/" in page and "overflow: hidden" in page
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


# --- restarting the frontend's own service --------------------------------------------------

def test_restart_refused_when_not_the_service():
    async def no():
        return False
    restarted = []

    async def restart():
        restarted.append(1)
    with TestClient(create_app([], transport=httpx.MockTransport(FakeNodes({})), is_service=no,
                               restart=restart)) as c:
        info = c.get("/api/frontend").json()
        assert info["service"] is False and info["unit"] == "robot-frontend.service"
        r = c.post("/api/frontend/restart")
        assert r.status_code == 409 and "not running as robot-frontend.service" in r.json()["detail"]
    assert restarted == []


def test_restart_replies_then_restarts():
    async def yes():
        return True
    restarted = []

    async def restart():
        restarted.append(1)
    with TestClient(create_app([], transport=httpx.MockTransport(FakeNodes({})), is_service=yes,
                               restart=restart)) as c:
        started = c.get("/api/frontend").json()["started"]
        r = c.post("/api/frontend/restart")
        assert r.status_code == 202 and r.json()["started"] == started
    assert restarted == [1]                     # ran after the reply (background task)
    with client_for([], FakeNodes({})) as c:
        page = c.get("/").text
        assert 'fetchJson("api/frontend/restart"' in page and "Restart landing page" in page


def test_local_node_with_port_uses_tcp_until_its_socket_exists():
    """A local node not yet on the serve helper (e.g. description on 8104) is reached on
    127.0.0.1:<port>; once its socket exists, over the socket."""
    from robot_utils.frontend.app import node_base_url
    from robot_utils.nodes.common.serve import bind_uds, socket_path
    node = NodeEntry("description", 8104)
    assert node_base_url(node) == "http://127.0.0.1:8104/"
    s = bind_uds(socket_path("description"))
    try:
        assert node_base_url(node) == "http://description/"
    finally:
        s.close()
    assert node_base_url(NodeEntry("swerve")) == "http://swerve/"
    assert node_base_url(NodeEntry("arm", 8103, host="arm-pi")) == "http://arm-pi:8103/"


def test_packaged_config_covers_description_for_estop():
    nodes = {n.name: n for n in load_config()}
    d = nodes["description"]
    assert d.enabled and d.node_kind == "description" and d.start.module == "robot_utils.nodes.description"
