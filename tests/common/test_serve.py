"""nodes.common.serve: instance names, unix sockets, the proxy prefix, name -> socket clients."""
import argparse
import os
import socket
import threading
import time

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from robot_utils.nodes.common.client import NodeClient
from robot_utils.nodes.common.serve import (PrefixMiddleware, add_serve_args, bind_uds, check_name,
                                            connect_target, socket_dir, socket_path)
from robot_utils.nodes.common.server import add_page_routes, node_meta


def test_socket_path_follows_runtime_dir(tmp_sock_dir):
    assert socket_dir() == __import__("pathlib").Path(tmp_sock_dir) / "robot"
    assert socket_path("camera2").name == "camera2.sock"


@pytest.mark.parametrize("name", ["", "a b", "../x", "a/b", "é"])
def test_bad_names(name):
    with pytest.raises(ValueError):
        check_name(name)


def test_args_default_name_is_the_kind():
    p = argparse.ArgumentParser()
    add_serve_args(p, "camera")
    a = p.parse_args([])
    assert (a.name, a.uds, a.port) == ("camera", None, None)
    a = p.parse_args(["--name", "cam2", "--port", "8101"])
    assert (a.name, a.port) == ("cam2", 8101)


def test_bind_uds_replaces_stale_and_refuses_live(tmp_sock_dir):
    path = socket_path("x")
    s = bind_uds(path)                       # creates the directory too
    assert path.is_socket() and oct(path.parent.stat().st_mode & 0o777) == "0o700"
    with pytest.raises(RuntimeError, match="already running"):
        bind_uds(path)                       # something listens: refuse
    s.close()                                # the process "died": file left behind
    assert path.exists()
    s2 = bind_uds(path)                      # stale: replaced
    s2.close()


def test_bind_uds_leaves_non_sockets_alone(tmp_sock_dir):
    path = socket_path("f")
    path.parent.mkdir(parents=True)
    path.write_text("not a socket")
    with pytest.raises(RuntimeError, match="not a socket"):
        bind_uds(path)
    assert path.read_text() == "not a socket"


def test_prefix_header_sets_root_path():
    app = FastAPI()

    @app.get("/where")
    def where(request: __import__("fastapi").Request):
        return {"root": request.scope.get("root_path", "")}
    c = TestClient(PrefixMiddleware(app))
    assert c.get("/where").json() == {"root": ""}                                   # direct (debug port)
    assert c.get("/where", headers={"X-Forwarded-Prefix": "/nodes/cam2"}).json() == {"root": "/nodes/cam2"}
    assert c.get("/where", headers={"X-Forwarded-Prefix": "/elsewhere"}).json() == {"root": ""}
    docs = c.get("/docs", headers={"X-Forwarded-Prefix": "/nodes/cam2"}).text
    assert "/nodes/cam2/openapi.json" in docs                                     # /docs works behind nginx


def test_connect_target():
    assert connect_target("http://botcore/nodes/swerve/") == ("http://botcore/nodes/swerve", None)
    url, uds = connect_target("swerve")
    assert url == "http://swerve" and uds.endswith("/robot/swerve.sock")
    with pytest.raises(ValueError):
        connect_target("not a name")


def test_node_client_reaches_an_instance_by_name(tmp_sock_dir):
    """What the estimator does with --swerve swerve: NodeClient("swerve") -> its unix socket."""
    import uvicorn
    app = FastAPI()
    add_page_routes(app, node_meta("standin", "Stand-in", "0", "standin", actuated=False))
    app.state.node_meta["name"] = "standin"
    sock = bind_uds(socket_path("standin"))
    server = uvicorn.Server(uvicorn.Config(app, log_level="warning"))
    t = threading.Thread(target=server.run, kwargs={"sockets": [sock]}, daemon=True)
    t.start()
    try:
        for _ in range(50):
            if server.started:
                break
            time.sleep(0.05)
        with NodeClient("standin") as c:
            meta = c.get("/meta")
        assert (meta["name"], meta["kind"]) == ("standin", "standin")
    finally:
        server.should_exit = True
        t.join(5)
        sock.close()


def test_node_url_and_for_instance_resolution(tmp_sock_dir):
    from robot_utils.nodes.common.client import node_url
    assert node_url("http://botcore/nodes/camera/") == "http://botcore/nodes/camera"
    assert node_url("camera") == "http://localhost/nodes/camera"      # no socket here: via nginx
    s = bind_uds(socket_path("camera"))
    try:
        assert node_url("camera") == "http://camera"                  # socket exists: use it
        c = NodeClient.for_instance("camera")
        assert c.base_url == "http://camera"
        c.close()
    finally:
        s.close()
