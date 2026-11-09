"""The generated nginx config (frontend/proxy.py); `nginx -t` when an nginx binary is around
(on PATH, or NGINX_BIN=/path/to/nginx; not NGINX, which nginx itself reads)."""
import os
import shutil
import subprocess
from pathlib import Path

import pytest

from robot_utils.frontend.proxy import render, write

SOCKS = Path("/run/user/1000/robot")


def test_routes_and_streaming_settings(tmp_path):
    conf = render(sockets=SOCKS, state_dir=tmp_path, port=80)
    assert "listen 80 default_server;" in conf
    # wildcard instance rule -> its own socket, prefix header, fallback to the landing page
    assert "location ~ ^/nodes/(?<node>[A-Za-z0-9_.-]+)(?<rest>/.*)$" in conf
    assert "proxy_pass http://unix:/run/user/1000/robot/$node.sock:$rest$is_args$args;" in conf
    assert "proxy_set_header X-Forwarded-Prefix /nodes/$node;" in conf
    assert "error_page 502 504 = @frontend;" in conf
    assert "return 301 /nodes/$node/$is_args$args;" in conf
    assert conf.count("proxy_pass http://unix:/run/user/1000/robot/frontend.sock;") == 2   # / and @frontend
    # streams pass straight through, in every proxied location
    for line in ("proxy_buffering off;", "proxy_request_buffering off;", "proxy_http_version 1.1;",
                 'proxy_set_header Connection "";', "proxy_read_timeout 1d;"):
        assert conf.count(line) == 3, line
    assert "gzip off;" in conf
    # runs as the user: pid, logs, temp files under the state dir
    assert f"pid {tmp_path}/nginx.pid;" in conf and f"error_log {tmp_path}/error.log" in conf
    assert f"proxy_temp_path {tmp_path}/tmp/proxy;" in conf
    # JSON access log (tailed by the observer), the node from the URL
    assert f"access_log {tmp_path}/access.log robot_json buffer=16k flush=2s;" in conf
    assert "log_format robot_json escape=json" in conf and '"node":"$robot_node"' in conf
    assert "~^/nodes/(?<n>[A-Za-z0-9_.-]+) $n;" in conf


def nginx_binary():
    return os.environ.get("NGINX_BIN") or shutil.which("nginx")


@pytest.mark.skipif(not nginx_binary(), reason="no nginx binary (set NGINX_BIN=/path/to/nginx)")
def test_nginx_accepts_it(tmp_path, tmp_sock_dir):
    state = tmp_path / "state"
    path = write(tmp_path / "conf", state, port=18080, sockets=Path(tmp_sock_dir) / "robot")
    r = subprocess.run([nginx_binary(), "-p", str(state), "-e", str(state / "error.log"), "-c", str(path), "-t"],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
