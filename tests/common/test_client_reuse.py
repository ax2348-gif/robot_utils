"""NodeClient builds its TLS context once per process, and agent_interface's 10 Hz pose_servo watch keeps one
client (both measured on the Pi: a fresh client per poll cost ~30% of a core, loading the CA bundle each time)."""
import ssl
import threading

from robot_utils.nodes.common import client as C


def test_node_clients_share_one_ssl_context(monkeypatch, tmp_path):
    calls = []
    real = ssl.create_default_context

    def counting(*a, **kw):
        calls.append(1)
        return real(*a, **kw)

    monkeypatch.setattr(ssl, "create_default_context", counting)
    monkeypatch.setattr(C, "_ssl_ctx", None)
    sock = tmp_path / "x.sock"
    sock.touch()
    monkeypatch.setattr(C, "socket_path", lambda name: sock)          # an instance name -> unix socket transport
    clients = [C.NodeClient("http://localhost/nodes/a"), C.NodeClient("x"), C.NodeClient("x"),
               C.NodeClient("https://example.invalid")]
    try:
        assert len(calls) == 1
        ctx = C._ssl_context()
        assert ctx.verify_mode == ssl.CERT_REQUIRED and ctx.check_hostname       # https still verified
        for c in clients:
            assert c.http._transport._pool._ssl_context is ctx
    finally:
        for c in clients:
            c.close()


def test_watch_keeps_its_pose_servo_client(monkeypatch):
    from robot_utils.nodes.agent_interface import node as N

    made, polls = [], []
    stop = threading.Event()

    class FakeClient:
        def __init__(self, target, timeout=5.0):
            made.append(self)
            self.closed = False

        def get(self, path, **kw):
            polls.append(self)
            if len(polls) == 3:
                raise ConnectionError("pose_servo restarted")
            if len(polls) >= 6:
                stop.set()
            return {"status": "idle"}

        def close(self):
            self.closed = True

    monkeypatch.setattr(N, "NodeClient", FakeClient)
    nav = object.__new__(N.AgentNav)
    nav.stop_ev, nav.targets, nav.goals = stop, {"pose_servo": "http://ps"}, {}
    nav.enabled_by, nav.enabled_at, nav.lock, nav._ps_status = None, 0.0, threading.Lock(), None
    nav._anomaly_check = lambda st: None
    nav.reflex, nav._reflex_ok_t, nav._ps_moving_last, nav._failsafe_episode = {"armed": False}, 0.0, False, False
    nav._watch()
    assert len(polls) == 6
    assert len(made) == 2                                     # one client, replaced once after the failed poll
    assert polls[:3] == [made[0]] * 3 and polls[3:] == [made[1]] * 3
    assert made[0].closed and made[1].closed
    assert nav._ps_status == {"status": "idle"}
