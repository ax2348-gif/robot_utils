"""common/fastpath.py: the RUC1 / RUS1 packets, the JSON control messages, the owner-only datagram socket."""
import math
import os
import socket
import stat
import tempfile

import pytest

from robot_utils.nodes.common import fastpath as F


@pytest.fixture
def sockdir():
    # short: a Unix socket path is limited to ~104 bytes on macOS (pytest's tmp_path is too long)
    d = tempfile.mkdtemp(prefix="rufp")
    yield d
    for n in os.listdir(d):
        os.unlink(os.path.join(d, n))
    os.rmdir(d)


def test_sizes_and_layout_are_the_contract():
    assert F.CMD.format == "<4sBBHIIdd6f6fff" and F.CMD_SIZE == 88
    assert F.STATE.format == "<4sBBHIIdIBB2x6f6f6f6fff" and F.STATE_SIZE == 136


def test_command_round_trip():
    q = (0.1, -0.2, 0.3, -0.4, 0.5, -0.6)
    qd = (1.0, 0.0, -1.0, 0.25, 0.0, 0.5)
    b = F.pack_command(7, 42, q, qd, gap=0.03, t_send=12.5, t_basis=11.0)
    assert len(b) == 88 and b[:4] == b"RUC1"
    c = F.unpack_command(b)
    assert (c.mode, c.flags, c.session, c.seq, c.t_send, c.t_basis) == (F.MODE_GOAL, 0, 7, 42, 12.5, 11.0)
    assert c.q == pytest.approx(q, abs=1e-7) and c.qd == pytest.approx(qd, abs=1e-7)
    assert c.gap == pytest.approx(0.03) and c.reserved == 0.0
    k = F.unpack_command(F.pack_command(7, 43, mode=F.MODE_KEEPALIVE))
    assert k.mode == F.MODE_KEEPALIVE and all(math.isnan(v) for v in k.q) and math.isnan(k.t_basis)
    # little-endian, field offsets as documented
    assert b[4] == 1 and b[5] == F.MODE_GOAL and int.from_bytes(b[8:12], "little") == 7
    assert int.from_bytes(b[12:16], "little") == 42


def test_command_rejects():
    b = F.pack_command(1, 1, (0.0,) * 6)
    with pytest.raises(F.PacketError) as e:
        F.unpack_command(b[:-1])
    assert e.value.reason == "size"
    with pytest.raises(F.PacketError) as e:
        F.unpack_command(b"RUX1" + b[4:])
    assert e.value.reason == "magic"
    with pytest.raises(F.PacketError) as e:
        F.unpack_command(b[:4] + bytes([2]) + b[5:])
    assert e.value.reason == "version"


def test_state_round_trip():
    s = F.State(F.STATE_SERVOING | F.STATE_STALE, 9, 1234, 55.125, 77, 8, 6, [0.1] * 6, [0.2] * 6, [0.3] * 6,
                [math.nan] * 6, 0.02, 0.0)
    b = F.pack_state(s)
    assert len(b) == 136 and b[:4] == b"RUS1"
    u = F.unpack_state(b)
    assert (u.flags, u.session, u.tick, u.t_tick, u.seq, u.source, u.state) == (3, 9, 1234, 55.125, 77, 8, 6)
    assert u.q == pytest.approx([0.1] * 6) and all(math.isnan(v) for v in u.q_cmd) and u.gap == pytest.approx(0.02)
    with pytest.raises(F.PacketError):
        F.unpack_state(b[:100])
    with pytest.raises(F.PacketError):
        F.unpack_state(b"RUC1" + b[4:])


def test_control_messages():
    m = F.hello("tok", "wbc")
    assert F.decode_control(F.encode_control(m)) == {"type": "hello", "token": "tok", "client": "wbc"}
    w = F.welcome(5, 30, {"lo": [0.0]})
    assert F.decode_control(F.encode_control(w))["session"] == 5
    assert F.bye() == {"type": "bye"} and F.bye("e-stop")["reason"] == "e-stop"
    assert F.refused("no lease") == {"type": "refused", "reason": "no lease"}
    assert F.is_control(F.encode_control(m)) and not F.is_control(F.pack_command(1, 1))
    for bad in (b"{nope", b"[1, 2]", b'{"x": 1}', b'{"type": 3}'):
        with pytest.raises(F.PacketError) as e:
            F.decode_control(bad)
        assert e.value.reason == "json"
    with pytest.raises(F.PacketError):
        F.decode_control(b"{" + b" " * F.MAX_CONTROL + b"}")
    with pytest.raises(ValueError):
        F.encode_control({"type": "x", "v": math.nan})


def test_bind_dgram_is_owner_only_and_replaces_stale(sockdir):
    p = os.path.join(sockdir, "a.joint.sock")
    s = F.bind_dgram(p)
    try:
        assert stat.S_ISSOCK(os.lstat(p).st_mode) and stat.S_IMODE(os.lstat(p).st_mode) == 0o600
        with pytest.raises(RuntimeError, match="already running"):
            F.bind_dgram(p)                       # a live one is refused
        c = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        c.sendto(b"x", p)
        c.close()
        assert s.recv(10) == b"x"
    finally:
        s.close()
    s = F.bind_dgram(p)                           # stale file left behind: replaced
    s.close()
    F.unlink_socket(p)
    assert not os.path.exists(p)
    open(p, "w").close()
    with pytest.raises(RuntimeError, match="not a socket"):
        F.bind_dgram(p)


def test_client_talks_to_a_server(sockdir):
    srv = F.bind_dgram(os.path.join(sockdir, "srv.sock"))
    srv.setblocking(True)
    srv.settimeout(2)
    c = F.Client(os.path.join(sockdir, "srv.sock"), os.path.join(sockdir, "cl.sock"))
    try:
        import threading
        out = {}

        def serve():
            data, addr = srv.recvfrom(4096)
            out["hello"] = F.decode_control(data)
            srv.sendto(F.encode_control(F.welcome(11, 30, {})), addr)
            data, addr = srv.recvfrom(4096)
            out["cmd"] = F.unpack_command(data)
            srv.sendto(F.pack_state(F.State(0, 11, 1, 1.0, 1, 0, 0, [0.0] * 6, [0.0] * 6, [0.0] * 6,
                                            [0.0] * 6, 0.0, 0.0)), addr)
        th = threading.Thread(target=serve)
        th.start()
        assert c.hello("t", "test")["session"] == 11
        assert c.send((0.0,) * 6) == 1
        st = c.recv(2)
        th.join(2)
        assert out["hello"]["token"] == "t" and out["cmd"].session == 11 and out["cmd"].seq == 1
        assert isinstance(st, F.State) and st.session == 11
    finally:
        c.close()
        srv.close()
