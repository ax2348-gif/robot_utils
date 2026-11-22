"""A fake arm node for the WBC tests: its HTTP control plane (lease, enable, disable, e-stop, tcp) as a FastAPI app,
and a peer on the internal joint channel (hello / welcome, RUC1 commands in, RUS1 state out every 10 ms and after
each command), written against docs/plans/fast_path_v1.md. The joints follow the commanded q exactly (kinematic),
optionally only within ``max_step`` per command. Until the real arm side is merged (the parallel branch)."""
import json
import math
import os
import secrets
import socket
import threading
import time

import numpy as np
from fastapi import FastAPI, Header, HTTPException

from robot_utils.nodes.wbc.arm_channel import (ARM_STATES, MODE_GOAL, STATE_SIZE, pack_state, unpack_command)

Q0 = (0.0, -0.6, -0.9, 0.0, 0.5, 0.0)


class FakeArm:
    def __init__(self, sock_path: str, q0=Q0, period: float = 0.01):
        self.path = sock_path
        self.q = np.array(q0, float)
        self.q_cmd = np.full(6, math.nan)
        self.token = None
        self.lease_id = 0
        self.enabled = False
        self.estopped = None
        self.state = "disabled"
        self.session = None
        self.client_addr = None
        self.seq = 0
        self.tick = 0
        self.commands = []          # (t, q, qd, gap)
        self.gap = math.nan
        self.refuse = None          # a reason to refuse hellos
        self.kinematic = True       # the joints jump to each command (False: a subclass moves them)
        self.period = period
        self.lock = threading.Lock()
        if os.path.exists(sock_path):
            os.unlink(sock_path)
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.bind(sock_path)
        self.sock.settimeout(0.002)
        self.stop = threading.Event()
        self.threads = [threading.Thread(target=self._rx, daemon=True), threading.Thread(target=self._tx, daemon=True)]
        for t in self.threads:
            t.start()
        self.app = self._make_app()

    # -- joint channel ------------------------------------------------------------------------------------

    def _send_state(self):
        if self.session is None or self.client_addr is None:
            return
        st = ARM_STATES.index(self.state)
        flags = 1 if self.state == "servoing" else 0
        pkt = pack_state(self.session, self.tick, time.monotonic(), self.seq, 8, st, self.q, np.zeros(6),
                         np.zeros(6), self.q_cmd, self.gap, 0.0, flags)
        try:
            self.sock.sendto(pkt, self.client_addr)
        except OSError:
            pass

    def _rx(self):
        while not self.stop.is_set():
            try:
                data, addr = self.sock.recvfrom(4096)
            except (socket.timeout, OSError):
                continue
            with self.lock:
                if data[:1] == b"{":
                    m = json.loads(data)
                    if m.get("type") == "hello":
                        if self.refuse or m.get("token") != self.token or not self.enabled:
                            why = self.refuse or ("not enabled" if m.get("token") == self.token else "bad token")
                            self.sock.sendto(json.dumps({"type": "refused", "reason": why}).encode(), addr)
                            continue
                        self.session = secrets.randbits(31)
                        self.client_addr, self.seq = addr, 0
                        self.sock.sendto(json.dumps({"type": "welcome", "session": self.session, "stale_ms": 30,
                                                     "limits": {}}).encode(), addr)
                        self._send_state()
                    elif m.get("type") == "bye":
                        self.session = None
                    continue
                c = unpack_command(data)
                if c is None or c.session != self.session or c.seq <= self.seq:
                    continue
                self.seq = c.seq
                if c.mode == MODE_GOAL and self.state in ("floating", "servoing"):
                    self.q_cmd = c.q.copy()
                    self.qd_cmd = c.qd.copy()
                    if self.kinematic:
                        self.q = c.q.copy()
                    self.state = "servoing"
                    if math.isfinite(c.gap):
                        self.gap = c.gap
                    self.commands.append((time.monotonic(), c.q.copy(), c.qd.copy(), c.gap))
                self._send_state()

    def _tx(self):
        while not self.stop.is_set():
            time.sleep(self.period)
            with self.lock:
                self.tick += 1
                self._send_state()

    def close(self):
        self.stop.set()
        for t in self.threads:
            t.join(1)
        self.sock.close()
        try:
            os.unlink(self.path)
        except OSError:
            pass

    def set_state(self, state):
        with self.lock:
            self.state = state

    # -- HTTP -----------------------------------------------------------------------------------------------

    def _make_app(self):
        app = FastAPI()
        arm = self

        def check(tok):
            if tok is None or tok != arm.token:
                raise HTTPException(403, "wrong lease token")

        @app.post("/lease")
        def lease(body: dict):
            if arm.token is not None:
                raise HTTPException(409, "the lease is held")
            arm.token = secrets.token_urlsafe(8)
            arm.lease_id += 1
            arm.holder = body.get("client")
            return {"token": arm.token, "client": arm.holder, "lease_id": arm.lease_id, "heartbeat": 3.0}

        @app.post("/lease/renew")
        def renew(x_lease_token: str = Header(None)):
            check(x_lease_token)
            return {"lease_id": arm.lease_id, "expires_in": 3.0}

        @app.delete("/lease")
        def release(x_lease_token: str = Header(None)):
            check(x_lease_token)
            with arm.lock:
                arm.token, arm.enabled, arm.session = None, False, None
                if arm.state in ("floating", "servoing"):
                    arm.state = "disabled"
            return {"released": True}

        @app.post("/enable")
        def enable(x_lease_token: str = Header(None)):
            check(x_lease_token)
            if arm.estopped:
                raise HTTPException(409, "e-stop is latched")
            with arm.lock:
                arm.enabled, arm.state = True, "floating"
            return {"enabled": True, "state": arm.state}

        @app.post("/disable")
        def disable(x_lease_token: str = Header(None)):
            check(x_lease_token)
            with arm.lock:
                arm.enabled, arm.state, arm.session = False, "disabled", None
            return {"enabled": False}

        @app.post("/estop")
        def estop(body: dict = None):
            with arm.lock:
                arm.estopped = (body or {}).get("reason", "")
                arm.enabled, arm.state = False, "off"
            return {"latched": True, "reason": arm.estopped, "source": "test"}

        @app.get("/tcp")
        def tcp():
            return {"offset": [-0.15, 0.0, 0.0]}

        return app
