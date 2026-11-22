"""A fake WBC node for the Quest client's end-to-end test: the frozen contract's HTTP routes (lease, enable,
disable, hints, session), a stream of rows (100 Hz, the contract's columns) and a UDP socket that records every
datagram with its arrival time. Not a controller: the measured EE and the base pose are whatever the test sets."""
import asyncio
import json
import math
import socket
import threading
import time

from fastapi import FastAPI, Header, HTTPException
from fastapi.responses import StreamingResponse

from robot_utils.nodes.common.protocol import pack_record, rows_record, schema_record

EE_COLS = [f"cmd_{k}" for k in ("x", "y", "z", "qx", "qy", "qz", "qw")] + \
          [f"meas_{k}" for k in ("x", "y", "z", "qx", "qy", "qz", "qw")] + \
          [f"twist_{k}" for k in ("vx", "vy", "vz", "wx", "wy", "wz")] + ["gripper"]
FIELDS = ["t_tick", "seq", "state", "base_mode", "solve_ms", "status", "base_x", "base_y", "base_yaw", "est_reset",
          "base_vx", "base_vy", "base_w"] + [f"arm_{c}" for c in EE_COLS] + ["arm_age", "est_age", "target_age"]


class FakeWbc:
    def __init__(self):
        self.calls: list[tuple[str, dict | None]] = []
        self.datagrams: list[tuple[float, dict]] = []
        self.token = "tok-1"
        self.session = 4242
        self.row = {f: 0.0 for f in FIELDS}
        self.row.update(base_x=1.0, base_y=2.0, base_yaw=0.3, arm_meas_x=1.5, arm_meas_y=2.1, arm_meas_z=0.8,
                        arm_meas_qz=math.sin(0.2), arm_meas_qw=math.cos(0.2),
                        arm_cmd_x=1.52, arm_cmd_y=2.1, arm_cmd_z=0.8, arm_cmd_qz=math.sin(0.2),
                        arm_cmd_qw=math.cos(0.2))
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(("127.0.0.1", 0))
        self.udp.settimeout(0.2)
        self.udp_port = self.udp.getsockname()[1]
        self._stop = threading.Event()
        threading.Thread(target=self._recv, daemon=True).start()
        self.app = self._make_app()

    def _recv(self):
        while not self._stop.is_set():
            try:
                data, _ = self.udp.recvfrom(65536)
            except OSError:
                continue
            self.datagrams.append((time.monotonic(), json.loads(data)))

    def close(self):
        self._stop.set()
        self.udp.close()

    def _make_app(self):
        app = FastAPI()
        fake = self

        def check(token):
            if token != fake.token:
                raise HTTPException(403, "wrong token")

        @app.post("/lease")
        async def lease(body: dict):
            fake.calls.append(("lease", body))
            return {"token": fake.token, "client": body.get("client"), "lease_id": 1, "heartbeat": 0.5}

        @app.post("/lease/renew")
        async def renew(x_lease_token: str = Header(None)):
            check(x_lease_token)
            fake.calls.append(("renew", None))
            return {"lease_id": 1, "expires_in": 0.5}

        @app.delete("/lease")
        async def release(x_lease_token: str = Header(None)):
            check(x_lease_token)
            fake.calls.append(("release", None))
            return {"released": True}

        @app.post("/enable")
        async def enable(x_lease_token: str = Header(None)):
            check(x_lease_token)
            fake.calls.append(("enable", None))
            return {"enabled": True}

        @app.post("/disable")
        async def disable(x_lease_token: str = Header(None)):
            check(x_lease_token)
            fake.calls.append(("disable", None))
            return {"enabled": False}

        @app.post("/hints")
        async def hints(body: dict, x_lease_token: str = Header(None)):
            check(x_lease_token)
            fake.calls.append(("hints", body))
            return body

        @app.post("/session")
        async def session(body: dict | None = None, x_lease_token: str = Header(None)):
            check(x_lease_token)
            fake.calls.append(("session", body))
            return {"session": fake.session, "udp_port": fake.udp_port, "unix_socket": "/nonexistent"}

        @app.get("/stream")
        async def stream():
            async def gen():
                yield schema_record(1, FIELDS, [""] * len(FIELDS), node="wbc")
                while not fake._stop.is_set():
                    row = dict(fake.row, t_tick=time.monotonic())
                    yield rows_record([[row[f] for f in FIELDS]], 1)
                    await asyncio.sleep(0.01)
                yield pack_record({"type": "ping"})
            return StreamingResponse(gen(), media_type="application/octet-stream")

        return app

    def names(self):
        return [c[0] for c in self.calls if c[0] != "renew"]
