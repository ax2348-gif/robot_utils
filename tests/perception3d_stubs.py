"""Peers for the 3d_perception tests: the fake camera node (a fixture), a stub estimator and a stub description
node (apps to serve with helpers.serve)."""
import asyncio
import threading
import time
from contextlib import asynccontextmanager

import numpy as np
import pytest
from fastapi import FastAPI

from helpers import serve, stop
from robot_utils.nodes.common.hub import RowHub
from robot_utils.nodes.common.server import add_base_routes, rows_stream


@pytest.fixture
def camera(tmp_path):
    from robot_utils.nodes.orbbec.fake_backend import FakeBackend
    from robot_utils.nodes.orbbec.hub import Hub
    from robot_utils.nodes.orbbec.models import Settings
    from robot_utils.nodes.orbbec.owner import Owner
    from robot_utils.nodes.orbbec.server import create_app as camera_app
    hub = Hub()
    url, srv, t = serve(camera_app(Owner(FakeBackend(), hub, Settings(streams={"color": True, "depth": True}),
                                         startup_s=1.5, calibration_file=str(tmp_path / "imu.json")), hub))
    yield url
    stop((srv, t))


def stub_estimator():
    """Serves /time and a /stream of poses (fields read by name; reset_id last, like the estimator)."""
    fields = ["t", "x", "y", "theta", "vx", "reset_id"]
    hub = RowHub({"pose": len(fields)})
    halt = threading.Event()

    def run():
        while not halt.is_set():
            t = time.monotonic()
            hub.publish_rows(np.array([[t, 0.2 * t, 0.0, 0.0, 0.2, 1.0]]), "pose")
            halt.wait(0.01)

    @asynccontextmanager
    async def lifespan(app):
        hub.attach(asyncio.get_running_loop())
        th = threading.Thread(target=run, daemon=True)
        th.start()
        yield
        halt.set()

    app = FastAPI(lifespan=lifespan)
    add_base_routes(app, lambda: {})

    @app.get("/stream")
    async def stream():
        from fastapi.responses import StreamingResponse
        return StreamingResponse(rows_stream(hub, fields, ["s", "m", "m", "rad", "m/s", ""], channel="pose", kind="pose"),
                                 media_type="application/octet-stream")
    return app


def stub_description(calibrated=True):
    app = FastAPI()
    add_base_routes(app, lambda: {})
    T = [[0, -0.38, 0.92, 0.1], [-1, 0, 0, 0], [0, -0.92, -0.38, 1.1], [0, 0, 0, 1]]

    @app.get("/description")
    async def description():
        src = {"nominal": True, **({"ground": "20261108-000000"} if calibrated else {})}
        return {"cameras": {"camera": {"node": "camera", "serial": "FAKE0001",
                                       "pose_in_base": {"matrix": T}, "from": src}}}
    return app
