"""HTTP API of the description node (FastAPI). Conventions: docs/nodes.md; guide: docs/description_node.md."""
from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from typing import Optional

from fastapi import FastAPI, HTTPException, Query, Request
from fastapi.responses import FileResponse, HTMLResponse, Response, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field

from ..common.server import add_base_routes, add_page_routes, node_meta, setup_app, sse
from . import __version__
from .node import DescriptionNode
from .routines import LIMITS, GroundParams, YawParams
from .store import TYPES

TAGS = [
    {"name": "description", "description": "Frames and transforms of the robot."},
    {"name": "calibration", "description": "Calibration routines (jobs), their records and the active ones."},
    {"name": "control", "description": "E-stop and abort (the yaw routine drives the base)."},
    {"name": "state", "description": "Status, clock sync, events."},
]


class NominalIn(BaseModel):
    xyz: list[float] = Field(min_length=3, max_length=3, description="m, position of depth_optical in base (measure x, y)")
    rpy: list[float] = Field(min_length=3, max_length=3, description="rad, fixed-axis X, Y, Z (URDF)")


class GroundIn(BaseModel):
    camera: str = "camera"
    polygons: Optional[list[list[list[float]]]] = Field(
        None, description="Floor regions on the depth image: polygons of [x, y] in normalised image coords "
                          "(0..1, origin top left). Omit for automatic (largest near-horizontal plane, using gravity).")
    frames: int = Field(10, ge=1, le=60, description="Depth frames to take the per-pixel median of.")
    max_tilt_deg: float = Field(30.0, gt=0, le=60)
    tol: float = Field(0.01, gt=0, le=0.1, description="m, inlier distance.")
    activate: bool = True
    model_config = ConfigDict(json_schema_extra={"examples": [
        {"polygons": [[[0.1, 0.7], [0.9, 0.7], [0.9, 0.98], [0.1, 0.98]]]}, {}]})


class YawIn(BaseModel):
    camera: str = "camera"
    vectors: list[list[float]] = Field([[1.0, 0.0], [0.0, 1.0]], description="Base-frame directions [vx, vy] (normalised).")
    amplitude: float = Field(0.08, gt=0, le=LIMITS["amplitude_max"], description="m; the base swings 0..2A along each vector.")
    frequency: float = Field(1.0, ge=LIMITS["frequency_min"], le=LIMITS["frequency_max"], description="Hz")
    cycles: int = Field(5, ge=LIMITS["cycles_min"], le=LIMITS["cycles_max"])
    confirm: bool = Field(False, description="Must be true: this drives the robot.")
    activate: bool = True
    model_config = ConfigDict(json_schema_extra={"examples": [{"confirm": True},
                                                              {"vectors": [[1, 0], [0, 1], [1, 1]], "amplitude": 0.06,
                                                               "frequency": 1.2, "cycles": 6, "confirm": True}]})


class ActivateIn(BaseModel):
    serial: str
    id: Optional[str] = Field(None, description="Calibration id; null = none active.")


class EStopIn(BaseModel):
    reason: str = ""
    source: str = ""


def create_app(node: DescriptionNode, start: bool = True) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app):
        node.hub.attach(asyncio.get_running_loop())
        if start:
            node.start()
        yield
        node.close()

    app = FastAPI(
        title="Robot description node",
        description=(
            "The robot's frames (base, cameras and their internal frames) and their calibration.\n\n"
            "**Frames**: `GET /description` (all frames, each with its parent; camera poses composed from the "
            "nominal pose and the active calibrations), `GET /transform?from=&to=`.\n\n"
            "**Calibration** (jobs; one at a time): `POST /calibrate/ground` (depth plane fit: roll, pitch, height; "
            "never moves), `POST /calibrate/yaw` (**drives the base** back and forth along given vectors: yaw; needs "
            "`confirm: true`, the swerve lease free; aborts on yaw rate/heading drift). Progress in `GET /jobs/{id}` "
            "and `/state`. Records: `GET /calibrations`, `POST /calibrations/{type}/activate`.\n\n"
            "`POST /estop` (anyone) aborts a running job and e-stops the swerve node too."),
        version=__version__, lifespan=lifespan, openapi_tags=TAGS)
    setup_app(app)
    add_base_routes(app, lambda: {"node": "description", "state": node.state()["state"]})
    add_page_routes(app, node_meta("description", "Robot description", __version__, "description", actuated=True,
                                   port_default=8104, moves="swerve base (yaw calibration only)"))

    def bad(e: Exception, code: int = 400):
        return HTTPException(code, str(e))

    @app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def index():
        from .page import PAGE
        return PAGE

    @app.get("/state", tags=["state"], summary="Status, running job, e-stop, cameras, active calibrations")
    async def get_state():
        return node.state()

    @app.get("/events", tags=["state"], summary="Server-Sent Events: /state on every change and every 1 s",
             response_class=StreamingResponse)
    async def events():
        return StreamingResponse(sse(node.hub, node.state), media_type="text/event-stream", headers={"Cache-Control": "no-store"})

    # -- description -----------------------------------------------------------------------------

    @app.get("/description", tags=["description"], summary="All frames, cameras and conventions")
    async def description():
        return await asyncio.to_thread(node.description)

    @app.get("/transform", tags=["description"], summary="Pose of frame `to` in frame `from` (p_from = T p_to)")
    async def transform(frm: str = Query("base", alias="from"), to: str = Query("camera/depth_optical")):
        try:
            return node.transform(frm, to)
        except (KeyError, ValueError) as e:
            raise bad(e)

    @app.post("/cameras/{name}/nominal", tags=["description"], summary="Set a camera's measured/nominal pose in base")
    async def set_nominal(name: str, body: NominalIn):
        """x and y (and anything not calibrated) come from here: measure them. Saved on the robot."""
        try:
            return node.set_nominal(name, body.xyz, body.rpy)
        except KeyError as e:
            raise bad(e, 404)

    @app.get("/cameras/{name}/depth.jpg", tags=["description"], summary="The camera's current depth image (for drawing ground polygons)")
    async def depth_jpg(name: str):
        import httpx
        cams = node.store.description().get("cameras") or {}
        if name not in cams:
            raise bad(KeyError(name), 404)
        async with httpx.AsyncClient(timeout=5.0) as c:
            r = await c.get(cams[name]["url"].rstrip("/") + "/snapshot/depth", params={"format": "jpg"})
        if r.status_code >= 400:
            raise HTTPException(503, f"camera: {r.text[:200]}")
        return Response(r.content, media_type="image/jpeg", headers={"Cache-Control": "no-store",
                                                                       "X-Frame-Header": r.headers.get("X-Frame-Header", "")})

    # -- calibration -----------------------------------------------------------------------------

    @app.post("/calibrate/ground", tags=["calibration"], summary="Ground-plane calibration (never moves): starts a job")
    async def calibrate_ground(body: GroundIn = GroundIn()):
        try:
            return node.start_ground(body.camera, GroundParams(body.polygons, body.frames, body.max_tilt_deg, body.tol),
                                     body.activate)
        except (KeyError, RuntimeError, ValueError) as e:
            raise bad(e, 409 if isinstance(e, RuntimeError) else 400)

    @app.post("/calibrate/yaw", tags=["calibration"], summary="Camera-yaw calibration: DRIVES THE BASE; starts a job")
    async def calibrate_yaw(body: YawIn):
        """Translates the base back and forth along each vector (omega always 0) while recording the
        camera IMU; aborts on yaw rate > 10°/s or heading drift > 5° (the robot may be tethered), lost
        streams, or `POST /estop` / `POST /jobs/abort`. Limits: amplitude ≤ 0.15 m, peak speed
        A·2πf ≤ 0.8 m/s, total travel ≤ 12 m."""
        try:
            return node.start_yaw(body.camera, YawParams(body.vectors, body.amplitude, body.frequency, body.cycles,
                                                         confirm=body.confirm), body.activate)
        except (KeyError, RuntimeError, ValueError) as e:
            raise bad(e, 409 if isinstance(e, RuntimeError) else 400)

    @app.get("/jobs", tags=["calibration"], summary="Recent jobs")
    async def jobs():
        return [j.info() for j in node.jobs.values()]

    @app.get("/jobs/{jid}", tags=["calibration"], summary="One job: state, progress, result")
    async def job(jid: int):
        j = node.jobs.get(jid)
        if j is None:
            raise bad(KeyError(f"no job {jid}"), 404)
        return j.info()

    @app.post("/jobs/abort", tags=["control"], summary="Abort the running job (stops and releases the base)")
    async def abort():
        return node.abort("abort requested") or {"detail": "no job running"}

    @app.get("/calibrations", tags=["calibration"], summary="Calibration records (newest first), active flagged")
    async def calibrations(type: Optional[str] = None, serial: Optional[str] = None):
        return node.store.list(type, serial)

    @app.get("/calibrations/{kind}/{serial}/{cid}", tags=["calibration"], summary="One calibration record (meta.json)")
    async def calibration(kind: str, serial: str, cid: str):
        try:
            return node.store.get(kind, serial, cid)
        except KeyError as e:
            raise bad(e, 404)

    @app.get("/calibrations/{kind}/{serial}/{cid}/raw.npz", tags=["calibration"], summary="Raw data of a record (numpy .npz)")
    async def calibration_raw(kind: str, serial: str, cid: str):
        p = node.store.raw_path(kind, serial, cid)
        if not p.exists():
            raise bad(KeyError("no raw data"), 404)
        return FileResponse(p, media_type="application/octet-stream", filename=f"{kind}-{cid}.npz")

    @app.post("/calibrations/{kind}/activate", tags=["calibration"], summary="Choose the active calibration of a type for a camera")
    async def activate(kind: str, body: ActivateIn):
        if kind not in TYPES:
            raise bad(ValueError(f"type must be one of {TYPES}"))
        try:
            node.activate(kind, body.serial, body.id)
        except KeyError as e:
            raise bad(e, 404)
        return node.store.active()

    # -- e-stop ----------------------------------------------------------------------------------

    @app.post("/estop", tags=["control"], summary="E-stop: aborts any job and e-stops the swerve node (anyone)")
    async def estop(request: Request, body: Optional[EStopIn] = None):
        body = body or EStopIn()
        src = body.source or (request.client.host if request.client else "")
        info = await asyncio.to_thread(node.estop.trigger, body.reason, src)
        return {"latched": True, "reason": info.reason, "source": info.source}

    @app.post("/estop/clear", tags=["control"], summary="Clear this node's e-stop (the swerve node's is separate)")
    async def estop_clear():
        node.estop.clear()
        node.events.add("e-stop cleared")
        node.hub.state_changed()
        return {"latched": False}

    return app
