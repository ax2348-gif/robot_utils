"""FastAPI app. Runs on the event loop only: it never calls the SDK and never blocks.

Frames come from the Hub (latest-frame-wins), encodes run in a small thread pool (once per frame
and mode, shared by every client), commands go to the owner thread as futures.
"""
from __future__ import annotations

import asyncio
import json
import logging
import time
from concurrent.futures import ThreadPoolExecutor
from contextlib import asynccontextmanager
from typing import Literal, Optional

import numpy as np
from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, Response, StreamingResponse

from . import __version__
from .encode import encode_frame
from .hub import Frame, Hub
from .models import (STREAM_NAMES, Catalog, ErrorResponse, PropertyInfo, PropertySet, Settings,
                     SettingsError, SettingsPatch, SettingsResult, State, TimeResponse,
                     ImuCalibrateRequest)
from .owner import DeviceError, Owner
from .imu_filter import STATE_FIELDS
from .protocol import IMU_FIELDS, imu_payload, pack_record

log = logging.getLogger("orbbec_node.api")

PING_S = 1.0

DESCRIPTION = """
HTTP node for the robot's **Orbbec Gemini 336L** (color, depth, left/right IR, IMU).
One process owns the camera; settings are global (one pipeline shared by all clients).
Full guide with curl / Python / JavaScript examples: `docs/orbbec_node.md` in robot_utils.

### Streaming (`GET /stream`, `GET /imu/stream`)
Long-lived responses; each record is
`[u32 big-endian header length][JSON header][payload of header.len bytes]`.
Video is **latest-frame-wins** per client (a slow client skips frames, `dropped` counts them);
IMU sends **every sample** in small batches. `mode=jpeg` is for monitoring (depth colormapped),
`mode=raw` is lossless (depth uint16 × `depth_scale` = mm; IR as captured; color = the camera's own
MJPEG bytes).

### Timestamps
All `t_*` values are seconds on the **server's `time.monotonic()`** clock:
`t_sensor` (device timestamp mapped to host by the SDK ≈ capture), `t_capture` (SDK received the
frame), `t_owner`, `t_publish`, `t_encoded`, `t_send`. Synchronise a client clock with
`GET /time` (NTP-style: offset = ts − (t0+t1)/2, uncertainty = RTT/2).
"""

TAGS = [
    {"name": "stream", "description": "Binary frame / IMU streams and single frames."},
    {"name": "control", "description": "Settings and device properties (global)."},
    {"name": "state", "description": "Status, catalog, time sync."},
]


class EncodeCache:
    """One encode per (stream, mode, compression) and frame, shared by all clients."""

    def __init__(self, executor, owner: Owner):
        self.executor = executor
        self.owner = owner
        self.entries: dict = {}

    def ensure(self, f: Frame, mode: str, compress: bool) -> asyncio.Future:
        loop = asyncio.get_running_loop()
        if f.fmt == "MJPG" or (mode == "raw" and not compress):   # no work: pass through / zero copy
            fut = loop.create_future()
            payload, extra = encode_frame(f, mode, None, False)
            fut.set_result((payload, extra, f.t_publish))
            return fut
        disp = self.owner.settings.display if mode == "jpeg" else None
        key = (f.stream, mode, compress)
        ent = self.entries.get(key)
        if ent is None or ent[0] is not f or ent[1] is not disp:
            fut = loop.run_in_executor(self.executor, _encode, f, mode, disp, compress)
            ent = self.entries[key] = (f, disp, fut)
        return ent[2]


def _encode(f, mode, disp, compress):
    payload, extra = encode_frame(f, mode, disp, compress)
    return payload, extra, time.monotonic()


def _parse_streams(streams: str, mode: str) -> dict[str, str]:
    out = {}
    for item in filter(None, (s.strip() for s in streams.split(","))):
        name, _, m = item.partition(":")
        if name not in STREAM_NAMES:
            raise HTTPException(400, f"unknown stream {name!r}; one of {list(STREAM_NAMES)}")
        m = m or mode
        if m not in ("jpeg", "raw"):
            raise HTTPException(400, f"unknown mode {m!r} (jpeg or raw)")
        out[name] = m
    if not out:
        raise HTTPException(400, "no streams given")
    return out


def _err(e: Exception) -> HTTPException:
    if isinstance(e, HTTPException):
        return e
    if isinstance(e, SettingsError):
        return HTTPException(409 if "restored the previous" in str(e) else 400, str(e))
    if isinstance(e, DeviceError):
        return HTTPException(503, str(e))
    if isinstance(e, (ValueError, TypeError)):
        return HTTPException(400, str(e))
    return HTTPException(500, f"{type(e).__name__}: {e}")


def create_app(owner: Owner, hub: Hub, encode_workers: int = 3) -> FastAPI:
    executor = ThreadPoolExecutor(encode_workers, thread_name_prefix="encode")
    cache = EncodeCache(executor, owner)

    @asynccontextmanager
    async def lifespan(app):
        hub.attach(asyncio.get_running_loop())
        owner.start()
        yield
        await asyncio.get_running_loop().run_in_executor(None, owner.close)
        executor.shutdown(wait=False)

    app = FastAPI(title="Orbbec camera node", version=__version__, description=DESCRIPTION,
                  lifespan=lifespan, openapi_tags=TAGS)
    app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"],
                       expose_headers=["X-Frame-Header", "X-Depth-Scale"])
    app.state.owner, app.state.hub = owner, hub
    errors = {400: {"model": ErrorResponse}, 409: {"model": ErrorResponse}, 503: {"model": ErrorResponse}}

    async def run_cmd(fut, timeout=20.0):
        try:
            return await asyncio.wait_for(asyncio.wrap_future(fut), timeout)
        except asyncio.TimeoutError:
            raise HTTPException(504, "the camera thread did not answer in time") from None
        except Exception as e:
            raise _err(e) from None

    # -- page / state ------------------------------------------------------------------------

    @app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def index():
        from .page import PAGE
        return HTMLResponse(PAGE, headers={"Cache-Control": "no-store"})

    @app.get("/time", response_model=TimeResponse, tags=["state"],
             summary="Server clocks for NTP-style sync")
    async def get_time():
        """Read `t0` on your clock, call this, read `t1`. Then `offset = mono - (t0+t1)/2` maps your
        clock to the server's (`t_server = t_client + offset`) within ±(t1-t0)/2. Repeat ~16 times
        over a kept-alive connection and keep the smallest round trips (see `client.py`)."""
        m = time.monotonic()
        w = time.time()
        return {"mono": m, "wall": w, "epoch_minus_mono": w - m}

    @app.get("/health", tags=["state"], summary="Liveness")
    async def health():
        return {"ok": True, "state": owner.state}

    @app.get("/state", response_model=State, tags=["state"], summary="Node, device, stream and IMU status")
    async def get_state():
        return owner.status()

    @app.get("/events", tags=["state"], summary="Server-Sent Events: state on every change (and every 1 s)",
             response_class=StreamingResponse,
             responses={200: {"content": {"text/event-stream": {}}, "description": "`event: state` records, "
                              "data = the same JSON as GET /state"}})
    async def events():
        async def gen():
            while True:
                fut = hub.state_future()
                st = State.model_validate(owner.status())
                yield f"event: state\ndata: {st.model_dump_json()}\n\n"
                await asyncio.wait({fut}, timeout=PING_S)
        return StreamingResponse(gen(), media_type="text/event-stream", headers={"Cache-Control": "no-store"})

    @app.get("/catalog", response_model=Catalog, tags=["state"],
             summary="Profiles, stereo modes, IMU rates and colormaps the device offers")
    async def get_catalog():
        return owner.catalog

    # -- settings / properties ---------------------------------------------------------------

    @app.get("/settings", response_model=Settings, tags=["control"], summary="Current settings")
    async def get_settings():
        return owner.settings

    @app.post("/settings", response_model=SettingsResult, tags=["control"], responses=errors,
              summary="Change settings (partial update)")
    async def post_settings(patch: SettingsPatch):
        """Only the given fields change. Stream/profile/alignment/frame-sync changes restart the video
        pipeline and the call returns once **every enabled stream has delivered a frame** with the new
        configuration (typically 0.3-1 s). A configuration the device refuses, or one that starts but
        stays silent for 8 s (uncompressed color on USB 2), returns **409** and the previous settings
        are restored.
        Display (`display.*`) and `imu.batch_ms` changes apply to the next frame without a restart."""
        try:
            fut = owner.request_settings(patch)
        except Exception as e:
            raise _err(e) from None
        return await run_cmd(fut)

    @app.get("/properties", response_model=list[PropertyInfo], tags=["control"], responses=errors,
             summary="Device properties (exposure, gain, laser, ...) the device reports read/write")
    async def get_properties(all: bool = Query(False, description="Also list every other read/write property "
                                                                   "the device reports.")):
        return await run_cmd(owner.submit("properties", all))

    @app.post("/properties", response_model=list[PropertyInfo], tags=["control"], responses=errors,
              summary="Set one device property; returns the updated list")
    async def post_property(body: PropertySet):
        """Some values are only writable in some states (e.g. exposure needs auto exposure off); the
        device's refusal comes back as a 500 with its message."""
        return await run_cmd(owner.submit("set_property", (body.id, body.value)))

    # -- streams -----------------------------------------------------------------------------

    async def frame_records(modes: dict[str, str], compress: bool):
        hub.clients += 1
        hub.state_changed()
        last: dict[str, int] = {}
        try:
            while True:
                fut = hub.frames_changed()
                latest = hub.latest(modes)
                todo = []
                for n, m in modes.items():
                    f = latest.get(n)
                    if f is not None and f.seq > last.get(n, 0):
                        dropped = f.seq - last[n] - 1 if n in last else 0
                        last[n] = f.seq
                        todo.append((f, dropped, cache.ensure(f, m, compress)))
                for f, dropped, enc in todo:     # encodes of all streams run in parallel
                    try:
                        payload, extra, t_enc = await asyncio.shield(enc)   # shared: never cancel
                    except Exception as e:
                        log.warning("encode %s failed: %s", f.stream, e)
                        continue
                    h = f.header()
                    h.update(extra)
                    h["t_encoded"] = t_enc
                    h["dropped"] = dropped
                    h["t_send"] = time.monotonic()
                    yield pack_record(h, payload)
                if not todo:
                    done, _ = await asyncio.wait({fut}, timeout=PING_S)
                    if not done:
                        yield pack_record({"type": "ping", "t_send": time.monotonic()})
        finally:
            hub.clients -= 1
            hub.state_changed()

    stream_headers = {"Cache-Control": "no-store", "X-Accel-Buffering": "no"}

    @app.get("/stream", tags=["stream"], response_class=StreamingResponse,
             summary="Live frames: one long-lived binary response for any set of streams",
             responses={200: {"content": {"application/octet-stream": {}},
                              "description": "Records `[u32 BE header length][JSON header][payload]` "
                                             "(see the API description / docs)."}, 400: {"model": ErrorResponse}})
    async def stream(
            streams: str = Query("color,depth", description="Comma-separated `color, depth, ir_left, ir_right`; "
                                 "a `:jpeg` / `:raw` suffix overrides `mode` per stream, e.g. `color,depth:raw`.",
                                 examples=["color,depth", "depth:raw,color", "color,depth,ir_left,ir_right"]),
            mode: Literal["jpeg", "raw"] = Query("jpeg", description="`jpeg`: monitoring (depth colormapped, IR "
                                                 "8-bit JPEG). `raw`: lossless (depth uint16, IR Y8/Y16). Color "
                                                 "is always the camera's MJPEG bytes."),
            compress: Literal["none", "zlib"] = Query("none", description="zlib (level 1) on raw payloads; "
                                                      "about 2-3x smaller depth for slow links, costs Pi CPU.")):
        modes = _parse_streams(streams, mode)
        return StreamingResponse(frame_records(modes, compress == "zlib"),
                                 media_type="application/octet-stream", headers=stream_headers)

    async def latest_frame(name: str, timeout: float = 3.0) -> Frame:
        if name not in STREAM_NAMES:
            raise HTTPException(404, f"unknown stream {name!r}")
        deadline = time.monotonic() + timeout
        while True:
            fut = hub.frames_changed()
            f = hub.latest([name]).get(name)
            if f is not None:
                return f
            left = deadline - time.monotonic()
            if left <= 0:
                raise HTTPException(503, f"no {name} frame (stream off or camera not streaming)")
            await asyncio.wait({fut}, timeout=left)

    @app.get("/frame/{name}", tags=["stream"], summary="The latest frame of one stream as a single record",
             responses={200: {"content": {"application/octet-stream": {}}}, 503: {"model": ErrorResponse}})
    async def frame(name: str, mode: Literal["jpeg", "raw"] = "raw", compress: Literal["none", "zlib"] = "none"):
        """Same record format as `/stream` (one record). Handy for scripts and curl."""
        f = await latest_frame(name)
        payload, extra, t_enc = await asyncio.shield(cache.ensure(f, mode, compress == "zlib"))
        h = {**f.header(), **extra, "t_encoded": t_enc, "dropped": 0, "t_send": time.monotonic()}
        return Response(pack_record(h, payload), media_type="application/octet-stream",
                        headers={"Cache-Control": "no-store"})

    @app.get("/snapshot/{name}", tags=["stream"], summary="The latest frame as an image file",
             responses={200: {"content": {"image/jpeg": {}, "image/png": {}}}, 503: {"model": ErrorResponse}})
    async def snapshot(name: str, format: Literal["jpg", "png"] = Query(
            "jpg", description="`jpg`: monitoring JPEG (color: camera MJPEG). `png`: lossless 16-bit PNG of "
                               "depth (units: X-Depth-Scale mm) or IR.")):
        """The JSON frame header is in the `X-Frame-Header` response header."""
        f = await latest_frame(name)
        if format == "png":
            if f.fmt not in ("Y16", "Y8"):
                raise HTTPException(400, f"png needs a depth or IR stream, {name} is {f.fmt}")
            body = await asyncio.get_running_loop().run_in_executor(executor, _png, f)
            ctype, extra = "image/png", {"encoding": "png"}
        else:
            body, extra, _ = await asyncio.shield(cache.ensure(f, "jpeg", False))
            body, ctype = bytes(body), "image/jpeg"
        hdr = {**f.header(), **extra}
        headers = {"Cache-Control": "no-store", "X-Frame-Header": json.dumps(hdr, separators=(",", ":")),
                   "Content-Disposition": f'inline; filename="{name}_{f.seq}.{format}"'}
        if f.stream == "depth":
            headers["X-Depth-Scale"] = str(f.depth_scale)
        return Response(body, media_type=ctype, headers=headers)

    @app.get("/mjpeg/{name}", tags=["stream"], summary="Plain multipart MJPEG of one stream (for <img>, VLC, OpenCV)",
             response_class=StreamingResponse)
    async def mjpeg(name: str):
        if name not in STREAM_NAMES:
            raise HTTPException(404, f"unknown stream {name!r}")

        async def gen():
            async for rec in frame_records({name: "jpeg"}, False):
                n = int.from_bytes(rec[:4], "big")
                h = json.loads(rec[4:4 + n])
                if h["type"] != "frame":
                    continue
                body = rec[4 + n:]
                yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(body) + body + b"\r\n"
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame", headers=stream_headers)

    @app.get("/depth_at", tags=["stream"], summary="Depth (mm) at a point of the latest depth frame")
    async def depth_at(x: float = Query(0.5, ge=0, le=1, description="Fraction of the width."),
                       y: float = Query(0.5, ge=0, le=1, description="Fraction of the height.")):
        f = hub.latest(["depth"]).get("depth")
        if f is None:
            return {"mm": None}
        u = min(max(int(x * f.w), 0), f.w - 1)
        v = min(max(int(y * f.h), 0), f.h - 1)
        win = f.data[max(v - 2, 0):v + 3, max(u - 2, 0):u + 3]
        win = win[win > 0]
        import numpy as np
        return {"u": u, "v": v, "seq": f.seq, "mm": round(float(f.data[v, u]) * f.depth_scale, 1) or None,
                "median5": round(float(np.median(win)) * f.depth_scale, 1) if win.size else None}

    # -- IMU ---------------------------------------------------------------------------------

    imu_units = {"t_capture": "s (server monotonic)", "t_sensor": "s (server monotonic)", "t_device_us": "us",
                 "ax": "m/s^2", "ay": "m/s^2", "az": "m/s^2", "gx": "rad/s", "gy": "rad/s", "gz": "rad/s",
                 "temperature": "degC"}

    state_units = {"t_start": "s (server monotonic, sensor time)", "t_end": "s (server monotonic, sensor time)",
                   "t_capture": "s (server monotonic)", "q*": "unit quaternion IMU->world (w, x, y, z), world z up",
                   "da*": "rad (delta angle over the record, IMU frame)",
                   "dv*": "m/s (delta velocity over the record, IMU frame, incl. gravity)",
                   "a*": "m/s^2 (mean)", "g*": "rad/s (mean, bias-corrected)", "bg*": "rad/s (gyro bias estimate)",
                   "flags": "bits: 1 calibrated, 2 still, 4 initializing, 8 sample gap",
                   "accel_used": "fraction of samples whose accel corrected the attitude", "temperature": "degC"}

    @app.get("/imu/stream", tags=["stream"], response_class=StreamingResponse,
             summary="IMU: filtered orientation records (default) or every raw sample (`raw=true`)",
             responses={200: {"content": {"application/octet-stream": {}},
                              "description": "Records with header type `imu`; payload float64 (n, len(fields)) "
                                             "little endian, columns = header.fields. `header.kind` is `state` "
                                             "(filter output, STATE_FIELDS) or `raw` (IMU_FIELDS)."}})
    async def imu_stream(raw: bool = Query(False, description="Every raw 1 kHz sample instead of the filter "
                                                              "output (default: ~100 Hz orientation records)."),
                         batch_ms: float = Query(0.0, ge=0, le=500,
                                                 description="0: send each record as soon as it exists. >0: wait "
                                                             "this long after the first pending one and send what "
                                                             "accumulated as one record (fewer, larger records)."),
                         max_backlog: int = Query(20000, ge=100, le=1_000_000,
                                                  description="Rows buffered for a slow client before the "
                                                              "oldest are dropped (and counted in `dropped`).")):
        channel = "raw" if raw else "state"
        fields, units = (IMU_FIELDS, imu_units) if raw else (STATE_FIELDS, state_units)
        sub = hub.subscribe_imu(max_backlog, channel)

        async def gen():
            hub.clients += 1
            try:
                while True:
                    sub.event.clear()
                    rows, dropped = sub.take()
                    if rows is None:
                        try:
                            await asyncio.wait_for(sub.event.wait(), PING_S)
                        except asyncio.TimeoutError:
                            yield pack_record({"type": "ping", "t_send": time.monotonic()})
                        continue
                    if batch_ms > 0:
                        await asyncio.sleep(batch_ms / 1000)
                        more, d2 = sub.take()
                        if more is not None:
                            rows, dropped = np.concatenate([rows, more]), dropped + d2
                    h = {"type": "imu", "kind": channel, "n": len(rows), "fields": fields, "units": units,
                         "dtype": "<f8", "seq_first": int(rows[0, 0]), "seq_last": int(rows[-1, 0]),
                         "dropped": dropped, "t_send": time.monotonic()}
                    yield pack_record(h, imu_payload(rows))
            finally:
                hub.unsubscribe_imu(sub)
                hub.clients -= 1
        return StreamingResponse(gen(), media_type="application/octet-stream", headers=stream_headers)

    @app.get("/imu/latest", tags=["stream"], summary="The most recent IMU records as JSON")
    async def imu_latest(n: int = Query(100, ge=1, le=5000),
                         raw: bool = Query(False, description="Raw samples instead of filter records.")):
        rows = hub.recent_imu(n, "raw" if raw else "state")
        return {"kind": "raw" if raw else "state", "fields": IMU_FIELDS if raw else STATE_FIELDS,
                "units": imu_units if raw else state_units,
                "samples": [[None if x != x else float(x) for x in r] for r in rows]}

    @app.post("/imu/calibrate", tags=["control"], responses=errors,
              summary="Calibrate gyro bias and gravity from the next few seconds of still data")
    async def imu_calibrate(req: ImuCalibrateRequest = ImuCalibrateRequest()):
        """Keep the camera still. Returns the new calibration (also saved on the Pi and used as the
        starting point next time). 400 if the IMU moved during it; the previous calibration stays."""
        fut = owner.request_imu_calibration(req.seconds)
        return await run_cmd(fut, timeout=(req.seconds or 1.0) + 10.0)

    @app.post("/imu/reset", tags=["control"], summary="Restart the orientation from gravity (yaw 0)")
    async def imu_reset():
        """Roll/pitch snap to the current gravity direction, yaw becomes 0; the calibration is kept."""
        owner.request_imu_reset()
        return {"ok": True}

    return app


def _png(f: Frame) -> bytes:
    import numpy as np
    img = f.data if f.data.ndim == 2 else (f.data.view(np.uint16) if f.fmt == "Y16" else f.data).reshape(f.h, f.w)
    try:
        import cv2
    except ImportError:
        import io
        from PIL import Image
        out = io.BytesIO()
        Image.fromarray(img).save(out, "PNG")
        return out.getvalue()
    ok, buf = cv2.imencode(".png", img)
    if not ok:
        raise RuntimeError("PNG encode failed")
    return buf.tobytes()
