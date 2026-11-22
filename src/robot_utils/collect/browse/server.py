"""The browser's HTTP API and page (docs/browse.md). Read-only on the data; writes only exports (and the index cache).

Every data route takes ``key`` (a library item: ``.`` for the opened path itself) and Pi-time bounds.

- ``GET /api/library``: the sessions / episodes under the opened path.
- ``GET /api/recording?key=``: streams (kind, fields, frame counts / sizes), episodes, events, gaps, anchors, range.
- ``GET /api/model?key=``: the description and the arm chain (model.py), with where each came from.
- ``GET /api/rows?key&stream&fields=a,b&t0&t1[&stride]``: binary: ``u32 BE`` header length, the JSON header
  ``{n, fields}``, then ``n`` float64 times and ``n * k`` float32 values (row-major; NaN = none).
- ``GET /api/envelope?key&stream&field&t0&t1&buckets``: per bucket ``min`` / ``max`` (an overview).
- ``GET /api/frames?key&stream&t0&t1``: ``{i: [record index], t: [Pi time]}`` of the frames in the window.
- ``GET /api/frame?key&stream&i``: that frame's JPEG bytes (immutable: cached by the browser).
- ``GET /api/export/info``; ``POST /api/export/estimate``; ``POST /api/export`` (a job the page feeds) then
  ``POST /api/export/{id}/frame`` (a JPEG; ``repeat``) and ``POST /api/export/{id}/finish``; ``GET
  /api/export/{id}``; ``GET /api/export/{id}/file``; ``POST /api/export/camera`` (one camera, server-side).
"""
from __future__ import annotations

import json
import struct
from pathlib import Path
from typing import Optional

import numpy as np
from fastapi import Body, FastAPI, HTTPException, Query, Request
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response

from ...nodes.common.server import add_page_routes
from .dataset import Library
from .export import Exporter, Settings, camera_frames, estimate
from .model import arm_chain, load_description, recorded_model
from .page import PAGE_HTML

MAX_ROWS = 400_000


def _nan_list(a) -> list:
    return [None if not np.isfinite(v) else round(float(v), 6) for v in a]


def build_app(lib: Library, exporter: Exporter, description: Optional[str] = None) -> FastAPI:
    app = FastAPI(title="robot_utils episode browser", docs_url="/docs")
    add_page_routes(app, {"name": "browse", "kind": "browse", "title": "Episode browser"})
    model_cache: dict = {}
    chain = {}

    def rec(key: str):
        try:
            return lib.open(key)
        except KeyError:
            raise HTTPException(404, f"no recording {key!r}")
        except ValueError as e:
            raise HTTPException(400, str(e))

    def stream(key: str, name: str):
        r = rec(key)
        s = r.streams.get(name)
        if s is None:
            raise HTTPException(404, f"no stream {name!r} in {key!r}")
        return s

    @app.get("/", response_class=HTMLResponse, include_in_schema=False)
    def page():
        return PAGE_HTML

    @app.get("/api/library")
    def library():
        return {"root": str(lib.path), "items": lib.items()}

    @app.get("/api/recording")
    def recording(key: str = "."):
        r = rec(key)
        return JSONResponse(_clean(r.summary()))

    @app.get("/api/model")
    def model(key: str = "."):
        r = rec(key)
        if key not in model_cache:
            try:
                desc, how = load_description(description, r.path)
            except Exception as e:  # noqa: BLE001
                desc, how = None, f"failed: {type(e).__name__}: {e}"
            model_cache[key] = (desc, how)
        if "c" not in chain:
            chain["c"] = arm_chain()
        desc, how = model_cache[key]
        c, chow = chain["c"]
        _, info = recorded_model(r.path)
        return {"description": desc, "description_source": how, "chain": c, "chain_source": chow,
                "model_version": info.get("model_version"), "model_options": info.get("options")}

    @app.get("/api/rows")
    def rows(key: str, stream_name: str = Query(..., alias="stream"), fields: str = "", t0: float = 0.0,
             t1: float = 0.0, stride: int = 1):
        s = stream(key, stream_name)
        if not hasattr(s, "rows"):
            raise HTTPException(400, f"{stream_name} has no rows")
        names = [f for f in fields.split(",") if f]
        t, v = s.rows(t0, t1, names, max(1, stride))
        if len(t) > MAX_ROWS:
            k = int(np.ceil(len(t) / MAX_ROWS))
            t, v = t[::k], v[::k]
        hdr = json.dumps({"n": int(len(t)), "fields": names}).encode()
        hdr += b" " * ((-(4 + len(hdr))) % 8)          # so the float64 block is aligned for the page's typed arrays
        body = struct.pack(">I", len(hdr)) + hdr
        return Response(body + t.astype("<f8").tobytes() + v.astype("<f4").tobytes(),
                        media_type="application/octet-stream")

    @app.get("/api/envelope")
    def envelope(key: str, field: str, t0: float, t1: float, buckets: int = 600, stream_name: str = Query(..., alias="stream")):
        s = stream(key, stream_name)
        e = s.envelope(t0, t1, field, max(10, min(4000, buckets)))
        return {"t0": t0, "t1": t1, "min": _nan_list(e["min"]), "max": _nan_list(e["max"])}

    @app.get("/api/frames")
    def frames(key: str, t0: float, t1: float, stream_name: str = Query(..., alias="stream")):
        s = stream(key, stream_name)
        ix = s.window(t0, t1, "frame")
        return {"i": ix.tolist(), "t": [round(float(x), 6) for x in np.asarray(s.a["t0"])[ix]]}

    @app.get("/api/frame")
    def frame(key: str, i: int, stream_name: str = Query(..., alias="stream")):
        s = stream(key, stream_name)
        if not (0 <= i < s.n_records):
            raise HTTPException(404, "no such frame")
        return Response(s.payload(i), media_type="image/jpeg",
                        headers={"Cache-Control": "public, max-age=31536000, immutable"})

    # -- export ----------------------------------------------------------------------------------

    @app.get("/api/export/info")
    def export_info():
        return exporter.info()

    @app.post("/api/export/estimate")
    def export_estimate(body: dict = Body(...)):
        s = Settings(**(body.get("settings") or {}))
        return estimate(s, float(body.get("duration") or 1.0), body.get("bpp"))

    @app.post("/api/export")
    def export_create(body: dict = Body(...)):
        try:
            job = exporter.create(Settings(**(body.get("settings") or {})), int(body.get("frames") or 0),
                                  str(body.get("name") or "replay"), bool(body.get("probe")), body.get("title"))
        except RuntimeError as e:
            raise HTTPException(409, str(e))
        return job.public()

    @app.post("/api/export/{jid}/frame")
    async def export_frame(jid: str, request: Request, repeat: int = 1):
        data = await request.body()
        try:
            n = exporter.add_frame(jid, data, repeat)
        except KeyError:
            raise HTTPException(404, "no such export")
        except (RuntimeError, ValueError) as e:
            raise HTTPException(409, str(e))
        return {"frames": n}

    @app.post("/api/export/{jid}/finish")
    def export_finish(jid: str, wait: bool = False):
        try:
            return exporter.finish(jid, wait).public()
        except KeyError:
            raise HTTPException(404, "no such export")

    @app.post("/api/export/{jid}/cancel")
    def export_cancel(jid: str):
        exporter.cancel(jid)
        return {"ok": True}

    @app.get("/api/export/{jid}")
    def export_get(jid: str):
        try:
            return exporter.get(jid).public()
        except KeyError:
            raise HTTPException(404, "no such export")

    @app.get("/api/export/{jid}/file")
    def export_file(jid: str):
        try:
            job = exporter.get(jid)
        except KeyError:
            raise HTTPException(404, "no such export")
        if job.state != "done" or job.probe:
            raise HTTPException(409, f"export is {job.state}")
        return FileResponse(job.out, media_type="video/mp4", filename=job.out.name)

    @app.post("/api/export/camera")
    def export_camera(body: dict = Body(...)):
        r = rec(str(body.get("key") or "."))
        name = body.get("stream")
        s = r.streams.get(name)
        if s is None or getattr(s, "kind", "") != "frames":
            raise HTTPException(404, f"no camera stream {name!r}")
        settings = Settings(**(body.get("settings") or {}))
        t0, t1 = float(body["t0"]), float(body["t1"])
        rate = float(body.get("rate") or 1.0)
        plan = list(camera_frames(s, t0, t1, settings.fps, rate))
        try:
            job = exporter.create(settings, sum(n for _, n in plan), str(body.get("name") or name),
                                  bool(body.get("probe")), body.get("title"))
        except RuntimeError as e:
            raise HTTPException(409, str(e))
        if body.get("probe"):                       # a sample: a few spread-out seconds
            keep = max(1, int(settings.fps * 2))
            step = max(1, len(plan) // 3)
            plan = [p for k in range(0, len(plan), step) for p in plan[k:k + keep // 3 + 1]][:keep]
        for i, n in plan:
            exporter.add_frame(job.id, s.payload(i), n)
        exporter.finish(job.id, wait=bool(body.get("wait")))
        return job.public()

    return app


def _clean(o):
    """JSON-safe: numpy scalars / arrays to Python, NaN and infinities to null, paths to strings."""
    if isinstance(o, dict):
        return {str(k): _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, np.ndarray):
        return _clean(o.tolist())
    if isinstance(o, np.generic):
        o = o.item()
    if isinstance(o, float) and not np.isfinite(o):
        return None
    if isinstance(o, Path):
        return str(o)
    return o
