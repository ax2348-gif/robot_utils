# Orbbec web viewer

`examples/orbbec_web_viewer.py` shows the robot's Orbbec camera (Gemini 336L) live in a browser on
any machine on the network. Run it on the machine the camera is plugged into:

```bash
python examples/orbbec_web_viewer.py            # then open http://<robot>:8080/
python examples/orbbec_web_viewer.py --port 8090 --streams color,depth -v
```

It prints the URLs for each of the machine's addresses. Ctrl-C quits. On Linux the camera needs
Orbbec's udev rule first (see the README; `install.sh` prints the commands).

## What it does

- **Streams:** color, depth, left IR and right IR, shown as tiles; any combination.
- **Profiles:** resolution / fps / format picked from the device's own profile list.
- **Depth display:** colormap, invert, range (auto percentiles or manual min/max mm), invalid-pixel
  color, colorbar; hover reads the depth in mm (pixel and 5×5 median), click pins a point.
- **Depth-to-color alignment:** optional; depth then comes out at the color resolution.
- **Device controls:** exposure, gain, white balance, brightness, laser, shown only when the
  device reports them read/write, with the device's ranges.
- **Readouts and tools:** camera fps, display fps and frame age per stream; a state pill, error
  banner and event log; JPEG snapshots and a 16-bit PNG of the raw depth.

## Design decisions

**No new dependencies.** The server is the standard library's `ThreadingHTTPServer`, and frames
are handled with numpy and Pillow, all already in the robot env. The page is inline HTML/JS with no
CDN, so it works on a network without internet access. If `cv2` is importable (lerobot brings
`opencv-python-headless`) it is used for faster JPEG encoding, more colormaps and YUYV/UYVY color;
without it the viewer still works with Pillow and built-in jet/turbo colormaps.

**One camera thread owns the SDK.** A single thread opens the device, starts the pipeline, waits for
framesets and stores the latest frame of each stream. HTTP handler threads never call the pipeline;
they read the latest frames under a condition variable. This keeps all pipeline restarts and error
handling in one place and avoids calling the SDK from many threads.

**Latest frame wins; no per-client queues.** Each client waits for a frame newer than the last one
it sent. A slow client (a weak Wi-Fi or Tailscale link) skips frames instead of building up a
backlog, so what it sees stays current and the server's memory stays flat.

**Encode once, fan out.** A depth or IR frame is turned into a JPEG at most once, the first time any
client asks for it, and the result is cached by frame number for every other client. Nothing is
encoded for streams nobody is watching. Color arrives from the camera as MJPG and is passed through
unchanged (only checked for a JPEG header), so color costs no CPU on the robot.

**One connection per browser tab.** The page reads all its streams over a single `/api/live`
response: a sequence of `[4-byte length][JSON header][JPEG]` records, read with `fetch` and a stream
reader. One `<img>` per MJPEG stream would open a connection per tile, and browsers allow only 6 per
host, so a few tiles in two tabs would stall. The JSON header carries per-frame data the page needs
(size, format, frame age, depth range used, valid-pixel fraction). A ping every ~2 s with no frames
keeps the connection alive and lets the page notice a dead server. A plain MJPEG URL per stream
(`/stream/<name>.mjpg`) is still there for other tools (VLC, an `<img>` tag, OpenCV).

**Settings are shared, not per viewer.** There is one camera and one pipeline, so stream selection,
profiles and display options are global; every viewer sees the same thing, and the page polls
`/api/state` to pick up changes made by others. Per-viewer display settings would mean encoding each
frame once per viewer.

**Depth and IR share one "Depth / IR" mode.** The SDK refuses to start depth, left IR and right IR
at different resolutions or frame rates ("The depth/ir streams must have the same resolution and
frame rate"), so the page offers one mode for all three instead of letting you pick combinations
that fail.

**Software alignment.** Depth-to-color uses the SDK's `AlignFilter` on the host. Hardware D2C
offered no depth profiles for the default color profile, which is the expected situation for a
camera on USB 2.

**Changes restart cleanly, and bad ones revert.** A change to streams, profiles or alignment stops
and restarts the pipeline. If the device rejects the new configuration, the error is shown and the
previous working settings are restored, so a bad choice never leaves the camera stopped. Display
options (colormap, range) apply to the next frame without a restart.

**The server outlives the camera.** If no frames arrive for 4 s while streaming (8 s after a
restart), or the SDK throws, the camera thread closes everything and tries to reopen the device
every 2 s. The page shows "no device" or "reconnecting" meanwhile and recovers by itself when the
camera is back.

**Binds every interface by default.** `--host 0.0.0.0` so the LAN can reach it, which is the point
of the tool. There is no authentication: anyone who can reach the port can view the camera and
change its settings. Bind to a specific address (`--host`) or rely on the network if that matters.

## SDK notes (pyorbbecsdk2 2.1.3)

- Keep the `Context()` in a variable for as long as the device is used. Chaining off a temporary,
  `Context().query_devices().get_device_by_index(0)`, crashes with "NULL pointer passed for argument
  deviceMgr" once Python frees the context.
- `fs.get_frame(OBFrameType.LEFT_IR_FRAME)` returns a generic `Frame`; call `.as_video_frame()`
  before `get_width()` / `get_height()`.
- `AlignFilter.process(fs)` returns `None` for the first framesets; check before using it.
- `OBIntPropertyRange` has `.current` and `.default_value` (no `.default`).
- A second `Pipeline` on a device that is already streaming fails ("The Color has already been
  started"), so only one process can view the camera at a time.
- Frame `get_system_timestamp_us()` uses the host's wall clock, so frame age is
  `time.time() - timestamp`.

## Known limits

- **USB 2.** The camera reports "USB2.1" on the robot right now. MJPG color works at every
  resolution tried; uncompressed color (RGB, YUYV) starts without an error but delivers no frames and
  slows depth/IR. Use MJPG, or a cable and port that give a USB 3 link.
- **No point clouds yet.**
- Not yet exercised: unplugging and replugging during a session, and the device property controls.
