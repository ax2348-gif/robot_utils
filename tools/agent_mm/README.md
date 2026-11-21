# agent_mm: small tools for agent-driven mobile manipulation episodes

Grown out of the 2026-11-10 pick-and-place trial (`docs/episodes/2026-11-10-pick-place/`). Those episode copies
are kept exactly as they ran; these are the reusable versions.

| file | what |
|---|---|
| `mm.py` | logged robot calls (`nav`/`arm METHOD PATH [JSON]`), arm lease `keeper`, `servo`/`step`/`grip`/`tcp`. Writes into `$MM_RUN` (default: the current folder): `calls.jsonl`, `img/` |
| `record.py` | high-rate client-side recording for the whole episode: arm + estimator `/stream` (raw bytes), camera depth (PNG, 16-bit mm) + colour at `--cam-hz` for each of `--cameras` (default head `camera` → `cam/`, `wrist_camera` → `cam_wrist_camera/`), calibrations. Restarting into the same folder continues it. Read-only |
| `ray.py` | pixel → 3D at a given height from `GET /agent_interface/robot` (approximate pinhole) |

## Per episode

```bash
export MM_RUN=docs/episodes/<date>-<name>; mkdir -p $MM_RUN
python3 tools/agent_mm/record.py $MM_RUN > $MM_RUN/record.log 2>&1 &     # whole episode
python3 tools/agent_mm/mm.py keeper > $MM_RUN/keeper.log 2>&1 &          # only while using the arm
python3 tools/agent_mm/mm.py nav POST look '{"grid":true}'
```

Stop the recorder with `kill -TERM <pid>`. Defaults: arm 50 Hz, estimator 100 Hz, camera 1 Hz, about **18 MB/min**
(camera ≈ 13, arm ≈ 3, estimator ≈ 1.6). Use `--cam-hz 0.5` or `--no-cam` for long runs.

## Reading it back

```python
import sys; sys.path.insert(0, "src")
from robot_utils.nodes.common.protocol import iter_records, iter_rows
for h, rows in iter_rows(iter_records(open(f"{run}/streams/arm.bin", "rb"))):
    rows["t_tick"], rows["j1_q"]          # columns: streams/arm_fields.json
```

**Ego point cloud for frame n:**
1. Back-project `cam/n_depth.png` (mm) with `calibration.json["depth"]` (fx, fy, cx, cy).
2. Move to the colour optical frame with `calibration.json["depth_to_color"]`.
3. Move to the base frame with `base_to_color` from `GET /agent_interface/robot`.
4. Move to the world frame with the estimator pose at `frames.jsonl[n].depth.t_capture` (camera monotonic clock; the
   estimator `t` is on the same Pi monotonic clock, so interpolate `estimator.bin` directly).
