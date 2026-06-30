# How This Codebase Draws an Image

A walkthrough of the full pipeline, from image file to physical pen strokes on paper, mapped to the actual code.

---

## Step 1 — Load and resize the image

**File:** `drawbot/ingestion.py`  
**Called from:** `drawbot/pipeline.py` → `build_toolpath()`

`load_image()` reads the file (PNG/JPG) via OpenCV, converts it to grayscale, and optionally downscales it so its longest dimension doesn't exceed `pipeline.ingestion.max_dim` (default 1200px). This caps the point count that flows through the rest of the pipeline and keeps preview/execution time reasonable.

---

## Step 2 — Filter the image into a binary edge map

**File:** `drawbot/stages.py` — `SkeletonFilter` or `CannyFilter`  
**Config key:** `pipeline.filter.name`

The grayscale image is processed into a binary (0/255) array marking only the pixels to draw. Two implementations:

- **`SkeletonFilter`** (current default): thresholds to foreground/background, then runs Zhang-Suen skeletonization (`skimage.morphology.skeletonize`) to thin every stroke region down to a **single-pixel-wide centerline**. Before thinning, an optional `binary_closing` pass (`scipy.ndimage`) heals small 1–3px gaps at junctions that skeletonization would otherwise disconnect (`close_kernel_size: 3` in config).
- **`CannyFilter`**: standard Canny edge detection — finds both edges of every stroke's width, so each line in the original ends up as two parallel edge lines rather than one centerline. Still available, but produces doubled strokes at the vectorizer stage.

Output: a `uint8` numpy array, same spatial dimensions as the loaded image, white pixels = "draw here."

---

## Step 3 — Vectorize the edge map into strokes

**File:** `drawbot/stages.py` — `SknwVectorizer` or `ContourVectorizer`  
**Config key:** `pipeline.vectorizer.name`

White pixels get traced into ordered `Stroke` objects (each stroke = a list of `Point(x, y)` in pixel space).

- **`SknwVectorizer`** (current default): calls `sknw.build_sknw()` to build an explicit **branch graph** from the skeleton — nodes are junctions/endpoints, edges are the pixel-polylines between them. One graph edge → one `Stroke`. At a 3-way junction the graph correctly terminates three separate branches; `ContourVectorizer` would walk the boundary of all three as one tangled contour. Also handles closed loops (a circle with no junction becomes a single self-loop edge → one closed stroke).
- **`ContourVectorizer`**: uses `cv2.findContours`. Correct for Canny-style double-edge maps, but at junctions on skeletonized images it produces crossing or doubled strokes.

Both implementations:
- Apply Douglas-Peucker simplification (`cv2.approxPolyDP`) when `simplify_epsilon > 0`, reducing point count on curves without significant visual loss.
- Drop strokes shorter than `min_stroke_length` (pixels).

Output: `list[Stroke]`, pixel-space coordinates.

---

## Step 3b — Snap nearby endpoints (optional gap closer)

**File:** `drawbot/stages.py` — `snap_nearby_endpoints()`  
**Config key:** `pipeline.snap_threshold_mm`  
**Runs:** immediately after vectorize, before region splitting

`SknwVectorizer` cuts a new stroke at every skeleton graph node. If `SkeletonFilter`'s thinning left even a 1–2px gap at a junction (making two disconnected graph components instead of one), strokes that should visually meet at a joint don't quite touch — showing up as gray pen-up jumps in the toolpath preview.

`snap_nearby_endpoints` fixes this without merging strokes: it clusters every stroke endpoint (across all strokes, regardless of draw order) by real mm distance, then nudges the pixel-space coordinates of each cluster to their centroid. Strokes still exist as separate objects; only their endpoint coordinates are adjusted. `len(strokes)` is unchanged, except for strokes that become zero-length after snapping (dropped). Off by default (`0.0`), opt-in per image.

---

## Step 4 — Order and merge the strokes

**Files:** `drawbot/stages.py` — `GreedyNearestNeighborOrderer`, `TwoOptOrderer`, `merge_near_strokes()`  
**Config keys:** `pipeline.ordering.name`, `pipeline.merge_threshold_mm`

**Ordering:** strokes within each region are sorted to minimize total pen-up travel — `greedy_nn` uses nearest-neighbor heuristic; `two_opt` adds a 2-opt local-search pass on top to untangle long-range crossings. Regions are also ordered by their centroid proximity (`two_opt` region ordering stays greedy). `TwoOptOrderer` has a `two_opt_stroke_limit` (default 200): if the stroke count exceeds the limit, it automatically falls back to `greedy_nn` to avoid O(n²) hangs on complex images.

**Merging:** after ordering, `merge_near_strokes()` walks strokes in sequence and splices any consecutive pair whose endpoints are within `merge_threshold_mm` of each other (real arm-space mm, not pixels) into one continuous CP stream. This eliminates the `pen_up → move_to → pen_down` transition between them — relevant because a near-zero-distance PTP move issued right after CP streaming has been confirmed to stall the Dobot firmware.

**Region splitting** (`GridRegionSplitter`) happens before ordering: the image is divided into an N×M grid and strokes are clipped to region boundaries via Liang-Barsky segment clipping. With `count: 1` (default) there's no clipping — one region = the whole image.

---

## Step 5 — Convert pixel coordinates to real mm coordinates

**File:** `drawbot/mapping.py` — `AffineCoordinateMapper`  
**Config key:** `calibration.points`

At draw time (in `drawbot/execution.py`), each `Point(x, y)` in pixel space is converted to real arm-space mm via a 2D affine transform fit from the 4-corner calibration points. The mapping also applies `center_content` logic (`drawbot/centering.py`): instead of mapping the full image frame, it maps the detected content bounding box — so an off-center source image draws centered on the calibrated paper rectangle regardless of how much whitespace surrounds the subject.

Calibration is a one-time manual step (`python -m drawbot.calibrate`): jog the arm to known paper corners, record the pixel fractions and arm mm coordinates in `config/calibration.generated.yaml`, then paste into `default.yaml`. A safety margin is auto-computed every time the config loads to keep the full drawing rectangle within the arm's confirmed reach limits (`min_reach_mm`/`max_reach_mm`).

---

## Step 6 — Send commands to the arm over USB

**Files:** `drawbot/execution.py` — `ExecutionEngine`, `drawbot/drivers.py` — `AsyncDobotDriver`, `arms/dobot_arm.py` — `DobotArm`

The architecture is split into three layers:

- **`DobotArm`** (`arms/dobot_arm.py`): thin synchronous wrapper around `pydobot`. All serial I/O lives here. Two modes: PTP (point-to-point, used for pen-up moves to stroke starts) and CP (continuous path, used for streaming the actual stroke points).
- **`AsyncDobotDriver`** (`drawbot/drivers.py`): wraps `DobotArm` in an `async`/`await` interface using a single-worker `ThreadPoolExecutor` — pydobot's serial connection isn't thread-safe, so all calls are serialized through one background thread.
- **`ExecutionEngine`** (`drawbot/execution.py`): iterates the `Toolpath` region-by-region, stroke-by-stroke. For each stroke:
  1. `pen_up()` — PTP move lifting pen to `z_up`
  2. `move_to(stroke.start)` — PTP move to stroke's first point
  3. `pen_down()` — PTP move lowering pen to `z_down`, then `_wait_for_settle` polls pose() until arm is physically still
  4. `stream_stroke(arm_points)` — CP stream through all remaining points, in batches of `cp_max_batch_points`

CP batch size matters: a 684-point stroke streamed all at once has been confirmed to silently wedge the firmware queue mid-stroke on this hardware. `cp_max_batch_points: 50` keeps bursts well below that limit.

Before sending to the arm, each stroke's points pass through a filter chain: `_dedup_consecutive` (removes near-duplicate points) → `_clamp_to_reach` (keeps points inside the safe reach envelope) → `_min_segment_filter` (drops segments shorter than `min_segment_mm`) → `_remove_direction_spikes` (drops points causing near-reversals > `max_spike_angle_deg`).

---

## Step 7 — Resilience and completion

**File:** `drawbot/execution.py`

**Retry/backoff (default mode):** if a `DobotStalledError` or `serial.SerialException` is raised mid-stroke, the engine disconnects, waits with exponential backoff, reconnects, clears alarms, and replays the failed stroke from its beginning. Up to `max_retries` attempts before raising `ArmConnectionLost`.

**Skip mode + PTP fallback (`skip_failed_strokes: true`, production default):** on CP wedge, instead of retrying the same CP stroke indefinitely, the engine:
1. Reconnects (fast, no backoff delay).
2. Replays the same stroke via `stream_stroke_ptp` — individual `move_to` calls (MOVJ_XYZ joint interpolation). This sidesteps CP-mode firmware coordinate deadlocks.
3. If PTP attempt 1 fails, reconnects again and tries PTP a second time (some wedges need 2 reconnects to stabilize).
4. Only skips (logs a warning, continues to next stroke) if CP + both PTP attempts all fail.

**Fast-fail on wedge:** `_set_cp_cmd` uses `cp_cmd_timeout_s: null` (50ms read timeout, down from 2s) and `retries=1`. `_set_ptp_cmd` also uses `retries=1`. A wedged firmware is detected in ~50–2000ms instead of the previous 6s per command.

**Ctrl+C handling:** a custom SIGINT handler (`main.py`) calls `stop_queue()` on the first interrupt to clear the firmware's command buffer immediately rather than letting the arm run out its backlog unattended. A second Ctrl+C force-exits via `os._exit()` if the worker thread is wedged in a blocking serial call.

**Parking:** on successful completion, the arm lifts its pen and moves to the calibrated paper center (`Point(0.5, 0.5)` in fraction space, always within the safe reach envelope) as a predictable resting position.

**Dry-run mode:** `--dry-run` substitutes `NoOpDriver` for `AsyncDobotDriver` — all the same pipeline stages run (including coordinate conversion), but no serial port is opened and no commands are sent. Used for previewing the toolpath without hardware attached.

---

## Config file

`config/default.yaml` controls every tunable parameter in the pipeline. The full load/validation path is `drawbot/config_schema.py` → `load_config()`, which resolves each named stage implementation from its registry dict in `stages.py`. Adding a new filter/vectorizer/orderer implementation means registering it in the corresponding `REGISTRY` dict — no schema changes needed.

---

## File map

```
main.py                    — entry point, CLI args, driver wiring, asyncio runner
drawbot/
  ingestion.py             — image load + resize
  stages.py                — SkeletonFilter, CannyFilter, SknwVectorizer, ContourVectorizer,
                             GridRegionSplitter, GreedyNearestNeighborOrderer, TwoOptOrderer,
                             merge_near_strokes, snap_nearby_endpoints
  pipeline.py              — build_toolpath(): orchestrates steps 1-4
  centering.py             — content bbox detection + fraction-space fitting
  mapping.py               — AffineCoordinateMapper (pixel fraction ↔ arm mm)
  config_schema.py         — YAML load/validate, AppConfig dataclass
  drivers.py               — AsyncDobotDriver (async wrapper), NoOpDriver (dry-run)
  execution.py             — ExecutionEngine: steps 5-7, retry/backoff
  models.py                — Point, Stroke, Region, Toolpath, ProgressEvent
  interfaces.py            — abstract base types (ImageFilter, Vectorizer, etc.)
  calibrate.py             — interactive jog-and-lock calibration tool
arms/
  dobot_arm.py             — synchronous pydobot wrapper (serial, PTP, CP)
config/
  default.yaml             — all pipeline parameters
  calibration.generated.yaml — calibration points from last `calibrate.py` run
tests/
  preview_filter.py        — visual: original | edge map | strokes-by-region
  preview_toolpath.py      — visual: draw order, pen-up travel, start/end dots
```
