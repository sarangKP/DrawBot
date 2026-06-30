# How This Codebase Works

A walkthrough of the full pipeline, from image file to physical pen strokes on paper, mapped to the actual code.

---

## Step 1 — Load and resize the image

**Function:** `load_image()` in `main.py` / `main_cp.py`

Reads the file via `cv2.imread(..., IMREAD_GRAYSCALE)`. If the longest dimension exceeds `MAX_SIDE_PX` (default 800), the image is downscaled proportionally with `cv2.INTER_AREA`. Smaller images are left at their native size — no upscaling.

Output: single-channel uint8 numpy array.

---

## Step 2 — Preprocess and detect edges

**Functions:** `preprocess()` → `detect_edges()`

`preprocess()` applies CLAHE (contrast-limited adaptive histogram equalization, 8×8 tile grid) followed by a Gaussian blur of kernel size `BLUR_KERNEL` (default 5). CLAHE boosts local contrast so edges in flat regions survive Canny; the blur removes high-frequency noise that would otherwise produce fragmented strokes.

`detect_edges()` runs `cv2.Canny` with thresholds `CANNY_LOW=80` / `CANNY_HIGH=200`. Optional morphological closing (`MORPH_CLOSE_PX`, default 0) can be enabled to bridge small gaps in broken edges — useful for sketchy or noisy source images.

Output: binary uint8 edge map, same size as the resized image.

---

## Step 3 — Skeletonize and extract strokes

**Function:** `extract_strokes()`

1. **Skeletonize** — `skimage.morphology.skeletonize` thins the edge map to single-pixel-wide centerlines. This collapses the two-pixel-wide Canny edges into one clean line per feature.

2. **Build skeleton graph** — `sknw.build_sknw()` converts the skeleton into a NetworkX graph. Nodes are branch junctions and endpoints; edges carry the pixel-coordinate polyline connecting each pair of nodes.

3. **Per-edge simplification** — for each graph edge, the point sequence (node_u + intermediate pts + node_v) is passed through `cv2.approxPolyDP` (RDP simplification, tolerance `APPROX_EPSILON=2.0` px). This reduces point count on smooth curves without visible loss.

4. **Length filter** — strokes whose total arc length is less than `MIN_STROKE_LEN` (default 15 px) are dropped.

Output: `list[np.ndarray]`, each array shape `(N, 2)` in pixel `(col, row)` space.

---

## Step 4 — Order strokes to minimize pen-up travel

**Function:** `order_strokes()`

Greedy nearest-neighbour heuristic. Starting from the first stroke's end point, at each step the closest unvisited stroke start **or** end is selected. If the nearest point is the stroke's end rather than its start, the stroke is reversed before being appended. This halves average pen-up distance without any expensive optimization.

Output: same list, reordered and possibly some strokes reversed.

---

## Step 5 — Map pixel coordinates to arm space

**Functions:** `_bilinear()`, `px_to_mm()`

Uses a **4-point bilinear interpolation** over the four calibrated paper corners (`CAL_ARM_XY`). This handles the arm's trapezoidal workspace — the paper corners do not need to form a rectangle.

For a pixel at column `c`, row `r` in an image of size `W × H`:

```
u = c / (W - 1)      # 0.0 = left,   1.0 = right
v = r / (H - 1)      # 0.0 = top,    1.0 = bottom

xy = TL*(1-u)*(1-v) + TR*u*(1-v) + BL*(1-u)*v + BR*u*v
z  = same formula over CAL_Z_CORNERS
```

`z` is the paper surface height at that position — calibrated per-corner to compensate for paper tilt and surface irregularity.

Output: `(x_mm, y_mm, z_mm)` in arm Cartesian coordinates.

---

## Step 6 — Draw (PTP mode — main.py)

**Function:** `draw()` in `main.py`

Connects to the arm, clears any alarm state, then for each stroke:

1. **Travel** — `MOVJ_XYZ` PTP move to stroke start XY at `Z_UP` (pen lifted). Joint-space interpolation used here to avoid kinematic alarms near workspace edges.
2. **Pen down** — `MOVJ_XYZ` PTP move to stroke start at paper-surface z. `wait=True` blocks until executed. If the arm alarms (no response within 1 s), the stroke is skipped, alarm is cleared, and execution continues.
3. **Draw** — for each subsequent stroke point, `MOVL_XYZ` PTP `move_to(..., wait=False)` is queued without blocking. Linear Cartesian interpolation produces straight segments between points.
4. **Pen up** — `MOVJ_XYZ` PTP move back to `Z_UP`. `wait=True` drains the queued draw commands before returning (the pen-up is queued after all draw points, so waiting for its execution index means all draw points have also completed).

Speeds: `TRAVEL_SPEED` for air moves, `Z_SPEED` for pen lift/lower, `DRAW_SPEED` for drawing.

---

## Step 6 (alt) — Draw (CP mode — main_cp.py)

**Function:** `draw()` in `main_cp.py`

Same travel and pen-down sequence as PTP mode (MOVJ_XYZ). The drawing phase differs:

- Each subsequent stroke point is sent via `_cp_cmd()` — a raw CP command (protocol ID 91, `SET_CP_CMD`) with `cpMode=1` (absolute coordinates) and `velocity=DRAW_SPEED`.
- CP mode uses firmware look-ahead planning — the arm pre-calculates the motion profile across upcoming points, producing smooth curves rather than point-to-point micro-movements.
- Pen-up is a `MOVJ_XYZ` PTP command queued after all CP commands. `wait=True` waits for its execution index, which implicitly means all preceding CP commands have also completed.

CP mode is faster and smoother than PTP for curves, but requires firmware version with CP support.

---

## Serial Protocol

Commands use the Dobot binary protocol over UART at 115200 baud.

Each packet: `AA AA | len | id | ctrl | params... | checksum`

- `len` = number of bytes from `id` to end of `params` (inclusive)
- `ctrl=0x01` = immediate (executed at once), `ctrl=0x03` = queued (appended to motion queue)
- Checksum = `(256 - (id + ctrl + Σparams) % 256) % 256`

**Important fix:** the original pydobot had `% 255` instead of `% 256` in the checksum. This produced checksum=0 for any packet whose payload byte-sum ≡ 1 (mod 256), causing the arm to silently drop those packets. Fixed in `pydobot/pydobot/message.py`.

For queued commands, the arm responds immediately with a queued execution index. `wait=True` polls `GET_QUEUED_CMD_CURRENT_INDEX` (ID 246) until the returned index `>=` the expected index.

---

## Calibration

**Script:** `calibrate.py`

Interactive: hold the arm's unlock button to move it freely, position at each of 5 points, release, press Enter. Records `arm.pose()` (x, y, z) at each position.

The 5 points:
- `Z_UP` — pen-lifted safe travel height
- `TL`, `TR`, `BL`, `BR` — paper corners with pen just touching the surface

On completion, patches `main.py` in-place via regex to update `CAL_ARM_XY`, `CAL_Z_CORNERS`, and `Z_UP`.

**Note:** `main_cp.py` must be updated manually after calibration (see H-3 in `Report.md`).

---

## File Map

```
main.py          — full pipeline + PTP draw mode
main_cp.py       — full pipeline + CP draw mode (smooth curves, higher speed)
calibrate.py     — interactive calibration, patches main.py on completion
Report.md        — known bugs and reliability issues
README.md        — setup, quick start, constants reference
images/          — input images
pydobot/         — modified local pydobot library
  pydobot/
    dobot.py     — Dobot class: serial connection, PTP, CP, wait loop
    message.py   — packet framing and checksum (checksum bug fixed here)
    enums/       — protocol IDs, PTP mode values, control values
```
