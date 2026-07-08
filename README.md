# Dobot v2 — Image-to-Pen-Plotter Pipeline

Converts any image into physical line drawings using a **Dobot Magician** robotic arm. Loads an image, detects edges, skeletonizes them into ordered strokes, and drives the arm over serial to draw on paper.

---

## How It Works

```
Image
  │
  ├─ Grayscale + resize to MAX_SIDE_PX
  ├─ Contrast enhancement + blur
  ├─ Edge / ridge / ink-mask detection  (script-dependent)
  ├─ Skeletonize (scikit-image) → sknw graph → stroke extraction
  ├─ RDP simplification (cv2.approxPolyDP)
  ├─ Stroke ordering (nearest-neighbour, some scripts add 2-opt + joining)
  ├─ 4-point bilinear calibration (pixel → arm mm + z)
  └─ Draw loop → Dobot Magician via serial
```

---

## Scripts

**`main_cp.py` is the main, actively used script.** The others in this repo are earlier/experimental variants kept for reference and comparison — not part of the primary workflow.

| Script | Role | Motion | Edge detection | Ordering | Notes |
|--------|------|--------|-----------------|----------|-------|
| **`main_cp.py`** | **Main script** | CP (continuous-path) | Canny | Nearest-neighbour (KDTree) | Baseline CP implementation. `PORT = /dev/ttyUSB0` |
| `main_ptp.py` | Older variant | PTP (point-to-point, blocking per segment) | Canny | Nearest-neighbour (KDTree) | Predates CP mode; simplest recovery logic. `PORT = /dev/ttyUSB0` |
| `main_cp_v2.py` | Experimental variant | CP | Global Otsu threshold on ink mask (optional `image_profile.binarize_auto`) | Nearest-neighbour + 2-opt refinement | Adds stroke joining, CP planner params, speed caching, `--verbose` logging. `PORT = /dev/ttyUSB1` |
| `main_cp_v3.py` | Experimental variant | CP | Sato ridge/vesselness filter | Nearest-neighbour + 2-opt refinement | Same ordering/joining/logging machinery as v2, swaps edge detection for a ridge filter (single centerline per stroke, keeps faint strands v2's Otsu would flatten). Auto-fallback to any available USB port. `PORT = /dev/ttyUSB1` |

### Why the variants differ

- **`main_ptp.py`** predates continuous-path support — each segment is a separate blocking PTP move. Slower, but simplest to reason about. Kept for debugging when CP-mode issues are suspected.
- **`main_cp_v2.py`** replaced Canny with a global Otsu threshold on the raw ink mask: Canny traces both edges of a drawn line, which skeletonize then centerlines separately into doubled/parallel strokes. Otsu treats each stroke as one solid blob → one centerline.
- **`main_cp_v3.py`** replaced Otsu with a Sato ridge filter: it detects the stroke centerline directly (like Otsu) but stays grayscale-sensitive, preserving faint/thin strands that Otsu's hard threshold flattens out. Confirmed visually against Canny (`main_cp_v3.py`'s predecessor) and Otsu (v2) on `Tier_4.png` before adopting.
- **v2/v3** both add: 2-opt tour refinement after the greedy nearest-neighbour pass (reduces pen-up travel), stroke joining across zero-gap skeleton junctions (fewer pen lifts), explicit `SET_CP_PARAMS` tuning, alarm-survivable moves (`_move_or_recover`), and `--verbose`/logging support.

---

## Requirements

- Python 3.11+
- [uv](https://docs.astral.sh/uv/) package manager
- Dobot Magician connected via USB (`/dev/ttyUSB0` or `/dev/ttyUSB1`, depending on script)

Install dependencies:

```bash
uv sync
```

Dependencies (from `pyproject.toml`):
- `opencv-python` — image processing
- `scikit-image` — skeletonization, Sato ridge filter (v3)
- `sknw` (from GitHub) — skeleton graph extraction
- `matplotlib` — dry-run preview
- `numpy`
- `scipy` — KDTree stroke ordering (optional, falls back to greedy O(n²) if absent)
- `pyserial`
- `pydobot` (local modified copy at `pydobot/`)

---

## Quick Start

### 1. Calibrate

Run once per paper setup. Hold the arm's unlock button, move it to each position, release, press Enter to record.

```bash
uv run calibrate.py
```

Positions to record (in order):
1. **Z_UP** — pen fully lifted (safe travel height)
2. **TL** — top-left paper corner, pen touching paper
3. **TR** — top-right paper corner, pen touching paper
4. **BL** — bottom-left paper corner, pen touching paper
5. **BR** — bottom-right paper corner, pen touching paper

Calibration auto-updates `CAL_ARM_XY`, `CAL_Z_CORNERS`, and `Z_UP` in `main_cp.py`.
**Note:** each script keeps its own copy of these constants — copy the updated values into any other script you intend to run.

### 2. Set your image

Edit the `IMAGE_PATH` constant at the top of the script you're using:

```python
IMAGE_PATH = "images/Butterfly.jpeg"
```

### 3. Dry run (preview only, no arm movement)

```bash
uv run main_cp.py --dry-run
```

Shows a matplotlib window with the preprocessed image and coloured stroke overlay (v2/v3 show extra panels: despeck/mask stages and dropped strokes).

### 4. Draw

```bash
uv run main_cp.py
```

A preview window opens first. Close it, then press Enter to start drawing.

`main_cp_v2.py` and `main_cp_v3.py` also accept `--verbose` / `-v` to log every serial command to `logs/draw_<timestamp>.log`.

---

## Constants Reference

All tunable parameters are at the top of each script. `main_cp.py` (the main script) shown as baseline; deltas in the variants are noted.

### Image Processing

| Constant | main_cp.py | Effect |
|----------|------------|--------|
| `IMAGE_PATH` | — | Input image |
| `MAX_SIDE_PX` | `800` | Resize longest side to this before processing |
| `CANNY_LOW` | `80` | Canny lower threshold — lower = more edges |
| `CANNY_HIGH` | `200` | Canny upper threshold |
| `BLUR_KERNEL` | `5` | Pre-edge Gaussian blur size (odd number, larger = fewer fine edges) |
| `MORPH_CLOSE_PX` | `0` | Morphological closing radius after edge detection (0 = off, try 2–4 to merge broken edges) |
| `APPROX_EPSILON` | `2.0` | RDP simplification tolerance in pixels (larger = fewer points per stroke) |
| `MIN_STROKE_LEN` | `15` | Drop strokes shorter than this in pixels |

`main_cp_v2.py`/`main_cp_v3.py` drop `CANNY_LOW`/`CANNY_HIGH` (no Canny stage) and add:
- `MIN_STROKE_EXTENT_PX` — bbox-diagonal filter, drops dot-like curls the length filter misses
- `JOIN_TOL_PX` — concatenate consecutive strokes with a gap ≤ this (skips a pen lift)
- `main_cp_v3.py` additionally adds `SATO_SIGMAS`, `CLAHE_CLIP`, `CLAHE_TILE` (ridge-filter tuning)

### Speed

| Constant | main_cp.py | Effect |
|----------|------------|--------|
| `DRAW_SPEED` | `200` | mm/s while drawing (CP firmware cap ~200) |
| `TRAVEL_SPEED` | `300` | mm/s pen-up travel between strokes |
| `Z_SPEED` | `150` | mm/s pen lift/lower |

`main_cp_v2.py`/`main_cp_v3.py` add `CP_DEDUP_EPS_MM` (skips zero-length CP segments — these silently hang the firmware's junction planner), `CP_PLAN_ACC`, `CP_JUNCTION_VEL`, `CP_ACC` (explicit `SET_CP_PARAMS` tuning, unset by default in pydobot), and `PTP_JOINT_VEL`/`PTP_JOINT_ACC` (confirmed-stable joint speed caps — pushing to the firmware spec max trips a motion alarm that hangs the command queue).

### Workspace

| Constant | Default | Effect |
|----------|---------|--------|
| `MIN_REACH_MM` | `170.0` | Inner reach limit — points closer than this are warned |
| `MAX_REACH_MM` | `315.0` | Outer reach limit |
| `PORT` | `/dev/ttyUSB0` (main_cp.py, main_ptp.py) / `/dev/ttyUSB1` (v2, v3) | Serial port for the arm |

### Calibration (set by calibrate.py)

| Constant | Description |
|----------|-------------|
| `CAL_ARM_XY` | 4 corner positions in arm mm: `[TL, TR, BL, BR]` |
| `CAL_Z_CORNERS` | Paper surface z at each corner for bilinear z interpolation |
| `Z_UP` | Pen-lifted z height (safe travel) |

---

## Project Structure

```
Dobot_v2/
├── main_cp.py         # MAIN SCRIPT — CP (continuous-path) draw mode
├── main_ptp.py         # Variant — PTP (point-to-point) draw mode
├── main_cp_v2.py       # Variant — Otsu ink-mask edge detection, 2-opt ordering, stroke joining
├── main_cp_v3.py       # Variant — Sato ridge-filter edge detection, same ordering/joining as v2
├── calibrate.py        # Interactive calibration script
├── versions/            # Older/archived copies of the above scripts
├── pyproject.toml      # uv/pip dependencies
├── Report.md            # Known bugs and reliability issues (future work)
├── images/               # Input images
│   ├── Butterfly.jpeg
│   ├── Tier_1.png … Tier_5.png
│   └── heart_pruned_thin.png
└── pydobot/              # Modified local copy of pydobot library
    └── pydobot/
        ├── dobot.py          # Serial communication, PTP/CP commands
        ├── message.py        # Packet framing and checksum
        └── enums/            # Protocol IDs, PTP modes, control values
```

---

## Calibration Area

The arm's workspace is **trapezoidal** (not rectangular) due to the radial nature of the arm. The 4-point bilinear mapping handles this — corners do not need to be a perfect rectangle on paper.

Safe reach at drawing z (~−57 mm): stay within roughly **180–290 mm** from the arm base. Corners calibrated near the arm's maximum 2D reach (~315 mm) can alarm when the arm cannot simultaneously be fully extended AND pitch downward to reach the paper surface.

---

## Modifications to pydobot

The bundled `pydobot/` is a modified fork of the original [pydobot](https://github.com/luismesas/pydobot). Key changes:

- **`message.py`** — Fixed checksum bug: `% 255` → `% 256`. The original produced checksum=0 for any packet whose payload byte-sum ≡ 1 (mod 256), causing the arm to silently drop those commands with no error.
- **`dobot.py`** — Replaced blocking byte-by-byte serial read with `read_all()` retry loop (up to 1 second). Fixed `wait=True` polling to use `>=` instead of `!=` (prevents infinite loop if arm executes ahead). Raises `RuntimeError` with command ID on timeout instead of crashing with `AttributeError`.
