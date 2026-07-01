# Dobot v2 — Image-to-Pen-Plotter Pipeline

Converts any image into physical line drawings using a **Dobot Magician** robotic arm. Loads an image, detects edges, skeletonizes them into ordered strokes, and drives the arm over serial to draw on paper.

---

## How It Works

```
Image
  │
  ├─ Grayscale + resize to MAX_SIDE_PX
  ├─ CLAHE contrast enhancement + Gaussian blur
  ├─ Canny edge detection
  ├─ Skeletonize (scikit-image) → sknw graph → stroke extraction
  ├─ RDP simplification (cv2.approxPolyDP)
  ├─ Nearest-neighbour stroke ordering (minimise pen-up travel)
  ├─ 4-point bilinear calibration (pixel → arm mm + z)
  └─ Draw loop → Dobot Magician via serial
```

Two draw modes:

| Script | Mode | Motion | Best for |
|--------|------|--------|----------|
| `main.py` | PTP | Point-to-point, each segment separate | Reliability, debugging |
| `main_cp.py` | CP | Continuous-path, look-ahead smoothing | Speed, smooth curves |

---

## Requirements

- Python 3.11+
- [uv](https://docs.astral.sh/uv/) package manager
- Dobot Magician connected via USB (`/dev/ttyUSB0`)

Install dependencies:

```bash
uv sync
```

Dependencies (from `pyproject.toml`):
- `opencv-python` — image processing
- `scikit-image` — skeletonization
- `sknw` (from GitHub) — skeleton graph extraction
- `matplotlib` — dry-run preview
- `numpy`
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

Calibration auto-updates `CAL_ARM_XY`, `CAL_Z_CORNERS`, and `Z_UP` in `main.py`.
**Note:** also manually copy the updated constants into `main_cp.py` until H-3 from `Report.md` is fixed.

### 2. Set your image

Edit the `IMAGE_PATH` constant at the top of `main.py` or `main_cp.py`:

```python
IMAGE_PATH = "images/Butterfly.jpeg"
```

### 3. Dry run (preview only, no arm movement)

```bash
uv run main.py --dry-run
# or
uv run main_cp.py --dry-run
```

Shows a matplotlib window with the preprocessed image and coloured stroke overlay.

### 4. Draw

```bash
uv run main.py
# or (faster, smoother)
uv run main_cp.py
```

A preview window opens first. Close it, then press Enter to start drawing.

---

## Constants Reference

All tunable parameters are at the top of each script.

### Image Processing

| Constant | Default | Effect |
|----------|---------|--------|
| `IMAGE_PATH` | `"images/Butterfly.jpeg"` | Input image |
| `MAX_SIDE_PX` | `800` | Resize longest side to this before processing |
| `CANNY_LOW` | `80` | Canny lower threshold — lower = more edges |
| `CANNY_HIGH` | `200` | Canny upper threshold |
| `BLUR_KERNEL` | `5` | Pre-Canny Gaussian blur size (odd number, larger = fewer fine edges) |
| `MORPH_CLOSE_PX` | `0` | Morphological closing radius after Canny (0 = off, try 2–4 to merge broken edges) |
| `APPROX_EPSILON` | `2.0` | RDP simplification tolerance in pixels (larger = fewer points per stroke) |
| `MIN_STROKE_LEN` | `15` | Drop strokes shorter than this in pixels |

### Speed

| Constant | Default | Effect |
|----------|---------|--------|
| `DRAW_SPEED` | `200` | mm/s while drawing (CP firmware cap ~200) |
| `TRAVEL_SPEED` | `300` | mm/s pen-up travel between strokes |
| `Z_SPEED` | `100` (`main.py`) / `150` (`main_cp.py`) | mm/s pen lift/lower |

### Workspace

| Constant | Default | Effect |
|----------|---------|--------|
| `MIN_REACH_MM` | `170.0` | Inner reach limit — points closer than this are warned |
| `MAX_REACH_MM` | `315.0` | Outer reach limit |
| `PORT` | `"/dev/ttyUSB0"` | Serial port for the arm |

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
├── main.py           # PTP draw mode
├── main_cp.py        # CP (continuous-path) draw mode
├── calibrate.py      # Interactive calibration script
├── pyproject.toml    # uv/pip dependencies
├── Report.md         # Known bugs and reliability issues (future work)
├── images/           # Input images
│   ├── Butterfly.jpeg
│   ├── Tier_1.png … Tier_5.png
│   └── heart_pruned_thin.png
└── pydobot/          # Modified local copy of pydobot library
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

