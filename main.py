# ── Constants ────────────────────────────────────────────────────────────────
IMAGE_PATH      = "images/heart_pruned_thin.png"
MAX_SIDE_PX     = 800
CANNY_LOW       = 50
CANNY_HIGH      = 150
APPROX_EPSILON  = 2.0           # higher = fewer points per stroke
MIN_CONTOUR_PTS = 5

# ── Calibration ───────────────────────────────────────────────────────────────
# 4-point bilinear: normalised pixel [0,1]×[0,1] → arm (x,y) mm
# Corner order: TL, TR, BL, BR  (pixel [0,0],[1,0],[0,1],[1,1])
CAL_ARM_XY = [
    (303.6646423339844,    73.79429626464844),   # TL
    (292.9056396484375, -120.05909729003906),  # TR
    (171.79347229003906,  72.93241882324219),  # BL
    (167.5027618408203,  -109.03446960449219),  # BR
]
# z at paper surface, per corner (TL TR BL BR) — pen-down z is interpolated
CAL_Z_CORNERS = [-57.175079345703125, -57.43455505371094,
                 -55.63251495361328, -56.73497772216797]

Z_UP         = -25.011795043945312   # pen-lifted height# pen-lifted height
DRAW_SPEED   = 450                   # mm/s while drawing
TRAVEL_SPEED = 500                   # mm/s while pen is up

MIN_REACH_MM = 170.0
MAX_REACH_MM = 315.0

# Serial port — run `python -m serial.tools.list_ports` to find yours
PORT = "/dev/ttyUSB0"

# ─────────────────────────────────────────────────────────────────────────────

import sys
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.collections as mc


# ── Step 1: load & resize ────────────────────────────────────────────────────

def load_image(path: str) -> np.ndarray:
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f"Cannot load image: {path}")
    h, w = img.shape
    scale = MAX_SIDE_PX / max(h, w)
    if scale < 1.0:
        img = cv2.resize(img, (int(w * scale), int(h * scale)),
                         interpolation=cv2.INTER_AREA)
    return img


# ── Step 2: edge detection ───────────────────────────────────────────────────

def detect_edges(gray: np.ndarray) -> np.ndarray:
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    return cv2.Canny(blurred, CANNY_LOW, CANNY_HIGH)


# ── Step 3: find & simplify contours ────────────────────────────────────────

def extract_strokes(edges: np.ndarray) -> list[np.ndarray]:
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    strokes = []
    for c in contours:
        simplified = cv2.approxPolyDP(c, APPROX_EPSILON, closed=False)
        pts = simplified[:, 0, :]
        if len(pts) >= MIN_CONTOUR_PTS:
            strokes.append(pts.astype(float))
    return strokes


# ── Step 4: nearest-neighbour stroke ordering ────────────────────────────────

def order_strokes(strokes: list[np.ndarray]) -> list[np.ndarray]:
    if not strokes:
        return strokes
    remaining = list(strokes)
    ordered   = [remaining.pop(0)]
    pen_pos   = ordered[0][-1]

    while remaining:
        best_idx  = 0
        best_dist = np.inf
        flip      = False
        for i, s in enumerate(remaining):
            d_start = np.linalg.norm(s[0]  - pen_pos)
            d_end   = np.linalg.norm(s[-1] - pen_pos)
            d = min(d_start, d_end)
            if d < best_dist:
                best_dist = d
                best_idx  = i
                flip      = d_end < d_start
        stroke = remaining.pop(best_idx)
        if flip:
            stroke = stroke[::-1]
        ordered.append(stroke)
        pen_pos = stroke[-1]

    return ordered


# ── Pixel → mm mapping ───────────────────────────────────────────────────────

def _bilinear(tl, tr, bl, br, u: float, v: float):
    return (tl * (1 - u) * (1 - v)
          + tr *      u  * (1 - v)
          + bl * (1 - u) *      v
          + br *      u  *      v)


def px_to_mm(px_col: float, px_row: float,
             img_w: int, img_h: int) -> tuple[float, float, float]:
    u = px_col / (img_w - 1)
    v = px_row / (img_h - 1)
    tl, tr, bl, br = [np.array(p) for p in CAL_ARM_XY]
    xy = _bilinear(tl, tr, bl, br, u, v)
    z  = _bilinear(*CAL_Z_CORNERS, u, v)
    return float(xy[0]), float(xy[1]), float(z)


def check_reach(x_mm: float, y_mm: float) -> bool:
    r = np.hypot(x_mm, y_mm)
    return MIN_REACH_MM <= r <= MAX_REACH_MM


# ── Matplotlib preview ───────────────────────────────────────────────────────

def preview(gray: np.ndarray, strokes: list[np.ndarray]) -> None:
    _, axes = plt.subplots(1, 2, figsize=(12, 6))

    axes[0].imshow(gray, cmap="gray")
    axes[0].set_title("Edge map")
    axes[0].axis("off")

    h, w   = gray.shape
    cmap   = plt.get_cmap("plasma")
    n      = len(strokes)
    segs   = []
    colors = []
    for i, s in enumerate(strokes):
        for j in range(len(s) - 1):
            segs.append([s[j], s[j + 1]])
            colors.append(cmap(i / max(n - 1, 1)))

    lc = mc.LineCollection(segs, colors=colors, linewidths=0.8)
    axes[1].add_collection(lc)
    axes[1].set_xlim(0, w)
    axes[1].set_ylim(h, 0)
    axes[1].set_aspect("equal")
    axes[1].set_title(f"Strokes  n={n}  pts={sum(len(s) for s in strokes)}")
    axes[1].axis("off")

    plt.tight_layout()
    plt.show()


# ── Draw loop ────────────────────────────────────────────────────────────────

def draw(strokes: list[np.ndarray], img_w: int, img_h: int) -> None:
    import pydobot
    from serial.tools import list_ports

    ports = list_ports.comports()
    if not ports:
        sys.exit("No serial ports found. Is the arm plugged in?")
    print(f"Connecting on {PORT} ...")
    arm = pydobot.Dobot(port=PORT, verbose=False)

    arm.speed(TRAVEL_SPEED, TRAVEL_SPEED)
    arm.move_to(arm.pose()[0], arm.pose()[1], Z_UP, 0, wait=True)   # lift first

    total = len(strokes)
    for i, stroke in enumerate(strokes):
        print(f"Stroke {i+1}/{total}  ({len(stroke)} pts)", end="\r")

        # travel to stroke start with pen up
        x0, y0, z0 = px_to_mm(stroke[0][0], stroke[0][1], img_w, img_h)
        arm.speed(TRAVEL_SPEED, TRAVEL_SPEED)
        arm.move_to(x0, y0, Z_UP, 0, wait=True)

        # lower pen
        arm.speed(DRAW_SPEED, DRAW_SPEED)
        arm.move_to(x0, y0, z0, 0, wait=True)
        time.sleep(0.5)

        # draw the stroke — queue all points, don't block per-point
        for pt in stroke[1:]:
            x, y, z = px_to_mm(pt[0], pt[1], img_w, img_h)
            arm.move_to(x, y, z, 0, wait=False)

        # lift pen — block here so we're actually done before moving up
        arm.speed(TRAVEL_SPEED, TRAVEL_SPEED)
        arm.move_to(arm.pose()[0], arm.pose()[1], Z_UP, 0, wait=True)

    print(f"\nDone. {total} strokes.")
    arm.close()


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    dry_run = "--dry-run" in sys.argv or "-n" in sys.argv

    gray    = load_image(IMAGE_PATH)
    h, w    = gray.shape
    edges   = detect_edges(gray)
    strokes = extract_strokes(edges)
    strokes = order_strokes(strokes)

    total_pts = sum(len(s) for s in strokes)
    print(f"Strokes: {len(strokes)},  total points: {total_pts}")

    # Reach check
    bad = 0
    for s in strokes:
        for col, row in s:
            x, y, _ = px_to_mm(col, row, w, h)
            if not check_reach(x, y):
                bad += 1
    if bad:
        print(f"WARNING: {bad}/{total_pts} points outside reach "
              f"[{MIN_REACH_MM}–{MAX_REACH_MM} mm]. Recalibrate or crop.")
    else:
        print("Reach check OK.")

    if dry_run:
        preview(gray, strokes)
    else:
        preview(gray, strokes)          # confirm visually, then close window to draw
        input("Close the preview window, then press Enter to start drawing...")
        draw(strokes, w, h)


if __name__ == "__main__":
    main()
