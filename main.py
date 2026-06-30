# ── Constants ────────────────────────────────────────────────────────────────
IMAGE_PATH      = "images/Butterfly.jpeg"
MAX_SIDE_PX     = 800
CANNY_LOW       = 80            # high thresholds → strong edges only
CANNY_HIGH      = 200
APPROX_EPSILON  = 2.0           # RDP tolerance after skeletonisation (px)
MIN_STROKE_LEN  = 15            # drop strokes shorter than this (px)
BLUR_KERNEL     = 5             # pre-Canny Gaussian blur (odd number; larger = smoother/fewer edges)
MORPH_CLOSE_PX  = 0             # morphological closing radius after Canny (0 = off; try 2-4)

# ── Calibration ───────────────────────────────────────────────────────────────
# 4-point bilinear: normalised pixel [0,1]×[0,1] → arm (x,y) mm
# Corner order: TL, TR, BL, BR  (pixel [0,0],[1,0],[0,1],[1,1])
CAL_ARM_XY = [
    (308.9599304199219,    68.61124420166016),   # TL
    (292.4645690917969, -135.81228637695312),  # TR
    (166.2382354736328,  69.30846405029297),  # BL
    (162.52162170410156,  -139.58758544921875),  # BR
]
# z at paper surface, per corner (TL TR BL BR) — pen-down z is interpolated
CAL_Z_CORNERS = [-51.48863220214844, -50.695167541503906,
                 -49.280738830566406, -52.93724822998047]

Z_UP         = -31.821640014648438   # pen-lifted height
DRAW_SPEED   = 200                   # mm/s while drawing
TRAVEL_SPEED = 300                   # mm/s pen-up travel between strokes
Z_SPEED      = 100                   # mm/s pen lift/lower — slow = no bounce/alarm

MIN_REACH_MM = 170.0
MAX_REACH_MM = 315.0

PORT = "/dev/ttyUSB0"

# ─────────────────────────────────────────────────────────────────────────────

import sys
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.collections as mc
from pathlib import Path

ROOT = Path(__file__).parent


# ── Step 1: load & resize ────────────────────────────────────────────────────

def load_image(path: str) -> np.ndarray:
    img = cv2.imread(str(ROOT / path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f"Cannot load image: {path}")
    h, w = img.shape
    if w < 2 or h < 2:
        sys.exit(f"Image too small: {w}×{h}")
    scale = MAX_SIDE_PX / max(h, w)
    if scale < 1.0:
        img = cv2.resize(img, (int(w * scale), int(h * scale)),
                         interpolation=cv2.INTER_AREA)
    return img


# ── Step 2: preprocess + edge detection ─────────────────────────────────────

def preprocess(gray: np.ndarray) -> np.ndarray:
    clahe    = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    k = BLUR_KERNEL | 1
    return cv2.GaussianBlur(enhanced, (k, k), 0)


def detect_edges(gray: np.ndarray) -> np.ndarray:
    edges = cv2.Canny(gray, CANNY_LOW, CANNY_HIGH)
    if MORPH_CLOSE_PX > 0:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * MORPH_CLOSE_PX + 1,) * 2
        )
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    return edges


# ── Step 3: skeletonise + sknw graph → strokes ──────────────────────────────

def extract_strokes(edges: np.ndarray) -> list[np.ndarray]:
    from skimage.morphology import skeletonize
    import sknw

    skeleton = skeletonize(edges > 0).astype(np.uint8)
    graph    = sknw.build_sknw(skeleton)

    strokes = []
    for u, v, data in graph.edges(data=True):
        node_u = graph.nodes[u]["o"].reshape(1, 2)
        node_v = graph.nodes[v]["o"].reshape(1, 2)
        inner  = data["pts"].reshape(-1, 2)
        pts    = np.vstack([node_u, inner, node_v])
        pts    = pts[:, ::-1].astype(float)   # (row,col) → (col,row)

        simplified = cv2.approxPolyDP(
            pts.reshape(-1, 1, 2).astype(np.float32), APPROX_EPSILON, closed=False
        )
        spts = simplified[:, 0, :].astype(float)
        length = np.sum(np.linalg.norm(np.diff(spts, axis=0), axis=1))
        if length < MIN_STROKE_LEN:
            continue
        strokes.append(spts)

    return strokes


# ── Step 4: nearest-neighbour stroke ordering ────────────────────────────────

def order_strokes(strokes: list[np.ndarray]) -> list[np.ndarray]:
    if not strokes:
        return strokes
    try:
        from scipy.spatial import KDTree
        return _order_strokes_kdtree(strokes)
    except ImportError:
        return _order_strokes_greedy(strokes)


def _order_strokes_kdtree(strokes: list[np.ndarray]) -> list[np.ndarray]:
    from scipy.spatial import KDTree

    n       = len(strokes)
    used    = [False] * n
    ordered = []
    pen_pos = strokes[0][0]

    for _ in range(n):
        idx_map = []
        pts     = []
        for i, s in enumerate(strokes):
            if not used[i]:
                pts.append(s[0]);  idx_map.append((i, False))
                pts.append(s[-1]); idx_map.append((i, True))

        tree = KDTree(np.array(pts))
        _, row = tree.query(pen_pos)
        stroke_i, flip = idx_map[row]

        stroke = strokes[stroke_i]
        if flip:
            stroke = stroke[::-1]
        ordered.append(stroke)
        used[stroke_i] = True
        pen_pos = stroke[-1]

    return ordered


def _order_strokes_greedy(strokes: list[np.ndarray]) -> list[np.ndarray]:
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
    axes[0].set_title("Input (preprocessed)")
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


# ── Draw helpers ──────────────────────────────────────────────────────────────

def _clear_alarm(arm) -> None:
    from pydobot.message import Message
    from pydobot.enums.CommunicationProtocolIDs import CommunicationProtocolIDs
    from pydobot.enums.ControlValues import ControlValues
    msg = Message()
    msg.id = CommunicationProtocolIDs.CLEAR_ALL_ALARMS_STATE
    msg.ctrl = ControlValues.ONE
    try:
        arm._send_command(msg)
    except RuntimeError:
        pass


def _recover(arm) -> None:
    try:
        _clear_alarm(arm)
    except Exception:
        pass
    try:
        arm._set_queued_cmd_clear()
    except Exception:
        pass
    try:
        arm._set_queued_cmd_start_exec()
    except Exception:
        pass


# ── Draw loop ────────────────────────────────────────────────────────────────

def draw(strokes: list[np.ndarray], img_w: int, img_h: int) -> None:
    import pydobot
    from pydobot.enums import PTPMode
    from serial.tools import list_ports

    ports = list_ports.comports()
    if not ports:
        sys.exit("No serial ports found. Is the arm plugged in?")
    print(f"Connecting on {PORT} ...")
    arm = pydobot.Dobot(port=PORT, verbose=False)

    try:
        _recover(arm)

        pose = arm.pose()
        print(f"Connected. Current pose: x={pose[0]:.1f} y={pose[1]:.1f} z={pose[2]:.1f}")

        arm.speed(TRAVEL_SPEED, TRAVEL_SPEED)
        arm.move_to(pose[0], pose[1], Z_UP, 0, wait=True)

        skipped = 0
        total = len(strokes)
        for i, stroke in enumerate(strokes):
            x0, y0, z0 = px_to_mm(stroke[0][0], stroke[0][1], img_w, img_h)
            print(f"Stroke {i+1}/{total}  ({len(stroke)} pts)  "
                  f"xy=({x0:.1f},{y0:.1f}) z={z0:.1f}", end="\r")

            # travel pen-up
            arm.speed(TRAVEL_SPEED, TRAVEL_SPEED)
            arm._set_ptp_cmd(x0, y0, Z_UP, 0, mode=PTPMode.MOVJ_XYZ, wait=True)

            # pen down
            arm.speed(Z_SPEED, Z_SPEED)
            try:
                arm._set_ptp_cmd(x0, y0, z0, 0, mode=PTPMode.MOVJ_XYZ, wait=True)
            except RuntimeError:
                print(f"\nSkipping stroke {i+1} — alarm at "
                      f"({x0:.1f},{y0:.1f},{z0:.1f}), clearing...")
                _recover(arm)
                skipped += 1
                continue
            time.sleep(0.3)

            # draw stroke — queue all points without blocking per-point
            arm.speed(DRAW_SPEED, DRAW_SPEED)
            x_last, y_last = x0, y0
            for pt in stroke[1:]:
                x, y, z = px_to_mm(pt[0], pt[1], img_w, img_h)
                arm.move_to(x, y, z, 0, wait=False)
                x_last, y_last = x, y

            # lift pen — PTP enqueued after draw points; wait=True drains queue
            arm.speed(Z_SPEED, Z_SPEED)
            arm._set_ptp_cmd(x_last, y_last, Z_UP, 0, mode=PTPMode.MOVJ_XYZ, wait=True)

        print(f"\nDone — {total} strokes, {skipped} skipped.")

    finally:
        arm.close()


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    dry_run = "--dry-run" in sys.argv or "-n" in sys.argv

    gray    = load_image(IMAGE_PATH)
    h, w    = gray.shape
    proc    = preprocess(gray)
    edges   = detect_edges(proc)
    strokes = extract_strokes(edges)
    strokes = order_strokes(strokes)
    strokes = [s for s in strokes
               if np.sum(np.linalg.norm(np.diff(s, axis=0), axis=1)) >= MIN_STROKE_LEN]

    total_pts = sum(len(s) for s in strokes)
    print(f"Strokes: {len(strokes)},  total points: {total_pts}")

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
        preview(proc, strokes)
    else:
        preview(proc, strokes)
        input("Close the preview window, then press Enter to start drawing...")
        draw(strokes, w, h)


if __name__ == "__main__":
    main()
