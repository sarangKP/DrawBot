# ── Constants ────────────────────────────────────────────────────────────────
IMAGE_PATH      = "images/Tier_4.png"
MAX_SIDE_PX     = 800
CANNY_LOW       = 80
CANNY_HIGH      = 200
APPROX_EPSILON  = 2.0
MIN_STROKE_LEN  = 15
BLUR_KERNEL     = 5
MORPH_CLOSE_PX  = 0

# ── Calibration ───────────────────────────────────────────────────────────────
CAL_ARM_XY = [
    (303.6646423339844,    73.79429626464844),   # TL
    (292.9056396484375, -120.05909729003906),    # TR
    (171.79347229003906,  72.93241882324219),    # BL
    (167.5027618408203,  -109.03446960449219),   # BR
]
CAL_Z_CORNERS = [-57.175079345703125, -57.43455505371094,
                 -55.63251495361328, -56.73497772216797]

Z_UP         = -25.011795043945312
DRAW_SPEED   = 200     # mm/s CP drawing — Dobot firmware hard-caps ~200
TRAVEL_SPEED = 300     # mm/s PTP pen-up travel
Z_SPEED      = 150     # mm/s pen lift/lower

MIN_REACH_MM = 170.0
MAX_REACH_MM = 315.0

PORT = "/dev/ttyUSB0"

# ─────────────────────────────────────────────────────────────────────────────

import struct
import sys
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.collections as mc


# ── Image pipeline ────────────────────────────────────────────────────────────

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


def extract_strokes(edges: np.ndarray) -> list[np.ndarray]:
    from skimage.morphology import skeletonize
    import sknw

    skeleton = skeletonize(edges > 0).astype(np.uint8)
    graph    = sknw.build_sknw(skeleton)

    strokes = []
    for u, v, data in graph.edges(data=True):
        node_u = graph.nodes[u]["o"]
        node_v = graph.nodes[v]["o"]
        pts    = np.vstack([node_u, data["pts"], node_v])
        pts    = pts[:, ::-1].astype(float)

        simplified = cv2.approxPolyDP(
            pts.reshape(-1, 1, 2).astype(np.float32), APPROX_EPSILON, closed=False
        )
        spts = simplified[:, 0, :].astype(float)
        length = np.sum(np.linalg.norm(np.diff(spts, axis=0), axis=1))
        if length < MIN_STROKE_LEN:
            continue
        strokes.append(spts)

    return strokes


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


# ── Preview ───────────────────────────────────────────────────────────────────

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


# ── CP draw helpers ───────────────────────────────────────────────────────────

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


def _cp_cmd(arm, x: float, y: float, z: float, velocity: float):
    from pydobot.message import Message
    from pydobot.enums.ControlValues import ControlValues
    msg = Message()
    msg.id = 91                    # SET_CP_CMD
    msg.ctrl = ControlValues.THREE
    msg.params = bytearray([0x01]) # cpMode = absolute
    msg.params.extend(struct.pack('f', x))
    msg.params.extend(struct.pack('f', y))
    msg.params.extend(struct.pack('f', z))
    msg.params.extend(struct.pack('f', velocity))
    return arm._send_command(msg)


# ── Draw loop ─────────────────────────────────────────────────────────────────

def draw(strokes: list[np.ndarray], img_w: int, img_h: int) -> None:
    import pydobot
    from pydobot.enums import PTPMode
    from serial.tools import list_ports

    ports = list_ports.comports()
    if not ports:
        sys.exit("No serial ports found. Is the arm plugged in?")
    print(f"Connecting on {PORT} ...")
    arm = pydobot.Dobot(port=PORT, verbose=False)
    _clear_alarm(arm)
    arm._set_queued_cmd_clear()
    arm._set_queued_cmd_start_exec()

    pose = arm.pose()
    print(f"Connected. Current pose: x={pose[0]:.1f} y={pose[1]:.1f} z={pose[2]:.1f}")

    arm.speed(TRAVEL_SPEED, TRAVEL_SPEED)
    arm._set_ptp_cmd(pose[0], pose[1], Z_UP, 0, mode=PTPMode.MOVJ_XYZ, wait=True)

    skipped = 0
    total = len(strokes)
    for i, stroke in enumerate(strokes):
        x0, y0, z0 = px_to_mm(stroke[0][0], stroke[0][1], img_w, img_h)
        print(f"Stroke {i+1}/{total}  ({len(stroke)} pts)  xy=({x0:.1f},{y0:.1f}) z={z0:.1f}", end="\r")

        # travel
        arm.speed(TRAVEL_SPEED, TRAVEL_SPEED)
        arm._set_ptp_cmd(x0, y0, Z_UP, 0, mode=PTPMode.MOVJ_XYZ, wait=True)

        # pen down
        arm.speed(Z_SPEED, Z_SPEED)
        try:
            arm._set_ptp_cmd(x0, y0, z0, 0, mode=PTPMode.MOVJ_XYZ, wait=True)
        except RuntimeError:
            print(f"\nSkipping stroke {i+1} — alarm at ({x0:.1f},{y0:.1f},{z0:.1f}), clearing...")
            _clear_alarm(arm)
            arm._set_queued_cmd_clear()
            arm._set_queued_cmd_start_exec()
            skipped += 1
            continue

        # CP draw — all points queued without blocking
        x_last, y_last = x0, y0
        for pt in stroke[1:]:
            x, y, z = px_to_mm(pt[0], pt[1], img_w, img_h)
            _cp_cmd(arm, x, y, z, DRAW_SPEED)
            x_last, y_last = x, y

        # pen up — PTP after CP; wait=True drains the CP queue first
        arm.speed(Z_SPEED, Z_SPEED)
        arm._set_ptp_cmd(x_last, y_last, Z_UP, 0, mode=PTPMode.MOVJ_XYZ, wait=True)

    print(f"\nDone — {total} strokes, {skipped} skipped.")
    arm.close()


# ── Main ──────────────────────────────────────────────────────────────────────

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
        preview(gray, strokes)
    else:
        preview(gray, strokes)
        input("Close the preview window, then press Enter to start drawing...")
        draw(strokes, w, h)


if __name__ == "__main__":
    main()
