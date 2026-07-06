# ── Constants ────────────────────────────────────────────────────────────────
IMAGE_PATH      = "/home/user/Dobot_v2/images/Eyes.jpeg"
MAX_SIDE_PX     = 800
APPROX_EPSILON  = 1.5
MIN_STROKE_LEN  = 3     # only drops degenerate specks, keeps short hatch marks
MIN_STROKE_EXTENT_PX = 3.0  # bbox diagonal — drops dot-like curls the length filter misses
JOIN_TOL_PX     = 2.0   # concatenate consecutive strokes with gap <= this (skips a pen lift)
MORPH_CLOSE_PX  = 1     # closes small gaps in the ink mask before skeletonizing

# ── Calibration ───────────────────────────────────────────────────────────────
CAL_ARM_XY = [
    (308.9599304199219,    68.61124420166016),   # TL
    (292.4645690917969, -135.81228637695312),  # TR
    (166.2382354736328,  69.30846405029297),  # BL
    (162.52162170410156,  -139.58758544921875),  # BR
]
CAL_Z_CORNERS = [-51.48863220214844, -50.695167541503906,
                 -49.280738830566406, -52.93724822998047]

Z_UP         = -31.821640014648438   # pen-lifted height

# Speeds: firmware does NOT just silently clamp overspeed requests — pushing
# PTP_JOINT_VEL to the spec-sheet max (320) tripped a motion alarm mid-move,
# which freezes the command queue permanently (not a slowdown, a hang) until
# the 60s wait() timeout blows up the whole script. Back off from the edge;
# these were confirmed stable, the joint knob above 200 was not.
DRAW_SPEED   = 200     # mm/s per CP segment — documented firmware cap ~200
TRAVEL_SPEED = 300     # mm/s PTP pen-up travel
Z_SPEED      = 150     # mm/s pen lift/lower — kept moderate for clean contact

CP_PLAN_ACC     = 250.0   # mm/s²  CP look-ahead planner acceleration
CP_JUNCTION_VEL = 250.0   # mm/s   blending speed at CP waypoint junctions
CP_ACC          = 250.0   # mm/s²  CP acceleration (non-realtime mode)

PTP_JOINT_VEL = 200.0     # deg/s  — firmware init default, confirmed stable
PTP_JOINT_ACC = 200.0     # deg/s²

MIN_REACH_MM = 170.0
MAX_REACH_MM = 315.0

PORT = "/dev/ttyUSB1"

# ─────────────────────────────────────────────────────────────────────────────

import struct
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.collections as mc
from pathlib import Path

ROOT = Path(__file__).parent

try:
    from image_profile import binarize_auto
except ImportError:
    binarize_auto = None   # falls back to fixed-Otsu detect_edges()


# ── Image pipeline ────────────────────────────────────────────────────────────

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


def detect_edges(gray: np.ndarray) -> np.ndarray:
    """Binarize ink strokes directly (no Canny, single global threshold).

    Canny finds both boundaries of every drawn line, which skeletonize
    then centerlines separately -> doubled/parallel strokes. A global
    Otsu threshold instead marks each stroke as one solid blob, so
    skeletonize produces a single centerline per stroke.

    Adaptive (local) thresholding was tried and rejected -- too sensitive
    to per-window contrast, manufactures noise in flat regions and washes
    out faint strokes near block boundaries. This image is closer to
    bimodal (ink vs. paper), so one global threshold captures the actual
    linework more faithfully.
    """
    _, mask = cv2.threshold(
        gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )
    if MORPH_CLOSE_PX > 0:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * MORPH_CLOSE_PX + 1,) * 2
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def extract_strokes(
    ink_mask: np.ndarray,
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """Returns (kept_strokes, dropped_strokes) — dropped is everything cut
    by the length/extent filters, for the preview to show what got lost."""
    from skimage.morphology import skeletonize
    import sknw

    skeleton = skeletonize(ink_mask > 0).astype(np.uint8)
    graph    = sknw.build_sknw(skeleton)

    strokes = []
    dropped = []
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
            dropped.append(spts)
            continue
        # A tight curl can pass the arc-length filter while still being a
        # dot on paper — reject on spatial extent too.
        extent = np.linalg.norm(spts.max(axis=0) - spts.min(axis=0))
        if extent < MIN_STROKE_EXTENT_PX:
            dropped.append(spts)
            continue
        strokes.append(spts)

    return strokes, dropped


def order_strokes(strokes: list[np.ndarray]) -> list[np.ndarray]:
    if not strokes:
        return strokes
    try:
        from scipy.spatial import KDTree
        ordered = _order_strokes_kdtree(strokes)
    except ImportError:
        ordered = _order_strokes_greedy(strokes)
    return _two_opt(ordered)


def _travel_cost(strokes: list[np.ndarray]) -> float:
    return sum(float(np.linalg.norm(strokes[k + 1][0] - strokes[k][-1]))
               for k in range(len(strokes) - 1))


def _two_opt(ordered: list[np.ndarray], max_passes: int = 6) -> list[np.ndarray]:
    """Improve the greedy order with 2-opt (open path, strokes reversible).

    Reversing the sub-sequence [i..j] (each stroke also reversed) keeps all
    internal link costs, so only the two boundary links change — classic
    2-opt. Inner loop over j is vectorized with numpy.
    """
    n = len(ordered)
    if n < 3:
        return ordered
    strokes = list(ordered)
    starts = np.array([s[0] for s in strokes])
    ends   = np.array([s[-1] for s in strokes])

    for _ in range(max_passes):
        improved = False
        for i in range(n - 1):
            js = np.arange(i + 1, n)
            has_next = js < n - 1
            nxt = starts[np.minimum(js + 1, n - 1)]

            old_next = np.where(
                has_next, np.linalg.norm(ends[js] - nxt, axis=1), 0.0)
            new_next = np.where(
                has_next, np.linalg.norm(starts[i] - nxt, axis=1), 0.0)
            if i > 0:
                old_prev = float(np.linalg.norm(ends[i - 1] - starts[i]))
                new_prev = np.linalg.norm(ends[js] - ends[i - 1], axis=1)
            else:
                old_prev = 0.0
                new_prev = np.zeros(len(js))

            gain = old_prev + old_next - new_prev - new_next
            k = int(np.argmax(gain))
            if gain[k] > 1e-6:
                j = int(js[k])
                seg = [s[::-1] for s in reversed(strokes[i:j + 1])]
                strokes[i:j + 1] = seg
                starts[i:j + 1] = [s[0] for s in seg]
                ends[i:j + 1]   = [s[-1] for s in seg]
                improved = True
        if not improved:
            break

    return strokes


def join_strokes(ordered: list[np.ndarray],
                 tol: float = JOIN_TOL_PX) -> list[np.ndarray]:
    """Concatenate consecutive strokes whose gap is <= tol px.

    Skeleton-graph edges share junction nodes, so many consecutive strokes
    have zero gap — joining them skips a pen lift/lower cycle each."""
    if not ordered:
        return ordered
    merged = [ordered[0]]
    for s in ordered[1:]:
        if np.linalg.norm(merged[-1][-1] - s[0]) <= tol:
            merged[-1] = np.vstack([merged[-1], s])
        else:
            merged.append(s)
    return merged


def _order_strokes_kdtree(strokes: list[np.ndarray]) -> list[np.ndarray]:
    from scipy.spatial import KDTree

    n       = len(strokes)
    used    = [False] * n
    ordered = []
    pen_pos = strokes[0][0]

    for _ in range(n):
        # rebuild tree each step from remaining unused endpoints
        idx_map = []   # maps tree-row → (stroke_idx, is_end)
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


# ── Preview ───────────────────────────────────────────────────────────────────

def preview(gray: np.ndarray, pre_despeck: np.ndarray, mask: np.ndarray,
           strokes: list[np.ndarray], dropped: list[np.ndarray]) -> None:
    """4-panel: input, mask before despeck, mask after despeck (fed to
    skeletonize), final strokes. Speck removed by despeck = pre_despeck
    minus mask, shown in orange. Strokes killed by the length/extent
    filters (still present in the mask) are shown in red on the final
    mask panel, so the two loss stages are visible separately."""
    _, axes = plt.subplots(1, 4, figsize=(22, 6))

    axes[0].imshow(gray, cmap="gray")
    axes[0].set_title("Input (preprocessed)")
    axes[0].axis("off")

    h, w = gray.shape

    removed_by_despeck = cv2.subtract(pre_despeck, mask)
    speck_rgb = np.zeros((*pre_despeck.shape, 3), dtype=np.uint8)
    speck_rgb[mask > 0] = (255, 255, 255)
    speck_rgb[removed_by_despeck > 0] = (255, 140, 0)
    n_speck_px = int((removed_by_despeck > 0).sum())
    axes[1].imshow(speck_rgb)
    axes[1].set_title(f"Despeck  (orange = {n_speck_px}px removed)")
    axes[1].axis("off")

    axes[2].imshow(mask, cmap="gray")
    for s in dropped:
        axes[2].plot(s[:, 0], s[:, 1], color="red", linewidth=1.2)
    axes[2].set_title(f"Ink mask  (red = {len(dropped)} dropped strokes)")
    axes[2].axis("off")

    cmap   = plt.get_cmap("plasma")
    n      = len(strokes)
    segs   = []
    colors = []
    for i, s in enumerate(strokes):
        for j in range(len(s) - 1):
            segs.append([s[j], s[j + 1]])
            colors.append(cmap(i / max(n - 1, 1)))

    lc = mc.LineCollection(segs, colors=colors, linewidths=0.8)
    axes[3].add_collection(lc)
    axes[3].set_xlim(0, w)
    axes[3].set_ylim(h, 0)
    axes[3].set_aspect("equal")
    axes[3].set_title(f"Strokes  n={n}  pts={sum(len(s) for s in strokes)}")
    axes[3].axis("off")

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
    msg.params.extend(struct.pack('<f', x))
    msg.params.extend(struct.pack('<f', y))
    msg.params.extend(struct.pack('<f', z))
    msg.params.extend(struct.pack('<f', velocity))
    return arm._send_command(msg)


def _set_cp_params(arm, plan_acc: float, junction_vel: float, acc: float):
    """SET_CP_PARAMS (ID 90) — never set by pydobot, so the CP look-ahead
    planner otherwise runs at firmware boot defaults. junctionVel is the
    blending speed at waypoint junctions; raising it is the main win for
    real drawing throughput. Firmware clamps out-of-range values."""
    from pydobot.message import Message
    from pydobot.enums.ControlValues import ControlValues
    msg = Message()
    msg.id = 90                    # SET_CP_PARAMS
    msg.ctrl = ControlValues.ONE   # immediate
    msg.params = bytearray()
    msg.params.extend(struct.pack('<f', plan_acc))
    msg.params.extend(struct.pack('<f', junction_vel))
    msg.params.extend(struct.pack('<f', acc))
    msg.params.extend(bytearray([0x00]))  # realTimeTrack off
    return arm._send_command(msg)


def _speed(arm, mm_s: float) -> None:
    """arm.speed() costs two serial round-trips; skip when unchanged.
    Cache lives on the arm object so a reconnect (which resets firmware
    speed params) starts with a fresh cache."""
    if getattr(arm, "_last_speed", None) != mm_s:
        arm.speed(mm_s, mm_s)
        arm._last_speed = mm_s


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


def _move_or_recover(arm, x: float, y: float, z: float, mode) -> bool:
    """PTP move that survives a firmware motion alarm. The firmware doesn't
    just clamp overspeed requests — it can trip an alarm mid-move, which
    freezes the queued-command index permanently until wait() times out
    (60s) and raises. Catch that, clear the alarm, and let the caller skip
    forward instead of crashing the whole run. Returns False on failure."""
    try:
        arm._set_ptp_cmd(x, y, z, 0, mode=mode, wait=True)
        return True
    except RuntimeError as e:
        print(f"\nAlarm at ({x:.1f},{y:.1f},{z:.1f}): {e}. Clearing...")
        _recover(arm)
        return False


# ── Draw loop ─────────────────────────────────────────────────────────────────

def draw(strokes: list[np.ndarray], img_w: int, img_h: int) -> None:
    import pydobot
    from pydobot.enums import PTPMode
    from serial.tools import list_ports

    ports = list_ports.comports()
    if not ports:
        sys.exit("No serial ports found. Is the arm plugged in?")
    available = [p.device for p in ports]
    if PORT not in available:
        sys.exit(f"{PORT} not found. Is the arm plugged in? "
                 f"Available ports: {', '.join(available)}")
    print(f"Connecting on {PORT} ...")
    arm = pydobot.Dobot(port=PORT, verbose=False)

    try:
        _recover(arm)

        pose = arm.pose()
        print(f"Connected. Current pose: x={pose[0]:.1f} y={pose[1]:.1f} z={pose[2]:.1f}")

        # Push speed limits once. Values above the confirmed-stable range
        # can trip a motion alarm rather than being silently clamped —
        # see PTP_JOINT_VEL comment above.
        arm._set_ptp_joint_params(PTP_JOINT_VEL, PTP_JOINT_VEL,
                                  PTP_JOINT_VEL, PTP_JOINT_VEL,
                                  PTP_JOINT_ACC, PTP_JOINT_ACC,
                                  PTP_JOINT_ACC, PTP_JOINT_ACC)
        _set_cp_params(arm, CP_PLAN_ACC, CP_JUNCTION_VEL, CP_ACC)

        _speed(arm, TRAVEL_SPEED)
        _move_or_recover(arm, pose[0], pose[1], Z_UP, PTPMode.MOVJ_XYZ)

        skipped = 0
        total = len(strokes)
        for i, stroke in enumerate(strokes):
            x0, y0, z0 = px_to_mm(stroke[0][0], stroke[0][1], img_w, img_h)
            print(f"Stroke {i+1}/{total}  ({len(stroke)} pts)  "
                  f"xy=({x0:.1f},{y0:.1f}) z={z0:.1f}", end="\r")

            # travel — an alarm here just means retry next stroke, not crash
            _speed(arm, TRAVEL_SPEED)
            if not _move_or_recover(arm, x0, y0, Z_UP, PTPMode.MOVJ_XYZ):
                skipped += 1
                continue

            # pen down
            _speed(arm, Z_SPEED)
            if not _move_or_recover(arm, x0, y0, z0, PTPMode.MOVJ_XYZ):
                skipped += 1
                continue

            # CP draw — all points queued without blocking
            # PTP pen-up is enqueued after all CP commands; waiting for its
            # execution index implicitly drains the entire CP queue first.
            x_last, y_last = x0, y0
            for pt in stroke[1:]:
                x, y, z = px_to_mm(pt[0], pt[1], img_w, img_h)
                _cp_cmd(arm, x, y, z, DRAW_SPEED)
                x_last, y_last = x, y

            # pen up — retry once after recovery; if it still fails the pen
            # may be physically down, which would drag into the next
            # stroke's travel move, so this is worth a loud warning.
            _speed(arm, Z_SPEED)
            if not _move_or_recover(arm, x_last, y_last, Z_UP, PTPMode.MOVJ_XYZ):
                if not _move_or_recover(arm, x_last, y_last, Z_UP, PTPMode.MOVJ_XYZ):
                    print(f"\nWARNING: pen-up failed twice after stroke {i+1} — "
                          f"pen may be dragging on the surface.")

        print(f"\nDone — {total} strokes, {skipped} skipped.")

    finally:
        arm.close()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    dry_run = "--dry-run" in sys.argv or "-n" in sys.argv

    gray = load_image(IMAGE_PATH)
    h, w = gray.shape

    if binarize_auto is not None:
        mask, prof, pre_despeck = binarize_auto(gray)
        print(prof.summary())
    else:
        mask = detect_edges(gray)
        pre_despeck = mask.copy()

    strokes, dropped = extract_strokes(mask)
    n_raw = len(strokes)
    strokes = order_strokes(strokes)
    strokes = join_strokes(strokes)

    total_pts = sum(len(s) for s in strokes)
    print(f"Strokes: {n_raw} extracted -> {len(strokes)} after join,  "
          f"total points: {total_pts},  "
          f"pen-up travel: {_travel_cost(strokes):.0f}px")

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
        preview(gray, pre_despeck, mask, strokes, dropped)
    else:
        preview(gray, pre_despeck, mask, strokes, dropped)
        input("Close the preview window, then press Enter to start drawing...")
        draw(strokes, w, h)


if __name__ == "__main__":
    main()