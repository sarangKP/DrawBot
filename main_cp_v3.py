# ── Constants ────────────────────────────────────────────────────────────────
IMAGE_PATH      = "/home/user/Dobot_v2/images/Eyes.jpeg"
MAX_SIDE_PX     = 800
APPROX_EPSILON  = 1.5
MIN_STROKE_LEN  = 1.5     # only drops degenerate specks, keeps short hatch marks
MIN_STROKE_EXTENT_PX = 2.5  # bbox diagonal — drops dot-like curls the length filter misses
JOIN_TOL_PX     = 2.0   # concatenate consecutive strokes with gap <= this (skips a pen lift)
MORPH_CLOSE_PX  = 0     # closes small gaps in the edge mask before skeletonizing (0 = off)

# Edge stage: Sato ridge/vesselness filter (skimage.filters.sato) instead
# of Canny. Canny traces stroke *boundaries*, which doubles every already-
# thick line in this source art. Sato responds to the ridge (centerline)
# itself, so it gives a single clean line per stroke like Otsu — but stays
# grayscale-sensitive, so faint/thin hair strands that Otsu's hard
# threshold would flatten out still register. Confirmed visually against
# main_cp_v3.py (Canny) and v2 (Otsu) on Tier_4.png before adopting.
SATO_SIGMAS     = (1, 2)   # ridge scales to test — matches stroke width range
BLUR_KERNEL     = 6    # odd kernel size for pre-filter Gaussian blur
CLAHE_CLIP      = 1.0
CLAHE_TILE      = 8     # tile grid is CLAHE_TILE x CLAHE_TILE

# ── Calibration ───────────────────────────────────────────────────────────────
CAL_ARM_XY = [
    (287.6391906738281,    123.99957275390625),   # TL
    (283.1493225097656, -131.48654174804688),  # TR
    (141.3195343017578,  127.28409576416016),  # BL
    (142.61416625976562,  -135.92137145996094),  # BR
]
CAL_Z_CORNERS = [-50.342613220214844, -54.00237274169922,
                 -50.32567596435547, -49.893287658691406]

Z_UP         = -18.61402130126953   # pen-lifted height

# Speeds: firmware does NOT just silently clamp overspeed requests — pushing
# PTP_JOINT_VEL to the spec-sheet max (320) tripped a motion alarm mid-move,
# which freezes the command queue permanently (not a slowdown, a hang) until
# the 60s wait() timeout blows up the whole script. Back off from the edge;
# these were confirmed stable, the joint knob above 200 was not.
DRAW_SPEED   = 200     # mm/s per CP segment — documented firmware cap ~200
CP_DEDUP_EPS_MM = 0.05  # skip a CP point coincident with the last one sent —
                        # a zero-length segment at speed hangs the firmware's
                        # CP junction planner silently (no alarm, queue just
                        # stops draining) rather than raising any error
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
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.collections as mc
from pathlib import Path

ROOT = Path(__file__).parent

# image_profile.binarize_auto was an Otsu-based auto-thresholder for v2's
# ink-mask approach; it doesn't apply to the ridge-filter pipeline below,
# so it's dropped here rather than left as dead-looking optional code.


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


def preprocess(gray: np.ndarray) -> np.ndarray:
    """CLAHE local-contrast boost + Gaussian blur, ahead of the ridge filter.

    CLAHE pulls out faint detail in flat regions before ridge detection.
    The blur softens sensor noise so Sato doesn't respond to it as
    spurious short ridges.
    """
    clahe    = cv2.createCLAHE(clipLimit=CLAHE_CLIP, tileGridSize=(CLAHE_TILE, CLAHE_TILE))
    enhanced = clahe.apply(gray)
    k = BLUR_KERNEL | 1   # force odd
    return cv2.GaussianBlur(enhanced, (k, k), 0)


def detect_edges(gray: np.ndarray) -> np.ndarray:
    """Sato ridge/vesselness filter + Otsu threshold on the response.

    Replaces v3's Canny. Canny finds the two boundaries of every drawn
    line, which skeletonize then centerlines separately -> doubled
    strokes. Sato is built to find the ridge (centerline) directly —
    black_ridges=True treats dark ink on light paper as the ridge — so
    thresholding its response gives a single clean line per stroke, the
    way Otsu does, while still picking up faint thin strands a hard
    binary threshold would lose.
    """
    from skimage.filters import sato
    resp = sato(gray, sigmas=SATO_SIGMAS, black_ridges=True)
    resp_norm = (resp / (resp.max() + 1e-9) * 255).astype(np.uint8)
    _, mask = cv2.threshold(resp_norm, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
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

def preview(gray: np.ndarray, edges: np.ndarray,
           strokes: list[np.ndarray], dropped: list[np.ndarray]) -> None:
    """3-panel: preprocessed input, Sato ridge mask (red = strokes dropped
    by the length/extent filters), final ordered/joined strokes."""
    _, axes = plt.subplots(1, 3, figsize=(18, 6))

    axes[0].imshow(gray, cmap="gray")
    axes[0].set_title("Input (CLAHE + blur)")
    axes[0].axis("off")

    h, w = gray.shape

    axes[1].imshow(edges, cmap="gray")
    for s in dropped:
        axes[1].plot(s[:, 0], s[:, 1], color="red", linewidth=1.2)
    axes[1].set_title(f"Sato ridge mask  (red = {len(dropped)} dropped strokes)")
    axes[1].axis("off")

    cmap   = plt.get_cmap("plasma")
    n      = len(strokes)
    segs   = []
    colors = []
    for i, s in enumerate(strokes):
        for j in range(len(s) - 1):
            segs.append([s[j], s[j + 1]])
            colors.append(cmap(i / max(n - 1, 1)))

    lc = mc.LineCollection(segs, colors=colors, linewidths=0.8)
    axes[2].add_collection(lc)
    axes[2].set_xlim(0, w)
    axes[2].set_ylim(h, 0)
    axes[2].set_aspect("equal")
    axes[2].set_title(f"Strokes  n={n}  pts={sum(len(s) for s in strokes)}")
    axes[2].axis("off")

    plt.tight_layout()
    plt.show()


# ── CP draw helpers ───────────────────────────────────────────────────────────

def _vlog(verbose: bool, msg: str) -> None:
    if verbose:
        print(f"[v {time.strftime('%H:%M:%S')}] {msg}")


class _Tee:
    """Duplicates writes to multiple streams — lets stdout keep printing to
    the terminal while everything (including pydobot's own verbose >>/<<
    prints) also lands in a log file, so -v runs don't need copy-pasting."""

    def __init__(self, *streams):
        self.streams = streams

    def write(self, data):
        for s in self.streams:
            s.write(data)

    def flush(self):
        for s in self.streams:
            s.flush()


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

    # Clearing the queue does NOT mean the pen is up — a stalled/timed-out
    # command (e.g. a pen-up move) may have frozen with the pen still down.
    # Read the arm's actual current pose (immediate query, unaffected by a
    # frozen queue) and force a blocking Z-only lift from there, so the next
    # travel move never drags a still-down pen across the page.
    from pydobot.enums import PTPMode
    try:
        pose = arm.pose()
        arm._set_ptp_cmd(pose[0], pose[1], Z_UP, 0, mode=PTPMode.MOVJ_XYZ, wait=True)
    except Exception as e:
        print(f"WARNING: forced pen-up after recovery failed: {e}")


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

def draw(strokes: list[np.ndarray], img_w: int, img_h: int, verbose: bool = False) -> None:
    import pydobot
    from pydobot.enums import PTPMode
    from serial.tools import list_ports

    ports = list_ports.comports()
    if not ports:
        sys.exit("No serial ports found. Is the arm plugged in?")
    available = [p.device for p in ports]
    port = PORT
    if port not in available:
        usb_ports = [d for d in available if "USB" in d]
        if len(usb_ports) == 1:
            port = usb_ports[0]
            print(f"{PORT} not found, using {port} instead.")
        else:
            sys.exit(f"{PORT} not found. Is the arm plugged in? "
                     f"Available ports: {', '.join(available)}")
    print(f"Connecting on {port} ...")
    # verbose=True on the Dobot itself makes pydobot print every raw
    # command/response pair (>>/<<), which is what shows whether the
    # queued-command index is actually advancing per CP point.
    arm = pydobot.Dobot(port=port, verbose=verbose)

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
            idx_before = arm._get_queued_cmd_current_index() if verbose else None
            t0 = time.time()
            x_last, y_last = x0, y0
            n_deduped = 0
            for pt in stroke[1:]:
                x, y, z = px_to_mm(pt[0], pt[1], img_w, img_h)
                if abs(x - x_last) < CP_DEDUP_EPS_MM and abs(y - y_last) < CP_DEDUP_EPS_MM:
                    n_deduped += 1
                    continue
                _cp_cmd(arm, x, y, z, DRAW_SPEED)
                x_last, y_last = x, y
            if verbose:
                idx_after = arm._get_queued_cmd_current_index()
                n_pts = len(stroke) - 1 - n_deduped
                _vlog(verbose,
                      f"stroke {i+1}/{total}: {n_pts} CP pts submitted "
                      f"({n_deduped} deduped) in {time.time() - t0:.2f}s, "
                      f"queue idx {idx_before} -> {idx_after} "
                      f"(delta {idx_after - idx_before}, expected >= {n_pts})")

            # pen up — retry once after recovery. If it still fails the pen
            # is likely physically down, and continuing would drag it through
            # every remaining stroke's travel move — abort instead of limping
            # on with a ruined drawing.
            _speed(arm, Z_SPEED)
            if not _move_or_recover(arm, x_last, y_last, Z_UP, PTPMode.MOVJ_XYZ):
                if not _move_or_recover(arm, x_last, y_last, Z_UP, PTPMode.MOVJ_XYZ):
                    print(f"\nERROR: pen-up failed twice after stroke {i+1}/{total} — "
                          f"pen may be dragging on the surface. Aborting rather than "
                          f"drawing further with the pen down.")
                    sys.exit(1)

        print(f"\nDone — {total} strokes, {skipped} skipped.")

    finally:
        arm.close()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    dry_run = "--dry-run" in sys.argv or "-n" in sys.argv
    verbose = "--verbose" in sys.argv or "-v" in sys.argv

    log_file = None
    if verbose:
        log_dir = ROOT / "logs"
        log_dir.mkdir(exist_ok=True)
        log_path = log_dir / f"draw_{time.strftime('%Y%m%d_%H%M%S')}.log"
        log_file = open(log_path, "w")
        sys.stdout = _Tee(sys.stdout, log_file)
        print(f"Logging to {log_path}")

    try:
        _main_body(dry_run, verbose)
    finally:
        if log_file is not None:
            sys.stdout = sys.stdout.streams[0]
            log_file.close()


def _main_body(dry_run: bool, verbose: bool) -> None:
    gray = load_image(IMAGE_PATH)
    h, w = gray.shape

    proc  = preprocess(gray)
    edges = detect_edges(proc)

    strokes, dropped = extract_strokes(edges)
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
        preview(proc, edges, strokes, dropped)
    else:
        preview(proc, edges, strokes, dropped)
        input("Close the preview window, then press Enter to start drawing...")
        draw(strokes, w, h, verbose=verbose)


if __name__ == "__main__":
    main()