# Dobot Magician Pen-Plotter — Bug & Reliability Audit

> Audit performed 2026-06-30. Do not modify until issues are ready to be addressed.

---

## CRITICAL

---

### C-1: Lock Deadlock on Serial Exception
**File:** `pydobot/pydobot/dobot.py:88–92`

`lock.acquire()` / `lock.release()` with no try/finally. If USB disconnects mid-write, the release never runs. Every subsequent call hangs forever — including alarm recovery and `close()`.

**Fix:** Use `with self.lock:` context manager.

---

### C-2: Alarm Recovery Can Crash Script
**File:** `main.py:241`, `main_cp.py:248`

After catching the pen-down `RuntimeError`, `_set_queued_cmd_clear()` and `_set_queued_cmd_start_exec()` are unprotected. If the arm still doesn't respond, they throw uncaught `RuntimeError` and kill the entire draw session.

**Fix:** Wrap the entire recovery block in a nested try/except.

---

### C-3: Stale Serial Response Contaminates Next Command
**File:** `pydobot/pydobot/dobot.py:76–92`

When `_read_message` times out (returns `None`), the arm's late response stays in the UART buffer. The next command's `read_all()` picks up the old response concatenated with the beginning of the new one. `Message(b)` blindly parses from index 0 — gets garbage. Can cause `wait=True` to exit early with a wrong `expected_idx`, starting the next stroke before the current one finishes.

**Fix:** Call `ser.reset_input_buffer()` before each write.

---

## HIGH

---

### H-1: No Bounds Check on Received Bytes
**File:** `pydobot/pydobot/message.py:10–15`

If `read_all()` returns a partial packet (< 6 bytes), `b[3]` raises `IndexError`. No header validation, no checksum verification on received messages.

**Fix:** Guard `len(b) >= 6` and validate `b[0:2] == bytes([0xAA, 0xAA])`.

---

### H-2: calibrate.py Z_UP Regex Duplicates Comment on Every Run
**File:** `calibrate.py:57`

Pattern `[^\n#]+` stops at `#`, leaves the old comment in place, appends a new one. Already visible in `main.py` line 24:
```
Z_UP = -25.011...   # pen-lifted height# pen-lifted height
```
Every subsequent calibration run appends another copy.

**Fix:** Change pattern to `[^\n]+` to consume the entire line including any existing comment.

---

### H-3: calibrate.py Only Patches main.py — main_cp.py Calibration Drifts
**File:** `calibrate.py:22`

After any recalibration, `main_cp.py` retains the old `CAL_ARM_XY`, `CAL_Z_CORNERS`, and `Z_UP`. CP mode draws in the wrong physical position.

**Fix:** Patch both files in `patch_main`, or extract constants to a shared `config.py` imported by both scripts.

---

### H-4: No Timeout in wait=True Polling Loop — Infinite Hang
**File:** `pydobot/pydobot/dobot.py:104–112`

If the arm stalls (obstruction, servo overload) without triggering an alarm, the `while True` polling loop runs forever with no output. Script hangs silently.

**Fix:** Add a 60-second wall-clock deadline and raise `RuntimeError` on expiry.

---

### H-5: arm.close() Not in Finally — Serial Port Leak
**File:** `main.py`, `main_cp.py` (draw function)

Any uncaught exception (`KeyboardInterrupt`, etc.) skips `arm.close()`. `/dev/ttyUSB0` stays locked until the process dies — the next run fails to open the port.

**Fix:** Wrap the stroke loop in `try/finally: arm.close()`.

---

### H-6: uint32 Queue Index Wrap-Around Causes Infinite Wait
**File:** `pydobot/pydobot/dobot.py:107`

If the firmware queue index wraps from `0xFFFFFFFF` to `0`, `current_idx >= expected_idx` is `False` forever. Also triggered if alarm recovery resets the firmware queue index to 0 mid-wait.

**Fix:** Combine with H-4's timeout fix, or use modular arithmetic: `(current_idx - expected_idx) % (2**32) < 2**31`.

---

## MEDIUM

---

### M-1: Checksum = 0 Still Possible for Some Coordinates
**File:** `pydobot/pydobot/message.py:37–39`

The `% 255` → `% 256` fix handles payload sum ≡ 1 (mod 256). But when payload sum ≡ 0 (mod 256): `256 - 0 = 256`, `256 % 256 = 0` — still produces checksum=0. Probability: 1 in 256 commands. Whether the Dobot firmware rejects checksum=0 in this case needs empirical testing.

**Fix:** Test empirically with a crafted packet whose bytes sum to 256. If rejected, pad with a 0ms `SET_WAIT_CMD` to shift the checksum.

---

### M-2: `struct.unpack_from('L', ...)` Reads 8 Bytes on 64-bit Linux
**File:** `pydobot/pydobot/dobot.py:45, 100`

`'L'` = native unsigned long = 8 bytes on LP64 Linux. Dobot sends 4-byte uint32. Raises `struct.error` if params is only 4 bytes.

**Fix:** Use `'<I'` (little-endian unsigned 32-bit, always 4 bytes).

---

### M-3: np.vstack Shape Error for Adjacent-Node sknw Edges
**File:** `main.py:91`, `main_cp.py:84`

When `data["pts"]` is empty with shape `(0,)` instead of `(0, 2)`, `np.vstack` produces a 1-D array. `pts[:, ::-1]` then raises `IndexError`. Happens for images with thin strokes producing adjacent skeleton nodes.

**Fix:**
```python
node_u = graph.nodes[u]["o"].reshape(1, 2)
node_v = graph.nodes[v]["o"].reshape(1, 2)
inner  = data["pts"].reshape(-1, 2)
pts    = np.vstack([node_u, inner, node_v])
```

---

### M-4: ZeroDivisionError If Image Is 1 Pixel Wide or Tall
**File:** `main.py:147`, `main_cp.py:138`

`px_col / (img_w - 1)` → division by zero for a 1×N or N×1 image.

**Fix:** Guard in `load_image`:
```python
if img_w < 2 or img_h < 2:
    raise ValueError(f"Image too small: {img_w}×{img_h}")
```

---

### M-5: CP Pen-Lift Ordering Is Correct but Comment Is Misleading
**File:** `main_cp.py:256–264`

Speed change and PTP pen-lift are queued after all CP commands — ordering is correct. But the comment "wait=True drains the CP queue" is inaccurate. It is the *queued position* of the PTP command that provides the ordering guarantee, not `wait=True` itself.

**Fix:** Update comment to: *"PTP command is enqueued after all CP commands; waiting for its execution index implicitly drains the CP queue."*

---

## LOW

---

### L-1: threading.Thread.__init__ on Non-Thread Class
**File:** `pydobot/pydobot/dobot.py:16`

`Dobot` does not inherit from `threading.Thread`. `threading.Thread.__init__(self)` is called anyway — leftover from a removed background-thread design. Harmless but wrong.

**Fix:** Remove the line entirely.

---

### L-2: 100ms Sleep Per Command Throttles Throughput
**File:** `pydobot/pydobot/dobot.py:117`

Every command sleeps 100ms before writing, including `wait=False` draw points. For a stroke with 50 draw points: 5 seconds of host-side delay just to enqueue. The arm sits idle waiting for the next command.

**Fix:** Reduce to 10–20ms and test whether the arm still responds reliably.

---

### L-3: O(n²) Stroke Ordering
**File:** `main.py:108–133`, `main_cp.py:99–124`

Fine up to ~500 strokes. For dense images with 5000+ strokes, can take minutes before drawing starts.

**Fix:** Use `scipy.spatial.KDTree` for O(n log n) nearest-neighbour lookup.

---

### L-4: No Settling Delay in calibrate.py After Unlock
**File:** `calibrate.py:25–29`

`arm.pose()` called immediately after Enter is pressed. Arm may still be drifting from the unlock-button nudge, recording a slightly wrong position.

**Fix:** Add `time.sleep(0.5)` inside `record()` after the input prompt, before `arm.pose()`.

---

### L-5: Relative Paths Break When Run From Wrong Directory
**File:** `main.py:2`, `calibrate.py:22`

`"images/Butterfly.jpeg"` and `"main.py"` resolve from the process CWD. Running from a parent directory silently uses wrong paths.

**Fix:** Anchor to the script's own directory:
```python
from pathlib import Path
ROOT = Path(__file__).parent
IMAGE_PATH = ROOT / "images" / "Butterfly.jpeg"
```

---

### L-6: MOVL_XYZ for Draw Points More Likely to Alarm Near Workspace Edges
**File:** `pydobot/pydobot/dobot.py:297` (`move_to`)

`move_to()` always uses `MOVL_XYZ` (Cartesian-linear). Near workspace extremes, linear Cartesian paths can pass through kinematic singularities that `MOVJ_XYZ` avoids. Known trade-off: MOVL = straight lines on paper, MOVJ = more robust at edges.

---

### L-7: 1-Point Strokes Are a Silent No-Op
**File:** `main.py:252`, `main_cp.py` (draw loop)

If a 1-point stroke passes the length filter, the draw loop body never executes. Pen goes down and immediately up at the same point — leaves a dot and wastes time. Currently blocked by `MIN_STROKE_LEN > 0` so not a live issue, but the assumption is undocumented.

---

## Summary Table

| ID  | Severity | File | Description |
|-----|----------|------|-------------|
| C-1 | Critical | dobot.py:88–92 | Lock not released on SerialException → permanent deadlock |
| C-2 | Critical | main.py:241 / main_cp.py:248 | Unprotected recovery commands crash on unresponsive arm |
| C-3 | Critical | dobot.py:76–92 | Stale buffered response contaminates next command parse |
| H-1 | High | message.py:10–15 | No length/header check on received bytes → IndexError |
| H-2 | High | calibrate.py:57 | Z_UP regex appends duplicate comment on every calibration run |
| H-3 | High | calibrate.py:22 | main_cp.py never patched by calibrate — calibration drifts |
| H-4 | High | dobot.py:104–112 | No timeout in wait loop → infinite hang on motor stall |
| H-5 | High | main.py / main_cp.py | arm.close() not in finally → serial port leak on exception |
| H-6 | High | dobot.py:107 | uint32 queue index wrap-around → infinite wait |
| M-1 | Medium | message.py:37–39 | Checksum=0 still possible when payload sum % 256 == 0 |
| M-2 | Medium | dobot.py:45,100 | struct 'L' = 8 bytes on 64-bit Linux; should be '<I' |
| M-3 | Medium | main.py:91 / main_cp.py:84 | np.vstack shape error if sknw data["pts"] shape is (0,) |
| M-4 | Medium | main.py:147 / main_cp.py:138 | ZeroDivisionError in px_to_mm for 1-pixel-wide/tall image |
| M-5 | Medium | main_cp.py:256–264 | CP+PTP ordering correct but comment is misleading |
| L-1 | Low | dobot.py:16 | threading.Thread.__init__ called on non-Thread class |
| L-2 | Low | dobot.py:117 | 100ms sleep per command severely throttles draw throughput |
| L-3 | Low | main.py:108 / main_cp.py:99 | O(n²) stroke reorder; slow for complex images |
| L-4 | Low | calibrate.py:25–29 | No settling delay after unlock; pose sampled while arm drifting |
| L-5 | Low | main.py:2, calibrate.py:22 | Relative paths break when run from a different directory |
| L-6 | Low | dobot.py:297 | MOVL_XYZ for draw points more likely to alarm at workspace edges |
| L-7 | Low | main.py:252 | 1-point strokes cause pen-down/up with no movement |
