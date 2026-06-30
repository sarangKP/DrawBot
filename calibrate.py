"""
Calibration script for Dobot Magician.

Hold the unlock button on the arm, move it to each position, release,
then press Enter to record. Five positions total:

  1. Z_UP       — pen fully lifted (safe travel height)
  2. Top-Left   — paper corner mapping to image pixel (0, 0)
  3. Top-Right  — paper corner mapping to image pixel (w, 0)
  4. Bot-Left   — paper corner mapping to image pixel (0, h)
  5. Bot-Right  — paper corner mapping to image pixel (w, h)

For corners 2-5: lower the pen until it just touches the paper surface.

On completion, CAL_ARM_XY, CAL_Z_CORNERS, and Z_UP in main.py are updated.
"""

import re
import sys

PORT      = "/dev/ttyUSB0"
MAIN_FILE = "main.py"


def record(arm, label: str) -> tuple[float, float, float]:
    input(f"\n  [{label}]  Move arm → press Enter to record ... ")
    x, y, z, r, *_ = arm.pose()
    print(f"    x={x:.4f}  y={y:.4f}  z={z:.4f}")
    return x, y, z


def patch_main(z_up: float,
               corners_xy: list[tuple[float, float]],
               corners_z: list[float]) -> None:
    with open(MAIN_FILE) as f:
        src = f.read()

    tl, tr, bl, br = corners_xy
    ztl, ztr, zbl, zbr = corners_z

    new_xy = (
        "CAL_ARM_XY = [\n"
        f"    ({tl[0]},    {tl[1]}),   # TL\n"
        f"    ({tr[0]}, {tr[1]}),  # TR\n"
        f"    ({bl[0]},  {bl[1]}),  # BL\n"
        f"    ({br[0]},  {br[1]}),  # BR\n"
        "]"
    )
    src = re.sub(r"CAL_ARM_XY\s*=\s*\[.*?\]", new_xy, src, flags=re.DOTALL)

    new_z = (
        f"CAL_Z_CORNERS = [{ztl}, {ztr},\n"
        f"                 {zbl}, {zbr}]"
    )
    src = re.sub(r"CAL_Z_CORNERS\s*=\s*\[.*?\]", new_z, src, flags=re.DOTALL)

    src = re.sub(
        r"Z_UP\s*=\s*[^\n#]+",
        f"Z_UP         = {z_up}   # pen-lifted height",
        src,
    )

    with open(MAIN_FILE, "w") as f:
        f.write(src)

    print(f"\n✓ {MAIN_FILE} updated.")


def main():
    import pydobot

    print(f"Connecting on {PORT} ...")
    arm = pydobot.Dobot(port=PORT, verbose=False)
    print("Connected.\n")
    print("Hold the unlock button on the arm to move it freely.")
    print("TL=image top-left  TR=top-right  BL=bot-left  BR=bot-right\n")

    _, _, z_up = record(arm, "Z_UP — lift pen to safe travel height")

    corners_xy, corners_z = [], []
    for label in ("TL — top-left corner, pen touching paper",
                  "TR — top-right corner, pen touching paper",
                  "BL — bot-left corner, pen touching paper",
                  "BR — bot-right corner, pen touching paper"):
        x, y, z = record(arm, label)
        corners_xy.append((x, y))
        corners_z.append(z)

    arm.close()

    print("\n── Summary ──────────────────────────────────────")
    print(f"  Z_UP : {z_up:.4f}")
    for name, (cx, cy), cz in zip(("TL", "TR", "BL", "BR"), corners_xy, corners_z):
        print(f"  {name}   xy=({cx:.4f}, {cy:.4f})  z={cz:.4f}")

    patch_main(z_up, corners_xy, corners_z)


if __name__ == "__main__":
    main()
