"""Image characteristic analysis and automatic binarization pipeline selection.

main_cp_v2.py calls binarize_auto(gray) in place of a fixed Otsu threshold.
analyze() measures the image (noise, contrast, histogram bimodality,
illumination evenness, ink coverage, stroke width); binarize_auto() then
assembles a matching pipeline: optional denoise -> optional CLAHE ->
threshold method (Otsu / adaptive / auto-Canny) -> morphological close ->
small-speck removal.
"""

from dataclasses import dataclass, field

import cv2
import numpy as np

# ── Decision thresholds ──────────────────────────────────────────────────────
NOISE_LIGHT      = 4.0    # estimated sigma above which a 3px median blur runs
NOISE_HEAVY      = 10.0   # sigma above which a 5px median blur runs instead
LOW_CONTRAST     = 40.0   # gray std-dev below which CLAHE is applied
UNEVEN_ILLUM     = 0.15   # background spread ratio above which adaptive threshold
STRONG_BIMODALITY = 0.90  # above this a global threshold separates cleanly anyway
PHOTO_INK_RATIO  = 0.35   # dark-pixel fraction above which image is photo-like
PHOTO_BIMODALITY = 0.55   # bimodality below which histogram is not ink-vs-paper
DESPECK_FACTOR   = 0.6    # despeck min-area = (stroke_width_px * this)**2 —
                          # full stroke_width**2 was killing short legit marks


@dataclass
class ImageProfile:
    contrast: float           # std-dev of gray values
    bimodality: float         # Otsu between-class variance / total variance (0..1)
    noise_sigma: float        # Immerkaer noise estimate
    ink_ratio: float          # fraction of pixels dark side of Otsu
    illum_unevenness: float   # (p95 - p5) of blurred background / 255
    stroke_width_px: float    # mean ink stroke width from distance transform
    kind: str = "lineart"     # "lineart" or "photo"
    steps: list[str] = field(default_factory=list)

    def summary(self) -> str:
        return (
            f"Image profile: kind={self.kind}  contrast={self.contrast:.0f}  "
            f"bimodality={self.bimodality:.2f}  noise={self.noise_sigma:.1f}  "
            f"ink={self.ink_ratio:.2f}  illum={self.illum_unevenness:.2f}  "
            f"stroke_w={self.stroke_width_px:.1f}px\n"
            f"Pipeline: {' -> '.join(self.steps) if self.steps else '(none)'}"
        )


# ── Metrics ──────────────────────────────────────────────────────────────────

def _noise_sigma(gray: np.ndarray) -> float:
    """Immerkaer fast noise estimate: sigma from a Laplacian-difference kernel."""
    h, w = gray.shape
    if h < 3 or w < 3:
        return 0.0
    kernel = np.array([[1, -2, 1], [-2, 4, -2], [1, -2, 1]], dtype=np.float64)
    lap = cv2.filter2D(gray.astype(np.float64), -1, kernel)
    return float(
        np.sqrt(np.pi / 2.0) / (6.0 * (w - 2) * (h - 2))
        * np.abs(lap[1:-1, 1:-1]).sum()
    )


def _bimodality(gray: np.ndarray) -> float:
    """Otsu between-class variance over total variance. ~1 for clean ink-vs-paper."""
    total_var = float(gray.std()) ** 2
    if total_var == 0.0:
        return 0.0
    t, _ = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    dark = gray <= t
    w0 = float(dark.mean())
    w1 = 1.0 - w0
    if w0 == 0.0 or w1 == 0.0:
        return 0.0
    mu0 = float(gray[dark].mean())
    mu1 = float(gray[~dark].mean())
    return w0 * w1 * (mu0 - mu1) ** 2 / total_var


def _illum_unevenness(gray: np.ndarray) -> float:
    """Spread of the low-frequency background: 0 = flat lighting."""
    small = cv2.resize(gray, (64, max(1, round(64 * gray.shape[0] / gray.shape[1]))),
                       interpolation=cv2.INTER_AREA)
    bg = cv2.GaussianBlur(small, (0, 0), sigmaX=8)
    lo, hi = np.percentile(bg, [5, 95])
    return float((hi - lo) / 255.0)


def _stroke_width(mask: np.ndarray) -> float:
    """Mean ink stroke width in px. For a strip of width w the distance
    transform averages w/4 over the strip, so width ~= 4 * mean(dt)."""
    ink = (mask > 0).astype(np.uint8)
    if ink.sum() == 0:
        return 1.0
    dt = cv2.distanceTransform(ink, cv2.DIST_L2, 3)
    return float(4.0 * dt[ink > 0].mean())


def analyze(gray: np.ndarray) -> ImageProfile:
    _, prelim = cv2.threshold(gray, 0, 255,
                              cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    return ImageProfile(
        contrast=float(gray.std()),
        bimodality=_bimodality(gray),
        noise_sigma=_noise_sigma(gray),
        ink_ratio=float((prelim > 0).mean()),
        illum_unevenness=_illum_unevenness(gray),
        stroke_width_px=_stroke_width(prelim),
    )


# ── Pipeline selection ───────────────────────────────────────────────────────

def _remove_specks(mask: np.ndarray, min_area: int) -> np.ndarray:
    num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num <= 1:
        return mask
    keep = stats[:, cv2.CC_STAT_AREA] >= min_area
    keep[0] = False  # background
    return np.where(keep[labels], 255, 0).astype(np.uint8)


def binarize_auto(
    gray: np.ndarray,
) -> tuple[np.ndarray, ImageProfile, np.ndarray]:
    """Measure the image and binarize it with a pipeline matched to it.

    Returns (ink_mask, profile, pre_despeck_mask). ink_mask is uint8
    {0, 255}, ready for skeletonize + sknw like the old fixed-Otsu
    detect_edges(). pre_despeck_mask is the mask right before the
    connected-component speck filter, for previewing what despeck removes.
    """
    prof = analyze(gray)
    img = gray

    if prof.noise_sigma > NOISE_HEAVY:
        img = cv2.medianBlur(img, 5)
        prof.steps.append("median-blur-5")
    elif prof.noise_sigma > NOISE_LIGHT:
        img = cv2.medianBlur(img, 3)
        prof.steps.append("median-blur-3")

    if prof.contrast < LOW_CONTRAST:
        img = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(img)
        prof.steps.append("clahe")

    if prof.ink_ratio > PHOTO_INK_RATIO and prof.bimodality < PHOTO_BIMODALITY:
        # Photo-like: Otsu would return huge blobs, so fall back to
        # median-anchored auto-Canny edge extraction.
        prof.kind = "photo"
        # Floor the thresholds: on dark images the median collapses toward 0
        # and unfloored Canny passes every noise gradient (near-white mask).
        med = float(np.median(img))
        hi = int(min(255.0, max(1.33 * med, 40.0)))
        lo = int(max(0.66 * med, hi / 2))
        mask = cv2.Canny(img, lo, hi)
        prof.steps.append(f"canny-auto({lo},{hi})")
        close_px = 2
    elif (prof.illum_unevenness > UNEVEN_ILLUM
          and prof.bimodality < STRONG_BIMODALITY):
        # Uneven lighting defeats a global threshold — but only when it
        # actually erodes bimodality; a strongly bimodal histogram still
        # splits cleanly with Otsu. Window size scales with stroke width
        # so each window still sees paper around the ink.
        block = int(round(prof.stroke_width_px * 4)) | 1
        block = min(51, max(15, block))
        mask = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY_INV, block, 7)
        prof.steps.append(f"adaptive-gaussian(block={block})")
        close_px = 1
    else:
        _, mask = cv2.threshold(img, 0, 255,
                                cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        prof.steps.append("otsu")
        close_px = 1

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                       (2 * close_px + 1,) * 2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    prof.steps.append(f"morph-close-{close_px}")

    pre_despeck = mask.copy()

    # Kill isolated specks smaller than a fraction of one stroke-width dot.
    # Canny lines are 1px wide, so the lineart heuristic would erase them.
    if prof.kind == "photo":
        min_area = 8
    else:
        min_area = max(4, int(round((prof.stroke_width_px * DESPECK_FACTOR) ** 2)))
    mask = _remove_specks(mask, min_area)
    prof.steps.append(f"despeck(area<{min_area})")

    return mask, prof, pre_despeck
