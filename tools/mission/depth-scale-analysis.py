#!/usr/bin/env python3
"""Pixel-level depth scale comparison: ONNX vs GT.

Answers the core question visible in debug_depth.mp4:
  "ONNX assigns far depth to pixels where GT says CLOSE — by how much?"

What frame0 shows (2026-07-16 run):
  - GT  min=0.1 m,  max=65504 m (AirSim sky sentinel — treated as invalid)
  - ONNX min=5.7 m, max=79.2 m  (no sky sentinel — bounded metric output)
  - ONNX cannot detect objects closer than ~5-6 m (explains missing yellow mask)
  - Ground at bottom of frame: GT=0-2 m → ONNX=5-15 m (~5-8x scale error at close range)
  - Buildings/terrain at mid-range: scale error is ~2x (matches JSONL histogram finding)
  - ONNX depth is smooth (no crisp object edges) → temporal noise in video is intrinsic

Usage:
  python3 tools/mission/depth-scale-analysis.py \\
      /tmp/dedalus_gt_frame0_depth.npy \\
      /tmp/dedalus_frame0_depth.npy

GT  map: 256×144 float32, metres. 0=invalid, 65504=AirSim sky sentinel (filtered).
ONNX map: 518×518 float32, raw metric depth metres (>0, no sentinel).

Both cover the same 84° × 53.72° FOV.  We resample onto a shared 40×22 angular
grid (matching the evidence grid) to get paired samples at identical bearings.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_PLT = True
except ImportError:
    HAS_PLT = False


HFOV_DEG  = 84.0
VFOV_DEG  = 53.72
GRID_COLS = 40
GRID_ROWS = 22

# AirSim uses 65504 m as the sky/no-hit sentinel in DepthPlanar.
# Filter anything above this threshold as invalid (sky or beyond-max-range).
GT_MAX_VALID_M = 1000.0

PNG_OUT = Path("/tmp/depth_scale_analysis.png")


def cell_min_depth(depth_m: np.ndarray, grid_cols: int, grid_rows: int) -> np.ndarray:
    """Block-minimum depth per grid cell (mirrors depth_projection_kernel logic).

    Returns shape (grid_rows, grid_cols) with 0 meaning no valid pixel.
    """
    h, w = depth_m.shape
    bw = w // grid_cols
    bh = h // grid_rows
    out = np.zeros((grid_rows, grid_cols), dtype=np.float32)
    for r in range(grid_rows):
        for c in range(grid_cols):
            patch = depth_m[r*bh:(r+1)*bh, c*bw:(c+1)*bw]
            valid = patch[patch > 0]
            if valid.size > 0:
                out[r, c] = valid.min()
    return out


def main() -> None:
    if len(sys.argv) < 3:
        sys.exit(f"Usage: {sys.argv[0]} gt.npy onnx.npy")

    gt_path   = Path(sys.argv[1])
    onnx_path = Path(sys.argv[2])

    gt_raw   = np.load(gt_path).astype(np.float32)    # metres, 0=invalid, 65504=sky
    onnx_raw = np.load(onnx_path).astype(np.float32)  # direct metric depth metres

    # ONNX engine stores inverse_depth internally; the NPY is the raw ONNX output
    # which is DIRECT METRIC DEPTH (metric_depth: true).  Values <0.01 (sky/OOD)
    # are stored as 0 (invalid) by the engine.  Treat onnx_raw directly as metres.
    # If your build stores inverse_depth in the NPY, invert here:
    #   onnx_raw = np.where(onnx_raw > 0, 1.0 / onnx_raw, 0)

    # Filter GT sky sentinel (AirSim uses 65504 m for "no hit").
    # Zero out sky pixels so they don't contaminate stats or grid sampling.
    sky_mask = gt_raw > GT_MAX_VALID_M
    gt_filtered = np.where(sky_mask, 0.0, gt_raw)
    n_sky = int(sky_mask.sum())
    sky_pct = n_sky / gt_raw.size * 100

    print(f"\n{'='*72}")
    print(f"  Depth Scale Analysis — ONNX vs GT  (frame 0, ground-level horizontal)")
    print(f"{'='*72}")

    gt_valid_px = gt_filtered[gt_filtered > 0]
    onnx_valid_px = onnx_raw[onnx_raw > 0]
    print(f"\n  GT   shape={gt_raw.shape}  "
          f"sky_pixels={n_sky} ({sky_pct:.0f}%)  "
          f"valid_min={gt_valid_px.min():.2f} m  valid_max={gt_valid_px.max():.2f} m")
    print(f"  ONNX shape={onnx_raw.shape}  "
          f"min={onnx_valid_px.min():.2f} m  max={onnx_valid_px.max():.2f} m")
    print(f"\n  NOTE: Frame 0 = drone on ground, camera horizontal (pitch=0).")
    print(f"  In-flight metrics (pitch~52°) are better captured by the JSONL histogram.")

    # ── Grid-level block-minimum (same logic as the C++ pipeline) ──────────────
    gt_grid   = cell_min_depth(gt_filtered, GRID_COLS, GRID_ROWS)
    onnx_grid = cell_min_depth(onnx_raw,    GRID_COLS, GRID_ROWS)

    # Paired valid cells (both GT and ONNX have a valid pixel in that cell)
    mask = (gt_grid > 0) & (onnx_grid > 0)
    gt_v   = gt_grid[mask]
    onnx_v = onnx_grid[mask]

    n_paired = int(mask.sum())
    total_cells = GRID_COLS * GRID_ROWS
    print(f"\n  Grid: {GRID_COLS}×{GRID_ROWS} = {total_cells} cells")
    print(f"  Paired (both valid): {n_paired} / {total_cells} cells")

    if n_paired == 0:
        sys.exit("ERROR: no paired cells — check that ONNX NPY contains direct metric depth")

    # ── ONNX minimum detection range ───────────────────────────────────────────
    # If ONNX min depth >> 0, the model cannot detect very close objects.
    # This directly causes the missing yellow mask (close-obstacle indicator).
    onnx_min = float(onnx_valid_px.min()) if onnx_valid_px.size > 0 else 0.0
    gt_min   = float(gt_valid_px.min())   if gt_valid_px.size  > 0 else 0.0
    print(f"\n  ONNX minimum detectable depth : {onnx_min:.2f} m")
    print(f"  GT  minimum depth             : {gt_min:.2f} m")
    if onnx_min > 3.0:
        blind_cells = int(np.sum(gt_grid > 0) - np.sum(onnx_grid > 0))
        print(f"  ✗  ONNX cannot detect objects closer than {onnx_min:.1f} m.")
        print(f"     This directly explains the missing yellow mask in debug_depth.mp4.")
        print(f"     Any obstacle within {onnx_min:.1f} m is invisible to the depth model.")

    # ── Depth error statistics ──────────────────────────────────────────────────
    ratio  = onnx_v / gt_v                   # >1 → ONNX over-estimates (thinks FAR)
    error  = onnx_v - gt_v                   # signed error in metres
    abs_err = np.abs(error)

    print(f"\n  Overall depth error (ONNX − GT) across {n_paired} paired cells")
    print(f"  {'─'*68}")
    print(f"  Median ratio ONNX/GT : {np.median(ratio):.2f}×  "
          f"(>1 = ONNX thinks scene is FARTHER)")
    print(f"  Mean signed error    : {error.mean():+.2f} m")
    print(f"  Median signed error  : {np.median(error):+.2f} m")
    print(f"  MAE                  : {abs_err.mean():.2f} m")
    print(f"  RMSE                 : {float(np.sqrt((error**2).mean())):.2f} m")

    # ── Per-GT-range breakdown ──────────────────────────────────────────────────
    bins = [(0, 5, "0–5 m (CLOSE)"),
            (5, 15, "5–15 m"),
            (15, 30, "15–30 m"),
            (30, 999, "30 m+ (FAR)")]

    print(f"\n  Per-depth-range breakdown")
    print(f"  {'─'*68}")
    print(f"  {'GT range':<16}  {'n cells':>8}  {'ONNX/GT ratio':>14}  "
          f"{'ONNX median':>12}  {'GT median':>10}")
    print(f"  {'─'*68}")

    for lo, hi, label in bins:
        sel = (gt_v >= lo) & (gt_v < hi)
        n = int(sel.sum())
        if n == 0:
            print(f"  {label:<16}  {n:>8}  {'—':>14}  {'—':>12}  {'—':>10}")
            continue
        r = ratio[sel]
        o = onnx_v[sel]
        g = gt_v[sel]
        print(f"  {label:<16}  {n:>8}  {np.median(r):>13.2f}×  "
              f"{np.median(o):>11.2f} m  {np.median(g):>9.2f} m")

    # ── Close-range miss analysis ───────────────────────────────────────────────
    close_gt_mask = (gt_v < 5.0)
    n_close = int(close_gt_mask.sum())
    print(f"\n  Close-range miss analysis  (GT < 5 m)")
    print(f"  {'─'*68}")
    if n_close == 0:
        print(f"  No GT cells < 5 m in this frame — try a later frame closer to obstacles.")
    else:
        onnx_at_close = onnx_v[close_gt_mask]
        n_miss = int(np.sum(onnx_at_close > 5.0))
        pct = n_miss / n_close * 100
        print(f"  GT cells < 5 m      : {n_close}")
        print(f"  ONNX predicts > 5 m : {n_miss}  ({pct:.0f}%)   ← yellow-mask misses")
        print(f"  ONNX depth there    : median={np.median(onnx_at_close):.1f} m  "
              f"max={onnx_at_close.max():.1f} m")
        scale_here = np.median(onnx_at_close / gt_v[close_gt_mask])
        print(f"  Scale factor here   : {scale_here:.1f}×  "
              f"(ONNX over-estimates by this factor at close range)")

    # ── Temporal noise proxy: per-pixel depth range in the single frame ─────────
    # We can't measure temporal noise from a single frame, but we can show the
    # ONNX depth variance across the frame compared to GT variance as a proxy
    # for "how much is the model pattern vs scene geometry".
    print(f"\n  Depth texture (frame-level proxy for temporal noise)")
    print(f"  {'─'*68}")
    gt_valid   = gt_raw[gt_raw > 0]
    onnx_valid = onnx_raw[onnx_raw > 0]
    print(f"  GT   std={gt_valid.std():.2f} m  p25={np.percentile(gt_valid,25):.1f}  "
          f"p50={np.percentile(gt_valid,50):.1f}  p75={np.percentile(gt_valid,75):.1f}")
    print(f"  ONNX std={onnx_valid.std():.2f} m  p25={np.percentile(onnx_valid,25):.1f}  "
          f"p50={np.percentile(onnx_valid,50):.1f}  p75={np.percentile(onnx_valid,75):.1f}")

    # ── Interpretation ──────────────────────────────────────────────────────────
    median_ratio = float(np.median(ratio))
    print(f"\n  Interpretation")
    print(f"  {'─'*68}")
    if median_ratio > 2.0:
        print(f"  ✗  ONNX over-estimates depth by {median_ratio:.1f}× overall.")
        print(f"     The model's metric scale is wrong for this scene/altitude.")
        print(f"     Root cause: domain gap (AirSim synthetic vs real training data)")
        print(f"     or incorrect altitude assumption in the metric head.")
        print(f"     Fix options: (1) fine-tune with AirSim depth GT,")
        print(f"                  (2) apply post-hoc scale correction,")
        print(f"                  (3) switch to a model trained on aerial data.")
    elif median_ratio > 1.3:
        print(f"  △  ONNX over-estimates depth by {median_ratio:.1f}×.")
        print(f"     Partial scale error — check per-range table; may be range-dependent.")
    else:
        print(f"  ✓  Depth scale broadly correct (median ratio {median_ratio:.2f}×).")
        print(f"     Yellow-mask mismatch is from temporal noise, not metric scale error.")

    # ── Plots ───────────────────────────────────────────────────────────────────
    if not HAS_PLT:
        print(f"\n  (matplotlib not available — skipping PNG output)")
        print(f"\n{'='*72}\n")
        return

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle("ONNX vs GT Depth Analysis — Frame 0 (ground-level horizontal)",
                 fontsize=14)

    # 1. GT depth map (sky masked to 0, capped at 60m for display)
    ax = axes[0, 0]
    gt_display = np.clip(gt_filtered, 0, 60)
    im = ax.imshow(gt_display, vmin=0, vmax=60, cmap="plasma", aspect="auto")
    ax.set_title(f"GT depth (sky masked)  {gt_raw.shape[1]}×{gt_raw.shape[0]}")
    ax.set_xlabel("pixel x"); ax.set_ylabel("pixel y")
    plt.colorbar(im, ax=ax, label="depth (m)")

    # 2. ONNX depth map
    ax = axes[0, 1]
    im = ax.imshow(onnx_raw, vmin=0, vmax=60, cmap="plasma", aspect="auto")
    ax.set_title(f"ONNX depth map  {onnx_raw.shape[1]}×{onnx_raw.shape[0]}")
    ax.set_xlabel("pixel x"); ax.set_ylabel("pixel y")
    plt.colorbar(im, ax=ax, label="depth (m)")

    # 3. ONNX/GT ratio per cell (grid level)
    ratio_grid = np.zeros((GRID_ROWS, GRID_COLS), dtype=np.float32)
    ratio_grid[mask] = ratio
    ax = axes[0, 2]
    im = ax.imshow(ratio_grid, vmin=0, vmax=5, cmap="RdYlGn_r", aspect="auto")
    ax.set_title(f"ONNX/GT ratio per grid cell  (red=ONNX too far)")
    ax.set_xlabel("grid col"); ax.set_ylabel("grid row")
    plt.colorbar(im, ax=ax, label="ratio")

    # 4. GT depth histogram (sky excluded)
    ax = axes[1, 0]
    gt_valid_plot = gt_filtered[gt_filtered > 0]
    ax.hist(gt_valid_plot, bins=50, range=(0, 60), color="steelblue", alpha=0.8)
    ax.set_title(f"GT depth histogram  (sky {sky_pct:.0f}% excluded)")
    ax.set_xlabel("depth (m)"); ax.set_ylabel("pixel count")
    ax.axvline(5, color="gold", linewidth=2, linestyle="--", label="5 m (yellow mask)")
    ax.legend(fontsize=8)

    # 5. ONNX depth histogram with ONNX-min marker
    ax = axes[1, 1]
    ax.hist(onnx_valid_px, bins=50, range=(0, 60), color="coral", alpha=0.8)
    ax.set_title("ONNX depth histogram")
    ax.set_xlabel("depth (m)"); ax.set_ylabel("pixel count")
    ax.axvline(5, color="gold", linewidth=2, linestyle="--", label="5 m (yellow mask)")
    ax.axvline(onnx_min, color="red", linewidth=2, linestyle=":",
               label=f"ONNX min={onnx_min:.1f} m")
    ax.legend(fontsize=8)

    # 6. Scatter: GT vs ONNX at grid cells
    ax = axes[1, 2]
    sc = ax.scatter(gt_v, onnx_v, alpha=0.5, s=20, c=np.log1p(abs(onnx_v - gt_v)),
                    cmap="hot_r")
    lim = max(gt_v.max(), onnx_v.max()) * 1.05
    ax.plot([0, lim], [0, lim], "k--", linewidth=1, label="GT = ONNX (ideal)")
    ax.set_xlim(0, lim); ax.set_ylim(0, lim)
    ax.set_title("GT vs ONNX depth per grid cell")
    ax.set_xlabel("GT depth (m)"); ax.set_ylabel("ONNX depth (m)")
    ax.legend(fontsize=8)
    plt.colorbar(sc, ax=ax, label="log |error|")

    plt.tight_layout()
    fig.savefig(PNG_OUT, dpi=120)
    print(f"\n  Plot saved → {PNG_OUT}")
    print(f"  Open:  open {PNG_OUT}")

    print(f"\n{'='*72}\n")


if __name__ == "__main__":
    main()
