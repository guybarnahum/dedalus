#!/usr/bin/env python3
"""Analyze a saved depth capture (.npy pair) from the Dedalus depth pipeline.

Two capture sources are supported:

  cap_*_depth.npy   — written by DEDALUS_DEPTH_CAPTURE_DIR (gimbal-gated frames).
                      Filenames encode pitch and close-pct: cap_1_p24_c44_depth.npy
                      → capture 1, pitch 24°, 44% close pixels.

  dedalus_frame0_depth.npy — written by ONNXDepthEngine on first inference.

Storage convention
------------------
  Pre-fix builds (before inverse-depth engine fix):
      stored value = raw model output = direct metres  →  depth_m = stored
  Post-fix builds (default after rebuild):
      stored value = inverse_depth = 1/raw_metres      →  depth_m = scale / stored

Use --inverse-depth for post-fix captures (will be the default once all missions
are rebuilt).  The tool auto-detects by checking whether the median stored value
is above 1.5 m (likely direct metres) or below (likely inverse depth).

Usage
-----
  # single capture (auto-detects paired rgb, auto-detects convention):
  python3 tools/perception/probe_depth_frame0.py depth_errors/cap_1_p24_c44_depth.npy

  # force inverse-depth convention (post-fix captures):
  python3 tools/perception/probe_depth_frame0.py cap_1_p24_c44_depth.npy --inverse-depth

  # force direct-metres convention (pre-fix captures):
  python3 tools/perception/probe_depth_frame0.py cap_1_p24_c44_depth.npy --direct-depth

  # explicit rgb path and output path:
  python3 tools/perception/probe_depth_frame0.py cap_1_p24_c44_depth.npy \\
      --rgb cap_1_p24_c44_rgb.npy --out cap_1_analysis.png

  # process all captures in a directory:
  for f in depth_errors/cap_*_depth.npy; do
      python3 tools/perception/probe_depth_frame0.py "$f"
  done
"""

import argparse
import os
import re
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LogNorm

SCALE = 1.0   # depth_m = SCALE / inverse_depth  (1.0 for metric model, scale=1.0)
MIN_DEPTH_M = 0.5   # current visual_onnx.min_depth_m


def banner(title: str) -> None:
    print(f'\n{"="*60}')
    print(f'  {title}')
    print(f'{"="*60}')


def detect_convention(stored: np.ndarray) -> str:
    """Detect pre-fix (direct metres) vs post-fix (inverse depth) from stored values.

    Pre-fix engine stored raw model output directly (direct metres).  The OOD AirSim
    model outputs up to ~40m for the far ground → stored.max() can reach 40.
    Post-fix engine stores 1/raw:
      - Intermediate post-fix (sky→1/1e-3=1000): stored.max() > 100
      - Latest post-fix (sky→0, arm→1/0.1=10): stored.max() ≤ 10

    Detection rule:
      stored.max() > 100   → intermediate post-fix (sky=999.9999) → inverse
      stored.max() > 5     → pre-fix direct (OOD far-ground model output > 5 m)
      stored.max() ≤ 5     → latest post-fix → inverse
    """
    mx = float(stored.max())
    if mx > 100.0:
        return 'inverse'   # intermediate post-fix: sky pixels clamped to 1/1e-3 = 1000
    if mx > 5.0:
        return 'direct'    # pre-fix: model outputs direct metres, far ground up to ~40 m
    return 'inverse'       # latest post-fix: no sky clamping, inverse depth stored


def to_depth_m(stored: np.ndarray, convention: str) -> np.ndarray:
    """Convert stored values to depth_m.

    direct  → depth_m = stored  (pre-fix captures: raw model output in metres)
    inverse → depth_m = 1/stored, with stored > 100 treated as invalid/sky
              (post-fix: engine stores 1/raw; sky pixels clamped to 1/1e-3=1000
               by the intermediate fix, now marked as 0 by the latest fix)
    """
    if convention == 'direct':
        return stored.astype(np.float32)
    else:
        # stored > 100  →  sky/OOD invalid (pre-kMinValidRaw fix clamped sky to 1000)
        valid = (stored > 1e-6) & (stored <= 100.0)
        return np.where(valid, SCALE / np.maximum(stored, 1e-6), np.nan).astype(np.float32)


def parse_filename_meta(depth_path: Path) -> dict:
    """Extract pitch_deg and close_pct from cap_N_pPP_cCC_depth.npy filenames."""
    m = re.search(r'_p(\d+)_c(\d+)', depth_path.stem)
    if m:
        return {'pitch_deg': int(m.group(1)), 'close_pct': int(m.group(2))}
    return {}


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('depth_npy', type=Path,
                    help='Path to _depth.npy file')
    ap.add_argument('--rgb', type=Path, default=None,
                    help='Path to paired _rgb.npy (auto-detected if omitted)')
    ap.add_argument('--out', type=Path, default=None,
                    help='Output PNG path (default: <depth_npy stem>_analysis.png)')
    group = ap.add_mutually_exclusive_group()
    group.add_argument('--inverse-depth', dest='convention', action='store_const',
                       const='inverse',
                       help='Stored values are inverse_depth=1/depth_m (post-fix captures)')
    group.add_argument('--direct-depth', dest='convention', action='store_const',
                       const='direct',
                       help='Stored values are direct depth in metres (pre-fix captures)')
    args = ap.parse_args()

    depth_path: Path = args.depth_npy.resolve()
    if not depth_path.exists():
        print(f'ERROR: not found: {depth_path}', file=sys.stderr)
        return 1

    # Auto-detect paired RGB
    rgb_path: Path = args.rgb or depth_path.parent / depth_path.name.replace('_depth.npy', '_rgb.npy')
    has_rgb = rgb_path.exists()
    if not has_rgb:
        print(f'  (no RGB file at {rgb_path} — depth-only analysis)', file=sys.stderr)

    # Output path
    out_path: Path = args.out or depth_path.parent / depth_path.name.replace('_depth.npy', '_analysis.png')

    # Load
    stored = np.load(str(depth_path)).astype(np.float32)
    rgb = np.load(str(rgb_path)).astype(np.uint8) if has_rgb else None

    # Determine convention
    convention = args.convention or detect_convention(stored)
    print(f'Convention: {convention}  (median stored={np.median(stored):.3f})')

    depth_m = to_depth_m(stored, convention)
    valid   = np.isfinite(depth_m) & (depth_m > 0)
    dv      = depth_m[valid].flatten()

    H_d, W_d = stored.shape
    H_r, W_r = (rgb.shape[:2] if rgb is not None else (H_d, W_d))

    # Filename metadata
    meta = parse_filename_meta(depth_path)
    pitch_deg  = meta.get('pitch_deg', '?')
    close_pct  = meta.get('close_pct', '?')

    # ------------------------------------------------------------------ #
    #  Stats                                                              #
    # ------------------------------------------------------------------ #
    banner(f'Depth capture: {depth_path.name}')
    print(f'  Shape:    {H_d}×{W_d}  |  convention: {convention}')
    print(f'  Pitch:    {pitch_deg}°  |  close_pct at capture: {close_pct}%')
    print(f'  stored  : min={stored.min():.4f}  max={stored.max():.4f}  '
          f'mean={stored.mean():.4f}  median={np.median(stored):.4f}')
    print(f'  depth_m : min={dv.min():.3f}m  max={dv.max():.1f}m  '
          f'mean={dv.mean():.3f}m  median={np.median(dv):.3f}m')

    banner('depth_m distribution')
    ntot = dv.size
    bins = [
        (0.0,   0.5,  f'< 0.5m  (filtered: below min_depth_m={MIN_DEPTH_M}m)'),
        (0.5,   1.0,  '0.5–1m  (valid close range)'),
        (1.0,   5.0,  '1–5m    (valid mid range)'),
        (5.0,  20.0,  '5–20m   (valid obstacle range)'),
        (20.0, 60.0,  '20–60m  (valid far range)'),
        (60.0, 1e9,   '>60m    (sky / beyond max_depth_m)'),
    ]
    print(f'  {"range":>22}  {"pixels":>7}  {"pct":>6}')
    print(f'  {"-"*42}')
    for lo, hi, label in bins:
        cnt = int(np.sum((dv >= lo) & (dv < hi)))
        print(f'  [{lo:5.1f} – {hi:6.1f}m]   {cnt:>7d}   {100.*cnt/ntot:>5.1f}%  {label}')

    # Close-pixel spatial breakdown
    close_mask = depth_m < MIN_DEPTH_M
    n_close = int(close_mask.sum())
    if n_close > 0:
        banner(f'Spatial: depth_m < {MIN_DEPTH_M}m  ({100.*n_close/(H_d*W_d):.1f}% of all pixels)')
        ys, xs = np.where(close_mask)
        cy, cx = float(np.mean(ys)) / H_d, float(np.mean(xs)) / W_d
        sy, sx = float(np.std(ys))  / H_d, float(np.std(xs))  / W_d
        print(f'  centroid: ({cx:.3f}, {cy:.3f})  spread: sx={sx:.3f}  sy={sy:.3f}')
        hh, hw = H_d // 2, W_d // 2
        q = {
            'TL': int(np.sum((ys < hh) & (xs < hw))),
            'TR': int(np.sum((ys < hh) & (xs >= hw))),
            'BL': int(np.sum((ys >= hh) & (xs < hw))),
            'BR': int(np.sum((ys >= hh) & (xs >= hw))),
        }
        print(f'  quadrant TL={q["TL"]}  TR={q["TR"]}  BL={q["BL"]}  BR={q["BR"]}')
        left_pct = 100. * (q['TL'] + q['BL']) / max(n_close, 1)
        top_pct  = 100. * (q['TL'] + q['TR']) / max(n_close, 1)
        print(f'  left-half: {left_pct:.1f}%  top-half: {top_pct:.1f}%')
        if abs(left_pct - 50) < 15:
            print('  → symmetric left/right: consistent with prop layout')
        else:
            print('  → asymmetric: possibly one prop or OOD region')

    # ------------------------------------------------------------------ #
    #  Figure                                                             #
    # ------------------------------------------------------------------ #
    ncols = 4
    nrows = 3 if rgb is not None else 2
    fig = plt.figure(figsize=(22, 5 * nrows))
    gs  = gridspec.GridSpec(nrows, ncols, figure=fig, hspace=0.40, wspace=0.32)
    row = 0

    # Row 0: RGB + depth heatmap
    if rgb is not None:
        ax = fig.add_subplot(gs[row, 0:2])
        ax.imshow(rgb)
        ax.set_title(f'RGB  ({W_r}×{H_r})', fontsize=10)
        ax.axis('off')

    ax = fig.add_subplot(gs[row, 2:4] if rgb is not None else gs[row, 0:2])
    d_disp = np.clip(depth_m, 0.05, 60.0)
    d_disp = np.where(valid, d_disp, 60.0)
    im = ax.imshow(d_disp, cmap='plasma_r', norm=LogNorm(vmin=0.05, vmax=60))
    plt.colorbar(im, ax=ax, label='depth_m (log scale)')
    ax.set_title(f'depth_m  dark=close  bright=far  ({W_d}×{H_d})', fontsize=10)
    ax.axis('off')
    row += 1

    # Row 1: zone overlay + depth histogram
    ax2 = fig.add_subplot(gs[row, 0:2])
    grey = np.clip((1.0 - np.log1p(d_disp) / np.log1p(60.0)) * 180, 0, 180).astype(np.uint8)
    overlay = np.stack([grey, grey, grey], axis=-1)
    # < min_depth_m → red; min-1m → yellow; 1-5m → cyan; rest grey
    overlay[depth_m < MIN_DEPTH_M]                           = [220,  40,  40]   # red:   too close
    overlay[(depth_m >= MIN_DEPTH_M) & (depth_m < 1.0)]      = [220, 220,   0]   # yellow: close valid
    overlay[(depth_m >= 1.0)         & (depth_m < 5.0)]       = [  0, 180, 220]   # cyan:   mid range
    ax2.imshow(overlay)
    ax2.set_title(f'Zones: red<{MIN_DEPTH_M}m  yellow<1m  cyan<5m  grey=rest', fontsize=9)
    ax2.axis('off')

    ax3 = fig.add_subplot(gs[row, 2:4])
    ax3.hist(np.clip(dv, 0, 30), bins=200, color='steelblue', edgecolor='none', alpha=0.85)
    ax3.axvline(MIN_DEPTH_M, color='red',  linewidth=1.8, label=f'min_depth_m={MIN_DEPTH_M}m')
    ax3.axvline(1.0,         color='gold', linewidth=1.4, label='1m')
    ax3.axvline(5.0,         color='cyan', linewidth=1.4, label='5m')
    ax3.set_xlabel('depth_m')
    ax3.set_ylabel('pixel count (log)')
    ax3.set_title('depth_m histogram  (clip@30m)', fontsize=10)
    ax3.legend(fontsize=8)
    ax3.set_yscale('log')
    ax3.set_xlim(0, 30)
    row += 1

    # Row 2 (if rgb): stored-value heatmap + stored histogram
    if rgb is not None:
        ax4 = fig.add_subplot(gs[row, 0:2])
        vmax = float(np.percentile(stored.flatten(), 99))
        im4 = ax4.imshow(stored, cmap='viridis', vmin=0, vmax=max(vmax, 0.1))
        plt.colorbar(im4, ax=ax4, label=f'stored  ({convention})')
        ax4.set_title(f'Stored values  ({convention})', fontsize=10)
        ax4.axis('off')

        ax5 = fig.add_subplot(gs[row, 2:4])
        flat = stored.flatten()
        clip_hi = min(float(stored.max()), 30.0) if convention == 'direct' else min(float(stored.max()), 5.0)
        ax5.hist(np.clip(flat, 0, clip_hi), bins=200, color='darkorange', edgecolor='none', alpha=0.85)
        ax5.set_xlabel(f'stored value  ({convention})')
        ax5.set_ylabel('pixel count (log)')
        ax5.set_title('Stored-value histogram', fontsize=10)
        ax5.set_yscale('log')
        ax5.set_xlim(0, clip_hi)

    close_at_capture = f'{close_pct}%' if close_pct != '?' else 'n/a'
    fig.suptitle(
        f'{depth_path.name}  |  pitch={pitch_deg}°  '
        f'close_pct@capture={close_at_capture}  '
        f'convention={convention}  |  '
        f'depth_m: mean={dv.mean():.2f}m  '
        f'<{MIN_DEPTH_M}m={100.*n_close/(H_d*W_d):.1f}%',
        fontsize=11, fontweight='bold')

    plt.savefig(str(out_path), dpi=150, bbox_inches='tight')
    print(f'\nSaved: {out_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
