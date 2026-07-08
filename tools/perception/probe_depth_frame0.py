#!/usr/bin/env python3
"""
Analyze frame-0 raw depth output to characterize prop/body pixels vs valid depth.

The depth model outputs calibrated inverse depth (1/m): high raw = close.
Frame 0 is captured on the ground with props spinning — ideal for isolating
prop signatures before the scene changes.

Usage:
    python3 tools/perception/probe_depth_frame0.py

Files read from /tmp/ (written automatically on first inference by ONNXDepthEngine):
    /tmp/dedalus_frame0_depth.npy   shape=(H_model, W_model) float32
    /tmp/dedalus_frame0_rgb.npy     shape=(H_cam, W_cam, 3)  uint8

Output:
    /tmp/dedalus_frame0_analysis.png   composite diagnostic image
    Printed to stdout: statistics, percentile table, spatial analysis
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LogNorm

DEPTH_NPY = '/tmp/dedalus_frame0_depth.npy'
RGB_NPY   = '/tmp/dedalus_frame0_rgb.npy'
OUT_PNG   = '/tmp/dedalus_frame0_analysis.png'
SCALE     = 1.0  # depth_m = SCALE / raw  (1.0 for metric model)


def banner(title):
    print(f'\n{"="*60}')
    print(f'  {title}')
    print(f'{"="*60}')


def load(path, label):
    if not os.path.exists(path):
        print(f'ERROR: {label} not found: {path}', file=sys.stderr)
        sys.exit(1)
    arr = np.load(path)
    print(f'{label}: shape={arr.shape}  dtype={arr.dtype}  '
          f'min={arr.min():.4f}  max={arr.max():.4f}  mean={arr.mean():.4f}')
    return arr


def main():
    print('=== Frame-0 Depth Diagnostic ===')
    raw = load(DEPTH_NPY, 'depth (raw inv-depth 1/m)')
    rgb = load(RGB_NPY,   'rgb')

    H_d, W_d = raw.shape
    H_r, W_r = rgb.shape[:2]

    # depth_m = 1/raw for metric model
    valid_mask = raw > 1e-4
    depth_m = np.where(valid_mask, SCALE / raw, np.nan)

    # ------------------------------------------------------------------ #
    #  Percentile table                                                    #
    # ------------------------------------------------------------------ #
    banner('Raw inverse-depth percentiles  (high = close)')
    flat = raw.flatten()
    print(f'  {"pct":>6}  {"raw (1/m)":>10}  {"depth_m":>10}')
    print(f'  {"-"*32}')
    for p in [0.1, 1, 5, 10, 25, 50, 75, 90, 95, 99, 99.5, 99.9, 100]:
        v = float(np.percentile(flat, p))
        dm = SCALE / v if v > 1e-4 else float('inf')
        print(f'  p{p:>5.1f}   {v:>10.4f}   {dm:>10.3f}')

    # ------------------------------------------------------------------ #
    #  Depth-bin distribution                                             #
    # ------------------------------------------------------------------ #
    banner('Depth-m distribution')
    dv = depth_m[np.isfinite(depth_m)]
    ntot = dv.size
    bins = [
        (0.0,   0.2,  'props/very-near body'),
        (0.2,   0.33, 'near body (landing gear?)'),
        (0.33,  0.5,  '<0.5m close'),
        (0.5,   1.0,  '0.5–1m'),
        (1.0,   2.0,  '1–2m'),
        (2.0,   5.0,  '2–5m'),
        (5.0,  20.0,  '5–20m (cruise obstacle range)'),
        (20.0, 999.0, '>20m'),
    ]
    print(f'  {"range":>18}  {"pixels":>7}  {"pct":>6}  label')
    print(f'  {"-"*55}')
    for lo, hi, label in bins:
        cnt = int(np.sum((dv >= lo) & (dv < hi)))
        print(f'  [{lo:5.2f} – {hi:6.1f} m]   {cnt:>7d}   {100.*cnt/ntot:>5.1f}%  {label}')

    # ------------------------------------------------------------------ #
    #  Spatial analysis of very-close pixels                              #
    # ------------------------------------------------------------------ #
    # Use p99 of raw as "very close" threshold to isolate prop candidates.
    p99_raw  = float(np.percentile(flat, 99.0))
    p995_raw = float(np.percentile(flat, 99.5))
    p999_raw = float(np.percentile(flat, 99.9))
    thresh_half_m = 2.0   # raw > 2.0 → depth_m < 0.5m

    banner(f'Spatial: raw > p99 = {p99_raw:.4f}  (depth_m < {SCALE/p99_raw:.3f} m)')
    ys99, xs99 = np.where(raw > p99_raw)
    n99 = len(ys99)
    if n99 > 0:
        cy = np.mean(ys99) / H_d
        cx = np.mean(xs99) / W_d
        sy = np.std(ys99) / H_d
        sx = np.std(xs99) / W_d
        print(f'  n={n99}  ({100.*n99/(H_d*W_d):.1f}% of pixels)')
        print(f'  centroid: ({cx:.3f}, {cy:.3f}) in [0..1]  (x=0=left  y=0=top)')
        print(f'  spread:   sx={sx:.3f}  sy={sy:.3f}')
        print(f'  x-range:  [{xs99.min()/W_d:.3f}, {xs99.max()/W_d:.3f}]')
        print(f'  y-range:  [{ys99.min()/H_d:.3f}, {ys99.max()/H_d:.3f}]')

        # Quadrant breakdown (top-left, top-right, bottom-left, bottom-right)
        hh, hw = H_d // 2, W_d // 2
        q = {
            'TL': int(np.sum((ys99 < hh) & (xs99 < hw))),
            'TR': int(np.sum((ys99 < hh) & (xs99 >= hw))),
            'BL': int(np.sum((ys99 >= hh) & (xs99 < hw))),
            'BR': int(np.sum((ys99 >= hh) & (xs99 >= hw))),
        }
        print(f'  quadrant: TL={q["TL"]}  TR={q["TR"]}  BL={q["BL"]}  BR={q["BR"]}')

        # Symmetry check: if props, expect left≈right and roughly top-biased
        left_pct = 100. * (q['TL'] + q['BL']) / max(n99, 1)
        top_pct  = 100. * (q['TL'] + q['TR']) / max(n99, 1)
        print(f'  left-half: {left_pct:.1f}%  top-half: {top_pct:.1f}%')
        if abs(left_pct - 50) < 15:
            print('  → LEFT≈RIGHT: consistent with symmetric prop layout')
        else:
            print('  → ASYMMETRIC: possibly one prop or non-prop region')

        # RGB color of prop-candidate pixels (nearest-neighbour mapping to RGB frame)
        ys_rgb = np.clip((ys99 * H_r / H_d).astype(int), 0, H_r - 1)
        xs_rgb = np.clip((xs99 * W_r / W_d).astype(int), 0, W_r - 1)
        prop_rgb = rgb[ys_rgb, xs_rgb]   # (N, 3) uint8
        print(f'\n  RGB of prop-candidate pixels:')
        print(f'  mean R={prop_rgb[:,0].mean():.1f}  G={prop_rgb[:,1].mean():.1f}  B={prop_rgb[:,2].mean():.1f}')
        print(f'  std  R={prop_rgb[:,0].std():.1f}   G={prop_rgb[:,1].std():.1f}   B={prop_rgb[:,2].std():.1f}')
    else:
        print('  No pixels above p99 threshold.')

    # Is raw value distribution bimodal? Look for a dip between prop cluster and rest.
    banner('Histogram shape: looking for bimodality')
    counts, edges = np.histogram(flat[flat > 0.01], bins=500)
    # Find local minimum between raw=1 and raw=raw.max() (prop region)
    lo_idx = int(np.searchsorted(edges, 1.0))
    hi_idx = int(np.searchsorted(edges, min(raw.max(), 12.0)))
    if hi_idx > lo_idx + 2:
        sub = counts[lo_idx:hi_idx]
        min_idx = lo_idx + int(np.argmin(sub))
        min_raw = float(edges[min_idx])
        min_dm  = SCALE / min_raw if min_raw > 1e-4 else float('inf')
        print(f'  Local minimum in raw=[1..{min(raw.max(),12):.1f}]: '
              f'raw≈{min_raw:.3f}  depth_m≈{min_dm:.3f}')
        print(f'  → If this is a clear valley, it could be a natural prop threshold.')
    print()

    # ------------------------------------------------------------------ #
    #  Figures                                                            #
    # ------------------------------------------------------------------ #
    fig = plt.figure(figsize=(22, 14))
    gs  = gridspec.GridSpec(3, 4, figure=fig, hspace=0.40, wspace=0.32)

    # 1) RGB
    ax1 = fig.add_subplot(gs[0, 0:2])
    ax1.imshow(rgb)
    ax1.set_title(f'Input RGB  ({W_r}×{H_r})', fontsize=10)
    ax1.axis('off')

    # 2) Raw depth heatmap (plasma, high=close=bright)
    ax2 = fig.add_subplot(gs[0, 2:4])
    vmax_raw = max(float(np.percentile(flat, 99)), 1.0)
    im2 = ax2.imshow(raw, cmap='plasma', vmin=0, vmax=vmax_raw)
    plt.colorbar(im2, ax=ax2, label='raw (1/m)  bright=CLOSE')
    ax2.set_title(f'Raw inverse-depth  ({W_d}×{H_d})', fontsize=10)
    ax2.axis('off')

    # 3) depth_m log-scale heatmap
    ax3 = fig.add_subplot(gs[1, 0:2])
    dm_disp = np.where(valid_mask, np.clip(depth_m, 0.05, 30.0), 30.0)
    im3 = ax3.imshow(dm_disp, cmap='inferno_r', norm=LogNorm(vmin=0.05, vmax=30))
    plt.colorbar(im3, ax=ax3, label='depth_m (log)')
    ax3.set_title('depth_m log scale  (dark=close, bright=far)', fontsize=10)
    ax3.axis('off')

    # 4) Zone overlay: props | close | medium | far
    ax4 = fig.add_subplot(gs[1, 2:4])
    grey = np.clip((1.0 - np.log1p(dm_disp) / np.log1p(30.0)) * 180, 0, 180).astype(np.uint8)
    overlay = np.stack([grey, grey, grey], axis=-1)
    # Zone masks
    zone_prop   = raw > p99_raw                           # top 1% raw: prop candidates
    zone_close  = (raw >= thresh_half_m) & ~zone_prop    # raw≥2 (depth_m<0.5m) excluding prop
    zone_medium = (raw >= 0.5) & (raw < thresh_half_m)   # 0.5–2m
    overlay[zone_prop]   = [255,  50,  50]   # red   = prop candidates
    overlay[zone_close]  = [220, 180,   0]   # amber = very-close but not top-1%
    overlay[zone_medium] = [  0, 170, 220]   # cyan  = medium range
    ax4.imshow(overlay)
    ax4.set_title('Zones: red=top1%  amber=<0.5m  cyan=0.5-2m  grey=rest', fontsize=9)
    ax4.axis('off')

    # 5) Raw histogram (log y scale)
    ax5 = fig.add_subplot(gs[2, 0:2])
    clip_max = min(float(raw.max()), 12.0)
    ax5.hist(np.clip(flat, 0, clip_max), bins=300,
             color='steelblue', edgecolor='none', alpha=0.85)
    for v, label, color in [
        (1.0,         'raw=1 (1m)',    'green'),
        (2.0,         'raw=2 (0.5m)', 'gold'),
        (3.0,         'raw=3 (0.33m)','orange'),
        (p99_raw,     f'p99={p99_raw:.2f}', 'red'),
        (p995_raw,    f'p99.5={p995_raw:.2f}','darkred'),
    ]:
        ax5.axvline(v, color=color, linewidth=1.5, label=label)
    ax5.set_xlabel('raw inverse-depth (1/m)')
    ax5.set_ylabel('pixel count (log)')
    ax5.set_title('Raw histogram  (clip@12)', fontsize=10)
    ax5.legend(fontsize=8, loc='upper right')
    ax5.set_yscale('log')
    ax5.set_xlim(0, clip_max)

    # 6) depth_m histogram
    ax6 = fig.add_subplot(gs[2, 2:4])
    ax6.hist(np.clip(dv, 0, 20), bins=300,
             color='darkorange', edgecolor='none', alpha=0.85)
    ax6.axvline(SCALE / p99_raw, color='red',  linewidth=1.5, label=f'p99 threshold ({SCALE/p99_raw:.2f}m)')
    ax6.axvline(0.5,             color='gold', linewidth=1.5, label='0.5m')
    ax6.axvline(1.0,             color='lime', linewidth=1.5, label='min_depth_m=1.0')
    ax6.set_xlabel('depth_m')
    ax6.set_ylabel('pixel count (log)')
    ax6.set_title('depth_m histogram  (clip@20m)', fontsize=10)
    ax6.legend(fontsize=8)
    ax6.set_yscale('log')
    ax6.set_xlim(0, 20)

    plt.suptitle(
        f'Frame-0 Prop Analysis  |  '
        f'raw: min={raw.min():.3f}  max={raw.max():.3f}  mean={raw.mean():.3f}  '
        f'p99={p99_raw:.3f} (depth_m={SCALE/p99_raw:.3f})',
        fontsize=12, fontweight='bold')

    plt.savefig(OUT_PNG, dpi=150, bbox_inches='tight')
    print(f'Saved: {OUT_PNG}')


if __name__ == '__main__':
    main()
