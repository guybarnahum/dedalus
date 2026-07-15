#!/usr/bin/env python3
"""Depth evidence range-distribution comparison.

Reads pipeline profiler JSONL and shows per-slot range histograms side-by-side.
Requires a build with the range histogram keys (depth.slot_a.range_0_5m etc.).

This answers the core diagnostic question:
  "Does ONNX flood close-range buckets where GT shows far geometry?"

If ONNX has most evidence at <5 m while GT has most at >30 m, the monocular
model is assigning close depth to background/sky pixels — a model quality issue.
If both distributions match, the miss rate is from sub-voxel 3D position mismatch.

Usage:
  python3 tools/mission/depth-evidence-dist.py out/.../profile/pipeline_*.jsonl

Also prints:
  - actual depth sidecar frame dimensions (confirms AirSim resolution)
  - per-frame range histogram summary
  - instructions for visual inspection via the frame0 NPY dumps
"""
from __future__ import annotations

import argparse
import collections
import json
import statistics
import sys
from pathlib import Path


BUCKETS = ["0_5m", "5_15m", "15_30m", "30m_plus"]
LABELS  = ["0–5 m", "5–15 m", "15–30 m", "30 m+"]


def load_jsonl(paths: list[Path]) -> list[dict]:
    frames = []
    for path in paths:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    frames.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return frames


def extract(frames: list[dict], key: str) -> list[float]:
    return [float(f["stages"][key]) for f in frames if key in f.get("stages", {})]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("jsonl", nargs="+", type=Path)
    args = ap.parse_args()

    paths: list[Path] = []
    for p in args.jsonl:
        paths.extend(sorted(p.parent.glob(p.name)) if "*" in str(p) else [p])
    if not paths:
        sys.exit("ERROR: no files matched")

    frames = load_jsonl(paths)
    if not frames:
        sys.exit("ERROR: no frames loaded")

    print(f"\n{'='*72}")
    print(f"  Depth Evidence Range Distribution")
    print(f"  frames: {len(frames)}   files: {len(paths)}")
    print(f"{'='*72}")

    # ── Depth sidecar frame dimensions ─────────────────────────────────────────
    depth_sizes: collections.Counter = collections.Counter()
    for f in frames:
        s = f.get("stages", {})
        w = s.get("frame_source.detail.depth_sidecar.width")
        h = s.get("frame_source.detail.depth_sidecar.height")
        if w is not None and h is not None:
            depth_sizes[(int(w), int(h))] += 1

    print(f"\n  GT depth sidecar dimensions (from JSONL)")
    print(f"  {'─'*68}")
    if depth_sizes:
        for (w, h), count in depth_sizes.most_common():
            # Derive expected intrinsics for this resolution + 84°/53.72° FOV
            import math
            hfov_deg, vfov_deg = 84.0, 53.72
            cx = (w - 1) / 2.0
            cy = (h - 1) / 2.0
            fx = cx / math.tan(math.radians(hfov_deg / 2))
            fy = cy / math.tan(math.radians(vfov_deg / 2))
            print(f"  {w}×{h}  ({count} frames)  →  fx={fx:.1f}  fy={fy:.1f}  "
                  f"(at hfov={hfov_deg}°, vfov={vfov_deg}°)")
    else:
        print("  (no depth sidecar data — old JSONL or depth not enabled)")

    # ── Range histogram: slot A ─────────────────────────────────────────────────
    hist_a: dict[str, list[float]] = {b: extract(frames, f"depth.slot_a.range_{b}") for b in BUCKETS}
    hist_b: dict[str, list[float]] = {b: extract(frames, f"depth.slot_b.range_{b}") for b in BUCKETS}

    has_hist = any(hist_a[b] for b in BUCKETS)
    if not has_hist:
        print(f"\n  ⚠  No range histogram data.  Rebuild after adding depth.slot_a.range_* keys.")
        print(f"     cmake --build build-staging -j$(nproc)")
        print(f"     Then re-run a mission and collect new JSONL.")
    else:
        # Compute mean counts per frame for each bucket
        def mean_or(vals: list[float]) -> float:
            return statistics.mean(vals) if vals else 0.0

        total_a = sum(mean_or(hist_a[b]) for b in BUCKETS)
        total_b = sum(mean_or(hist_b[b]) for b in BUCKETS)

        print(f"\n  Range distribution  (mean evidence count per frame)")
        print(f"  {'─'*68}")
        print(f"  {'Bucket':<12}  {'Slot A (ONNX)':>16}  {'%':>6}    {'Slot B (GT)':>14}  {'%':>6}")
        print(f"  {'─'*68}")
        for bucket, label in zip(BUCKETS, LABELS):
            a = mean_or(hist_a[bucket])
            b = mean_or(hist_b[bucket])
            pct_a = (a / total_a * 100) if total_a > 0 else 0
            pct_b = (b / total_b * 100) if total_b > 0 else 0
            flag = ""
            if abs(pct_a - pct_b) > 20:
                flag = "  ← MISMATCH"
            print(f"  {label:<12}  {a:>10.1f} obs  {pct_a:>5.1f}%    {b:>10.1f} obs  {pct_b:>5.1f}%{flag}")
        print(f"  {'─'*68}")
        print(f"  {'Total':<12}  {total_a:>10.1f} obs  {'100%':>6}    {total_b:>10.1f} obs  {'100%':>6}")

        # Interpretation
        close_a = mean_or(hist_a["0_5m"]) + mean_or(hist_a["5_15m"])
        far_b   = mean_or(hist_b["15_30m"]) + mean_or(hist_b["30m_plus"])
        close_pct_a = (close_a / total_a * 100) if total_a > 0 else 0
        far_pct_b   = (far_b   / total_b * 100) if total_b > 0 else 0

        print()
        if close_pct_a > 40 and far_pct_b > 60:
            print(f"  ✗  ONNX puts {close_pct_a:.0f}% of evidence at <15 m while GT puts "
                  f"{far_pct_b:.0f}% at ≥15 m.")
            print(f"     Diagnosis: model assigns close depth to far/background geometry.")
            print(f"     The monocular model has not learned the absolute scale of this scene.")
        elif abs(close_pct_a - (100 - far_pct_b)) < 15:
            print(f"  △  Range distributions are broadly similar.")
            print(f"     Recall miss is from sub-voxel 3D position mismatch within grid cells,")
            print(f"     not from ONNX hallucinating close obstacles.")
        else:
            print(f"  ℹ  Distributions differ — inspect per-bucket numbers above for the pattern.")

    # ── Per-frame distribution stability ───────────────────────────────────────
    if has_hist and hist_a["30m_plus"]:
        far_a  = hist_a["30m_plus"]
        far_b  = hist_b["30m_plus"]
        print(f"\n  30 m+ bucket per frame (stability)")
        print(f"  {'─'*68}")
        sv_a = sorted(far_a)
        sv_b = sorted(far_b) if far_b else []
        n = len(sv_a)
        print(f"  Slot A  p25={sv_a[n//4]:.0f}  p50={sv_a[n//2]:.0f}  "
              f"p75={sv_a[min(n-1, 3*n//4)]:.0f}  max={sv_a[-1]:.0f}")
        if sv_b:
            nb = len(sv_b)
            print(f"  Slot B  p25={sv_b[nb//4]:.0f}  p50={sv_b[nb//2]:.0f}  "
                  f"p75={sv_b[min(nb-1, 3*nb//4)]:.0f}  max={sv_b[-1]:.0f}")

    # ── Visual inspection instructions ─────────────────────────────────────────
    print(f"\n  Visual inspection (frame0 depth dumps)")
    print(f"  {'─'*68}")
    print(f"  After a run, compare the two frame0 depth maps:")
    print()
    print(f"    python3 - <<'EOF'")
    print(f"import numpy as np, matplotlib.pyplot as plt")
    print(f"gt   = np.load('/tmp/dedalus_gt_frame0_depth.npy')   # metres, 0=invalid")
    print(f"onnx = np.load('/tmp/dedalus_frame0_depth.npy')      # raw metric depth metres")
    print(f"fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))")
    print(f"im1 = ax1.imshow(gt,   vmin=0, vmax=60, cmap='plasma'); ax1.set_title('GT depth (m)')")
    print(f"im2 = ax2.imshow(onnx, vmin=0, vmax=60, cmap='plasma'); ax2.set_title('ONNX depth (m)')")
    print(f"plt.colorbar(im1, ax=ax1); plt.colorbar(im2, ax=ax2)")
    print(f"print(f'GT   {{gt.shape}}  min={{gt[gt>0].min():.1f}}  max={{gt.max():.1f}} m')")
    print(f"print(f'ONNX {{onnx.shape}}  min={{onnx[onnx>0].min():.1f}}  max={{onnx.max():.1f}} m')")
    print(f"plt.tight_layout(); plt.savefig('/tmp/depth_comparison.png', dpi=120)")
    print(f"print('saved /tmp/depth_comparison.png')")
    print(f"EOF")

    print(f"\n{'='*72}\n")


if __name__ == "__main__":
    main()
