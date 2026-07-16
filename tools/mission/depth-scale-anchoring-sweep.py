#!/usr/bin/env python3
"""Depth scale-anchoring sweep: predict recall gain from a depth multiplier.

Applies a depth scale factor (α) to ONNX range-bucket evidence counts and
computes a bucket-overlap recall proxy.  Sweeps α over a configurable range
to find the optimal multiplier and predict the recall ceiling before a C++
implementation of AGL-based scale anchoring.

  AGL scale-anchoring formula (C++ candidate):
    α = AGL_m / (sin(|pitch_rad|) × ground_plane_median_range_onnx)
  Mission constants → α ≈ 7.6  (AGL=18m, pitch=52°, ONNX_ground≈3m)
  Obstacle-tuned α  → α ≈ 2.0–2.5  (empirical from sweep on profiler JSONL)

IMPORTANT CAVEATS
-----------------
1. Bucket-overlap recall is an UPPER BOUND on voxel recall.
   It measures whether ONNX and GT evidence occupy the same 1D range band, not
   the same 3D voxel.  Real voxel recall will be lower (azimuth/elevation must
   also match).  The RELATIVE improvement across α values is directionally correct.

2. ONNX scale compression is non-uniform:
   - Ground plane: model compresses 22.8m true → ~3m (α≈7.6 needed)
   - Mid-scene obstacles: GT 15-30m → ONNX 5-15m (α≈2.5 needed)
   A single global α cannot fix both simultaneously.

3. Evidence counts come from the JSONL range-histogram keys.
   No per-frame AGL/pitch is logged — the sweep covers the plausible range.

Usage:
  python3 tools/mission/depth-scale-anchoring-sweep.py out/.../profile/pipeline_*.jsonl
  python3 tools/mission/depth-scale-anchoring-sweep.py --valid-only out/.../profile/pipeline_*.jsonl
  python3 tools/mission/depth-scale-anchoring-sweep.py --alpha 2.5 out/.../profile/pipeline_*.jsonl
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path

# Range bins in metres.  Last bin capped at 200m (practical max for the mission).
BINS: list[tuple[float, float]] = [(0, 5), (5, 15), (15, 30), (30, 200)]
KEYS: list[str] = ["0_5m", "5_15m", "15_30m", "30m_plus"]
LABELS: list[str] = ["0–5 m", "5–15 m", "15–30 m", "30 m+"]

VALID_DEPTH_WH = (256, 144)

# Alpha sweep: 1.0 = no change, 7.6 = ground-anchored (mission constants)
DEFAULT_ALPHAS = [1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 7.6, 10.0]


# ── Data loading ───────────────────────────────────────────────────────────────

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


def is_valid(frame: dict) -> bool:
    s = frame.get("stages", {})
    w = s.get("frame_source.detail.depth_sidecar.width")
    h = s.get("frame_source.detail.depth_sidecar.height")
    return w is not None and h is not None and (int(w), int(h)) == VALID_DEPTH_WH


def get_bucket_counts(frame: dict, slot: str) -> list[float]:
    s = frame.get("stages", {})
    return [float(s.get(f"depth.{slot}.range_{k}", 0)) for k in KEYS]


# ── Scale simulation ───────────────────────────────────────────────────────────

def rescale_buckets(counts: list[float], alpha: float) -> list[float]:
    """Shift ONNX range-bucket counts by multiplying all depths by alpha.

    Assumes a uniform depth distribution within each source bucket and
    distributes evidence proportionally into the destination buckets.

    Example: counts=[100, 200, 10, 0], alpha=2.5
      → 0-5m evidence maps to 0-12.5m (split across 0-5m and 5-15m)
      → 5-15m evidence maps to 12.5-37.5m (split across 5-15m and 15-30m)
    """
    if alpha == 1.0:
        return list(counts)
    new: list[float] = [0.0] * len(BINS)
    for src_i, (lo, hi) in enumerate(BINS):
        n = counts[src_i]
        if n == 0.0:
            continue
        scaled_lo = lo * alpha
        scaled_hi = hi * alpha
        span = scaled_hi - scaled_lo
        for dst_i, (dst_lo, dst_hi) in enumerate(BINS):
            overlap = max(0.0, min(scaled_hi, dst_hi) - max(scaled_lo, dst_lo))
            if span > 0:
                new[dst_i] += n * overlap / span
    return new


def bucket_overlap_recall(scaled_onnx: list[float], gt: list[float]) -> float | None:
    """Upper-bound on voxel recall: fraction of GT count matched by ONNX in each band."""
    gt_total = sum(gt)
    if gt_total == 0:
        return None
    matched = sum(min(scaled_onnx[i], gt[i]) for i in range(len(BINS)))
    return matched / gt_total


def percentile(vals: list[float], p: float) -> float:
    if not vals:
        return 0.0
    sv = sorted(vals)
    idx = int(len(sv) * p / 100)
    return sv[min(idx, len(sv) - 1)]


# ── Alpha sweep ────────────────────────────────────────────────────────────────

def run_sweep(frames: list[dict], alphas: list[float]) -> dict[float, dict]:
    """Return per-alpha stats: recall list, mean ONNX distribution."""
    results: dict[float, dict] = {}
    for alpha in alphas:
        recalls: list[float] = []
        dist_sum = [0.0] * len(BINS)
        n_frames = 0

        for f in frames:
            onnx = get_bucket_counts(f, "slot_a")
            gt   = get_bucket_counts(f, "slot_b")
            if sum(gt) == 0:
                continue

            scaled = rescale_buckets(onnx, alpha)
            r = bucket_overlap_recall(scaled, gt)
            if r is not None:
                recalls.append(r)
                for i in range(len(BINS)):
                    dist_sum[i] += scaled[i]
                n_frames += 1

        if n_frames == 0:
            continue
        dist_mean = [v / n_frames for v in dist_sum]
        dist_total = sum(dist_mean) or 1.0
        results[alpha] = {
            "recalls": recalls,
            "mean_recall": statistics.mean(recalls) if recalls else 0.0,
            "median_recall": percentile(recalls, 50),
            "p95_recall": percentile(recalls, 95),
            "dist_mean": dist_mean,
            "dist_pct": [v / dist_total * 100 for v in dist_mean],
            "n_frames": n_frames,
        }
    return results


def dominant_gt_bucket(frame: dict) -> int | None:
    gt = get_bucket_counts(frame, "slot_b")
    total = sum(gt)
    if total == 0:
        return None
    return int(max(range(len(BINS)), key=lambda i: gt[i]))


def cross_compression(frames: list[dict], alpha: float) -> None:
    """Print compression table for the given alpha."""
    by_dom: dict[int, list[list[float]]] = {i: [] for i in range(len(BINS))}
    for f in frames:
        dom = dominant_gt_bucket(f)
        if dom is None:
            continue
        onnx = get_bucket_counts(f, "slot_a")
        scaled = rescale_buckets(onnx, alpha)
        by_dom[dom].append(scaled)

    print(f"\n  Cross-compression table at α={alpha:.1f}")
    print(f"  (rows = dominant GT bucket; columns = ONNX evidence distribution)")
    print(f"  A healthy model would show high % on the diagonal.")
    print(f"  {'─'*72}")
    print(f"  {'GT dominant':<16}  {'frames':>7}  "
          f"{'ONNX 0-5m':>9}  {'ONNX 5-15m':>10}  "
          f"{'ONNX 15-30m':>11}  {'ONNX 30m+':>9}")
    print(f"  {'─'*72}")

    for dom_i, label in enumerate(LABELS):
        rows = by_dom[dom_i]
        if not rows:
            continue
        n = len(rows)
        means = [statistics.mean(r[i] for r in rows) for i in range(len(BINS))]
        total = sum(means) or 1.0
        pcts = [v / total * 100 for v in means]
        marker = "  ← key row" if dom_i == 2 else ""
        print(f"  {label:<16}  {n:>7}  "
              f"{pcts[0]:>8.1f}%  "
              f"{pcts[1]:>9.1f}%  "
              f"{pcts[2]:>10.1f}%  "
              f"{pcts[3]:>8.1f}%{marker}")


# ── Main ───────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("jsonl", nargs="+", type=Path)
    ap.add_argument("--valid-only", action="store_true",
                    help="Exclude frames whose depth sidecar is not 256×144")
    ap.add_argument("--alpha", type=float, nargs="+",
                    help="Override alpha sweep values (default: 1.0 1.5 2.0 2.5 3.0 4.0 5.0 6.0 7.6 10.0)")
    ap.add_argument("--detail", type=float, nargs="+", metavar="ALPHA",
                    help="Print cross-compression table for these alpha values "
                         "(default: best α, 2.5, and 7.6)")
    ap.add_argument("--no-detail", action="store_true",
                    help="Skip cross-compression tables")
    args = ap.parse_args()

    paths: list[Path] = []
    for p in args.jsonl:
        paths.extend(sorted(p.parent.glob(p.name)) if "*" in str(p) else [p])
    if not paths:
        sys.exit("ERROR: no files matched")

    all_frames = load_jsonl(paths)
    if not all_frames:
        sys.exit("ERROR: no frames loaded")

    frames = [f for f in all_frames if is_valid(f)] if args.valid_only else all_frames
    if not frames:
        sys.exit("ERROR: no valid frames after filtering")

    # Restrict to frames where GT evidence exists (A/B eval active)
    frames = [f for f in frames if sum(get_bucket_counts(f, "slot_b")) > 0]
    if not frames:
        sys.exit("ERROR: no frames with slot_b (GT) evidence — is A/B eval configured?")

    alphas = args.alpha if args.alpha else DEFAULT_ALPHAS

    print(f"\n{'='*72}")
    print(f"  VD4b: AGL Scale-Anchoring Simulation")
    print(f"  frames with GT evidence: {len(frames)}  (of {len(all_frames)} total)")
    print(f"  {'─'*68}")
    print(f"  NOTE: 'recall' here is a RANGE-BAND OVERLAP PROXY — an upper bound.")
    print(f"  Real voxel recall will be lower (azimuth/elevation must also match).")
    print(f"  Look at relative improvement, not absolute values.")
    print(f"{'='*72}")

    # ── Baseline (α=1.0 from JSONL actual recall) ──────────────────────────────
    baseline_recalls_ppt = [
        float(f["stages"]["depth.voxel_recall_ppt"]) / 10.0
        for f in frames
        if "depth.voxel_recall_ppt" in f.get("stages", {})
    ]
    if baseline_recalls_ppt:
        b_mean = statistics.mean(baseline_recalls_ppt)
        b_med  = percentile(baseline_recalls_ppt, 50)
        b_p95  = percentile(baseline_recalls_ppt, 95)
        print(f"\n  Actual voxel recall (from JSONL, no simulation):")
        print(f"  {'─'*68}")
        print(f"  mean={b_mean:.1f}%  median={b_med:.1f}%  p95={b_p95:.1f}%")
        print(f"  (This is ground truth recall with 0.5m voxels — use as sanity anchor.)")

    # ── Alpha sweep ────────────────────────────────────────────────────────────
    results = run_sweep(frames, alphas)

    print(f"\n  Alpha sweep — bucket-overlap recall proxy")
    print(f"  {'─'*68}")
    print(f"  {'α':>6}  {'mean%':>7}  {'median%':>8}  {'p95%':>6}  "
          f"  ONNX distribution after scaling")
    print(f"  {'─'*68}")

    best_alpha = 1.0
    best_mean  = 0.0
    for alpha in alphas:
        if alpha not in results:
            continue
        r = results[alpha]
        m  = r["mean_recall"]   * 100
        md = r["median_recall"] * 100
        p  = r["p95_recall"]    * 100
        dp = r["dist_pct"]

        # Mark the ground-anchored alpha
        note = "  ← mission AGL/pitch" if abs(alpha - 7.6) < 0.05 else ""
        if m > best_mean:
            best_mean  = m
            best_alpha = alpha

        dist_str = (f"0-5m={dp[0]:4.1f}%  5-15m={dp[1]:4.1f}%  "
                    f"15-30m={dp[2]:4.1f}%  30m+={dp[3]:4.1f}%")
        print(f"  {alpha:>6.1f}  {m:>7.1f}%  {md:>8.1f}%  {p:>6.1f}%    {dist_str}{note}")

    print(f"  {'─'*68}")
    print(f"\n  Best simulated α: {best_alpha:.1f}  "
          f"(mean bucket-overlap recall {best_mean:.1f}%)")

    # ── Scale ratio analysis ───────────────────────────────────────────────────
    scale_raw = [
        float(f["stages"]["depth.scale_ratio"]) / 1000.0
        for f in frames
        if "depth.scale_ratio" in f.get("stages", {})
    ]
    if scale_raw:
        print(f"\n  JSONL depth.scale_ratio (GT|center_local| / ONNX|center_local|)")
        print(f"  {'─'*68}")
        print(f"  ⚠  WARNING: scale_ratio uses world-origin absolute distance, NOT drone-")
        print(f"     relative range.  30m orbit radius dilutes the signal.")
        print(f"     Reads ~1.5 even when true scale error is ~7×.")
        sr_med = percentile(scale_raw, 50)
        sr_p25 = percentile(scale_raw, 25)
        sr_p75 = percentile(scale_raw, 75)
        print(f"  p25={sr_p25:.2f}  median={sr_med:.2f}  p75={sr_p75:.2f}")
        if sr_med < 2.0:
            print(f"  → Metric is too diluted by orbit radius to use as per-frame α.")
            print(f"     AGL + pitch from IMU is the correct per-frame signal (VD4b C++ plan).")

    # ── Non-uniform scale compression note ────────────────────────────────────
    print(f"\n  Non-uniform scale compression analysis")
    print(f"  {'─'*68}")
    print(f"  From VD4a cross-compression table (GT dominant 15-30m frames):")
    print(f"    ONNX 0-5m:  ~46%  (avg ~2.5m)  → needs α≈8-9 to reach 15-30m GT")
    print(f"    ONNX 5-15m: ~52%  (avg ~8m)    → needs α≈2.5 to reach 15-30m GT")
    print(f"  A single global α cannot optimally correct both classes simultaneously.")
    print(f"  Mission AGL-anchored α=7.6:")
    print(f"    ✓ Shifts 0-5m evidence (2.5m → 19m) into 15-30m GT band")
    print(f"    ✗ Overshoots 5-15m evidence (8m → 61m) into 30m+ (misses GT)")
    print(f"  α=2.5 (obstacle-tuned):")
    print(f"    ✓ Shifts 5-15m evidence (8m → 20m) into 15-30m GT band")
    print(f"    → 0-5m evidence (2.5m → 6.25m) stays in 5-15m (does not reach GT)")
    print(f"  → VD4b recall gain limited by whichever class dominates the frame.")

    # ── Cross-compression detail ───────────────────────────────────────────────
    if args.no_detail:
        detail_alphas = []
    elif args.detail:
        detail_alphas = args.detail
    else:
        # Default: best α + two reference points
        detail_alphas = sorted({best_alpha, 2.5, 7.6})

    for alpha in detail_alphas:
        if alpha in results:
            cross_compression(frames, alpha)
        else:
            # Run on-demand even if not in the sweep
            cross_compression(frames, alpha)

    # ── Recommendation ─────────────────────────────────────────────────────────
    print(f"\n  {'='*68}")
    print(f"  Recommendation")
    print(f"  {'─'*68}")

    gain = best_mean - (statistics.mean(baseline_recalls_ppt) if baseline_recalls_ppt else 0.0)
    print(f"  Best simulated recall (proxy): {best_mean:.1f}%  at α={best_alpha:.1f}")
    if baseline_recalls_ppt:
        print(f"  Actual baseline recall:        {statistics.mean(baseline_recalls_ppt):.1f}%")
        print(f"  Upper-bound gain estimate:     +{gain:.1f} pp")

    if best_mean < 25.0:
        print(f"\n  ✗ VD4b alone unlikely to reach >25% recall.")
        print(f"    Root cause: ONNX model has non-uniform, view-angle-dependent scale")
        print(f"    compression that a single post-processing multiplier cannot fix.")
        print(f"    Recommend skipping VD4b C++ implementation and proceeding to VD4c")
        print(f"    (UniDepth V2 ViT-S with camera intrinsics).")
    elif best_mean < 40.0:
        print(f"\n  ⚠  VD4b may provide modest improvement but is unlikely to be sufficient alone.")
        print(f"    Implement VD4b C++ as a cheap quick-win, then proceed to VD4c.")
        print(f"    Use α={best_alpha:.1f} as starting point; measure on real mission JSONL.")
    else:
        print(f"\n  ✓ VD4b shows meaningful simulated improvement.")
        print(f"    Implement in C++ with α calibrated per-frame using AGL + IMU pitch.")
        print(f"    Use α={best_alpha:.1f} as starting point; measure on real mission JSONL.")

    print(f"\n  Next: VD4c — UniDepth V2 ViT-S with known camera intrinsics (fx=141.6")
    print(f"  fy=141.2 cx=128 cy=72). No training required. ONNX export supported.")
    print(f"  {'='*68}\n")


if __name__ == "__main__":
    main()
