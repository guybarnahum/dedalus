#!/usr/bin/env python3
"""Depth evidence range-distribution comparison.

Reads pipeline profiler JSONL and shows per-slot range histograms side-by-side.
Requires a build with the range histogram keys (depth.slot_a.range_0_5m etc.).

Key diagnostic questions answered:
  1. Scale calibration — how far off is ONNX metric depth from GT? (depth.scale_ratio)
  2. Range compression — where does ONNX put evidence that GT puts at 15-30m?
  3. Safety metrics — per-frame recall/precision/F1/FP/FN counts

Findings from the 2026-07-16 run:
  - ONNX compresses GT's 15-30m evidence into 5-15m (~2x scale under-estimation)
  - ONNX adds spurious 0-5m evidence (block-minimum captures scattered noise pixels)
  - Root cause: training domain gap — model scale priors are for ground-level cameras,
    not aerial drone view at 18m altitude

Usage:
  python3 tools/mission/depth-evidence-dist.py out/.../profile/pipeline_*.jsonl
  python3 tools/mission/depth-evidence-dist.py --valid-only out/.../profile/pipeline_*.jsonl
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import statistics
import sys
from pathlib import Path


BUCKETS = ["0_5m", "5_15m", "15_30m", "30m_plus"]
LABELS  = ["0–5 m", "5–15 m", "15–30 m", "30 m+"]

# AirSim DepthPlanar delivers 256×144 for our camera config.
# Any other size in the JSONL is a stale frame from an earlier/different run.
VALID_DEPTH_WH = (256, 144)


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


def mean_or(vals: list[float]) -> float:
    return statistics.mean(vals) if vals else 0.0


def percentile(vals: list[float], p: float) -> float:
    if not vals:
        return 0.0
    sv = sorted(vals)
    idx = int(len(sv) * p / 100)
    return sv[min(idx, len(sv) - 1)]


def pct_str(val: float, total: float) -> str:
    if total <= 0:
        return "   — "
    return f"{val / total * 100:5.1f}%"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("jsonl", nargs="+", type=Path)
    ap.add_argument("--valid-only", action="store_true",
                    help="Exclude frames whose depth sidecar is not 256×144 "
                         "(filters stale frames from prior runs in the same output dir)")
    args = ap.parse_args()

    paths: list[Path] = []
    for p in args.jsonl:
        paths.extend(sorted(p.parent.glob(p.name)) if "*" in str(p) else [p])
    if not paths:
        sys.exit("ERROR: no files matched")

    all_frames = load_jsonl(paths)
    if not all_frames:
        sys.exit("ERROR: no frames loaded")

    # ── Classify by depth sidecar dimensions ───────────────────────────────────
    depth_sizes: collections.Counter = collections.Counter()
    valid_frames: list[dict] = []
    for f in all_frames:
        s = f.get("stages", {})
        w = s.get("frame_source.detail.depth_sidecar.width")
        h = s.get("frame_source.detail.depth_sidecar.height")
        if w is not None and h is not None:
            wh = (int(w), int(h))
            depth_sizes[wh] += 1
            if wh == VALID_DEPTH_WH:
                valid_frames.append(f)

    frames = valid_frames if args.valid_only else all_frames

    print(f"\n{'='*72}")
    print(f"  Depth Evidence Range Distribution")
    print(f"  total frames: {len(all_frames)}   files: {len(paths)}")
    if args.valid_only:
        print(f"  analysed (valid depth sidecar only): {len(frames)}")
    print(f"{'='*72}")

    # ── Depth sidecar frame dimensions ─────────────────────────────────────────
    print(f"\n  GT depth sidecar dimensions (from JSONL)")
    print(f"  {'─'*68}")
    for (w, h), count in depth_sizes.most_common():
        hfov_deg, vfov_deg = 84.0, 53.72
        cx = (w - 1) / 2.0
        cy = (h - 1) / 2.0
        fx = cx / math.tan(math.radians(hfov_deg / 2))
        fy = cy / math.tan(math.radians(vfov_deg / 2))
        tag = "  ← valid" if (w, h) == VALID_DEPTH_WH else \
              "  ← STALE (excluded with --valid-only)"
        print(f"  {w}×{h}  ({count:>5} frames)  →  fx={fx:.1f}  fy={fy:.1f}{tag}")

    if not frames:
        sys.exit("\nERROR: no frames to analyse.")

    # ── Scale calibration ───────────────────────────────────────────────────────
    # depth.scale_ratio stored as int(GT_median_range_m / ONNX_median_range_m * 1000).
    # ratio > 1.0 → GT sees farther → ONNX under-estimates depth (thinks objects closer).
    # ratio < 1.0 → ONNX over-estimates depth (thinks objects farther).
    scale_raw   = extract(frames, "depth.scale_ratio")       # stored × 1000
    range_a_raw = extract(frames, "depth.median_range_a_m")  # stored × 1000 (mm)
    range_b_raw = extract(frames, "depth.median_range_b_m")  # stored × 1000 (mm)

    print(f"\n  Scale calibration  (GT median range ÷ ONNX median range per frame)")
    print(f"  {'─'*68}")
    if not scale_raw:
        print(f"  (no depth.scale_ratio data — old JSONL or A/B eval not configured)")
    else:
        scale   = [v / 1000.0 for v in scale_raw]
        range_a = [v / 1000.0 for v in range_a_raw]  # metres
        range_b = [v / 1000.0 for v in range_b_raw]  # metres
        p25 = percentile(scale, 25)
        p50 = percentile(scale, 50)
        p75 = percentile(scale, 75)
        p95 = percentile(scale, 95)
        print(f"  n={len(scale)}  mean={statistics.mean(scale):.2f}  "
              f"p25={p25:.2f}  median={p50:.2f}  p75={p75:.2f}  p95={p95:.2f}")
        if range_a and range_b:
            print(f"  ONNX median range (slot A): "
                  f"mean={statistics.mean(range_a):.1f} m  median={percentile(range_a,50):.1f} m")
            print(f"  GT   median range (slot B): "
                  f"mean={statistics.mean(range_b):.1f} m  median={percentile(range_b,50):.1f} m")
        print()
        if p50 > 1.5:
            print(f"  ✗  ONNX under-estimates depth by {p50:.2f}× at median.")
            print(f"     GT sees the scene {p50:.2f}× farther than ONNX predicts.")
            print(f"     Training domain gap: model scale priors are for ground-level cameras,")
            print(f"     not aerial drone view at altitude.")
        elif p50 < 0.7:
            print(f"  ✗  ONNX over-estimates depth by {1.0/max(p50,0.01):.2f}× at median.")
        else:
            print(f"  ✓  Scale broadly correct (median ratio {p50:.2f}).")

    # ── Safety metrics ──────────────────────────────────────────────────────────
    # _ppt keys are per-mille (‰); divide by 10 to get percent.
    recall_ppt = extract(frames, "depth.voxel_recall_ppt")
    prec_ppt   = extract(frames, "depth.voxel_precision_ppt")
    f1_ppt     = extract(frames, "depth.voxel_f1_ppt")
    fp_count   = extract(frames, "depth.false_positive_count")
    fn_count   = extract(frames, "depth.false_negative_count")

    print(f"\n  Safety metrics  (voxel overlap, ONNX slot A vs GT slot B)")
    print(f"  {'─'*68}")
    if not recall_ppt:
        print(f"  (no voxel metrics — old JSONL or A/B eval not configured)")
    else:
        def ppt_stats(vals: list[float]) -> tuple[float, float, float, float]:
            pcts = [v / 10.0 for v in vals]  # ‰ → %
            return (statistics.mean(pcts),
                    percentile(pcts, 50),
                    percentile(pcts, 5),
                    percentile(pcts, 95))

        rm, r50, r05, r95 = ppt_stats(recall_ppt)
        pm, p50v, p05v, p95v = ppt_stats(prec_ppt)
        fm, f50, f05, f95 = ppt_stats(f1_ppt)

        print(f"  {'Metric':<28}  {'mean':>7}  {'median':>7}  {'p5':>7}  {'p95':>7}")
        print(f"  {'─'*68}")
        print(f"  {'Recall (SAFETY — missed obstacles)':<28}  {rm:>6.1f}%  {r50:>6.1f}%  "
              f"{r05:>6.1f}%  {r95:>6.1f}%")
        print(f"  {'Precision':<28}  {pm:>6.1f}%  {p50v:>6.1f}%  {p05v:>6.1f}%  {p95v:>6.1f}%")
        print(f"  {'F1':<28}  {fm:>6.1f}%  {f50:>6.1f}%  {f05:>6.1f}%  {f95:>6.1f}%")
        if fp_count:
            print(f"  {'False positives / frame':<28}  {mean_or(fp_count):>6.0f}   "
                  f"{percentile(fp_count,50):>6.0f}   {'':>6}  "
                  f"{percentile(fp_count,95):>6.0f}  (p95)")
        if fn_count:
            fn_mean = mean_or(fn_count)
            fn_p95  = percentile(fn_count, 95)
            print(f"  {'False negatives / frame':<28}  {fn_mean:>6.0f}   "
                  f"{percentile(fn_count,50):>6.0f}   {'':>6}  {fn_p95:>6.0f}  (p95)")
            print()
            print(f"  ⚠  {fn_mean:.0f} missed GT voxels per frame on average. "
                  f"These are obstacles ONNX doesn't see.")

    # ── Range histogram ─────────────────────────────────────────────────────────
    hist_a: dict[str, list[float]] = {b: extract(frames, f"depth.slot_a.range_{b}") for b in BUCKETS}
    hist_b: dict[str, list[float]] = {b: extract(frames, f"depth.slot_b.range_{b}") for b in BUCKETS}

    has_hist = any(hist_a[b] for b in BUCKETS)
    print(f"\n  Range distribution  (mean evidence count per frame)")
    print(f"  {'─'*68}")
    if not has_hist:
        print(f"  ⚠  No range histogram data.")
        print(f"     cmake --build build-staging -j$(nproc)  then re-run mission.")
    else:
        total_a = sum(mean_or(hist_a[b]) for b in BUCKETS)
        total_b = sum(mean_or(hist_b[b]) for b in BUCKETS)

        print(f"  {'Bucket':<12}  {'Slot A (ONNX)':>16}  {'%':>6}    "
              f"{'Slot B (GT)':>14}  {'%':>6}")
        print(f"  {'─'*68}")
        for bucket, label in zip(BUCKETS, LABELS):
            a = mean_or(hist_a[bucket])
            b = mean_or(hist_b[bucket])
            pa = a / max(total_a, 1) * 100
            pb = b / max(total_b, 1) * 100
            flag = "  ← MISMATCH" if abs(pa - pb) > 15 else ""
            print(f"  {label:<12}  {a:>10.1f} obs  {pa:>5.1f}%    "
                  f"{b:>10.1f} obs  {pb:>5.1f}%{flag}")
        print(f"  {'─'*68}")
        print(f"  {'Total':<12}  {total_a:>10.1f} obs  {'100%':>6}    "
              f"{total_b:>10.1f} obs  {'100%':>6}")

    # ── Cross-compression table ─────────────────────────────────────────────────
    # For each frame, identify the dominant GT bucket, then aggregate ONNX distribution.
    # Answers: "when GT puts most evidence at 15-30m, where does ONNX put it?"
    if has_hist:
        print(f"\n  Range compression table")
        print(f"  (rows = dominant GT bucket; columns = ONNX evidence distribution)")
        print(f"  A healthy model would show high % on the diagonal.")
        print(f"  {'─'*68}")
        print(f"  {'GT dominant':<16}  {'frames':>7}  "
              f"{'ONNX 0-5m':>9}  {'ONNX 5-15m':>10}  "
              f"{'ONNX 15-30m':>11}  {'ONNX 30m+':>9}")
        print(f"  {'─'*68}")

        n = min((len(hist_a[b]) for b in BUCKETS if hist_a[b]), default=0)
        if n > 0:
            for dom_b, dom_label in zip(BUCKETS, LABELS):
                sel = []
                for i in range(n):
                    b_vals = {b: hist_b[b][i] for b in BUCKETS}
                    total = sum(b_vals.values())
                    if total > 0 and dom_b == max(b_vals, key=b_vals.get):
                        sel.append(i)
                if not sel:
                    continue
                a_means = {b: statistics.mean(hist_a[b][i] for i in sel) for b in BUCKETS}
                a_tot   = sum(a_means.values())
                marker  = "  ← key row" if dom_b == "15_30m" else ""
                print(f"  {dom_label:<16}  {len(sel):>7}  "
                      f"{pct_str(a_means['0_5m'],   a_tot):>9}  "
                      f"{pct_str(a_means['5_15m'],  a_tot):>10}  "
                      f"{pct_str(a_means['15_30m'], a_tot):>11}  "
                      f"{pct_str(a_means['30m_plus'], a_tot):>9}{marker}")
        print()
        print(f"  If the '15-30m' row shows most ONNX evidence at 5-15m,")
        print(f"  ONNX is compressing medium-range geometry by ~2x.")

    # ── 30 m+ stability ─────────────────────────────────────────────────────────
    if has_hist and hist_a["30m_plus"]:
        far_a = hist_a["30m_plus"]
        far_b = hist_b["30m_plus"]
        print(f"\n  30 m+ bucket per frame (long-range stability)")
        print(f"  {'─'*68}")
        print(f"  Slot A  p25={percentile(far_a,25):.0f}  p50={percentile(far_a,50):.0f}  "
              f"p75={percentile(far_a,75):.0f}  p95={percentile(far_a,95):.0f}  "
              f"max={max(far_a):.0f}")
        if far_b:
            print(f"  Slot B  p25={percentile(far_b,25):.0f}  p50={percentile(far_b,50):.0f}  "
                  f"p75={percentile(far_b,75):.0f}  p95={percentile(far_b,95):.0f}  "
                  f"max={max(far_b):.0f}")

    # ── Visual inspection instructions ─────────────────────────────────────────
    print(f"\n  Frame0 depth map comparison")
    print(f"  {'─'*68}")
    print(f"  python3 tools/mission/depth-scale-analysis.py \\")
    print(f"      /tmp/dedalus_gt_frame0_depth.npy \\")
    print(f"      /tmp/dedalus_frame0_depth.npy")
    print(f"  open /tmp/depth_scale_analysis.png")
    print(f"\n  Note: frame0 is ground-level horizontal (pitch=0).")
    print(f"  In-flight analysis uses the JSONL histograms above.")

    print(f"\n{'='*72}\n")


if __name__ == "__main__":
    main()
