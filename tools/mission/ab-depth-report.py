#!/usr/bin/env python3
"""A/B depth provider comparison report.

Reads a pipeline profiler JSONL (one frame per line, stages in a nested object)
and prints a GT-centric summary of the two-slot depth eval metrics.

Slot B (eval) is always treated as ground truth (e.g. airsim_gt_vd).
Slot A (primary) is evaluated against it (e.g. visual_onnx).

  depth_slot_a.evidence_count   — primary obstacle evidence per frame
  depth_slot_b.evidence_count   — GT (eval) obstacle evidence per frame
  depth.voxel_precision_ppt     — fraction of slot-A confirmed by GT within ±0.5 m
  depth.voxel_recall_ppt        — fraction of GT confirmed by slot-A (SAFETY METRIC)
  depth.voxel_f1_ppt            — harmonic mean of precision and recall
  depth.false_positive_count    — slot-A voxels with no GT neighbor (phantom obstacles)
  depth.false_negative_count    — GT voxels missed by slot-A (missed obstacles)
  depth.median_range_a_m        — primary median obstacle range × 1000 (÷1000 → m)
  depth.median_range_b_m        — GT median obstacle range × 1000 (÷1000 → m)
  depth.scale_ratio             — (GT_range / primary_range) × 1000

Usage:
  python3 tools/mission/ab-depth-report.py out/circle_visual_eval/profile/pipeline_*.jsonl
  python3 tools/mission/ab-depth-report.py --tsv out/.../pipeline_*.jsonl > ab.tsv
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path


def stats_line(label: str, values: list[float], unit: str = "", scale: float = 1.0) -> str:
    if not values:
        return f"  {label:<42} NO DATA"
    vs = [v * scale for v in values]
    sv = sorted(vs)
    n = len(sv)
    mean = statistics.mean(vs)
    med = sv[n // 2]
    p25 = sv[n // 4]
    p75 = sv[min(n - 1, 3 * n // 4)]
    p95 = sv[min(n - 1, int(n * 0.95))]
    return (f"  {label:<42} n={n:<5d}  "
            f"mean={mean:7.2f}{unit}  p25={p25:6.2f}  p50={med:6.2f}  "
            f"p75={p75:6.2f}  p95={p95:6.2f}{unit}")


def load_jsonl(paths: list[Path]) -> list[dict]:
    frames = []
    errors = 0
    for path in paths:
        with open(path) as f:
            for lineno, line in enumerate(f, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    frames.append(json.loads(line))
                except json.JSONDecodeError as e:
                    errors += 1
                    if errors <= 3:
                        print(f"  WARN: {path}:{lineno}: {e}", file=sys.stderr)
    if errors:
        print(f"  WARN: {errors} malformed line(s) skipped", file=sys.stderr)
    return frames


def extract(frames: list[dict], key: str) -> list[float]:
    out = []
    for f in frames:
        stages = f.get("stages", {})
        v = stages.get(key)
        if v is not None:
            out.append(float(v))
    return out


def extract_pair(frames: list[dict], key_a: str, key_b: str) -> list[tuple[float, float]]:
    """Extract pairs of values that are both present in the same frame."""
    out = []
    for f in frames:
        s = f.get("stages", {})
        va = s.get(key_a)
        vb = s.get(key_b)
        if va is not None and vb is not None:
            out.append((float(va), float(vb)))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("jsonl", nargs="+", type=Path, help="pipeline profiler JSONL file(s)")
    ap.add_argument("--tsv", action="store_true",
                    help="emit frame-level TSV instead of summary (for plotting)")
    ap.add_argument("--min-frames", type=int, default=10,
                    help="warn if fewer frames than this have slot-B data (default: 10)")
    args = ap.parse_args()

    paths = []
    for p in args.jsonl:
        paths.extend(sorted(p.parent.glob(p.name)) if "*" in str(p) else [p])
    if not paths:
        sys.exit("ERROR: no JSONL files matched")

    print(f"Loading {len(paths)} file(s)...", file=sys.stderr)
    frames = load_jsonl(paths)
    if not frames:
        sys.exit("ERROR: no frames loaded")

    # ── TSV mode ─────────────────────────────────────────────────────────────
    if args.tsv:
        cols = [
            "frame_idx", "timestamp_ns",
            "ev_a", "ev_b",
            "range_a_m", "range_b_m", "scale_ratio",
            "precision_pct", "recall_pct", "f1_pct",
            "fp_count", "fn_count",
        ]
        print("\t".join(cols))
        for i, f in enumerate(frames):
            s = f.get("stages", {})
            ev_a      = s.get("depth_slot_a.evidence_count", "")
            ev_b      = s.get("depth_slot_b.evidence_count", "")
            range_a   = (s["depth.median_range_a_m"] / 1000) if "depth.median_range_a_m" in s else ""
            range_b   = (s["depth.median_range_b_m"] / 1000) if "depth.median_range_b_m" in s else ""
            ratio     = (s["depth.scale_ratio"] / 1000)      if "depth.scale_ratio"       in s else ""
            # Prefer new keys; fall back to legacy depth.voxel_overlap_ppt for old JSONLs.
            prec      = (s.get("depth.voxel_precision_ppt", s.get("depth.voxel_overlap_ppt", None)))
            prec_pct  = (prec / 10.0) if prec is not None else ""
            rec_raw   = s.get("depth.voxel_recall_ppt")
            rec_pct   = (rec_raw / 10.0) if rec_raw is not None else ""
            f1_raw    = s.get("depth.voxel_f1_ppt")
            f1_pct    = (f1_raw / 10.0)  if f1_raw  is not None else ""
            fp        = s.get("depth.false_positive_count", "")
            fn        = s.get("depth.false_negative_count", "")
            ts        = f.get("timestamp_ns", "")
            print(f"{i}\t{ts}\t{ev_a}\t{ev_b}\t{range_a}\t{range_b}\t{ratio}"
                  f"\t{prec_pct}\t{rec_pct}\t{f1_pct}\t{fp}\t{fn}")
        return

    # ── Summary mode ──────────────────────────────────────────────────────────
    total = len(frames)
    all_stages: set[str] = set()
    for f in frames:
        all_stages.update(f.get("stages", {}).keys())

    depth_stages = sorted(s for s in all_stages if "depth" in s)

    print(f"\n{'='*72}")
    print(f"  A/B Depth Eval Report  (slot B = ground truth)")
    print(f"  frames: {total}   files: {len(paths)}")
    print(f"{'='*72}")

    # ── Stage presence check ──────────────────────────────────────────────────
    new_keys = {
        "depth.voxel_precision_ppt", "depth.voxel_recall_ppt",
        "depth.voxel_f1_ppt", "depth.false_positive_count", "depth.false_negative_count",
    }
    legacy_key = "depth.voxel_overlap_ppt"
    has_new    = bool(new_keys & all_stages)
    has_legacy = legacy_key in all_stages
    has_slot_b = "depth_slot_b.evidence_count" in all_stages

    if not has_slot_b:
        print(f"\n  ⚠️  Slot B has NO data. Likely causes:")
        print(f"     1. Binary is stale — rebuild: cmake --build build-staging -j$(nproc)")
        print(f"     2. Config missing  depth_eval: airsim_gt_vd")
        print(f"     3. Env var         DEDALUS_DEPTH_EVAL=airsim_gt_vd  not set")
        print()

    if has_legacy and not has_new:
        print(f"\n  ℹ  Legacy JSONL (depth.voxel_overlap_ppt only = precision).")
        print(f"     Rebuild and re-run for recall, F1, FP/FN metrics.")
        print()

    # ── Evidence counts ───────────────────────────────────────────────────────
    ev_a = extract(frames, "depth_slot_a.evidence_count")
    ev_b = extract(frames, "depth_slot_b.evidence_count")
    ev_b_nonzero = [v for v in ev_b if v > 0]

    print(f"\n  Evidence counts (obstacles per frame)")
    print(f"  {'─'*68}")
    print(stats_line("slot A  (primary, e.g. visual_onnx)", ev_a, " obs"))
    print(stats_line("slot B  (GT eval, e.g. airsim_gt_vd)", ev_b, " obs"))
    if ev_b:
        nonzero_pct = 100.0 * len(ev_b_nonzero) / len(ev_b)
        print(f"  {'slot B non-zero frames':<42} {len(ev_b_nonzero)}/{len(ev_b)}  ({nonzero_pct:.1f}%)")
        if len(ev_b_nonzero) < args.min_frames:
            print(f"\n  ⚠️  Slot B produced evidence in fewer than {args.min_frames} frames.")
            print(f"     Check that --include-depth is in the bridge command.")

    # ── Precision / Recall / F1 ───────────────────────────────────────────────
    # Accept new keys; fall back to legacy for old JSONLs (precision only).
    raw_prec = extract(frames, "depth.voxel_precision_ppt") or extract(frames, "depth.voxel_overlap_ppt")
    raw_rec  = extract(frames, "depth.voxel_recall_ppt")
    raw_f1   = extract(frames, "depth.voxel_f1_ppt")

    prec_pct = [v / 10.0 for v in raw_prec]
    rec_pct  = [v / 10.0 for v in raw_rec]
    f1_pct   = [v / 10.0 for v in raw_f1]

    print(f"\n  GT-centric detection quality  (slot B = ground truth)")
    print(f"  {'─'*68}")
    print(stats_line("Precision  (A confirmed by GT)", prec_pct, "%"))
    print(stats_line("Recall     (GT confirmed by A) ← SAFETY", rec_pct, "%"))
    print(stats_line("F1         (harmonic mean)", f1_pct, "%"))

    if prec_pct and rec_pct:
        mean_p = statistics.mean(prec_pct)
        mean_r = statistics.mean(rec_pct)
        print()
        if mean_r < 70.0:
            print(f"  ⚠️  LOW RECALL ({mean_r:.1f}%): ONNX misses ~{100-mean_r:.0f}% of GT obstacles on average.")
            print(f"     Safety risk: real obstacles not detected. Check scale calibration and model quality.")
        elif mean_r < 85.0:
            print(f"  △  Recall {mean_r:.1f}%: acceptable but improve before disabling GT eval.")
        else:
            print(f"  ✓  Recall {mean_r:.1f}%: ONNX detects most GT obstacles.")

        if mean_p < 50.0:
            print(f"  ℹ  Low precision ({mean_p:.1f}%): ONNX reports many false-alarm obstacles.")
            print(f"     Not a safety issue but may cause unnecessary avoidance maneuvers.")

    # ── FP / FN counts ────────────────────────────────────────────────────────
    fp_counts = extract(frames, "depth.false_positive_count")
    fn_counts = extract(frames, "depth.false_negative_count")

    if fp_counts or fn_counts:
        print(f"\n  Per-frame error counts")
        print(f"  {'─'*68}")
        print(stats_line("False positives / frame (phantom obs)", fp_counts, " vox"))
        print(stats_line("False negatives / frame (missed obs)  ← SAFETY", fn_counts, " vox"))

    # ── Range comparison ──────────────────────────────────────────────────────
    range_a = [v / 1000.0 for v in extract(frames, "depth.median_range_a_m") if v > 0]
    range_b = [v / 1000.0 for v in extract(frames, "depth.median_range_b_m") if v > 0]

    print(f"\n  Median obstacle range per frame (metres)")
    print(f"  {'─'*68}")
    print(stats_line("slot A  (primary)", range_a, " m"))
    print(stats_line("slot B  (GT)",      range_b, " m"))

    # ── Scale ratio ───────────────────────────────────────────────────────────
    ratios = [v / 1000.0 for v in extract(frames, "depth.scale_ratio") if v > 0]

    print(f"\n  Scale ratio  (GT_range / primary_range)  — 1.0 = perfect calibration")
    print(f"  {'─'*68}")
    if ratios:
        sv = sorted(ratios)
        n = len(sv)
        mean = statistics.mean(ratios)
        med  = sv[n // 2]
        p25  = sv[n // 4]
        p75  = sv[min(n - 1, 3 * n // 4)]
        p95  = sv[min(n - 1, int(n * 0.95))]
        print(f"  {'ratio':<42} n={n:<5d}  mean={mean:6.3f}x  "
              f"p25={p25:.3f}x  p50={med:.3f}x  p75={p75:.3f}x  p95={p95:.3f}x")
        print()
        if med > 1.15:
            suggested = round(med, 2)
            print(f"  → Primary under-estimates range by ~{(1-1/med)*100:.0f}% relative to GT.")
            print(f"    If using visual_onnx:  visual_onnx.scale: {suggested}  in config/pipeline/visual.yaml")
            print(f"    If using unidepth_v2:  metric provider — scale calibration not applicable.")
        elif med < 0.87:
            print(f"  → Primary over-estimates range by ~{(1/med - 1)*100:.0f}% relative to GT.")
            print(f"    If using visual_onnx:  visual_onnx.scale: {round(med, 2)}  in config/pipeline/visual.yaml")
            print(f"    If using unidepth_v2:  metric provider — scale calibration not applicable.")
        else:
            print(f"  ✓  Scale within ±15% of GT — well calibrated.")

    # ── Angular (range-proportional) agreement ───────────────────────────────
    # Threshold grows with range: max(0.5m, range × tan(~1.1°)).
    # If angular >> fixed: providers agree on SCENE REGIONS but not exact depth.
    # If angular ≈ fixed: providers look at spatially distinct scene regions.
    raw_ang_prec = extract(frames, "depth.angular_precision_ppt")
    raw_ang_rec  = extract(frames, "depth.angular_recall_ppt")
    raw_ang_f1   = extract(frames, "depth.angular_f1_ppt")
    if raw_ang_prec or raw_ang_rec:
        ang_prec_pct = [v / 10.0 for v in raw_ang_prec]
        ang_rec_pct  = [v / 10.0 for v in raw_ang_rec]
        ang_f1_pct   = [v / 10.0 for v in raw_ang_f1]
        print(f"\n  Angular (range-proportional) agreement  [threshold = max(0.5m, range×0.020)]")
        print(f"  {'─'*68}")
        print(stats_line("Angular precision", ang_prec_pct, "%"))
        print(stats_line("Angular recall    ← compare to fixed recall", ang_rec_pct, "%"))
        print(stats_line("Angular F1", ang_f1_pct, "%"))
        if ang_rec_pct and rec_pct:
            lift = statistics.mean(ang_rec_pct) - statistics.mean(rec_pct)
            print()
            if lift > 15.0:
                print(f"  → Angular recall is {lift:.1f}pp higher than fixed recall.")
                print(f"    Providers agree on scene regions but evidence is spatially offset")
                print(f"    (range-proportional displacement — likely intrinsic mismatch).")
            elif lift > 5.0:
                print(f"  → Modest angular lift ({lift:.1f}pp). Mix of offset and genuine misses.")
            else:
                print(f"  → Angular and fixed recall are similar (lift {lift:.1f}pp).")
                print(f"    Providers disagree on scene regions, not just depth.")

    # ── Nearest-neighbour distance ────────────────────────────────────────────
    nn_dists_m = [v / 1000.0 for v in extract(frames, "depth.nn_median_dist_mm") if v > 0]
    if nn_dists_m:
        sv = sorted(nn_dists_m)
        n  = len(sv)
        print(f"\n  Nearest-neighbour distance A→B  (median distance per frame, no threshold)")
        print(f"  {'─'*68}")
        print(stats_line("NN median dist (A→B)", nn_dists_m, " m"))
        med_nn = sv[n // 2]
        print()
        if med_nn < 0.5:
            print(f"  ✓  Median NN dist {med_nn:.2f} m — providers are spatially close.")
            print(f"     If recall is still low, the ±0.5 m matching window is too tight at range.")
        elif med_nn < 2.0:
            print(f"  △  Median NN dist {med_nn:.2f} m — providers are offset by more than 1 voxel.")
            print(f"     Likely cause: intrinsic mismatch (fx/fy differ between slots).")
        else:
            print(f"  ✗  Median NN dist {med_nn:.2f} m — providers look at different scene regions.")
            print(f"     Check intrinsics (depth.slot_a.fx vs depth.slot_b.fx in JSONL).")

    # ── Intrinsics comparison ─────────────────────────────────────────────────
    fx_a_vals = [v / 1000.0 for v in extract(frames, "depth.slot_a.fx") if v > 0]
    fx_b_vals = [v / 1000.0 for v in extract(frames, "depth.slot_b.fx") if v > 0]
    fy_a_vals = [v / 1000.0 for v in extract(frames, "depth.slot_a.fy") if v > 0]
    fy_b_vals = [v / 1000.0 for v in extract(frames, "depth.slot_b.fy") if v > 0]
    if fx_a_vals and fx_b_vals:
        fx_a = statistics.median(fx_a_vals)
        fx_b = statistics.median(fx_b_vals)
        fy_a = statistics.median(fy_a_vals) if fy_a_vals else 0.0
        fy_b = statistics.median(fy_b_vals) if fy_b_vals else 0.0
        print(f"\n  Intrinsics per slot  (median across frames)")
        print(f"  {'─'*68}")
        print(f"  {'slot A fx':<30} {fx_a:8.2f} px")
        print(f"  {'slot B fx':<30} {fx_b:8.2f} px")
        print(f"  {'slot A fy':<30} {fy_a:8.2f} px")
        print(f"  {'slot B fy':<30} {fy_b:8.2f} px")
        fx_diff_pct = abs(fx_a - fx_b) / max(fx_a, fx_b, 1.0) * 100.0
        fy_diff_pct = abs(fy_a - fy_b) / max(fy_a, fy_b, 1.0) * 100.0
        print()
        if fx_diff_pct > 5.0 or fy_diff_pct > 5.0:
            print(f"  ⚠️  fx mismatch: {fx_diff_pct:.1f}%   fy mismatch: {fy_diff_pct:.1f}%")
            print(f"     At 30 m, {fx_diff_pct:.1f}% fx error → {30*fx_diff_pct/100*0.017:.2f} m lateral displacement.")
            print(f"     Fix: set visual_onnx.fx_cal to match AirSim camera FoV-derived fx.")
        elif fx_diff_pct > 1.0:
            print(f"  △  fx mismatch {fx_diff_pct:.1f}% — small but visible at 30+ m range.")
        else:
            print(f"  ✓  Intrinsics match (fx diff {fx_diff_pct:.1f}%, fy diff {fy_diff_pct:.1f}%).")
            print(f"     If recall is still low, the issue is depth value distribution (not direction).")
    else:
        print(f"\n  Intrinsics: no data (rebuild needed for depth.slot_a.fx / depth.slot_b.fx).")

    # ── Recall by range bucket ────────────────────────────────────────────────
    prec_range_pairs = extract_pair(frames, "depth.median_range_a_m", "depth.voxel_recall_ppt")
    if prec_range_pairs:
        buckets: dict[str, list[float]] = {"0–5 m": [], "5–15 m": [], "15–30 m": [], "30 m+": []}
        for range_mm, recall_ppt in prec_range_pairs:
            r_m = range_mm / 1000.0
            pct_val = recall_ppt / 10.0
            if r_m < 5.0:       buckets["0–5 m"].append(pct_val)
            elif r_m < 15.0:    buckets["5–15 m"].append(pct_val)
            elif r_m < 30.0:    buckets["15–30 m"].append(pct_val)
            else:               buckets["30 m+"].append(pct_val)

        if any(buckets.values()):
            print(f"\n  Recall by range bucket  (frames binned by median obstacle range)")
            print(f"  {'─'*68}")
            for label, vals in buckets.items():
                if vals:
                    med_r = sorted(vals)[len(vals) // 2]
                    print(f"  {label:<12}  n={len(vals):<5d}  median recall={med_r:.1f}%"
                          + ("  ← SAFETY" if med_r < 70 else ""))
                else:
                    print(f"  {label:<12}  (no frames)")

    # ── All depth stages present in file ─────────────────────────────────────
    print(f"\n  Depth-related stages in JSONL:")
    print(f"  {'─'*68}")
    if depth_stages:
        for s in depth_stages:
            vals = extract(frames, s)
            if vals:
                print(f"    {s:<50} n={len(vals)}")
    else:
        print("  (none — stale build or slot B inactive)")

    print(f"\n{'='*72}\n")


if __name__ == "__main__":
    main()
