#!/usr/bin/env python3
"""A/B depth provider comparison report.

Reads a pipeline profiler JSONL (one frame per line, stages in a nested object)
and prints a summary of the two-slot depth eval metrics:

  depth_slot_a.evidence_count   — ONNX obstacle evidence per frame
  depth_slot_b.evidence_count   — GT (AirSim) obstacle evidence per frame
  depth.voxel_overlap_ppt       — voxel agreement ‰ between A and B
  depth.median_range_a_m        — ONNX median obstacle range × 1000 (÷1000 → m)
  depth.median_range_b_m        — GT   median obstacle range × 1000 (÷1000 → m)
  depth.scale_ratio             — (GT_range / ONNX_range) × 1000

Usage:
  python3 tools/mission/ab-depth-report.py out/seq_b_visual/profile/pipeline_*.jsonl
  python3 tools/mission/ab-depth-report.py --tsv out/.../pipeline_*.jsonl > scale.tsv
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path


def pct(p: float, data: list[float]) -> float:
    if not data:
        return 0.0
    idx = max(0, min(len(data) - 1, int(len(data) * p)))
    return sorted(data)[idx]


def stats_line(label: str, values: list[float], unit: str = "", scale: float = 1.0) -> str:
    if not values:
        return f"  {label:<38} NO DATA"
    vs = [v * scale for v in values]
    sv = sorted(vs)
    n = len(sv)
    mean = statistics.mean(vs)
    med = sv[n // 2]
    p25 = sv[n // 4]
    p75 = sv[min(n - 1, 3 * n // 4)]
    p95 = sv[min(n - 1, int(n * 0.95))]
    return (f"  {label:<38} n={n:<5d}  "
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


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
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
            "overlap_ppt",
        ]
        print("\t".join(cols))
        for i, f in enumerate(frames):
            s = f.get("stages", {})
            ev_a      = s.get("depth_slot_a.evidence_count", "")
            ev_b      = s.get("depth_slot_b.evidence_count", "")
            range_a   = (s["depth.median_range_a_m"] / 1000) if "depth.median_range_a_m" in s else ""
            range_b   = (s["depth.median_range_b_m"] / 1000) if "depth.median_range_b_m" in s else ""
            ratio     = (s["depth.scale_ratio"] / 1000)      if "depth.scale_ratio"       in s else ""
            overlap   = s.get("depth.voxel_overlap_ppt", "")
            ts        = f.get("timestamp_ns", "")
            print(f"{i}\t{ts}\t{ev_a}\t{ev_b}\t{range_a}\t{range_b}\t{ratio}\t{overlap}")
        return

    # ── Summary mode ──────────────────────────────────────────────────────────
    total = len(frames)
    all_stages: set[str] = set()
    for f in frames:
        all_stages.update(f.get("stages", {}).keys())

    depth_stages = sorted(s for s in all_stages if "depth" in s)

    print(f"\n{'='*70}")
    print(f"  A/B Depth Eval Report")
    print(f"  frames: {total}   files: {len(paths)}")
    print(f"{'='*70}")

    # ── Stage presence check ──────────────────────────────────────────────────
    expected = {
        "depth_slot_a.evidence_count",
        "depth_slot_b.evidence_count",
        "depth.voxel_overlap_ppt",
        "depth.median_range_a_m",
        "depth.median_range_b_m",
        "depth.scale_ratio",
    }
    missing = expected - all_stages
    if missing:
        print(f"\n  ⚠️  MISSING STAGES (stale build or slot B inactive):")
        for s in sorted(missing):
            print(f"     - {s}")
        print()
        if "depth_slot_b.evidence_count" not in all_stages:
            print("  → depth_slot_b has NO data. Likely causes:")
            print("    1. EC2 build is stale — rebuild with: cmake --build build-staging -j$(nproc)")
            print("    2. DEDALUS_DEPTH_EVAL=airsim_gt_detector not set")
            print("    3. Config missing 'depth_eval: airsim_gt_detector'")
            print()

    # ── Evidence counts ───────────────────────────────────────────────────────
    ev_a = extract(frames, "depth_slot_a.evidence_count")
    ev_b = extract(frames, "depth_slot_b.evidence_count")
    ev_b_nonzero = [v for v in ev_b if v > 0]

    print(f"\n  Evidence counts (obstacles per frame)")
    print(f"  {'─'*65}")
    print(stats_line("slot A  (ONNX primary)", ev_a, " obs"))
    print(stats_line("slot B  (GT eval)", ev_b, " obs"))
    if ev_b:
        nonzero_pct = 100.0 * len(ev_b_nonzero) / len(ev_b)
        print(f"  {'slot B non-zero frames':<38} {len(ev_b_nonzero)}/{len(ev_b)}  ({nonzero_pct:.1f}%)")
        if len(ev_b_nonzero) < args.min_frames:
            print(f"\n  ⚠️  Slot B produced evidence in fewer than {args.min_frames} frames.")
            print("     Check that --include-depth is in the bridge command")
            print("     (use config/drone/px4_front_center_depth.yaml).")

    # ── Range comparison ──────────────────────────────────────────────────────
    range_a = [v / 1000.0 for v in extract(frames, "depth.median_range_a_m") if v > 0]
    range_b = [v / 1000.0 for v in extract(frames, "depth.median_range_b_m") if v > 0]

    print(f"\n  Median obstacle range per frame (metres)")
    print(f"  {'─'*65}")
    print(stats_line("slot A  range (ONNX)", range_a, " m"))
    print(stats_line("slot B  range (GT)",   range_b, " m"))

    # ── Scale ratio ───────────────────────────────────────────────────────────
    ratios = [v / 1000.0 for v in extract(frames, "depth.scale_ratio") if v > 0]

    print(f"\n  Scale ratio  (GT_range / ONNX_range) — 1.0 = perfect")
    print(f"  {'─'*65}")
    if ratios:
        sv = sorted(ratios)
        n = len(sv)
        mean = statistics.mean(ratios)
        med  = sv[n // 2]
        p25  = sv[n // 4]
        p75  = sv[min(n - 1, 3 * n // 4)]
        p95  = sv[min(n - 1, int(n * 0.95))]
        print(f"  {'ratio':<38} n={n:<5d}  mean={mean:6.2f}x  "
              f"p25={p25:.2f}x  p50={med:.2f}x  p75={p75:.2f}x  p95={p95:.2f}x")
        print()
        if med > 1.5:
            suggested = round(med, 2)
            print(f"  → ONNX reads ~{1/med:.2f}× the GT distance (ONNX under-estimates range).")
            print(f"    To correct: set  visual_onnx.scale: {suggested}  in config/pipeline/visual.yaml")
            print(f"    (current value is 1.0 — multiply raw ONNX output by {suggested})")
        elif med < 0.67:
            print(f"  → ONNX over-estimates range vs GT (ratio {med:.2f}×).")
            print(f"    Set  visual_onnx.scale: {round(med, 2)}  to compensate.")
        else:
            print(f"  → Scale error within ±50% of GT — reasonably calibrated.")
    else:
        print(f"  {'scale_ratio':<38} NO DATA")

    # ── Voxel overlap ─────────────────────────────────────────────────────────
    overlap = [v / 10.0 for v in extract(frames, "depth.voxel_overlap_ppt")]  # ‰ → %
    print(f"\n  Voxel overlap agreement (A vs B within ±0.5 m voxel)")
    print(f"  {'─'*65}")
    print(stats_line("overlap", overlap, "%"))

    # ── All depth stages present in file ─────────────────────────────────────
    print(f"\n  All depth-related stages in this JSONL:")
    print(f"  {'─'*65}")
    if depth_stages:
        for s in depth_stages:
            vals = extract(frames, s)
            if vals:
                print(f"    {s:<45} n={len(vals)}")
    else:
        print("  (none found — likely a stale build missing depth metrics)")

    print(f"\n{'='*70}\n")


if __name__ == "__main__":
    main()
