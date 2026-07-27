#!/usr/bin/env python3
"""Report L3/ESDF catch-up behavior from a Dedalus pipeline profiler JSONL.

L3 is a lagging cache of L2, kept up to date by two independent sources in
CoreStackRunner::run_once():
  - L2 value changes, pulled as one coalesced delta at most once per
    kESDFValueCatchupMinIntervalUs (default 1s) -- NOT every tick.
    update_incremental()'s cost is dominated by an unindexed scan over all
    of L2 (MissionLocalPlanningMap::query_occupied_ts_in_box()), which
    doesn't shrink with a smaller window, so this is throttled rather than
    attempted every tick.
  - L2 residency changes (cells slide_window() loads from disk), queued as
    they occur and drained up to esdf_catchup_budget_us_ (default 20ms;
    esdf.catchup_budget_us) of wall-clock time per tick. Not throttled the
    same way -- slides are infrequent enough already that this doesn't add
    up the way the every-tick value-delta did.

Both only show up in [PipelineSlow] terminal output when slow enough to
blow the frame budget -- which by design they almost never should be. This
tool reads every frame instead, so gradual drift or a growing backlog is
visible even when nothing ever printed to stderr.

Stages read (all optional -- absent unless the catch-up step actually did
something that tick):
  esdf.catchup_us                 time spent applying catch-up work this
                                   tick (residency batches, and/or the
                                   value-delta on ticks where its throttle
                                   interval has elapsed)
  esdf.pending_residency_batches  residency queue depth after this tick's
                                   drain -- expect this near 0 most of the
                                   time; growing means slides are outpacing
                                   esdf_catchup_budget_us_
  esdf.publish                    time spent serializing + sending to SSE
  esdf.cell_count                 L3 cell count, sampled only on esdf.publish
                                   ticks (throttled to the same ~2s cadence
                                   as L1/L2) -- expect mostly blank rows in
                                   the per-frame table below; that's normal.

Usage:
  python3 tools/mission/esdf-catchup-report.py out/circle_airsim_gt/profile/pipeline_*.jsonl
  python3 tools/mission/esdf-catchup-report.py --sample-every 50 pipeline_*.jsonl
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path


def load_frames(paths: list[Path]) -> list[dict]:
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


def stage(frame: dict, key: str) -> float | None:
    v = frame.get("stages", {}).get(key)
    return None if v is None else float(v)


def fmt(v: float | None, decimals: int = 0) -> str:
    return "-" if v is None else f"{v:.{decimals}f}"


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("jsonl", nargs="+", type=Path, help="pipeline profiler JSONL file(s)")
    ap.add_argument("--sample-every", type=int, default=0,
                     help="print an L1/L2/L3 cell-count row every N frames (0 = auto, ~25 rows)")
    args = ap.parse_args()

    frames = load_frames(args.jsonl)
    if not frames:
        print("No frames loaded.", file=sys.stderr)
        return 1

    n = len(frames)
    sample_every = args.sample_every or max(1, n // 25)

    print(f"Loaded {n} frames from {len(args.jsonl)} file(s)\n")

    catchup_us = [v for f in frames if (v := stage(f, "esdf.catchup_us")) is not None]
    if catchup_us:
        print(f"esdf.catchup_us                 n={len(catchup_us)}/{n} frames applied something  "
              f"mean={statistics.mean(catchup_us):.1f}us  "
              f"p50={statistics.median(catchup_us):.1f}us  "
              f"max={max(catchup_us):.1f}us")
        print("  (expect n well below the frame count -- the value-delta half is throttled to "
              "~once/sec; a count close to n means the throttle isn't taking effect)")
    else:
        print("esdf.catchup_us                 never recorded -- catch-up step never applied "
              "anything (no L2 changes seen, or esdf_map_publisher_ not configured)")

    pending = [v for f in frames if (v := stage(f, "esdf.pending_residency_batches")) is not None]
    if pending:
        trend = "draining/stable" if pending[-1] <= max(pending) / 2 or max(pending) <= 1 else "GROWING"
        print(f"esdf.pending_residency_batches  max={max(pending):.0f}  end={pending[-1]:.0f}  "
              f"({trend} -- a backlog that keeps growing means esdf_catchup_budget_us is too small "
              "for how fast slide_window() is loading new cells)")

    publish_us = [v for f in frames if (v := stage(f, "esdf.publish")) is not None]
    if publish_us:
        print(f"esdf.publish                    n={len(publish_us)} publishes over {n} frames  "
              f"mean={statistics.mean(publish_us):.1f}us  max={max(publish_us):.1f}us")

    print(f"\nDoes L3 track L1/L2 growth over the mission? (blank esdf_cells = no publish this row,\n"
          f"not a lag -- publish is throttled to ~2s; check the trend across rows instead.)\n")
    header = f"{'frame':>6}  {'l1_total':>9}  {'l2_cells':>9}  {'esdf_cells':>10}  {'catchup_us':>10}  {'pending':>7}"
    print(header)
    print("-" * len(header))
    last_printed = -1
    for i, f in enumerate(frames):
        is_last = i == n - 1
        if i % sample_every != 0 and not is_last:
            continue
        if i == last_printed:
            continue
        last_printed = i
        l1 = stage(f, "mission_map_assimilator.l1_total_cells")
        l2 = stage(f, "planning_map.cell_count")
        esdf_cells = stage(f, "esdf.cell_count")
        c_us = stage(f, "esdf.catchup_us")
        pend = stage(f, "esdf.pending_residency_batches")
        print(f"{i:>6}  {fmt(l1):>9}  {fmt(l2):>9}  {fmt(esdf_cells):>10}  {fmt(c_us):>10}  {fmt(pend):>7}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
