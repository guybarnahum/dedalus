#!/usr/bin/env python3
"""
ego_state_report.py — VisualEgoStateProvider vs AirSim ground-truth comparison.

Reads per-frame profiler JSONL from a mission output directory and generates
an HTML report comparing slot A (visual_odometry) against slot B (airsim / frame_hint).

Profiler stages consumed:
  ego_provider.slot.agreement_ppt   — int, position distance ppt of 1 m (0 = 0 mm, 1000 = 1 m+)
  ego_state_provider.estimate       — int, estimate latency (µs)

The ego_state_provider.estimate stage is logged by CoreStackRunner for every frame.
The agreement is logged whenever ego_provider_eval is active (slot B configured).

Typical config that produces this data:
  ego_provider:      visual_odometry
  ego_provider_eval: airsim        # AirSim telemetry as slot B oracle
    or
  ego_provider_eval: frame_hint    # AirSim pose hint as slot B oracle

Usage:
  python3 tools/perception/ego_state_report.py <mission_output_dir> [-o report.html]

The profiler JSONL is expected at:
  <mission_output_dir>/profiler_log.jsonl   (one JSON object per line per frame)
"""

import argparse
import json
import math
import os
import sys
from pathlib import Path


# ── Data loading ──────────────────────────────────────────────────────────────

STAGE_KEYS = {
    "agreement": "ego_provider.slot.agreement_ppt",
    "latency":   "ego_state_provider.estimate",
}


def load_profiler_jsonl(path: Path) -> list[dict]:
    records = []
    with open(path) as fh:
        for lineno, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as exc:
                print(f"  WARNING: line {lineno} skipped ({exc})", file=sys.stderr)
    return records


def extract_frames(records: list[dict]) -> list[dict]:
    frames = []
    for rec in records:
        stages = rec.get("stages", rec)

        def get(key):
            return stages.get(STAGE_KEYS[key])

        agreement = get("agreement")
        latency   = get("latency")

        if agreement is None and latency is None:
            continue

        frames.append({
            "frame":     rec.get("frame_id", len(frames)),
            "agreement": int(agreement) if agreement is not None else None,
            "latency":   int(latency)   if latency   is not None else None,
        })
    return frames


# ── Statistics helpers ────────────────────────────────────────────────────────

def stats(values: list[float]) -> dict:
    if not values:
        return {"min": 0, "max": 0, "mean": 0, "p50": 0, "p95": 0}
    sv = sorted(values)
    n  = len(sv)
    return {
        "min":  sv[0],
        "max":  sv[-1],
        "mean": sum(sv) / n,
        "p50":  sv[n // 2],
        "p95":  sv[min(n - 1, int(n * 0.95))],
    }


def ppt_to_mm(ppt: int) -> float:
    """Convert ppt-of-1m to millimetres."""
    return ppt * 1.0  # 1 ppt = 1 mm (1000 ppt = 1 m)


# ── HTML generation ───────────────────────────────────────────────────────────

CHART_JS_CDN = "https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.0/chart.umd.min.js"

def _js_array(values) -> str:
    return "[" + ",".join("null" if v is None else str(v) for v in values) + "]"


def render_html(frames: list[dict], source_dir: str) -> str:
    frame_ids   = [f["frame"] for f in frames]
    agreements  = [f["agreement"] for f in frames]
    latencies   = [f["latency"]   for f in frames]

    agr_vals   = [ppt_to_mm(v) for v in agreements if v is not None]
    lat_vals   = [v / 1000.0   for v in latencies  if v is not None]  # µs → ms

    agr_st  = stats(agr_vals)
    lat_st  = stats(lat_vals)

    # Convert agreement ppt → mm for chart
    agr_mm = [ppt_to_mm(v) if v is not None else None for v in agreements]

    # Histogram bins for agreement (mm)
    hist_bins = list(range(0, 1010, 50))  # 0..1000 mm in 50 mm steps → 20 bins
    hist_counts = [0] * (len(hist_bins) - 1)
    for v in agr_vals:
        bi = int(min(v, 999.9) // 50)
        hist_counts[min(bi, len(hist_counts) - 1)] += 1
    hist_labels = [f"{hist_bins[i]}–{hist_bins[i+1]}" for i in range(len(hist_bins)-1)]

    # Cumulative drift: running sum of agreement / 1000 (metres)
    cum_drift = []
    total = 0.0
    for v in agreements:
        if v is not None:
            total += v / 1000.0  # ppt → m per frame (rough)
        cum_drift.append(round(total, 3))

    n_frames    = len(frames)
    n_agr       = len(agr_vals)
    n_lat       = len(lat_vals)
    has_agr     = n_agr > 0
    has_lat     = n_lat > 0

    def fmt(v, decimals=2):
        return f"{v:.{decimals}f}" if v is not None else "—"

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Ego State Report — {source_dir}</title>
<script src="{CHART_JS_CDN}"></script>
<style>
*{{box-sizing:border-box;margin:0;padding:0}}
body{{font-family:system-ui,sans-serif;background:#0e1117;color:#d0d6e8;padding:24px}}
h1{{font-size:1.35rem;color:#7eb3f7;margin-bottom:4px}}
.subtitle{{color:#5a6478;font-size:.85rem;margin-bottom:28px}}
.grid{{display:grid;grid-template-columns:repeat(auto-fill,minmax(200px,1fr));gap:12px;margin-bottom:28px}}
.card{{background:#161b27;border-radius:8px;padding:16px;border:1px solid #1f2a40}}
.card h3{{font-size:.75rem;color:#5a7090;text-transform:uppercase;letter-spacing:.05em;margin-bottom:8px}}
.card .val{{font-size:1.6rem;font-weight:700;color:#e2e8f8}}
.card .unit{{font-size:.75rem;color:#4a6080;margin-left:4px}}
.card .sub{{font-size:.75rem;color:#5a7090;margin-top:4px}}
.panel{{background:#161b27;border-radius:8px;padding:20px;border:1px solid #1f2a40;margin-bottom:20px}}
.panel h2{{font-size:.9rem;color:#8ab0d8;text-transform:uppercase;letter-spacing:.06em;margin-bottom:16px}}
canvas{{max-height:240px}}
.good{{color:#34d399}} .warn{{color:#fbbf24}} .bad{{color:#f87171}}
</style>
</head>
<body>
<h1>Ego State Provider Report</h1>
<div class="subtitle">
  Source: {source_dir} &nbsp;·&nbsp;
  {n_frames} frames &nbsp;·&nbsp;
  {n_agr} agreement samples &nbsp;·&nbsp;
  slot A = visual_odometry &nbsp;·&nbsp; slot B = airsim / frame_hint
</div>

<div class="grid">
  <div class="card">
    <h3>Mean position error</h3>
    <div class="val {"good" if agr_st["mean"]<200 else "warn" if agr_st["mean"]<500 else "bad"}">{fmt(agr_st["mean"])}<span class="unit">mm</span></div>
    <div class="sub">p95 = {fmt(agr_st["p95"])} mm</div>
  </div>
  <div class="card">
    <h3>Max position error</h3>
    <div class="val">{fmt(agr_st["max"])}<span class="unit">mm</span></div>
    <div class="sub">min = {fmt(agr_st["min"])} mm</div>
  </div>
  <div class="card">
    <h3>Total drift (sum)</h3>
    <div class="val">{fmt(total, 3)}<span class="unit">m</span></div>
    <div class="sub">over {n_agr} frames</div>
  </div>
  <div class="card">
    <h3>VO latency (mean)</h3>
    <div class="val {"good" if lat_st["mean"]<5 else "warn" if lat_st["mean"]<15 else "bad"}">{fmt(lat_st["mean"])}<span class="unit">ms</span></div>
    <div class="sub">p95 = {fmt(lat_st["p95"])} ms</div>
  </div>
</div>

<div class="panel">
  <h2>Position error vs AirSim oracle (mm per frame)</h2>
  <canvas id="agr-chart"></canvas>
</div>

<div class="panel">
  <h2>Cumulative drift (metres)</h2>
  <canvas id="drift-chart"></canvas>
</div>

<div class="panel">
  <h2>Error distribution (mm)</h2>
  <canvas id="hist-chart"></canvas>
</div>

<div class="panel">
  <h2>Estimate latency (ms per frame)</h2>
  <canvas id="lat-chart"></canvas>
</div>

<script>
const frameIds  = {_js_array(frame_ids)};
const agrMm     = {_js_array(agr_mm)};
const latMs     = {_js_array([round(v/1000, 3) if v is not None else None for v in latencies] if has_lat else [])};
const cumDrift  = {_js_array(cum_drift)};
const histLabels = {json.dumps(hist_labels)};
const histCounts = {json.dumps(hist_counts)};

const DARK = '#0e1117';
Chart.defaults.color = '#6a7898';
Chart.defaults.borderColor = '#1f2a40';

function lineChart(id, labels, datasets, ylabel) {{
  new Chart(document.getElementById(id), {{
    type: 'line',
    data: {{ labels, datasets }},
    options: {{
      responsive: true, maintainAspectRatio: true,
      animation: false,
      plugins: {{ legend: {{ display: datasets.length > 1 }} }},
      scales: {{
        x: {{ ticks: {{ maxTicksLimit: 12 }} }},
        y: {{ title: {{ display: true, text: ylabel }} }}
      }}
    }}
  }});
}}

lineChart('agr-chart', frameIds, [{{
  label: 'Error (mm)', data: agrMm,
  borderColor: '#7eb3f7', backgroundColor: 'rgba(126,179,247,0.08)',
  pointRadius: 0, borderWidth: 1.5, tension: 0.3, fill: true,
  spanGaps: true
}}], 'mm');

lineChart('drift-chart', frameIds, [{{
  label: 'Cumulative drift (m)', data: cumDrift,
  borderColor: '#fbbf24', backgroundColor: 'rgba(251,191,36,0.08)',
  pointRadius: 0, borderWidth: 1.5, tension: 0.3, fill: true
}}], 'm');

new Chart(document.getElementById('hist-chart'), {{
  type: 'bar',
  data: {{
    labels: histLabels,
    datasets: [{{ label: 'Frames', data: histCounts,
      backgroundColor: '#3b5fa0', borderColor: '#4a7ac8', borderWidth: 1 }}]
  }},
  options: {{
    responsive: true, maintainAspectRatio: true, animation: false,
    plugins: {{ legend: {{ display: false }} }},
    scales: {{ y: {{ title: {{ display: true, text: 'Frames' }} }} }}
  }}
}});

lineChart('lat-chart', frameIds, [{{
  label: 'Latency (ms)', data: latMs,
  borderColor: '#34d399', backgroundColor: 'rgba(52,211,153,0.08)',
  pointRadius: 0, borderWidth: 1.2, tension: 0.3, fill: true,
  spanGaps: true
}}], 'ms');
</script>
</body>
</html>
"""
    return html


# ── CLI ───────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mission_dir", help="Mission output directory containing profiler_log.jsonl")
    ap.add_argument("-o", "--output", default="ego_state_report.html",
                    help="Output HTML path (default: ego_state_report.html)")
    args = ap.parse_args()

    mission_dir = Path(args.mission_dir)
    jsonl_path  = mission_dir / "profiler_log.jsonl"

    if not jsonl_path.exists():
        print(f"ERROR: profiler log not found at {jsonl_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Loading {jsonl_path} …")
    records = load_profiler_jsonl(jsonl_path)
    print(f"  {len(records)} records loaded")

    frames = extract_frames(records)
    print(f"  {len(frames)} ego frames extracted")

    if not frames:
        print("WARNING: no ego provider data found in profiler log.", file=sys.stderr)
        print("  Ensure ego_provider: visual_odometry and ego_provider_eval: airsim"
              " are set in config.", file=sys.stderr)
        sys.exit(1)

    agr_count = sum(1 for f in frames if f["agreement"] is not None)
    if agr_count == 0:
        print("WARNING: no ego_provider.slot.agreement_ppt found.", file=sys.stderr)
        print("  Set ego_provider_eval: airsim (or frame_hint) in config to enable"
              " slot B comparison.", file=sys.stderr)

    html = render_html(frames, str(mission_dir))

    out_path = Path(args.output)
    out_path.write_text(html, encoding="utf-8")
    print(f"\nReport written to {out_path}")
    print(f"Open with: open {out_path}")


if __name__ == "__main__":
    main()
