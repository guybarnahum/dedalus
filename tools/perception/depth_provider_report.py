#!/usr/bin/env python3
"""
depth_provider_report.py — A vs B depth provider comparison report.

Reads per-frame profiler JSONL from a mission output directory and generates
a rich HTML report comparing slot A and slot B depth providers.

Profiler stages consumed:
  depth_slot_a.evidence_count   — int, evidence items from slot A
  depth_slot_b.evidence_count   — int, evidence items from slot B (optional)
  depth_slot.agreement_ppt      — int, A∩B agreement parts-per-thousand (0–1000)
  depth_slot_a.detect           — int, slot A detect latency (µs)

Usage:
  python3 tools/perception/depth_provider_report.py <mission_output_dir> [-o report.html]

The profiler JSONL is expected at:
  <mission_output_dir>/profiler_log.jsonl   (one JSON object per line per frame)
"""

import argparse
import json
import math
import os
import re
import sys
from pathlib import Path


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

STAGE_KEYS = {
    "a_count":    "depth_slot_a.evidence_count",
    "b_count":    "depth_slot_b.evidence_count",
    "agreement":  "depth_slot.agreement_ppt",
    "a_latency":  "depth_slot_a.detect",
}


def load_profiler_jsonl(path: Path) -> list[dict]:
    """Load a profiler JSONL file.  Each line is one frame record."""
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
    """Extract per-frame slot stats from profiler records.

    The profiler log schema is:
      {"frame_id": <int>, "stages": {"<name>": <value>, ...}, ...}
    """
    frames = []
    for rec in records:
        stages = rec.get("stages", rec)  # tolerate flat and nested formats
        def get(key):
            return stages.get(STAGE_KEYS[key])

        a_count   = get("a_count")
        b_count   = get("b_count")
        agreement = get("agreement")
        a_latency = get("a_latency")

        if a_count is None:
            continue  # frame has no depth slot data

        frames.append({
            "frame":     rec.get("frame_id", len(frames)),
            "a_count":   int(a_count),
            "b_count":   int(b_count) if b_count is not None else None,
            "agreement": int(agreement) / 10.0 if agreement is not None else None,  # % (0–100)
            "a_latency": int(a_latency) if a_latency is not None else None,
        })
    return frames


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------

def _safe_mean(values):
    vals = [v for v in values if v is not None]
    return sum(vals) / len(vals) if vals else None


def _safe_pct(values, pct):
    vals = sorted(v for v in values if v is not None)
    if not vals:
        return None
    idx = max(0, min(len(vals) - 1, int(math.ceil(pct / 100.0 * len(vals))) - 1))
    return vals[idx]


def compute_summary(frames: list[dict]) -> dict:
    a_counts   = [f["a_count"]   for f in frames]
    b_counts   = [f["b_count"]   for f in frames if f["b_count"] is not None]
    agreements = [f["agreement"] for f in frames if f["agreement"] is not None]
    latencies  = [f["a_latency"] for f in frames if f["a_latency"] is not None]

    has_b = len(b_counts) > 0

    return {
        "frame_count": len(frames),
        "has_b": has_b,
        "a_mean":     _safe_mean(a_counts),
        "a_p50":      _safe_pct(a_counts, 50),
        "a_p95":      _safe_pct(a_counts, 95),
        "b_mean":     _safe_mean(b_counts) if has_b else None,
        "b_p50":      _safe_pct(b_counts, 50) if has_b else None,
        "b_p95":      _safe_pct(b_counts, 95) if has_b else None,
        "agree_mean": _safe_mean(agreements),
        "agree_p50":  _safe_pct(agreements, 50),
        "agree_min":  min(agreements) if agreements else None,
        "agree_max":  max(agreements) if agreements else None,
        "lat_mean":   _safe_mean(latencies),
        "lat_p95":    _safe_pct(latencies, 95),
    }


def build_iou_histogram(agreements, n_bins=20):
    """Return (bin_labels, counts) for an agreement % histogram."""
    if not agreements:
        return [], []
    width = 100.0 / n_bins
    counts = [0] * n_bins
    for v in agreements:
        idx = min(n_bins - 1, int(v / width))
        counts[idx] += 1
    labels = [f"{i * width:.0f}–{(i+1) * width:.0f}%" for i in range(n_bins)]
    return labels, counts


# ---------------------------------------------------------------------------
# HTML generation
# ---------------------------------------------------------------------------

_HTML_TEMPLATE = """\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Depth Provider Report — {title}</title>
<style>
  :root {{
    --bg: #0d1117; --card: #161b22; --border: #30363d;
    --text: #c9d1d9; --muted: #8b949e;
    --a-col: #58a6ff; --b-col: #3fb950; --agree-col: #d2a8ff;
    --warn: #f0883e; --danger: #ff7b72;
  }}
  * {{ box-sizing: border-box; margin: 0; padding: 0; }}
  body {{ background: var(--bg); color: var(--text); font-family: -apple-system, sans-serif; padding: 24px; }}
  h1 {{ font-size: 1.4rem; margin-bottom: 4px; }}
  .subtitle {{ color: var(--muted); font-size: 0.85rem; margin-bottom: 24px; }}
  .grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 16px; margin-bottom: 24px; }}
  .card {{ background: var(--card); border: 1px solid var(--border); border-radius: 8px; padding: 16px; }}
  .card .label {{ font-size: 0.75rem; color: var(--muted); margin-bottom: 6px; text-transform: uppercase; letter-spacing: 0.05em; }}
  .card .value {{ font-size: 1.6rem; font-weight: 600; }}
  .card .sub {{ font-size: 0.8rem; color: var(--muted); margin-top: 4px; }}
  .chart-wrap {{ background: var(--card); border: 1px solid var(--border); border-radius: 8px; padding: 16px; margin-bottom: 24px; }}
  .chart-wrap h2 {{ font-size: 1rem; margin-bottom: 14px; }}
  canvas {{ width: 100% !important; }}
  table {{ width: 100%; border-collapse: collapse; font-size: 0.82rem; }}
  th, td {{ text-align: right; padding: 6px 10px; border-bottom: 1px solid var(--border); }}
  th {{ color: var(--muted); font-weight: 400; }}
  td:first-child, th:first-child {{ text-align: left; }}
  tr:hover td {{ background: rgba(255,255,255,0.03); }}
  .agree-good {{ color: var(--b-col); }}
  .agree-warn {{ color: var(--warn); }}
  .agree-bad  {{ color: var(--danger); }}
</style>
</head>
<body>
<h1>Depth Provider Report</h1>
<div class="subtitle">{subtitle}</div>

<div class="grid">
  <div class="card">
    <div class="label">Frames analysed</div>
    <div class="value">{frame_count}</div>
  </div>
  <div class="card">
    <div class="label">Slot A mean evidence</div>
    <div class="value" style="color:var(--a-col)">{a_mean}</div>
    <div class="sub">p50={a_p50} &nbsp; p95={a_p95}</div>
  </div>
  {b_card}
  {agree_card}
  <div class="card">
    <div class="label">Slot A detect latency</div>
    <div class="value">{lat_mean} µs</div>
    <div class="sub">p95={lat_p95} µs</div>
  </div>
</div>

<!-- Evidence count time-series -->
<div class="chart-wrap">
  <h2>Evidence count per frame</h2>
  <canvas id="tsChart" height="80"></canvas>
</div>

<!-- Agreement time-series (shown only when slot B active) -->
{agree_ts_section}

<!-- IoU / agreement histogram -->
{hist_section}

<!-- Per-frame data table -->
<div class="chart-wrap">
  <h2>Per-frame data (last 500 frames)</h2>
  <table id="frameTable">
    <thead><tr>
      <th>Frame</th>
      <th>A evidence</th>
      {b_th}
      {agree_th}
      <th>A latency (µs)</th>
    </tr></thead>
    <tbody>
      {table_rows}
    </tbody>
  </table>
</div>

<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.2/dist/chart.umd.min.js"></script>
<script>
const FRAMES = {frames_json};
const labels = FRAMES.map(f => f.frame);
const aData  = FRAMES.map(f => f.a_count);
const bData  = FRAMES.map(f => f.b_count);
const agData = FRAMES.map(f => f.agreement);
const hasB   = bData.some(v => v !== null);
const hasAg  = agData.some(v => v !== null);

Chart.defaults.color = '#8b949e';
Chart.defaults.borderColor = '#30363d';

// Evidence time-series
(function() {{
  const datasets = [{{
    label: 'Slot A',
    data: aData,
    borderColor: '#58a6ff',
    backgroundColor: 'rgba(88,166,255,0.08)',
    borderWidth: 1.5,
    pointRadius: 0,
    fill: true,
    tension: 0.2,
  }}];
  if (hasB) datasets.push({{
    label: 'Slot B',
    data: bData,
    borderColor: '#3fb950',
    backgroundColor: 'rgba(63,185,80,0.06)',
    borderWidth: 1.5,
    pointRadius: 0,
    fill: true,
    tension: 0.2,
  }});
  new Chart(document.getElementById('tsChart'), {{
    type: 'line',
    data: {{ labels, datasets }},
    options: {{ animation: false, responsive: true, plugins: {{ legend: {{ position: 'top' }} }} }},
  }});
}})();

// Agreement time-series
if (hasAg && document.getElementById('agChart')) {{
  new Chart(document.getElementById('agChart'), {{
    type: 'line',
    data: {{
      labels,
      datasets: [{{
        label: 'Agreement %',
        data: agData,
        borderColor: '#d2a8ff',
        backgroundColor: 'rgba(210,168,255,0.08)',
        borderWidth: 1.5,
        pointRadius: 0,
        fill: true,
        tension: 0.2,
      }}],
    }},
    options: {{
      animation: false, responsive: true,
      scales: {{ y: {{ min: 0, max: 100 }} }},
      plugins: {{ legend: {{ position: 'top' }} }},
    }},
  }});
}}

// IoU histogram
if (document.getElementById('histChart')) {{
  new Chart(document.getElementById('histChart'), {{
    type: 'bar',
    data: {{
      labels: {hist_labels_json},
      datasets: [{{
        label: 'Frame count',
        data: {hist_counts_json},
        backgroundColor: 'rgba(210,168,255,0.7)',
        borderColor: '#d2a8ff',
        borderWidth: 1,
      }}],
    }},
    options: {{
      animation: false, responsive: true,
      plugins: {{ legend: {{ display: false }} }},
      scales: {{ x: {{ ticks: {{ maxRotation: 45 }} }} }},
    }},
  }});
}}
</script>
</body>
</html>
"""


def _fmt(v, fmt=".1f", fallback="—"):
    if v is None:
        return fallback
    return format(v, fmt)


def _agree_class(v):
    if v is None:
        return ""
    if v >= 70:
        return "agree-good"
    if v >= 40:
        return "agree-warn"
    return "agree-bad"


def render_html(frames: list[dict], summary: dict, source_path: Path, output_path: Path) -> str:
    title = source_path.parent.name
    subtitle = f"Mission: {source_path.parent.resolve()} &nbsp;|&nbsp; Frames: {summary['frame_count']}"

    # Cards
    if summary["has_b"]:
        b_card = f"""
  <div class="card">
    <div class="label">Slot B mean evidence</div>
    <div class="value" style="color:var(--b-col)">{_fmt(summary['b_mean'])}</div>
    <div class="sub">p50={_fmt(summary['b_p50'],'g')} &nbsp; p95={_fmt(summary['b_p95'],'g')}</div>
  </div>"""
        agree_card = f"""
  <div class="card">
    <div class="label">Mean agreement</div>
    <div class="value {_agree_class(summary['agree_mean'])}">{_fmt(summary['agree_mean'])}%</div>
    <div class="sub">min={_fmt(summary['agree_min'])}% &nbsp; max={_fmt(summary['agree_max'])}%</div>
  </div>"""
    else:
        b_card = ""
        agree_card = ""

    # Agreement time-series section
    if summary["has_b"] and summary["agree_mean"] is not None:
        agree_ts_section = """
<div class="chart-wrap">
  <h2>A vs B agreement % per frame</h2>
  <canvas id="agChart" height="60"></canvas>
</div>"""
    else:
        agree_ts_section = ""

    # Histogram section
    agreements = [f["agreement"] for f in frames if f["agreement"] is not None]
    if agreements:
        h_labels, h_counts = build_iou_histogram(agreements)
        hist_section = """
<div class="chart-wrap">
  <h2>Agreement % distribution</h2>
  <canvas id="histChart" height="60"></canvas>
</div>"""
    else:
        h_labels, h_counts = [], []
        hist_section = ""

    # Table — last 500 frames
    b_th     = "<th>B evidence</th>" if summary["has_b"] else ""
    agree_th = "<th>Agreement %</th>" if summary["has_b"] else ""

    display_frames = frames[-500:]
    rows = []
    for f in display_frames:
        ag_cls = _agree_class(f["agreement"])
        b_td     = f"<td>{f['b_count'] if f['b_count'] is not None else '—'}</td>" if summary["has_b"] else ""
        agree_td = (f"<td class='{ag_cls}'>{_fmt(f['agreement'])}%</td>"
                    if summary["has_b"] else "")
        rows.append(
            f"<tr>"
            f"<td>{f['frame']}</td>"
            f"<td>{f['a_count']}</td>"
            f"{b_td}"
            f"{agree_td}"
            f"<td>{f['a_latency'] if f['a_latency'] is not None else '—'}</td>"
            f"</tr>"
        )

    # Chart JSON (last 2000 frames for readability)
    chart_frames = frames[-2000:]
    frames_json  = json.dumps([{k: f[k] for k in ("frame", "a_count", "b_count", "agreement")}
                                for f in chart_frames])

    return _HTML_TEMPLATE.format(
        title=title,
        subtitle=subtitle,
        frame_count=summary["frame_count"],
        a_mean=_fmt(summary["a_mean"]),
        a_p50=_fmt(summary["a_p50"], "g"),
        a_p95=_fmt(summary["a_p95"], "g"),
        b_card=b_card,
        agree_card=agree_card,
        lat_mean=_fmt(summary["lat_mean"], ".0f"),
        lat_p95=_fmt(summary["lat_p95"], ".0f"),
        agree_ts_section=agree_ts_section,
        hist_section=hist_section,
        b_th=b_th,
        agree_th=agree_th,
        table_rows="\n      ".join(rows) if rows else "<tr><td colspan='5'>No data</td></tr>",
        frames_json=frames_json,
        hist_labels_json=json.dumps(h_labels),
        hist_counts_json=json.dumps(h_counts),
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mission_dir", help="Mission output directory containing profiler_log.jsonl")
    parser.add_argument("-o", "--output", default=None, help="Output HTML path (default: <mission_dir>/depth_provider_report.html)")
    args = parser.parse_args()

    mission_dir = Path(args.mission_dir)
    profiler_path = mission_dir / "profiler_log.jsonl"

    if not profiler_path.exists():
        print(f"ERROR: profiler log not found: {profiler_path}", file=sys.stderr)
        sys.exit(1)

    output_path = Path(args.output) if args.output else (mission_dir / "depth_provider_report.html")

    print(f"Loading {profiler_path} ...")
    records = load_profiler_jsonl(profiler_path)
    print(f"  {len(records)} log records")

    frames = extract_frames(records)
    if not frames:
        print("ERROR: no depth_slot_a frames found in profiler log.", file=sys.stderr)
        print("  Make sure the mission ran with a depth slot configured.", file=sys.stderr)
        sys.exit(1)

    print(f"  {len(frames)} frames with depth slot data")

    summary = compute_summary(frames)
    has_b = summary["has_b"]

    print(f"  Slot A: mean={_fmt(summary['a_mean'])} evidence/frame, p95={_fmt(summary['a_p95'],'g')}")
    if has_b:
        print(f"  Slot B: mean={_fmt(summary['b_mean'])} evidence/frame")
        print(f"  Agreement: mean={_fmt(summary['agree_mean'])}%  min={_fmt(summary['agree_min'])}%  max={_fmt(summary['agree_max'])}%")
    else:
        print("  Slot B: not active (no b_count data in profiler log)")

    html = render_html(frames, summary, profiler_path, output_path)
    output_path.write_text(html, encoding="utf-8")
    print(f"OK: report written → {output_path}  ({len(html):,} bytes)")


if __name__ == "__main__":
    main()
