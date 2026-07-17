#!/usr/bin/env python3
"""
depth_provider_report.py — A vs B depth provider comparison report.

Reads per-frame profiler JSONL from a mission output directory and generates
a rich HTML report comparing slot A (ONNX) and slot B (AirSim GT) depth providers.

Profiler stages consumed:
  depth_slot_a.evidence_count   — int, evidence items from slot A
  depth_slot_b.evidence_count   — int, evidence items from slot B (optional)
  depth.voxel_overlap_ppt       — int, A∩B voxel overlap parts-per-thousand (0–1000)
  depth.median_range_a_m        — int, slot A median obstacle range (millimetres)
  depth.median_range_b_m        — int, slot B median obstacle range (millimetres)
  depth.scale_ratio             — int, (range_b / range_a) × 1000  → recommended scale
  depth_slot_a.detect           — int, slot A detect latency (µs)

Usage:
  python3 tools/perception/depth_provider_report.py <mission_output_dir> [-o report.html]

The profiler JSONL is expected at:
  <mission_output_dir>/profiler_log.jsonl   (one JSON object per line per frame)
"""

import argparse
import json
import math
import sys
from pathlib import Path


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

STAGE_KEYS = {
    "a_count":    "depth_slot_a.evidence_count",
    "b_count":    "depth_slot_b.evidence_count",
    "agreement":  "depth.voxel_overlap_ppt",
    "range_a_mm": "depth.median_range_a_m",   # stored as millimetres
    "range_b_mm": "depth.median_range_b_m",   # stored as millimetres
    "scale_ratio_ppt": "depth.scale_ratio",   # (b/a) × 1000
    "a_latency":  "depth_slot_a.detect",
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

        a_count = get("a_count")
        if a_count is None:
            continue

        b_count_raw    = get("b_count")
        agreement_raw  = get("agreement")
        range_a_raw    = get("range_a_mm")
        range_b_raw    = get("range_b_mm")
        scale_ratio_raw = get("scale_ratio_ppt")
        a_latency_raw  = get("a_latency")

        frames.append({
            "frame":       rec.get("frame_id", len(frames)),
            "a_count":     int(a_count),
            "b_count":     int(b_count_raw)     if b_count_raw    is not None else None,
            "agreement":   int(agreement_raw) / 10.0  if agreement_raw   is not None else None,
            "range_a_m":   int(range_a_raw)    / 1000.0 if range_a_raw    is not None else None,
            "range_b_m":   int(range_b_raw)    / 1000.0 if range_b_raw    is not None else None,
            "scale_ratio": int(scale_ratio_raw) / 1000.0 if scale_ratio_raw is not None else None,
            "a_latency":   int(a_latency_raw)  if a_latency_raw  is not None else None,
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
    a_counts    = [f["a_count"]   for f in frames]
    b_counts    = [f["b_count"]   for f in frames if f["b_count"]   is not None]
    agreements  = [f["agreement"] for f in frames if f["agreement"] is not None]
    range_a     = [f["range_a_m"] for f in frames if f["range_a_m"] is not None]
    range_b     = [f["range_b_m"] for f in frames if f["range_b_m"] is not None]
    scale_ratios = [f["scale_ratio"] for f in frames if f["scale_ratio"] is not None]
    latencies   = [f["a_latency"] for f in frames if f["a_latency"] is not None]

    has_b     = len(b_counts) > 0
    has_range = len(range_a) > 0 and len(range_b) > 0

    return {
        "frame_count":    len(frames),
        "has_b":          has_b,
        "has_range":      has_range,
        "a_mean":         _safe_mean(a_counts),
        "a_p50":          _safe_pct(a_counts, 50),
        "a_p95":          _safe_pct(a_counts, 95),
        "b_mean":         _safe_mean(b_counts)   if has_b    else None,
        "b_p50":          _safe_pct(b_counts, 50) if has_b   else None,
        "b_p95":          _safe_pct(b_counts, 95) if has_b   else None,
        "agree_mean":     _safe_mean(agreements),
        "agree_p50":      _safe_pct(agreements, 50),
        "agree_min":      min(agreements)         if agreements else None,
        "agree_max":      max(agreements)         if agreements else None,
        "range_a_mean":   _safe_mean(range_a),
        "range_a_p50":    _safe_pct(range_a, 50),
        "range_b_mean":   _safe_mean(range_b),
        "range_b_p50":    _safe_pct(range_b, 50),
        "scale_ratio_mean": _safe_mean(scale_ratios),
        "scale_ratio_p50":  _safe_pct(scale_ratios, 50),
        "scale_ratio_p10":  _safe_pct(scale_ratios, 10),
        "scale_ratio_p90":  _safe_pct(scale_ratios, 90),
        "lat_mean":       _safe_mean(latencies),
        "lat_p95":        _safe_pct(latencies, 95),
    }


def build_histogram(values, n_bins=20, v_min=None, v_max=None):
    if not values:
        return [], []
    lo = v_min if v_min is not None else min(values)
    hi = v_max if v_max is not None else max(values)
    if hi == lo:
        hi = lo + 1
    width = (hi - lo) / n_bins
    counts = [0] * n_bins
    for v in values:
        idx = min(n_bins - 1, int((v - lo) / width))
        counts[idx] += 1
    labels = [f"{lo + i*width:.2f}–{lo + (i+1)*width:.2f}" for i in range(n_bins)]
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
    --scale-col: #ffa657;
    --warn: #f0883e; --danger: #ff7b72; --good: #3fb950;
  }}
  * {{ box-sizing: border-box; margin: 0; padding: 0; }}
  body {{ background: var(--bg); color: var(--text); font-family: -apple-system, sans-serif; padding: 24px; }}
  h1 {{ font-size: 1.4rem; margin-bottom: 4px; }}
  .subtitle {{ color: var(--muted); font-size: 0.85rem; margin-bottom: 24px; }}
  .section-title {{ font-size: 1rem; color: var(--muted); margin: 28px 0 12px; text-transform: uppercase; letter-spacing: 0.06em; font-size: 0.75rem; }}
  .grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 16px; margin-bottom: 16px; }}
  .card {{ background: var(--card); border: 1px solid var(--border); border-radius: 8px; padding: 16px; }}
  .card .label {{ font-size: 0.75rem; color: var(--muted); margin-bottom: 6px; text-transform: uppercase; letter-spacing: 0.05em; }}
  .card .value {{ font-size: 1.6rem; font-weight: 600; }}
  .card .sub {{ font-size: 0.8rem; color: var(--muted); margin-top: 4px; }}
  .recommend {{ background: var(--card); border: 2px solid var(--scale-col); border-radius: 8px; padding: 18px 20px; margin-bottom: 24px; }}
  .recommend .label {{ font-size: 0.75rem; color: var(--scale-col); text-transform: uppercase; letter-spacing: 0.05em; margin-bottom: 8px; }}
  .recommend .cmd {{ font-family: monospace; font-size: 1rem; color: var(--text); background: rgba(255,166,87,0.08); padding: 8px 12px; border-radius: 4px; display: inline-block; margin-top: 6px; }}
  .recommend .note {{ font-size: 0.8rem; color: var(--muted); margin-top: 8px; }}
  .chart-wrap {{ background: var(--card); border: 1px solid var(--border); border-radius: 8px; padding: 16px; margin-bottom: 24px; }}
  .chart-wrap h2 {{ font-size: 1rem; margin-bottom: 14px; }}
  canvas {{ width: 100% !important; }}
  table {{ width: 100%; border-collapse: collapse; font-size: 0.82rem; }}
  th, td {{ text-align: right; padding: 6px 10px; border-bottom: 1px solid var(--border); }}
  th {{ color: var(--muted); font-weight: 400; }}
  td:first-child, th:first-child {{ text-align: left; }}
  tr:hover td {{ background: rgba(255,255,255,0.03); }}
  .agree-good {{ color: var(--good); }}
  .agree-warn {{ color: var(--warn); }}
  .agree-bad  {{ color: var(--danger); }}
  .scale-ok   {{ color: var(--good); }}
  .scale-off  {{ color: var(--warn); }}
  .scale-bad  {{ color: var(--danger); }}
</style>
</head>
<body>
<h1>Depth Provider Report</h1>
<div class="subtitle">{subtitle}</div>

<div class="section-title">Scale calibration</div>
{recommend_block}

<div class="section-title">Evidence counts</div>
<div class="grid">
  <div class="card">
    <div class="label">Frames analysed</div>
    <div class="value">{frame_count}</div>
  </div>
  <div class="card">
    <div class="label">Slot A (ONNX) mean evidence</div>
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

<div class="section-title">Median range A vs B</div>
<div class="grid">
  <div class="card">
    <div class="label">ONNX median range</div>
    <div class="value" style="color:var(--a-col)">{range_a_mean} m</div>
    <div class="sub">p50={range_a_p50} m</div>
  </div>
  <div class="card">
    <div class="label">GT median range</div>
    <div class="value" style="color:var(--b-col)">{range_b_mean} m</div>
    <div class="sub">p50={range_b_p50} m</div>
  </div>
  <div class="card">
    <div class="label">Scale ratio (GT/ONNX)</div>
    <div class="value" style="color:var(--scale-col)">{scale_ratio_mean}</div>
    <div class="sub">p10={scale_ratio_p10} &nbsp; p50={scale_ratio_p50} &nbsp; p90={scale_ratio_p90}</div>
  </div>
</div>

<!-- Evidence count time-series -->
<div class="chart-wrap">
  <h2>Evidence count per frame</h2>
  <canvas id="tsChart" height="80"></canvas>
</div>

<!-- Range time-series -->
{range_ts_section}

<!-- Scale ratio time-series -->
{scale_ts_section}

<!-- Agreement time-series -->
{agree_ts_section}

<!-- Scale ratio histogram -->
{scale_hist_section}

<!-- Per-frame data table -->
<div class="chart-wrap">
  <h2>Per-frame data (last 500 frames)</h2>
  <table>
    <thead><tr>
      <th>Frame</th>
      <th>A evidence</th>
      {b_th}
      {range_th}
      {scale_th}
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
const labels   = FRAMES.map(f => f.frame);
const aData    = FRAMES.map(f => f.a_count);
const bData    = FRAMES.map(f => f.b_count);
const agData   = FRAMES.map(f => f.agreement);
const raData   = FRAMES.map(f => f.range_a_m);
const rbData   = FRAMES.map(f => f.range_b_m);
const srData   = FRAMES.map(f => f.scale_ratio);
const hasB     = bData.some(v => v !== null);
const hasAg    = agData.some(v => v !== null);
const hasRange = raData.some(v => v !== null);
const hasScale = srData.some(v => v !== null);

Chart.defaults.color = '#8b949e';
Chart.defaults.borderColor = '#30363d';

// Evidence time-series
(function() {{
  const datasets = [{{
    label: 'Slot A (ONNX)',
    data: aData,
    borderColor: '#58a6ff',
    backgroundColor: 'rgba(88,166,255,0.08)',
    borderWidth: 1.5, pointRadius: 0, fill: true, tension: 0.2,
  }}];
  if (hasB) datasets.push({{
    label: 'Slot B (GT)',
    data: bData,
    borderColor: '#3fb950',
    backgroundColor: 'rgba(63,185,80,0.06)',
    borderWidth: 1.5, pointRadius: 0, fill: true, tension: 0.2,
  }});
  new Chart(document.getElementById('tsChart'), {{
    type: 'line',
    data: {{ labels, datasets }},
    options: {{ animation: false, responsive: true, plugins: {{ legend: {{ position: 'top' }} }} }},
  }});
}})();

// Range time-series
if (hasRange && document.getElementById('rangeChart')) {{
  new Chart(document.getElementById('rangeChart'), {{
    type: 'line',
    data: {{
      labels,
      datasets: [
        {{ label: 'ONNX median range (m)', data: raData, borderColor: '#58a6ff', borderWidth: 1.5, pointRadius: 0, fill: false, tension: 0.2 }},
        {{ label: 'GT median range (m)',   data: rbData, borderColor: '#3fb950', borderWidth: 1.5, pointRadius: 0, fill: false, tension: 0.2 }},
      ],
    }},
    options: {{
      animation: false, responsive: true,
      scales: {{ y: {{ title: {{ display: true, text: 'metres' }} }} }},
      plugins: {{ legend: {{ position: 'top' }} }},
    }},
  }});
}}

// Scale ratio time-series
if (hasScale && document.getElementById('scaleChart')) {{
  new Chart(document.getElementById('scaleChart'), {{
    type: 'line',
    data: {{
      labels,
      datasets: [{{
        label: 'Scale ratio GT/ONNX',
        data: srData,
        borderColor: '#ffa657',
        backgroundColor: 'rgba(255,166,87,0.08)',
        borderWidth: 1.5, pointRadius: 0, fill: true, tension: 0.2,
      }}],
    }},
    options: {{
      animation: false, responsive: true,
      scales: {{ y: {{ title: {{ display: true, text: 'ratio (1.0 = calibrated)' }} }} }},
      plugins: {{
        legend: {{ position: 'top' }},
        annotation: {{ annotations: {{ line1: {{ type: 'line', yMin: 1, yMax: 1, borderColor: 'rgba(255,255,255,0.2)', borderDash: [4,4] }} }} }},
      }},
    }},
  }});
}}

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
        borderWidth: 1.5, pointRadius: 0, fill: true, tension: 0.2,
      }}],
    }},
    options: {{
      animation: false, responsive: true,
      scales: {{ y: {{ min: 0, max: 100 }} }},
      plugins: {{ legend: {{ position: 'top' }} }},
    }},
  }});
}}

// Scale ratio histogram
if (document.getElementById('scaleHistChart')) {{
  new Chart(document.getElementById('scaleHistChart'), {{
    type: 'bar',
    data: {{
      labels: {scale_hist_labels_json},
      datasets: [{{
        label: 'Frame count',
        data: {scale_hist_counts_json},
        backgroundColor: 'rgba(255,166,87,0.7)',
        borderColor: '#ffa657',
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


def _fmt(v, fmt=".2f", fallback="—"):
    if v is None:
        return fallback
    return format(v, fmt)


def _agree_class(v):
    if v is None: return ""
    if v >= 70:   return "agree-good"
    if v >= 40:   return "agree-warn"
    return "agree-bad"


def _scale_class(v):
    if v is None: return ""
    if 0.9 <= v <= 1.1: return "scale-ok"
    if 0.7 <= v <= 1.4: return "scale-off"
    return "scale-bad"


def render_html(frames: list[dict], summary: dict, source_path: Path, output_path: Path) -> str:
    title    = source_path.parent.name
    subtitle = f"Mission: {source_path.parent.resolve()} &nbsp;|&nbsp; Frames: {summary['frame_count']}"

    has_b     = summary["has_b"]
    has_range = summary["has_range"]

    # Scale calibration recommendation
    sr = summary.get("scale_ratio_p50")
    if sr is not None and has_range:
        sr_cls = _scale_class(sr)
        if 0.95 <= sr <= 1.05:
            advice = "Scale is well-calibrated — no change needed."
        else:
            advice = (
                f"If using <b>visual_onnx</b>: set <code>visual_onnx.scale: {sr:.3f}</code> "
                f"in config/pipeline/visual.yaml to align with GT.<br>"
                f"If using <b>unidepth_v2</b>: metric provider — scale calibration is not applicable."
            )
        recommend_block = f"""
<div class="recommend">
  <div class="label">Scale calibration</div>
  <div class="cmd {sr_cls}">visual_onnx.scale: {sr:.3f} &nbsp;(visual_onnx only)</div>
  <div class="note">{advice}<br>
  p50 scale ratio = {sr:.3f} &nbsp;|&nbsp; p10={_fmt(summary['scale_ratio_p10'])} &nbsp;p90={_fmt(summary['scale_ratio_p90'])}<br>
  Primary median range = {_fmt(summary['range_a_mean'])} m &nbsp;|&nbsp; GT median range = {_fmt(summary['range_b_mean'])} m</div>
</div>"""
    else:
        recommend_block = """
<div class="recommend">
  <div class="label">Scale calibration</div>
  <div class="note">No slot B (GT) range data found. Run with <code>depth_eval: airsim_gt_detector</code>
  in visual.yaml (or set <code>DEDALUS_DEPTH_EVAL=airsim_gt_detector</code>) to enable A/B comparison.</div>
</div>"""

    # Cards
    b_card = ""
    agree_card = ""
    if has_b:
        b_card = f"""
  <div class="card">
    <div class="label">Slot B (GT) mean evidence</div>
    <div class="value" style="color:var(--b-col)">{_fmt(summary['b_mean'])}</div>
    <div class="sub">p50={_fmt(summary['b_p50'],'g')} &nbsp; p95={_fmt(summary['b_p95'],'g')}</div>
  </div>"""
        agree_card = f"""
  <div class="card">
    <div class="label">Mean voxel agreement</div>
    <div class="value {_agree_class(summary['agree_mean'])}">{_fmt(summary['agree_mean'])}%</div>
    <div class="sub">min={_fmt(summary['agree_min'])}% &nbsp; max={_fmt(summary['agree_max'])}%</div>
  </div>"""

    # Range section
    range_ts_section = ""
    if has_range:
        range_ts_section = """
<div class="chart-wrap">
  <h2>Median obstacle range per frame — ONNX vs GT</h2>
  <canvas id="rangeChart" height="70"></canvas>
</div>"""

    scale_ts_section = ""
    scale_hist_section = ""
    scale_ratios = [f["scale_ratio"] for f in frames if f["scale_ratio"] is not None]
    if scale_ratios:
        scale_ts_section = """
<div class="chart-wrap">
  <h2>Scale ratio (GT / ONNX) per frame &mdash; target: 1.0</h2>
  <canvas id="scaleChart" height="70"></canvas>
</div>"""
        sh_labels, sh_counts = build_histogram(scale_ratios, n_bins=20)
        scale_hist_section = """
<div class="chart-wrap">
  <h2>Scale ratio distribution</h2>
  <canvas id="scaleHistChart" height="60"></canvas>
</div>"""
    else:
        sh_labels, sh_counts = [], []

    # Agreement time-series
    agreements = [f["agreement"] for f in frames if f["agreement"] is not None]
    agree_ts_section = ""
    if agreements:
        agree_ts_section = """
<div class="chart-wrap">
  <h2>Voxel agreement % per frame</h2>
  <canvas id="agChart" height="60"></canvas>
</div>"""

    # Table headers
    b_th     = "<th>B evidence</th>" if has_b else ""
    range_th = "<th>A range (m)</th><th>B range (m)</th>" if has_range else ""
    scale_th = "<th>Scale ratio</th>" if scale_ratios else ""
    agree_th = "<th>Agreement %</th>" if has_b else ""

    # Table rows (last 500)
    rows = []
    for f in frames[-500:]:
        ag_cls = _agree_class(f["agreement"])
        sr_cls = _scale_class(f["scale_ratio"])
        b_td     = f"<td>{f['b_count'] if f['b_count'] is not None else '—'}</td>" if has_b else ""
        range_td = (f"<td>{_fmt(f['range_a_m'])}</td><td>{_fmt(f['range_b_m'])}</td>"
                    if has_range else "")
        scale_td = (f"<td class='{sr_cls}'>{_fmt(f['scale_ratio'])}</td>"
                    if scale_ratios else "")
        agree_td = (f"<td class='{ag_cls}'>{_fmt(f['agreement'])}%</td>"
                    if has_b else "")
        rows.append(
            f"<tr>"
            f"<td>{f['frame']}</td><td>{f['a_count']}</td>"
            f"{b_td}{range_td}{scale_td}{agree_td}"
            f"<td>{f['a_latency'] if f['a_latency'] is not None else '—'}</td>"
            f"</tr>"
        )

    # Chart JSON (last 2000 frames)
    chart_frames = frames[-2000:]
    frames_json = json.dumps([{
        "frame": f["frame"], "a_count": f["a_count"], "b_count": f["b_count"],
        "agreement": f["agreement"],
        "range_a_m": f["range_a_m"], "range_b_m": f["range_b_m"],
        "scale_ratio": f["scale_ratio"],
    } for f in chart_frames])

    return _HTML_TEMPLATE.format(
        title=title,
        subtitle=subtitle,
        frame_count=summary["frame_count"],
        recommend_block=recommend_block,
        a_mean=_fmt(summary["a_mean"]),
        a_p50=_fmt(summary["a_p50"], "g"),
        a_p95=_fmt(summary["a_p95"], "g"),
        b_card=b_card,
        agree_card=agree_card,
        lat_mean=_fmt(summary["lat_mean"], ".0f"),
        lat_p95=_fmt(summary["lat_p95"], ".0f"),
        range_a_mean=_fmt(summary["range_a_mean"]),
        range_a_p50=_fmt(summary["range_a_p50"]),
        range_b_mean=_fmt(summary["range_b_mean"]),
        range_b_p50=_fmt(summary["range_b_p50"]),
        scale_ratio_mean=_fmt(summary["scale_ratio_mean"]),
        scale_ratio_p50=_fmt(summary["scale_ratio_p50"]),
        scale_ratio_p10=_fmt(summary["scale_ratio_p10"]),
        scale_ratio_p90=_fmt(summary["scale_ratio_p90"]),
        range_ts_section=range_ts_section,
        scale_ts_section=scale_ts_section,
        agree_ts_section=agree_ts_section,
        scale_hist_section=scale_hist_section,
        b_th=b_th,
        range_th=range_th,
        scale_th=scale_th,
        agree_th=agree_th,
        table_rows="\n      ".join(rows) if rows else "<tr><td colspan='8'>No data</td></tr>",
        frames_json=frames_json,
        scale_hist_labels_json=json.dumps(sh_labels),
        scale_hist_counts_json=json.dumps(sh_counts),
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _resolve_profiler_path(arg: Path) -> Path:
    """Accept a direct .jsonl path or a directory.

    Directory resolution order:
      1. profiler_log.jsonl   (legacy name)
      2. pipeline_*.jsonl     (current name) — picks the most recent by mtime
    """
    if arg.is_file():
        return arg
    if not arg.is_dir():
        print(f"ERROR: path not found: {arg}", file=sys.stderr)
        sys.exit(1)
    legacy = arg / "profiler_log.jsonl"
    if legacy.exists():
        return legacy
    candidates = sorted(arg.glob("pipeline_*.jsonl"), key=lambda p: p.stat().st_mtime)
    if candidates:
        return candidates[-1]   # most recent
    print(f"ERROR: no pipeline_*.jsonl or profiler_log.jsonl found in {arg}", file=sys.stderr)
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mission_dir",
                        help="Mission output directory (or direct path to a pipeline_*.jsonl file)")
    parser.add_argument("-o", "--output", default=None,
                        help="Output HTML path (default: <dir>/depth_provider_report.html)")
    args = parser.parse_args()

    arg_path      = Path(args.mission_dir)
    profiler_path = _resolve_profiler_path(arg_path)
    mission_dir   = profiler_path.parent

    output_path = Path(args.output) if args.output else (mission_dir / "depth_provider_report.html")

    print(f"Loading {profiler_path} ...")
    records = load_profiler_jsonl(profiler_path)
    print(f"  {len(records)} log records")

    frames = extract_frames(records)
    if not frames:
        print("ERROR: no depth_slot_a frames found in profiler log.", file=sys.stderr)
        sys.exit(1)

    print(f"  {len(frames)} frames with depth slot data")

    summary = compute_summary(frames)
    has_b   = summary["has_b"]

    print(f"  Slot A (ONNX): mean={_fmt(summary['a_mean'])} ev/frame  p95={_fmt(summary['a_p95'],'g')}")
    if has_b:
        print(f"  Slot B (GT):   mean={_fmt(summary['b_mean'])} ev/frame")
        print(f"  Voxel agreement: mean={_fmt(summary['agree_mean'])}%  min={_fmt(summary['agree_min'])}%  max={_fmt(summary['agree_max'])}%")

    if summary["has_range"]:
        print(f"  ONNX median range: {_fmt(summary['range_a_mean'])} m  (p50={_fmt(summary['range_a_p50'])} m)")
        print(f"  GT   median range: {_fmt(summary['range_b_mean'])} m  (p50={_fmt(summary['range_b_p50'])} m)")
        sr = summary["scale_ratio_p50"]
        if sr is not None:
            print(f"  Scale ratio p50:   {sr:.3f}")
            if not (0.95 <= sr <= 1.05):
                print(f"    visual_onnx provider:  set visual_onnx.scale: {sr:.3f}")
                print(f"    unidepth_v2 provider:  metric — scale calibration not applicable")
    else:
        print("  No A/B range data — run with DEDALUS_DEPTH_EVAL=airsim_gt_detector to enable")

    html = render_html(frames, summary, profiler_path, output_path)
    output_path.write_text(html, encoding="utf-8")
    print(f"OK: report written → {output_path}  ({len(html):,} bytes)")


if __name__ == "__main__":
    main()
