#!/usr/bin/env python3
"""Experimental probe for the depth ONNX model output encoding.

Runs the model on synthetic test images AND (optionally) real PNG frames,
prints per-image depth statistics, and writes a histogram PNG so we can
verify that the output encoding (inverse-depth 1/m, high=close) is as
expected and understand what the model does with AirSim renders.

Usage:
    # Synthetic inputs only (no GPU needed)
    python3 tools/perception/probe_depth_frames.py \
        --model models/depth_anything_v2_metric_vits.onnx

    # Also probe real frames captured from AirSim
    python3 tools/perception/probe_depth_frames.py \
        --model models/depth_anything_v2_metric_vits.onnx \
        --frames-dir /tmp/airsim_frames \
        --out-png /tmp/depth_probe.png

The script replicates the C++ preprocessing exactly (nearest-neighbour
resize → ImageNet normalise → NCHW float32 tensor).

Correct model output encoding (from export_depth_anything.py contract):
  Metric model:   raw = absolute depth in METRES, HIGH = FAR
  Relative model: raw = disparity score, HIGH = CLOSE (normalised 0..1)

For the metric model (the default, depth_anything_v2_metric_vits.onnx):
  raw_min ≈ 0.3 m   (props / very close objects)
  raw_max ≈ 60+ m   (open sky or terrain at distance)
  raw_mean ≈ 5-20 m (typical drone-level outdoor scene)

The C++ engine stores depth_relative = 1/raw so the downstream formula
  depth_m = scale / depth_relative = raw
recovers physical metres with scale=1.0.

If raw_mean < 1.0: the model is predicting most of the scene as sub-1 m,
likely due to AirSim out-of-distribution input or attention bleed from props.
→ check DEDALUS_DEPTH_DEBUG_DIR .npy files for actual frame values.
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

# ── constants matching C++ ONNXDepthEngineConfig defaults ──────────────────────
MODEL_INPUT_SIZE = 518
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)


# ── preprocessing (mirrors resize_and_normalise in onnx_depth_engine.cpp) ─────

def preprocess_rgb(img_rgb: np.ndarray, size: int = MODEL_INPUT_SIZE) -> np.ndarray:
    """Nearest-neighbour resize + ImageNet normalise → NCHW float32 [1,3,H,W]."""
    H, W = img_rgb.shape[:2]
    xs = (np.arange(size) * W / size).astype(np.int32).clip(0, W - 1)
    ys = (np.arange(size) * H / size).astype(np.int32).clip(0, H - 1)
    resized = img_rgb[np.ix_(ys, xs)].astype(np.float32) / 255.0  # (size, size, 3)
    normed  = (resized - IMAGENET_MEAN) / IMAGENET_STD             # (size, size, 3)
    return normed.transpose(2, 0, 1)[np.newaxis]                   # (1, 3, size, size)


# ── synthetic test images ─────────────────────────────────────────────────────

def make_synthetic_images() -> list[tuple[str, np.ndarray]]:
    S = 640
    images: list[tuple[str, np.ndarray]] = []

    # All black
    images.append(("all_black",    np.zeros((S, S, 3), dtype=np.uint8)))
    # All white
    images.append(("all_white",    np.full((S, S, 3), 255, dtype=np.uint8)))
    # Horizontal gradient (left=black, right=white) — tests luminance encoding
    grad_h = np.tile(np.arange(S, dtype=np.uint8)[:, np.newaxis], (1, 3))[np.newaxis]
    grad_h = np.broadcast_to(
        np.arange(S, dtype=np.uint8)[np.newaxis, :, np.newaxis], (S, S, 3)).copy()
    images.append(("gradient_h",   grad_h))
    # Vertical gradient (top=black, bottom=white)
    grad_v = np.broadcast_to(
        np.arange(S, dtype=np.uint8)[:, np.newaxis, np.newaxis], (S, S, 3)).copy()
    images.append(("gradient_v",   grad_v))
    # Checkerboard 32-px cells
    board = np.zeros((S, S, 3), dtype=np.uint8)
    for row in range(0, S, 32):
        for col in range(0, S, 32):
            if (row // 32 + col // 32) % 2 == 0:
                board[row:row+32, col:col+32] = 255
    images.append(("checkerboard", board))
    # Sky (upper half blue, lower half brown) — crude outdoor scene proxy
    outdoor = np.zeros((S, S, 3), dtype=np.uint8)
    outdoor[:S//2]  = [135, 206, 235]   # sky blue
    outdoor[S//2:]  = [139,  90,  43]   # earth brown
    images.append(("sky_ground",   outdoor))

    return images


# ── statistics helper ─────────────────────────────────────────────────────────

def raw_stats(raw: np.ndarray) -> dict[str, float]:
    """Summary stats for a 2-D raw model output array.
    Metric model: raw = inverse depth in 1/m (high=close).
    depth_m = 1/raw recovers physical metres.
    """
    depth_m = np.where(raw > 1e-6, 1.0 / raw.astype(np.float64), np.inf)
    bins = [0.5, 1.0, 5.0, 20.0, 60.0]
    pcts = []
    prev = 0.0
    total = depth_m.size
    for b in bins:
        count = int(np.sum((depth_m >= prev) & (depth_m < b)))
        pcts.append(100.0 * count / total)
        prev = b
    pcts.append(100.0 * int(np.sum(depth_m >= 60.0)) / total)

    return {
        "raw_min":   float(raw.min()),
        "raw_max":   float(raw.max()),
        "raw_mean":  float(raw.mean()),
        "raw_p25":   float(np.percentile(raw, 25)),
        "raw_p50":   float(np.percentile(raw, 50)),
        "raw_p75":   float(np.percentile(raw, 75)),
        "dm_min":    float(np.where(np.isfinite(depth_m), depth_m, np.nan).min()),
        "dm_max":    float(np.where(depth_m < 1e6, depth_m, np.nan).max()),
        "dm_mean":   float(np.nanmean(np.where(depth_m < 1e6, depth_m, np.nan))),
        "pct_lt0.5": pcts[0],
        "pct_0.5-1": pcts[1],
        "pct_1-5":   pcts[2],
        "pct_5-20":  pcts[3],
        "pct_20-60": pcts[4],
        "pct_gt60":  pcts[5],
    }


def print_stats(label: str, s: dict[str, float]) -> None:
    print(f"\n{'─'*60}")
    print(f"  {label}")
    print(f"{'─'*60}")
    print(f"  raw   : min={s['raw_min']:.4f}  max={s['raw_max']:.4f}  "
          f"mean={s['raw_mean']:.4f}  p25={s['raw_p25']:.3f}  "
          f"p50={s['raw_p50']:.3f}  p75={s['raw_p75']:.3f}")
    print(f"  depth_m: min={s['dm_min']:.2f}m  max={s['dm_max']:.1f}m  "
          f"mean={s['dm_mean']:.2f}m")
    print(f"  depth_m bins:")
    print(f"    <0.5m  : {s['pct_lt0.5']:5.1f}%  [props / immediate clutter]")
    print(f"    0.5-1m : {s['pct_0.5-1']:5.1f}%  [below min_depth_m=1.0 → magenta]")
    print(f"    1-5m   : {s['pct_1-5']:5.1f}%  [valid close range]")
    print(f"    5-20m  : {s['pct_5-20']:5.1f}%  [valid mid range]")
    print(f"    20-60m : {s['pct_20-60']:5.1f}%  [valid far range]")
    print(f"    >60m   : {s['pct_gt60']:5.1f}%  [sky / max clamp]")
    magenta_pct = s['pct_lt0.5'] + s['pct_0.5-1']
    print(f"  → {magenta_pct:.0f}% of pixels filtered as magenta "
          f"(depth_m < min_depth_m=1.0m)")


# ── main ──────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--model", default="models/depth_anything_v2_metric_vits.onnx",
                   help="Path to .onnx model file")
    p.add_argument("--frames-dir", default="",
                   help="Directory of .png or .jpg AirSim frames to probe")
    p.add_argument("--max-frames", type=int, default=10,
                   help="Maximum real frames to probe (default 10)")
    p.add_argument("--out-png", default="",
                   help="Write histogram PNG to this path (requires matplotlib)")
    p.add_argument("--cuda", action="store_true",
                   help="Use CUDA execution provider")
    p.add_argument("--input-name", default="image")
    p.add_argument("--output-name", default="depth")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    try:
        import onnxruntime as ort
    except ImportError:
        print("ERROR: onnxruntime not installed. Run: pip install onnxruntime", file=sys.stderr)
        return 1

    model_path = args.model
    if not Path(model_path).exists():
        print(f"ERROR: model not found: {model_path}", file=sys.stderr)
        return 1

    providers = ["CUDAExecutionProvider", "CPUExecutionProvider"] if args.cuda \
                else ["CPUExecutionProvider"]
    print(f"Loading model: {model_path}")
    sess = ort.InferenceSession(model_path, providers=providers)
    actual_providers = sess.get_providers()
    print(f"Execution providers: {actual_providers}")

    # Discover actual input/output names from the model
    in_names  = [i.name for i in sess.get_inputs()]
    out_names = [o.name for o in sess.get_outputs()]
    print(f"Model inputs:  {in_names}")
    print(f"Model outputs: {out_names}")
    input_name  = in_names[0]
    output_name = out_names[0]

    def infer(img_rgb: np.ndarray) -> np.ndarray:
        tensor = preprocess_rgb(img_rgb)
        raw_out = sess.run([output_name], {input_name: tensor})[0]
        # Output shape: [1,H,W] or [1,1,H,W] — squeeze to 2-D
        return raw_out.squeeze()

    all_stats: list[tuple[str, dict[str, float]]] = []

    # ── synthetic ──
    print("\n=== Synthetic test images ===")
    for name, img in make_synthetic_images():
        raw = infer(img)
        s = raw_stats(raw)
        print_stats(f"synthetic/{name}", s)
        all_stats.append((f"syn/{name}", s))

    # ── real frames ──
    if args.frames_dir:
        frames_dir = Path(args.frames_dir)
        frames = sorted(
            list(frames_dir.glob("*.png")) + list(frames_dir.glob("*.jpg"))
        )[:args.max_frames]
        if not frames:
            print(f"\nNo PNG/JPG frames found in {frames_dir}", file=sys.stderr)
        else:
            print(f"\n=== Real frames ({len(frames)} files from {frames_dir}) ===")
            for fpath in frames:
                try:
                    # Load with PIL (no OpenCV dependency)
                    from PIL import Image
                    img_rgb = np.array(Image.open(fpath).convert("RGB"))
                except ImportError:
                    print("PIL not available; skipping real frames. "
                          "pip install pillow", file=sys.stderr)
                    break
                except Exception as e:
                    print(f"  SKIP {fpath.name}: {e}", file=sys.stderr)
                    continue
                raw = infer(img_rgb)
                s = raw_stats(raw)
                print_stats(fpath.name, s)
                all_stats.append((fpath.name, s))

    # ── summary table ──
    print(f"\n{'═'*80}")
    print("SUMMARY TABLE")
    print(f"{'Label':<30} {'raw_mean':>8} {'dm_mean':>8} "
          f"{'%<1m':>6} {'%1-5m':>6} {'%5-60m':>7}")
    print(f"{'─'*30} {'─'*8} {'─'*8} {'─'*6} {'─'*6} {'─'*7}")
    for label, s in all_stats:
        pct_lt1   = s['pct_lt0.5'] + s['pct_0.5-1']
        pct_1_5   = s['pct_1-5']
        pct_5_60  = s['pct_5-20'] + s['pct_20-60']
        print(f"{label:<30} {s['raw_mean']:8.3f} {s['dm_mean']:8.2f}m "
              f"{pct_lt1:6.0f}% {pct_1_5:6.0f}% {pct_5_60:7.0f}%")

    print(f"\nInterpretation (metric model: raw = inverse depth 1/m, high=CLOSE):")
    print(f"  depth_m = 1/raw.  raw > 1.0 → depth_m < 1.0m → MAGENTA (too-close filter)")
    print(f"  raw 0.017-1.0 → depth_m 1-60m → valid range → white→black in right panel")
    print(f"  raw < 0.017 → depth_m > 60m → dark navy (too-far filter)")
    print(f"  Healthy scene: props raw≈2-7 (depth 0.1-0.5m), scene raw≈0.05-0.2 (5-20m)")
    print(f"  If %<1m is large: ViT attention bleed from props → lower min_depth_m in visual.yaml")

    # ── optional histogram PNG ──
    if args.out_png and all_stats:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            # Re-run to collect raw arrays for histogram
            raw_arrays: dict[str, np.ndarray] = {}
            print(f"\nCollecting raw arrays for histogram PNG …")
            for name, img in make_synthetic_images():
                raw_arrays[f"syn/{name}"] = infer(img)
            if args.frames_dir:
                frames_dir = Path(args.frames_dir)
                for fpath in sorted(
                    list(frames_dir.glob("*.png")) + list(frames_dir.glob("*.jpg"))
                )[:min(args.max_frames, 4)]:
                    try:
                        from PIL import Image
                        img_rgb = np.array(Image.open(fpath).convert("RGB"))
                        raw_arrays[fpath.name[:20]] = infer(img_rgb)
                    except Exception:
                        pass

            n = len(raw_arrays)
            fig, axes = plt.subplots(2, (n + 1) // 2, figsize=(4 * ((n + 1) // 2), 8))
            axes = np.array(axes).flatten()
            for ax, (lbl, raw) in zip(axes, raw_arrays.items()):
                # Metric model: raw = inverse depth 1/m (high=close), depth_m = 1/raw.
                dm = np.where(raw > 1e-6, 1.0 / raw, 80.0).flatten().clip(0, 80).astype(np.float32)
                ax.hist(dm, bins=60, range=(0, 30), color="steelblue", edgecolor="none")
                ax.axvline(1.0,  color="magenta", linewidth=1.5, label="min_depth_m=1.0")
                ax.axvline(60.0, color="navy",    linewidth=1.5, label="max_depth_m=60")
                ax.set_title(lbl, fontsize=8)
                ax.set_xlabel("depth_m (m)")
                ax.set_ylabel("pixel count")
                ax.legend(fontsize=6)
            for ax in axes[len(raw_arrays):]:
                ax.set_visible(False)
            fig.suptitle("depth_m distribution per input image  (metric model: raw = metres)\n"
                         "Magenta line = min_depth_m filter — pixels left of it → magenta in debug MP4",
                         fontsize=9)
            fig.tight_layout()
            fig.savefig(args.out_png, dpi=120)
            print(f"Histogram saved → {args.out_png}")
        except ImportError as e:
            print(f"matplotlib/PIL not available, skipping PNG: {e}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
