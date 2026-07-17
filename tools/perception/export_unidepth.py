#!/usr/bin/env python3
"""Export UniDepth V2 (metric, any backbone) to ONNX from HuggingFace weights.

The exported model is NOT committed to the repo.
Run this once on each target device to generate the .onnx file.

SETUP
-----
UniDepth V2 must be installed from source before running this script:

    git clone https://github.com/lpiccinelli-eth/UniDepth.git
    cd UniDepth && pip install -e .
    pip install onnxruntime          # for sanity check
    pip install onnxruntime-gpu      # if CUDA EP is available

USAGE
-----
Default (ViT-S, 256×144 camera, single-input):
    python3 tools/perception/export_unidepth.py \\
        --output models/unidepth_v2_vits_336x602.onnx

With camera-ray second input (slightly more accurate, requires C++ ray precomputation):
    python3 tools/perception/export_unidepth.py \\
        --output models/unidepth_v2_vits_336x602_cam.onnx \\
        --with-camera-rays

Override inference shape (must be multiples of 14, ≥200K pixels total):
    python3 tools/perception/export_unidepth.py \\
        --output models/unidepth_v2_vits_448x798.onnx \\
        --inference-height 448 --inference-width 798

EXPORTED ONNX CONTRACT
-----------------------
Single-input (default):
    Input  "rgbs"        float32  [1, 3, H_inf, W_inf]  ImageNet-normalised RGB
    Output "pts_3d"      float32  [1, 3, H_inf, W_inf]  Metric XYZ in camera frame
           "confidence"  float32  [1, 1, H_inf, W_inf]  Log-scale uncertainty
           "intrinsics"  float32  [1, 3, 3]              Predicted K matrix

Two-input (--with-camera-rays):
    Input  "rgbs"        float32  [1, 3, H_inf, W_inf]
           "rays"        float32  [1, 3, H_inf, W_inf]  Per-pixel unit ray vectors
    (outputs identical)

Shape is baked at export time — cannot change at runtime.
Opset 14 minimum (ONNX Runtime ≥ 1.11 required).

DEPTH EXTRACTION
-----------------
    depth_m = pts_3d[0, 2, :, :]   # Z channel, metric metres, same H_inf × W_inf
    # Provider downsamples to native camera resolution (e.g. 256×144) before projection.

INFERENCE SHAPE SELECTION
--------------------------
UniDepth V2 ViT-S was trained with images between 200K and 600K pixels.
Our camera is 256×144 = 36 864 px (5.4× below the training floor).
The script auto-computes the smallest inference shape that:
  - Preserves the native camera aspect ratio (256/144 ≈ 1.778)
  - Has at least 200 000 pixels (training distribution minimum)
  - Has both H and W as multiples of 14 (ViT patch size)
Result for our camera: 336 × 602 = 202 272 pixels.

YAML CONFIG SNIPPET
--------------------
The script prints the YAML keys to add to your mission config.
For an A/B eval with the AirSim GT oracle in slot B:

    depth:      unidepth_v2
    depth_eval: airsim_gt_vd
    unidepth.model_path:       models/unidepth_v2_vits_336x602.onnx
    unidepth.inference_height: 336
    unidepth.inference_width:  602
    unidepth.use_camera_rays:  false
    unidepth.use_cuda:         true

Or set DEDALUS_DEPTH=unidepth_v2 at runtime to override without editing YAML.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from pathlib import Path

# ── Constants ────────────────────────────────────────────────────────────────

# ViT patch size — H and W must be exact multiples.
_PATCH_SIZE = 14

# UniDepth V2 training pixel count bounds (from training config).
_MIN_PIXELS = 200_000
_MAX_PIXELS = 600_000

# HuggingFace repo IDs.
_HF_REPOS = {
    "vits": "lpiccinelli/unidepth-v2-vits14",
    "vitb": "lpiccinelli/unidepth-v2-vitb14",
    "vitl": "lpiccinelli/unidepth-v2-vitl14",
}

# Our production camera intrinsics (256×144, 84° hFoV, 53.72° vFoV).
_NATIVE_FX = 141.6
_NATIVE_FY = 141.2
_NATIVE_CX = 128.0
_NATIVE_CY = 72.0


# ── Shape helpers ─────────────────────────────────────────────────────────────

def snap_to_patch(n: int, patch: int = _PATCH_SIZE) -> int:
    """Round n up to the nearest multiple of patch."""
    return math.ceil(n / patch) * patch


def compute_inference_shape(
    native_h: int,
    native_w: int,
    min_pixels: int = _MIN_PIXELS,
    patch: int = _PATCH_SIZE,
) -> tuple[int, int]:
    """Return (inf_h, inf_w): smallest shape at the native aspect ratio
    with ≥ min_pixels pixels and both dimensions multiples of patch.
    """
    aspect = native_w / native_h
    # Minimum H to hit the pixel floor: H × (H × aspect) ≥ min_pixels
    h_exact = math.sqrt(min_pixels / aspect)
    inf_h   = snap_to_patch(math.ceil(h_exact), patch)
    inf_w   = snap_to_patch(math.ceil(inf_h * aspect), patch)
    # Guard: rounding may have just clipped below the minimum.
    if inf_h * inf_w < min_pixels:
        inf_h += patch
        inf_w = snap_to_patch(math.ceil(inf_h * aspect), patch)
    return inf_h, inf_w


# ── Export ────────────────────────────────────────────────────────────────────

def find_unidepth_export_script() -> Path | None:
    """Return the path to UniDepth's own export.py if importable, else None."""
    try:
        import unidepth  # type: ignore
    except ImportError:
        return None
    candidate = Path(unidepth.__file__).parent / "models" / "unidepthv2" / "export.py"
    return candidate if candidate.is_file() else None


def run_unidepth_export(
    export_py: Path,
    backbone: str,
    inf_h: int,
    inf_w: int,
    output: Path,
    with_camera_rays: bool,
    opset: int,
) -> None:
    """Invoke UniDepth's own export.py as a subprocess."""
    cmd = [
        sys.executable,
        str(export_py),
        "--version",  "v2",
        "--backbone", backbone,
        "--shape",    str(inf_h), str(inf_w),
        "--output-path", str(output),
    ]
    if with_camera_rays:
        cmd.append("--with-camera")

    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        sys.exit(f"ERROR: UniDepth export script exited with code {result.returncode}")


def run_direct_export(
    backbone: str,
    hf_repo: str,
    inf_h: int,
    inf_w: int,
    output: Path,
    with_camera_rays: bool,
    opset: int,
) -> None:
    """Direct PyTorch export when UniDepth's export.py is unavailable or fails.

    Wraps the model in a thin tracer shim that returns positional tensors
    (ONNX cannot trace dict-returning forwards directly).
    """
    try:
        import torch
        from unidepth.models import UniDepthV2  # type: ignore
    except ImportError as exc:
        sys.exit(f"ERROR: {exc}")

    print(f"Loading {hf_repo} …")
    model = UniDepthV2.from_pretrained(hf_repo)
    model.eval()

    # Disable memory-efficient attention (xFormers) before tracing.
    # UniDepth's own export.py sets this flag; we mirror it here.
    if hasattr(model, "config") and isinstance(model.config, dict):
        model.config.setdefault("training", {})["export"] = True

    class _SingleInputShim(torch.nn.Module):
        def __init__(self, inner: torch.nn.Module) -> None:
            super().__init__()
            self.inner = inner

        def forward(self, rgbs: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
            out = self.inner(rgbs)
            return out["pts_3d"], out["confidence"], out["intrinsics"]

    class _TwoInputShim(torch.nn.Module):
        def __init__(self, inner: torch.nn.Module) -> None:
            super().__init__()
            self.inner = inner

        def forward(
            self, rgbs: torch.Tensor, rays: torch.Tensor
        ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
            out = self.inner(rgbs, rays=rays)
            return out["pts_3d"], out["confidence"], out["intrinsics"]

    dummy_rgb = torch.zeros(1, 3, inf_h, inf_w, dtype=torch.float32)

    if with_camera_rays:
        shim = _TwoInputShim(model)
        shim.eval()
        dummy_rays = torch.zeros(1, 3, inf_h, inf_w, dtype=torch.float32)
        dummy_rays[:, 2] = 1.0  # neutral forward-pointing rays
        inputs    = (dummy_rgb, dummy_rays)
        in_names  = ["rgbs", "rays"]
        dyn_axes  = {"rgbs": {0: "batch"}, "rays": {0: "batch"},
                     "pts_3d": {0: "batch"}, "confidence": {0: "batch"},
                     "intrinsics": {0: "batch"}}
    else:
        shim = _SingleInputShim(model)
        shim.eval()
        inputs    = (dummy_rgb,)
        in_names  = ["rgbs"]
        dyn_axes  = {"rgbs": {0: "batch"}, "pts_3d": {0: "batch"},
                     "confidence": {0: "batch"}, "intrinsics": {0: "batch"}}

    print(f"Exporting to {output} (opset {opset}, {inf_h}×{inf_w}) …")
    torch.onnx.export(
        shim,
        inputs,
        str(output),
        input_names=in_names,
        output_names=["pts_3d", "confidence", "intrinsics"],
        dynamic_axes=dyn_axes,
        opset_version=opset,
        do_constant_folding=True,
    )


# ── ORT sanity check ─────────────────────────────────────────────────────────

def sanity_check(output: Path, inf_h: int, inf_w: int, with_camera_rays: bool) -> None:
    try:
        import numpy as np
        import onnxruntime as ort
    except ImportError:
        print("WARNING: onnxruntime not installed — skipping sanity check")
        return

    providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
    sess = ort.InferenceSession(str(output), providers=providers)
    active_ep = sess.get_providers()[0]

    feed: dict = {"rgbs": np.zeros((1, 3, inf_h, inf_w), dtype=np.float32)}
    if with_camera_rays:
        rays = np.zeros((1, 3, inf_h, inf_w), dtype=np.float32)
        rays[:, 2] = 1.0
        feed["rays"] = rays

    pts3d, conf, K = sess.run(["pts_3d", "confidence", "intrinsics"], feed)

    assert pts3d.shape == (1, 3, inf_h, inf_w), \
        f"pts_3d shape mismatch: got {pts3d.shape}"
    assert conf.shape  == (1, 1, inf_h, inf_w), \
        f"confidence shape mismatch: got {conf.shape}"
    assert K.shape     == (1, 3, 3), \
        f"intrinsics shape mismatch: got {K.shape}"

    depth_z = pts3d[0, 2]
    print(f"OK: sanity check passed [{active_ep}]")
    print(f"    pts_3d      {pts3d.shape}  Z: {depth_z.min():.3f}..{depth_z.max():.3f} m")
    print(f"    confidence  {conf.shape}")
    print(f"    intrinsics  {K.shape}  "
          f"predicted fx={K[0,0,0]:.1f} fy={K[0,1,1]:.1f} "
          f"cx={K[0,0,2]:.1f} cy={K[0,1,2]:.1f}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--output", required=True, type=Path,
                    help="Output .onnx path (not committed to repo)")
    ap.add_argument("--backbone", choices=["vits", "vitb", "vitl"], default="vits",
                    help="UniDepth V2 backbone (default: vits)")
    ap.add_argument("--native-height", type=int, default=144,
                    help="Native camera height in pixels (default: 144)")
    ap.add_argument("--native-width",  type=int, default=256,
                    help="Native camera width in pixels (default: 256)")
    ap.add_argument("--inference-height", type=int,
                    help="Override inference height (multiple of 14)")
    ap.add_argument("--inference-width",  type=int,
                    help="Override inference width (multiple of 14)")
    ap.add_argument("--with-camera-rays", action="store_true",
                    help="Export two-input model (rgbs + rays per-pixel unit vectors). "
                         "Requires C++ ray precomputation from camera intrinsics.")
    ap.add_argument("--opset", type=int, default=14,
                    help="ONNX opset version (minimum 14, default: 14)")
    args = ap.parse_args()

    if args.opset < 14:
        sys.exit("ERROR: UniDepth V2 requires opset ≥ 14 (ORT ≥ 1.11)")

    # Both override flags must be present together or absent together.
    if (args.inference_height is None) != (args.inference_width is None):
        sys.exit("ERROR: --inference-height and --inference-width must be given together")

    # Resolve inference shape.
    if args.inference_height is not None:
        inf_h, inf_w = args.inference_height, args.inference_width
        if inf_h % _PATCH_SIZE != 0 or inf_w % _PATCH_SIZE != 0:
            sys.exit(f"ERROR: {inf_h}×{inf_w} — both dims must be multiples of {_PATCH_SIZE}")
        if inf_h * inf_w < _MIN_PIXELS:
            sys.exit(f"ERROR: {inf_h}×{inf_w} = {inf_h*inf_w} px < {_MIN_PIXELS} px training minimum")
    else:
        inf_h, inf_w = compute_inference_shape(args.native_height, args.native_width)

    hf_repo = _HF_REPOS[args.backbone]
    native_px = args.native_height * args.native_width
    inf_px    = inf_h * inf_w
    scale_h   = inf_h / args.native_height
    scale_w   = inf_w / args.native_width

    print("=" * 64)
    print(f"  UniDepth V2 {args.backbone.upper()} ONNX export")
    print("=" * 64)
    print(f"  HuggingFace : {hf_repo}")
    print(f"  Native res  : {args.native_width}×{args.native_height}  "
          f"({native_px:,} px = {native_px/_MIN_PIXELS:.2f}× training minimum)")
    print(f"  Inference   : {inf_w}×{inf_h}  "
          f"({inf_px:,} px, multiples of {_PATCH_SIZE})")
    print(f"  Upsample    : {scale_w:.3f}× W  {scale_h:.3f}× H")
    print(f"  Camera rays : {'yes (2-input)' if args.with_camera_rays else 'no (1-input)'}")
    print(f"  Opset       : {args.opset}")
    print(f"  Output      : {args.output}")
    print()

    args.output.parent.mkdir(parents=True, exist_ok=True)

    # Prefer UniDepth's own export.py — it handles model-specific tracing quirks.
    export_py = find_unidepth_export_script()
    if export_py is not None:
        print(f"Found UniDepth export.py at: {export_py}")
        run_unidepth_export(
            export_py, args.backbone, inf_h, inf_w,
            args.output, args.with_camera_rays, args.opset)
    else:
        print("UniDepth export.py not found — using direct PyTorch export.")
        print("NOTE: if this fails, install UniDepth from source:")
        print("  git clone https://github.com/lpiccinelli-eth/UniDepth.git")
        print("  cd UniDepth && pip install -e .")
        print()
        run_direct_export(
            args.backbone, hf_repo, inf_h, inf_w,
            args.output, args.with_camera_rays, args.opset)

    size_mb = args.output.stat().st_size / (1024 * 1024)
    print(f"OK: wrote {args.output}  ({size_mb:.1f} MB)")
    print()

    print("Running ORT sanity check …")
    sanity_check(args.output, inf_h, inf_w, args.with_camera_rays)
    print()

    # Print scaled intrinsics for reference (C++ computes these automatically).
    fx_inf = _NATIVE_FX * scale_w
    fy_inf = _NATIVE_FY * scale_h
    cx_inf = _NATIVE_CX * scale_w
    cy_inf = _NATIVE_CY * scale_h

    print("─" * 64)
    print("YAML config (add to mission YAML or set DEDALUS_DEPTH=unidepth_v2):")
    print("─" * 64)
    print(f"depth: unidepth_v2")
    print(f"depth_eval: airsim_gt_vd   # keep GT in slot B for A/B eval")
    print(f"unidepth.model_path:       {args.output}")
    print(f"unidepth.backbone:         {args.backbone}")
    print(f"unidepth.inference_height: {inf_h}")
    print(f"unidepth.inference_width:  {inf_w}")
    print(f"unidepth.native_height:    {args.native_height}")
    print(f"unidepth.native_width:     {args.native_width}")
    print(f"unidepth.use_camera_rays:  {'true' if args.with_camera_rays else 'false'}")
    print(f"unidepth.use_cuda:         true")
    print()
    print("Inference-space intrinsics (computed automatically by the engine):")
    print(f"  fx={fx_inf:.1f}  fy={fy_inf:.1f}  cx={cx_inf:.1f}  cy={cy_inf:.1f}")
    print("─" * 64)


if __name__ == "__main__":
    main()
