#!/usr/bin/env python3
"""Export DepthAnythingV2-Small to ONNX from HuggingFace weights.

The exported model is NOT committed to the repo.
Run this once on each target device to generate the .onnx file,
then run compile_depth_engine.sh to produce a TensorRT .engine (Jetson only).

Requirements:
    pip install torch transformers onnx onnxruntime

Usage:
    python3 tools/perception/export_depth_anything.py --output /path/to/model.onnx
    python3 tools/perception/export_depth_anything.py --output /path/to/model.onnx \\
        --input-size 518 --opset 17

Exported ONNX contract:
    Input:  "image"  float32  [1, 3, H, W]  ImageNet-normalised RGB
    Output: "depth"  float32  [1, H, W]     Relative depth (higher = closer)

ONNXDepthEngine default input size: 518×518 (DepthAnythingV2-Small native).
"""

import argparse
import sys
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", required=True, type=Path,
                        help="Output .onnx path (not committed to repo)")
    parser.add_argument("--input-size", type=int, default=518,
                        help="Square input resolution fed to the model (default: 518)")
    parser.add_argument("--opset", type=int, default=17,
                        help="ONNX opset version (default: 17)")
    parser.add_argument("--model-id", default="depth-anything/Depth-Anything-V2-Small-hf",
                        help="HuggingFace model ID")
    args = parser.parse_args()

    try:
        import torch
        from transformers import AutoModelForDepthEstimation
    except ImportError as exc:
        sys.exit(f"ERROR: {exc}\n  pip install torch transformers onnx")

    print(f"Loading {args.model_id} …")
    model = AutoModelForDepthEstimation.from_pretrained(args.model_id)
    model.eval()

    H = W = args.input_size
    dummy = torch.zeros(1, 3, H, W, dtype=torch.float32)

    # Wrap the HuggingFace model to produce a plain [1, H, W] depth tensor.
    # The HF model returns a ModelOutput with a .predicted_depth attribute.
    class _Wrapper(torch.nn.Module):
        def __init__(self, inner: torch.nn.Module) -> None:
            super().__init__()
            self.inner = inner

        def forward(self, image: torch.Tensor) -> torch.Tensor:
            out = self.inner(pixel_values=image)
            depth = out.predicted_depth          # [1, H, W] or [B, H, W]
            if depth.dim() == 2:
                depth = depth.unsqueeze(0)       # [1, H, W]
            return depth

    wrapper = _Wrapper(model)
    wrapper.eval()

    output_path = args.output
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"Exporting to {output_path}  (opset {args.opset}, input {H}×{W}) …")
    torch.onnx.export(
        wrapper,
        dummy,
        str(output_path),
        input_names=["image"],
        output_names=["depth"],
        dynamic_axes={"image": {0: "batch"}, "depth": {0: "batch"}},
        opset_version=args.opset,
        do_constant_folding=True,
    )

    # Quick sanity check via onnxruntime
    try:
        import onnxruntime as ort
        import numpy as np
        sess = ort.InferenceSession(str(output_path), providers=["CPUExecutionProvider"])
        inp = np.zeros((1, 3, H, W), dtype=np.float32)
        out = sess.run(["depth"], {"image": inp})
        assert out[0].shape[-2:] == (H, W), f"Unexpected output shape: {out[0].shape}"
        print(f"OK: sanity check passed — output shape {out[0].shape}")
    except ImportError:
        print("WARNING: onnxruntime not installed — skipping sanity check")

    print(f"OK: wrote {output_path}  ({output_path.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
