#!/usr/bin/env python3
"""
audit_depth_model.py — Empirical ONNX depth model auditor.

Answers two questions without touching the C++ stack:
  1. GRAPH AUDIT  — Inspect final nodes for Sigmoid→Mul(×N) (metric) or
                    ReLU/linear tail (relative). No image needed.
  2. INFERENCE    — Run a synthetic sky+ground image through the model and
                    read the raw output tensor. Verdict printed at the end.

Usage:
    python3 tools/perception/audit_depth_model.py models/depth_anything_v2_metric_vits.onnx
    python3 tools/perception/audit_depth_model.py models/depth_anything_v2_metric_vits.onnx --image frame.png
    python3 tools/perception/audit_depth_model.py models/depth_anything_v2_metric_vits.onnx --graph-only
"""

import argparse
import sys
from pathlib import Path

import numpy as np


# ---------------------------------------------------------------------------
# Graph audit — no inference required
# ---------------------------------------------------------------------------

def audit_graph(model_path: Path) -> dict:
    try:
        import onnx
    except ImportError:
        print("pip install onnx", file=sys.stderr)
        sys.exit(1)

    model   = onnx.load(str(model_path))
    graph   = model.graph
    nodes   = list(graph.node)
    tail    = nodes[-10:]

    initializers: dict[str, np.ndarray] = {}
    for init in graph.initializer:
        if init.float_data:
            initializers[init.name] = np.array(list(init.float_data), dtype=np.float32)
        elif init.raw_data:
            initializers[init.name] = np.frombuffer(init.raw_data, dtype=np.float32).copy()

    print("=" * 60)
    print("GRAPH AUDIT")
    print("=" * 60)
    print(f"Nodes total : {len(nodes)}")
    print(f"Inputs      : {[i.name for i in graph.input]}")
    print(f"Outputs     : {[o.name for o in graph.output]}")
    print()
    print("Last 10 nodes:")

    sigmoid_found = False
    mul_scale     = None
    relu_found    = False

    for node in tail:
        attr_str = ""
        for a in node.attribute:
            if a.type == 1:   # float
                attr_str += f" {a.name}={a.f:.4g}"
            elif a.type == 2: # int
                attr_str += f" {a.name}={a.i}"
        const_str = ""
        for inp in node.input:
            if inp in initializers and initializers[inp].size <= 4:
                val = initializers[inp]
                const_str += f"  ← const {inp}={val}"
                if node.op_type in ("Mul", "Div"):
                    mul_scale = float(val.flat[0])
        print(f"  {node.op_type:14s} {attr_str}{const_str}")

        if node.op_type == "Sigmoid":
            sigmoid_found = True
        if node.op_type == "Relu":
            relu_found = True

    print()

    # Verdict
    if sigmoid_found and mul_scale is not None:
        kind = "METRIC"
        detail = (f"Sigmoid → Mul(×{mul_scale:.1f}) tail. "
                  f"Output = metric depth in metres. HIGH VALUE = FAR.")
    elif sigmoid_found:
        kind = "METRIC (scale constant not found in last 10 nodes)"
        detail = "Sigmoid tail present. Likely metric."
    elif relu_found:
        kind = "RELATIVE"
        detail = "ReLU tail (no Sigmoid×scale). Output = relative disparity. HIGH VALUE = CLOSER."
    else:
        kind = "UNKNOWN"
        detail = "Could not determine from tail nodes."

    print(f"GRAPH VERDICT: {kind}")
    print(f"  {detail}")
    print()

    return {"kind": kind, "sigmoid": sigmoid_found, "mul_scale": mul_scale, "relu": relu_found}


# ---------------------------------------------------------------------------
# Inference audit
# ---------------------------------------------------------------------------

def make_synthetic_image(size: int = 518) -> np.ndarray:
    """Sky (top half, blue) + ground (bottom half, brown) synthetic frame."""
    img = np.zeros((size, size, 3), dtype=np.uint8)
    img[:size // 2, :] = [135, 206, 235]  # sky  (BGR)
    img[size // 2:, :] = [80,   60,  40]  # ground (BGR)
    return img


def preprocess(img_bgr: np.ndarray, size: int = 518) -> np.ndarray:
    """Resize → RGB → ImageNet normalise → NCHW float32."""
    import cv2
    img = cv2.resize(img_bgr, (size, size))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    std  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    img  = (img - mean) / std
    return np.expand_dims(img.transpose(2, 0, 1), 0)   # [1, 3, H, W]


def audit_inference(model_path: Path, image_path: Path | None) -> dict:
    try:
        import onnxruntime as ort
    except ImportError:
        print("pip install onnxruntime  (or onnxruntime-gpu)", file=sys.stderr)
        sys.exit(1)
    try:
        import cv2
    except ImportError:
        print("pip install opencv-python-headless", file=sys.stderr)
        sys.exit(1)

    providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
    session   = ort.InferenceSession(str(model_path), providers=providers)
    used_ep   = session.get_providers()[0]

    if image_path and image_path.exists():
        img = cv2.imread(str(image_path))
        if img is None:
            print(f"WARNING: could not read {image_path}, using synthetic image")
            img = make_synthetic_image()
        else:
            print(f"Image source : {image_path}")
    else:
        print("Image source : synthetic (sky top-half, ground bottom-half)")
        img = make_synthetic_image()

    inp = preprocess(img)
    raw = session.run(["depth"], {"image": inp})[0].squeeze()  # [H, W]

    H, W   = raw.shape
    sky_y  = H // 4          # upper quarter → sky
    gnd_y  = 3 * H // 4     # lower quarter → ground
    cx     = W // 2

    sky_val = float(raw[sky_y,  cx])
    gnd_val = float(raw[gnd_y, cx])

    print()
    print("=" * 60)
    print("INFERENCE AUDIT")
    print("=" * 60)
    print(f"Execution provider : {used_ep}")
    print(f"Output shape       : {raw.shape}")
    print(f"Min                : {raw.min():.4f}")
    print(f"Max                : {raw.max():.4f}")
    print(f"Mean               : {raw.mean():.4f}")
    print(f"Std                : {raw.std():.4f}")
    print()
    print(f"Sky pixel   [{sky_y:3d}, {cx:3d}] = {sky_val:.4f}")
    print(f"Ground pixel[{gnd_y:3d}, {cx:3d}] = {gnd_val:.4f}")
    print()

    # Directional verdict
    if sky_val > gnd_val:
        direction = "sky > ground  →  HIGH VALUE = FAR  (metric convention)"
        kind = "METRIC"
    else:
        direction = "ground > sky  →  HIGH VALUE = CLOSE  (inverse-depth / relative convention)"
        kind = "INVERSE_DEPTH"

    # Scale verdict
    if raw.max() > 50.0:
        scale_verdict = f"max≈{raw.max():.1f} m  →  saturating at metric max-depth cap (~80 m)"
    elif raw.max() <= 1.05:
        scale_verdict = f"max≈{raw.max():.4f}  →  normalised [0,1] relative disparity"
    else:
        scale_verdict = f"max≈{raw.max():.2f}  →  unbounded relative disparity"

    print(f"DIRECTIONAL: {direction}")
    print(f"SCALE:       {scale_verdict}")
    print()

    # Print 5×5 corners
    np.set_printoptions(precision=3, suppress=True)
    print("Top-left 5×5 (sky):")
    print(raw[:5, :5])
    print("Bottom-left 5×5 (ground):")
    print(raw[-5:, :5])
    print()

    print(f"INFERENCE VERDICT: {kind}")

    return {"kind": kind, "min": float(raw.min()), "max": float(raw.max()),
            "mean": float(raw.mean()), "sky": sky_val, "ground": gnd_val}


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("model", type=Path, help="Path to .onnx model file")
    parser.add_argument("--image", type=Path, default=None,
                        help="Optional BGR image for inference (default: synthetic sky+ground)")
    parser.add_argument("--graph-only", action="store_true",
                        help="Skip inference, only inspect graph structure")
    args = parser.parse_args()

    if not args.model.exists():
        print(f"ERROR: model not found: {args.model}", file=sys.stderr)
        sys.exit(1)

    graph_result = audit_graph(args.model)

    if not args.graph_only:
        infer_result = audit_inference(args.model, args.image)

        # Combined verdict
        print("=" * 60)
        print("COMBINED VERDICT")
        print("=" * 60)
        g = graph_result["kind"].split()[0]
        i = infer_result["kind"]
        if g == i or g in i:
            print(f"CONSISTENT: both graph and inference agree → {g}")
        else:
            print(f"CONFLICT: graph says {g}, inference says {i}")
            print("  → Check the export script; the graph tail may use a custom op.")

        if infer_result["kind"] == "METRIC":
            print()
            print("Pipeline config for metric model loaded via visual_onnx engine (high=FAR):")
            print("  visual_onnx.metric_depth: true")
            print("  visual_onnx.scale: 1.0  (calibrate per scene; engine uses scale/raw)")
            print("  ONNXDepthEngine converts: inverse_depth = scale/raw (1/metres)")
            print("  Kernel recovers: depth_m = scale/inverse_depth = raw ✓")
            print("  Note: UniDepth V2 uses the unidepth_v2 engine and unidepth.* config keys,")
            print("        not visual_onnx.*  (export via tools/perception/export_unidepth.py).")
        else:
            print()
            print("Pipeline config for inverse-depth model loaded via visual_onnx engine (high=CLOSE):")
            print("  visual_onnx.metric_depth: false   # or true with 1/raw conversion")
            print("  visual_onnx.scale: <calibrate>")


if __name__ == "__main__":
    main()
