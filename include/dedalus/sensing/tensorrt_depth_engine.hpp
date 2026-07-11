#pragma once

// TensorRTDepthEngine — TensorRT-accelerated depth inference for NVIDIA GPUs.
//
// Only compiled when DEDALUS_TENSORRT_ENABLED (CMake DEDALUS_TENSORRT=ON).
// Requires DEDALUS_CUDA=ON.
//
// Model loading strategy:
//   engine_path ends in .engine → deserialize directly (< 100 ms).
//   engine_path ends in .onnx   → build TRT engine on first run with FP16,
//                                 cache to <engine_path>.trt_cache for reuse.
//
// Precision: FP16 by default (L4 = 242 TFLOPS FP16 vs 121 TFLOPS FP32).
//
// API: compatible with TensorRT 8.x and 10.x.
//   TRT 8: IExecutionContext::enqueueV2()
//   TRT 10: IExecutionContext::executeV2()
// Handled at compile time via NV_TENSORRT_MAJOR version check.

#ifdef DEDALUS_TENSORRT_ENABLED

#include <memory>
#include <string>

#include "dedalus/sensing/depth_engine.hpp"

namespace dedalus {

struct TensorRTDepthEngineConfig {
    // Path to a pre-built .engine file or an .onnx model.
    // If .onnx, the engine is built on first run and cached alongside the model.
    std::string engine_path;

    int  device_id{0};
    bool use_fp16{true};   // FP16 precision (L4, A10, A100 all support it)
    bool use_int8{false};  // INT8 requires an external calibration cache

    int model_input_width{518};   // DepthAnythingV2-Small default
    int model_input_height{518};

    // ImageNet normalisation
    float mean_r{0.485F}, mean_g{0.456F}, mean_b{0.406F};
    float std_r{0.229F},  std_g{0.224F},  std_b{0.225F};

    // TRT builder workspace — limits VRAM used during engine build (not at inference).
    // 512 MiB is sufficient for DepthAnythingV2-Small; AirSim uses ~3 GiB separately.
    std::size_t builder_workspace_bytes{512ULL * 1024 * 1024};

    // When true the model outputs LINEAR METRIC DEPTH in metres (HIGH=FAR):
    //   e.g. DepthAnythingV2-Metric-Outdoor: Sigmoid → Mul(×80), raw ∈ [0, 80] m.
    // The engine converts to pipeline convention: inverse_depth = scale / raw.
    // When false the engine clamps raw values as already valid inverse_depth (HIGH=CLOSE).
    bool metric_depth{true};
    // Engine-internal scale for inverse_depth = scale / raw conversion.
    // Keep at 1.0 — calibrated scene scale lives in MetricScaleEstimate / ProjectionParams.
    float scale{1.0F};
};

class TensorRTDepthEngine final : public DepthEngineInterface {
public:
    explicit TensorRTDepthEngine(TensorRTDepthEngineConfig config);
    ~TensorRTDepthEngine() override;

    TensorRTDepthEngine(const TensorRTDepthEngine&)            = delete;
    TensorRTDepthEngine& operator=(const TensorRTDepthEngine&) = delete;

    [[nodiscard]] DepthInferenceResult infer(const VisualDepthFrame& frame) override;
    [[nodiscard]] std::string engine_name() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace dedalus

#endif  // DEDALUS_TENSORRT_ENABLED
