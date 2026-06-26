#pragma once

#include <memory>
#include <string>
#include <vector>

#include "dedalus/sensing/visual_depth_frame.hpp"

namespace dedalus {

// Output of a single depth inference pass.
//
// depth_relative is a H×W float buffer in row-major order.
// Values are relative (dimensionless); convert to metric via MetricScaleEstimate:
//   depth_m = scale / depth_relative[i]  (disparity convention)
//
// The buffer lives on the CPU here. The CUDA path (VD6) keeps it on device;
// the interface remains the same — the CPU path fills depth_relative normally,
// the CUDA path transfers only the compact DeviceObstacleEvidence result.
struct DepthInferenceResult {
    int width{0};
    int height{0};
    std::vector<float> depth_relative;  // H × W, values in (0, 1]
    float inference_time_ms{0.0F};
    bool valid{false};
};

// Platform-transparent depth estimation interface.
//
// Implementations:
//   ONNXDepthEngine      — ONNX Runtime, CPU / CoreML / MPS (Mac dev)
//   TensorRTDepthEngine  — TensorRT INT8 (Jetson, VD6)
//   AirSimEmulationDepthEngine — wraps AirSim DepthPlanar GT as a validation oracle
class DepthEngineInterface {
public:
    virtual ~DepthEngineInterface() = default;

    // Run depth inference on a single RGB frame.
    // Thread ownership: called from the detector tick only; not thread-safe.
    [[nodiscard]] virtual DepthInferenceResult infer(const VisualDepthFrame& frame) = 0;

    // Human-readable identifier for logging and delta-log comparisons.
    [[nodiscard]] virtual std::string engine_name() const = 0;
};

}  // namespace dedalus
