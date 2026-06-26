#pragma once

#include <memory>
#include <string>

#include "dedalus/sensing/depth_engine.hpp"

namespace dedalus {

struct ONNXDepthEngineConfig {
    std::string model_path;        // path to .onnx file (not committed — generated on device)
    std::string input_name{"image"};
    std::string output_name{"depth"};
    int model_input_width{518};    // DepthAnythingV2-Small default
    int model_input_height{518};

    // ImageNet normalisation applied before inference
    float mean_r{0.485F};
    float mean_g{0.456F};
    float mean_b{0.406F};
    float std_r{0.229F};
    float std_g{0.224F};
    float std_b{0.225F};

    bool use_coreml{false};        // enable CoreML EP on macOS (MPS acceleration)
};

// ONNXDepthEngine — wraps ONNX Runtime for CPU / CoreML depth inference.
//
// Only compiled when DEDALUS_ENABLE_ONNX_DEPTH=ON (requires ONNX Runtime).
// Instantiate via make_onnx_depth_engine() to keep the build-flag guard at
// one site — callers that only hold a DepthEngineInterface* are unaffected
// when the flag is off.
class ONNXDepthEngine final : public DepthEngineInterface {
public:
    explicit ONNXDepthEngine(ONNXDepthEngineConfig config);
    ~ONNXDepthEngine() override;

    ONNXDepthEngine(const ONNXDepthEngine&)            = delete;
    ONNXDepthEngine& operator=(const ONNXDepthEngine&) = delete;

    [[nodiscard]] DepthInferenceResult infer(const VisualDepthFrame& frame) override;
    [[nodiscard]] std::string engine_name() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace dedalus
