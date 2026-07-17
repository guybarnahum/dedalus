#pragma once

// ONNXDepthEngine — DepthAnythingV2 provider over ONNX Runtime.
// Only compiled when DEDALUS_ENABLE_ONNX_DEPTH=ON.
// Include only inside #ifdef DEDALUS_ONNX_DEPTH_ENABLED guards.

#include <cstddef>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include "dedalus/sensing/depth_engine_utils.hpp"
#include "dedalus/sensing/onnx_depth_engine_base.hpp"
#include "dedalus/sensing/visual_depth_frame.hpp"

namespace dedalus {

struct ONNXDepthEngineConfig {
    std::string model_path;
    std::string input_name{"image"};
    std::string output_name{"depth"};
    int model_input_width{518};    // DepthAnythingV2-Small default
    int model_input_height{518};

    // ImageNet normalisation
    float mean_r{0.485F};
    float mean_g{0.456F};
    float mean_b{0.406F};
    float std_r{0.229F};
    float std_g{0.224F};
    float std_b{0.225F};

    bool use_coreml{false};
    bool metric_depth{true};
    float scale{1.0F};

    bool use_cuda{false};
    int  cuda_device_id{0};
    std::size_t cuda_arena_limit_bytes{4ULL * 1024 * 1024 * 1024};
};

class ONNXDepthEngine final : public OnnxDepthEngineBase {
public:
    explicit ONNXDepthEngine(ONNXDepthEngineConfig config);
    ~ONNXDepthEngine() override;

    ONNXDepthEngine(const ONNXDepthEngine&)            = delete;
    ONNXDepthEngine& operator=(const ONNXDepthEngine&) = delete;

    [[nodiscard]] std::string engine_name() const override;

protected:
    [[nodiscard]] OnnxInputs               prepare_inputs(const VisualDepthFrame& frame) override;
    [[nodiscard]] std::vector<std::string> output_names() const override;
    [[nodiscard]] DepthInferenceResult     extract_result(const VisualDepthFrame& frame,
                                                          std::vector<Ort::Value>& outputs,
                                                          float inference_time_ms,
                                                          bool is_first) override;

private:
    ONNXDepthEngineConfig config_;
};

}  // namespace dedalus
