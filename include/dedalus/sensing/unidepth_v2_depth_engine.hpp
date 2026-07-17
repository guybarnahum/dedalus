#pragma once

// UniDepthV2DepthEngine — UniDepth V2 metric depth provider over ONNX Runtime.
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

struct UniDepthV2DepthEngineConfig {
    std::string model_path;            // path to exported .onnx file

    // Inference resolution: both dims must be multiples of ViT patch size 14,
    // and inference_height * inference_width >= 200 000 px.
    // Default 336×602 = 202 032 px for our 256×144 camera.
    int inference_width{602};
    int inference_height{336};

    // Native camera resolution — dimensions returned in DepthInferenceResult.
    // Engine downsamples inference Z map back to native dims so all downstream
    // consumers (annotator, L0 viz, frame0 dump) see a consistent buffer.
    int native_width{256};
    int native_height{144};

    // When true, compute per-pixel unit-direction camera rays from intrinsics
    // and pass as second ONNX input "rays" (two-input export variant required).
    bool use_camera_rays{false};

    float scale{1.0F};

    bool use_coreml{false};
    bool use_cuda{false};
    int  cuda_device_id{0};
    std::size_t cuda_arena_limit_bytes{1ULL * 1024 * 1024 * 1024};
};

class UniDepthV2DepthEngine final : public OnnxDepthEngineBase {
public:
    explicit UniDepthV2DepthEngine(UniDepthV2DepthEngineConfig config);
    ~UniDepthV2DepthEngine() override;

    UniDepthV2DepthEngine(const UniDepthV2DepthEngine&)            = delete;
    UniDepthV2DepthEngine& operator=(const UniDepthV2DepthEngine&) = delete;

    [[nodiscard]] std::string engine_name() const override;

protected:
    [[nodiscard]] OnnxInputs               prepare_inputs(const VisualDepthFrame& frame) override;
    [[nodiscard]] std::vector<std::string> output_names() const override;
    [[nodiscard]] DepthInferenceResult     extract_result(const VisualDepthFrame& frame,
                                                          std::vector<Ort::Value>& outputs,
                                                          float inference_time_ms,
                                                          bool is_first) override;

private:
    UniDepthV2DepthEngineConfig config_;

    static std::vector<float> build_camera_rays(
        double fx, double fy, double cx, double cy,
        int native_w, int native_h,
        int dst_w,   int dst_h);
};

}  // namespace dedalus
