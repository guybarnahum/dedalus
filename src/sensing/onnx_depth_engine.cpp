#include "dedalus/sensing/onnx_depth_engine.hpp"

// ONNXDepthEngine — DepthAnythingV2 subclass of OnnxDepthEngineBase.
// Only compiled when DEDALUS_ENABLE_ONNX_DEPTH=ON.

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include "dedalus/sensing/depth_engine_utils.hpp"

namespace dedalus {

ONNXDepthEngine::ONNXDepthEngine(ONNXDepthEngineConfig config)
    : OnnxDepthEngineBase(
          SessionConfig{
              .model_path             = config.model_path,
              .use_cuda               = config.use_cuda,
              .cuda_device_id         = config.cuda_device_id,
              .cuda_arena_limit_bytes = config.cuda_arena_limit_bytes,
              .use_coreml             = config.use_coreml,
          },
          "ONNXDepthEngine"),
      config_(std::move(config)) {}

ONNXDepthEngine::~ONNXDepthEngine() = default;

std::string ONNXDepthEngine::engine_name() const {
    return "onnx_depth_engine";
}

// --- Input translation ---

OnnxDepthEngineBase::OnnxInputs ONNXDepthEngine::prepare_inputs(
    const VisualDepthFrame& frame) {

    const int mw = config_.model_input_width;
    const int mh = config_.model_input_height;

    OnnxInputs inputs;
    inputs.names = {config_.input_name};
    inputs.buffers.push_back(depth_engine_utils::resize_and_normalise(
        frame.bytes, frame.width, frame.height, mw, mh,
        config_.mean_r, config_.mean_g, config_.mean_b,
        config_.std_r,  config_.std_g,  config_.std_b));

    const std::array<std::int64_t, 4> shape{1, 3, mh, mw};
    Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    inputs.values.push_back(Ort::Value::CreateTensor<float>(
        mem, inputs.buffers[0].data(), inputs.buffers[0].size(),
        shape.data(), shape.size()));

    return inputs;
}

std::vector<std::string> ONNXDepthEngine::output_names() const {
    return {config_.output_name};
}

// --- Output translation ---

DepthInferenceResult ONNXDepthEngine::extract_result(
    const VisualDepthFrame& frame,
    std::vector<Ort::Value>& outputs,
    float inference_time_ms,
    bool is_first) {

    const float* raw  = outputs[0].GetTensorData<float>();
    const auto   info = outputs[0].GetTensorTypeAndShapeInfo();
    const auto   shape = info.GetShape();

    const int out_h = static_cast<int>(shape[shape.size() - 2]);
    const int out_w = static_cast<int>(shape[shape.size() - 1]);
    const std::size_t n = static_cast<std::size_t>(out_h * out_w);

    DepthInferenceResult result;
    result.width             = out_w;
    result.height            = out_h;
    result.inference_time_ms = inference_time_ms;
    result.inverse_depth.resize(n);

    if (config_.metric_depth) {
        // DepthAnythingV2-Metric-Outdoor: raw ∈ [0, 80] m, HIGH=FAR.
        // Convert to inverse_depth = scale / raw  (pipeline convention: HIGH=CLOSE).
        static constexpr float kMinValidRaw = 0.01F;
        const float scale = config_.scale > 0.0F ? config_.scale : 1.0F;
        for (std::size_t i = 0; i < n; ++i) {
            result.inverse_depth[i] = (raw[i] >= kMinValidRaw) ? (scale / raw[i]) : 0.0F;
        }
    } else {
        // Relative model: normalise by per-frame max (HIGH=CLOSE already).
        float max_val = 1e-6F;
        for (std::size_t i = 0; i < n; ++i) max_val = std::max(max_val, raw[i]);
        for (std::size_t i = 0; i < n; ++i) {
            result.inverse_depth[i] = std::max(raw[i] / max_val, 1e-6F);
        }
    }
    result.valid = true;

    // First-inference: log model-specific raw stats.
    // Expected at 5-30 m AGL: metric mean ~10..30 m, relative mean ~0.3..0.7.
    // BAD: mean < 1.0 (metric) → relative model deployed; max ≈ min → EP failure.
    if (is_first) {
        float raw_min = raw[0], raw_max = raw[0], raw_sum = 0.0F;
        for (std::size_t i = 0; i < n; ++i) {
            if (raw[i] < raw_min) raw_min = raw[i];
            if (raw[i] > raw_max) raw_max = raw[i];
            raw_sum += raw[i];
        }
        std::fprintf(stderr,
            "[ONNXDepthEngine] raw  min=%.3f  max=%.3f  mean=%.3f  mode=%s\n",
            static_cast<double>(raw_min),
            static_cast<double>(raw_max),
            static_cast<double>(raw_sum / static_cast<float>(n)),
            config_.metric_depth ? "metric" : "relative");

        depth_engine_utils::dump_npy_f32("/tmp/dedalus_frame0_depth.npy", raw, out_h, out_w);
        if (!frame.bytes.empty() && frame.channels > 0) {
            depth_engine_utils::dump_npy_u8("/tmp/dedalus_frame0_rgb.npy",
                frame.bytes.data(), frame.height, frame.width, frame.channels);
        }
    }

    depth_engine_utils::maybe_write_debug_npy(
        std::getenv("DEDALUS_DEPTH_DEBUG_DIR"), frame.frame_id.value, raw, out_h, out_w);

    return result;
}

}  // namespace dedalus
