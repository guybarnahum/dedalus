#include "dedalus/sensing/onnx_depth_engine.hpp"

// ONNXDepthEngine requires ONNX Runtime.
// Build with -DDEDALUS_ENABLE_ONNX_DEPTH=ON and ONNX Runtime installed.
// When the flag is off this TU is excluded from the build by CMakeLists.txt.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace dedalus {

// ---------------------------------------------------------------------------
// Impl
// ---------------------------------------------------------------------------

struct ONNXDepthEngine::Impl {
    ONNXDepthEngineConfig config;

    Ort::Env            env{ORT_LOGGING_LEVEL_WARNING, "dedalus_depth"};
    Ort::SessionOptions session_options;
    Ort::Session        session{nullptr};

    explicit Impl(ONNXDepthEngineConfig cfg) : config(std::move(cfg)) {
        session_options.SetIntraOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(
            GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

#if defined(__APPLE__)
        if (config.use_coreml) {
            // CoreML EP — hardware-accelerated on Apple Silicon / MPS
            uint32_t coreml_flags = 0;
            OrtSessionOptionsAppendExecutionProvider_CoreML(
                session_options, coreml_flags);
        }
#endif

#if defined(DEDALUS_CUDA_ENABLED)
        if (config.use_cuda) {
            // CUDA EP — requires onnxruntime-gpu package.
            // arena_limit caps VRAM to coexist with AirSim (~3 GiB).
            OrtCUDAProviderOptionsV2* cuda_opts = nullptr;
            Ort::ThrowOnError(Ort::GetApi().CreateCUDAProviderOptions(&cuda_opts));
            const std::string dev_id_str = std::to_string(config.cuda_device_id);
            const std::string arena_str  = std::to_string(config.cuda_arena_limit_bytes);
            const char* keys[]   = {"device_id", "gpu_mem_limit"};
            const char* values[] = {dev_id_str.c_str(), arena_str.c_str()};
            Ort::ThrowOnError(Ort::GetApi().UpdateCUDAProviderOptions(
                cuda_opts, keys, values, 2));
            Ort::ThrowOnError(Ort::GetApi().SessionOptionsAppendExecutionProvider_CUDA_V2(
                session_options, cuda_opts));
            Ort::GetApi().ReleaseCUDAProviderOptions(cuda_opts);
        }
#endif

        session = Ort::Session{env, config.model_path.c_str(), session_options};
    }

    // Bilinear resize of a uint8 HxWx3 image to target_h x target_w.
    static std::vector<float> resize_and_normalise(
        const std::vector<std::uint8_t>& bytes,
        int src_w, int src_h,
        int dst_w, int dst_h,
        float mean_r, float mean_g, float mean_b,
        float std_r,  float std_g,  float std_b) {

        // Output: NCHW float tensor [1, 3, dst_h, dst_w]
        std::vector<float> tensor(static_cast<std::size_t>(3 * dst_h * dst_w), 0.0F);

        const float sx = static_cast<float>(src_w) / static_cast<float>(dst_w);
        const float sy = static_cast<float>(src_h) / static_cast<float>(dst_h);

        const float mean[3] = {mean_r, mean_g, mean_b};
        const float stdv[3] = {std_r,  std_g,  std_b};

        for (int dy = 0; dy < dst_h; ++dy) {
            for (int dx = 0; dx < dst_w; ++dx) {
                // Nearest-neighbour sampling (sufficient for inference preprocessing)
                const int sx_i = std::min(static_cast<int>(dx * sx), src_w - 1);
                const int sy_i = std::min(static_cast<int>(dy * sy), src_h - 1);
                const std::size_t src_idx =
                    (static_cast<std::size_t>(sy_i) * static_cast<std::size_t>(src_w) +
                     static_cast<std::size_t>(sx_i)) * 3U;

                for (int c = 0; c < 3; ++c) {
                    const float pixel = static_cast<float>(bytes[src_idx + c]) / 255.0F;
                    const float norm  = (pixel - mean[c]) / stdv[c];
                    const std::size_t dst_idx =
                        static_cast<std::size_t>(c) * static_cast<std::size_t>(dst_h * dst_w) +
                        static_cast<std::size_t>(dy * dst_w + dx);
                    tensor[dst_idx] = norm;
                }
            }
        }
        return tensor;
    }
};

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

ONNXDepthEngine::ONNXDepthEngine(ONNXDepthEngineConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

ONNXDepthEngine::~ONNXDepthEngine() = default;

std::string ONNXDepthEngine::engine_name() const {
    return "onnx_depth_engine";
}

DepthInferenceResult ONNXDepthEngine::infer(const VisualDepthFrame& frame) {
    if (frame.bytes.empty() || frame.width <= 0 || frame.height <= 0) {
        return {};
    }

    const auto t0 = std::chrono::steady_clock::now();

    const int mw = impl_->config.model_input_width;
    const int mh = impl_->config.model_input_height;

    // Preprocess: resize + ImageNet normalise → NCHW float tensor
    std::vector<float> input_tensor = Impl::resize_and_normalise(
        frame.bytes,
        frame.width, frame.height,
        mw, mh,
        impl_->config.mean_r, impl_->config.mean_g, impl_->config.mean_b,
        impl_->config.std_r,  impl_->config.std_g,  impl_->config.std_b);

    // Build ONNX input
    Ort::AllocatorWithDefaultOptions allocator;
    const std::array<std::int64_t, 4> input_shape{1, 3, mh, mw};

    Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator, OrtMemTypeDefault);

    Ort::Value input_val = Ort::Value::CreateTensor<float>(
        mem_info,
        input_tensor.data(),
        input_tensor.size(),
        input_shape.data(),
        input_shape.size());

    const char* input_names[]  = {impl_->config.input_name.c_str()};
    const char* output_names[] = {impl_->config.output_name.c_str()};

    auto output_tensors = impl_->session.Run(
        Ort::RunOptions{nullptr},
        input_names, &input_val, 1U,
        output_names, 1U);

    // Extract output — shape may be [1, H, W] or [1, 1, H, W]
    const float* raw = output_tensors[0].GetTensorData<float>();
    const auto   info = output_tensors[0].GetTensorTypeAndShapeInfo();
    const auto   shape = info.GetShape();

    const int out_h = static_cast<int>(shape[shape.size() - 2]);
    const int out_w = static_cast<int>(shape[shape.size() - 1]);
    const std::size_t n = static_cast<std::size_t>(out_h * out_w);

    // Normalise output to [epsilon, 1.0] (model may output raw logits or [0, ∞))
    float max_val = 1e-6F;
    for (std::size_t i = 0; i < n; ++i) {
        max_val = std::max(max_val, raw[i]);
    }

    DepthInferenceResult result;
    result.width  = out_w;
    result.height = out_h;
    result.depth_relative.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        result.depth_relative[i] = std::max(raw[i] / max_val, 1e-6F);
    }

    const auto t1 = std::chrono::steady_clock::now();
    result.inference_time_ms = static_cast<float>(
        std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count()) / 1000.0F;
    result.valid = true;

    return result;
}

}  // namespace dedalus
