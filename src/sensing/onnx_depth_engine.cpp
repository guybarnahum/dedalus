#include "dedalus/sensing/onnx_depth_engine.hpp"

// ONNXDepthEngine requires ONNX Runtime.
// Build with -DDEDALUS_ENABLE_ONNX_DEPTH=ON and ONNX Runtime installed.
// When the flag is off this TU is excluded from the build by CMakeLists.txt.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
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

    bool cuda_ep_registered  = false;  // set true only when CUDA EP is actually registered
    bool first_inference_logged = false;

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
            // CUDA EP — requires onnxruntime-gpu package and libcublasLt.so.12 in
            // LD_LIBRARY_PATH. Falls back to CPU silently if the shared library is
            // absent; the log line below distinguishes the two cases.
            try {
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
                cuda_ep_registered = true;
                std::fprintf(stderr,
                    "[ONNXDepthEngine] CUDA EP registered (device_id=%d, "
                    "arena_limit=%zu bytes)\n",
                    config.cuda_device_id,
                    static_cast<std::size_t>(config.cuda_arena_limit_bytes));
            } catch (const Ort::Exception& e) {
                std::fprintf(stderr,
                    "[ONNXDepthEngine] WARN: CUDA EP registration failed (%s); "
                    "falling back to CPU. Check that libcublasLt.so.12 is in "
                    "LD_LIBRARY_PATH (typically /usr/local/cuda/lib64).\n",
                    e.what());
            }
        }
#endif

        session = Ort::Session{env, config.model_path.c_str(), session_options};
        std::fprintf(stderr,
            "[ONNXDepthEngine] ============================================\n"
            "[ONNXDepthEngine]   EP    : %s\n"
            "[ONNXDepthEngine]   model : %s\n"
            "[ONNXDepthEngine] ============================================\n",
            cuda_ep_registered ? "GPU (CUDA)" : "CPU  <-- WARNING: running on CPU",
            config.model_path.c_str());
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

    DepthInferenceResult result;
    result.width  = out_w;
    result.height = out_h;
    result.depth_relative.resize(n);

    if (impl_->config.metric_depth) {
        // Metric model (e.g. DepthAnythingV2-Metric-Outdoor) outputs absolute depth
        // in metres, with HIGH value = FAR (not inverse depth).
        //
        // depth_relative convention throughout the codebase: HIGH = CLOSE (like disparity).
        // Downstream formula: depth_m = scale / depth_relative.
        //
        // To satisfy both: store depth_relative = 1 / raw, so that:
        //   depth_m = scale / depth_relative = scale * raw = physical_metres  (scale=1.0)
        //
        // Do NOT store raw directly — that would invert the depth, making far objects
        // appear close and vice versa, causing almost the entire scene to be filtered
        // as too-close (raw ~5-30m → 1/raw ~0.03-0.2m → below min_depth_m=1.0m).
        for (std::size_t i = 0; i < n; ++i) {
            result.depth_relative[i] = 1.0F / std::max(raw[i], 1e-6F);
        }
    } else {
        // Relative model (DepthAnythingV2 default): normalise by per-frame max so
        // the closest pixel gets depth_relative = 1.0.  Set scale = physical distance
        // to that closest pixel (calibrated empirically per scene/drone).
        // NOTE: this destroys absolute depth across frames — use metric_depth: true
        // whenever the model supports it.
        float max_val = 1e-6F;
        for (std::size_t i = 0; i < n; ++i) {
            max_val = std::max(max_val, raw[i]);
        }
        for (std::size_t i = 0; i < n; ++i) {
            result.depth_relative[i] = std::max(raw[i] / max_val, 1e-6F);
        }
    }

    const auto t1 = std::chrono::steady_clock::now();
    result.inference_time_ms = static_cast<float>(
        std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count()) / 1000.0F;
    result.valid = true;

    // Log first inference — confirms which EP is executing and verifies model encoding.
    //
    // Metric model (metric_depth=true): raw = absolute depth in metres, HIGH=FAR.
    //   Expected at drone altitude (5-30 m AGL):
    //     raw min ~0.3 m (props),  max ~60 m (sky/terrain),  mean ~5-20 m  → ✓
    //   BAD signs:
    //     mean < 1.0  → encoding wrong (storing raw as disparity?) or OOD AirSim input
    //     max ≈ min   → model load / EP failure
    //
    // Relative model (metric_depth=false): raw is arbitrary disparity, HIGH=CLOSE.
    //   Expected: raw range [0, ~1] normalised by per-frame max.
    if (!impl_->first_inference_logged) {
        impl_->first_inference_logged = true;
        float raw_min = raw[0], raw_max = raw[0], raw_sum = 0.0F;
        for (std::size_t i = 0; i < n; ++i) {
            if (raw[i] < raw_min) raw_min = raw[i];
            if (raw[i] > raw_max) raw_max = raw[i];
            raw_sum += raw[i];
        }
        std::fprintf(stderr,
            "[ONNXDepthEngine] raw output  min=%.3f  max=%.3f  mean=%.3f  "
            "mode=%s\n",
            static_cast<double>(raw_min),
            static_cast<double>(raw_max),
            static_cast<double>(raw_sum / static_cast<float>(n)),
            impl_->config.metric_depth ? "metric" : "relative");
        const bool looks_like_gpu = result.inference_time_ms < 100.0F;
        std::fprintf(stderr,
            "[ONNXDepthEngine] ============================================\n"
            "[ONNXDepthEngine]   first inference : %.0f ms  [%s]\n"
            "[ONNXDepthEngine] ============================================\n",
            static_cast<double>(result.inference_time_ms),
            looks_like_gpu ? "GPU confirmed"
                           : "WARN: CPU timing - check LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH");
    }

    // Ongoing warn for sustained slow inference.
    if (result.inference_time_ms > 200.0F) {
        std::fprintf(stderr,
            "[ONNXDepthEngine] WARN: inference %.0f ms — likely CPU "
            "(CUDA EP missing libcublasLt.so.12?)\n",
            static_cast<double>(result.inference_time_ms));
    }

    // Debug: write raw model output as a NumPy .npy file when DEDALUS_DEPTH_DEBUG_DIR
    // is set.  Values are the direct model output (metres for metric mode, arbitrary
    // disparity for relative mode) — full float32 precision, no remapping.
    //
    // Load in Python:
    //   import numpy as np
    //   raw = np.load("/tmp/depth_dbg/frame_0001.npy")   # shape (H, W)
    //   depth_m = raw                                     # metric: raw IS metres
    //   print(raw.min(), raw.max(), raw.mean())
    //
    // Filename: <dir>/<frame_id>.npy
    const char* debug_dir = std::getenv("DEDALUS_DEPTH_DEBUG_DIR");
    if (debug_dir && !frame.frame_id.value.empty()) {
        const std::string path =
            std::string{debug_dir} + "/" + frame.frame_id.value + ".npy";
        // NumPy 1.0 format: magic(6) + version(2) + header_len(2) + header + data
        std::string hdr = "{'descr': '<f4', 'fortran_order': False, 'shape': (";
        hdr += std::to_string(out_h) + ", " + std::to_string(out_w) + "), }";
        // Pad so that (10 + header.size()) is a multiple of 64 bytes.
        while ((hdr.size() + 1U + 10U) % 64U != 0U) hdr += ' ';
        hdr += '\n';
        const auto hdr_len = static_cast<std::uint16_t>(hdr.size());
        if (std::ofstream npy{path, std::ios::binary}) {
            npy.write("\x93NUMPY", 6);
            npy.put('\x01'); npy.put('\x00');                    // version 1.0
            npy.put(static_cast<char>(hdr_len & 0xFFU));
            npy.put(static_cast<char>((hdr_len >> 8U) & 0xFFU));
            npy.write(hdr.c_str(), static_cast<std::streamsize>(hdr.size()));
            // Write the raw model output (not depth_relative) for unambiguous inspection.
            npy.write(reinterpret_cast<const char*>(raw),
                      static_cast<std::streamsize>(n * sizeof(float)));
        }
    }

    return result;
}

}  // namespace dedalus
