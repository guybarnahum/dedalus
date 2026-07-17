#include "dedalus/sensing/onnx_depth_engine_base.hpp"

// OnnxDepthEngineBase requires ONNX Runtime.
// Only compiled when DEDALUS_ENABLE_ONNX_DEPTH=ON.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace dedalus {

struct OnnxDepthEngineBase::Impl {
    Ort::Env            env;
    Ort::SessionOptions session_options;
    Ort::Session        session{nullptr};

    bool cuda_ep_registered     = false;
    bool first_inference_done   = false;
    const char* log_tag;

    Impl(const SessionConfig& cfg, const char* tag) :
        env{ORT_LOGGING_LEVEL_WARNING, tag},
        log_tag{tag}
    {
        session_options.SetIntraOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(
            GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

#if defined(__APPLE__)
        if (cfg.use_coreml) {
            uint32_t coreml_flags = 0;
            OrtSessionOptionsAppendExecutionProvider_CoreML(
                session_options, coreml_flags);
        }
#endif

#if defined(DEDALUS_CUDA_ENABLED)
        if (cfg.use_cuda) {
            try {
                OrtCUDAProviderOptionsV2* cuda_opts = nullptr;
                Ort::ThrowOnError(Ort::GetApi().CreateCUDAProviderOptions(&cuda_opts));
                const std::string dev_id_str = std::to_string(cfg.cuda_device_id);
                const std::string arena_str  = std::to_string(cfg.cuda_arena_limit_bytes);
                const char* keys[]   = {"device_id", "gpu_mem_limit"};
                const char* values[] = {dev_id_str.c_str(), arena_str.c_str()};
                Ort::ThrowOnError(Ort::GetApi().UpdateCUDAProviderOptions(
                    cuda_opts, keys, values, 2));
                Ort::ThrowOnError(Ort::GetApi().SessionOptionsAppendExecutionProvider_CUDA_V2(
                    session_options, cuda_opts));
                Ort::GetApi().ReleaseCUDAProviderOptions(cuda_opts);
                cuda_ep_registered = true;
                std::fprintf(stderr,
                    "[%s] CUDA EP registered (device_id=%d, arena_limit=%zu bytes)\n",
                    tag, cfg.cuda_device_id,
                    static_cast<std::size_t>(cfg.cuda_arena_limit_bytes));
            } catch (const Ort::Exception& e) {
                std::fprintf(stderr,
                    "[%s] WARN: CUDA EP registration failed (%s); falling back to CPU.\n",
                    tag, e.what());
            }
        }
#endif

        session = Ort::Session{env, cfg.model_path.c_str(), session_options};
        std::fprintf(stderr,
            "[%s] ============================================\n"
            "[%s]   EP    : %s\n"
            "[%s]   model : %s\n"
            "[%s] ============================================\n",
            tag, tag,
            cuda_ep_registered ? "GPU (CUDA)" : "CPU  <-- WARNING: running on CPU",
            tag, cfg.model_path.c_str(),
            tag);
    }
};

OnnxDepthEngineBase::OnnxDepthEngineBase(SessionConfig cfg, const char* log_tag)
    : impl_(std::make_unique<Impl>(cfg, log_tag)) {}

OnnxDepthEngineBase::~OnnxDepthEngineBase() = default;

bool OnnxDepthEngineBase::cuda_ep_active() const {
    return impl_->cuda_ep_registered;
}

DepthInferenceResult OnnxDepthEngineBase::infer(const VisualDepthFrame& frame) {
    if (frame.bytes.empty() || frame.width <= 0 || frame.height <= 0) {
        return {};
    }

    // --- Preprocess: frame → ONNX input tensors ---
    OnnxInputs inputs = prepare_inputs(frame);

    // Build c_str() pointer arrays for session.Run().
    std::vector<const char*> in_name_ptrs;
    in_name_ptrs.reserve(inputs.names.size());
    for (const auto& n : inputs.names) in_name_ptrs.push_back(n.c_str());

    const auto out_specs = output_names();
    std::vector<const char*> out_name_ptrs;
    out_name_ptrs.reserve(out_specs.size());
    for (const auto& n : out_specs) out_name_ptrs.push_back(n.c_str());

    // --- Run session ---
    const auto t0 = std::chrono::steady_clock::now();

    auto outputs = impl_->session.Run(
        Ort::RunOptions{nullptr},
        in_name_ptrs.data(),  inputs.values.data(),  inputs.names.size(),
        out_name_ptrs.data(), out_specs.size());

    const auto t1 = std::chrono::steady_clock::now();
    const float inference_time_ms = static_cast<float>(
        std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count()) / 1000.0F;

    // --- Postprocess: raw outputs → DepthInferenceResult ---
    const bool is_first = !impl_->first_inference_done;
    impl_->first_inference_done = true;

    DepthInferenceResult result = extract_result(frame, outputs, inference_time_ms, is_first);

    // --- Common logging ---
    if (is_first) {
        const bool looks_like_gpu = inference_time_ms < 100.0F;
        std::fprintf(stderr,
            "[%s] ============================================\n"
            "[%s]   first inference : %.0f ms  [%s]\n"
            "[%s] ============================================\n",
            impl_->log_tag,
            impl_->log_tag, static_cast<double>(inference_time_ms),
            looks_like_gpu ? "GPU confirmed"
                           : "WARN: CPU timing — check LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH",
            impl_->log_tag);
    }

    if (inference_time_ms > 200.0F) {
        std::fprintf(stderr,
            "[%s] WARN: inference %.0f ms — likely CPU "
            "(CUDA EP missing libcublasLt.so.12?)\n",
            impl_->log_tag, static_cast<double>(inference_time_ms));
    }

    return result;
}

}  // namespace dedalus
