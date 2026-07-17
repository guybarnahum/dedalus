#pragma once

// OnnxDepthEngineBase — shared infrastructure for ONNX-backed depth engines.
//
// Handles: ORT session construction, CUDA/CoreML EP registration, the infer()
// orchestration loop (timing, first-inference logging, slow-inference warn).
//
// Subclasses implement three hooks:
//   prepare_inputs()  — frame → ONNX input tensors
//   output_names()    — which ONNX output names to fetch
//   extract_result()  — raw ONNX outputs → DepthInferenceResult
//
// Only compiled when DEDALUS_ENABLE_ONNX_DEPTH=ON.

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include "dedalus/sensing/depth_engine.hpp"
#include "dedalus/sensing/visual_depth_frame.hpp"

namespace dedalus {

class OnnxDepthEngineBase : public DepthEngineInterface {
public:
    struct SessionConfig {
        std::string model_path;
        bool        use_cuda{false};
        int         cuda_device_id{0};
        std::size_t cuda_arena_limit_bytes{1ULL * 1024 * 1024 * 1024};
        bool        use_coreml{false};
    };

    explicit OnnxDepthEngineBase(SessionConfig cfg, const char* log_tag);
    ~OnnxDepthEngineBase() override;

    OnnxDepthEngineBase(const OnnxDepthEngineBase&)            = delete;
    OnnxDepthEngineBase& operator=(const OnnxDepthEngineBase&) = delete;

    // Orchestrates prepare_inputs → session.Run → extract_result → logging.
    // Subclasses must NOT override this — they implement the three hooks below.
    [[nodiscard]] DepthInferenceResult infer(const VisualDepthFrame& frame) final;

protected:
    // Inputs to the ONNX session for one frame.
    // buffers owns the float data backing each Ort::Value (non-owning view semantics).
    // OnnxInputs stays alive through session.Run() so the pointers remain valid.
    struct OnnxInputs {
        std::vector<std::string>        names;    // must match model input order
        std::vector<std::vector<float>> buffers;  // backing stores for values[]
        std::vector<Ort::Value>         values;
    };

    // Subclass: build ONNX input tensors from the frame.
    virtual OnnxInputs prepare_inputs(const VisualDepthFrame& frame) = 0;

    // Subclass: return the output name(s) to fetch from the session.
    virtual std::vector<std::string> output_names() const = 0;

    // Subclass: convert raw ONNX outputs → DepthInferenceResult.
    // Called once per frame, after session.Run().  inference_time_ms is the
    // wall-clock time of the Run() call (not including preprocess/postprocess).
    // is_first is true on the first call — use it to log model-specific stats
    // (e.g. raw value range) and dump frame-0 npy files.
    virtual DepthInferenceResult extract_result(
        const VisualDepthFrame& frame,
        std::vector<Ort::Value>& outputs,
        float inference_time_ms,
        bool is_first) = 0;

    bool cuda_ep_active() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace dedalus
