#include "dedalus/sensing/unidepth_v2_depth_engine.hpp"

// UniDepthV2DepthEngine — UniDepth V2 subclass of OnnxDepthEngineBase.
// Only compiled when DEDALUS_ENABLE_ONNX_DEPTH=ON.

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include "dedalus/sensing/depth_engine_utils.hpp"

namespace dedalus {

UniDepthV2DepthEngine::UniDepthV2DepthEngine(UniDepthV2DepthEngineConfig config)
    : OnnxDepthEngineBase(
          SessionConfig{
              .model_path             = config.model_path,
              .use_cuda               = config.use_cuda,
              .cuda_device_id         = config.cuda_device_id,
              .cuda_arena_limit_bytes = config.cuda_arena_limit_bytes,
              .use_coreml             = config.use_coreml,
          },
          "UniDepthV2DepthEngine"),
      config_(std::move(config)) {}

UniDepthV2DepthEngine::~UniDepthV2DepthEngine() = default;

std::string UniDepthV2DepthEngine::engine_name() const {
    return "unidepth_v2_depth_engine";
}

// --- Camera ray helper ---

std::vector<float> UniDepthV2DepthEngine::build_camera_rays(
    double fx, double fy, double cx, double cy,
    int native_w, int native_h,
    int dst_w,   int dst_h) {

    // Scale intrinsics from native → inference resolution.
    const double scale_x = static_cast<double>(dst_w) / static_cast<double>(native_w);
    const double scale_y = static_cast<double>(dst_h) / static_cast<double>(native_h);
    const double fx_s = fx * scale_x;
    const double fy_s = fy * scale_y;
    const double cx_s = cx * scale_x;
    const double cy_s = cy * scale_y;

    std::vector<float> rays(static_cast<std::size_t>(3 * dst_h * dst_w), 0.0F);
    const std::size_t hw = static_cast<std::size_t>(dst_h * dst_w);

    for (int dy = 0; dy < dst_h; ++dy) {
        for (int dx = 0; dx < dst_w; ++dx) {
            const double rx = (static_cast<double>(dx) - cx_s) / fx_s;
            const double ry = (static_cast<double>(dy) - cy_s) / fy_s;
            const double rz = 1.0;
            const double norm = std::sqrt(rx * rx + ry * ry + rz * rz);
            const std::size_t idx = static_cast<std::size_t>(dy * dst_w + dx);
            rays[0 * hw + idx] = static_cast<float>(rx / norm);
            rays[1 * hw + idx] = static_cast<float>(ry / norm);
            rays[2 * hw + idx] = static_cast<float>(rz / norm);
        }
    }
    return rays;
}

// --- Input translation ---

OnnxDepthEngineBase::OnnxInputs UniDepthV2DepthEngine::prepare_inputs(
    const VisualDepthFrame& frame) {

    const int inf_w = config_.inference_width;
    const int inf_h = config_.inference_height;

    OnnxInputs inputs;
    inputs.names = {"rgbs"};
    inputs.buffers.push_back(depth_engine_utils::resize_and_normalise(
        frame.bytes, frame.width, frame.height, inf_w, inf_h,
        0.485F, 0.456F, 0.406F,   // ImageNet mean (R, G, B)
        0.229F, 0.224F, 0.225F)); // ImageNet std

    const std::array<std::int64_t, 4> shape{1, 3, inf_h, inf_w};
    Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    inputs.values.push_back(Ort::Value::CreateTensor<float>(
        mem, inputs.buffers[0].data(), inputs.buffers[0].size(),
        shape.data(), shape.size()));

    if (config_.use_camera_rays && frame.fx > 0.0 && frame.fy > 0.0) {
        inputs.names.push_back("rays");
        inputs.buffers.push_back(build_camera_rays(
            frame.fx, frame.fy, frame.cx, frame.cy,
            frame.width, frame.height, inf_w, inf_h));
        inputs.values.push_back(Ort::Value::CreateTensor<float>(
            mem, inputs.buffers[1].data(), inputs.buffers[1].size(),
            shape.data(), shape.size()));
    }

    return inputs;
}

std::vector<std::string> UniDepthV2DepthEngine::output_names() const {
    return {"pts_3d"};
}

// --- Output translation ---

DepthInferenceResult UniDepthV2DepthEngine::extract_result(
    const VisualDepthFrame& frame,
    std::vector<Ort::Value>& outputs,
    float inference_time_ms,
    bool is_first) {

    // pts_3d layout: [1, 3, H, W] — channel 0=X, 1=Y, 2=Z (forward metric depth, metres).
    // Tolerate [3, H, W] if the exporter drops the batch dim.
    const float* pts3d = outputs[0].GetTensorData<float>();
    const auto   info  = outputs[0].GetTensorTypeAndShapeInfo();
    const auto   shape = info.GetShape();

    const int out_h = static_cast<int>(shape[shape.size() - 2]);
    const int out_w = static_cast<int>(shape[shape.size() - 1]);
    const std::size_t hw = static_cast<std::size_t>(out_h * out_w);

    // Z channel: offset 2*hw in the flattened NCHW buffer.
    const float* z_channel = pts3d + 2U * hw;

    // Downsample Z to native resolution so downstream consumers see 256×144.
    const int nat_w = config_.native_width;
    const int nat_h = config_.native_height;

    std::vector<float> z_native;
    const float* z_ptr = z_channel;
    int z_w = out_w, z_h = out_h;

    if (out_w != nat_w || out_h != nat_h) {
        z_native = depth_engine_utils::downsample(z_channel, out_w, out_h, nat_w, nat_h);
        z_ptr    = z_native.data();
        z_w      = nat_w;
        z_h      = nat_h;
    }

    // Convert metric Z → inverse_depth (pipeline convention: HIGH=CLOSE).
    static constexpr float kMinValidZ = 0.01F;
    const float scale = config_.scale > 0.0F ? config_.scale : 1.0F;
    const std::size_t n = static_cast<std::size_t>(z_h * z_w);

    DepthInferenceResult result;
    result.width             = z_w;
    result.height            = z_h;
    result.inference_time_ms = inference_time_ms;
    result.inverse_depth.resize(n);

    for (std::size_t i = 0; i < n; ++i) {
        result.inverse_depth[i] = (z_ptr[i] >= kMinValidZ) ? (scale / z_ptr[i]) : 0.0F;
    }
    result.valid = true;

    // First-inference: log Z channel stats.
    // Expected at 5-30 m AGL: mean Z ~10..30 m.
    // BAD: max Z ≈ 0 → model failure; all Z < 0.1 → wrong channel offset; mean > 60 → all sky.
    if (is_first) {
        float z_min = z_ptr[0], z_max = z_ptr[0], z_sum = 0.0F;
        for (std::size_t i = 0; i < n; ++i) {
            if (z_ptr[i] < z_min) z_min = z_ptr[i];
            if (z_ptr[i] > z_max) z_max = z_ptr[i];
            z_sum += z_ptr[i];
        }
        std::fprintf(stderr,
            "[UniDepthV2DepthEngine] Z (pts_3d ch2) min=%.3f  max=%.3f  mean=%.3f m\n",
            static_cast<double>(z_min),
            static_cast<double>(z_max),
            static_cast<double>(z_sum / static_cast<float>(n)));

        // Dump native-resolution Z map + input RGB for offline analysis.
        depth_engine_utils::dump_npy_f32("/tmp/dedalus_frame0_depth.npy", z_ptr, z_h, z_w);
        if (!frame.bytes.empty() && frame.channels > 0) {
            depth_engine_utils::dump_npy_u8("/tmp/dedalus_frame0_rgb.npy",
                frame.bytes.data(), frame.height, frame.width, frame.channels);
        }
    }

    depth_engine_utils::maybe_write_debug_npy(
        std::getenv("DEDALUS_DEPTH_DEBUG_DIR"), frame.frame_id.value, z_ptr, z_h, z_w);

    return result;
}

}  // namespace dedalus
