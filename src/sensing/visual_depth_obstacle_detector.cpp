#include "dedalus/sensing/visual_depth_obstacle_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "dedalus/sensing/depth_projection_kernel.hpp"
#include "dedalus/sensing/visual_depth_frame.hpp"

namespace dedalus {
namespace {

// Extract VisualDepthFrame from EgoSensingFrame.
// Copies image bytes — avoids lifetime coupling to the source frame.
VisualDepthFrame make_visual_depth_frame(const EgoSensingFrame& ego_frame) {
    VisualDepthFrame vdf;
    vdf.timestamp  = ego_frame.frame.timestamp;
    vdf.frame_id   = ego_frame.frame.frame_id;
    vdf.camera_id  = ego_frame.frame.camera_id;
    vdf.map_frame_id = ego_frame.ego.map_frame_id;

    vdf.width    = ego_frame.frame.image.width;
    vdf.height   = ego_frame.frame.image.height;
    vdf.channels = ego_frame.frame.image.channels;
    vdf.bytes    = ego_frame.frame.image.bytes;

    vdf.fx = ego_frame.frame.intrinsics.fx;
    vdf.fy = ego_frame.frame.intrinsics.fy;
    vdf.cx = ego_frame.frame.intrinsics.cx;
    vdf.cy = ego_frame.frame.intrinsics.cy;
    vdf.distortion.k1 = ego_frame.frame.intrinsics.distortion_k1;
    vdf.distortion.k2 = ego_frame.frame.intrinsics.distortion_k2;

    return vdf;
}

}  // namespace

// ---------------------------------------------------------------------------
// VisualDepthObstacleDetector
// ---------------------------------------------------------------------------

VisualDepthObstacleDetector::VisualDepthObstacleDetector(
    std::unique_ptr<DepthEngineInterface> engine,
    MetricScaleEstimate                   scale,
    VisualDepthObstacleDetectorConfig     config)
    : engine_(std::move(engine))
    , scale_(scale)
    , config_(config) {}

VisualDepthObstacleDetector::~VisualDepthObstacleDetector() = default;

std::string VisualDepthObstacleDetector::provider_name() const {
    return "visual_depth_obstacle_detector";
}

std::vector<ObstacleEvidence> VisualDepthObstacleDetector::detect(
    const EgoSensingFrame& ego_frame) {

    const VisualDepthFrame vdf = make_visual_depth_frame(ego_frame);
    const DepthInferenceResult inferred = engine_->infer(vdf);
    if (!inferred.valid || inferred.inverse_depth.empty()) {
        last_has_result_ = false;
        return {};
    }

    // Camera pitch: NED convention, forward_axis_local.z > 0 means looking downward.
    const float fwd_z = static_cast<float>(ego_frame.sensing_volume.forward_axis_local.z);
    const float pitch_down_deg = std::asin(std::clamp(fwd_z, -1.0f, 1.0f))
                                 * (180.0f / 3.14159265f);

    // ── Build DepthPipelineInput ──────────────────────────────────────────
    DepthPipelineInput input;
    input.inverse_depth = inferred.inverse_depth;  // already disparity convention
    input.width  = inferred.width;
    input.height = inferred.height;
    input.scale  = scale_.scale;

    // FoV-based intrinsics, matching the GT provider convention:
    //   cx = (W-1)/2, fx = cx / tan(hfov/2)
    // Uses the ONNX output dimensions and sensing volume FoV directly so that
    // the pixel-to-bearing mapping agrees with airsim_gt_vd regardless of the
    // ONNX resize ratio or frame.intrinsics source.
    const float hfov = static_cast<float>(ego_frame.sensing_volume.horizontal_fov_rad);
    const float vfov = static_cast<float>(ego_frame.sensing_volume.vertical_fov_rad);
    input.cx = (static_cast<float>(inferred.width)  - 1.0F) * 0.5F;
    input.cy = (static_cast<float>(inferred.height) - 1.0F) * 0.5F;
    input.fx = input.cx / std::tan(hfov * 0.5F);
    input.fy = input.cy / std::tan(vfov * 0.5F);
    input.k1 = static_cast<float>(ego_frame.frame.intrinsics.distortion_k1);
    input.k2 = static_cast<float>(ego_frame.frame.intrinsics.distortion_k2);

    input.source_kind     = OccupancySourceKind::VisualObstacleDetector;
    input.sensor_name     = ego_frame.sensing_volume.camera_name;
    input.source_provider = provider_name();

    last_input_       = input;
    last_pitch_deg_   = pitch_down_deg;
    last_has_result_  = true;

    // ── Provider-side diagnostics ─────────────────────────────────────────

    // Depth histogram — every 30 frames.
    static int diag_frame_count = 0;
    if (++diag_frame_count % 30 == 1) {
        const int npix = inferred.width * inferred.height;
        int b0 = 0, b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0;
        for (int pi = 0; pi < npix; ++pi) {
            const float dr = inferred.inverse_depth[pi];
            float dm = 120.0f;
            if (std::isfinite(dr) && dr > 1e-6f) dm = scale_.scale / dr;
            if      (dm <  0.5f) ++b0;
            else if (dm <  1.0f) ++b1;
            else if (dm <  5.0f) ++b2;
            else if (dm < 20.0f) ++b3;
            else if (dm < 60.0f) ++b4;
            else                  ++b5;
        }
        const float pct = 100.0f / static_cast<float>(npix);
        std::fprintf(stderr,
            "[DepthDebug] frame=%d pitch=%.1f°  depth_m bins: "
            "<0.5m=%.0f%%  0.5-1m=%.0f%%  1-5m=%.0f%%  "
            "5-20m=%.0f%%  20-60m=%.0f%%  >60m=%.0f%%  "
            "[filter %.1f..%.1f m]\n",
            diag_frame_count, pitch_down_deg,
            b0 * pct, b1 * pct, b2 * pct, b3 * pct, b4 * pct, b5 * pct,
            config_.min_depth_m, config_.max_depth_m);
    }

    // Gimbal-gated range-error frame capture.
    //
    // When the camera is tilted > 15° downward nothing physical can be < min_depth_m
    // from the drone.  If > 5% of pixels are predicted below that floor the model is
    // producing OOD noise.  Set DEDALUS_DEPTH_CAPTURE_DIR=/tmp/depth_errors to save
    // depth + RGB .npy pairs for each such frame; load with probe_depth_frame0.py.
    const char* capture_dir = std::getenv("DEDALUS_DEPTH_CAPTURE_DIR");
    constexpr float k_capture_pitch_deg = 15.0f;
    constexpr float k_capture_close_pct = 5.0f;
    if (capture_dir != nullptr && pitch_down_deg > k_capture_pitch_deg) {
        const int npix = inferred.width * inferred.height;
        int close_count = 0;
        for (int pi = 0; pi < npix; ++pi) {
            const float dr = inferred.inverse_depth[pi];
            if (std::isfinite(dr) && dr > 1e-6f
                    && (scale_.scale / dr) < config_.min_depth_m) {
                ++close_count;
            }
        }
        const float close_pct = 100.0f * static_cast<float>(close_count)
                                / static_cast<float>(npix);
        if (close_pct > k_capture_close_pct) {
            // Rate limiting: max 10 captures per run, minimum 60 frames apart.
            constexpr int k_max_captures    = 10;
            constexpr int k_capture_interval = 60;
            static int capture_idx          = 0;
            static int frames_since_capture = k_capture_interval;  // allow first immediately
            ++frames_since_capture;
            if (capture_idx >= k_max_captures || frames_since_capture < k_capture_interval) {
                // Skip — either hit the cap or too soon since last capture.
            } else {
            ++capture_idx;
            frames_since_capture = 0;
            const std::string base = std::string(capture_dir)
                + "/cap_" + std::to_string(capture_idx)
                + "_p"    + std::to_string(static_cast<int>(pitch_down_deg))
                + "_c"    + std::to_string(static_cast<int>(close_pct));

            // Minimal NPY writer (float32, shape H×W).
            auto write_npy_f32 = [&](const std::string& path,
                                     const float* data, int rows, int cols) {
                FILE* f = std::fopen(path.c_str(), "wb");
                if (!f) return;
                char hdr_buf[256];
                const int hdr_len = std::snprintf(hdr_buf, sizeof(hdr_buf),
                    "{'descr': '<f4', 'fortran_order': False, 'shape': (%d, %d), }",
                    rows, cols);
                const std::uint8_t magic[] = {0x93,'N','U','M','P','Y',0x01,0x00};
                const int needed = 10 + hdr_len + 1;
                const int pad    = (64 - (needed % 64)) % 64;
                const std::uint16_t hdr_size =
                    static_cast<std::uint16_t>(hdr_len + pad + 1);
                std::fwrite(magic, 1, 8, f);
                std::fwrite(&hdr_size, 2, 1, f);
                std::fwrite(hdr_buf, 1, static_cast<std::size_t>(hdr_len), f);
                for (int i = 0; i < pad; ++i) std::fputc(' ', f);
                std::fputc('\n', f);
                std::fwrite(data, sizeof(float),
                            static_cast<std::size_t>(rows * cols), f);
                std::fclose(f);
            };

            // Minimal NPY writer (uint8, shape H×W×C).
            auto write_npy_u8 = [&](const std::string& path,
                                    const std::uint8_t* data,
                                    int rows, int cols, int ch) {
                FILE* f = std::fopen(path.c_str(), "wb");
                if (!f) return;
                char hdr_buf[256];
                const int hdr_len = std::snprintf(hdr_buf, sizeof(hdr_buf),
                    "{'descr': '|u1', 'fortran_order': False, 'shape': (%d, %d, %d), }",
                    rows, cols, ch);
                const std::uint8_t magic[] = {0x93,'N','U','M','P','Y',0x01,0x00};
                const int needed = 10 + hdr_len + 1;
                const int pad    = (64 - (needed % 64)) % 64;
                const std::uint16_t hdr_size =
                    static_cast<std::uint16_t>(hdr_len + pad + 1);
                std::fwrite(magic, 1, 8, f);
                std::fwrite(&hdr_size, 2, 1, f);
                std::fwrite(hdr_buf, 1, static_cast<std::size_t>(hdr_len), f);
                for (int i = 0; i < pad; ++i) std::fputc(' ', f);
                std::fputc('\n', f);
                std::fwrite(data, 1,
                            static_cast<std::size_t>(rows * cols * ch), f);
                std::fclose(f);
            };

            write_npy_f32(base + "_depth.npy",
                          inferred.inverse_depth.data(),
                          inferred.height, inferred.width);
            write_npy_u8(base + "_rgb.npy",
                         vdf.bytes.data(),
                         vdf.height, vdf.width, vdf.channels);

            std::fprintf(stderr,
                "[DepthCapture] %d/%d pitch=%.1f° close=%.0f%% → %s_{depth,rgb}.npy\n",
                capture_idx, k_max_captures, pitch_down_deg, close_pct, base.c_str());
            }  // else (rate limit passed)
        }
    }

    // ── Delegate all downstream processing to the shared pipeline ─────────
    const DepthPipelineConfig cfg{
        static_cast<int>(config_.depth_grid_cols),
        static_cast<int>(config_.depth_grid_rows),
        config_.min_depth_m,
        config_.max_depth_m,
        config_.voxel_size_m,
        config_.max_evidence,
        config_.detect_surface_patches,
        config_.detect_thin_structures,
    };

    return run_depth_pipeline(input, ego_frame, cfg, &last_params_);
}

// ---------------------------------------------------------------------------
// Free-function variant (test path — no class instance required).
// ---------------------------------------------------------------------------

std::vector<ObstacleEvidence> detect_visual_depth_obstacles(
    const EgoSensingFrame&                   ego_frame,
    DepthEngineInterface&                    engine,
    const MetricScaleEstimate&               scale,
    const VisualDepthObstacleDetectorConfig& cfg) {

    VisualDepthFrame vdf = make_visual_depth_frame(ego_frame);
    const DepthInferenceResult inferred = engine.infer(vdf);
    if (!inferred.valid || inferred.inverse_depth.empty()) return {};

    DepthPipelineInput input;
    input.inverse_depth = inferred.inverse_depth;
    input.width  = inferred.width;
    input.height = inferred.height;
    input.scale  = scale.scale;

    const float hfov = static_cast<float>(ego_frame.sensing_volume.horizontal_fov_rad);
    const float vfov = static_cast<float>(ego_frame.sensing_volume.vertical_fov_rad);
    input.cx = (static_cast<float>(inferred.width)  - 1.0F) * 0.5F;
    input.cy = (static_cast<float>(inferred.height) - 1.0F) * 0.5F;
    input.fx = input.cx / std::tan(hfov * 0.5F);
    input.fy = input.cy / std::tan(vfov * 0.5F);
    input.k1 = static_cast<float>(ego_frame.frame.intrinsics.distortion_k1);
    input.k2 = static_cast<float>(ego_frame.frame.intrinsics.distortion_k2);

    input.source_kind     = OccupancySourceKind::VisualObstacleDetector;
    input.sensor_name     = ego_frame.sensing_volume.camera_name;
    input.source_provider = "visual_depth_obstacle_detector";

    const DepthPipelineConfig pcfg{
        static_cast<int>(cfg.depth_grid_cols),
        static_cast<int>(cfg.depth_grid_rows),
        cfg.min_depth_m,
        cfg.max_depth_m,
        cfg.voxel_size_m,
        cfg.max_evidence,
        cfg.detect_surface_patches,
        cfg.detect_thin_structures,
    };

    return run_depth_pipeline(input, ego_frame, pcfg);
}

}  // namespace dedalus
