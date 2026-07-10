#include "dedalus/sensing/visual_depth_obstacle_detector.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "dedalus/sensing/depth_projection_kernel.hpp"
#include "dedalus/sensing/visual_depth_frame.hpp"

#ifdef DEDALUS_CUDA_ENABLED
#include "dedalus/sensing/cuda_depth_kernels.hpp"
#endif

namespace dedalus {
namespace {

#ifdef DEDALUS_CUDA_ENABLED
// File-static singleton dispatcher — one CudaDepthDispatcher per process.
// Safe: VisualDepthObstacleDetector is single-threaded (see class comment).
CudaDepthDispatcher& cuda_dispatch() {
    static CudaDepthDispatcher s;
    return s;
}
#endif

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

// Build ProjectionParams from EgoSensingFrame fields + detector config.
ProjectionParams make_projection_params(
    const EgoSensingFrame&                   ego_frame,
    const DepthInferenceResult&              inferred,
    const MetricScaleEstimate&               scale,
    const VisualDepthObstacleDetectorConfig& cfg) {

    const auto& sv = ego_frame.sensing_volume;
    ProjectionParams p;

    // Intrinsics from the source frame, scaled to the ONNX output resolution.
    //
    // The ONNX model resizes the input (e.g. 640×360) to its native square
    // input (518×518) before inference, producing a 518×518 depth map.  The
    // resize is non-uniform (different scale factors in x and y) because the
    // camera aspect ratio ≠ 1.  Back-projecting with the original intrinsics
    // on the resized depth map gives wrong 3D positions: the principal point
    // is off-centre and the focal lengths are wrong for the output resolution.
    //
    // Scaling fx/fy/cx/cy by the per-axis resize ratio corrects this: a squash
    // of the horizontal axis by s_x means apparent horizontal angles shrink by
    // s_x, so fx must shrink by the same factor for tan(θ) = (u-cx)/fx to
    // recover the original bearing.
    const float s_x = (ego_frame.frame.image.width  > 0 && inferred.width  > 0)
        ? static_cast<float>(inferred.width)  / static_cast<float>(ego_frame.frame.image.width)
        : 1.0F;
    const float s_y = (ego_frame.frame.image.height > 0 && inferred.height > 0)
        ? static_cast<float>(inferred.height) / static_cast<float>(ego_frame.frame.image.height)
        : 1.0F;

    p.fx = static_cast<float>(ego_frame.frame.intrinsics.fx) * s_x;
    p.fy = static_cast<float>(ego_frame.frame.intrinsics.fy) * s_y;
    p.cx = static_cast<float>(ego_frame.frame.intrinsics.cx) * s_x;
    p.cy = static_cast<float>(ego_frame.frame.intrinsics.cy) * s_y;
    // Distortion coefficients were measured in the original pixel space; they
    // apply to normalised coordinates (u-cx)/fx and are dimensionless, so they
    // do not need rescaling.
    p.k1 = static_cast<float>(ego_frame.frame.intrinsics.distortion_k1);
    p.k2 = static_cast<float>(ego_frame.frame.intrinsics.distortion_k2);

    // Depth map dimensions (may differ from image if model rescales)
    p.width  = inferred.width;
    p.height = inferred.height;
    p.stride = static_cast<int>(cfg.pixel_stride);

    p.min_depth_m  = cfg.min_depth_m;
    p.max_depth_m  = cfg.max_depth_m;
    p.scale        = scale.scale;
    p.voxel_size_m = cfg.voxel_size_m;
    p.max_evidence = static_cast<std::uint32_t>(cfg.max_evidence);

    // Gimbal-corrected sensing volume axes (encoder reading at frame timestamp)
    p.origin_x  = static_cast<float>(sv.origin_local.x);
    p.origin_y  = static_cast<float>(sv.origin_local.y);
    p.origin_z  = static_cast<float>(sv.origin_local.z);
    p.forward_x = static_cast<float>(sv.forward_axis_local.x);
    p.forward_y = static_cast<float>(sv.forward_axis_local.y);
    p.forward_z = static_cast<float>(sv.forward_axis_local.z);
    p.right_x   = static_cast<float>(sv.right_axis_local.x);
    p.right_y   = static_cast<float>(sv.right_axis_local.y);
    p.right_z   = static_cast<float>(sv.right_axis_local.z);
    p.up_x      = static_cast<float>(sv.up_axis_local.x);
    p.up_y      = static_cast<float>(sv.up_axis_local.y);
    p.up_z      = static_cast<float>(sv.up_axis_local.z);

    return p;
}

// Project a validated DepthInferenceResult to ObstacleEvidence.
// Shared between the class method (which also taps the result for debug) and
// the free function (test path, no debug pipe).
std::vector<ObstacleEvidence> project_from_inferred(
    const EgoSensingFrame&                   ego_frame,
    const DepthInferenceResult&              inferred,
    const MetricScaleEstimate&               scale,
    const VisualDepthObstacleDetectorConfig& cfg) {

    const ProjectionParams params = make_projection_params(ego_frame, inferred, scale, cfg);

    // Allocate evidence buffer (shared across projection + surface + thin)
    std::vector<DeviceObstacleEvidence> buf(cfg.max_evidence);
    std::uint32_t count = 0U;

#ifdef DEDALUS_CUDA_ENABLED
    const bool cdebug = (std::getenv("DEDALUS_MISSION_DEBUG") != nullptr);
    if (cdebug) { std::fprintf(stderr, "[CudaDepth] project...\n"); std::fflush(stderr); }
    cuda_dispatch().project(inferred.inverse_depth.data(), params, buf.data(), count);
    if (cdebug) { std::fprintf(stderr, "[CudaDepth] project done (%u ev)\n", count); std::fflush(stderr); }
#else
    project_depth_to_device_evidence(
        inferred.inverse_depth.data(), params, buf.data(), count);
#endif

    if (cfg.detect_surface_patches && count > 0U) {
        std::vector<DeviceObstacleEvidence> patches(64U);
        std::uint32_t patch_count = 0U;
#ifdef DEDALUS_CUDA_ENABLED
        if (cdebug) { std::fprintf(stderr, "[CudaDepth] fit_patches...\n"); std::fflush(stderr); }
        cuda_dispatch().fit_patches(buf.data(), count, params, patches.data(), patch_count);
        if (cdebug) { std::fprintf(stderr, "[CudaDepth] fit_patches done (%u patches)\n", patch_count); std::fflush(stderr); }
#else
        fit_surface_patches_device(
            buf.data(), count, params, patches.data(), patch_count);
#endif
        for (std::uint32_t i = 0U; i < patch_count && count < cfg.max_evidence; ++i) {
            buf[count++] = patches[i];
        }
    }

    if (cfg.detect_thin_structures) {
        std::vector<DeviceObstacleEvidence> thin(64U);
        std::uint32_t thin_count = 0U;
#ifdef DEDALUS_CUDA_ENABLED
        if (cdebug) { std::fprintf(stderr, "[CudaDepth] detect_thin...\n"); std::fflush(stderr); }
        cuda_dispatch().detect_thin(
            inferred.inverse_depth.data(), params, thin.data(), thin_count);
        if (cdebug) { std::fprintf(stderr, "[CudaDepth] detect_thin done (%u thin)\n", thin_count); std::fflush(stderr); }
#else
        detect_thin_structures_device(
            inferred.inverse_depth.data(), params, thin.data(), thin_count);
#endif
        for (std::uint32_t i = 0U; i < thin_count && count < cfg.max_evidence; ++i) {
            buf[count++] = thin[i];
        }
    }

    auto result = inflate(
        buf.data(),
        count,
        ego_frame.sensing_volume.camera_name,
        "visual_depth_obstacle_detector",
        ego_frame.ego.map_frame_id,
        ego_frame.frame.timestamp);

    // Compute body-frame bearing and elevation from projected local positions.
    // Uses the sensing volume axes (forward/right/up in local frame) — the same
    // geometry the AirSim GT path uses — so L0 sensor observations are populated.
    const auto& sv = ego_frame.sensing_volume;
    for (auto& ev : result) {
        const double dx = ev.center_local.x - sv.origin_local.x;
        const double dy = ev.center_local.y - sv.origin_local.y;
        const double dz = ev.center_local.z - sv.origin_local.z;
        const double fwd   = dx*sv.forward_axis_local.x + dy*sv.forward_axis_local.y + dz*sv.forward_axis_local.z;
        const double right = dx*sv.right_axis_local.x   + dy*sv.right_axis_local.y   + dz*sv.right_axis_local.z;
        const double up    = dx*sv.up_axis_local.x      + dy*sv.up_axis_local.y      + dz*sv.up_axis_local.z;
        ev.bearing_rad   = static_cast<float>(std::atan2(right, fwd));
        ev.elevation_rad = static_cast<float>(std::atan2(up, std::hypot(fwd, right)));
    }

    return result;
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

VisualDepthObstacleDetector::~VisualDepthObstacleDetector() {
    if (debug_pipe_ != nullptr) {
        const int rc = pclose(debug_pipe_);
        if (rc != 0) {
            std::fprintf(stderr, "[DepthDebug] depth_mp4 ffmpeg exited with code %d\n", rc);
        }
        debug_pipe_ = nullptr;
    }
}

std::string VisualDepthObstacleDetector::provider_name() const {
    return "visual_depth_obstacle_detector";
}

void VisualDepthObstacleDetector::write_debug_frame(
    const DepthInferenceResult& inferred,
    const ProjectionParams&     params,
    float                       pitch_down_deg) {

    if (inferred.width <= 0 || inferred.height <= 0 || inferred.inverse_depth.empty()) return;

    const int      W = inferred.width;
    const int      H = inferred.height;

    // Side-by-side: left panel = raw model output, right panel = evidence filter view.
    // ffmpeg output width is 2*W.
    if (debug_pipe_ == nullptr) {
        const std::string cmd =
            "ffmpeg -f rawvideo -pixel_format rgb24"
            " -video_size " + std::to_string(2 * W) + "x" + std::to_string(H) +
            " -framerate 5 -i pipe:0"
            " -vcodec libx264 -crf 23 -pix_fmt yuv420p"
            " -movflags frag_keyframe+empty_moov+default_base_moof"
            " -y " + config_.debug_depth_mp4 +
            " 2>/dev/null";
        std::fprintf(stderr, "[DepthDebug] opening depth_mp4 pipe: %s\n", cmd.c_str());
        debug_pipe_ = popen(cmd.c_str(), "w");
        if (debug_pipe_ == nullptr) {
            std::fprintf(stderr, "[DepthDebug] ERROR: popen failed (errno=%d: %s) — depth MP4 disabled\n",
                         errno, std::strerror(errno));
            return;
        }
    }

    // inverse_depth = 1/depth_m (inverse depth, stored by engine).
    // depth_m = scale / inverse_depth = physical metres.
    //
    // Display scale: 30 m anchor with log compression.
    // Linear 60 m compresses everything < 10 m into the top 17% of the range,
    // making the whole scene look uniformly white.  Log scale gives perceptually
    // uniform contrast across the 0.1–30 m obstacle-avoidance range:
    //   0.1 m (props)  → lum ≈ 252  (nearly white)
    //   2 m            → lum ≈ 178  (light grey)
    //   5 m            → lum ≈ 127  (medium grey)
    //   10 m           → lum ≈  80  (medium dark)
    //   20 m           → lum ≈  33  (dark)
    //   30 m+          → lum =   0  (black)
    const float display_max_m = 30.0f;
    const float log_denom     = std::log1p(display_max_m);  // log(1 + 30) ≈ 3.43

    // Grayscale helper: inverse_depth → luminance.
    // white (255) = close (0 m), black (0) = far (display_max_m).
    // Identical mapping for both panels so left/right are directly comparable.
    auto grey_lum = [&](float dr) -> std::uint8_t {
        float depth_m = display_max_m;
        if (std::isfinite(dr) && dr > 1e-6f) {
            depth_m = std::min(params.scale / dr, display_max_m);
        }
        const float t = 1.0f - std::log1p(depth_m) / log_denom;  // 1=close/white, 0=far/black
        return static_cast<std::uint8_t>(std::clamp(t * 255.0f, 0.0f, 255.0f));
    };

    // Output: (2W)×H row-major RGB
    std::vector<std::uint8_t> frame_rgb(static_cast<std::size_t>(2 * W * H) * 3U, 0U);

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const std::size_t pi = static_cast<std::size_t>(y * W + x);
            const float       dr = inferred.inverse_depth[pi];

            // LEFT panel: raw ONNX output — white (close) → black (far) grayscale.
            // Fixed metric scale so intensity is directly comparable across frames.
            const std::uint8_t lum = grey_lum(dr);
            const std::size_t  li  = static_cast<std::size_t>(y * 2 * W + x) * 3U;
            frame_rgb[li + 0U] = lum;
            frame_rgb[li + 1U] = lum;
            frame_rgb[li + 2U] = lum;

            // RIGHT panel: evidence filter view.
            //   Dark grey  — invalid model output (dr ≤ 0)
            //   Dark navy  — too far   (depth_m > max_depth_m)
            //   Red/Orange/Yellow — filtered (depth_m < min_depth_m), colored by gimbal pitch:
            //       pitch > 15° → red (OOD — nothing should be this close when looking down)
            //       pitch <  5° → yellow (expected — props/arms in field of view)
            //       between     → orange (ambiguous)
            //   Grey gradient — valid depth, not stride-sampled
            //   Cyan dot   — stride-sampled pixel that passes the depth filter → evidence
            const std::size_t ri = static_cast<std::size_t>(y * 2 * W + W + x) * 3U;
            if (!std::isfinite(dr) || dr <= 1e-6f) {
                // Invalid model output — dark grey
                frame_rgb[ri + 0U] = 30U;
                frame_rgb[ri + 1U] = 30U;
                frame_rgb[ri + 2U] = 30U;
            } else {
                const float depth_m = params.scale / dr;
                if (depth_m > params.max_depth_m) {
                    // Too far — dark navy
                    frame_rgb[ri + 0U] = 10U;
                    frame_rgb[ri + 1U] = 10U;
                    frame_rgb[ri + 2U] = 50U;
                } else if (depth_m >= params.min_depth_m
                           && x % params.stride == 0
                           && y % params.stride == 0) {
                    // Valid range, stride-sampled — cyan evidence dot
                    frame_rgb[ri + 0U] = 0U;
                    frame_rgb[ri + 1U] = 220U;
                    frame_rgb[ri + 2U] = 220U;
                } else if (depth_m < params.min_depth_m) {
                    // Filtered (too close): color by gimbal pitch to distinguish
                    // OOD noise (camera looking down) from expected close readings (props).
                    //   pitch > 15° (looking down)  → RED   — OOD noise, nothing should be this close
                    //   pitch <  5° (near level)     → YELLOW — expected (arms / props in view)
                    //   between                      → ORANGE — ambiguous
                    if (pitch_down_deg > 15.0f) {
                        frame_rgb[ri + 0U] = 220U; frame_rgb[ri + 1U] =  40U; frame_rgb[ri + 2U] =  40U;
                    } else if (pitch_down_deg < 5.0f) {
                        frame_rgb[ri + 0U] = 220U; frame_rgb[ri + 1U] = 220U; frame_rgb[ri + 2U] =   0U;
                    } else {
                        frame_rgb[ri + 0U] = 220U; frame_rgb[ri + 1U] = 130U; frame_rgb[ri + 2U] =   0U;
                    }
                } else {
                    // Valid depth, not stride-sampled — grey gradient
                    frame_rgb[ri + 0U] = lum;
                    frame_rgb[ri + 1U] = lum;
                    frame_rgb[ri + 2U] = lum;
                }
            }
        }
    }

    const std::size_t written = std::fwrite(frame_rgb.data(), 1U, frame_rgb.size(), debug_pipe_);
    if (written != frame_rgb.size()) {
        // ffmpeg pipe broken (bad path/extension, codec error, etc.) — close and
        // disable to avoid SIGPIPE killing the mission on the next write.
        std::fclose(debug_pipe_);
        debug_pipe_ = nullptr;
        std::fprintf(stderr, "[DepthDebug] depth_mp4 pipe broken after %zu/%zu bytes — disabling\n",
                     written, frame_rgb.size());
        return;
    }
    std::fflush(debug_pipe_);
}

std::vector<ObstacleEvidence> VisualDepthObstacleDetector::detect(
    const EgoSensingFrame& ego_frame) {
    const VisualDepthFrame vdf = make_visual_depth_frame(ego_frame);
    const DepthInferenceResult inferred = engine_->infer(vdf);
    if (!inferred.valid || inferred.inverse_depth.empty()) return {};

    // Camera pitch: NED convention, forward_axis_local.z > 0 means looking downward.
    const float fwd_z = static_cast<float>(ego_frame.sensing_volume.forward_axis_local.z);
    const float pitch_down_deg = std::asin(std::clamp(fwd_z, -1.0f, 1.0f))
                                 * (180.0f / 3.14159265f);

    // Depth histogram — always active (every 30 frames), regardless of debug MP4.
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

    if (!config_.debug_depth_mp4.empty()) {
        const ProjectionParams dbg_params = make_projection_params(
            ego_frame, inferred, scale_, config_);
        write_debug_frame(inferred, dbg_params, pitch_down_deg);
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

    return project_from_inferred(ego_frame, inferred, scale_, config_);
}

// ---------------------------------------------------------------------------
// Free-function implementation (shared by class and test code)
// ---------------------------------------------------------------------------

std::vector<ObstacleEvidence> detect_visual_depth_obstacles(
    const EgoSensingFrame&                   ego_frame,
    DepthEngineInterface&                    engine,
    const MetricScaleEstimate&               scale,
    const VisualDepthObstacleDetectorConfig& cfg) {

    const VisualDepthFrame vdf = make_visual_depth_frame(ego_frame);
    const DepthInferenceResult inferred = engine.infer(vdf);
    if (!inferred.valid || inferred.inverse_depth.empty()) return {};
    return project_from_inferred(ego_frame, inferred, scale, cfg);
}

}  // namespace dedalus
