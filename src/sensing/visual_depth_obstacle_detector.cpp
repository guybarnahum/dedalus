#include "dedalus/sensing/visual_depth_obstacle_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
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
    cuda_dispatch().project(inferred.depth_relative.data(), params, buf.data(), count);
    if (cdebug) { std::fprintf(stderr, "[CudaDepth] project done (%u ev)\n", count); std::fflush(stderr); }
#else
    project_depth_to_device_evidence(
        inferred.depth_relative.data(), params, buf.data(), count);
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
            inferred.depth_relative.data(), params, thin.data(), thin_count);
        if (cdebug) { std::fprintf(stderr, "[CudaDepth] detect_thin done (%u thin)\n", thin_count); std::fflush(stderr); }
#else
        detect_thin_structures_device(
            inferred.depth_relative.data(), params, thin.data(), thin_count);
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
        pclose(debug_pipe_);
    }
}

std::string VisualDepthObstacleDetector::provider_name() const {
    return "visual_depth_obstacle_detector";
}

void VisualDepthObstacleDetector::write_debug_frame(
    const DepthInferenceResult& inferred,
    const ProjectionParams&     params) {

    if (inferred.width <= 0 || inferred.height <= 0 || inferred.depth_relative.empty()) return;

    const int      W = inferred.width;
    const int      H = inferred.height;
    const std::size_t n = static_cast<std::size_t>(W * H);

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
        debug_pipe_ = popen(cmd.c_str(), "w");
        if (debug_pipe_ == nullptr) return;
    }

    // Normalise depth_relative to [0,1] for consistent rainbow mapping.
    // t=0 → blue (far), t=1 → red (near).
    float vmin = inferred.depth_relative[0];
    float vmax = inferred.depth_relative[0];
    for (std::size_t i = 0U; i < n; ++i) {
        const float v = inferred.depth_relative[i];
        if (!std::isfinite(v)) continue;
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
    }
    const float range = (vmax - vmin > 1.0e-6f) ? (vmax - vmin) : 1.0f;

    // Rainbow helper: depth_relative → RGB
    auto rainbow_rgb = [&](float dr, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b) {
        const float t = std::isfinite(dr)
            ? std::clamp((dr - vmin) / range, 0.0f, 1.0f) : 0.0f;
        r = static_cast<std::uint8_t>(std::clamp(255.0f*(1.5f-std::abs(4.0f*t-3.0f)), 0.0f, 255.0f));
        g = static_cast<std::uint8_t>(std::clamp(255.0f*(1.5f-std::abs(4.0f*t-2.0f)), 0.0f, 255.0f));
        b = static_cast<std::uint8_t>(std::clamp(255.0f*(1.5f-std::abs(4.0f*t-1.0f)), 0.0f, 255.0f));
    };

    // Output: (2W)×H row-major RGB
    std::vector<std::uint8_t> frame_rgb(static_cast<std::size_t>(2 * W * H) * 3U, 0U);

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const std::size_t i  = static_cast<std::size_t>(y * W + x);
            const float       dr = inferred.depth_relative[i];

            // --- Left panel: raw ONNX output (model diagnostic) ---
            std::uint8_t r, g, b;
            rainbow_rgb(dr, r, g, b);
            const std::size_t li = static_cast<std::size_t>(y * 2 * W + x) * 3U;
            frame_rgb[li + 0U] = r;
            frame_rgb[li + 1U] = g;
            frame_rgb[li + 2U] = b;

            // --- Right panel: evidence filter view ---
            // Colour coding:
            //   Magenta   — too close (depth_m < min_depth_m): props, near-field clutter
            //   Dark navy — too far   (depth_m > max_depth_m)
            //   Rainbow   — valid evidence range
            //   Brightened — stride-sampled pixel (contributes to L0 evidence)
            const std::size_t ri = static_cast<std::size_t>(y * 2 * W + W + x) * 3U;
            if (!std::isfinite(dr) || dr <= 1e-6f) {
                // Invalid model output — dark grey
                frame_rgb[ri + 0U] = 30U;
                frame_rgb[ri + 1U] = 30U;
                frame_rgb[ri + 2U] = 30U;
            } else {
                const float depth_m = params.scale / dr;
                if (depth_m < params.min_depth_m) {
                    // Too close — magenta
                    frame_rgb[ri + 0U] = 255U;
                    frame_rgb[ri + 1U] = 0U;
                    frame_rgb[ri + 2U] = 255U;
                } else if (depth_m > params.max_depth_m) {
                    // Too far — dark navy
                    frame_rgb[ri + 0U] = 10U;
                    frame_rgb[ri + 1U] = 10U;
                    frame_rgb[ri + 2U] = 50U;
                } else {
                    // Valid range — rainbow, brightened at stride positions
                    rainbow_rgb(dr, r, g, b);
                    if (x % params.stride == 0 && y % params.stride == 0) {
                        r = static_cast<std::uint8_t>(std::min(255, static_cast<int>(r) + 80));
                        g = static_cast<std::uint8_t>(std::min(255, static_cast<int>(g) + 80));
                        b = static_cast<std::uint8_t>(std::min(255, static_cast<int>(b) + 80));
                    }
                    frame_rgb[ri + 0U] = r;
                    frame_rgb[ri + 1U] = g;
                    frame_rgb[ri + 2U] = b;
                }
            }
        }
    }

    std::fwrite(frame_rgb.data(), 1U, frame_rgb.size(), debug_pipe_);
    std::fflush(debug_pipe_);
}

std::vector<ObstacleEvidence> VisualDepthObstacleDetector::detect(
    const EgoSensingFrame& ego_frame) {
    const VisualDepthFrame vdf = make_visual_depth_frame(ego_frame);
    const DepthInferenceResult inferred = engine_->infer(vdf);
    if (!inferred.valid || inferred.depth_relative.empty()) return {};
    if (!config_.debug_depth_mp4.empty()) {
        const ProjectionParams dbg_params = make_projection_params(
            ego_frame, inferred, scale_, config_);
        write_debug_frame(inferred, dbg_params);
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
    if (!inferred.valid || inferred.depth_relative.empty()) return {};
    return project_from_inferred(ego_frame, inferred, scale, cfg);
}

}  // namespace dedalus
