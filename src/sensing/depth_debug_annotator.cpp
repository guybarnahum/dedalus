#include "dedalus/sensing/depth_debug_annotator.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace dedalus {
namespace {

// Log-luminance depth → greyscale.
// white (255) = 0 m (close), black (0) = display_max_m (far).
// log1p scale gives perceptual uniformity in the 0.1–30 m obstacle range.
std::uint8_t depth_to_lum(float depth_m, float display_max_m, float log_denom) {
    const float d = std::min(depth_m, display_max_m);
    const float t = 1.0f - std::log1p(d) / log_denom;  // 1=close, 0=far
    return static_cast<std::uint8_t>(std::clamp(t * 255.0f, 0.0f, 255.0f));
}

// Color coding shared between ONNX and GT filter panels.
//
//   dark grey (30,30,30)       — invalid (NaN / ≤ 0)
//   dark navy (10,10,50)       — too far (> max_depth_m)
//   red (220,40,0)             — too close + gimbal > 15° down (OOD noise)
//   orange (220,130,0)         — too close + gimbal 5–15° (ambiguous)
//   yellow (220,220,0)         — too close + gimbal < 5° (expected: props/arms)
//   background grey (90,90,90) — unsampled area (GT only)
//   grey gradient (lum,lum,lum)— valid depth, not evidence (ONNX non-stride pixel)
//   cyan (0,220,220)           — valid depth, evidence
struct Rgb { std::uint8_t r, g, b; };

Rgb filter_color(float depth_m,      // physical depth; NaN or <=0 = invalid
                 bool   valid,        // depth_m > 0 && finite
                 bool   is_evidence, // passes filter and sampled
                 float  pitch_down_deg,
                 float  min_depth_m,
                 float  max_depth_m,
                 std::uint8_t grey_lum) {
    if (!valid) return {30U, 30U, 30U};
    if (depth_m > max_depth_m)    return {10U, 10U, 50U};
    if (depth_m < min_depth_m) {
        if (pitch_down_deg > 15.0f) return {220U,  40U,   0U};
        if (pitch_down_deg <  5.0f) return {220U, 220U,   0U};
        return                            {220U, 130U,   0U};
    }
    if (is_evidence) return {0U, 220U, 220U};
    return {grey_lum, grey_lum, grey_lum};
}

}  // namespace

DepthDebugAnnotator::DepthDebugAnnotator(DepthDebugAnnotatorConfig config)
    : config_(std::move(config)) {}

DepthDebugAnnotator::~DepthDebugAnnotator() {
    if (pipe_ != nullptr) {
        const int rc = pclose(pipe_);
        if (rc != 0) {
            std::fprintf(stderr, "[DepthAnnotator] ffmpeg exited with code %d\n", rc);
        }
        pipe_ = nullptr;
    }
}

void DepthDebugAnnotator::open_pipe(int frame_w, int frame_h) {
    const std::string cmd =
        "ffmpeg -f rawvideo -pixel_format rgb24"
        " -video_size " + std::to_string(frame_w) + "x" + std::to_string(frame_h) +
        " -framerate 5 -i pipe:0"
        " -vcodec libx264 -crf 23 -pix_fmt yuv420p"
        " -movflags frag_keyframe+empty_moov+default_base_moof"
        " -y " + config_.output_path +
        " 2>/dev/null";
    std::fprintf(stderr, "[DepthAnnotator] opening pipe: %s\n", cmd.c_str());
    pipe_ = popen(cmd.c_str(), "w");
    if (pipe_ == nullptr) {
        std::fprintf(stderr, "[DepthAnnotator] ERROR: popen failed (errno=%d: %s) — MP4 disabled\n",
                     errno, std::strerror(errno));
    }
}

void DepthDebugAnnotator::annotate(
    const DepthInferenceResult& onnx_inferred,
    const ProjectionParams&     onnx_params,
    float                       pitch_down_deg,
    const AirSimDepthFrame*     gt) {

    if (config_.output_path.empty()) return;
    if (!onnx_inferred.valid || onnx_inferred.inverse_depth.empty()) return;
    if (onnx_inferred.width <= 0 || onnx_inferred.height <= 0)       return;

    const int W = onnx_inferred.width;
    const int H = onnx_inferred.height;

    // Frame geometry:
    //   four_panel=true  → 2W × 2H: ONNX row on top, GT eval row on bottom.
    //   four_panel=false → 2W × H:  ONNX left-right only (no GT wired).
    // Geometry is fixed at pipe-open time and cannot change mid-session.
    const int frame_w = 2 * W;
    const int frame_h = config_.four_panel ? 2 * H : H;

    if (pipe_ == nullptr) {
        open_pipe(frame_w, frame_h);
        if (pipe_ == nullptr) return;  // popen failed; error already logged
    }

    const float display_max_m = config_.display_max_m;
    const float log_denom     = std::log1p(display_max_m);

    // ONNX inverse_depth stores disparity (1/m): high = close, low = far.
    // depth_m = scale / dr;  white (255) = close (0 m), black (0) = display_max_m.
    auto onnx_lum = [&](float dr) -> std::uint8_t {
        if (!std::isfinite(dr) || dr <= 1e-6f) return 0U;
        return depth_to_lum(onnx_params.scale / dr, display_max_m, log_denom);
    };

    // Allocate full 2W×2H frame — default black (invalid).
    std::vector<std::uint8_t> frame_rgb(
        static_cast<std::size_t>(frame_w * frame_h) * 3U, 0U);

    // Helper: write one RGB triple at (row, col) in the 2W-wide frame.
    auto set_px = [&](int row, int col, Rgb c) {
        const std::size_t i = static_cast<std::size_t>(row * frame_w + col) * 3U;
        frame_rgb[i + 0U] = c.r;
        frame_rgb[i + 1U] = c.g;
        frame_rgb[i + 2U] = c.b;
    };

    // ── TOP ROW: ONNX panels ─────────────────────────────────────────────────
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const float        dr  = onnx_inferred.inverse_depth[static_cast<std::size_t>(y * W + x)];
            const std::uint8_t lum = onnx_lum(dr);

            // Top-left: raw greyscale
            set_px(y, x, {lum, lum, lum});

            // Top-right: filter view
            const bool  valid_px = std::isfinite(dr) && dr > 1e-6f;
            const float depth_m  = valid_px ? (onnx_params.scale / dr) : 0.0f;
            const bool  evidence = valid_px
                                 && depth_m >= onnx_params.min_depth_m
                                 && depth_m <= onnx_params.max_depth_m
                                 && (x % onnx_params.stride == 0)
                                 && (y % onnx_params.stride == 0);
            const Rgb fc = filter_color(depth_m, valid_px, evidence,
                                        pitch_down_deg,
                                        onnx_params.min_depth_m,
                                        onnx_params.max_depth_m,
                                        lum);
            set_px(y, W + x, fc);
        }
    }

    // ── BOTTOM ROW: GT panels (four_panel only) ───────────────────────────────
    if (config_.four_panel && gt && gt->width > 0 && gt->height > 0
            && !gt->depth_m.empty()
            && static_cast<int>(gt->depth_m.size()) >= gt->width * gt->height) {

        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                // Nearest-neighbor lookup into the sparse GT grid.
                const int gx  = (x * gt->width)  / W;
                const int gy  = (y * gt->height) / H;
                const float dm = gt->depth_m[static_cast<std::size_t>(gy * gt->width + gx)];

                const bool valid_px = std::isfinite(dm) && dm > 0.0f;
                const std::uint8_t lum = valid_px
                    ? depth_to_lum(dm, display_max_m, log_denom)
                    : std::uint8_t{0U};

                // Bottom-left: GT raw
                set_px(H + y, x, {lum, lum, lum});

                // Bottom-right: GT filter view
                // All GT samples at their native grid density count as evidence
                // (no additional stride pass on GT — AirSim detector uses stride=1).
                const bool evidence = valid_px
                                    && dm >= onnx_params.min_depth_m
                                    && dm <= onnx_params.max_depth_m;
                const Rgb fc = filter_color(dm, valid_px, evidence,
                                            pitch_down_deg,
                                            onnx_params.min_depth_m,
                                            onnx_params.max_depth_m,
                                            lum);
                set_px(H + y, W + x, fc);
            }
        }
    } else if (config_.four_panel) {
        // GT frame missing this tick — fill bottom row with mid-grey.
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < frame_w; ++x) {
                set_px(H + y, x, {60U, 60U, 60U});
            }
        }
    }

    // Write frame to ffmpeg pipe.
    const std::size_t written = std::fwrite(frame_rgb.data(), 1U, frame_rgb.size(), pipe_);
    if (written != frame_rgb.size()) {
        std::fclose(pipe_);
        pipe_ = nullptr;
        std::fprintf(stderr,
            "[DepthAnnotator] pipe broken after %zu/%zu bytes — MP4 disabled\n",
            written, frame_rgb.size());
        return;
    }
    std::fflush(pipe_);
}

}  // namespace dedalus
