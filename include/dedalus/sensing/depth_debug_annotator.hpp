#pragma once

#include <cstdio>
#include <string>

#include "dedalus/sensing/airsim_depth_obstacle_detector.hpp"
#include "dedalus/sensing/depth_engine.hpp"
#include "dedalus/sensing/depth_projection_kernel.hpp"

namespace dedalus {

struct DepthDebugAnnotatorConfig {
    // Output .mp4 path.  Empty = disabled.
    std::string output_path;

    // Log-scale depth anchor: depth_m >= display_max_m → black.
    // white (255) = 0 m, black (0) = display_max_m.
    float display_max_m{30.0f};

    // When true: 2W×2H frame — ONNX panels on top, GT eval panels on bottom.
    // When false: 2W×H frame — original left-right ONNX-only layout.
    // Set automatically from whether slot B (GT) is wired at CoreStackRunner construction.
    bool four_panel{false};
};

// Renders a 4-panel 2W×2H H.264 debug MP4 via an ffmpeg pipe.
//
// Frame layout (all panels share the same log-luminance depth scale):
//
//   ┌──────────────────────┬──────────────────────┐
//   │  ONNX raw depth      │  ONNX filter view    │
//   │  white=near          │  cyan  = evidence    │
//   │  black=far (30 m)    │  red   = OOD-close   │
//   │                      │  navy  = too-far     │
//   ├──────────────────────┼──────────────────────┤
//   │  GT raw depth        │  GT filter view      │
//   │  (same scale)        │  (same color coding) │
//   │  filled from sparse  │  background = mid    │
//   │  stride samples      │  grey (unsampled)    │
//   └──────────────────────┴──────────────────────┘
//
// Thread ownership: single-threaded. annotate() is not reentrant.
class DepthDebugAnnotator {
public:
    explicit DepthDebugAnnotator(DepthDebugAnnotatorConfig config);
    ~DepthDebugAnnotator();

    DepthDebugAnnotator(const DepthDebugAnnotator&)            = delete;
    DepthDebugAnnotator& operator=(const DepthDebugAnnotator&) = delete;

    // Render one debug frame.
    // gt may be nullptr when slot B is inactive or returned no depth frame.
    void annotate(const DepthInferenceResult& onnx_inferred,
                  const ProjectionParams&     onnx_params,
                  float                       pitch_down_deg,
                  const AirSimDepthFrame*     gt = nullptr);

private:
    void open_pipe(int frame_w, int frame_h);

    DepthDebugAnnotatorConfig config_;
    FILE*                     pipe_{nullptr};
};

}  // namespace dedalus
