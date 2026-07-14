#pragma once

#include <cstdio>
#include <string>

#include "dedalus/sensing/depth_projection_kernel.hpp"

namespace dedalus {

struct DepthDebugAnnotatorConfig {
    // Output .mp4 path.  Empty = disabled.
    std::string output_path;

    // Log-scale depth anchor: depth_m >= display_max_m → black.
    // white (255) = 0 m, black (0) = display_max_m.
    float display_max_m{30.0f};
};

// Generic depth panel — one per slot (primary + optional eval).
//
// inverse_depth carries the disparity-convention buffer from the provider
// (depth_m = params.scale / inverse_depth[i]).  Both AirSim and ONNX providers
// now store this convention after the refactor; depth_m_data is gone.
//
// params carries scale, min/max_depth_m, grid_cols/rows for the filter view.
// source_name is rendered as a label in the frame header strip.
struct DepthDebugPanel {
    const float* inverse_depth{nullptr};
    int          width{0};
    int          height{0};
    ProjectionParams params;
    std::string  source_name;
    float        pitch_down_deg{0.0f};
};

// Renders a 2W×H (primary only) or 2W×2H (primary + eval) H.264 debug MP4
// via an ffmpeg pipe.
//
// Frame layout (per row — top = primary, bottom = eval if present):
//
//   ┌──────────────────────┬──────────────────────┐
//   │  raw depth           │  filter view         │
//   │  white=near          │  cyan  = evidence    │
//   │  black=far           │  red   = OOD-close   │
//   │  [source label]      │  navy  = too-far     │
//   └──────────────────────┴──────────────────────┘
//
// Thread ownership: single-threaded. annotate() is not reentrant.
class DepthDebugAnnotator {
public:
    explicit DepthDebugAnnotator(DepthDebugAnnotatorConfig config);
    ~DepthDebugAnnotator();

    DepthDebugAnnotator(const DepthDebugAnnotator&)            = delete;
    DepthDebugAnnotator& operator=(const DepthDebugAnnotator&) = delete;

    // Render one debug frame from primary (required) and optional eval panel.
    void annotate(const DepthDebugPanel& primary,
                  const DepthDebugPanel* eval = nullptr);

private:
    void open_pipe(int frame_w, int frame_h);

    DepthDebugAnnotatorConfig config_;
    FILE*                     pipe_{nullptr};
};

}  // namespace dedalus
