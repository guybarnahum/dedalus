#pragma once

#include <cstdint>
#include <vector>

#include "dedalus/core/types.hpp"
#include "dedalus/sensing/metric_scale_estimate.hpp"

namespace dedalus {

// ── Configuration ────────────────────────────────────────────────────────────

struct VisualOdometryConfig {
    MapFrameId map_frame_id{"map_visual_0001"};

    // VL1 — feature detection
    int   fast_threshold{20};    // FAST-9 corner threshold (intensity difference)
    int   max_features{200};     // maximum tracked corners per frame
    int   min_features{40};      // re-detect when below this count

    // VL1 — Lucas-Kanade tracking
    int   lk_patch_radius{3};    // patch half-width: (2r+1)² pixels, default 7×7
    int   lk_iterations{5};      // Newton-Raphson iterations per feature

    // VL1 — Focus-of-Expansion RANSAC (translation direction)
    int    foe_ransac_iterations{80};   // RANSAC iterations
    double foe_ransac_threshold{1.5};   // inlier threshold (pixels in image space)
    int    foe_min_inliers{6};          // accept estimate when >= this many inliers

    // VL1 — initial metric scale (metres per frame of unit-direction motion).
    // If an AirSim velocity hint is present in the frame, it overrides this.
    // VL2 replaces it dynamically from depth observations.
    float initial_scale_m{0.1F};   // ~0.1 m/frame at 30 Hz, 3 m/s forward cruise

    // VL2 — scale update
    float scale_update_alpha{0.1F};             // EMA coefficient for scale updates
    int   scale_update_min_samples{5};          // min matching features before update

    // VL2 — re-localization
    float relocalization_confidence_threshold{0.5F};  // trigger ICP when below
    float relocalization_search_radius_m{5.0F};        // L2 voxel search sphere

    // VL3 — uncertainty propagation
    double translation_noise_per_m{0.01};  // drift sigma growth per metre traveled
    double rotation_noise_per_rad{0.02};   // drift sigma growth per radian rotated

    // VL3 — AirSim fallback (only active when a fallback provider is set)
    float fallback_confidence_threshold{0.3F};
};

// ── Feature tracking state ────────────────────────────────────────────────────

struct TrackedFeature {
    float x{0.0F};       // current position (pixels)
    float y{0.0F};
    float x_prev{0.0F};  // position in previous frame
    float y_prev{0.0F};
    bool  valid{true};
    int   age{0};        // frames this feature has been tracked
};

// Row-major 3×3 rotation matrix.
using Rot3 = std::array<double, 9>;

// ── Running VO state ─────────────────────────────────────────────────────────

struct VisualOdometryState {
    // Cumulative pose in the declared map frame (ENU/NED matches L2 convention).
    Vec3 position{0.0, 0.0, 0.0};
    Rot3 rotation{1,0,0, 0,1,0, 0,0,1};  // R_map_from_camera

    // Metric scale — updated by VL2 from depth observations.
    MetricScaleEstimate scale;

    // Uncertainty accumulators (VL3).
    double cumulative_drift_m{0.0};  // integrated position uncertainty (metres)
    double translation_sigma{0.0};   // 1-σ position uncertainty
    double rotation_sigma{0.0};      // 1-σ orientation uncertainty (radians)

    // Feature tracking buffers.
    std::vector<TrackedFeature>  features;
    std::vector<std::uint8_t>    prev_gray;  // greyscale image of previous frame
    int prev_width{0};
    int prev_height{0};

    bool initialized{false};
};

}  // namespace dedalus
