#pragma once

namespace dedalus {

// Single global scale factor mapping relative depth output to metric depth.
//
//   depth_m = scale / inverse_depth
//
// V0: fixed at startup from AirSim camera config. L1 map persistence covers
// peripheral geometry from prior frames, so single-scale is acceptable at
// this stage.
//
// VD7 (deferred): replace with VIO-coupled per-region scale.
struct MetricScaleEstimate {
    float scale{1.0F};       // metres per (1/relative_depth)
    float confidence{0.0F};  // [0,1]; 0 = uninitialised
    float age_s{0.0F};       // seconds since last update
};

}  // namespace dedalus
