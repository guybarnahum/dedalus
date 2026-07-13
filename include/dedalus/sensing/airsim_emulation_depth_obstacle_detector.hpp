#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "dedalus/occupancy/occupancy_types.hpp"
#include "dedalus/sensing/airsim_depth_obstacle_detector.hpp"
#include "dedalus/sensing/depth_projection_kernel.hpp"
#include "dedalus/sensing/obstacle_evidence_provider.hpp"

namespace dedalus {

// Config mirrors VisualDepthObstacleDetectorConfig.
// No engine fields — this provider uses AirSim GT metric depth directly.
struct AirSimEmulationDepthObstacleDetectorConfig {
    // N×M grid over the GT depth map — mirrors VisualDepthObstacleDetectorConfig.
    // Default 40×22 matches the AirSim bridge stride=16 on 640×360 (one GT sample per cell).
    std::size_t depth_grid_cols{40U};
    std::size_t depth_grid_rows{22U};

    float min_depth_m{0.2F};
    float max_depth_m{80.0F};

    // Voxel grid resolution for evidence deduplication.
    float voxel_size_m{0.5F};

    float confidence{0.75F};
    std::size_t max_evidence{512U};

    // Enable RANSAC surface patch detection (landable-surface evaluation).
    bool detect_surface_patches{true};

    // Enable local-depth-contrast thin-structure detection.
    bool detect_thin_structures{true};

    // Metric scale: inverse_depth = scale / depth_m.
    // GT depth is in metres so scale=1 is correct for unmodified AirSim output.
    float scale{1.0F};
};

// AirSim GT depth path threaded through the VD projection/surface/thin kernels.
//
// Purpose: validate VD kernel math against known GT geometry without ONNX.
//
// Pipeline per frame:
//   AirSimDepthFrame::depth_m → invert to inverse_depth (scale/depth_m)
//     → project_depth_to_device_evidence()
//     → [optional] fit_surface_patches_device()    (landable-surface evaluation)
//     → [optional] detect_thin_structures_device() (thin-obstacle evaluation)
//     → inflate() → patch source_kind = AirSimGroundTruthVisualEmulation
//
// Loaded into slot A or B via config — never hard-disabled.
class AirSimEmulationDepthObstacleDetector final : public ObstacleEvidenceProvider {
public:
    explicit AirSimEmulationDepthObstacleDetector(
        AirSimEmulationDepthObstacleDetectorConfig config = {});

    ~AirSimEmulationDepthObstacleDetector() override = default;

    AirSimEmulationDepthObstacleDetector(const AirSimEmulationDepthObstacleDetector&)            = delete;
    AirSimEmulationDepthObstacleDetector& operator=(const AirSimEmulationDepthObstacleDetector&) = delete;

    [[nodiscard]] std::string provider_name() const override;

    [[nodiscard]] std::vector<ObstacleEvidence> detect(const EgoSensingFrame& frame) override;

    // Returns the AirSimDepthFrame from the most recent detect() call.
    // nullptr if detect() has never been called or no depth frame was present.
    // Pointer is valid until the next detect() call.
    [[nodiscard]] const AirSimDepthFrame*  last_depth_frame() const {
        return last_frame_valid_ ? &last_frame_ : nullptr;
    }

    // Returns the ProjectionParams built during the most recent detect() call.
    // Valid when last_depth_frame() is non-null.
    [[nodiscard]] const ProjectionParams& last_params() const { return last_params_; }

private:
    AirSimEmulationDepthObstacleDetectorConfig config_;
    AirSimDepthFrame last_frame_;
    bool             last_frame_valid_{false};
    ProjectionParams last_params_;
};

}  // namespace dedalus
