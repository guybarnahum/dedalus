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

// AirSim GT depth path threaded through the shared VD processing pipeline.
//
// Provider responsibility: sensor name filter + metric→inverse_depth conversion
// + FoV-based intrinsics computation. Everything downstream (kernels, inflate,
// bearing/elevation, source_kind) is handled by run_depth_pipeline().
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

    // ── Introspection (base class overrides) ───────────────────────────────
    [[nodiscard]] const float*     last_inverse_depth() const override {
        return last_has_result_ ? last_input_.inverse_depth.data() : nullptr;
    }
    [[nodiscard]] int              last_depth_width()   const override {
        return last_has_result_ ? last_input_.width : 0;
    }
    [[nodiscard]] int              last_depth_height()  const override {
        return last_has_result_ ? last_input_.height : 0;
    }
    [[nodiscard]] ProjectionParams last_params()        const override { return last_params_; }

private:
    AirSimEmulationDepthObstacleDetectorConfig config_;
    DepthPipelineInput last_input_;
    ProjectionParams   last_params_;
    bool               last_has_result_{false};
};

}  // namespace dedalus
