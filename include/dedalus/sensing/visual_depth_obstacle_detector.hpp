#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include "dedalus/sensing/depth_engine.hpp"
#include "dedalus/sensing/depth_projection_kernel.hpp"
#include "dedalus/sensing/metric_scale_estimate.hpp"
#include "dedalus/sensing/obstacle_evidence_provider.hpp"

namespace dedalus {

struct VisualDepthObstacleDetectorConfig {
    // N×M grid over the depth map: divide into depth_grid_cols × depth_grid_rows cells,
    // project the closest valid pixel per cell. Produces at most N×M evidence points.
    // Set depth_grid_cols=W, depth_grid_rows=H for 1-per-pixel (stride-1 equivalent).
    std::size_t depth_grid_cols{40U};
    std::size_t depth_grid_rows{22U};

    float min_depth_m{0.2F};
    float max_depth_m{80.0F};

    // Voxel grid resolution used to deduplicate back-projected evidence.
    // Should match L1 cell size (0.5 m).
    float voxel_size_m{0.5F};

    float confidence{0.75F};

    // Upper bound on ObstacleEvidence items emitted per frame.
    // Includes projection evidence + surface patches + thin structures.
    std::size_t max_evidence{512U};

    // Enable RANSAC surface patch detection (is_surface_hint).
    bool detect_surface_patches{true};

    // Enable Sobel + NMS + CC thin structure detection (is_thin_structure_hint).
    bool detect_thin_structures{true};
};

// Visual depth obstacle detector.
//
// Provider responsibility: ONNX inference + scale estimation + diagnostic
// logging (depth histogram, OOD frame capture) + calibrated intrinsics
// scaling. Everything downstream (kernels, inflate, bearing/elevation,
// source_kind) is handled by run_depth_pipeline().
//
// Thread ownership: single-threaded tick. detect() is not reentrant.
class VisualDepthObstacleDetector final : public ObstacleEvidenceProvider {
public:
    VisualDepthObstacleDetector(
        std::unique_ptr<DepthEngineInterface> engine,
        MetricScaleEstimate                   scale,
        VisualDepthObstacleDetectorConfig     config = {});

    ~VisualDepthObstacleDetector() override;  // defined out-of-line to anchor vtable

    VisualDepthObstacleDetector(const VisualDepthObstacleDetector&)            = delete;
    VisualDepthObstacleDetector& operator=(const VisualDepthObstacleDetector&) = delete;

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
    [[nodiscard]] float            last_pitch_deg()     const override { return last_pitch_deg_; }
    [[nodiscard]] bool             has_last_result()    const { return last_has_result_; }

private:
    std::unique_ptr<DepthEngineInterface> engine_;
    MetricScaleEstimate                   scale_;
    VisualDepthObstacleDetectorConfig     config_;

    // Cache populated by each detect() call.
    DepthPipelineInput last_input_;
    ProjectionParams   last_params_;
    float              last_pitch_deg_{0.0f};
    bool               last_has_result_{false};
};

// Free-function variant for testing without a class instance.
[[nodiscard]] std::vector<ObstacleEvidence> detect_visual_depth_obstacles(
    const EgoSensingFrame&                    frame,
    DepthEngineInterface&                     engine,
    const MetricScaleEstimate&                scale,
    const VisualDepthObstacleDetectorConfig&  config = {});

}  // namespace dedalus
