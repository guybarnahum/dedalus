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
    // Pixel stride over the depth map produced by the depth engine.
    // stride=4 gives 1/16 density relative to the full H×W output.
    std::size_t pixel_stride{4U};

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

    // When non-empty, write a false-color depth PPM per frame to this directory.
    // Files: depth_0000.ppm, depth_0001.ppm, ...
    // Convert to MP4: ffmpeg -framerate 5 -pattern_type glob -i 'depth_*.ppm' depth_debug.mp4
    std::string debug_depth_output_dir;
};

// Visual depth obstacle detector.
//
// Replaces AirSimDepthObstacleDetector as the primary slot-A provider.
// AirSim DepthPlanar (GT) remains available as a plug-in provider for
// training and calibration via AirSimDepthEvidenceProvider (slot A or B).
//
// Pipeline per frame:
//   VisualDepthFrame → DepthEngineInterface::infer()
//     → project_depth_to_device_evidence()
//     → [optional] fit_surface_patches_device()
//     → [optional] detect_thin_structures_device()
//     → inflate() → ObstacleEvidence[]
//
// Thread ownership: single-threaded tick. detect() is not reentrant.
class VisualDepthObstacleDetector final : public ObstacleEvidenceProvider {
public:
    VisualDepthObstacleDetector(
        std::unique_ptr<DepthEngineInterface> engine,
        MetricScaleEstimate                   scale,
        VisualDepthObstacleDetectorConfig     config = {});

    ~VisualDepthObstacleDetector() override = default;

    VisualDepthObstacleDetector(const VisualDepthObstacleDetector&)            = delete;
    VisualDepthObstacleDetector& operator=(const VisualDepthObstacleDetector&) = delete;

    [[nodiscard]] std::string provider_name() const override;

    // Extracts VisualDepthFrame from EgoSensingFrame::frame.image, runs the
    // full pipeline, and returns inflated ObstacleEvidence.
    [[nodiscard]] std::vector<ObstacleEvidence> detect(const EgoSensingFrame& frame) override;

private:
    std::unique_ptr<DepthEngineInterface> engine_;
    MetricScaleEstimate                   scale_;
    VisualDepthObstacleDetectorConfig     config_;
};

// Free-function variant for testing without a class instance.
[[nodiscard]] std::vector<ObstacleEvidence> detect_visual_depth_obstacles(
    const EgoSensingFrame&                    frame,
    DepthEngineInterface&                     engine,
    const MetricScaleEstimate&                scale,
    const VisualDepthObstacleDetectorConfig&  config = {});

}  // namespace dedalus
