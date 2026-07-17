#include "dedalus/visualization/obstacle_evidence_visualization.hpp"

namespace dedalus {

bool is_depth_surface_patch_evidence(const ObstacleEvidence& evidence) {
    return evidence.shape == ObstacleEvidenceShape::SurfacePatch;
}

ObstacleEvidenceVisualClass obstacle_evidence_visual_class(const ObstacleEvidence& evidence) {
    if (is_depth_surface_patch_evidence(evidence)) {
        return ObstacleEvidenceVisualClass::DepthSurfacePatch;
    }
    if (evidence.inside_swept_volume) {
        return ObstacleEvidenceVisualClass::BlockingObstacle;
    }
    if (evidence.state == ObstacleEvidenceState::ThinStructureRisk) {
        return ObstacleEvidenceVisualClass::ThinStructureRisk;
    }
    if (evidence.state == ObstacleEvidenceState::Occupied) {
        return ObstacleEvidenceVisualClass::OccupiedObstacle;
    }
    if (evidence.state == ObstacleEvidenceState::Free) {
        return ObstacleEvidenceVisualClass::FreeSpace;
    }
    return ObstacleEvidenceVisualClass::UnknownObstacle;
}

ObstacleEvidenceVisualStyle obstacle_evidence_visual_style(const ObstacleEvidence& evidence) {
    const auto visual_class = obstacle_evidence_visual_class(evidence);

    switch (visual_class) {
        case ObstacleEvidenceVisualClass::DepthSurfacePatch:
            return ObstacleEvidenceVisualStyle{
                visual_class,
                "depth_surf",
                true,
                evidence.has_surface_normal,
                true};

        case ObstacleEvidenceVisualClass::BlockingObstacle:
            return ObstacleEvidenceVisualStyle{
                visual_class,
                "block",
                false,
                false,
                false};

        case ObstacleEvidenceVisualClass::ThinStructureRisk:
            return ObstacleEvidenceVisualStyle{
                visual_class,
                "thin",
                false,
                false,
                false};

        case ObstacleEvidenceVisualClass::OccupiedObstacle:
            return ObstacleEvidenceVisualStyle{
                visual_class,
                evidence.shape == ObstacleEvidenceShape::SurfacePatch ? "surf" : "occ",
                evidence.shape == ObstacleEvidenceShape::SurfacePatch,
                evidence.has_surface_normal,
                false};

        case ObstacleEvidenceVisualClass::FreeSpace:
            return ObstacleEvidenceVisualStyle{
                visual_class,
                "free",
                false,
                false,
                false};

        case ObstacleEvidenceVisualClass::UnknownObstacle:
        default:
            return ObstacleEvidenceVisualStyle{
                visual_class,
                "obs",
                false,
                false,
                false};
    }
}

}  // namespace dedalus
