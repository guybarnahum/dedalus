#pragma once

#include <string_view>

#include "dedalus/occupancy/occupancy_types.hpp"

namespace dedalus {

enum class ObstacleEvidenceVisualClass {
    DepthSurfacePatch,
    BlockingObstacle,
    ThinStructureRisk,
    OccupiedObstacle,
    FreeSpace,
    UnknownObstacle,
};

struct ObstacleEvidenceVisualStyle {
    ObstacleEvidenceVisualClass visual_class{ObstacleEvidenceVisualClass::UnknownObstacle};
    std::string_view label{"obs"};
    bool draw_surface_patch{false};
    bool draw_surface_normal{false};
    bool include_in_depth_surface_count{false};
};

bool is_depth_surface_patch_evidence(const ObstacleEvidence& evidence);

ObstacleEvidenceVisualClass obstacle_evidence_visual_class(const ObstacleEvidence& evidence);

ObstacleEvidenceVisualStyle obstacle_evidence_visual_style(const ObstacleEvidence& evidence);

}  // namespace dedalus
