#pragma once

#include <vector>

#include "dedalus/perception/perception_pipeline.hpp"

namespace dedalus {

enum class GhostTrajectoryType {
    Linear,
};

struct GhostTargetTrajectory {
    GhostTrajectoryType type{GhostTrajectoryType::Linear};
    Vec3 start_local_m;
    Vec3 velocity_local_mps;
};

struct GhostTargetSpec {
    TrackId track_id;
    ClassLabel class_label{ClassLabel::Unknown};
    FactionLabel faction{FactionLabel::Unknown};
    float confidence{0.0F};
    GhostTargetTrajectory trajectory;
};

class GhostTargetProvider {
public:
    explicit GhostTargetProvider(std::vector<GhostTargetSpec> targets);

    std::vector<Observation3D> observations_at(
        TimePoint timestamp,
        MapFrameId map_frame_id,
        TimePoint scenario_start = TimePoint{0}) const;

    PerceptionPipelineOutput output_at(
        TimePoint timestamp,
        MapFrameId map_frame_id,
        TimePoint scenario_start = TimePoint{0}) const;

private:
    std::vector<GhostTargetSpec> targets_;
};

}  // namespace dedalus
