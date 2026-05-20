#pragma once

#include <vector>

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/simulation/ghost_scenario.hpp"

namespace dedalus {

class GhostTargetProvider {
public:
    explicit GhostTargetProvider(GhostScenario scenario);

    std::vector<Observation3D> observations_at(
        TimePoint timestamp,
        MapFrameId map_frame_id,
        TimePoint scenario_start = TimePoint{0}) const;

    PerceptionPipelineOutput output_at(
        TimePoint timestamp,
        MapFrameId map_frame_id,
        TimePoint scenario_start = TimePoint{0}) const;

private:
    GhostScenario scenario_;
};

}  // namespace dedalus
