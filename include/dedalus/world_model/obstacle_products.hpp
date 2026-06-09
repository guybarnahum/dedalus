#pragma once

#include <vector>

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

// Rebuilds ego occupancy, swept volume, sensing volumes, and obstacle evidence
// from a synthetic fixture. Called on every ego update.
void refresh_synthetic_obstacle_products(WorldSnapshot& snapshot);

// Rebuilds ego occupancy, swept volume, sensing volumes, and obstacle evidence
// from AirSim ground-truth perception output.
void refresh_ground_truth_obstacle_products(
    WorldSnapshot& snapshot,
    const PerceptionPipelineOutput& output,
    const std::vector<ObstacleSensingVolume>& configured_volumes);

}  // namespace dedalus
