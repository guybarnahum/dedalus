#pragma once

#include <vector>

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/world_model/effective_world_view.hpp"
#include "dedalus/world_model/rough_flight_map_builder.hpp"
#include "dedalus/world_model/tactical_obstacle_mapper.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

struct InMemoryWorldModelConfig {
    MapFrameId map_frame_id{"map_local_0001"};
    OccupancySourceKind occupancy_source_kind{OccupancySourceKind::SyntheticFixture};
};

class InMemoryWorldModel {
public:
    explicit InMemoryWorldModel(MapFrameId map_frame_id = MapFrameId{"map_local_0001"});
    explicit InMemoryWorldModel(InMemoryWorldModelConfig config);

    void update_ego(const EgoState& ego);
    void update_appearance(const AppearanceCondition& appearance_condition);
    void update_obstacle_sensing_volumes(std::vector<ObstacleSensingVolume> volumes);
    void ingest(const PerceptionPipelineOutput& perception_output);

    WorldSnapshot snapshot() const;
    EffectiveWorldView effective_view() const;

private:
    InMemoryWorldModelConfig config_;
    WorldSnapshot snapshot_;
    std::vector<ObstacleSensingVolume> configured_obstacle_sensing_volumes_;
    ConeExclusionMapper cone_exclusion_mapper_;
    RoughFlightMapBuilder rough_flight_map_builder_;
};

}  // namespace dedalus
