#pragma once

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/world_model/effective_world_view.hpp"
#include "dedalus/world_model/rough_flight_map_builder.hpp"
#include "dedalus/world_model/tactical_obstacle_mapper.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

class InMemoryWorldModel {
public:
    explicit InMemoryWorldModel(MapFrameId map_frame_id = MapFrameId{"map_local_0001"});

    void update_ego(const EgoState& ego);
    void update_appearance(const AppearanceCondition& appearance_condition);
    void ingest(const PerceptionPipelineOutput& perception_output);

    WorldSnapshot snapshot() const;
    EffectiveWorldView effective_view() const;

private:
    WorldSnapshot snapshot_;
    ConeExclusionMapper cone_exclusion_mapper_;
    RoughFlightMapBuilder rough_flight_map_builder_;
};

}  // namespace dedalus
