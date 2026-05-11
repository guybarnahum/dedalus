#include "dedalus/world_model/rough_flight_map_builder.hpp"

namespace dedalus {

RoughFlightMapUpdate RoughFlightMapBuilder::build(
    const std::vector<Observation3D>& observations,
    const EgoState& ego,
    TimePoint now,
    MapFrameId map_frame_id) const {
    RoughFlightMapUpdate update;

    StaticStructure building;
    building.structure_id = StructureId{"structure_building_0001"};
    building.type = "building";
    building.bounds.min = Vec3{14.0, -8.0, -14.0};
    building.bounds.max = Vec3{28.0, -5.0, 2.0};
    building.map_frame_id = map_frame_id;
    building.feature_signature = FeatureVector{0.2F, 0.5F, 0.9F};
    building.confidence = 0.55F;
    building.first_seen = now;
    building.last_confirmed = now;
    update.static_structures.push_back(building);

    FlightCorridor corridor;
    corridor.corridor_id = CorridorId{"corridor_forward_0001"};
    corridor.centerline = {
        ego.local_T_body.position,
        Vec3{ego.local_T_body.position.x + 40.0, ego.local_T_body.position.y, ego.local_T_body.position.z},
        Vec3{ego.local_T_body.position.x + 80.0, ego.local_T_body.position.y + 4.0, ego.local_T_body.position.z}
    };
    corridor.map_frame_id = map_frame_id;
    corridor.radius_m = 6.0F;
    corridor.min_altitude_m = 8.0F;
    corridor.max_altitude_m = 30.0F;
    corridor.confidence = observations.empty() ? 0.25F : 0.45F;
    update.flight_corridors.push_back(corridor);

    Landmark landmark;
    landmark.landmark_id = LandmarkId{"landmark_building_corner_0001"};
    landmark.type = "building_corner";
    landmark.position_local = Vec3{14.0, -5.0, -8.0};
    landmark.map_frame_id = map_frame_id;
    landmark.feature_signature = FeatureVector{0.3F, 0.1F, 0.7F};
    landmark.confidence = 0.5F;
    update.landmarks.push_back(landmark);

    return update;
}

}  // namespace dedalus
