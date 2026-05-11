#include "dedalus/world_model/tactical_obstacle_mapper.hpp"

#include <string>

namespace dedalus {

std::vector<ExclusionZone> ConeExclusionMapper::map(
    const std::vector<Observation3D>& observations,
    TimePoint now,
    MapFrameId map_frame_id) {
    std::vector<ExclusionZone> zones;
    zones.reserve(observations.size());

    for (std::size_t i = 0; i < observations.size(); ++i) {
        const auto& observation = observations[i];

        ExclusionZone zone;
        zone.zone_id = ZoneId{"zone_" + std::string(4 - std::to_string(i + 1).size(), '0') + std::to_string(i + 1)};
        zone.timestamp = now;
        zone.type = ZoneType::Cone;
        zone.local_T_zone.position = observation.position_local;
        zone.dimensions = Vec3{6.0, 3.0, 3.0};
        zone.map_frame_id = map_frame_id;
        zone.confidence = observation.confidence;
        zone.inflation_radius_m = 1.5F;
        zone.expires_at = TimePoint{now.timestamp_ns + 2'000'000'000};
        zone.reason = "dynamic_observation_cone";
        zones.push_back(zone);
    }

    return zones;
}

}  // namespace dedalus
