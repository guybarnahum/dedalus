#include "dedalus/avoidance/local_flight_map.hpp"
#include "dedalus/avoidance/mission_local_obstacle_map.hpp"
#include "dedalus/geometry/pose_transform.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>

namespace {

using namespace dedalus;

MapFrameId map_frame(const std::string& value) {
    MapFrameId frame;
    frame.value = value;
    return frame;
}

TimePoint at_ms(const std::int64_t ms) {
    return TimePoint{ms * 1'000'000LL};
}

ObstacleEvidence occupied_depth_evidence(const Vec3& center, const float range_m = 8.0F) {
    ObstacleEvidence evidence;
    evidence.timestamp = at_ms(0);
    evidence.has_source_frame = true;
    evidence.source_frame_id = FrameId{"frame_0001"};
    evidence.source_provider = "airsim_depth_obstacle_detector";
    evidence.source_kind = OccupancySourceKind::DepthProvider;
    evidence.state = ObstacleEvidenceState::Occupied;
    evidence.shape = ObstacleEvidenceShape::SurfacePatch;
    evidence.center_local = center;
    evidence.size_m = Vec3{0.5, 0.5, 0.2};
    evidence.confidence = 0.9F;
    evidence.range_m = range_m;
    evidence.inside_sensing_volume = true;
    evidence.map_frame_id = map_frame("mission_local");
    return evidence;
}

WorldSnapshot snapshot_with(std::initializer_list<ObstacleEvidence> evidence, const std::int64_t ms = 0) {
    WorldSnapshot snapshot;
    snapshot.timestamp = at_ms(ms);
    snapshot.obstacle_evidence.assign(evidence.begin(), evidence.end());
    return snapshot;
}

const LocalFlightMapCell& cell_for(const LocalFlightMapAccumulator& accumulator, const Vec3& local) {
    const auto index = accumulator.local_to_grid(local);
    assert(index.has_value());
    const auto* cell = accumulator.cell_at(index->ix, index->iy);
    assert(cell != nullptr);
    return *cell;
}

void occupied_evidence_marks_expected_cell() {
    LocalFlightMapConfig config;
    config.cell_size_m = 1.0F;
    config.forward_range_m = 10.0F;
    config.rear_range_m = 2.0F;
    config.lateral_range_m = 5.0F;
    config.safety_margin_m = 0.0F;
    config.vehicle_radius_m = 0.0F;

    LocalFlightMapAccumulator accumulator{config};
    const Vec3 obstacle{4.0, 1.0, -0.5};

    const auto result = accumulator.update(snapshot_with({occupied_depth_evidence(obstacle)}));
    const auto& cell = cell_for(accumulator, obstacle);

    assert(cell.occupied);
    assert(cell.occupied_score >= config.occupied_threshold);
    assert(result.occupied_count >= 1U);
    assert(std::isfinite(result.nearest_obstacle_m));
    assert(result.nearest_obstacle_m <= 8.0F);
}

void out_of_range_evidence_is_ignored() {
    LocalFlightMapConfig config;
    config.cell_size_m = 1.0F;
    config.forward_range_m = 8.0F;
    config.rear_range_m = 1.0F;
    config.lateral_range_m = 3.0F;
    config.max_evidence_range_m = 5.0F;

    LocalFlightMapAccumulator accumulator{config};
    auto evidence = occupied_depth_evidence(Vec3{4.0, 0.0, 0.0}, 12.0F);

    const auto result = accumulator.update(snapshot_with({evidence}));
    assert(result.occupied_count == 0U);
    assert(result.inflated_blocked_count == 0U);
}

void evidence_decays_below_threshold() {
    LocalFlightMapConfig config;
    config.cell_size_m = 1.0F;
    config.forward_range_m = 10.0F;
    config.rear_range_m = 2.0F;
    config.lateral_range_m = 5.0F;
    config.decay_half_life_s = 0.25F;
    config.safety_margin_m = 0.0F;
    config.vehicle_radius_m = 0.0F;

    LocalFlightMapAccumulator accumulator{config};
    const Vec3 obstacle{3.0, 0.0, 0.0};

    auto first = accumulator.update(snapshot_with({occupied_depth_evidence(obstacle)}, 0));
    assert(first.occupied_count >= 1U);

    auto later = accumulator.update(snapshot_with({}, 2000));
    assert(later.occupied_count == 0U);
    assert(later.inflated_blocked_count == 0U);
}

void inflation_marks_neighbor_cells() {
    LocalFlightMapConfig config;
    config.cell_size_m = 1.0F;
    config.forward_range_m = 10.0F;
    config.rear_range_m = 2.0F;
    config.lateral_range_m = 5.0F;
    config.vehicle_radius_m = 0.5F;
    config.safety_margin_m = 1.0F;

    LocalFlightMapAccumulator accumulator{config};
    const Vec3 obstacle{4.0, 0.0, 0.0};

    const auto result = accumulator.update(snapshot_with({occupied_depth_evidence(obstacle)}));
    assert(result.occupied_count >= 1U);
    assert(result.inflated_blocked_count > result.occupied_count);
}

void max_evidence_per_update_is_enforced() {
    LocalFlightMapConfig config;
    config.cell_size_m = 1.0F;
    config.forward_range_m = 20.0F;
    config.rear_range_m = 2.0F;
    config.lateral_range_m = 10.0F;
    config.max_evidence_per_update = 1U;
    config.safety_margin_m = 0.0F;
    config.vehicle_radius_m = 0.0F;

    LocalFlightMapAccumulator accumulator{config};

    auto first = occupied_depth_evidence(Vec3{3.0, -2.0, 0.0}, 3.0F);
    auto second = occupied_depth_evidence(Vec3{8.0, 2.0, 0.0}, 8.0F);

    const auto result = accumulator.update(snapshot_with({first, second}));
    assert(result.occupied_count >= 1U);

    const auto& second_cell = cell_for(accumulator, second.center_local);
    assert(!second_cell.occupied);
}

void mission_local_crop_accounts_for_ego_translation() {
    MissionLocalObstacleMapConfig mission_config;
    mission_config.cell_size_m = 1.0;
    mission_config.vertical_cell_size_m = 1.0;
    mission_config.occupied_threshold = 0.5;
    mission_config.occupied_hit_score = 1.0;

    MissionLocalObstacleMap mission_map{mission_config};
    mission_map.update(
        {occupied_depth_evidence(Vec3{10.0, 0.0, 0.0}, 10.0F)},
        at_ms(0),
        map_frame("mission_local"));

    LocalFlightMapConfig local_config;
    local_config.cell_size_m = 1.0F;
    local_config.forward_range_m = 5.0F;
    local_config.rear_range_m = 2.0F;
    local_config.lateral_range_m = 3.0F;
    local_config.vehicle_radius_m = 0.0F;
    local_config.safety_margin_m = 0.0F;

    LocalFlightMapAccumulator accumulator{local_config};

    Pose3 map_T_body;
    map_T_body.position = Vec3{8.0, 0.0, 0.0};
    map_T_body.rotation_rpy = Vec3{0.0, 0.0, 0.0};

    const auto result = accumulator.update_from_mission_local_map(
        mission_map.snapshot(),
        map_T_body,
        at_ms(100));

    const auto& local_cell = cell_for(accumulator, Vec3{2.0, 0.0, 0.0});
    assert(local_cell.occupied);
    assert(result.occupied_count >= 1U);
}

void mission_local_crop_accounts_for_ego_yaw() {
    constexpr double kPi = 3.141592653589793238462643383279502884;

    MissionLocalObstacleMapConfig mission_config;
    mission_config.cell_size_m = 1.0;
    mission_config.vertical_cell_size_m = 1.0;
    mission_config.occupied_threshold = 0.5;
    mission_config.occupied_hit_score = 1.0;

    MissionLocalObstacleMap mission_map{mission_config};
    mission_map.update(
        {occupied_depth_evidence(Vec3{0.0, 4.0, 0.0}, 4.0F)},
        at_ms(0),
        map_frame("mission_local"));

    LocalFlightMapConfig local_config;
    local_config.cell_size_m = 1.0F;
    local_config.forward_range_m = 6.0F;
    local_config.rear_range_m = 2.0F;
    local_config.lateral_range_m = 3.0F;
    local_config.vehicle_radius_m = 0.0F;
    local_config.safety_margin_m = 0.0F;

    LocalFlightMapAccumulator accumulator{local_config};

    const Pose3 map_T_body = make_yaw_pose(Vec3{0.0, 0.0, 0.0}, kPi / 2.0);

    const auto result = accumulator.update_from_mission_local_map(
        mission_map.snapshot(),
        map_T_body,
        at_ms(100));

    const auto& local_cell = cell_for(accumulator, Vec3{4.0, 0.0, 0.0});
    assert(local_cell.occupied);
    assert(result.occupied_count >= 1U);
}


}  // namespace

int main() {
    occupied_evidence_marks_expected_cell();
    out_of_range_evidence_is_ignored();
    evidence_decays_below_threshold();
    inflation_marks_neighbor_cells();
    max_evidence_per_update_is_enforced();
    mission_local_crop_accounts_for_ego_translation();
    mission_local_crop_accounts_for_ego_yaw();

    std::cout << "local flight map accumulator tests passed\n";
    return 0;
}
