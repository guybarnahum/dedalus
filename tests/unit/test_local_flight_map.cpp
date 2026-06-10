#include "dedalus/avoidance/local_flight_map.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>

namespace {

using namespace dedalus;

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

}  // namespace

int main() {
    occupied_evidence_marks_expected_cell();
    out_of_range_evidence_is_ignored();
    evidence_decays_below_threshold();
    inflation_marks_neighbor_cells();
    max_evidence_per_update_is_enforced();

    std::cout << "local flight map accumulator tests passed\n";
    return 0;
}
