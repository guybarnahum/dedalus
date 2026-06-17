#include "dedalus/avoidance/mission_map_assimilator.hpp"
#include "dedalus/avoidance/mission_local_traversability_map.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using namespace dedalus;

namespace {

MapFrameId map_frame(const std::string& value) {
    MapFrameId frame;
    frame.value = value;
    return frame;
}

TimePoint at_ns(const std::uint64_t timestamp_ns) {
    TimePoint t;
    t.timestamp_ns = static_cast<Nanoseconds>(timestamp_ns);
    return t;
}

MissionLocalObstacleCell source_cell(
    const Vec3& center,
    const double occupied_score,
    const double free_score,
    const std::uint64_t timestamp_ns = 1000U) {
    MissionLocalObstacleCell cell;
    cell.center_map = center;
    cell.size_m = Vec3{1.0, 1.0, 1.0};
    cell.occupied_score = occupied_score;
    cell.free_score = free_score;
    cell.confidence = 1.0;
    cell.observed = true;
    cell.occupied = occupied_score >= 1.0;
    cell.free = free_score >= 1.0;
    cell.positive_observation_count = occupied_score > 0.0 ? 7U : 0U;
    cell.negative_observation_count = free_score > 0.0 ? 5U : 0U;
    cell.first_observed_timestamp_ns = timestamp_ns;
    cell.last_observed_timestamp_ns = timestamp_ns;
    cell.last_confirmed_occupied_timestamp_ns = occupied_score > 0.0 ? timestamp_ns : 0U;
    cell.last_observed_free_timestamp_ns = free_score > 0.0 ? timestamp_ns : 0U;
    return cell;
}

MissionLocalObstacleMapSnapshot source_snapshot(
    const std::vector<MissionLocalObstacleCell>& cells,
    const std::string& frame_id = "mission_local") {
    MissionLocalObstacleMapSnapshot snapshot;
    snapshot.config.cell_size_m = 1.0;
    snapshot.config.vertical_cell_size_m = 1.0;
    snapshot.summary.map_frame_id = map_frame(frame_id);
    snapshot.summary.observed_cell_count = cells.size();
    snapshot.summary.occupied_cell_count = 0U;
    snapshot.summary.free_cell_count = 0U;
    snapshot.cells = cells;
    for (const auto& cell : snapshot.cells) {
        if (cell.occupied) {
            ++snapshot.summary.occupied_cell_count;
        }
        if (cell.free) {
            ++snapshot.summary.free_cell_count;
        }
    }
    return snapshot;
}

const MissionLocalTraversabilityCell& find_cell_near(
    const MissionLocalTraversabilityMapSnapshot& snapshot,
    const Vec3& expected_center) {
    for (const auto& cell : snapshot.cells) {
        if (std::abs(cell.center_map.x - expected_center.x) < 0.51 &&
            std::abs(cell.center_map.y - expected_center.y) < 0.51 &&
            std::abs(cell.center_map.z - expected_center.z) < 0.51) {
            return cell;
        }
    }
    assert(false && "expected traversability cell was not found");
    return snapshot.cells.front();
}

void compacted_obstacle_snapshot_builds_foundational_map_without_trajectory() {
    MissionLocalTraversabilityMapConfig config;
    config.cell_size_m = 1.0;
    config.vertical_cell_size_m = 1.0;
    config.required_clearance_m = 1.5;
    config.soft_clearance_m = 3.0;
    config.free_score_decay_per_second = 0.0;

    MissionLocalTraversabilityMap map(config);
    const auto snapshot = map.update_from_mission_obstacle_map(
        source_snapshot({
            source_cell(Vec3{0.1, 0.1, 0.1}, 1.0, 0.0),
            source_cell(Vec3{2.1, 0.1, 0.1}, 0.0, 1.0),
            source_cell(Vec3{2.1, 0.1, 2.1}, 1.0, 0.0),
        }),
        at_ns(1000U));

    assert(snapshot.summary.map_frame_id.value == "mission_local");
    assert(snapshot.summary.source_obstacle_cell_count == 3U);
    assert(snapshot.summary.accepted_source_cell_count == 3U);
    assert(snapshot.summary.cell_count == 3U);
    assert(snapshot.summary.occupied_cell_count == 2U);
    assert(snapshot.summary.free_cell_count == 1U);
    assert(snapshot.summary.overhead_risk_cell_count >= 1U);

    const auto& free_cell = find_cell_near(snapshot, Vec3{2.5, 0.5, 0.5});
    assert(free_cell.state == TraversabilityCellState::ObservedFree);
    assert(std::isfinite(free_cell.nearest_obstacle_distance_m));
    assert(free_cell.nearest_obstacle_distance_m >= 1.9);
    assert(free_cell.vertical_clearance_up_m >= 1.9);
    assert(free_cell.overhead_cost > 0.0);

    const auto occupied_query = map.query_sphere(Vec3{0.1, 0.1, 0.1}, 0.5);
    assert(occupied_query.known);
    assert(occupied_query.occupied);

    const auto unknown_query = map.query_sphere(Vec3{50.0, 50.0, 50.0}, 0.5);
    assert(!unknown_query.known);
    assert(unknown_query.unknown_fraction == 1.0);
}

void repeated_full_snapshots_do_not_bloat_or_oversaturate_counts() {
    MissionLocalTraversabilityMapConfig config;
    config.cell_size_m = 1.0;
    config.vertical_cell_size_m = 1.0;
    config.free_score_decay_per_second = 0.0;

    MissionLocalTraversabilityMap map(config);
    const auto obstacle_snapshot = source_snapshot({
        source_cell(Vec3{1.1, 1.1, 0.1}, 1.0, 0.0),
    });

    auto snapshot = map.update_from_mission_obstacle_map(obstacle_snapshot, at_ns(1000U));
    assert(snapshot.summary.cell_count == 1U);
    assert(snapshot.cells.front().occupied_hits_capped == 7U);

    snapshot = map.update_from_mission_obstacle_map(obstacle_snapshot, at_ns(2000U));
    assert(snapshot.summary.cell_count == 1U);
    assert(snapshot.summary.new_cell_count == 0U);
    assert(snapshot.summary.updated_cell_count == 1U);
    assert(snapshot.cells.front().occupied_hits_capped == 7U);
    assert(snapshot.cells.front().occupied_score <= config.max_score);
}

void stale_free_space_ages_without_becoming_erased() {
    MissionLocalTraversabilityMapConfig config;
    config.cell_size_m = 1.0;
    config.vertical_cell_size_m = 1.0;
    config.stale_after_seconds = 1.0;
    config.free_score_decay_per_second = 0.0;

    MissionLocalTraversabilityMap map(config);
    auto snapshot = map.update_from_mission_obstacle_map(
        source_snapshot({
            source_cell(Vec3{3.1, 0.1, 0.1}, 0.0, 1.0, 1000U),
        }),
        at_ns(1000U));

    assert(snapshot.summary.free_cell_count == 1U);
    assert(snapshot.summary.stale_cell_count == 0U);

    MissionLocalObstacleMapSnapshot empty_snapshot;
    empty_snapshot.summary.map_frame_id = map_frame("mission_local");
    snapshot = map.update_from_mission_obstacle_map(empty_snapshot, at_ns(3000001000ULL));

    assert(snapshot.summary.cell_count == 1U);
    assert(snapshot.summary.stale_cell_count == 1U);
    assert(snapshot.cells.front().stale);
}

void post_landing_flush_drains_pending_evidence_for_map_learning() {
    MissionMapAssimilatorConfig config;
    config.traversability_config.cell_size_m = 1.0;
    config.traversability_config.vertical_cell_size_m = 1.0;
    config.max_pending_snapshots = 4U;
    config.max_snapshots_per_background_tick = 1U;
    config.max_snapshots_per_high_priority_tick = 1U;
    config.max_post_landing_flush_ticks = 4U;

    MissionMapAssimilator assimilator(config);
    assimilator.enqueue_mission_obstacle_map(source_snapshot({
        source_cell(Vec3{0.1, 0.1, 0.1}, 1.0, 0.0),
    }));
    assimilator.enqueue_mission_obstacle_map(source_snapshot({
        source_cell(Vec3{1.1, 0.1, 0.1}, 1.0, 0.0),
    }));

    assert(assimilator.status().pending_snapshot_count == 2U);

    const auto result = assimilator.flush_after_landing(at_ns(5000U));
    assert(result.completed);
    assert(!result.timed_out);
    assert(result.can_forget_raw_evidence);
    assert(result.drained_snapshot_count == 2U);
    assert(result.pending_snapshot_count == 0U);
    assert(assimilator.status().state == MissionMapAssimilationState::Finalized);
    assert(assimilator.status().can_forget_raw_evidence);
    assert(assimilator.traversability_map().summary().cell_count == 2U);
}

}  // namespace

int main() {
    compacted_obstacle_snapshot_builds_foundational_map_without_trajectory();
    repeated_full_snapshots_do_not_bloat_or_oversaturate_counts();
    stale_free_space_ages_without_becoming_erased();
    post_landing_flush_drains_pending_evidence_for_map_learning();

    std::cout << "mission local traversability map tests passed\n";
    return 0;
}
