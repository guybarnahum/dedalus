#include "dedalus/avoidance/mission_local_obstacle_map.hpp"

#include <cassert>
#include <iostream>
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
    t.timestamp_ns = timestamp_ns;
    return t;
}

ObstacleEvidence occupied_evidence(const Vec3& center, const std::string& map_frame_id = "mission_local") {
    ObstacleEvidence evidence;
    evidence.state = ObstacleEvidenceState::Occupied;
    evidence.source_kind = OccupancySourceKind::DepthProvider;
    evidence.source_provider = "unit_test_depth";
    evidence.map_frame_id = map_frame(map_frame_id);
    evidence.center_local = center;
    evidence.size_m = Vec3{0.25, 0.25, 0.25};
    evidence.confidence = 1.0F;
    return evidence;
}

void occupied_evidence_creates_mission_local_cell() {
    MissionLocalObstacleMap map(MissionLocalObstacleMapConfig{});

    const auto snapshot = map.update(
        {occupied_evidence(Vec3{1.1, -0.2, 0.3})},
        at_ns(1000),
        map_frame("mission_local"));

    assert(snapshot.summary.map_frame_id.value == "mission_local");
    assert(snapshot.summary.update_count == 1U);
    assert(snapshot.summary.observed_cell_count == 1U);
    assert(snapshot.summary.occupied_cell_count == 1U);
    assert(snapshot.cells.size() == 1U);
    assert(snapshot.cells.front().occupied);
    assert(snapshot.cells.front().last_source_provider == "unit_test_depth");
}

void repeated_evidence_accumulates_score_in_same_cell() {
    MissionLocalObstacleMapConfig config;
    config.cell_size_m = 1.0;
    config.vertical_cell_size_m = 1.0;
    config.occupied_threshold = 2.0;
    config.occupied_hit_score = 1.0;

    MissionLocalObstacleMap map(config);

    auto snapshot = map.update(
        {occupied_evidence(Vec3{1.1, 2.1, 0.2})},
        at_ns(1000),
        map_frame("mission_local"));
    assert(snapshot.summary.observed_cell_count == 1U);
    assert(snapshot.summary.occupied_cell_count == 0U);

    snapshot = map.update(
        {occupied_evidence(Vec3{1.2, 2.2, 0.3})},
        at_ns(2000),
        map_frame("mission_local"));

    assert(snapshot.summary.observed_cell_count == 1U);
    assert(snapshot.summary.occupied_cell_count == 1U);
    assert(snapshot.cells.front().occupied_score >= 2.0);
}

void decay_reduces_occupied_state() {
    MissionLocalObstacleMapConfig config;
    config.cell_size_m = 1.0;
    config.vertical_cell_size_m = 1.0;
    config.occupied_threshold = 1.0;
    config.occupied_hit_score = 1.0;
    config.score_decay_per_second = 2.0;

    MissionLocalObstacleMap map(config);

    auto snapshot = map.update(
        {occupied_evidence(Vec3{0.1, 0.1, 0.1})},
        at_ns(0),
        map_frame("mission_local"));
    assert(snapshot.summary.occupied_cell_count == 1U);

    snapshot = map.update({}, at_ns(1000000000ULL), map_frame("mission_local"));

    assert(snapshot.summary.observed_cell_count == 1U);
    assert(snapshot.summary.occupied_cell_count == 0U);
}

void evidence_from_other_map_frame_is_ignored() {
    MissionLocalObstacleMap map;

    auto snapshot = map.update(
        {occupied_evidence(Vec3{0.1, 0.1, 0.1}, "mission_local")},
        at_ns(1000),
        map_frame("mission_local"));
    assert(snapshot.summary.observed_cell_count == 1U);

    snapshot = map.update(
        {occupied_evidence(Vec3{5.1, 5.1, 0.1}, "other_map")},
        at_ns(2000),
        map_frame("mission_local"));

    assert(snapshot.summary.map_frame_id.value == "mission_local");
    assert(snapshot.summary.observed_cell_count == 1U);
}

void snapshot_cell_limit_returns_highest_priority_cells() {
    MissionLocalObstacleMap map;

    std::vector<ObstacleEvidence> evidence;
    evidence.push_back(occupied_evidence(Vec3{0.1, 0.1, 0.1}));
    evidence.push_back(occupied_evidence(Vec3{2.1, 0.1, 0.1}));
    evidence.push_back(occupied_evidence(Vec3{4.1, 0.1, 0.1}));

    map.update(evidence, at_ns(1000), map_frame("mission_local"));
    const auto limited = map.snapshot(2U);

    assert(limited.cells.size() == 2U);
    assert(limited.summary.observed_cell_count == 3U);
}

}  // namespace

int main() {
    occupied_evidence_creates_mission_local_cell();
    repeated_evidence_accumulates_score_in_same_cell();
    decay_reduces_occupied_state();
    evidence_from_other_map_frame_is_ignored();
    snapshot_cell_limit_returns_highest_priority_cells();

    std::cout << "mission local obstacle map tests passed\n";
    return 0;
}
