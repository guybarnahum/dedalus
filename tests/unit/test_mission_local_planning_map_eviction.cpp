// test_mission_local_planning_map_eviction.cpp
//
// Validates evict_cleared_cells()'s targeted swap-and-pop removal (replacing
// a full cells_ scan + cell_index_ rebuild) against the exact same semantics
// as the old full-scan sweep:
//   1. basic_eviction_removes_cell: single cell driven below threshold.
//   2. surviving_cells_remain_correct_after_eviction: swap-and-pop must not
//      corrupt or mislabel cells that were not evicted.
//   3. duplicate_candidate_key_in_one_tick_evicts_once: the same L2 key can
//      appear more than once in one update_from_traversability() call (two
//      L1 sub-voxels mapping to the same coarser L2 voxel); must not double-
//      remove or corrupt cell_index_.
//   4. resurrected_cell_within_same_tick_survives: a key that crosses the
//      eviction threshold and is later re-hit by positive evidence within
//      the SAME update_from_traversability() call must survive — matching
//      the old code's behavior of checking occupied_score at the end of the
//      tick, not at the moment the threshold was first crossed.

#include "dedalus/avoidance/mission_local_planning_map.hpp"
#include "dedalus/avoidance/mission_local_traversability_map.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace {

using namespace dedalus;

MissionLocalTraversabilityCell occupied_cell(const Vec3& center, double confidence = 1.0) {
    MissionLocalTraversabilityCell c;
    c.center_map = center;
    c.state = TraversabilityCellState::Occupied;
    c.occupied_score = 2.0;  // well above L2's min_occupied_score (0.5 default)
    c.confidence = confidence;
    return c;
}

MissionLocalTraversabilityCell free_cell(const Vec3& center, double confidence = 1.0) {
    MissionLocalTraversabilityCell c;
    c.center_map = center;
    c.state = TraversabilityCellState::ObservedFree;
    c.occupied_score = 0.0;
    c.confidence = confidence;
    return c;
}

void feed_one(MissionLocalPlanningMap& map, const MissionLocalTraversabilityCell& cell) {
    MissionLocalTraversabilityMapSnapshot snap;
    snap.cells.push_back(cell);
    map.update_from_traversability(snap);
}

// ─── test 1: basic eviction ────────────────────────────────────────────────

void basic_eviction_removes_cell() {
    MissionLocalPlanningMap map;

    feed_one(map, occupied_cell(Vec3{0.5, 0.5, 1.0}));
    assert(map.cell_count() == 1U);

    feed_one(map, free_cell(Vec3{0.5, 0.5, 1.0}));
    assert(map.cell_count() == 0U);
    assert(map.last_update_stats().cells_evicted == 1U);

    std::printf("PASS  basic_eviction_removes_cell\n");
}

// ─── test 2: swap-and-pop must not corrupt survivors ──────────────────────

void surviving_cells_remain_correct_after_eviction() {
    MissionLocalPlanningMap map;

    const Vec3 a{0.5, 0.5, 1.0};
    const Vec3 b{5.5, 0.5, 1.0};
    const Vec3 c{10.5, 0.5, 1.0};

    // Three distinct L2 voxels, all occupied. b is inserted between a and c
    // so a swap-with-last during b's removal exercises index re-pointing.
    feed_one(map, occupied_cell(a));
    feed_one(map, occupied_cell(b));
    feed_one(map, occupied_cell(c));
    assert(map.cell_count() == 3U);

    // Evict only b.
    feed_one(map, free_cell(b));
    assert(map.cell_count() == 2U);

    const Bounds3 bbox{Vec3{-1.0, -1.0, -1.0}, Vec3{20.0, 2.0, 3.0}};
    const auto occupied_points = map.query_occupied_in_box(bbox);
    assert(occupied_points.size() == 2U);

    bool found_a = false;
    bool found_c = false;
    bool found_b = false;
    for (const auto& p : occupied_points) {
        if (std::abs(p.x - a.x) < 0.01 && std::abs(p.y - a.y) < 0.01) { found_a = true; }
        if (std::abs(p.x - c.x) < 0.01 && std::abs(p.y - c.y) < 0.01) { found_c = true; }
        if (std::abs(p.x - b.x) < 0.01 && std::abs(p.y - b.y) < 0.01) { found_b = true; }
    }
    assert(found_a && "cell a corrupted or lost after evicting b");
    assert(found_c && "cell c corrupted or lost after evicting b");
    assert(!found_b && "evicted cell b still present");

    std::printf("PASS  surviving_cells_remain_correct_after_eviction\n");
}

// ─── test 3: duplicate candidate key within one tick ──────────────────────

void duplicate_candidate_key_in_one_tick_evicts_once() {
    MissionLocalPlanningMap map;

    const Vec3 k{0.5, 0.5, 1.0};
    feed_one(map, occupied_cell(k));
    assert(map.cell_count() == 1U);

    // Two distinct L1 sub-voxels mapping to the same L2 key, both reporting
    // free in the SAME update_from_traversability() call. local_evicted will
    // contain k twice; evict_cleared_cells() must remove it exactly once
    // (second lookup finds it already gone) rather than corrupting state.
    MissionLocalTraversabilityMapSnapshot snap;
    snap.cells.push_back(free_cell(k));
    snap.cells.push_back(free_cell(k));
    map.update_from_traversability(snap);

    assert(map.cell_count() == 0U);

    std::printf("PASS  duplicate_candidate_key_in_one_tick_evicts_once\n");
}

// ─── test 4: resurrection within the same tick must survive ───────────────

void resurrected_cell_within_same_tick_survives() {
    MissionLocalPlanningMap map;

    const Vec3 k{0.5, 0.5, 1.0};
    feed_one(map, occupied_cell(k));
    assert(map.cell_count() == 1U);

    // In one update_from_traversability() call: a free hit first drives k's
    // log_odds below the eviction threshold (queuing it in local_evicted),
    // then an occupied hit for the SAME key later in the same batch recomputes
    // occupied_score from sigmoid(log_odds), which is always > 0 for finite
    // log_odds. The old full-scan sweep only evicted cells whose
    // occupied_score was still <= 0 by the end of the tick, so this cell must
    // survive — evict_cleared_cells() re-checks the live score rather than
    // trusting candidate_keys unconditionally, to match that exactly.
    MissionLocalTraversabilityMapSnapshot snap;
    snap.cells.push_back(free_cell(k));
    snap.cells.push_back(occupied_cell(k));
    map.update_from_traversability(snap);

    assert(map.cell_count() == 1U && "resurrected cell was incorrectly evicted");
    // The eviction attempt is still counted even though the cell survived —
    // this stat tracks threshold-crossing events this tick, not net removals,
    // and that counting behavior is unchanged by this refactor.
    assert(map.last_update_stats().cells_evicted == 1U);

    std::printf("PASS  resurrected_cell_within_same_tick_survives\n");
}

}  // namespace

int main() {
    basic_eviction_removes_cell();
    surviving_cells_remain_correct_after_eviction();
    duplicate_candidate_key_in_one_tick_evicts_once();
    resurrected_cell_within_same_tick_survives();
    std::printf("OK    all planning map eviction tests passed\n");
    return 0;
}
