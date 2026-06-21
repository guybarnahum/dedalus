// test_mission_local_planning_map_streaming.cpp
//
// Stage 5 validation: per-cell write_seq tracking + delta snapshot().
//
//   1. delta_correctness:  100 cells inserted; take seq0.  20 more inserted.
//      snapshot(seq0) must return exactly 20 cells.
//   2. delta_empty_no_changes:  50 cells inserted; take seq0.  snapshot(seq0)
//      must return 0 cells (nothing changed after the watermark).
//   3. full_snapshot_completeness:  1200 cells inserted.  snapshot(0) must
//      return all 1200 cells.
//   4. chunk_serialize_time:  serialize a 500-cell snapshot in < 20 ms.

#include "dedalus/avoidance/mission_local_planning_map.hpp"
#include "dedalus/avoidance/mission_local_planning_map_publisher.hpp"

#include <cassert>
#include <chrono>
#include <cstdio>

namespace {

using namespace dedalus;

// ─── helper ──────────────────────────────────────────────────────────────────

void insert_occupied(MissionLocalPlanningMap& map, double x, double y, double z) {
    MissionLocalTraversabilityMapSnapshot snap;
    MissionLocalTraversabilityCell c;
    c.center_map     = Vec3{x, y, z};
    c.state          = TraversabilityCellState::Occupied;
    c.occupied_score = 2.0;
    c.confidence     = 0.9;
    snap.cells.push_back(c);
    map.update_from_traversability(snap);
}

// Each call inserts n cells starting at (x_offset, 0, 1) with 1 m spacing.
void insert_batch(MissionLocalPlanningMap& map, int n, double x_offset = 0.0) {
    for (int i = 0; i < n; ++i) {
        insert_occupied(map, x_offset + static_cast<double>(i) + 0.5, 0.5, 1.0);
    }
}

// ─── test 1: delta returns only new cells ─────────────────────────────────────

void test_delta_correctness() {
    MissionLocalPlanningMap map;

    // Insert 100 cells spread across 100 distinct voxels.
    insert_batch(map, 100, 0.0);
    const std::uint64_t seq0 = map.current_seq();

    // Insert 20 more at a different x range (distinct keys).
    insert_batch(map, 20, 200.0);

    const auto delta = map.snapshot(seq0);
    assert(delta.is_delta);
    assert(delta.seq == map.current_seq());
    assert(delta.cells.size() == 20U);

    std::printf("PASS  test_delta_correctness\n");
}

// ─── test 2: delta is empty when nothing changed ──────────────────────────────

void test_delta_empty_no_changes() {
    MissionLocalPlanningMap map;

    insert_batch(map, 50, 0.0);
    const std::uint64_t seq0 = map.current_seq();

    const auto delta = map.snapshot(seq0);
    assert(delta.is_delta);
    assert(delta.cells.empty());

    std::printf("PASS  test_delta_empty_no_changes\n");
}

// ─── test 3: full snapshot returns all cells ─────────────────────────────────

void test_full_snapshot_completeness() {
    MissionLocalPlanningMap map;

    // 1200 cells across a 1200 m strip — all distinct 1 m voxels.
    insert_batch(map, 1200, 0.0);

    const auto full = map.snapshot(0U);
    assert(!full.is_delta);
    assert(full.cells.size() == 1200U);
    assert(full.cell_count == 1200U);

    std::printf("PASS  test_full_snapshot_completeness\n");
}

// ─── test 4: to_compact_stream_json for 500 cells < 20 ms ────────────────────

void test_chunk_serialize_time() {
    MissionLocalPlanningMap map;
    insert_batch(map, 500, 0.0);

    const auto snap = map.snapshot(0U);
    assert(snap.cells.size() == 500U);

    const auto t0 = std::chrono::steady_clock::now();
    const auto json = to_compact_stream_json(snap, 0U);
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();

    assert(!json.empty());
    std::printf("INFO  to_compact_stream_json 500 cells → %lld ms\n",
                static_cast<long long>(elapsed_ms));
    assert(elapsed_ms < 20 && "serialize 500-cell snapshot >= 20 ms");

    std::printf("PASS  test_chunk_serialize_time\n");
}

}  // namespace

int main() {
    test_delta_correctness();
    test_delta_empty_no_changes();
    test_full_snapshot_completeness();
    test_chunk_serialize_time();
    std::printf("OK    all planning map streaming tests passed\n");
    return 0;
}
