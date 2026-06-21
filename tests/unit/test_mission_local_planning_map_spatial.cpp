// test_mission_local_planning_map_spatial.cpp
//
// Stage 2 validation: sliding in-memory window.
//   - Cell count bounded after drone moves far from initial obstacles.
//   - Cells reloaded on re-entry.
//   - slide_window() latency < 5 ms.

#include "dedalus/avoidance/mission_local_planning_map.hpp"
#include "dedalus/avoidance/mission_local_traversability_map.hpp"

#include <cassert>
#include <chrono>
#include <cstdio>
#include <filesystem>

namespace {

using namespace dedalus;

// ─── helpers ─────────────────────────────────────────────────────────────────

// Push N occupied cells clustered around (cx, cy, 1.0) with 1 m spacing.
void push_cluster(MissionLocalPlanningMap& map,
                  double cx, double cy, int n) {
    MissionLocalTraversabilityMapSnapshot snap;
    for (int i = 0; i < n; ++i) {
        MissionLocalTraversabilityCell c;
        c.center_map    = Vec3{cx + static_cast<double>(i), cy, 1.0};
        c.state         = TraversabilityCellState::Occupied;
        c.occupied_score = 2.0;
        c.confidence    = 0.9;
        snap.cells.push_back(c);
    }
    map.update_from_traversability(snap);
}

// ─── test 1: eviction bounds memory ──────────────────────────────────────────

void test_eviction_bounds_memory() {
    const auto path = std::filesystem::temp_directory_path() /
                      "dedalus_spatial_test_evict.db";
    std::filesystem::remove(path);

    MissionLocalPlanningMapConfig cfg;
    cfg.horizon_m = 50.0;  // small window for test speed

    MissionLocalPlanningMap map{cfg};
    assert(map.open_db(path));

    // Load 100 cells near (0, 0).
    push_cluster(map, 0.0, 0.0, 100);
    assert(map.cell_count() == 100U);
    assert(map.flush_dirty_to_db());

    // Drone starts at (0,0,0) — slide initialises, no eviction yet.
    map.slide_window(Vec3{0.0, 0.0, 0.0});
    assert(map.cell_count() == 100U);

    // Drone moves > horizon/4 = 12.5 m but within 2×horizon = 100 m: no eviction.
    map.slide_window(Vec3{20.0, 0.0, 0.0});
    assert(map.cell_count() == 100U);

    // Drone moves far away (> 2×horizon = 100 m from original cells at x∈[0,99]).
    // Cells at x∈[0,99] are all > 100 m from (250,0,0) → all evicted.
    map.slide_window(Vec3{250.0, 0.0, 0.0});
    assert(map.cell_count() == 0U);

    assert(map.close_db());

    std::filesystem::remove(path);
    std::filesystem::remove(path.string() + "-wal");
    std::filesystem::remove(path.string() + "-shm");

    std::printf("PASS  test_eviction_bounds_memory\n");
}

// ─── test 2: re-entry reloads evicted cells ───────────────────────────────────

void test_reentry_reloads() {
    const auto path = std::filesystem::temp_directory_path() /
                      "dedalus_spatial_test_reentry.db";
    std::filesystem::remove(path);

    MissionLocalPlanningMapConfig cfg;
    cfg.horizon_m = 50.0;

    MissionLocalPlanningMap map{cfg};
    assert(map.open_db(path));

    // Cluster A near origin (already persisted by open_db + flush).
    push_cluster(map, 0.0, 0.0, 20);
    assert(map.flush_dirty_to_db());

    // Cluster B far away (also persisted).
    push_cluster(map, 300.0, 0.0, 20);
    assert(map.flush_dirty_to_db());

    // 40 cells in memory total.
    assert(map.cell_count() == 40U);

    // Fly to cluster B area: cluster A should be evicted.
    map.slide_window(Vec3{300.0, 0.0, 0.0});
    // Cluster A (x∈[0,19]) is > 2×50 = 100 m from (300,0,0) → evicted.
    // Cluster B (x∈[300,319]) is within 50 m of (300,0,0) → retained.
    const std::size_t count_at_b = map.cell_count();
    assert(count_at_b == 20U);

    // Return to origin: cluster A reloaded, cluster B evicted.
    map.slide_window(Vec3{0.0, 0.0, 0.0});
    // cluster B (x∈[300,319]) > 100 m from (0,0,0) → evicted.
    // cluster A (x∈[0,19]) within 50 m → reloaded from DB.
    assert(map.cell_count() == 20U);

    assert(map.close_db());

    std::filesystem::remove(path);
    std::filesystem::remove(path.string() + "-wal");
    std::filesystem::remove(path.string() + "-shm");

    std::printf("PASS  test_reentry_reloads\n");
}

// ─── test 3: slide latency < 5 ms ────────────────────────────────────────────

void test_slide_latency() {
    const auto path = std::filesystem::temp_directory_path() /
                      "dedalus_spatial_test_latency.db";
    std::filesystem::remove(path);

    MissionLocalPlanningMapConfig cfg;
    cfg.horizon_m = 150.0;

    MissionLocalPlanningMap map{cfg};
    assert(map.open_db(path));

    // 5000 cells near origin.
    push_cluster(map, 0.0, 0.0, 5000);
    assert(map.flush_dirty_to_db());

    // Initial slide (loads from DB nothing new since cells already in memory).
    map.slide_window(Vec3{0.0, 0.0, 0.0});

    // Trigger eviction: move > 2×horizon = 300 m.
    const auto t0 = std::chrono::steady_clock::now();
    map.slide_window(Vec3{500.0, 0.0, 0.0});
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();

    assert(map.close_db());

    std::filesystem::remove(path);
    std::filesystem::remove(path.string() + "-wal");
    std::filesystem::remove(path.string() + "-shm");

    std::printf("INFO  slide of 5000 cells took %lld ms\n",
                static_cast<long long>(elapsed_ms));
    assert(elapsed_ms < 5 && "slide_window latency >= 5 ms");

    std::printf("PASS  test_slide_latency\n");
}

// ─── test 4: no-op when DB is closed ─────────────────────────────────────────

void test_noop_without_db() {
    MissionLocalPlanningMap map;

    // No DB open — slide_window must not crash and must be a true no-op.
    push_cluster(map, 0.0, 0.0, 10);
    assert(map.cell_count() == 10U);

    map.slide_window(Vec3{1000.0, 0.0, 0.0});
    // Cells must NOT be evicted (no DB to reload from).
    assert(map.cell_count() == 10U);

    std::printf("PASS  test_noop_without_db\n");
}

}  // namespace

int main() {
    test_eviction_bounds_memory();
    test_reentry_reloads();
    test_slide_latency();
    test_noop_without_db();
    std::printf("OK    all spatial window tests passed\n");
    return 0;
}
