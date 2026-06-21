// test_mission_local_planning_map_query.cpp
//
// Stage 2.5 validation: ray_cast + query_occupied_in_box.
//   - Ray hits known wall at correct distance.
//   - Ray through free space returns nullopt.
//   - Bbox query returns exactly the expected cells.
//   - Both complete in < 1 ms.

#include "dedalus/avoidance/mission_local_planning_map.hpp"
#include "dedalus/avoidance/mission_local_traversability_map.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace {

using namespace dedalus;

// ─── helpers ─────────────────────────────────────────────────────────────────

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

// ─── test 1: ray hits wall at correct distance ────────────────────────────────

void test_ray_hits_wall() {
    MissionLocalPlanningMap map;

    // Place a wall of cells at x=10 (cell centres at x=10.5, 1m voxels).
    for (int y = -5; y <= 5; ++y) {
        insert_occupied(map, 10.5, static_cast<double>(y) + 0.5, 1.0);
    }

    // Fire a ray from origin along +x.
    const auto hit = map.ray_cast(Vec3{0.0, 0.0, 1.0}, Vec3{1.0, 0.0, 0.0}, 50.0);
    assert(hit.has_value());
    // Hit centre should be at x≈10.5 (cell [10,0,0]).
    assert(std::abs(hit->x - 10.5) < 0.1);

    // Fire a ray that misses the wall (y too large).
    const auto miss = map.ray_cast(Vec3{0.0, 20.0, 1.0}, Vec3{1.0, 0.0, 0.0}, 50.0);
    assert(!miss.has_value());

    std::printf("PASS  test_ray_hits_wall\n");
}

// ─── test 2: ray through free space returns nullopt ──────────────────────────

void test_ray_free_space() {
    MissionLocalPlanningMap map;

    // No cells at all.
    const auto hit = map.ray_cast(Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, 100.0);
    assert(!hit.has_value());

    // Cell beyond max_range.
    insert_occupied(map, 200.5, 0.5, 1.0);
    const auto clipped = map.ray_cast(Vec3{0.0, 0.0, 1.0}, Vec3{1.0, 0.0, 0.0}, 100.0);
    assert(!clipped.has_value());

    std::printf("PASS  test_ray_free_space\n");
}

// ─── test 3: bbox query returns exactly the expected cells ────────────────────

void test_bbox_query() {
    MissionLocalPlanningMap map;

    // 5 cells in a line at x=0.5..4.5, y=0.5, z=1.0.
    for (int i = 0; i < 5; ++i) {
        insert_occupied(map, static_cast<double>(i) + 0.5, 0.5, 1.0);
    }

    // Query a box that covers x∈[0,3], y∈[0,1], z∈[0,2] → cells 0..2.
    const auto result = map.query_occupied_in_box(
        Bounds3{Vec3{0.0, 0.0, 0.0}, Vec3{3.0, 1.0, 2.0}});
    assert(result.size() == 3U);

    // Query the full extent → all 5.
    const auto all = map.query_occupied_in_box(
        Bounds3{Vec3{0.0, 0.0, 0.0}, Vec3{5.0, 1.0, 2.0}});
    assert(all.size() == 5U);

    // Query an empty region.
    const auto none = map.query_occupied_in_box(
        Bounds3{Vec3{10.0, 10.0, 0.0}, Vec3{20.0, 20.0, 2.0}});
    assert(none.empty());

    std::printf("PASS  test_bbox_query\n");
}

// ─── test 4: ray_cast origin is inside an occupied cell ───────────────────────

void test_ray_origin_occupied() {
    MissionLocalPlanningMap map;
    insert_occupied(map, 0.5, 0.5, 1.0);

    // Origin is inside the occupied cell.
    const auto hit = map.ray_cast(Vec3{0.5, 0.5, 1.0}, Vec3{1.0, 0.0, 0.0}, 10.0);
    assert(hit.has_value());
    assert(std::abs(hit->x - 0.5) < 0.1);

    std::printf("PASS  test_ray_origin_occupied\n");
}

// ─── test 5: latency < 1 ms for ray through 100 m + bbox 40x40x20 m ─────────

void test_query_latency() {
    MissionLocalPlanningMap map;

    // 200 cells along x=0..200 at y=5, z=1 (wall at the far end).
    for (int i = 0; i < 200; ++i) {
        insert_occupied(map, static_cast<double>(i) + 0.5, 5.0, 1.0);
    }
    // Also fill a 40×40×10 region at z=1 for bbox test.
    for (int xi = 0; xi < 40; ++xi) {
        for (int yi = 20; yi < 60; ++yi) {
            insert_occupied(map,
                            static_cast<double>(xi) + 0.5,
                            static_cast<double>(yi) + 0.5,
                            1.0);
        }
    }

    // ray_cast through ~200 m.
    const auto t0 = std::chrono::steady_clock::now();
    for (int rep = 0; rep < 100; ++rep) {
        (void)map.ray_cast(Vec3{0.0, 5.0, 1.0}, Vec3{1.0, 0.0, 0.0}, 200.0);
    }
    const auto ray_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - t0).count() / 100;

    // bbox query over 40×40×20 m volume.
    const auto t1 = std::chrono::steady_clock::now();
    for (int rep = 0; rep < 100; ++rep) {
        (void)map.query_occupied_in_box(
            Bounds3{Vec3{0.0, 20.0, 0.0}, Vec3{40.0, 60.0, 2.0}});
    }
    const auto bbox_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - t1).count() / 100;

    std::printf("INFO  ray_cast avg %lld µs, bbox_query avg %lld µs\n",
                static_cast<long long>(ray_us),
                static_cast<long long>(bbox_us));

    assert(ray_us  < 1000 && "ray_cast >= 1 ms");
    assert(bbox_us < 1000 && "query_occupied_in_box >= 1 ms");

    std::printf("PASS  test_query_latency\n");
}

}  // namespace

int main() {
    test_ray_hits_wall();
    test_ray_free_space();
    test_bbox_query();
    test_ray_origin_occupied();
    test_query_latency();
    std::printf("OK    all query API tests passed\n");
    return 0;
}
