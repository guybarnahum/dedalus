// test_local_esdf_map_incremental.cpp
//
// Stage 4 validation: update_incremental + update_tube.
//   1. Incremental ≡ full: adding a wall incrementally matches full recompute
//      (< 0.01 m error at every queried point in the affected region).
//   2. Incremental perf: < 1 ms for 50 dirty cells.
//   3. Tube correctness: distances along a trajectory tube are correct.
//   4. Tube perf: < 2 ms for a 5-waypoint trajectory.

#include "dedalus/avoidance/local_esdf_map.hpp"
#include "dedalus/avoidance/mission_local_planning_map.hpp"
#include "dedalus/avoidance/mission_local_traversability_map.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

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

// ─── test 1: incremental ≡ full ───────────────────────────────────────────────

void test_incremental_equiv_full() {
    MissionLocalPlanningMap l2;

    // Start with an empty ESDF centred at (5, 0, 1).
    const Vec3 centre{5.0, 0.0, 1.0};
    auto esdf = compute_esdf(l2, centre, 12.0, 4.0, 6.0, 1.0);

    // Add a flat wall at x = 8 spanning y = −3..3.
    std::vector<Vec3> dirty;
    for (int y = -3; y <= 3; ++y) {
        const double yc = static_cast<double>(y) + 0.5;
        insert_occupied(l2, 8.5, yc, 1.0);
        dirty.push_back(Vec3{8.5, yc, 1.0});
    }

    // Incremental update.
    esdf.update_incremental(l2, dirty, 6.0);

    // Full recompute for reference.
    const auto esdf_full = compute_esdf(l2, centre, 12.0, 4.0, 6.0, 1.0);

    // Compare at a grid of points in the affected region.
    for (int xi = 3; xi <= 11; ++xi) {
        for (int yi = -3; yi <= 3; ++yi) {
            const Vec3 pos{static_cast<double>(xi) + 0.5,
                           static_cast<double>(yi) + 0.5,
                           1.0};
            const auto r_incr = esdf.query(pos);
            const auto r_full = esdf_full.query(pos);
            assert(std::abs(r_incr.d - r_full.d) < 0.01f);
        }
    }

    std::printf("PASS  test_incremental_equiv_full\n");
}

// ─── test 2: incremental perf < 1 ms for 50 dirty cells ─────────────────────

void test_incremental_perf() {
    MissionLocalPlanningMap l2;

    // A background wall at x = 5 to give the ESDF something real.
    for (int y = -5; y <= 5; ++y) {
        insert_occupied(l2, 5.5, static_cast<double>(y) + 0.5, 1.0);
    }
    auto esdf = compute_esdf(l2, Vec3{10.0, 0.0, 1.0}, 15.0, 5.0, 6.0);

    // 50 new dirty cells in a 5×10×1 patch.
    std::vector<Vec3> dirty;
    dirty.reserve(50);
    for (int i = 0; i < 50; ++i) {
        const double x = static_cast<double>(i % 5) + 20.5;
        const double y = static_cast<double>(i / 5) + 0.5;
        insert_occupied(l2, x, y, 1.0);
        dirty.push_back(Vec3{x, y, 1.0});
    }

    const auto t0 = std::chrono::steady_clock::now();
    esdf.update_incremental(l2, dirty, 6.0);
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();

    std::printf("INFO  update_incremental 50 dirty cells → %lld ms\n",
                static_cast<long long>(elapsed_ms));
    assert(elapsed_ms < 1 && "update_incremental >= 1 ms");

    std::printf("PASS  test_incremental_perf\n");
}

// ─── test 3: tube correctness ─────────────────────────────────────────────────

void test_tube_correctness() {
    MissionLocalPlanningMap l2;

    // Wall at x = 15 spanning y = −5..5, z = 1.
    for (int y = -5; y <= 5; ++y) {
        insert_occupied(l2, 15.5, static_cast<double>(y) + 0.5, 1.0);
    }

    // Start with empty ESDF, then fill via update_tube along a straight path.
    LocalESDFConfig cfg;
    cfg.cell_size_m          = 1.0;
    cfg.vertical_cell_size_m = 2.0;
    cfg.d0_m                 = 6.0;
    cfg.sample_spacing_m     = 1.0;  // fine resolution for correctness checks
    LocalESDFMap esdf(cfg);

    // Trajectory: x = 10..14 at y = 0, z = 1.
    const std::vector<Vec3> waypoints{
        Vec3{10.5, 0.5, 1.0},
        Vec3{12.5, 0.5, 1.0},
        Vec3{14.5, 0.5, 1.0},
    };
    esdf.update_tube(l2, waypoints, 2.0);

    // 1 m left of wall (query at x=14.5, wall at x=15.5) → d ≈ 1 m.
    // Gradient points away from the wall, i.e. in the −x direction.
    {
        const auto r = esdf.query(Vec3{14.5, 0.5, 1.0});
        assert(r.d > 0.0f);
        assert(std::abs(r.d - 1.0f) < 0.1f);
        assert(r.grad.x < -0.4);  // gradient points away from wall (−x)
    }
    // 3 m left of wall → d ≈ 3 m.
    {
        const auto r = esdf.query(Vec3{12.5, 0.5, 1.0});
        assert(r.d > 0.0f);
        assert(std::abs(r.d - 3.0f) < 0.15f);
    }

    std::printf("PASS  test_tube_correctness\n");
}

// ─── test 4: tube perf < 2 ms ─────────────────────────────────────────────────

void test_tube_perf() {
    MissionLocalPlanningMap l2;

    // Scatter 50 occupied cells.
    for (int i = 0; i < 50; ++i) {
        insert_occupied(l2,
                        static_cast<double>(i % 10) + 0.5,
                        static_cast<double>(i / 10) + 0.5,
                        1.0);
    }

    LocalESDFConfig cfg;
    cfg.cell_size_m          = 1.0;
    cfg.vertical_cell_size_m = 2.0;
    cfg.d0_m                 = 5.0;
    LocalESDFMap esdf(cfg);

    // 5-waypoint trajectory.
    const std::vector<Vec3> waypoints{
        Vec3{5.5,  0.5, 1.0},
        Vec3{8.5,  2.5, 1.0},
        Vec3{11.5, 2.5, 1.0},
        Vec3{14.5, 1.5, 1.0},
        Vec3{17.5, 0.5, 1.0},
    };

    const auto t0 = std::chrono::steady_clock::now();
    esdf.update_tube(l2, waypoints, 3.0);
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();

    std::printf("INFO  update_tube 5 waypoints → %lld ms\n",
                static_cast<long long>(elapsed_ms));
    assert(elapsed_ms < 2 && "update_tube >= 2 ms");

    std::printf("PASS  test_tube_perf\n");
}

}  // namespace

int main() {
    test_incremental_equiv_full();
    test_incremental_perf();
    test_tube_correctness();
    test_tube_perf();
    std::printf("OK    all incremental ESDF tests passed\n");
    return 0;
}
