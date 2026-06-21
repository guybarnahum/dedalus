// test_local_esdf_map.cpp
//
// Stage 3 validation: compute_esdf + LocalESDFMap.
//   - Flat wall: correct distance field and gradient direction.
//   - Corner geometry: sqrt(2) distance to diagonal neighbour.
//   - Signed field: occupied cells have d = −0.5 m.
//   - Clearance: is_clear() correct for varying radii.
//   - Perf: < 5 ms for 80×80×20 m window.

#include "dedalus/avoidance/local_esdf_map.hpp"
#include "dedalus/avoidance/mission_local_planning_map.hpp"
#include "dedalus/avoidance/mission_local_traversability_map.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace {

using namespace dedalus;

// ─── helper: insert one occupied L2 cell at world position ────────────────────

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

// ─── test 1: flat wall — distance and gradient ────────────────────────────────

void test_flat_wall() {
    MissionLocalPlanningMap l2;

    // Wall of cells at xi=10 (centre x=10.5) spanning y=−5..5, z=1
    for (int y = -5; y <= 5; ++y) {
        insert_occupied(l2, 10.5, static_cast<double>(y) + 0.5, 1.0);
    }

    // Centre the ESDF window on (15, 0, 1), half=20 m, d0=8 m.
    const auto esdf = compute_esdf(l2, Vec3{15.0, 0.0, 1.0}, 20.0, 8.0);

    // xi=10 (centre 10.5) — occupied cell
    {
        const auto r = esdf.query(Vec3{10.5, 0.5, 1.0});
        assert(r.d < 0.0f);
        assert(std::abs(r.d - (-0.5f)) < 0.01f);
    }
    // xi=11 (centre 11.5) — free, 1 m from wall
    {
        const auto r = esdf.query(Vec3{11.5, 0.5, 1.0});
        assert(std::abs(r.d - 1.0f) < 0.05f);
        // Gradient should point away from wall (positive x direction)
        assert(r.grad.x > 0.5);
    }
    // xi=14 (centre 14.5) — free, 4 m from wall
    {
        const auto r = esdf.query(Vec3{14.5, 0.5, 1.0});
        assert(std::abs(r.d - 4.0f) < 0.1f);
    }
    // xi=9 (centre 9.5) — free cell on the other side, 1 m from wall
    {
        const auto r = esdf.query(Vec3{9.5, 0.5, 1.0});
        assert(std::abs(r.d - 1.0f) < 0.05f);
        // Gradient points in −x direction (away from wall)
        assert(r.grad.x < -0.5);
    }

    std::printf("PASS  test_flat_wall\n");
}

// ─── test 2: corner geometry — diagonal distance ──────────────────────────────

void test_corner_geometry() {
    MissionLocalPlanningMap l2;

    // Single occupied cell at (0.5, 0.5, 1.0) → (xi=0, yi=0, zi=0)
    insert_occupied(l2, 0.5, 0.5, 1.0);

    const auto esdf = compute_esdf(l2, Vec3{5.0, 5.0, 1.0}, 10.0, 8.0);

    // Diagonal neighbour at (2.5, 2.5, 1.0) = 2 cells away in X and Y.
    // Expected d = sqrt((2.5-0.5)^2 + (2.5-0.5)^2) = sqrt(8) ≈ 2.83 m
    {
        const auto r = esdf.query(Vec3{2.5, 2.5, 1.0});
        const float expected = std::sqrt(8.0f);
        assert(std::abs(r.d - expected) < 0.1f);
    }
    // Direct neighbour at (1.5, 0.5, 1.0) — 1 m away
    {
        const auto r = esdf.query(Vec3{1.5, 0.5, 1.0});
        assert(std::abs(r.d - 1.0f) < 0.05f);
    }

    std::printf("PASS  test_corner_geometry\n");
}

// ─── test 3: signed field — occupied cells d = −0.5 ──────────────────────────

void test_signed_field() {
    MissionLocalPlanningMap l2;

    // 3×3 block of occupied cells at z=1
    for (int xi = 5; xi <= 7; ++xi) {
        for (int yi = 5; yi <= 7; ++yi) {
            insert_occupied(l2,
                            static_cast<double>(xi) + 0.5,
                            static_cast<double>(yi) + 0.5,
                            1.0);
        }
    }

    const auto esdf = compute_esdf(l2, Vec3{6.5, 6.5, 1.0}, 10.0, 6.0);

    // All occupied cells must have d = −0.5
    for (int xi = 5; xi <= 7; ++xi) {
        for (int yi = 5; yi <= 7; ++yi) {
            const auto r = esdf.query(Vec3{static_cast<double>(xi) + 0.5,
                                           static_cast<double>(yi) + 0.5,
                                           1.0});
            assert(r.d < 0.0f);
            assert(std::abs(r.d - (-0.5f)) < 0.01f);
        }
    }

    // Free cell adjacent to block — d ≈ 1 m
    {
        const auto r = esdf.query(Vec3{8.5, 6.5, 1.0});
        assert(r.d > 0.0f);
        assert(std::abs(r.d - 1.0f) < 0.1f);
    }

    std::printf("PASS  test_signed_field\n");
}

// ─── test 4: is_clear ─────────────────────────────────────────────────────────

void test_is_clear() {
    MissionLocalPlanningMap l2;
    insert_occupied(l2, 10.5, 0.5, 1.0);

    const auto esdf = compute_esdf(l2, Vec3{10.0, 0.0, 1.0}, 8.0, 6.0);

    // Cell at d=1 m: clear for r=0.9, blocked for r=1.1
    assert( esdf.is_clear(Vec3{11.5, 0.5, 1.0}, 0.9));
    assert(!esdf.is_clear(Vec3{11.5, 0.5, 1.0}, 1.1));

    std::printf("PASS  test_is_clear\n");
}

// ─── test 5: performance < 5 ms for 80×80×20 m window ────────────────────────

void test_perf() {
    MissionLocalPlanningMap l2;

    // Scatter 200 occupied cells across the window
    for (int i = 0; i < 200; ++i) {
        insert_occupied(l2,
                        static_cast<double>(i % 40) + 0.5,
                        static_cast<double>(i / 40) + 0.5,
                        1.0);
    }

    const auto t0 = std::chrono::steady_clock::now();
    const auto esdf = compute_esdf(l2, Vec3{20.0, 20.0, 10.0}, 40.0, 5.0);
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();

    std::printf("INFO  compute_esdf 80×80×20 m → %zu shell cells in %lld ms\n",
                esdf.cell_count(), static_cast<long long>(elapsed_ms));
    assert(elapsed_ms < 5 && "compute_esdf >= 5 ms");

    std::printf("PASS  test_perf\n");
}

}  // namespace

int main() {
    test_flat_wall();
    test_corner_geometry();
    test_signed_field();
    test_is_clear();
    test_perf();
    std::printf("OK    all ESDF tests passed\n");
    return 0;
}
