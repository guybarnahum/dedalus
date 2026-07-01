// test_esdf_streaming.cpp
//
// Stage 6 validation: LocalESDFMap::snapshot() + to_compact_stream_json().
//
//   1. snapshot_fields:  JSON contains required keys.
//   2. clearance_correct: is_clear() and net_repulsion direction verified on
//      a known flat-wall geometry.
//   3. serialize_perf:   to_compact_stream_json() for a ~2600-cell ESDF < 20 ms.

#include "dedalus/avoidance/local_esdf_map.hpp"
#include "dedalus/avoidance/local_esdf_map_publisher.hpp"
#include "dedalus/avoidance/mission_local_planning_map.hpp"
#include "dedalus/avoidance/mission_local_traversability_map.hpp"

#include <cassert>
#include <chrono>
#include <cstdio>
#include <string>

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

// ─── test 1: snapshot JSON fields ─────────────────────────────────────────────

void test_snapshot_fields() {
    MissionLocalPlanningMap l2;
    insert_occupied(l2, 10.5, 0.5, 1.0);

    const Vec3 drone{5.0, 0.0, 1.0};
    auto esdf = compute_esdf(l2, drone, 12.0, 4.0, 5.0);
    auto snap = esdf.snapshot(drone, 1.0);
    snap.seq = 42U;

    const auto json = to_compact_stream_json(snap, 0U);

    // Required keys
    assert(json.find("\"cell_size_m\"") != std::string::npos);
    assert(json.find("\"vcell_size_m\"") != std::string::npos);
    assert(json.find("\"d0_m\"") != std::string::npos);
    assert(json.find("\"is_delta\"") != std::string::npos);
    assert(json.find("\"net_rep\"") != std::string::npos);
    assert(json.find("\"cells\"") != std::string::npos);
    // Cell fields
    assert(json.find("\"d\"") != std::string::npos);
    assert(json.find("\"gx\"") != std::string::npos);
    // Has actual cells
    assert(snap.cell_count > 0U);

    std::printf("PASS  test_snapshot_fields  (json size=%zu, cells=%zu)\n",
                json.size(), snap.cell_count);
}

// ─── test 2: clearance + net repulsion direction ───────────────────────────────

void test_clearance_correct() {
    MissionLocalPlanningMap l2;

    // Flat wall at x = 10 (cells at x=10.5), y = -3..3.
    for (int y = -3; y <= 3; ++y) {
        insert_occupied(l2, 10.5, static_cast<double>(y) + 0.5, 1.0);
    }

    const Vec3 centre{5.0, 0.0, 1.0};
    auto esdf = compute_esdf(l2, centre, 10.0, 4.0, 5.0, 1.0);

    // 4 m left of wall: drone at x=6.5, wall at x=10.5 → d ≈ 4 m
    const Vec3 query{6.5, 0.5, 1.0};
    const auto r = esdf.query(query);
    assert(r.d > 0.0f);
    assert(std::abs(r.d - 4.0f) < 0.2f);

    // is_clear with r=3m should pass; r=5m should fail.
    assert(esdf.is_clear(query, 3.0));
    assert(!esdf.is_clear(query, 5.0));

    // Net repulsion at query points away from wall (in -x direction).
    auto snap = esdf.snapshot(query, 1.0);
    // Repulsion force is only non-zero when d < d0 = 5.
    // d ≈ 4 < 5, so repulsion should be non-zero and negative x.
    assert(snap.net_repulsion.x < -0.001);

    std::printf("PASS  test_clearance_correct  (d=%.2f, rep.x=%.4f)\n",
                static_cast<double>(r.d), snap.net_repulsion.x);
}

// ─── test 3: serialize perf < 20 ms for ~2600 cells ───────────────────────────

void test_serialize_perf() {
    MissionLocalPlanningMap l2;

    // Dense obstacle cluster to produce many shell cells.
    for (int x = -10; x <= 10; ++x) {
        for (int y = -10; y <= 10; ++y) {
            insert_occupied(l2,
                            static_cast<double>(x) + 0.5,
                            static_cast<double>(y) + 0.5,
                            1.0);
        }
    }

    const Vec3 centre{0.0, 0.0, 1.0};
    auto esdf = compute_esdf(l2, centre, 20.0, 6.0, 5.0);
    const auto snap = esdf.snapshot(centre, 1.0);

    std::printf("INFO  serialize_perf: %zu shell cells\n", snap.cell_count);

    const auto t0 = std::chrono::steady_clock::now();
    const auto json = to_compact_stream_json(snap, 0U);
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();

    assert(!json.empty());
    std::printf("INFO  to_compact_stream_json %zu cells → %lld ms\n",
                snap.cell_count, static_cast<long long>(elapsed_ms));
    assert(elapsed_ms < 20 && "serialize ESDF >= 20 ms");

    std::printf("PASS  test_serialize_perf\n");
}

}  // namespace

int main() {
    test_snapshot_fields();
    test_clearance_correct();
    test_serialize_perf();
    std::printf("OK    all ESDF streaming tests passed\n");
    return 0;
}
