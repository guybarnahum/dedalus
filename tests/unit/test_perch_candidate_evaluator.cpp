// VD5: PerchCandidateEvaluator unit tests.
//
// Tests:
//   1. Flat horizontal surface patch → candidate produced with score > 0.
//   2. Vertical surface → filtered out (flatness below threshold).
//   3. is_surface_hint=false → filtered out.
//   4. Area below minimum → filtered out.
//   5. Candidates sorted by score, best first.
//   6. max_candidates cap is respected.

#include <cassert>
#include <cstdio>
#include <string>
#include <vector>

#include "dedalus/occupancy/occupancy_types.hpp"
#include "dedalus/runtime/perch_candidate_evaluator.hpp"

namespace {

void require(bool cond, const char* msg) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", msg);
        std::exit(1);
    }
}

// Build a minimal surface-patch ObstacleEvidence.
dedalus::ObstacleEvidence make_patch(
    dedalus::Vec3 center,
    dedalus::Vec3 normal,
    dedalus::Vec3 size,
    float confidence = 1.0F) {
    dedalus::ObstacleEvidence e;
    e.shape              = dedalus::ObstacleEvidenceShape::SurfacePatch;
    e.is_surface_hint    = true;
    e.has_surface_normal = true;
    e.center_local       = center;
    e.surface_normal_local = normal;
    e.size_m             = size;
    e.confidence         = confidence;
    return e;
}

// 1. Flat horizontal patch → candidate produced.
void test_flat_patch_produces_candidate() {
    dedalus::PerchCandidateEvaluator eval;

    // Normal = world-up (NED: -Z).  Generous 2×2 m footprint.
    auto patch = make_patch(
        {5.0, 0.0, -3.0},          // position
        {0.0, 0.0, -1.0},          // normal pointing up (NED)
        {2.0, 2.0, 0.1});          // size (4 m²)

    const auto candidates = eval.evaluate({patch});
    require(!candidates.empty(), "flat patch should produce a candidate");
    require(candidates[0].score > 0.0F, "candidate score should be positive");
    require(candidates[0].position_local.x == 5.0, "position_local.x should match center_local");
    std::puts("PASS test_flat_patch_produces_candidate");
}

// 2. Vertical surface normal → flatness filter rejects it.
void test_vertical_surface_filtered() {
    dedalus::PerchCandidateEvaluator eval;

    auto patch = make_patch(
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},           // pointing forward — vertical wall
        {2.0, 2.0, 0.1});

    const auto candidates = eval.evaluate({patch});
    require(candidates.empty(), "vertical wall normal should be filtered");
    std::puts("PASS test_vertical_surface_filtered");
}

// 3. is_surface_hint=false → filtered out.
void test_non_surface_hint_filtered() {
    dedalus::PerchCandidateEvaluator eval;

    auto patch = make_patch({0.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {2.0, 2.0, 0.1});
    patch.is_surface_hint = false;

    const auto candidates = eval.evaluate({patch});
    require(candidates.empty(), "non-surface-hint evidence should be filtered");
    std::puts("PASS test_non_surface_hint_filtered");
}

// 4. Area below minimum → filtered.
void test_small_area_filtered() {
    dedalus::PerchCandidateEvaluatorConfig cfg;
    cfg.min_area_m2 = 0.4F;
    dedalus::PerchCandidateEvaluator eval{cfg};

    // 0.1×0.1 m = 0.01 m² — well below 0.4 m²
    auto patch = make_patch({0.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {0.1, 0.1, 0.05});
    const auto candidates = eval.evaluate({patch});
    require(candidates.empty(), "sub-minimum area patch should be filtered");
    std::puts("PASS test_small_area_filtered");
}

// 5. Multiple patches: sorted by score, best first.
void test_sorted_by_score() {
    dedalus::PerchCandidateEvaluator eval;

    // Low confidence (score will be lower).
    auto weak = make_patch({1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {2.0, 2.0, 0.1}, 0.3F);
    // High confidence (score will be higher).
    auto strong = make_patch({2.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {2.0, 2.0, 0.1}, 0.9F);

    const auto candidates = eval.evaluate({weak, strong});
    require(candidates.size() == 2U, "both candidates should pass filters");
    require(candidates[0].score >= candidates[1].score, "should be sorted best-first");
    require(candidates[0].position_local.x == 2.0, "higher-confidence patch should be first");
    std::puts("PASS test_sorted_by_score");
}

// 6. max_candidates cap.
void test_max_candidates_cap() {
    dedalus::PerchCandidateEvaluatorConfig cfg;
    cfg.max_candidates = 3U;
    dedalus::PerchCandidateEvaluator eval{cfg};

    std::vector<dedalus::ObstacleEvidence> patches;
    for (int i = 0; i < 10; ++i) {
        patches.push_back(make_patch(
            {static_cast<double>(i), 0.0, 0.0},
            {0.0, 0.0, -1.0},
            {2.0, 2.0, 0.1},
            static_cast<float>(i + 1) / 10.0F));
    }

    const auto candidates = eval.evaluate(patches);
    require(candidates.size() == 3U, "max_candidates=3 should cap the list");
    std::puts("PASS test_max_candidates_cap");
}

}  // namespace

int main() {
    test_flat_patch_produces_candidate();
    test_vertical_surface_filtered();
    test_non_surface_hint_filtered();
    test_small_area_filtered();
    test_sorted_by_score();
    test_max_candidates_cap();
    std::puts("All perch_candidate_evaluator tests passed.");
    return 0;
}
