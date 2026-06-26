#include "dedalus/runtime/perch_candidate_evaluator.hpp"

#include <algorithm>
#include <cmath>

namespace dedalus {

PerchCandidateEvaluator::PerchCandidateEvaluator(PerchCandidateEvaluatorConfig config)
    : config_(config) {}

std::vector<PerchCandidate> PerchCandidateEvaluator::evaluate(
    const std::vector<ObstacleEvidence>& evidence,
    const MissionLocalPlanningMap* l2_map) const {
    std::vector<PerchCandidate> candidates;

    for (const auto& e : evidence) {
        if (!e.is_surface_hint) continue;
        if (!e.has_surface_normal) continue;
        if (e.confidence < config_.min_confidence) continue;

        // Flatness: |n.z| approaches 1 for horizontal surfaces, 0 for vertical.
        // Works for both NED (up=-Z, normal.z≈-1) and ENU (up=+Z, normal.z≈+1).
        const auto flatness = static_cast<float>(std::abs(e.surface_normal_local.z));
        if (flatness < config_.min_flatness_dot) continue;

        // Area from bounding-box XY footprint.
        const auto area_m2 = static_cast<float>(e.size_m.x * e.size_m.y);
        if (area_m2 < config_.min_area_m2) continue;

        // Vertical clearance: ray upward (+Z in ENU / -Z in NED) from the
        // surface centre.  If an L2 obstacle is hit within min_clearance_m,
        // the candidate is not landable (e.g. covered rooftop overhang).
        if (l2_map != nullptr && l2_map->cell_count() > 0U) {
            const Vec3 up{0.0, 0.0, 1.0};   // ENU world-up; NED negated below if needed
            const auto hit = l2_map->ray_cast(e.center_local, up, config_.min_clearance_m);
            if (hit.has_value()) continue;   // obstacle overhead — reject
        }

        // area_score: linear ramp 0→1 over [min_area, 4×min_area].
        const float area_score = std::min(1.0F, area_m2 / (config_.min_area_m2 * 4.0F));

        const float score = flatness * area_score * e.confidence;
        candidates.push_back({e.center_local, e.surface_normal_local, score});
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const PerchCandidate& a, const PerchCandidate& b) {
                  return a.score > b.score;
              });

    if (candidates.size() > config_.max_candidates) {
        candidates.resize(config_.max_candidates);
    }

    return candidates;
}

}  // namespace dedalus
