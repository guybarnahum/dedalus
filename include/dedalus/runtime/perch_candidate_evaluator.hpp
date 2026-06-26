#pragma once

#include <cstddef>
#include <vector>

#include "dedalus/occupancy/occupancy_types.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

struct PerchCandidateEvaluatorConfig {
    // Surface normal must be within this angle of horizontal
    // (|n.z| >= min_flatness_dot).  cos(30°) ≈ 0.866.
    float min_flatness_dot{0.866F};

    // Minimum bounding-box XY area for a usable landing zone.
    // Default: 0.4 m² ≈ 63×63 cm (drone footprint + margin).
    float min_area_m2{0.4F};

    // Discard evidence below this confidence.
    float min_confidence{0.1F};

    // Top-N candidates returned (sorted by composite score, best first).
    std::size_t max_candidates{10U};

    // Run evaluation every N ticks.  At 30 Hz tick rate, 10 → ~3 Hz.
    int cadence_ticks{10};
};

// Stateless evaluator: filters and ranks surface-patch ObstacleEvidence
// into a list of PerchCandidate records.
//
// Thread ownership: single-threaded tick.  evaluate() is not reentrant.
class PerchCandidateEvaluator {
public:
    explicit PerchCandidateEvaluator(PerchCandidateEvaluatorConfig config = {});

    [[nodiscard]] std::vector<PerchCandidate> evaluate(
        const std::vector<ObstacleEvidence>& evidence) const;

    int cadence_ticks() const { return config_.cadence_ticks; }

private:
    PerchCandidateEvaluatorConfig config_;
};

}  // namespace dedalus
