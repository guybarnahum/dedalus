#include "dedalus/avoidance/local_flight_map.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace dedalus {
namespace {

constexpr double kNanosecondsPerSecond = 1'000'000'000.0;

std::uint64_t timestamp_ns(const TimePoint& time) {
    return static_cast<std::uint64_t>(time.timestamp_ns);
}

float clamp_non_negative(const float value) {
    return value < 0.0F ? 0.0F : value;
}

bool is_finite_positive(const float value) {
    return std::isfinite(value) && value > 0.0F;
}

}  // namespace

void LocalFlightMapAccumulator::ingest_obstacle_evidence(const WorldSnapshot& snapshot) {
    std::size_t consumed = 0U;
    for (const auto& evidence : snapshot.obstacle_evidence) {
        if (consumed >= config_.max_evidence_per_update) {
            break;
        }
        if (!evidence_is_usable(evidence)) {
            continue;
        }
        ingest_single_evidence(evidence, snapshot.timestamp);
        ++consumed;
    }
}

void LocalFlightMapAccumulator::ingest_single_evidence(
    const ObstacleEvidence& evidence,
    const TimePoint snapshot_time) {
    switch (evidence.state) {
        case ObstacleEvidenceState::Occupied:
        case ObstacleEvidenceState::ThinStructureRisk:
            splat_occupied_evidence(evidence, snapshot_time);
            break;
        case ObstacleEvidenceState::Free:
            splat_free_evidence(evidence, snapshot_time);
            break;
        case ObstacleEvidenceState::Unknown:
        default:
            break;
    }
}

void LocalFlightMapAccumulator::splat_occupied_evidence(
    const ObstacleEvidence& evidence,
    const TimePoint snapshot_time) {
    const auto maybe_center = local_to_grid(evidence.center_local);
    if (!maybe_center.has_value()) {
        return;
    }

    const int radius_cells = footprint_radius_cells(evidence_footprint_radius_m(evidence));
    const auto now_ns = timestamp_ns(snapshot_time);
    const float confidence = std::clamp(evidence.confidence, 0.05F, 1.0F);
    const float occupied_delta = config_.occupied_score_hit * confidence;
    const float risk_delta =
        evidence.state == ObstacleEvidenceState::ThinStructureRisk || evidence.is_thin_structure_hint
            ? config_.thin_risk_score_hit * confidence
            : 0.0F;

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            if ((dx * dx) + (dy * dy) > radius_cells * radius_cells) {
                continue;
            }
            auto* cell = mutable_cell_at(maybe_center->ix + dx, maybe_center->iy + dy);
            if (cell == nullptr) {
                continue;
            }

            cell->occupied_score = std::min(1.0F, cell->occupied_score + occupied_delta);
            cell->risk_score = std::min(1.0F, cell->risk_score + risk_delta);
            cell->free_score = std::max(0.0F, cell->free_score - 0.25F * occupied_delta);
            if (is_finite_positive(evidence.range_m)) {
                cell->nearest_range_m = std::min(cell->nearest_range_m, evidence.range_m);
            }
            cell->min_z_m = evidence.center_local.z - (0.5F * evidence.size_m.z);
            cell->max_z_m = evidence.center_local.z + (0.5F * evidence.size_m.z);
            cell->last_observed_ns = now_ns;
            cell->recently_observed = true;
        }
    }
}

void LocalFlightMapAccumulator::splat_free_evidence(
    const ObstacleEvidence& evidence,
    const TimePoint snapshot_time) {
    const auto maybe_center = local_to_grid(evidence.center_local);
    if (!maybe_center.has_value()) {
        return;
    }

    auto* cell = mutable_cell_at(maybe_center->ix, maybe_center->iy);
    if (cell == nullptr) {
        return;
    }

    const float confidence = std::clamp(evidence.confidence, 0.05F, 1.0F);
    const float free_delta = config_.free_score_hit * confidence;
    cell->free_score = std::min(1.0F, cell->free_score + free_delta);
    cell->occupied_score = clamp_non_negative(cell->occupied_score - 0.15F * free_delta);
    cell->risk_score = clamp_non_negative(cell->risk_score - 0.10F * free_delta);
    cell->last_observed_ns = timestamp_ns(snapshot_time);
    cell->recently_observed = true;
}

void LocalFlightMapAccumulator::classify_cells() {
    for (auto& cell : latest_.cells) {
        cell.occupied =
            cell.occupied_score >= config_.occupied_threshold ||
            cell.risk_score >= config_.risk_threshold;
        cell.inflated_blocked = false;
    }
}

void LocalFlightMapAccumulator::inflate_blocked_cells() {
    const float inflation_radius_m = std::max(0.0F, config_.vehicle_radius_m + config_.safety_margin_m);
    const int inflation_cells = footprint_radius_cells(inflation_radius_m);
    if (inflation_cells <= 0) {
        for (auto& cell : latest_.cells) {
            cell.inflated_blocked = cell.occupied;
        }
        return;
    }

    std::vector<LocalFlightMapIndex> occupied_indices;
    occupied_indices.reserve(latest_.cells.size());

    for (int iy = 0; iy < latest_.y_cells; ++iy) {
        for (int ix = 0; ix < latest_.x_cells; ++ix) {
            const auto* cell = cell_at(ix, iy);
            if (cell != nullptr && cell->occupied) {
                occupied_indices.push_back(LocalFlightMapIndex{ix, iy});
            }
        }
    }

    for (const auto& occupied : occupied_indices) {
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
            for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                if ((dx * dx) + (dy * dy) > inflation_cells * inflation_cells) {
                    continue;
                }
                auto* cell = mutable_cell_at(occupied.ix + dx, occupied.iy + dy);
                if (cell != nullptr) {
                    cell->inflated_blocked = true;
                }
            }
        }
    }
}

void LocalFlightMapAccumulator::update_summary() {
    latest_.occupied_count = 0U;
    latest_.inflated_blocked_count = 0U;
    latest_.nearest_obstacle_m = std::numeric_limits<float>::infinity();

    for (const auto& cell : latest_.cells) {
        if (cell.occupied) {
            ++latest_.occupied_count;
            latest_.nearest_obstacle_m = std::min(latest_.nearest_obstacle_m, cell.nearest_range_m);
        }
        if (cell.inflated_blocked) {
            ++latest_.inflated_blocked_count;
        }
    }
}

}  // namespace dedalus
