#include "dedalus/avoidance/mission_local_traversability_map.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace dedalus {
namespace {

std::uint64_t timestamp_ns(const TimePoint& time) {
    return time.timestamp_ns > 0 ? static_cast<std::uint64_t>(time.timestamp_ns) : 0U;
}

double elapsed_seconds(const TimePoint& from, const TimePoint& to) {
    const auto from_ns = timestamp_ns(from);
    const auto to_ns = timestamp_ns(to);
    if (to_ns <= from_ns) {
        return 0.0;
    }
    return static_cast<double>(to_ns - from_ns) * 1.0e-9;
}

bool finite_point(const Vec3& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double clamp01(const double value) {
    return std::max(0.0, std::min(value, 1.0));
}

double clamp_score(const double value, const double max_score) {
    return std::max(0.0, std::min(value, max_score));
}

double normalize_score(const double value, const double threshold) {
    if (!(threshold > 0.0)) {
        return value > 0.0 ? 1.0 : 0.0;
    }
    return clamp01(value / threshold);
}

double distance(const Vec3& a, const Vec3& b) {
    const auto dx = a.x - b.x;
    const auto dy = a.y - b.y;
    const auto dz = a.z - b.z;
    return std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
}

std::uint32_t capped_max_count(
    const std::uint32_t existing,
    const std::uint32_t observed,
    const std::uint32_t cap) {
    return std::max(existing, std::min(observed, cap));
}

bool frame_empty(const MapFrameId& frame_id) {
    return frame_id.value.empty();
}

bool frame_equal(const MapFrameId& lhs, const MapFrameId& rhs) {
    return lhs.value == rhs.value;
}

}  // namespace

MissionLocalTraversabilityMap::MissionLocalTraversabilityMap(
    MissionLocalTraversabilityMapConfig config)
    : config_(config) {
    if (!(config_.cell_size_m > 0.0)) {
        config_.cell_size_m = 0.5;
    }
    if (!(config_.vertical_cell_size_m > 0.0)) {
        config_.vertical_cell_size_m = config_.cell_size_m;
    }
    if (!(config_.required_clearance_m > 0.0)) {
        config_.required_clearance_m = 1.5;
    }
    if (!(config_.soft_clearance_m >= config_.required_clearance_m)) {
        config_.soft_clearance_m = config_.required_clearance_m;
    }
    if (!(config_.clearance_search_radius_m > 0.0)) {
        config_.clearance_search_radius_m = std::max(2.0 * config_.soft_clearance_m, config_.cell_size_m);
    }
}

std::size_t MissionLocalTraversabilityMap::CellKeyHash::operator()(const CellKey& key) const noexcept {
    std::size_t seed = 0U;
    const auto mix = [&seed](const int value) {
        seed ^= std::hash<int>{}(value) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    };
    mix(key.x);
    mix(key.y);
    mix(key.z);
    return seed;
}

MissionLocalTraversabilityMap::CellKey MissionLocalTraversabilityMap::key_for_point(
    const Vec3& point) const noexcept {
    return CellKey{
        static_cast<int>(std::floor(point.x / config_.cell_size_m)),
        static_cast<int>(std::floor(point.y / config_.cell_size_m)),
        static_cast<int>(std::floor(point.z / config_.vertical_cell_size_m)),
    };
}

Vec3 MissionLocalTraversabilityMap::center_for_key(const CellKey& key) const noexcept {
    return Vec3{
        (static_cast<double>(key.x) + 0.5) * config_.cell_size_m,
        (static_cast<double>(key.y) + 0.5) * config_.cell_size_m,
        (static_cast<double>(key.z) + 0.5) * config_.vertical_cell_size_m,
    };
}

MissionLocalTraversabilityCell& MissionLocalTraversabilityMap::ensure_cell(const CellKey& key) {
    const auto found = cell_index_.find(key);
    if (found != cell_index_.end()) {
        return cells_[found->second].cell;
    }

    MissionLocalTraversabilityCell cell;
    cell.center_map = center_for_key(key);
    cell.size_m = Vec3{config_.cell_size_m, config_.cell_size_m, config_.vertical_cell_size_m};

    cells_.push_back(StoredCell{key, cell});
    cell_index_.emplace(key, cells_.size() - 1U);
    ++summary_.new_cell_count;
    return cells_.back().cell;
}

const MissionLocalTraversabilityCell* MissionLocalTraversabilityMap::cell_at_key(
    const CellKey& key) const {
    const auto found = cell_index_.find(key);
    if (found == cell_index_.end()) {
        return nullptr;
    }
    return &cells_[found->second].cell;
}

void MissionLocalTraversabilityMap::apply_aging(const TimePoint now) {
    if (!has_last_update_) {
        return;
    }

    const auto seconds = elapsed_seconds(last_update_, now);
    if (seconds <= 0.0) {
        return;
    }

    for (auto& stored : cells_) {
        auto& cell = stored.cell;
        cell.occupied_score = clamp_score(
            cell.occupied_score - (config_.occupied_score_decay_per_second * seconds),
            config_.max_score);
        cell.free_score = clamp_score(
            cell.free_score - (config_.free_score_decay_per_second * seconds),
            config_.max_score);
        cell.confidence = clamp01(cell.confidence - (config_.confidence_decay_per_second * seconds));
    }
}

MissionLocalTraversabilityMapSnapshot MissionLocalTraversabilityMap::update_from_mission_obstacle_map(
    const MissionLocalObstacleMapSnapshot& obstacle_map,
    const TimePoint now) {
    apply_aging(now);

    if (frame_empty(summary_.map_frame_id) && !frame_empty(obstacle_map.summary.map_frame_id)) {
        summary_.map_frame_id = obstacle_map.summary.map_frame_id;
    }

    summary_.source_obstacle_cell_count = obstacle_map.cells.size();
    summary_.accepted_source_cell_count = 0U;
    summary_.new_cell_count = 0U;
    summary_.updated_cell_count = 0U;

    const auto now_ns = timestamp_ns(now);

    for (const auto& source : obstacle_map.cells) {
        if (!source.observed || !finite_point(source.center_map)) {
            continue;
        }
        if (!frame_empty(obstacle_map.summary.map_frame_id) &&
            !frame_empty(summary_.map_frame_id) &&
            !frame_equal(obstacle_map.summary.map_frame_id, summary_.map_frame_id)) {
            continue;
        }

        ++summary_.accepted_source_cell_count;

        const auto key = key_for_point(source.center_map);
        const auto existed = cell_index_.find(key) != cell_index_.end();
        auto& cell = ensure_cell(key);
        if (existed) {
            ++summary_.updated_cell_count;
        }

        cell.center_map = center_for_key(key);
        cell.size_m = Vec3{config_.cell_size_m, config_.cell_size_m, config_.vertical_cell_size_m};

        const auto source_time = source.last_observed_timestamp_ns != 0U
            ? source.last_observed_timestamp_ns
            : now_ns;
        if (cell.first_observed_timestamp_ns == 0U) {
            cell.first_observed_timestamp_ns =
                source.first_observed_timestamp_ns != 0U ? source.first_observed_timestamp_ns : source_time;
        }
        cell.last_observed_timestamp_ns = std::max(cell.last_observed_timestamp_ns, source_time);

        cell.occupied_score = clamp_score(
            std::max(cell.occupied_score, source.occupied_score),
            config_.max_score);
        cell.free_score = clamp_score(
            std::max(cell.free_score, source.free_score),
            config_.max_score);
        cell.confidence = clamp01(std::max(cell.confidence, source.confidence));

        cell.occupied_hits_capped = capped_max_count(
            cell.occupied_hits_capped,
            source.positive_observation_count,
            config_.max_evidence_count);
        cell.free_rays_capped = capped_max_count(
            cell.free_rays_capped,
            source.negative_observation_count,
            config_.max_evidence_count);
        cell.conflict_count_capped = capped_max_count(
            cell.conflict_count_capped,
            source.same_update_duplicate_count,
            config_.max_evidence_count);
        if (cell.refresh_count_capped < config_.max_evidence_count) {
            ++cell.refresh_count_capped;
        }
    }

    recompute_derived_fields(now);

    summary_.update_count += 1U;
    summary_.last_update_timestamp_ns = now_ns;
    last_update_ = now;
    has_last_update_ = true;

    refresh_summary();
    return snapshot();
}

void MissionLocalTraversabilityMap::recompute_derived_fields(const TimePoint now) {
    std::vector<const StoredCell*> occupied_cells;
    occupied_cells.reserve(cells_.size());
    for (const auto& stored : cells_) {
        const auto& cell = stored.cell;
        if (cell.occupied_score >= config_.occupied_threshold) {
            occupied_cells.push_back(&stored);
        }
    }

    const auto now_ns = timestamp_ns(now);
    for (auto& stored : cells_) {
        auto& cell = stored.cell;

        const auto occupied_strength = normalize_score(cell.occupied_score, config_.occupied_threshold);
        const auto free_strength = normalize_score(cell.free_score, config_.free_threshold);
        const auto strongest_evidence = std::max(occupied_strength, free_strength);
        cell.unknown_score = clamp01(1.0 - strongest_evidence);

        const auto age_seconds = cell.last_observed_timestamp_ns == 0U || now_ns <= cell.last_observed_timestamp_ns
            ? 0.0
            : static_cast<double>(now_ns - cell.last_observed_timestamp_ns) * 1.0e-9;
        cell.age_score = config_.stale_after_seconds > 0.0
            ? clamp01(age_seconds / config_.stale_after_seconds)
            : 0.0;
        cell.stale = config_.stale_after_seconds > 0.0 && age_seconds > config_.stale_after_seconds;

        const auto occupied = occupied_strength >= 1.0;
        const auto observed_free = free_strength >= 1.0;
        if (occupied && observed_free) {
            cell.state = TraversabilityCellState::Mixed;
        } else if (occupied) {
            cell.state = TraversabilityCellState::Occupied;
        } else if (observed_free) {
            cell.state = cell.stale ? TraversabilityCellState::Stale : TraversabilityCellState::ObservedFree;
        } else {
            cell.state = cell.stale ? TraversabilityCellState::Stale : TraversabilityCellState::Unknown;
        }

        cell.volatility_score = clamp01(std::min(occupied_strength, free_strength) + (0.25 * cell.unknown_score));
        cell.stability_score = clamp01(std::max(occupied_strength, free_strength) - cell.volatility_score);

        cell.nearest_obstacle_distance_m = std::numeric_limits<double>::infinity();
        cell.vertical_clearance_up_m = std::numeric_limits<double>::infinity();
        cell.vertical_clearance_down_m = std::numeric_limits<double>::infinity();

        for (const auto* occupied_cell : occupied_cells) {
            const auto d = distance(cell.center_map, occupied_cell->cell.center_map);
            if (d <= config_.clearance_search_radius_m) {
                cell.nearest_obstacle_distance_m = std::min(cell.nearest_obstacle_distance_m, d);
            }
            if (occupied_cell->key.x == stored.key.x && occupied_cell->key.y == stored.key.y) {
                const auto dz = occupied_cell->cell.center_map.z - cell.center_map.z;
                if (dz > 0.0) {
                    cell.vertical_clearance_up_m = std::min(cell.vertical_clearance_up_m, dz);
                } else if (dz < 0.0) {
                    cell.vertical_clearance_down_m = std::min(cell.vertical_clearance_down_m, -dz);
                } else if (occupied) {
                    cell.vertical_clearance_up_m = 0.0;
                    cell.vertical_clearance_down_m = 0.0;
                }
            }
        }

        if (occupied) {
            cell.nearest_obstacle_distance_m = 0.0;
            cell.clearance_margin_m = -config_.required_clearance_m;
        } else if (std::isfinite(cell.nearest_obstacle_distance_m)) {
            cell.clearance_margin_m =
                cell.nearest_obstacle_distance_m - config_.required_clearance_m;
        } else {
            cell.clearance_margin_m = std::numeric_limits<double>::infinity();
        }

        cell.occupied_cost = occupied ? 1.0 : 0.0;

        if (occupied) {
            cell.proximity_cost = 1.0;
        } else if (std::isfinite(cell.nearest_obstacle_distance_m) &&
                   cell.nearest_obstacle_distance_m < config_.soft_clearance_m) {
            cell.proximity_cost = clamp01(
                (config_.soft_clearance_m - cell.nearest_obstacle_distance_m) /
                config_.soft_clearance_m);
        } else {
            cell.proximity_cost = 0.0;
        }

        cell.unknown_cost = cell.unknown_score;
        cell.stale_cost = cell.stale ? cell.age_score : 0.0;

        if (std::isfinite(cell.vertical_clearance_up_m) &&
            cell.vertical_clearance_up_m < (2.0 * config_.required_clearance_m)) {
            cell.overhead_cost = clamp01(
                ((2.0 * config_.required_clearance_m) - cell.vertical_clearance_up_m) /
                (2.0 * config_.required_clearance_m));
        } else {
            cell.overhead_cost = 0.0;
        }

        cell.thin_structure_cost = cell.volatility_score;

        const auto weighted =
            (config_.occupied_weight * cell.occupied_cost) +
            (config_.proximity_weight * cell.proximity_cost) +
            (config_.unknown_weight * cell.unknown_cost) +
            (config_.stale_weight * cell.stale_cost) +
            (config_.overhead_weight * cell.overhead_cost) +
            (config_.thin_structure_weight * cell.thin_structure_cost);
        const auto total_weight =
            config_.occupied_weight +
            config_.proximity_weight +
            config_.unknown_weight +
            config_.stale_weight +
            config_.overhead_weight +
            config_.thin_structure_weight;
        cell.total_traversability_cost =
            total_weight > 0.0 ? clamp01(weighted / total_weight) : clamp01(weighted);
    }
}

void MissionLocalTraversabilityMap::refresh_summary() {
    summary_.cell_count = cells_.size();
    summary_.occupied_cell_count = 0U;
    summary_.free_cell_count = 0U;
    summary_.mixed_cell_count = 0U;
    summary_.stale_cell_count = 0U;
    summary_.low_clearance_cell_count = 0U;
    summary_.overhead_risk_cell_count = 0U;
    summary_.volatile_cell_count = 0U;
    summary_.minimum_clearance_m = std::numeric_limits<double>::infinity();
    summary_.minimum_vertical_clearance_up_m = std::numeric_limits<double>::infinity();

    for (const auto& stored : cells_) {
        const auto& cell = stored.cell;
        if (cell.state == TraversabilityCellState::Occupied) {
            ++summary_.occupied_cell_count;
        } else if (cell.state == TraversabilityCellState::ObservedFree) {
            ++summary_.free_cell_count;
        } else if (cell.state == TraversabilityCellState::Mixed) {
            ++summary_.mixed_cell_count;
            ++summary_.occupied_cell_count;
            ++summary_.free_cell_count;
        }
        if (cell.stale || cell.state == TraversabilityCellState::Stale) {
            ++summary_.stale_cell_count;
        }
        if (std::isfinite(cell.clearance_margin_m) && cell.clearance_margin_m < 0.0) {
            ++summary_.low_clearance_cell_count;
        }
        if (cell.overhead_cost > 0.0) {
            ++summary_.overhead_risk_cell_count;
        }
        if (cell.volatility_score > 0.25) {
            ++summary_.volatile_cell_count;
        }
        if (std::isfinite(cell.nearest_obstacle_distance_m)) {
            summary_.minimum_clearance_m =
                std::min(summary_.minimum_clearance_m, cell.nearest_obstacle_distance_m);
        }
        if (std::isfinite(cell.vertical_clearance_up_m)) {
            summary_.minimum_vertical_clearance_up_m =
                std::min(summary_.minimum_vertical_clearance_up_m, cell.vertical_clearance_up_m);
        }
    }
}

MissionLocalTraversabilityMapSnapshot MissionLocalTraversabilityMap::snapshot(
    const std::size_t max_cells) const {
    MissionLocalTraversabilityMapSnapshot result;
    result.config = config_;
    result.summary = summary_;
    result.cells.reserve(cells_.size());

    for (const auto& stored : cells_) {
        result.cells.push_back(stored.cell);
    }

    std::sort(
        result.cells.begin(),
        result.cells.end(),
        [](const MissionLocalTraversabilityCell& lhs, const MissionLocalTraversabilityCell& rhs) {
            if (lhs.total_traversability_cost == rhs.total_traversability_cost) {
                if (lhs.center_map.x == rhs.center_map.x) {
                    if (lhs.center_map.y == rhs.center_map.y) {
                        return lhs.center_map.z < rhs.center_map.z;
                    }
                    return lhs.center_map.y < rhs.center_map.y;
                }
                return lhs.center_map.x < rhs.center_map.x;
            }
            return lhs.total_traversability_cost > rhs.total_traversability_cost;
        });

    if (max_cells > 0U && result.cells.size() > max_cells) {
        result.cells.resize(max_cells);
    }

    return result;
}

TraversabilityQueryResult MissionLocalTraversabilityMap::query_sphere(
    const Vec3& center_map,
    const double radius_m) const {
    TraversabilityQueryResult result;
    if (!finite_point(center_map) || !(radius_m >= 0.0)) {
        return result;
    }

    const auto key = key_for_point(center_map);
    if (const auto* cell = cell_at_key(key)) {
        result.known = cell->state != TraversabilityCellState::Unknown;
        result.occupied =
            cell->state == TraversabilityCellState::Occupied ||
            cell->state == TraversabilityCellState::Mixed ||
            (std::isfinite(cell->nearest_obstacle_distance_m) &&
             cell->nearest_obstacle_distance_m <= radius_m);
        result.observed_free =
            cell->state == TraversabilityCellState::ObservedFree ||
            cell->state == TraversabilityCellState::Mixed;
        result.stale = cell->stale;
        result.nearest_obstacle_distance_m = cell->nearest_obstacle_distance_m;
        result.clearance_margin_m =
            std::isfinite(cell->nearest_obstacle_distance_m)
                ? cell->nearest_obstacle_distance_m - radius_m
                : std::numeric_limits<double>::infinity();
        result.occupied_fraction = result.occupied ? 1.0 : 0.0;
        result.unknown_fraction = cell->unknown_score;
        result.total_cost = cell->total_traversability_cost;
        return result;
    }

    return result;
}

void MissionLocalTraversabilityMap::reset() {
    cells_.clear();
    cell_index_.clear();
    summary_ = MissionLocalTraversabilityMapSummary{};
    has_last_update_ = false;
    last_update_ = TimePoint{};
}

}  // namespace dedalus
