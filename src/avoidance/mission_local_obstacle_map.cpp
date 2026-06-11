#include "dedalus/avoidance/mission_local_obstacle_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dedalus {
namespace {

std::uint64_t timestamp_ns(const TimePoint& time) {
    return static_cast<std::uint64_t>(time.timestamp_ns);
}

double elapsed_seconds(const TimePoint& from, const TimePoint& to) {
    const auto from_ns = timestamp_ns(from);
    const auto to_ns = timestamp_ns(to);
    if (to_ns <= from_ns) {
        return 0.0;
    }
    return static_cast<double>(to_ns - from_ns) * 1.0e-9;
}

double clamp_score(const double value, const double max_score) {
    return std::max(0.0, std::min(value, max_score));
}

bool finite_point(const Vec3& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

std::string map_frame_value(const MapFrameId& frame_id) {
    return frame_id.value;
}

bool map_frame_empty(const MapFrameId& frame_id) {
    return map_frame_value(frame_id).empty();
}

bool map_frame_equal(const MapFrameId& lhs, const MapFrameId& rhs) {
    return map_frame_value(lhs) == map_frame_value(rhs);
}

bool is_occupied_evidence(const ObstacleEvidence& evidence) {
    return evidence.state == ObstacleEvidenceState::Occupied;
}

double confidence_or_default(const ObstacleEvidence& evidence) {
    if (std::isfinite(evidence.confidence) && evidence.confidence > 0.0F) {
        return static_cast<double>(evidence.confidence);
    }
    return 1.0;
}

}  // namespace

MissionLocalObstacleMap::MissionLocalObstacleMap(MissionLocalObstacleMapConfig config)
    : config_(config) {
    if (!(config_.cell_size_m > 0.0)) {
        config_.cell_size_m = 0.5;
    }
    if (!(config_.vertical_cell_size_m > 0.0)) {
        config_.vertical_cell_size_m = config_.cell_size_m;
    }
}

const MissionLocalObstacleMapConfig& MissionLocalObstacleMap::config() const {
    return config_;
}

std::size_t MissionLocalObstacleMap::CellKeyHash::operator()(const CellKey& key) const {
    const auto x = static_cast<std::size_t>(key.x * 73856093);
    const auto y = static_cast<std::size_t>(key.y * 19349663);
    const auto z = static_cast<std::size_t>(key.z * 83492791);
    return x ^ y ^ z;
}

MissionLocalObstacleMap::CellKey MissionLocalObstacleMap::key_for_point(const Vec3& point) const {
    return CellKey{
        static_cast<int>(std::floor(point.x / config_.cell_size_m)),
        static_cast<int>(std::floor(point.y / config_.cell_size_m)),
        static_cast<int>(std::floor(point.z / config_.vertical_cell_size_m)),
    };
}

Vec3 MissionLocalObstacleMap::center_for_key(const CellKey& key) const {
    return Vec3{
        (static_cast<double>(key.x) + 0.5) * config_.cell_size_m,
        (static_cast<double>(key.y) + 0.5) * config_.cell_size_m,
        (static_cast<double>(key.z) + 0.5) * config_.vertical_cell_size_m,
    };
}

void MissionLocalObstacleMap::decay_to(const TimePoint& now) {
    if (!has_last_update_ || config_.score_decay_per_second <= 0.0) {
        return;
    }

    const auto decay = config_.score_decay_per_second * elapsed_seconds(last_update_, now);
    if (decay <= 0.0) {
        return;
    }

    for (auto& stored : cells_) {
        auto& cell = stored.cell;
        cell.occupied_score = clamp_score(cell.occupied_score - decay, config_.max_score);
        cell.free_score = clamp_score(cell.free_score - decay, config_.max_score);
        cell.risk_score = clamp_score(cell.risk_score - decay, config_.max_score);
        cell.occupied = cell.occupied_score >= config_.occupied_threshold;
        cell.free = cell.free_score >= config_.free_threshold;
    }
}

void MissionLocalObstacleMap::refresh_summary() {
    summary_.observed_cell_count = 0U;
    summary_.occupied_cell_count = 0U;
    summary_.free_cell_count = 0U;

    for (const auto& stored : cells_) {
        const auto& cell = stored.cell;
        if (cell.observed) {
            ++summary_.observed_cell_count;
        }
        if (cell.occupied) {
            ++summary_.occupied_cell_count;
        }
        if (cell.free) {
            ++summary_.free_cell_count;
        }
    }
}

MissionLocalObstacleMapSnapshot MissionLocalObstacleMap::snapshot(const std::size_t max_cells) const {
    MissionLocalObstacleMapSnapshot result;
    result.config = config_;
    result.summary = summary_;

    result.cells.reserve(cells_.size());
    for (const auto& stored : cells_) {
        result.cells.push_back(stored.cell);
    }

    if (max_cells > 0U && result.cells.size() > max_cells) {
        result.cells.resize(max_cells);
    }

    return result;
}

MissionLocalObstacleMapSnapshot MissionLocalObstacleMap::update(
    const std::vector<ObstacleEvidence>& evidence,
    const TimePoint now,
    const MapFrameId& fallback_map_frame_id) {
    decay_to(now);

    if (map_frame_empty(summary_.map_frame_id) && !map_frame_empty(fallback_map_frame_id)) {
        summary_.map_frame_id = fallback_map_frame_id;
    }

    const auto now_ns = timestamp_ns(now);
    const auto limit = std::min(evidence.size(), config_.max_evidence_per_update);

    for (std::size_t i = 0U; i < limit; ++i) {
        const auto& item = evidence[i];
        if (!finite_point(item.center_local)) {
            continue;
        }

        if (map_frame_empty(summary_.map_frame_id) && !map_frame_empty(item.map_frame_id)) {
            summary_.map_frame_id = item.map_frame_id;
        }

        if (!map_frame_empty(item.map_frame_id) &&
            !map_frame_empty(summary_.map_frame_id) &&
            !map_frame_equal(item.map_frame_id, summary_.map_frame_id)) {
            continue;
        }

        const auto key = key_for_point(item.center_local);

        auto found = std::find_if(cells_.begin(), cells_.end(), [&key](const StoredCell& stored) {
            return stored.key == key;
        });

        if (found == cells_.end()) {
            MissionLocalObstacleCell cell;
            cell.center_map = center_for_key(key);
            cell.size_m = Vec3{config_.cell_size_m, config_.cell_size_m, config_.vertical_cell_size_m};
            cell.first_observed_timestamp_ns = now_ns;
            cell.min_z_m = static_cast<float>(item.center_local.z);
            cell.max_z_m = static_cast<float>(item.center_local.z);
            cells_.push_back(StoredCell{key, cell});
            found = std::prev(cells_.end());
        }

        auto& cell = found->cell;
        const auto confidence = confidence_or_default(item);

        cell.observed = true;
        cell.last_observed_timestamp_ns = now_ns;
        cell.confidence = std::max(cell.confidence, confidence);
        cell.min_z_m = std::min(cell.min_z_m, static_cast<float>(item.center_local.z));
        cell.max_z_m = std::max(cell.max_z_m, static_cast<float>(item.center_local.z));
        cell.last_source_kind = item.source_kind;
        cell.last_source_provider = item.source_provider;

        if (is_occupied_evidence(item)) {
            cell.occupied_score = clamp_score(
                cell.occupied_score + (config_.occupied_hit_score * confidence),
                config_.max_score);
        } else {
            cell.free_score = clamp_score(
                cell.free_score + (config_.free_hit_score * confidence),
                config_.max_score);
        }

        cell.occupied = cell.occupied_score >= config_.occupied_threshold;
        cell.free = cell.free_score >= config_.free_threshold;
    }

    summary_.update_count += 1U;
    summary_.last_update_timestamp_ns = now_ns;
    last_update_ = now;
    has_last_update_ = true;

    refresh_summary();
    return snapshot();
}

void MissionLocalObstacleMap::reset() {
    cells_.clear();
    summary_ = MissionLocalObstacleMapSummary{};
    has_last_update_ = false;
    last_update_ = TimePoint{};
}

}  // namespace dedalus
