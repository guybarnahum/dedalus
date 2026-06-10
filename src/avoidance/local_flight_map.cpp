#include "dedalus/avoidance/local_flight_map.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dedalus {
namespace {

constexpr double kNanosecondsPerSecond = 1'000'000'000.0;

std::uint64_t timestamp_ns(const TimePoint& time) {
    return static_cast<std::uint64_t>(time.timestamp_ns);
}


bool is_finite_positive(const float value) {
    return std::isfinite(value) && value > 0.0F;
}

}  // namespace

LocalFlightMapAccumulator::LocalFlightMapAccumulator(LocalFlightMapConfig config)
    : config_{config} {
    if (config_.cell_size_m <= 0.0F) {
        config_.cell_size_m = 0.5F;
    }
    if (config_.forward_range_m <= 0.0F) {
        config_.forward_range_m = 30.0F;
    }
    if (config_.rear_range_m < 0.0F) {
        config_.rear_range_m = 0.0F;
    }
    if (config_.lateral_range_m <= 0.0F) {
        config_.lateral_range_m = 15.0F;
    }

    latest_.cell_size_m = config_.cell_size_m;
    latest_.forward_range_m = config_.forward_range_m;
    latest_.rear_range_m = config_.rear_range_m;
    latest_.lateral_range_m = config_.lateral_range_m;

    latest_.x_cells = static_cast<int>(
        std::ceil((config_.forward_range_m + config_.rear_range_m) / config_.cell_size_m));
    latest_.y_cells = static_cast<int>(
        std::ceil((2.0F * config_.lateral_range_m) / config_.cell_size_m));

    latest_.x_cells = std::max(1, latest_.x_cells);
    latest_.y_cells = std::max(1, latest_.y_cells);

    reset_cells();
}

LocalFlightMapSnapshot LocalFlightMapAccumulator::update(const WorldSnapshot& snapshot) {
    latest_.timestamp = snapshot.timestamp;
    latest_.source_frame_id = FrameId{};
    latest_.has_source_frame = false;
    if (!snapshot.obstacle_evidence.empty() && snapshot.obstacle_evidence.front().has_source_frame) {
        latest_.source_frame_id = snapshot.obstacle_evidence.front().source_frame_id;
        latest_.has_source_frame = true;
    }

    decay_scores(snapshot.timestamp);
    ingest_obstacle_evidence(snapshot);
    classify_cells();
    inflate_blocked_cells();
    update_summary();

    last_update_ns_ = timestamp_ns(snapshot.timestamp);
    has_last_update_ = true;
    return latest_;
}

void LocalFlightMapAccumulator::reset_cells() {
    latest_.cells.clear();
    latest_.cells.resize(static_cast<std::size_t>(latest_.x_cells * latest_.y_cells));

    for (int iy = 0; iy < latest_.y_cells; ++iy) {
        for (int ix = 0; ix < latest_.x_cells; ++ix) {
            auto& cell = latest_.cells[static_cast<std::size_t>(flat_index(ix, iy))];
            cell = LocalFlightMapCell{};
            cell.center_local = cell_center_local(ix, iy);
        }
    }

    update_summary();
}

void LocalFlightMapAccumulator::decay_scores(const TimePoint timestamp) {
    if (!has_last_update_) {
        return;
    }

    const auto now_ns = timestamp_ns(timestamp);
    if (now_ns <= last_update_ns_) {
        return;
    }

    const double dt_s = static_cast<double>(now_ns - last_update_ns_) / kNanosecondsPerSecond;
    if (dt_s <= 0.0) {
        return;
    }

    float decay = 0.0F;
    if (config_.decay_half_life_s <= 0.0F) {
        decay = 0.0F;
    } else {
        decay = static_cast<float>(std::pow(0.5, dt_s / static_cast<double>(config_.decay_half_life_s)));
    }

    for (auto& cell : latest_.cells) {
        cell.occupied_score *= decay;
        cell.free_score *= decay;
        cell.risk_score *= decay;
        cell.recently_observed = false;
        if (cell.occupied_score < 0.001F) {
            cell.occupied_score = 0.0F;
        }
        if (cell.free_score < 0.001F) {
            cell.free_score = 0.0F;
        }
        if (cell.risk_score < 0.001F) {
            cell.risk_score = 0.0F;
        }
        if (cell.occupied_score == 0.0F && cell.risk_score == 0.0F) {
            cell.nearest_range_m = std::numeric_limits<float>::infinity();
        }
    }
}


std::optional<LocalFlightMapIndex> LocalFlightMapAccumulator::local_to_grid(const Vec3& local) const {
    if (local.x < -config_.rear_range_m || local.x >= config_.forward_range_m) {
        return std::nullopt;
    }
    if (local.y < -config_.lateral_range_m || local.y >= config_.lateral_range_m) {
        return std::nullopt;
    }

    const auto ix = static_cast<int>(std::floor((local.x + config_.rear_range_m) / config_.cell_size_m));
    const auto iy = static_cast<int>(std::floor((local.y + config_.lateral_range_m) / config_.cell_size_m));

    if (ix < 0 || ix >= latest_.x_cells || iy < 0 || iy >= latest_.y_cells) {
        return std::nullopt;
    }

    return LocalFlightMapIndex{ix, iy};
}

const LocalFlightMapCell* LocalFlightMapAccumulator::cell_at(const int ix, const int iy) const {
    if (ix < 0 || ix >= latest_.x_cells || iy < 0 || iy >= latest_.y_cells) {
        return nullptr;
    }
    return &latest_.cells[static_cast<std::size_t>(flat_index(ix, iy))];
}

LocalFlightMapCell* LocalFlightMapAccumulator::mutable_cell_at(const int ix, const int iy) {
    if (ix < 0 || ix >= latest_.x_cells || iy < 0 || iy >= latest_.y_cells) {
        return nullptr;
    }
    return &latest_.cells[static_cast<std::size_t>(flat_index(ix, iy))];
}

int LocalFlightMapAccumulator::flat_index(const int ix, const int iy) const noexcept {
    return (iy * latest_.x_cells) + ix;
}

Vec3 LocalFlightMapAccumulator::cell_center_local(const int ix, const int iy) const noexcept {
    return Vec3{
        -config_.rear_range_m + (static_cast<double>(ix) + 0.5) * config_.cell_size_m,
        -config_.lateral_range_m + (static_cast<double>(iy) + 0.5) * config_.cell_size_m,
        0.0,
    };
}

float LocalFlightMapAccumulator::evidence_footprint_radius_m(const ObstacleEvidence& evidence) const noexcept {
    float radius = std::max(config_.cell_size_m, evidence.radius_m);
    const auto footprint_xy_m = static_cast<float>(std::max(evidence.size_m.x, evidence.size_m.y));
    radius = std::max(radius, 0.5F * footprint_xy_m);
    if (evidence.shape == ObstacleEvidenceShape::SurfacePatch) {
        radius = std::max(radius, 0.5F * config_.cell_size_m);
    }
    return radius;
}

int LocalFlightMapAccumulator::footprint_radius_cells(const float radius_m) const noexcept {
    if (radius_m <= 0.0F) {
        return 0;
    }
    return std::max(0, static_cast<int>(std::ceil(radius_m / config_.cell_size_m)));
}

bool LocalFlightMapAccumulator::evidence_is_usable(const ObstacleEvidence& evidence) const noexcept {
    if (evidence.source_kind != OccupancySourceKind::DepthProvider) {
        return false;
    }
    if (evidence.range_m > config_.max_evidence_range_m && is_finite_positive(evidence.range_m)) {
        return false;
    }
    if (evidence.state == ObstacleEvidenceState::Unknown) {
        return false;
    }
    return local_to_grid(evidence.center_local).has_value();
}

}  // namespace dedalus
