#include "dedalus/avoidance/mission_local_planning_map.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

namespace dedalus {

MissionLocalPlanningMap::MissionLocalPlanningMap(MissionLocalPlanningMapConfig config)
    : config_(config) {
    if (!(config_.cell_size_m > 0.0)) {
        config_.cell_size_m = 1.0;
    }
    if (!(config_.vertical_cell_size_m > 0.0)) {
        config_.vertical_cell_size_m = 2.0;
    }
    if (!(config_.min_occupied_score > 0.0)) {
        config_.min_occupied_score = 1.0;
    }
    snapshot_.config = config_;
}

std::size_t MissionLocalPlanningMap::CellKeyHash::operator()(const CellKey& key) const noexcept {
    std::size_t seed = 0U;
    const auto mix = [&seed](const int value) {
        seed ^= std::hash<int>{}(value) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    };
    mix(key.x);
    mix(key.y);
    mix(key.z);
    return seed;
}

MissionLocalPlanningMap::CellKey MissionLocalPlanningMap::key_for_point(
    const Vec3& point) const noexcept {
    return CellKey{
        static_cast<int>(std::floor(point.x / config_.cell_size_m)),
        static_cast<int>(std::floor(point.y / config_.cell_size_m)),
        static_cast<int>(std::floor(point.z / config_.vertical_cell_size_m)),
    };
}

Vec3 MissionLocalPlanningMap::center_for_key(const CellKey& key) const noexcept {
    return Vec3{
        (static_cast<double>(key.x) + 0.5) * config_.cell_size_m,
        (static_cast<double>(key.y) + 0.5) * config_.cell_size_m,
        (static_cast<double>(key.z) + 0.5) * config_.vertical_cell_size_m,
    };
}

void MissionLocalPlanningMap::update_from_traversability(
    const MissionLocalTraversabilityMapSnapshot& source) {
    // Reset and rebuild from scratch each update.
    snapshot_ = MissionLocalPlanningMapSnapshot{};
    snapshot_.config = config_;
    snapshot_.l1_input_cell_count = source.cells.size();

    // Aggregate Level 1 occupied cells into the coarser planning grid.
    struct Accumulator {
        float max_occupied_score{0.0F};
        float max_confidence{0.0F};
        std::uint32_t count{0U};
    };
    // Optimistic pre-size: planning cells are larger so expect fewer of them.
    std::unordered_map<CellKey, Accumulator, CellKeyHash> grid;
    grid.reserve(std::max<std::size_t>(1U, source.cells.size() / 8U));

    for (const auto& l1_cell : source.cells) {
        if (l1_cell.occupied_score < config_.min_occupied_score) {
            continue;
        }
        ++snapshot_.l1_occupied_cell_count;

        const auto key = key_for_point(l1_cell.center_map);
        auto& acc = grid[key];
        acc.max_occupied_score = std::max(
            acc.max_occupied_score,
            static_cast<float>(l1_cell.occupied_score));
        acc.max_confidence = std::max(
            acc.max_confidence,
            static_cast<float>(l1_cell.confidence));
        ++acc.count;
    }

    // Materialise planning cells from the aggregation grid.
    snapshot_.cells.reserve(grid.size());
    for (const auto& [key, acc] : grid) {
        MissionLocalPlanningCell cell;
        cell.center_map = center_for_key(key);
        cell.max_occupied_score = acc.max_occupied_score;
        cell.confidence = acc.max_confidence;
        cell.source_cell_count = acc.count;
        snapshot_.cells.push_back(cell);
    }

    snapshot_.cell_count = snapshot_.cells.size();
}

}  // namespace dedalus
