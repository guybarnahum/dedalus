#include "dedalus/avoidance/mission_local_planning_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>

namespace dedalus {

namespace {

inline float sigmoid(const float x) noexcept {
    return 1.0F / (1.0F + std::exp(-x));
}

}  // namespace

MissionLocalPlanningMap::MissionLocalPlanningMap(MissionLocalPlanningMapConfig config)
    : config_(config) {
    if (!(config_.cell_size_m > 0.0)) {
        config_.cell_size_m = 1.0;
    }
    if (!(config_.vertical_cell_size_m > 0.0)) {
        config_.vertical_cell_size_m = 2.0;
    }
    if (!(config_.min_occupied_score > 0.0)) {
        config_.min_occupied_score = 0.5;
    }
    if (!(config_.log_odds_max > 0.0)) {
        config_.log_odds_max = 8.0;
    }
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

    ++map_seq_;  // Stage 5: bump version before any writes this tick
    last_update_stats_ = MissionLocalPlanningMapUpdateStats{};
    last_update_stats_.l1_input_cells = source.cells.size();

    const auto now_ns = static_cast<std::int64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

    bool any_evicted = false;

    // Collect dirty cells (key + value at mark-time) and evicted keys locally
    // — single lock acquisition at the end.
    std::vector<std::pair<CellKey, MissionLocalPlanningCell>> local_dirty;
    std::vector<CellKey> local_evicted;

    for (const auto& l1_cell : source.cells) {
        const bool is_occupied =
            l1_cell.state == TraversabilityCellState::Occupied ||
            l1_cell.state == TraversabilityCellState::Mixed;
        const bool is_free =
            l1_cell.state == TraversabilityCellState::ObservedFree;

        if (!is_occupied && !is_free) {
            continue;  // Unknown / Stale → no change to L2
        }

        const auto key = key_for_point(l1_cell.center_map);

        if (is_occupied && l1_cell.occupied_score >= config_.min_occupied_score) {
            ++last_update_stats_.l1_occupied_merged;

            const float increment = static_cast<float>(
                l1_cell.confidence * config_.log_odds_occupied_increment);
            const float max_lo = static_cast<float>(config_.log_odds_max);

            const auto it = cell_index_.find(key);
            if (it != cell_index_.end()) {
                // Reinforce existing L2 cell via additive log-odds accumulation.
                auto& sc = cells_[it->second];
                sc.cell.log_odds = std::min(sc.cell.log_odds + increment, max_lo);
                sc.cell.occupied_score = sigmoid(sc.cell.log_odds);
                sc.cell.confidence = std::max(
                    sc.cell.confidence,
                    static_cast<float>(l1_cell.confidence));
                ++sc.cell.source_cell_count;
                sc.cell.last_updated_ns = now_ns;
                sc.write_seq = map_seq_;  // Stage 5
            } else {
                // Create new L2 cell.
                MissionLocalPlanningCell cell;
                cell.center_map = center_for_key(key);
                cell.log_odds = std::min(increment, max_lo);
                cell.occupied_score = sigmoid(cell.log_odds);
                cell.confidence = static_cast<float>(l1_cell.confidence);
                cell.source_cell_count = 1U;
                cell.last_updated_ns = now_ns;
                cells_.push_back(StoredCell{key, cell, map_seq_});  // Stage 5
                cell_index_.emplace(key, cells_.size() - 1U);
            }
            // Capture the updated cell value at mark-time so the flush thread
            // never needs to read cells_ directly.
            local_dirty.emplace_back(key, cells_[cell_index_.at(key)].cell);
        } else if (is_free) {
            ++last_update_stats_.l1_free_applied;

            const auto it = cell_index_.find(key);
            if (it != cell_index_.end()) {
                // Reduce the L2 voxel's log_odds by a confidence-weighted decrement.
                auto& sc = cells_[it->second];
                const float decrement = static_cast<float>(
                    l1_cell.confidence * config_.log_odds_free_decrement);
                const float min_lo = -static_cast<float>(config_.log_odds_max);
                sc.cell.log_odds = std::max(sc.cell.log_odds - decrement, min_lo);
                sc.cell.occupied_score = sigmoid(sc.cell.log_odds);
                if (sc.cell.log_odds < static_cast<float>(config_.log_odds_eviction_threshold)) {
                    // Mark for eviction: occupied_score sentinel 0 — evict_cleared_cells() sweeps.
                    sc.cell.occupied_score = 0.0F;
                    ++last_update_stats_.cells_evicted;
                    any_evicted = true;
                    local_evicted.push_back(key);
                } else {
                    sc.write_seq = map_seq_;  // Stage 5
                    local_dirty.emplace_back(key, sc.cell);
                }
            }
        }
    }

    if (any_evicted) {
        evict_cleared_cells();
    }

    // Merge into shared dirty/evicted maps under a single lock.
    if (!local_dirty.empty() || !local_evicted.empty()) {
        std::lock_guard<std::mutex> lk(db_mutex_);
        for (const auto& [k, c] : local_dirty) {
            dirty_cells_[k] = c;  // last write wins if key appears multiple times
        }
        for (const auto& k : local_evicted) {
            dirty_cells_.erase(k);  // evicted cell is no longer "dirty"
            evicted_keys_.insert(k);
        }
    }
}

void MissionLocalPlanningMap::evict_cleared_cells() {
    // Compact: keep cells whose score is still at or above the floor.
    const auto keep_end = std::stable_partition(
        cells_.begin(), cells_.end(),
        [](const StoredCell& sc) { return sc.cell.occupied_score > 0.0F; });

    if (keep_end == cells_.end()) {
        return;
    }

    cells_.erase(keep_end, cells_.end());

    // Rebuild index.
    cell_index_.clear();
    cell_index_.reserve(cells_.size());
    for (std::size_t i = 0U; i < cells_.size(); ++i) {
        cell_index_.emplace(cells_[i].key, i);
    }
}

MissionLocalPlanningMapSnapshot MissionLocalPlanningMap::snapshot(std::uint64_t since_seq) const {
    MissionLocalPlanningMapSnapshot snap;
    snap.config = config_;
    snap.cell_count = cells_.size();
    snap.last_update_stats = last_update_stats_;
    snap.seq = map_seq_;
    snap.is_delta = (since_seq > 0U);
    snap.cells.reserve(since_seq == 0U ? cells_.size() : 0U);
    for (const auto& sc : cells_) {
        if (sc.write_seq > since_seq) {
            snap.cells.push_back(sc.cell);
        }
    }
    return snap;
}

void MissionLocalPlanningMap::reset() {
    cells_.clear();
    cell_index_.clear();
    last_update_stats_ = MissionLocalPlanningMapUpdateStats{};
    map_seq_ = 0U;
}

// ─── Persistence ─────────────────────────────────────────────────────────────
//
// Format (versioned text, one cell per line after the header):
//
//   planning_map_v2 cell_size=<m> vcell_size=<m> min_score=<f> eviction=<f>
//   <cx> <cy> <cz> <log_odds> <conf> <count>
//   ...

bool MissionLocalPlanningMap::save_to_file(const std::filesystem::path& path) const {
    std::ofstream f(path);
    if (!f) {
        return false;
    }
    f << "planning_map_v2"
      << " cell_size=" << config_.cell_size_m
      << " vcell_size=" << config_.vertical_cell_size_m
      << " min_score=" << config_.min_occupied_score
      << " eviction=" << config_.log_odds_eviction_threshold
      << "\n";
    f.precision(9);
    for (const auto& sc : cells_) {
        const auto& c = sc.cell;
        f << c.center_map.x << " "
          << c.center_map.y << " "
          << c.center_map.z << " "
          << c.log_odds << " "
          << c.confidence << " "
          << c.source_cell_count << "\n";
    }
    return f.good();
}

bool MissionLocalPlanningMap::load_from_file(const std::filesystem::path& path) {
    std::ifstream f(path);
    if (!f) {
        return false;
    }

    std::string header;
    if (!std::getline(f, header)) {
        return false;
    }
    if (header.rfind("planning_map_v2", 0) != 0) {
        return false;  // unrecognised or legacy version
    }

    reset();

    double cx = 0.0, cy = 0.0, cz = 0.0;
    float lo = 0.0F, conf = 0.0F;
    std::uint32_t count = 0U;
    const float eviction_floor = static_cast<float>(config_.log_odds_eviction_threshold);

    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream ss(line);
        if (!(ss >> cx >> cy >> cz >> lo >> conf >> count)) {
            continue;
        }
        if (lo < eviction_floor) {
            continue;  // skip cells that would be evicted under current config
        }

        const Vec3 center{cx, cy, cz};
        const auto key = key_for_point(center);
        if (cell_index_.find(key) != cell_index_.end()) {
            continue;  // duplicate (shouldn't happen in a well-formed file)
        }
        MissionLocalPlanningCell cell;
        cell.center_map = center_for_key(key);  // re-snap to grid
        cell.log_odds = lo;
        cell.occupied_score = sigmoid(lo);
        cell.confidence = conf;
        cell.source_cell_count = count;
        cells_.push_back(StoredCell{key, cell});
        cell_index_.emplace(key, cells_.size() - 1U);
    }

    return true;
}

// ─── extent ──────────────────────────────────────────────────────────────────

std::optional<MissionLocalPlanningMapExtent>
MissionLocalPlanningMap::extent() const noexcept {
    if (cells_.empty()) return std::nullopt;
    Vec3 mn = cells_[0].cell.center_map;
    Vec3 mx = cells_[0].cell.center_map;
    for (const auto& sc : cells_) {
        const Vec3& c = sc.cell.center_map;
        if (c.x < mn.x) { mn.x = c.x; } else if (c.x > mx.x) { mx.x = c.x; }
        if (c.y < mn.y) { mn.y = c.y; } else if (c.y > mx.y) { mx.y = c.y; }
        if (c.z < mn.z) { mn.z = c.z; } else if (c.z > mx.z) { mx.z = c.z; }
    }
    return MissionLocalPlanningMapExtent{mn, mx};
}

}  // namespace dedalus
