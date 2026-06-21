// local_esdf_map.cpp
//
// LocalESDFMap public interface: query(), repulsion(), is_clear().
// The EDT computation lives in compute_esdf.cpp.

#include "dedalus/avoidance/local_esdf_map.hpp"

#include <cmath>

namespace dedalus {

// ─── CellKeyHash ─────────────────────────────────────────────────────────────

std::size_t LocalESDFMap::CellKeyHash::operator()(const CellKey& k) const noexcept {
    std::size_t h = 14695981039346656037ULL;
    auto mix = [&](int v) {
        h ^= static_cast<std::size_t>(static_cast<std::uint32_t>(v));
        h *= 1099511628211ULL;
    };
    mix(k.x);
    mix(k.y);
    mix(k.z);
    return h;
}

// ─── key_for_point ───────────────────────────────────────────────────────────

LocalESDFMap::CellKey LocalESDFMap::key_for_point(const Vec3& pos) const noexcept {
    return CellKey{
        static_cast<int>(std::floor(pos.x / config_.cell_size_m)),
        static_cast<int>(std::floor(pos.y / config_.cell_size_m)),
        static_cast<int>(std::floor(pos.z / config_.vertical_cell_size_m)),
    };
}

// ─── query ───────────────────────────────────────────────────────────────────

LocalESDFQueryResult LocalESDFMap::query(const Vec3& pos) const noexcept {
    const auto it = cells_.find(key_for_point(pos));
    if (it == cells_.end()) {
        return {static_cast<float>(config_.d0_m), Vec3{0.0, 0.0, 0.0}};
    }
    return {it->second.d, it->second.grad};
}

// ─── repulsion ───────────────────────────────────────────────────────────────

Vec3 LocalESDFMap::repulsion(const Vec3& pos, double d0, double k) const noexcept {
    const auto r = query(pos);
    // Only apply when inside the truncation radius and in free space.
    if (r.d <= 0.0f || static_cast<double>(r.d) >= d0) {
        return Vec3{0.0, 0.0, 0.0};
    }
    const double d     = static_cast<double>(r.d);
    const double scale = k * (1.0 / d - 1.0 / d0) / (d * d);
    return Vec3{scale * r.grad.x, scale * r.grad.y, scale * r.grad.z};
}

// ─── is_clear ────────────────────────────────────────────────────────────────

bool LocalESDFMap::is_clear(const Vec3& pos, double r) const noexcept {
    return static_cast<double>(query(pos).d) >= r;
}

// ─── snapshot ────────────────────────────────────────────────────────────────

LocalESDFMapSnapshot LocalESDFMap::snapshot(const Vec3& query_pos, double repulsion_k) const {
    LocalESDFMapSnapshot snap;
    snap.config = config_;
    snap.cell_count = cells_.size();
    snap.cells.reserve(cells_.size());
    for (const auto& [key, cell] : cells_) {
        snap.cells.push_back(cell);
    }
    snap.net_repulsion = repulsion(query_pos, config_.d0_m, repulsion_k);
    return snap;
}

}  // namespace dedalus
