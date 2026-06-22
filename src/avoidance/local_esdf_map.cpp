// local_esdf_map.cpp
//
// LocalESDFMap public interface: query(), repulsion(), is_clear(), save(), load().
// The EDT computation lives in compute_esdf.cpp.

#include "dedalus/avoidance/local_esdf_map.hpp"

#include <cmath>
#include <cstdio>

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

// ─── save ────────────────────────────────────────────────────────────────────
//
// Binary layout (little-endian native):
//   [0..3]   magic   'E','S','D','F'
//   [4..7]   version uint32 = 1
//   [8..15]  n       uint64  (cell count)
//   [16..23] cell_size_m          double
//   [24..31] vertical_cell_size_m double
//   [32..39] d0_m                 double
//   Repeated n times:
//     centre.x  double (8)
//     centre.y  double (8)
//     centre.z  double (8)
//     d         float  (4)
//     grad.x    double (8)
//     grad.y    double (8)
//     grad.z    double (8)   → 52 bytes per cell

bool LocalESDFMap::save(const std::filesystem::path& path) const {
    std::FILE* fp = std::fopen(path.c_str(), "wb");
    if (!fp) { return false; }

    const char     magic[4] = {'E', 'S', 'D', 'F'};
    std::uint32_t  version  = 1U;
    std::uint64_t  n        = static_cast<std::uint64_t>(cells_.size());

    bool ok = true;
    ok = ok && std::fwrite(magic,              1, 4, fp) == 4;
    ok = ok && std::fwrite(&version,           4, 1, fp) == 1;
    ok = ok && std::fwrite(&n,                 8, 1, fp) == 1;
    ok = ok && std::fwrite(&config_.cell_size_m,          8, 1, fp) == 1;
    ok = ok && std::fwrite(&config_.vertical_cell_size_m, 8, 1, fp) == 1;
    ok = ok && std::fwrite(&config_.d0_m,                 8, 1, fp) == 1;

    for (const auto& [key, cell] : cells_) {
        ok = ok && std::fwrite(&cell.centre.x, 8, 1, fp) == 1;
        ok = ok && std::fwrite(&cell.centre.y, 8, 1, fp) == 1;
        ok = ok && std::fwrite(&cell.centre.z, 8, 1, fp) == 1;
        ok = ok && std::fwrite(&cell.d,         4, 1, fp) == 1;
        ok = ok && std::fwrite(&cell.grad.x,   8, 1, fp) == 1;
        ok = ok && std::fwrite(&cell.grad.y,   8, 1, fp) == 1;
        ok = ok && std::fwrite(&cell.grad.z,   8, 1, fp) == 1;
        if (!ok) { break; }
    }

    std::fclose(fp);
    return ok;
}

// ─── load ────────────────────────────────────────────────────────────────────

bool LocalESDFMap::load(const std::filesystem::path& path) {
    std::FILE* fp = std::fopen(path.c_str(), "rb");
    if (!fp) { return false; }

    char          magic[4] = {};
    std::uint32_t version  = 0U;
    std::uint64_t n        = 0U;
    double        cell_size_m = 0.0, vertical_cell_size_m = 0.0, d0_m = 0.0;

    bool ok = true;
    ok = ok && std::fread(magic,                 1, 4, fp) == 4
            && magic[0]=='E' && magic[1]=='S' && magic[2]=='D' && magic[3]=='F';
    ok = ok && std::fread(&version,              4, 1, fp) == 1 && version == 1U;
    ok = ok && std::fread(&n,                    8, 1, fp) == 1;
    ok = ok && std::fread(&cell_size_m,          8, 1, fp) == 1;
    ok = ok && std::fread(&vertical_cell_size_m, 8, 1, fp) == 1;
    ok = ok && std::fread(&d0_m,                 8, 1, fp) == 1;

    if (!ok) { std::fclose(fp); return false; }

    config_.cell_size_m          = cell_size_m;
    config_.vertical_cell_size_m = vertical_cell_size_m;
    config_.d0_m                 = d0_m;

    cells_.clear();
    cells_.reserve(static_cast<std::size_t>(n));

    for (std::uint64_t i = 0U; i < n; ++i) {
        LocalESDFCell cell{};
        ok = ok && std::fread(&cell.centre.x, 8, 1, fp) == 1;
        ok = ok && std::fread(&cell.centre.y, 8, 1, fp) == 1;
        ok = ok && std::fread(&cell.centre.z, 8, 1, fp) == 1;
        ok = ok && std::fread(&cell.d,         4, 1, fp) == 1;
        ok = ok && std::fread(&cell.grad.x,   8, 1, fp) == 1;
        ok = ok && std::fread(&cell.grad.y,   8, 1, fp) == 1;
        ok = ok && std::fread(&cell.grad.z,   8, 1, fp) == 1;
        if (!ok) { break; }
        cells_.emplace(key_for_point(cell.centre), cell);
    }

    std::fclose(fp);
    if (!ok) { cells_.clear(); }
    return ok;
}

}  // namespace dedalus
