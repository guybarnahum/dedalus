// local_esdf_map.cpp
//
// LocalESDFMap public interface: query(), repulsion(), is_clear(), save(), load().
// The EDT computation lives in compute_esdf.cpp.

#include "dedalus/avoidance/local_esdf_map.hpp"

#include <algorithm>
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
    // Trilinear interpolation over the 8 surrounding coarse cells.
    // APF force at each cell = k * (1/d - 1/d0) / d * sgrad.
    // Cells missing from the map contribute zero weight (boundary handling).
    const double csx = config_.cell_size_m;
    const double csz = config_.vertical_cell_size_m;

    // Fractional position within coarse voxel
    const double fx = pos.x / csx;
    const double fy = pos.y / csx;
    const double fz = pos.z / csz;

    const int x0 = static_cast<int>(std::floor(fx));
    const int y0 = static_cast<int>(std::floor(fy));
    const int z0 = static_cast<int>(std::floor(fz));

    const double tx = fx - x0;
    const double ty = fy - y0;
    const double tz = fz - z0;

    Vec3   result{0.0, 0.0, 0.0};
    double w_found = 0.0;

    for (int di = 0; di <= 1; ++di) {
        for (int dj = 0; dj <= 1; ++dj) {
            for (int dk = 0; dk <= 1; ++dk) {
                const auto it = cells_.find(CellKey{x0+di, y0+dj, z0+dk});
                if (it == cells_.end()) continue;
                const auto& cell = it->second;
                if (cell.d <= 0.0f) continue;
                const double d = static_cast<double>(cell.d);
                if (d >= d0) continue;

                const double wx = (di == 0) ? (1.0 - tx) : tx;
                const double wy = (dj == 0) ? (1.0 - ty) : ty;
                const double wz = (dk == 0) ? (1.0 - tz) : tz;
                const double w  = wx * wy * wz;

                const double scale = k * (1.0/d - 1.0/d0) / (d * d) * w;
                result.x += scale * cell.sgrad.x;
                result.y += scale * cell.sgrad.y;
                result.z += scale * cell.sgrad.z;
                w_found  += w;
            }
        }
    }
    // If boundary cells are missing, scale by found fraction so magnitude is consistent.
    if (w_found > 1.0e-10 && w_found < 0.999) {
        const double inv_w = 1.0 / w_found;
        result.x *= inv_w;
        result.y *= inv_w;
        result.z *= inv_w;
    }
    return result;
}

// ─── repulsion_smoothed ──────────────────────────────────────────────────────

Vec3 LocalESDFMap::repulsion_smoothed(const Vec3& pos,
                                       const Vec3& /* vel */,
                                       double d0,
                                       double k,
                                       double /* a_max_mps2 */) const noexcept {
    // L3 cells are pre-averaged (Gaussian-weighted APF computed in compute_esdf).
    // The stored sgrad already encodes spatial smoothing at sample_spacing_m scale.
    // Trilinear interpolation in repulsion() provides sub-cell continuity.
    // Velocity-dependent radius is implicit: at high speed, the planner calls this
    // over a trajectory horizon, not at a single point.
    return repulsion(pos, d0, k);
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
//   [4..7]   version uint32 = 2
//   [8..15]  n       uint64  (cell count)
//   [16..23] cell_size_m          double
//   [24..31] vertical_cell_size_m double
//   [32..39] d0_m                 double
//   Repeated n times (76 bytes/cell):
//     centre.x  double (8)
//     centre.y  double (8)
//     centre.z  double (8)
//     d         float  (4)
//     grad.x    double (8)
//     grad.y    double (8)
//     grad.z    double (8)
//     sgrad.x   double (8)  ← added in v2
//     sgrad.y   double (8)
//     sgrad.z   double (8)
//
// Version 1 (52 bytes/cell, no sgrad) can be loaded; sgrad defaults to grad.

bool LocalESDFMap::save(const std::filesystem::path& path) const {
    std::FILE* fp = std::fopen(path.c_str(), "wb");
    if (!fp) { return false; }

    const char     magic[4] = {'E', 'S', 'D', 'F'};
    std::uint32_t  version  = 2U;
    std::uint64_t  n        = static_cast<std::uint64_t>(cells_.size());

    bool ok = true;
    ok = ok && std::fwrite(magic,              1, 4, fp) == 4;
    ok = ok && std::fwrite(&version,           4, 1, fp) == 1;
    ok = ok && std::fwrite(&n,                 8, 1, fp) == 1;
    ok = ok && std::fwrite(&config_.cell_size_m,          8, 1, fp) == 1;
    ok = ok && std::fwrite(&config_.vertical_cell_size_m, 8, 1, fp) == 1;
    ok = ok && std::fwrite(&config_.d0_m,                 8, 1, fp) == 1;

    for (const auto& [key, cell] : cells_) {
        ok = ok && std::fwrite(&cell.centre.x,  8, 1, fp) == 1;
        ok = ok && std::fwrite(&cell.centre.y,  8, 1, fp) == 1;
        ok = ok && std::fwrite(&cell.centre.z,  8, 1, fp) == 1;
        ok = ok && std::fwrite(&cell.d,          4, 1, fp) == 1;
        ok = ok && std::fwrite(&cell.grad.x,    8, 1, fp) == 1;
        ok = ok && std::fwrite(&cell.grad.y,    8, 1, fp) == 1;
        ok = ok && std::fwrite(&cell.grad.z,    8, 1, fp) == 1;
        ok = ok && std::fwrite(&cell.sgrad.x,   8, 1, fp) == 1;
        ok = ok && std::fwrite(&cell.sgrad.y,   8, 1, fp) == 1;
        ok = ok && std::fwrite(&cell.sgrad.z,   8, 1, fp) == 1;
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
    ok = ok && std::fread(&version,              4, 1, fp) == 1
            && (version == 1U || version == 2U);
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
        ok = ok && std::fread(&cell.centre.x,  8, 1, fp) == 1;
        ok = ok && std::fread(&cell.centre.y,  8, 1, fp) == 1;
        ok = ok && std::fread(&cell.centre.z,  8, 1, fp) == 1;
        ok = ok && std::fread(&cell.d,          4, 1, fp) == 1;
        ok = ok && std::fread(&cell.grad.x,    8, 1, fp) == 1;
        ok = ok && std::fread(&cell.grad.y,    8, 1, fp) == 1;
        ok = ok && std::fread(&cell.grad.z,    8, 1, fp) == 1;
        if (version >= 2U) {
            ok = ok && std::fread(&cell.sgrad.x, 8, 1, fp) == 1;
            ok = ok && std::fread(&cell.sgrad.y, 8, 1, fp) == 1;
            ok = ok && std::fread(&cell.sgrad.z, 8, 1, fp) == 1;
        } else {
            // v1 file: no sgrad stored; use grad as a reasonable fallback.
            // Will be recomputed to a proper smoothed gradient on next full ESDF build.
            cell.sgrad = cell.grad;
        }
        if (!ok) { break; }
        cells_.emplace(key_for_point(cell.centre), cell);
    }

    std::fclose(fp);
    if (!ok) { cells_.clear(); }
    return ok;
}

}  // namespace dedalus
