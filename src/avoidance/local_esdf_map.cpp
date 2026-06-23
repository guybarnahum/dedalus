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
    const auto it = cells_.find(key_for_point(pos));
    if (it == cells_.end()) return Vec3{0.0, 0.0, 0.0};
    const auto& cell = it->second;
    // Only apply when inside the truncation radius and in free space.
    if (cell.d <= 0.0f || static_cast<double>(cell.d) >= d0) {
        return Vec3{0.0, 0.0, 0.0};
    }
    const double d     = static_cast<double>(cell.d);
    const double scale = k * (1.0 / d - 1.0 / d0) / (d * d);
    // Use sgrad (smoothed direction) for APF; exact d is preserved for safety.
    const Vec3& dir = cell.sgrad;
    return Vec3{scale * dir.x, scale * dir.y, scale * dir.z};
}

// ─── repulsion_smoothed ──────────────────────────────────────────────────────
//
// Velocity-aware APF repulsion.  The repulsion force F = k·(1/d−1/d0)/d²·dir
// is averaged over a Gaussian kernel of radius R(v) = clamp(v²/(2·a_max), R_min, d0)
// in world space.  R_min = cell_size_m (one cell).
//
// Each cell's contribution is weighted by exp(−‖Δx‖²/(2σ²)), σ=R/2.
// The weighted sum is divided by the total weight to give the average force;
// this preserves physical units (m/s² when k has appropriate scaling).
//
// Complexity: O((2·iR+1)³) unordered_map probes per call.  For R=3 m (1 m cells)
// that is ≤7³=343 probes; most miss in a sparse field, so real cost is much lower.

Vec3 LocalESDFMap::repulsion_smoothed(
    const Vec3& pos, const Vec3& vel,
    double d0, double k, double a_max_mps2) const noexcept {

    const double speed2 = vel.x*vel.x + vel.y*vel.y + vel.z*vel.z;
    const double R = std::clamp(speed2 / (2.0 * a_max_mps2),
                                config_.cell_size_m,   // R_min: at least 1 cell
                                d0);                   // R_max: truncation radius
    const double sigma  = R * 0.5;
    const double inv2s2 = 1.0 / (2.0 * sigma * sigma);

    // Integer cell radii per axis (ceil so we don't clip the R boundary).
    const int iR_xy = static_cast<int>(std::ceil(R / config_.cell_size_m));
    const int iR_z  = static_cast<int>(std::ceil(R / config_.vertical_cell_size_m));

    const auto cen = key_for_point(pos);

    double ax = 0.0, ay = 0.0, az = 0.0, total_w = 0.0;

    for (int di = -iR_xy; di <= iR_xy; ++di) {
        for (int dj = -iR_xy; dj <= iR_xy; ++dj) {
            for (int dk = -iR_z; dk <= iR_z; ++dk) {
                const auto nk = CellKey{cen.x + di, cen.y + dj, cen.z + dk};
                const auto it = cells_.find(nk);
                if (it == cells_.end()) continue;
                const auto& cell = it->second;
                // Only free shell cells contribute repulsion.
                if (cell.d <= 0.0f || static_cast<double>(cell.d) >= d0) continue;

                // World-space squared distance from query point to cell centre.
                const double ex = cell.centre.x - pos.x;
                const double ey = cell.centre.y - pos.y;
                const double ez = cell.centre.z - pos.z;
                const double w  = std::exp(-(ex*ex + ey*ey + ez*ez) * inv2s2);

                const double d     = static_cast<double>(cell.d);
                const double scale = k * (1.0/d - 1.0/d0) / (d * d);
                ax += w * scale * cell.sgrad.x;
                ay += w * scale * cell.sgrad.y;
                az += w * scale * cell.sgrad.z;
                total_w += w;
            }
        }
    }

    if (total_w < 1.0e-12) return Vec3{0.0, 0.0, 0.0};
    return Vec3{ax / total_w, ay / total_w, az / total_w};
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
