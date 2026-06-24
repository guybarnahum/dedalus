// compute_esdf.cpp
//
// Stage 3: L3 ESDF free function.
//
// Implements compute_esdf(l2, centre_world, horiz_half_m, vert_half_m, d0_m).
//
// Algorithm: 3-phase separable 1D EDT (Felzenszwalb & Huttenlocher, TPAMI 2012).
//   Phase 1 (X): Binary occupancy → 1D squared distance along X for each (Y,Z) column.
//   Phase 2 (Y): Phase-1 output → extend to 2D squared distance along Y for each (X,Z).
//   Phase 3 (Z): Phase-2 output → full 3D squared Euclidean distance field.
// Each phase is O(N) via a lower-envelope-of-parabolas sweep.
//
// Anisotropic voxels (sx ≠ sz) are handled by scaling the parabola step per phase.
//
// Only shell cells (|d| < d0_m or occupied) are stored in the output LocalESDFMap.
// Gradient ∇d is computed by central finite differences on g3 (squared distances);
// sqrt is called only for the shell cells that pass the g < d0² filter.
//
// Window alignment: the grid origin is snapped to the nearest L2 cell boundary so
// ESDF keys are identical to L2 keys in world coordinates.

#include "dedalus/avoidance/local_esdf_map.hpp"
#include "dedalus/avoidance/mission_local_planning_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace dedalus {
namespace {

// ─── 1D separable EDT ────────────────────────────────────────────────────────
//
// g[q] = min_p { (q−p)² · s² + f[p] }  for valid (non-INF) sources p.
// Inputs and outputs are in world-space squared units.
// v_work / z_work must be pre-allocated to at least n / n+1 elements.

static constexpr float kInfSq = 1.0e20f;

static void dt1d(const float* f, float* g, int n, float s,
                 int* v_work, float* z_work) {
    const float s2 = s * s;
    int k = -1;

    for (int q = 0; q < n; ++q) {
        if (f[q] >= kInfSq) continue;

        float s_int = -kInfSq;
        while (k >= 0) {
            const int p = v_work[k];
            s_int = (static_cast<float>(p + q) * 0.5f) +
                    (f[q] - f[p]) / (2.0f * s2 * static_cast<float>(q - p));
            if (s_int > z_work[k]) break;
            --k;
            s_int = -kInfSq;
        }

        ++k;
        v_work[k]     = q;
        z_work[k]     = s_int;
        z_work[k + 1] = kInfSq;
    }

    if (k < 0) {
        for (int q = 0; q < n; ++q) g[q] = kInfSq;
        return;
    }

    int j = 0;
    for (int q = 0; q < n; ++q) {
        while (j < k && z_work[j + 1] < static_cast<float>(q)) ++j;
        const float diff = static_cast<float>(q - v_work[j]) * s;
        g[q] = diff * diff + f[v_work[j]];
    }
}

}  // namespace

// ─── compute_esdf ────────────────────────────────────────────────────────────

LocalESDFMap compute_esdf(const MissionLocalPlanningMap& l2,
                           const Vec3& centre_world,
                           double horiz_half_m,
                           double vert_half_m,
                           double d0_m,
                           double sample_spacing_m) {
    const auto& l2cfg = l2.config();
    const double sx = l2cfg.cell_size_m;
    const double sy = l2cfg.cell_size_m;
    const double sz = l2cfg.vertical_cell_size_m;

    LocalESDFConfig cfg;
    cfg.cell_size_m          = sx;
    cfg.vertical_cell_size_m = sz;
    cfg.d0_m                 = d0_m;

    LocalESDFMap esdf(cfg);

    // ── Grid dimensions ───────────────────────────────────────────────────────
    const int Nx = std::max(1, static_cast<int>(std::ceil(2.0 * horiz_half_m / sx)));
    const int Ny = std::max(1, static_cast<int>(std::ceil(2.0 * horiz_half_m / sy)));
    const int Nz = std::max(1, static_cast<int>(std::ceil(2.0 * vert_half_m  / sz)));

    // Snap window origin to the nearest L2 grid corner so ESDF and L2 keys
    // are aligned in world coordinates.
    const int xi_origin = static_cast<int>(
        std::floor((centre_world.x - static_cast<double>(Nx) * sx * 0.5) / sx));
    const int yi_origin = static_cast<int>(
        std::floor((centre_world.y - static_cast<double>(Ny) * sy * 0.5) / sy));
    const int zi_origin = static_cast<int>(
        std::floor((centre_world.z - static_cast<double>(Nz) * sz * 0.5) / sz));

    const Vec3 origin{
        static_cast<double>(xi_origin) * sx,
        static_cast<double>(yi_origin) * sy,
        static_cast<double>(zi_origin) * sz,
    };

    auto cell_centre = [&](int i, int j, int k) -> Vec3 {
        return Vec3{
            origin.x + (static_cast<double>(i) + 0.5) * sx,
            origin.y + (static_cast<double>(j) + 0.5) * sy,
            origin.z + (static_cast<double>(k) + 0.5) * sz,
        };
    };

    auto world_to_idx = [&](const Vec3& p, int* gi, int* gj, int* gk) -> bool {
        *gi = static_cast<int>(std::floor((p.x - origin.x) / sx));
        *gj = static_cast<int>(std::floor((p.y - origin.y) / sy));
        *gk = static_cast<int>(std::floor((p.z - origin.z) / sz));
        return (*gi >= 0 && *gi < Nx &&
                *gj >= 0 && *gj < Ny &&
                *gk >= 0 && *gk < Nz);
    };

    const int Nyz = Ny * Nz;
    const int N   = Nx * Nyz;

    // ── Binary occupancy grid (uint8_t — avoid vector<bool> bit-packing) ──────
    // ts_grid: last_updated_ns of the L2 cell occupying each voxel (0 = free).
    std::vector<std::uint8_t>  occ(static_cast<std::size_t>(N), 0U);
    std::vector<std::int64_t>  ts_grid(static_cast<std::size_t>(N), 0);

    {
        const Bounds3 bbox{
            Vec3{origin.x, origin.y, origin.z},
            Vec3{origin.x + static_cast<double>(Nx) * sx,
                 origin.y + static_cast<double>(Ny) * sy,
                 origin.z + static_cast<double>(Nz) * sz},
        };
        for (const auto& [c, ts] : l2.query_occupied_ts_in_box(bbox)) {
            int gi, gj, gk;
            if (world_to_idx(c, &gi, &gj, &gk)) {
                const auto idx = static_cast<std::size_t>(gi * Nyz + gj * Nz + gk);
                occ[idx]     = 1U;
                ts_grid[idx] = ts;
            }
        }
    }

    // Pre-allocate dt1d work arrays — one pair covers all phases.
    const int max_n = std::max({Nx, Ny, Nz});
    std::vector<int>   v_work(static_cast<std::size_t>(max_n));
    std::vector<float> z_work(static_cast<std::size_t>(max_n + 1));

    // ── Phase 1: 1D EDT along X ───────────────────────────────────────────────
    std::vector<float> g1(static_cast<std::size_t>(N));
    {
        std::vector<float> col(static_cast<std::size_t>(Nx));
        std::vector<float> out(static_cast<std::size_t>(Nx));
        for (int j = 0; j < Ny; ++j) {
            for (int k = 0; k < Nz; ++k) {
                for (int i = 0; i < Nx; ++i) {
                    col[static_cast<std::size_t>(i)] =
                        occ[static_cast<std::size_t>(i * Nyz + j * Nz + k)]
                        ? 0.0f : kInfSq;
                }
                dt1d(col.data(), out.data(), Nx,
                     static_cast<float>(sx), v_work.data(), z_work.data());
                for (int i = 0; i < Nx; ++i) {
                    g1[static_cast<std::size_t>(i * Nyz + j * Nz + k)] =
                        out[static_cast<std::size_t>(i)];
                }
            }
        }
    }

    // ── Phase 2: 1D EDT along Y ───────────────────────────────────────────────
    std::vector<float> g2(static_cast<std::size_t>(N));
    {
        std::vector<float> col(static_cast<std::size_t>(Ny));
        std::vector<float> out(static_cast<std::size_t>(Ny));
        for (int i = 0; i < Nx; ++i) {
            for (int k = 0; k < Nz; ++k) {
                for (int j = 0; j < Ny; ++j) {
                    col[static_cast<std::size_t>(j)] =
                        g1[static_cast<std::size_t>(i * Nyz + j * Nz + k)];
                }
                dt1d(col.data(), out.data(), Ny,
                     static_cast<float>(sy), v_work.data(), z_work.data());
                for (int j = 0; j < Ny; ++j) {
                    g2[static_cast<std::size_t>(i * Nyz + j * Nz + k)] =
                        out[static_cast<std::size_t>(j)];
                }
            }
        }
    }

    // ── Phase 3: 1D EDT along Z ───────────────────────────────────────────────
    std::vector<float> g3(static_cast<std::size_t>(N));
    {
        std::vector<float> col(static_cast<std::size_t>(Nz));
        std::vector<float> out(static_cast<std::size_t>(Nz));
        for (int i = 0; i < Nx; ++i) {
            for (int j = 0; j < Ny; ++j) {
                for (int k = 0; k < Nz; ++k) {
                    col[static_cast<std::size_t>(k)] =
                        g2[static_cast<std::size_t>(i * Nyz + j * Nz + k)];
                }
                dt1d(col.data(), out.data(), Nz,
                     static_cast<float>(sz), v_work.data(), z_work.data());
                for (int k = 0; k < Nz; ++k) {
                    g3[static_cast<std::size_t>(i * Nyz + j * Nz + k)] =
                        out[static_cast<std::size_t>(k)];
                }
            }
        }
    }

    // ── Coarse APF sampling ───────────────────────────────────────────────────
    // The EDT ran at fine L2 resolution (sx,sz); output cells are stored every
    // sample_spacing_m metres so the map is ~(spacing/cell_size)^2 times sparser.
    const float d0f   = static_cast<float>(d0_m);
    const float d0_sq = d0f * d0f;
    const float sxf   = static_cast<float>(sx);
    const float syf   = static_cast<float>(sy);
    const float szf   = static_cast<float>(sz);

    auto g3_at = [&](int i, int j, int k) -> float {
        i = std::max(0, std::min(Nx - 1, i));
        j = std::max(0, std::min(Ny - 1, j));
        k = std::max(0, std::min(Nz - 1, k));
        return g3[static_cast<std::size_t>(i * Nyz + j * Nz + k)];
    };

    const double spc_xy = std::max(sx, sample_spacing_m);  // >= fine cell size
    const double spc_z  = sz;                               // keep Z at L2 resolution

    // step_x/step_z: coarse stride in fine-grid cell units
    const int step_x = std::max(1, static_cast<int>(std::round(spc_xy / sx)));
    const int step_z = std::max(1, static_cast<int>(std::round(spc_z  / sz)));

    // Gaussian sigma = spc_xy / 2 (so the 2-sigma radius matches one coarse step)
    const double sigma_xy    = spc_xy * 0.5;
    const double sigma_z     = spc_z  * 0.5;
    const double inv2sig2_xy = 1.0 / (2.0 * sigma_xy * sigma_xy);
    const double inv2sig2_z  = 1.0 / (2.0 * sigma_z  * sigma_z);

    // Gather radius in fine-grid cell units (±1 coarse step)
    const int rx = step_x;
    const int rz = step_z;

    // Update output config so key_for_point() uses coarse resolution.
    esdf.config_.cell_size_m          = spc_xy;
    esdf.config_.vertical_cell_size_m = spc_z;
    esdf.config_.sample_spacing_m     = spc_xy;

    // Iterate coarse grid centres (every step_x/step_z fine cells).
    // For each, gather Gaussian-weighted APF from fine shell cells nearby.
    for (int ci = step_x / 2; ci < Nx; ci += step_x) {
        for (int cj = step_x / 2; cj < Ny; cj += step_x) {
            for (int ck = step_z / 2; ck < Nz; ck += step_z) {

                // World position of this coarse sample centre
                const Vec3 cc = cell_centre(ci, cj, ck);

                // Accumulate Gaussian-weighted APF from fine shell cells nearby.
                // Also gather max timestamp of nearby occupied cells (L2 age).
                double ax = 0.0, ay = 0.0, az = 0.0;
                double wapf_sum = 0.0, w_sum = 0.0;
                float d_min = d0f;
                std::int64_t max_ts = 0;

                for (int di = -rx; di <= rx; ++di) {
                    const int ni = ci + di;
                    if (ni < 0 || ni >= Nx) continue;
                    for (int dj = -rx; dj <= rx; ++dj) {
                        const int nj = cj + dj;
                        if (nj < 0 || nj >= Ny) continue;
                        for (int dk = -rz; dk <= rz; ++dk) {
                            const int nk = ck + dk;
                            if (nk < 0 || nk >= Nz) continue;

                            const std::size_t idx =
                                static_cast<std::size_t>(ni * Nyz + nj * Nz + nk);
                            // Harvest timestamp from occupied neighbours.
                            if (occ[idx]) {
                                if (ts_grid[idx] > max_ts) max_ts = ts_grid[idx];
                                continue;
                            }
                            const float g_val = g3[idx];
                            if (g_val >= d0_sq) continue;  // outside shell

                            const float d_i = std::sqrt(g_val);

                            // Gradient from g3 central differences (normalised)
                            const float gx_r = (g3_at(ni+1,nj,nk) - g3_at(ni-1,nj,nk)) / sxf;
                            const float gy_r = (g3_at(ni,nj+1,nk) - g3_at(ni,nj-1,nk)) / syf;
                            const float gz_r = (g3_at(ni,nj,nk+1) - g3_at(ni,nj,nk-1)) / szf;
                            const float glen =
                                std::sqrt(gx_r*gx_r + gy_r*gy_r + gz_r*gz_r);
                            if (glen < 1.0e-6f) continue;

                            // Spatial Gaussian weight
                            const double ex = di * sx;
                            const double ey = dj * sy;
                            const double ez = dk * sz;
                            const double w  =
                                std::exp(-(ex*ex + ey*ey) * inv2sig2_xy
                                         - ez*ez          * inv2sig2_z);

                            // APF magnitude for this fine cell
                            const double apf_i =
                                (1.0 / static_cast<double>(d_i) - 1.0 / d0_m)
                                / static_cast<double>(d_i);

                            ax      += w * apf_i * static_cast<double>(gx_r / glen);
                            ay      += w * apf_i * static_cast<double>(gy_r / glen);
                            az      += w * apf_i * static_cast<double>(gz_r / glen);
                            wapf_sum += w * apf_i;
                            w_sum    += w;
                            d_min = std::min(d_min, d_i);
                        }
                    }
                }

                if (w_sum < 1.0e-10 || wapf_sum < 1.0e-10) continue;

                const double alen = std::sqrt(ax*ax + ay*ay + az*az);
                if (alen < 1.0e-10) continue;

                const Vec3 dir{ax / alen, ay / alen, az / alen};

                LocalESDFCell cell;
                cell.centre          = cc;
                cell.d               = d_min;
                cell.grad            = dir;   // APF-weighted direction (grad == sgrad at coarse scale)
                cell.sgrad           = dir;
                cell.last_updated_ns = max_ts;  // inherited from nearest L2 obstacle

                // Coarse key: use coarse (spc_xy) resolution
                const int cki = static_cast<int>(
                    std::floor(cc.x / spc_xy));
                const int ckj = static_cast<int>(
                    std::floor(cc.y / spc_xy));
                const int ckk = static_cast<int>(
                    std::floor(cc.z / spc_z));
                esdf.cells_[LocalESDFMap::CellKey{cki, ckj, ckk}] = cell;
            }
        }
    }

    return esdf;
}

}  // namespace dedalus
