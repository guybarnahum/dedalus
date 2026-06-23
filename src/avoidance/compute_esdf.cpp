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
                           double d0_m) {
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
    std::vector<std::uint8_t> occ(static_cast<std::size_t>(N), 0U);

    {
        const Bounds3 bbox{
            Vec3{origin.x, origin.y, origin.z},
            Vec3{origin.x + static_cast<double>(Nx) * sx,
                 origin.y + static_cast<double>(Ny) * sy,
                 origin.z + static_cast<double>(Nz) * sz},
        };
        for (const auto& c : l2.query_occupied_in_box(bbox)) {
            int gi, gj, gk;
            if (world_to_idx(c, &gi, &gj, &gk)) {
                occ[static_cast<std::size_t>(gi * Nyz + gj * Nz + gk)] = 1U;
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

    // ── Build shell cells ─────────────────────────────────────────────────────
    // No pre-computed dist[] array: filter on g3 (squared) first, call sqrt
    // only for the small fraction of cells that are actually shell cells.
    //
    // Gradient direction: ∂d/∂x ∝ (g3[i+1]-g3[i-1])/sx (the common 4d factor
    // cancels during normalisation), so we differentiate g3 directly.
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

    for (int i = 0; i < Nx; ++i) {
        for (int j = 0; j < Ny; ++j) {
            for (int k = 0; k < Nz; ++k) {
                const std::size_t idx =
                    static_cast<std::size_t>(i * Nyz + j * Nz + k);
                const bool is_occ = occ[idx] != 0U;
                const float g_val = g3[idx];

                // Skip non-shell free cells with a squared comparison (no sqrt).
                if (!is_occ && g_val >= d0_sq) continue;

                // sqrt only for free shell cells (typically a few thousand).
                const float d = is_occ ? -0.5f : std::sqrt(g_val);

                // Gradient direction from central differences on g3.
                // ∂d/∂x = Δg_x / (4d·sx); normalising cancels the 4d factor.
                const float gx = (g3_at(i + 1, j, k) - g3_at(i - 1, j, k)) / sxf;
                const float gy = (g3_at(i, j + 1, k) - g3_at(i, j - 1, k)) / syf;
                const float gz = (g3_at(i, j, k + 1) - g3_at(i, j, k - 1)) / szf;
                const float glen = std::sqrt(gx * gx + gy * gy + gz * gz);

                Vec3 grad{0.0, 0.0, 0.0};
                if (glen > 1.0e-6f) {
                    grad = Vec3{
                        static_cast<double>(gx / glen),
                        static_cast<double>(gy / glen),
                        static_cast<double>(gz / glen),
                    };
                }

                LocalESDFCell cell;
                cell.centre = cell_centre(i, j, k);
                cell.d      = d;
                cell.grad   = grad;

                // Direct key: cell_centre(i,j,k) always maps to
                // {xi_origin+i, yi_origin+j, zi_origin+k} by construction.
                esdf.cells_[LocalESDFMap::CellKey{
                    xi_origin + i, yi_origin + j, zi_origin + k}] = cell;
            }
        }
    }

    // ── Smoothing pass ────────────────────────────────────────────────────────
    // Average each cell's gradient with its 6-connected neighbours that are
    // also shell cells, then renormalize → sgrad.  This gives a more
    // surface-normal-like direction than the raw EDT central-difference grad,
    // reducing APF discontinuities at edges and corners.  Exact distance d is
    // unchanged — only the repulsion direction is smoothed.
    static constexpr int kDirs[6][3] = {
        {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    for (auto& [key, cell] : esdf.cells_) {
        double ax = cell.grad.x, ay = cell.grad.y, az = cell.grad.z;
        int w = 1;
        for (const auto& dk : kDirs) {
            const auto nk = LocalESDFMap::CellKey{
                key.x + dk[0], key.y + dk[1], key.z + dk[2]};
            const auto it = esdf.cells_.find(nk);
            if (it == esdf.cells_.end()) continue;
            ax += it->second.grad.x;
            ay += it->second.grad.y;
            az += it->second.grad.z;
            ++w;
        }
        const double len = std::sqrt(ax * ax + ay * ay + az * az);
        cell.sgrad = (len > 1.0e-6)
            ? Vec3{ax / len, ay / len, az / len}
            : cell.grad;
    }

    return esdf;
}

}  // namespace dedalus
