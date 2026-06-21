// compute_esdf.cpp
//
// Stage 3: L3 ESDF free function.
//
// Implements compute_esdf(l2, centre_world, window_half_m, d0_m).
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
// Gradient ∇d is computed by central finite differences on the EDT result.
//
// Window alignment: the grid origin is snapped to the nearest L2 cell boundary so
// ESDF keys are identical to L2 keys in world coordinates.

#include "dedalus/avoidance/local_esdf_map.hpp"
#include "dedalus/avoidance/mission_local_planning_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace dedalus {
namespace {

// ─── 1D separable EDT ────────────────────────────────────────────────────────
//
// g[q] = min_p { (q−p)² · s² + f[p] }  for valid (non-INF) sources p.
// Inputs and outputs are in world-space squared units.

static constexpr float kInfSq = 1.0e20f;

static void dt1d(const float* f, float* g, int n, float s) {
    const float s2 = s * s;

    std::vector<int>   v(static_cast<std::size_t>(n));
    std::vector<float> z(static_cast<std::size_t>(n + 1));

    int k = -1;  // index of topmost parabola in lower envelope (-1 = empty)

    for (int q = 0; q < n; ++q) {
        if (f[q] >= kInfSq) continue;

        // Compute intersection of the new parabola (source q, value f[q])
        // with the current top parabola.  Pop the stack while the new parabola
        // dominates from a point left of the current top's left boundary.
        float s_int = -kInfSq;
        while (k >= 0) {
            const int p = v[k];
            // Intersection in grid-index space:
            //   x = (p+q)/2 + (f[q]−f[p]) / (2·s²·(q−p))
            s_int = (static_cast<float>(p + q) * 0.5f) +
                    (f[q] - f[p]) / (2.0f * s2 * static_cast<float>(q - p));
            if (s_int > z[k]) break;  // new parabola takes over to the right
            --k;
            s_int = -kInfSq;
        }

        ++k;
        v[static_cast<std::size_t>(k)] = q;
        z[static_cast<std::size_t>(k)]     = s_int;
        z[static_cast<std::size_t>(k + 1)] = kInfSq;
    }

    if (k < 0) {
        // No occupied source in this column.
        for (int q = 0; q < n; ++q) g[q] = kInfSq;
        return;
    }

    int j = 0;
    for (int q = 0; q < n; ++q) {
        while (j < k && z[static_cast<std::size_t>(j + 1)] < static_cast<float>(q)) ++j;
        const float diff = static_cast<float>(q - v[static_cast<std::size_t>(j)]) * s;
        g[q] = diff * diff + f[v[static_cast<std::size_t>(j)]];
    }
}

}  // namespace

// ─── compute_esdf ────────────────────────────────────────────────────────────

LocalESDFMap compute_esdf(const MissionLocalPlanningMap& l2,
                           const Vec3& centre_world,
                           double window_half_m,
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
    const int Nx = std::max(1, static_cast<int>(std::ceil(2.0 * window_half_m / sx)));
    const int Ny = std::max(1, static_cast<int>(std::ceil(2.0 * window_half_m / sy)));
    const int Nz = std::max(1, static_cast<int>(std::ceil(2.0 * window_half_m / sz)));

    // Snap window origin to the nearest L2 grid corner so ESDF and L2 keys
    // are aligned in world coordinates.
    const int xi_origin = static_cast<int>(
        std::floor((centre_world.x - static_cast<double>(Nx) * sx * 0.5) / sx));
    const int yi_origin = static_cast<int>(
        std::floor((centre_world.y - static_cast<double>(Ny) * sy * 0.5) / sy));
    const int zi_origin = static_cast<int>(
        std::floor((centre_world.z - static_cast<double>(Nz) * sz * 0.5) / sz));

    // World-frame lower corner.
    const Vec3 origin{
        static_cast<double>(xi_origin) * sx,
        static_cast<double>(yi_origin) * sy,
        static_cast<double>(zi_origin) * sz,
    };

    // Grid index → world-frame voxel centre.
    auto cell_centre = [&](int i, int j, int k) -> Vec3 {
        return Vec3{
            origin.x + (static_cast<double>(i) + 0.5) * sx,
            origin.y + (static_cast<double>(j) + 0.5) * sy,
            origin.z + (static_cast<double>(k) + 0.5) * sz,
        };
    };

    // World position → grid index.  Returns false if outside the window.
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

    // ── Binary occupancy grid ─────────────────────────────────────────────────
    std::vector<bool> occ(static_cast<std::size_t>(N), false);

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
                occ[static_cast<std::size_t>(gi * Nyz + gj * Nz + gk)] = true;
            }
        }
    }

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
                dt1d(col.data(), out.data(), Nx, static_cast<float>(sx));
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
                dt1d(col.data(), out.data(), Ny, static_cast<float>(sy));
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
                dt1d(col.data(), out.data(), Nz, static_cast<float>(sz));
                for (int k = 0; k < Nz; ++k) {
                    g3[static_cast<std::size_t>(i * Nyz + j * Nz + k)] =
                        out[static_cast<std::size_t>(k)];
                }
            }
        }
    }

    // ── Build shell cells ─────────────────────────────────────────────────────
    //
    // Gradient by central differences on sqrt(g3), clamped at window boundary.
    // For occupied cells: d = −0.5 m (clamped).
    // Skip free cells with d ≥ d0.

    const float d0f   = static_cast<float>(d0_m);
    const float inv2sx = 1.0f / (2.0f * static_cast<float>(sx));
    const float inv2sy = 1.0f / (2.0f * static_cast<float>(sy));
    const float inv2sz = 1.0f / (2.0f * static_cast<float>(sz));

    // Clamped sqrt accessor for gradient finite differences.
    auto dist_at = [&](int i, int j, int k) -> float {
        i = std::max(0, std::min(Nx - 1, i));
        j = std::max(0, std::min(Ny - 1, j));
        k = std::max(0, std::min(Nz - 1, k));
        return std::sqrt(g3[static_cast<std::size_t>(i * Nyz + j * Nz + k)]);
    };

    for (int i = 0; i < Nx; ++i) {
        for (int j = 0; j < Ny; ++j) {
            for (int k = 0; k < Nz; ++k) {
                const std::size_t idx =
                    static_cast<std::size_t>(i * Nyz + j * Nz + k);
                const bool is_occ = occ[idx];
                const float d_raw = std::sqrt(g3[idx]);

                float d;
                if (is_occ) {
                    d = -0.5f;
                } else {
                    d = d_raw;
                    if (d >= d0f) continue;  // outside truncation — skip
                }

                // Gradient ∇d by central differences (one-sided at boundary).
                const float gx =
                    (dist_at(i + 1, j, k) - dist_at(i - 1, j, k)) * inv2sx;
                const float gy =
                    (dist_at(i, j + 1, k) - dist_at(i, j - 1, k)) * inv2sy;
                const float gz =
                    (dist_at(i, j, k + 1) - dist_at(i, j, k - 1)) * inv2sz;
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

                esdf.cells_[esdf.key_for_point(cell.centre)] = cell;
            }
        }
    }

    return esdf;
}

}  // namespace dedalus
