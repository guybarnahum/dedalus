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
// Resolution: the dense grid (and hence Nx/Ny/Nz, the dominant O(N) cost) is
// sized at max(l2.config().cell_size_m, sample_spacing_m) in XY, not at L2's
// raw fine resolution — computing densely at the fine resolution and only
// subsampling the *output* afterward (the previous design) never actually
// made the expensive part cheaper, regardless of how coarse the stored
// result was. Occupancy is snapped to this coarser grid before the EDT
// runs, so the distance field is accurate to ~sample_spacing_m near
// obstacle boundaries rather than ~cell_size_m — acceptable since L3 is an
// intentionally-lagging, approximate cache of L2 (see CoreStackRunner's
// ESDF catch-up step), not a real-time collision surface.
// LocalESDFMap::repulsion() already interpolates smoothly between stored
// cells, so no smoothing is lost by not computing at the finer resolution.
//
// Only shell cells (|d| < d0_m or occupied) are stored in the output LocalESDFMap.
// Gradient ∇d is computed by central finite differences on g3 (squared distances);
// sqrt is called only for the shell cells that pass the g < d0² filter.
//
// Window alignment: the grid origin is snapped to the nearest coarse-grid
// boundary so repeated calls at the same resolution produce identical keys.

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
    // >= cell_size_m per LocalESDFConfig::sample_spacing_m's contract — this
    // guard used to live only at the output-sampling stage; moving it here
    // means it now also governs the dense grid the EDT actually pays for.
    const double sx = std::max(l2cfg.cell_size_m, sample_spacing_m);
    const double sy = sx;
    const double sz = l2cfg.vertical_cell_size_m;  // already coarse; unchanged

    LocalESDFConfig cfg;
    cfg.cell_size_m          = sx;
    cfg.vertical_cell_size_m = sz;
    cfg.d0_m                 = d0_m;
    cfg.sample_spacing_m     = sx;

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
                occ[idx] = 1U;
                // Multiple fine L2 cells can land in the same coarse voxel
                // now that sx/sy may be coarser than l2cfg.cell_size_m; keep
                // the most recent of them.
                ts_grid[idx] = std::max(ts_grid[idx], ts);
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

    // ── Store shell cells ─────────────────────────────────────────────────────
    // g3 is already at the target (coarse) resolution now, so every grid
    // cell IS an output cell — no striding and no gather-from-finer-cells
    // pass needed (there's nothing finer left to gather from). Gradients
    // come directly from central differences on g3 — previously the
    // "gather window was empty" fallback path; now the only path.
    const float d0f = static_cast<float>(d0_m);
    const float sxf  = static_cast<float>(sx);
    const float syf  = static_cast<float>(sy);
    const float szf  = static_cast<float>(sz);

    auto g3_at = [&](int i, int j, int k) -> float {
        i = std::max(0, std::min(Nx - 1, i));
        j = std::max(0, std::min(Ny - 1, j));
        k = std::max(0, std::min(Nz - 1, k));
        return g3[static_cast<std::size_t>(i * Nyz + j * Nz + k)];
    };

    for (int i = 0; i < Nx; ++i) {
        for (int j = 0; j < Ny; ++j) {
            for (int k = 0; k < Nz; ++k) {
                const std::size_t idx = static_cast<std::size_t>(i * Nyz + j * Nz + k);
                const Vec3 cc = cell_centre(i, j, k);
                const LocalESDFMap::CellKey key{
                    static_cast<int>(std::floor(cc.x / sx)),
                    static_cast<int>(std::floor(cc.y / sy)),
                    static_cast<int>(std::floor(cc.z / sz)),
                };

                if (occ[idx]) {
                    LocalESDFCell cell;
                    cell.centre          = cc;
                    cell.d               = -0.5f;  // clamped interior value
                    cell.last_updated_ns = ts_grid[idx];
                    esdf.cells_[key] = cell;
                    continue;
                }

                const float d = std::sqrt(g3[idx]);
                if (d >= d0f) continue;  // outside shell — skip

                const float gx = (g3_at(i+1,j,k) - g3_at(i-1,j,k)) / sxf;
                const float gy = (g3_at(i,j+1,k) - g3_at(i,j-1,k)) / syf;
                const float gz = (g3_at(i,j,k+1) - g3_at(i,j,k-1)) / szf;
                const float glen = std::sqrt(gx*gx + gy*gy + gz*gz);
                Vec3 dir{};
                if (glen > 1.0e-6f) {
                    dir = {gx / glen, gy / glen, gz / glen};
                }

                // This cell has no last_updated_ns of its own (it isn't
                // occupied) — take the freshest timestamp among directly
                // adjacent occupied cells. Fixed ±1 radius, plain array
                // indexing: O(1) per neighbour, not a scan, and the same
                // real-world radius the old fine-grid gather used.
                std::int64_t nearest_ts = 0;
                for (int di = -1; di <= 1; ++di) {
                    const int ni = i + di;
                    if (ni < 0 || ni >= Nx) continue;
                    for (int dj = -1; dj <= 1; ++dj) {
                        const int nj = j + dj;
                        if (nj < 0 || nj >= Ny) continue;
                        for (int dk = -1; dk <= 1; ++dk) {
                            const int nk = k + dk;
                            if (nk < 0 || nk >= Nz) continue;
                            const auto nidx = static_cast<std::size_t>(ni * Nyz + nj * Nz + nk);
                            if (occ[nidx]) {
                                nearest_ts = std::max(nearest_ts, ts_grid[nidx]);
                            }
                        }
                    }
                }

                LocalESDFCell cell;
                cell.centre          = cc;
                cell.d               = d;
                cell.grad            = dir;
                cell.sgrad           = dir;
                cell.last_updated_ns = nearest_ts;
                esdf.cells_[key] = cell;
            }
        }
    }

    return esdf;
}

}  // namespace dedalus
