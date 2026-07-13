#include "dedalus/sensing/depth_projection_kernel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <random>
#include <vector>

namespace dedalus {
namespace {

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

void undistort_k1k2(float xn, float yn, float k1, float k2, float& xd, float& yd) {
    const float r2     = xn * xn + yn * yn;
    const float radial = 1.0F + k1 * r2 + k2 * r2 * r2;
    xd = xn / radial;
    yd = yn / radial;
}

// Unproject a depth pixel to a local-frame 3D point.
// Returns false if depth is outside [min, max].
bool unproject(int u, int v, float inverse_depth,
               const ProjectionParams& p,
               float& lx, float& ly, float& lz) {
    if (!std::isfinite(inverse_depth) || inverse_depth <= 0.0F) return false;
    const float depth_m = p.scale / inverse_depth;
    if (depth_m < p.min_depth_m || depth_m > p.max_depth_m) return false;

    const float inv_fx = 1.0F / p.fx;
    const float inv_fy = 1.0F / p.fy;
    float xn = (static_cast<float>(u) - p.cx) * inv_fx;
    float yn = (static_cast<float>(v) - p.cy) * inv_fy;
    if (p.k1 != 0.0F || p.k2 != 0.0F) {
        undistort_k1k2(xn, yn, p.k1, p.k2, xn, yn);
    }
    const float xc = xn * depth_m;
    const float yc = yn * depth_m;
    const float zc = depth_m;

    lx = p.origin_x + zc * p.forward_x + xc * p.right_x - yc * p.up_x;
    ly = p.origin_y + zc * p.forward_y + xc * p.right_y - yc * p.up_y;
    lz = p.origin_z + zc * p.forward_z + xc * p.right_z - yc * p.up_z;
    return true;
}

// ---------------------------------------------------------------------------
// RANSAC plane helpers
// ---------------------------------------------------------------------------

struct Plane { float nx, ny, nz, d; };

// Fit a plane through three points. Returns false if degenerate.
bool fit_plane_3pts(const float* a, const float* b, const float* c, Plane& out) {
    const float abx = b[0]-a[0], aby = b[1]-a[1], abz = b[2]-a[2];
    const float acx = c[0]-a[0], acy = c[1]-a[1], acz = c[2]-a[2];
    const float nx = aby*acz - abz*acy;
    const float ny = abz*acx - abx*acz;
    const float nz = abx*acy - aby*acx;
    const float len = std::sqrt(nx*nx + ny*ny + nz*nz);
    if (len < 1.0e-6F) return false;
    out.nx = nx/len;  out.ny = ny/len;  out.nz = nz/len;
    out.d  = -(out.nx*a[0] + out.ny*a[1] + out.nz*a[2]);
    return true;
}

float plane_dist(const Plane& p, float x, float y, float z) {
    return std::abs(p.nx*x + p.ny*y + p.nz*z + p.d);
}

// Least-squares plane fit over a set of points (Welford centroid + covariance).
// Returns false if fewer than 3 points.
bool fit_plane_ls(const std::vector<std::array<float,3>>& pts, Plane& out) {
    if (pts.size() < 3) return false;
    // Centroid
    float cx = 0.0F, cy = 0.0F, cz = 0.0F;
    for (const auto& p : pts) { cx += p[0]; cy += p[1]; cz += p[2]; }
    const float n = static_cast<float>(pts.size());
    cx /= n;  cy /= n;  cz /= n;
    // Scatter matrix (symmetric 3×3)
    float sxx=0,sxy=0,sxz=0,syy=0,syz=0,szz=0;
    for (const auto& p : pts) {
        const float dx=p[0]-cx, dy=p[1]-cy, dz=p[2]-cz;
        sxx+=dx*dx; sxy+=dx*dy; sxz+=dx*dz;
        syy+=dy*dy; syz+=dy*dz; szz+=dz*dz;
    }
    // Power-iteration to find smallest eigenvector (normal direction).
    // Start with the column of the scatter matrix with the smallest diagonal.
    float nx, ny, nz;
    if (sxx <= syy && sxx <= szz)      { nx=1.0F; ny=0.0F; nz=0.0F; }
    else if (syy <= sxx && syy <= szz) { nx=0.0F; ny=1.0F; nz=0.0F; }
    else                                { nx=0.0F; ny=0.0F; nz=1.0F; }
    // 16 iterations of inverse power method (approximation)
    for (int iter = 0; iter < 16; ++iter) {
        // Multiply by (I - S/trace) or similar: use deflation via cross products.
        // Simpler: multiply by scatter and find largest; then subtract to get smallest.
        // Actually: just do 8 steps of multiplying by (diag_max*I - S) to find min eigenvec.
        const float t  = sxx + syy + szz;
        const float wx = (t - sxx)*nx - sxy*ny - sxz*nz;
        const float wy = -sxy*nx + (t - syy)*ny - syz*nz;
        const float wz = -sxz*nx - syz*ny + (t - szz)*nz;
        const float wlen = std::sqrt(wx*wx + wy*wy + wz*wz);
        if (wlen < 1.0e-8F) break;
        nx = wx/wlen;  ny = wy/wlen;  nz = wz/wlen;
    }
    const float len = std::sqrt(nx*nx + ny*ny + nz*nz);
    if (len < 1.0e-6F) return false;
    out.nx = nx/len;  out.ny = ny/len;  out.nz = nz/len;
    out.d  = -(out.nx*cx + out.ny*cy + out.nz*cz);
    return true;
}

// ---------------------------------------------------------------------------
// Thin-structure helpers
// ---------------------------------------------------------------------------

// Clamp inverse_depth to a metric depth, treating invalid pixels as max_depth_m.
float safe_depth_m(float dr, const ProjectionParams& p) {
    if (!std::isfinite(dr) || dr <= 0.0F) return p.max_depth_m;
    const float d = p.scale / dr;
    return (d >= p.min_depth_m && d <= p.max_depth_m) ? d : p.max_depth_m;
}

// Local max depth in a 5×5 neighbourhood, excluding the centre pixel.
float local_max_depth(const float* inverse_depth, int W, int H,
                      int u, int v, const ProjectionParams& p) {
    float mx = 0.0F;
    for (int dv = -2; dv <= 2; ++dv) {
        for (int du = -2; du <= 2; ++du) {
            if (du == 0 && dv == 0) continue;
            const int nu = u + du;
            const int nv = v + dv;
            if (nu < 0 || nu >= W || nv < 0 || nv >= H) continue;
            const float d = safe_depth_m(
                inverse_depth[static_cast<std::size_t>(nv)*static_cast<std::size_t>(W) +
                               static_cast<std::size_t>(nu)], p);
            if (d > mx) mx = d;
        }
    }
    return mx;
}

// BFS connected-component labelling on a boolean mask.
// Returns the list of pixels in this component.
std::vector<std::pair<int,int>> bfs_component(
    const std::vector<bool>& mask, std::vector<bool>& visited,
    int W, int H, int u0, int v0) {
    std::vector<std::pair<int,int>> comp;
    std::queue<std::pair<int,int>> q;
    q.push({u0, v0});
    visited[static_cast<std::size_t>(v0)*static_cast<std::size_t>(W) +
            static_cast<std::size_t>(u0)] = true;
    while (!q.empty()) {
        auto [u, v] = q.front();  q.pop();
        comp.push_back({u, v});
        // 4-connectivity neighbours
        const int du[4] = {1,-1,0,0};
        const int dv[4] = {0,0,1,-1};
        for (int k = 0; k < 4; ++k) {
            const int nu = u + du[k];
            const int nv = v + dv[k];
            if (nu < 0 || nu >= W || nv < 0 || nv >= H) continue;
            const std::size_t idx =
                static_cast<std::size_t>(nv)*static_cast<std::size_t>(W) +
                static_cast<std::size_t>(nu);
            if (!mask[idx] || visited[idx]) continue;
            visited[idx] = true;
            q.push({nu, nv});
        }
    }
    return comp;
}

}  // namespace

// ---------------------------------------------------------------------------
// CPU projection kernel
// ---------------------------------------------------------------------------

void project_depth_to_device_evidence(
    const float*             inverse_depth,
    const ProjectionParams&  params,
    DeviceObstacleEvidence*  out,
    std::uint32_t&           count_out) {

    count_out = 0;
    if (params.fx <= 0.0F || params.fy <= 0.0F || params.scale <= 0.0F) return;
    if (params.grid_cols <= 0 || params.grid_rows <= 0) return;

    const float inv_fx = 1.0F / params.fx;
    const float inv_fy = 1.0F / params.fy;

    // Block-minimum sampling: divide the depth map into grid_cols × grid_rows cells.
    // For each cell, project the closest valid pixel (max inverse_depth = min depth_m).
    // One evidence point per cell — no cross-cell voxel deduplication.
    const int BW = params.width  / params.grid_cols;  // block width in pixels
    const int BH = params.height / params.grid_rows;  // block height in pixels
    if (BW <= 0 || BH <= 0) return;

    for (int gr = 0; gr < params.grid_rows && count_out < params.max_evidence; ++gr) {
        const int v0 = gr * BH;
        const int v1 = std::min(v0 + BH, params.height);

        for (int gc = 0; gc < params.grid_cols && count_out < params.max_evidence; ++gc) {
            const int u0 = gc * BW;
            const int u1 = std::min(u0 + BW, params.width);

            // Find the pixel in this cell with the highest inverse_depth (= closest obstacle).
            float best_id = 0.0F;
            int   best_u  = -1;
            int   best_v  = -1;
            for (int v = v0; v < v1; ++v) {
                for (int u = u0; u < u1; ++u) {
                    const float id = inverse_depth[
                        static_cast<std::size_t>(v) * static_cast<std::size_t>(params.width) +
                        static_cast<std::size_t>(u)];
                    if (!std::isfinite(id) || id <= 0.0F) continue;
                    const float dm = params.scale / id;
                    if (dm < params.min_depth_m || dm > params.max_depth_m) continue;
                    if (id > best_id) { best_id = id; best_u = u; best_v = v; }
                }
            }
            if (best_u < 0) continue;  // no valid pixel in this cell

            const float depth_m = params.scale / best_id;
            float xn = (static_cast<float>(best_u) - params.cx) * inv_fx;
            float yn = (static_cast<float>(best_v) - params.cy) * inv_fy;
            if (params.k1 != 0.0F || params.k2 != 0.0F) {
                undistort_k1k2(xn, yn, params.k1, params.k2, xn, yn);
            }
            const float xc = xn * depth_m;
            const float yc = yn * depth_m;
            const float zc = depth_m;

            const float lx = params.origin_x + zc*params.forward_x + xc*params.right_x - yc*params.up_x;
            const float ly = params.origin_y + zc*params.forward_y + xc*params.right_y - yc*params.up_y;
            const float lz = params.origin_z + zc*params.forward_z + xc*params.right_z - yc*params.up_z;

            const auto ix = static_cast<std::int32_t>(std::floor(lx / params.voxel_size_m));
            const auto iy = static_cast<std::int32_t>(std::floor(ly / params.voxel_size_m));
            const auto iz = static_cast<std::int32_t>(std::floor(lz / params.voxel_size_m));

            const float half = 0.5F * params.voxel_size_m;
            DeviceObstacleEvidence& ev = out[count_out++];
            ev.center_x = (static_cast<float>(ix) + 0.5F) * params.voxel_size_m;
            ev.center_y = (static_cast<float>(iy) + 0.5F) * params.voxel_size_m;
            ev.center_z = (static_cast<float>(iz) + 0.5F) * params.voxel_size_m;
            ev.size_x = ev.size_y = ev.size_z = half * 2.0F;
            ev.confidence             = 0.75F;
            ev.range_m                = depth_m;
            ev.state                  = 2U;  // Occupied
            ev.shape                  = 3U;  // SurfacePatch
            ev.is_thin_structure_hint = 0U;
            ev.is_surface_hint        = 1U;

            // Per-voxel surface normal via finite differences.
            // Unproject the right (u+1) and down (v+1) neighbors and form the
            // cross product of the two tangent vectors.  Falls back to zero
            // (has_surface_normal=false in inflate()) if a neighbor is invalid.
            {
                float rx, ry, rz, bx, by, bz;
                bool has_r = false, has_b = false;
                if (best_u + 1 < params.width) {
                    const float id_r = inverse_depth[
                        static_cast<std::size_t>(best_v) * static_cast<std::size_t>(params.width) +
                        static_cast<std::size_t>(best_u + 1)];
                    has_r = unproject(best_u + 1, best_v, id_r, params, rx, ry, rz);
                }
                if (best_v + 1 < params.height) {
                    const float id_b = inverse_depth[
                        static_cast<std::size_t>(best_v + 1) * static_cast<std::size_t>(params.width) +
                        static_cast<std::size_t>(best_u)];
                    has_b = unproject(best_u, best_v + 1, id_b, params, bx, by, bz);
                }
                ev.normal_x = ev.normal_y = ev.normal_z = 0.0F;
                if (has_r && has_b) {
                    const float tx = rx - lx, ty = ry - ly, tz = rz - lz;
                    const float dx = bx - lx, dy = by - ly, dz = bz - lz;
                    float nx = ty*dz - tz*dy;
                    float ny = tz*dx - tx*dz;
                    float nz = tx*dy - ty*dx;
                    const float len = std::sqrt(nx*nx + ny*ny + nz*nz);
                    if (len > 1e-6F) {
                        nx /= len; ny /= len; nz /= len;
                        // Orient toward camera (dot with vector from point to origin).
                        if (nx*(params.origin_x - lx) +
                            ny*(params.origin_y - ly) +
                            nz*(params.origin_z - lz) < 0.0F) {
                            nx = -nx; ny = -ny; nz = -nz;
                        }
                        ev.normal_x = nx; ev.normal_y = ny; ev.normal_z = nz;
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Surface patch detection — RANSAC plane fitting
// ---------------------------------------------------------------------------

void fit_surface_patches_device(
    const DeviceObstacleEvidence* evidence,
    std::uint32_t                 count,
    const ProjectionParams&       params,
    DeviceObstacleEvidence*       patches_out,
    std::uint32_t&                patches_count_out) {

    patches_count_out = 0;
    if (count < 3U) return;

    const float inlier_threshold = params.voxel_size_m;        // 1 voxel tolerance
    const auto  min_inliers      = std::max(3U, count / 10U);  // at least 10% of points
    const int   max_patches      = 3;
    const int   ransac_iters     = 100;

    std::mt19937 rng{42U};

    // Work on a copy so we can remove inliers for subsequent patches
    std::vector<std::array<float,3>> pts(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        pts[i] = {evidence[i].center_x, evidence[i].center_y, evidence[i].center_z};
    }

    for (int patch = 0; patch < max_patches && pts.size() >= 3; ++patch) {
        const auto n = static_cast<std::uint32_t>(pts.size());
        std::uniform_int_distribution<std::uint32_t> dist(0U, n - 1U);

        Plane best{};
        std::uint32_t best_count = 0U;
        bool found = false;

        for (int iter = 0; iter < ransac_iters; ++iter) {
            const std::uint32_t ia = dist(rng);
            std::uint32_t ib = dist(rng);
            std::uint32_t ic = dist(rng);
            while (ib == ia)           ib = dist(rng);
            while (ic == ia || ic == ib) ic = dist(rng);

            Plane p{};
            if (!fit_plane_3pts(pts[ia].data(), pts[ib].data(), pts[ic].data(), p)) continue;

            std::uint32_t cnt = 0U;
            for (const auto& pt : pts) {
                if (plane_dist(p, pt[0], pt[1], pt[2]) < inlier_threshold) ++cnt;
            }
            if (cnt > best_count) { best = p; best_count = cnt; found = true; }
        }

        if (!found || best_count < min_inliers) break;

        // Refine with least-squares over inliers
        std::vector<std::array<float,3>> inliers;
        inliers.reserve(best_count);
        for (const auto& pt : pts) {
            if (plane_dist(best, pt[0], pt[1], pt[2]) < inlier_threshold) {
                inliers.push_back(pt);
            }
        }
        Plane refined{};
        if (fit_plane_ls(inliers, refined)) best = refined;

        // Ensure normal points toward origin (toward camera)
        const float dot = best.nx * (params.origin_x - inliers[0][0])
                        + best.ny * (params.origin_y - inliers[0][1])
                        + best.nz * (params.origin_z - inliers[0][2]);
        if (dot < 0.0F) { best.nx=-best.nx; best.ny=-best.ny; best.nz=-best.nz; }

        // Centroid of inliers
        float cx=0.0F, cy=0.0F, cz=0.0F;
        for (const auto& pt : inliers) { cx+=pt[0]; cy+=pt[1]; cz+=pt[2]; }
        const float fn = static_cast<float>(inliers.size());
        cx/=fn; cy/=fn; cz/=fn;

        // Emit SurfacePatch evidence (voxel_size_m as size placeholder)
        DeviceObstacleEvidence& ev = patches_out[patches_count_out++];
        ev.center_x = cx;  ev.center_y = cy;  ev.center_z = cz;
        ev.size_x = ev.size_y = ev.size_z = params.voxel_size_m;
        ev.normal_x = best.nx;  ev.normal_y = best.ny;  ev.normal_z = best.nz;
        ev.confidence             = static_cast<float>(inliers.size()) / static_cast<float>(n);
        ev.range_m                = std::abs(best.d);  // plane distance from origin
        ev.state                  = 2U;  // Occupied
        ev.shape                  = 3U;  // SurfacePatch
        ev.is_thin_structure_hint = 0U;
        ev.is_surface_hint        = 1U;

        // Remove inliers before next patch search
        pts.erase(std::remove_if(pts.begin(), pts.end(),
            [&best, inlier_threshold](const std::array<float,3>& pt) {
                return plane_dist(best, pt[0], pt[1], pt[2]) < inlier_threshold;
            }), pts.end());
    }
}

// ---------------------------------------------------------------------------
// Thin structure detection — local depth contrast + BFS connected components
// ---------------------------------------------------------------------------

void detect_thin_structures_device(
    const float*            inverse_depth,
    const ProjectionParams& params,
    DeviceObstacleEvidence* thin_out,
    std::uint32_t&          thin_count_out) {

    thin_count_out = 0;
    const int W = params.width;
    const int H = params.height;
    if (W < 5 || H < 5) return;

    // Threshold: pixel is a thin-structure candidate if its depth is
    // significantly closer than its 5×5 neighbourhood background.
    const float contrast_fraction = 0.7F;  // must be < 70% of local max depth
    const float min_abs_contrast  = 1.0F;  // and at least 1 m closer

    // Build candidate mask
    const std::size_t npix = static_cast<std::size_t>(W) * static_cast<std::size_t>(H);
    std::vector<bool> mask(npix, false);

    for (int v = 2; v < H-2; ++v) {
        for (int u = 2; u < W-2; ++u) {
            const std::size_t idx = static_cast<std::size_t>(v)*static_cast<std::size_t>(W) +
                                    static_cast<std::size_t>(u);
            const float dr = inverse_depth[idx];
            if (!std::isfinite(dr) || dr <= 0.0F) continue;
            const float dm = params.scale / dr;
            if (dm < params.min_depth_m || dm > params.max_depth_m) continue;

            const float lmax = local_max_depth(inverse_depth, W, H, u, v, params);
            if (lmax < dm + min_abs_contrast) continue;  // not significantly closer
            if (dm >= lmax * contrast_fraction) continue; // not close enough relative to bg

            mask[idx] = true;
        }
    }

    // BFS connected components on candidate pixels
    const int  min_length  = 4;   // minimum number of pixels in component
    const float min_aspect = 2.5F; // min(width,height) / max(width,height) inverted
    const int  max_thin    = 8;   // max thin structures to emit per frame

    std::vector<bool> visited(npix, false);

    for (int v = 0; v < H && static_cast<int>(thin_count_out) < max_thin; ++v) {
        for (int u = 0; u < W && static_cast<int>(thin_count_out) < max_thin; ++u) {
            const std::size_t idx = static_cast<std::size_t>(v)*static_cast<std::size_t>(W) +
                                    static_cast<std::size_t>(u);
            if (!mask[idx] || visited[idx]) continue;

            const auto comp = bfs_component(mask, visited, W, H, u, v);
            if (static_cast<int>(comp.size()) < min_length) continue;

            // Bounding box
            int u_min=W, u_max=0, v_min=H, v_max=0;
            for (const auto& [pu, pv] : comp) {
                u_min=std::min(u_min,pu); u_max=std::max(u_max,pu);
                v_min=std::min(v_min,pv); v_max=std::max(v_max,pv);
            }
            const int bb_w = u_max - u_min + 1;
            const int bb_h = v_max - v_min + 1;
            const float aspect = static_cast<float>(std::max(bb_w, bb_h)) /
                                 static_cast<float>(std::max(1, std::min(bb_w, bb_h)));
            if (aspect < min_aspect) continue;

            // Pick the two endpoints along the long axis and back-project them.
            // Also compute centroid depth from component pixels.
            int ua, va, ub, vb;
            if (bb_h >= bb_w) {
                // Vertical: top and bottom, at horizontal centre
                ua = (u_min + u_max) / 2;  va = v_min;
                ub = ua;                    vb = v_max;
            } else {
                // Horizontal: left and right, at vertical centre
                va = (v_min + v_max) / 2;  ua = u_min;
                vb = va;                    ub = u_max;
            }

            float ax, ay, az, bx, by, bz;
            // If a pixel itself is invalid try neighbours for depth
            auto best_dr = [&](int pu, int pv) -> float {
                const std::size_t i0 = static_cast<std::size_t>(pv)*static_cast<std::size_t>(W) +
                                       static_cast<std::size_t>(pu);
                float dr0 = inverse_depth[i0];
                if (!std::isfinite(dr0) || dr0 <= 0.0F) {
                    // Try horizontal neighbours
                    for (int du = -1; du <= 1; ++du) {
                        const int nu = pu + du;
                        if (nu < 0 || nu >= W) continue;
                        const std::size_t ni = static_cast<std::size_t>(pv)*static_cast<std::size_t>(W) +
                                               static_cast<std::size_t>(nu);
                        if (mask[ni]) { dr0 = inverse_depth[ni]; break; }
                    }
                }
                return dr0;
            };

            if (!unproject(ua, va, best_dr(ua, va), params, ax, ay, az)) continue;
            if (!unproject(ub, vb, best_dr(ub, vb), params, bx, by, bz)) continue;

            // Encode: center = midpoint, size = half-displacement (b-a)/2
            const float mx = (ax + bx) * 0.5F;
            const float my = (ay + by) * 0.5F;
            const float mz = (az + bz) * 0.5F;
            const float hx = (bx - ax) * 0.5F;
            const float hy = (by - ay) * 0.5F;
            const float hz = (bz - az) * 0.5F;
            const float length = 2.0F * std::sqrt(hx*hx + hy*hy + hz*hz);
            if (length < 0.01F) continue;

            // Unit direction
            const float inv_len = 1.0F / (length * 0.5F);
            DeviceObstacleEvidence& ev = thin_out[thin_count_out++];
            ev.center_x = mx;  ev.center_y = my;  ev.center_z = mz;
            ev.size_x   = hx;  ev.size_y   = hy;  ev.size_z   = hz;
            ev.normal_x = hx * inv_len;  // unit direction (for display)
            ev.normal_y = hy * inv_len;
            ev.normal_z = hz * inv_len;
            ev.confidence             = std::min(1.0F, aspect / 10.0F);
            ev.range_m                = length;
            ev.state                  = 3U;  // ThinStructureRisk
            ev.shape                  = 4U;  // LineSegment
            ev.is_thin_structure_hint = 1U;
            ev.is_surface_hint        = 0U;
        }
    }
}

// ---------------------------------------------------------------------------
// inflate: DeviceObstacleEvidence[] → ObstacleEvidence[]
// ---------------------------------------------------------------------------

std::vector<ObstacleEvidence> inflate(
    const DeviceObstacleEvidence* evidence,
    std::uint32_t                 count,
    const std::string&            sensor_name,
    const std::string&            source_provider,
    const MapFrameId&             map_frame_id,
    TimePoint                     timestamp) {

    std::vector<ObstacleEvidence> result;
    result.reserve(count);

    for (std::uint32_t i = 0; i < count; ++i) {
        const DeviceObstacleEvidence& src = evidence[i];
        ObstacleEvidence ev;

        ev.timestamp       = timestamp;
        ev.sensor_name     = sensor_name;
        ev.source_provider = source_provider;
        ev.source_kind     = OccupancySourceKind::VisualObstacleDetector;
        ev.map_frame_id    = map_frame_id;

        ev.state = static_cast<ObstacleEvidenceState>(src.state);
        ev.shape = static_cast<ObstacleEvidenceShape>(src.shape);

        ev.center_local = Vec3{
            static_cast<double>(src.center_x),
            static_cast<double>(src.center_y),
            static_cast<double>(src.center_z)};

        if (src.shape == 4U) {
            // LineSegment: center=midpoint, size=half-displacement
            // Reconstruct endpoints: a = center - size, b = center + size
            ev.endpoint_a_local = Vec3{
                static_cast<double>(src.center_x - src.size_x),
                static_cast<double>(src.center_y - src.size_y),
                static_cast<double>(src.center_z - src.size_z)};
            ev.endpoint_b_local = Vec3{
                static_cast<double>(src.center_x + src.size_x),
                static_cast<double>(src.center_y + src.size_y),
                static_cast<double>(src.center_z + src.size_z)};
            ev.size_m = Vec3{
                std::abs(static_cast<double>(src.size_x)) * 2.0,
                std::abs(static_cast<double>(src.size_y)) * 2.0,
                std::abs(static_cast<double>(src.size_z)) * 2.0};
        } else {
            ev.size_m = Vec3{
                static_cast<double>(src.size_x),
                static_cast<double>(src.size_y),
                static_cast<double>(src.size_z)};
        }

        const float nsq = src.normal_x*src.normal_x
                        + src.normal_y*src.normal_y
                        + src.normal_z*src.normal_z;
        ev.has_surface_normal = nsq > 0.5F;
        if (ev.has_surface_normal) {
            ev.surface_normal_local = Vec3{
                static_cast<double>(src.normal_x),
                static_cast<double>(src.normal_y),
                static_cast<double>(src.normal_z)};
            ev.normal_confidence = src.confidence;
        }

        ev.confidence            = src.confidence;
        ev.occupancy_probability = src.confidence;
        ev.range_m               = src.range_m;
        ev.is_thin_structure_hint = src.is_thin_structure_hint != 0U;
        ev.is_surface_hint        = src.is_surface_hint != 0U;
        ev.inside_sensing_volume  = true;

        result.push_back(std::move(ev));
    }

    return result;
}

}  // namespace dedalus
