#include "dedalus/sensors/visual_ego_state_provider.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <numeric>
#include <optional>
#include <random>
#include <utility>

namespace dedalus {
namespace {

// ── Greyscale conversion ─────────────────────────────────────────────────────

void rgb_to_gray(const std::uint8_t* rgb, int width, int height,
                 std::vector<std::uint8_t>& out) {
    out.resize(static_cast<std::size_t>(width * height));
    for (int i = 0; i < width * height; ++i) {
        const auto r = static_cast<int>(rgb[i * 3 + 0]);
        const auto g = static_cast<int>(rgb[i * 3 + 1]);
        const auto b = static_cast<int>(rgb[i * 3 + 2]);
        out[static_cast<std::size_t>(i)] =
            static_cast<std::uint8_t>((77 * r + 150 * g + 29 * b) >> 8);
    }
}

// ── Bilinear interpolation ───────────────────────────────────────────────────

float interp(const std::uint8_t* img, int width, int height, float x, float y) {
    const int xi = static_cast<int>(x);
    const int yi = static_cast<int>(y);
    if (xi < 0 || xi >= width - 1 || yi < 0 || yi >= height - 1) return 128.0F;
    const float fx = x - static_cast<float>(xi);
    const float fy = y - static_cast<float>(yi);
    const auto* row0 = img + yi * width;
    const auto* row1 = row0 + width;
    return (1.0F - fx) * (1.0F - fy) * static_cast<float>(row0[xi])
         + fx          * (1.0F - fy) * static_cast<float>(row0[xi + 1])
         + (1.0F - fx) * fy          * static_cast<float>(row1[xi])
         + fx          * fy          * static_cast<float>(row1[xi + 1]);
}

// ── FAST-9 corner detector ───────────────────────────────────────────────────
// Bresenham circle of radius 3: 16 pixel offsets (row, col).
static constexpr int kFastCircle[16][2] = {
    { 0,-3},{  1,-3},{  2,-2},{  3,-1},
    { 3, 0},{  3, 1},{  2, 2},{  1, 3},
    { 0, 3},{ -1, 3},{ -2, 2},{ -3, 1},
    {-3, 0},{ -3,-1},{ -2,-2},{ -1,-3}
};

// Score: sum of absolute differences from threshold contiguous arc for the
// winning direction; used to NMS by score.
static int fast9_score(const std::uint8_t* img, int width,
                       int x, int y, int threshold) {
    const int centre = img[y * width + x];
    const int hi = centre + threshold;
    const int lo = centre - threshold;
    // Find longest arc of pixels all above hi or all below lo
    // (simplified: sum of excess for the dominant direction)
    int score_hi = 0, score_lo = 0;
    for (const auto& d : kFastCircle) {
        const int v = img[(y + d[0]) * width + (x + d[1])];
        if (v > hi)  score_hi += v - hi;
        if (v < lo)  score_lo += lo - v;
    }
    return std::max(score_hi, score_lo);
}

static bool is_fast9(const std::uint8_t* img, int width, int x, int y, int t) {
    const int c = img[y * width + x];
    const int hi = c + t, lo = c - t;
    // Quick reject: pixels 0, 4, 8, 12 must have at least one extreme
    const int p0  = img[(y + kFastCircle[0][0])  * width + x + kFastCircle[0][1]];
    const int p4  = img[(y + kFastCircle[4][0])  * width + x + kFastCircle[4][1]];
    const int p8  = img[(y + kFastCircle[8][0])  * width + x + kFastCircle[8][1]];
    const int p12 = img[(y + kFastCircle[12][0]) * width + x + kFastCircle[12][1]];
    const bool any_bright = (p0>hi)||(p4>hi)||(p8>hi)||(p12>hi);
    const bool any_dark   = (p0<lo)||(p4<lo)||(p8<lo)||(p12<lo);
    if (!any_bright && !any_dark) return false;

    // Check for 9 consecutive bright or dark pixels
    for (int start = 0; start < 16; ++start) {
        bool bright_run = true, dark_run = true;
        for (int k = 0; k < 9; ++k) {
            const int idx = (start + k) % 16;
            const int v = img[(y + kFastCircle[idx][0]) * width + (x + kFastCircle[idx][1])];
            if (v <= hi) bright_run = false;
            if (v >= lo) dark_run   = false;
            if (!bright_run && !dark_run) break;
        }
        if (bright_run || dark_run) return true;
    }
    return false;
}

void detect_fast9(const std::uint8_t* gray, int width, int height,
                  int threshold, int max_features,
                  std::vector<TrackedFeature>& features) {
    const int border = 4;
    std::vector<std::pair<int, std::pair<int,int>>> scored;

    for (int y = border; y < height - border; ++y) {
        for (int x = border; x < width - border; ++x) {
            if (is_fast9(gray, width, x, y, threshold)) {
                scored.emplace_back(fast9_score(gray, width, x, y, threshold),
                                    std::make_pair(x, y));
            }
        }
    }

    // Sort descending by score
    std::sort(scored.begin(), scored.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    // Non-max suppression: skip corners within 8 pixels of an already-selected one
    constexpr int kNmsDist = 8;
    std::vector<std::pair<int,int>> accepted;
    for (const auto& [sc, pt] : scored) {
        bool blocked = false;
        for (const auto& a : accepted) {
            if (std::abs(pt.first - a.first) < kNmsDist &&
                std::abs(pt.second - a.second) < kNmsDist) {
                blocked = true; break;
            }
        }
        if (!blocked) {
            accepted.push_back(pt);
            if (static_cast<int>(accepted.size()) >= max_features) break;
        }
    }

    for (const auto& [x, y] : accepted) {
        TrackedFeature f;
        f.x = f.x_prev = static_cast<float>(x);
        f.y = f.y_prev = static_cast<float>(y);
        f.valid = true; f.age = 0;
        features.push_back(f);
    }
}

// ── Lucas-Kanade optical flow (single scale) ─────────────────────────────────

void track_lk(const std::uint8_t* prev, const std::uint8_t* curr,
              int width, int height, int patch_radius, int iterations,
              std::vector<TrackedFeature>& features) {
    const int border = patch_radius + 2;

    for (auto& f : features) {
        if (!f.valid) continue;
        const float px = f.x, py = f.y;

        // Precompute spatial gradients and patch from prev frame
        double Axx = 0, Axy = 0, Ayy = 0;
        struct PxInfo { float Ix, Iy, Iprev; };
        std::vector<PxInfo> patch;
        patch.reserve(static_cast<std::size_t>((2*patch_radius+1)*(2*patch_radius+1)));

        for (int dy = -patch_radius; dy <= patch_radius; ++dy) {
            for (int dx = -patch_radius; dx <= patch_radius; ++dx) {
                const float cx = px + static_cast<float>(dx);
                const float cy = py + static_cast<float>(dy);
                if (cx < 1 || cx >= width-1 || cy < 1 || cy >= height-1) {
                    patch.push_back({0.0F, 0.0F, 0.0F});
                    continue;
                }
                const float ix = (interp(prev,width,height,cx+1,cy)
                                - interp(prev,width,height,cx-1,cy)) * 0.5F;
                const float iy = (interp(prev,width,height,cx,cy+1)
                                - interp(prev,width,height,cx,cy-1)) * 0.5F;
                const float iv = interp(prev, width, height, cx, cy);
                patch.push_back({ix, iy, iv});
                Axx += ix * ix; Axy += ix * iy; Ayy += iy * iy;
            }
        }

        const double det = Axx * Ayy - Axy * Axy;
        if (std::abs(det) < 1e-6) { f.valid = false; continue; }
        const double inv_det = 1.0 / det;

        float ox = 0.0F, oy = 0.0F;  // accumulated displacement
        for (int iter = 0; iter < iterations; ++iter) {
            double bx = 0, by = 0;
            std::size_t pi = 0;
            for (int dy = -patch_radius; dy <= patch_radius; ++dy) {
                for (int dx = -patch_radius; dx <= patch_radius; ++dx, ++pi) {
                    const float cx = px + static_cast<float>(dx) + ox;
                    const float cy = py + static_cast<float>(dy) + oy;
                    const float it = interp(curr,width,height,cx,cy) - patch[pi].Iprev;
                    bx -= patch[pi].Ix * it;
                    by -= patch[pi].Iy * it;
                }
            }
            ox += static_cast<float>(inv_det * (Ayy * bx - Axy * by));
            oy += static_cast<float>(inv_det * (Axx * by - Axy * bx));
        }

        const float nx = px + ox, ny = py + oy;
        if (nx < border || nx >= width - border ||
            ny < border || ny >= height - border ||
            ox*ox + oy*oy > 900.0F) {  // > 30 px/frame = unreliable
            f.valid = false; continue;
        }
        f.x_prev = px; f.y_prev = py;
        f.x = nx;       f.y = ny;
        f.age++;
    }
}

// ── Focus of Expansion RANSAC ─────────────────────────────────────────────────
// Finds the translation direction (Focus of Expansion) and a rotation estimate
// from the optical flow field, using RANSAC for robustness.
//
// p_norm: normalized image coords of previous features ((u-cx)/fx, (v-cy)/fy)
// q_norm: normalized image coords of current features
// Returns: (success, tx_norm, ty_norm, rotation_omega[3], inlier_count)

struct FoeResult {
    bool   valid{false};
    double tx{0}, ty{0};          // FoE in normalized coords (tz = 1)
    double omega[3]{0,0,0};       // angular velocity (radians/frame)
    int    inliers{0};
};

FoeResult foe_ransac(const std::vector<std::pair<double,double>>& p_norm,
                     const std::vector<std::pair<double,double>>& q_norm,
                     int iterations, double threshold_norm, int min_inliers,
                     std::mt19937& rng) {
    const int N = static_cast<int>(p_norm.size());
    if (N < 8) return {};

    FoeResult best;
    std::uniform_int_distribution<int> dist(0, N - 1);

    for (int it = 0; it < iterations; ++it) {
        // Sample 2 correspondences to solve 2×2 FoE system exactly
        const int a = dist(rng), b = dist(rng);
        if (a == b) continue;

        // Constraint: for feature i with flow f_i = q_i - p_i:
        //   f_i.y * (e.x - p_i.x) = f_i.x * (e.y - p_i.y)
        // → f_i.y * e.x - f_i.x * e.y = f_i.y*p_i.x - f_i.x*p_i.y
        double A[4], rhs[2];
        for (int k = 0; k < 2; ++k) {
            const int i = (k == 0) ? a : b;
            const double fx = q_norm[i].first  - p_norm[i].first;
            const double fy = q_norm[i].second - p_norm[i].second;
            A[k*2+0] = fy; A[k*2+1] = -fx;
            rhs[k]   = fy * p_norm[i].first - fx * p_norm[i].second;
        }
        const double det = A[0]*A[3] - A[1]*A[2];
        if (std::abs(det) < 1e-10) continue;

        const double ex = (rhs[0]*A[3] - rhs[1]*A[1]) / det;
        const double ey = (rhs[1]*A[0] - rhs[0]*A[2]) / det;

        // Count inliers: perpendicular distance from flow line through FoE
        int inliers = 0;
        for (int i = 0; i < N; ++i) {
            const double fx = q_norm[i].first  - p_norm[i].first;
            const double fy = q_norm[i].second - p_norm[i].second;
            const double fl = std::sqrt(fx*fx + fy*fy);
            if (fl < 1e-6) { ++inliers; continue; }  // static feature = inlier
            // Perpendicular dist from flow line: |f × (p - e)| / |f|
            const double cross = fx * (p_norm[i].second - ey)
                               - fy * (p_norm[i].first  - ex);
            if (std::abs(cross) / fl < threshold_norm) ++inliers;
        }

        if (inliers > best.inliers) {
            best.inliers = inliers;
            best.tx = ex; best.ty = ey;
            best.valid = (inliers >= min_inliers);
        }
    }

    if (!best.valid) return best;

    // Refit FoE on all inliers (normal equations 2×2)
    double AtA[4] = {}, Atb[2] = {};
    for (int i = 0; i < N; ++i) {
        const double fx = q_norm[i].first  - p_norm[i].first;
        const double fy = q_norm[i].second - p_norm[i].second;
        const double fl = std::sqrt(fx*fx + fy*fy);
        if (fl >= 1e-6) {
            const double cross = fx * (p_norm[i].second - best.ty)
                               - fy * (p_norm[i].first  - best.tx);
            if (std::abs(cross) / fl >= threshold_norm) continue;
        }
        AtA[0] += fy*fy; AtA[1] -= fy*fx;
        AtA[2] -= fx*fy; AtA[3] += fx*fx;
        Atb[0] += fy * (fy*p_norm[i].first - fx*p_norm[i].second);
        Atb[1] -= fx * (fy*p_norm[i].first - fx*p_norm[i].second);
    }
    const double det2 = AtA[0]*AtA[3] - AtA[1]*AtA[2];
    if (std::abs(det2) > 1e-12) {
        best.tx = (Atb[0]*AtA[3] - Atb[1]*AtA[1]) / det2;
        best.ty = (Atb[1]*AtA[0] - Atb[0]*AtA[2]) / det2;
    }

    // Estimate angular velocity from rotational residual flow.
    // rot_flow_i ≈ [omega_z * p.x - omega_x, omega_z * p.y - omega_y]
    // (for camera z-axis rotation and xy tilts)
    // Build 3-column normal equations for [omega_x, omega_y, omega_z].
    //   f_i.x - t_flow_i.x ≈ -omega_y + omega_z * p_i.y
    //   f_i.y - t_flow_i.y ≈  omega_x - omega_z * p_i.x
    // (Camera-frame axes: x=right, y=down, z=forward in NED camera)
    double Ro[9] = {}, Rs[3] = {};
    const double foe_norm = std::sqrt(best.tx*best.tx + best.ty*best.ty + 1.0);
    const double txu = best.tx/foe_norm, tyu = best.ty/foe_norm;
    for (int i = 0; i < N; ++i) {
        const double fx = q_norm[i].first  - p_norm[i].first;
        const double fy = q_norm[i].second - p_norm[i].second;
        const double fl = std::sqrt(fx*fx + fy*fy);
        if (fl < 1e-6) continue;
        const double cross = fx * (p_norm[i].second - best.ty)
                           - fy * (p_norm[i].first  - best.tx);
        if (std::abs(cross) / fl >= threshold_norm) continue;
        // Translational flow component (direction of t = (tx,ty,1)/norm)
        const double depth_proxy = 1.0;  // normalized; absorbed into omega units
        const double tfx = (txu - p_norm[i].first)  / depth_proxy;
        const double tfy = (tyu - p_norm[i].second) / depth_proxy;
        const double rfx = fx - tfx * fl;  // rotational residual (approx)
        const double rfy = fy - tfy * fl;
        // Row: [−1, 0, p.y] for rfx; [0, 1, −p.x] for rfy
        const double px = p_norm[i].first, py = p_norm[i].second;
        Ro[0] += 1.0;             Ro[2] += -py;      Rs[0] += -rfx;
        Ro[4] += 1.0;             Ro[5] +=  px;      Rs[1] +=  rfy;
        Ro[6] += -py; Ro[7] += px; Ro[8] += px*px+py*py; // ω_z row accumulates
    }
    // Solve 3×3 normal equations Ro * omega = Rs via Cramer (trivial for 3×3)
    // det(Ro)
    const double d = Ro[0]*(Ro[4]*Ro[8]-Ro[5]*Ro[7])
                   - Ro[1]*(Ro[3]*Ro[8]-Ro[5]*Ro[6])
                   + Ro[2]*(Ro[3]*Ro[7]-Ro[4]*Ro[6]);
    if (std::abs(d) > 1e-10) {
        auto sub3 = [&](int r, int s) {
            // 3x3 det after replacing column s with Rs
            double M[9]; std::memcpy(M, Ro, sizeof(M));
            M[0*3+s] = Rs[0]; M[1*3+s] = Rs[1]; M[2*3+s] = Rs[2];
            return M[0]*(M[4]*M[8]-M[5]*M[7]) - M[1]*(M[3]*M[8]-M[5]*M[6])
                  + M[2]*(M[3]*M[7]-M[4]*M[6]);
            (void)r; (void)s;
        };
        best.omega[0] = sub3(0,0) / d;
        best.omega[1] = sub3(0,1) / d;
        best.omega[2] = sub3(0,2) / d;
    }

    return best;
}

// ── 3×3 matrix helpers ───────────────────────────────────────────────────────

inline void mat3_mul(const double A[9], const double B[9], double C[9]) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            C[i*3+j] = 0;
            for (int k = 0; k < 3; ++k) C[i*3+j] += A[i*3+k] * B[k*3+j];
        }
}

// Build rotation matrix from axis-angle omega vector (small-angle safe).
inline void rot_from_omega(const double omega[3], double R[9]) {
    const double ax = omega[0], ay = omega[1], az = omega[2];
    const double theta2 = ax*ax + ay*ay + az*az;
    if (theta2 < 1e-20) {
        // identity
        for (int i = 0; i < 9; ++i) R[i] = (i%4==0) ? 1.0 : 0.0;
        return;
    }
    const double theta = std::sqrt(theta2);
    const double s = std::sin(theta) / theta;
    const double c1 = (1.0 - std::cos(theta)) / theta2;
    R[0] = 1.0 - c1*(ay*ay+az*az); R[1] = -s*az + c1*ax*ay;  R[2] =  s*ay + c1*ax*az;
    R[3] =  s*az + c1*ax*ay;       R[4] = 1.0 - c1*(ax*ax+az*az); R[5] = -s*ax + c1*ay*az;
    R[6] = -s*ay + c1*ax*az;       R[7] =  s*ax + c1*ay*az;  R[8] = 1.0 - c1*(ax*ax+ay*ay);
}

}  // namespace

// ── VisualEgoStateProvider ────────────────────────────────────────────────────

VisualEgoStateProvider::VisualEgoStateProvider(VisualOdometryConfig config)
    : config_(std::move(config)) {
    state_.scale.scale      = config_.initial_scale_m;
    state_.scale.confidence = 0.1F;
}

// VL1: one frame of visual odometry.  Updates state_.  Returns inlier fraction.
float VisualEgoStateProvider::step_vl1(const std::vector<std::uint8_t>& gray,
                                        int width, int height,
                                        const CameraIntrinsics& K,
                                        double dt_s) {
    // ── Track existing features ──────────────────────────────────────────────
    if (!state_.prev_gray.empty()) {
        track_lk(state_.prev_gray.data(), gray.data(), width, height,
                 config_.lk_patch_radius, config_.lk_iterations,
                 state_.features);
    }

    // Remove invalid features
    state_.features.erase(
        std::remove_if(state_.features.begin(), state_.features.end(),
                       [](const TrackedFeature& f) { return !f.valid; }),
        state_.features.end());

    // Re-detect if below minimum
    if (static_cast<int>(state_.features.size()) < config_.min_features) {
        detect_fast9(gray.data(), width, height,
                     config_.fast_threshold, config_.max_features,
                     state_.features);
    }

    if (state_.features.size() < 8) return 0.0F;

    // ── Build normalized correspondences ─────────────────────────────────────
    std::vector<std::pair<double,double>> p_norm, q_norm;
    for (const auto& f : state_.features) {
        if (!f.valid || f.age < 1) continue;
        p_norm.emplace_back((f.x_prev - K.cx) / K.fx,
                            (f.y_prev - K.cy) / K.fy);
        q_norm.emplace_back((f.x - K.cx) / K.fx,
                            (f.y - K.cy) / K.fy);
    }
    if (static_cast<int>(p_norm.size()) < config_.foe_min_inliers) return 0.0F;

    // ── Focus of Expansion RANSAC → translation direction + omega ────────────
    static thread_local std::mt19937 rng{42};
    const double threshold_norm = config_.foe_ransac_threshold / K.fx;
    const FoeResult foe = foe_ransac(p_norm, q_norm,
                                      config_.foe_ransac_iterations,
                                      threshold_norm,
                                      config_.foe_min_inliers,
                                      rng);
    if (!foe.valid) return 0.0F;

    const float inlier_frac = static_cast<float>(foe.inliers) /
                              static_cast<float>(p_norm.size());

    // ── Metric scale ──────────────────────────────────────────────────────────
    // state_.scale.scale = metres per unit direction norm.
    // (Updated by VL2; for VL1 stays at initial_scale_m or velocity-derived.)

    // ── Rotation integration (Rodrigues) ─────────────────────────────────────
    double dR[9];
    rot_from_omega(foe.omega, dR);
    // Apply: R_new = dR * R_old  (incremental rotation in camera frame)
    double R_tmp[9];
    mat3_mul(dR, state_.rotation.data(), R_tmp);
    std::copy(R_tmp, R_tmp + 9, state_.rotation.begin());

    // ── Translation integration ───────────────────────────────────────────────
    // Translation direction in camera frame: (tx, ty, 1) normalized.
    const double tnorm = std::sqrt(foe.tx*foe.tx + foe.ty*foe.ty + 1.0);
    const double tx = foe.tx / tnorm;
    const double ty = foe.ty / tnorm;
    const double tz = 1.0   / tnorm;
    const double scale = static_cast<double>(state_.scale.scale);

    // Rotate camera-frame translation to map frame and accumulate.
    const auto& R = state_.rotation;
    state_.position.x += scale * (R[0]*tx + R[1]*ty + R[2]*tz);
    state_.position.y += scale * (R[3]*tx + R[4]*ty + R[5]*tz);
    state_.position.z += scale * (R[6]*tx + R[7]*ty + R[8]*tz);

    // ── VL3: uncertainty propagation ─────────────────────────────────────────
    const double omega_mag = std::sqrt(foe.omega[0]*foe.omega[0]
                                     + foe.omega[1]*foe.omega[1]
                                     + foe.omega[2]*foe.omega[2]);
    state_.cumulative_drift_m += (1.0 - inlier_frac) * scale;
    state_.translation_sigma  += scale * config_.translation_noise_per_m;
    state_.rotation_sigma     += omega_mag * config_.rotation_noise_per_rad;

    return inlier_frac;
    (void)dt_s;
}

// VL2: update scale from depth frame vs nearest L2 voxels.
void VisualEgoStateProvider::update_scale_vl2(
    const std::vector<std::uint8_t>& gray,
    int width, int height,
    const std::vector<float>& depth_m,
    int depth_w, int depth_h,
    const CameraIntrinsics& K) {
    if (!l2_map_ || l2_map_->cell_count() == 0U) return;

    // For each tracked feature, look up metric depth from depth_m and compare
    // to the L2 voxel distance along that ray.
    double scale_sum = 0.0;
    int    scale_cnt = 0;

    for (const auto& f : state_.features) {
        if (!f.valid || f.age < 1) continue;

        // Sample depth at feature pixel (nearest-neighbour into depth frame)
        const int dxi = static_cast<int>(f.x * depth_w / width);
        const int dyi = static_cast<int>(f.y * depth_h / height);
        if (dxi < 0 || dxi >= depth_w || dyi < 0 || dyi >= depth_h) continue;
        const float dm = depth_m[static_cast<std::size_t>(dyi * depth_w + dxi)];
        if (dm <= 0.05F || dm > 80.0F) continue;

        // Ray direction in map frame for this feature
        const double ndx = (f.x - K.cx) / K.fx;
        const double ndy = (f.y - K.cy) / K.fy;
        const double rn  = std::sqrt(ndx*ndx + ndy*ndy + 1.0);
        const auto& R    = state_.rotation;
        const Vec3 dir{
            (R[0]*ndx + R[1]*ndy + R[2]) / rn,
            (R[3]*ndx + R[4]*ndy + R[5]) / rn,
            (R[6]*ndx + R[7]*ndy + R[8]) / rn};

        const auto hit = l2_map_->ray_cast(state_.position, dir, 50.0);
        if (!hit) continue;

        const double dx = hit->x - state_.position.x;
        const double dy = hit->y - state_.position.y;
        const double dz = hit->z - state_.position.z;
        const double l2_dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (l2_dist < 0.1) continue;

        // depth_m = l2_dist → scale should be: scale * ray_len ≈ l2_dist
        // Current model: translation per frame ≈ scale (constant).
        // Scale for depth: depth_relative used by the VD stack, not f here.
        // Here we just re-calibrate state_.scale.scale as:
        //   new_scale = l2_dist / dm * (current scale magnification factor)
        // This is a ratio between the L2 map's metric and the current scale.
        const double implied_scale = l2_dist / static_cast<double>(dm)
                                   * static_cast<double>(state_.scale.scale);
        scale_sum += implied_scale;
        ++scale_cnt;
    }

    if (scale_cnt < config_.scale_update_min_samples) return;

    const float new_scale = static_cast<float>(scale_sum / scale_cnt);
    const float alpha     = config_.scale_update_alpha;
    state_.scale.scale      = (1.0F - alpha) * state_.scale.scale + alpha * new_scale;
    state_.scale.confidence = std::min(1.0F, state_.scale.confidence + 0.05F);
    (void)gray; (void)width; (void)height;
}

// VL2: ICP-lite re-localization.  Finds the median offset between VO-projected
// feature depths and the nearest L2 voxels, then corrects state_.position.
void VisualEgoStateProvider::relocalize_vl2() {
    if (!l2_map_ || l2_map_->cell_count() == 0U) return;

    // Query L2 voxels near current position
    const double r = static_cast<double>(config_.relocalization_search_radius_m);
    const Vec3 bmin{state_.position.x - r,
                    state_.position.y - r,
                    state_.position.z - r};
    const Vec3 bmax{state_.position.x + r,
                    state_.position.y + r,
                    state_.position.z + r};
    const auto voxels = l2_map_->query_occupied_in_box(Bounds3{bmin, bmax});
    if (voxels.size() < 4) return;

    // For each tracked feature, find nearest L2 voxel.
    // Compute VO-predicted 3D position and the offset to the nearest voxel.
    std::vector<double> dx_offsets, dy_offsets, dz_offsets;
    for (const auto& f : state_.features) {
        if (!f.valid || f.age < 2) continue;
        // Project feature at an estimated depth (scale as proxy)
        const double depth = static_cast<double>(state_.scale.scale) * 5.0;
        const double& fx = f.x, fy = f.y;
        // This is a stub — in practice we'd use f's tracked depth.
        // For now: predicted position in map frame for this ray direction.
        const double ndx = (fx - 320.0) / 525.0;  // approximate K
        const double ndy = (fy - 240.0) / 525.0;
        const auto& R = state_.rotation;
        const Vec3 feat_pos{
            state_.position.x + depth * (R[0]*ndx + R[1]*ndy + R[2]),
            state_.position.y + depth * (R[3]*ndx + R[4]*ndy + R[5]),
            state_.position.z + depth * (R[6]*ndx + R[7]*ndy + R[8])};

        // Find nearest voxel
        double best_d2 = r * r;
        const Vec3* nearest = nullptr;
        for (const auto& v : voxels) {
            const double d2 = (v.x-feat_pos.x)*(v.x-feat_pos.x)
                            + (v.y-feat_pos.y)*(v.y-feat_pos.y)
                            + (v.z-feat_pos.z)*(v.z-feat_pos.z);
            if (d2 < best_d2) { best_d2 = d2; nearest = &v; }
        }
        if (!nearest || std::sqrt(best_d2) > 2.0) continue;
        dx_offsets.push_back(nearest->x - feat_pos.x);
        dy_offsets.push_back(nearest->y - feat_pos.y);
        dz_offsets.push_back(nearest->z - feat_pos.z);
    }

    if (dx_offsets.size() < 4) return;

    // Median offset → position correction
    auto med = [](std::vector<double>& v) {
        std::nth_element(v.begin(), v.begin() + v.size()/2, v.end());
        return v[v.size()/2];
    };
    state_.position.x += med(dx_offsets);
    state_.position.y += med(dy_offsets);
    state_.position.z += med(dz_offsets);

    // Partial drift reset after successful relocalization
    state_.cumulative_drift_m *= 0.5;
    state_.translation_sigma  *= 0.5;
}

// ── estimate() ── VL1 + VL2 + VL3 orchestration ─────────────────────────────

EgoStateEstimate VisualEgoStateProvider::estimate(const FramePacket& frame) {
    // Convert to greyscale
    std::vector<std::uint8_t> gray;
    const int w = frame.image.width;
    const int h = frame.image.height;

    if (w <= 0 || h <= 0 || frame.image.bytes.empty()) {
        EgoStateEstimate est;
        est.confidence = 0.0F;
        return est;
    }

    if (frame.image.channels == 3) {
        rgb_to_gray(frame.image.bytes.data(), w, h, gray);
    } else if (frame.image.channels == 1) {
        gray = frame.image.bytes;
    } else {
        // Unsupported — take first channel
        gray.resize(static_cast<std::size_t>(w * h));
        for (int i = 0; i < w * h; ++i)
            gray[static_cast<std::size_t>(i)] =
                frame.image.bytes[static_cast<std::size_t>(i * frame.image.channels)];
    }

    // Initialise on first frame
    if (!state_.initialized) {
        state_.prev_gray   = gray;
        state_.prev_width  = w;
        state_.prev_height = h;
        state_.initialized = true;
        detect_fast9(gray.data(), w, h,
                     config_.fast_threshold, config_.max_features,
                     state_.features);

        // Seed position from ego_hint if available
        if (frame.ego_hint) {
            state_.position = frame.ego_hint->local_T_body.position;
        }
        state_.scale.scale      = config_.initial_scale_m;
        state_.scale.confidence = 0.1F;

        EgoStateEstimate est;
        EgoState ego;
        ego.timestamp    = frame.timestamp;
        ego.map_frame_id = config_.map_frame_id;
        ego.local_T_body.position = state_.position;
        ego.confidence   = 0.1F;
        est.ego = ego;
        est.confidence = 0.1F;
        est.telemetry_available = false;
        return est;
    }

    // ── VL1: metric scale hint from AirSim velocity (if present) ─────────────
    double dt_s = 1.0 / 30.0;  // nominal 30 Hz
    if (frame.ego_hint && frame.ego_hint->velocity_local.x != 0.0) {
        const auto& v = frame.ego_hint->velocity_local;
        const double speed = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
        if (speed > 0.05) {
            // Use AirSim speed to set metric scale (VL1 only; VL2 overrides)
            state_.scale.scale      = static_cast<float>(speed * dt_s);
            state_.scale.confidence = 0.5F;
        }
    }

    // ── VL1: run one VO step ─────────────────────────────────────────────────
    const CameraIntrinsics K = (frame.intrinsics.fx > 1.0)
        ? frame.intrinsics
        : CameraIntrinsics{525.0, 525.0, static_cast<double>(w)/2.0,
                                          static_cast<double>(h)/2.0};
    const float inlier_frac = step_vl1(gray, w, h, K, dt_s);

    // ── VL2: scale update and re-localization ─────────────────────────────────
    if (l2_map_) {
        if (frame.depth_frame) {
            update_scale_vl2(gray, w, h,
                             frame.depth_frame->depth_m,
                             frame.depth_frame->width,
                             frame.depth_frame->height, K);
        }

        const float conf_now = 1.0F / (1.0F + static_cast<float>(state_.cumulative_drift_m));
        if (conf_now < config_.relocalization_confidence_threshold) {
            relocalize_vl2();
        }
    }

    // ── Confidence ────────────────────────────────────────────────────────────
    const float confidence = 1.0F / (1.0F + static_cast<float>(state_.cumulative_drift_m));

    // ── VL3: fallback to reference provider when confidence too low ───────────
    if (fallback_ && confidence < config_.fallback_confidence_threshold) {
        auto fb = fallback_->estimate(frame);
        // Restore VL position from fallback to re-anchor; reset drift partially.
        if (fb.ego) {
            state_.position           = fb.ego->local_T_body.position;
            state_.cumulative_drift_m *= 0.3;
            state_.translation_sigma  *= 0.3;
        }
        return fb;
    }

    // ── Build EgoState ────────────────────────────────────────────────────────
    EgoState ego;
    ego.timestamp    = frame.timestamp;
    ego.map_frame_id = config_.map_frame_id;
    ego.local_T_body.position = state_.position;

    // Derive RPY from rotation matrix (roll=0 assumption — VO gives yaw/pitch)
    const auto& R = state_.rotation;
    ego.local_T_body.rotation_rpy.z = std::atan2(R[3], R[0]);        // yaw
    ego.local_T_body.rotation_rpy.y = std::atan2(-R[6],              // pitch
        std::sqrt(R[7]*R[7] + R[8]*R[8]));
    ego.local_T_body.rotation_rpy.x = std::atan2(R[7], R[8]);        // roll

    if (frame.ego_hint) {
        ego.height_m    = frame.ego_hint->height_m;
        ego.height_valid = frame.ego_hint->height_valid;
    }
    ego.confidence = confidence;

    // Update previous frame
    state_.prev_gray   = gray;
    state_.prev_width  = w;
    state_.prev_height = h;

    EgoStateEstimate est;
    est.ego                 = ego;
    est.confidence          = confidence;
    est.telemetry_available = false;
    (void)inlier_frac;
    return est;
}

}  // namespace dedalus
