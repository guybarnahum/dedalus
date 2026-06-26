#pragma once

// A/B Evaluation Pipeline — first-level design primitive.
//
// Every perception stage supports an optional reference (slot B) provider alongside
// the primary (slot A) provider.  Slot B receives the same inputs as slot A, but its
// outputs are never fed downstream — they are logged only.
//
// Agreement metric conventions:
//   - Returned as float in [0, 1].
//   - Callers log as parts-per-thousand:
//       timing_writer_->record_stage("<stage>.slot.agreement_ppt",
//                                    static_cast<int64_t>(agreement * 1000));
//   - Undefined (no data) → returns 0.
//
// EvaluationSlotPair<T>: typed container.  Callers hold the primary provider
// separately (e.g. in CoreStackProviders); only the reference lives here.

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "dedalus/perception/types.hpp"
#include "dedalus/sensors/ego_state_provider.hpp"

namespace dedalus {

// ---------------------------------------------------------------------------
// Generic container
// ---------------------------------------------------------------------------

// Reference provider for a single pipeline stage.
// null reference_ means the stage is inactive (zero overhead).
template<typename Provider>
struct EvaluationSlotPair {
    std::shared_ptr<Provider> primary;    // slot A — feeds downstream
    std::shared_ptr<Provider> reference;  // slot B — delta-log only; null = inactive

    bool has_reference() const noexcept { return reference != nullptr; }
};

// ---------------------------------------------------------------------------
// Agreement metric: Detector (Detection2D[])
//
// Agreement = fraction of slot-A detections with a slot-B detection at IoU > 0.5.
// Returns 0 when a is empty or b is empty.
// ---------------------------------------------------------------------------

// IoU helper — inline so it is ODR-safe in multi-TU builds.
// Internal use only; not part of the public API.
inline double eval_slot_iou(const Rect2& a, const Rect2& b) {
    const double ax2 = a.x + a.width;
    const double ay2 = a.y + a.height;
    const double bx2 = b.x + b.width;
    const double by2 = b.y + b.height;

    const double ix1 = std::max(a.x, b.x);
    const double iy1 = std::max(a.y, b.y);
    const double ix2 = std::min(ax2, bx2);
    const double iy2 = std::min(ay2, by2);

    if (ix2 <= ix1 || iy2 <= iy1) return 0.0;
    const double inter = (ix2 - ix1) * (iy2 - iy1);
    const double area_a = a.width * a.height;
    const double area_b = b.width * b.height;
    const double uni = area_a + area_b - inter;
    return (uni > 0.0) ? (inter / uni) : 0.0;
}

[[nodiscard]] inline float detection_agreement(
    const std::vector<Detection2D>& a,
    const std::vector<Detection2D>& b,
    double iou_threshold = 0.5) {
    if (a.empty() || b.empty()) return 0.0F;

    std::uint32_t matched = 0U;
    for (const auto& da : a) {
        for (const auto& db : b) {
            if (eval_slot_iou(da.bbox_px, db.bbox_px) >= iou_threshold) {
                ++matched;
                break;  // first match suffices
            }
        }
    }
    return static_cast<float>(matched) / static_cast<float>(a.size());
}

// ---------------------------------------------------------------------------
// Agreement metric: CameraStabilizer (StabilizedFrame)
//
// Agreement = 1 - normalized_delta, where delta is the Euclidean distance
// between slot-A and slot-B (dx, dy) translation estimates, clamped to
// [0, max_delta_px].  Rotation ignored (translation dominates for single-camera).
// Returns 0 when either transform is unavailable.
// ---------------------------------------------------------------------------

struct StabilizerOutput {
    bool transform_available{false};
    double dx_px{0.0};
    double dy_px{0.0};
};

[[nodiscard]] inline float stabilizer_agreement(
    const StabilizerOutput& a,
    const StabilizerOutput& b,
    double max_delta_px = 50.0) {
    if (!a.transform_available || !b.transform_available) return 0.0F;

    const double ddx = a.dx_px - b.dx_px;
    const double ddy = a.dy_px - b.dy_px;
    const double dist = std::sqrt(ddx * ddx + ddy * ddy);
    const double clamped = std::min(dist, max_delta_px);
    return static_cast<float>(1.0 - clamped / max_delta_px);
}

// ---------------------------------------------------------------------------
// Agreement metric: Tracker (Track2D[])
//
// Agreement = fraction of slot-A tracks with a slot-B track of the same
// class_label whose bbox centroid is within centroid_threshold_px.
// Returns 0 when a is empty.
// ---------------------------------------------------------------------------

[[nodiscard]] inline float tracker_agreement(
    const std::vector<Track2D>& a,
    const std::vector<Track2D>& b,
    double centroid_threshold_px = 50.0) {
    if (a.empty() || b.empty()) return 0.0F;

    std::uint32_t matched = 0U;
    for (const auto& ta : a) {
        const double cax = ta.bbox_px.x + ta.bbox_px.width  * 0.5;
        const double cay = ta.bbox_px.y + ta.bbox_px.height * 0.5;
        for (const auto& tb : b) {
            if (ta.class_label != tb.class_label) continue;
            const double cbx = tb.bbox_px.x + tb.bbox_px.width  * 0.5;
            const double cby = tb.bbox_px.y + tb.bbox_px.height * 0.5;
            const double dx = cax - cbx;
            const double dy = cay - cby;
            if (std::sqrt(dx * dx + dy * dy) < centroid_threshold_px) {
                ++matched;
                break;
            }
        }
    }
    return static_cast<float>(matched) / static_cast<float>(a.size());
}

// ---------------------------------------------------------------------------
// Agreement metric: IdentityResolver (IdentityHypothesis[])
//
// Agreement = fraction of slot-A identities with a slot-B identity on the
// same track_id whose identity_id matches.
// Returns 0 when a is empty.
// ---------------------------------------------------------------------------

[[nodiscard]] inline float identity_agreement(
    const std::vector<IdentityHypothesis>& a,
    const std::vector<IdentityHypothesis>& b) {
    if (a.empty() || b.empty()) return 0.0F;

    std::uint32_t matched = 0U;
    for (const auto& ia : a) {
        for (const auto& ib : b) {
            if (ia.track_id.value == ib.track_id.value && ia.identity_id.value == ib.identity_id.value) {
                ++matched;
                break;
            }
        }
    }
    return static_cast<float>(matched) / static_cast<float>(a.size());
}

// ---------------------------------------------------------------------------
// Agreement metric: Projector3D (Observation3D[])
//
// Agreement = fraction of slot-A observations with a slot-B observation whose
// position_local is within distance_threshold_m.
// Returns 0 when a is empty.
// ---------------------------------------------------------------------------

[[nodiscard]] inline float observation_agreement(
    const std::vector<Observation3D>& a,
    const std::vector<Observation3D>& b,
    double distance_threshold_m = 1.0) {
    if (a.empty() || b.empty()) return 0.0F;

    std::uint32_t matched = 0U;
    for (const auto& oa : a) {
        for (const auto& ob : b) {
            const double dx = oa.position_local.x - ob.position_local.x;
            const double dy = oa.position_local.y - ob.position_local.y;
            const double dz = oa.position_local.z - ob.position_local.z;
            if (std::sqrt(dx*dx + dy*dy + dz*dz) < distance_threshold_m) {
                ++matched;
                break;
            }
        }
    }
    return static_cast<float>(matched) / static_cast<float>(a.size());
}

// ---------------------------------------------------------------------------
// Agreement metric: EgoStateProvider (EgoStateEstimate)
//
// Agreement = 1 - (position_distance_m / max_distance_m), clamped to [0, 1].
// max_distance_m defaults to 1.0 m (one L2 voxel lateral / 2 m vertical).
// Returns 0 when either estimate has no ego.
// ---------------------------------------------------------------------------

[[nodiscard]] inline float ego_agreement(
    const EgoStateEstimate& a,
    const EgoStateEstimate& b,
    double max_distance_m = 1.0) {
    if (!a.ego || !b.ego) return 0.0F;
    const double dx = a.ego->local_T_body.position.x - b.ego->local_T_body.position.x;
    const double dy = a.ego->local_T_body.position.y - b.ego->local_T_body.position.y;
    const double dz = a.ego->local_T_body.position.z - b.ego->local_T_body.position.z;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    return static_cast<float>(std::max(0.0, 1.0 - dist / max_distance_m));
}

}  // namespace dedalus
