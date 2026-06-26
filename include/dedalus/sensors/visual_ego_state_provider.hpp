#pragma once

#include <memory>

#include "dedalus/avoidance/mission_local_planning_map.hpp"
#include "dedalus/sensors/ego_state_provider.hpp"
#include "dedalus/sensors/visual_odometry_state.hpp"

namespace dedalus {

// ── VisualEgoStateProvider ───────────────────────────────────────────────────
//
// Camera-only ego state: no GPS, no AirSim pose API.  Implements the VL series.
//
//   VL1  FAST-9 corners + Lucas-Kanade tracking + Focus-of-Expansion RANSAC.
//        Angular velocity from flow residual.  Metric scale from velocity hint
//        (AirSim) or config.initial_scale_m.  Integrates cumulative pose.
//
//   VL2  Scale from depth frame: depth_m at tracked features vs. nearest L2
//        voxel distances → EMA scale update.  ICP-lite re-localization against
//        L2 when confidence drops below relocalization threshold.
//
//   VL3  Per-frame uncertainty propagation via scalar covariance proxy.
//        Falls back to an injected reference provider (typically frame_hint /
//        AirSim telemetry) when confidence < fallback_confidence_threshold.
//        In real-world context leave fallback null: no fall-back, just wider
//        L0 avoidance margins driven by reduced confidence.
//
// Provider contract
//   YAML key  : ego_provider: visual_odometry
//   Env var   : DEDALUS_EGO_PROVIDER=visual_odometry
//   Eval slot : ego_provider_eval: airsim  (slot B oracle for agreement metric)
//   Agreement : ego_provider.slot.agreement_ppt  (position distance ppt of 1 m)
//   Report    : tools/perception/ego_state_report.py
//
// Thread ownership: single-threaded tick; estimate() is not reentrant.

class VisualEgoStateProvider final : public EgoStateProvider {
public:
    explicit VisualEgoStateProvider(VisualOdometryConfig config = {});

    EgoStateEstimate estimate(const FramePacket& frame) override;

    // VL2: inject L2 planning map for scale estimation + ICP re-localization.
    // Raw non-owning pointer; must outlive this provider (runner owns both).
    void set_l2_map(const MissionLocalPlanningMap* map) { l2_map_ = map; }

    // VL3: inject fallback ego provider (typically FrameHintEgoProvider /
    // AirSim telemetry).  Activated when confidence < fallback threshold.
    void set_fallback_provider(std::shared_ptr<EgoStateProvider> fallback) {
        fallback_ = std::move(fallback);
    }

    const VisualOdometryState& state() const { return state_; }

private:
    // VL1: one frame of visual odometry; returns inlier fraction [0,1].
    float step_vl1(const std::vector<std::uint8_t>& gray,
                   int width, int height,
                   const CameraIntrinsics& K,
                   double dt_s);

    // VL2: update MetricScaleEstimate from depth frame vs L2 occupied cells.
    void update_scale_vl2(const std::vector<std::uint8_t>& gray,
                          int width, int height,
                          const std::vector<float>& depth_m,
                          int depth_w, int depth_h,
                          const CameraIntrinsics& K);

    // VL2: ICP-lite re-localization; corrects state_.position in place.
    void relocalize_vl2();

    VisualOdometryConfig       config_;
    VisualOdometryState        state_;
    const MissionLocalPlanningMap* l2_map_{nullptr};
    std::shared_ptr<EgoStateProvider> fallback_;
};

}  // namespace dedalus
