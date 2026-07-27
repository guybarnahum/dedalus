#pragma once

#include <atomic>
#include <cstddef>
#include <deque>
#include <filesystem>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include "dedalus/avoidance/local_esdf_map.hpp"
#include "dedalus/avoidance/local_esdf_map_publisher.hpp"
#include "dedalus/avoidance/local_flight_map.hpp"
#include "dedalus/avoidance/mission_local_obstacle_map.hpp"
#include "dedalus/avoidance/mission_local_planning_map.hpp"
#include "dedalus/avoidance/mission_local_planning_map_publisher.hpp"
#include "dedalus/avoidance/mission_local_traversability_map.hpp"
#include "dedalus/avoidance/mission_local_traversability_map_publisher.hpp"
#include "dedalus/avoidance/mission_map_assimilator.hpp"
#include "dedalus/avoidance/mission_obstacle_map_artifact_writer.hpp"
#include "dedalus/avoidance/mission_obstacle_map_delta_writer.hpp"
#include "dedalus/avoidance/mission_traversability_map_artifact_writer.hpp"
#include "dedalus/avoidance/trajectory_safety_evaluator.hpp"
#include "dedalus/runtime/perch_candidate_evaluator.hpp"

#include "dedalus/perception/ghost_targets.hpp"
#include "dedalus/runtime/evaluation_slot.hpp"
#include "dedalus/runtime/pipeline_profiler.hpp"
#include "dedalus/runtime/provider_registry.hpp"
#include "dedalus/sensing/airsim_emulation_depth_obstacle_detector.hpp"
#include "dedalus/sensing/depth_debug_annotator.hpp"
#include "dedalus/sensing/obstacle_evidence_provider.hpp"
#include "dedalus/sensing/sensing_coverage.hpp"
#include "dedalus/sensing/visual_depth_obstacle_detector.hpp"
#include "dedalus/world_model/world_snapshot.hpp"
#include "dedalus/world_model/world_snapshot_publisher.hpp"

namespace dedalus {

struct CoreStackRunnerConfig {
    std::unique_ptr<PipelineProfiler> timing_writer;
    std::shared_ptr<WorldSnapshotPublisher> snapshot_publisher;
    std::shared_ptr<GhostDetectionsPublisher> ghost_detections_publisher;
    std::shared_ptr<MissionObstacleMapDeltaPublisher> mission_obstacle_map_delta_publisher;
    std::shared_ptr<MissionLocalTraversabilityMapPublisher> traversability_map_publisher;
    // Optional: publish Level 2 planning map snapshots to SSE subscribers.
    std::shared_ptr<MissionLocalPlanningMapPublisher> planning_map_publisher;
    // Optional: publish L3 ESDF snapshots to SSE subscribers (Stage 6).
    std::shared_ptr<LocalESDFMapPublisher> esdf_map_publisher;
    // Subscribers subscribed to snapshot_publisher at construction time.
    // CoreStackRunner retains these shared_ptrs (the publisher holds weak refs).
    std::vector<std::shared_ptr<WorldSnapshotSubscriber>> snapshot_subscribers;
    // Config for the visual_onnx depth provider (DepthAnythingV2).
    // Populated by config_loader when depth: visual_onnx (or depth_eval: visual_onnx).
    VisualONNXDepthConfig visual_onnx_depth;

    // Config for the unidepth_v2 depth provider (UniDepth V2).
    // Populated by config_loader when depth: unidepth_v2 (or depth_eval: unidepth_v2).
    UniDepthV2Config unidepth_v2_depth;

    // Optional: 4-panel debug MP4 (ONNX top, GT eval bottom).
    // Empty output_path disables the annotator entirely (zero overhead).
    // Set via visual_onnx.debug_depth_mp4 or unidepth.debug_depth_mp4 in config.
    DepthDebugAnnotatorConfig debug_depth_annotator;

    // Two-slot depth provider injection.
    //
    // Slot A (primary): evidence feeds L1/L2 map.
    // Slot B (reference): evidence delta-logged only; not fed to the map.
    //
    // If depth_slot_b is null slot B is inactive.
    std::unique_ptr<ObstacleEvidenceProvider> depth_slot_a;
    std::unique_ptr<ObstacleEvidenceProvider> depth_slot_b;

    // Reference slots (slot B) for each perception pipeline stage.
    // Slot A (primary) for each stage lives in CoreStackProviders.
    // Null = stage B inactive (zero overhead).  All default to null.
    //
    // Contract: slot B receives the same primary-slot inputs as slot A.
    //           slot B output is never fed downstream — logged only.
    std::shared_ptr<EgoStateProvider> ego_provider_reference;
    std::shared_ptr<Detector>         detector_reference;
    std::shared_ptr<CameraStabilizer> stabilizer_reference;
    std::shared_ptr<Tracker>          tracker_reference;
    std::shared_ptr<IdentityResolver> identity_resolver_reference;
    std::shared_ptr<Projector3D>      projector_reference;

    PerchCandidateEvaluatorConfig perch_candidate_evaluator;

    MissionMapAssimilatorConfig mission_map_assimilator;

    // Optional path for Level 2 planning map cross-mission persistence.
    // If set: the planning map is loaded from this file at construction
    // (if it exists) and saved back at finalize_mission_map_after_landing().
    std::filesystem::path planning_map_persistence_path;

    // Per-tick wall-clock budget (microseconds) for L3/ESDF catch-up with L2 —
    // see CoreStackRunner::esdf_map_'s doc comment for the full mechanism.
    // L3 is purely a cache of L2, so it's allowed to lag; this bounds how much
    // time each tick may spend draining pending catch-up work before deferring
    // the rest to the next tick. 0 disables catch-up entirely (L3 then only
    // ever reflects L2's state at mission startup). Set via esdf.catchup_budget_us.
    std::int64_t esdf_catchup_budget_us{20'000LL};
};
// Note: L3 (LocalESDFMap) is never saved to disk — it's always derived from L2,
// which is the persisted source of truth. It's fully computed once at startup
// from whatever L2 state exists then, and kept caught up afterward by the
// bounded per-tick catch-up step above (esdf_catchup_budget_us), not by saving
// and reloading a stale L3 binary. If you need to inspect L3 off-line, use
// compute_esdf() directly against the l2_map.db (see
// include/dedalus/avoidance/local_esdf_map.hpp).

class CoreStackRunner {
public:
    CoreStackRunner(CoreStackProviders providers, CoreStackRunnerConfig config = {});
    ~CoreStackRunner();

    CoreStackRunner(const CoreStackRunner&) = delete;
    CoreStackRunner& operator=(const CoreStackRunner&) = delete;
    CoreStackRunner(CoreStackRunner&&) = delete;
    CoreStackRunner& operator=(CoreStackRunner&&) = delete;

    void update_camera_pointing_states(std::vector<CameraPointingState> states);

    [[nodiscard]] bool run_once();
    [[nodiscard]] WorldSnapshot snapshot() const;

    [[nodiscard]] MissionMapFlushResult finalize_mission_map_after_landing(TimePoint now);
    [[nodiscard]] MissionMapAssimilationStatus mission_map_assimilation_status() const;
    [[nodiscard]] MissionLocalTraversabilityMapSnapshot mission_local_traversability_map_snapshot(
        std::size_t max_cells = 0U) const;

private:
    CoreStackProviders providers_;
    std::unique_ptr<PipelineProfiler> timing_writer_;
    std::shared_ptr<WorldSnapshotPublisher> snapshot_publisher_;
    std::shared_ptr<GhostDetectionsPublisher> ghost_detections_publisher_;
    std::shared_ptr<MissionObstacleMapDeltaPublisher> mission_obstacle_map_delta_publisher_;
    std::shared_ptr<MissionLocalTraversabilityMapPublisher> traversability_map_publisher_;
    std::shared_ptr<MissionLocalPlanningMapPublisher> planning_map_publisher_;
    std::shared_ptr<LocalESDFMapPublisher> esdf_map_publisher_;
    std::vector<std::shared_ptr<WorldSnapshotSubscriber>> snapshot_subscriber_handles_;
    // Slot A: primary depth provider (evidence → L1/L2 map).
    // Slot B: reference depth provider (delta-log only; null = inactive).
    std::unique_ptr<ObstacleEvidenceProvider> depth_slot_a_;
    std::unique_ptr<ObstacleEvidenceProvider> depth_slot_b_;

    // Optional debug MP4 annotator; null when output_path is empty.
    std::unique_ptr<DepthDebugAnnotator> depth_annotator_;

    // Reference slots (slot B) for the five perception pipeline stages.
    // Slot A for each stage lives in providers_.  Null = inactive.
    std::shared_ptr<EgoStateProvider> ego_provider_reference_;
    std::shared_ptr<Detector>         detector_reference_;
    std::shared_ptr<CameraStabilizer> stabilizer_reference_;
    std::shared_ptr<Tracker>          tracker_reference_;
    std::shared_ptr<IdentityResolver> identity_resolver_reference_;
    std::shared_ptr<Projector3D>      projector_reference_;

    SensingCoverageProvider sensing_coverage_provider_;
    MissionLocalObstacleMap mission_local_obstacle_map_;
    MissionMapAssimilator mission_map_assimilator_;
    // Level 2: compressed planning map rebuilt from Level 1 after each assimilator drain.
    MissionLocalPlanningMap mission_local_planning_map_;
    // Level 3: ESDF derived from L2 — nothing else, it's purely a cache of L2.
    // Not needed in real time (it helps plan a *future* mission's trajectory,
    // not steer this one), so it's allowed to lag, but every L2 change must
    // eventually reach it — never silently dropped. Computed fully at startup
    // from whatever L2 state exists then; kept caught up afterward by a
    // bounded per-tick catch-up step (see run_once()) instead of a live
    // full-recompute-on-every-slide trigger.
    LocalESDFMap      esdf_map_;
    std::uint64_t     esdf_seq_{0U};
    // L3 catch-up state (see the "Level 3 ESDF catch-up" step in run_once()):
    //   esdf_last_l2_seq_          — how far the L2 *value*-delta catch-up has
    //                                progressed; only advances once its delta
    //                                has actually been applied.
    //   esdf_pending_residency_    — FIFO of newly-loaded-from-DB position
    //                                batches (from slide_window()) not yet
    //                                applied; new batches append, only ever
    //                                popped after being applied — never
    //                                dropped, only deferred to a later tick.
    std::uint64_t     esdf_last_l2_seq_{0U};
    std::deque<std::vector<Vec3>> esdf_pending_residency_batches_;
    // Set when the per-tick catch-up step applies any update to esdf_map_;
    // cleared once that update has actually been published. Decouples "keep
    // esdf_map_ caught up" (every tick, cheap) from "tell SSE clients" (only
    // when something changed, throttled to the same 2s cadence as L1/L2).
    bool esdf_dirty_since_publish_{false};
    MissionObstacleMapArtifactWriter mission_obstacle_map_artifact_writer_;
    MissionObstacleMapDeltaWriter mission_obstacle_map_delta_writer_;
    MissionTraversabilityMapArtifactWriter mission_traversability_map_artifact_writer_;
    LocalFlightMapAccumulator local_flight_map_accumulator_;
    std::string               depth_slot_a_name_;   // provider_name() cached at construction
    // Pipeline provider names cached from CoreStackProviders at construction.
    std::string               ego_provider_name_;
    std::string               detector_name_;
    std::string               camera_stabilizer_name_;
    std::string               tracker_name_;
    std::string               identity_resolver_name_;
    std::string               projector_name_;
    TrajectorySafetyEvaluator trajectory_safety_evaluator_;
    PerchCandidateEvaluator   perch_candidate_evaluator_;
    std::uint32_t             perch_cadence_tick_{0U};
    std::vector<PerchCandidate> cached_perch_candidates_;
    std::vector<CameraPointingState> camera_pointing_states_;
    std::optional<TimePoint> ghost_scenario_start_;
    // Optional file path for Level 2 planning map cross-mission persistence.
    std::filesystem::path planning_map_persistence_path_;
    // From CoreStackRunnerConfig::esdf_catchup_budget_us (esdf.catchup_budget_us).
    std::int64_t esdf_catchup_budget_us_{20'000LL};
    // Background flush thread: calls flush_dirty_to_db() every 10 s.
    // Only started when planning_map_persistence_path_ is non-empty.
    std::atomic<bool> planning_map_flush_stop_{false};
    std::thread       planning_map_flush_thread_;
    // Stage 5: last L2 seq number sent to SSE; used to produce delta snapshots.
    std::uint64_t     l2_last_published_seq_{0U};
    // Throttle: timestamp (ns) of the last traversability snapshot published to
    // traversability_map_publisher_.  Publish at most once every
    // kTravPublishMinIntervalNs to avoid drowning the SSE stream.
    std::uint64_t last_trav_publish_ns_{0U};
};

}  // namespace dedalus
