#pragma once

#include <atomic>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

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

#include "dedalus/perception/ghost_targets.hpp"
#include "dedalus/runtime/pipeline_profiler.hpp"
#include "dedalus/runtime/provider_registry.hpp"
#include "dedalus/sensing/airsim_depth_obstacle_detector.hpp"
#include "dedalus/sensing/sensing_coverage.hpp"
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
    // Subscribers subscribed to snapshot_publisher at construction time.
    // CoreStackRunner retains these shared_ptrs (the publisher holds weak refs).
    std::vector<std::shared_ptr<WorldSnapshotSubscriber>> snapshot_subscribers;
    AirSimDepthObstacleDetectorConfig airsim_depth_obstacle_detector;
    MissionMapAssimilatorConfig mission_map_assimilator;

    // Optional path for Level 2 planning map cross-mission persistence.
    // If set: the planning map is loaded from this file at construction
    // (if it exists) and saved back at finalize_mission_map_after_landing().
    std::filesystem::path planning_map_persistence_path;
};

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
    std::vector<std::shared_ptr<WorldSnapshotSubscriber>> snapshot_subscriber_handles_;
    AirSimDepthObstacleDetectorConfig airsim_depth_obstacle_detector_config_;
    SensingCoverageProvider sensing_coverage_provider_;
    MissionLocalObstacleMap mission_local_obstacle_map_;
    MissionMapAssimilator mission_map_assimilator_;
    // Level 2: compressed planning map rebuilt from Level 1 after each assimilator drain.
    MissionLocalPlanningMap mission_local_planning_map_;
    MissionObstacleMapArtifactWriter mission_obstacle_map_artifact_writer_;
    MissionObstacleMapDeltaWriter mission_obstacle_map_delta_writer_;
    MissionTraversabilityMapArtifactWriter mission_traversability_map_artifact_writer_;
    LocalFlightMapAccumulator local_flight_map_accumulator_;
    TrajectorySafetyEvaluator trajectory_safety_evaluator_;
    std::vector<CameraPointingState> camera_pointing_states_;
    std::optional<TimePoint> ghost_scenario_start_;
    // Optional file path for Level 2 planning map cross-mission persistence.
    std::filesystem::path planning_map_persistence_path_;
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
