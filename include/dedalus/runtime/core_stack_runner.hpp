#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "dedalus/avoidance/local_flight_map.hpp"
#include "dedalus/avoidance/mission_local_obstacle_map.hpp"
#include "dedalus/avoidance/mission_obstacle_map_artifact_writer.hpp"
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
    // Subscribers subscribed to snapshot_publisher at construction time.
    // CoreStackRunner retains these shared_ptrs (the publisher holds weak refs).
    std::vector<std::shared_ptr<WorldSnapshotSubscriber>> snapshot_subscribers;
    AirSimDepthObstacleDetectorConfig airsim_depth_obstacle_detector;
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

private:
    CoreStackProviders providers_;
    std::unique_ptr<PipelineProfiler> timing_writer_;
    std::shared_ptr<WorldSnapshotPublisher> snapshot_publisher_;
    std::shared_ptr<GhostDetectionsPublisher> ghost_detections_publisher_;
    std::vector<std::shared_ptr<WorldSnapshotSubscriber>> snapshot_subscriber_handles_;
    AirSimDepthObstacleDetectorConfig airsim_depth_obstacle_detector_config_;
    SensingCoverageProvider sensing_coverage_provider_;
    MissionLocalObstacleMap mission_local_obstacle_map_;
    MissionObstacleMapArtifactWriter mission_obstacle_map_artifact_writer_;
    LocalFlightMapAccumulator local_flight_map_accumulator_;
    TrajectorySafetyEvaluator trajectory_safety_evaluator_;
    std::vector<CameraPointingState> camera_pointing_states_;
    std::optional<TimePoint> ghost_scenario_start_;
};

}  // namespace dedalus
