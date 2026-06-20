#include "dedalus/runtime/core_stack_runner.hpp"

#include <utility>
#include <vector>
#include <filesystem>

namespace dedalus {
namespace {

}  // namespace

CoreStackRunner::CoreStackRunner(CoreStackProviders providers, CoreStackRunnerConfig config)
    : providers_(std::move(providers)),
      timing_writer_(std::move(config.timing_writer)),
      snapshot_publisher_(std::move(config.snapshot_publisher)),
      ghost_detections_publisher_(std::move(config.ghost_detections_publisher)),
      mission_obstacle_map_delta_publisher_(std::move(config.mission_obstacle_map_delta_publisher)),
      traversability_map_publisher_(std::move(config.traversability_map_publisher)),
      snapshot_subscriber_handles_(std::move(config.snapshot_subscribers)),
      airsim_depth_obstacle_detector_config_(config.airsim_depth_obstacle_detector),
      sensing_coverage_provider_(providers_.obstacle_sensing_cameras),
      mission_map_assimilator_(config.mission_map_assimilator),
      mission_obstacle_map_artifact_writer_(MissionObstacleMapArtifactWriter::from_environment()),
      mission_obstacle_map_delta_writer_(MissionObstacleMapDeltaWriter::from_environment()),
      mission_traversability_map_artifact_writer_(
          MissionTraversabilityMapArtifactWriter::from_environment()),
      planning_map_persistence_path_(std::move(config.planning_map_persistence_path)) {
    if (!snapshot_subscriber_handles_.empty()) {
        if (!snapshot_publisher_) {
            snapshot_publisher_ = std::make_shared<WorldSnapshotPublisher>();
        }
        for (const auto& sub : snapshot_subscriber_handles_) {
            snapshot_publisher_->subscribe(sub);
        }
    }

    // Load Level 2 planning map from the previous mission's persistence file, if present.
    // A missing file is not an error — it simply means a fresh map starts from empty.
    if (!planning_map_persistence_path_.empty() &&
        std::filesystem::exists(planning_map_persistence_path_)) {
        mission_local_planning_map_.load_from_file(planning_map_persistence_path_);
    }
}

CoreStackRunner::~CoreStackRunner() {
    try {
        const auto latest = snapshot();
        (void)finalize_mission_map_after_landing(latest.timestamp);
    } catch (...) {
        // Destruction must remain best-effort: map finalization is a shutdown
        // persistence/diagnostics barrier, not a command-sink or safety path.
    }
}

void CoreStackRunner::update_camera_pointing_states(std::vector<CameraPointingState> states) {
    camera_pointing_states_ = std::move(states);
}


WorldSnapshot CoreStackRunner::snapshot() const {
    return providers_.world_model->snapshot();
}

MissionMapFlushResult CoreStackRunner::finalize_mission_map_after_landing(const TimePoint now) {
    const auto obstacle_snapshot = mission_local_obstacle_map_.snapshot();
    if (obstacle_snapshot.summary.update_count > 0U || !obstacle_snapshot.cells.empty()) {
        mission_map_assimilator_.enqueue_mission_obstacle_map(obstacle_snapshot);
    }
    auto result = mission_map_assimilator_.flush_after_landing(now);
    mission_traversability_map_artifact_writer_.write_final(
        mission_map_assimilator_.traversability_map().snapshot());

    // Persist Level 2 planning map so it survives to the next mission.
    // The file is written atomically via a temp-then-rename pattern to avoid
    // leaving a half-written file if the process is interrupted.
    if (!planning_map_persistence_path_.empty()) {
        const auto tmp = std::filesystem::path{
            planning_map_persistence_path_.string() + ".tmp"};
        if (mission_local_planning_map_.save_to_file(tmp)) {
            std::filesystem::rename(tmp, planning_map_persistence_path_);
        }
    }

    return result;
}

MissionMapAssimilationStatus CoreStackRunner::mission_map_assimilation_status() const {
    return mission_map_assimilator_.status();
}

MissionLocalTraversabilityMapSnapshot CoreStackRunner::mission_local_traversability_map_snapshot(
    const std::size_t max_cells) const {
    return mission_map_assimilator_.traversability_map().snapshot(max_cells);
}

}  // namespace dedalus
