#include "dedalus/runtime/core_stack_runner.hpp"

#include <chrono>
#include <filesystem>
#include <thread>
#include <utility>
#include <vector>

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
      planning_map_publisher_(std::move(config.planning_map_publisher)),
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

    // Open (or create) the Level 2 SQLite persistence DB.  A missing file is not
    // an error — open_db() creates a fresh DB with the correct schema.
    // A missing path means no persistence this session.
    if (!planning_map_persistence_path_.empty()) {
        mission_local_planning_map_.open_db(planning_map_persistence_path_);

        // Start background flush thread: drains dirty cells every 10 s.
        planning_map_flush_stop_.store(false, std::memory_order_relaxed);
        planning_map_flush_thread_ = std::thread([this] {
            while (!planning_map_flush_stop_.load(std::memory_order_acquire)) {
                std::this_thread::sleep_for(std::chrono::seconds(10));
                if (!planning_map_flush_stop_.load(std::memory_order_acquire)) {
                    mission_local_planning_map_.flush_dirty_to_db();
                }
            }
        });
    }
}

CoreStackRunner::~CoreStackRunner() {
    // Stop the flush thread before finalizing (prevents concurrent flush during close_db).
    planning_map_flush_stop_.store(true, std::memory_order_release);
    if (planning_map_flush_thread_.joinable()) {
        planning_map_flush_thread_.join();
    }

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

    // Final flush + close the SQLite DB (WAL checkpoint included in sqlite3_close).
    if (!planning_map_persistence_path_.empty()) {
        mission_local_planning_map_.close_db();
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
