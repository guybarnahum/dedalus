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
      esdf_map_publisher_(std::move(config.esdf_map_publisher)),
      snapshot_subscriber_handles_(std::move(config.snapshot_subscribers)),
      depth_slot_a_(std::move(config.depth_slot_a)),
      depth_slot_b_(std::move(config.depth_slot_b)),
      ego_provider_reference_(std::move(config.ego_provider_reference)),
      detector_reference_(std::move(config.detector_reference)),
      stabilizer_reference_(std::move(config.stabilizer_reference)),
      tracker_reference_(std::move(config.tracker_reference)),
      identity_resolver_reference_(std::move(config.identity_resolver_reference)),
      projector_reference_(std::move(config.projector_reference)),
      sensing_coverage_provider_(providers_.obstacle_sensing_cameras),
      mission_map_assimilator_(config.mission_map_assimilator),
      mission_obstacle_map_artifact_writer_(MissionObstacleMapArtifactWriter::from_environment()),
      mission_obstacle_map_delta_writer_(MissionObstacleMapDeltaWriter::from_environment()),
      mission_traversability_map_artifact_writer_(
          MissionTraversabilityMapArtifactWriter::from_environment()),
      perch_candidate_evaluator_(config.perch_candidate_evaluator),
      planning_map_persistence_path_(std::move(config.planning_map_persistence_path)) {
    depth_slot_a_name_ = depth_slot_a_ ? depth_slot_a_->provider_name() : "";

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

                    // Drain staged ESDF snapshot into the same DB.
                    std::vector<MissionLocalPlanningMap::ESDFCellRecord> esdf_cells;
                    double d0_m = 5.0;
                    {
                        std::lock_guard<std::mutex> lk(esdf_flush_mutex_);
                        if (esdf_flush_pending_) {
                            esdf_cells          = std::move(esdf_flush_cells_);
                            esdf_flush_cells_   = {};
                            d0_m                = esdf_flush_d0_m_;
                            esdf_flush_pending_ = false;
                        }
                    }
                    if (!esdf_cells.empty()) {
                        mission_local_planning_map_.flush_esdf_to_db(esdf_cells, d0_m);
                    }
                }
            }
        });
    }

}
// Note: L3 is always recomputed from L2 (first run_once() call, before blocking on
// the frame source).  No disk load at startup — recompute from the in-memory L2
// window is ~6 ms and always consistent with the current L2 state.

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
    // L3 is not saved: always recomputed from L2 at next startup.

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
