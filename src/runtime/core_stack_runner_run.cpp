#include "dedalus/runtime/core_stack_runner.hpp"

#include "dedalus/avoidance/local_esdf_map.hpp"
#include "dedalus/avoidance/local_esdf_map_publisher.hpp"
#include "dedalus/avoidance/local_flight_map.hpp"
#include "dedalus/avoidance/mission_local_planning_map_publisher.hpp"
#include "dedalus/avoidance/mission_local_traversability_map_publisher.hpp"
#include "dedalus/sensing/airsim_depth_obstacle_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace dedalus {
namespace {

using SteadyClock = std::chrono::steady_clock;

static constexpr double kESDFHorizHalfM = 40.0;
static constexpr double kESDFVertHalfM  = 10.0;
static constexpr double kESDFD0M        = 5.0;

std::int64_t duration_us(const SteadyClock::time_point start) {
    return std::chrono::duration_cast<std::chrono::microseconds>(SteadyClock::now() - start).count();
}

std::int64_t duration_between_us(const SteadyClock::time_point start, const SteadyClock::time_point end) {
    return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

std::vector<ObstacleSensingVolume> obstacle_sensing_volumes_from(const SensingCoverageSnapshot& coverage) {
    std::vector<ObstacleSensingVolume> volumes;
    volumes.reserve(coverage.camera_volumes.size());
    for (const auto& camera_volume : coverage.camera_volumes) {
        volumes.push_back(to_obstacle_sensing_volume(camera_volume));
    }
    return volumes;
}

}  // namespace

bool CoreStackRunner::run_once() {
    const auto run_once_start = SteadyClock::now();

    auto start = SteadyClock::now();
    auto frame = providers_.frame_source->next_frame();
    const auto frame_available_time = SteadyClock::now();
    const auto frame_source_wait_duration_us = duration_between_us(start, frame_available_time);

    if (!frame.has_value()) {
        providers_.frame_annotator->finish();
        return false;
    }

    if (timing_writer_) {
        std::int64_t frame_source_reported_io_us = 0;
        timing_writer_->begin_frame(*frame);
        timing_writer_->record_stage("frame_source.next_frame_wait", frame_source_wait_duration_us);
        timing_writer_->record_stage("runtime.frame_source_wall_wait", frame_source_wait_duration_us);
        for (const auto& source_timing : frame->source_timings) {
            timing_writer_->record_stage(source_timing.name, source_timing.duration_us);
            frame_source_reported_io_us += source_timing.duration_us;
        }
        timing_writer_->record_stage("runtime.frame_source_reported_io", frame_source_reported_io_us);
        timing_writer_->record_stage(
            "runtime.frame_source_unattributed_wait",
            std::max<std::int64_t>(0, frame_source_wait_duration_us - frame_source_reported_io_us));
    }

    start = SteadyClock::now();
    const auto ego_estimate = providers_.ego_provider->estimate(*frame);
    if (timing_writer_) {
        timing_writer_->record_stage("ego_provider.estimate", duration_us(start));
    }
    if (!ego_estimate.ego.has_value()) {
        if (timing_writer_) {
            timing_writer_->record_stage("runtime.post_frame_compute", duration_between_us(frame_available_time, SteadyClock::now()));
            timing_writer_->set_measured_total(duration_us(run_once_start));
            timing_writer_->end_frame();
        }
        return false;
    }

    std::vector<ObstacleSensingVolume> current_sensing_volumes;
    if (!providers_.obstacle_sensing_cameras.empty()) {
        start = SteadyClock::now();
        const auto coverage = sensing_coverage_provider_.snapshot({*frame}, *ego_estimate.ego, camera_pointing_states_);
        current_sensing_volumes = obstacle_sensing_volumes_from(coverage);
        if (timing_writer_) {
            timing_writer_->record_stage("sensing_coverage.snapshot", duration_us(start));
        }
    }

    start = SteadyClock::now();
    PerceptionPipeline pipeline(
        providers_.detector,
        providers_.camera_stabilizer,
        providers_.tracker,
        providers_.identity_resolver,
        providers_.projector);
    if (timing_writer_) {
        timing_writer_->record_stage("perception_pipeline.construct", duration_us(start));
    }

    start = SteadyClock::now();
    auto perception_output = pipeline.process(*frame, *ego_estimate.ego);
    if (timing_writer_) {
        timing_writer_->record_stage("perception_pipeline.process", duration_us(start));
    }

    if (frame->depth_frame.has_value() && !current_sensing_volumes.empty()) {
        start = SteadyClock::now();
        AirSimDepthObstacleDetector depth_detector{airsim_depth_obstacle_detector_config_};
        if (timing_writer_) {
            timing_writer_->record_stage("airsim_depth_obstacle_detector.depth_samples", frame->depth_frame->depth_m.size());
            timing_writer_->record_stage("airsim_depth_obstacle_detector.sensing_volumes", current_sensing_volumes.size());
        }
        std::size_t matched_sensing_volumes = 0U;
        std::size_t produced_depth_evidence = 0U;
        for (const auto& sensing_volume : current_sensing_volumes) {
            if (!frame->depth_frame->sensor_name.empty() &&
                !sensing_volume.sensor_name.empty() &&
                frame->depth_frame->sensor_name != sensing_volume.sensor_name) {
                continue;
            }
            ++matched_sensing_volumes;
            const auto depth_evidence = depth_detector.detect(*frame->depth_frame, sensing_volume);
            produced_depth_evidence += depth_evidence.size();
            perception_output.obstacle_evidence.insert(
                perception_output.obstacle_evidence.end(),
                depth_evidence.begin(),
                depth_evidence.end());
        }
        if (timing_writer_) {
            timing_writer_->record_stage("airsim_depth_obstacle_detector.matched_sensing_volumes", matched_sensing_volumes);
            timing_writer_->record_stage("airsim_depth_obstacle_detector.evidence_count", produced_depth_evidence);
            timing_writer_->record_stage("airsim_depth_obstacle_detector.detect", duration_us(start));
        }
    } else if (timing_writer_) {
        timing_writer_->record_stage(
            frame->depth_frame.has_value()
                ? "airsim_depth_obstacle_detector.skipped_no_sensing_volume"
                : "airsim_depth_obstacle_detector.skipped_no_depth_frame",
            current_sensing_volumes.size());
    }

    if (providers_.ghost_targets) {
        if (!ghost_scenario_start_.has_value()) {
            ghost_scenario_start_ = frame->timestamp;
        }
        start = SteadyClock::now();
        const auto ghost_frame = providers_.ghost_targets->frame_at(
            frame->timestamp,
            ego_estimate.ego->map_frame_id,
            *ghost_scenario_start_,
            ego_estimate.ego->local_T_body.position);
        if (timing_writer_) {
            timing_writer_->record_stage("ghost_targets.frame_at", duration_us(start));
        }

        if (ghost_detections_publisher_) {
            start = SteadyClock::now();
            ghost_detections_publisher_->publish(ghost_frame);
            if (timing_writer_) {
                timing_writer_->record_stage("ghost_detections.publish", duration_us(start));
            }
        }

        start = SteadyClock::now();
        perception_output.observations.insert(
            perception_output.observations.end(),
            ghost_frame.observations.begin(),
            ghost_frame.observations.end());
        if (timing_writer_) {
            timing_writer_->record_stage("perception_output.merge_ghost_observations", duration_us(start));
        }
    }

    start = SteadyClock::now();
    providers_.world_model->update_ego(*ego_estimate.ego);
    if (timing_writer_) {
        timing_writer_->record_stage("world_model.update_ego", duration_us(start));
    }

    if (frame->appearance_condition.has_value()) {
        start = SteadyClock::now();
        providers_.world_model->update_appearance(*frame->appearance_condition);
        if (timing_writer_) {
            timing_writer_->record_stage("world_model.update_appearance", duration_us(start));
        }
    }

    if (!current_sensing_volumes.empty()) {
        start = SteadyClock::now();
        // copy: ingest reads configured_obstacle_sensing_volumes_ and calls
        // refresh_ground_truth_obstacle_products which replaces
        // snapshot_.obstacle_sensing_volumes with emulation volumes.
        // The original vector is preserved here so it can be restored below.
        providers_.world_model->update_obstacle_sensing_volumes(current_sensing_volumes);
        if (timing_writer_) {
            timing_writer_->record_stage("world_model.update_obstacle_sensing_volumes.pre_ingest", duration_us(start));
        }
    }

    start = SteadyClock::now();
    providers_.world_model->ingest(perception_output);
    if (timing_writer_) {
        timing_writer_->record_stage("world_model.ingest", duration_us(start));
    }

    if (!current_sensing_volumes.empty()) {
        start = SteadyClock::now();
        // move: restore snapshot_.obstacle_sensing_volumes to the raw camera
        // sensing volumes that ingest overwrote with emulation volumes.
        providers_.world_model->update_obstacle_sensing_volumes(std::move(current_sensing_volumes));
        if (timing_writer_) {
            timing_writer_->record_stage("world_model.update_obstacle_sensing_volumes.post_ingest", duration_us(start));
        }
    }

    start = SteadyClock::now();
    auto snapshot_for_annotation = providers_.world_model->snapshot();
    if (timing_writer_) {
        timing_writer_->record_stage("world_model.snapshot", duration_us(start));
    }

    start = SteadyClock::now();
    const auto mission_local_obstacle_map_snapshot = mission_local_obstacle_map_.update(
        perception_output.obstacle_evidence,
        frame->timestamp,
        snapshot_for_annotation.ego.map_frame_id);
    snapshot_for_annotation.mission_local_obstacle_map = mission_local_obstacle_map_snapshot;
    snapshot_for_annotation.has_mission_local_obstacle_map = true;

    start = SteadyClock::now();
    mission_obstacle_map_artifact_writer_.write_if_due(mission_local_obstacle_map_snapshot);
    if (timing_writer_) {
        timing_writer_->record_stage("mission_obstacle_map_artifact_writer.write_if_due", duration_us(start));
    }

    start = SteadyClock::now();
    auto mission_obstacle_map_delta_batch =
        mission_obstacle_map_delta_writer_.append_if_due(mission_local_obstacle_map_snapshot);
    if (mission_obstacle_map_delta_batch.has_value() && mission_obstacle_map_delta_publisher_) {
        mission_obstacle_map_delta_publisher_->publish(
            MissionObstacleMapDeltaFrame{
                .timestamp_ns = mission_local_obstacle_map_snapshot.summary.last_update_timestamp_ns,
                .json = *mission_obstacle_map_delta_batch});
    }
    if (timing_writer_) {
        timing_writer_->record_stage("mission_obstacle_map_delta_writer.append_if_due", duration_us(start));
        timing_writer_->record_stage(
            "mission_obstacle_map_delta_writer.published_live_batch",
            mission_obstacle_map_delta_batch.has_value() ? 1 : 0);
    }

    if (timing_writer_) {
        timing_writer_->record_stage("mission_local_obstacle_map.update", duration_us(start));
        timing_writer_->record_stage(
            "mission_local_obstacle_map.observed_cells",
            mission_local_obstacle_map_snapshot.summary.observed_cell_count);
        timing_writer_->record_stage(
            "mission_local_obstacle_map.occupied_cells",
            mission_local_obstacle_map_snapshot.summary.occupied_cell_count);
    }

    // Background-assimilate the current cumulative obstacle map into the
    // traversability map so it builds in-flight, not only at landing.
    // Enqueue only when the map has cells; tick() unconditionally (empty
    // queue is a cheap no-op).  finalize_mission_map_after_landing() handles
    // the final enqueue + high-priority flush at shutdown.
    start = SteadyClock::now();
    if (!mission_local_obstacle_map_snapshot.cells.empty()) {
        mission_map_assimilator_.enqueue_mission_obstacle_map(mission_local_obstacle_map_snapshot);
    }
    const auto drained_before = mission_map_assimilator_.status().drained_snapshot_count;
    mission_map_assimilator_.tick(frame->timestamp);
    const bool trav_map_updated =
        mission_map_assimilator_.status().drained_snapshot_count > drained_before;
    if (timing_writer_) {
        timing_writer_->record_stage("mission_map_assimilator.tick", duration_us(start));
        timing_writer_->record_stage(
            "mission_map_assimilator.pending_snapshots",
            mission_map_assimilator_.status().pending_snapshot_count);
        timing_writer_->record_stage(
            "mission_map_assimilator.drained_snapshots",
            mission_map_assimilator_.status().drained_snapshot_count);
    }

    // Slide the L2 in-memory window to follow the drone.
    // No-op when no DB is open or the drone has not moved > horizon_m/4.
    mission_local_planning_map_.slide_window(
        snapshot_for_annotation.ego.local_T_body.position);
    // Newly loaded cells carry their original write_seq (not current map_seq_),
    // so they won't appear in dirty_centers_since().  Force a full ESDF recompute
    // to cover the newly entered window region.
    if (esdf_map_publisher_) { esdf_needs_full_recompute_ = true; }

    // ── L3 eager startup publish ─────────────────────────────────────────────
    // On the very first tick: if L2 (or a persisted ESDF cache) already has cells,
    // compute and publish L3 now — don't wait for a traversability update.
    // This ensures SSE clients always receive the persistent ESDF immediately
    // on connect rather than waiting for the first new obstacle event.
    if (esdf_map_publisher_ && esdf_seq_ == 0U) {
        const Vec3& drone_pos = snapshot_for_annotation.ego.local_T_body.position;
        if (esdf_map_.cell_count() == 0U && mission_local_planning_map_.cell_count() > 0U) {
            // No cache loaded — derive from current L2 window.
            esdf_map_ = compute_esdf(mission_local_planning_map_, drone_pos,
                                     kESDFHorizHalfM, kESDFVertHalfM, kESDFD0M);
            esdf_last_l2_seq_          = mission_local_planning_map_.current_seq();
            esdf_needs_full_recompute_ = false;
        }
        if (esdf_map_.cell_count() > 0U) {
            LocalESDFMapFrame esdf_frame;
            esdf_frame.timestamp_ns      = frame->timestamp.timestamp_ns;
            esdf_frame.snapshot          = esdf_map_.snapshot(drone_pos, 1.0);
            esdf_frame.snapshot.seq      = ++esdf_seq_;
            esdf_frame.snapshot.is_delta = false;  // always full on startup
            esdf_map_publisher_->publish(esdf_frame);
        }
    }

    // When the assimilator drained at least one obstacle-map snapshot this tick:
    //  1. Rebuild the Level 2 planning map from the fresh Level 1 snapshot.
    //  2. Publish Level 1 to SSE subscribers (throttled to once per 2 s).
    static constexpr std::uint64_t kTravPublishMinIntervalNs = 2'000'000'000ULL;
    if (trav_map_updated) {
        start = SteadyClock::now();
        // Take one snapshot and share it between L2 rebuild and SSE publish.
        // No cell cap: the SSE subscriber delta-filters internally, so every
        // cell must be present in the full snapshot.
        auto trav_snapshot = mission_map_assimilator_.traversability_map().snapshot();
        if (timing_writer_) {
            timing_writer_->record_stage(
                "mission_map_assimilator.traversability_snapshot", duration_us(start));
        }

        // ── Level 2: rebuild planning map ───────────────────────────────────
        start = SteadyClock::now();
        mission_local_planning_map_.update_from_traversability(trav_snapshot);
        if (timing_writer_) {
            const auto& pm_stats = mission_local_planning_map_.last_update_stats();
            timing_writer_->record_stage(
                "planning_map.update_from_traversability", duration_us(start));
            timing_writer_->record_stage(
                "planning_map.l1_occupied_merged",
                pm_stats.l1_occupied_merged);
            timing_writer_->record_stage(
                "planning_map.l1_free_applied",
                pm_stats.l1_free_applied);
            timing_writer_->record_stage(
                "planning_map.cells_evicted",
                pm_stats.cells_evicted);
            timing_writer_->record_stage(
                "planning_map.cell_count",
                mission_local_planning_map_.cell_count());
        }

        // ── Level 1 SSE publish (throttled) ─────────────────────────────────
        if (traversability_map_publisher_) {
            const std::uint64_t now_ns =
                mission_map_assimilator_.status().last_update_timestamp_ns;
            const bool throttled =
                last_trav_publish_ns_ != 0U &&
                now_ns != 0U &&
                (now_ns - last_trav_publish_ns_) < kTravPublishMinIntervalNs;
            if (!throttled) {
                start = SteadyClock::now();
                MissionLocalTraversabilityMapFrame trav_frame;
                trav_frame.timestamp_ns =
                    trav_snapshot.summary.last_update_timestamp_ns != 0U
                        ? trav_snapshot.summary.last_update_timestamp_ns
                        : mission_local_obstacle_map_snapshot.summary.last_update_timestamp_ns;
                last_trav_publish_ns_ = trav_frame.timestamp_ns != 0U
                    ? trav_frame.timestamp_ns
                    : now_ns;
                trav_frame.snapshot = std::move(trav_snapshot);
                traversability_map_publisher_->publish(trav_frame);
                if (timing_writer_) {
                    timing_writer_->record_stage(
                        "traversability_map_publisher.publish", duration_us(start));
                }

                // ── Level 2 SSE publish (piggybacks on L1 throttle) ─────────
                // L2 is small (~3-6K cells); publish a full snapshot every time
                // L1 is published.  The timestamp is the same as L1's frame.
                if (planning_map_publisher_) {
                    start = SteadyClock::now();
                    MissionLocalPlanningMapFrame pm_frame;
                    pm_frame.timestamp_ns = trav_frame.timestamp_ns;
                    pm_frame.snapshot = mission_local_planning_map_.snapshot(l2_last_published_seq_);
                    planning_map_publisher_->publish(pm_frame);
                    l2_last_published_seq_ = pm_frame.snapshot.seq;
                    if (timing_writer_) {
                        timing_writer_->record_stage(
                            "planning_map_publisher.publish", duration_us(start));
                    }
                }

                // ── Level 3 ESDF (piggybacks on L2 publish tick) ────────────
                // Three paths, in order of priority:
                //  1. Full recompute  — startup or post-slide_window (~6 ms, rare)
                //  2. Incremental     — dirty L2 cells only via update_incremental (<1 ms)
                //  3. Skip cell update — L2 unchanged; re-snapshot for updated net_rep
                if (esdf_map_publisher_) {
                    start = SteadyClock::now();
                    const Vec3& drone_pos =
                        snapshot_for_annotation.ego.local_T_body.position;
                    const std::uint64_t cur_l2_seq =
                        mission_local_planning_map_.current_seq();
                    bool is_full = false;

                    if (esdf_needs_full_recompute_) {
                        // Path 1: full recompute (startup / slide_window).
                        esdf_map_ = compute_esdf(mission_local_planning_map_,
                                                 drone_pos,
                                                 kESDFHorizHalfM,
                                                 kESDFVertHalfM,
                                                 kESDFD0M);
                        esdf_last_l2_seq_       = cur_l2_seq;
                        esdf_needs_full_recompute_ = false;
                        is_full = true;
                    } else if (cur_l2_seq != esdf_last_l2_seq_) {
                        // Path 2: incremental — update only cells near dirty L2 voxels.
                        auto delta = mission_local_planning_map_.snapshot(esdf_last_l2_seq_);
                        esdf_last_l2_seq_ = cur_l2_seq;
                        if (!delta.cells.empty()) {
                            std::vector<Vec3> dirty;
                            dirty.reserve(delta.cells.size());
                            for (const auto& c : delta.cells) {
                                dirty.push_back(c.center_map);
                            }
                            esdf_map_.update_incremental(
                                mission_local_planning_map_, dirty, kESDFD0M);
                        }
                        // is_full stays false → viewer merges delta cells
                    }
                    // Path 3 (L2 unchanged): fall through — just re-snapshot below.

                    LocalESDFMapFrame esdf_frame;
                    esdf_frame.timestamp_ns          = trav_frame.timestamp_ns;
                    esdf_frame.snapshot              = esdf_map_.snapshot(drone_pos, 1.0);
                    esdf_frame.snapshot.seq          = ++esdf_seq_;
                    esdf_frame.snapshot.is_delta     = !is_full;
                    esdf_map_publisher_->publish(esdf_frame);
                    if (timing_writer_) {
                        timing_writer_->record_stage("esdf.compute_and_publish", duration_us(start));
                        timing_writer_->record_stage("esdf.cell_count",
                            esdf_map_.cell_count());
                    }
                }
            }
        }
    }

    start = SteadyClock::now();
    snapshot_for_annotation.local_flight_map = local_flight_map_accumulator_.update_from_mission_local_map(
        mission_local_obstacle_map_snapshot,
        snapshot_for_annotation.ego.local_T_body,
        frame->timestamp);
    snapshot_for_annotation.has_local_flight_map = true;
    if (timing_writer_) {
        timing_writer_->record_stage("local_flight_map.update_from_mission_local_map", duration_us(start));
    }

    // Rotate map-frame velocity into body frame (yaw-only, NED convention),
    // compute the L0 polar + spherical risk grid, then collect per-sensor
    // observations from the raw evidence list (source-tagged, bearing/elevation intact).
    {
        const double yaw = snapshot_for_annotation.ego.local_T_body.rotation_rpy.z;
        const double vx  = snapshot_for_annotation.ego.velocity_local.x;
        const double vy  = snapshot_for_annotation.ego.velocity_local.y;
        const Vec3 vel_body{
             vx * std::cos(yaw) + vy * std::sin(yaw),
            -vx * std::sin(yaw) + vy * std::cos(yaw),
            0.0
        };
        compute_l0_polar_risk(snapshot_for_annotation.local_flight_map, vel_body);
        collect_l0_sensor_observations(
            snapshot_for_annotation.local_flight_map,
            snapshot_for_annotation.obstacle_evidence,
            vel_body);
    }

    start = SteadyClock::now();
    const auto ego_speed_mps = std::sqrt(
        (snapshot_for_annotation.ego.velocity_local.x * snapshot_for_annotation.ego.velocity_local.x) +
        (snapshot_for_annotation.ego.velocity_local.y * snapshot_for_annotation.ego.velocity_local.y));
    const auto diagnostic_speed_mps = std::max(ego_speed_mps, 2.0);
    const auto diagnostic_trajectory = make_forward_trajectory_samples(diagnostic_speed_mps, 3.0, 0.25);
    snapshot_for_annotation.trajectory_safety =
        trajectory_safety_evaluator_.evaluate(snapshot_for_annotation.local_flight_map, diagnostic_trajectory);
    snapshot_for_annotation.has_trajectory_safety = true;
    if (timing_writer_) {
        timing_writer_->record_stage("trajectory_safety.evaluate", duration_us(start));
    }

    if (snapshot_publisher_) {
        start = SteadyClock::now();
        snapshot_publisher_->publish(std::make_shared<WorldSnapshot>(snapshot_for_annotation));
        if (timing_writer_) {
            timing_writer_->record_stage("snapshot_publisher.publish", duration_us(start));
        }
    }

    AnnotationContext annotation;
    annotation.frame = perception_output.stabilized_frame.frame;
    annotation.perception = perception_output;
    annotation.world_snapshot = snapshot_for_annotation;

    start = SteadyClock::now();
    providers_.frame_annotator->annotate(annotation);
    if (timing_writer_) {
        timing_writer_->record_stage("frame_annotator.annotate", duration_us(start));
        timing_writer_->record_stage("runtime.post_frame_compute", duration_between_us(frame_available_time, SteadyClock::now()));
        timing_writer_->set_measured_total(duration_us(run_once_start));
        timing_writer_->end_frame();
    }

    return true;
}

}  // namespace dedalus
