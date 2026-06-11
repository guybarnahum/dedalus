#include "dedalus/runtime/core_stack_runner.hpp"

#include "dedalus/sensing/airsim_depth_obstacle_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace dedalus {
namespace {

using SteadyClock = std::chrono::steady_clock;

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
        providers_.world_model->update_obstacle_sensing_volumes(std::move(current_sensing_volumes));
        if (timing_writer_) {
            timing_writer_->record_stage("world_model.update_obstacle_sensing_volumes", duration_us(start));
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

    if (timing_writer_) {
        timing_writer_->record_stage("mission_local_obstacle_map.update", duration_us(start));
        timing_writer_->record_stage(
            "mission_local_obstacle_map.observed_cells",
            mission_local_obstacle_map_snapshot.summary.observed_cell_count);
        timing_writer_->record_stage(
            "mission_local_obstacle_map.occupied_cells",
            mission_local_obstacle_map_snapshot.summary.occupied_cell_count);
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
        snapshot_publisher_->publish(snapshot_for_annotation);
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
