#include "dedalus/runtime/core_stack_runner.hpp"

#include "dedalus/avoidance/local_esdf_map.hpp"
#include "dedalus/avoidance/local_esdf_map_publisher.hpp"
#include "dedalus/avoidance/local_flight_map.hpp"
#include "dedalus/avoidance/mission_local_planning_map_publisher.hpp"
#include "dedalus/avoidance/mission_local_traversability_map_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <unordered_set>
#include <vector>

namespace dedalus {
namespace {

using SteadyClock = std::chrono::steady_clock;

// Compute the fraction of slot-A evidence voxels that have a slot-B voxel
// within a ±1-voxel neighborhood.  Returns 0 if either set is empty.
// O(|B|×27 + |A|) time, O(|B|×27) space.
float compute_depth_agreement(
    const std::vector<ObstacleEvidence>& a,
    const std::vector<ObstacleEvidence>& b,
    float voxel_size_m) {
    if (a.empty() || b.empty()) return 0.0F;

    const float inv_v = (voxel_size_m > 0.0F) ? (1.0F / voxel_size_m) : 1.0F;
    static constexpr std::uint64_t kOffset = 1U << 20U;  // bias 21-bit fields
    auto pack = [](std::uint64_t x, std::uint64_t y, std::uint64_t z) -> std::uint64_t {
        return (x << 42U) | (y << 21U) | z;
    };

    std::unordered_set<std::uint64_t> b_set;
    b_set.reserve(b.size() * 27U);
    for (const auto& ev : b) {
        const auto bx = static_cast<std::int64_t>(std::round(static_cast<float>(ev.center_local.x) * inv_v));
        const auto by = static_cast<std::int64_t>(std::round(static_cast<float>(ev.center_local.y) * inv_v));
        const auto bz = static_cast<std::int64_t>(std::round(static_cast<float>(ev.center_local.z) * inv_v));
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz)
                    b_set.insert(pack(
                        static_cast<std::uint64_t>(bx + dx) + kOffset,
                        static_cast<std::uint64_t>(by + dy) + kOffset,
                        static_cast<std::uint64_t>(bz + dz) + kOffset));
    }

    std::uint32_t matched = 0U;
    for (const auto& ev : a) {
        const auto ax = static_cast<std::uint64_t>(
            static_cast<std::int64_t>(std::round(static_cast<float>(ev.center_local.x) * inv_v)) + kOffset);
        const auto ay = static_cast<std::uint64_t>(
            static_cast<std::int64_t>(std::round(static_cast<float>(ev.center_local.y) * inv_v)) + kOffset);
        const auto az = static_cast<std::uint64_t>(
            static_cast<std::int64_t>(std::round(static_cast<float>(ev.center_local.z) * inv_v)) + kOffset);
        if (b_set.count(pack(ax, ay, az)) > 0U) ++matched;
    }

    return static_cast<float>(matched) / static_cast<float>(a.size());
}

// L3 truncation radius: only cells within d0 of an obstacle surface are stored.
// No fixed XY/Z window — L3 covers the full extent of whatever L2 has in memory.
static constexpr double kESDFD0M = 5.0;
// L3 coarse output spacing: EDT runs at fine (1 m) resolution; output cells are
// stored every kESDFSampleSpacingM metres, giving ~4× fewer cells than dense L3.
static constexpr double kESDFSampleSpacingM = 2.0;

// Derive the ESDF computation window from the actual in-memory L2 extent.
// Returns false (and leaves *out unchanged) when L2 is empty.
// Adds a d0 margin on every axis so shell cells at the L2 boundary aren't clipped.
struct ESDFWindow { Vec3 centre; double horiz_half; double vert_half; };
static bool esdf_window_from_l2(const MissionLocalPlanningMap& l2,
                                 double d0_m, ESDFWindow* out) {
    const auto ext = l2.extent();
    if (!ext) return false;
    out->centre = Vec3{
        (ext->min.x + ext->max.x) * 0.5,
        (ext->min.y + ext->max.y) * 0.5,
        (ext->min.z + ext->max.z) * 0.5,
    };
    // Use the larger of X/Y extents so the EDT grid is square in the horizontal
    // plane (avoids asymmetric clipping on non-square L2 footprints).
    out->horiz_half = std::max(ext->max.x - ext->min.x,
                               ext->max.y - ext->min.y) * 0.5 + d0_m;
    out->vert_half  = (ext->max.z - ext->min.z) * 0.5 + d0_m;
    return true;
}

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
    // DEDALUS_MISSION_DEBUG=1 — print each stage name + ms since run_once() entry.
    // Set this to isolate hangs: the last printed stage is where execution is stuck.
    const bool mdebug = (std::getenv("DEDALUS_MISSION_DEBUG") != nullptr);
    auto mdebug_print = [&](const char* label) {
        if (!mdebug) return;
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            SteadyClock::now() - run_once_start).count();
        std::fprintf(stderr, "[MissionDebug] %-54s %5ld ms\n", label, static_cast<long>(ms));
        std::fflush(stderr);
    };

    // ── Startup publish of persisted L2 and L3 maps ──────────────────────────
    // Runs on the very first call, before blocking on the frame source.
    // open_db() pre-loads all SQLite cells so cell_count() is valid immediately.
    // Publishing here (not after next_frame()) means the SSE cache is populated
    // even when no flight is active and frames never arrive.
    if (esdf_seq_ == 0U) {
        // If no ESDF binary was loaded from disk but L2 has cells, compute L3.
        if (esdf_map_publisher_ &&
            esdf_map_.cell_count() == 0U &&
            mission_local_planning_map_.cell_count() > 0U) {
            ESDFWindow w;
            if (esdf_window_from_l2(mission_local_planning_map_, kESDFD0M, &w)) {
                esdf_map_ = compute_esdf(mission_local_planning_map_,
                                         w.centre, w.horiz_half, w.vert_half,
                                         kESDFD0M, kESDFSampleSpacingM);
                esdf_last_l2_seq_          = mission_local_planning_map_.current_seq();
                esdf_needs_full_recompute_ = false;
            }
        }
        // Publish full L2 snapshot so connecting viewers see the persisted map.
        if (planning_map_publisher_ && mission_local_planning_map_.cell_count() > 0U) {
            MissionLocalPlanningMapFrame pm_frame;
            pm_frame.timestamp_ns      = 0U;
            pm_frame.snapshot          = mission_local_planning_map_.snapshot(0U);
            pm_frame.snapshot.is_delta = false;
            planning_map_publisher_->publish(pm_frame);
            l2_last_published_seq_ = pm_frame.snapshot.seq;
        }
        // Publish full L3 snapshot (cells from disk load or just computed above).
        if (esdf_map_publisher_ && esdf_map_.cell_count() > 0U) {
            LocalESDFMapFrame esdf_frame;
            esdf_frame.timestamp_ns      = 0U;
            esdf_frame.snapshot          = esdf_map_.snapshot(Vec3{0.0, 0.0, 0.0}, 1.0);
            esdf_frame.snapshot.seq      = ++esdf_seq_;
            esdf_frame.snapshot.is_delta = false;
            esdf_map_publisher_->publish(esdf_frame);
        }
    }

    mdebug_print("→ frame_source.next_frame");
    auto start = SteadyClock::now();
    auto frame = providers_.frame_source->next_frame();
    const auto frame_available_time = SteadyClock::now();
    mdebug_print("✓ frame_source.next_frame");
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

    mdebug_print("→ ego_provider.estimate");
    start = SteadyClock::now();
    const auto ego_estimate = providers_.ego_provider->estimate(*frame);
    if (timing_writer_) {
        timing_writer_->record_stage("ego_provider.estimate", duration_us(start));
    }

    // DEDALUS_DEBUG_EGO: uniform debug output for both primary and reference ego
    // providers.  Both slots use an identical format — set DEDALUS_DEBUG_EGO=1 and
    // grep '[EgoDebug:ego_a]' vs '[EgoDebug:ego_b]' to compare them directly.
    const bool ego_debug = (std::getenv("DEDALUS_DEBUG_EGO") != nullptr);
    auto log_ego = [&](const char* tag, const EgoStateEstimate& est) {
        if (!ego_debug) return;
        if (est.ego.has_value()) {
            const auto& p = est.ego->local_T_body.position;
            const auto& v = est.ego->velocity_local;
            std::fprintf(stderr,
                "[EgoDebug:%s] pos=(%.3f,%.3f,%.3f) vel=(%.3f,%.3f,%.3f) "
                "yaw=%.3f h=%.2f conf=%.2f telemetry=%d\n",
                tag,
                p.x, p.y, p.z, v.x, v.y, v.z,
                est.ego->local_T_body.rotation_rpy.z,
                est.ego->height_m,
                static_cast<double>(est.confidence),
                static_cast<int>(est.telemetry_available));
        } else {
            std::fprintf(stderr,
                "[EgoDebug:%s] EMPTY — frame will be dropped\n", tag);
        }
    };
    log_ego("ego_a", ego_estimate);

    // Slot B ego (reference): runs on same frame, agreement logged only.
    if (ego_provider_reference_) {
        const auto b_ego = ego_provider_reference_->estimate(*frame);
        log_ego("ego_b", b_ego);
        if (timing_writer_) {
            const float ag = ego_agreement(ego_estimate, b_ego);
            timing_writer_->record_stage("ego_provider.slot.agreement_ppt",
                static_cast<std::int64_t>(ag * 1000.0F));
        }
    }
    if (!ego_estimate.ego.has_value()) {
        if (timing_writer_) {
            timing_writer_->record_stage("runtime.post_frame_compute", duration_between_us(frame_available_time, SteadyClock::now()));
            timing_writer_->set_measured_total(duration_us(run_once_start));
            timing_writer_->end_frame();
        }
        return false;
    }

    SensingCoverageSnapshot coverage;
    std::vector<ObstacleSensingVolume> current_sensing_volumes;
    if (!providers_.obstacle_sensing_cameras.empty()) {
        start = SteadyClock::now();
        coverage = sensing_coverage_provider_.snapshot({*frame}, *ego_estimate.ego, camera_pointing_states_);
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

    mdebug_print("→ perception_pipeline.process");
    start = SteadyClock::now();
    auto perception_output = pipeline.process(*frame, *ego_estimate.ego);
    if (timing_writer_) {
        timing_writer_->record_stage("perception_pipeline.process", duration_us(start));
    }

    // EP1: Perception pipeline reference slot evaluation.
    //
    // Each reference provider (slot B) receives the same primary-slot inputs as slot A.
    // Outputs are never fed downstream — agreement is logged only.
    // Zero overhead when all reference slots are null.
    if (timing_writer_) {
        if (detector_reference_) {
            start = SteadyClock::now();
            const auto b_dets = detector_reference_->detect(*frame);
            timing_writer_->record_stage("detector.slot.detect", duration_us(start));
            timing_writer_->record_stage("detector.slot.b_count",
                static_cast<std::int64_t>(b_dets.size()));
            const float ag = detection_agreement(perception_output.detections, b_dets);
            timing_writer_->record_stage("detector.slot.agreement_ppt",
                static_cast<std::int64_t>(ag * 1000.0F));
        }

        if (stabilizer_reference_) {
            start = SteadyClock::now();
            const auto b_stab = stabilizer_reference_->stabilize(
                *frame, perception_output.detections);
            timing_writer_->record_stage("stabilizer.slot.stabilize", duration_us(start));
            const StabilizerOutput a_out{
                perception_output.stabilized_frame.transform_available,
                perception_output.stabilized_frame.dx_px,
                perception_output.stabilized_frame.dy_px};
            const StabilizerOutput b_out{
                b_stab.transform_available, b_stab.dx_px, b_stab.dy_px};
            const float ag = stabilizer_agreement(a_out, b_out);
            timing_writer_->record_stage("stabilizer.slot.agreement_ppt",
                static_cast<std::int64_t>(ag * 1000.0F));
        }

        if (tracker_reference_) {
            start = SteadyClock::now();
            const auto b_tracks = tracker_reference_->update(perception_output.detections);
            timing_writer_->record_stage("tracker.slot.update", duration_us(start));
            const float ag = tracker_agreement(perception_output.tracks, b_tracks);
            timing_writer_->record_stage("tracker.slot.agreement_ppt",
                static_cast<std::int64_t>(ag * 1000.0F));
        }

        if (identity_resolver_reference_) {
            start = SteadyClock::now();
            const auto b_ids = identity_resolver_reference_->resolve(perception_output.tracks);
            timing_writer_->record_stage("identity_resolver.slot.resolve", duration_us(start));
            const float ag = identity_agreement(perception_output.identities, b_ids);
            timing_writer_->record_stage("identity_resolver.slot.agreement_ppt",
                static_cast<std::int64_t>(ag * 1000.0F));
        }

        if (projector_reference_) {
            start = SteadyClock::now();
            const auto b_obs = projector_reference_->project(
                perception_output.tracks, *frame, *ego_estimate.ego);
            timing_writer_->record_stage("projector.slot.project", duration_us(start));
            const float ag = observation_agreement(perception_output.observations, b_obs);
            timing_writer_->record_stage("projector.slot.agreement_ppt",
                static_cast<std::int64_t>(ag * 1000.0F));
        }
    }

    // Two-slot depth provider loop.
    //
    // Slot A (primary): evidence appended to perception_output.obstacle_evidence → L1/L2 map.
    // Slot B (reference): evidence collected separately for agreement metric only.
    // Agreement = fraction of slot-A voxels with a slot-B voxel within ±1 voxel.
    if (!coverage.camera_volumes.empty() && depth_slot_a_) {
        mdebug_print("→ depth_slot_a.detect");
        start = SteadyClock::now();

        std::vector<ObstacleEvidence> slot_a_evidence;
        std::vector<ObstacleEvidence> slot_b_evidence;

        for (const auto& csv : coverage.camera_volumes) {
            EgoSensingFrame ego_sf;
            ego_sf.frame          = *frame;
            ego_sf.ego            = *ego_estimate.ego;
            ego_sf.sensing_volume = csv;

            const auto a_ev = depth_slot_a_->detect(ego_sf);
            slot_a_evidence.insert(slot_a_evidence.end(), a_ev.begin(), a_ev.end());

            if (depth_slot_b_) {
                const auto b_ev = depth_slot_b_->detect(ego_sf);
                slot_b_evidence.insert(slot_b_evidence.end(), b_ev.begin(), b_ev.end());
            }
        }

        mdebug_print("✓ depth_slot_a.detect");
        perception_output.obstacle_evidence.insert(
            perception_output.obstacle_evidence.end(),
            slot_a_evidence.begin(),
            slot_a_evidence.end());

        if (timing_writer_) {
            timing_writer_->record_stage("depth_slot_a.sensing_volumes",
                static_cast<std::int64_t>(coverage.camera_volumes.size()));
            timing_writer_->record_stage("depth_slot_a.evidence_count",
                static_cast<std::int64_t>(slot_a_evidence.size()));
            timing_writer_->record_stage("depth_slot_a.detect", duration_us(start));

            if (depth_slot_b_) {
                timing_writer_->record_stage("depth_slot_b.evidence_count",
                    static_cast<std::int64_t>(slot_b_evidence.size()));

                // Fraction of primary-slot voxels confirmed by eval-slot within ±1 voxel.
                // Logged as parts-per-thousand (0–1000).
                static constexpr float kVoxelSizeM = 0.5F;
                const float agreement = compute_depth_agreement(slot_a_evidence, slot_b_evidence, kVoxelSizeM);
                timing_writer_->record_stage("depth.voxel_overlap_ppt",
                    static_cast<std::int64_t>(agreement * 1000.0F));
            }
        }
    } else if (timing_writer_) {
        timing_writer_->record_stage("depth_slot_a.skipped_no_sensing_volume",
            static_cast<std::int64_t>(0));
    }

    if (providers_.ghost_targets) {
        if (!ghost_scenario_start_.has_value()) {
            ghost_scenario_start_ = frame->timestamp;
        }
        mdebug_print("→ ghost_targets.frame_at");
        start = SteadyClock::now();
        const auto ghost_frame = providers_.ghost_targets->frame_at(
            frame->timestamp,
            ego_estimate.ego->map_frame_id,
            *ghost_scenario_start_,
            ego_estimate.ego->local_T_body.position);
        mdebug_print("✓ ghost_targets.frame_at");
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

    mdebug_print("→ world_model.update_ego+ingest");
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

    mdebug_print("→ world_model.snapshot+L1/L2");
    start = SteadyClock::now();
    auto snapshot_for_annotation = providers_.world_model->snapshot();
    snapshot_for_annotation.depth_source_name      = depth_slot_a_name_;
    snapshot_for_annotation.ego_provider_name      = ego_provider_name_;
    snapshot_for_annotation.detector_name          = detector_name_;
    snapshot_for_annotation.camera_stabilizer_name = camera_stabilizer_name_;
    snapshot_for_annotation.tracker_name           = tracker_name_;
    snapshot_for_annotation.identity_resolver_name = identity_resolver_name_;
    snapshot_for_annotation.projector_name         = projector_name_;
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
                    bool is_full    = false;
                    bool esdf_changed = false;

                    if (esdf_needs_full_recompute_) {
                        // Path 1: full recompute (startup / slide_window / new cells).
                        // Window is derived from L2's actual in-memory extent so L3
                        // always covers exactly what L2 has — no artificial crop.
                        ESDFWindow w;
                        if (esdf_window_from_l2(mission_local_planning_map_, kESDFD0M, &w)) {
                            esdf_map_ = compute_esdf(mission_local_planning_map_,
                                                     w.centre, w.horiz_half, w.vert_half,
                                                     kESDFD0M, kESDFSampleSpacingM);
                        }
                        esdf_last_l2_seq_          = cur_l2_seq;
                        esdf_needs_full_recompute_ = false;
                        is_full      = true;
                        esdf_changed = true;
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
                        esdf_changed = true;
                    }
                    // Path 3 (L2 unchanged): fall through — just re-snapshot below.

                    // Stage ESDF for DB flush (paths 1 + 2 only).
                    if (esdf_changed && !planning_map_persistence_path_.empty() &&
                        esdf_map_.cell_count() > 0U) {
                        const auto snap_cells =
                            esdf_map_.snapshot(Vec3{0.0, 0.0, 0.0}, 0.0).cells;
                        std::vector<MissionLocalPlanningMap::ESDFCellRecord> records;
                        records.reserve(snap_cells.size());
                        for (const auto& c : snap_cells) {
                            records.push_back({
                                c.centre.x, c.centre.y, c.centre.z,
                                static_cast<double>(c.d),
                                c.grad.x,  c.grad.y,  c.grad.z,
                                c.sgrad.x, c.sgrad.y, c.sgrad.z,
                            });
                        }
                        std::lock_guard<std::mutex> lk(esdf_flush_mutex_);
                        esdf_flush_cells_   = std::move(records);
                        esdf_flush_d0_m_    = kESDFD0M;
                        esdf_flush_pending_ = true;
                    }

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

    if (++perch_cadence_tick_ % static_cast<std::uint32_t>(
            perch_candidate_evaluator_.cadence_ticks()) == 0U) {
        start = SteadyClock::now();
        cached_perch_candidates_ =
            perch_candidate_evaluator_.evaluate(snapshot_for_annotation.obstacle_evidence,
                                                &mission_local_planning_map_);
        if (timing_writer_) {
            timing_writer_->record_stage("perch_candidate_evaluator.evaluate", duration_us(start));
        }
    }
    snapshot_for_annotation.perch_candidates = cached_perch_candidates_;

    mdebug_print("→ snapshot_publisher.publish");
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

    mdebug_print("→ frame_annotator.annotate");
    start = SteadyClock::now();
    providers_.frame_annotator->annotate(annotation);
    if (timing_writer_) {
        timing_writer_->record_stage("frame_annotator.annotate", duration_us(start));
        timing_writer_->record_stage("runtime.post_frame_compute", duration_between_us(frame_available_time, SteadyClock::now()));
        timing_writer_->set_measured_total(duration_us(run_once_start));
        timing_writer_->end_frame();
    }

    mdebug_print("✓ run_once complete");
    return true;
}

}  // namespace dedalus
