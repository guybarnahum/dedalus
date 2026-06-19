#include "dedalus/avoidance/mission_map_assimilator.hpp"

#include <algorithm>

namespace dedalus {
namespace {

std::uint64_t timestamp_ns(const TimePoint& time) {
    return time.timestamp_ns > 0 ? static_cast<std::uint64_t>(time.timestamp_ns) : 0U;
}

}  // namespace

MissionMapAssimilator::MissionMapAssimilator(MissionMapAssimilatorConfig config)
    : config_(config),
      traversability_map_(config_.traversability_config) {
    if (config_.max_pending_snapshots == 0U) {
        config_.max_pending_snapshots = 1U;
    }
    if (config_.max_snapshots_per_background_tick == 0U) {
        config_.max_snapshots_per_background_tick = 1U;
    }
    if (config_.max_snapshots_per_high_priority_tick == 0U) {
        config_.max_snapshots_per_high_priority_tick = config_.max_snapshots_per_background_tick;
    }
    if (config_.max_post_landing_flush_ticks == 0U) {
        config_.max_post_landing_flush_ticks = 1U;
    }
}

void MissionMapAssimilator::enqueue_mission_obstacle_map(
    const MissionLocalObstacleMapSnapshot& snapshot) {
    while (pending_snapshots_.size() >= config_.max_pending_snapshots) {
        pending_snapshots_.pop_front();
        ++status_.dropped_snapshot_count;
    }
    pending_snapshots_.push_back(snapshot);
    status_.pending_snapshot_count = pending_snapshots_.size();
    status_.can_forget_raw_evidence = false;
    if (status_.state == MissionMapAssimilationState::Finalized) {
        status_.state = MissionMapAssimilationState::Idle;
    }
}

MissionMapAssimilationStatus MissionMapAssimilator::tick(const TimePoint now) {
    return drain_snapshots(
        now,
        config_.max_snapshots_per_background_tick,
        MissionMapAssimilationState::BackgroundAssimilating,
        /*include_clearance=*/false);
}

MissionMapAssimilationStatus MissionMapAssimilator::tick_high_priority(const TimePoint now) {
    return drain_snapshots(
        now,
        config_.max_snapshots_per_high_priority_tick,
        MissionMapAssimilationState::PostLandingFinalizing,
        /*include_clearance=*/true);
}

MissionMapAssimilationStatus MissionMapAssimilator::drain_snapshots(
    const TimePoint now,
    const std::size_t max_snapshots,
    const MissionMapAssimilationState active_state,
    const bool include_clearance) {
    status_.state = pending_snapshots_.empty() ? MissionMapAssimilationState::Idle : active_state;

    const auto limit = std::min(max_snapshots, pending_snapshots_.size());
    for (std::size_t i = 0U; i < limit; ++i) {
        const auto snapshot = pending_snapshots_.front();
        pending_snapshots_.pop_front();
        traversability_map_.update_from_mission_obstacle_map(snapshot, now, include_clearance);
        ++status_.drained_snapshot_count;
        ++status_.compacted_generation;
    }

    status_.pending_snapshot_count = pending_snapshots_.size();
    status_.last_update_timestamp_ns = timestamp_ns(now);
    status_.can_forget_raw_evidence = pending_snapshots_.empty();

    if (pending_snapshots_.empty() && active_state == MissionMapAssimilationState::PostLandingFinalizing) {
        status_.state = MissionMapAssimilationState::Finalized;
    } else if (pending_snapshots_.empty()) {
        status_.state = MissionMapAssimilationState::Idle;
    }

    return status_;
}

MissionMapFlushResult MissionMapAssimilator::flush_after_landing(const TimePoint now) {
    status_.state = MissionMapAssimilationState::PostLandingFinalizing;
    const auto drained_before = status_.drained_snapshot_count;

    std::size_t tick_count = 0U;
    while (!pending_snapshots_.empty() && tick_count < config_.max_post_landing_flush_ticks) {
        tick_high_priority(now);
        ++tick_count;
    }

    MissionMapFlushResult result;
    result.completed = pending_snapshots_.empty();
    result.timed_out = !result.completed;
    result.can_forget_raw_evidence = result.completed;
    result.drained_snapshot_count = status_.drained_snapshot_count - drained_before;
    result.pending_snapshot_count = pending_snapshots_.size();
    result.compacted_generation = status_.compacted_generation;

    status_.pending_snapshot_count = pending_snapshots_.size();
    status_.can_forget_raw_evidence = result.can_forget_raw_evidence;
    status_.state = result.completed ? MissionMapAssimilationState::Finalized : MissionMapAssimilationState::Partial;

    return result;
}

void MissionMapAssimilator::reset() {
    pending_snapshots_.clear();
    traversability_map_.reset();
    status_ = MissionMapAssimilationStatus{};
}

}  // namespace dedalus
