#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>

#include "dedalus/avoidance/mission_local_obstacle_map.hpp"
#include "dedalus/avoidance/mission_local_traversability_map.hpp"
#include "dedalus/core/types.hpp"

namespace dedalus {

enum class MissionMapAssimilationState {
    Idle,
    BackgroundAssimilating,
    PostLandingFinalizing,
    Finalized,
    Partial,
};

struct MissionMapAssimilatorConfig {
    MissionLocalTraversabilityMapConfig traversability_config;

    std::size_t max_pending_snapshots{16U};
    std::size_t max_snapshots_per_background_tick{1U};
    std::size_t max_snapshots_per_high_priority_tick{8U};
    std::size_t max_post_landing_flush_ticks{64U};
};

struct MissionMapAssimilationStatus {
    MissionMapAssimilationState state{MissionMapAssimilationState::Idle};

    std::size_t pending_snapshot_count{0U};
    std::size_t drained_snapshot_count{0U};
    std::size_t dropped_snapshot_count{0U};

    std::uint64_t compacted_generation{0U};
    std::uint64_t last_update_timestamp_ns{0U};

    bool can_forget_raw_evidence{false};
};

struct MissionMapFlushResult {
    bool completed{false};
    bool timed_out{false};
    bool can_forget_raw_evidence{false};

    std::size_t drained_snapshot_count{0U};
    std::size_t pending_snapshot_count{0U};
    std::uint64_t compacted_generation{0U};
};

class MissionMapAssimilator {
public:
    explicit MissionMapAssimilator(MissionMapAssimilatorConfig config = {});

    const MissionMapAssimilatorConfig& config() const noexcept { return config_; }
    const MissionMapAssimilationStatus& status() const noexcept { return status_; }

    const MissionLocalTraversabilityMap& traversability_map() const noexcept { return traversability_map_; }
    MissionLocalTraversabilityMap& mutable_traversability_map() noexcept { return traversability_map_; }

    void enqueue_mission_obstacle_map(const MissionLocalObstacleMapSnapshot& snapshot);

    MissionMapAssimilationStatus tick(TimePoint now);
    MissionMapAssimilationStatus tick_high_priority(TimePoint now);

    MissionMapFlushResult flush_after_landing(TimePoint now);

    void reset();

private:
    MissionMapAssimilationStatus drain_snapshots(
        TimePoint now,
        std::size_t max_snapshots,
        MissionMapAssimilationState active_state);

    MissionMapAssimilatorConfig config_;
    MissionLocalTraversabilityMap traversability_map_;
    std::deque<MissionLocalObstacleMapSnapshot> pending_snapshots_;
    MissionMapAssimilationStatus status_;
};

}  // namespace dedalus
