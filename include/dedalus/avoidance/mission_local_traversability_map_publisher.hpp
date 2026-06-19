#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "dedalus/avoidance/mission_local_traversability_map.hpp"

namespace dedalus {

// Pre-serialized compact inner JSON payload for one traversability map snapshot.
// The outer stream-line wrapper (type, seq, timestamp_ns) is added by the
// stream server's on_traversability_map_snapshot() handler.
struct MissionLocalTraversabilityMapFrame {
    std::uint64_t timestamp_ns{0U};
    std::string json;  // compact inner JSON: summary + cells (may be cell-capped)
};

class MissionLocalTraversabilityMapSubscriber {
public:
    virtual ~MissionLocalTraversabilityMapSubscriber() = default;
    virtual void on_traversability_map_snapshot(
        const MissionLocalTraversabilityMapFrame& frame) = 0;
};

class MissionLocalTraversabilityMapPublisher {
public:
    void subscribe(std::shared_ptr<MissionLocalTraversabilityMapSubscriber> subscriber);
    void publish(const MissionLocalTraversabilityMapFrame& frame);

private:
    std::mutex mutex_;
    std::vector<std::weak_ptr<MissionLocalTraversabilityMapSubscriber>> subscribers_;
};

// Serialize a traversability map snapshot to compact inner JSON suitable for
// embedding in a MissionLocalTraversabilityMapFrame.  max_cells == 0 means no
// cap.  The format is compatible with the mission_traversability_map_viewer.
std::string to_compact_stream_json(
    const MissionLocalTraversabilityMapSnapshot& snapshot,
    std::size_t max_cells = 4096U);

}  // namespace dedalus
