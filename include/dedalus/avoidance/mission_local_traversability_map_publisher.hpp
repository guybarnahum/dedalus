#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "dedalus/avoidance/mission_local_traversability_map.hpp"

namespace dedalus {

// Raw traversability map snapshot carried to the stream server.
// The writer thread serializes it to JSON via serialize_traversability_snapshot(),
// keeping the expensive to_compact_stream_json() off the hot path.
struct MissionLocalTraversabilityMapFrame {
    std::uint64_t timestamp_ns{0U};
    MissionLocalTraversabilityMapSnapshot snapshot;
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
// embedding in a stream line.  max_cells == 0 means no cap.
// The format is compatible with the mission_traversability_map_viewer.
std::string to_compact_stream_json(
    const MissionLocalTraversabilityMapSnapshot& snapshot,
    std::size_t max_cells = 4096U);

}  // namespace dedalus
