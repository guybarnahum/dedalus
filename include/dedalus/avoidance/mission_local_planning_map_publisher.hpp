#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "dedalus/avoidance/mission_local_planning_map.hpp"

namespace dedalus {

// Raw planning map frame carried to the stream server.
// The writer thread serializes it via serialize_planning_snapshot(),
// keeping the expensive to_compact_stream_json() off the hot path.
struct MissionLocalPlanningMapFrame {
    std::uint64_t timestamp_ns{0U};
    MissionLocalPlanningMapSnapshot snapshot;
};

class MissionLocalPlanningMapSubscriber {
public:
    virtual ~MissionLocalPlanningMapSubscriber() = default;
    virtual void on_planning_map_snapshot(const MissionLocalPlanningMapFrame& frame) = 0;
};

class MissionLocalPlanningMapPublisher {
public:
    void subscribe(std::shared_ptr<MissionLocalPlanningMapSubscriber> subscriber);
    void publish(const MissionLocalPlanningMapFrame& frame);

private:
    std::mutex mutex_;
    std::vector<std::weak_ptr<MissionLocalPlanningMapSubscriber>> subscribers_;
};

// Serialize a planning map snapshot to compact inner JSON suitable for
// embedding in a stream line.  max_cells == 0 means no cap.
// L2 is small (~3-6K cells), so the cap is rarely needed.
std::string to_compact_stream_json(
    const MissionLocalPlanningMapSnapshot& snapshot,
    std::size_t max_cells = 0U);

}  // namespace dedalus
