#pragma once

#include <memory>
#include <mutex>

#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

// Pure SPSC snapshot holder. Publishes perception snapshots from the subscriber
// thread; MissionRuntime reads via latest(). Flight control state is tracked
// separately in FlightControlStateTracker.
class LatestWorldSnapshot {
public:
    void publish(WorldSnapshot snapshot) {
        std::lock_guard<std::mutex> lock{mutex_};
        snapshot_ = std::make_shared<const WorldSnapshot>(std::move(snapshot));
    }

    [[nodiscard]] std::shared_ptr<const WorldSnapshot> latest() const {
        std::lock_guard<std::mutex> lock{mutex_};
        return snapshot_;
    }

private:
    mutable std::mutex mutex_;
    std::shared_ptr<const WorldSnapshot> snapshot_;
};

}  // namespace dedalus
