#pragma once

#include <atomic>
#include <memory>

#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

// Pure SPSC snapshot holder. set() is called from the perception subscriber
// thread; latest() is called from MissionRuntime's tick thread. The atomic
// shared_ptr provides lock-free handoff under C++20 (std::atomic<shared_ptr>).
// Flight control state is tracked separately in FlightControlStateTracker.
class LatestWorldSnapshot {
public:
    void set(std::shared_ptr<const WorldSnapshot> snapshot) {
        snapshot_.store(std::move(snapshot), std::memory_order_release);
    }

    [[nodiscard]] std::shared_ptr<const WorldSnapshot> latest() const {
        return snapshot_.load(std::memory_order_acquire);
    }

private:
    std::atomic<std::shared_ptr<const WorldSnapshot>> snapshot_;
};

}  // namespace dedalus
