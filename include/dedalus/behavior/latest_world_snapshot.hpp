#pragma once

#include <memory>

// C++20 feature-test macro for std::atomic<shared_ptr<T>> (§20.12.6).
// GCC 12+ / libstdc++ defines this; Apple libc++ (Xcode ≤15) does not.
#ifdef __cpp_lib_atomic_shared_ptr
#  include <atomic>
#else
#  include <mutex>
#endif

#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

// Pure SPSC snapshot holder. set() is called from the perception subscriber
// thread; latest() is called from MissionRuntime's tick thread.
//
// On Linux/GCC (EC2 G6 target) the C++20 atomic<shared_ptr> partial
// specialisation is available, giving lock-free load/store semantics.
// On macOS/Apple libc++ the same interface falls back to a mutex so the
// build stays green on both hosts. Flight control state is tracked
// separately in FlightControlStateTracker.
class LatestWorldSnapshot {
public:
    void set(std::shared_ptr<const WorldSnapshot> snapshot) {
#ifdef __cpp_lib_atomic_shared_ptr
        snapshot_.store(std::move(snapshot), std::memory_order_release);
#else
        std::lock_guard<std::mutex> lock{mutex_};
        snapshot_ = std::move(snapshot);
#endif
    }

    [[nodiscard]] std::shared_ptr<const WorldSnapshot> latest() const {
#ifdef __cpp_lib_atomic_shared_ptr
        return snapshot_.load(std::memory_order_acquire);
#else
        std::lock_guard<std::mutex> lock{mutex_};
        return snapshot_;
#endif
    }

private:
#ifdef __cpp_lib_atomic_shared_ptr
    std::atomic<std::shared_ptr<const WorldSnapshot>> snapshot_;
#else
    mutable std::mutex mutex_;
    std::shared_ptr<const WorldSnapshot> snapshot_;
#endif
};

}  // namespace dedalus
