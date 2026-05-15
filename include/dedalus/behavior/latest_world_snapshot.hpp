#pragma once

#include <mutex>
#include <optional>

#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

class LatestWorldSnapshot {
public:
    void publish(WorldSnapshot snapshot) {
        std::lock_guard<std::mutex> lock{mutex_};
        snapshot_ = std::move(snapshot);
    }

    [[nodiscard]] std::optional<WorldSnapshot> latest() const {
        std::lock_guard<std::mutex> lock{mutex_};
        return snapshot_;
    }

private:
    mutable std::mutex mutex_;
    std::optional<WorldSnapshot> snapshot_;
};

}  // namespace dedalus
