#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

class WorldSnapshotSubscriber {
public:
    virtual ~WorldSnapshotSubscriber() = default;
    virtual void on_snapshot(const WorldSnapshot& snapshot) = 0;
};

class WorldSnapshotPublisher final {
public:
    void subscribe(std::shared_ptr<WorldSnapshotSubscriber> subscriber);
    void publish(const WorldSnapshot& snapshot);

private:
    mutable std::mutex mutex_;
    std::vector<std::weak_ptr<WorldSnapshotSubscriber>> subscribers_;
};

}  // namespace dedalus
