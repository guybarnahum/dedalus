#pragma once

#include <memory>

#include "dedalus/runtime/pubsub.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

class WorldSnapshotSubscriber : public EventSubscriber<std::shared_ptr<const WorldSnapshot>> {
public:
    ~WorldSnapshotSubscriber() override = default;

    void on_event(const std::shared_ptr<const WorldSnapshot>& snapshot) final {
        on_snapshot(snapshot);
    }

    virtual void on_snapshot(const std::shared_ptr<const WorldSnapshot>& snapshot) = 0;
};

using WorldSnapshotPublisher = EventPublisher<std::shared_ptr<const WorldSnapshot>>;

}  // namespace dedalus
