#pragma once

#include "dedalus/runtime/pubsub.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

class WorldSnapshotSubscriber : public Subscriber<WorldSnapshot> {
public:
    ~WorldSnapshotSubscriber() override = default;

    void on_event(const WorldSnapshot& snapshot) final {
        on_snapshot(snapshot);
    }

    virtual void on_snapshot(const WorldSnapshot& snapshot) = 0;
};

using WorldSnapshotPublisher = Publisher<WorldSnapshot>;

}  // namespace dedalus
