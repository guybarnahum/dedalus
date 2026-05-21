#include "dedalus/world_model/world_snapshot_publisher.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace dedalus {

void WorldSnapshotPublisher::subscribe(std::shared_ptr<WorldSnapshotSubscriber> subscriber) {
    if (!subscriber) {
        throw std::invalid_argument("WorldSnapshotPublisher::subscribe requires a subscriber");
    }
    std::lock_guard<std::mutex> lock{mutex_};
    subscribers_.push_back(std::move(subscriber));
}

void WorldSnapshotPublisher::publish(const WorldSnapshot& snapshot) {
    std::vector<std::shared_ptr<WorldSnapshotSubscriber>> subscribers;
    {
        std::lock_guard<std::mutex> lock{mutex_};
        auto output = subscribers_.begin();
        for (auto input = subscribers_.begin(); input != subscribers_.end(); ++input) {
            if (auto subscriber = input->lock()) {
                subscribers.push_back(subscriber);
                *output++ = *input;
            }
        }
        subscribers_.erase(output, subscribers_.end());
    }

    for (const auto& subscriber : subscribers) {
        subscriber->on_snapshot(snapshot);
    }
}

}  // namespace dedalus
