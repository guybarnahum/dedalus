#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dedalus {

template <typename EventT>
class EventSubscriber {
public:
    virtual ~EventSubscriber() = default;
    virtual void on_event(const EventT& event) = 0;
};

template <typename EventT>
class EventPublisher final {
public:
    using SubscriberType = EventSubscriber<EventT>;

    void subscribe(std::shared_ptr<SubscriberType> subscriber) {
        if (!subscriber) {
            throw std::invalid_argument("EventPublisher::subscribe requires a subscriber");
        }
        std::lock_guard<std::mutex> lock{mutex_};
        subscribers_.push_back(std::move(subscriber));
    }

    void publish(const EventT& event) {
        std::vector<std::shared_ptr<SubscriberType>> subscribers;
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
            subscriber->on_event(event);
        }
    }

    [[nodiscard]] std::size_t subscriber_count() const {
        std::lock_guard<std::mutex> lock{mutex_};
        std::size_t count = 0;
        for (const auto& subscriber : subscribers_) {
            if (!subscriber.expired()) {
                ++count;
            }
        }
        return count;
    }

private:
    mutable std::mutex mutex_;
    std::vector<std::weak_ptr<SubscriberType>> subscribers_;
};

}  // namespace dedalus
