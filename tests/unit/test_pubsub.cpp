#include "dedalus/runtime/pubsub.hpp"

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct TestEvent {
    int seq{0};
    std::string label;
};

class RecordingSubscriber final : public dedalus::EventSubscriber<TestEvent> {
public:
    void on_event(const TestEvent& event) override {
        seqs.push_back(event.seq);
        labels.push_back(event.label);
    }

    std::vector<int> seqs;
    std::vector<std::string> labels;
};

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void publishes_typed_events_to_multiple_subscribers() {
    dedalus::EventPublisher<TestEvent> publisher;
    auto first = std::make_shared<RecordingSubscriber>();
    auto second = std::make_shared<RecordingSubscriber>();

    publisher.subscribe(first);
    publisher.subscribe(second);
    require(publisher.subscriber_count() == 2U, "subscriber_count should report live subscribers");

    publisher.publish(TestEvent{.seq = 1, .label = "one"});
    publisher.publish(TestEvent{.seq = 2, .label = "two"});

    require(first->seqs == std::vector<int>{1, 2}, "first subscriber missed typed events");
    require(second->seqs == std::vector<int>{1, 2}, "second subscriber missed typed events");
    require(first->labels == std::vector<std::string>{"one", "two"}, "first subscriber labels mismatch");
}

void expired_subscribers_are_pruned_on_publish() {
    dedalus::EventPublisher<TestEvent> publisher;
    auto survivor = std::make_shared<RecordingSubscriber>();
    {
        auto temporary = std::make_shared<RecordingSubscriber>();
        publisher.subscribe(temporary);
        publisher.subscribe(survivor);
        require(publisher.subscriber_count() == 2U, "expected two live subscribers before temporary expires");
    }

    require(publisher.subscriber_count() == 1U, "subscriber_count should ignore expired subscribers");
    publisher.publish(TestEvent{.seq = 7, .label = "survives"});
    require(survivor->seqs == std::vector<int>{7}, "surviving subscriber should receive event after prune");
}

void rejects_null_subscriber() {
    dedalus::EventPublisher<TestEvent> publisher;
    bool rejected = false;
    try {
        publisher.subscribe(nullptr);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "null subscriber should be rejected");
}

}  // namespace

int main() {
    try {
        publishes_typed_events_to_multiple_subscribers();
        expired_subscribers_are_pruned_on_publish();
        rejects_null_subscriber();
    } catch (const std::exception& exc) {
        std::cerr << "test_pubsub failed: " << exc.what() << '\n';
        return 1;
    }
    return 0;
}
