#pragma once

#include <optional>
#include <utility>

namespace dedalus {

// Minimal in-process mailbox used by early tests and single-process pipeline
// runs. Production IPC can later implement the same conceptual bus boundary
// with iceoryx without forcing shared-memory setup into unit tests.
template <typename Message>
class InProcessBus {
public:
    void publish(Message message) {
        latest_ = std::move(message);
    }

    [[nodiscard]] std::optional<Message> latest() const {
        return latest_;
    }

    void clear() {
        latest_.reset();
    }

private:
    std::optional<Message> latest_;
};

}  // namespace dedalus
