#pragma once

#include <optional>
#include <string>

namespace dedalus {

class BridgeTransport {
public:
    virtual ~BridgeTransport() = default;

    virtual std::string request_once(const std::string& command) = 0;
    virtual std::optional<std::string> read_stream_line(const std::string& command) = 0;
    virtual void close_stream() = 0;
};

class PipeBridgeTransport final : public BridgeTransport {
public:
    PipeBridgeTransport() = default;
    ~PipeBridgeTransport() override;

    PipeBridgeTransport(const PipeBridgeTransport&) = delete;
    PipeBridgeTransport& operator=(const PipeBridgeTransport&) = delete;

    std::string request_once(const std::string& command) override;
    std::optional<std::string> read_stream_line(const std::string& command) override;
    void close_stream() override;

private:
    struct Impl;
    Impl* impl_{nullptr};
};

class SharedMemoryBridgeTransport final : public BridgeTransport {
public:
    std::string request_once(const std::string& command) override;
    std::optional<std::string> read_stream_line(const std::string& command) override;
    void close_stream() override;
};

}  // namespace dedalus
