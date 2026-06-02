#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace dedalus {

class BridgeTransport {
public:
    virtual ~BridgeTransport() = default;

    virtual std::string request_once(const std::string& command) = 0;
    virtual std::optional<std::string> read_stream_line(const std::string& command) = 0;
    virtual std::optional<std::string> read_stream_bytes(const std::string& command, std::size_t byte_count) = 0;
    virtual std::optional<std::vector<std::uint8_t>> read_stream_byte_vector(
        const std::string& command,
        std::size_t byte_count) = 0;
    virtual void close_stream() = 0;
};

class PipeBridgeTransport final : public BridgeTransport {
public:
    PipeBridgeTransport();
    ~PipeBridgeTransport() override;

    PipeBridgeTransport(const PipeBridgeTransport&) = delete;
    PipeBridgeTransport& operator=(const PipeBridgeTransport&) = delete;

    std::string request_once(const std::string& command) override;
    std::optional<std::string> read_stream_line(const std::string& command) override;
    std::optional<std::string> read_stream_bytes(const std::string& command, std::size_t byte_count) override;
    std::optional<std::vector<std::uint8_t>> read_stream_byte_vector(
        const std::string& command,
        std::size_t byte_count) override;
    void close_stream() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class SharedMemoryBridgeTransport final : public BridgeTransport {
public:
    std::string request_once(const std::string& command) override;
    std::optional<std::string> read_stream_line(const std::string& command) override;
    std::optional<std::string> read_stream_bytes(const std::string& command, std::size_t byte_count) override;
    std::optional<std::vector<std::uint8_t>> read_stream_byte_vector(
        const std::string& command,
        std::size_t byte_count) override;
    void close_stream() override;
};

}  // namespace dedalus
