#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace dedalus {

// Validates that a bridge_command base string (the config-file value, before
// argument appending) contains only characters safe to pass to popen().
// Throws std::invalid_argument on violation.  Call this on the base command
// only — code-appended shell-quoted arguments (added by build_bridge_command
// etc.) are not subject to this check.
void validate_bridge_base_command(const std::string& command);

// Single-request transport: sends a command and reads the full response.
class OneShotTransport {
public:
    virtual ~OneShotTransport() = default;
    virtual std::string request_once(const std::string& command) = 0;
};

// Persistent-stream transport: opens a long-lived pipe and reads lines or bytes.
class StreamTransport {
public:
    virtual ~StreamTransport() = default;
    // Open the subprocess pipe without reading any data.  Call this to pre-warm
    // the subprocess before the first read_stream_line() / read_stream_bytes()
    // call, giving it time to start up without blocking the caller.
    // Default is a no-op; override in concrete transports that support it.
    virtual void open_stream(const std::string& command) { (void)command; }
    virtual std::optional<std::string> read_stream_line(const std::string& command) = 0;
    virtual std::optional<std::string> read_stream_bytes(const std::string& command, std::size_t byte_count) = 0;
    virtual std::optional<std::vector<std::uint8_t>> read_stream_byte_vector(
        const std::string& command,
        std::size_t byte_count) = 0;
    virtual void close_stream() = 0;
};

// Combined interface for adapters that need both modes (e.g. AirSimFrameSource).
class BridgeTransport : public OneShotTransport, public StreamTransport {
public:
    ~BridgeTransport() override = default;
};

class PipeBridgeTransport final : public BridgeTransport {
public:
    PipeBridgeTransport();
    ~PipeBridgeTransport() override;

    PipeBridgeTransport(const PipeBridgeTransport&) = delete;
    PipeBridgeTransport& operator=(const PipeBridgeTransport&) = delete;

    std::string request_once(const std::string& command) override;
    void open_stream(const std::string& command) override;
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

// Planned stub — not yet implemented. Constructing this transport throws immediately.
// Re-enable when the shared-memory ring-buffer protocol is ready.
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
