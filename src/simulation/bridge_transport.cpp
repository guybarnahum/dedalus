#include "dedalus/simulation/bridge_transport.hpp"

#include <array>
#include <cstdio>
#include <stdexcept>
#include <string>

namespace dedalus {
namespace {

std::string read_pipe_to_end(FILE* pipe) {
    std::array<char, 4096> buffer{};
    std::string output;

    while (true) {
        const std::size_t bytes_read = std::fread(buffer.data(), 1U, buffer.size(), pipe);
        if (bytes_read > 0U) {
            output.append(buffer.data(), bytes_read);
        }
        if (bytes_read < buffer.size()) {
            if (std::feof(pipe) != 0) {
                break;
            }
            if (std::ferror(pipe) != 0) {
                throw std::runtime_error("failed while reading bridge command output");
            }
        }
    }

    return output;
}

}  // namespace

struct PipeBridgeTransport::Impl {
    FILE* stream_pipe{nullptr};
};

PipeBridgeTransport::~PipeBridgeTransport() {
    close_stream();
    delete impl_;
    impl_ = nullptr;
}

std::string PipeBridgeTransport::request_once(const std::string& command) {
    FILE* pipe = popen(command.c_str(), "r");
    if (pipe == nullptr) {
        throw std::runtime_error("failed to start bridge command");
    }

    std::string output;
    try {
        output = read_pipe_to_end(pipe);
    } catch (...) {
        (void)pclose(pipe);
        throw;
    }

    const int status = pclose(pipe);
    if (status != 0) {
        throw std::runtime_error("bridge command failed with status " + std::to_string(status));
    }
    if (output.empty()) {
        throw std::runtime_error("bridge command produced no output");
    }

    return output;
}

std::optional<std::string> PipeBridgeTransport::read_stream_line(const std::string& command) {
    if (impl_ == nullptr) {
        impl_ = new Impl{};
    }

    if (impl_->stream_pipe == nullptr) {
        impl_->stream_pipe = popen(command.c_str(), "r");
        if (impl_->stream_pipe == nullptr) {
            throw std::runtime_error("failed to start persistent bridge command");
        }
    }

    std::array<char, 65536> line{};
    if (std::fgets(line.data(), static_cast<int>(line.size()), impl_->stream_pipe) == nullptr) {
        FILE* pipe = impl_->stream_pipe;
        impl_->stream_pipe = nullptr;
        const int status = pclose(pipe);
        if (status != 0) {
            throw std::runtime_error("persistent bridge command exited with status " + std::to_string(status));
        }
        return std::nullopt;
    }

    return std::string{line.data()};
}

void PipeBridgeTransport::close_stream() {
    if (impl_ != nullptr && impl_->stream_pipe != nullptr) {
        (void)pclose(impl_->stream_pipe);
        impl_->stream_pipe = nullptr;
    }
}

std::string SharedMemoryBridgeTransport::request_once(const std::string&) {
    throw std::runtime_error("shared_memory bridge transport is not implemented yet");
}

std::optional<std::string> SharedMemoryBridgeTransport::read_stream_line(const std::string&) {
    throw std::runtime_error("shared_memory bridge transport is not implemented yet");
}

void SharedMemoryBridgeTransport::close_stream() {}

}  // namespace dedalus
