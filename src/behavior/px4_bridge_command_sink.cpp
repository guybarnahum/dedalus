#include "dedalus/behavior/flight_command_sinks.hpp"

#include <sys/select.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace dedalus {
namespace {

bool json_bool_value(const std::string& json, const std::string& key) {
    const std::string marker = "\"" + key + "\":";
    const auto pos = json.find(marker);
    if (pos == std::string::npos) {
        return false;
    }
    const auto start = pos + marker.size();
    return json.compare(start, 4U, "true") == 0;
}

std::string json_string_value(const std::string& json, const std::string& key) {
    const std::string marker = "\"" + key + "\":";
    const auto pos = json.find(marker);
    if (pos == std::string::npos) {
        return {};
    }
    const auto open = json.find('"', pos + marker.size());
    if (open == std::string::npos) {
        return {};
    }
    const auto close = json.find('"', open + 1U);
    if (close == std::string::npos) {
        return {};
    }
    return json.substr(open + 1U, close - open - 1U);
}

}  // namespace

struct Px4BridgeCommandSink::Impl {
    explicit Impl(Px4BridgeCommandSinkConfig config)
        : config(std::move(config)) {
        if (this->config.command_duration_s <= 0.0) {
            throw std::invalid_argument("Px4BridgeCommandSink requires positive command_duration_s");
        }
        if (this->config.max_velocity_mps <= 0.0) {
            throw std::invalid_argument("Px4BridgeCommandSink requires positive max_velocity_mps");
        }
        start_bridge();
    }

    ~Impl() {
        try {
            if (child_pid > 0 && write_fd >= 0) {
                const std::string shutdown = "{\"command\":\"shutdown\"}\n";
                (void)::write(write_fd, shutdown.data(), shutdown.size());
            }
        } catch (...) {
        }
        close_fds();
        if (child_pid > 0) {
            int status = 0;
            const pid_t waited = ::waitpid(child_pid, &status, WNOHANG);
            if (waited == 0) {
                ::kill(child_pid, SIGTERM);
                (void)::waitpid(child_pid, &status, 0);
            }
            child_pid = -1;
        }
    }

    Vec3 bounded_velocity(Vec3 velocity) const {
        const double limit = config.max_velocity_mps;
        velocity.x = std::max(-limit, std::min(limit, velocity.x));
        velocity.y = std::max(-limit, std::min(limit, velocity.y));
        velocity.z = std::max(-limit, std::min(limit, velocity.z));
        return velocity;
    }

    void close_fds() {
        if (write_fd >= 0) {
            ::close(write_fd);
            write_fd = -1;
        }
        if (read_fd >= 0) {
            ::close(read_fd);
            read_fd = -1;
        }
    }

    void start_bridge() {
        int stdin_pipe[2] = {-1, -1};
        int stdout_pipe[2] = {-1, -1};
        if (::pipe(stdin_pipe) != 0 || ::pipe(stdout_pipe) != 0) {
            throw std::runtime_error("failed to create PX4 bridge pipes: " + std::string(std::strerror(errno)));
        }

        child_pid = ::fork();
        if (child_pid < 0) {
            throw std::runtime_error("failed to fork PX4 bridge: " + std::string(std::strerror(errno)));
        }

        if (child_pid == 0) {
            ::dup2(stdin_pipe[0], STDIN_FILENO);
            ::dup2(stdout_pipe[1], STDOUT_FILENO);
            ::close(stdin_pipe[0]);
            ::close(stdin_pipe[1]);
            ::close(stdout_pipe[0]);
            ::close(stdout_pipe[1]);

            const std::string command = "exec " + config.bridge_command;
            execl("/bin/sh", "sh", "-lc", command.c_str(), static_cast<char*>(nullptr));
            _exit(127);
        }

        ::close(stdin_pipe[0]);
        ::close(stdout_pipe[1]);
        write_fd = stdin_pipe[1];
        read_fd = stdout_pipe[0];

        const auto ready = read_response();
        if (!json_bool_value(ready, "ok")) {
            throw std::runtime_error("PX4 bridge failed to become ready: " + ready);
        }
        if (config.debug_logging) {
            std::cerr << "dedalus_px4_bridge_sink: ready " << ready << "\n";
        }
    }

    std::string read_response() {
        std::string line;
        std::array<char, 1> byte{};
        while (true) {
            const ssize_t got = ::read(read_fd, byte.data(), 1U);
            if (got < 0) {
                if (errno == EINTR) {
                    continue;
                }
                throw std::runtime_error("failed reading PX4 bridge response: " + std::string(std::strerror(errno)));
            }
            if (got == 0) {
                throw std::runtime_error("PX4 bridge closed stdout");
            }
            if (byte[0] == '\n') {
                return line;
            }
            line.push_back(byte[0]);
        }
    }

    FlightCommandResult send_json(const VelocityCommand& command, const std::string& request) {
        const std::string line = request + "\n";
        std::size_t total_written = 0U;
        while (total_written < line.size()) {
            const ssize_t written = ::write(write_fd, line.data() + total_written, line.size() - total_written);
            if (written < 0) {
                if (errno == EINTR) {
                    continue;
                }
                throw std::runtime_error("failed writing PX4 bridge request: " + std::string(std::strerror(errno)));
            }
            total_written += static_cast<std::size_t>(written);
        }

        const auto response = read_response();
        const bool ok = json_bool_value(response, "ok");
        const auto error = json_string_value(response, "error");
        FlightCommandResult result;
        result.kind = command.kind;
        result.success = ok;
        result.status = ok ? response : (error.empty() ? response : error);
        if (!ok) {
            throw std::runtime_error("PX4 bridge command failed: " + result.status);
        }
        return result;
    }

    std::string request_for(const VelocityCommand& command) const {
        switch (command.kind) {
            case FlightCommandKind::Arm:
                return "{\"command\":\"arm\"}";
            case FlightCommandKind::Takeoff:
                return "{\"command\":\"takeoff\"}";
            case FlightCommandKind::Land:
                return "{\"command\":\"land\"}";
            case FlightCommandKind::Disarm:
                return "{\"command\":\"disarm\"}";
            case FlightCommandKind::Velocity:
            default: {
                const auto v = bounded_velocity(command.velocity_local_mps);
                std::ostringstream out;
                out << "{\"command\":\"velocity\","
                    << "\"vx\":" << v.x << ","
                    << "\"vy\":" << v.y << ","
                    << "\"vz\":" << v.z << ","
                    << "\"duration\":" << config.command_duration_s << "}";
                return out.str();
            }
        }
    }

    Px4BridgeCommandSinkConfig config;
    pid_t child_pid{-1};
    int write_fd{-1};
    int read_fd{-1};
};

Px4BridgeCommandSink::Px4BridgeCommandSink(Px4BridgeCommandSinkConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

Px4BridgeCommandSink::~Px4BridgeCommandSink() = default;

FlightCommandResult Px4BridgeCommandSink::send(const VelocityCommand& command) {
    const auto request = impl_->request_for(command);
    if (impl_->config.debug_logging) {
        const auto v = impl_->bounded_velocity(command.velocity_local_mps);
        std::cerr << "dedalus_px4_bridge_sink: command=" << to_string(command.kind)
                  << " vx=" << v.x
                  << " vy=" << v.y
                  << " vz=" << v.z
                  << " request=" << request
                  << "\n";
    }
    auto result = impl_->send_json(command, request);
    if (impl_->config.debug_logging) {
        std::cerr << "dedalus_px4_bridge_sink: response=" << result.status << "\n";
    }
    return result;
}

}  // namespace dedalus