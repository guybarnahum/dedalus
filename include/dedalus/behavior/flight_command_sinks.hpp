#pragma once

#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "dedalus/behavior/mission_controller.hpp"
#include "dedalus/simulation/bridge_transport.hpp"

namespace dedalus {

struct AirSimVelocityCommandSinkConfig {
    std::string host{"127.0.0.1"};
    int rpc_port{41451};
    std::string vehicle_name{"PX4"};
    std::string bridge_command{"python3 simulation/airsim-send-velocity.py"};
    double command_duration_s{0.1};
    double max_velocity_mps{5.0};
};

class AirSimVelocityCommandSink final : public FlightCommandSink {
public:
    explicit AirSimVelocityCommandSink(AirSimVelocityCommandSinkConfig config)
        : AirSimVelocityCommandSink(std::move(config), std::make_unique<PipeBridgeTransport>()) {}

    AirSimVelocityCommandSink(
        AirSimVelocityCommandSinkConfig config,
        std::unique_ptr<BridgeTransport> transport)
        : config_(std::move(config)), transport_(std::move(transport)) {
        if (config_.command_duration_s <= 0.0) {
            throw std::invalid_argument("AirSimVelocityCommandSink requires positive command_duration_s");
        }
        if (config_.max_velocity_mps <= 0.0) {
            throw std::invalid_argument("AirSimVelocityCommandSink requires positive max_velocity_mps");
        }
        if (!transport_) {
            throw std::invalid_argument("AirSimVelocityCommandSink requires a transport");
        }
    }

    void send(const VelocityCommand& command) override {
        const auto output = transport_->request_once(build_command(command));
        if (output.find("OK") == std::string::npos) {
            throw std::runtime_error("AirSim velocity command helper did not report OK: " + output);
        }
    }

private:
    [[nodiscard]] static std::string shell_quote(const std::string& value) {
        std::string quoted = "'";
        for (const char ch : value) {
            if (ch == '\'') {
                quoted += "'\\''";
            } else {
                quoted += ch;
            }
        }
        quoted += "'";
        return quoted;
    }

    [[nodiscard]] Vec3 bounded_velocity(Vec3 velocity) const {
        const double limit = config_.max_velocity_mps;
        velocity.x = std::max(-limit, std::min(limit, velocity.x));
        velocity.y = std::max(-limit, std::min(limit, velocity.y));
        velocity.z = std::max(-limit, std::min(limit, velocity.z));
        return velocity;
    }

    [[nodiscard]] std::string build_command(const VelocityCommand& command) const {
        const Vec3 velocity = bounded_velocity(command.velocity_local_mps);
        std::ostringstream out;
        out << config_.bridge_command
            << " --host " << shell_quote(config_.host)
            << " --rpc-port " << config_.rpc_port
            << " --vehicle-name " << shell_quote(config_.vehicle_name)
            << " --vx " << velocity.x
            << " --vy " << velocity.y
            << " --vz " << velocity.z
            << " --duration " << config_.command_duration_s;
        if (command.yaw_rate_valid) {
            out << " --yaw-rate " << command.yaw_rate_radps;
        }
        return out.str();
    }

    AirSimVelocityCommandSinkConfig config_;
    std::unique_ptr<BridgeTransport> transport_;
};

class NullFlightCommandSink final : public FlightCommandSink {
public:
    void send(const VelocityCommand&) override {}
};

}  // namespace dedalus
