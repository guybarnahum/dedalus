#include "dedalus/behavior/flight_command_sinks.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace dedalus {
namespace {

std::string shell_quote(const std::string& value) {
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

double clamp(double value, double limit) {
    return std::max(-limit, std::min(limit, value));
}

}  // namespace

AirSimVelocityCommandSink::AirSimVelocityCommandSink(AirSimVelocityCommandSinkConfig config)
    : AirSimVelocityCommandSink(std::move(config), std::make_unique<PipeBridgeTransport>()) {}

AirSimVelocityCommandSink::AirSimVelocityCommandSink(
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

Vec3 AirSimVelocityCommandSink::bounded_velocity(Vec3 velocity) const {
    const double limit = config_.max_velocity_mps;
    velocity.x = clamp(velocity.x, limit);
    velocity.y = clamp(velocity.y, limit);
    velocity.z = clamp(velocity.z, limit);
    return velocity;
}

std::string AirSimVelocityCommandSink::build_command(const VelocityCommand& command) const {
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

void AirSimVelocityCommandSink::send(const VelocityCommand& command) {
    const auto output = transport_->request_once(build_command(command));
    if (output.find("OK") == std::string::npos) {
        throw std::runtime_error("AirSim velocity command helper did not report OK: " + output);
    }
}

}  // namespace dedalus
