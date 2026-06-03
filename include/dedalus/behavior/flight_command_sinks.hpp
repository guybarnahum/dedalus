#pragma once

#include <algorithm>
#include <cstdint>
#include <iostream>
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
    std::string bridge_command{"python3 simulation/airsim/scripts/airsim-send-velocity.py"};
    double command_duration_s{0.1};
    double max_velocity_mps{5.0};
    bool debug_logging{false};
};

class AirSimVelocityCommandSink final : public FlightCommandSink {
public:
    explicit AirSimVelocityCommandSink(AirSimVelocityCommandSinkConfig config)
        : AirSimVelocityCommandSink(std::move(config), std::make_unique<PipeBridgeTransport>()) {}

    AirSimVelocityCommandSink(
        AirSimVelocityCommandSinkConfig config,
        std::unique_ptr<OneShotTransport> transport)
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

    FlightCommandResult send(const VelocityCommand& command) override {
        const auto rendered_command = build_command(command);
        if (config_.debug_logging) {
            const auto v = bounded_velocity(command.velocity_local_mps);
            std::cerr << "dedalus_flight_sink: command=" << to_string(command.kind)
                      << " vx=" << v.x
                      << " vy=" << v.y
                      << " vz=" << v.z
                      << " duration_s=" << config_.command_duration_s
                      << " helper=" << rendered_command
                      << "\n";
        }

        const auto output = transport_->request_once(rendered_command);
        if (config_.debug_logging) {
            std::cerr << "dedalus_flight_sink: helper_output=" << output;
            if (output.empty() || output.back() != '\n') {
                std::cerr << "\n";
            }
        }

        FlightCommandResult result;
        result.kind = command.kind;
        result.status = output;
        result.success = json_ok_value(output);
        if (!result.success) {
            throw std::runtime_error("AirSim command helper did not report ok: " + output);
        }
        return result;
    }

private:
    // Returns true when the JSON response contains "ok":true.
    [[nodiscard]] static bool json_ok_value(const std::string& json) {
        const auto pos = json.find("\"ok\":");
        if (pos == std::string::npos) {
            return false;
        }
        const auto start = pos + 5U;
        return json.compare(start, 4U, "true") == 0;
    }

    [[nodiscard]] static std::string shell_quote(const std::string& value) {        std::string quoted = "'";
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
            << " --command ";
        switch (command.kind) {
            case FlightCommandKind::Arm:
                out << "arm";
                break;
            case FlightCommandKind::Takeoff:
                out << "takeoff";
                break;
            case FlightCommandKind::Land:
                out << "land";
                break;
            case FlightCommandKind::Disarm:
                out << "disarm";
                break;
            case FlightCommandKind::Velocity:
            default:
                out << "velocity";
                break;
        }
        out << " --host " << shell_quote(config_.host)
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
    std::unique_ptr<OneShotTransport> transport_;
};

struct Px4BridgeCommandSinkConfig {
    std::string bridge_command{"python3 tools/px4/px4-command-bridge.py"};
    double command_duration_s{0.1};
    double max_velocity_mps{5.0};
    int verbosity{0};
    bool debug_logging{false};
};

class Px4BridgeCommandSink final : public FlightCommandSink {
public:
    explicit Px4BridgeCommandSink(Px4BridgeCommandSinkConfig config);
    ~Px4BridgeCommandSink() override;

    Px4BridgeCommandSink(const Px4BridgeCommandSink&) = delete;
    Px4BridgeCommandSink& operator=(const Px4BridgeCommandSink&) = delete;
    Px4BridgeCommandSink(Px4BridgeCommandSink&&) = delete;
    Px4BridgeCommandSink& operator=(Px4BridgeCommandSink&&) = delete;

    FlightCommandResult send(const VelocityCommand& command) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

struct Px4MavlinkCommandSinkConfig {
    std::string endpoints{"udpout:127.0.0.1:14550,udpout:127.0.0.1:14540,udpout:127.0.0.1:14600"};
    std::string px4_tmux_target{"dedalus-sim:px4"};
    bool use_px4_shell_lifecycle{true};
    std::uint8_t source_system_id{255U};
    std::uint8_t source_component_id{190U};
    std::uint8_t target_system_id{1U};
    std::uint8_t target_component_id{1U};
    double command_duration_s{0.1};
    double max_velocity_mps{5.0};
    double takeoff_altitude_m{8.0};
    bool set_offboard_on_velocity{true};
    bool debug_logging{false};
};

class Px4MavlinkCommandSink final : public FlightCommandSink {
public:
    explicit Px4MavlinkCommandSink(Px4MavlinkCommandSinkConfig config);
    ~Px4MavlinkCommandSink() override;

    Px4MavlinkCommandSink(const Px4MavlinkCommandSink&) = delete;
    Px4MavlinkCommandSink& operator=(const Px4MavlinkCommandSink&) = delete;
    Px4MavlinkCommandSink(Px4MavlinkCommandSink&&) = delete;
    Px4MavlinkCommandSink& operator=(Px4MavlinkCommandSink&&) = delete;

    FlightCommandResult send(const VelocityCommand& command) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class NullFlightCommandSink final : public FlightCommandSink {
public:
    FlightCommandResult send(const VelocityCommand& command) override {
        return FlightCommandResult{command.kind, true, "OK null flight command sink"};
    }
};

struct MavlinkGimbalPointingSinkConfig {
    // Native C++ MAVLink writer. Use udpout endpoints that point at the PX4 /
    // companion MAVLink listener. This sink intentionally does not use the
    // Python MAVLink bridge.
    std::string endpoints{"udpout:127.0.0.1:14550"};
    std::uint8_t source_system_id{255U};
    std::uint8_t source_component_id{191U};
    std::uint8_t target_system_id{1U};
    std::uint8_t target_component_id{1U};
    std::uint8_t gimbal_device_id{0U};
    std::uint32_t gimbal_manager_flags{0U};
    double deadband_rad{0.004363323129985824};  // 0.25 deg
    double resend_interval_s{0.25};
    bool debug_logging{false};
};

class MavlinkGimbalPointingSink final : public CameraPointingSink {
public:
    explicit MavlinkGimbalPointingSink(MavlinkGimbalPointingSinkConfig config);
    ~MavlinkGimbalPointingSink() override;

    CameraPointingResult send(const CameraPointingCommand& command) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace dedalus