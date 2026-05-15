#pragma once

#include <memory>
#include <string>

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
    explicit AirSimVelocityCommandSink(AirSimVelocityCommandSinkConfig config);
    AirSimVelocityCommandSink(
        AirSimVelocityCommandSinkConfig config,
        std::unique_ptr<BridgeTransport> transport);

    void send(const VelocityCommand& command) override;

private:
    [[nodiscard]] std::string build_command(const VelocityCommand& command) const;
    [[nodiscard]] Vec3 bounded_velocity(Vec3 velocity) const;

    AirSimVelocityCommandSinkConfig config_;
    std::unique_ptr<BridgeTransport> transport_;
};

class NullFlightCommandSink final : public FlightCommandSink {
public:
    void send(const VelocityCommand&) override {}
};

}  // namespace dedalus
