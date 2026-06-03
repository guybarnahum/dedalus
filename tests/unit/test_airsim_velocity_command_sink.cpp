#include <iostream>
#include <string>
#include <vector>

#include "dedalus/behavior/flight_command_sinks.hpp"

namespace {

class FakeTransport final : public dedalus::OneShotTransport {
public:
    explicit FakeTransport(std::string response) : response_(std::move(response)) {}

    std::string request_once(const std::string& command) override {
        commands.push_back(command);
        return response_;
    }

    std::vector<std::string> commands;

private:
    std::string response_;
};

bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

}  // namespace

int main() {
    dedalus::AirSimVelocityCommandSinkConfig config;
    config.host = "127.0.0.1";
    config.rpc_port = 41451;
    config.vehicle_name = "PX4";
    config.bridge_command = "python3 simulation/airsim/scripts/airsim-send-velocity.py";
    config.command_duration_s = 0.2;
    config.max_velocity_mps = 2.0;

    auto fake_transport = std::make_unique<FakeTransport>("OK sent\n");
    auto* fake_transport_ptr = fake_transport.get();
    dedalus::AirSimVelocityCommandSink sink{config, std::move(fake_transport)};

    dedalus::VelocityCommand arm;
    arm.kind = dedalus::FlightCommandKind::Arm;
    sink.send(arm);

    dedalus::VelocityCommand takeoff;
    takeoff.kind = dedalus::FlightCommandKind::Takeoff;
    sink.send(takeoff);

    dedalus::VelocityCommand command;
    command.kind = dedalus::FlightCommandKind::Velocity;
    command.velocity_local_mps = dedalus::Vec3{3.0, -3.0, 1.5};
    command.yaw_rate_radps = 0.25;
    command.yaw_rate_valid = true;
    sink.send(command);

    dedalus::VelocityCommand disarm;
    disarm.kind = dedalus::FlightCommandKind::Disarm;
    sink.send(disarm);

    if (fake_transport_ptr->commands.size() != 4U) {
        std::cerr << "AirSimVelocityCommandSink did not send exactly four commands\n";
        return 1;
    }

    if (!contains(fake_transport_ptr->commands[0], "--command arm")) {
        std::cerr << "arm command missing --command arm\n";
        std::cerr << fake_transport_ptr->commands[0] << "\n";
        return 1;
    }

    if (!contains(fake_transport_ptr->commands[1], "--command takeoff")) {
        std::cerr << "takeoff command missing --command takeoff\n";
        std::cerr << fake_transport_ptr->commands[1] << "\n";
        return 1;
    }

    const auto& rendered = fake_transport_ptr->commands[2];
    const std::string required_tokens[] = {
        "python3 simulation/airsim/scripts/airsim-send-velocity.py",
        "--command velocity",
        "--host '127.0.0.1'",
        "--rpc-port 41451",
        "--vehicle-name 'PX4'",
        "--vx 2",
        "--vy -2",
        "--vz 1.5",
        "--duration 0.2",
        "--yaw-rate 0.25"};

    for (const auto& token : required_tokens) {
        if (!contains(rendered, token)) {
            std::cerr << "rendered command missing token: " << token << "\n";
            std::cerr << rendered << "\n";
            return 1;
        }
    }

    if (!contains(fake_transport_ptr->commands[3], "--command disarm")) {
        std::cerr << "disarm command missing --command disarm\n";
        std::cerr << fake_transport_ptr->commands[3] << "\n";
        return 1;
    }

    bool threw = false;
    try {
        auto failing_transport = std::make_unique<FakeTransport>("ERROR nope\n");
        dedalus::AirSimVelocityCommandSink failing_sink{config, std::move(failing_transport)};
        failing_sink.send(command);
    } catch (const std::runtime_error&) {
        threw = true;
    }

    if (!threw) {
        std::cerr << "AirSimVelocityCommandSink did not reject non-OK helper output\n";
        return 1;
    }

    return 0;
}