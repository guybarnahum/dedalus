#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "dedalus/behavior/flight_command_sinks.hpp"

namespace {

class FakeTransport final : public dedalus::BridgeTransport {
public:
    explicit FakeTransport(std::string response) : response_(std::move(response)) {}

    std::string request_once(const std::string& command) override {
        commands.push_back(command);
        return response_;
    }

    std::optional<std::string> read_stream_line(const std::string&) override {
        return std::nullopt;
    }

    std::optional<std::string> read_stream_bytes(const std::string&, std::size_t) override {
        return std::nullopt;
    }

    std::optional<std::vector<std::uint8_t>> read_stream_byte_vector(
        const std::string&,
        std::size_t) override {
        return std::nullopt;
    }

    void close_stream() override {}

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
    config.bridge_command = "python3 simulation/airsim-send-velocity.py";
    config.command_duration_s = 0.2;
    config.max_velocity_mps = 2.0;

    auto fake_transport = std::make_unique<FakeTransport>("OK sent\n");
    auto* fake_transport_ptr = fake_transport.get();
    dedalus::AirSimVelocityCommandSink sink{config, std::move(fake_transport)};

    dedalus::VelocityCommand command;
    command.velocity_local_mps = dedalus::Vec3{3.0, -3.0, 1.5};
    command.yaw_rate_radps = 0.25;
    command.yaw_rate_valid = true;
    sink.send(command);

    if (fake_transport_ptr->commands.size() != 1U) {
        std::cerr << "AirSimVelocityCommandSink did not send exactly one command\n";
        return 1;
    }

    const auto& rendered = fake_transport_ptr->commands.front();
    const std::string required_tokens[] = {
        "python3 simulation/airsim-send-velocity.py",
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
