#include <iostream>
#include <stdexcept>

#include "dedalus/behavior/flight_command_sinks.hpp"

int main() {
    dedalus::Px4MavlinkCommandSinkConfig config;
    config.endpoints = "udpout:127.0.0.1:15999";
    config.command_duration_s = 0.1;
    config.max_velocity_mps = 2.0;
    config.takeoff_altitude_m = 8.0;
    config.set_offboard_on_velocity = false;

    try {
        dedalus::Px4MavlinkCommandSink sink{config};

        dedalus::VelocityCommand arm;
        arm.kind = dedalus::FlightCommandKind::Arm;
        auto result = sink.send(arm);
        if (!result.success || result.status.find("arm") == std::string::npos) {
            std::cerr << "unexpected arm result: " << result.status << "\n";
            return 1;
        }

        dedalus::VelocityCommand takeoff;
        takeoff.kind = dedalus::FlightCommandKind::Takeoff;
        result = sink.send(takeoff);
        if (!result.success || result.status.find("takeoff") == std::string::npos) {
            std::cerr << "unexpected takeoff result: " << result.status << "\n";
            return 1;
        }

        dedalus::VelocityCommand velocity;
        velocity.kind = dedalus::FlightCommandKind::Velocity;
        velocity.velocity_local_mps = dedalus::Vec3{3.0, -3.0, 1.5};
        result = sink.send(velocity);
        if (!result.success || result.status.find("vx=2") == std::string::npos ||
            result.status.find("vy=-2") == std::string::npos) {
            std::cerr << "unexpected bounded velocity result: " << result.status << "\n";
            return 1;
        }

        dedalus::VelocityCommand disarm;
        disarm.kind = dedalus::FlightCommandKind::Disarm;
        result = sink.send(disarm);
        if (!result.success || result.status.find("disarm") == std::string::npos) {
            std::cerr << "unexpected disarm result: " << result.status << "\n";
            return 1;
        }
    } catch (const std::exception& ex) {
        std::cerr << "PX4 MAVLink sink smoke test failed: " << ex.what() << "\n";
        return 1;
    }

    bool rejected = false;
    try {
        dedalus::Px4MavlinkCommandSinkConfig bad_config;
        bad_config.endpoints = "";
        dedalus::Px4MavlinkCommandSink bad_sink{bad_config};
        (void)bad_sink;
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    if (!rejected) {
        std::cerr << "PX4 MAVLink sink did not reject empty endpoints\n";
        return 1;
    }

    return 0;
}
