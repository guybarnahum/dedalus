#include <iostream>
#include <stdexcept>
#include <string>

#include "dedalus/behavior/flight_command_sinks.hpp"

namespace {

bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

}  // namespace

int main() {
    // Happy path: drive all five command types through the JSON pipe protocol
    // against a stub that implements the same protocol as px4-command-bridge.py.
    {
        dedalus::Px4BridgeCommandSinkConfig cfg;
        cfg.bridge_command = "python3 tests/fixtures/px4_bridge_ci_stub.py";
        cfg.command_duration_s = 0.1;
        cfg.max_velocity_mps = 2.0;

        try {
            dedalus::Px4BridgeCommandSink sink{cfg};

            // arm
            dedalus::VelocityCommand arm;
            arm.kind = dedalus::FlightCommandKind::Arm;
            const auto arm_result = sink.send(arm);
            if (!arm_result.success || !contains(arm_result.status, "\"status\":\"arm\"")) {
                std::cerr << "arm response unexpected: " << arm_result.status << "\n";
                return 1;
            }

            // takeoff
            dedalus::VelocityCommand takeoff;
            takeoff.kind = dedalus::FlightCommandKind::Takeoff;
            const auto takeoff_result = sink.send(takeoff);
            if (!takeoff_result.success || !contains(takeoff_result.status, "\"status\":\"takeoff\"")) {
                std::cerr << "takeoff response unexpected: " << takeoff_result.status << "\n";
                return 1;
            }

            // velocity — input exceeds max_velocity_mps=2.0 so vx/vy are clamped
            dedalus::VelocityCommand velocity;
            velocity.kind = dedalus::FlightCommandKind::Velocity;
            velocity.velocity_local_mps = dedalus::Vec3{3.0, -3.0, 1.5};
            velocity.yaw_valid = true;
            velocity.yaw_rad = 0.5;
            const auto vel_result = sink.send(velocity);
            if (!vel_result.success || !contains(vel_result.status, "\"status\":\"velocity\"")) {
                std::cerr << "velocity response unexpected: " << vel_result.status << "\n";
                return 1;
            }
            // Stub echoes the vx it received; must be 2 (clamped from 3).
            if (!contains(vel_result.status, "\"vx\":2")) {
                std::cerr << "velocity clamping not reflected in stub echo: " << vel_result.status << "\n";
                return 1;
            }

            // land
            dedalus::VelocityCommand land;
            land.kind = dedalus::FlightCommandKind::Land;
            const auto land_result = sink.send(land);
            if (!land_result.success || !contains(land_result.status, "\"status\":\"land\"")) {
                std::cerr << "land response unexpected: " << land_result.status << "\n";
                return 1;
            }

            // disarm
            dedalus::VelocityCommand disarm;
            disarm.kind = dedalus::FlightCommandKind::Disarm;
            const auto disarm_result = sink.send(disarm);
            if (!disarm_result.success || !contains(disarm_result.status, "\"status\":\"disarm\"")) {
                std::cerr << "disarm response unexpected: " << disarm_result.status << "\n";
                return 1;
            }

        } catch (const std::exception& ex) {
            std::cerr << "Px4BridgeCommandSink happy path threw: " << ex.what() << "\n";
            return 1;
        }
    }

    // Error path: bridge exits without printing a ready signal.
    // The C++ constructor reads the ready line; when the child closes stdout
    // before writing anything, read() returns 0 and the constructor throws.
    {
        bool threw = false;
        try {
            dedalus::Px4BridgeCommandSinkConfig bad_cfg;
            bad_cfg.bridge_command = "false";  // shell builtin; exits 1, no output
            bad_cfg.command_duration_s = 0.1;
            bad_cfg.max_velocity_mps = 2.0;
            dedalus::Px4BridgeCommandSink bad_sink{bad_cfg};
        } catch (const std::runtime_error&) {
            threw = true;
        }
        if (!threw) {
            std::cerr << "Px4BridgeCommandSink should throw when bridge exits without ready signal\n";
            return 1;
        }
    }

    return 0;
}
