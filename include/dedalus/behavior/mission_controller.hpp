#pragma once

#include <optional>
#include <string>
#include <unordered_map>

#include "dedalus/core/types.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

struct MissionOptions {
    std::unordered_map<std::string, std::string> values;

    [[nodiscard]] bool contains(const std::string& key) const {
        return values.find(key) != values.end();
    }

    [[nodiscard]] std::string get_or(const std::string& key, const std::string& fallback) const {
        const auto it = values.find(key);
        return it == values.end() ? fallback : it->second;
    }
};

enum class FlightCommandKind {
    Velocity,
    Arm,
    Takeoff,
    Disarm,
};

inline const char* to_string(FlightCommandKind kind) {
    switch (kind) {
        case FlightCommandKind::Velocity:
            return "Velocity";
        case FlightCommandKind::Arm:
            return "Arm";
        case FlightCommandKind::Takeoff:
            return "Takeoff";
        case FlightCommandKind::Disarm:
            return "Disarm";
        default:
            return "Unknown";
    }
}

struct VelocityCommand {
    FlightCommandKind kind{FlightCommandKind::Velocity};
    TimePoint timestamp;
    Vec3 velocity_local_mps;
    double yaw_rate_radps{0.0};
    double yaw_rad{0.0};
    bool yaw_rate_valid{true};
    bool yaw_valid{false};
};

struct FlightCommandResult {
    FlightCommandKind kind{FlightCommandKind::Velocity};
    bool success{false};
    std::string status;
};

enum class MissionLifecycleState {
    Idle,
    Prepare,
    Takeoff,
    ExecuteMission,
    GoHome,
    Land,
    Complete,
    Abort,
};

inline const char* to_string(MissionLifecycleState state) {
    switch (state) {
        case MissionLifecycleState::Idle:
            return "Idle";
        case MissionLifecycleState::Prepare:
            return "Prepare";
        case MissionLifecycleState::Takeoff:
            return "Takeoff";
        case MissionLifecycleState::ExecuteMission:
            return "ExecuteMission";
        case MissionLifecycleState::GoHome:
            return "GoHome";
        case MissionLifecycleState::Land:
            return "Land";
        case MissionLifecycleState::Complete:
            return "Complete";
        case MissionLifecycleState::Abort:
            return "Abort";
        default:
            return "Unknown";
    }
}

struct MissionTickInput {
    TimePoint now;
    WorldSnapshot snapshot;
    std::optional<FlightCommandResult> last_command_result;
};

struct MissionTickOutput {
    MissionLifecycleState state{MissionLifecycleState::Idle};
    std::optional<VelocityCommand> command;
    std::string status;
};

class MissionController {
public:
    virtual ~MissionController() = default;
    virtual MissionTickOutput tick(const MissionTickInput& input) = 0;
};

class FlightCommandSink {
public:
    virtual ~FlightCommandSink() = default;
    virtual FlightCommandResult send(const VelocityCommand& command) = 0;
};

}  // namespace dedalus