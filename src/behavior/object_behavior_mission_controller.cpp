#include "dedalus/behavior/object_behavior_mission_controller.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace dedalus {
namespace {

std::string json_escape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());
    for (const char ch : value) {
        switch (ch) {
            case '\\':
                escaped += "\\\\";
                break;
            case '"':
                escaped += "\\\"";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            case '\t':
                escaped += "\\t";
                break;
            default:
                escaped.push_back(ch);
                break;
        }
    }
    return escaped;
}

std::string q(const std::string& value) {
    return "\"" + json_escape(value) + "\"";
}

double seconds_between(TimePoint start, TimePoint end) {
    return static_cast<double>(end.timestamp_ns - start.timestamp_ns) / 1'000'000'000.0;
}

std::string class_label_event_string(ClassLabel label) {
    switch (label) {
        case ClassLabel::Person:
            return "person";
        case ClassLabel::Drone:
            return "drone";
        case ClassLabel::Car:
            return "car";
        case ClassLabel::Boat:
            return "boat";
        case ClassLabel::House:
            return "house";
        case ClassLabel::Building:
            return "building";
        case ClassLabel::Tree:
            return "tree";
        case ClassLabel::Road:
            return "road";
        case ClassLabel::River:
            return "river";
        case ClassLabel::Terrain:
            return "terrain";
        case ClassLabel::Unknown:
        default:
            return "unknown";
    }
}

}  // namespace

ObjectBehaviorMissionConfig load_object_behavior_mission_config(const MissionOptions& options) {
    ObjectBehaviorMissionConfig config;
    const auto behavior_spec_path = options.get_or("behavior_spec_path", "");
    if (behavior_spec_path.empty()) {
        throw std::invalid_argument("object_behavior mission_controller requires mission_options.behavior_spec_path");
    }
    config.behavior_spec = parse_behavior_spec_file(behavior_spec_path);
    config.hold_velocity_mps = std::stod(options.get_or("object_behavior_hold_velocity_mps", "0.0"));
    return config;
}

ObjectBehaviorMissionController::ObjectBehaviorMissionController(ObjectBehaviorMissionConfig config)
    : config_(std::move(config)) {}

VelocityCommand ObjectBehaviorMissionController::command_from_velocity(
    TimePoint timestamp,
    Vec3 velocity_local_mps) const {
    VelocityCommand command;
    command.kind = FlightCommandKind::Velocity;
    command.timestamp = timestamp;
    command.velocity_local_mps = velocity_local_mps;
    command.yaw_rate_radps = 0.0;
    command.yaw_rate_valid = true;
    command.yaw_valid = false;
    return command;
}

std::string ObjectBehaviorMissionController::target_event(const TargetSelection& selection) const {
    return "\"event\":\"target_selected\""
        ",\"agent_id\":" + q(selection.agent_id.value) +
        ",\"source_track_id\":" + q(selection.source_track_id.value) +
        ",\"identity_id\":" + q(selection.identity_id.value) +
        ",\"class\":" + q(class_label_event_string(selection.class_label)) +
        ",\"confidence\":" + std::to_string(selection.confidence) +
        ",\"reason\":" + q(selection.reason);
}

std::string ObjectBehaviorMissionController::behavior_event(
    const std::string& event,
    const std::string& reason) const {
    std::string fields = "\"event\":" + q(event) +
        ",\"behavior\":" + q(to_string(config_.behavior_spec.behavior.type)) +
        ",\"mission\":" + q(config_.behavior_spec.mission_name) +
        ",\"reason\":" + q(reason);
    if (previous_selection_.has_value()) {
        fields += ",\"agent_id\":" + q(previous_selection_->agent_id.value) +
            ",\"source_track_id\":" + q(previous_selection_->source_track_id.value) +
            ",\"identity_id\":" + q(previous_selection_->identity_id.value);
    }
    return fields;
}

bool ObjectBehaviorMissionController::completion_elapsed(TimePoint now) const {
    const double after_s = config_.behavior_spec.completion.after_s;
    if (after_s <= 0.0) {
        return false;
    }
    return seconds_between(behavior_start_, now) >= after_s;
}

MissionTickOutput ObjectBehaviorMissionController::tick(const MissionTickInput& input) {
    MissionTickOutput output;
    output.state = state_;

    if (!mission_started_) {
        mission_started_ = true;
        mission_start_ = input.now;
        behavior_start_ = input.now;
        state_ = MissionLifecycleState::ExecuteMission;
    }

    if (state_ == MissionLifecycleState::ExecuteMission) {
        auto selection = selector_.select(input.snapshot, config_.behavior_spec.target, previous_selection_);
        if (selection.selected) {
            previous_selection_ = selection;
            if (!target_selected_emitted_) {
                target_selected_emitted_ = true;
                output.events.push_back(target_event(selection));
            }
            if (!behavior_start_emitted_) {
                behavior_start_emitted_ = true;
                behavior_start_ = input.now;
                output.events.push_back(behavior_event("behavior_start", "target_selected"));
            }
            if (input.finish_requested || completion_elapsed(input.now)) {
                if (!behavior_complete_emitted_) {
                    behavior_complete_emitted_ = true;
                    output.events.push_back(behavior_event(
                        "behavior_complete",
                        input.finish_requested ? "finish_requested" : "duration_elapsed"));
                }
                state_ = MissionLifecycleState::GoHome;
                output.status = input.finish_requested ? "object_behavior_finish_requested" : "object_behavior_complete";
            } else {
                output.command = command_from_velocity(input.now, Vec3{0.0, 0.0, 0.0});
                output.status = "object_behavior_hold";
            }
        } else {
            if (input.finish_requested) {
                state_ = MissionLifecycleState::GoHome;
                output.status = "object_behavior_finish_requested_no_target";
            } else {
                output.command = command_from_velocity(input.now, Vec3{0.0, 0.0, 0.0});
                output.status = "object_behavior_waiting_for_target_" + to_string(selection.status);
            }
        }
    } else if (state_ == MissionLifecycleState::GoHome) {
        state_ = MissionLifecycleState::Land;
        output.status = "object_behavior_go_home_delegated";
    } else if (state_ == MissionLifecycleState::Land) {
        state_ = MissionLifecycleState::Complete;
        output.status = "object_behavior_landed";
    } else if (state_ == MissionLifecycleState::Complete) {
        output.status = "complete";
    } else {
        state_ = MissionLifecycleState::ExecuteMission;
        output.status = "object_behavior_start";
    }

    output.state = state_;
    return output;
}

}  // namespace dedalus
