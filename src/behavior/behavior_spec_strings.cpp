#include "dedalus/behavior/behavior_spec.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace dedalus {

std::string to_string(TargetSelectionPolicy policy) {
    switch (policy) {
        case TargetSelectionPolicy::HighestConfidence:
            return "highest_confidence";
        case TargetSelectionPolicy::Nearest:
            return "nearest";
        case TargetSelectionPolicy::PersistentTrack:
            return "persistent_track";
    }
    return "highest_confidence";
}

std::string to_string(BehaviorType type) {
    switch (type) {
        case BehaviorType::Hold:
            return "hold";
        case BehaviorType::Search:
            return "search";
        case BehaviorType::Follow:
            return "follow";
        case BehaviorType::Approach:
            return "approach";
        case BehaviorType::Circle:
            return "circle";
        case BehaviorType::GoHome:
            return "go_home";
        case BehaviorType::Land:
            return "land";
        case BehaviorType::GoHomeLand:
            return "go_home_land";
        case BehaviorType::Sequence:
            return "sequence";
    }
    return "hold";
}

std::string to_string(ReferenceFrame frame) {
    switch (frame) {
        case ReferenceFrame::TargetHeadingFrame:
            return "target_heading_frame";
        case ReferenceFrame::WorldLocalFrame:
            return "world_local_frame";
        case ReferenceFrame::DroneHeadingFrame:
            return "drone_heading_frame";
        case ReferenceFrame::CameraFrame:
            return "camera_frame";
    }
    return "world_local_frame";
}

std::string to_string(CircleDirection direction) {
    switch (direction) {
        case CircleDirection::Clockwise:
            return "clockwise";
        case CircleDirection::CounterClockwise:
            return "counter_clockwise";
    }
    return "clockwise";
}

std::string to_string(CompletionAction action) {
    switch (action) {
        case CompletionAction::None:
            return "none";
        case CompletionAction::Hold:
            return "hold";
        case CompletionAction::GoHome:
            return "go_home";
        case CompletionAction::Land:
            return "land";
        case CompletionAction::GoHomeLand:
            return "go_home_land";
    }
    return "none";
}

std::string to_string(TargetLostFallback fallback) {
    switch (fallback) {
        case TargetLostFallback::None:
            return "none";
        case TargetLostFallback::HoldThenGoHome:
            return "hold_then_go_home";
        case TargetLostFallback::SearchThenGoHome:
            return "search_then_go_home";
        case TargetLostFallback::Abort:
            return "abort";
    }
    return "none";
}

BehaviorMissionSpec parse_behavior_spec_file(const std::string& path, BehaviorSpecFormat format) {
    std::ifstream input{path};
    if (!input) {
        throw std::runtime_error("failed to open behavior spec: " + path);
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return parse_behavior_spec_text(buffer.str(), format);
}

}  // namespace dedalus
