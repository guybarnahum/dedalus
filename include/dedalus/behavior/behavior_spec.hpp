#pragma once

#include <string>
#include <vector>

#include "dedalus/core/types.hpp"

namespace dedalus {

enum class BehaviorSpecFormat {
    Auto,
    Yaml,
    Json,
};

enum class TargetSelectionPolicy {
    HighestConfidence,
    Nearest,
    PersistentTrack,
};

enum class BehaviorType {
    Hold,
    Search,
    Follow,
    Approach,
    Circle,
    GoHome,
    Land,
    GoHomeLand,
    Sequence,
};

enum class ReferenceFrame {
    TargetHeadingFrame,
    WorldLocalFrame,
    DroneHeadingFrame,
    CameraFrame,
};

enum class CircleDirection {
    Clockwise,
    CounterClockwise,
};

enum class CompletionAction {
    None,
    Hold,
    GoHome,
    Land,
    GoHomeLand,
};

enum class TargetLostFallback {
    None,
    HoldThenGoHome,
    SearchThenGoHome,
    Abort,
};

struct BehaviorVector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct TargetSelectorSpec {
    ClassLabel class_label{ClassLabel::Unknown};
    std::string track_id;
    std::string agent_id;
    double confidence_min{0.5};
    TargetSelectionPolicy policy{TargetSelectionPolicy::HighestConfidence};
    double reacquire_timeout_s{5.0};
};

struct CompletionSpec {
    double after_s{0.0};
    CompletionAction then{CompletionAction::None};
};

struct FallbackSpec {
    TargetLostFallback on_target_lost{TargetLostFallback::None};
    double hold_s{5.0};
    double search_s{10.0};
};

struct AltitudeProfileSpec {
    bool enabled{false};
    double start_height_m{0.0};
    double end_height_m{0.0};
    double duration_s{0.0};
    std::string easing{"smoothstep"};
};

struct BehaviorSpec {
    BehaviorType type{BehaviorType::Hold};
    ReferenceFrame target_frame{ReferenceFrame::WorldLocalFrame};
    BehaviorVector3 relative_offset_m{};
    double max_speed_mps{1.0};
    double max_vertical_speed_mps{1.0};
    double position_tolerance_m{1.0};
    double lost_target_timeout_s{5.0};

    // Optional per-behavior policy overrides. Empty string means inherit the
    // mission/controller default. Values are intentionally stored as strings so
    // the behavior spec parser stays independent from ObjectBehaviorMissionController
    // enums while still preserving the validated syntax.
    std::string yaw_mode{};
    std::string camera_pointing_mode{};
    double yaw_offset_rad{0.0};

    double radius_m{0.0};
    double altitude_offset_m{0.0};
    AltitudeProfileSpec altitude_profile;
    double angular_speed_deg_s{0.0};
    CircleDirection direction{CircleDirection::Clockwise};
    double orbit_count{0.0};

    double stop_distance_m{0.0};
    double duration_s{0.0};

    std::vector<BehaviorSpec> steps;
};

struct BehaviorMissionSpec {
    std::string mission_name{"object_behavior"};
    TargetSelectorSpec target;
    BehaviorSpec behavior;
    CompletionSpec completion;
    FallbackSpec fallback;
};

std::string to_string(TargetSelectionPolicy policy);
std::string to_string(BehaviorType type);
std::string to_string(ReferenceFrame frame);
std::string to_string(CircleDirection direction);
std::string to_string(CompletionAction action);
std::string to_string(TargetLostFallback fallback);

BehaviorMissionSpec parse_behavior_spec_text(
    const std::string& text,
    BehaviorSpecFormat format = BehaviorSpecFormat::Auto);

BehaviorMissionSpec parse_behavior_spec_file(
    const std::string& path,
    BehaviorSpecFormat format = BehaviorSpecFormat::Auto);

}  // namespace dedalus
