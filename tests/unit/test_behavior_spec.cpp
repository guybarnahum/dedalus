#include "dedalus/behavior/behavior_spec.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void require_near(double actual, double expected, const std::string& message) {
    if (std::fabs(actual - expected) > 1.0e-9) {
        throw std::runtime_error(message);
    }
}

template <typename Fn>
void require_throws(Fn&& fn, const std::string& expected_message_part) {
    try {
        fn();
    } catch (const std::exception& exc) {
        const std::string message = exc.what();
        if (message.find(expected_message_part) == std::string::npos) {
            throw std::runtime_error("unexpected exception: " + message);
        }
        return;
    }
    throw std::runtime_error("expected exception containing: " + expected_message_part);
}

void parses_follow_yaml() {
    const std::string yaml = R"YAML(
mission:
  name: follow_person_demo

target:
  selector:
    class: person
    confidence_min: 0.55
    policy: highest_confidence
    reacquire_timeout_s: 5.0

behavior:
  type: follow
  target_frame: target_heading_frame
  relative_offset_m:
    x: -8.0
    y: 0.0
    z: 4.0
  max_speed_mps: 2.0
  position_tolerance_m: 1.5
  lost_target_timeout_s: 5.0

completion:
  after_s: 30
  then: go_home_land

fallback:
  on_target_lost: hold_then_go_home
  hold_s: 5.0
)YAML";

    const auto spec = dedalus::parse_behavior_spec_text(yaml);
    require(spec.mission_name == "follow_person_demo", "mission name should parse");
    require(spec.target.class_label == "person", "target class should parse");
    require(spec.target.track_id.empty(), "default target track id should be empty");
    require(spec.target.agent_id.empty(), "default target agent id should be empty");
    require(spec.target.policy == dedalus::TargetSelectionPolicy::HighestConfidence, "policy should parse");
    require_near(spec.target.confidence_min, 0.55, "confidence_min should parse");
    require(spec.behavior.type == dedalus::BehaviorType::Follow, "follow type should parse");
    require(spec.behavior.target_frame == dedalus::ReferenceFrame::TargetHeadingFrame, "target frame should parse");
    require_near(spec.behavior.relative_offset_m.x, -8.0, "offset x should parse");
    require_near(spec.behavior.relative_offset_m.z, 4.0, "offset z should parse");
    require_near(spec.behavior.max_speed_mps, 2.0, "max speed should parse");
    require(spec.completion.then == dedalus::CompletionAction::GoHomeLand, "completion action should parse");
    require(spec.fallback.on_target_lost == dedalus::TargetLostFallback::HoldThenGoHome, "fallback should parse");
}

void parses_json_circle() {
    const std::string json = R"JSON({
  "mission": {"name": "circle_vehicle_demo"},
  "target": {
    "selector": {
      "class": "vehicle",
      "confidence_min": 0.6,
      "policy": "nearest"
    }
  },
  "behavior": {
    "type": "circle",
    "radius_m": 10.0,
    "altitude_offset_m": 5.0,
    "angular_speed_deg_s": 12.0,
    "direction": "counter_clockwise",
    "max_speed_mps": 3.0
  },
  "completion": {"after_s": 30, "then": "go_home_land"}
})JSON";

    const auto spec = dedalus::parse_behavior_spec_text(json);
    require(spec.mission_name == "circle_vehicle_demo", "JSON mission name should parse");
    require(spec.target.class_label == "vehicle", "JSON target class should parse");
    require(spec.target.policy == dedalus::TargetSelectionPolicy::Nearest, "JSON policy should parse");
    require(spec.behavior.type == dedalus::BehaviorType::Circle, "circle type should parse");
    require(spec.behavior.direction == dedalus::CircleDirection::CounterClockwise, "circle direction should parse");
    require_near(spec.behavior.radius_m, 10.0, "circle radius should parse");
    require_near(spec.behavior.angular_speed_deg_s, 12.0, "circle angular speed should parse");
}

void parses_track_and_agent_selectors() {
    const std::string track_yaml = R"YAML(
mission:
  name: follow_specific_track

target:
  selector:
    class: person
    track_id: ghost_person_001
    policy: persistent_track
    confidence_min: 0.4

behavior:
  type: follow
  relative_offset_m:
    x: -5.0
    y: 0.0
    z: 3.0
)YAML";

    const auto track_spec = dedalus::parse_behavior_spec_text(track_yaml);
    require(track_spec.target.class_label == "person", "track selector class should parse");
    require(track_spec.target.track_id == "ghost_person_001", "track selector track_id should parse");
    require(track_spec.target.agent_id.empty(), "track selector agent_id should default empty");
    require(track_spec.target.policy == dedalus::TargetSelectionPolicy::PersistentTrack, "track selector policy should parse");

    const std::string agent_json = R"JSON({
  "target": {
    "selector": {
      "agent_id": "agent_track_0007",
      "confidence_min": 0.25
    }
  },
  "behavior": {"type": "hold"}
})JSON";

    const auto agent_spec = dedalus::parse_behavior_spec_text(agent_json);
    require(agent_spec.target.class_label.empty(), "agent-only selector class should default empty");
    require(agent_spec.target.track_id.empty(), "agent-only selector track_id should default empty");
    require(agent_spec.target.agent_id == "agent_track_0007", "agent selector agent_id should parse");
    require_near(agent_spec.target.confidence_min, 0.25, "agent selector confidence should parse");
}

void applies_defaults() {
    const std::string yaml = R"YAML(
target:
  selector:
    class: person

behavior:
  type: hold
)YAML";

    const auto spec = dedalus::parse_behavior_spec_text(yaml);
    require(spec.mission_name == "object_behavior", "default mission name should apply");
    require_near(spec.target.confidence_min, 0.5, "default confidence_min should apply");
    require(spec.target.policy == dedalus::TargetSelectionPolicy::HighestConfidence, "default policy should apply");
    require_near(spec.target.reacquire_timeout_s, 5.0, "default reacquire timeout should apply");
    require(spec.target.track_id.empty(), "default track_id should apply");
    require(spec.target.agent_id.empty(), "default agent_id should apply");
    require(spec.behavior.type == dedalus::BehaviorType::Hold, "hold behavior should parse");
    require(spec.behavior.target_frame == dedalus::ReferenceFrame::WorldLocalFrame, "default frame should apply");
    require_near(spec.behavior.max_speed_mps, 1.0, "default max speed should apply");
    require(spec.completion.then == dedalus::CompletionAction::None, "default completion should apply");
    require(spec.fallback.on_target_lost == dedalus::TargetLostFallback::None, "default fallback should apply");
}

void parses_sequence_yaml() {
    const std::string yaml = R"YAML(
mission:
  name: sequence_demo

target:
  selector:
    class: vehicle
    policy: nearest

behavior:
  type: sequence
  steps:
    - type: approach
      stop_distance_m: 8.0
      altitude_offset_m: 4.0
      max_speed_mps: 2.0
    - type: circle
      radius_m: 10.0
      altitude_offset_m: 5.0
      angular_speed_deg_s: 10.0
      duration_s: 20.0
    - type: go_home_land
)YAML";

    const auto spec = dedalus::parse_behavior_spec_text(yaml);
    require(spec.behavior.type == dedalus::BehaviorType::Sequence, "sequence type should parse");
    require(spec.behavior.steps.size() == 3U, "sequence steps should parse");
    require(spec.behavior.steps[0].type == dedalus::BehaviorType::Approach, "first step should be approach");
    require_near(spec.behavior.steps[0].stop_distance_m, 8.0, "approach stop distance should parse");
    require(spec.behavior.steps[1].type == dedalus::BehaviorType::Circle, "second step should be circle");
    require_near(spec.behavior.steps[1].duration_s, 20.0, "circle duration should parse");
    require(spec.behavior.steps[2].type == dedalus::BehaviorType::GoHomeLand, "third step should go home and land");
}

void rejects_invalid_specs() {
    require_throws([] {
        (void)dedalus::parse_behavior_spec_text(R"YAML(
target:
  selector:
    class: person
behavior:
  type: dance
)YAML");
    }, "unknown behavior type");

    require_throws([] {
        (void)dedalus::parse_behavior_spec_text(R"YAML(
target:
  selector:
    confidence_min: 0.5
behavior:
  type: hold
)YAML");
    }, "at least one of class, track_id, or agent_id");

    require_throws([] {
        (void)dedalus::parse_behavior_spec_text(R"YAML(
target:
  selector:
    class: person
behavior:
  type: follow
)YAML");
    }, "relative_offset_m is required");

    require_throws([] {
        (void)dedalus::parse_behavior_spec_text(R"YAML(
target:
  selector:
    class: vehicle
behavior:
  type: circle
  radius_m: 0.0
  angular_speed_deg_s: 12.0
)YAML");
    }, "radius_m must be positive");

    require_throws([] {
        (void)dedalus::parse_behavior_spec_text(R"YAML(
target:
  selector:
    class: vehicle
behavior:
  type: approach
  stop_distance_m: 0.0
)YAML");
    }, "stop_distance_m must be positive");

    require_throws([] {
        (void)dedalus::parse_behavior_spec_text(R"YAML(
target:
  selector:
    class: person
behavior:
  type: sequence
)YAML");
    }, "steps must contain");
}

}  // namespace

int main() {
    try {
        parses_follow_yaml();
        parses_json_circle();
        parses_track_and_agent_selectors();
        applies_defaults();
        parses_sequence_yaml();
        rejects_invalid_specs();
    } catch (const std::exception& exc) {
        std::cerr << "test_behavior_spec failed: " << exc.what() << '\n';
        return 1;
    }
    return 0;
}
