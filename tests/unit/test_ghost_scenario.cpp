#include "dedalus/simulation/ghost_scenario.hpp"

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

void require_near(double actual, double expected, double tolerance, const std::string& message) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(message + ": actual=" + std::to_string(actual) + " expected=" + std::to_string(expected));
    }
}

const dedalus::GhostDetectionState& find_state(
    const std::vector<dedalus::GhostDetectionState>& states,
    const std::string& track_id) {
    for (const auto& state : states) {
        if (state.source_track_id.value == track_id) {
            return state;
        }
    }
    throw std::runtime_error("missing state for track_id: " + track_id);
}

void loads_json_and_referenced_trajectories() {
    const auto scenario = dedalus::GhostScenario::load_from_file("simulation/ghost_detections/person_pair_crossing.json");
    require(scenario.name() == "person_pair_crossing", "scenario name should parse");
    require(scenario.map_frame_id().value == "map_airsim_mission_0001", "map frame should parse");
    require(scenario.detections().size() == 3U, "expected three detections");
    require(!scenario.detections()[0].trajectory.empty(), "first person should load trajectory");
    require(!scenario.detections()[1].trajectory.empty(), "second person should load trajectory");
    require(scenario.detections()[2].trajectory.empty(), "car should be static without trajectory");
}

void evaluates_static_and_dynamic_positions() {
    const auto scenario = dedalus::GhostScenario::load_from_file("simulation/ghost_detections/person_pair_crossing.json");

    const auto start = scenario.evaluate(0.0);
    require(start.size() == 3U, "evaluate(0) should return three detections");
    const auto& p1_start = find_state(start, "ghost_person_001");
    const auto& p2_start = find_state(start, "ghost_person_002");
    const auto& car_start = find_state(start, "ghost_car_001");
    require_near(p1_start.position_local_m.x, 12.0, 1.0e-9, "p1 start x");
    require_near(p1_start.position_local_m.y, -4.0, 1.0e-9, "p1 start y");
    require_near(p2_start.position_local_m.x, 8.0, 1.0e-9, "p2 start x");
    require_near(car_start.position_local_m.x, 4.0, 1.0e-9, "car start x");
    require_near(car_start.velocity_local_mps.x, 0.0, 1.0e-9, "static car vx");

    const auto later = scenario.evaluate(10.0);
    const auto& p1 = find_state(later, "ghost_person_001");
    const auto& p2 = find_state(later, "ghost_person_002");
    const auto& car = find_state(later, "ghost_car_001");
    require(p1.class_label == dedalus::ClassLabel::Person, "p1 class should be person");
    require(p2.class_label == dedalus::ClassLabel::Person, "p2 class should be person");
    require(car.class_label == dedalus::ClassLabel::Car, "car class should be car");
    require_near(p1.position_local_m.x, 15.0, 1.0e-6, "p1 should integrate +0.3 mps for 10s");
    require_near(p1.position_local_m.y, -4.0, 1.0e-6, "p1 y should remain fixed");
    require_near(p1.velocity_local_mps.x, 0.3, 1.0e-9, "p1 vx should come from trajectory");
    require_near(p2.position_local_m.x, 6.0, 1.0e-6, "p2 should integrate -0.2 mps for 10s");
    require_near(p2.velocity_local_mps.x, -0.2, 1.0e-9, "p2 vx should come from trajectory");
    require_near(car.position_local_m.x, 4.0, 1.0e-9, "static car should not move");
    require_near(car.position_local_m.y, 0.0, 1.0e-9, "static car y should not move");
}

void rejects_invalid_specs() {
    bool rejected_missing_track = false;
    try {
        (void)dedalus::GhostScenario::parse_json(R"JSON({
          "detections": [
            {"class": "person", "confidence": 0.8, "initial_position_local_m": [0, 0, 0]}
          ]
        })JSON");
    } catch (const std::exception&) {
        rejected_missing_track = true;
    }
    require(rejected_missing_track, "missing source_track_id should be rejected");

    bool rejected_unknown_class = false;
    try {
        (void)dedalus::GhostScenario::parse_json(R"JSON({
          "detections": [
            {"source_track_id": "ghost_x", "class": "alien", "confidence": 0.8, "initial_position_local_m": [0, 0, 0]}
          ]
        })JSON");
    } catch (const std::exception&) {
        rejected_unknown_class = true;
    }
    require(rejected_unknown_class, "unknown class should be rejected");
}

}  // namespace

int main() {
    try {
        loads_json_and_referenced_trajectories();
        evaluates_static_and_dynamic_positions();
        rejects_invalid_specs();
    } catch (const std::exception& exc) {
        std::cerr << "test_ghost_scenario failed: " << exc.what() << '\n';
        return 1;
    }
    return 0;
}
