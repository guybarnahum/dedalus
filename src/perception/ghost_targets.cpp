#include "dedalus/perception/ghost_targets.hpp"

#include <iomanip>
#include <sstream>
#include <utility>

namespace dedalus {
namespace {

constexpr double kNsPerSecond = 1000000000.0;

double elapsed_seconds(TimePoint timestamp, TimePoint scenario_start) {
    const auto delta_ns = timestamp.timestamp_ns - scenario_start.timestamp_ns;
    if (delta_ns <= 0) {
        return 0.0;
    }
    return static_cast<double>(delta_ns) / kNsPerSecond;
}

std::string json_string(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() + 8U);
    for (const char ch : value) {
        switch (ch) {
            case '"':
                escaped += "\\\"";
                break;
            case '\\':
                escaped += "\\\\";
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
    return "\"" + escaped + "\"";
}

std::string vec3_json(const Vec3& value) {
    std::ostringstream out;
    out << '[' << value.x << ',' << value.y << ',' << value.z << ']';
    return out.str();
}

Observation3D observation_from_state(
    const GhostDetectionState& state,
    TimePoint timestamp,
    const MapFrameId& map_frame_id) {
    Observation3D observation;
    observation.track_id = state.source_track_id;
    observation.timestamp = timestamp;
    observation.position_local = state.position_local_m;
    observation.position_body = state.position_local_m;
    observation.map_frame_id = map_frame_id;
    observation.class_label = state.class_label;
    observation.faction = FactionLabel::Unknown;
    observation.confidence = static_cast<float>(state.confidence);
    return observation;
}

}  // namespace

GhostTargetProvider::GhostTargetProvider(GhostScenario scenario)
    : scenario_(std::move(scenario)) {}

GhostDetectionsFrame GhostTargetProvider::frame_at(
    TimePoint timestamp,
    MapFrameId map_frame_id,
    TimePoint scenario_start) const {
    GhostDetectionsFrame frame;
    frame.timestamp = timestamp;
    frame.map_frame_id = map_frame_id;
    frame.scenario_elapsed_s = elapsed_seconds(timestamp, scenario_start);
    frame.detections = scenario_.evaluate(frame.scenario_elapsed_s);
    frame.observations.reserve(frame.detections.size());
    for (const auto& state : frame.detections) {
        frame.observations.push_back(observation_from_state(state, timestamp, frame.map_frame_id));
    }
    return frame;
}

std::vector<Observation3D> GhostTargetProvider::observations_at(
    TimePoint timestamp,
    MapFrameId map_frame_id,
    TimePoint scenario_start) const {
    return frame_at(timestamp, map_frame_id, scenario_start).observations;
}

PerceptionPipelineOutput GhostTargetProvider::output_at(
    TimePoint timestamp,
    MapFrameId map_frame_id,
    TimePoint scenario_start) const {
    PerceptionPipelineOutput output;
    output.observations = observations_at(timestamp, map_frame_id, scenario_start);
    return output;
}

std::string to_json(const GhostDetectionsFrame& frame) {
    std::ostringstream out;
    out << std::setprecision(9);
    out << "{\"timestamp_ns\":" << frame.timestamp.timestamp_ns;
    out << ",\"map_frame_id\":" << json_string(frame.map_frame_id.value);
    out << ",\"scenario_elapsed_s\":" << frame.scenario_elapsed_s;
    out << ",\"detections\":[";
    for (std::size_t index = 0; index < frame.detections.size(); ++index) {
        if (index > 0U) {
            out << ',';
        }
        const auto& detection = frame.detections[index];
        out << '{';
        out << "\"source_track_id\":" << json_string(detection.source_track_id.value);
        out << ",\"class\":" << json_string(to_string(detection.class_label));
        out << ",\"confidence\":" << detection.confidence;
        out << ",\"position_local_m\":" << vec3_json(detection.position_local_m);
        out << ",\"velocity_local_mps\":" << vec3_json(detection.velocity_local_mps);
        out << ",\"size_m\":" << vec3_json(detection.size_m);
        out << '}';
    }
    out << "]}";
    return out.str();
}

}  // namespace dedalus
