#include "dedalus/perception/ghost_targets.hpp"

#include "dedalus/core/json_utils.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

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
    observation.velocity_local = state.velocity_local_mps;
    observation.map_frame_id = map_frame_id;
    observation.class_label = state.class_label;
    observation.faction = FactionLabel::Unknown;
    observation.confidence = static_cast<float>(state.confidence);
    return observation;
}

std::unique_ptr<BridgeTransport> make_transport(const std::string& transport_name) {
    if (transport_name == "pipe") {
        return std::make_unique<PipeBridgeTransport>();
    }
    if (transport_name == "shared_memory") {
        throw std::runtime_error(
            "shared_memory bridge transport is not yet implemented; use bridge_transport: pipe");
    }
    throw std::runtime_error("unknown AirSim ghost object bridge transport: " + transport_name);
}

std::string shell_quote(const std::string& value) {
    std::string quoted = "'";
    for (const char ch : value) {
        if (ch == '\'') {
            quoted += "'\\''";
        } else {
            quoted += ch;
        }
    }
    quoted += "'";
    return quoted;
}

std::string build_object_pose_command(const AirSimGhostObjectSourceConfig& config, TimePoint timestamp) {
    std::ostringstream command;
    command << config.bridge_command
            << " --host " << shell_quote(config.host)
            << " --rpc-port " << config.rpc_port
            << " --timestamp-ns " << timestamp.timestamp_ns;
    int max_pattern_matches = 0;
    for (const auto& object : config.objects) {
        command << " --object " << shell_quote(object.airsim_object_name);
    }
    for (const auto& pattern : config.patterns) {
        command << " --pattern " << shell_quote(pattern.airsim_object_pattern);
        max_pattern_matches = std::max(max_pattern_matches, pattern.max_matches);
    }
    if (max_pattern_matches > 0) {
        command << " --max-pattern-matches " << max_pattern_matches;
    }
    return command.str();
}

std::string parse_json_string_optional(const std::string& json, const std::string& key, std::size_t search_start = 0U) {
    const std::string marker = "\"" + key + "\":";
    const auto marker_pos = json.find(marker, search_start);
    if (marker_pos == std::string::npos) {
        return {};
    }
    const auto open_pos = json.find('"', marker_pos + marker.size());
    if (open_pos == std::string::npos) {
        throw std::runtime_error("invalid JSON string for key: " + key);
    }
    std::string value;
    bool escaped = false;
    for (std::size_t i = open_pos + 1U; i < json.size(); ++i) {
        const char ch = json[i];
        if (escaped) {
            value.push_back(ch);
            escaped = false;
            continue;
        }
        if (ch == '\\') {
            escaped = true;
            continue;
        }
        if (ch == '"') {
            return value;
        }
        value.push_back(ch);
    }
    throw std::runtime_error("unterminated JSON string for key: " + key);
}

std::vector<double> parse_json_number_array(const std::string& json, const std::string& key, std::size_t search_start = 0U) {
    const std::string marker = "\"" + key + "\":";
    const auto marker_pos = json.find(marker, search_start);
    if (marker_pos == std::string::npos) {
        throw std::runtime_error("missing JSON array key: " + key);
    }
    const auto open_pos = json.find('[', marker_pos + marker.size());
    const auto close_pos = json.find(']', open_pos);
    if (open_pos == std::string::npos || close_pos == std::string::npos || close_pos <= open_pos) {
        throw std::runtime_error("invalid JSON array for key: " + key);
    }
    std::string body = json.substr(open_pos + 1U, close_pos - open_pos - 1U);
    std::replace(body.begin(), body.end(), ',', ' ');
    std::istringstream input{body};
    std::vector<double> values;
    double value = 0.0;
    while (input >> value) {
        values.push_back(value);
    }
    return values;
}

Vec3 vec3_from_json_array(const std::string& json, const std::string& key, std::size_t search_start = 0U) {
    const auto values = parse_json_number_array(json, key, search_start);
    if (values.size() != 3U) {
        throw std::runtime_error("JSON array key has wrong size: " + key);
    }
    return Vec3{values.at(0), values.at(1), values.at(2)};
}

struct AirSimObjectPose {
    std::string name;
    std::string source_pattern;
    Vec3 position_ned_m;
};

std::vector<AirSimObjectPose> parse_object_pose_json(const std::string& json) {
    std::vector<AirSimObjectPose> poses;
    std::size_t cursor = 0U;
    while (true) {
        const auto name_marker = json.find("\"name\":", cursor);
        if (name_marker == std::string::npos) {
            break;
        }
        const auto object_end = json.find('}', name_marker);
        if (object_end == std::string::npos) {
            throw std::runtime_error("unterminated AirSim object pose JSON object");
        }
        const auto object_json = json.substr(name_marker, object_end - name_marker + 1U);
        AirSimObjectPose pose;
        pose.name = parse_json_string_optional(object_json, "name");
        pose.source_pattern = parse_json_string_optional(object_json, "source_pattern");
        pose.position_ned_m = vec3_from_json_array(object_json, "position_ned_m");
        poses.push_back(std::move(pose));
        cursor = object_end + 1U;
    }
    return poses;
}

const AirSimObjectPose& find_pose_for_object(
    const std::vector<AirSimObjectPose>& poses,
    const std::string& object_name) {
    const auto it = std::find_if(
        poses.begin(),
        poses.end(),
        [&](const AirSimObjectPose& pose) { return pose.name == object_name; });
    if (it == poses.end()) {
        throw std::runtime_error("AirSim object pose response missing object: " + object_name);
    }
    return *it;
}

const AirSimGhostObjectPatternBinding* find_pattern_for_pose(
    const std::vector<AirSimGhostObjectPatternBinding>& patterns,
    const AirSimObjectPose& pose) {
    const auto it = std::find_if(
        patterns.begin(),
        patterns.end(),
        [&](const AirSimGhostObjectPatternBinding& binding) {
            return binding.airsim_object_pattern == pose.source_pattern;
        });
    return it == patterns.end() ? nullptr : &*it;
}

std::string sanitized_id_suffix(const std::string& value) {
    std::string out;
    out.reserve(value.size());
    for (const char ch : value) {
        const auto uch = static_cast<unsigned char>(ch);
        if (std::isalnum(uch) || ch == '_' || ch == '-') {
            out.push_back(ch);
        } else {
            out.push_back('_');
        }
    }
    return out;
}

GhostDetectionState state_from_binding_and_pose(
    const AirSimGhostObjectBinding& binding,
    const AirSimObjectPose& pose) {
    GhostDetectionState state;
    state.source_track_id = binding.source_track_id;
    state.class_label = binding.class_label;
    state.confidence = binding.confidence;
    state.position_local_m = pose.position_ned_m;
    state.velocity_local_mps = Vec3{0.0, 0.0, 0.0};
    state.size_m = binding.size_m;
    return state;
}

GhostDetectionState state_from_pattern_and_pose(
    const AirSimGhostObjectPatternBinding& binding,
    const AirSimObjectPose& pose,
    std::size_t pattern_index) {
    GhostDetectionState state;
    const auto suffix = sanitized_id_suffix(pose.name.empty() ? ("match_" + std::to_string(pattern_index)) : pose.name);
    state.source_track_id = TrackId{binding.source_track_prefix + "_" + suffix};
    state.class_label = binding.class_label;
    state.confidence = binding.confidence;
    state.position_local_m = pose.position_ned_m;
    state.velocity_local_mps = Vec3{0.0, 0.0, 0.0};
    state.size_m = binding.size_m;
    return state;
}

// Abstract backend interface — each backend converts timestamp/elapsed-time
// to a flat list of GhostDetectionState without knowing about frame assembly.
class GhostTargetBackend {
public:
    virtual ~GhostTargetBackend() = default;
    virtual std::vector<GhostDetectionState> detections_at(
        TimePoint timestamp, double scenario_elapsed_s) const = 0;
};

class TrajectoryScenarioBackend final : public GhostTargetBackend {
public:
    explicit TrajectoryScenarioBackend(GhostScenario scenario)
        : scenario_(std::move(scenario)) {}

    std::vector<GhostDetectionState> detections_at(
        TimePoint /*timestamp*/, double scenario_elapsed_s) const override {
        return scenario_.evaluate(scenario_elapsed_s);
    }

private:
    GhostScenario scenario_;
};

class AirSimObjectsBackend final : public GhostTargetBackend {
public:
    explicit AirSimObjectsBackend(AirSimGhostObjectSourceConfig config)
        : config_(std::move(config)),
          transport_(make_transport(config_.bridge_transport)) {
        if (config_.objects.empty() && config_.patterns.empty()) {
            throw std::invalid_argument(
                "AirSim ghost object source requires exact object or pattern bindings");
        }
    }

    std::vector<GhostDetectionState> detections_at(
        TimePoint timestamp, double /*scenario_elapsed_s*/) const override {
        const auto command = build_object_pose_command(config_, timestamp);
        const auto poses = parse_object_pose_json(transport_->request_once(command));
        std::vector<GhostDetectionState> detections;
        detections.reserve(config_.objects.size() + poses.size());
        for (const auto& binding : config_.objects) {
            detections.push_back(state_from_binding_and_pose(
                binding, find_pose_for_object(poses, binding.airsim_object_name)));
        }
        std::size_t pattern_index = 0U;
        for (const auto& pose : poses) {
            if (pose.source_pattern.empty()) {
                continue;
            }
            const auto* binding = find_pattern_for_pose(config_.patterns, pose);
            if (binding == nullptr) {
                continue;
            }
            detections.push_back(state_from_pattern_and_pose(*binding, pose, pattern_index));
            ++pattern_index;
        }
        return detections;
    }

private:
    AirSimGhostObjectSourceConfig config_;
    std::unique_ptr<BridgeTransport> transport_;
};

}  // namespace

struct GhostTargetProvider::Impl {
    std::unique_ptr<GhostTargetBackend> backend;
};

GhostTargetProvider::GhostTargetProvider(GhostScenario scenario)
    : impl_(std::make_unique<Impl>(Impl{std::make_unique<TrajectoryScenarioBackend>(std::move(scenario))})) {}

GhostTargetProvider::GhostTargetProvider(AirSimGhostObjectSourceConfig config)
    : impl_(std::make_unique<Impl>(Impl{std::make_unique<AirSimObjectsBackend>(std::move(config))})) {}

GhostTargetProvider::~GhostTargetProvider() = default;
GhostTargetProvider::GhostTargetProvider(GhostTargetProvider&&) noexcept = default;
GhostTargetProvider& GhostTargetProvider::operator=(GhostTargetProvider&&) noexcept = default;

GhostDetectionsFrame GhostTargetProvider::frame_at(
    TimePoint timestamp,
    MapFrameId map_frame_id,
    TimePoint scenario_start) const {
    GhostDetectionsFrame frame;
    frame.timestamp = timestamp;
    frame.map_frame_id = map_frame_id;
    frame.scenario_elapsed_s = elapsed_seconds(timestamp, scenario_start);
    frame.detections = impl_->backend->detections_at(timestamp, frame.scenario_elapsed_s);
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
    out << ",\"map_frame_id\":" << q(frame.map_frame_id.value);
    out << ",\"scenario_elapsed_s\":" << frame.scenario_elapsed_s;
    out << ",\"detections\":[";
    for (std::size_t index = 0; index < frame.detections.size(); ++index) {
        if (index > 0U) {
            out << ',';
        }
        const auto& detection = frame.detections[index];
        out << '{';
        out << "\"source_track_id\":" << q(detection.source_track_id.value);
        out << ",\"class\":" << q(to_string(detection.class_label));
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
