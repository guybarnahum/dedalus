#include "dedalus/perception/ghost_targets.hpp"

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
        return std::make_unique<SharedMemoryBridgeTransport>();
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
    for (const auto& object : config.objects) {
        command << " --object " << shell_quote(object.airsim_object_name);
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
        AirSimObjectPose pose;
        pose.name = parse_json_string_optional(json, "name", name_marker);
        pose.position_ned_m = vec3_from_json_array(json, "position_ned_m", name_marker);
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

GhostDetectionState state_from_binding_and_pose(
    const AirSimGhostObjectBinding& binding,
    const AirSimObjectPose& pose) {
    GhostDetectionState state;
    state.source_track_id = binding.source_track_id;
    state.class_label = class_label_from_string(binding.class_label);
    state.confidence = binding.confidence;
    state.position_local_m = pose.position_ned_m;
    state.velocity_local_mps = Vec3{0.0, 0.0, 0.0};
    state.size_m = binding.size_m;
    return state;
}

}  // namespace

struct GhostTargetProvider::Impl {
    enum class SourceType {
        TrajectoryScenario,
        AirSimObjects,
    };

    explicit Impl(GhostScenario scenario)
        : source_type(SourceType::TrajectoryScenario), scenario(std::move(scenario)) {}

    explicit Impl(AirSimGhostObjectSourceConfig source_config)
        : source_type(SourceType::AirSimObjects), airsim_config(std::move(source_config)) {
        if (airsim_config.objects.empty()) {
            throw std::invalid_argument("AirSim ghost object source requires at least one object binding");
        }
        transport = make_transport(airsim_config.bridge_transport);
    }

    SourceType source_type{SourceType::TrajectoryScenario};
    GhostScenario scenario;
    AirSimGhostObjectSourceConfig airsim_config;
    std::unique_ptr<BridgeTransport> transport;
};

GhostTargetProvider::GhostTargetProvider(GhostScenario scenario)
    : impl_(std::make_unique<Impl>(std::move(scenario))) {}

GhostTargetProvider::GhostTargetProvider(AirSimGhostObjectSourceConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

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

    if (impl_->source_type == Impl::SourceType::TrajectoryScenario) {
        frame.detections = impl_->scenario.evaluate(frame.scenario_elapsed_s);
    } else {
        const auto command = build_object_pose_command(impl_->airsim_config, timestamp);
        const auto poses = parse_object_pose_json(impl_->transport->request_once(command));
        frame.detections.reserve(impl_->airsim_config.objects.size());
        for (const auto& binding : impl_->airsim_config.objects) {
            frame.detections.push_back(state_from_binding_and_pose(
                binding,
                find_pose_for_object(poses, binding.airsim_object_name)));
        }
    }

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
