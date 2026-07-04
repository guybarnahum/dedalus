#include "dedalus/perception/ghost_targets.hpp"

#include "dedalus/core/json_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <set>
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
    if (delta_ns <= 0) return 0.0;
    return static_cast<double>(delta_ns) / kNsPerSecond;
}

std::string vec3_json(const Vec3& value) {
    std::ostringstream out;
    out << '[' << value.x << ',' << value.y << ',' << value.z << ']';
    return out.str();
}

Observation3D observation_from_state(const GhostDetectionState& state, TimePoint timestamp, const MapFrameId& map_frame_id) {
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
    if (transport_name == "pipe") return std::make_unique<PipeBridgeTransport>();
    if (transport_name == "shared_memory") {
        throw std::runtime_error("shared_memory bridge transport is not yet implemented; use bridge_transport: pipe");
    }
    throw std::runtime_error("unknown AirSim ghost object bridge transport: " + transport_name);
}

std::string shell_quote(const std::string& value) {
    std::string quoted = "'";
    for (const char ch : value) quoted += (ch == '\'') ? "'\\''" : std::string(1, ch);
    quoted += "'";
    return quoted;
}

bool is_dynamic_class(ClassLabel label) {
    return label == ClassLabel::Person || label == ClassLabel::Car ||
           label == ClassLabel::Drone || label == ClassLabel::Animal ||
           label == ClassLabel::Boat;
}

std::string build_object_pose_stream_command(
    const AirSimGhostObjectSourceConfig& config,
    const std::vector<AirSimGhostObjectBinding>& objects) {
    validate_bridge_base_command(config.bridge_command);
    std::ostringstream command;
    command << config.bridge_command
            << " --host " << shell_quote(config.host)
            << " --rpc-port " << config.rpc_port
            << " --stream-jsonl"
            << " --stream-rate-hz " << config.stream_rate_hz
            << " --static-refresh-every-frames " << config.static_refresh_every_n_frames;
    for (const auto& object : objects) {
        command << " --object " << shell_quote(object.airsim_object_name);
    }
    for (const auto& object : objects) {
        if (is_dynamic_class(object.class_label)) {
            command << " --dynamic-object " << shell_quote(object.airsim_object_name);
        }
    }
    return command.str();
}

std::string read_text_file(const std::string& path) {
    std::ifstream input{path};
    if (!input) throw std::runtime_error("failed to open AirSim scene inventory: " + path);
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

std::size_t find_matching_delim(const std::string& text, std::size_t open_pos, char open_ch, char close_ch) {
    int depth = 0;
    bool in_string = false;
    bool escaped = false;
    for (std::size_t i = open_pos; i < text.size(); ++i) {
        const char ch = text[i];
        if (escaped) { escaped = false; continue; }
        if (ch == '\\') { escaped = true; continue; }
        if (ch == '"') { in_string = !in_string; continue; }
        if (in_string) continue;
        if (ch == open_ch) ++depth;
        else if (ch == close_ch) {
            --depth;
            if (depth == 0) return i;
        }
    }
    throw std::runtime_error("JSON has unmatched delimiter");
}

std::string parse_json_string_optional(const std::string& json, const std::string& key, std::size_t search_start = 0U) {
    const std::string marker = "\"" + key + "\":";
    const auto marker_pos = json.find(marker, search_start);
    if (marker_pos == std::string::npos) return {};
    const auto open_pos = json.find('"', marker_pos + marker.size());
    if (open_pos == std::string::npos) throw std::runtime_error("invalid JSON string for key: " + key);
    std::string value;
    bool escaped = false;
    for (std::size_t i = open_pos + 1U; i < json.size(); ++i) {
        const char ch = json[i];
        if (escaped) { value.push_back(ch); escaped = false; continue; }
        if (ch == '\\') { escaped = true; continue; }
        if (ch == '"') return value;
        value.push_back(ch);
    }
    throw std::runtime_error("unterminated JSON string for key: " + key);
}

std::vector<double> parse_json_number_array_optional(const std::string& json, const std::string& key, std::size_t search_start = 0U) {
    const std::string marker = "\"" + key + "\":";
    const auto marker_pos = json.find(marker, search_start);
    if (marker_pos == std::string::npos) return {};
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
    while (input >> value) values.push_back(value);
    return values;
}

Vec3 vec3_from_json_array(const std::string& json, const std::string& key, std::size_t search_start = 0U) {
    const auto values = parse_json_number_array_optional(json, key, search_start);
    if (values.size() != 3U) throw std::runtime_error("JSON array key has wrong size: " + key);
    return Vec3{values.at(0), values.at(1), values.at(2)};
}

std::optional<Vec3> optional_vec3_from_json_array(const std::string& json, const std::string& key) {
    const auto values = parse_json_number_array_optional(json, key);
    if (values.empty()) return std::nullopt;
    if (values.size() != 3U) throw std::runtime_error("JSON array key has wrong size: " + key);
    return Vec3{values.at(0), values.at(1), values.at(2)};
}

double distance_m(const Vec3& a, const Vec3& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

struct AirSimObjectPose {
    std::string name;
    std::string source_pattern;
    Vec3 position_ned_m;
};

struct SceneInventoryObject {
    std::string name;
    std::string canonical_class;
    std::optional<Vec3> position_ned_m;
};

struct InventoryCandidate {
    AirSimGhostObjectBinding binding;
    std::optional<Vec3> inventory_position_local;
};

std::vector<AirSimObjectPose> parse_object_pose_json(const std::string& json) {
    std::vector<AirSimObjectPose> poses;
    std::size_t cursor = 0U;
    while (true) {
        const auto name_marker = json.find("\"name\":", cursor);
        if (name_marker == std::string::npos) break;
        const auto object_start = json.rfind('{', name_marker);
        const auto object_end = find_matching_delim(json, object_start, '{', '}');
        const auto object_json = json.substr(object_start, object_end - object_start + 1U);
        AirSimObjectPose pose;
        pose.name = parse_json_string_optional(object_json, "name");
        pose.source_pattern = parse_json_string_optional(object_json, "source_pattern");
        pose.position_ned_m = vec3_from_json_array(object_json, "position_ned_m");
        poses.push_back(std::move(pose));
        cursor = object_end + 1U;
    }
    return poses;
}

std::vector<SceneInventoryObject> parse_scene_inventory_objects(const std::string& json) {
    const auto objects_key = json.find("\"objects\"");
    if (objects_key == std::string::npos) throw std::runtime_error("AirSim scene inventory missing objects array");
    const auto array_start = json.find('[', objects_key);
    if (array_start == std::string::npos) throw std::runtime_error("AirSim scene inventory objects field is not an array");
    const auto array_end = find_matching_delim(json, array_start, '[', ']');
    std::vector<SceneInventoryObject> objects;
    std::size_t cursor = array_start + 1U;
    while (cursor < array_end) {
        const auto object_start = json.find('{', cursor);
        if (object_start == std::string::npos || object_start > array_end) break;
        const auto object_end = find_matching_delim(json, object_start, '{', '}');
        const auto object_json = json.substr(object_start, object_end - object_start + 1U);
        SceneInventoryObject object;
        object.name = parse_json_string_optional(object_json, "name");
        object.canonical_class = parse_json_string_optional(object_json, "canonical_class");
        object.position_ned_m = optional_vec3_from_json_array(object_json, "position_ned_m");
        if (!object.name.empty()) objects.push_back(std::move(object));
        cursor = object_end + 1U;
    }
    return objects;
}

const AirSimObjectPose& find_pose_for_object(const std::vector<AirSimObjectPose>& poses, const std::string& object_name) {
    const auto it = std::find_if(poses.begin(), poses.end(), [&](const AirSimObjectPose& pose) { return pose.name == object_name; });
    if (it == poses.end()) throw std::runtime_error("AirSim object pose response missing object: " + object_name);
    return *it;
}

const AirSimGhostObjectPatternBinding* find_pattern_for_pose(const std::vector<AirSimGhostObjectPatternBinding>& patterns, const AirSimObjectPose& pose) {
    const auto it = std::find_if(patterns.begin(), patterns.end(), [&](const AirSimGhostObjectPatternBinding& binding) {
        return binding.airsim_object_pattern == pose.source_pattern;
    });
    return it == patterns.end() ? nullptr : &*it;
}

std::string sanitized_id_suffix(const std::string& value) {
    std::string out;
    out.reserve(value.size());
    for (const char ch : value) {
        const auto uch = static_cast<unsigned char>(ch);
        if (std::isalnum(uch) || ch == '_' || ch == '-') out.push_back(ch);
        else out.push_back('_');
    }
    return out;
}

GhostDetectionState state_from_binding_and_pose(const AirSimGhostObjectBinding& binding, const AirSimObjectPose& pose) {
    GhostDetectionState state;
    state.source_track_id = binding.source_track_id;
    state.class_label = binding.class_label;
    state.confidence = binding.confidence;
    state.position_local_m = pose.position_ned_m;
    state.velocity_local_mps = Vec3{0.0, 0.0, 0.0};
    state.size_m = binding.size_m;
    return state;
}

GhostDetectionState state_from_pattern_and_pose(const AirSimGhostObjectPatternBinding& binding, const AirSimObjectPose& pose, std::size_t pattern_index) {
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

bool inventory_object_matches_pattern(const SceneInventoryObject& object, const AirSimGhostObjectPatternBinding& binding) {
    const auto class_name = to_string(binding.class_label);
    if (!object.canonical_class.empty() && object.canonical_class == class_name) return true;
    try { return std::regex_search(object.name, std::regex(binding.airsim_object_pattern)); }
    catch (const std::regex_error&) { return false; }
}

std::vector<InventoryCandidate> expand_patterns_from_scene_inventory(const AirSimGhostObjectSourceConfig& config) {
    if (config.scene_inventory_path.empty() || config.patterns.empty()) return {};
    const auto inventory_objects = parse_scene_inventory_objects(read_text_file(config.scene_inventory_path));
    std::vector<InventoryCandidate> expanded;
    std::set<std::string> used_names;
    for (const auto& existing : config.objects) used_names.insert(existing.airsim_object_name);
    for (const auto& pattern : config.patterns) {
        int matched = 0;
        for (const auto& object : inventory_objects) {
            if (pattern.max_matches > 0 && matched >= pattern.max_matches) break;
            if (used_names.find(object.name) != used_names.end()) continue;
            if (!inventory_object_matches_pattern(object, pattern)) continue;
            AirSimGhostObjectBinding binding;
            binding.source_track_id = TrackId{pattern.source_track_prefix + "_" + sanitized_id_suffix(object.name)};
            binding.airsim_object_name = object.name;
            binding.class_label = pattern.class_label;
            binding.confidence = pattern.confidence;
            binding.size_m = pattern.size_m;
            expanded.push_back(InventoryCandidate{.binding = std::move(binding), .inventory_position_local = object.position_ned_m});
            used_names.insert(object.name);
            ++matched;
        }
    }
    return expanded;
}

std::vector<AirSimGhostObjectBinding> select_nearby_candidates(
    const AirSimGhostObjectSourceConfig& config,
    const std::vector<InventoryCandidate>& candidates,
    std::optional<Vec3> ego_position_local) {
    std::vector<AirSimGhostObjectBinding> selected = config.objects;
    if (config.max_objects_per_frame > 0 && static_cast<int>(selected.size()) >= config.max_objects_per_frame) {
        selected.resize(static_cast<std::size_t>(config.max_objects_per_frame));
        return selected;
    }

    struct RankedCandidate {
        InventoryCandidate candidate;
        double distance_m{0.0};
    };

    std::map<ClassLabel, std::vector<RankedCandidate>> by_class;
    for (const auto& candidate : candidates) {
        double dist = 0.0;
        if (ego_position_local.has_value() && candidate.inventory_position_local.has_value()) {
            dist = distance_m(*ego_position_local, *candidate.inventory_position_local);
            if (config.nearby_radius_m > 0.0 && dist > config.nearby_radius_m && !is_dynamic_class(candidate.binding.class_label)) {
                continue;
            }
        }
        by_class[candidate.binding.class_label].push_back(RankedCandidate{.candidate = candidate, .distance_m = dist});
    }
    for (auto& [_, bucket] : by_class) {
        std::sort(bucket.begin(), bucket.end(), [](const RankedCandidate& a, const RankedCandidate& b) {
            return a.distance_m < b.distance_m;
        });
    }

    bool added = true;
    std::size_t round = 0U;
    while (added) {
        added = false;
        for (auto& [_, bucket] : by_class) {
            if (round >= bucket.size()) continue;
            if (config.max_objects_per_frame > 0 && static_cast<int>(selected.size()) >= config.max_objects_per_frame) return selected;
            selected.push_back(bucket[round].candidate.binding);
            added = true;
        }
        ++round;
    }
    return selected;
}

class GhostTargetBackend {
public:
    virtual ~GhostTargetBackend() = default;
    virtual void pre_warm(std::optional<Vec3> /*ego_position_local*/) const {}
    virtual std::vector<GhostDetectionState> detections_at(TimePoint timestamp, double scenario_elapsed_s, std::optional<Vec3> ego_position_local) const = 0;
};

class TrajectoryScenarioBackend final : public GhostTargetBackend {
public:
    explicit TrajectoryScenarioBackend(GhostScenario scenario) : scenario_(std::move(scenario)) {}

    void pre_warm(std::optional<Vec3> /*ego_position_local*/) const override {}  // no subprocess

    std::vector<GhostDetectionState> detections_at(TimePoint /*timestamp*/, double scenario_elapsed_s, std::optional<Vec3> /*ego_position_local*/) const override {
        return scenario_.evaluate(scenario_elapsed_s);
    }

private:
    GhostScenario scenario_;
};

class AirSimObjectsBackend final : public GhostTargetBackend {
public:
    explicit AirSimObjectsBackend(AirSimGhostObjectSourceConfig config) : config_(std::move(config)), transport_(make_transport(config_.bridge_transport)) {
        if (config_.objects.empty() && config_.patterns.empty()) {
            throw std::invalid_argument("AirSim ghost object source requires exact object or pattern bindings");
        }
        inventory_candidates_ = expand_patterns_from_scene_inventory(config_);
        if (!inventory_candidates_.empty()) config_.patterns.clear();
    }

    // Start the subprocess pipe early (before the first frame loop call) so
    // it has time to connect to AirSim.  Ego position is optional; when absent
    // the full object list is streamed (fine when no scene inventory is loaded).
    void pre_warm(std::optional<Vec3> ego_position_local) const override {
        if (stream_started_) return;
        selected_stream_objects_ = select_nearby_candidates(config_, inventory_candidates_, ego_position_local);
        stream_command_ = build_object_pose_stream_command(config_, selected_stream_objects_);
        stream_started_ = true;
        transport_->open_stream(stream_command_);
        std::cerr << "ghost_targets: pre-warm started subprocess: " << stream_command_ << "\n";
    }

    std::vector<GhostDetectionState> detections_at(TimePoint /*timestamp*/, double /*scenario_elapsed_s*/, std::optional<Vec3> ego_position_local) const override {
        if (!stream_started_) {
            selected_stream_objects_ = select_nearby_candidates(config_, inventory_candidates_, ego_position_local);
            stream_command_ = build_object_pose_stream_command(config_, selected_stream_objects_);
            stream_started_ = true;
        }
        const auto line = transport_->read_stream_line(stream_command_);
        if (!line.has_value()) throw std::runtime_error("persistent AirSim object-pose stream ended unexpectedly");
        std::vector<AirSimObjectPose> poses = parse_object_pose_json(*line);
        const std::vector<AirSimGhostObjectBinding>* bindings = &selected_stream_objects_;

        std::vector<GhostDetectionState> detections;
        detections.reserve(bindings->size() + poses.size());
        for (const auto& binding : *bindings) {
            detections.push_back(state_from_binding_and_pose(binding, find_pose_for_object(poses, binding.airsim_object_name)));
        }
        std::size_t pattern_index = 0U;
        for (const auto& pose : poses) {
            if (pose.source_pattern.empty()) continue;
            const auto* binding = find_pattern_for_pose(config_.patterns, pose);
            if (binding == nullptr) continue;
            detections.push_back(state_from_pattern_and_pose(*binding, pose, pattern_index));
            ++pattern_index;
        }
        return detections;
    }

private:
    AirSimGhostObjectSourceConfig config_;
    std::unique_ptr<BridgeTransport> transport_;
    std::vector<InventoryCandidate> inventory_candidates_;
    mutable bool stream_started_{false};
    mutable std::vector<AirSimGhostObjectBinding> selected_stream_objects_;
    mutable std::string stream_command_;
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

void GhostTargetProvider::pre_warm(std::optional<Vec3> ego_position_local) const {
    impl_->backend->pre_warm(ego_position_local);
}

GhostDetectionsFrame GhostTargetProvider::frame_at(TimePoint timestamp, MapFrameId map_frame_id, TimePoint scenario_start, std::optional<Vec3> ego_position_local) const {
    GhostDetectionsFrame frame;
    frame.timestamp = timestamp;
    frame.map_frame_id = map_frame_id;
    frame.scenario_elapsed_s = elapsed_seconds(timestamp, scenario_start);
    frame.detections = impl_->backend->detections_at(timestamp, frame.scenario_elapsed_s, ego_position_local);
    frame.observations.reserve(frame.detections.size());
    for (const auto& state : frame.detections) {
        frame.observations.push_back(observation_from_state(state, timestamp, frame.map_frame_id));
    }
    return frame;
}

std::vector<Observation3D> GhostTargetProvider::observations_at(TimePoint timestamp, MapFrameId map_frame_id, TimePoint scenario_start, std::optional<Vec3> ego_position_local) const {
    return frame_at(timestamp, map_frame_id, scenario_start, ego_position_local).observations;
}

PerceptionPipelineOutput GhostTargetProvider::output_at(TimePoint timestamp, MapFrameId map_frame_id, TimePoint scenario_start, std::optional<Vec3> ego_position_local) const {
    PerceptionPipelineOutput output;
    output.observations = observations_at(timestamp, map_frame_id, scenario_start, ego_position_local);
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
        if (index > 0U) out << ',';
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
