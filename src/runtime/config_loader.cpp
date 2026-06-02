#include "dedalus/runtime/config_loader.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace dedalus {
namespace {

std::string trim(std::string value) {
    const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };

    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

std::string strip_quotes(std::string value) {
    if (value.size() >= 2U) {
        const char first = value.front();
        const char last = value.back();
        if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
            return value.substr(1U, value.size() - 2U);
        }
    }
    return value;
}

bool parse_bool(const std::string& value) {
    if (value == "true" || value == "1" || value == "yes" || value == "on") {
        return true;
    }
    if (value == "false" || value == "0" || value == "no" || value == "off") {
        return false;
    }
    throw std::invalid_argument("invalid boolean config value: " + value);
}

Vec3 parse_vec3(const std::string& value) {
    std::string normalized;
    normalized.reserve(value.size());
    for (const char ch : value) {
        if (ch == '[' || ch == ']' || ch == ',') {
            normalized.push_back(' ');
        } else {
            normalized.push_back(ch);
        }
    }

    std::istringstream input{normalized};
    Vec3 result;
    if (!(input >> result.x >> result.y >> result.z)) {
        throw std::invalid_argument("invalid vec3 config value: " + value);
    }
    std::string extra;
    if (input >> extra) {
        throw std::invalid_argument("invalid vec3 config value with extra data: " + value);
    }
    return result;
}

std::string mission_option_key_from(const std::string& key) {
    constexpr auto prefix = "mission_options.";
    return key.substr(std::string{prefix}.size());
}

bool parse_airsim_object_binding_key(
    CoreStackProviderConfig& config,
    const std::string& key,
    const std::string& value) {
    constexpr auto prefix = "ghost_targets_airsim.objects.";
    if (key.rfind(prefix, 0U) != 0U) {
        return false;
    }

    const auto remainder = key.substr(std::string{prefix}.size());
    const auto dot_pos = remainder.find('.');
    if (dot_pos == std::string::npos || dot_pos == 0U || dot_pos + 1U >= remainder.size()) {
        throw std::invalid_argument("invalid AirSim ghost object binding key: " + key);
    }

    const auto index_text = remainder.substr(0U, dot_pos);
    const auto field = remainder.substr(dot_pos + 1U);
    std::size_t parsed_chars = 0U;
    const auto index = std::stoul(index_text, &parsed_chars);
    if (parsed_chars != index_text.size()) {
        throw std::invalid_argument("invalid AirSim ghost object binding index: " + key);
    }
    if (index >= 1024U) {
        throw std::invalid_argument("unreasonable AirSim ghost object binding index: " + key);
    }
    if (config.ghost_targets_airsim_objects.size() <= index) {
        config.ghost_targets_airsim_objects.resize(index + 1U);
    }

    auto& binding = config.ghost_targets_airsim_objects[index];
    if (field == "source_track_id") {
        binding.source_track_id = TrackId{value};
    } else if (field == "airsim_object_name") {
        binding.airsim_object_name = value;
    } else if (field == "class") {
        binding.class_label = value;
    } else if (field == "confidence") {
        binding.confidence = std::stod(value);
    } else if (field == "size_m") {
        binding.size_m = parse_vec3(value);
    } else {
        throw std::invalid_argument("unknown AirSim ghost object binding field: " + key);
    }
    return true;
}

void apply_config_value(CoreStackProviderConfig& config, const std::string& key, const std::string& value) {
    if (parse_airsim_object_binding_key(config, key, value)) {
        return;
    }

    if (key == "frame_source") {
        config.frame_source = value;
    } else if (key == "ego_provider") {
        config.ego_provider = value;
    } else if (key == "detector") {
        config.detector = value;
    } else if (key == "camera_stabilizer") {
        config.camera_stabilizer = value;
    } else if (key == "tracker") {
        config.tracker = value;
    } else if (key == "identity_resolver") {
        config.identity_resolver = value;
    } else if (key == "projector") {
        config.projector = value;
    } else if (key == "ghost_targets_enabled") {
        config.ghost_targets_enabled = parse_bool(value);
    } else if (key == "ghost_targets_source") {
        config.ghost_targets_source = value;
    } else if (key == "ghost_targets_scenario") {
        config.ghost_targets_scenario = value;
    } else if (key == "ghost_targets_scenario_path") {
        config.ghost_targets_scenario_path = value;
    } else if (key == "world_model") {
        config.world_model = value;
    } else if (key == "track4_occupancy_source") {
        config.track4_occupancy_source = value;
    } else if (key == "frame_annotator") {
        config.frame_annotator = value;
    } else if (key == "annotation_output_path") {
        config.annotation_output_path = value;
    } else if (key == "annotation_output_fps") {
        config.annotation_output_fps = std::stod(value);
    } else if (key == "pipeline_timing_enabled") {
        config.pipeline_timing_enabled = parse_bool(value);
    } else if (key == "pipeline_timing_output_path") {
        config.pipeline_timing_output_path = value;
    } else if (key == "recorded_manifest_path") {
        config.recorded_manifest_path = value;
    } else if (key == "source_host") {
        config.source_host = value;
    } else if (key == "source_rpc_port") {
        config.source_rpc_port = std::stoi(value);
    } else if (key == "vehicle_name") {
        config.vehicle_name = value;
    } else if (key == "vehicle_camera_name") {
        config.vehicle_camera_name = value;
    } else if (key == "bridge_transport") {
        config.bridge_transport = value;
    } else if (key == "bridge_command") {
        config.bridge_command = value;
    } else if (key == "bridge_mode") {
        config.bridge_mode = value;
    } else if (key == "ego_bridge_command") {
        config.ego_bridge_command = value;
    } else if (key == "fallback_map_frame_id") {
        config.fallback_map_frame_id = MapFrameId{value};
    } else if (key == "mission_controller") {
        config.mission_controller = value;
    } else if (key == "mission_tick_hz") {
        config.mission_tick_hz = std::stod(value);
    } else if (key == "flight_command_sink") {
        config.flight_command_sink = value;
    } else if (key.rfind("mission_options.", 0U) == 0U) {
        const auto option_key = mission_option_key_from(key);
        if (option_key.empty()) {
            throw std::invalid_argument("empty mission_options key");
        }
        config.mission_options.values[option_key] = value;
        if (MissionOptions::known_keys().find(option_key) == MissionOptions::known_keys().end()) {
            std::cerr << "WARNING: unrecognized mission_options key: " << option_key
                      << " (check for typos; value will be ignored)\n";
        }
    } else {
        throw std::invalid_argument("unknown core-stack config key: " + key);
    }
}

void validate_airsim_object_bindings(const CoreStackProviderConfig& config) {
    if (config.ghost_targets_source != "airsim_objects") {
        return;
    }
    if (config.ghost_targets_airsim_objects.empty()) {
        throw std::invalid_argument("ghost_targets_source=airsim_objects requires at least one ghost_targets_airsim.objects.<N> binding");
    }
    for (std::size_t index = 0; index < config.ghost_targets_airsim_objects.size(); ++index) {
        const auto& binding = config.ghost_targets_airsim_objects[index];
        const auto prefix = "ghost_targets_airsim.objects." + std::to_string(index);
        if (binding.source_track_id.value.empty()) {
            throw std::invalid_argument(prefix + ".source_track_id is required");
        }
        if (binding.airsim_object_name.empty()) {
            throw std::invalid_argument(prefix + ".airsim_object_name is required");
        }
        if (binding.class_label.empty()) {
            throw std::invalid_argument(prefix + ".class is required");
        }
        if (binding.confidence < 0.0 || binding.confidence > 1.0) {
            throw std::invalid_argument(prefix + ".confidence must be in [0, 1]");
        }
        if (binding.size_m.x <= 0.0 || binding.size_m.y <= 0.0 || binding.size_m.z <= 0.0) {
            throw std::invalid_argument(prefix + ".size_m components must be positive");
        }
    }
}

void validate_config(const CoreStackProviderConfig& config) {
    if (config.ghost_targets_source != "trajectory_scenario" && config.ghost_targets_source != "airsim_objects") {
        throw std::invalid_argument("unknown ghost_targets_source: " + config.ghost_targets_source);
    }
    if (config.ghost_targets_source != "airsim_objects" && !config.ghost_targets_airsim_objects.empty()) {
        throw std::invalid_argument("ghost_targets_airsim.objects.* requires ghost_targets_source=airsim_objects");
    }
    if (config.track4_occupancy_source != "synthetic_fixture" && config.track4_occupancy_source != "airsim_ground_truth") {
        throw std::invalid_argument("unknown track4_occupancy_source: " + config.track4_occupancy_source);
    }
    if (config.track4_occupancy_source == "airsim_ground_truth" && config.ghost_targets_source != "airsim_objects") {
        throw std::invalid_argument("track4_occupancy_source=airsim_ground_truth requires ghost_targets_source=airsim_objects for the 4.0F-GT named-object provider");
    }
    validate_airsim_object_bindings(config);
}

}  // namespace

CoreStackProviderConfig load_core_stack_config(const std::string& path) {
    std::ifstream input{path};
    if (!input) {
        throw std::runtime_error("failed to open core-stack config: " + path);
    }

    CoreStackProviderConfig config;
    std::string line;
    int line_number = 0;

    while (std::getline(input, line)) {
        ++line_number;

        const auto comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
            line = line.substr(0U, comment_pos);
        }

        line = trim(line);
        if (line.empty()) {
            continue;
        }

        const auto separator_pos = line.find(':');
        if (separator_pos == std::string::npos) {
            throw std::runtime_error(
                "invalid core-stack config line " + std::to_string(line_number) + ": missing ':'");
        }

        const std::string key = trim(line.substr(0U, separator_pos));
        const std::string value = strip_quotes(trim(line.substr(separator_pos + 1U)));
        if (key.empty() || value.empty()) {
            throw std::runtime_error(
                "invalid core-stack config line " + std::to_string(line_number) + ": empty key or value");
        }

        try {
            apply_config_value(config, key, value);
        } catch (const std::exception& ex) {
            throw std::runtime_error(
                "invalid core-stack config line " + std::to_string(line_number) + ": " + ex.what());
        }
    }

    validate_config(config);
    return config;
}

}  // namespace dedalus
