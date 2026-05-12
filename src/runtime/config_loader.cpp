#include "dedalus/runtime/config_loader.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
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

void apply_config_value(CoreStackProviderConfig& config, const std::string& key, const std::string& value) {
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
    } else if (key == "world_model") {
        config.world_model = value;
    } else if (key == "frame_annotator") {
        config.frame_annotator = value;
    } else if (key == "annotation_output_path") {
        config.annotation_output_path = value;
    } else if (key == "annotation_output_fps") {
        config.annotation_output_fps = std::stod(value);
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
    } else {
        throw std::invalid_argument("unknown core-stack config key: " + key);
    }
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

        apply_config_value(config, key, value);
    }

    return config;
}

}  // namespace dedalus
