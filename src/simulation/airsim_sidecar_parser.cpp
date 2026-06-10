#include "dedalus/simulation/airsim_sidecar_parser.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace dedalus {
namespace {

std::string parse_json_string(const std::string& json, const std::string& key) {
    const std::string marker = "\"" + key + "\":";
    const auto marker_pos = json.find(marker);
    if (marker_pos == std::string::npos) {
        throw std::runtime_error("AirSim stream JSON missing key: " + key);
    }

    const auto open_pos = json.find('"', marker_pos + marker.size());
    if (open_pos == std::string::npos) {
        throw std::runtime_error("AirSim stream JSON has invalid string for key: " + key);
    }

    const auto close_pos = json.find('"', open_pos + 1U);
    if (close_pos == std::string::npos) {
        throw std::runtime_error("AirSim stream JSON has unterminated string for key: " + key);
    }

    return json.substr(open_pos + 1U, close_pos - open_pos - 1U);
}

std::vector<double> parse_json_number_array(const std::string& json, const std::string& key, std::size_t expected_size) {
    const std::string marker = "\"" + key + "\":";
    const auto marker_pos = json.find(marker);
    if (marker_pos == std::string::npos) {
        throw std::runtime_error("AirSim ego bridge JSON missing key: " + key);
    }

    const auto open_pos = json.find('[', marker_pos + marker.size());
    const auto close_pos = json.find(']', open_pos);
    if (open_pos == std::string::npos || close_pos == std::string::npos || close_pos <= open_pos) {
        throw std::runtime_error("AirSim ego bridge JSON has invalid array for key: " + key);
    }

    std::string body = json.substr(open_pos + 1U, close_pos - open_pos - 1U);
    std::replace(body.begin(), body.end(), ',', ' ');
    std::istringstream input{body};

    std::vector<double> values;
    double value = 0.0;
    while (input >> value) {
        values.push_back(value);
    }

    if (values.size() != expected_size) {
        throw std::runtime_error("AirSim ego bridge JSON has wrong array size for key: " + key);
    }

    return values;
}

Nanoseconds parse_json_i64(const std::string& json, const std::string& key) {
    const std::string marker = "\"" + key + "\":";
    const auto marker_pos = json.find(marker);
    if (marker_pos == std::string::npos) {
        throw std::runtime_error("AirSim bridge JSON missing key: " + key);
    }

    const auto value_start = marker_pos + marker.size();
    const auto value_end = json.find_first_of(",}\n\r\t ", value_start);
    const auto token = json.substr(value_start, value_end == std::string::npos ? std::string::npos : value_end - value_start);
    return static_cast<Nanoseconds>(std::stoll(token));
}

double parse_json_double_or(const std::string& json, const std::string& key, double fallback) {
    const std::string marker = "\"" + key + "\":";
    const auto marker_pos = json.find(marker);
    if (marker_pos == std::string::npos) {
        return fallback;
    }

    const auto value_start = marker_pos + marker.size();
    const auto value_end = json.find_first_of(",}\n\r\t ", value_start);
    const auto token = json.substr(value_start, value_end == std::string::npos ? std::string::npos : value_end - value_start);
    return std::stod(token);
}

std::optional<bool> parse_json_bool_optional(const std::string& json, const std::string& key) {
    const std::string marker = "\"" + key + "\":";
    const auto marker_pos = json.find(marker);
    if (marker_pos == std::string::npos) {
        return std::nullopt;
    }
    const auto value_start = marker_pos + marker.size();
    if (json.compare(value_start, 4U, "true") == 0) {
        return true;
    }
    if (json.compare(value_start, 5U, "false") == 0) {
        return false;
    }
    return std::nullopt;
}

std::optional<int> parse_json_int_optional(const std::string& json, const std::string& key) {
    const std::string marker = "\"" + key + "\":";
    const auto marker_pos = json.find(marker);
    if (marker_pos == std::string::npos) {
        return std::nullopt;
    }
    const auto value_start = marker_pos + marker.size();
    const auto value_end = json.find_first_of(",}\n\r\t ", value_start);
    const auto token = json.substr(value_start, value_end == std::string::npos ? std::string::npos : value_end - value_start);
    return std::stoi(token);
}

std::optional<std::vector<float>> parse_json_float_array_optional(const std::string& json, const std::string& key) {
    const std::string marker = "\"" + key + "\":";
    const auto marker_pos = json.find(marker);
    if (marker_pos == std::string::npos) {
        return std::nullopt;
    }
    const auto open_pos = json.find('[', marker_pos + marker.size());
    const auto close_pos = json.find(']', open_pos);
    if (open_pos == std::string::npos || close_pos == std::string::npos || close_pos <= open_pos) {
        return std::nullopt;
    }

    std::string body = json.substr(open_pos + 1U, close_pos - open_pos - 1U);
    std::replace(body.begin(), body.end(), ',', ' ');
    std::istringstream input{body};
    std::vector<float> values;
    float value = 0.0F;
    while (input >> value) {
        values.push_back(value);
    }
    return values;
}


}  // namespace

std::optional<AirSimDepthFrame> parse_depth_frame_optional(
    const std::string& json,
    const FramePacket& frame,
    const MapFrameId& map_frame_id) {
    const auto width = parse_json_int_optional(json, "depth_width");
    const auto height = parse_json_int_optional(json, "depth_height");
    const auto depth = parse_json_float_array_optional(json, "depth_m");
    if (!width.has_value() || !height.has_value() || !depth.has_value()) {
        return std::nullopt;
    }
    if (*width <= 0 || *height <= 0 ||
        depth->size() < static_cast<std::size_t>(*width) * static_cast<std::size_t>(*height)) {
        return std::nullopt;
    }
    std::vector<float> normal_values;

    AirSimDepthFrame depth_frame;
    depth_frame.timestamp = frame.timestamp;
    depth_frame.source_frame_id = frame.frame_id;
    depth_frame.has_source_frame = true;
    depth_frame.sensor_name = frame.camera_id.value;
    depth_frame.map_frame_id = map_frame_id;
    depth_frame.width = *width;
    depth_frame.height = *height;
    depth_frame.depth_m = *depth;
    return depth_frame;
}

Vec3 to_vec3(const std::vector<double>& values) {
    return Vec3{values.at(0), values.at(1), values.at(2)};
}

EgoState parse_ego_json(const std::string& json, const MapFrameId& map_frame_id, TimePoint frame_timestamp) {
    EgoState ego;
    ego.timestamp = TimePoint{parse_json_i64(json, "timestamp_ns")};
    if (ego.timestamp.timestamp_ns == 0) {
        ego.timestamp = frame_timestamp;
    }
    ego.local_T_body.position = to_vec3(parse_json_number_array(json, "position", 3U));
    ego.local_T_body.rotation_rpy = to_vec3(parse_json_number_array(json, "rotation_rpy", 3U));
    ego.velocity_local = to_vec3(parse_json_number_array(json, "velocity", 3U));
    ego.angular_velocity_body = to_vec3(parse_json_number_array(json, "angular_velocity", 3U));
    ego.height_m = parse_json_double_or(json, "height_m", -ego.local_T_body.position.z);
    if (const auto height_valid = parse_json_bool_optional(json, "height_valid")) {
        ego.height_valid = *height_valid;
    } else {
        ego.height_valid = true;
    }
    if (const auto armed = parse_json_bool_optional(json, "armed")) {
        ego.armed = *armed;
        ego.armed_valid = true;
    }
    if (const auto armed_valid = parse_json_bool_optional(json, "armed_valid")) {
        ego.armed_valid = *armed_valid;
    }
    ego.flight_status = ego.height_valid && ego.height_m > 0.25
        ? EgoFlightStatus::Airborne
        : EgoFlightStatus::Landed;
    ego.map_frame_id = map_frame_id;
    return ego;
}


}  // namespace dedalus
