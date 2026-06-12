#include "dedalus/simulation/ghost_scenario.hpp"

#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dedalus {
namespace {

std::string read_text_file(const std::string& path) {
    std::ifstream input{path};
    if (!input) {
        throw std::runtime_error("failed to open ghost scenario file: " + path);
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

std::string parent_dir_or_dot(const std::string& path) {
    const auto parent = std::filesystem::path(path).parent_path();
    if (parent.empty()) {
        return ".";
    }
    return parent.string();
}

std::string resolve_path(const std::string& base_dir, const std::string& path) {
    const std::filesystem::path candidate{path};
    if (candidate.is_absolute()) {
        return candidate.string();
    }
    return (std::filesystem::path(base_dir) / candidate).lexically_normal().string();
}

std::size_t find_matching_delim(const std::string& text, std::size_t open_pos, char open_ch, char close_ch) {
    int depth = 0;
    bool in_string = false;
    bool escaped = false;
    for (std::size_t i = open_pos; i < text.size(); ++i) {
        const char ch = text[i];
        if (escaped) {
            escaped = false;
            continue;
        }
        if (ch == '\\') {
            escaped = true;
            continue;
        }
        if (ch == '"') {
            in_string = !in_string;
            continue;
        }
        if (in_string) {
            continue;
        }
        if (ch == open_ch) {
            ++depth;
        } else if (ch == close_ch) {
            --depth;
            if (depth == 0) {
                return i;
            }
        }
    }
    throw std::runtime_error("ghost scenario JSON has unmatched delimiter");
}

std::size_t find_matching_brace(const std::string& text, std::size_t open_pos) {
    return find_matching_delim(text, open_pos, '{', '}');
}

std::size_t find_matching_bracket(const std::string& text, std::size_t open_pos) {
    return find_matching_delim(text, open_pos, '[', ']');
}

std::string find_string_or(const std::string& body, const std::string& key, const std::string& fallback) {
    const std::string marker = "\"" + key + "\"";
    const auto key_pos = body.find(marker);
    if (key_pos == std::string::npos) {
        return fallback;
    }
    const auto colon_pos = body.find(':', key_pos + marker.size());
    if (colon_pos == std::string::npos) {
        return fallback;
    }
    const auto null_pos = body.find("null", colon_pos + 1U);
    const auto open_pos = body.find('"', colon_pos + 1U);
    if (null_pos != std::string::npos && (open_pos == std::string::npos || null_pos < open_pos)) {
        return fallback;
    }
    if (open_pos == std::string::npos) {
        return fallback;
    }
    const auto close_pos = body.find('"', open_pos + 1U);
    if (close_pos == std::string::npos) {
        return fallback;
    }
    return body.substr(open_pos + 1U, close_pos - open_pos - 1U);
}

std::optional<std::string> find_optional_string(const std::string& body, const std::string& key) {
    const auto value = find_string_or(body, key, "");
    if (value.empty()) {
        return std::nullopt;
    }
    return value;
}

double find_number_or(const std::string& body, const std::string& key, double fallback) {
    const std::string marker = "\"" + key + "\"";
    const auto key_pos = body.find(marker);
    if (key_pos == std::string::npos) {
        return fallback;
    }
    const auto colon_pos = body.find(':', key_pos + marker.size());
    if (colon_pos == std::string::npos) {
        return fallback;
    }
    const auto value_start = body.find_first_of("-0123456789.", colon_pos + 1U);
    if (value_start == std::string::npos) {
        return fallback;
    }
    const auto value_end = body.find_first_not_of("-0123456789.eE+", value_start);
    return std::stod(body.substr(value_start, value_end - value_start));
}

Vec3 find_vec3_or(const std::string& body, const std::string& key, Vec3 fallback) {
    const std::string marker = "\"" + key + "\"";
    const auto key_pos = body.find(marker);
    if (key_pos == std::string::npos) {
        return fallback;
    }
    const auto array_open = body.find('[', key_pos + marker.size());
    if (array_open == std::string::npos) {
        return fallback;
    }
    const auto array_close = find_matching_bracket(body, array_open);
    std::vector<double> values;
    std::size_t cursor = array_open + 1U;
    while (cursor < array_close) {
        const auto value_start = body.find_first_of("-0123456789.", cursor);
        if (value_start == std::string::npos || value_start >= array_close) {
            break;
        }
        const auto value_end = body.find_first_not_of("-0123456789.eE+", value_start);
        values.push_back(std::stod(body.substr(value_start, value_end - value_start)));
        cursor = value_end;
    }
    if (values.size() != 3U) {
        return fallback;
    }
    return Vec3{values[0], values[1], values[2]};
}

std::vector<std::string> detection_objects(const std::string& text) {
    const auto detections_pos = text.find("\"detections\"");
    if (detections_pos == std::string::npos) {
        throw std::runtime_error("ghost scenario JSON missing required 'detections' array");
    }
    const auto array_open = text.find('[', detections_pos);
    if (array_open == std::string::npos) {
        throw std::runtime_error("ghost scenario detections field is missing an array");
    }
    const auto array_close = find_matching_bracket(text, array_open);
    std::vector<std::string> objects;
    std::size_t cursor = array_open + 1U;
    while (cursor < array_close) {
        const auto object_open = text.find('{', cursor);
        if (object_open == std::string::npos || object_open > array_close) {
            break;
        }
        const auto object_close = find_matching_brace(text, object_open);
        objects.push_back(text.substr(object_open, object_close - object_open + 1U));
        cursor = object_close + 1U;
    }
    return objects;
}

}  // namespace

GhostScenario GhostScenario::load_from_file(const std::string& path) {
    return parse_json(read_text_file(path), parent_dir_or_dot(path));
}

GhostScenario GhostScenario::parse_json(const std::string& text, const std::string& base_dir) {
    const auto name = find_string_or(text, "name", "ghost_scenario");
    const auto map_frame_id = MapFrameId{find_string_or(text, "map_frame_id", "map_local_0001")};

    std::vector<GhostDetectionSpec> detections;
    for (const auto& object : detection_objects(text)) {
        GhostDetectionSpec spec;
        spec.source_track_id = TrackId{find_string_or(object, "source_track_id", "")};
        if (spec.source_track_id.value.empty()) {
            throw std::runtime_error("ghost detection missing required source_track_id");
        }
        spec.class_label = class_label_from_string(find_string_or(object, "class", ""));
        if (spec.class_label == ClassLabel::Unknown) {
            throw std::runtime_error("ghost detection has unknown or missing class: " + spec.source_track_id.value);
        }
        spec.confidence = find_number_or(object, "confidence", -1.0);
        if (spec.confidence < 0.0) {
            throw std::runtime_error("ghost detection missing required confidence: " + spec.source_track_id.value);
        }
        spec.initial_position_local_m = find_vec3_or(object, "initial_position_local_m", Vec3{0.0, 0.0, 0.0});
        spec.size_m = find_vec3_or(object, "size_m", Vec3{1.0, 1.0, 1.0});
        spec.trajectory_path = find_optional_string(object, "trajectory_path");
        if (spec.trajectory_path.has_value()) {
            spec.trajectory_path = resolve_path(base_dir, *spec.trajectory_path);
            spec.trajectory = VelocityTrajectory::load_from_file(*spec.trajectory_path);
        }
        detections.push_back(std::move(spec));
    }
    if (detections.empty()) {
        throw std::runtime_error("ghost scenario must contain at least one detection");
    }
    return GhostScenario{name, map_frame_id, std::move(detections)};
}

}  // namespace dedalus
