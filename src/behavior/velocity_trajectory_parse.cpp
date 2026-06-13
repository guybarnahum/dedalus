#include "dedalus/behavior/velocity_trajectory.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace dedalus {
namespace {

std::string read_text_file(const std::string& path) {
    std::ifstream input{path};
    if (!input) {
        throw std::runtime_error("failed to open trajectory file: " + path);
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

std::string lower_string(std::string value) {
    for (char& ch : value) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return value;
}

std::size_t find_matching_delim(
    const std::string& text,
    std::size_t open_pos,
    char open_ch,
    char close_ch) {
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
    throw std::runtime_error("trajectory JSON has unmatched delimiter");
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
    const auto open_pos = body.find('"', colon_pos);
    if (colon_pos == std::string::npos || open_pos == std::string::npos) {
        return fallback;
    }
    const auto close_pos = body.find('"', open_pos + 1U);
    if (close_pos == std::string::npos) {
        return fallback;
    }
    return body.substr(open_pos + 1U, close_pos - open_pos - 1U);
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

std::vector<TrajectoryKeyframe> parse_keyframes(const std::string& segment_body) {
    std::vector<TrajectoryKeyframe> keyframes;
    const auto keyframes_pos = segment_body.find("\"keyframes\"");
    if (keyframes_pos == std::string::npos) {
        return keyframes;
    }
    const auto array_open = segment_body.find('[', keyframes_pos);
    if (array_open == std::string::npos) {
        throw std::runtime_error("trajectory keyframes field is missing an array");
    }
    const auto array_close = find_matching_bracket(segment_body, array_open);

    std::size_t cursor = array_open + 1U;
    while (cursor < array_close) {
        const auto object_open = segment_body.find('{', cursor);
        if (object_open == std::string::npos || object_open > array_close) {
            break;
        }
        const auto object_close = find_matching_brace(segment_body, object_open);
        const std::string body = segment_body.substr(object_open, object_close - object_open + 1U);

        TrajectoryKeyframe keyframe;
        keyframe.t_s = find_number_or(body, "t", 0.0);
        keyframe.vx_mps = find_number_or(body, "vx_mps", 0.0);
        keyframe.vy_mps = find_number_or(body, "vy_mps", 0.0);
        keyframe.vz_mps = find_number_or(body, "vz_mps", 0.0);
        keyframes.push_back(keyframe);
        cursor = object_close + 1U;
    }

    std::sort(keyframes.begin(), keyframes.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.t_s < rhs.t_s;
    });
    return keyframes;
}

std::vector<TrajectorySegment> parse_segments(const std::string& text) {
    std::vector<TrajectorySegment> segments;
    const auto segments_pos = text.find("\"segments\"");
    if (segments_pos == std::string::npos) {
        throw std::runtime_error("trajectory JSON missing required 'segments' array");
    }
    const auto array_open = text.find('[', segments_pos);
    if (array_open == std::string::npos) {
        throw std::runtime_error("trajectory JSON has invalid segments array");
    }
    const auto array_close = find_matching_bracket(text, array_open);

    std::size_t cursor = array_open + 1U;
    while (cursor < array_close) {
        const auto object_open = text.find('{', cursor);
        if (object_open == std::string::npos || object_open > array_close) {
            break;
        }
        const auto object_close = find_matching_brace(text, object_open);
        const std::string body = text.substr(object_open, object_close - object_open + 1U);

        TrajectorySegment segment;
        segment.type = find_string_or(body, "type", "hold");
        segment.label = find_string_or(body, "label", segment.type);
        segment.duration_s = find_number_or(body, "duration_s", find_number_or(body, "duration", 0.0));
        segment.vx_mps = find_number_or(body, "vx_mps", 0.0);
        segment.vy_mps = find_number_or(body, "vy_mps", 0.0);
        segment.vz_mps = find_number_or(body, "vz_mps", 0.0);
        segment.speed_mps = find_number_or(body, "speed_mps", 0.0);
        segment.radius_m = find_number_or(body, "radius_m", 1.0);
        segment.scale_m = find_number_or(body, "scale_m", segment.radius_m);
        segment.yaw_offset_rad = find_number_or(body, "yaw_offset_rad", 0.0);
        segment.direction = lower_string(find_string_or(body, "direction", "ccw"));
        segment.keyframes = parse_keyframes(body);

        if (segment.type == "velocity_keyframes") {
            if (segment.keyframes.empty()) {
                throw std::runtime_error("trajectory velocity_keyframes segment missing keyframes");
            }
            segment.duration_s = segment.keyframes.back().t_s;
        }
        if (segment.duration_s <= 0.0) {
            cursor = object_close + 1U;
            continue;
        }

        segments.push_back(segment);
        cursor = object_close + 1U;
    }
    return segments;
}

}  // namespace

VelocityTrajectory VelocityTrajectory::load_from_file(const std::string& path) {
    return parse_json(read_text_file(path));
}

VelocityTrajectory VelocityTrajectory::parse_json(const std::string& text) {
    return VelocityTrajectory{parse_segments(text)};
}

}  // namespace dedalus
