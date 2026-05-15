#include "dedalus/behavior/trajectory_mission_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace dedalus {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinArrivedDistanceM = 0.5;
constexpr double kLandHeightM = 0.25;
constexpr double kTakeoffVelocityAssistHeightM = 0.5;

std::string read_text_file(const std::string& path) {
    std::ifstream input{path};
    if (!input) {
        throw std::runtime_error("failed to open trajectory mission file: " + path);
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

std::string strip_json_string(std::string value) {
    value.erase(std::remove(value.begin(), value.end(), '"'), value.end());
    value.erase(std::remove(value.begin(), value.end(), '\''), value.end());
    return value;
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
        segment.direction = lower_string(find_string_or(body, "direction", "ccw"));
        segment.keyframes = parse_keyframes(body);

        if (segment.type == "velocity_keyframes") {
            if (segment.keyframes.empty()) {
                throw std::runtime_error("trajectory velocity_keyframes segment missing keyframes");
            }
            segment.duration_s = segment.keyframes.back().t_s;
        }
        if (segment.duration_s <= 0.0) {
            // Match test-flight behavior: skip empty segments by not adding them.
            cursor = object_close + 1U;
            continue;
        }

        segments.push_back(segment);
        cursor = object_close + 1U;
    }
    return segments;
}

double seconds_between(TimePoint start, TimePoint end) {
    return static_cast<double>(end.timestamp_ns - start.timestamp_ns) / 1'000'000'000.0;
}

bool elapsed_at_least(TimePoint start, TimePoint end, double seconds) {
    return seconds_between(start, end) >= seconds;
}

double norm_xy(const Vec3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y);
}

Vec3 velocity_toward_xy(const Vec3& from, const Vec3& to, double speed_mps) {
    const Vec3 delta{to.x - from.x, to.y - from.y, 0.0};
    const double distance = norm_xy(delta);
    if (distance <= kMinArrivedDistanceM || speed_mps <= 0.0) {
        return Vec3{0.0, 0.0, 0.0};
    }
    return Vec3{delta.x / distance * speed_mps, delta.y / distance * speed_mps, 0.0};
}

double lerp(double a, double b, double u) {
    return a + (b - a) * u;
}

Vec3 interpolate_keyframes(const std::vector<TrajectoryKeyframe>& keyframes, double t_s) {
    if (keyframes.empty()) {
        return Vec3{0.0, 0.0, 0.0};
    }
    if (t_s <= keyframes.front().t_s) {
        return Vec3{keyframes.front().vx_mps, keyframes.front().vy_mps, keyframes.front().vz_mps};
    }
    if (t_s >= keyframes.back().t_s) {
        return Vec3{keyframes.back().vx_mps, keyframes.back().vy_mps, keyframes.back().vz_mps};
    }

    for (std::size_t i = 0; i + 1U < keyframes.size(); ++i) {
        const auto& a = keyframes[i];
        const auto& b = keyframes[i + 1U];
        if (a.t_s <= t_s && t_s <= b.t_s) {
            const double span = std::max(b.t_s - a.t_s, 1e-6);
            const double u = (t_s - a.t_s) / span;
            return Vec3{
                lerp(a.vx_mps, b.vx_mps, u),
                lerp(a.vy_mps, b.vy_mps, u),
                lerp(a.vz_mps, b.vz_mps, u)};
        }
    }

    return Vec3{keyframes.back().vx_mps, keyframes.back().vy_mps, keyframes.back().vz_mps};
}

}  // namespace

TrajectoryMissionConfig load_trajectory_mission_config(const MissionOptions& options) {
    TrajectoryMissionConfig config;
    config.safe_height_m = std::stod(options.get_or("flight_safe_height_m", "8"));
    config.takeoff_velocity_mps = std::stod(options.get_or("flight_takeoff_velocity_mps", "1.0"));
    config.go_home_velocity_mps = std::stod(options.get_or("flight_go_home_velocity_mps", "1.0"));
    config.land_velocity_mps = std::stod(options.get_or("flight_land_velocity_mps", "0.5"));
    config.arm_retry_interval_s = std::stod(options.get_or("flight_arm_retry_interval_s", "1.0"));
    config.arm_timeout_s = std::stod(options.get_or("flight_arm_timeout_s", "10.0"));
    config.takeoff_retry_interval_s = std::stod(options.get_or("flight_takeoff_retry_interval_s", "1.0"));
    config.disarm_retry_interval_s = std::stod(options.get_or("flight_disarm_retry_interval_s", "1.0"));
    config.disarm_timeout_s = std::stod(options.get_or("flight_disarm_timeout_s", "10.0"));
    config.home_policy = options.get_or("flight_home_policy", "initial_ego_pose");

    const auto trajectory_path = options.get_or("flight_trajectory_path", "");
    if (!trajectory_path.empty()) {
        config.segments = parse_segments(read_text_file(strip_json_string(trajectory_path)));
    }
    if (config.segments.empty()) {
        TrajectorySegment hold;
        hold.type = "hold";
        hold.label = "default_hold";
        hold.duration_s = 1.0;
        config.segments.push_back(hold);
    }
    return config;
}

TrajectoryMissionController::TrajectoryMissionController(TrajectoryMissionConfig config)
    : config_(std::move(config)) {}

VelocityCommand TrajectoryMissionController::command_from_velocity(
    TimePoint timestamp,
    Vec3 velocity_local_mps) const {
    VelocityCommand command;
    command.kind = FlightCommandKind::Velocity;
    command.timestamp = timestamp;
    command.velocity_local_mps = velocity_local_mps;
    command.yaw_rate_radps = 0.0;
    command.yaw_rate_valid = true;
    command.yaw_valid = false;
    return command;
}

VelocityCommand TrajectoryMissionController::command_with_kind(
    TimePoint timestamp,
    FlightCommandKind kind) const {
    VelocityCommand command;
    command.kind = kind;
    command.timestamp = timestamp;
    command.velocity_local_mps = Vec3{0.0, 0.0, 0.0};
    command.yaw_rate_valid = false;
    command.yaw_valid = false;
    return command;
}

VelocityCommand TrajectoryMissionController::trajectory_command(TimePoint timestamp) const {
    const auto& segment = config_.segments.at(segment_index_);
    Vec3 velocity{segment.vx_mps, segment.vy_mps, segment.vz_mps};

    if (segment.type == "hold") {
        velocity = Vec3{segment.vx_mps, segment.vy_mps, segment.vz_mps};
    } else if (segment.type == "velocity_keyframes") {
        velocity = interpolate_keyframes(segment.keyframes, segment_elapsed_s_);
    } else if (segment.type == "circle_velocity") {
        const double speed = segment.speed_mps > 0.0 ? segment.speed_mps : 2.0;
        const double radius = std::max(segment.radius_m, 1e-6);
        const double sign = segment.direction == "cw" ? -1.0 : 1.0;
        const double theta = speed / radius * segment_elapsed_s_;
        velocity.x = speed * std::cos(theta);
        velocity.y = sign * speed * std::sin(theta);
        velocity.z = segment.vz_mps;
    } else if (segment.type == "figure8_velocity") {
        const double duration = std::max(segment.duration_s, 1e-6);
        const double speed = segment.speed_mps > 0.0 ? segment.speed_mps : 2.0;
        const double scale = std::max(segment.scale_m, 1e-6);
        const double theta = 2.0 * kPi * segment_elapsed_s_ / duration;
        const double dx = scale * std::cos(theta);
        const double dy = scale * std::cos(2.0 * theta);
        const double norm = std::max(std::hypot(dx, dy), 1e-6);
        velocity.x = speed * dx / norm;
        velocity.y = speed * dy / norm;
        velocity.z = segment.vz_mps;
    } else {
        throw std::runtime_error("unknown trajectory segment type: " + segment.type);
    }

    return command_from_velocity(timestamp, velocity);
}

bool TrajectoryMissionController::trajectory_complete() const {
    return segment_index_ >= config_.segments.size();
}

void TrajectoryMissionController::advance_segment_if_needed() {
    while (!trajectory_complete() && segment_elapsed_s_ >= config_.segments.at(segment_index_).duration_s) {
        segment_elapsed_s_ -= config_.segments.at(segment_index_).duration_s;
        ++segment_index_;
    }
}

MissionTickOutput TrajectoryMissionController::tick(const MissionTickInput& input) {
    MissionTickOutput output;
    output.state = state_;

    if (!mission_started_) {
        mission_started_ = true;
        mission_start_ = input.now;
        state_start_ = input.now;
        last_tick_time_ = input.now;
        home_pose_ = input.snapshot.ego.local_T_body;
        home_initialized_ = true;
        state_ = MissionLifecycleState::Prepare;
    }

    const double dt_s = std::max(0.0, seconds_between(last_tick_time_, input.now));
    last_tick_time_ = input.now;

    const auto& ego = input.snapshot.ego;
    const double height_m = ego.height_valid ? ego.height_m : -ego.local_T_body.position.z;

    switch (state_) {
        case MissionLifecycleState::Prepare:
            if (input.finish_requested && ego.armed_valid && !ego.armed) {
                state_ = MissionLifecycleState::Complete;
                state_start_ = input.now;
                output.status = "finish_requested_before_arm";
            } else if (ego.armed_valid && ego.armed) {
                state_ = MissionLifecycleState::Takeoff;
                state_start_ = input.now;
                output.status = "armed_confirmed_by_ego";
            } else if (elapsed_at_least(state_start_, input.now, config_.arm_timeout_s)) {
                state_ = MissionLifecycleState::Abort;
                output.status = "arm_timeout";
            } else if (!arm_command_sent_ || elapsed_at_least(arm_last_command_time_, input.now, config_.arm_retry_interval_s)) {
                arm_command_sent_ = true;
                arm_last_command_time_ = input.now;
                output.command = command_with_kind(input.now, FlightCommandKind::Arm);
                output.status = "arming";
            } else {
                output.status = ego.armed_valid ? "waiting_for_armed_state" : "waiting_for_armed_telemetry";
            }
            break;
        case MissionLifecycleState::Takeoff:
            if (input.finish_requested) {
                state_ = height_m > kLandHeightM ? MissionLifecycleState::Land : MissionLifecycleState::Complete;
                state_start_ = input.now;
                output.status = height_m > kLandHeightM ? "finish_requested_land" : "finish_requested_complete";
            } else if (height_m >= config_.safe_height_m) {
                state_ = MissionLifecycleState::ExecuteMission;
                state_start_ = input.now;
                segment_index_ = 0U;
                segment_elapsed_s_ = 0.0;
                output.status = "takeoff_complete";
            } else if (!takeoff_command_sent_ ||
                       elapsed_at_least(takeoff_last_command_time_, input.now, config_.takeoff_retry_interval_s)) {
                takeoff_command_sent_ = true;
                takeoff_last_command_time_ = input.now;
                output.command = command_with_kind(input.now, FlightCommandKind::Takeoff);
                output.status = "takeoff_request";
            } else if (height_m >= kTakeoffVelocityAssistHeightM) {
                output.command = command_from_velocity(
                    input.now,
                    Vec3{0.0, 0.0, -std::abs(config_.takeoff_velocity_mps)});
                output.status = "takeoff_climb";
            } else {
                output.status = "waiting_for_takeoff_climb";
            }
            break;
        case MissionLifecycleState::ExecuteMission:
            if (input.finish_requested) {
                state_ = MissionLifecycleState::GoHome;
                state_start_ = input.now;
                output.status = "finish_requested_go_home";
            } else {
                segment_elapsed_s_ += dt_s;
                advance_segment_if_needed();
                if (trajectory_complete()) {
                    state_ = MissionLifecycleState::GoHome;
                    state_start_ = input.now;
                    output.status = "trajectory_complete";
                } else {
                    output.command = trajectory_command(input.now);
                    output.status = "trajectory_execute";
                }
            }
            break;
        case MissionLifecycleState::GoHome: {
            const Vec3 velocity = home_initialized_
                ? velocity_toward_xy(ego.local_T_body.position, home_pose_.position, config_.go_home_velocity_mps)
                : Vec3{0.0, 0.0, 0.0};
            if (norm_xy(velocity) <= 0.0) {
                state_ = MissionLifecycleState::Land;
                state_start_ = input.now;
                output.status = "home_reached";
            } else {
                output.command = command_from_velocity(input.now, velocity);
                output.status = "go_home";
            }
            break;
        }
        case MissionLifecycleState::Land:
            if (height_m <= kLandHeightM) {
                state_ = MissionLifecycleState::Complete;
                state_start_ = input.now;
                output.status = "landed";
            } else {
                output.command = command_from_velocity(
                    input.now,
                    Vec3{0.0, 0.0, std::abs(config_.land_velocity_mps)});
                output.status = "landing";
            }
            break;
        case MissionLifecycleState::Complete:
            if (ego.armed_valid && !ego.armed) {
                output.status = "complete";
            } else if (elapsed_at_least(state_start_, input.now, config_.disarm_timeout_s)) {
                state_ = MissionLifecycleState::Abort;
                output.status = "disarm_timeout";
            } else if (!disarm_command_sent_ || elapsed_at_least(disarm_last_command_time_, input.now, config_.disarm_retry_interval_s)) {
                disarm_command_sent_ = true;
                disarm_last_command_time_ = input.now;
                output.command = command_with_kind(input.now, FlightCommandKind::Disarm);
                output.status = "disarming";
            } else {
                output.status = ego.armed_valid ? "waiting_for_disarmed_state" : "waiting_for_disarmed_telemetry";
            }
            break;
        case MissionLifecycleState::Abort:
            output.command = command_from_velocity(input.now, Vec3{0.0, 0.0, 0.0});
            output.status = "abort_hold";
            break;
        case MissionLifecycleState::Idle:
        default:
            state_ = MissionLifecycleState::Prepare;
            state_start_ = input.now;
            output.status = "idle";
            break;
    }

    output.state = state_;
    return output;
}

}  // namespace dedalus