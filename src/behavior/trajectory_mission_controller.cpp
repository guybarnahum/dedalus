#include "dedalus/behavior/trajectory_mission_controller.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace dedalus {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinArrivedDistanceM = 0.5;
constexpr double kLandHeightM = 0.25;

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

std::size_t find_matching_brace(const std::string& text, std::size_t open_pos) {
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
        if (ch == '{') {
            ++depth;
        } else if (ch == '}') {
            --depth;
            if (depth == 0) {
                return i;
            }
        }
    }
    throw std::runtime_error("trajectory JSON has unmatched object brace");
}

std::vector<TrajectorySegment> parse_segments(const std::string& text) {
    std::vector<TrajectorySegment> segments;
    const auto segments_pos = text.find("\"segments\"");
    if (segments_pos == std::string::npos) {
        return segments;
    }
    const auto array_open = text.find('[', segments_pos);
    const auto array_close = text.find(']', array_open);
    if (array_open == std::string::npos || array_close == std::string::npos) {
        throw std::runtime_error("trajectory JSON has invalid segments array");
    }

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
        segment.direction = find_string_or(body, "direction", "ccw");

        if (segment.type == "velocity_keyframes") {
            segment.duration_s = find_number_or(body, "t", segment.duration_s);
            const auto last_t_marker = body.rfind("\"t\"");
            if (last_t_marker != std::string::npos) {
                const auto tail = body.substr(last_t_marker);
                segment.duration_s = find_number_or(tail, "t", segment.duration_s);
            }
        }
        if (segment.duration_s <= 0.0) {
            segment.duration_s = 1.0;
        }

        segments.push_back(segment);
        cursor = object_close + 1U;
    }
    return segments;
}

double seconds_between(TimePoint start, TimePoint end) {
    return static_cast<double>(end.timestamp_ns - start.timestamp_ns) / 1'000'000'000.0;
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

}  // namespace

TrajectoryMissionConfig load_trajectory_mission_config(const MissionOptions& options) {
    TrajectoryMissionConfig config;
    config.safe_height_m = std::stod(options.get_or("flight_safe_height_m", "8"));
    config.takeoff_velocity_mps = std::stod(options.get_or("flight_takeoff_velocity_mps", "1.0"));
    config.go_home_velocity_mps = std::stod(options.get_or("flight_go_home_velocity_mps", "1.0"));
    config.land_velocity_mps = std::stod(options.get_or("flight_land_velocity_mps", "0.5"));
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

    if (segment.type == "circle_velocity") {
        const double direction = segment.direction == "cw" ? -1.0 : 1.0;
        const double speed = segment.speed_mps > 0.0 ? segment.speed_mps : 1.0;
        const double radius = std::max(segment.radius_m, 1.0);
        const double omega = direction * speed / radius;
        const double theta = omega * segment_elapsed_s_;
        velocity.x = -std::sin(theta) * speed;
        velocity.y = std::cos(theta) * speed * direction;
        velocity.z = segment.vz_mps;
    } else if (segment.type == "figure8_velocity") {
        const double speed = segment.speed_mps > 0.0 ? segment.speed_mps : 1.0;
        const double scale = std::max(segment.scale_m, 1.0);
        const double theta = speed / scale * segment_elapsed_s_;
        velocity.x = std::cos(theta) * speed;
        velocity.y = std::cos(2.0 * theta) * speed * 0.5;
        velocity.z = segment.vz_mps;
    } else if (segment.type == "velocity_keyframes") {
        velocity.x = segment.vx_mps;
        velocity.y = segment.vy_mps;
        velocity.z = segment.vz_mps;
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
            if (!arm_command_sent_) {
                arm_command_sent_ = true;
                output.command = command_with_kind(input.now, FlightCommandKind::Arm);
                output.status = "arming";
            } else {
                state_ = MissionLifecycleState::Takeoff;
                output.status = "armed";
            }
            break;
        case MissionLifecycleState::Takeoff:
            if (height_m >= config_.safe_height_m) {
                state_ = MissionLifecycleState::ExecuteMission;
                state_start_ = input.now;
                segment_index_ = 0U;
                segment_elapsed_s_ = 0.0;
                output.status = "takeoff_complete";
            } else {
                output.command = command_from_velocity(
                    input.now,
                    Vec3{0.0, 0.0, -std::abs(config_.takeoff_velocity_mps)});
                output.status = "takeoff_climb";
            }
            break;
        case MissionLifecycleState::ExecuteMission:
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
                output.status = "landed";
            } else {
                output.command = command_from_velocity(
                    input.now,
                    Vec3{0.0, 0.0, std::abs(config_.land_velocity_mps)});
                output.status = "landing";
            }
            break;
        case MissionLifecycleState::Complete:
            if (!disarm_command_sent_) {
                disarm_command_sent_ = true;
                output.command = command_with_kind(input.now, FlightCommandKind::Disarm);
                output.status = "disarming";
            } else {
                output.status = "complete";
            }
            break;
        case MissionLifecycleState::Abort:
            output.command = command_from_velocity(input.now, Vec3{0.0, 0.0, 0.0});
            output.status = "abort_hold";
            break;
        case MissionLifecycleState::Idle:
        default:
            state_ = MissionLifecycleState::Prepare;
            output.status = "idle";
            break;
    }

    output.state = state_;
    return output;
}

}  // namespace dedalus
