#include "dedalus/behavior/behavior_spec.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace dedalus {
namespace {

struct ConfigNode {
    enum class Kind { Undefined, Scalar, Map, Sequence };

    Kind kind{Kind::Undefined};
    std::string scalar;
    std::vector<std::pair<std::string, ConfigNode>> map;
    std::vector<ConfigNode> sequence;
};

std::string trim(std::string value) {
    const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

std::string strip_quotes(std::string value) {
    value = trim(value);
    if (value.size() >= 2U) {
        const char first = value.front();
        const char last = value.back();
        if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
            return value.substr(1U, value.size() - 2U);
        }
    }
    return value;
}

std::string strip_yaml_comment(const std::string& line) {
    bool in_single = false;
    bool in_double = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char ch = line[i];
        if (ch == '\'' && !in_double) {
            in_single = !in_single;
        } else if (ch == '"' && !in_single) {
            in_double = !in_double;
        } else if (ch == '#' && !in_single && !in_double) {
            return line.substr(0U, i);
        }
    }
    return line;
}

std::size_t leading_spaces(const std::string& line) {
    std::size_t count = 0;
    while (count < line.size() && line[count] == ' ') {
        ++count;
    }
    return count;
}

void set_scalar(ConfigNode& node, std::string value) {
    node.kind = ConfigNode::Kind::Scalar;
    node.scalar = strip_quotes(std::move(value));
    node.map.clear();
    node.sequence.clear();
}

ConfigNode& map_child(ConfigNode& node, const std::string& key) {
    if (node.kind == ConfigNode::Kind::Undefined) {
        node.kind = ConfigNode::Kind::Map;
    }
    if (node.kind != ConfigNode::Kind::Map) {
        throw std::runtime_error("behavior spec parser expected a map while assigning key: " + key);
    }
    for (auto& entry : node.map) {
        if (entry.first == key) {
            return entry.second;
        }
    }
    node.map.emplace_back(key, ConfigNode{});
    return node.map.back().second;
}

ConfigNode& append_sequence_item(ConfigNode& node) {
    if (node.kind == ConfigNode::Kind::Undefined) {
        node.kind = ConfigNode::Kind::Sequence;
    }
    if (node.kind != ConfigNode::Kind::Sequence) {
        throw std::runtime_error("behavior spec parser expected a sequence item");
    }
    node.sequence.emplace_back();
    return node.sequence.back();
}

const ConfigNode* child(const ConfigNode& node, const std::string& key) {
    if (node.kind != ConfigNode::Kind::Map) {
        return nullptr;
    }
    for (const auto& entry : node.map) {
        if (entry.first == key) {
            return &entry.second;
        }
    }
    return nullptr;
}

const ConfigNode& required_child(const ConfigNode& node, const std::string& key, const std::string& context) {
    const auto* value = child(node, key);
    if (value == nullptr) {
        throw std::runtime_error("missing required behavior spec field: " + context + "." + key);
    }
    return *value;
}

std::string scalar_string(const ConfigNode& node, const std::string& context) {
    if (node.kind != ConfigNode::Kind::Scalar) {
        throw std::runtime_error("expected scalar behavior spec field: " + context);
    }
    return node.scalar;
}

std::string optional_string(const ConfigNode& node, const std::string& key, std::string fallback) {
    const auto* value = child(node, key);
    if (value == nullptr) {
        return fallback;
    }
    return scalar_string(*value, key);
}

double parse_double_text(const std::string& value, const std::string& context) {
    try {
        std::size_t consumed = 0;
        const double parsed = std::stod(value, &consumed);
        if (consumed != value.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return parsed;
    } catch (const std::exception&) {
        throw std::runtime_error("invalid numeric behavior spec field: " + context + "=" + value);
    }
}

double optional_double(const ConfigNode& node, const std::string& key, double fallback) {
    const auto* value = child(node, key);
    if (value == nullptr) {
        return fallback;
    }
    return parse_double_text(scalar_string(*value, key), key);
}

double required_double(const ConfigNode& node, const std::string& key, const std::string& context) {
    return parse_double_text(scalar_string(required_child(node, key, context), context + "." + key), context + "." + key);
}

BehaviorVector3 parse_vector3(const ConfigNode& node, const std::string& context) {
    BehaviorVector3 vector;
    vector.x = required_double(node, "x", context);
    vector.y = required_double(node, "y", context);
    vector.z = required_double(node, "z", context);
    return vector;
}

TargetSelectionPolicy parse_policy(const std::string& value) {
    if (value == "highest_confidence") {
        return TargetSelectionPolicy::HighestConfidence;
    }
    if (value == "nearest") {
        return TargetSelectionPolicy::Nearest;
    }
    if (value == "persistent_track") {
        return TargetSelectionPolicy::PersistentTrack;
    }
    throw std::runtime_error("unknown target selector policy: " + value);
}

BehaviorType parse_behavior_type(const std::string& value) {
    if (value == "hold") {
        return BehaviorType::Hold;
    }
    if (value == "search") {
        return BehaviorType::Search;
    }
    if (value == "follow") {
        return BehaviorType::Follow;
    }
    if (value == "approach") {
        return BehaviorType::Approach;
    }
    if (value == "circle") {
        return BehaviorType::Circle;
    }
    if (value == "go_home") {
        return BehaviorType::GoHome;
    }
    if (value == "land") {
        return BehaviorType::Land;
    }
    if (value == "go_home_land") {
        return BehaviorType::GoHomeLand;
    }
    if (value == "sequence") {
        return BehaviorType::Sequence;
    }
    throw std::runtime_error("unknown behavior type: " + value);
}

ReferenceFrame parse_reference_frame(const std::string& value) {
    if (value == "target_heading_frame") {
        return ReferenceFrame::TargetHeadingFrame;
    }
    if (value == "world_local_frame") {
        return ReferenceFrame::WorldLocalFrame;
    }
    if (value == "drone_heading_frame") {
        return ReferenceFrame::DroneHeadingFrame;
    }
    if (value == "camera_frame") {
        return ReferenceFrame::CameraFrame;
    }
    throw std::runtime_error("unknown behavior reference frame: " + value);
}

CircleDirection parse_circle_direction(const std::string& value) {
    if (value == "clockwise") {
        return CircleDirection::Clockwise;
    }
    if (value == "counter_clockwise" || value == "counterclockwise") {
        return CircleDirection::CounterClockwise;
    }
    throw std::runtime_error("unknown circle direction: " + value);
}

CompletionAction parse_completion_action(const std::string& value) {
    if (value.empty() || value == "none") {
        return CompletionAction::None;
    }
    if (value == "hold") {
        return CompletionAction::Hold;
    }
    if (value == "go_home") {
        return CompletionAction::GoHome;
    }
    if (value == "land") {
        return CompletionAction::Land;
    }
    if (value == "go_home_land") {
        return CompletionAction::GoHomeLand;
    }
    throw std::runtime_error("unknown completion action: " + value);
}

TargetLostFallback parse_target_lost_fallback(const std::string& value) {
    if (value.empty() || value == "none") {
        return TargetLostFallback::None;
    }
    if (value == "hold_then_go_home") {
        return TargetLostFallback::HoldThenGoHome;
    }
    if (value == "search_then_go_home") {
        return TargetLostFallback::SearchThenGoHome;
    }
    if (value == "abort") {
        return TargetLostFallback::Abort;
    }
    throw std::runtime_error("unknown target-lost fallback: " + value);
}

struct YamlStackEntry {
    int indent{0};
    ConfigNode* node{nullptr};
};

ConfigNode parse_yaml_subset(const std::string& text) {
    ConfigNode root;
    root.kind = ConfigNode::Kind::Map;
    std::vector<YamlStackEntry> stack{{-1, &root}};

    std::istringstream input{text};
    std::string raw_line;
    int line_number = 0;
    while (std::getline(input, raw_line)) {
        ++line_number;
        if (raw_line.find('\t') != std::string::npos) {
            throw std::runtime_error("tabs are not supported in behavior spec YAML at line " + std::to_string(line_number));
        }
        const auto line_without_comments = strip_yaml_comment(raw_line);
        const auto indent = static_cast<int>(leading_spaces(line_without_comments));
        const auto line = trim(line_without_comments);
        if (line.empty()) {
            continue;
        }

        while (stack.size() > 1U && indent <= stack.back().indent) {
            stack.pop_back();
        }
        ConfigNode* parent = stack.back().node;
        if (line.rfind("- ", 0U) == 0U) {
            ConfigNode& item = append_sequence_item(*parent);
            const auto rest = trim(line.substr(2U));
            if (rest.empty()) {
                item.kind = ConfigNode::Kind::Map;
                stack.push_back({indent, &item});
                continue;
            }
            const auto colon = rest.find(':');
            if (colon == std::string::npos) {
                set_scalar(item, rest);
                continue;
            }
            item.kind = ConfigNode::Kind::Map;
            const auto key = trim(rest.substr(0U, colon));
            const auto value = trim(rest.substr(colon + 1U));
            if (key.empty()) {
                throw std::runtime_error("empty sequence map key at behavior spec YAML line " + std::to_string(line_number));
            }
            ConfigNode& node = map_child(item, key);
            if (value.empty()) {
                stack.push_back({indent, &item});
                stack.push_back({indent + 2, &node});
            } else {
                set_scalar(node, value);
                stack.push_back({indent, &item});
            }
            continue;
        }

        const auto colon = line.find(':');
        if (colon == std::string::npos) {
            throw std::runtime_error("invalid behavior spec YAML line " + std::to_string(line_number) + ": missing ':'");
        }
        const auto key = trim(line.substr(0U, colon));
        const auto value = trim(line.substr(colon + 1U));
        if (key.empty()) {
            throw std::runtime_error("empty behavior spec YAML key at line " + std::to_string(line_number));
        }
        ConfigNode& node = map_child(*parent, key);
        if (value.empty()) {
            stack.push_back({indent, &node});
        } else {
            set_scalar(node, value);
        }
    }
    return root;
}

class JsonParser {
public:
    explicit JsonParser(std::string_view text) : text_{text} {}

    ConfigNode parse() {
        ConfigNode node = parse_value();
        skip_ws();
        if (pos_ != text_.size()) {
            throw std::runtime_error("unexpected trailing behavior spec JSON content");
        }
        return node;
    }

private:
    void skip_ws() {
        while (pos_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[pos_]))) {
            ++pos_;
        }
    }

    char peek() {
        skip_ws();
        if (pos_ >= text_.size()) {
            throw std::runtime_error("unexpected end of behavior spec JSON");
        }
        return text_[pos_];
    }

    char get() {
        const char ch = peek();
        ++pos_;
        return ch;
    }

    ConfigNode parse_value() {
        const char ch = peek();
        if (ch == '{') {
            return parse_object();
        }
        if (ch == '[') {
            return parse_array();
        }
        ConfigNode node;
        if (ch == '"') {
            set_scalar(node, parse_string());
            return node;
        }
        set_scalar(node, parse_literal());
        return node;
    }

    std::string parse_string() {
        if (get() != '"') {
            throw std::runtime_error("expected JSON string");
        }
        std::string value;
        while (pos_ < text_.size()) {
            const char ch = text_[pos_++];
            if (ch == '"') {
                return value;
            }
            if (ch == '\\') {
                if (pos_ >= text_.size()) {
                    throw std::runtime_error("invalid JSON escape");
                }
                const char escaped = text_[pos_++];
                if (escaped == '"' || escaped == '\\' || escaped == '/') {
                    value.push_back(escaped);
                } else if (escaped == 'n') {
                    value.push_back('\n');
                } else if (escaped == 't') {
                    value.push_back('\t');
                } else {
                    throw std::runtime_error("unsupported JSON escape in behavior spec");
                }
            } else {
                value.push_back(ch);
            }
        }
        throw std::runtime_error("unterminated JSON string in behavior spec");
    }

    std::string parse_literal() {
        skip_ws();
        const std::size_t start = pos_;
        while (pos_ < text_.size()) {
            const char ch = text_[pos_];
            if (std::isspace(static_cast<unsigned char>(ch)) || ch == ',' || ch == '}' || ch == ']') {
                break;
            }
            ++pos_;
        }
        if (start == pos_) {
            throw std::runtime_error("expected JSON literal in behavior spec");
        }
        return std::string{text_.substr(start, pos_ - start)};
    }

    ConfigNode parse_object() {
        if (get() != '{') {
            throw std::runtime_error("expected JSON object");
        }
        ConfigNode node;
        node.kind = ConfigNode::Kind::Map;
        skip_ws();
        if (pos_ < text_.size() && text_[pos_] == '}') {
            ++pos_;
            return node;
        }
        while (true) {
            const auto key = parse_string();
            if (get() != ':') {
                throw std::runtime_error("expected ':' in behavior spec JSON object");
            }
            node.map.emplace_back(key, parse_value());
            const char ch = get();
            if (ch == '}') {
                break;
            }
            if (ch != ',') {
                throw std::runtime_error("expected ',' or '}' in behavior spec JSON object");
            }
        }
        return node;
    }

    ConfigNode parse_array() {
        if (get() != '[') {
            throw std::runtime_error("expected JSON array");
        }
        ConfigNode node;
        node.kind = ConfigNode::Kind::Sequence;
        skip_ws();
        if (pos_ < text_.size() && text_[pos_] == ']') {
            ++pos_;
            return node;
        }
        while (true) {
            node.sequence.push_back(parse_value());
            const char ch = get();
            if (ch == ']') {
                break;
            }
            if (ch != ',') {
                throw std::runtime_error("expected ',' or ']' in behavior spec JSON array");
            }
        }
        return node;
    }

    std::string_view text_;
    std::size_t pos_{0U};
};

TargetSelectorSpec parse_target_selector(const ConfigNode& root) {
    const auto& target = required_child(root, "target", "root");
    const auto& selector = required_child(target, "selector", "target");
    TargetSelectorSpec spec;
    spec.class_label = scalar_string(required_child(selector, "class", "target.selector"), "target.selector.class");
    if (spec.class_label.empty()) {
        throw std::runtime_error("target.selector.class must not be empty");
    }
    spec.confidence_min = optional_double(selector, "confidence_min", spec.confidence_min);
    spec.policy = parse_policy(optional_string(selector, "policy", to_string(spec.policy)));
    spec.reacquire_timeout_s = optional_double(selector, "reacquire_timeout_s", spec.reacquire_timeout_s);
    if (spec.confidence_min < 0.0 || spec.confidence_min > 1.0) {
        throw std::runtime_error("target.selector.confidence_min must be in [0, 1]");
    }
    if (spec.reacquire_timeout_s < 0.0) {
        throw std::runtime_error("target.selector.reacquire_timeout_s must be non-negative");
    }
    return spec;
}

BehaviorSpec parse_behavior(const ConfigNode& node, const std::string& context) {
    BehaviorSpec spec;
    spec.type = parse_behavior_type(scalar_string(required_child(node, "type", context), context + ".type"));
    spec.target_frame = parse_reference_frame(optional_string(node, "target_frame", to_string(spec.target_frame)));
    if (const auto* offset = child(node, "relative_offset_m")) {
        spec.relative_offset_m = parse_vector3(*offset, context + ".relative_offset_m");
    }
    spec.max_speed_mps = optional_double(node, "max_speed_mps", spec.max_speed_mps);
    spec.max_vertical_speed_mps = optional_double(node, "max_vertical_speed_mps", spec.max_vertical_speed_mps);
    spec.position_tolerance_m = optional_double(node, "position_tolerance_m", spec.position_tolerance_m);
    spec.lost_target_timeout_s = optional_double(node, "lost_target_timeout_s", spec.lost_target_timeout_s);
    spec.radius_m = optional_double(node, "radius_m", spec.radius_m);
    spec.altitude_offset_m = optional_double(node, "altitude_offset_m", spec.altitude_offset_m);
    spec.angular_speed_deg_s = optional_double(node, "angular_speed_deg_s", spec.angular_speed_deg_s);
    spec.direction = parse_circle_direction(optional_string(node, "direction", to_string(spec.direction)));
    spec.stop_distance_m = optional_double(node, "stop_distance_m", spec.stop_distance_m);
    spec.duration_s = optional_double(node, "duration_s", spec.duration_s);

    if (const auto* steps = child(node, "steps")) {
        if (steps->kind != ConfigNode::Kind::Sequence) {
            throw std::runtime_error(context + ".steps must be a sequence");
        }
        for (std::size_t i = 0; i < steps->sequence.size(); ++i) {
            spec.steps.push_back(parse_behavior(steps->sequence[i], context + ".steps[" + std::to_string(i) + "]"));
        }
    }

    if (spec.max_speed_mps <= 0.0) {
        throw std::runtime_error(context + ".max_speed_mps must be positive");
    }
    if (spec.max_vertical_speed_mps <= 0.0) {
        throw std::runtime_error(context + ".max_vertical_speed_mps must be positive");
    }
    if (spec.position_tolerance_m < 0.0) {
        throw std::runtime_error(context + ".position_tolerance_m must be non-negative");
    }
    if (spec.lost_target_timeout_s < 0.0) {
        throw std::runtime_error(context + ".lost_target_timeout_s must be non-negative");
    }

    switch (spec.type) {
        case BehaviorType::Follow:
            if (child(node, "relative_offset_m") == nullptr) {
                throw std::runtime_error(context + ".relative_offset_m is required for follow behavior");
            }
            break;
        case BehaviorType::Circle:
            if (spec.radius_m <= 0.0) {
                throw std::runtime_error(context + ".radius_m must be positive for circle behavior");
            }
            if (spec.angular_speed_deg_s <= 0.0) {
                throw std::runtime_error(context + ".angular_speed_deg_s must be positive for circle behavior");
            }
            break;
        case BehaviorType::Approach:
            if (spec.stop_distance_m <= 0.0) {
                throw std::runtime_error(context + ".stop_distance_m must be positive for approach behavior");
            }
            break;
        case BehaviorType::Sequence:
            if (spec.steps.empty()) {
                throw std::runtime_error(context + ".steps must contain at least one step for sequence behavior");
            }
            break;
        default:
            break;
    }

    return spec;
}

CompletionSpec parse_completion(const ConfigNode& root) {
    CompletionSpec spec;
    const auto* completion = child(root, "completion");
    if (completion == nullptr) {
        return spec;
    }
    spec.after_s = optional_double(*completion, "after_s", spec.after_s);
    spec.then = parse_completion_action(optional_string(*completion, "then", to_string(spec.then)));
    if (spec.after_s < 0.0) {
        throw std::runtime_error("completion.after_s must be non-negative");
    }
    return spec;
}

FallbackSpec parse_fallback(const ConfigNode& root) {
    FallbackSpec spec;
    const auto* fallback = child(root, "fallback");
    if (fallback == nullptr) {
        return spec;
    }
    spec.on_target_lost = parse_target_lost_fallback(optional_string(*fallback, "on_target_lost", to_string(spec.on_target_lost)));
    spec.hold_s = optional_double(*fallback, "hold_s", spec.hold_s);
    spec.search_s = optional_double(*fallback, "search_s", spec.search_s);
    if (spec.hold_s < 0.0 || spec.search_s < 0.0) {
        throw std::runtime_error("fallback hold/search durations must be non-negative");
    }
    return spec;
}

BehaviorSpecFormat infer_format(const std::string& text, BehaviorSpecFormat format) {
    if (format != BehaviorSpecFormat::Auto) {
        return format;
    }
    const auto stripped = trim(text);
    if (!stripped.empty() && (stripped.front() == '{' || stripped.front() == '[')) {
        return BehaviorSpecFormat::Json;
    }
    return BehaviorSpecFormat::Yaml;
}

}  // namespace

std::string to_string(TargetSelectionPolicy policy) {
    switch (policy) {
        case TargetSelectionPolicy::HighestConfidence:
            return "highest_confidence";
        case TargetSelectionPolicy::Nearest:
            return "nearest";
        case TargetSelectionPolicy::PersistentTrack:
            return "persistent_track";
    }
    return "highest_confidence";
}

std::string to_string(BehaviorType type) {
    switch (type) {
        case BehaviorType::Hold:
            return "hold";
        case BehaviorType::Search:
            return "search";
        case BehaviorType::Follow:
            return "follow";
        case BehaviorType::Approach:
            return "approach";
        case BehaviorType::Circle:
            return "circle";
        case BehaviorType::GoHome:
            return "go_home";
        case BehaviorType::Land:
            return "land";
        case BehaviorType::GoHomeLand:
            return "go_home_land";
        case BehaviorType::Sequence:
            return "sequence";
    }
    return "hold";
}

std::string to_string(ReferenceFrame frame) {
    switch (frame) {
        case ReferenceFrame::TargetHeadingFrame:
            return "target_heading_frame";
        case ReferenceFrame::WorldLocalFrame:
            return "world_local_frame";
        case ReferenceFrame::DroneHeadingFrame:
            return "drone_heading_frame";
        case ReferenceFrame::CameraFrame:
            return "camera_frame";
    }
    return "world_local_frame";
}

std::string to_string(CircleDirection direction) {
    switch (direction) {
        case CircleDirection::Clockwise:
            return "clockwise";
        case CircleDirection::CounterClockwise:
            return "counter_clockwise";
    }
    return "clockwise";
}

std::string to_string(CompletionAction action) {
    switch (action) {
        case CompletionAction::None:
            return "none";
        case CompletionAction::Hold:
            return "hold";
        case CompletionAction::GoHome:
            return "go_home";
        case CompletionAction::Land:
            return "land";
        case CompletionAction::GoHomeLand:
            return "go_home_land";
    }
    return "none";
}

std::string to_string(TargetLostFallback fallback) {
    switch (fallback) {
        case TargetLostFallback::None:
            return "none";
        case TargetLostFallback::HoldThenGoHome:
            return "hold_then_go_home";
        case TargetLostFallback::SearchThenGoHome:
            return "search_then_go_home";
        case TargetLostFallback::Abort:
            return "abort";
    }
    return "none";
}

BehaviorMissionSpec parse_behavior_spec_text(const std::string& text, BehaviorSpecFormat format) {
    const BehaviorSpecFormat resolved_format = infer_format(text, format);
    ConfigNode root = resolved_format == BehaviorSpecFormat::Json ? JsonParser{text}.parse() : parse_yaml_subset(text);
    if (root.kind != ConfigNode::Kind::Map) {
        throw std::runtime_error("behavior spec root must be a map/object");
    }

    BehaviorMissionSpec spec;
    if (const auto* mission = child(root, "mission")) {
        spec.mission_name = optional_string(*mission, "name", spec.mission_name);
    }
    spec.target = parse_target_selector(root);
    spec.behavior = parse_behavior(required_child(root, "behavior", "root"), "behavior");
    spec.completion = parse_completion(root);
    spec.fallback = parse_fallback(root);
    return spec;
}

BehaviorMissionSpec parse_behavior_spec_file(const std::string& path, BehaviorSpecFormat format) {
    std::ifstream input{path};
    if (!input) {
        throw std::runtime_error("failed to open behavior spec: " + path);
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return parse_behavior_spec_text(buffer.str(), format);
}

}  // namespace dedalus
