#include "dedalus/simulation/airsim_providers.hpp"

#include <algorithm>
#include <cctype>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dedalus {
namespace {

std::runtime_error unavailable(const char* provider_name) {
    return std::runtime_error(
        std::string{provider_name} +
        " is an integration provider and is not available in the dependency-free core build");
}

std::unique_ptr<BridgeTransport> make_transport(const std::string& transport_name) {
    if (transport_name == "pipe") {
        return std::make_unique<PipeBridgeTransport>();
    }
    if (transport_name == "shared_memory") {
        return std::make_unique<SharedMemoryBridgeTransport>();
    }
    throw std::runtime_error("unknown AirSim bridge transport: " + transport_name);
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

std::string build_bridge_command(const AirSimProviderConfig& config, const std::string& base_command) {
    std::ostringstream command;
    command << base_command
            << " --host " << shell_quote(config.host)
            << " --rpc-port " << config.rpc_port
            << " --vehicle-name " << shell_quote(config.vehicle_name)
            << " --camera-name " << shell_quote(config.camera_name);
    return command.str();
}

std::string read_ppm_token(std::istream& input) {
    std::string token;
    while (input >> token) {
        if (!token.empty() && token.front() == '#') {
            std::string ignored;
            std::getline(input, ignored);
            continue;
        }
        return token;
    }
    throw std::runtime_error("unexpected end of AirSim bridge PPM output");
}

ImageView parse_ppm_bytes(const std::string& ppm) {
    std::istringstream input{ppm};

    const std::string magic = read_ppm_token(input);
    const int width = std::stoi(read_ppm_token(input));
    const int height = std::stoi(read_ppm_token(input));
    const int max_value = std::stoi(read_ppm_token(input));
    if (magic != "P6" || width <= 0 || height <= 0 || max_value != 255) {
        throw std::runtime_error("AirSim bridge returned invalid PPM header");
    }

    input.get();

    ImageView image;
    image.width = width;
    image.height = height;
    image.channels = 3;
    image.bytes.resize(static_cast<std::size_t>(width * height * image.channels));
    input.read(reinterpret_cast<char*>(image.bytes.data()), static_cast<std::streamsize>(image.bytes.size()));
    if (input.gcount() != static_cast<std::streamsize>(image.bytes.size())) {
        throw std::runtime_error("AirSim bridge returned truncated PPM image data");
    }

    return image;
}

int b64_value(char ch) {
    if (ch >= 'A' && ch <= 'Z') {
        return ch - 'A';
    }
    if (ch >= 'a' && ch <= 'z') {
        return ch - 'a' + 26;
    }
    if (ch >= '0' && ch <= '9') {
        return ch - '0' + 52;
    }
    if (ch == '+') {
        return 62;
    }
    if (ch == '/') {
        return 63;
    }
    if (ch == '=') {
        return -2;
    }
    if (std::isspace(static_cast<unsigned char>(ch)) != 0) {
        return -3;
    }
    return -1;
}

std::string base64_decode(const std::string& encoded) {
    std::string output;
    int value = 0;
    int bits = -8;

    for (const char ch : encoded) {
        const int decoded = b64_value(ch);
        if (decoded == -3) {
            continue;
        }
        if (decoded == -2) {
            break;
        }
        if (decoded < 0) {
            throw std::runtime_error("invalid base64 character in AirSim stream frame");
        }

        value = (value << 6) + decoded;
        bits += 6;
        if (bits >= 0) {
            output.push_back(static_cast<char>((value >> bits) & 0xFF));
            bits -= 8;
        }
    }

    return output;
}

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
    ego.map_frame_id = map_frame_id;
    return ego;
}

FramePacket frame_from_image(const AirSimProviderConfig& config, ImageView image, FrameId frame_id, TimePoint timestamp) {
    FramePacket frame;
    frame.frame_id = std::move(frame_id);
    frame.timestamp = timestamp;
    frame.camera_id = CameraId{config.camera_name};
    frame.image = std::move(image);
    frame.intrinsics.fx = 420.0;
    frame.intrinsics.fy = 420.0;
    frame.intrinsics.cx = static_cast<double>(frame.image.width) * 0.5;
    frame.intrinsics.cy = static_cast<double>(frame.image.height) * 0.5;

    AppearanceCondition appearance;
    appearance.timestamp = frame.timestamp;
    appearance.lighting_mode = LightingMode::Unknown;
    appearance.weather_mode = WeatherMode::Unknown;
    appearance.sensor_mode = SensorMode::Rgb;
    appearance.confidence = 0.45F;
    frame.appearance_condition = appearance;
    return frame;
}

}  // namespace

AirSimFrameSource::AirSimFrameSource(AirSimProviderConfig config)
    : config_(std::move(config)), transport_(make_transport(config_.transport)) {}

AirSimFrameSource::~AirSimFrameSource() = default;

FramePacket AirSimFrameSource::next_one_shot_frame() {
    const auto command = build_bridge_command(config_, config_.bridge_command);
    return frame_from_image(
        config_,
        parse_ppm_bytes(transport_->request_once(command)),
        FrameId{"airsim_live_frame_" + std::to_string(++next_frame_index_)},
        TimePoint{0});
}

std::optional<FramePacket> AirSimFrameSource::next_stream_jsonl_frame() {
    const auto command = build_bridge_command(config_, config_.bridge_command);
    const auto json_line = transport_->read_stream_line(command);
    if (!json_line.has_value()) {
        return std::nullopt;
    }

    const auto frame_id = parse_json_string(*json_line, "frame_id");
    const auto timestamp = parse_json_i64(*json_line, "timestamp_ns");
    const auto ppm_b64 = parse_json_string(*json_line, "ppm_b64");

    ++next_frame_index_;
    return frame_from_image(
        config_,
        parse_ppm_bytes(base64_decode(ppm_b64)),
        FrameId{frame_id},
        TimePoint{timestamp});
}

std::optional<FramePacket> AirSimFrameSource::next_frame() {
    if (config_.bridge_mode == "stream_jsonl") {
        return next_stream_jsonl_frame();
    }
    if (config_.bridge_mode == "one_shot_ppm") {
        return next_one_shot_frame();
    }
    throw std::runtime_error("unknown AirSim bridge mode: " + config_.bridge_mode);
}

AirSimEgoStateProvider::AirSimEgoStateProvider(AirSimProviderConfig config)
    : config_(std::move(config)), transport_(make_transport(config_.transport)) {}

EgoStateEstimate AirSimEgoStateProvider::estimate(const FramePacket& frame) {
    const auto command = build_bridge_command(config_, config_.ego_bridge_command);

    EgoStateEstimate estimate;
    estimate.ego = parse_ego_json(transport_->request_once(command), config_.map_frame_id, frame.timestamp);
    estimate.telemetry_available = true;
    estimate.confidence = 0.85F;
    return estimate;
}

AirSimDepthProjector::AirSimDepthProjector(AirSimProviderConfig config)
    : config_(std::move(config)) {}

std::vector<Observation3D> AirSimDepthProjector::project(
    const std::vector<Track2D>&,
    const FramePacket&,
    const EgoState&) {
    throw unavailable("AirSimDepthProjector");
}

AirSimGroundTruthDetector::AirSimGroundTruthDetector(AirSimProviderConfig config)
    : config_(std::move(config)) {}

std::vector<Detection2D> AirSimGroundTruthDetector::detect(const FramePacket&) {
    throw unavailable("AirSimGroundTruthDetector");
}

}  // namespace dedalus
