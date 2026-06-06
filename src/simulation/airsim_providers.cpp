#include "dedalus/simulation/airsim_providers.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dedalus {
namespace {

using SteadyClock = std::chrono::steady_clock;

std::int64_t elapsed_us(const SteadyClock::time_point start) {
    return std::chrono::duration_cast<std::chrono::microseconds>(SteadyClock::now() - start).count();
}

constexpr std::size_t kBinaryFrameHeaderSize = 56U;
constexpr std::uint32_t kBinaryFrameVersion = 1U;
constexpr std::uint32_t kBinaryFrameEgoVersion = 2U;
constexpr std::uint32_t kBinaryPixelFormatRgb8 = 1U;
constexpr char kBinaryFrameMagic[8] = {'D', 'E', 'D', 'F', 'R', 'M', '1', '\0'};

struct BinaryFrameHeader {
    std::uint32_t header_size{0};
    std::uint32_t version{0};
    std::uint64_t sequence{0};
    std::int64_t timestamp_ns{0};
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::uint32_t channels{0};
    std::uint32_t pixel_format{0};
    std::uint32_t payload_size{0};
    std::uint32_t sidecar_size{0};
};

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
        throw std::runtime_error(
            "shared_memory bridge transport is not yet implemented; use bridge_transport: pipe");
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
    validate_bridge_base_command(base_command);
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

std::uint32_t read_u32_le(const std::string& bytes, std::size_t offset) {
    return static_cast<std::uint32_t>(static_cast<unsigned char>(bytes.at(offset))) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(bytes.at(offset + 1U))) << 8U) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(bytes.at(offset + 2U))) << 16U) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(bytes.at(offset + 3U))) << 24U);
}

std::uint64_t read_u64_le(const std::string& bytes, std::size_t offset) {
    std::uint64_t value = 0U;
    for (std::size_t index = 0U; index < 8U; ++index) {
        value |= static_cast<std::uint64_t>(static_cast<unsigned char>(bytes.at(offset + index))) << (8U * index);
    }
    return value;
}

std::int64_t read_i64_le(const std::string& bytes, std::size_t offset) {
    return static_cast<std::int64_t>(read_u64_le(bytes, offset));
}

BinaryFrameHeader parse_binary_header(const std::string& header_bytes) {
    if (header_bytes.size() != kBinaryFrameHeaderSize) {
        throw std::runtime_error("binary frame header has invalid size");
    }
    if (std::memcmp(header_bytes.data(), kBinaryFrameMagic, sizeof(kBinaryFrameMagic)) != 0) {
        throw std::runtime_error("binary frame header has invalid magic");
    }

    BinaryFrameHeader header;
    header.header_size = read_u32_le(header_bytes, 8U);
    header.version = read_u32_le(header_bytes, 12U);
    header.sequence = read_u64_le(header_bytes, 16U);
    header.timestamp_ns = read_i64_le(header_bytes, 24U);
    header.width = read_u32_le(header_bytes, 32U);
    header.height = read_u32_le(header_bytes, 36U);
    header.channels = read_u32_le(header_bytes, 40U);
    header.pixel_format = read_u32_le(header_bytes, 44U);
    header.payload_size = read_u32_le(header_bytes, 48U);
    header.sidecar_size = read_u32_le(header_bytes, 52U);

    if (header.header_size != kBinaryFrameHeaderSize ||
        (header.version != kBinaryFrameVersion && header.version != kBinaryFrameEgoVersion)) {
        throw std::runtime_error("binary frame header has unsupported version or header size");
    }
    if (header.width == 0U || header.height == 0U || header.channels != 3U || header.pixel_format != kBinaryPixelFormatRgb8) {
        throw std::runtime_error("binary frame header has unsupported image shape or pixel format");
    }
    if (header.payload_size != header.width * header.height * header.channels) {
        throw std::runtime_error("binary frame payload size does not match image shape");
    }
    if (header.version == kBinaryFrameVersion && header.sidecar_size != 0U) {
        throw std::runtime_error("binary frame version 1 cannot carry a sidecar payload");
    }

    return header;
}

ImageView image_from_rgb_payload(const BinaryFrameHeader& header, std::vector<std::uint8_t> payload) {
    if (payload.size() != header.payload_size) {
        throw std::runtime_error("binary frame payload has invalid size");
    }

    ImageView image;
    image.width = static_cast<int>(header.width);
    image.height = static_cast<int>(header.height);
    image.channels = static_cast<int>(header.channels);
    image.bytes = std::move(payload);
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

std::optional<FramePacket> AirSimFrameSource::next_stream_binary_frame() {
    const auto command = build_bridge_command(config_, config_.bridge_command);
    std::vector<FrameSourceTiming> timings;

    auto start = SteadyClock::now();
    const auto header_bytes = transport_->read_stream_bytes(command, kBinaryFrameHeaderSize);
    timings.push_back(FrameSourceTiming{"frame_source.detail.read_header", elapsed_us(start)});
    if (!header_bytes.has_value()) {
        return std::nullopt;
    }

    start = SteadyClock::now();
    const auto header = parse_binary_header(*header_bytes);
    timings.push_back(FrameSourceTiming{"frame_source.detail.parse_header", elapsed_us(start)});

    start = SteadyClock::now();
    auto payload = transport_->read_stream_byte_vector(command, header.payload_size);
    timings.push_back(FrameSourceTiming{"frame_source.detail.read_payload", elapsed_us(start)});
    if (!payload.has_value()) {
        throw std::runtime_error("binary stream ended before frame payload");
    }

    std::string sidecar_payload;
    if (header.sidecar_size > 0U) {
        start = SteadyClock::now();
        const auto sidecar = transport_->read_stream_bytes(command, header.sidecar_size);
        timings.push_back(FrameSourceTiming{"frame_source.detail.read_sidecar", elapsed_us(start)});
        if (!sidecar.has_value()) {
            throw std::runtime_error("binary stream ended before frame sidecar payload");
        }
        sidecar_payload = *sidecar;
    }

    ++next_frame_index_;
    start = SteadyClock::now();
    auto frame = frame_from_image(
        config_,
        image_from_rgb_payload(header, std::move(*payload)),
        FrameId{"binary_stream_frame_" + std::to_string(header.sequence)},
        TimePoint{header.timestamp_ns});
    timings.push_back(FrameSourceTiming{"frame_source.detail.construct_frame", elapsed_us(start)});

    if (!sidecar_payload.empty()) {
        start = SteadyClock::now();
        frame.ego_hint = parse_ego_json(sidecar_payload, config_.map_frame_id, frame.timestamp);
        frame.depth_frame = parse_depth_frame_optional(sidecar_payload, frame, config_.map_frame_id);
        timings.push_back(FrameSourceTiming{"frame_source.detail.parse_sidecar", elapsed_us(start)});
    }
    frame.source_timings = std::move(timings);
    return frame;
}

std::optional<FramePacket> AirSimFrameSource::next_frame() {
    if (config_.bridge_mode == "stream_binary" || config_.bridge_mode == "stream_binary_ego") {
        return next_stream_binary_frame();
    }
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