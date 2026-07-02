#include "dedalus/simulation/airsim_providers.hpp"
#include "dedalus/simulation/airsim_sidecar_parser.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
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

    // DEDALUS_DEBUG_EGO: warn immediately when bridge sends no sidecar.
    if (std::getenv("DEDALUS_DEBUG_EGO") && sidecar_payload.empty()) {
        std::fprintf(stderr,
            "[EgoDebug:sidecar] seq=%llu NO SIDECAR (sidecar_size=%u, version=%u) "
            "— ego_hint will be null; is --include-ego missing from bridge_command?\n",
            static_cast<unsigned long long>(header.sequence),
            static_cast<unsigned>(header.sidecar_size),
            static_cast<unsigned>(header.version));
    }
    if (!sidecar_payload.empty()) {
        start = SteadyClock::now();
        frame.ego_hint = parse_ego_json(sidecar_payload, config_.map_frame_id, frame.timestamp);
        frame.depth_frame = parse_depth_frame_optional(sidecar_payload, frame, config_.map_frame_id);
        timings.push_back(FrameSourceTiming{"frame_source.detail.parse_sidecar", elapsed_us(start)});
        // DEDALUS_DEBUG_EGO: trace what AirSim sidecar delivered this frame.
        if (std::getenv("DEDALUS_DEBUG_EGO")) {
            if (frame.ego_hint) {
                const auto& p = frame.ego_hint->local_T_body.position;
                const auto& v = frame.ego_hint->velocity_local;
                std::fprintf(stderr,
                    "[EgoDebug:sidecar] seq=%llu pos=(%.3f,%.3f,%.3f) "
                    "vel=(%.3f,%.3f,%.3f) yaw=%.3f h=%.2f\n",
                    static_cast<unsigned long long>(header.sequence),
                    p.x, p.y, p.z, v.x, v.y, v.z,
                    frame.ego_hint->local_T_body.rotation_rpy.z,
                    frame.ego_hint->height_m);
            } else {
                std::fprintf(stderr,
                    "[EgoDebug:sidecar] seq=%llu ego_hint MISSING "
                    "(parse_ego_json returned nullopt despite sidecar present)\n",
                    static_cast<unsigned long long>(header.sequence));
            }
        }
        timings.push_back(FrameSourceTiming{
            frame.depth_frame.has_value()
                ? "frame_source.detail.depth_sidecar.present"
                : "frame_source.detail.depth_sidecar.missing",
            frame.depth_frame.has_value() ? static_cast<std::int64_t>(frame.depth_frame->depth_m.size()) : 0});
        if (frame.depth_frame.has_value()) {
            std::int64_t valid_samples = 0;
            float min_depth = std::numeric_limits<float>::infinity();
            float max_depth = 0.0F;
            for (const auto depth : frame.depth_frame->depth_m) {
                if (std::isfinite(depth) && depth > 0.0F) {
                    ++valid_samples;
                    min_depth = std::min(min_depth, depth);
                    max_depth = std::max(max_depth, depth);
                }
            }
            timings.push_back(FrameSourceTiming{"frame_source.detail.depth_sidecar.width", frame.depth_frame->width});
            timings.push_back(FrameSourceTiming{"frame_source.detail.depth_sidecar.height", frame.depth_frame->height});
            timings.push_back(FrameSourceTiming{"frame_source.detail.depth_sidecar.valid_samples", valid_samples});
            timings.push_back(FrameSourceTiming{"frame_source.detail.depth_sidecar.min_mm", valid_samples > 0 ? static_cast<std::int64_t>(min_depth * 1000.0F) : 0});
            timings.push_back(FrameSourceTiming{"frame_source.detail.depth_sidecar.max_mm", valid_samples > 0 ? static_cast<std::int64_t>(max_depth * 1000.0F) : 0});
        }
    }
    frame.source_timings = std::move(timings);
    return frame;
}

}  // namespace dedalus
