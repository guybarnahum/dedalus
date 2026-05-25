#include "dedalus/behavior/flight_command_sinks.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dedalus {
namespace {

constexpr std::uint8_t kMavlinkV2Stx = 0xFD;
constexpr std::uint32_t kMavlinkMsgIdCommandLong = 76U;
constexpr std::uint8_t kMavlinkCrcExtraCommandLong = 152U;
constexpr std::uint16_t kMavCmdDoGimbalManagerPitchYaw = 1000U;
constexpr double kRadToDeg = 57.2957795130823208768;

std::vector<std::string> split_csv(const std::string& value) {
    std::vector<std::string> out;
    std::string token;
    for (const char ch : value) {
        if (ch == ',') {
            if (!token.empty()) {
                out.push_back(token);
                token.clear();
            }
        } else if (ch != ' ' && ch != '\t' && ch != '\n' && ch != '\r') {
            token.push_back(ch);
        }
    }
    if (!token.empty()) {
        out.push_back(token);
    }
    return out;
}

std::string strip_prefix(std::string value, const std::string& prefix) {
    if (value.rfind(prefix, 0) == 0) {
        value.erase(0, prefix.size());
    }
    return value;
}

struct UdpTarget {
    std::string host;
    std::uint16_t port{0U};
    sockaddr_storage addr{};
    socklen_t addr_len{0U};
};

UdpTarget parse_udp_endpoint(std::string endpoint) {
    endpoint = strip_prefix(endpoint, "udpout:");
    endpoint = strip_prefix(endpoint, "udpin:");
    endpoint = strip_prefix(endpoint, "udp:");

    const auto sep = endpoint.rfind(':');
    if (sep == std::string::npos) {
        throw std::invalid_argument("MAVLink gimbal endpoint must be host:port: " + endpoint);
    }

    UdpTarget target;
    target.host = endpoint.substr(0, sep);
    const auto port_str = endpoint.substr(sep + 1U);
    const int port_int = std::stoi(port_str);
    if (port_int <= 0 || port_int > 65535) {
        throw std::invalid_argument("invalid MAVLink gimbal endpoint port: " + endpoint);
    }
    target.port = static_cast<std::uint16_t>(port_int);

    addrinfo hints{};
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_family = AF_UNSPEC;

    addrinfo* result = nullptr;
    const int rc = ::getaddrinfo(target.host.c_str(), port_str.c_str(), &hints, &result);
    if (rc != 0 || result == nullptr) {
        throw std::runtime_error(
            "failed to resolve MAVLink gimbal endpoint " + endpoint + ": " +
            (rc == 0 ? std::string{"no address"} : std::string{::gai_strerror(rc)}));
    }

    std::memcpy(&target.addr, result->ai_addr, result->ai_addrlen);
    target.addr_len = static_cast<socklen_t>(result->ai_addrlen);
    ::freeaddrinfo(result);
    return target;
}

void append_u8(std::vector<std::uint8_t>& out, std::uint8_t value) {
    out.push_back(value);
}

void append_u16_le(std::vector<std::uint8_t>& out, std::uint16_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    out.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
}

void append_u32_le(std::vector<std::uint8_t>& out, std::uint32_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    out.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
    out.push_back(static_cast<std::uint8_t>((value >> 16U) & 0xFFU));
    out.push_back(static_cast<std::uint8_t>((value >> 24U) & 0xFFU));
}

void append_float_le(std::vector<std::uint8_t>& out, float value) {
    static_assert(sizeof(float) == 4U);
    std::uint32_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(float));
    append_u32_le(out, bits);
}

std::uint16_t crc_accumulate(std::uint8_t byte, std::uint16_t crc) {
    byte ^= static_cast<std::uint8_t>(crc & 0xFFU);
    byte ^= static_cast<std::uint8_t>(byte << 4U);
    return static_cast<std::uint16_t>(
        (crc >> 8U) ^
        (static_cast<std::uint16_t>(byte) << 8U) ^
        (static_cast<std::uint16_t>(byte) << 3U) ^
        (static_cast<std::uint16_t>(byte) >> 4U));
}

std::uint16_t mavlink_crc_x25(const std::vector<std::uint8_t>& bytes, std::uint8_t crc_extra) {
    std::uint16_t crc = 0xFFFFU;
    for (const auto byte : bytes) {
        crc = crc_accumulate(byte, crc);
    }
    crc = crc_accumulate(crc_extra, crc);
    return crc;
}

std::vector<std::uint8_t> build_command_long_payload(
    std::uint8_t target_system,
    std::uint8_t target_component,
    std::uint16_t command,
    std::uint8_t confirmation,
    float param1,
    float param2,
    float param3,
    float param4,
    float param5,
    float param6,
    float param7) {
    std::vector<std::uint8_t> payload;
    payload.reserve(33U);

    // COMMAND_LONG payload wire order:
    // param1..param7 float, command uint16, target_system uint8,
    // target_component uint8, confirmation uint8.
    append_float_le(payload, param1);
    append_float_le(payload, param2);
    append_float_le(payload, param3);
    append_float_le(payload, param4);
    append_float_le(payload, param5);
    append_float_le(payload, param6);
    append_float_le(payload, param7);
    append_u16_le(payload, command);
    append_u8(payload, target_system);
    append_u8(payload, target_component);
    append_u8(payload, confirmation);
    return payload;
}

std::vector<std::uint8_t> build_mavlink_v2_packet(
    std::uint8_t sequence,
    std::uint8_t source_system,
    std::uint8_t source_component,
    std::uint32_t message_id,
    std::uint8_t crc_extra,
    const std::vector<std::uint8_t>& payload) {
    if (payload.size() > 255U) {
        throw std::invalid_argument("MAVLink v2 payload too large");
    }

    std::vector<std::uint8_t> packet;
    packet.reserve(12U + payload.size());
    packet.push_back(kMavlinkV2Stx);
    packet.push_back(static_cast<std::uint8_t>(payload.size()));
    packet.push_back(0U);  // incompat_flags
    packet.push_back(0U);  // compat_flags
    packet.push_back(sequence);
    packet.push_back(source_system);
    packet.push_back(source_component);
    packet.push_back(static_cast<std::uint8_t>(message_id & 0xFFU));
    packet.push_back(static_cast<std::uint8_t>((message_id >> 8U) & 0xFFU));
    packet.push_back(static_cast<std::uint8_t>((message_id >> 16U) & 0xFFU));
    packet.insert(packet.end(), payload.begin(), payload.end());

    // CRC is over packet bytes excluding STX, followed by CRC_EXTRA.
    std::vector<std::uint8_t> crc_bytes(packet.begin() + 1, packet.end());
    const auto crc = mavlink_crc_x25(crc_bytes, crc_extra);
    append_u16_le(packet, crc);
    return packet;
}

std::string endpoint_summary(const std::vector<UdpTarget>& targets) {
    std::ostringstream out;
    for (std::size_t i = 0; i < targets.size(); ++i) {
        if (i > 0U) {
            out << ",";
        }
        out << targets[i].host << ":" << targets[i].port;
    }
    return out.str();
}

}  // namespace

struct MavlinkGimbalPointingSink::Impl {
    explicit Impl(MavlinkGimbalPointingSinkConfig config)
        : config(std::move(config)) {
        if (this->config.endpoints.empty()) {
            throw std::invalid_argument("MavlinkGimbalPointingSink requires at least one endpoint");
        }
        if (this->config.deadband_rad < 0.0) {
            throw std::invalid_argument("MavlinkGimbalPointingSink requires non-negative deadband");
        }
        if (this->config.resend_interval_s < 0.0) {
            throw std::invalid_argument("MavlinkGimbalPointingSink requires non-negative resend interval");
        }

        socket_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd < 0) {
            throw std::runtime_error("failed to create MAVLink gimbal UDP socket: " + std::string(std::strerror(errno)));
        }

        for (const auto& endpoint : split_csv(this->config.endpoints)) {
            targets.push_back(parse_udp_endpoint(endpoint));
        }
        if (targets.empty()) {
            throw std::invalid_argument("MavlinkGimbalPointingSink endpoint list parsed empty");
        }

        if (this->config.debug_logging) {
            std::cerr << "dedalus_mavlink_gimbal_sink: endpoints=" << endpoint_summary(targets)
                      << " source_system=" << static_cast<int>(this->config.source_system_id)
                      << " source_component=" << static_cast<int>(this->config.source_component_id)
                      << " target_system=" << static_cast<int>(this->config.target_system_id)
                      << " target_component=" << static_cast<int>(this->config.target_component_id)
                      << " gimbal_device_id=" << static_cast<int>(this->config.gimbal_device_id)
                      << "\n";
        }
    }

    ~Impl() {
        if (socket_fd >= 0) {
            ::close(socket_fd);
            socket_fd = -1;
        }
    }

    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;

    bool should_send(double pitch_rad) const {
        if (!last_pitch_valid) {
            return true;
        }
        if (std::abs(pitch_rad - last_pitch_rad) >= config.deadband_rad) {
            return true;
        }
        if (config.resend_interval_s <= 0.0) {
            return false;
        }
        const auto now = std::chrono::steady_clock::now();
        const std::chrono::duration<double> elapsed = now - last_send_time;
        return elapsed.count() >= config.resend_interval_s;
    }

    std::vector<std::uint8_t> build_pitchyaw_command(double pitch_rad) {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const float pitch_deg = static_cast<float>(pitch_rad * kRadToDeg);
        const float yaw_deg = nan;
        const float pitch_rate_deg_s = nan;
        const float yaw_rate_deg_s = nan;
        const float flags = static_cast<float>(config.gimbal_manager_flags);
        const float reserved = 0.0F;
        const float gimbal_device_id = static_cast<float>(config.gimbal_device_id);

        const auto payload = build_command_long_payload(
            config.target_system_id,
            config.target_component_id,
            kMavCmdDoGimbalManagerPitchYaw,
            0U,
            pitch_deg,
            yaw_deg,
            pitch_rate_deg_s,
            yaw_rate_deg_s,
            flags,
            reserved,
            gimbal_device_id);

        const auto packet = build_mavlink_v2_packet(
            sequence++,
            config.source_system_id,
            config.source_component_id,
            kMavlinkMsgIdCommandLong,
            kMavlinkCrcExtraCommandLong,
            payload);

        return packet;
    }

    CameraPointingResult send(const CameraPointingCommand& command) {
        if (!command.pitch_valid) {
            return CameraPointingResult{false, "camera_pointing_invalid"};
        }
        if (!should_send(command.pitch_rad)) {
            return CameraPointingResult{true, "OK mavlink gimbal command suppressed by deadband"};
        }

        const auto packet = build_pitchyaw_command(command.pitch_rad);
        int sent_count = 0;
        for (const auto& target : targets) {
            const auto sent = ::sendto(
                socket_fd,
                packet.data(),
                packet.size(),
                0,
                reinterpret_cast<const sockaddr*>(&target.addr),
                target.addr_len);
            if (sent < 0 || static_cast<std::size_t>(sent) != packet.size()) {
                throw std::runtime_error("failed sending MAVLink gimbal packet: " + std::string(std::strerror(errno)));
            }
            ++sent_count;
        }

        last_pitch_rad = command.pitch_rad;
        last_pitch_valid = true;
        last_send_time = std::chrono::steady_clock::now();

        if (config.debug_logging) {
            std::cerr << "dedalus_mavlink_gimbal_sink: mode=" << command.mode
                      << " pitch_deg=" << command.pitch_rad * kRadToDeg
                      << " endpoints=" << sent_count
                      << "\n";
        }

        std::ostringstream status;
        status << "OK mavlink gimbal command sent endpoints=" << sent_count
               << " pitch_deg=" << command.pitch_rad * kRadToDeg;
        return CameraPointingResult{true, status.str()};
    }

    MavlinkGimbalPointingSinkConfig config;
    int socket_fd{-1};
    std::vector<UdpTarget> targets;
    std::uint8_t sequence{0U};
    bool last_pitch_valid{false};
    double last_pitch_rad{0.0};
    std::chrono::steady_clock::time_point last_send_time{};
};

MavlinkGimbalPointingSink::MavlinkGimbalPointingSink(MavlinkGimbalPointingSinkConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

MavlinkGimbalPointingSink::~MavlinkGimbalPointingSink() = default;

CameraPointingResult MavlinkGimbalPointingSink::send(const CameraPointingCommand& command) {
    return impl_->send(command);
}

}  // namespace dedalus
