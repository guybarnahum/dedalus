#include "dedalus/behavior/flight_command_sinks.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace dedalus {
namespace {

constexpr std::uint8_t kMavlinkV1Magic = 0xFEU;
constexpr std::uint8_t kMavlinkFrameLocalNed = 1U;
constexpr std::uint16_t kMavCmdNavLand = 21U;
constexpr std::uint16_t kMavCmdNavTakeoff = 22U;
constexpr std::uint16_t kMavCmdComponentArmDisarm = 400U;
constexpr std::uint16_t kMavCmdDoSetMode = 176U;
constexpr std::uint8_t kMsgIdHeartbeat = 0U;
constexpr std::uint8_t kMsgIdCommandLong = 76U;
constexpr std::uint8_t kMsgIdSetPositionTargetLocalNed = 84U;
constexpr std::uint8_t kCrcExtraCommandLong = 152U;
constexpr std::uint8_t kCrcExtraSetPositionTargetLocalNed = 143U;
constexpr std::uint16_t kTypeMaskVelocityOnly =
    (1U << 0U) |
    (1U << 1U) |
    (1U << 2U) |
    (1U << 6U) |
    (1U << 7U) |
    (1U << 8U) |
    (1U << 10U) |
    (1U << 11U);

struct Endpoint {
    sockaddr_in configured_address{};
    sockaddr_in learned_peer{};
    std::string text;
    int socket_fd{-1};
    bool bind_and_learn{false};
    bool has_learned_peer{false};
};

// Allowlist for the tmux session:window target passed to `tmux send-keys -t`.
// Typical value: "dedalus-sim:px4".  Rejects shell metacharacters that could
// escape the shell_quote() wrapping or manipulate the tmux command.
void validate_tmux_target(const std::string& target) {
    if (target.empty()) {
        throw std::invalid_argument("px4_tmux_target must not be empty");
    }
    for (const unsigned char ch : target) {
        if (std::isalnum(ch) || ch == '-' || ch == '_' || ch == '.' || ch == ':') {
            continue;
        }
        throw std::invalid_argument(
            std::string("px4_tmux_target contains disallowed character '") +
            static_cast<char>(ch) + "': " + target);
    }
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

std::vector<std::string> split_csv(const std::string& value) {
    std::vector<std::string> output;
    std::string token;
    std::istringstream input{value};
    while (std::getline(input, token, ',')) {
        const auto first = token.find_first_not_of(" \t\n\r");
        if (first == std::string::npos) {
            continue;
        }
        const auto last = token.find_last_not_of(" \t\n\r");
        output.push_back(token.substr(first, last - first + 1U));
    }
    return output;
}

Endpoint parse_endpoint(std::string endpoint) {
    constexpr auto udpout_prefix = "udpout:";
    constexpr auto udpin_prefix = "udpin:";
    Endpoint parsed;
    if (endpoint.rfind(udpout_prefix, 0U) == 0U) {
        endpoint = endpoint.substr(std::strlen(udpout_prefix));
        parsed.bind_and_learn = false;
    } else if (endpoint.rfind(udpin_prefix, 0U) == 0U) {
        endpoint = endpoint.substr(std::strlen(udpin_prefix));
        parsed.bind_and_learn = true;
    }

    const auto colon = endpoint.rfind(':');
    if (colon == std::string::npos) {
        throw std::invalid_argument("MAVLink endpoint missing host:port: " + endpoint);
    }
    const std::string host = endpoint.substr(0U, colon);
    const int port = std::stoi(endpoint.substr(colon + 1U));
    if (port <= 0 || port > 65535) {
        throw std::invalid_argument("MAVLink endpoint port out of range: " + endpoint);
    }

    parsed.text = (parsed.bind_and_learn ? "udpin:" : "udpout:") + host + ":" + std::to_string(port);
    parsed.configured_address.sin_family = AF_INET;
    parsed.configured_address.sin_port = htons(static_cast<std::uint16_t>(port));
    if (inet_pton(AF_INET, host.c_str(), &parsed.configured_address.sin_addr) != 1) {
        throw std::invalid_argument("MAVLink endpoint host must be an IPv4 address: " + host);
    }
    return parsed;
}

void crc_accumulate(std::uint8_t data, std::uint16_t& crc) {
    data ^= static_cast<std::uint8_t>(crc & 0xFFU);
    data ^= static_cast<std::uint8_t>(data << 4U);
    crc = static_cast<std::uint16_t>(
        (crc >> 8U) ^
        (static_cast<std::uint16_t>(data) << 8U) ^
        (static_cast<std::uint16_t>(data) << 3U) ^
        (static_cast<std::uint16_t>(data) >> 4U));
}

std::uint16_t mavlink_crc(const std::vector<std::uint8_t>& header_without_magic, const std::vector<std::uint8_t>& payload, std::uint8_t crc_extra) {
    std::uint16_t crc = 0xFFFFU;
    for (const auto byte : header_without_magic) {
        crc_accumulate(byte, crc);
    }
    for (const auto byte : payload) {
        crc_accumulate(byte, crc);
    }
    crc_accumulate(crc_extra, crc);
    return crc;
}

void append_u16(std::vector<std::uint8_t>& bytes, std::uint16_t value) {
    bytes.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    bytes.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
}

void append_u32(std::vector<std::uint8_t>& bytes, std::uint32_t value) {
    bytes.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    bytes.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
    bytes.push_back(static_cast<std::uint8_t>((value >> 16U) & 0xFFU));
    bytes.push_back(static_cast<std::uint8_t>((value >> 24U) & 0xFFU));
}

void append_float(std::vector<std::uint8_t>& bytes, float value) {
    static_assert(sizeof(float) == sizeof(std::uint32_t));
    std::uint32_t raw = 0U;
    std::memcpy(&raw, &value, sizeof(float));
    append_u32(bytes, raw);
}

std::vector<std::uint8_t> packet_v1(
    std::uint8_t sequence,
    std::uint8_t source_system,
    std::uint8_t source_component,
    std::uint8_t message_id,
    std::uint8_t crc_extra,
    const std::vector<std::uint8_t>& payload) {
    if (payload.size() > 255U) {
        throw std::invalid_argument("MAVLink v1 payload too large");
    }

    std::vector<std::uint8_t> header;
    header.push_back(static_cast<std::uint8_t>(payload.size()));
    header.push_back(sequence);
    header.push_back(source_system);
    header.push_back(source_component);
    header.push_back(message_id);

    const auto crc = mavlink_crc(header, payload, crc_extra);

    std::vector<std::uint8_t> packet;
    packet.reserve(1U + header.size() + payload.size() + 2U);
    packet.push_back(kMavlinkV1Magic);
    packet.insert(packet.end(), header.begin(), header.end());
    packet.insert(packet.end(), payload.begin(), payload.end());
    append_u16(packet, crc);
    return packet;
}

std::vector<std::uint8_t> command_long_payload(
    std::uint8_t target_system,
    std::uint8_t target_component,
    std::uint16_t command,
    float p1 = 0.0F,
    float p2 = 0.0F,
    float p3 = 0.0F,
    float p4 = 0.0F,
    float p5 = 0.0F,
    float p6 = 0.0F,
    float p7 = 0.0F) {
    std::vector<std::uint8_t> payload;
    payload.reserve(33U);
    append_float(payload, p1);
    append_float(payload, p2);
    append_float(payload, p3);
    append_float(payload, p4);
    append_float(payload, p5);
    append_float(payload, p6);
    append_float(payload, p7);
    append_u16(payload, command);
    payload.push_back(target_system);
    payload.push_back(target_component);
    payload.push_back(0U);
    return payload;
}

std::vector<std::uint8_t> set_position_target_local_ned_payload(
    std::uint32_t time_boot_ms,
    std::uint8_t target_system,
    std::uint8_t target_component,
    Vec3 velocity_mps) {
    std::vector<std::uint8_t> payload;
    payload.reserve(53U);
    append_u32(payload, time_boot_ms);
    append_float(payload, 0.0F);
    append_float(payload, 0.0F);
    append_float(payload, 0.0F);
    append_float(payload, static_cast<float>(velocity_mps.x));
    append_float(payload, static_cast<float>(velocity_mps.y));
    append_float(payload, static_cast<float>(velocity_mps.z));
    append_float(payload, 0.0F);
    append_float(payload, 0.0F);
    append_float(payload, 0.0F);
    append_float(payload, 0.0F);
    append_float(payload, 0.0F);
    append_u16(payload, kTypeMaskVelocityOnly);
    payload.push_back(target_system);
    payload.push_back(target_component);
    payload.push_back(kMavlinkFrameLocalNed);
    return payload;
}

std::uint32_t time_boot_ms() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count() & 0xFFFFFFFFU);
}

std::string sockaddr_to_string(const sockaddr_in& address) {
    char ip[INET_ADDRSTRLEN]{};
    inet_ntop(AF_INET, &address.sin_addr, ip, sizeof(ip));
    return std::string{ip} + ":" + std::to_string(ntohs(address.sin_port));
}

}  // namespace

struct Px4MavlinkCommandSink::Impl {
    explicit Impl(Px4MavlinkCommandSinkConfig config)
        : config(std::move(config)) {
        if (this->config.command_duration_s <= 0.0) {
            throw std::invalid_argument("Px4MavlinkCommandSink requires positive command_duration_s");
        }
        if (this->config.max_velocity_mps <= 0.0) {
            throw std::invalid_argument("Px4MavlinkCommandSink requires positive max_velocity_mps");
        }

        for (const auto& endpoint_text : split_csv(this->config.endpoints)) {
            auto endpoint = parse_endpoint(endpoint_text);
            endpoint.socket_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
            if (endpoint.socket_fd < 0) {
                throw std::runtime_error("failed to create MAVLink UDP socket: " + std::string(std::strerror(errno)));
            }
            if (endpoint.bind_and_learn) {
                int reuse = 1;
                (void)::setsockopt(endpoint.socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
                if (::bind(
                        endpoint.socket_fd,
                        reinterpret_cast<const sockaddr*>(&endpoint.configured_address),
                        sizeof(endpoint.configured_address)) < 0) {
                    throw std::runtime_error("failed to bind MAVLink UDP endpoint " + endpoint.text + ": " + std::strerror(errno));
                }
            }
            endpoints.push_back(endpoint);
        }
        if (endpoints.empty()) {
            throw std::invalid_argument("Px4MavlinkCommandSink requires at least one endpoint");
        }
    }

    ~Impl() {
        for (auto& endpoint : endpoints) {
            if (endpoint.socket_fd >= 0) {
                ::close(endpoint.socket_fd);
                endpoint.socket_fd = -1;
            }
        }
    }

    Vec3 bounded_velocity(Vec3 velocity) const {
        const double limit = config.max_velocity_mps;
        velocity.x = std::max(-limit, std::min(limit, velocity.x));
        velocity.y = std::max(-limit, std::min(limit, velocity.y));
        velocity.z = std::max(-limit, std::min(limit, velocity.z));
        return velocity;
    }

    void learn_peer_if_needed(Endpoint& endpoint) {
        if (!endpoint.bind_and_learn || endpoint.has_learned_peer) {
            return;
        }

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{4};
        while (std::chrono::steady_clock::now() < deadline) {
            fd_set read_set;
            FD_ZERO(&read_set);
            FD_SET(endpoint.socket_fd, &read_set);
            timeval timeout{};
            timeout.tv_sec = 0;
            timeout.tv_usec = 200000;
            const int ready = ::select(endpoint.socket_fd + 1, &read_set, nullptr, nullptr, &timeout);
            if (ready < 0) {
                throw std::runtime_error("select failed on MAVLink endpoint " + endpoint.text + ": " + std::strerror(errno));
            }
            if (ready == 0) {
                continue;
            }

            std::uint8_t buffer[512]{};
            sockaddr_in peer{};
            socklen_t peer_len = sizeof(peer);
            const auto received = ::recvfrom(
                endpoint.socket_fd,
                buffer,
                sizeof(buffer),
                0,
                reinterpret_cast<sockaddr*>(&peer),
                &peer_len);
            if (received < 6) {
                continue;
            }
            if (buffer[0] != kMavlinkV1Magic) {
                continue;
            }
            const std::uint8_t message_id = buffer[5];
            if (message_id != kMsgIdHeartbeat) {
                continue;
            }
            endpoint.learned_peer = peer;
            endpoint.has_learned_peer = true;
            if (config.debug_logging) {
                std::cerr << "dedalus_px4_mavlink_sink: learned_peer endpoint=" << endpoint.text
                          << " peer=" << sockaddr_to_string(peer)
                          << " system=" << static_cast<int>(buffer[3])
                          << " component=" << static_cast<int>(buffer[4])
                          << "\n";
            }
            return;
        }

        throw std::runtime_error("timed out waiting for MAVLink heartbeat on " + endpoint.text);
    }

    void send_packet(const std::vector<std::uint8_t>& packet) {
        bool sent_any = false;
        for (auto& endpoint : endpoints) {
            learn_peer_if_needed(endpoint);
            const sockaddr_in& destination = endpoint.bind_and_learn ? endpoint.learned_peer : endpoint.configured_address;
            const auto sent = ::sendto(
                endpoint.socket_fd,
                packet.data(),
                packet.size(),
                0,
                reinterpret_cast<const sockaddr*>(&destination),
                sizeof(destination));
            if (sent < 0) {
                throw std::runtime_error("failed to send MAVLink UDP packet via " + endpoint.text + ": " + std::strerror(errno));
            }
            if (static_cast<std::size_t>(sent) != packet.size()) {
                throw std::runtime_error("short MAVLink UDP send via " + endpoint.text);
            }
            sent_any = true;
        }
        if (!sent_any) {
            throw std::runtime_error("no MAVLink endpoints available for send");
        }
    }

    void run_px4_shell(const std::string& command) const {
        validate_tmux_target(config.px4_tmux_target);
        const std::string rendered =
            "tmux send-keys -t " + shell_quote(config.px4_tmux_target) + " " + shell_quote(command) + " C-m";
        const int rc = std::system(rendered.c_str());
        if (rc != 0) {
            throw std::runtime_error("PX4 shell command failed: " + command);
        }
    }

    void send_command_long(std::uint16_t command, float p1 = 0.0F, float p2 = 0.0F, float p3 = 0.0F, float p4 = 0.0F, float p5 = 0.0F, float p6 = 0.0F, float p7 = 0.0F) {
        const auto payload = command_long_payload(
            config.target_system_id,
            config.target_component_id,
            command,
            p1,
            p2,
            p3,
            p4,
            p5,
            p6,
            p7);
        send_packet(packet_v1(next_sequence++, config.source_system_id, config.source_component_id, kMsgIdCommandLong, kCrcExtraCommandLong, payload));
    }

    void send_offboard_mode() {
        send_command_long(kMavCmdDoSetMode, 29.0F, 6.0F, 0.0F);
        offboard_requested = true;
    }

    void send_velocity(Vec3 velocity) {
        if (config.set_offboard_on_velocity && !offboard_requested) {
            prime_zero_setpoints();
            send_offboard_mode();
        }
        const auto payload = set_position_target_local_ned_payload(
            time_boot_ms(),
            config.target_system_id,
            config.target_component_id,
            bounded_velocity(velocity));
        send_packet(packet_v1(next_sequence++, config.source_system_id, config.source_component_id, kMsgIdSetPositionTargetLocalNed, kCrcExtraSetPositionTargetLocalNed, payload));
    }

    void prime_zero_setpoints() {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{2};
        while (std::chrono::steady_clock::now() < deadline) {
            const auto payload = set_position_target_local_ned_payload(
                time_boot_ms(),
                config.target_system_id,
                config.target_component_id,
                Vec3{0.0, 0.0, 0.0});
            send_packet(packet_v1(next_sequence++, config.source_system_id, config.source_component_id, kMsgIdSetPositionTargetLocalNed, kCrcExtraSetPositionTargetLocalNed, payload));
            std::this_thread::sleep_for(std::chrono::milliseconds{50});
        }
    }

    FlightCommandResult result(FlightCommandKind kind, const std::string& status) const {
        return FlightCommandResult{kind, true, status};
    }

    Px4MavlinkCommandSinkConfig config;
    std::vector<Endpoint> endpoints;
    std::uint8_t next_sequence{0U};
    bool offboard_requested{false};
};

Px4MavlinkCommandSink::Px4MavlinkCommandSink(Px4MavlinkCommandSinkConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

Px4MavlinkCommandSink::~Px4MavlinkCommandSink() = default;

FlightCommandResult Px4MavlinkCommandSink::send(const VelocityCommand& command) {
    switch (command.kind) {
        case FlightCommandKind::Arm:
            if (impl_->config.use_px4_shell_lifecycle) {
                impl_->run_px4_shell("commander arm");
            } else {
                impl_->send_command_long(kMavCmdComponentArmDisarm, 1.0F, 0.0F);
            }
            if (impl_->config.debug_logging) {
                std::cerr << "dedalus_px4_mavlink_sink: command=Arm dispatch="
                          << (impl_->config.use_px4_shell_lifecycle ? "px4_shell" : "mavlink") << "\n";
            }
            return impl_->result(command.kind, impl_->config.use_px4_shell_lifecycle ? "OK px4_shell command=arm" : "OK mavlink command=arm");
        case FlightCommandKind::Takeoff:
            if (impl_->config.use_px4_shell_lifecycle) {
                impl_->run_px4_shell("commander takeoff");
            } else {
                impl_->send_command_long(kMavCmdNavTakeoff, 0.0F, 0.0F, 0.0F, NAN, 0.0F, 0.0F, static_cast<float>(impl_->config.takeoff_altitude_m));
            }
            if (impl_->config.debug_logging) {
                std::cerr << "dedalus_px4_mavlink_sink: command=Takeoff dispatch="
                          << (impl_->config.use_px4_shell_lifecycle ? "px4_shell" : "mavlink")
                          << " altitude_m=" << impl_->config.takeoff_altitude_m << "\n";
            }
            return impl_->result(command.kind, impl_->config.use_px4_shell_lifecycle ? "OK px4_shell command=takeoff" : "OK mavlink command=takeoff");
        case FlightCommandKind::Land:
            if (impl_->config.use_px4_shell_lifecycle) {
                impl_->run_px4_shell("commander land");
            } else {
                impl_->send_command_long(kMavCmdNavLand, 0.0F, 0.0F, 0.0F, NAN, 0.0F, 0.0F, 0.0F);
            }
            if (impl_->config.debug_logging) {
                std::cerr << "dedalus_px4_mavlink_sink: command=Land dispatch="
                          << (impl_->config.use_px4_shell_lifecycle ? "px4_shell" : "mavlink") << "\n";
            }
            return impl_->result(command.kind, impl_->config.use_px4_shell_lifecycle ? "OK px4_shell command=land" : "OK mavlink command=land");
        case FlightCommandKind::Disarm:
            if (impl_->config.use_px4_shell_lifecycle) {
                impl_->run_px4_shell("commander disarm");
            } else {
                impl_->send_command_long(kMavCmdComponentArmDisarm, 0.0F, 0.0F);
            }
            if (impl_->config.debug_logging) {
                std::cerr << "dedalus_px4_mavlink_sink: command=Disarm dispatch="
                          << (impl_->config.use_px4_shell_lifecycle ? "px4_shell" : "mavlink") << "\n";
            }
            return impl_->result(command.kind, impl_->config.use_px4_shell_lifecycle ? "OK px4_shell command=disarm" : "OK mavlink command=disarm");
        case FlightCommandKind::Velocity:
        default: {
            const auto velocity = impl_->bounded_velocity(command.velocity_local_mps);
            impl_->send_velocity(velocity);
            if (impl_->config.debug_logging) {
                std::cerr << "dedalus_px4_mavlink_sink: command=Velocity vx=" << velocity.x
                          << " vy=" << velocity.y
                          << " vz=" << velocity.z
                          << " endpoint_count=" << impl_->endpoints.size()
                          << "\n";
            }
            std::ostringstream status;
            status << "OK mavlink command=velocity vx=" << velocity.x
                   << " vy=" << velocity.y
                   << " vz=" << velocity.z;
            return impl_->result(command.kind, status.str());
        }
    }
}

}  // namespace dedalus
