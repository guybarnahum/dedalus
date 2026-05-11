#include "dedalus/simulation/airsim_providers.hpp"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace dedalus {
namespace {

std::runtime_error unavailable(const char* provider_name) {
    return std::runtime_error(
        std::string{provider_name} +
        " is an integration provider and is not available in the dependency-free core build");
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

std::string run_bridge_command(const std::string& command) {
    std::array<char, 4096> buffer{};
    std::string output;

    FILE* pipe = popen(command.c_str(), "r");
    if (pipe == nullptr) {
        throw std::runtime_error("failed to start AirSim bridge command");
    }

    while (true) {
        const std::size_t bytes_read = std::fread(buffer.data(), 1U, buffer.size(), pipe);
        if (bytes_read > 0U) {
            output.append(buffer.data(), bytes_read);
        }
        if (bytes_read < buffer.size()) {
            if (std::feof(pipe) != 0) {
                break;
            }
            if (std::ferror(pipe) != 0) {
                const int status = pclose(pipe);
                (void)status;
                throw std::runtime_error("failed while reading AirSim bridge command output");
            }
        }
    }

    const int status = pclose(pipe);
    if (status != 0) {
        throw std::runtime_error("AirSim bridge command failed with status " + std::to_string(status));
    }

    if (output.empty()) {
        throw std::runtime_error("AirSim bridge command produced no image data");
    }

    return output;
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

    input.get();  // consume one byte of whitespace after max value

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

}  // namespace

AirSimFrameSource::AirSimFrameSource(AirSimProviderConfig config)
    : config_(std::move(config)) {}

std::optional<FramePacket> AirSimFrameSource::next_frame() {
    std::ostringstream command;
    command << config_.bridge_command
            << " --host " << shell_quote(config_.host)
            << " --rpc-port " << config_.rpc_port
            << " --vehicle-name " << shell_quote(config_.vehicle_name)
            << " --camera-name " << shell_quote(config_.camera_name);

    FramePacket frame;
    frame.frame_id = FrameId{"airsim_live_frame_" + std::to_string(++next_frame_index_)};
    frame.timestamp = TimePoint{0};
    frame.camera_id = CameraId{config_.camera_name};
    frame.image = parse_ppm_bytes(run_bridge_command(command.str()));
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

AirSimEgoStateProvider::AirSimEgoStateProvider(AirSimProviderConfig config)
    : config_(std::move(config)) {}

EgoStateEstimate AirSimEgoStateProvider::estimate(const FramePacket&) {
    throw unavailable("AirSimEgoStateProvider");
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
