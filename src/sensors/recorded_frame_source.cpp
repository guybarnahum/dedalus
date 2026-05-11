#include "dedalus/sensors/recorded_frame_source.hpp"

#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace dedalus {
namespace {

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
    throw std::runtime_error("unexpected end of PPM file");
}

ImageView load_binary_ppm(const std::filesystem::path& path) {
    std::ifstream input{path, std::ios::binary};
    if (!input) {
        throw std::runtime_error("failed to open PPM image: " + path.string());
    }

    const std::string magic = read_ppm_token(input);
    if (magic != "P6") {
        throw std::runtime_error("unsupported PPM format in " + path.string() + ": expected P6");
    }

    const int width = std::stoi(read_ppm_token(input));
    const int height = std::stoi(read_ppm_token(input));
    const int max_value = std::stoi(read_ppm_token(input));
    if (width <= 0 || height <= 0 || max_value != 255) {
        throw std::runtime_error("invalid PPM header in " + path.string());
    }

    input.get();  // consume one byte of whitespace after max value

    ImageView image;
    image.width = width;
    image.height = height;
    image.channels = 3;
    image.bytes.resize(static_cast<std::size_t>(width * height * image.channels));
    input.read(reinterpret_cast<char*>(image.bytes.data()), static_cast<std::streamsize>(image.bytes.size()));
    if (input.gcount() != static_cast<std::streamsize>(image.bytes.size())) {
        throw std::runtime_error("truncated PPM image: " + path.string());
    }

    return image;
}

RecordedFrameManifestEntry parse_manifest_line(const std::string& line) {
    std::istringstream input{line};

    std::string image_path;
    std::string frame_id;
    std::int64_t timestamp_ns = 0;
    std::string camera_id;

    if (!(input >> image_path >> frame_id >> timestamp_ns >> camera_id)) {
        throw std::runtime_error("invalid recorded-frame manifest line: " + line);
    }

    RecordedFrameManifestEntry entry;
    entry.image_path = image_path;
    entry.frame_id = FrameId{frame_id};
    entry.timestamp = TimePoint{timestamp_ns};
    entry.camera_id = CameraId{camera_id};
    return entry;
}

}  // namespace

RecordedFrameSource::RecordedFrameSource(std::filesystem::path manifest_path)
    : manifest_dir_(manifest_path.parent_path()) {
    std::ifstream input{manifest_path};
    if (!input) {
        throw std::runtime_error("failed to open recorded-frame manifest: " + manifest_path.string());
    }

    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line.front() == '#') {
            continue;
        }
        entries_.push_back(parse_manifest_line(line));
    }

    if (entries_.empty()) {
        throw std::runtime_error("recorded-frame manifest contains no frames: " + manifest_path.string());
    }
}

std::optional<FramePacket> RecordedFrameSource::next_frame() {
    if (next_index_ >= entries_.size()) {
        return std::nullopt;
    }

    const auto& entry = entries_.at(next_index_++);
    const auto image_path = entry.image_path.is_absolute() ? entry.image_path : manifest_dir_ / entry.image_path;

    FramePacket frame;
    frame.frame_id = entry.frame_id;
    frame.timestamp = entry.timestamp;
    frame.camera_id = entry.camera_id;
    frame.image = load_binary_ppm(image_path);
    frame.intrinsics.fx = 420.0;
    frame.intrinsics.fy = 420.0;
    frame.intrinsics.cx = static_cast<double>(frame.image.width) * 0.5;
    frame.intrinsics.cy = static_cast<double>(frame.image.height) * 0.5;

    AppearanceCondition appearance;
    appearance.timestamp = frame.timestamp;
    appearance.lighting_mode = LightingMode::Unknown;
    appearance.weather_mode = WeatherMode::Unknown;
    appearance.sensor_mode = SensorMode::Rgb;
    appearance.confidence = 0.4F;
    frame.appearance_condition = appearance;

    return frame;
}

}  // namespace dedalus
