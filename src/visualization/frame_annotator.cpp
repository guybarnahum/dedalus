#include "dedalus/visualization/frame_annotator.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dedalus {
namespace {

struct RgbColor {
    std::uint8_t r{255U};
    std::uint8_t g{255U};
    std::uint8_t b{255U};
};

std::string class_label_to_string(const ClassLabel label) {
    switch (label) {
        case ClassLabel::Person:
            return "person";
        case ClassLabel::Drone:
            return "drone";
        case ClassLabel::Car:
            return "car";
        case ClassLabel::Boat:
            return "boat";
        case ClassLabel::House:
            return "house";
        case ClassLabel::Building:
            return "building";
        case ClassLabel::Tree:
            return "tree";
        case ClassLabel::Road:
            return "road";
        case ClassLabel::River:
            return "river";
        case ClassLabel::Terrain:
            return "terrain";
        case ClassLabel::Unknown:
        default:
            return "unknown";
    }
}

std::string track_state_to_string(const TrackState state) {
    switch (state) {
        case TrackState::Tentative:
            return "tentative";
        case TrackState::Confirmed:
            return "confirmed";
        case TrackState::Lost:
            return "lost";
    }
    return "unknown";
}

std::array<std::uint8_t, 7> glyph_for(char ch) {
    ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    switch (ch) {
        case 'A': return {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11};
        case 'B': return {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E};
        case 'C': return {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E};
        case 'D': return {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E};
        case 'E': return {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F};
        case 'F': return {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10};
        case 'G': return {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F};
        case 'H': return {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11};
        case 'I': return {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E};
        case 'J': return {0x01, 0x01, 0x01, 0x01, 0x11, 0x11, 0x0E};
        case 'K': return {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11};
        case 'L': return {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F};
        case 'M': return {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11};
        case 'N': return {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11};
        case 'O': return {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E};
        case 'P': return {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10};
        case 'Q': return {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D};
        case 'R': return {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11};
        case 'S': return {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E};
        case 'T': return {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04};
        case 'U': return {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E};
        case 'V': return {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04};
        case 'W': return {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A};
        case 'X': return {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11};
        case 'Y': return {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04};
        case 'Z': return {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F};
        case '0': return {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E};
        case '1': return {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E};
        case '2': return {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F};
        case '3': return {0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E};
        case '4': return {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02};
        case '5': return {0x1F, 0x10, 0x10, 0x1E, 0x01, 0x01, 0x1E};
        case '6': return {0x0E, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x0E};
        case '7': return {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08};
        case '8': return {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E};
        case '9': return {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E};
        case ':': return {0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x00};
        case '.': return {0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04};
        case '-': return {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00};
        case '_': return {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F};
        case '/': return {0x01, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10};
        case ' ': return {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        default: return {0x1F, 0x01, 0x02, 0x04, 0x04, 0x00, 0x04};
    }
}

void set_pixel(ImageView& image, const int x, const int y, const RgbColor color) {
    if (x < 0 || y < 0 || x >= image.width || y >= image.height || image.channels != 3) {
        return;
    }
    const auto offset = static_cast<std::size_t>((y * image.width + x) * image.channels);
    if (offset + 2U >= image.bytes.size()) {
        return;
    }
    image.bytes[offset] = color.r;
    image.bytes[offset + 1U] = color.g;
    image.bytes[offset + 2U] = color.b;
}

void fill_rect(ImageView& image, int x, int y, int width, int height, const RgbColor color) {
    if (width <= 0 || height <= 0) {
        return;
    }
    const int x0 = std::clamp(x, 0, image.width);
    const int y0 = std::clamp(y, 0, image.height);
    const int x1 = std::clamp(x + width, 0, image.width);
    const int y1 = std::clamp(y + height, 0, image.height);
    for (int py = y0; py < y1; ++py) {
        for (int px = x0; px < x1; ++px) {
            set_pixel(image, px, py, color);
        }
    }
}

void draw_rect(ImageView& image, int x, int y, int width, int height, const RgbColor color, const int thickness) {
    if (width <= 0 || height <= 0 || thickness <= 0) {
        return;
    }
    fill_rect(image, x, y, width, thickness, color);
    fill_rect(image, x, y + height - thickness, width, thickness, color);
    fill_rect(image, x, y, thickness, height, color);
    fill_rect(image, x + width - thickness, y, thickness, height, color);
}

void draw_text(ImageView& image, int x, int y, const std::string& text, const RgbColor color, const int scale) {
    int cursor_x = x;
    const int safe_scale = std::max(1, scale);
    for (const char ch : text) {
        const auto glyph = glyph_for(ch);
        for (int row = 0; row < 7; ++row) {
            for (int col = 0; col < 5; ++col) {
                const auto bit = static_cast<std::uint8_t>(1U << (4 - col));
                if ((glyph[static_cast<std::size_t>(row)] & bit) != 0U) {
                    fill_rect(image, cursor_x + col * safe_scale, y + row * safe_scale, safe_scale, safe_scale, color);
                }
            }
        }
        cursor_x += 6 * safe_scale;
    }
}

std::string frame_file_name(const std::size_t frame_index) {
    std::ostringstream stream;
    stream << "frame_" << std::setw(6) << std::setfill('0') << frame_index << ".ppm";
    return stream.str();
}

int rounded_int(const double value) {
    return static_cast<int>(std::lround(value));
}

void validate_rgb_image(const ImageView& image) {
    if (image.width <= 0 || image.height <= 0 || image.channels != 3) {
        throw std::runtime_error("PPM frame annotation sink requires non-empty RGB images");
    }

    const auto expected_size = static_cast<std::size_t>(image.width * image.height * image.channels);
    if (image.bytes.size() != expected_size) {
        throw std::runtime_error("PPM frame annotation sink image byte count does not match dimensions");
    }
}

void write_ppm(const std::filesystem::path& path, const ImageView& image) {
    std::ofstream output{path, std::ios::binary};
    if (!output) {
        throw std::runtime_error("failed to open PPM annotation output: " + path.string());
    }

    output << "P6\n" << image.width << ' ' << image.height << "\n255\n";
    output.write(
        reinterpret_cast<const char*>(image.bytes.data()),
        static_cast<std::streamsize>(image.bytes.size()));

    if (!output) {
        throw std::runtime_error("failed to write PPM annotation output: " + path.string());
    }
}

void append_manifest_row(
    const std::filesystem::path& manifest_path,
    const std::size_t frame_index,
    const AnnotationContext& context,
    const std::filesystem::path& frame_path,
    const double output_fps) {
    const bool write_header = !std::filesystem::exists(manifest_path);
    std::ofstream manifest{manifest_path, std::ios::app};
    if (!manifest) {
        throw std::runtime_error("failed to open annotation manifest: " + manifest_path.string());
    }

    if (write_header) {
        manifest << "frame_index,frame_id,timestamp_ns,path,output_fps\n";
    }

    manifest << frame_index << ','
             << context.frame.frame_id.value << ','
             << context.frame.timestamp.timestamp_ns << ','
             << frame_path.string() << ','
             << output_fps << '\n';
}

void draw_detection_overlay(ImageView& image, const Detection2D& detection) {
    const RgbColor detection_color{255U, 210U, 0U};
    const int x = rounded_int(detection.bbox_px.x);
    const int y = rounded_int(detection.bbox_px.y);
    const int w = rounded_int(detection.bbox_px.width);
    const int h = rounded_int(detection.bbox_px.height);
    draw_rect(image, x, y, w, h, detection_color, 2);

    const std::string label = "D:" + class_label_to_string(detection.class_label);
    const int label_y = std::max(0, y - 12);
    fill_rect(image, x, label_y, static_cast<int>(label.size()) * 6 + 4, 10, RgbColor{0U, 0U, 0U});
    draw_text(image, x + 2, label_y + 1, label, detection_color, 1);
}

void draw_track_overlay(ImageView& image, const Track2D& track) {
    const RgbColor track_color{0U, 255U, 180U};
    const int x = rounded_int(track.bbox_px.x);
    const int y = rounded_int(track.bbox_px.y);
    const int w = rounded_int(track.bbox_px.width);
    const int h = rounded_int(track.bbox_px.height);
    draw_rect(image, x - 3, y - 3, w + 6, h + 6, track_color, 1);

    std::ostringstream label;
    label << "T:" << track.track_id.value << " " << class_label_to_string(track.class_label);
    const int label_y = std::min(image.height - 10, std::max(0, y + h + 3));
    fill_rect(image, x, label_y, static_cast<int>(label.str().size()) * 6 + 4, 10, RgbColor{0U, 0U, 0U});
    draw_text(image, x + 2, label_y + 1, label.str(), track_color, 1);
}

void draw_world_metadata(ImageView& image, const AnnotationContext& context) {
    const RgbColor text_color{255U, 255U, 255U};
    fill_rect(image, 0, 0, image.width, 38, RgbColor{0U, 0U, 0U});

    const std::string line1 = "FRAME:" + context.frame.frame_id.value +
                              " MAP:" + context.world_snapshot.active_map_frame_id.value;
    const std::string line2 = "EGO MAP:" + context.world_snapshot.ego.map_frame_id.value +
                              " TS:" + std::to_string(context.frame.timestamp.timestamp_ns);
    draw_text(image, 6, 5, line1, text_color, 1);
    draw_text(image, 6, 19, line2, text_color, 1);
}

void draw_world_debug_shapes(ImageView& image, const WorldSnapshot& snapshot) {
    const RgbColor zone_color{255U, 80U, 80U};
    const RgbColor corridor_color{90U, 160U, 255U};
    const RgbColor landmark_color{190U, 120U, 255U};

    int y = 44;
    for (const auto& zone : snapshot.tactical_exclusion_zones) {
        draw_rect(image, 8, y, 22, 12, zone_color, 1);
        draw_text(image, 36, y + 2, "ZONE:" + zone.zone_id.value, zone_color, 1);
        y += 16;
        if (y > image.height - 20) {
            break;
        }
    }

    y = image.height - 18;
    for (const auto& corridor : snapshot.flight_corridors) {
        draw_rect(image, 8, y, 50, 8, corridor_color, 1);
        draw_text(image, 64, y, "COR:" + corridor.corridor_id.value, corridor_color, 1);
        y -= 14;
        if (y < image.height / 2) {
            break;
        }
    }

    int x = image.width - 110;
    y = 44;
    for (const auto& landmark : snapshot.landmarks) {
        draw_rect(image, x, y, 8, 8, landmark_color, 1);
        draw_text(image, x + 12, y, "LM:" + landmark.landmark_id.value, landmark_color, 1);
        y += 14;
        if (y > image.height - 20) {
            break;
        }
    }
}

}  // namespace

void NullFrameAnnotationSink::annotate(const AnnotationContext&) {}

void NullFrameAnnotationSink::finish() {}

PpmFrameAnnotationSink::PpmFrameAnnotationSink(std::string output_dir, double output_fps)
    : output_dir_(std::move(output_dir)), output_fps_(output_fps) {
    if (output_dir_.empty()) {
        throw std::invalid_argument("ppm_sequence frame annotation sink requires annotation_output_path");
    }
    if (output_fps_ <= 0.0) {
        throw std::invalid_argument("ppm_sequence frame annotation sink requires annotation_output_fps > 0");
    }
}

void PpmFrameAnnotationSink::annotate(const AnnotationContext& context) {
    validate_rgb_image(context.frame.image);

    std::filesystem::create_directories(output_dir_);
    ImageView annotated = context.frame.image;

    draw_world_metadata(annotated, context);
    for (const auto& detection : context.perception.stabilized_frame.detections) {
        draw_detection_overlay(annotated, detection);
    }
    for (const auto& track : context.perception.tracks) {
        draw_track_overlay(annotated, track);
    }
    draw_world_debug_shapes(annotated, context.world_snapshot);

    ++frame_index_;
    const auto frame_path = std::filesystem::path{output_dir_} / frame_file_name(frame_index_);
    write_ppm(frame_path, annotated);
    append_manifest_row(
        std::filesystem::path{output_dir_} / "manifest.txt",
        frame_index_,
        context,
        frame_path,
        output_fps_);
}

void PpmFrameAnnotationSink::finish() {}

Mp4FrameAnnotationSink::Mp4FrameAnnotationSink(std::string output_path, double output_fps)
    : output_path_(std::move(output_path)), output_fps_(output_fps) {}

void Mp4FrameAnnotationSink::annotate(const AnnotationContext&) {
    throw std::runtime_error(
        "mp4 frame annotation sink is not implemented yet; use frame_annotator: null or ppm_sequence");
}

void Mp4FrameAnnotationSink::finish() {}

}  // namespace dedalus
