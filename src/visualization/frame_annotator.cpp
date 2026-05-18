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

#include "dedalus/visualization/world_to_image_projector.hpp"
#include "dedalus/visualization/world_overlay_sidecar.hpp"

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

struct OverlayStyle {
    int glyph_scale = 1;
    int stroke_px = 1;
    int padding_px = 2;
    int line_gap_px = 2;
};

int clamp_int(const int value, const int lo, const int hi) {
    return std::max(lo, std::min(value, hi));
}

OverlayStyle choose_overlay_style(const int width, const int height) {
    const int min_dim = std::min(width, height);

    int scale = 1;
    if (min_dim <= 360) {
        scale = 1;
    } else if (min_dim <= 900) {
        scale = 2;
    } else if (min_dim <= 1600) {
        scale = 3;
    } else {
        scale = 4;
    }

    return OverlayStyle{
        .glyph_scale = scale,
        .stroke_px = std::max(1, scale),
        .padding_px = std::max(2, 2 * scale),
        .line_gap_px = std::max(2, scale),
    };
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

int text_pixel_width(const std::string& text, const int glyph_scale) {
    const int scale = std::max(1, glyph_scale);
    if (text.empty()) {
        return 0;
    }
    return static_cast<int>(text.size()) * 6 * scale;
}

int text_pixel_height(const int glyph_scale) {
    return 8 * std::max(1, glyph_scale);
}

void draw_text_background(
    ImageView& image,
    const int x,
    const int y,
    const std::string& text,
    const OverlayStyle& style,
    const RgbColor bg_color) {
    const int tw = text_pixel_width(text, style.glyph_scale);
    const int th = text_pixel_height(style.glyph_scale);
    const int left = clamp_int(x - style.padding_px, 0, image.width - 1);
    const int top = clamp_int(y - style.padding_px, 0, image.height - 1);
    const int right = clamp_int(x + tw + style.padding_px - 1, 0, image.width - 1);
    const int bottom = clamp_int(y + th + style.padding_px - 1, 0, image.height - 1);
    fill_rect(image, left, top, right - left + 1, bottom - top + 1, bg_color);
}

void draw_text_label(
    ImageView& image,
    const int x,
    const int y,
    const std::string& text,
    const OverlayStyle& style,
    const RgbColor fg_color,
    const RgbColor bg_color) {
    draw_text_background(image, x, y, text, style, bg_color);
    draw_text(image, x, y, text, fg_color, style.glyph_scale);
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

void draw_detection_overlay(
    ImageView& image, const Detection2D& detection, const OverlayStyle& style) {
    const RgbColor detection_color{255U, 210U, 0U};
    const int x = rounded_int(detection.bbox_px.x);
    const int y = rounded_int(detection.bbox_px.y);
    const int w = rounded_int(detection.bbox_px.width);
    const int h = rounded_int(detection.bbox_px.height);
    draw_rect(image, x, y, w, h, detection_color, style.stroke_px);

    const std::string label = "D:" + class_label_to_string(detection.class_label);
    const int label_x = clamp_int(x + style.padding_px, 0, std::max(0, image.width - 1));
    const int label_y = clamp_int(
        y - text_pixel_height(style.glyph_scale) - style.padding_px,
        0,
        std::max(0, image.height - 1));
    draw_text_label(image, label_x, label_y, label, style, detection_color, RgbColor{0U, 0U, 0U});
}

void draw_track_overlay(
    ImageView& image, const Track2D& track, const OverlayStyle& style) {
    const RgbColor track_color{0U, 255U, 180U};
    const int x = rounded_int(track.bbox_px.x);
    const int y = rounded_int(track.bbox_px.y);
    const int w = rounded_int(track.bbox_px.width);
    const int h = rounded_int(track.bbox_px.height);
    const int stroke = std::max(1, style.stroke_px);
    draw_rect(image, x - stroke, y - stroke, w + 2 * stroke, h + 2 * stroke, track_color, stroke);

    std::ostringstream label;
    label << "T:" << track.track_id.value << " " << class_label_to_string(track.class_label);
    const int track_label_x = clamp_int(x + style.padding_px, 0, std::max(0, image.width - 1));
    const int track_label_y = clamp_int(y + style.padding_px, 0, std::max(0, image.height - 1));
    draw_text_label(
        image, track_label_x, track_label_y, label.str(), style, track_color, RgbColor{0U, 0U, 0U});
}

void draw_world_metadata(ImageView& image, const AnnotationContext& context, const OverlayStyle& style) {
    const RgbColor text_color{255U, 255U, 255U};
    const RgbColor bg_color{0U, 0U, 0U};
    const int hud_x = style.padding_px * 2;
    int hud_y = style.padding_px * 2;
    const int hud_line_advance = text_pixel_height(style.glyph_scale) + style.line_gap_px;

    const std::string line1 = "FRAME:" + context.frame.frame_id.value +
                              " MAP:" + context.world_snapshot.active_map_frame_id.value;
    const std::string line2 = "EGO MAP:" + context.world_snapshot.ego.map_frame_id.value +
                              " TS:" + std::to_string(context.frame.timestamp.timestamp_ns);
    draw_text_label(image, hud_x, hud_y, line1, style, text_color, bg_color);
    hud_y += hud_line_advance;
    draw_text_label(image, hud_x, hud_y, line2, style, text_color, bg_color);
}

void draw_projected_agent_overlays(ImageView& image, const WorldSnapshot& snapshot, const OverlayStyle& style) {
    if (snapshot.agents.empty()) {
        return;
    }

    WorldToImageProjectionConfig projection_config;
    projection_config.intrinsics = pinhole_intrinsics_from_horizontal_fov(image.width, image.height, 90.0);
    projection_config.extrinsics.body_T_camera.position = Vec3{0.0, 0.0, 0.0};
    projection_config.extrinsics.body_T_camera.rotation_rpy = Vec3{0.0, 0.0, 0.0};

    const RgbColor agent_color{80U, 255U, 120U};
    const RgbColor bg_color{0U, 0U, 0U};
    const int marker_radius = std::max(5, 4 * style.stroke_px);
    const int marker_thickness = std::max(1, style.stroke_px);
    const int label_gap = std::max(3, style.padding_px);

    for (const auto& agent : snapshot.agents) {
        const auto projected = project_local_point_to_image(agent.position_local, snapshot.ego, projection_config);
        if (!projected.visible) {
            continue;
        }

        const int u = clamp_int(rounded_int(projected.u_px), 0, image.width - 1);
        const int v = clamp_int(rounded_int(projected.v_px), 0, image.height - 1);

        fill_rect(image, u - marker_radius, v - marker_thickness / 2, marker_radius * 2 + 1, marker_thickness, agent_color);
        fill_rect(image, u - marker_thickness / 2, v - marker_radius, marker_thickness, marker_radius * 2 + 1, agent_color);
        draw_rect(
            image,
            u - marker_radius,
            v - marker_radius,
            marker_radius * 2 + 1,
            marker_radius * 2 + 1,
            agent_color,
            marker_thickness);

        std::ostringstream label;
        label << "AG:" << agent.source_track_id.value << " " << class_label_to_string(agent.class_label)
              << " D" << std::fixed << std::setprecision(1) << projected.depth_m;

        const int label_w = text_pixel_width(label.str(), style.glyph_scale) + style.padding_px * 2;
        const int label_h = text_pixel_height(style.glyph_scale) + style.padding_px * 2;
        int label_x = u + marker_radius + label_gap;
        int label_y = v - marker_radius - label_h;
        if (label_x + label_w >= image.width) {
            label_x = u - marker_radius - label_gap - label_w;
        }
        if (label_y < 0) {
            label_y = v + marker_radius + label_gap;
        }
        label_x = clamp_int(label_x, 0, std::max(0, image.width - label_w));
        label_y = clamp_int(label_y, 0, std::max(0, image.height - label_h));
        draw_text_label(image, label_x + style.padding_px, label_y + style.padding_px, label.str(), style, agent_color, bg_color);
    }
}

void draw_world_debug_shapes(ImageView& image, const WorldSnapshot& snapshot, const OverlayStyle& style) {
    const RgbColor zone_color{255U, 80U, 80U};
    const RgbColor corridor_color{90U, 160U, 255U};
    const RgbColor landmark_color{190U, 120U, 255U};
    const RgbColor agent_color{80U, 255U, 120U};
    const RgbColor bg_color{0U, 0U, 0U};
    const int line_h = text_pixel_height(style.glyph_scale);
    const int line_advance = line_h + style.line_gap_px;
    const int indicator_size = std::max(8, line_h);
    const int text_offset_x = indicator_size + style.padding_px * 2;

    int y = style.padding_px * 2 + line_h * 2 + style.line_gap_px * 2 + style.padding_px;
    for (const auto& agent : snapshot.agents) {
        draw_rect(image, style.padding_px * 2, y, indicator_size, indicator_size, agent_color, style.stroke_px);
        std::ostringstream label;
        label << "AG:" << agent.source_track_id.value << " " << class_label_to_string(agent.class_label)
              << " C" << std::fixed << std::setprecision(2) << agent.confidence;
        const int text_x = style.padding_px * 2 + text_offset_x;
        draw_text_label(image, text_x, y, label.str(), style, agent_color, bg_color);
        y += line_advance;
        if (y > image.height - line_advance) {
            break;
        }
    }

    for (const auto& zone : snapshot.tactical_exclusion_zones) {
        draw_rect(image, style.padding_px * 2, y, indicator_size, indicator_size, zone_color, style.stroke_px);
        const int text_x = style.padding_px * 2 + text_offset_x;
        draw_text_label(image, text_x, y, "ZONE:" + zone.zone_id.value, style, zone_color, bg_color);
        y += line_advance;
        if (y > image.height - line_advance) {
            break;
        }
    }

    y = image.height - line_advance - style.padding_px;
    for (const auto& corridor : snapshot.flight_corridors) {
        draw_rect(image, style.padding_px * 2, y, indicator_size * 3, indicator_size / 2 + 1, corridor_color, style.stroke_px);
        const int text_x = style.padding_px * 2 + text_offset_x;
        draw_text_label(image, text_x, y, "COR:" + corridor.corridor_id.value, style, corridor_color, bg_color);
        y -= line_advance;
        if (y < image.height / 2) {
            break;
        }
    }

    const int landmark_col_width = text_pixel_width("LM:XXXXXXXX", style.glyph_scale) + text_offset_x + style.padding_px * 4;
    int lm_x = image.width - landmark_col_width;
    y = style.padding_px * 2 + line_h * 2 + style.line_gap_px * 2 + style.padding_px;
    for (const auto& landmark : snapshot.landmarks) {
        draw_rect(image, lm_x, y, indicator_size, indicator_size, landmark_color, style.stroke_px);
        draw_text_label(image, lm_x + text_offset_x, y, "LM:" + landmark.landmark_id.value, style, landmark_color, bg_color);
        y += line_advance;
        if (y > image.height - line_advance) {
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
    const OverlayStyle overlay_style = choose_overlay_style(annotated.width, annotated.height);

    draw_world_metadata(annotated, context, overlay_style);
    for (const auto& detection : context.perception.stabilized_frame.detections) {
        draw_detection_overlay(annotated, detection, overlay_style);
    }
    for (const auto& track : context.perception.tracks) {
        draw_track_overlay(annotated, track, overlay_style);
    }
    draw_projected_agent_overlays(annotated, context.world_snapshot, overlay_style);
    draw_world_debug_shapes(annotated, context.world_snapshot, overlay_style);

    ++frame_index_;
    const auto frame_path = std::filesystem::path{output_dir_} / frame_file_name(frame_index_);
    write_ppm(frame_path, annotated);
    write_world_overlay_sidecar(
        std::filesystem::path{output_dir_},
        frame_index_,
        context);
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
