#include "dedalus/visualization/frame_annotator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "dedalus/visualization/world_overlay_sidecar.hpp"
#include "dedalus/visualization/world_to_image_projector.hpp"

namespace dedalus {
namespace {

struct RgbColor {
    std::uint8_t r{255U};
    std::uint8_t g{255U};
    std::uint8_t b{255U};
};

std::string class_label_to_string(const ClassLabel label) {
    switch (label) {
        case ClassLabel::Person: return "person";
        case ClassLabel::Drone: return "drone";
        case ClassLabel::Car: return "car";
        case ClassLabel::Boat: return "boat";
        case ClassLabel::Animal: return "animal";
        case ClassLabel::House: return "house";
        case ClassLabel::Building: return "building";
        case ClassLabel::Tree: return "tree";
        case ClassLabel::Road: return "road";
        case ClassLabel::River: return "river";
        case ClassLabel::Terrain: return "terrain";
        case ClassLabel::Unknown:
        default: return "unknown";
    }
}

std::string occupancy_state_to_string(const OccupancyCellState state) {
    switch (state) {
        case OccupancyCellState::Free: return "free";
        case OccupancyCellState::Occupied: return "occupied";
        case OccupancyCellState::Unknown:
        default: return "unknown";
    }
}

std::string occupancy_source_to_string(const OccupancySourceKind source) {
    switch (source) {
        case OccupancySourceKind::AirSimGroundTruth: return "airsim_gt";
        case OccupancySourceKind::VisualObstacleDetector: return "visual_det";
        case OccupancySourceKind::DepthProvider: return "depth";
        case OccupancySourceKind::Fused: return "fused";
        case OccupancySourceKind::SyntheticFixture:
        default: return "synthetic";
    }
}

int clamp_int(const int value, const int lo, const int hi) {
    return std::max(lo, std::min(value, hi));
}

int rounded_int(const double value) {
    return static_cast<int>(std::lround(value));
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

void draw_label_bar(ImageView& image, int x, int y, const std::string& text, const RgbColor fg, const RgbColor bg) {
    const int safe_x = clamp_int(x, 0, std::max(0, image.width - 1));
    const int safe_y = clamp_int(y, 0, std::max(0, image.height - 1));
    const int char_w = 4;
    const int height = 9;
    const int width = std::min(std::max(16, static_cast<int>(text.size()) * char_w + 4), std::max(1, image.width - safe_x));
    fill_rect(image, safe_x, safe_y, width, height, bg);
    for (std::size_t i = 0; i < text.size(); ++i) {
        const int cx = safe_x + 2 + static_cast<int>(i) * char_w;
        if (cx + 2 >= image.width) {
            break;
        }
        const auto code = static_cast<unsigned char>(text[i]);
        const int h = 2 + static_cast<int>(code % 6U);
        fill_rect(image, cx, safe_y + 1 + (6 - h), 2, h, fg);
    }
}

std::string frame_file_name(const std::size_t frame_index) {
    std::ostringstream stream;
    stream << "frame_" << std::setw(6) << std::setfill('0') << frame_index << ".ppm";
    return stream.str();
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
    output.write(reinterpret_cast<const char*>(image.bytes.data()), static_cast<std::streamsize>(image.bytes.size()));
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
    manifest << frame_index << ',' << context.frame.frame_id.value << ','
             << context.frame.timestamp.timestamp_ns << ',' << frame_path.string() << ','
             << output_fps << '\n';
}

WorldToImageProjectionConfig projection_config_for(const ImageView& image) {
    WorldToImageProjectionConfig projection_config;
    projection_config.intrinsics = pinhole_intrinsics_from_horizontal_fov(image.width, image.height, 90.0);
    projection_config.extrinsics.body_T_camera.position = Vec3{0.0, 0.0, 0.0};
    projection_config.extrinsics.body_T_camera.rotation_rpy = Vec3{0.0, 0.0, 0.0};
    return projection_config;
}

void draw_detection_overlay(ImageView& image, const Detection2D& detection) {
    const RgbColor detection_color{255U, 210U, 0U};
    draw_rect(
        image,
        rounded_int(detection.bbox_px.x),
        rounded_int(detection.bbox_px.y),
        rounded_int(detection.bbox_px.width),
        rounded_int(detection.bbox_px.height),
        detection_color,
        2);
    draw_label_bar(image, rounded_int(detection.bbox_px.x), rounded_int(detection.bbox_px.y) - 12, "D:" + class_label_to_string(detection.class_label), detection_color, RgbColor{0U, 0U, 0U});
}

void draw_track_overlay(ImageView& image, const Track2D& track) {
    const RgbColor track_color{0U, 255U, 180U};
    draw_rect(
        image,
        rounded_int(track.bbox_px.x) - 2,
        rounded_int(track.bbox_px.y) - 2,
        rounded_int(track.bbox_px.width) + 4,
        rounded_int(track.bbox_px.height) + 4,
        track_color,
        2);
}

void draw_world_metadata(ImageView& image, const AnnotationContext& context) {
    draw_label_bar(image, 4, 4, "FRAME:" + context.frame.frame_id.value + " MAP:" + context.world_snapshot.active_map_frame_id.value, RgbColor{255U, 255U, 255U}, RgbColor{0U, 0U, 0U});
    draw_label_bar(image, 4, 16, "EGO MAP:" + context.world_snapshot.ego.map_frame_id.value + " TS:" + std::to_string(context.frame.timestamp.timestamp_ns), RgbColor{255U, 255U, 255U}, RgbColor{0U, 0U, 0U});
}

void draw_projected_agent_overlays(ImageView& image, const WorldSnapshot& snapshot) {
    const auto projection_config = projection_config_for(image);
    const RgbColor agent_color{80U, 255U, 120U};
    for (const auto& agent : snapshot.agents) {
        const auto projected = project_local_point_to_image(agent.position_local, snapshot.ego, projection_config);
        if (!projected.visible) {
            continue;
        }
        const int u = clamp_int(rounded_int(projected.u_px), 0, image.width - 1);
        const int v = clamp_int(rounded_int(projected.v_px), 0, image.height - 1);
        fill_rect(image, u - 8, v - 1, 17, 3, agent_color);
        fill_rect(image, u - 1, v - 8, 3, 17, agent_color);
        draw_rect(image, u - 7, v - 7, 15, 15, agent_color, 1);
    }
}

RgbColor occupancy_color(const OccupancyCellState state) {
    switch (state) {
        case OccupancyCellState::Occupied: return RgbColor{255U, 60U, 20U};
        case OccupancyCellState::Free: return RgbColor{80U, 190U, 255U};
        case OccupancyCellState::Unknown:
        default: return RgbColor{255U, 210U, 40U};
    }
}

void draw_occupancy_overlay(ImageView& image, const WorldSnapshot& snapshot) {
    if (!snapshot.has_ego_occupancy || !snapshot.ego_occupancy.has_valid_occupancy) {
        return;
    }

    const auto& occupancy = snapshot.ego_occupancy;
    std::ostringstream summary;
    summary << "OCC src=" << occupancy_source_to_string(occupancy.source_kind)
            << " occ=" << occupancy.occupied_count
            << " free=" << occupancy.free_count
            << " unk=" << occupancy.unknown_count
            << " clear=" << std::fixed << std::setprecision(1) << occupancy.forward_corridor_clearance_m;
    draw_label_bar(image, 4, 28, summary.str(), RgbColor{255U, 210U, 40U}, RgbColor{0U, 0U, 0U});

    const auto projection_config = projection_config_for(image);
    int drawn = 0;
    for (const auto& cell : occupancy.debug_cells) {
        if (drawn >= 80) {
            break;
        }
        const auto projected = project_local_point_to_image(cell.center_local, snapshot.ego, projection_config);
        if (!projected.visible) {
            continue;
        }
        const int u = clamp_int(rounded_int(projected.u_px), 0, image.width - 1);
        const int v = clamp_int(rounded_int(projected.v_px), 0, image.height - 1);
        const auto color = occupancy_color(cell.state);
        const int radius = cell.state == OccupancyCellState::Occupied ? 6 : 4;
        fill_rect(image, u - radius, v - radius, radius * 2 + 1, radius * 2 + 1, color);
        if (cell.state == OccupancyCellState::Occupied) {
            draw_label_bar(image, u + radius + 2, v - radius, "OCC:" + occupancy_state_to_string(cell.state), color, RgbColor{0U, 0U, 0U});
        }
        ++drawn;
    }
}

void draw_world_debug_shapes(ImageView& image, const WorldSnapshot& snapshot) {
    int y = 44;
    const RgbColor zone_color{255U, 80U, 80U};
    const RgbColor corridor_color{90U, 160U, 255U};
    for (const auto& zone : snapshot.tactical_exclusion_zones) {
        draw_label_bar(image, 4, y, "ZONE:" + zone.zone_id.value, zone_color, RgbColor{0U, 0U, 0U});
        y += 12;
        if (y > image.height - 24) {
            break;
        }
    }
    y = image.height - 14;
    for (const auto& corridor : snapshot.flight_corridors) {
        draw_label_bar(image, 4, y, "COR:" + corridor.corridor_id.value, corridor_color, RgbColor{0U, 0U, 0U});
        y -= 12;
        if (y < image.height / 2) {
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
    draw_projected_agent_overlays(annotated, context.world_snapshot);
    draw_occupancy_overlay(annotated, context.world_snapshot);
    draw_world_debug_shapes(annotated, context.world_snapshot);

    ++frame_index_;
    const auto frame_path = std::filesystem::path{output_dir_} / frame_file_name(frame_index_);
    write_ppm(frame_path, annotated);
    write_world_overlay_sidecar(std::filesystem::path{output_dir_}, frame_index_, context);
    append_manifest_row(std::filesystem::path{output_dir_} / "manifest.txt", frame_index_, context, frame_path, output_fps_);
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
