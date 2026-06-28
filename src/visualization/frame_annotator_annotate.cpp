#include "dedalus/visualization/frame_annotator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

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
        case OccupancySourceKind::AirSimGroundTruth: return "airsim_gt_detector";
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

void draw_line(ImageView& image, int x0, int y0, int x1, int y1, const RgbColor color) {
    const int dx = std::abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        set_pixel(image, x0, y0, color);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
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

const char* obstacle_shape_to_string(const ObstacleEvidenceShape shape) {
    switch (shape) {
        case ObstacleEvidenceShape::SurfacePatch: return "surf";
        case ObstacleEvidenceShape::Voxel: return "vox";
        case ObstacleEvidenceShape::FrustumBin: return "frustum";
        case ObstacleEvidenceShape::RaySegment: return "ray";
        case ObstacleEvidenceShape::LineSegment: return "line";
        case ObstacleEvidenceShape::Capsule: return "cap";
        default: return "obs";
    }
}

// Returns true for SurfacePatch evidence from any depth provider (GT or VD).
bool is_depth_surface_patch(const ObstacleEvidence& evidence) {
    return evidence.shape == ObstacleEvidenceShape::SurfacePatch &&
           (evidence.source_provider == "airsim_depth_obstacle_detector" ||
            evidence.source_provider == "visual_depth_obstacle_detector");
}

bool is_visual_depth_evidence(const ObstacleEvidence& evidence) {
    return evidence.source_provider == "visual_depth_obstacle_detector";
}

// Color scheme:
//   Magenta diamond  — SurfacePatch (any depth provider)
//   Red              — ThinStructureRisk
//   Cyan             — visual_depth_obstacle_detector voxels
//   Orange           — airsim_depth_obstacle_detector voxels / other
RgbColor obstacle_evidence_color(const ObstacleEvidence& evidence) {
    if (is_depth_surface_patch(evidence)) {
        return RgbColor{255U, 0U, 255U};    // magenta
    }
    if (evidence.state == ObstacleEvidenceState::ThinStructureRisk) {
        return RgbColor{255U, 70U, 70U};    // red
    }
    if (is_visual_depth_evidence(evidence)) {
        return RgbColor{0U, 210U, 255U};    // cyan — VD voxels
    }
    return RgbColor{255U, 150U, 60U};       // orange — GT voxels / other
}

void draw_surface_patch_marker(
    ImageView& image,
    const int u,
    const int v,
    const int radius,
    const RgbColor color) {
    const int r = std::max(6, radius);

    // Bold diamond footprint.
    draw_line(image, u, v - r, u + r, v, color);
    draw_line(image, u + r, v, u, v + r, color);
    draw_line(image, u, v + r, u - r, v, color);
    draw_line(image, u - r, v, u, v - r, color);

    // Internal X makes it visually different from object boxes and occupancy squares.
    draw_line(image, u - r / 2, v - r / 2, u + r / 2, v + r / 2, color);
    draw_line(image, u - r / 2, v + r / 2, u + r / 2, v - r / 2, color);

    // Center anchor.
    fill_rect(image, u - 2, v - 2, 5, 5, color);
}

void draw_obstacle_evidence_overlay(ImageView& image, const WorldSnapshot& snapshot) {
    if (snapshot.obstacle_evidence.empty()) {
        return;
    }

    const auto projection_config = projection_config_for(image);
    constexpr int kMaxDrawn = 140;
    int drawn = 0;
    int depth_surface_count = 0;
    int other_count = 0;

    for (const auto& evidence : snapshot.obstacle_evidence) {
        if (is_depth_surface_patch(evidence)) {
            ++depth_surface_count;
        } else {
            ++other_count;
        }

        if (drawn >= kMaxDrawn) {
            continue;
        }

        const auto projected = project_local_point_to_image(evidence.center_local, snapshot.ego, projection_config);
        if (!projected.visible) {
            continue;
        }

        const int u = clamp_int(rounded_int(projected.u_px), 0, image.width - 1);
        const int v = clamp_int(rounded_int(projected.v_px), 0, image.height - 1);
        const auto color = obstacle_evidence_color(evidence);

        if (evidence.shape == ObstacleEvidenceShape::SurfacePatch) {
            int radius = is_depth_surface_patch(evidence) ? 10 : 6;
            if (evidence.range_m > 0.1F) {
                const float patch_m = std::max(evidence.size_m.x, evidence.size_m.y);
                radius = clamp_int(
                    static_cast<int>(std::round(120.0F * patch_m / std::max(2.0F, evidence.range_m))),
                    is_depth_surface_patch(evidence) ? 7 : 5,
                    is_depth_surface_patch(evidence) ? 22 : 12);
            }
            draw_surface_patch_marker(image, u, v, radius, color);
            if (evidence.has_surface_normal && drawn < 60) {
                Vec3 normal_tip = evidence.center_local;
                normal_tip.x += evidence.surface_normal_local.x * 0.75;
                normal_tip.y += evidence.surface_normal_local.y * 0.75;
                normal_tip.z += evidence.surface_normal_local.z * 0.75;
                const auto projected_tip = project_local_point_to_image(normal_tip, snapshot.ego, projection_config);
                if (projected_tip.visible) {
                    draw_line(
                        image,
                        u,
                        v,
                        clamp_int(rounded_int(projected_tip.u_px), 0, image.width - 1),
                        clamp_int(rounded_int(projected_tip.v_px), 0, image.height - 1),
                        color);
                }
            }
        } else {
            fill_rect(image, u - 3, v - 3, 7, 7, color);
        }

        if (drawn < 8) {
            draw_label_bar(
                image,
                u + 7,
                v - 6,
                std::string{"OBS:"} + obstacle_shape_to_string(evidence.shape),
                color,
                RgbColor{0U, 0U, 0U});
        }

        ++drawn;
    }

    if (depth_surface_count > 0 || other_count > 0) {
        std::ostringstream summary;
        summary << "OBS depth_surf=" << depth_surface_count << " other=" << other_count;
        if (static_cast<int>(snapshot.obstacle_evidence.size()) > kMaxDrawn) {
            summary << " drawn=" << kMaxDrawn;
        }
        draw_label_bar(image, 4, 52, summary.str(), RgbColor{255U, 0U, 255U}, RgbColor{0U, 0U, 0U});
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

void annotate_frame_overlays(ImageView& image, const AnnotationContext& context) {
    draw_world_metadata(image, context);
    for (const auto& detection : context.perception.stabilized_frame.detections) {
        draw_detection_overlay(image, detection);
    }
    for (const auto& track : context.perception.tracks) {
        draw_track_overlay(image, track);
    }
    draw_projected_agent_overlays(image, context.world_snapshot);
    draw_occupancy_overlay(image, context.world_snapshot);
    draw_world_debug_shapes(image, context.world_snapshot);
    draw_obstacle_evidence_overlay(image, context.world_snapshot);
}

}  // namespace dedalus
