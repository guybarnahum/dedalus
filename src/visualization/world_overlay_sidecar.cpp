#include "dedalus/visualization/world_overlay_sidecar.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include "dedalus/visualization/world_to_image_projector.hpp"

namespace dedalus {
namespace {

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
        case ClassLabel::Animal:
            return "animal";
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

std::string sidecar_file_name(const std::size_t frame_index) {
    std::ostringstream stream;
    stream << "frame_" << std::setw(6) << std::setfill('0') << frame_index << ".world_overlay.json";
    return stream.str();
}

void write_vec2_json(std::ostream& output, const Vec2& value) {
    output << '[' << value.x << ',' << value.y << ']';
}

void write_vec3_json(std::ostream& output, const Vec3& value) {
    output << '[' << value.x << ',' << value.y << ',' << value.z << ']';
}

void write_rect_json(std::ostream& output, const Rect2& value) {
    output << "{\"x\":" << value.x << ",\"y\":" << value.y << ",\"width\":" << value.width
           << ",\"height\":" << value.height << '}';
}

Vec2 bbox_center(const Rect2& bbox) {
    return Vec2{bbox.x + bbox.width * 0.5, bbox.y + bbox.height * 0.5};
}

double pixel_residual(const Vec2& source_center, const ProjectedWorldPoint& projected) {
    const double du = projected.u_px - source_center.x;
    const double dv = projected.v_px - source_center.y;
    return std::sqrt(du * du + dv * dv);
}

bool has_finite_pixel_projection(const ProjectedWorldPoint& projected) {
    return std::isfinite(projected.u_px) && std::isfinite(projected.v_px) &&
           (projected.visible || projected.reason == "outside_image");
}

void write_json_string(std::ostream& output, const std::string& value) {
    output << '"';
    for (const char ch : value) {
        switch (ch) {
            case '"':
                output << "\\\"";
                break;
            case '\\':
                output << "\\\\";
                break;
            case '\n':
                output << "\\n";
                break;
            case '\r':
                output << "\\r";
                break;
            case '\t':
                output << "\\t";
                break;
            default:
                output << ch;
                break;
        }
    }
    output << '"';
}

WorldToImageProjectionConfig projection_config_for(const ImageView& image, const double horizontal_fov_deg) {
    WorldToImageProjectionConfig config;
    config.intrinsics = pinhole_intrinsics_from_horizontal_fov(image.width, image.height, horizontal_fov_deg);
    config.extrinsics.body_T_camera.position = Vec3{0.0, 0.0, 0.0};
    config.extrinsics.body_T_camera.rotation_rpy = Vec3{0.0, 0.0, 0.0};
    return config;
}

}  // namespace

void write_world_overlay_sidecar(
    const std::filesystem::path& output_dir,
    const std::size_t frame_index,
    const AnnotationContext& context,
    const double horizontal_fov_deg) {
    if (context.frame.image.width <= 0 || context.frame.image.height <= 0) {
        throw std::runtime_error("world overlay sidecar requires a valid image size");
    }

    std::filesystem::create_directories(output_dir);
    const auto config = projection_config_for(context.frame.image, horizontal_fov_deg);
    const auto sidecar_path = output_dir / sidecar_file_name(frame_index);

    std::ofstream output{sidecar_path};
    if (!output) {
        throw std::runtime_error("failed to open world overlay sidecar: " + sidecar_path.string());
    }

    output << std::fixed << std::setprecision(6);
    output << "{\n";
    output << "  \"frame_index\": " << frame_index << ",\n";
    output << "  \"frame_id\": ";
    write_json_string(output, context.frame.frame_id.value);
    output << ",\n";
    output << "  \"timestamp_ns\": " << context.frame.timestamp.timestamp_ns << ",\n";
    output << "  \"coordinate_products\": {\n";
    output << "    \"viewport\": \"camera pixel coordinates\",\n";
    output << "    \"ego_relative_3d\": \"current drone/body-relative tactical coordinates\",\n";
    output << "    \"map_3d\": \"floating or anchored local/global Cartesian map coordinates\"\n";
    output << "  },\n";
    output << "  \"camera_model\": {\n";
    output << "    \"width\": " << config.intrinsics.width << ",\n";
    output << "    \"height\": " << config.intrinsics.height << ",\n";
    output << "    \"horizontal_fov_deg\": " << horizontal_fov_deg << ",\n";
    output << "    \"fx\": " << config.intrinsics.fx << ",\n";
    output << "    \"fy\": " << config.intrinsics.fy << ",\n";
    output << "    \"cx\": " << config.intrinsics.cx << ",\n";
    output << "    \"cy\": " << config.intrinsics.cy << ",\n";
    output << "    \"near_plane_m\": " << config.intrinsics.near_plane_m << "\n";
    output << "  },\n";
    output << "  \"ego\": {\n";
    output << "    \"map_frame_id\": ";
    write_json_string(output, context.world_snapshot.ego.map_frame_id.value);
    output << ",\n";
    output << "    \"position_local\": ";
    write_vec3_json(output, context.world_snapshot.ego.local_T_body.position);
    output << ",\n";
    output << "    \"rotation_rpy\": ";
    write_vec3_json(output, context.world_snapshot.ego.local_T_body.rotation_rpy);
    output << "\n";
    output << "  },\n";
    output << "  \"agents\": [\n";

    for (std::size_t i = 0; i < context.world_snapshot.agents.size(); ++i) {
        const auto& agent = context.world_snapshot.agents[i];
        const auto projected = project_local_point_to_image(agent.position_local, context.world_snapshot.ego, config);
        output << "    {\n";
        output << "      \"agent_id\": ";
        write_json_string(output, agent.agent_id.value);
        output << ",\n";
        output << "      \"source_track_id\": ";
        write_json_string(output, agent.source_track_id.value);
        output << ",\n";
        output << "      \"identity_id\": ";
        write_json_string(output, agent.identity_id.value);
        output << ",\n";
        if (agent.has_source_detection) {
            output << "      \"source_detection_id\": ";
            write_json_string(output, agent.source_detection_id.value);
            output << ",\n";
        }
        if (agent.has_source_frame) {
            output << "      \"source_frame_id\": ";
            write_json_string(output, agent.source_frame_id.value);
            output << ",\n";
        }
        output << "      \"class\": ";
        write_json_string(output, class_label_to_string(agent.class_label));
        output << ",\n";
        output << "      \"confidence\": " << agent.confidence << ",\n";
        output << "      \"position_local\": ";
        write_vec3_json(output, agent.position_local);
        output << ",\n";
        output << "      \"position_ego_relative\": ";
        write_vec3_json(output, projected.position_ego_relative);
        output << ",\n";
        output << "      \"position_body\": ";
        write_vec3_json(output, projected.position_body);
        output << ",\n";
        output << "      \"position_camera\": ";
        write_vec3_json(output, projected.position_camera);
        output << ",\n";
        output << "      \"range_m\": " << projected.range_m << ",\n";
        output << "      \"bearing_deg\": " << projected.bearing_deg << ",\n";
        output << "      \"visible\": " << (projected.visible ? "true" : "false") << ",\n";
        output << "      \"u_px\": " << projected.u_px << ",\n";
        output << "      \"v_px\": " << projected.v_px << ",\n";
        output << "      \"depth_m\": " << projected.depth_m << ",\n";
        if (agent.has_source_bbox) {
            const auto source_center = bbox_center(agent.source_bbox_px);
            output << "      \"source_bbox_px\": ";
            write_rect_json(output, agent.source_bbox_px);
            output << ",\n";
            output << "      \"source_center_px\": ";
            write_vec2_json(output, source_center);
            output << ",\n";
            output << "      \"reprojected_center_px\": [" << projected.u_px << ',' << projected.v_px << "],\n";
            if (has_finite_pixel_projection(projected)) {
                output << "      \"residual_px\": " << pixel_residual(source_center, projected) << ",\n";
            }
        }
        output << "      \"reason\": ";
        write_json_string(output, projected.reason);
        output << "\n";
        output << "    }" << (i + 1U < context.world_snapshot.agents.size() ? "," : "") << "\n";
    }

    output << "  ]\n";
    output << "}\n";
}

}  // namespace dedalus
