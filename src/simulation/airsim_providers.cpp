#include "dedalus/simulation/airsim_providers.hpp"
#include "dedalus/simulation/airsim_sidecar_parser.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dedalus {
namespace {

std::runtime_error unavailable(const char* provider_name) {
    return std::runtime_error(
        std::string{provider_name} +
        " is an integration provider and is not available in the dependency-free core build");
}

std::unique_ptr<BridgeTransport> make_transport(const std::string& transport_name) {
    if (transport_name == "pipe") {
        return std::make_unique<PipeBridgeTransport>();
    }
    if (transport_name == "shared_memory") {
        throw std::runtime_error(
            "shared_memory bridge transport is not yet implemented; use bridge_transport: pipe");
    }
    throw std::runtime_error("unknown AirSim bridge transport: " + transport_name);
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

    input.get();

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

int b64_value(char ch) {
    if (ch >= 'A' && ch <= 'Z') {
        return ch - 'A';
    }
    if (ch >= 'a' && ch <= 'z') {
        return ch - 'a' + 26;
    }
    if (ch >= '0' && ch <= '9') {
        return ch - '0' + 52;
    }
    if (ch == '+') {
        return 62;
    }
    if (ch == '/') {
        return 63;
    }
    if (ch == '=') {
        return -2;
    }
    if (std::isspace(static_cast<unsigned char>(ch)) != 0) {
        return -3;
    }
    return -1;
}

std::string base64_decode(const std::string& encoded) {
    std::string output;
    int value = 0;
    int bits = -8;

    for (const char ch : encoded) {
        const int decoded = b64_value(ch);
        if (decoded == -3) {
            continue;
        }
        if (decoded == -2) {
            break;
        }
        if (decoded < 0) {
            throw std::runtime_error("invalid base64 character in AirSim stream frame");
        }

        value = (value << 6) + decoded;
        bits += 6;
        if (bits >= 0) {
            output.push_back(static_cast<char>((value >> bits) & 0xFF));
            bits -= 8;
        }
    }

    return output;
}

std::string parse_json_string(const std::string& json, const std::string& key) {
    const std::string marker = "\"" + key + "\":";
    const auto marker_pos = json.find(marker);
    if (marker_pos == std::string::npos) {
        throw std::runtime_error("AirSim stream JSON missing key: " + key);
    }
    const auto open_pos = json.find('"', marker_pos + marker.size());
    if (open_pos == std::string::npos) {
        throw std::runtime_error("AirSim stream JSON has invalid string for key: " + key);
    }
    const auto close_pos = json.find('"', open_pos + 1U);
    if (close_pos == std::string::npos) {
        throw std::runtime_error("AirSim stream JSON has unterminated string for key: " + key);
    }
    return json.substr(open_pos + 1U, close_pos - open_pos - 1U);
}

Nanoseconds parse_json_i64(const std::string& json, const std::string& key) {
    const std::string marker = "\"" + key + "\":";
    const auto marker_pos = json.find(marker);
    if (marker_pos == std::string::npos) {
        throw std::runtime_error("AirSim bridge JSON missing key: " + key);
    }
    const auto value_start = marker_pos + marker.size();
    const auto value_end = json.find_first_of(",}\n\r\t ", value_start);
    const auto token = json.substr(value_start, value_end == std::string::npos ? std::string::npos : value_end - value_start);
    return static_cast<Nanoseconds>(std::stoll(token));
}

FramePacket frame_from_image(const AirSimProviderConfig& config, ImageView image, FrameId frame_id, TimePoint timestamp) {
    FramePacket frame;
    frame.frame_id = std::move(frame_id);
    frame.timestamp = timestamp;
    frame.camera_id = CameraId{config.camera_name};
    frame.image = std::move(image);
    // Compute focal lengths from configured FOV when available; fall back to legacy constant.
    if (config.camera_hfov_rad > 0.0 && config.camera_vfov_rad > 0.0) {
        frame.intrinsics.fx = (frame.image.width  * 0.5) / std::tan(config.camera_hfov_rad * 0.5);
        frame.intrinsics.fy = (frame.image.height * 0.5) / std::tan(config.camera_vfov_rad * 0.5);
    } else {
        frame.intrinsics.fx = 420.0;
        frame.intrinsics.fy = 420.0;
    }
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

// ── AirSimGroundTruthDetector helpers ─────────────────────────────────────

struct GtObjectPose {
    std::string name;
    bool pose_available{false};
    Vec3 position_ned_m;
};

// Parse the compact JSON emitted by airsim-object-poses.py (one-shot mode).
// Format: {"objects":[{"name":"X","pose_available":true,"position_ned_m":[x,y,z],...},...]}
std::vector<GtObjectPose> parse_gt_object_poses(const std::string& json) {
    std::vector<GtObjectPose> result;
    std::size_t cursor = 0U;
    while (true) {
        const auto name_marker = json.find("\"name\":", cursor);
        if (name_marker == std::string::npos) break;
        // Find the enclosing object.
        const auto obj_start = json.rfind('{', name_marker);
        if (obj_start == std::string::npos) break;
        std::size_t depth = 0U;
        std::size_t obj_end = obj_start;
        for (std::size_t i = obj_start; i < json.size(); ++i) {
            if (json[i] == '{') ++depth;
            else if (json[i] == '}') { --depth; if (depth == 0U) { obj_end = i; break; } }
        }
        const auto obj = json.substr(obj_start, obj_end - obj_start + 1U);

        GtObjectPose pose;
        // name
        const auto nq1 = obj.find('"', obj.find("\"name\":") + 7U);
        const auto nq2 = nq1 != std::string::npos ? obj.find('"', nq1 + 1U) : std::string::npos;
        if (nq1 != std::string::npos && nq2 != std::string::npos)
            pose.name = obj.substr(nq1 + 1U, nq2 - nq1 - 1U);
        // pose_available
        const auto pa_marker = obj.find("\"pose_available\":");
        if (pa_marker != std::string::npos) {
            const auto pa_val = obj.find_first_not_of(" \t", pa_marker + 17U);
            pose.pose_available = (pa_val != std::string::npos && obj.compare(pa_val, 4U, "true") == 0);
        }
        // position_ned_m
        if (pose.pose_available) {
            const auto arr_marker = obj.find("\"position_ned_m\":");
            if (arr_marker != std::string::npos) {
                const auto open = obj.find('[', arr_marker);
                const auto close = obj.find(']', open);
                if (open != std::string::npos && close != std::string::npos) {
                    std::string body = obj.substr(open + 1U, close - open - 1U);
                    std::replace(body.begin(), body.end(), ',', ' ');
                    std::istringstream iss{body};
                    double x = 0.0, y = 0.0, z = 0.0;
                    iss >> x >> y >> z;
                    pose.position_ned_m = Vec3{x, y, z};
                }
            }
        }
        if (!pose.name.empty()) result.push_back(std::move(pose));
        cursor = obj_end + 1U;
    }
    return result;
}

// Transform a NED-local relative vector into OpenCV camera frame.
// Drone body frame: x=forward, y=right, z=down (NED body).
// Camera (front-center): x=right, y=down, z=forward.
// RPY: {roll, pitch, yaw} in radians (aerospace ZYX convention).
Vec3 ned_local_to_camera(const Vec3& p_rel, const Vec3& rpy) {
    const double cr = std::cos(rpy.x);
    const double sr = std::sin(rpy.x);
    const double cp = std::cos(rpy.y);
    const double sp = std::sin(rpy.y);
    const double cy = std::cos(rpy.z);
    const double sy = std::sin(rpy.z);

    // R_body_from_local = (Rz(yaw)*Ry(pitch)*Rx(roll))^T
    const double bx =  cy*cp*p_rel.x + sy*cp*p_rel.y - sp*p_rel.z;
    const double by =  (cy*sp*sr - sy*cr)*p_rel.x + (sy*sp*sr + cy*cr)*p_rel.y + cp*sr*p_rel.z;
    const double bz =  (cy*sp*cr + sy*sr)*p_rel.x + (sy*sp*cr - cy*sr)*p_rel.y + cp*cr*p_rel.z;

    // Front-center camera: cam_x=body_y(right), cam_y=body_z(down), cam_z=body_x(forward)
    return Vec3{by, bz, bx};
}

}  // namespace

AirSimFrameSource::AirSimFrameSource(AirSimProviderConfig config)
    : config_(std::move(config)), transport_(make_transport(config_.transport)) {}

AirSimFrameSource::~AirSimFrameSource() = default;

FramePacket AirSimFrameSource::next_one_shot_frame() {
    const auto command = build_bridge_command(config_, config_.bridge_command);
    return frame_from_image(
        config_,
        parse_ppm_bytes(transport_->request_once(command)),
        FrameId{"airsim_live_frame_" + std::to_string(++next_frame_index_)},
        TimePoint{0});
}

std::optional<FramePacket> AirSimFrameSource::next_stream_jsonl_frame() {
    const auto command = build_bridge_command(config_, config_.bridge_command);
    const auto json_line = transport_->read_stream_line(command);
    if (!json_line.has_value()) {
        return std::nullopt;
    }

    const auto frame_id = parse_json_string(*json_line, "frame_id");
    const auto timestamp = parse_json_i64(*json_line, "timestamp_ns");
    const auto ppm_b64 = parse_json_string(*json_line, "ppm_b64");

    ++next_frame_index_;
    return frame_from_image(
        config_,
        parse_ppm_bytes(base64_decode(ppm_b64)),
        FrameId{frame_id},
        TimePoint{timestamp});
}

std::optional<FramePacket> AirSimFrameSource::next_frame() {
    if (config_.bridge_mode == "stream_binary" || config_.bridge_mode == "stream_binary_ego") {
        return next_stream_binary_frame();
    }
    if (config_.bridge_mode == "stream_jsonl") {
        return next_stream_jsonl_frame();
    }
    if (config_.bridge_mode == "one_shot_ppm") {
        return next_one_shot_frame();
    }
    throw std::runtime_error("unknown AirSim bridge mode: " + config_.bridge_mode);
}

AirSimEgoStateProvider::AirSimEgoStateProvider(AirSimProviderConfig config)
    : config_(std::move(config)), transport_(make_transport(config_.transport)) {}

EgoStateEstimate AirSimEgoStateProvider::estimate(const FramePacket& frame) {
    const auto command = build_bridge_command(config_, config_.ego_bridge_command);

    EgoStateEstimate estimate;
    estimate.ego = parse_ego_json(transport_->request_once(command), config_.map_frame_id, frame.timestamp);
    estimate.telemetry_available = true;
    estimate.confidence = 0.85F;
    return estimate;
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
    : config_(std::move(config)), transport_(std::make_unique<PipeBridgeTransport>()) {
    // Pre-build the streaming command once so detect() pays no formatting cost per frame.
    if (!config_.detector_objects.empty()) {
        validate_bridge_base_command(config_.objects_bridge_command);
        std::ostringstream cmd;
        cmd << config_.objects_bridge_command
            << " --host " << shell_quote(config_.host)
            << " --rpc-port " << config_.rpc_port
            << " --stream-jsonl"
            << " --no-fail-on-missing";
        for (const auto& obj : config_.detector_objects) {
            cmd << " --object " << shell_quote(obj.airsim_object_name);
        }
        stream_command_ = cmd.str();
    }
}

AirSimGroundTruthDetector::~AirSimGroundTruthDetector() = default;

std::vector<Detection2D> AirSimGroundTruthDetector::detect(const FramePacket& frame) {
    if (stream_command_.empty() || !frame.ego_hint.has_value()) {
        return {};
    }

    // Read one pose snapshot from the persistent streaming process.
    // PipeBridgeTransport keeps the process alive across calls as long as
    // the command string is identical — no per-frame spawn overhead.
    const auto json_opt = transport_->read_stream_line(stream_command_);
    if (!json_opt.has_value()) return {};
    const auto poses = parse_gt_object_poses(*json_opt);

    const auto& ego = *frame.ego_hint;
    const double fx = frame.intrinsics.fx;
    const double fy = frame.intrinsics.fy;
    const double cx = frame.intrinsics.cx;
    const double cy = frame.intrinsics.cy;
    const auto img_w = static_cast<double>(frame.image.width);
    const auto img_h = static_cast<double>(frame.image.height);

    std::vector<Detection2D> detections;
    int det_index = 0;
    for (const auto& obj : config_.detector_objects) {
        const auto it = std::find_if(poses.begin(), poses.end(),
            [&](const GtObjectPose& p) { return p.name == obj.airsim_object_name; });
        if (it == poses.end() || !it->pose_available) continue;

        // NED relative vector: object minus drone, in local frame.
        const Vec3 p_rel{
            it->position_ned_m.x - ego.local_T_body.position.x,
            it->position_ned_m.y - ego.local_T_body.position.y,
            it->position_ned_m.z - ego.local_T_body.position.z};

        // Transform to OpenCV camera frame.
        const Vec3 p_cam = ned_local_to_camera(p_rel, ego.local_T_body.rotation_rpy);
        if (p_cam.z <= 0.5) continue;  // behind camera or too close

        const double u = fx * p_cam.x / p_cam.z + cx;
        const double v = fy * p_cam.y / p_cam.z + cy;

        // Project object half-extents to pixel half-widths/heights.
        // size_m.y = lateral (East) width, size_m.z = vertical (Down) height.
        const double half_w_px = fx * (obj.size_m.y * 0.5) / p_cam.z;
        const double half_h_px = fy * (obj.size_m.z * 0.5) / p_cam.z;

        const double x0 = u - half_w_px;
        const double y0 = v - half_h_px;
        const double x1 = u + half_w_px;
        const double y1 = v + half_h_px;

        // Skip if entirely outside the image.
        if (x1 <= 0.0 || y1 <= 0.0 || x0 >= img_w || y0 >= img_h) continue;

        Detection2D det;
        det.detection_id = DetectionId{"airsim_gt_" + std::to_string(det_index++)};
        det.frame_id     = frame.frame_id;
        det.timestamp    = frame.timestamp;
        det.bbox_px      = Rect2{
            std::max(0.0, x0),
            std::max(0.0, y0),
            std::min(x1, img_w) - std::max(0.0, x0),
            std::min(y1, img_h) - std::max(0.0, y0)};
        det.confidence   = static_cast<float>(obj.confidence);
        det.class_label  = obj.class_label;
        det.depth_m      = p_cam.z;  // exact depth carried to FlatGroundProjector
        detections.push_back(std::move(det));
    }
    return detections;
}

}  // namespace dedalus
