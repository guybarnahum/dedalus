#include "dedalus/perception/perception_pipeline.hpp"

#include <stdexcept>
#include <string>

#include "dedalus/geometry/pose_transform.hpp"

namespace dedalus {

PerceptionPipeline::PerceptionPipeline(
    std::shared_ptr<Detector> detector,
    std::shared_ptr<CameraStabilizer> stabilizer,
    std::shared_ptr<Tracker> tracker,
    std::shared_ptr<IdentityResolver> identity_resolver,
    std::shared_ptr<Projector3D> projector)
    : detector_(std::move(detector)),
      stabilizer_(std::move(stabilizer)),
      tracker_(std::move(tracker)),
      identity_resolver_(std::move(identity_resolver)),
      projector_(std::move(projector)) {}

PerceptionPipelineOutput PerceptionPipeline::process(const FramePacket& frame, const EgoState& ego) {
    PerceptionPipelineOutput output;
    output.detections = detector_->detect(frame);
    output.stabilized_frame = stabilizer_->stabilize(frame, output.detections);
    output.tracks = tracker_->update(output.stabilized_frame.detections);
    output.identities = identity_resolver_->resolve(output.tracks);
    output.observations = projector_->project(output.tracks, output.stabilized_frame.frame, ego);
    return output;
}

std::vector<Detection2D> ScriptedDetector::detect(const FramePacket& frame) {
    Detection2D person;
    person.detection_id = DetectionId{"det_0001"};
    person.frame_id = frame.frame_id;
    person.timestamp = frame.timestamp;
    person.bbox_px = Rect2{260.0, 160.0, 80.0, 180.0};
    person.confidence = 0.88F;
    person.class_label = ClassLabel::Person;
    person.faction = FactionLabel::Unknown;
    person.appearance = FeatureVector{0.1F, 0.4F, 0.8F};
    return {person};
}

StabilizedFrame NullCameraStabilizer::stabilize(
    const FramePacket& frame,
    const std::vector<Detection2D>& detections) {
    StabilizedFrame stabilized;
    stabilized.frame = frame;
    stabilized.detections = detections;
    stabilized.transform_available = false;
    stabilized.confidence = 0.0;
    return stabilized;
}

std::vector<Track2D> SimpleCentroidTracker::update(const std::vector<Detection2D>& detections) {
    std::vector<Track2D> tracks;
    tracks.reserve(detections.size());

    for (std::size_t i = 0; i < detections.size(); ++i) {
        const auto& detection = detections[i];
        Track2D track;
        track.track_id = TrackId{"track_" + std::string(4 - std::to_string(i + 1).size(), '0') + std::to_string(i + 1)};
        track.source_detection_id = detection.detection_id;
        track.has_source_detection = true;
        track.timestamp = detection.timestamp;
        track.bbox_px = detection.bbox_px;
        track.class_label = detection.class_label;
        track.faction = detection.faction;
        track.confidence = detection.confidence;
        track.state = TrackState::Confirmed;
        track.age_frames = 1;
        track.missed_frames = 0;
        track.depth_m = detection.depth_m;
        tracks.push_back(track);
    }

    return tracks;
}

std::vector<IdentityHypothesis> AppearanceOnlyIdentityResolver::resolve(const std::vector<Track2D>& tracks) {
    std::vector<IdentityHypothesis> identities;
    identities.reserve(tracks.size());

    for (std::size_t i = 0; i < tracks.size(); ++i) {
        const auto& track = tracks[i];
        IdentityHypothesis identity;
        identity.identity_id = IdentityId{"identity_unknown_" + std::string(4 - std::to_string(i + 1).size(), '0') + std::to_string(i + 1)};
        identity.track_id = track.track_id;
        identity.timestamp = track.timestamp;
        identity.confidence = 0.55F;
        identity.evidence = {"appearance_placeholder", "single_frame_track"};
        identities.push_back(identity);
    }

    return identities;
}

FlatGroundProjector::FlatGroundProjector(FlatGroundProjectorConfig config)
    : config_(config) {}

std::vector<Observation3D> FlatGroundProjector::project(
    const std::vector<Track2D>& tracks,
    const FramePacket& frame,
    const EgoState& ego) {
    std::vector<Observation3D> observations;
    observations.reserve(tracks.size());

    for (const auto& track : tracks) {
        const double bbox_cx = track.bbox_px.x + (track.bbox_px.width  * 0.5);
        const double bbox_cy = track.bbox_px.y + (track.bbox_px.height * 0.5);
        // Normalized image coordinates (optical-axis = 0).
        const double nx = (bbox_cx - frame.intrinsics.cx) / frame.intrinsics.fx;
        const double ny = (bbox_cy - frame.intrinsics.cy) / frame.intrinsics.fy;
        // Use detector-supplied depth.  In production all detectors set depth_m
        // (airsim_ground_truth always sets it; real depth-capable detectors do too).
        // If require_depth is true, missing depth is a hard error.
        constexpr double kFlatGroundDepthM = 18.0;
        if (!track.depth_m.has_value() && config_.require_depth) {
            throw std::runtime_error(
                "FlatGroundProjector: track " + track.track_id.value +
                " has no depth_m but require_depth is set");
        }
        const double depth_m = track.depth_m.value_or(kFlatGroundDepthM);

        // Back-project from camera frame to body frame.
        // Front-center camera convention: cam_x=right=body_y, cam_y=down=body_z, cam_z=forward=body_x.
        // p_cam = {nx*depth_m, ny*depth_m, depth_m}  →  p_body = {depth_m, nx*depth_m, ny*depth_m}
        const Vec3 p_body{depth_m, nx * depth_m, ny * depth_m};

        Observation3D observation;
        observation.track_id             = track.track_id;
        observation.source_detection_id  = track.source_detection_id;
        observation.has_source_detection = track.has_source_detection;
        observation.source_bbox_px       = track.bbox_px;
        observation.has_source_bbox      = true;
        observation.source_frame_id      = frame.frame_id;
        observation.has_source_frame     = true;
        observation.timestamp            = track.timestamp;
        observation.position_body        = p_body;
        // Rotate body-frame vector to local frame and translate by ego position.
        // transform_point(local_T_body, p_body) = ego.position + R_local_from_body * p_body
        observation.position_local       = transform_point(ego.local_T_body, p_body);
        observation.map_frame_id         = ego.map_frame_id;
        observation.class_label          = track.class_label;
        observation.faction              = track.faction;
        observation.confidence           = track.confidence * 0.7F;
        observations.push_back(observation);
    }

    return observations;
}

}  // namespace dedalus
