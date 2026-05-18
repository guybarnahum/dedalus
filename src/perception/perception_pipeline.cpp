#include "dedalus/perception/perception_pipeline.hpp"

#include <string>

namespace dedalus {

PerceptionPipeline::PerceptionPipeline(
    Detector& detector,
    CameraStabilizer& stabilizer,
    Tracker& tracker,
    IdentityResolver& identity_resolver,
    Projector3D& projector)
    : detector_(detector),
      stabilizer_(stabilizer),
      tracker_(tracker),
      identity_resolver_(identity_resolver),
      projector_(projector) {}

PerceptionPipelineOutput PerceptionPipeline::process(const FramePacket& frame, const EgoState& ego) {
    PerceptionPipelineOutput output;
    output.detections = detector_.detect(frame);
    output.stabilized_frame = stabilizer_.stabilize(frame, output.detections);
    output.tracks = tracker_.update(output.stabilized_frame.detections);
    output.identities = identity_resolver_.resolve(output.tracks);
    output.observations = projector_.project(output.tracks, output.stabilized_frame.frame, ego);
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

std::vector<Observation3D> FlatGroundProjector::project(
    const std::vector<Track2D>& tracks,
    const FramePacket& frame,
    const EgoState& ego) {
    std::vector<Observation3D> observations;
    observations.reserve(tracks.size());

    for (const auto& track : tracks) {
        const double bbox_center_x = track.bbox_px.x + (track.bbox_px.width * 0.5);
        const double normalized_x = (bbox_center_x - frame.intrinsics.cx) / frame.intrinsics.fx;

        Observation3D observation;
        observation.track_id = track.track_id;
        observation.source_detection_id = track.source_detection_id;
        observation.has_source_detection = track.has_source_detection;
        observation.source_bbox_px = track.bbox_px;
        observation.has_source_bbox = true;
        observation.source_frame_id = frame.frame_id;
        observation.has_source_frame = true;
        observation.timestamp = track.timestamp;
        observation.position_body = Vec3{18.0, normalized_x * 18.0, 1.5};
        observation.position_local = Vec3{
            ego.local_T_body.position.x + observation.position_body.x,
            ego.local_T_body.position.y + observation.position_body.y,
            ego.local_T_body.position.z + observation.position_body.z};
        observation.map_frame_id = ego.map_frame_id;
        observation.class_label = track.class_label;
        observation.faction = track.faction;
        observation.confidence = track.confidence * 0.7F;
        observations.push_back(observation);
    }

    return observations;
}

}  // namespace dedalus
