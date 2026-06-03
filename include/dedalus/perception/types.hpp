#pragma once

#include <optional>
#include <string>
#include <vector>

#include "dedalus/core/types.hpp"

namespace dedalus {

struct Detection2D {
    DetectionId detection_id;
    FrameId frame_id;
    TimePoint timestamp;
    Rect2 bbox_px;
    float confidence{0.0F};
    ClassLabel class_label{ClassLabel::Unknown};
    FactionLabel faction{FactionLabel::Unknown};
    FeatureVector appearance;
    // Optional depth at the bounding-box centroid, metres.  Populated by
    // detectors that have direct depth access (e.g. AirSim ground-truth).
    // When present, Projector3D implementations may use it instead of making
    // a separate depth acquisition call.
    std::optional<double> depth_m;
};

enum class TrackState {
    Tentative,
    Confirmed,
    Lost
};

struct Track2D {
    TrackId track_id;
    DetectionId source_detection_id;
    bool has_source_detection{false};
    TimePoint timestamp;
    Rect2 bbox_px;
    ClassLabel class_label{ClassLabel::Unknown};
    FactionLabel faction{FactionLabel::Unknown};
    float confidence{0.0F};
    TrackState state{TrackState::Tentative};
    int age_frames{0};
    int missed_frames{0};
    // Depth carried forward from the source Detection2D, if available.
    std::optional<double> depth_m;
};

struct IdentityHypothesis {
    IdentityId identity_id;
    TrackId track_id;
    TimePoint timestamp;
    float confidence{0.0F};
    std::vector<std::string> evidence;
};

struct Observation3D {
    TrackId track_id;
    DetectionId source_detection_id;
    bool has_source_detection{false};
    Rect2 source_bbox_px;
    bool has_source_bbox{false};
    FrameId source_frame_id;
    bool has_source_frame{false};
    TimePoint timestamp;
    Vec3 position_body;
    Vec3 position_local;
    Vec3 velocity_local;
    MapFrameId map_frame_id{"map_unknown"};
    Covariance3 covariance{};
    ClassLabel class_label{ClassLabel::Unknown};
    FactionLabel faction{FactionLabel::Unknown};
    float confidence{0.0F};
};

}  // namespace dedalus
