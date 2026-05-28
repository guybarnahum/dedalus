#pragma once

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
