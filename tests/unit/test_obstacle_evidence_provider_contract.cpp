#include "dedalus/sensing/obstacle_evidence_provider.hpp"

#include <cassert>
#include <iostream>
#include <string>
#include <vector>

using namespace dedalus;

namespace {

class FakeObstacleEvidenceProvider final : public ObstacleEvidenceProvider {
public:
    std::string provider_name() const override {
        return "fake_obstacle_evidence_provider";
    }

    std::vector<ObstacleEvidence> detect(const EgoSensingFrame& frame) override {
        ObstacleEvidence evidence;
        evidence.source_provider = provider_name();
        evidence.source_kind = OccupancySourceKind::VisualObstacleDetector;
        evidence.state = OccupancyState::Occupied;
        evidence.shape = OccupancyShape::SurfacePatch;
        evidence.map_frame_id = frame.ego.map_frame_id;
        evidence.center_local = Vec3{1.0, 2.0, 3.0};
        evidence.size_m = Vec3{0.5, 0.5, 0.1};
        evidence.confidence = 0.75F;
        evidence.has_source_frame = true;
        evidence.source_frame_id = frame.frame.frame_id;
        return {evidence};
    }
};

void provider_uses_existing_evidence_contract() {
    EgoSensingFrame frame;
    frame.frame.frame_id = "frame_001";
    frame.ego.map_frame_id = "mission_local";
    frame.sensing_volume.volume_id = "front_camera";
    frame.has_depth_frame = false;
    frame.has_camera_extrinsics = true;

    FakeObstacleEvidenceProvider provider;
    const auto evidence = provider.detect(frame);

    assert(provider.provider_name() == "fake_obstacle_evidence_provider");
    assert(evidence.size() == 1U);

    const auto& first = evidence.front();
    assert(first.source_provider == "fake_obstacle_evidence_provider");
    assert(first.source_kind == OccupancySourceKind::VisualObstacleDetector);
    assert(first.state == OccupancyState::Occupied);
    assert(first.shape == OccupancyShape::SurfacePatch);
    assert(first.map_frame_id == "mission_local");
    assert(first.has_source_frame);
    assert(first.source_frame_id == "frame_001");
    assert(first.confidence > 0.7F);
}

}  // namespace

int main() {
    provider_uses_existing_evidence_contract();
    std::cout << "obstacle evidence provider contract tests passed\n";
    return 0;
}
