#pragma once

#include <memory>
#include <string>
#include <vector>

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/sensors/ego_state_provider.hpp"
#include "dedalus/sensors/frame_source.hpp"
#include "dedalus/simulation/bridge_transport.hpp"

namespace dedalus {

// Minimal object binding used by AirSimGroundTruthDetector.  Mirrors the
// relevant fields of AirSimGhostObjectBinding without pulling in ghost_targets.hpp.
struct AirSimDetectorObjectBinding {
    std::string airsim_object_name;
    ClassLabel class_label{ClassLabel::Person};
    double confidence{1.0};
    Vec3 size_m{0.5, 0.5, 1.8};  // y=lateral width, z=vertical height (NED body frame)
};

struct AirSimProviderConfig {
    std::string host{"127.0.0.1"};
    int rpc_port{41451};
    std::string vehicle_name{"PX4"};
    std::string camera_name{"front_center"};
    std::string transport{"pipe"};
    std::string bridge_command{"python3 simulation/airsim/scripts/airsim-capture-frame.py"};
    std::string bridge_mode{"one_shot_ppm"};
    std::string ego_bridge_command{"python3 simulation/airsim/scripts/airsim-capture-ego.py"};
    MapFrameId map_frame_id{"map_airsim_0001"};

    // Used by AirSimGroundTruthDetector — objects to detect each frame.
    std::string objects_bridge_command{"python3 simulation/airsim/scripts/airsim-object-poses.py"};
    std::vector<AirSimDetectorObjectBinding> detector_objects;
};

class AirSimFrameSource final : public FrameSource {
public:
    explicit AirSimFrameSource(AirSimProviderConfig config);
    ~AirSimFrameSource() override;

    AirSimFrameSource(const AirSimFrameSource&) = delete;
    AirSimFrameSource& operator=(const AirSimFrameSource&) = delete;

    std::optional<FramePacket> next_frame() override;

private:
    FramePacket next_one_shot_frame();
    std::optional<FramePacket> next_stream_jsonl_frame();
    std::optional<FramePacket> next_stream_binary_frame();

    AirSimProviderConfig config_;
    std::unique_ptr<BridgeTransport> transport_;
    int next_frame_index_{0};
};

class AirSimEgoStateProvider final : public EgoStateProvider {
public:
    explicit AirSimEgoStateProvider(AirSimProviderConfig config);

    EgoStateEstimate estimate(const FramePacket& frame) override;

private:
    AirSimProviderConfig config_;
    std::unique_ptr<OneShotTransport> transport_;
};

class AirSimDepthProjector final : public Projector3D {
public:
    explicit AirSimDepthProjector(AirSimProviderConfig config);

    std::vector<Observation3D> project(
        const std::vector<Track2D>& tracks,
        const FramePacket& frame,
        const EgoState& ego) override;

private:
    AirSimProviderConfig config_;
};

class AirSimGroundTruthDetector final : public Detector {
public:
    explicit AirSimGroundTruthDetector(AirSimProviderConfig config);
    ~AirSimGroundTruthDetector() override;

    std::vector<Detection2D> detect(const FramePacket& frame) override;

private:
    AirSimProviderConfig config_;
    std::unique_ptr<PipeBridgeTransport> transport_;
};

}  // namespace dedalus
