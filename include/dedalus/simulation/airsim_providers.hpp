#pragma once

#include <string>

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/sensors/ego_state_provider.hpp"
#include "dedalus/sensors/frame_source.hpp"

namespace dedalus {

struct AirSimProviderConfig {
    std::string host{"127.0.0.1"};
    int rpc_port{41451};
    std::string vehicle_name{"PX4"};
    std::string camera_name{"front_center"};
    std::string bridge_command{"python3 simulation/airsim-capture-frame.py"};
    MapFrameId map_frame_id{"map_airsim_0001"};
};

class AirSimFrameSource final : public FrameSource {
public:
    explicit AirSimFrameSource(AirSimProviderConfig config);

    std::optional<FramePacket> next_frame() override;

private:
    AirSimProviderConfig config_;
    int next_frame_index_{0};
};

class AirSimEgoStateProvider final : public EgoStateProvider {
public:
    explicit AirSimEgoStateProvider(AirSimProviderConfig config);

    EgoStateEstimate estimate(const FramePacket& frame) override;

private:
    AirSimProviderConfig config_;
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

    std::vector<Detection2D> detect(const FramePacket& frame) override;

private:
    AirSimProviderConfig config_;
};

}  // namespace dedalus
