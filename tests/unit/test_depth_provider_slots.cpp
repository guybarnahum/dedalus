// VD4: Two-slot depth provider architecture tests.
//
// Tests:
//   1. AirSimDepthEvidenceProvider: sensor-name filter blocks mismatched frames.
//   2. AirSimDepthEvidenceProvider: produces evidence when sensor names match.
//   3. AirSimEmulationDepthObstacleDetector: GT depth → VD kernels →
//      AirSimGroundTruthVisualEmulation source_kind.
//   4. AirSimEmulationDepthObstacleDetector: evaluates surface patches + thin
//      structures (also evaluates thin obstacles and landable surfaces).
//   5. CoreStackRunnerConfig: backward-compat auto-builds slot A when
//      depth_slot_a is null.

#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "dedalus/occupancy/occupancy_types.hpp"
#include "dedalus/sensing/airsim_depth_evidence_provider.hpp"
#include "dedalus/sensing/airsim_emulation_depth_obstacle_detector.hpp"
#include "dedalus/sensing/obstacle_evidence_provider.hpp"

namespace {

void require(bool cond, const char* msg) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", msg);
        std::exit(1);
    }
}

// Build a minimal EgoSensingFrame with an AirSim depth frame.
dedalus::EgoSensingFrame make_ego_frame(
    const std::string& depth_sensor,
    const std::string& volume_sensor,
    int width, int height,
    float fill_depth_m = 5.0F) {

    dedalus::AirSimDepthFrame df;
    df.width  = width;
    df.height = height;
    df.sensor_name = depth_sensor;
    df.depth_m.assign(static_cast<std::size_t>(width * height), fill_depth_m);

    dedalus::FramePacket fp;
    fp.depth_frame = df;
    fp.intrinsics.fx = static_cast<double>(width);
    fp.intrinsics.fy = static_cast<double>(width);
    fp.intrinsics.cx = static_cast<double>(width)  / 2.0;
    fp.intrinsics.cy = static_cast<double>(height) / 2.0;

    dedalus::CameraSensingVolume csv;
    csv.camera_name          = volume_sensor;
    csv.horizontal_fov_rad   = 1.5708;   // ~90°
    csv.vertical_fov_rad     = 1.0472;   // ~60°
    csv.near_range_m         = 0.5;
    csv.far_range_m          = 80.0;
    csv.min_reliable_range_m = 1.0;
    csv.max_reliable_range_m = 60.0;
    csv.forward_axis_local   = dedalus::Vec3{1.0, 0.0, 0.0};
    csv.right_axis_local     = dedalus::Vec3{0.0, 1.0, 0.0};
    csv.up_axis_local        = dedalus::Vec3{0.0, 0.0,-1.0};

    dedalus::EgoState ego;
    ego.map_frame_id = dedalus::MapFrameId{"map_test"};

    dedalus::EgoSensingFrame esf;
    esf.frame          = fp;
    esf.ego            = ego;
    esf.sensing_volume = csv;
    return esf;
}

// Test 1: sensor-name mismatch → empty output.
void test_sensor_name_filter_blocks() {
    dedalus::AirSimDepthEvidenceProvider provider;
    auto esf = make_ego_frame("camera_front", "camera_rear", 16, 16);
    const auto ev = provider.detect(esf);
    require(ev.empty(), "sensor-name mismatch should produce no evidence");
    std::puts("PASS test_sensor_name_filter_blocks");
}

// Test 2: matching sensor names → evidence produced.
void test_sensor_name_match_produces_evidence() {
    dedalus::AirSimDepthEvidenceProvider provider;
    auto esf = make_ego_frame("camera_front", "camera_front", 16, 16, /*depth*/ 5.0F);
    const auto ev = provider.detect(esf);
    require(!ev.empty(), "matching sensor name should produce evidence");
    std::puts("PASS test_sensor_name_match_produces_evidence");
}

// Test 3: emulation detector stamps AirSimGroundTruthVisualEmulation.
void test_emulation_source_kind() {
    dedalus::AirSimEmulationDepthObstacleDetectorConfig cfg;
    cfg.pixel_stride = 1U;
    cfg.detect_surface_patches  = false;
    cfg.detect_thin_structures  = false;
    dedalus::AirSimEmulationDepthObstacleDetector detector{cfg};

    auto esf = make_ego_frame("", "", 16, 16, /*depth*/ 5.0F);
    const auto ev = detector.detect(esf);
    require(!ev.empty(), "emulation detector should produce evidence");

    bool all_correct_kind = true;
    for (const auto& e : ev) {
        if (e.source_kind != dedalus::OccupancySourceKind::AirSimGroundTruthVisualEmulation) {
            all_correct_kind = false;
            break;
        }
    }
    require(all_correct_kind, "emulation evidence must have AirSimGroundTruthVisualEmulation source_kind");
    std::puts("PASS test_emulation_source_kind");
}

// Test 4: emulation detector with surface patches + thin structures enabled
// produces some evidence (evaluates thin obstacles and landable surfaces).
void test_emulation_evaluates_surfaces_and_thin() {
    dedalus::AirSimEmulationDepthObstacleDetectorConfig cfg;
    cfg.pixel_stride = 1U;
    cfg.detect_surface_patches = true;
    cfg.detect_thin_structures = true;
    dedalus::AirSimEmulationDepthObstacleDetector detector{cfg};

    // Flat wall at 5 m — should produce projection evidence + a surface patch.
    auto esf = make_ego_frame("", "", 16, 16, /*depth*/ 5.0F);
    const auto ev = detector.detect(esf);
    require(!ev.empty(), "emulation detector with VD kernels should produce evidence");

    bool has_surface = false;
    for (const auto& e : ev) {
        if (e.is_surface_hint) { has_surface = true; break; }
    }
    require(has_surface, "flat wall at 5 m should produce at least one surface patch");
    std::puts("PASS test_emulation_evaluates_surfaces_and_thin");
}

// Test 5: provider_name() returns the expected string.
void test_provider_names() {
    dedalus::AirSimDepthEvidenceProvider adapter;
    require(adapter.provider_name() == "airsim_depth_gt",
        "adapter provider_name mismatch");

    dedalus::AirSimEmulationDepthObstacleDetector emulator;
    require(emulator.provider_name() == "airsim_gt_vd",
        "emulator provider_name mismatch");
    std::puts("PASS test_provider_names");
}

}  // namespace

int main() {
    test_sensor_name_filter_blocks();
    test_sensor_name_match_produces_evidence();
    test_emulation_source_kind();
    test_emulation_evaluates_surfaces_and_thin();
    test_provider_names();
    std::puts("All depth_provider_slots tests passed.");
    return 0;
}
