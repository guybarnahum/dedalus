// VD4: Two-slot depth provider architecture tests.
//
// Tests:
//   1. AirSimEmulationDepthObstacleDetector: sensor-name filter blocks mismatched frames.
//   2. AirSimEmulationDepthObstacleDetector: produces evidence when sensor names match.
//   3. AirSimEmulationDepthObstacleDetector: GT depth → VD kernels →
//      AirSimGroundTruthVisualEmulation source_kind.
//   4. AirSimEmulationDepthObstacleDetector: evaluates surface patches + thin
//      structures (also evaluates thin obstacles and landable surfaces).
//   5. provider_name() returns "airsim_gt_vd".

#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "dedalus/occupancy/occupancy_types.hpp"
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
    dedalus::AirSimEmulationDepthObstacleDetector provider;
    // 80×44: BW = 80/40 = 2, BH = 44/22 = 2 — block-min kernel produces evidence.
    auto esf = make_ego_frame("camera_front", "camera_rear", 80, 44);
    const auto ev = provider.detect(esf);
    require(ev.empty(), "sensor-name mismatch should produce no evidence");
    std::puts("PASS test_sensor_name_filter_blocks");
}

// Test 2: matching sensor names → evidence produced.
void test_sensor_name_match_produces_evidence() {
    dedalus::AirSimEmulationDepthObstacleDetector provider;
    // 80×44: 40×22 grid, each cell 2×2 pixels — all filled at depth=5 m → non-empty.
    auto esf = make_ego_frame("camera_front", "camera_front", 80, 44, /*depth*/ 5.0F);
    const auto ev = provider.detect(esf);
    require(!ev.empty(), "matching sensor name should produce evidence");
    std::puts("PASS test_sensor_name_match_produces_evidence");
}

// Test 3: emulation detector stamps AirSimGroundTruthVisualEmulation.
void test_emulation_source_kind() {
    dedalus::AirSimEmulationDepthObstacleDetectorConfig cfg;
    cfg.depth_grid_cols = 16U;  // 16×16 grid on 16×16 frame → 1 px per cell (dense)
    cfg.depth_grid_rows = 16U;
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
    cfg.depth_grid_cols = 16U;  // 16×16 grid on 16×16 frame → 1 px per cell (dense)
    cfg.depth_grid_rows = 16U;
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

// Test 5: provider_name() returns "airsim_gt_vd".
void test_provider_names() {
    dedalus::AirSimEmulationDepthObstacleDetector provider;
    require(provider.provider_name() == "airsim_gt_vd",
        "provider_name should be airsim_gt_vd");
    std::puts("PASS test_provider_names");
}

// Test 6: FoV-driven intrinsics — production scenario.
// make_params() computes intrinsics directly from the sensing volume FoV +
// depth frame dimensions.  For a 90°×60° camera with a 40×22 depth frame:
//   fx = (40/2) / tan(45°) = 20.0,  cx = 20.0
//   fy = (22/2) / tan(30°) ≈ 19.05, cy = 11.0
// Columns [0,39] then span ±45° horizontally, so roughly half the cells
// should project to the left of center and half to the right.
// Before this change (image.width-based scaling), any code path that delivers
// an EgoSensingFrame without a populated RGB image (image.width=0) would
// trigger the fallback s_y=1.0, leaving cy=180 >> max row 21, making every
// yn negative and projecting ALL evidence above the camera.
void test_intrinsic_scaling_production_scenario() {
    const int depth_w = 40, depth_h = 22;

    dedalus::AirSimDepthFrame df;
    df.width  = depth_w;
    df.height = depth_h;
    df.depth_m.assign(static_cast<std::size_t>(depth_w * depth_h), 5.0F);

    dedalus::FramePacket fp;
    fp.depth_frame = df;
    // image.width/height intentionally NOT set — FoV path must not need them.
    fp.intrinsics.fx = 999.0;  // deliberately wrong to confirm they're unused
    fp.intrinsics.fy = 999.0;
    fp.intrinsics.cx = 999.0;
    fp.intrinsics.cy = 999.0;

    dedalus::CameraSensingVolume csv;
    csv.horizontal_fov_rad   = 1.5708;
    csv.vertical_fov_rad     = 1.0472;
    csv.near_range_m         = 0.5;
    csv.far_range_m          = 80.0;
    csv.min_reliable_range_m = 1.0;
    csv.max_reliable_range_m = 60.0;
    csv.forward_axis_local   = dedalus::Vec3{1.0, 0.0, 0.0};
    csv.right_axis_local     = dedalus::Vec3{0.0, 1.0, 0.0};
    csv.up_axis_local        = dedalus::Vec3{0.0, 0.0,-1.0};

    dedalus::EgoSensingFrame esf;
    esf.frame          = fp;
    esf.sensing_volume = csv;

    dedalus::AirSimEmulationDepthObstacleDetectorConfig cfg;
    cfg.depth_grid_cols = static_cast<std::size_t>(depth_w);
    cfg.depth_grid_rows = static_cast<std::size_t>(depth_h);
    cfg.detect_surface_patches = false;
    cfg.detect_thin_structures = false;
    dedalus::AirSimEmulationDepthObstacleDetector detector{cfg};

    const auto ev = detector.detect(esf);
    require(!ev.empty(), "production intrinsic scaling: evidence must be produced");

    // Flat wall 5 m ahead: columns span ±45° → half project left (ly<0), half right (ly>0).
    int leftward_count  = 0;
    int rightward_count = 0;
    for (const auto& e : ev) {
        if (e.center_local.y < 0.0) ++leftward_count;
        else                        ++rightward_count;
    }
    require(rightward_count > 0,
        "FoV intrinsics: some evidence must project right of center");
    require(leftward_count  > 0,
        "FoV intrinsics: some evidence must project left of center");

    // Vertical: top rows look above horizon (center_local.z < origin_z = 0),
    // bottom rows look below (center_local.z > 0).  NOT all above.
    int above_count = 0;
    int below_count = 0;
    for (const auto& e : ev) {
        if (e.center_local.z < 0.0) ++above_count;
        else                        ++below_count;
    }
    require(below_count > 0,
        "FoV intrinsics: some evidence must project below camera (vertical coverage)");
    require(above_count < static_cast<int>(ev.size()),
        "FoV intrinsics: not all evidence may be above camera (regression: cy=180)");
    std::puts("PASS test_intrinsic_scaling_production_scenario");
}

}  // namespace

int main() {
    test_sensor_name_filter_blocks();
    test_sensor_name_match_produces_evidence();
    test_emulation_source_kind();
    test_emulation_evaluates_surfaces_and_thin();
    test_provider_names();
    test_intrinsic_scaling_production_scenario();
    std::puts("All depth_provider_slots tests passed.");
    return 0;
}
