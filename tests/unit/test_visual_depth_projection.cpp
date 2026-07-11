#include <cmath>
#include <iostream>
#include <vector>

#include "dedalus/sensing/depth_engine.hpp"
#include "dedalus/sensing/depth_projection_kernel.hpp"
#include "dedalus/sensing/metric_scale_estimate.hpp"
#include "dedalus/sensing/visual_depth_obstacle_detector.hpp"
#include "dedalus/sensing/visual_depth_frame.hpp"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace {

bool near(double actual, double expected, double eps) {
    return std::abs(actual - expected) < eps;
}

// A mock depth engine that returns a known synthetic depth map.
// Ignores the frame content; outputs a caller-supplied inverse_depth buffer.
class MockDepthEngine final : public dedalus::DepthEngineInterface {
public:
    int                out_width{0};
    int                out_height{0};
    std::vector<float> inverse_depth;

    [[nodiscard]] dedalus::DepthInferenceResult infer(
        const dedalus::VisualDepthFrame& /*frame*/) override {
        dedalus::DepthInferenceResult r;
        r.width          = out_width;
        r.height         = out_height;
        r.inverse_depth = inverse_depth;
        r.valid          = !inverse_depth.empty();
        return r;
    }

    [[nodiscard]] std::string engine_name() const override { return "mock"; }
};

// Build a minimal EgoSensingFrame for tests.
// Camera at origin, forward = +X, right = +Y, up = -Z (NED).
// Intrinsics: fx=fy=100, cx=cy=centre of a 9×9 image.
dedalus::EgoSensingFrame make_ego_frame(int w, int h) {
    dedalus::EgoSensingFrame f;
    f.frame.timestamp          = dedalus::TimePoint{1'000'000};
    f.frame.frame_id           = dedalus::FrameId{"test_frame"};
    f.frame.image.width        = w;
    f.frame.image.height       = h;
    f.frame.image.channels     = 3;
    f.frame.image.bytes.assign(static_cast<std::size_t>(w * h * 3), 128U);
    f.frame.intrinsics.fx      = 100.0;
    f.frame.intrinsics.fy      = 100.0;
    f.frame.intrinsics.cx      = (w - 1) / 2.0;
    f.frame.intrinsics.cy      = (h - 1) / 2.0;

    f.ego.map_frame_id         = dedalus::MapFrameId{"map_test"};

    f.sensing_volume.timestamp       = f.frame.timestamp;
    f.sensing_volume.camera_name     = "front_center";
    f.sensing_volume.origin_local    = dedalus::Vec3{0.0, 0.0, 0.0};
    f.sensing_volume.forward_axis_local = dedalus::Vec3{1.0, 0.0, 0.0};
    f.sensing_volume.right_axis_local   = dedalus::Vec3{0.0, 1.0, 0.0};
    f.sensing_volume.up_axis_local      = dedalus::Vec3{0.0, 0.0, -1.0};

    return f;
}

}  // namespace

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// Test 1: Single obstacle at 10 m directly forward.
//
// Setup: 9×9 depth map, all infinity except the centre pixel.
// Centre pixel encodes 10 m forward via disparity convention:
//   depth_m = scale / inverse_depth  →  inverse_depth = scale / depth_m
//
// Expected: exactly one ObstacleEvidence whose center_x ≈ 10.0 within
//           one voxel (0.5 m). center_y and center_z ≈ 0 within one voxel.
static bool test_single_forward_obstacle() {
    const int W = 9;
    const int H = 9;
    const float SCALE = 1000.0F;
    const float TARGET_DEPTH_M = 10.0F;
    const float VOXEL = 0.5F;

    // Build depth map: all very large (effectively invalid) except centre
    std::vector<float> depth_rel(static_cast<std::size_t>(W * H), 1e-6F);
    const int cx_px = (W - 1) / 2;  // 4
    const int cy_px = (H - 1) / 2;  // 4
    depth_rel[static_cast<std::size_t>(cy_px * W + cx_px)] =
        SCALE / TARGET_DEPTH_M;  // = 100.0

    auto engine = std::make_unique<MockDepthEngine>();
    engine->out_width      = W;
    engine->out_height     = H;
    engine->inverse_depth = depth_rel;

    dedalus::MetricScaleEstimate scale;
    scale.scale      = SCALE;
    scale.confidence = 1.0F;

    dedalus::VisualDepthObstacleDetectorConfig cfg;
    cfg.depth_grid_cols        = static_cast<std::size_t>(W);  // 1 pixel per cell
    cfg.depth_grid_rows        = static_cast<std::size_t>(H);
    cfg.min_depth_m            = 0.5F;
    cfg.max_depth_m            = 80.0F;
    cfg.voxel_size_m           = VOXEL;
    cfg.confidence             = 0.75F;
    cfg.max_evidence           = 512U;
    cfg.detect_surface_patches = false;
    cfg.detect_thin_structures = false;

    dedalus::VisualDepthObstacleDetector detector(
        std::move(engine), scale, cfg);

    const auto ego = make_ego_frame(W, H);
    const auto evidence = detector.detect(ego);

    // Grid W×H (1 pixel per cell): all non-centre cells have invalid depth → filtered.
    // Only the centre cell (containing the 10m pixel) produces evidence.
    if (evidence.size() != 1U) {
        std::cerr << "FAIL test_single_forward_obstacle: expected 1 evidence, got "
                  << evidence.size() << "\n";
        return false;
    }

    const auto& ev = evidence[0];
    // Centre pixel (cx=4, cy=4): xn=0, yn=0 → xc=0, yc=0, zc=10
    // local = origin + 10*forward + 0*right - 0*up = (10, 0, 0)
    // Snapped to voxel grid: ix=20 → centre = 20.25 → 10.25? No:
    //   ix = floor(10.0 / 0.5) = 20
    //   voxel centre = (20 + 0.5) * 0.5 = 10.25
    // Within 1 voxel of 10.0: |10.25 - 10.0| = 0.25 < 0.5 ✓
    const double expected_x = TARGET_DEPTH_M;
    const double tol = VOXEL;

    if (!near(ev.center_local.x, expected_x, tol)) {
        std::cerr << "FAIL test_single_forward_obstacle: center_x="
                  << ev.center_local.x << " expected≈" << expected_x
                  << " tol=" << tol << "\n";
        return false;
    }
    if (!near(ev.center_local.y, 0.0, tol)) {
        std::cerr << "FAIL test_single_forward_obstacle: center_y="
                  << ev.center_local.y << " expected≈0 tol=" << tol << "\n";
        return false;
    }
    if (!near(ev.center_local.z, 0.0, tol)) {
        std::cerr << "FAIL test_single_forward_obstacle: center_z="
                  << ev.center_local.z << " expected≈0 tol=" << tol << "\n";
        return false;
    }

    // State must be Occupied; shape SurfacePatch (project_depth_to_device_evidence
    // emits shape=3 so evidence renders as normal-oriented diamonds in the overlay).
    if (ev.state != dedalus::ObstacleEvidenceState::Occupied) {
        std::cerr << "FAIL test_single_forward_obstacle: wrong state\n";
        return false;
    }
    if (ev.shape != dedalus::ObstacleEvidenceShape::SurfacePatch) {
        std::cerr << "FAIL test_single_forward_obstacle: wrong shape "
                  << static_cast<int>(ev.shape) << " (expected SurfacePatch=3)\n";
        return false;
    }

    // is_surface_hint must be set; is_thin_structure_hint must be off
    if (ev.is_thin_structure_hint) {
        std::cerr << "FAIL test_single_forward_obstacle: unexpected thin_structure_hint\n";
        return false;
    }
    if (!ev.is_surface_hint) {
        std::cerr << "FAIL test_single_forward_obstacle: is_surface_hint not set\n";
        return false;
    }

    // source_provider and map_frame_id must be stamped
    if (ev.source_provider != "visual_depth_obstacle_detector") {
        std::cerr << "FAIL test_single_forward_obstacle: wrong source_provider\n";
        return false;
    }
    if (ev.map_frame_id.value != "map_test") {
        std::cerr << "FAIL test_single_forward_obstacle: wrong map_frame_id\n";
        return false;
    }

    return true;
}

// Test 2: Depth range filter — all pixels outside [min, max] produce no evidence.
static bool test_depth_range_filter() {
    const int W = 5;
    const int H = 5;
    const float SCALE = 1000.0F;
    const float VOXEL = 0.5F;

    // All pixels at 100 m → beyond max_depth_m=80 → filtered
    std::vector<float> depth_rel(static_cast<std::size_t>(W * H),
                                 SCALE / 100.0F);  // inverse_depth=10 → depth_m=100

    auto engine = std::make_unique<MockDepthEngine>();
    engine->out_width      = W;
    engine->out_height     = H;
    engine->inverse_depth = depth_rel;

    dedalus::MetricScaleEstimate scale;
    scale.scale = SCALE;

    dedalus::VisualDepthObstacleDetectorConfig cfg;
    cfg.depth_grid_cols        = static_cast<std::size_t>(W);
    cfg.depth_grid_rows        = static_cast<std::size_t>(H);
    cfg.min_depth_m            = 0.5F;
    cfg.max_depth_m            = 80.0F;
    cfg.voxel_size_m           = VOXEL;
    cfg.detect_surface_patches = false;
    cfg.detect_thin_structures = false;

    dedalus::VisualDepthObstacleDetector detector(
        std::move(engine), scale, cfg);

    const auto ego    = make_ego_frame(W, H);
    const auto evidence = detector.detect(ego);

    if (!evidence.empty()) {
        std::cerr << "FAIL test_depth_range_filter: expected 0 evidence, got "
                  << evidence.size() << "\n";
        return false;
    }
    return true;
}

// Test 3: Grid sampling — N×M grid produces exactly N×M evidence points
// (one per cell) when all pixels are valid.
static bool test_grid_sampling() {
    const int W = 8;
    const int H = 8;
    const std::size_t GC = 4U;  // 4 columns → 2px per cell
    const std::size_t GR = 4U;  // 4 rows    → 2px per cell
    const float SCALE = 1000.0F;
    const float DEPTH = 5.0F;

    // All pixels valid at 5 m.
    std::vector<float> depth_rel(static_cast<std::size_t>(W * H), SCALE / DEPTH);

    auto engine = std::make_unique<MockDepthEngine>();
    engine->out_width      = W;
    engine->out_height     = H;
    engine->inverse_depth = depth_rel;

    dedalus::MetricScaleEstimate scale;
    scale.scale = SCALE;

    dedalus::VisualDepthObstacleDetectorConfig cfg;
    cfg.depth_grid_cols        = GC;
    cfg.depth_grid_rows        = GR;
    cfg.min_depth_m            = 0.5F;
    cfg.max_depth_m            = 80.0F;
    cfg.voxel_size_m           = 0.5F;
    cfg.max_evidence           = 512U;
    cfg.detect_surface_patches = false;
    cfg.detect_thin_structures = false;

    dedalus::VisualDepthObstacleDetector detector(std::move(engine), scale, cfg);

    const auto ego      = make_ego_frame(W, H);
    const auto evidence = detector.detect(ego);

    // 4×4 grid, all cells valid → exactly 16 evidence points.
    const std::size_t expected = GC * GR;
    if (evidence.size() != expected) {
        std::cerr << "FAIL test_grid_sampling: expected " << expected
                  << " evidence, got " << evidence.size() << "\n";
        return false;
    }
    return true;
}

// Test 4: inflate stamps DeviceObstacleEvidence correctly.
static bool test_inflate_direct() {
    dedalus::DeviceObstacleEvidence dev;
    dev.center_x  = 3.0F;
    dev.center_y  = 1.0F;
    dev.center_z  = -2.0F;
    dev.size_x = dev.size_y = dev.size_z = 0.5F;
    dev.normal_x  = 0.0F;
    dev.normal_y  = 0.0F;
    dev.normal_z  = 1.0F;
    dev.confidence = 0.9F;
    dev.range_m    = 3.7F;
    dev.state      = 2U;   // Occupied
    dev.shape      = 3U;   // SurfacePatch
    dev.is_surface_hint = 1U;

    const auto result = dedalus::inflate(
        &dev, 1U,
        "test_sensor",
        "test_provider",
        dedalus::MapFrameId{"map_inflate"},
        dedalus::TimePoint{42});

    if (result.size() != 1U) {
        std::cerr << "FAIL test_inflate_direct: expected 1, got " << result.size() << "\n";
        return false;
    }

    const auto& ev = result[0];
    if (ev.state != dedalus::ObstacleEvidenceState::Occupied) {
        std::cerr << "FAIL test_inflate_direct: wrong state\n"; return false;
    }
    if (ev.shape != dedalus::ObstacleEvidenceShape::SurfacePatch) {
        std::cerr << "FAIL test_inflate_direct: wrong shape\n"; return false;
    }
    if (!ev.is_surface_hint) {
        std::cerr << "FAIL test_inflate_direct: is_surface_hint not set\n"; return false;
    }
    if (ev.sensor_name != "test_sensor") {
        std::cerr << "FAIL test_inflate_direct: wrong sensor_name\n"; return false;
    }
    if (ev.source_provider != "test_provider") {
        std::cerr << "FAIL test_inflate_direct: wrong source_provider\n"; return false;
    }
    if (ev.map_frame_id.value != "map_inflate") {
        std::cerr << "FAIL test_inflate_direct: wrong map_frame_id\n"; return false;
    }
    // Normal is non-zero → has_surface_normal must be set
    if (!ev.has_surface_normal) {
        std::cerr << "FAIL test_inflate_direct: has_surface_normal not set\n"; return false;
    }
    if (std::abs(ev.center_local.x - 3.0) > 1e-5) {
        std::cerr << "FAIL test_inflate_direct: center_x wrong\n"; return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
// VD3 Test 5: Flat wall → SurfacePatch with is_surface_hint
// ---------------------------------------------------------------------------

// Build the ProjectionParams used by kernel tests directly.
static dedalus::ProjectionParams make_params(int W, int H, float scale, float voxel) {
    dedalus::ProjectionParams p;
    p.fx = p.fy = 100.0F;
    p.cx = static_cast<float>(W - 1) / 2.0F;
    p.cy = static_cast<float>(H - 1) / 2.0F;
    p.width     = W;  p.height    = H;
    p.grid_cols = W;  p.grid_rows = H;  // 1 pixel per cell (stride-1 equivalent)
    p.min_depth_m = 0.5F;  p.max_depth_m = 80.0F;
    p.scale       = scale;
    p.voxel_size_m = voxel;
    p.max_evidence = 512U;
    // Camera at origin, forward=+X, right=+Y, up=-Z
    p.origin_x = p.origin_y = p.origin_z = 0.0F;
    p.forward_x=1.0F; p.forward_y=0.0F; p.forward_z=0.0F;
    p.right_x=0.0F;   p.right_y=1.0F;   p.right_z=0.0F;
    p.up_x=0.0F;      p.up_y=0.0F;      p.up_z=-1.0F;
    return p;
}

static bool test_surface_patch_wall() {
    const int W = 9, H = 9;
    const float SCALE = 1000.0F;
    const float VOXEL = 0.5F;
    const float DEPTH = 10.0F;

    // Flat wall at 10 m: all pixels valid
    const std::size_t N = static_cast<std::size_t>(W * H);
    std::vector<float> depth_rel(N, SCALE / DEPTH);

    const auto params = make_params(W, H, SCALE, VOXEL);

    // Project voxels
    std::vector<dedalus::DeviceObstacleEvidence> voxels(512U);
    std::uint32_t voxel_count = 0U;
    dedalus::project_depth_to_device_evidence(
        depth_rel.data(), params, voxels.data(), voxel_count);

    if (voxel_count == 0U) {
        std::cerr << "FAIL test_surface_patch_wall: no voxels projected\n";
        return false;
    }

    // Run RANSAC
    std::vector<dedalus::DeviceObstacleEvidence> patches(64U);
    std::uint32_t patch_count = 0U;
    dedalus::fit_surface_patches_device(
        voxels.data(), voxel_count, params, patches.data(), patch_count);

    if (patch_count == 0U) {
        std::cerr << "FAIL test_surface_patch_wall: no patches found ("
                  << voxel_count << " voxels)\n";
        return false;
    }

    const auto& p0 = patches[0];
    if (p0.is_surface_hint != 1U) {
        std::cerr << "FAIL test_surface_patch_wall: is_surface_hint not set\n";
        return false;
    }
    if (p0.shape != 3U) {  // SurfacePatch
        std::cerr << "FAIL test_surface_patch_wall: wrong shape " << +p0.shape << "\n";
        return false;
    }

    // Normal should be approximately ±X (forward axis)
    const float nx_abs = std::abs(p0.normal_x);
    if (nx_abs < 0.9F) {
        std::cerr << "FAIL test_surface_patch_wall: normal_x=" << p0.normal_x
                  << " expected ≈±1\n";
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
// VD3 Test 6: Vertical pole → ThinStructureRisk LineSegment
// ---------------------------------------------------------------------------

static bool test_thin_structure_pole() {
    const int W = 21, H = 21;
    const float SCALE   = 1000.0F;
    const float POLE_D  = 5.0F;   // pole at 5 m
    const float BG_D    = 50.0F;  // background at 50 m
    const int   POLE_COL = 10;

    const std::size_t N = static_cast<std::size_t>(W * H);
    std::vector<float> depth_rel(N, SCALE / BG_D);

    // Set pole column
    for (int v = 0; v < H; ++v) {
        depth_rel[static_cast<std::size_t>(v) * static_cast<std::size_t>(W) +
                  static_cast<std::size_t>(POLE_COL)] = SCALE / POLE_D;
    }

    const auto params = make_params(W, H, SCALE, 0.5F);

    std::vector<dedalus::DeviceObstacleEvidence> thin(64U);
    std::uint32_t thin_count = 0U;
    dedalus::detect_thin_structures_device(
        depth_rel.data(), params, thin.data(), thin_count);

    if (thin_count == 0U) {
        std::cerr << "FAIL test_thin_structure_pole: no thin structures detected\n";
        return false;
    }

    const auto& t0 = thin[0];
    if (t0.is_thin_structure_hint != 1U) {
        std::cerr << "FAIL test_thin_structure_pole: is_thin_structure_hint not set\n";
        return false;
    }
    if (t0.state != 3U) {  // ThinStructureRisk
        std::cerr << "FAIL test_thin_structure_pole: wrong state " << +t0.state << "\n";
        return false;
    }
    if (t0.shape != 4U) {  // LineSegment
        std::cerr << "FAIL test_thin_structure_pole: wrong shape " << +t0.shape << "\n";
        return false;
    }

    // The segment must have meaningful length (pole spans full image height)
    if (t0.range_m < 0.5F) {
        std::cerr << "FAIL test_thin_structure_pole: segment too short ("
                  << t0.range_m << " m)\n";
        return false;
    }

    // Verify inflate sets endpoints correctly for LineSegment shape
    const auto inflated = dedalus::inflate(
        &t0, 1U, "test", "test", dedalus::MapFrameId{"m"}, dedalus::TimePoint{0});
    if (inflated.size() != 1U) {
        std::cerr << "FAIL test_thin_structure_pole: inflate returned wrong count\n";
        return false;
    }
    const auto& ie = inflated[0];
    if (ie.shape != dedalus::ObstacleEvidenceShape::LineSegment) {
        std::cerr << "FAIL test_thin_structure_pole: inflated shape wrong\n";
        return false;
    }
    if (!ie.is_thin_structure_hint) {
        std::cerr << "FAIL test_thin_structure_pole: inflated hint not set\n";
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main() {
    int failures = 0;

    auto run = [&](const char* name, bool (*fn)()) {
        if (fn()) {
            std::cout << "PASS " << name << "\n";
        } else {
            ++failures;
        }
    };

    run("test_single_forward_obstacle", test_single_forward_obstacle);
    run("test_depth_range_filter",      test_depth_range_filter);
    run("test_grid_sampling",           test_grid_sampling);
    run("test_inflate_direct",          test_inflate_direct);
    run("test_surface_patch_wall",      test_surface_patch_wall);
    run("test_thin_structure_pole",     test_thin_structure_pole);

    if (failures == 0) {
        std::cout << "OK: all visual_depth_projection tests passed\n";
        return 0;
    }
    std::cerr << failures << " test(s) FAILED\n";
    return 1;
}
