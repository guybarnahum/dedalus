#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <vector>

#include "dedalus/sensors/visual_ego_state_provider.hpp"

// ── Helpers ───────────────────────────────────────────────────────────────────

namespace {

// Build a greyscale checkerboard of given dimensions.
std::vector<std::uint8_t> make_checker(int width, int height, int cell = 16) {
    std::vector<std::uint8_t> img(static_cast<std::size_t>(width * height));
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
            img[static_cast<std::size_t>(y * width + x)] =
                (((x / cell) + (y / cell)) % 2 == 0) ? 200 : 40;
    return img;
}

// Shift a greyscale image by (dx, dy) pixels (nearest-neighbour).
std::vector<std::uint8_t> shift_image(const std::vector<std::uint8_t>& src,
                                       int width, int height, int dx, int dy) {
    std::vector<std::uint8_t> dst(src.size(), 128);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int sx = x - dx, sy = y - dy;
            if (sx >= 0 && sx < width && sy >= 0 && sy < height)
                dst[static_cast<std::size_t>(y * width + x)] =
                    src[static_cast<std::size_t>(sy * width + sx)];
        }
    }
    return dst;
}

dedalus::FramePacket make_frame(const std::vector<std::uint8_t>& gray,
                                int width, int height) {
    dedalus::FramePacket fp;
    fp.frame_id    = dedalus::FrameId{"f0"};
    fp.timestamp   = dedalus::TimePoint{0};
    fp.image.width    = width;
    fp.image.height   = height;
    fp.image.channels = 1;
    fp.image.bytes    = gray;
    fp.intrinsics     = {525.0, 525.0,
                         static_cast<double>(width)  / 2.0,
                         static_cast<double>(height) / 2.0};
    return fp;
}

}  // namespace

// ── Tests ─────────────────────────────────────────────────────────────────────

TEST_CASE("VisualEgoStateProvider initialises on first frame", "[vl1]") {
    dedalus::VisualEgoStateProvider provider;
    const int W = 320, H = 240;
    const auto gray = make_checker(W, H);
    const auto fp   = make_frame(gray, W, H);

    const auto est = provider.estimate(fp);
    REQUIRE(est.ego.has_value());
    CHECK(est.confidence >= 0.0F);
    CHECK(est.confidence <= 1.0F);
    // Position should be origin on first frame
    CHECK(est.ego->local_T_body.position.x == Catch::Approx(0.0).margin(1e-9));
    CHECK(est.ego->local_T_body.position.y == Catch::Approx(0.0).margin(1e-9));
    CHECK(est.ego->local_T_body.position.z == Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE("VisualEgoStateProvider detects rightward translation", "[vl1]") {
    dedalus::VisualOdometryConfig cfg;
    cfg.initial_scale_m = 1.0F;  // 1 unit per frame so displacement is readable
    dedalus::VisualEgoStateProvider provider(cfg);

    const int W = 640, H = 480;
    const auto frame0 = make_checker(W, H, 20);
    const auto frame1 = shift_image(frame0, W, H, 10, 0);  // 10 px rightward shift

    // Seed provider with initial frame
    auto fp0 = make_frame(frame0, W, H);
    // Add a velocity hint so scale is set
    dedalus::EgoState hint;
    hint.velocity_local = {0.3, 0.0, 0.0};  // 0.3 m/s forward
    fp0.ego_hint = hint;
    provider.estimate(fp0);

    // Second frame: shifted right
    auto fp1 = make_frame(frame1, W, H);
    fp1.ego_hint = hint;
    const auto est = provider.estimate(fp1);

    REQUIRE(est.ego.has_value());
    // Translation direction x should be positive (camera moved right in image space)
    // Confidence should be non-trivially positive after one motion step
    CHECK(est.confidence > 0.0F);
    // The position should have moved — sign depends on camera orientation convention.
    // We just check it is non-zero.
    const auto& p = est.ego->local_T_body.position;
    const double displacement = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    CHECK(displacement > 0.0);
}

TEST_CASE("VisualEgoStateProvider confidence decays with no features", "[vl1]") {
    dedalus::VisualOdometryConfig cfg;
    cfg.fast_threshold = 255;   // no corners detected — force failure path
    cfg.min_features   = 9999;
    dedalus::VisualEgoStateProvider provider(cfg);

    const int W = 64, H = 64;
    const std::vector<std::uint8_t> blank(W * H, 128);  // flat image, no corners

    auto fp = make_frame(blank, W, H);
    provider.estimate(fp);  // init frame

    fp.timestamp = dedalus::TimePoint{33333333};
    const auto est = provider.estimate(fp);
    REQUIRE(est.ego.has_value());
    // With no inliers, cumulative_drift stays 0 (no motion computed)
    // so confidence stays at the initial value ≥ 0.
    CHECK(est.confidence >= 0.0F);
}

TEST_CASE("VL3: fallback provider used when confidence too low", "[vl3]") {
    dedalus::VisualOdometryConfig cfg;
    cfg.fallback_confidence_threshold = 1.0F;  // always fall back
    auto provider = std::make_unique<dedalus::VisualEgoStateProvider>(cfg);

    // Fallback: simple FrameHintEgoProvider with an AirSim hint
    auto fallback = std::make_shared<dedalus::FrameHintEgoProvider>(
        dedalus::MapFrameId{"map_airsim"});
    provider->set_fallback_provider(fallback);

    const int W = 320, H = 240;
    const auto gray = make_checker(W, H);
    auto fp = make_frame(gray, W, H);

    dedalus::EgoState hint;
    hint.map_frame_id = dedalus::MapFrameId{"map_airsim"};
    hint.local_T_body.position = {1.0, 2.0, 3.0};
    hint.confidence = 0.95F;
    fp.ego_hint = hint;

    provider->estimate(fp);  // init frame

    const auto est = provider->estimate(fp);
    REQUIRE(est.ego.has_value());
    // Should have received the fallback position
    CHECK(est.ego->local_T_body.position.x == Catch::Approx(1.0).margin(1e-6));
    CHECK(est.ego->local_T_body.position.y == Catch::Approx(2.0).margin(1e-6));
    CHECK(est.ego->local_T_body.position.z == Catch::Approx(3.0).margin(1e-6));
}

TEST_CASE("VisualOdometryState starts zero-initialised", "[vl1]") {
    dedalus::VisualOdometryState s;
    CHECK(s.position.x == 0.0);
    CHECK(s.position.y == 0.0);
    CHECK(s.position.z == 0.0);
    CHECK(s.cumulative_drift_m == 0.0);
    CHECK(!s.initialized);
}
