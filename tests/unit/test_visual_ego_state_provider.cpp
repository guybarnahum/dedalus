// VL1-VL3: VisualEgoStateProvider unit tests.
//
// Tests:
//   1. Initialises to origin pose on first frame.
//   2. Detects non-zero displacement from a shifted image pair.
//   3. Confidence stays in [0,1] with no trackable features.
//   4. VL3 fallback provider returns hint pose when confidence threshold forces fallback.
//   5. VisualOdometryState zero-initialised by default.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "dedalus/sensors/visual_ego_state_provider.hpp"

// ── Helpers ───────────────────────────────────────────────────────────────────

namespace {

std::vector<std::uint8_t> make_checker(int width, int height, int cell = 16) {
    std::vector<std::uint8_t> img(static_cast<std::size_t>(width * height));
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
            img[static_cast<std::size_t>(y * width + x)] =
                (((x / cell) + (y / cell)) % 2 == 0) ? 200 : 40;
    return img;
}

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
    fp.frame_id       = dedalus::FrameId{"f0"};
    fp.timestamp      = dedalus::TimePoint{0};
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

static void test_init_origin() {
    dedalus::VisualEgoStateProvider provider;
    const int W = 320, H = 240;
    const auto fp  = make_frame(make_checker(W, H), W, H);
    const auto est = provider.estimate(fp);

    assert(est.ego.has_value());
    assert(est.confidence >= 0.0F && est.confidence <= 1.0F);
    assert(std::abs(est.ego->local_T_body.position.x) < 1e-9);
    assert(std::abs(est.ego->local_T_body.position.y) < 1e-9);
    assert(std::abs(est.ego->local_T_body.position.z) < 1e-9);
    std::puts("PASS test_init_origin");
}

static void test_rightward_translation() {
    dedalus::VisualOdometryConfig cfg;
    cfg.initial_scale_m = 1.0F;
    dedalus::VisualEgoStateProvider provider(cfg);

    const int W = 640, H = 480;
    const auto frame0 = make_checker(W, H, 20);
    const auto frame1 = shift_image(frame0, W, H, 10, 0);

    dedalus::EgoState hint;
    hint.velocity_local = {0.3, 0.0, 0.0};

    auto fp0 = make_frame(frame0, W, H);
    fp0.ego_hint = hint;
    provider.estimate(fp0);

    auto fp1 = make_frame(frame1, W, H);
    fp1.ego_hint = hint;
    const auto est = provider.estimate(fp1);

    assert(est.ego.has_value());
    assert(est.confidence >= 0.0F);
    const auto& p = est.ego->local_T_body.position;
    const double d = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    assert(d > 0.0);
    std::puts("PASS test_rightward_translation");
}

static void test_no_features_confidence() {
    dedalus::VisualOdometryConfig cfg;
    cfg.fast_threshold = 255;
    cfg.min_features   = 9999;
    dedalus::VisualEgoStateProvider provider(cfg);

    const int W = 64, H = 64;
    const std::vector<std::uint8_t> blank(W * H, 128);

    auto fp = make_frame(blank, W, H);
    provider.estimate(fp);

    fp.timestamp = dedalus::TimePoint{33333333};
    const auto est = provider.estimate(fp);

    assert(est.ego.has_value());
    assert(est.confidence >= 0.0F && est.confidence <= 1.0F);
    std::puts("PASS test_no_features_confidence");
}

static void test_vl3_fallback() {
    dedalus::VisualOdometryConfig cfg;
    cfg.fallback_confidence_threshold = 1.0F;  // always fall back
    auto provider = std::make_unique<dedalus::VisualEgoStateProvider>(cfg);

    auto fallback = std::make_shared<dedalus::FrameHintEgoProvider>(
        dedalus::MapFrameId{"map_airsim"});
    provider->set_fallback_provider(fallback);

    const int W = 320, H = 240;
    auto fp = make_frame(make_checker(W, H), W, H);

    dedalus::EgoState hint;
    hint.map_frame_id              = dedalus::MapFrameId{"map_airsim"};
    hint.local_T_body.position     = {1.0, 2.0, 3.0};
    hint.confidence                = 0.95F;
    fp.ego_hint = hint;

    provider->estimate(fp);  // init
    const auto est = provider->estimate(fp);

    assert(est.ego.has_value());
    assert(std::abs(est.ego->local_T_body.position.x - 1.0) < 1e-6);
    assert(std::abs(est.ego->local_T_body.position.y - 2.0) < 1e-6);
    assert(std::abs(est.ego->local_T_body.position.z - 3.0) < 1e-6);
    std::puts("PASS test_vl3_fallback");
}

static void test_state_zero_init() {
    dedalus::VisualOdometryState s;
    assert(s.position.x == 0.0);
    assert(s.position.y == 0.0);
    assert(s.position.z == 0.0);
    assert(s.cumulative_drift_m == 0.0);
    assert(!s.initialized);
    std::puts("PASS test_state_zero_init");
}

int main() {
    test_init_origin();
    test_rightward_translation();
    test_no_features_confidence();
    test_vl3_fallback();
    test_state_zero_init();
    std::puts("All visual_ego_state_provider tests passed.");
    return 0;
}
