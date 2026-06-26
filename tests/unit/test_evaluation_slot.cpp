// EP1: EvaluationSlotPair agreement metric tests.
//
// Validates each agreement function with synthetic slot A and slot B outputs.
// No real providers or CoreStackRunner needed — tests the free functions directly.

#include <cassert>
#include <cstdio>
#include <vector>

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/runtime/evaluation_slot.hpp"

namespace {

void require(bool cond, const char* msg) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", msg);
        std::exit(1);
    }
}

// ---------------------------------------------------------------------------
// Detection agreement (IoU-based)
// ---------------------------------------------------------------------------

void test_detection_agreement_perfect() {
    dedalus::Rect2 r{0.0, 0.0, 100.0, 100.0};
    dedalus::Detection2D d;
    d.bbox_px = r;

    std::vector<dedalus::Detection2D> a = {d};
    std::vector<dedalus::Detection2D> b = {d};  // identical

    const float ag = dedalus::detection_agreement(a, b);
    require(ag > 0.99F, "identical bboxes should give ~1.0 agreement");
    std::puts("PASS test_detection_agreement_perfect");
}

void test_detection_agreement_no_overlap() {
    dedalus::Detection2D da, db;
    da.bbox_px = dedalus::Rect2{0.0,   0.0,  50.0, 50.0};
    db.bbox_px = dedalus::Rect2{200.0, 0.0,  50.0, 50.0};  // non-overlapping

    std::vector<dedalus::Detection2D> a = {da};
    std::vector<dedalus::Detection2D> b = {db};

    const float ag = dedalus::detection_agreement(a, b);
    require(ag < 0.01F, "non-overlapping bboxes should give 0.0 agreement");
    std::puts("PASS test_detection_agreement_no_overlap");
}

void test_detection_agreement_empty() {
    std::vector<dedalus::Detection2D> empty;
    dedalus::Detection2D d;
    d.bbox_px = dedalus::Rect2{0.0, 0.0, 50.0, 50.0};
    std::vector<dedalus::Detection2D> one = {d};

    require(dedalus::detection_agreement(empty, one) < 0.01F, "empty a → 0");
    require(dedalus::detection_agreement(one, empty) < 0.01F, "empty b → 0");
    std::puts("PASS test_detection_agreement_empty");
}

void test_detection_agreement_partial() {
    // 2 slot-A detections: first matches B, second does not.
    dedalus::Detection2D da1, da2, db;
    da1.bbox_px = dedalus::Rect2{0.0,   0.0, 100.0, 100.0};
    da2.bbox_px = dedalus::Rect2{500.0, 0.0, 100.0, 100.0};
    db.bbox_px  = dedalus::Rect2{0.0,   0.0, 100.0, 100.0};  // matches da1 only

    std::vector<dedalus::Detection2D> a = {da1, da2};
    std::vector<dedalus::Detection2D> b = {db};

    const float ag = dedalus::detection_agreement(a, b);
    require(ag > 0.49F && ag < 0.51F, "1/2 matched → ~0.5 agreement");
    std::puts("PASS test_detection_agreement_partial");
}

// ---------------------------------------------------------------------------
// Stabilizer agreement
// ---------------------------------------------------------------------------

void test_stabilizer_agreement_identical() {
    dedalus::StabilizerOutput a{true, 5.0, 3.0};
    dedalus::StabilizerOutput b{true, 5.0, 3.0};
    const float ag = dedalus::stabilizer_agreement(a, b);
    require(ag > 0.99F, "identical stabilizer outputs → 1.0");
    std::puts("PASS test_stabilizer_agreement_identical");
}

void test_stabilizer_agreement_max_delta() {
    dedalus::StabilizerOutput a{true, 50.0, 0.0};
    dedalus::StabilizerOutput b{true, 0.0, 0.0};  // delta = 50 = max → 0
    const float ag = dedalus::stabilizer_agreement(a, b, /*max_delta_px=*/50.0);
    require(ag < 0.01F, "delta at max → 0.0");
    std::puts("PASS test_stabilizer_agreement_max_delta");
}

void test_stabilizer_agreement_unavailable() {
    dedalus::StabilizerOutput a{false, 0.0, 0.0};
    dedalus::StabilizerOutput b{true,  0.0, 0.0};
    require(dedalus::stabilizer_agreement(a, b) < 0.01F, "unavailable transform → 0");
    std::puts("PASS test_stabilizer_agreement_unavailable");
}

// ---------------------------------------------------------------------------
// Tracker agreement (class-label + centroid)
// ---------------------------------------------------------------------------

void test_tracker_agreement_match() {
    dedalus::Track2D ta, tb;
    ta.bbox_px     = dedalus::Rect2{0.0, 0.0, 100.0, 100.0};
    ta.class_label = dedalus::ClassLabel::Person;
    tb.bbox_px     = dedalus::Rect2{5.0, 5.0, 100.0, 100.0};  // close centroid
    tb.class_label = dedalus::ClassLabel::Person;

    const float ag = dedalus::tracker_agreement({ta}, {tb});
    require(ag > 0.99F, "same class + close centroid → 1.0");
    std::puts("PASS test_tracker_agreement_match");
}

void test_tracker_agreement_class_mismatch() {
    dedalus::Track2D ta, tb;
    ta.bbox_px     = dedalus::Rect2{0.0, 0.0, 100.0, 100.0};
    ta.class_label = dedalus::ClassLabel::Person;
    tb.bbox_px     = dedalus::Rect2{0.0, 0.0, 100.0, 100.0};
    tb.class_label = dedalus::ClassLabel::Car;

    const float ag = dedalus::tracker_agreement({ta}, {tb});
    require(ag < 0.01F, "class mismatch → 0.0");
    std::puts("PASS test_tracker_agreement_class_mismatch");
}

// ---------------------------------------------------------------------------
// Observation agreement (3D position)
// ---------------------------------------------------------------------------

void test_observation_agreement_close() {
    dedalus::Observation3D oa, ob;
    oa.position_local = dedalus::Vec3{1.0, 2.0, 3.0};
    ob.position_local = dedalus::Vec3{1.1, 2.0, 3.0};  // 0.1 m apart

    const float ag = dedalus::observation_agreement({oa}, {ob}, /*threshold_m=*/1.0);
    require(ag > 0.99F, "0.1 m apart → 1.0 agreement");
    std::puts("PASS test_observation_agreement_close");
}

void test_observation_agreement_far() {
    dedalus::Observation3D oa, ob;
    oa.position_local = dedalus::Vec3{0.0, 0.0, 0.0};
    ob.position_local = dedalus::Vec3{10.0, 0.0, 0.0};  // 10 m apart

    const float ag = dedalus::observation_agreement({oa}, {ob}, /*threshold_m=*/1.0);
    require(ag < 0.01F, "10 m apart → 0.0 agreement");
    std::puts("PASS test_observation_agreement_far");
}

// ---------------------------------------------------------------------------
// EvaluationSlotPair: has_reference()
// ---------------------------------------------------------------------------

void test_evaluation_slot_pair() {
    dedalus::EvaluationSlotPair<dedalus::Detector> pair;
    require(!pair.has_reference(), "null reference → has_reference() == false");

    pair.reference = std::make_shared<dedalus::ScriptedDetector>();
    require(pair.has_reference(), "non-null reference → has_reference() == true");
    std::puts("PASS test_evaluation_slot_pair");
}

}  // namespace

int main() {
    test_detection_agreement_perfect();
    test_detection_agreement_no_overlap();
    test_detection_agreement_empty();
    test_detection_agreement_partial();
    test_stabilizer_agreement_identical();
    test_stabilizer_agreement_max_delta();
    test_stabilizer_agreement_unavailable();
    test_tracker_agreement_match();
    test_tracker_agreement_class_mismatch();
    test_observation_agreement_close();
    test_observation_agreement_far();
    test_evaluation_slot_pair();
    std::puts("All evaluation_slot tests passed.");
    return 0;
}
