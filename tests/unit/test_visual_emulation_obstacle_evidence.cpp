#include <cmath>
#include <iostream>
#include <vector>

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/world_model/in_memory_world_model.hpp"

namespace {

bool near(double lhs, double rhs) {
    return std::abs(lhs - rhs) < 1.0e-9;
}

dedalus::ObstacleSensingVolume explicit_front_coverage(
    dedalus::TimePoint timestamp,
    const dedalus::MapFrameId&