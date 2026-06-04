#include <cmath>
#include <iostream>
#include <vector>

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/world_model/in_memory_world_model.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

bool near(double lhs, double rhs, double tolerance = 1.0e-9) {
    return std::abs(lhs - rhs) <= tolerance;
}

dedalus::ObstacleS