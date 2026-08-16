#pragma once

#include <raylib.h>

#include <string_view>
#include <vector>

namespace robot_arm {

enum class PathwayShape {
    None,
    PickAndPlace,
    Cube,
    Circle,
    Pringle,
};

std::vector<Vector3> createPathwayPoints(PathwayShape shape, float radius);
std::string_view pathwayName(PathwayShape shape);
PathwayShape nextPathway(PathwayShape shape);

} // namespace robot_arm
