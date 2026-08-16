#include "robot_arm/pathways.hpp"

#include <cmath>
#include <raymath.h>

namespace robot_arm {

std::vector<Vector3> createPathwayPoints(PathwayShape shape, float radius)
{
    // Function applies different pathways once a different shape is toggled.
    // Initialize points.
    switch (shape) {
    case PathwayShape::None:
        // None
        return {};

    case PathwayShape::PickAndPlace:
        // Pick and Place pathway
        return {
            {0.4f, 0.0f, 0.0f},
            {0.4f, 0.0f, 0.4f},
            {0.0f, 0.4f, 0.4f},
            {0.0f, 0.4f, 0.0f},
            {0.0f, 0.4f, 0.4f},
            {-0.4f, 0.0f, 0.4f},
            {-0.4f, 0.0f, 0.0f},
            {-0.4f, 0.0f, 0.4f},
        };

    case PathwayShape::Cube:
        // Cube
        return {
            {-0.3f, 0.3f, 0.3f},
            {-0.3f, -0.3f, 0.3f},
            {-0.3f, -0.3f, -0.3f},
            {0.3f, -0.3f, -0.3f},
            {-0.3f, 0.3f, -0.3f},
            {0.3f, -0.3f, 0.3f},
            {0.3f, 0.3f, -0.3f},
            {0.3f, 0.3f, 0.3f},
        };

    case PathwayShape::Circle:
    case PathwayShape::Pringle: {
        // Circle and Pringle pathways are built from evenly spaced angles.
        constexpr int pointCount = 100;
        std::vector<Vector3> points;
        points.reserve(pointCount);

        for (int index = 0; index < pointCount; ++index) {
            const float angleDegrees = 360.0f * static_cast<float>(index) /
                static_cast<float>(pointCount);
            const float angleRadians = angleDegrees * DEG2RAD;
            const float x = radius * std::cos(angleRadians);
            const float y = radius * std::sin(angleRadians);
            const float z = shape == PathwayShape::Pringle ? x * x : 0.0f;
            points.push_back({x, y, z});
        }

        return points;
    }
    }

    return {};
}

std::string_view pathwayName(PathwayShape shape)
{
    switch (shape) {
    case PathwayShape::None:
        return "None";
    case PathwayShape::PickAndPlace:
        return "Pick-and-Place";
    case PathwayShape::Cube:
        return "Cube";
    case PathwayShape::Circle:
        return "Circle";
    case PathwayShape::Pringle:
        return "Pringle";
    }

    return "Unknown";
}

PathwayShape nextPathway(PathwayShape shape)
{
    switch (shape) {
    case PathwayShape::None:
        return PathwayShape::PickAndPlace;
    case PathwayShape::PickAndPlace:
        return PathwayShape::Cube;
    case PathwayShape::Cube:
        return PathwayShape::Circle;
    case PathwayShape::Circle:
        return PathwayShape::Pringle;
    case PathwayShape::Pringle:
        return PathwayShape::None;
    }

    return PathwayShape::None;
}

} // namespace robot_arm
