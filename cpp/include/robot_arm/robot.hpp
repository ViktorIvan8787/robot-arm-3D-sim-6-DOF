#pragma once

#include <array>
#include <cstddef>

namespace robot_arm {

inline constexpr std::size_t kJointCount = 6;

// Create the structure of the DH row for 6 DOF. theta not stored because its live variable
struct DHRow {
    float a;
    float alpha;
    float d;
};

using JointAngles = std::array<float, kJointCount>;
using DHTable = std::array<DHRow, kJointCount>;

struct RobotModel {
    DHTable joints;
    JointAngles minimumAngles;
    JointAngles maximumAngles;
    JointAngles homeAngles;
};

// Central definition of the current prototype. Replace these values with
// measured dimensions and limits when the mechanical design is final.
inline RobotModel createDefaultRobotModel()
{
    constexpr float pi = 3.14159265358979323846f;
    constexpr float degreesToRadians = pi / 180.0f;

    return {
        // Geometry for all 6 joints (a, alpha, d) First joint is the base,
        // second and third joints are the next two arms, third and fourth
        // joints are the wrist roll and pitch, and the sixth joint is the
        // wrist yaw.
        // Notice how the arms have length along x axis, the others dont.
        // pitch and roll have angles, and so does the base point as it can
        // move around its centre.
        {{
            {0.0f, 90.0f * degreesToRadians, 0.3f},
            {0.3f, 0.0f, 0.0f},
            {0.3f, 0.0f, 0.0f},
            {0.0f, 90.0f * degreesToRadians, 0.2f},
            {0.0f, -90.0f * degreesToRadians, 0.0f},
            {0.0f, 0.0f, 0.1f},
        }},
        // Set min and max theta values to represent the joint limits and
        // avoids breaking. Can be modified for different motors/robot-settings.
        // CURRENT: base values for kuka robot
        {
            -180.0f * degreesToRadians,
            -90.0f * degreesToRadians,
            -150.0f * degreesToRadians,
            -180.0f * degreesToRadians,
            -120.0f * degreesToRadians,
            -180.0f * degreesToRadians,
        },
        {
            180.0f * degreesToRadians,
            90.0f * degreesToRadians,
            150.0f * degreesToRadians,
            180.0f * degreesToRadians,
            120.0f * degreesToRadians,
            180.0f * degreesToRadians,
        },
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    };
}

} // namespace robot_arm
