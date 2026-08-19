#pragma once

#include <array>
#include <cstddef>

namespace robot_arm {

inline constexpr std::size_t kJointCount = 6;

// Create the structure of the DH row for 6 DOF. theta not stored because its live variable. 
// thetaOffset allows the angle to be initially changed so that a -90 degrees in the DH doesn't cause the FK calc to stick it 90 degrees to the side.
struct DHRow {
    float a;
    float alpha;
    float d;
    float thetaOffset;
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
    // DEG2RAD is a built in RayLib variable, but this file doesn't include RayLib lib 
    constexpr float DEG2RAD = pi / 180.0f;

    return {
        // Geometry for all 6 joints (a, alpha, d) First joint is the base,
        // second and third joints are the next two arms, third and fourth
        // joints are the wrist roll and pitch, and the sixth joint is the
        // wrist yaw.
        // Notice how the arms have length along x axis, the others dont.
        // pitch and roll have angles, and so does the base point as it can
        // move around its centre.
        // Fourth value is thetaOffset, not THETA (normally theta in DH). Theta is not included as its a live variable.
        {{
            // {a, alpha, d, theta_offset}
            {0.00f, 90.0f * DEG2RAD, 0.22f, 0.0f * DEG2RAD}, // Joint1 base rotation (19.5 + 2.5 cm)
            {0.15f, 0.0f * DEG2RAD, 0.00f, 90.0f * DEG2RAD}, // Joint2 pitch 
            {0.00f, 90.0f * DEG2RAD, 0.00f, 90.0f * DEG2RAD}, // Joint3 second pitch (a3 used if the arm has a lateral offset)

            {0.00f, -90.0f * DEG2RAD, 0.15f, 0.0f * DEG2RAD}, // Joint4 forearm rotation 
            {0.00f, 90.0f * DEG2RAD, 0.00f, 0.0f * DEG2RAD}, // Joint5 PITCH (for YAW, the arm will rotate Joint4 +-90 degrees)
            {0.00f, 0.0f * DEG2RAD, 0.11f, 0.0f * DEG2RAD}  // Joint6 end claw/suction rotation

        }},
        // Set min and max theta values to represent the joint limits and
        // avoids breaking. Modified for different motors/robot-settings.
        {
            -10000.0f * DEG2RAD, // Infinite (no limit)
            -90.0f * DEG2RAD,
            -150.0f * DEG2RAD,
            -10000.0f * DEG2RAD, // Infinite (no limit)
            -90.0f * DEG2RAD,
            -10000.0f * DEG2RAD, // Infinite (no limit)
        },
        {
            10000.0f * DEG2RAD, // Infinite (no limit)
            90.0f * DEG2RAD,
            150.0f * DEG2RAD,
            10000.0f * DEG2RAD, // Infinite (no limit)
            90.0f * DEG2RAD,
            10000.0f * DEG2RAD, // Infinite (no limit)
        },
        {0.0f * DEG2RAD, -30.0f * DEG2RAD, 90.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, 0.0f * DEG2RAD}, // homeAngles - starting angles position
    };
}

} // namespace robot_arm
