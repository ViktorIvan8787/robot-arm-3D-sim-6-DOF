#pragma once

#include "robot_arm/robot.hpp"

#include <array>
#include <raylib.h>

namespace robot_arm {

using JointPositions = std::array<Vector3, kJointCount + 1>;
using JointTransforms = std::array<Matrix, kJointCount + 1>;

struct IKSettings {
    float minimumDamping = 0.01f;
    float maximumDamping = 0.30f;
    float singularityThreshold = 0.05f;
    float maximumStepRadians = 0.02f;
};

Matrix dhTransform(float theta, float d, float a, float alpha);

void forwardKinematics(
    const JointAngles& angles,
    const RobotModel& model,
    JointPositions& positions,
    JointTransforms& transforms);

// Performs one damped least-squares position-only IK iteration. This does not
// solve TCP orientation and is not yet a complete trajectory planner.
float performIKStep(
    JointAngles& angles,
    const RobotModel& model,
    Vector3 target,
    const IKSettings& settings,
    bool enforceJointLimits);

} // namespace robot_arm
