#pragma once

#include "robot_arm/kinematics.hpp"
#include "robot_arm/pathways.hpp"
#include "robot_arm/robot.hpp"

#include <cstddef>
#include <raylib.h>
#include <vector>

struct SimulationState {
    // ========= ARM VARAIBLES / JOINT SPECS =========
    robot_arm::RobotModel model = robot_arm::createDefaultRobotModel();
    // Initial theta values (they update live, start at homeAngles)
    robot_arm::JointAngles angles = model.homeAngles;
    // Varaible for damping IK calc when target surpasses arm reach
    robot_arm:: IKSettings ikSettings {};
    float maximumApproximateReach = 0.0f;

    // ========= WINDOW / 3D ==========
    Camera3D camera {};

    // ========= TARGET / IK TRACKING ========
    // Robot target coordinates (user interactable)
    Vector3 target {-0.3f, 0.0f, 0.3f};
    bool targetMode = false;
    float targetDistance = 0.0f;

    // ========== ARM USER CONTROL ==========
    std::size_t selectedJoint = 0;
    bool enforceJointLimits = true; // (angular)

    // ========= PATHWAY TRACING VARIABLES & SHAPES =========
    // (Starts at none)
    robot_arm::PathwayShape pathwayShape = robot_arm::PathwayShape::None;
    std::vector<Vector3> pathwayPoints;
    std::size_t currentWaypoint = 0;
    bool pathwayMode = false;
};