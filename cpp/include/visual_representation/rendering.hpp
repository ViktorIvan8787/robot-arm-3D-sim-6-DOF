#pragma once 

#include "robot_arm/kinematics.hpp"
#include "simulation_state/simulation_state.hpp"

void drawRobot(
    const SimulationState& state,
    const robot_arm::JointPositions& positions,
    const robot_arm::JointTransforms& transforms
);

void drawInterface(const SimulationState& state);