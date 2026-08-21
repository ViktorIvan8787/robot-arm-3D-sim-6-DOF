#include "robot_arm/kinematics.hpp"
#include "robot_arm/pathways.hpp"
#include "robot_arm/robot.hpp"
#include "simulation_assets/simulation_state.hpp"
#include "simulation_assets/rendering.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <raylib.h>
#include <raymath.h>
#include <vector>
#include <iostream>

namespace {

constexpr int kWindowWidth = 1000;
constexpr int kWindowHeight = 700;
constexpr int kTargetFramesPerSecond = 60;

constexpr float kCameraMovementSpeed = 3.0f;
constexpr float kCameraMouseSensitivity = 0.7;
constexpr float kCameraZoomSpeed = 2.0f;
constexpr float kTargetMovementSpeed = 0.6f;
constexpr float kOrientationStep = 45.0f * DEG2RAD;
constexpr float kManualJointSpeed = 60.0f * DEG2RAD;
constexpr float kPathRadius = 0.67f;
constexpr float kWaypointTolerance = 0.02f;
constexpr float kReachSafetyFactor = 0.85f;


float estimateMaximumReach(const robot_arm::RobotModel& model)
{
    // Variable for maximum reach. Used to stop arm movement beyond physically
    // capable. This remains an approximate visual guard, not a safety test.
    float total = 0.0f;
    for (const robot_arm::DHRow& joint : model.joints) {
        total += std::fabs(joint.a) + std::fabs(joint.d);
    }
    return total * kReachSafetyFactor;
}

void initialiseSimulation(SimulationState& state)
{
    // Setting up 3D camera
    // Where camera is in space
    state.camera.position = {1.0f, 1.0f, 1.0f};
    // Point camera is looking at
    state.camera.target = {0.0f, 0.5f, 0.0f};
    // For the camera, y axis is upwards
    state.camera.up = {0.0f, 1.0f, 0.0f};
    // Field of view
    state.camera.fovy = 50.0f;
    state.camera.projection = CAMERA_PERSPECTIVE;
    state.maximumApproximateReach = estimateMaximumReach(state.model);
}

void updateViewCamera(Camera3D& camera, float deltaTime)
{
    // ======== CAMERA INTERACTION ==========
    // Movement of camera from WASD keys. It is multiplied by deltaTime so the
    // speed does not depend on the number of frames drawn each second.
    const float movement = kCameraMovementSpeed * deltaTime;
    Vector3 translation {0.0f, 0.0f, 0.0f};

    if (IsKeyDown(KEY_W)) translation.x += movement;
    if (IsKeyDown(KEY_S)) translation.x -= movement;
    if (IsKeyDown(KEY_D)) translation.y += movement;
    if (IsKeyDown(KEY_A)) translation.y -= movement;
    if (IsKeyDown(KEY_SPACE)) translation.z += movement;
    if (IsKeyDown(KEY_LEFT_SHIFT)) translation.z -= movement;

    // Sensitivity of the mouse in the camera
    Vector2 rotation {0.0f, 0.0f};
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        const Vector2 mouseDelta = GetMouseDelta();
        rotation.x = mouseDelta.x * kCameraMouseSensitivity;
        rotation.y = mouseDelta.y * kCameraMouseSensitivity;
        DisableCursor();
    } if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) {
        EnableCursor();
    }

    UpdateCameraPro(
        &camera,
        translation,
        {rotation.x, rotation.y, 0.0f},
        GetMouseWheelMove() * kCameraZoomSpeed);
}

void selectJointFromKeyboard(SimulationState& state)
{
    // Input for which joint to be moved, using keys 1-6 to select
    if (IsKeyPressed(KEY_ONE)) state.selectedJoint = 0;
    if (IsKeyPressed(KEY_TWO)) state.selectedJoint = 1;
    if (IsKeyPressed(KEY_THREE)) state.selectedJoint = 2;
    if (IsKeyPressed(KEY_FOUR)) state.selectedJoint = 3;
    if (IsKeyPressed(KEY_FIVE)) state.selectedJoint = 4;
    if (IsKeyPressed(KEY_SIX)) state.selectedJoint = 5;
}

void updateTargetFromKeyboard(SimulationState& state, float deltaTime)
{
    // ========= TARGET INTERACTION ==========

    // Moving the targetPosition coordinate with the camera
    const float movement = kTargetMovementSpeed * deltaTime;
    if (IsKeyDown(KEY_E)) state.targetPosition.z += movement;
    if (IsKeyDown(KEY_Q)) state.targetPosition.z -= movement;
    if (IsKeyDown(KEY_LEFT)) state.targetPosition.y += movement;
    if (IsKeyDown(KEY_RIGHT)) state.targetPosition.y -= movement;
    if (IsKeyDown(KEY_DOWN)) state.targetPosition.x += movement;
    if (IsKeyDown(KEY_UP)) state.targetPosition.x -= movement;
}

void updateTargetOrientationFromKeyboard(SimulationState& state) 
{
    // ========= TARGET ORIENTATION USER INTERACTION =============
    // Keys will alter orientation by kOrientationStep (current 45deg)
    // once key is pressed. Helps appraoch objects from side

    if (IsKeyPressed(KEY_MINUS)) state.targetOrientation = MatrixMultiply(state.targetOrientation, MatrixRotateY(kOrientationStep));
    if (IsKeyPressed(KEY_EQUAL)) state.targetOrientation = MatrixMultiply(state.targetOrientation, MatrixRotateY(-kOrientationStep));

    if (IsKeyPressed(KEY_LEFT_BRACKET)) state.targetOrientation = MatrixMultiply(state.targetOrientation, MatrixRotateX(kOrientationStep));
    if (IsKeyPressed(KEY_RIGHT_BRACKET)) state.targetOrientation = MatrixMultiply(state.targetOrientation, MatrixRotateX(-kOrientationStep));

    if (IsKeyPressed(KEY_APOSTROPHE)) state.targetOrientation = MatrixMultiply(state.targetOrientation, MatrixRotateZ(kOrientationStep));
    if (IsKeyPressed(KEY_BACKSLASH)) state.targetOrientation = MatrixMultiply(state.targetOrientation, MatrixRotateZ(-kOrientationStep));
}

void updateManualJointControl(SimulationState& state, float deltaTime)
{
    // ========= MOVEMENT INTERACTION ==========
    // Manual and IK control are deliberately mutually exclusive.
    if (state.targetMode) {
        return;
    }

    // Increase or decrease theta for keys held down. O and P are used for
    // accessibility. Joint limits are applied only when the mode is enabled.
    const float movement = kManualJointSpeed * deltaTime;
    if (IsKeyDown(KEY_O)) state.angles[state.selectedJoint] += movement;
    if (IsKeyDown(KEY_P)) state.angles[state.selectedJoint] -= movement;

    if (state.enforceJointLimits) {
        state.angles[state.selectedJoint] = std::clamp(
            state.angles[state.selectedJoint],
            state.model.minimumAngles[state.selectedJoint],
            state.model.maximumAngles[state.selectedJoint]);
    }
}

void updatePathwaySelection(SimulationState& state)
{
    // ========= PATHWAY INTERACTION =========
    if (IsKeyPressed(KEY_C)) {
        state.pathwayShape = robot_arm::nextPathway(state.pathwayShape);
        state.pathwayPoints =
            robot_arm::createPathwayPoints(state.pathwayShape, kPathRadius);
        state.currentWaypoint = 0;

        if (state.pathwayPoints.empty()) {
            state.pathwayMode = false;
        }
    }

    if (IsKeyPressed(KEY_V) && !state.pathwayPoints.empty()) {
        state.pathwayMode = !state.pathwayMode;
        if (state.pathwayMode) {
            state.targetMode = true;
        }
    }
}

void handleInput(SimulationState& state, float deltaTime)
{
    updateViewCamera(state.camera, deltaTime);
    selectJointFromKeyboard(state);

    // Target mode controlled by button 0
    if (IsKeyPressed(KEY_ZERO)) {
        state.targetMode = !state.targetMode;
        if (!state.targetMode) {
            state.pathwayMode = false;
        }
    }

    // ========= LIMITING JOINTS TOGGLE ===========
    if (IsKeyPressed(KEY_L)) {
        state.enforceJointLimits = !state.enforceJointLimits;
    }

    updatePathwaySelection(state);
    updateTargetFromKeyboard(state, deltaTime);
    updateTargetOrientationFromKeyboard(state);
    updateManualJointControl(state, deltaTime);
}

void updateRobot(SimulationState& state)
{
    // ================== CALCULATIONS / FORWARD KINEMATICS ================
    // If targetPosition mode is active, the arm automatically moves its joints toward
    // the targetPosition. If not, each joint can be controlled manually.

    // Pathway Mode Initiation
    if (state.pathwayMode && !state.pathwayPoints.empty()) {
        state.targetPosition = state.pathwayPoints[state.currentWaypoint];
    }

    // Create a fixed position for when the coordinates are out of reach.
    Vector3 reachableTarget = state.targetPosition;
    // If the coordinate is larger than the approximate maximum arm reach, it
    // is normalised and scaled so calculations continue at a reachable point.
    if (Vector3Length(reachableTarget) > state.maximumApproximateReach) {
        reachableTarget = Vector3Scale(
            Vector3Normalize(reachableTarget), state.maximumApproximateReach);
    }

    // Main calc
    state.targetDistance = 0.0f;
    if (state.targetMode) {
        state.targetDistance = robot_arm::performIKStep(
            state.angles,
            state.model,
            reachableTarget,
            state.targetOrientation,
            state.ikSettings,
            state.enforceJointLimits);
    }

    if (state.pathwayMode &&
        state.targetDistance < kWaypointTolerance &&
        !state.pathwayPoints.empty()) {
        state.currentWaypoint =
            (state.currentWaypoint + 1) % state.pathwayPoints.size();
    }
}



} // namespace


int main()
{
    // Window
    InitWindow(kWindowWidth, kWindowHeight, "6 DOF Robot Arm");
    SetTargetFPS(kTargetFramesPerSecond);

    SimulationState state;
    initialiseSimulation(state);

    // ======= MAIN 3D LOOP ========
    while (!WindowShouldClose()) {
        const float deltaTime = GetFrameTime();
        handleInput(state, deltaTime);
        updateRobot(state);

        robot_arm::JointPositions positions {};
        robot_arm::JointTransforms transforms {};
        robot_arm::forwardKinematics(state.angles, state.model, positions, transforms);

        BeginDrawing();
        ClearBackground(RAYWHITE);
        drawRobot(state, positions, transforms);
        drawInterface(state);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
