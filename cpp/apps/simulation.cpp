#include "robot_arm/kinematics.hpp"
#include "robot_arm/pathways.hpp"
#include "robot_arm/robot.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <raylib.h>
#include <raymath.h>
#include <vector>

namespace {

constexpr int kWindowWidth = 1000;
constexpr int kWindowHeight = 700;
constexpr int kTargetFramesPerSecond = 60;

constexpr float kCameraMovementSpeed = 3.0f;
constexpr float kCameraMouseSensitivity = 0.8f;
constexpr float kCameraZoomSpeed = 2.0f;
constexpr float kTargetMovementSpeed = 0.6f;
constexpr float kManualJointSpeed = 60.0f * DEG2RAD;
constexpr float kPathRadius = 0.67f;
constexpr float kWaypointTolerance = 0.02f;
constexpr float kReachSafetyFactor = 0.85f;

struct SimulationState {
    // ======== ARM VARIABLES / JOINT SPECS =========
    robot_arm::RobotModel model = robot_arm::createDefaultRobotModel();
    // Initial theta values (update live)
    robot_arm::JointAngles angles = model.homeAngles;
    // Variable for damping IK calc when target surpasses arm reach
    robot_arm::IKSettings ikSettings {};

    // ======== WINDOW AND 3D ========
    Camera3D camera {};
    // Variables for coordinate targeting with the camera
    Vector3 target {0.4f, 0.3f, 0.5f};

    // Variables for arm movement
    std::size_t selectedJoint = 0;
    bool targetMode = false;
    // Toggle the limiting of joints
    bool enforceJointLimits = true;

    // ======= PATHWAY TRACING VARIABLES & SHAPES ========
    robot_arm::PathwayShape pathwayShape = robot_arm::PathwayShape::None;
    // Points for tracing out a pathway
    std::vector<Vector3> pathwayPoints;
    std::size_t currentWaypoint = 0;
    bool pathwayMode = false;

    float maximumApproximateReach = 0.0f;
    float targetDistance = 0.0f;
};

Vector3 flipZY(Vector3 dhPos) 
{
    return {dhPos.x, dhPos.z, dhPos.y};
}

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
    state.camera.position = {2.0f, 2.0f, 2.0f};
    // Point camera is looking at
    state.camera.target = {0.0f, 0.0f, 0.0f};
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
    const Vector2 mouseDelta = GetMouseDelta();
    UpdateCameraPro(
        &camera,
        translation,
        {mouseDelta.x * kCameraMouseSensitivity,
         mouseDelta.y * kCameraMouseSensitivity,
         0.0f},
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
    if (!state.targetMode || state.pathwayMode) {
        return;
    }

    // Moving the target coordinate with the camera
    const float movement = kTargetMovementSpeed * deltaTime;
    if (IsKeyDown(KEY_UP)) state.target.z += movement;
    if (IsKeyDown(KEY_DOWN)) state.target.z -= movement;
    if (IsKeyDown(KEY_Q)) state.target.y += movement;
    if (IsKeyDown(KEY_E)) state.target.y -= movement;
    if (IsKeyDown(KEY_LEFT)) state.target.x += movement;
    if (IsKeyDown(KEY_RIGHT)) state.target.x -= movement;
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
    updateManualJointControl(state, deltaTime);
}

void updateRobot(SimulationState& state)
{
    // ================== CALCULATIONS / FORWARD KINEMATICS ================
    // If target mode is active, the arm automatically moves its joints toward
    // the target. If not, each joint can be controlled manually.

    // Pathway Mode Initiation
    if (state.pathwayMode && !state.pathwayPoints.empty()) {
        state.target = state.pathwayPoints[state.currentWaypoint];
    }

    // Create a fixed position for when the coordinates are out of reach.
    Vector3 reachableTarget = state.target;
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

void drawRobot(const SimulationState& state, const robot_arm::JointPositions& positions)
{
    // ========= DRAWING / TEXT / OUTPUT ==========
    BeginMode3D(state.camera);
    DrawGrid(10, 0.5f);
    // Sphere for target coord
    DrawSphere(flipZY(state.target), 0.03f, RED);

    // Sphere for each joint, cylinder for each arm. Modified varaibles just for the base
    for (std::size_t joint = 0; joint < robot_arm::kJointCount; ++joint) {
        if (joint == 0) 
        {
            DrawCylinderEx(flipZY(positions[joint]), flipZY(Vector3{0.0f, 0.0f, 0.195f}), 0.025f, 0.025f, 30, GRAY);
            DrawCylinderEx(flipZY(positions[joint]), flipZY(positions[joint + 1]), 0.01f, 0.01f, 30, BLACK);
        }
         else 
        {
            DrawCylinderEx(flipZY(positions[joint]), flipZY(positions[joint + 1]), 0.01f, 0.01f, 30, BLACK);
            DrawSphere(flipZY(positions[joint]), 0.02f, DARKBLUE);
        }
    }
    // Last joint has a distinguishable colour (end point of arm)
    DrawSphere(flipZY(positions.back()), 0.03f, GREEN);

    // Pathway trajectory
    for (std::size_t index = 0; index < state.pathwayPoints.size(); ++index) {
        const bool isCurrent = index == state.currentWaypoint;
        DrawSphere(
            flipZY(state.pathwayPoints[index]),
            isCurrent ? 0.025f : 0.015f,
            isCurrent ? YELLOW : GRAY);
    }

    EndMode3D();
}

void drawInterface(const SimulationState& state)
{
    // Text showing user interaction with theta of the joints
    DrawText("6 DOF Robot Arm :: Kinematics Prototype", 10, 10, 20, DARKGRAY);
    DrawText(
        TextFormat("Selected joint: %d (keys 1-6)",
                   static_cast<int>(state.selectedJoint + 1)),
        10,
        40,
        20,
        BLACK);
    DrawText(
        TextFormat("theta[%d] = %.1f degrees",
                   static_cast<int>(state.selectedJoint + 1),
                   state.angles[state.selectedJoint] * RAD2DEG),
        10,
        70,
        20,
        BLACK);
    DrawText(
        TextFormat("Inverse kinematics: %s (0 to toggle)",
                   state.targetMode ? "ACTIVE" : "OFF"),
        10,
        100,
        20,
        RED);
    DrawText(
        TextFormat("Target: %.3f %.3f %.3f; error: %.4f",
                   state.target.x,
                   state.target.y,
                   state.target.z,
                   state.targetDistance),
        10,
        130,
        20,
        BLACK);
    DrawText(
        TextFormat("Pathway mode: %s (V)", state.pathwayMode ? "ON" : "OFF"),
        10,
        160,
        20,
        DARKGREEN);
    DrawText(
        TextFormat("Current shape: %s (C)",
                   robot_arm::pathwayName(state.pathwayShape).data()),
        10,
        190,
        20,
        DARKGREEN);
    DrawText(
        TextFormat("Joint limits: %s (L)",
                   state.enforceJointLimits ? "ON" : "OFF"),
        10,
        220,
        20,
        DARKGREEN);
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
        drawRobot(state, positions);
        drawInterface(state);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
