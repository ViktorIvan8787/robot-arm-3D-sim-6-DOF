#include "simulation_assets/rendering.hpp"

#include <raylib.h>
#include <raymath.h>

namespace {
// Flip Z-Y coordinates as raylib uses y as up on default but robot kinematics use z as up
Vector3 flipZY(Vector3 dhPos) {return {dhPos.x, dhPos.z, dhPos.y};}
} // namespace



// ==================================== GRIPPER ==========================================

void drawGripper(Vector3 tip, Matrix tipTransform) // Credit to Claude Sonnet 5 AI for the design (used for visual implementation)
{
    // Extract the gripper's true forward (approach) and sideways axes
    // directly from the accumulated joint transform, so claws follow
    // every joint's rotation, including roll/yaw at the wrist.
    Vector3 origin {0.0f, 0.0f, 0.0f};
    Vector3 localForward {0.0f, 0.0f, 1.0f};
    Vector3 localSideways {1.0f, 0.0f, 0.0f};

    Vector3 approachDirection = Vector3Normalize(
        Vector3Subtract(Vector3Transform(localForward, tipTransform), Vector3Transform(origin, tipTransform))
        );
    Vector3 sideways = Vector3Normalize(
        Vector3Subtract(Vector3Transform(localSideways, tipTransform), Vector3Transform(origin, tipTransform))
        );
    
    constexpr float suctionCupOffset = 0.000f; // How far from the tip
    constexpr float suctionCupNeckLength = 0.012f;
    constexpr float suctionCupNeckRadius = 0.005f;
    constexpr float suctionCupCupLength = 0.008f;
    constexpr float suctionCupBaseRadius = 0.006f;
    constexpr float suctionCupRimRadius = 0.014f;   // flared rim, wider than the neck

    constexpr float hubRadius = 0.01f;
    constexpr float hubLength = 0.008f;

    constexpr float clawSpread = 0.012f;
    constexpr float clawBackOffset = 0.005f;
    constexpr float clawStraightLength = 0.028f;    // straight section from the hub
    constexpr float clawTipLength = 0.014f;         // angled-in tip section
    constexpr float clawBaseRadius = 0.005f;
    constexpr float clawTipRadius = 0.0025f;        // tapers to a point
    constexpr float clawInwardBend = 0.006f;        // how far the tip bends toward centre

    // Small mounting hub where the claws attach, sitting just behind the tip.
    Vector3 hubCenter = Vector3Subtract(tip, Vector3Scale(approachDirection, clawBackOffset));
    Vector3 hubBack = Vector3Subtract(hubCenter, Vector3Scale(approachDirection, hubLength));
    DrawCylinderEx(flipZY(hubBack), flipZY(hubCenter), hubRadius, hubRadius * 0.8f, 20, DARKGRAY);

    // Suction cup: a narrow neck plus a flared cup, mounted slightly forward of the tip.
    Vector3 suctionAnchor = Vector3Subtract(tip, Vector3Scale(approachDirection, suctionCupOffset));
    Vector3 neckEnd = Vector3Add(suctionAnchor, Vector3Scale(approachDirection, suctionCupNeckLength));
    Vector3 cupEnd = Vector3Add(neckEnd, Vector3Scale(approachDirection, suctionCupCupLength));

    DrawCylinderEx(flipZY(suctionAnchor), flipZY(neckEnd), suctionCupNeckRadius, suctionCupBaseRadius, 16, YELLOW);
    DrawCylinderEx(flipZY(neckEnd), flipZY(cupEnd), suctionCupBaseRadius, suctionCupRimRadius, 20, YELLOW);
    // Thin rim disc at the very end, so the cup silhouette reads clearly front-on.
    DrawCylinderEx(flipZY(cupEnd), flipZY(Vector3Add(cupEnd, Vector3Scale(approachDirection, 0.001f))), suctionCupRimRadius, suctionCupRimRadius, 20, GOLD);

    // Two claws: straight section from the hub, then a shorter tapered tip bent inward.
    for (float side : {1.0f, -1.0f}) {
        Vector3 sideOffset = Vector3Scale(sideways, clawSpread * side);

        Vector3 clawStart = Vector3Add(hubCenter, sideOffset);
        Vector3 clawBend = Vector3Add(
            Vector3Add(hubCenter, Vector3Scale(approachDirection, clawStraightLength)),
            sideOffset);
        Vector3 clawTip = Vector3Add(
            Vector3Add(hubCenter, Vector3Scale(approachDirection, clawStraightLength + clawTipLength)),
            Vector3Scale(sideways, (clawSpread - clawInwardBend) * side)); // bends toward centre

        DrawCylinderEx(flipZY(clawStart), flipZY(clawBend), clawBaseRadius, clawBaseRadius * 0.6f, 12, BLUE);
        DrawCylinderEx(flipZY(clawBend), flipZY(clawTip), clawBaseRadius * 0.6f, clawTipRadius, 10, BLUE);
    }
}


// ==================================== ROBOT ==========================================
// CREDIT TO ANTHROPIC CLAUDE AI (Sonnet 5) USED FOR VISUAL ROBOT DESIGN

namespace {

// Blends between two colors — used to gradient the arm from base to tip.
Color lerpColor(Color a, Color b, float t)
{
    return Color{
        static_cast<unsigned char>(a.r + (b.r - a.r) * t),
        static_cast<unsigned char>(a.g + (b.g - a.g) * t),
        static_cast<unsigned char>(a.b + (b.b - a.b) * t),
        255};
}

} // namespace

void drawRobot(
    const SimulationState& state,
    const robot_arm::JointPositions& positions,
    const robot_arm::JointTransforms& transforms)
{
    // ========= DRAWING / TEXT / OUTPUT ==========
    BeginMode3D(state.camera);

    // Light brown walls enclosing the grid's perimeter, sized to match
    // DrawGrid(40, 0.05f)'s footprint. No floor — just the four walls.
    constexpr float wallExtent = 40 * 0.05f / 2.0f;
    constexpr float wallHeight = 1.2f;
    constexpr float wallThickness = 0.01f;
    const Color wallBase {181, 140, 99, 255};

    // Fake directional lighting: walls facing an assumed light direction
    // (+X, +Z) get brighter shading, the far side walls get darker —
    // cheap trick for a sense of depth without real shaders.
    auto shade = [](Color c, float factor) {
        return Color{
            static_cast<unsigned char>(c.r * factor),
            static_cast<unsigned char>(c.g * factor),
            static_cast<unsigned char>(c.b * factor),
            255};
    };

    DrawCube(Vector3{0.0f, wallHeight / 2.0f, -wallExtent}, wallExtent * 2.0f, wallHeight, wallThickness, shade(wallBase, 1.15f)); // lit side
    DrawCube(Vector3{0.0f, wallHeight / 2.0f, wallExtent}, wallExtent * 2.0f, wallHeight, wallThickness, shade(wallBase, 0.75f));  // shadow side
    DrawCube(Vector3{-wallExtent, wallHeight / 2.0f, 0.0f}, wallThickness, wallHeight, wallExtent * 2.0f, shade(wallBase, 1.0f));  // side wall, neutral
    DrawCube(Vector3{wallExtent, wallHeight / 2.0f, 0.0f}, wallThickness, wallHeight, wallExtent * 2.0f, shade(wallBase, 0.85f));  // side wall, slightly dim

    DrawGrid(40, 0.05f);

    // Target: a small crosshair gizmo instead of a plain sphere.
    Vector3 targetDraw = flipZY(state.targetPosition);
    constexpr float crosshairSize = 0.03f;
    DrawSphere(targetDraw, 0.01f, RED);
    DrawLine3D(Vector3Subtract(targetDraw, Vector3{crosshairSize, 0, 0}), Vector3Add(targetDraw, Vector3{crosshairSize, 0, 0}), RED);
    DrawLine3D(Vector3Subtract(targetDraw, Vector3{0, crosshairSize, 0}), Vector3Add(targetDraw, Vector3{0, crosshairSize, 0}), RED);
    DrawLine3D(Vector3Subtract(targetDraw, Vector3{0, 0, crosshairSize}), Vector3Add(targetDraw, Vector3{0, 0, crosshairSize}), RED);

    // Base housing: dark, chunky, tapered slightly for a grounded look.
    Vector3 baseTop = flipZY(Vector3{0.0f, 0.0f, 0.195f});
    DrawCylinderEx(flipZY(positions[0]), baseTop, 0.03f, 0.022f, 30, DARKGRAY);
    DrawCylinderEx(flipZY(positions[0]), baseTop, 0.028f, 0.02f, 30, BLACK); // inner shadow band for depth

    const Color armStart {70, 130, 220, 255};   // steel blue at the base
    const Color armEnd {80, 220, 210, 255};     // cyan toward the tip

    for (std::size_t joint = 0; joint < robot_arm::kJointCount; ++joint) {
        const float t = static_cast<float>(joint) / static_cast<float>(robot_arm::kJointCount - 1);
        const Color linkColor = lerpColor(armStart, armEnd, t);

        // Tapered link: slightly thicker at the base end, thinner toward the tip.
        DrawCylinderEx(
            flipZY(positions[joint]), flipZY(positions[joint + 1]),
            0.012f - 0.004f * t, 0.009f - 0.003f * t,
            20, linkColor);

        // Joint: a bright core sphere with a darker outer ring for contrast.
        DrawSphere(flipZY(positions[joint]), 0.017f, DARKGRAY);
        DrawSphere(flipZY(positions[joint]), 0.011f, linkColor);
    }

    // End effector marker + gripper.
    DrawSphere(flipZY(positions.back()), 0.01f, WHITE);
    drawGripper(positions.back(), transforms.back());

    // Pathway trajectory: connected line instead of floating dots, with the
    // current waypoint glowing.
    for (std::size_t index = 0; index + 1 < state.pathwayPoints.size(); ++index) {
        DrawLine3D(flipZY(state.pathwayPoints[index]), flipZY(state.pathwayPoints[index + 1]), Fade(GRAY, 0.5f));
    }
    for (std::size_t index = 0; index < state.pathwayPoints.size(); ++index) {
        const bool isCurrent = index == state.currentWaypoint;
        DrawSphere(
            flipZY(state.pathwayPoints[index]),
            isCurrent ? 0.022f : 0.01f,
            isCurrent ? YELLOW : Fade(GRAY, 0.6f));
    }

    EndMode3D();
}


// ==================================== HUD / Text ==========================================

void drawInterface(const SimulationState& state)
{
    // Checks if robot is at home angles. From kinematics.hpp 
    const bool atHome = robot_arm::isAtHomeAngles(state.angles, state.model.homeAngles);

    // Text showing user interaction with theta of the joints
    DrawText("6 DOF Robot Arm :: FK & IK Kinematics Prototype", 10, 10, 20, DARKGRAY);
    DrawText(
        TextFormat("Selected joint: %d [Buttons 1-6]",
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
        TextFormat("Inverse kinematics: %s [0 to toggle, arrows + E,Q to navigate]",
                   state.targetMode ? "ACTIVE" : "OFF"),
        10,
        100,
        20,
        RED);
    DrawText(
        TextFormat("Target: %.3f %.3f %.3f; error: %.4f",
                   state.targetPosition.x,
                   state.targetPosition.y,
                   state.targetPosition.z,
                   state.targetDistance),
        10,
        130,
        20,
        BLACK);
    DrawText(
        TextFormat("Pathway mode: %s (Button V)", state.pathwayMode ? "ON" : "OFF"),
        10,
        160,
        20,
        DARKGREEN);
    DrawText(
        TextFormat("Current shape: %s (Button C)",
                   robot_arm::pathwayName(state.pathwayShape).data()),
        10,
        190,
        20,
        DARKGREEN);
    DrawText(
        TextFormat("Joint limits: %s (Button L)",
                   state.enforceJointLimits ? "ON" : "OFF"),
        10,
        220,
        20,
        DARKGREEN);
    DrawText(
        TextFormat("Home Postition: %s ", 
                    (atHome) ? "YES" : "NO"),
        10,
        250,
        20,
        GRAY);
}


