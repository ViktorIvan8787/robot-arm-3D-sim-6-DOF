#include "src/kinematics.h"
#include <raylib.h>
#include <raymath.h>
#include <vector>
#include <string>

int main() {
    
    // ======== WINDOW AND 3D ========
    
    // Window 
    InitWindow(1000, 700, "6 DOF Robot Arm :: Forward Kinematics");
    SetTargetFPS(60);
    
    // Setting up 3D camera 
    Camera3D camera = {0};
    camera.position = {2.0f, 2.0f, 2.0f}; // Where camera is in space 
    camera.target = {0.0f, 0.0f, 0.0f}; // Point camera is looking at 
    camera.up = {0.0f, 1.0f, 0.0f}; // For the camera, y axis is upwards
    camera.fovy = 50.0f; // Field of view 
    camera.projection = CAMERA_PERSPECTIVE;
    
    // Variables for coordinate targeting with the camera
    Vector3 camTarget = {0.4f, 0.3f, 0.5f}; // starting target pos 
    bool TargMode = false; 
    
    // ======== ARM VARIABLES / JOINT SPECS =========
    
    // Geometry for all 6 joints (a, alpha, d) First joint is the base, second and third joints are the next two arms, third and fourth joints are the write roll and pitch, and the sixth join is the wrist yaw.
    // Notice how the arms have length along x axis, the others dont. pitch and roll have angles, and so does the base point as it can move around its centre.
    DHRow joints[6] = {
        {0.0f, 90.0f * DEG2RAD, 0.3f},
        {0.3f, 0.0f * DEG2RAD, 0.0f},
        {0.3f, 0.0f * DEG2RAD, 0.0f},
        {0.0f, 90.0f * DEG2RAD, 0.2f},
        {0.0f, -90.0f * DEG2RAD, 0.0f},
        {0.0f, 0.0f * DEG2RAD, 0.1f}
    };
    
    // Initial theta values (update live)
    bool limitJointsMode = true; // Toggle the limiting of Joints
    float theta[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    // Set min and max theta values to represent the joint limits and avoids breaking. Can be modified for different motors/robot-settings.
    // CURRENT: base values for kuka robot 
    float thetaMin[6] = { -180*DEG2RAD, -90*DEG2RAD, -150*DEG2RAD, -180*DEG2RAD, -120*DEG2RAD, -180*DEG2RAD };
    float thetaMax[6] = { 180*DEG2RAD, 90*DEG2RAD, 150*DEG2RAD, 180*DEG2RAD, 120*DEG2RAD, 180*DEG2RAD };
    
    // Variables for arm movement
    int selectedJoint = 0; // Which joint selected 
    float SpeedJoint = 1.0f * DEG2RAD; // Radians moved per frame tick when arm being moved
    
    // Variable for maximum reach. Used to stop arm movement beyond physically capable
    float reachMax = 0.0f;
    for (int i=0; i<6; i++) {
        reachMax += joints[i].a + joints[i].d; // a and d are the separate lengths of the arms. 
    }
    reachMax *= 0.85f; // Just limit it slightly to avoid any glitches
    
    // Variable for damping FK calc when target surpasses arm reach 
    float lambda = 0.05f;
    
    // ======= PATHWAY TRACING VARIABLES & SHAPES ========
    
    // Points for tracing out a pathway
    int waypointAmount = 0; // Used for reverting back
    // Create circle of points 
    float traceCircleRadius = 0.67f;
    float traceCircleAngle = 0.0f;  
    
    // Different pathways to toggle through
    std::vector<std::string> shapes = {"None", "Pick-and-Place", "Cube", "Circle", "Pringle"};
    int toggleShape = 0;
    std::string currentShape = shapes[toggleShape];
    
    std::vector<Vector3> tracePoints = CreatePathwayPoints(toggleShape, traceCircleRadius, waypointAmount);
    
    int currentWaypoint = 0;
    float arrivalAccuracy = 0.02f; // Once within this pixel accuracy, move onto the next point
    bool pathwayMode = false;
    
    // ======= MAIN 3D LOOP ========
    while (!WindowShouldClose()) {
        
        // ========================== USER INTERACTION ==========================
        
        // ======== CAMERA INTERACTION ==========
        float moveSpeed = 0.05f; // Movement of camera from WASD keys
        
        Vector3 position = { 0.0f, 0.0f, 0.0f };
        if (IsKeyDown(KEY_W)) position.x += moveSpeed; // Forward
        if (IsKeyDown(KEY_S)) position.x -= moveSpeed; // Back
        if (IsKeyDown(KEY_D)) position.y += moveSpeed; // Right
        if (IsKeyDown(KEY_A)) position.y -= moveSpeed; // Left
        if (IsKeyDown(KEY_SPACE)) position.z += moveSpeed; // Up
        if (IsKeyDown(KEY_LEFT_SHIFT)) position.z -= moveSpeed; // Down
        
        float mouseSens = 0.8f; // Sensitivity of the mouse in the camera 
        Vector2 mouseDelta = GetMouseDelta();
        
        UpdateCameraPro(&camera, position, 
            (Vector3){mouseDelta.x * mouseSens, mouseDelta.y * mouseSens, 0.0f // Roll (not used)
            },
            GetMouseWheelMove() * 2.0f // Zoom speed
        );
        
        // ========= TARGET INTERACTION ==========
        if (IsKeyPressed(KEY_ZERO)) TargMode = !TargMode; // Target mode controlled by button 0
        
        // Moving the target coordinate with the camera 
        if (TargMode) {
           float targetMove = 0.01f;
           if (IsKeyDown(KEY_UP)) camTarget.z += targetMove;
           if (IsKeyDown(KEY_DOWN)) camTarget.z -= targetMove;
           if (IsKeyDown(KEY_Q)) camTarget.y += targetMove;
           if (IsKeyDown(KEY_E)) camTarget.y -= targetMove; 
           if (IsKeyDown(KEY_LEFT)) camTarget.x += targetMove;
           if (IsKeyDown(KEY_RIGHT)) camTarget.x -= targetMove;   
        }; 
        
       // ========= PATHWAY INTERACTION =========
        if (IsKeyPressed(KEY_V)) {
            if (waypointAmount > 0) { // only allow pathway mode when there's an actual path to follow
                pathwayMode = !pathwayMode;
                if (pathwayMode) TargMode = true;
            }
        };
        
        if (IsKeyPressed(KEY_C)) {
            toggleShape = (toggleShape + 1) % (int)shapes.size();
            currentShape = shapes[toggleShape];
            tracePoints = CreatePathwayPoints(toggleShape, traceCircleRadius, waypointAmount);
            currentWaypoint = 0;
            if (waypointAmount == 0) pathwayMode = false; // When we go back to "None" pathway mode turns off and no points to follow
        };
        
        // ========= MOVEMENT INTERACTION ==========
        
        // Input for which join to be moved, using keys 1-6 to select 
        if (IsKeyPressed(KEY_ONE)) selectedJoint = 0;
        if (IsKeyPressed(KEY_TWO)) selectedJoint = 1;
        if (IsKeyPressed(KEY_THREE)) selectedJoint = 2;
        if (IsKeyPressed(KEY_FOUR)) selectedJoint = 3;
        if (IsKeyPressed(KEY_FIVE)) selectedJoint = 4;
        if (IsKeyPressed(KEY_SIX)) selectedJoint = 5;
        
        // Increased or decreasing theta at SpeedJoint per frame-tick for keys held down. O and P used for accessability. Also adding the limiting joints only if the mode is targeted
        if (IsKeyDown(KEY_O)) theta[selectedJoint] += SpeedJoint;
        if (IsKeyDown(KEY_P)) theta[selectedJoint] -= SpeedJoint;
        if (limitJointsMode) {
        theta[selectedJoint] = Clamp(theta[selectedJoint], thetaMin[selectedJoint], thetaMax[selectedJoint]);
        };
        
        // ========= LIMITING JOINTS TOGGLE ===========
        if (IsKeyPressed(KEY_L)) limitJointsMode = !limitJointsMode;
        
        // ================== CALCULATIONS / FORWARD KINEMATICS ================
        
        // IF CAM TARGET MODE IS ACTIVE, the arm will automatically move its joints to the correct target. If not, a separate simulation is present where you can control each arm at your own preference.
        
        Vector3 jointPositions[7];
        Matrix combinedMatrices[7];
        
        // Pathway Mode Initiation 
        if (pathwayMode) {
            camTarget = tracePoints[currentWaypoint];
        };
        
        // Create a fixed position for when the coordinates are out of reach.
        Vector3 tempFixedReach = camTarget;
        
        // If the coordinate is larger than the maximum arm reach, the coor is normalised by the scale factor to which it will stay in its same position. Instead of fixing its position, calculations continue but its scaled to one position.
        if (Vector3Length(camTarget) > reachMax) {
            tempFixedReach = Vector3Scale(Vector3Normalize(camTarget), reachMax);
        };
        
        // Main calc
        float targetDistance = 0.0f;
        if (TargMode) {
            targetDistance = IKstep(theta, joints, tempFixedReach, lambda, thetaMin, thetaMax, limitJointsMode);
        } ;
        
        if (pathwayMode && targetDistance < arrivalAccuracy) {
            currentWaypoint = (currentWaypoint + 1) % waypointAmount; // Goes in incrememnts of 1 and comes back to the final
        };
        
        // Call Kinematics function
        ForwardKinematics(theta, joints, jointPositions, combinedMatrices);
        
        // ========= DRAWING / TEXT / OUTPUT ==========
        BeginDrawing();
        ClearBackground(RAYWHITE);
        
        // SPHERE FOR TARGET COORD 
        DrawSphere(camTarget, 0.03f, RED);
        
        BeginMode3D(camera);
        DrawGrid(10, 0.5f); // %squares, %lengthOfSquareSides :: used for visual reference
        
        // Sphere for each joint, cylinder for each arm
        for (int i=0; i<6; i++) {
            DrawCylinderEx(jointPositions[i], jointPositions[i+1], 0.01f, 0.01f, 30, BLACK);
            DrawSphere(jointPositions[i], 0.02f, DARKBLUE);
        };
        // Last joint has a distinguishable colour (end point of arm)
        DrawSphere(jointPositions[6], 0.03f, GREEN);
        
        // Pathway trajectory
        for (int i=0; i<waypointAmount; i++) {
            Color c = (i == currentWaypoint) ? YELLOW : GRAY;
            float r = (i == currentWaypoint) ? 0.025f : 0.015f;
            DrawSphere(tracePoints[i], r, c);
        };
        
        EndMode3D();
        
        // TEXT
        DrawText("6 DOF Robot Arm :: Forward Kinematics Model", 10, 10, 20, DARKGRAY);
        // Text showing user intercation with theta of the joints 
        DrawText(TextFormat("Selected joint: %d (select keys 1-6 for specifict joints)", selectedJoint+1), 10, 40, 20, BLACK);
        DrawText(TextFormat("theta[%d} = %.1f degrees", selectedJoint, theta[selectedJoint] * RAD2DEG), 10, 70, 20, BLACK);
        
        // Targeting Mode
        DrawText(TextFormat("Inverse Kinematics Targeting: %s (0 to toggle, arrows and E,Q to move)", TargMode ? "ACTIVE" : "NOT"), 10, 100, 20, RED);
        DrawText(TextFormat("Target coord: %.3f %.3f %.3f", camTarget.x, camTarget.y, camTarget.z), 10, 130, 20, BLACK);
        
        // Pathway mode and pathway toggles
        DrawText(TextFormat("Pathway mode: %s (Key V)", pathwayMode ? "ON" : "OFF"), 10, 160, 20, DARKGREEN);
        DrawText(TextFormat("Current Shape: %s (Key C)", currentShape.c_str()), 10, 190, 20, DARKGREEN);
        
        // Toggle Joint Angle Limits
        DrawText(TextFormat("Limit Joint Angles Mode: %s (Key L)", limitJointsMode ? "ON" : "OFF"), 10, 220, 20, DARKGREEN);
        
        EndDrawing();
    };
    
    CloseWindow();
    return 0;
};