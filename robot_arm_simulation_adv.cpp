#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <string>

// Code structure: STRUCTURES, FUNCTIONS, MAIN

// ======== NEW STRUCTS ========

// Create the structure of the DH row for 6 DOF. theta not stored because its live variable
struct DHRow {float a, alpha, d; };



// ======== FUNCTIONS =========

// Function det 3x3 matrix just for efficiency
float det3x3(float A[3][3]) {
    return ( A[0][0]*(A[1][1]*A[2][2] - A[2][1]*A[1][2]) - A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0]) + A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]) );
};


Matrix DHTransform(float theta, float d, float a, float alpha) {
    Matrix rotZ = MatrixRotateZ(theta);
    Matrix transZ = MatrixTranslate(0.0f, 0.0f, d);
    Matrix transX = MatrixTranslate(a, 0.0f, 0.0f);
    Matrix rotX = MatrixRotateX(alpha);
    
    // In order apply formula 
    Matrix T = MatrixMultiply(rotX, transX);
    T = MatrixMultiply(T, transZ);
    T = MatrixMultiply(T, rotZ);
    
    return T;
};



// Function for building 3x6 Jacobian for the IK Function 
void ComputeJacobian(Vector3 jointPositions[7], Matrix combinedMatrices[7], float J[3][6]) {
    Vector3 endPos = jointPositions[6]; // Position of end joint 
    Vector3 zLocal = {0.0f, 0.0f, 1.0f }; // Reference for one unit in z axis
    Vector3 origin {0.0f, 0.0f, 0.0f }; // Reference for the origin
    
    for (int i=0; i<6; i++) {
        // Here we will apply the formula for finding the direction and magnitude that the end joint moves per unit angle of the current joint being moved. This unit angle is tiny and in one axis (z in this case). Tiny because we want to check its updates every single frame to avoid singularities and also find the most efficient way.
        
        // We get a pure direction vector from subtracting these two transformed vectors 
        // We get the rotations effect on the direction by just seeing where the z-point goes
        Vector3 zTiny = Vector3Subtract(
            Vector3Transform(zLocal, combinedMatrices[i]),
            Vector3Transform(origin, combinedMatrices[i])
        );
        zTiny = Vector3Normalize(zTiny); // Making sure this z movement stays unit length 
        
        Vector3 toEnd = Vector3Subtract(endPos, jointPositions[i]); // Difference between end joint to current joint - used below
        Vector3 col = Vector3CrossProduct(zTiny, toEnd); // column_i = z_i × (p_end − p_i) Formula for finding the dir and mag the end pos moves per unit angle of the current joint 
        
        J[0][i] = col.x;
        J[1][i] = col.y;
        J[2][i] = col.z;
        
    };
};



    
// Function for solving a 3x3 matrix using Cramer's Rule (this will be needed)
// Solves A . x = b for unknown vector x, given 3x3 A and 3vector b 
void Solve3x3(float A[3][3], float b[3], float x[3]) {
    
    // Find det 
    float det = det3x3(A);
    
    // Check if det if 0 (singularity) We will just return 0s
    if (fabsf(det) < 1e-9f) { x[0]=x[1]=x[2]=0.0f; return; };
    
    // Using Cramer's Rule to solve for each unknown 
    for (int col=0; col<3; col++) {
        // Create matrix to subsitute values and find separate determinants 
        float M[3][3]; 
        for (int r=0; r<3; r++) {
            for (int c=0; c<3; c++) {
                // Sets each column in order by the vector B and calcualtes the matrix's determinant, later for use in finding the x, y, and z uknowns - Cramer's Rule.
                M[r][c] = (c == col) ? b[r] : A[r][c];
                
                float detM = det3x3(M);
                
                // Final calc (finding x vector)
                x[col] = detM / det;
        
            };
        };
    };
    
};


// Forward Kinematics Function (Applies DH row for every joint and transforms each joint)
void ForwardKinematics(float theta[6], DHRow joints[6], Vector3 jointPositions[7], Matrix combinedMatrices[7]) {
    // States origin, Joint positions, combined matrices
    Vector3 origin = { 0.0f, 0.0f, 0.0f };
    jointPositions[0] = origin;
    combinedMatrices[0] = MatrixIdentity();
    
    // For each combination of the DHRow, applies tranformation to the corresponding joint
    Matrix combined = MatrixIdentity();
    for (int i = 0; i < 6; i++) {
        Matrix Ti = DHTransform(theta[i], joints[i].d, joints[i].a, joints[i].alpha);
        combined = MatrixMultiply(Ti, combined);
        combinedMatrices[i+1] = combined;
        jointPositions[i+1] = Vector3Transform(origin, combined);
    };
};



    
// Function Inverse Kinematics (IK) 
// Takes current joint angles (theta), arm locations via DH, target position, and the damping constant lambda which prevents singularities and surpassing mechanical machine limits. Returns the distance to the target. Will be used iteratively in the main loop until target coordinate reached.

// Will compute matrix from a damped least-squares formula:  A = J·J^T + λ²I

float IKstep(float theta[6], DHRow joints[6], Vector3 target, float lambda, float thetaMin[6], float thetaMax[6], bool limitJointsMode) {
    // Initialise joint positions vector and combined  matrices and recompute the FK 
    Vector3 jointPositions[7]; 
    Matrix combinedMatrices[7]; 
    ForwardKinematics(theta, joints, jointPositions, combinedMatrices);
    
    // Find current distance from target 
    Vector3 dist = Vector3Subtract(target, jointPositions[6]); 
    
    // Find current Jacobian
    float J[3][6];
    ComputeJacobian(jointPositions, combinedMatrices, J);
    
    
    // Minimum lambda for near-singularity damping for error proning.
    // We will vary this to increase when the coordinate is too far from the robots reach which causes the angles to max out and the robot to glitch and move franticly due to the arms maxing out.
    // This is done with Yoshiwa's manipulability measure which decides when lambda should increase, and when it should stay constant at 0.05. Yoshikawa's measure = sqrt (det(JxJ^T))
    float JJt[3][3];
    for (int i=0; i<3; i++) {
        for (int j=0; j <3; j++) {
            float sum = 0.0f;
            for (int k=0; k<6; k++) {
                sum += J[i][k] * J[j][k];
                JJt[i][j] = sum;
            };
        };
    };
    
    float Yoshikawa = sqrtf(fabsf(det3x3(JJt))); // Manipubility measure
    
    // Different lambda values for different positions 
    float lambdaMin = 0.01f, lambdaMax = 0.3f, threshold = 0.05f;
    
    // Linear relationship between lambda and the damping constant (the further the target is from arm reach, the less the arm looks for a solution)
    float lambdaEff = lambdaMin;
    if (Yoshikawa < threshold) {
        float fraction_lambda = 1.0f - (Yoshikawa / threshold);
        lambdaEff = lambdaMin + fraction_lambda * (lambdaMax - lambdaMin);
    };
    
    
    
    // Applying damped least-squares formula by steps 
    // A = J·J^T + λ²I
    float A[3][3];
    for (int r=0; r<3;r++) {
        for (int c=0; c<3; c++) {
            A[r][c] = JJt[r][c] + (r == c ? lambdaEff*lambdaEff : 0.0f);
        };
    };
        
    // Solve the unknown vector x using Cramer's Rule func    
    float b[3] = {dist.x, dist.y, dist.z};
    float x[3];
    Solve3x3(A, b, x);
    
    // Finds Δθ = J^T · x where Δθ is the tiny change in joint angles per iter
    float deltaTheta[6]; 
    for (int i=0; i<6; i++) {
        deltaTheta[i] = J[0][i]*x[0] + J[1][i]*x[1] + J[2][i]*x[2];
    };
    
    // Applies the change in theta. sense_ can be modified to make sure that the change never goes over the mechanical limit of the machine until the coordinates converge
    // UPDATE: Introduce a max step so the angle can only change with a maximum velocity at a time (realistic to motor)
    float sens_ = 1.0f;
    float thetaVelMax = 0.02f;
    for (int i=0; i<6; i++) {
        float d = deltaTheta[i] * sens_;
        if (d > thetaVelMax) {
            d = thetaVelMax;
        } if (d < -thetaVelMax) {
            d = -thetaVelMax;
        };
        theta[i] += d;
        // Limit max and min theta values 
        if (limitJointsMode) {
        theta[i] = Clamp(theta[i], thetaMin[i], thetaMax[i]);
        };
    };
    
    // So we know until its converged.
    return Vector3Length(dist);
    
};


// Function applied different pathways once a different shape is toggled
std::vector<Vector3> CreatePathwayPoints(int toggleShape, float radius, int& waypointAmount) {
    // Initialize Points
    std::vector<Vector3> tracePoints;
    if (toggleShape == 0) {
        // None
        waypointAmount = 0;
        
    } if (toggleShape == 1) {
        // Pick and Place pathway 
        waypointAmount = 8;
        tracePoints = {
            {0.4f, 0.0f, 0.0f},
            {0.4f, 0.0f, 0.4f},
            {0.0f, 0.4f, 0.4f},
            {0.0f, 0.4f, 0.0f},
            {0.0f, 0.4f, 0.4f},
            {-0.4f, 0.0f, 0.4f},
            {-0.4f, 0.0f, 0.0f},
            {-0.4f, 0.0f, 0.4f}
        };
            
    } if (toggleShape == 2) {
        // Cube 
        waypointAmount = 8; 
        tracePoints = {
            {-0.3f, 0.3f, 0.3f},
            {-0.3f, -0.3f, 0.3f},
            {-0.3f, -0.3f, -0.3f},
            {0.3f, -0.3f, -0.3f},
            {-0.3f, 0.3f, -0.3f},
            {0.3f, -0.3f, 0.3f},
            {0.3f, 0.3f, -0.3f},
            {0.3f, 0.3f, 0.3f}
        };
        
    } if (toggleShape == 3) {
        // Circle 
        float angle = 0.0f;
        waypointAmount = 100;
        for (int i=0; i<waypointAmount; i++) {
            tracePoints.push_back({radius*std::cos(angle*DEG2RAD), radius*std::sin(angle*DEG2RAD), 0.0f});
            angle += (360.0f/waypointAmount);
        };
    } if (toggleShape == 4) {
        float angle = 0.0f;
        // Pringle 
        waypointAmount = 100;
        for (int i=0; i<waypointAmount; i++) {
            tracePoints.push_back({radius*std::cos(angle*DEG2RAD), radius*std::sin(angle*DEG2RAD), radius*std::cos(angle*DEG2RAD)*radius*std::cos(angle*DEG2RAD)});
            angle += (360.0f/waypointAmount);
        };
    };
    
    return tracePoints;
    
};










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
    };
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
        
        
        
        // ========= TARGET INTERACTION =========
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
