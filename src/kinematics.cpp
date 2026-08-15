#include "kinematics.h"
#include <raymath.h>
#include <cmath>

// ======== FUNCTIONS =========

// Function det 3x3 matrix just for efficiency
float det3x3(float A[3][3]) {
    return ( A[0][0]*(A[1][1]*A[2][2] - A[2][1]*A[1][2]) - A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0]) + A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]) );
}

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
}

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
        
    }
}

// Function for solving a 3x3 matrix using Cramer's Rule (this will be needed)
// Solves A . x = b for unknown vector x, given 3x3 A and 3vector b 
void Solve3x3(float A[3][3], float b[3], float x[3]) {
    
    // Find det 
    float det = det3x3(A);
    
    // Check if det if 0 (singularity) We will just return 0s
    if (fabsf(det) < 1e-9f) { x[0]=x[1]=x[2]=0.0f; return; }
    
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
        
            }
        }
    }
}

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
    }
}

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
            }
        }
    }
    
    float Yoshikawa = sqrtf(fabsf(det3x3(JJt))); // Manipubility measure
    
    // Different lambda values for different positions 
    float lambdaMin = 0.01f, lambdaMax = 0.3f, threshold = 0.05f;
    
    // Linear relationship between lambda and the damping constant (the further the target is from arm reach, the less the arm looks for a solution)
    float lambdaEff = lambdaMin;
    if (Yoshikawa < threshold) {
        float fraction_lambda = 1.0f - (Yoshikawa / threshold);
        lambdaEff = lambdaMin + fraction_lambda * (lambdaMax - lambdaMin);
    }
    
    // Applying damped least-squares formula by steps 
    // A = J·J^T + λ²I
    float A[3][3];
    for (int r=0; r<3;r++) {
        for (int c=0; c<3; c++) {
            A[r][c] = JJt[r][c] + (r == c ? lambdaEff*lambdaEff : 0.0f);
        }
    }
        
    // Solve the unknown vector x using Cramer's Rule func    
    float b[3] = {dist.x, dist.y, dist.z};
    float x[3];
    Solve3x3(A, b, x);
    
    // Finds Δθ = J^T · x where Δθ is the tiny change in joint angles per iter
    float deltaTheta[6]; 
    for (int i=0; i<6; i++) {
        deltaTheta[i] = J[0][i]*x[0] + J[1][i]*x[1] + J[2][i]*x[2];
    }
    
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
        }
        theta[i] += d;
        // Limit max and min theta values 
        if (limitJointsMode) {
        theta[i] = Clamp(theta[i], thetaMin[i], thetaMax[i]);
        }
    }
    
    // So we know until its converged.
    return Vector3Length(dist);
}

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
        }
    } if (toggleShape == 4) {
        float angle = 0.0f;
        // Pringle 
        waypointAmount = 100;
        for (int i=0; i<waypointAmount; i++) {
            tracePoints.push_back({radius*std::cos(angle*DEG2RAD), radius*std::sin(angle*DEG2RAD), radius*std::cos(angle*DEG2RAD)*radius*std::cos(angle*DEG2RAD)});
            angle += (360.0f/waypointAmount);
        }
    }
    
    return tracePoints;
}