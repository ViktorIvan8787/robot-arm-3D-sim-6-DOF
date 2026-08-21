#include "robot_arm/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <raymath.h>
#include <iostream>

namespace robot_arm {
namespace {

// ================ MATRIX OPERATION FUNCTIONS ===============

// Function det NxN using Gaussian elimination and partial pivoting
template <std::size_t N>
float determinantNxN(float matrix[N][N])
{
    // Create dupe
    float working[N][N];
    for (std::size_t row = 0; row < N; ++row) {
        for (std::size_t col = 0; col < N; ++col) {
            working[row][col] = matrix[row][col];
        }
    }

    // Track swaps. Each odd swap x -1
    int swapCount = 0;

    // Check solveLinearSystem for more info
    for (std::size_t pivot = 0; pivot < N; ++pivot) {
        std::size_t bestRow = pivot;
        float largestValue = std::fabs(working[pivot][pivot]);
        for (std::size_t row = pivot + 1; row < N; ++row) {
            const float tempValue = std::fabs(working[row][pivot]);
            if (tempValue > largestValue) {
                largestValue = tempValue;
                bestRow = row;
            }
        }

        if (largestValue < 1.0e-9f) {
            return 0.0f; // singular matrix, determinant = zero
        }

        if (bestRow != pivot) {
            for (std::size_t col = 0; col < N; ++col) {
                std::swap(working[pivot][col], working[bestRow][col]);
            }
            ++swapCount;
        }

        // Create upper triangular
        for (std::size_t row = pivot + 1; row < N; ++row) {
            const float factor = working[row][pivot] / working[pivot][pivot];
            for (std::size_t col = pivot; col < N; ++col) {
                working[row][col] -= factor * working[pivot][col];
            }
        }
    }

    // Determinant is product of diagonals. 
    float determinant = 1.0f;
    for (std::size_t i = 0; i < N; ++i) {
        determinant *= working[i][i];
    }

    // If odd num of swaps, then determinant is negative
    if (swapCount % 2 != 0) {
        determinant = -determinant;
    }

    return determinant;
}

template float determinantNxN<3>(float matrix[3][3]); // <- NOT currently used
template float determinantNxN<6>(float matrix[6][6]);

// ============ SOLVING 3x3/6x6 LINEAR SYSTEM ============

// For arm orientation
// Solves a general N x N linear system A*x = b using Gaussian elimination
// with partial pivoting. Return false if matrix is singular to within tolerance level.
// ONLY N=3 N=6. One is translation 3x3, one is translation+orientation 6x6
// matrix is the current 6x6/3x3 damped system which relates joint vel and end vel.
// rightHandSide is the error of all parts. This function finds out how to best 
// reduce error every step. solution is change in each angle that will reduce error 
// the best.
template <std::size_t N> 
bool solveLinearSystem(float matrix[N][N], const float rightHandSide [N], float solution[N]) 
{
    // Create augmented matrix to setup elimination
    // First we copy the whole matrix and then add the RHS 
    float augmented[N][N+1];
    for (std::size_t row = 0; row < N; row++) {
        for (std::size_t col = 0; col < N; col++) {
            augmented[row][col] = matrix[row][col];
        }
        augmented[row][N] = rightHandSide[row];
    }

    // Forward elimination with partial pivoting - 
    // Find largest value in each column, replace specific 
    // row with column (with largest value). Calculate 
    // multiplier with each row below to make the column 
    // have zeros underneath, until it becomes an upper 
    // traingular matrix. 
    // We want largest value, as if we divide by small,
    // calculations amplify largely.
    for (std::size_t pivot = 0; pivot < N; ++pivot) {
        std::size_t bestRow = pivot;
        float largestValue = std::fabs(augmented[pivot][pivot]);
        for (std::size_t row = pivot + 1; row < N; ++row) {
            const float tempLargest = std::fabs(augmented[row][pivot]);
            // Find largest val and its row
            if (tempLargest > largestValue) {
                largestValue = tempLargest;
                bestRow = row;
            }
        }

        // Return false if largestValue is too small
        // to contribute to modifying the angles
        if (largestValue < 1.0e-9f) {
            for (std::size_t i = 0; i < N; ++i) {
                solution[i] = 0.0f;
            }
            return false;
        }

        // Swap bestRow with column 
        if (bestRow != pivot) {
            for (std::size_t col = 0; col <= N; ++col) {
                std::swap(augmented[pivot][col], augmented[bestRow][col]);
            }
        }

        // Find factor between largestValue and other values 
        // in column and subtract to turn into zeros
        for (std::size_t row = pivot + 1; row < N; ++row) {
            const float factor = augmented[row][pivot] / augmented[pivot][pivot];
            for (std::size_t col = pivot; col <= N; ++col) {
                augmented[row][col] -= factor * augmented[pivot][col];
            }
        }
    }

    // Back substition
    // (solution is RHS / sums of rows on LHS)
    for (std::size_t i = N; i-- > 0;) {
        float sum = augmented[i][N];
        for (std::size_t col = i + 1; col < N; ++col) {
            sum -= augmented[i][col] * solution[col];
        }
        solution[i] = sum / augmented[i][i];
    }

    return true;

}

// Generate specific tempaltes for solveLinearSystem
template bool solveLinearSystem<3>(float matrix[3][3], const float rightHandSide[3], float solution[3]); // <- NOT currently used
template bool solveLinearSystem<6>(float matrix[6][6], const float rightHandSide[6], float solution[6]);

// ============ QUATERNIONS / ORIENTATION ============

// Remove a specific axis for free movement to not be constrained orientationally
// Used ot leave one rotational degree in current case (Pitch "only" wrist, that 
// doesn't twist to the side)
Vector3 removeConstrictedAxis(Vector3 vector, Vector3 freeAxis)
{
    const float alongFreeAxis = Vector3DotProduct(vector, freeAxis);
    return Vector3Subtract(vector, Vector3Scale(freeAxis, alongFreeAxis));
}

// For our specific pitch case, this function computes "pitch" axis
// for any current orientation the robot finds itself in. This way the 
// claw only rotates to be parallel to the robot. Future complications
// can be added to orientation when needed
Vector3 computePitchAxis(Vector3 targetPosition) 
{
    Vector3 horizontalAxis = {targetPosition.x, targetPosition.y, 0.0f};
    const float horizontalLength = Vector3Length(horizontalAxis);
    // Checks if target right in front. If not, returns the z axis direction
    if (Vector3Length(horizontalAxis) < 0.001f) {
        return Vector3{1.0f, 0.0f, 0.0f};
    }
    horizontalAxis = Vector3Normalize(horizontalAxis);
    Vector3 pitchAxis = Vector3CrossProduct(Vector3{0.0f, 0.0f, 1.0f}, horizontalAxis); 
    return Vector3Normalize(pitchAxis);
}

// Turns a matrix 3x3 matrix into a quaternion (w + xi + yj + zk)
Quaternion matrixToQuaternion(Matrix m)
{
    // Raylib's matrix is column-major (index of r1c2 is m4)
    const float r00 = m.m0, r01 = m.m4, r02 = m.m8;
    const float r10 = m.m1, r11 = m.m5, r12 = m.m9;
    const float r20 = m.m2, r21 = m.m6, r22 = m.m10;

    const float trace = r00 + r11 + r22; // Diagonal

    // Uses formula for 3x3 matrix into quaternion. Trace = sum of diagonals
    Quaternion q {};
    if (trace > 0.0f) {
        const float s = std::sqrt(trace + 1.0f) * 2.0f; // s = 4*w like in formulas
        q.w = 0.25f * s;                  // Formula for real value w  
        q.x = (r21 - r12) / s;            // Formula for imaginary value x 
        q.y = (r02 - r20) / s;            // Formula for imaginary value y
        q.z = (r10 - r01) / s;            // Formula for imaginary value z
        // and assigning.

        // Other formulas are for when trace < 0.0f. 
        // Which varaibles used is determined on which diagonal is largest

        // s is used because if trace is too small, then division amplifies it alot.
    } else if (r00 > r11 && r00 > r22) {
        const float s = std::sqrt(1.0f + r00 - r11 - r22) * 2.0f; // s = 4*x
        q.w = (r21 - r12) / s;
        q.x = 0.25f * s;
        q.y = (r01 + r10) / s;
        q.z = (r02 + r20) / s;
    } else if (r11 > r22) {
        const float s = std::sqrt(1.0f + r11 - r00 - r22) * 2.0f; // s = 4*y
        q.w = (r02 - r20) / s;
        q.x = (r01 + r10) / s;
        q.y = 0.25f * s;
        q.z = (r12 + r21) / s;
    } else {
        const float s = std::sqrt(1.0f + r22 - r00 - r11) * 2.0f; // s = 4*z
        q.w = (r10 - r01) / s;
        q.x = (r02 + r20) / s;
        q.y = (r12 + r21) / s;
        q.z = 0.25f * s;
    }
    return q;
};

// Multiplying two Quaternions together 
Quaternion multiplyQuaternions(Quaternion a, Quaternion b) 
{
    // Hamiltonian product used as imaginary numbers 
    // multiplying together add negatives and real nums
    return Quaternion{
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
};

// Getting the inverse of a quaternion 
// (just the negative of the imaginary numbers and flipped)
Quaternion conjugateQuaternion(Quaternion q)
{
    return Quaternion{-q.x, -q.y, -q.z, q.w};
};

// Extracts original 3D axis and angle
Vector3 quaternionToAxisAngle(Quaternion q, float& outAngle)
{
    // Confirming w is in a valid range for inverse cos. 
    // Due to potential floating-point rounding errors.
    // Original formula for q.w - cos(halfangle) 
    // so inverse is performed for w (acos)
    const float w = std::clamp(q.w, -1.0f, 1.0f);
    outAngle = 2.0f * std::acos(w);

    // Getting scaling multiply to divide imaginary components 
    // 1 - cos^2 acts as sin^2 here
    const float sinHalfAngle = std::sqrt(1.0f - w * w);
    if (sinHalfAngle < 1.0e-6f) {
        // Angle is around 0 so angle direction doesn't
        // matter and return default. Avoids crash
        return Vector3{0.0f, 0.0f, 0.1f};
    };

    return Vector3{q.x / sinHalfAngle, q.y / sinHalfAngle, q.z / sinHalfAngle}; // Direction vector
};

// ============ JACOBIANS 6x6 =============

void computePositionOrientationJacobian(
    const JointPositions& positions,
    const JointTransforms& transforms,
    float jacobian[6][kJointCount])
{
    const Vector3 endPosition = positions.back(); // Position of end joint
    constexpr Vector3 localZ {0.0f, 0.0f, 1.0f}; // Reference z unit
    constexpr Vector3 origin {0.0f, 0.0f, 0.0f}; // Reference origin

    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        // For each J col, its equal to the end eff position minus 
        // centre position vector of joint i, cross multiplied by 
        // the axis direction unit vector, as seen below. 
        // See computePositionJacobian for more info.

        // Get axis direction
        Vector3 axis = Vector3Subtract(
            Vector3Transform(localZ, transforms[joint]),
            Vector3Transform(origin, transforms[joint]));
        axis = Vector3Normalize(axis);
        
        // end eff - joint i centre pos
        // Cross product to axis. Finds mag and dir end
        // position moves per angel of joint i
        const Vector3 jointToEnd = Vector3Subtract(endPosition, positions[joint]);
        const Vector3 column = Vector3CrossProduct(axis, jointToEnd);

        jacobian[0][joint] = column.x;
        jacobian[1][joint] = column.y;
        jacobian[2][joint] = column.z;
        jacobian[3][joint] = axis.x; 
        jacobian[4][joint] = axis.y;
        jacobian[5][joint] = axis.z;
        // No cross product for orientation as only changes rotation axis
    }
}

} // namespace











Matrix dhTransform(float theta, float d, float a, float alpha)
{
    const Matrix rotationZ = MatrixRotateZ(theta);
    const Matrix translationZ = MatrixTranslate(0.0f, 0.0f, d);
    const Matrix translationX = MatrixTranslate(a, 0.0f, 0.0f);
    const Matrix rotationX = MatrixRotateX(alpha);

    // In order apply formula.
    // This multiplication order preserves the prototype's Raylib convention.
    // It should be checked against hand-calculated FK test cases before the
    // model is used to command physical hardware.
    Matrix transform = MatrixMultiply(rotationX, translationX);
    transform = MatrixMultiply(transform, translationZ);
    transform = MatrixMultiply(transform, rotationZ);
    return transform;
}

Vector3 computeOrientationError(Matrix current, Matrix target)
{
    Quaternion currentQ = matrixToQuaternion(current);
    Quaternion targetQ = matrixToQuaternion(target);

    // Multiply the inverse of the current q position to the target q. 
    // Gives rotation that gets from current to target (undo current, then apply target)
    Quaternion errorQ = multiplyQuaternions(targetQ, conjugateQuaternion(currentQ));

    float angle = 0.0f;
    Vector3 axis = quaternionToAxisAngle(errorQ, angle);

    // If the angle is > 360, then return it to 
    // its negative. Prevents angles from overshooting.
    // If angle is too small, just return null vector (no error)
    if (angle > PI) {
        angle -= 2.0f * PI;
    };
    if (std::fabs(angle) < 1.0e-6f) {
        return Vector3{0.0f, 0.0f, 0.0f};
    }

    return Vector3Scale(axis, angle);
}

void forwardKinematics(
    const JointAngles& angles,
    const RobotModel& model,
    JointPositions& positions,
    JointTransforms& transforms)
{
    // States origin, Joint positions, combined matrices
    constexpr Vector3 origin {0.0f, 0.0f, 0.0f};
    positions[0] = origin;
    transforms[0] = MatrixIdentity();

    // For each combination of the DHRow, applies transformation to the
    // corresponding joint.
    Matrix combined = MatrixIdentity();
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        const DHRow& parameters = model.joints[joint];
        const Matrix localTransform = dhTransform(
            // Adding thetaOffset to prevent the sicking of the joint to the side
            angles[joint] + parameters.thetaOffset, parameters.d, parameters.a, parameters.alpha); 

        combined = MatrixMultiply(localTransform, combined);
        transforms[joint + 1] = combined;
        positions[joint + 1] = Vector3Transform(origin, combined);
    }
}

float performIKStep(
    JointAngles& angles,
    const RobotModel& model,
    Vector3 targetPosition,
    Matrix targetOrientation,
    const IKSettings& settings,
    bool enforceJointLimits)
{
    // Initialise joint positions vector and combined matrices and recompute FK
    JointPositions positions {};
    JointTransforms transforms {};
    forwardKinematics(angles, model, positions, transforms);

    // Find current distance from target (Position Error)
    // and angle from target orientation (Orientation Error) Related to the transformations
    const Vector3 positionError = Vector3Subtract(targetPosition, positions.back());
    const Vector3 orientationError = computeOrientationError(transforms.back(), targetOrientation);

    // Find current Jacobian
    float jacobian[6][kJointCount] {};
    computePositionOrientationJacobian(positions, transforms, jacobian);

    // Ignore rotational pitch axis to stop robot from going bonkers. 
    // Only approaches from a "pitch" axis, parallel to rest of robot. No side.
    // Done using functions that caculate current vertical place
    const Vector3 pitchAxis = computePitchAxis(targetPosition);
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        // Removing column
        Vector3 removeOrientCol {jacobian[3][joint], jacobian[4][joint], jacobian[5][joint]};
        removeOrientCol = removeConstrictedAxis(removeOrientCol, pitchAxis);
        jacobian[3][joint] = removeOrientCol.x;
        jacobian[4][joint] = removeOrientCol.y;
        jacobian[5][joint] = removeOrientCol.z;
    }

    // Will compute matrix from a damped least-squares formula:
    // A = J * J^T + lambda^2 I
    float jacobianTimesTranspose[6][6] {};
    for (int row = 0; row < 6; ++row) {
        for (int column = 0; column < 6; ++column) {
            for (std::size_t joint = 0; joint < kJointCount; ++joint) {
                jacobianTimesTranspose[row][column] +=
                    jacobian[row][joint] * jacobian[column][joint];
            }
        }
    }

    // Minimum lambda for near-singularity damping. We vary this to increase
    // when the coordinate is too far from the robot's reach, which previously
    // caused the angles to max out and the robot to move frantically.
    // Yoshikawa's manipulability measure = sqrt(det(J * J^T)).
    const float manipulability =
        std::sqrt(std::fabs(determinantNxN<6>(jacobianTimesTranspose)));

    // Linear relationship between manipulability and the damping constant.
    const float dampingRange =
        settings.maximumDamping - settings.minimumDamping;
    float effectiveDamping = settings.minimumDamping;
    if (settings.singularityThreshold > 0.0f &&
        manipulability < settings.singularityThreshold) {
        const float fraction = 1.0f - manipulability / settings.singularityThreshold;
        effectiveDamping = settings.minimumDamping + fraction * dampingRange;
    }

    // Applying damped least-squares formula by steps:
    // A = J * J^T + lambda^2 I
    float dampedSystem[6][6] {};
    for (int row = 0; row < 6; ++row) {
        for (int column = 0; column < 6; ++column) {
            dampedSystem[row][column] = jacobianTimesTranspose[row][column];
            if (row == column) {
                dampedSystem[row][column] += effectiveDamping * effectiveDamping;
            }
        }
    }

    // Remove constricted axis that we saw earlier
    const Vector3 modifiedOrientationError = removeConstrictedAxis(orientationError, pitchAxis);

    // Solve the unknown vector x using Gaussian Elimination (solveLinearSystem<6>)
    const float rightHandSide[6] = {
        positionError.x, positionError.y, positionError.z,
        modifiedOrientationError.x, modifiedOrientationError.y, modifiedOrientationError.z
    };
    float solution[6] {};
    if (!solveLinearSystem<6>(dampedSystem, rightHandSide, solution)) {
        return Vector3Length(positionError);
    }

    // Finds delta theta = J^T * x, where delta theta is the tiny change in
    // joint angles per iteration.
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        float correction = 0.0f;
        for (int row = 0; row < 6; ++row) {
            correction += jacobian[row][joint] * solution[row];
        }

        // Applies the change in theta and ensures one numerical step never
        // exceeds the configured maximum.
        angles[joint] += std::clamp(
            correction,
            -settings.maximumStepRadians,
            settings.maximumStepRadians);

        // Limit max and min theta values
        if (enforceJointLimits) {
            angles[joint] = std::clamp(
                angles[joint],
                model.minimumAngles[joint],
                model.maximumAngles[joint]);
        }
    }

    // So we know until it has converged.
    return Vector3Length(positionError);
}

// Returns true if the robot has reached home position (home position defined by homeAngles in robot.hpp.)
// This checks the angles are equal and uses a tolerance value in radians to make the check more approximate. 
bool isAtHomeAngles(
    const JointAngles& angles,
    const JointAngles& home,
    float toleranceRadians)  
{
    // Joint count is declared in simulation.cpp
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        if (std::fabs(angles[joint] - home[joint]) > toleranceRadians) {
            return false;
        }
    }
    return true;
}

} // namespace robot_arm
