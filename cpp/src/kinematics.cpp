#include "robot_arm/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <raymath.h>

namespace robot_arm {
namespace {

// Function det 3x3 matrix just for efficiency
float determinant3x3(const float matrix[3][3])
{
    return matrix[0][0] *
               (matrix[1][1] * matrix[2][2] - matrix[2][1] * matrix[1][2])
        - matrix[0][1] *
               (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] *
               (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
}

bool solve3x3(const float matrix[3][3], const float rightHandSide[3], float solution[3])
{
    // Find det
    const float determinant = determinant3x3(matrix);

    // Check if det is 0 (singularity). We will just return 0s.
    if (std::fabs(determinant) < 1.0e-9f) {
        solution[0] = 0.0f;
        solution[1] = 0.0f;
        solution[2] = 0.0f;
        return false;
    }

    // Using Cramer's Rule to solve for each unknown
    for (int replacedColumn = 0; replacedColumn < 3; ++replacedColumn) {
        // Create matrix to substitute values and find separate determinants
        float substituted[3][3] {};

        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                substituted[row][column] = column == replacedColumn
                    ? rightHandSide[row]
                    : matrix[row][column];
            }
        }

        // Final calc (finding x vector). This is intentionally done after the
        // whole matrix has been filled; doing it inside the loops reads values
        // before they have been initialised.
        solution[replacedColumn] = determinant3x3(substituted) / determinant;
    }

    return true;
}

void computePositionJacobian(
    const JointPositions& positions,
    const JointTransforms& transforms,
    float jacobian[3][kJointCount])
{
    const Vector3 endPosition = positions.back(); // Position of end joint
    constexpr Vector3 localZ {0.0f, 0.0f, 1.0f}; // Reference for one unit in z axis
    constexpr Vector3 origin {0.0f, 0.0f, 0.0f}; // Reference for the origin

    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        // Here we apply the formula for finding the direction and magnitude
        // that the end joint moves per unit angle of the current joint. (Jacobian)
        // We get a pure direction vector from subtracting these two
        // transformed vectors and get the rotation's effect by seeing where
        // the z-point goes.
        Vector3 axis = Vector3Subtract(
            Vector3Transform(localZ, transforms[joint]),
            Vector3Transform(origin, transforms[joint]));
        axis = Vector3Normalize(axis);

        const Vector3 jointToEnd = Vector3Subtract(endPosition, positions[joint]);
        // column_i = z_i x (p_end - p_i): direction and magnitude the end
        // position moves per unit angle of the current joint.
        const Vector3 column = Vector3CrossProduct(axis, jointToEnd);

        jacobian[0][joint] = column.x;
        jacobian[1][joint] = column.y;
        jacobian[2][joint] = column.z;
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
    Vector3 target,
    const IKSettings& settings,
    bool enforceJointLimits)
{
    // Initialise joint positions vector and combined matrices and recompute FK
    JointPositions positions {};
    JointTransforms transforms {};
    forwardKinematics(angles, model, positions, transforms);

    // Find current distance from target
    const Vector3 positionError = Vector3Subtract(target, positions.back());

    // Find current Jacobian
    float jacobian[3][kJointCount] {};
    computePositionJacobian(positions, transforms, jacobian);

    // Will compute matrix from a damped least-squares formula:
    // A = J * J^T + lambda^2 I
    float jacobianTimesTranspose[3][3] {};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
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
        std::sqrt(std::fabs(determinant3x3(jacobianTimesTranspose)));

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
    float dampedSystem[3][3] {};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            dampedSystem[row][column] = jacobianTimesTranspose[row][column];
            if (row == column) {
                dampedSystem[row][column] += effectiveDamping * effectiveDamping;
            }
        }
    }

    // Solve the unknown vector x using Cramer's Rule func
    const float rightHandSide[3] = {
        positionError.x,
        positionError.y,
        positionError.z,
    };
    float solution[3] {};
    if (!solve3x3(dampedSystem, rightHandSide, solution)) {
        return Vector3Length(positionError);
    }

    // Finds delta theta = J^T * x, where delta theta is the tiny change in
    // joint angles per iteration.
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        const float correction = jacobian[0][joint] * solution[0]
            + jacobian[1][joint] * solution[1]
            + jacobian[2][joint] * solution[2];

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
