#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <raylib.h>
#include <vector>

// Code structure: STRUCTURES, FUNCTIONS, MAIN

// ======== NEW STRUCTS ========

// Create the structure of the DH row for 6 DOF. theta not stored because its live variable
struct DHRow {float a, alpha, d; };

// ======== FUNCTIONS =========

// Function det 3x3 matrix just for efficiency
float det3x3(float A[3][3]);

Matrix DHTransform(float theta, float d, float a, float alpha);

// Function for building 3x6 Jacobian for the IK Function
void ComputeJacobian(Vector3 jointPositions[7], Matrix combinedMatrices[7], float J[3][6]);

// Function for solving a 3x3 matrix using Cramer's Rule (this will be needed)
// Solves A . x = b for unknown vector x, given 3x3 A and 3vector b
void Solve3x3(float A[3][3], float b[3], float x[3]);

// Forward Kinematics Function (Applies DH row for every joint and transforms each joint)
void ForwardKinematics(float theta[6], DHRow joints[6], Vector3 jointPositions[7], Matrix combinedMatrices[7]);

// Function Inverse Kinematics (IK)
// Takes current joint angles (theta), arm locations via DH, target position, and the damping constant lambda which prevents singularities and surpassing mechanical machine limits. Returns the distance to the target. Will be used iteratively in the main loop until target coordinate reached.

// Will compute matrix from a damped least-squares formula:  A = J·J^T + λ²I
float IKstep(float theta[6], DHRow joints[6], Vector3 target, float lambda, float thetaMin[6], float thetaMax[6], bool limitJointsMode);

// Function applied different pathways once a different shape is toggled
std::vector<Vector3> CreatePathwayPoints(int toggleShape, float radius, int& waypointAmount);

#endif