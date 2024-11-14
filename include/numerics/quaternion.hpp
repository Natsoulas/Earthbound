/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <Eigen/Dense>

namespace numerics {

/**
 * Computes quaternion derivative for integration
 * q_dot = 1/2 * q ⊗ ω where ω = [0; ω_body]
 * @param q Current quaternion (scalar first) from GCRS to body frame
 * @param w Angular velocity vector in body frame (rad/s)
 * @return Quaternion derivative
 */
static Eigen::Vector4d computeQuaternionDerivative(
    const Eigen::Vector4d& q,
    const Eigen::Vector3d& w
) {
    // Form pure quaternion from angular velocity
    Eigen::Vector4d w_quat;
    w_quat << 0.0, w.x(), w.y(), w.z();
    
    // Quaternion derivative = 1/2 * q ⊗ ω
    Eigen::Vector4d q_dot;
    q_dot(0) = -0.5 * (q(1)*w.x() + q(2)*w.y() + q(3)*w.z());
    q_dot(1) =  0.5 * (q(0)*w.x() + q(2)*w.z() - q(3)*w.y());
    q_dot(2) =  0.5 * (q(0)*w.y() + q(3)*w.x() - q(1)*w.z());
    q_dot(3) =  0.5 * (q(0)*w.z() + q(1)*w.y() - q(2)*w.x());
    
    return q_dot;
}

/**
 * Converts quaternion to rotation matrix
 * @param q Quaternion (scalar first) representing rotation from frame A to frame B
 * @return Rotation matrix R_A2B that transforms vectors from frame A to frame B
 */
static Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Vector4d& q) {
    double qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    
    Eigen::Matrix3d R;
    R << 1-2*(qy*qy+qz*qz),   2*(qx*qy-qw*qz),   2*(qx*qz+qw*qy),
         2*(qx*qy+qw*qz),   1-2*(qx*qx+qz*qz),   2*(qy*qz-qw*qx),
         2*(qx*qz-qw*qy),     2*(qy*qz+qw*qx), 1-2*(qx*qx+qy*qy);
    
    return R;
}

static Eigen::Vector3d rotationMatrixToAxisAngle(const Eigen::Matrix3d& R) {
    // Get the skew-symmetric part of R
    Eigen::Matrix3d skew = 0.5 * (R - R.transpose());
    
    // Extract axis-angle representation
    Eigen::Vector3d axis(skew(2,1), skew(0,2), skew(1,0));
    double angle = axis.norm();
    
    if (angle < 1e-10) {
        return Eigen::Vector3d::Zero();
    }
    
    axis /= angle;  // Normalize axis
    
    // Use trace to determine angle in correct quadrant
    double trace = R.trace();
    if (trace < -1.0) {
        angle = M_PI;
    } else {
        angle = std::acos((trace - 1.0) / 2.0);
    }
    
    return angle * axis;
}

/**
 * @brief Convert rotation matrix to quaternion (scalar first convention)
 */
static Eigen::Vector4d rotationMatrixToQuaternion(const Eigen::Matrix3d& R) {
    Eigen::Vector4d q;
    double tr = R.trace();

    if (tr > 0) {
        double S = sqrt(tr + 1.0) * 2;
        q(0) = 0.25 * S;
        q(1) = (R(2,1) - R(1,2)) / S;
        q(2) = (R(0,2) - R(2,0)) / S;
        q(3) = (R(1,0) - R(0,1)) / S;
    } else if ((R(0,0) > R(1,1)) && (R(0,0) > R(2,2))) {
        double S = sqrt(1.0 + R(0,0) - R(1,1) - R(2,2)) * 2;
        q(0) = (R(2,1) - R(1,2)) / S;
        q(1) = 0.25 * S;
        q(2) = (R(0,1) + R(1,0)) / S;
        q(3) = (R(0,2) + R(2,0)) / S;
    } else if (R(1,1) > R(2,2)) {
        double S = sqrt(1.0 + R(1,1) - R(0,0) - R(2,2)) * 2;
        q(0) = (R(0,2) - R(2,0)) / S;
        q(1) = (R(0,1) + R(1,0)) / S;
        q(2) = 0.25 * S;
        q(3) = (R(1,2) + R(2,1)) / S;
    } else {
        double S = sqrt(1.0 + R(2,2) - R(0,0) - R(1,1)) * 2;
        q(0) = (R(1,0) - R(0,1)) / S;
        q(1) = (R(0,2) + R(2,0)) / S;
        q(2) = (R(1,2) + R(2,1)) / S;
        q(3) = 0.25 * S;
    }
    return q;
}

/**
 * @brief Compute quaternion conjugate (scalar first convention)
 */
static Eigen::Vector4d quaternionConjugate(const Eigen::Vector4d& q) {
    return Eigen::Vector4d(q(0), -q(1), -q(2), -q(3));
}

/**
 * @brief Multiply two quaternions (scalar first convention)
 */
static Eigen::Vector4d quaternionMultiply(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2) {
    Eigen::Vector4d q;
    q(0) = q1(0)*q2(0) - q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3);
    q(1) = q1(0)*q2(1) + q1(1)*q2(0) + q1(2)*q2(3) - q1(3)*q2(2);
    q(2) = q1(0)*q2(2) - q1(1)*q2(3) + q1(2)*q2(0) + q1(3)*q2(1);
    q(3) = q1(0)*q2(3) + q1(1)*q2(2) - q1(2)*q2(1) + q1(3)*q2(0);
    return q;
}

} // namespace numerics

#endif // QUATERNION_HPP
