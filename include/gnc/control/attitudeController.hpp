/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */
 
#ifndef GNC_ATTITUDE_CONTROLLER_HPP
#define GNC_ATTITUDE_CONTROLLER_HPP

#include <Eigen/Dense>
#include "../../astro/frames.hpp"
#include "../../numerics/quaternion.hpp"

namespace gnc::control {

class GeometricAttitudeController {
public:
    GeometricAttitudeController(double kp, double kd, const Eigen::Matrix3d& inertia) 
        : kp_(kp), kd_(kd), inertia_(inertia) {}

    Eigen::Vector3d computeControlTorque(
        const Eigen::Vector3d& r_gcrs,
        const Eigen::Vector3d& v_gcrs,
        const Eigen::Vector4d& q_gcrs2body,
        const Eigen::Vector3d& w_body,
        double dt
    ) {
        // Get desired RSW frame
        Eigen::Matrix3d R_gcrs2rsw = astro::Frames::computeRSWFrame(r_gcrs, v_gcrs);
        Eigen::Matrix3d R_current = numerics::quaternionToRotationMatrix(q_gcrs2body);
        
        // Compute attitude error in SO(3)
        Eigen::Matrix3d R_error = R_current.transpose() * R_gcrs2rsw;
        Eigen::Matrix3d E = 0.5 * (R_error.transpose() - R_error);
        
        // Extract vector form of error
        Eigen::Vector3d e_R(E(2,1), E(0,2), E(1,0));
        
        // Compute desired angular velocity
        double orbital_rate = v_gcrs.norm() / r_gcrs.norm();
        Eigen::Vector3d w_desired = R_current.transpose() * R_gcrs2rsw * 
                                  Eigen::Vector3d(0, 0, -orbital_rate);
        
        // Angular velocity error
        Eigen::Vector3d e_w = w_body - w_desired;
        
        // Geometric control law on SO(3)
        Eigen::Vector3d control_torque = inertia_ * (
            -kp_ * e_R 
            -kd_ * e_w
        );

        // Smooth saturation function
        const double max_torque = 0.001;  // Further reduced
        double torque_mag = control_torque.norm();
        if (torque_mag > max_torque) {
            double scale = max_torque * (1 - exp(-torque_mag/max_torque)) / torque_mag;
            control_torque *= scale;
        }

        return control_torque;
    }

private:
    double kp_;
    double kd_;
    Eigen::Matrix3d inertia_;
};

} // namespace control

#endif // GNC_ATTITUDE_CONTROLLER_HPP
