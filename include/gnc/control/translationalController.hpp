/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#ifndef GNC_CONTROL_TRANSLATIONAL_CONTROLLER_HPP
#define GNC_CONTROL_TRANSLATIONAL_CONTROLLER_HPP

#include <Eigen/Dense>

namespace gnc::control {

class TranslationalController {
public:
    TranslationalController(double kp, double kd) 
        : kp_(kp), kd_(kd) {}

    Eigen::Vector3d computeControlForce(
        const Eigen::Vector3d& r_current,
        const Eigen::Vector3d& v_current,
        const Eigen::Vector3d& f_desired,
        double dt
    ) {
        // Store previous error for derivative
        Eigen::Vector3d prev_error = error_;
        
        // Compute current error (simplified - just track desired force)
        error_ = f_desired - force_prev_;
        
        // Compute error derivative
        Eigen::Vector3d error_dot = (error_ - prev_error) / dt;
        
        // PD control law
        Eigen::Vector3d control_force = kp_ * error_ + kd_ * error_dot;
        
        // Store current force for next iteration
        force_prev_ = control_force;
        
        return control_force;
    }

private:
    double kp_;
    double kd_;
    Eigen::Vector3d error_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d force_prev_ = Eigen::Vector3d::Zero();
};

} // namespace gnc::control

#endif // GNC_CONTROL_TRANSLATIONAL_CONTROLLER_HPP 