/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#ifndef EOM_HPP
#define EOM_HPP

#include <Eigen/Dense>
#include "../astro/gravity.hpp"
#include "../numerics/quaternion.hpp"

namespace dynamics {

class EOM {
public:
    // State vector size constants
    static constexpr int STATE_SIZE = 13;  // [r(3), v(3), q(4), w(3)]
    static constexpr int POS_IDX = 0;
    static constexpr int VEL_IDX = 3;
    static constexpr int QUAT_IDX = 6;
    static constexpr int OMEGA_IDX = 10;

    /**
     * Equations of Motion for 6-DOF spacecraft dynamics
     * State vector definition:
     * - Position [0:2]: Position in GCRS frame (m)
     * - Velocity [3:5]: Velocity in GCRS frame (m/s)
     * - Quaternion [6:9]: Rotation from GCRS to body frame (scalar first)
     * - Angular velocity [10:12]: Body angular velocity in body frame (rad/s)
     */

    /**
     * Computes state derivative for integration
     * @param state Current state vector [r_eci; v_eci; q_eci2body; w_body]
     * @param inertia Inertia tensor in body frame
     * @param external_torque External torques in body frame
     * @param external_force External forces in body frame
     * @return State derivative vector
     */
    static Eigen::VectorXd computeStateDerivative(
        const Eigen::VectorXd& state,
        const Eigen::Matrix3d& inertia,
        const Eigen::Vector3d& external_torque = Eigen::Vector3d::Zero(),
        const Eigen::Vector3d& external_force = Eigen::Vector3d::Zero()
    ) {
        Eigen::VectorXd state_dot = Eigen::VectorXd::Zero(STATE_SIZE);

        // Extract state components
        Eigen::Vector3d r_eci = state.segment<3>(POS_IDX);
        Eigen::Vector3d v_eci = state.segment<3>(VEL_IDX);
        Eigen::Vector4d q_eci2body = state.segment<4>(QUAT_IDX);
        Eigen::Vector3d w_body = state.segment<3>(OMEGA_IDX);

        // Position derivative is velocity
        state_dot.segment<3>(POS_IDX) = v_eci;

        // Velocity derivative is acceleration (gravity only for now)
        state_dot.segment<3>(VEL_IDX) = astro::Gravity::computeTwoBodyAcceleration(r_eci) + external_force;

        // Compute orbital angular velocity in GCRS
        Eigen::Vector3d w_orbit = r_eci.cross(v_eci) / r_eci.squaredNorm();

        // Convert orbital angular velocity to body frame
        Eigen::Matrix3d R_eci2body = numerics::quaternionToRotationMatrix(q_eci2body);
        Eigen::Vector3d w_orbit_body = R_eci2body * w_orbit;

        // Total angular velocity for quaternion propagation (body relative to GCRS)
        Eigen::Vector3d w_total = w_body + w_orbit_body;

        // Quaternion derivative using total angular velocity
        state_dot.segment<4>(QUAT_IDX) = numerics::computeQuaternionDerivative(q_eci2body, w_total);

        // Angular acceleration (Euler's equation)
        Eigen::Vector3d w_dot = inertia.inverse() * 
            (external_torque - w_body.cross(inertia * w_body));
        state_dot.segment<3>(OMEGA_IDX) = w_dot;

        return state_dot;
    }

    /**
     * Creates initial state vector from components
     */
    static Eigen::VectorXd createStateVector(
        const Eigen::Vector3d& r_eci,
        const Eigen::Vector3d& v_eci,
        const Eigen::Vector4d& q_eci2body,
        const Eigen::Vector3d& w_body
    ) {
        Eigen::VectorXd state(STATE_SIZE);
        state << r_eci, v_eci, q_eci2body, w_body;
        return state;
    }
};

} // namespace dynamics

#endif // EOM_HPP
