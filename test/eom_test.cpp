/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include <gtest/gtest.h>
#include "../include/dynamics/eom.hpp"
#include "../include/config/simpleSat.hpp"

class EOMTest : public ::testing::Test {
protected:
    const double tol = 1e-10;
    
    // Test initial conditions
    const Eigen::Vector3d r_test{7000000.0, 0.0, 0.0};
    const Eigen::Vector3d v_test{0.0, 7546.0, 0.0};
    const Eigen::Vector4d q_test{1.0, 0.0, 0.0, 0.0};  // Identity quaternion
    const Eigen::Vector3d w_test{0.01, 0.01, 0.01};    // Small angular velocity
};

TEST_F(EOMTest, StateVectorCreation) {
    Eigen::VectorXd state = dynamics::EOM::createStateVector(r_test, v_test, q_test, w_test);
    
    // Test state vector size
    EXPECT_EQ(state.size(), dynamics::EOM::STATE_SIZE);
    
    // Test state vector components
    EXPECT_TRUE(state.segment<3>(0).isApprox(r_test, tol));
    EXPECT_TRUE(state.segment<3>(3).isApprox(v_test, tol));
    EXPECT_TRUE(state.segment<4>(6).isApprox(q_test, tol));
    EXPECT_TRUE(state.segment<3>(10).isApprox(w_test, tol));
}

TEST_F(EOMTest, QuaternionNormalization) {
    // Create state vector with non-normalized quaternion
    Eigen::Vector4d q_unnormalized{2.0, 0.0, 0.0, 0.0};
    Eigen::VectorXd state = dynamics::EOM::createStateVector(r_test, v_test, q_unnormalized, w_test);
    
    // Compute derivative
    Eigen::VectorXd state_dot = dynamics::EOM::computeStateDerivative(
        state, config::SimpleSat::inertia);
    
    // Extract quaternion derivative
    Eigen::Vector4d q_dot = state_dot.segment<4>(6);
    
    // Verify that quaternion derivative maintains unit norm
    // d/dt(q·q) = 2q·q_dot should be zero for unit quaternion
    double norm_derivative = 2.0 * q_unnormalized.dot(q_dot);
    EXPECT_NEAR(norm_derivative, 0.0, tol);
}

TEST_F(EOMTest, AngularMomentumConservation) {
    // Create initial state
    Eigen::VectorXd state = dynamics::EOM::createStateVector(r_test, v_test, q_test, w_test);
    
    // Compute derivative
    Eigen::VectorXd state_dot = dynamics::EOM::computeStateDerivative(
        state, config::SimpleSat::inertia);
    
    // Extract angular velocity derivative
    Eigen::Vector3d w_dot = state_dot.segment<3>(10);
    
    // For torque-free motion, the angular momentum should be conserved
    // H = I·ω, so d/dt(H) = I·ω_dot + ω×(I·ω) should be zero
    Eigen::Vector3d H_dot = config::SimpleSat::inertia * w_dot + 
                           w_test.cross(config::SimpleSat::inertia * w_test);
    
    EXPECT_NEAR(H_dot.norm(), 0.0, tol);
}

TEST_F(EOMTest, AttitudeFrameRelationships) {
    // Create a test state with known attitude
    // Rotate 90 degrees around Z axis
    const double sqrt2_2 = std::sqrt(2.0)/2.0;
    Eigen::Vector4d q_gcrs2body(sqrt2_2, 0.0, 0.0, sqrt2_2); // 90-deg Z rotation
    
    // Create a vector in GCRS frame
    Eigen::Vector3d v_gcrs(1.0, 0.0, 0.0);
    
    // Create state vector with this attitude
    Eigen::VectorXd state = dynamics::EOM::createStateVector(
        r_test, v_test, q_gcrs2body, w_test);
    
    // Extract quaternion and convert to rotation matrix
    Eigen::Vector4d q = state.segment<4>(dynamics::EOM::QUAT_IDX);
    Eigen::Matrix3d R_gcrs2body = numerics::quaternionToRotationMatrix(q);
    
    // Transform vector from GCRS to body
    Eigen::Vector3d v_body = R_gcrs2body * v_gcrs;
    
    // After 90-deg Z rotation, [1,0,0] should become [0,1,0]
    EXPECT_NEAR(v_body.x(), 0.0, tol);
    EXPECT_NEAR(v_body.y(), 1.0, tol);
    EXPECT_NEAR(v_body.z(), 0.0, tol);
}

TEST_F(EOMTest, OrbitalQuaternionEvolution) {
    // Create initial state for circular orbit in XY plane
    Eigen::Vector3d r_init(7000000.0, 0.0, 0.0);  // 7000 km radius
    double v_circ = std::sqrt(constants::earth::MU / r_init.norm());
    Eigen::Vector3d v_init(0.0, v_circ, 0.0);
    Eigen::Vector4d q_init(1.0, 0.0, 0.0, 0.0);  // Identity initial attitude
    Eigen::Vector3d w_init(0.0, 0.0, 0.0);  // No initial body rotation

    Eigen::VectorXd state = dynamics::EOM::createStateVector(
        r_init, v_init, q_init, w_init);
    
    // Integrate for 1/4 orbit
    double orbit_period = 2.0 * M_PI * std::sqrt(std::pow(r_init.norm(), 3) / constants::earth::MU);
    double dt = 1.0;
    double t = 0;
    
    while (t < orbit_period/4) {
        Eigen::VectorXd state_dot = dynamics::EOM::computeStateDerivative(
            state, config::SimpleSat::inertia);
        state += state_dot * dt;
        t += dt;
    }
    
    // After 1/4 orbit, body frame should have rotated ~90 degrees around orbit normal
    Eigen::Vector4d q_final = state.segment<4>(dynamics::EOM::QUAT_IDX);
    Eigen::Vector4d q_expected(std::cos(M_PI/4), 0, 0, std::sin(M_PI/4));
    
    EXPECT_TRUE(q_final.isApprox(q_expected, 0.1));  // Loose tolerance due to simple integration
}

TEST_F(EOMTest, ExternalTorqueResponse) {
    // Create asymmetric inertia tensor
    Eigen::Matrix3d test_inertia;
    test_inertia << 100.0, 0.0,   0.0,
                    0.0,   200.0, 0.0,
                    0.0,   0.0,   300.0;

    // Initial conditions: rotation around X AND Z axes
    Eigen::Vector3d w_init(5.0, 0.0, 2.0);  // Combined rotation
    Eigen::VectorXd state = dynamics::EOM::createStateVector(
        r_test, v_test, q_test, w_init);

    // Apply torque around Y axis
    Eigen::Vector3d external_torque(0.0, 1.0, 0.0);

    // Get state derivative
    Eigen::VectorXd state_dot = dynamics::EOM::computeStateDerivative(
        state, test_inertia, external_torque);
    Eigen::Vector3d w_dot = state_dot.segment<3>(dynamics::EOM::OMEGA_IDX);

    // Manually compute expected angular acceleration
    Eigen::Vector3d w = w_init;
    Eigen::Vector3d Iw = test_inertia * w;
    Eigen::Vector3d w_cross_Iw = w.cross(Iw);
    
    // Expected angular acceleration: I⁻¹(τ - ω × (Iω))
    Eigen::Vector3d expected_w_dot = test_inertia.inverse() * 
        (external_torque - w_cross_Iw);

    // Test each component
    EXPECT_NEAR(w_dot.x(), expected_w_dot.x(), tol);
    EXPECT_NEAR(w_dot.y(), expected_w_dot.y(), tol);
    EXPECT_NEAR(w_dot.z(), expected_w_dot.z(), tol);

    // With rotation around both X and Z axes, we should see coupling
    EXPECT_TRUE(std::abs(w_cross_Iw.norm()) > tol);
} 