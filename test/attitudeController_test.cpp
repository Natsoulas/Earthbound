/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */
 
#include <gtest/gtest.h>
#include "../include/gnc/control/attitudeController.hpp"
#include "../include/config/simpleSat.hpp"

class AttitudeControllerTest : public ::testing::Test {
protected:
    const double tol = 1e-10;
    gnc::control::GeometricAttitudeController controller{
        0.1, 0.2, config::SimpleSat::inertia
    };

    // Test initial conditions
    const Eigen::Vector3d r_test{7000000.0, 0.0, 0.0};  // 7000 km in x
    const Eigen::Vector3d v_test{0.0, 7546.0, 0.0};     // Circular orbit velocity in y
    const Eigen::Vector4d q_test{1.0, 0.0, 0.0, 0.0};   // Identity quaternion
    const Eigen::Vector3d w_test{0.0, 0.0, 0.0};        // Zero angular velocity
};

TEST_F(AttitudeControllerTest, ZeroErrorCase) {
    // For identity quaternion and zero angular velocity, 
    // torque should be very small (only orbital rate compensation)
    Eigen::Vector3d torque = controller.computeControlTorque(
        r_test, v_test, q_test, w_test, 0.1);
    
    // Check torque magnitude is small
    EXPECT_LT(torque.norm(), 1e-3);
}

TEST_F(AttitudeControllerTest, LargeAttitudeError) {
    // Create 90-degree rotation error around Z
    const double sqrt2_2 = std::sqrt(2.0)/2.0;
    Eigen::Vector4d q_error(sqrt2_2, 0.0, 0.0, sqrt2_2);
    
    Eigen::Vector3d torque = controller.computeControlTorque(
        r_test, v_test, q_error, w_test, 0.1);
    
    // Torque should not be zero but should respect max limits
    EXPECT_GT(torque.norm(), 1e-6);
    EXPECT_LE(torque.norm(), 0.001);  // Changed to less than or equal
}

TEST_F(AttitudeControllerTest, AngularVelocityDamping) {
    // Test with zero attitude error but non-zero angular velocity
    Eigen::Vector3d w_nonzero(0.1, -0.1, 0.05);
    
    Eigen::Vector3d torque = controller.computeControlTorque(
        r_test, v_test, q_test, w_nonzero, 0.1);
    
    // Torque should oppose angular velocity
    for(int i = 0; i < 3; i++) {
        if(std::abs(w_nonzero[i]) > 1e-10) {
            EXPECT_TRUE(torque[i] * w_nonzero[i] < 0);
        }
    }
}
