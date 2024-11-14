/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include <gtest/gtest.h>
#include "../include/config/simpleSat.hpp"

TEST(SimpleSatTest, InertiaProperties) {
    // Test that inertia tensor is symmetric
    EXPECT_TRUE(config::SimpleSat::inertia.isApprox(
        config::SimpleSat::inertia.transpose()));
    
    // Test that inertia tensor is positive definite
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(config::SimpleSat::inertia);
    EXPECT_GT(eigensolver.eigenvalues().minCoeff(), 0.0);
    
    // Test physical constraints on moments of inertia
    double Ixx = config::SimpleSat::inertia(0,0);
    double Iyy = config::SimpleSat::inertia(1,1);
    double Izz = config::SimpleSat::inertia(2,2);
    
    // Triangle inequality for rigid body inertia
    EXPECT_GT(Ixx + Iyy, Izz);
    EXPECT_GT(Iyy + Izz, Ixx);
    EXPECT_GT(Izz + Ixx, Iyy);
}

TEST(SimpleSatTest, InitialConditions) {
    // Test that initial position is at specified altitude
    double altitude = config::SimpleSat::initial_position.norm() - constants::earth::RADIUS;
    EXPECT_NEAR(altitude, 500000.0, 1.0);  // 500 km altitude with 1m tolerance
    
    // Test that initial velocity is approximately circular
    double v_circular = std::sqrt(constants::earth::MU / 
                                config::SimpleSat::initial_position.norm());
    EXPECT_NEAR(config::SimpleSat::initial_velocity.norm(), v_circular, 1.0);
    
    // Test that initial quaternion is normalized
    EXPECT_NEAR(config::SimpleSat::initial_quaternion.norm(), 1.0, 1e-10);
} 