/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include <gtest/gtest.h>
#include "../include/astro/gravity.hpp"

class GravityTest : public ::testing::Test {
protected:
    const double tol = 1e-10;
    
    // Test points
    const Eigen::Vector3d equatorial_point{constants::earth::RADIUS, 0.0, 0.0};
    const Eigen::Vector3d polar_point{0.0, 0.0, constants::earth::RADIUS};
    const Eigen::Vector3d arbitrary_point{4000000.0, 4000000.0, 4000000.0};
};

TEST_F(GravityTest, TwoBodyAcceleration) {
    // Test that acceleration points toward center
    Eigen::Vector3d acc = astro::Gravity::computeTwoBodyAcceleration(arbitrary_point);
    
    // Acceleration should point opposite to position vector
    EXPECT_NEAR(acc.normalized().dot(-arbitrary_point.normalized()), 1.0, tol);
    
    // Test inverse square law
    double r = arbitrary_point.norm();
    double expected_mag = constants::earth::MU / (r * r);
    EXPECT_NEAR(acc.norm(), expected_mag, tol);
}

TEST_F(GravityTest, J2Effect) {
    // Get accelerations with and without J2
    Eigen::Vector3d acc_with_j2 = astro::Gravity::computeAcceleration(polar_point);
    Eigen::Vector3d acc_two_body = astro::Gravity::computeTwoBodyAcceleration(polar_point);
    
    // J2 effect should be stronger at poles than at equator
    double j2_effect_pole = (acc_with_j2 - acc_two_body).norm();
    
    acc_with_j2 = astro::Gravity::computeAcceleration(equatorial_point);
    acc_two_body = astro::Gravity::computeTwoBodyAcceleration(equatorial_point);
    double j2_effect_equator = (acc_with_j2 - acc_two_body).norm();
    
    EXPECT_GT(j2_effect_pole, j2_effect_equator);
}

TEST_F(GravityTest, ConservativeField) {
    // For conservative field, curl should be zero
    // Test using finite differences
    const double h = 1.0;  // Step size for finite differences
    Eigen::Vector3d point = arbitrary_point;
    
    // Compute partial derivatives
    Eigen::Vector3d dx = (astro::Gravity::computeAcceleration(point + Eigen::Vector3d(h,0,0)) -
                         astro::Gravity::computeAcceleration(point - Eigen::Vector3d(h,0,0))) / (2*h);
    Eigen::Vector3d dy = (astro::Gravity::computeAcceleration(point + Eigen::Vector3d(0,h,0)) -
                         astro::Gravity::computeAcceleration(point - Eigen::Vector3d(0,h,0))) / (2*h);
    Eigen::Vector3d dz = (astro::Gravity::computeAcceleration(point + Eigen::Vector3d(0,0,h)) -
                         astro::Gravity::computeAcceleration(point - Eigen::Vector3d(0,0,h))) / (2*h);
    
    // Compute curl components
    double curl_x = dz.y() - dy.z();
    double curl_y = dx.z() - dz.x();
    double curl_z = dy.x() - dx.y();
    
    // Check if curl components are close to zero
    EXPECT_NEAR(curl_x, 0.0, tol);
    EXPECT_NEAR(curl_y, 0.0, tol);
    EXPECT_NEAR(curl_z, 0.0, tol);
} 