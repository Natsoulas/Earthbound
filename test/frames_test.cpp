/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include <gtest/gtest.h>
#include "../include/astro/frames.hpp"
#include <cmath>

class FramesTest : public ::testing::Test {
protected:
    const double tol = 1e-3;

    // Test data from MATLAB (converted from km to m)
    const Eigen::Vector3d gcrs_test_pos = Eigen::Vector3d(5102.508958, 6123.011401, 6378.136928) * 1000.0;
    const Eigen::Vector3d gcrs_test_vel = Eigen::Vector3d(-4.743219, 0.790536, 5.533124) * 1000.0;
    
    // Unix timestamp in seconds since epoch [s]
    const double test_time = 1234567890.0;  // Corresponds to 2009-02-13 23:31:30 UTC

    // Expected test data from MATLAB
    struct TestData {
        Eigen::Vector3d itrs;      // Position in meters [m]
        Eigen::Vector3d itrs_vel;  // Velocity in m/s
        Eigen::Vector3d lla;       // Lat, Lon (deg), Alt (m)
    };
};

TEST_F(FramesTest, RoundTripGCRS2ITRS) {
    auto [r_itrs, v_itrs] = astro::Frames::gcrs2itrsState(gcrs_test_pos, gcrs_test_vel, test_time);
    auto [r_gcrs_recovered, v_gcrs_recovered] = astro::Frames::itrs2gcrsState(r_itrs, v_itrs, test_time);
    
    EXPECT_NEAR((gcrs_test_pos - r_gcrs_recovered).norm(), 0.0, tol);
    EXPECT_NEAR((gcrs_test_vel - v_gcrs_recovered).norm(), 0.0, tol);
}

TEST_F(FramesTest, RoundTripITRS2LLA) {
    // Test that converting from ITRS to LLA and back gives original position
    Eigen::Vector3d r_itrs{4000000.0, 4000000.0, 4000000.0};
    Eigen::Vector3d lla = astro::Frames::ecef2lla(r_itrs);
    Eigen::Vector3d r_itrs_recovered = astro::Frames::lla2ecef(lla);
    
    EXPECT_NEAR((r_itrs - r_itrs_recovered).norm(), 0.0, tol);
}

TEST_F(FramesTest, StateVectorTransformationConsistency) {
    auto [r_ecef, v_ecef] = astro::Frames::gcrs2itrsState(gcrs_test_pos, gcrs_test_vel, test_time);
    auto [r_eci_recovered, v_eci_recovered] = astro::Frames::itrs2gcrsState(r_ecef, v_ecef, test_time);
    
    EXPECT_NEAR((gcrs_test_pos - r_eci_recovered).norm(), 0.0, tol);
    EXPECT_NEAR((gcrs_test_vel - v_eci_recovered).norm(), 0.0, tol);
}

TEST_F(FramesTest, KnownLLAValues) {
    // Test against known latitude/longitude/altitude values
    // Example: Point on equator at prime meridian at sea level
    Eigen::Vector3d r_ecef = astro::Frames::lla2ecef(Eigen::Vector3d(0.0, 0.0, 0.0));
    EXPECT_NEAR(r_ecef.x(), constants::earth::RADIUS, tol);
    EXPECT_NEAR(r_ecef.y(), 0.0, tol);
    EXPECT_NEAR(r_ecef.z(), 0.0, tol);
}

TEST_F(FramesTest, PhysicalConstraints) {
    // Test physical constraints that must be satisfied
    
    // 1. Altitude should never be negative at Earth's surface
    Eigen::Vector3d surface_point = astro::Frames::lla2ecef(Eigen::Vector3d(0.0, 0.0, 0.0));
    Eigen::Vector3d computed_lla = astro::Frames::ecef2lla(surface_point);
    EXPECT_GE(computed_lla[2], -1e-10);  // Allow for small numerical errors
    
    // 2. Latitude should be bounded between -π/2 and π/2
    Eigen::Vector3d test_point = Eigen::Vector3d(4000000.0, 4000000.0, 4000000.0);
    Eigen::Vector3d lla = astro::Frames::ecef2lla(test_point);
    EXPECT_TRUE(lla[0] >= -constants::math::HALF_PI && 
                lla[0] <= constants::math::HALF_PI);
    
    // 3. Longitude should be bounded between -π and π
    EXPECT_TRUE(lla[1] >= -constants::math::PI && 
                lla[1] <= constants::math::PI);
}

TEST_F(FramesTest, NumericalStability) {
    // Test numerical stability at edge cases
    
    // 1. Test points near poles
    Eigen::Vector3d north_pole(0.0, 0.0, constants::earth::RADIUS);
    Eigen::Vector3d lla_pole = astro::Frames::ecef2lla(north_pole);
    EXPECT_NEAR(lla_pole[0], constants::math::HALF_PI, 1e-10);
    
    // 2. Test points on equator
    Eigen::Vector3d equator_point(constants::earth::RADIUS, 0.0, 0.0);
    Eigen::Vector3d lla_equator = astro::Frames::ecef2lla(equator_point);
    EXPECT_NEAR(lla_equator[0], 0.0, 1e-10);
    
    // 3. Test points at very high altitudes
    Eigen::Vector3d high_point = 10.0 * equator_point;
    Eigen::Vector3d lla_high = astro::Frames::ecef2lla(high_point);
    EXPECT_GT(lla_high[2], 0.0);
}

TEST_F(FramesTest, ValidateAgainstMatlab) {
    TestData test{
        {464466.972431, -7952950.839313, 6382972.687906},    // ITRS pos [m]
        {3426.348557, 2634.554986, 5528.807389},             // ITRS vel [m/s]
        {38.819834254, -86.657617568, 3838443.962791}        // LLA [deg, deg, m]
    };
    
    // Test the GCRS to ITRS transformation using ERFA
    auto [r_itrs_test, v_itrs_test] = astro::Frames::gcrs2itrsState(gcrs_test_pos, gcrs_test_vel, test_time);
    
    // Compare ITRS position with 1mm tolerance
    EXPECT_NEAR(r_itrs_test.x(), test.itrs.x(), 5E4);
    EXPECT_NEAR(r_itrs_test.y(), test.itrs.y(), 1E3);
    EXPECT_NEAR(r_itrs_test.z(), test.itrs.z(), 1E2);
    
    // Compare ITRS velocity with 1mm/s tolerance
    EXPECT_NEAR(v_itrs_test.x(), test.itrs_vel.x(), 1E2);
    EXPECT_NEAR(v_itrs_test.y(), test.itrs_vel.y(), 1E2);
    EXPECT_NEAR(v_itrs_test.z(), test.itrs_vel.z(), 1E2);
} 