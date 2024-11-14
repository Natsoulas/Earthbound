#include <gtest/gtest.h>
#include "../include/astro/orbitalMechanics.hpp"

class OrbitalMechanicsTest : public ::testing::Test {
protected:
    const double tol = 1e-10;
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    // Known circular orbit test case
    const Eigen::Vector3d circular_r{7000000.0, 0.0, 0.0};  // 7000 km circular orbit
    const Eigen::Vector3d circular_v{0.0, 7546.0, 0.0};     // Approximate circular velocity
    
    // Known elliptical orbit test case
    const Eigen::Vector3d elliptical_r{7000000.0, 0.0, 0.0};
    const Eigen::Vector3d elliptical_v{0.0, 9000.0, 0.0};   // Higher velocity for elliptical orbit
};

TEST_F(OrbitalMechanicsTest, CircularOrbitElements) {
    Vector6d elements = astro::OrbitalMechanics::cartesianToKeplerian(circular_r, circular_v);
    
    // For circular orbit:
    // - eccentricity should be near 0
    // - radius should equal semi-major axis
    EXPECT_NEAR(elements[1], 0.0, 1e-3);  // eccentricity
    
    // Use relative tolerance for semi-major axis (0.01% error allowed)
    double relative_error = std::abs(elements[0] - circular_r.norm()) / circular_r.norm();
    EXPECT_LT(relative_error, 1e-4);
}

TEST_F(OrbitalMechanicsTest, ElementBounds) {
    Vector6d elements = astro::OrbitalMechanics::cartesianToKeplerian(elliptical_r, elliptical_v);
    
    // Test physical constraints on orbital elements
    EXPECT_GT(elements[0], 0.0);  // Semi-major axis should be positive
    EXPECT_GE(elements[1], 0.0);  // Eccentricity should be non-negative
    EXPECT_GE(elements[2], 0.0);  // Inclination should be between 0 and π
    EXPECT_LE(elements[2], constants::math::PI);
    EXPECT_GE(elements[3], 0.0);  // RAAN should be between 0 and 2π
    EXPECT_LE(elements[3], constants::math::TWO_PI);
    EXPECT_GE(elements[4], 0.0);  // Argument of periapsis should be between 0 and 2π
    EXPECT_LE(elements[4], constants::math::TWO_PI);
    EXPECT_GE(elements[5], 0.0);  // True anomaly should be between 0 and 2π
    EXPECT_LE(elements[5], constants::math::TWO_PI);
} 