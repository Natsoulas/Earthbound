/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#ifndef GNC_GUIDANCE_HOHMANN_HPP
#define GNC_GUIDANCE_HOHMANN_HPP

#include <Eigen/Dense>
#include "../../astro/orbitalMechanics.hpp"
#include "../../constants/constants.hpp"
#include "../../config/simpleSat.hpp"

namespace gnc::guidance {

enum class ApsisType {
    PERIGEE,
    APOGEE
};

class ApsisTargeting {
public:
    ApsisTargeting(double target_radius, ApsisType apsis_type) 
        : target_radius_(target_radius),
          apsis_type_(apsis_type) {}

    // Add getter methods
    ApsisType getApsisType() const { return apsis_type_; }
    double getTargetRadius() const { return target_radius_; }

    // Get guidance commands at current time
    Eigen::Vector3d getDesiredForce(
        const Eigen::Vector3d& r_current,
        const Eigen::Vector3d& v_current,
        double time_since_start
    ) {
        if (time_since_start < 0) {
            return Eigen::Vector3d::Zero();
        }

        // Get current apsides using OrbitalMechanics class
        auto [ra, rp] = astro::OrbitalMechanics::computeApsides(r_current, v_current);

        // Determine if we need to burn
        bool should_burn = false;
        if (apsis_type_ == ApsisType::PERIGEE) {
            should_burn = (std::abs(rp - target_radius_) > 100.0);  // 100m tolerance
        } else {
            should_burn = (std::abs(ra - target_radius_) > 100.0);  // 100m tolerance
        }

        if (!should_burn) {
            return Eigen::Vector3d::Zero();
        }

        // Check if we're at the correct apsis for burning
        auto [at_apogee, at_perigee] = astro::OrbitalMechanics::isNearApsis(r_current, v_current);

        // At apogee, burn prograde to raise perigee
        // At perigee, burn prograde to raise apogee
        if ((apsis_type_ == ApsisType::PERIGEE && at_apogee) ||
            (apsis_type_ == ApsisType::APOGEE && at_perigee)) {
            
            Eigen::Vector3d burn_direction = v_current.normalized();
            
            // Calculate required delta-v based on current orbit
            double r = r_current.norm();
            double v = v_current.norm();
            double target_v;
            
            if (apsis_type_ == ApsisType::APOGEE) {
                // At perigee, calculate required velocity for new apogee
                target_v = std::sqrt(constants::earth::MU * (2/r - 2/(target_radius_ + r)));
            } else {
                // At apogee, calculate required velocity for new perigee
                target_v = std::sqrt(constants::earth::MU * (2/r - 2/(target_radius_ + r)));
            }
            
            double delta_v = target_v - v;
            double burn_magnitude = std::min(std::abs(delta_v), 100.0);  // Increased to 100 m/s per step
            if (delta_v < 0) burn_magnitude *= -1;
            
            return burn_direction * burn_magnitude * config::SimpleSat::mass;
        }

        return Eigen::Vector3d::Zero();
    }

private:
    double target_radius_;
    ApsisType apsis_type_;
};

} // namespace gnc::guidance

#endif // GNC_GUIDANCE_HOHMANN_HPP
