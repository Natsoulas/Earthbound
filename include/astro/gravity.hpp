/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#ifndef GRAVITY_HPP
#define GRAVITY_HPP

#include <Eigen/Dense>
#include "../constants/constants.hpp"

namespace astro {

class Gravity {
public:
    /**
     * Computes the gravitational acceleration including J2 perturbation
     * @param position Position vector in Earth-Centered Inertial (ECI) frame [m]
     * @return Acceleration vector in ECI frame [m/s^2]
     */
    static Eigen::Vector3d computeAcceleration(const Eigen::Vector3d& position) {
        // Magnitude of position vector
        double r = position.norm();
        
        // Basic two-body acceleration
        Eigen::Vector3d acc = -constants::earth::MU * position / (r * r * r);
        
        // J2 perturbation
        double r2 = r * r;
        double r4 = r2 * r2;
        double z2 = position.z() * position.z();
        double coefficient = 1.5 * constants::earth::J2 * 
                           constants::earth::MU * 
                           constants::earth::RADIUS * 
                           constants::earth::RADIUS / r4;
        
        Eigen::Vector3d j2_acc;
        j2_acc.x() = coefficient * position.x() / r2 * (5 * z2 / r2 - 1);
        j2_acc.y() = coefficient * position.y() / r2 * (5 * z2 / r2 - 1);
        j2_acc.z() = coefficient * position.z() / r2 * (5 * z2 / r2 - 3);
        
        return acc + j2_acc;
    }
    
    /**
     * Computes only the two-body gravitational acceleration without J2
     * @param position Position vector in ECI frame [m]
     * @return Acceleration vector in ECI frame [m/s^2]
     */
    static Eigen::Vector3d computeTwoBodyAcceleration(const Eigen::Vector3d& position) {
        double r = position.norm();
        return -constants::earth::MU * position / (r * r * r);
    }
};

} // namespace astro

#endif // GRAVITY_HPP
