/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */
#ifndef ORBITAL_MECHANICS_HPP
#define ORBITAL_MECHANICS_HPP

#include <Eigen/Dense>
#include "../constants/constants.hpp"

namespace astro {

class OrbitalMechanics {
public:
    /**
     * Converts Cartesian state (position and velocity) to Keplerian orbital elements
     * @param r Position vector in ECI frame [m]
     * @param v Velocity vector in ECI frame [m/s]
     * @return Vector of Keplerian elements [a, e, i, Omega, omega, nu]
     *         a: semi-major axis [m]
     *         e: eccentricity [-]
     *         i: inclination [rad]
     *         Omega: right ascension of ascending node [rad]
     *         omega: argument of periapsis [rad]
     *         nu: true anomaly [rad]
     */
    static Eigen::Matrix<double, 6, 1> cartesianToKeplerian(const Eigen::Vector3d& r, const Eigen::Vector3d& v) {
        using namespace constants::earth;
        Eigen::Matrix<double, 6, 1> elements;
        
        // Calculate angular momentum vector
        Eigen::Vector3d h = r.cross(v);
        double h_mag = h.norm();
        
        // Calculate node vector (points toward ascending node)
        Eigen::Vector3d n = Eigen::Vector3d::UnitZ().cross(h);
        double n_mag = n.norm();
        
        // Calculate eccentricity vector
        double r_mag = r.norm();
        double v_mag = v.norm();
        Eigen::Vector3d e_vec = ((v_mag * v_mag - MU / r_mag) * r - r.dot(v) * v) / MU;
        double e = e_vec.norm();
        elements(1) = e;
        
        // Semi-major axis
        double specific_energy = v_mag * v_mag / 2.0 - MU / r_mag;
        elements(0) = -MU / (2.0 * specific_energy);
        
        // Inclination - already in correct quadrant from acos [0, π]
        elements(2) = std::acos(h.z() / h_mag);
        
        // Right ascension of ascending node
        // For zero inclination orbits, RAAN is undefined (set to 0)
        if (n_mag < 1e-11) {
            elements(3) = 0.0;
        } else {
            // atan2 returns [-π, π], we want [0, 2π]
            elements(3) = std::atan2(n.y(), n.x());
            if (elements(3) < 0.0) {
                elements(3) += 2.0 * constants::math::PI;
            }
        }
        
        // Argument of periapsis
        // For circular orbits, argument of periapsis is undefined (set to 0)
        if (e < 1e-11) {
            elements(4) = 0.0;
        } else if (n_mag < 1e-11) {
            // For equatorial orbits, use the x-axis as reference for ω
            elements(4) = std::atan2(e_vec.y(), e_vec.x());
            if (elements(4) < 0.0) {
                elements(4) += 2.0 * constants::math::PI;
            }
        } else {
            // Use atan2 instead of acos for proper quadrant
            elements(4) = std::atan2(
                h.dot(e_vec.cross(n)), 
                n.dot(e_vec)
            );
            if (elements(4) < 0.0) {
                elements(4) += 2.0 * constants::math::PI;
            }
        }
        
        // True anomaly
        // For circular orbits, use the node vector as reference
        if (e < 1e-11) {
            if (n_mag < 1e-11) {
                // For circular equatorial orbits, use the x-axis as reference
                elements(5) = std::atan2(r.y(), r.x());
            } else {
                elements(5) = std::atan2(
                    n.dot(r.cross(n)),
                    n.dot(r)
                );
            }
        } else {
            // Use atan2 instead of acos for proper quadrant
            elements(5) = std::atan2(
                h.dot(e_vec.cross(r)),
                e_vec.dot(r)
            );
        }
        // Ensure true anomaly is in [0, 2π]
        if (elements(5) < 0.0) {
            elements(5) += 2.0 * constants::math::PI;
        }
        
        return elements;
    }

    /**
     * Computes orbital period given semi-major axis
     * @param a Semi-major axis [m]
     * @return Orbital period [s]
     */
    static double computeOrbitalPeriod(double a) {
        return 2.0 * constants::math::PI * std::sqrt(std::pow(a, 3) / constants::earth::MU);
    }

    /**
     * Computes circular orbit velocity at given radius
     * @param r Orbital radius [m]
     * @return Circular orbit velocity [m/s]
     */
    static double computeCircularVelocity(double r) {
        return std::sqrt(constants::earth::MU / r);
    }

    /**
     * Computes apogee and perigee radii from current state
     * @param r Position vector [m]
     * @param v Velocity vector [m/s]
     * @return std::pair<double, double> {apogee_radius, perigee_radius} [m]
     */
    static std::pair<double, double> computeApsides(
        const Eigen::Vector3d& r,
        const Eigen::Vector3d& v
    ) {
        double r_mag = r.norm();
        double v_mag = v.norm();
        double specific_energy = (v_mag*v_mag/2.0) - constants::earth::MU/r_mag;
        Eigen::Vector3d h = r.cross(v);
        double h_mag2 = h.squaredNorm();
        
        // Semi-major axis
        double a = -constants::earth::MU/(2*specific_energy);
        
        // Eccentricity
        double e = std::sqrt(1.0 + (2.0*specific_energy*h_mag2)/(constants::earth::MU*constants::earth::MU));
        
        // Apogee and perigee radii
        double ra = a*(1.0 + e);
        double rp = a*(1.0 - e);
        
        return {ra, rp};
    }

    /**
     * Determines if current position is near an apsis
     * @param r Position vector [m]
     * @param v Velocity vector [m/s]
     * @param tolerance Distance tolerance [m]
     * @return std::pair<bool, bool> {at_apogee, at_perigee}
     */
    static std::pair<bool, bool> isNearApsis(
        const Eigen::Vector3d& r,
        const Eigen::Vector3d& v,
        double tolerance = 1000.0
    ) {
        auto [ra, rp] = computeApsides(r, v);
        double r_mag = r.norm();
        
        bool at_apogee = std::abs(r_mag - ra) < tolerance;
        bool at_perigee = std::abs(r_mag - rp) < tolerance;
        
        return {at_apogee, at_perigee};
    }

    /**
     * Converts true anomaly to eccentric anomaly
     * @param nu True anomaly [rad]
     * @param e Eccentricity [-]
     * @return Eccentric anomaly [rad]
     */
    static double trueToEccentricAnomaly(double nu, double e) {
        // Handle circular orbit as special case
        if (e < 1e-11) return nu;
        
        double cos_nu = std::cos(nu);
        double E = std::atan2(
            std::sqrt(1.0 - e * e) * std::sin(nu),
            e + cos_nu
        );
        
        // Ensure E is in [0, 2π]
        if (E < 0.0) {
            E += 2.0 * constants::math::PI;
        }
        return E;
    }

    /**
     * Converts eccentric anomaly to mean anomaly
     * @param E Eccentric anomaly [rad]
     * @param e Eccentricity [-]
     * @return Mean anomaly [rad]
     */
    static double eccentricToMeanAnomaly(double E, double e) {
        double M = E - e * std::sin(E);
        
        // Ensure M is in [0, 2π]
        if (M < 0.0) {
            M += 2.0 * constants::math::PI;
        }
        return M;
    }

    /**
     * Converts mean anomaly to eccentric anomaly using Newton-Raphson iteration
     * @param M Mean anomaly [rad]
     * @param e Eccentricity [-]
     * @param tolerance Convergence tolerance [rad]
     * @param max_iterations Maximum number of iterations
     * @return Eccentric anomaly [rad]
     */
    static double meanToEccentricAnomaly(
        double M, 
        double e, 
        double tolerance = 1e-8,
        int max_iterations = 10
    ) {
        // Handle circular orbit as special case
        if (e < 1e-11) return M;
        
        // Initial guess (from "Fundamentals of Astrodynamics" by Bate, Mueller, and White)
        double E;
        if (M < constants::math::PI) {
            E = M + e/2.0;
        } else {
            E = M - e/2.0;
        }
        
        // Newton-Raphson iteration
        int iter = 0;
        double delta;
        do {
            delta = (E - e * std::sin(E) - M) / (1.0 - e * std::cos(E));
            E -= delta;
            iter++;
        } while (std::abs(delta) > tolerance && iter < max_iterations);
        
        // Ensure E is in [0, 2π]
        if (E < 0.0) {
            E += 2.0 * constants::math::PI;
        }
        return E;
    }

    /**
     * Converts eccentric anomaly to true anomaly
     * @param E Eccentric anomaly [rad]
     * @param e Eccentricity [-]
     * @return True anomaly [rad]
     */
    static double eccentricToTrueAnomaly(double E, double e) {
        // Handle circular orbit as special case
        if (e < 1e-11) return E;
        
        double cos_E = std::cos(E);
        double nu = std::atan2(
            std::sqrt(1.0 - e * e) * std::sin(E),
            cos_E - e
        );
        
        // Ensure nu is in [0, 2π]
        if (nu < 0.0) {
            nu += 2.0 * constants::math::PI;
        }
        return nu;
    }

    /**
     * Computes time since periapsis passage given mean anomaly
     * @param M Mean anomaly [rad]
     * @param a Semi-major axis [m]
     * @return Time since periapsis passage [s]
     */
    static double timeSincePeriapsis(double M, double a) {
        double n = std::sqrt(constants::earth::MU / (a * a * a)); // mean motion
        return M / n;
    }

    /**
     * Computes mean anomaly given time since periapsis passage
     * @param t Time since periapsis passage [s]
     * @param a Semi-major axis [m]
     * @return Mean anomaly [rad]
     */
    static double meanAnomalyFromTime(double t, double a) {
        double n = std::sqrt(constants::earth::MU / (a * a * a)); // mean motion
        double M = n * t;
        
        // Ensure M is in [0, 2π]
        M = std::fmod(M, 2.0 * constants::math::PI);
        if (M < 0.0) {
            M += 2.0 * constants::math::PI;
        }
        return M;
    }

    /**
     * Computes true anomaly given time since periapsis passage
     * @param t Time since periapsis passage [s]
     * @param a Semi-major axis [m]
     * @param e Eccentricity [-]
     * @return True anomaly [rad]
     */
    static double trueAnomalyFromTime(double t, double a, double e) {
        double M = meanAnomalyFromTime(t, a);
        double E = meanToEccentricAnomaly(M, e);
        return eccentricToTrueAnomaly(E, e);
    }
};

}

#endif
