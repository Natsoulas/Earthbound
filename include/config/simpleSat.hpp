/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */
#ifndef SIMPLE_SAT_HPP
#define SIMPLE_SAT_HPP

#include <Eigen/Dense>

namespace config {

struct SimpleSat {
    // Mass properties
    static constexpr double mass = 100.0;  // kg
    
    // Inertia tensor (kg*m^2)
    static const inline Eigen::Matrix3d inertia = (Eigen::Matrix3d() << 
        10.0, 0.0, 0.0,
        0.0, 10.0, 0.0,
        0.0, 0.0, 10.0
    ).finished();

    // Initial conditions
    static const inline Eigen::Vector3d initial_position = (Eigen::Vector3d() <<
        6878137.0,    // r_x (m) (500 km altitude)
        0.0,          // r_y (m)
        0.0           // r_z (m)
    ).finished();

    static const inline Eigen::Vector3d initial_velocity = (Eigen::Vector3d() <<
        0.0,          // v_x (m/s)
        5514.26,      // v_y (m/s) 
        5514.26       // v_z (m/s)
    ).finished();

    // Initial attitude quaternion (scalar first convention)
    // Represents rotation from GCRS (ECI) frame to body frame
    // Body frame definition:
    //   +X: Spacecraft forward direction
    //   +Y: Spacecraft right wing
    //   +Z: Spacecraft nadir (Earth-pointing)
    static const inline Eigen::Vector4d initial_quaternion = (Eigen::Vector4d() <<
        1.0,    // w (scalar)
        0.0,    // x
        0.0,    // y
        0.0     // z
    ).finished();

    // Initial angular velocity (rad/s)
    static const inline Eigen::Vector3d initial_omega = (Eigen::Vector3d() <<
        0.0,    // ω_x
        0.0,    // ω_y
        0.0     // ω_z
    ).finished();

    // Simulation parameters
    static constexpr double sim_duration = 3.5*3600.0;  // seconds (1 hour)
    static constexpr double time_step = 0.100;      // seconds
};

} // namespace config

#endif // SIMPLE_SAT_HPP
