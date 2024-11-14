/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#ifndef SPACECRAFT_SIMPLE_SAT_HPP
#define SPACECRAFT_SIMPLE_SAT_HPP

#include <Eigen/Dense>
#include <string>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <cmath>
#include <memory>
#include <functional>
#include <iostream>

#include "../config/simpleSat.hpp"
#include "../gnc/control/attitudeController.hpp"
#include "../dynamics/eom.hpp"
#include "../numerics/rk4.hpp"
#include "../astro/frames.hpp"
#include "../utilities/csvWriter.hpp"
#include "../gnc/guidance/hohmann.hpp"
#include "../gnc/control/translationalController.hpp"
#include "../astro/orbitalMechanics.hpp"

namespace spacecraft {

class SimpleSat {
public:
    // Constructor
    inline SimpleSat() 
        : attitude_controller_(0.005, 0.1, config::SimpleSat::inertia),
          translational_controller_(0.1, 0.01),
          currentTime_(0.0),
          startUnixTime_(std::time(nullptr))
    {
        initializeSimulation();
    }

    // Run the simulation and save data
    inline void runSimulation(
        const std::string& outputPath = "../data/satellite_state.csv",
        std::function<void(double)> callback = [](double){}) 
    {
        // Setup CSV writer with headers
        std::vector<std::string> headers = {
            "time", "x", "y", "z", "vx", "vy", "vz",
            "q1", "q2", "q3", "q4", "wx", "wy", "wz",
            "lat", "lon", "alt", "tx", "ty", "tz",
            "fx", "fy", "fz",
            "rsw_err_x", "rsw_err_y", "rsw_err_z",
            "apogee_alt", "perigee_alt"
        };
        utilities::CSVWriter writer(outputPath, headers);

        // Run simulation
        currentTime_ = 0.0;
        while (currentTime_ <= config::SimpleSat::sim_duration) {
            callback(currentTime_);  // Call the callback with current time
            saveStateToFile(writer);
            
            // Integrate one step forward
            state_ = numerics::RK4::step(
                [this](const Eigen::VectorXd& state) { return computeStateDerivative(state); },
                state_, 
                config::SimpleSat::time_step
            );
            currentTime_ += config::SimpleSat::time_step;
        }
    }

    // Getters for current state
    inline Eigen::Vector3d getPosition() const {
        return state_.segment<3>(dynamics::EOM::POS_IDX);
    }

    inline Eigen::Vector3d getVelocity() const {
        return state_.segment<3>(dynamics::EOM::VEL_IDX);
    }

    inline Eigen::Vector4d getQuaternion() const {
        return state_.segment<4>(dynamics::EOM::QUAT_IDX);
    }

    inline Eigen::Vector3d getAngularVelocity() const {
        return state_.segment<3>(dynamics::EOM::OMEGA_IDX);
    }

    inline double getCurrentTime() const { return currentTime_; }

    inline double getApogeeAltitude() const {
        auto [ra, rp] = astro::OrbitalMechanics::computeApsides(getPosition(), getVelocity());
        return (ra - constants::earth::RADIUS) / 1000.0;  // Convert to km
    }

    inline double getPerigeeAltitude() const {
        auto [ra, rp] = astro::OrbitalMechanics::computeApsides(getPosition(), getVelocity());
        return (rp - constants::earth::RADIUS) / 1000.0;  // Convert to km
    }

    inline void raiseApogee(double target_altitude_km) {
        startApsisTargeting(
            target_altitude_km * 1000.0 + constants::earth::RADIUS,
            gnc::guidance::ApsisType::APOGEE
        );
    }

    inline void raisePerigee(double target_altitude_km) {
        // Convert km to m
        double target_radius = (target_altitude_km * 1000.0) + constants::earth::RADIUS;
        
        // Create new guidance object
        apsis_guidance_ = std::make_unique<gnc::guidance::ApsisTargeting>(
            target_radius,
            gnc::guidance::ApsisType::PERIGEE  // This is correct
        );
        
        // Start the transfer
        transfer_active_ = true;
        transfer_start_time_ = currentTime_;
    }

private:
    // State vector and controller
    Eigen::VectorXd state_;
    gnc::control::GeometricAttitudeController attitude_controller_;
    gnc::control::TranslationalController translational_controller_;

    // Simulation parameters
    double currentTime_;
    double startUnixTime_;

    // Update member variable
    std::unique_ptr<gnc::guidance::ApsisTargeting> apsis_guidance_;
    bool transfer_active_ = false;
    double transfer_start_time_ = 0.0;

    // Update method
    inline void startApsisTargeting(double target_radius, gnc::guidance::ApsisType apsis_type) {
        apsis_guidance_ = std::make_unique<gnc::guidance::ApsisTargeting>(target_radius, apsis_type);
        transfer_active_ = true;
        transfer_start_time_ = currentTime_;
    }

    // Helper methods
    inline Eigen::VectorXd computeStateDerivative(const Eigen::VectorXd& state) {
        // Get guidance command if transfer is active
        Eigen::Vector3d desired_force = Eigen::Vector3d::Zero();
        if (transfer_active_ && apsis_guidance_) {
            auto [ra, rp] = astro::OrbitalMechanics::computeApsides(getPosition(), getVelocity());
            
            desired_force = apsis_guidance_->getDesiredForce(
                getPosition(),
                getVelocity(),
                currentTime_ - transfer_start_time_
            );
        }

        // Compute actual control force using controller
        Eigen::Vector3d control_force = translational_controller_.computeControlForce(
            getPosition(),
            getVelocity(),
            desired_force,
            config::SimpleSat::time_step
        );

        // Get attitude control torque
        Eigen::Vector3d control_torque = attitude_controller_.computeControlTorque(
            getPosition(),
            getVelocity(),
            getQuaternion(),
            getAngularVelocity(),
            config::SimpleSat::time_step
        );

        // Return combined state derivative
        return dynamics::EOM::computeStateDerivative(
            state,
            config::SimpleSat::inertia,
            control_torque,
            control_force
        );
    }

    inline void saveStateToFile(utilities::CSVWriter& writer) {
        // Get current control commands
        Eigen::Vector3d desired_force = Eigen::Vector3d::Zero();
        if (transfer_active_ && apsis_guidance_) {
            desired_force = apsis_guidance_->getDesiredForce(
                getPosition(),
                getVelocity(),
                currentTime_ - transfer_start_time_
            );
        }

        Eigen::Vector3d control_force = translational_controller_.computeControlForce(
            getPosition(),
            getVelocity(),
            desired_force,
            config::SimpleSat::time_step
        );

        // Get GCRS position and velocity
        Eigen::Vector3d r_gcrs = getPosition();
        Eigen::Vector3d v_gcrs = getVelocity();

        // Convert to ITRS (ECEF)
        double unix_time = startUnixTime_ + currentTime_;
        auto [r_itrs, v_itrs] = astro::Frames::gcrs2itrsState(r_gcrs, v_gcrs, unix_time);

        // Convert ITRS position to LLA
        Eigen::Vector3d lla = astro::Frames::ecef2lla(r_itrs);

        // Compute control torque and RSW error
        Eigen::Vector3d control_torque = attitude_controller_.computeControlTorque(
            r_gcrs, v_gcrs, getQuaternion(), getAngularVelocity(),
            config::SimpleSat::time_step);

        // Get RSW frame error
        Eigen::Matrix3d R_gcrs2rsw = astro::Frames::computeRSWFrame(r_gcrs, v_gcrs);
        Eigen::Matrix3d R_gcrs2body = numerics::quaternionToRotationMatrix(getQuaternion());
        Eigen::Matrix3d R_error = R_gcrs2rsw * R_gcrs2body.transpose();
        Eigen::Vector3d rsw_error = numerics::rotationMatrixToAxisAngle(R_error);

        const double RAD2DEG = 180.0/M_PI;
        auto [ra, rp] = astro::OrbitalMechanics::computeApsides(r_gcrs, v_gcrs);
        double apogee_alt = (ra - constants::earth::RADIUS) / 1000.0;   // Convert to km
        double perigee_alt = (rp - constants::earth::RADIUS) / 1000.0;  // Convert to km

        writer.writeRow(
            currentTime_,
            state_[0], state_[1], state_[2],
            state_[3], state_[4], state_[5],
            state_[6], state_[7], state_[8], state_[9],
            state_[10], state_[11], state_[12],
            lla[0] * RAD2DEG, lla[1] * RAD2DEG, lla[2],
            control_torque[0], control_torque[1], control_torque[2],
            control_force[0], control_force[1], control_force[2],
            rsw_error[0], rsw_error[1], rsw_error[2],
            apogee_alt, perigee_alt
        );
    }

    inline void initializeSimulation() {
        state_ = dynamics::EOM::createStateVector(
            config::SimpleSat::initial_position,
            config::SimpleSat::initial_velocity,
            config::SimpleSat::initial_quaternion,
            config::SimpleSat::initial_omega
        );
    }
};

} // namespace spacecraft

#endif // SIMPLE_SAT_HPP
