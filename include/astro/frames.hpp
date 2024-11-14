/*
 * Copyright (c) 2024 Niko Natsoulas
 * 
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#ifndef FRAMES_HPP
#define FRAMES_HPP

#include <Eigen/Dense>
#include <cmath>
#include "erfa/erfa.h"
#include "../constants/constants.hpp"

namespace astro {
namespace Frames {

// Constants for Earth rotation
const double OMEGA_EARTH = 7.2921150e-5;  // Earth's angular velocity [rad/s]
const Eigen::Matrix3d EARTH_ROTATION_MATRIX = (Eigen::Matrix3d() << 
    0.0,  OMEGA_EARTH, 0.0,
    -OMEGA_EARTH, 0.0, 0.0,
    0.0, 0.0, 0.0).finished();

// Helper function to convert Unix time to JD components for UT1 and TT
inline void unixTimeToJD(double unix_time, double dUT1,
                        double& jdut1, double& jdut2,
                        double& jdtt1, double& jdtt2) {
    // Convert Unix time to calendar date
    time_t unix_time_ = unix_time;
    struct tm time_;
#ifndef _WIN32
    gmtime_r(&unix_time_, &time_);
#else
    gmtime_s(&time_, &unix_time_);
#endif

    const int y = time_.tm_year + 1900;
    const int m = time_.tm_mon + 1;
    const int d = time_.tm_mday;

    const double unix_time_at_midnight = ((int)unix_time / 86400) * 86400;
    const double time = unix_time - unix_time_at_midnight;

    const int hh = (int)time / 3600;
    const double remainingSec = fmod(time, 3600.0);
    const int mm = (int)remainingSec / 60;
    const double sec = fmod(remainingSec, 60.);

    // Convert to JD using ERFA
    double jdutc1, jdutc2;
    eraDtf2d("UTC", y, m, d, hh, mm, sec, &jdutc1, &jdutc2);
    eraUtcut1(jdutc1, jdutc2, dUT1, &jdut1, &jdut2);

    // Get TT
    double tai1, tai2;
    eraUtctai(jdutc1, jdutc2, &tai1, &tai2);
    eraTaitt(tai1, tai2, &jdtt1, &jdtt2);
}

inline Eigen::Matrix3d computeGCRStoITRSMatrix(double jdut1, double jdut2,
                                              double jdtt1, double jdtt2,
                                              double xp = 0.0, double yp = 0.0) {
    // Get the NPB matrix (IAU 2006/2000A)
    double rnpb[3][3];
    eraPnm06a(jdut1, jdut2, rnpb);
    
    // Get Earth rotation angle
    double era = eraEra00(jdut1, jdut2);
    
    // Create rotation matrices
    double cos_era = std::cos(era);
    double sin_era = std::sin(era);
    
    // Match teme2itrs matrix construction
    const Eigen::Matrix3d ST_transp({
        { cos_era,  sin_era, 0.0},
        {-sin_era,  cos_era, 0.0},
        { 0.0,      0.0,     1.0}
    });
    
    // Convert NPB matrix to Eigen
    const Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> NPB(rnpb[0]);
    
    // Include polar motion if provided
    if (xp != 0.0 || yp != 0.0) {
        double sp = std::sin(xp);
        double cp = std::cos(xp);
        double se = std::sin(yp);
        double ce = std::cos(yp);
        
        Eigen::Matrix3d PM;
        PM << cp,    0,    -sp,
             -se*sp, ce, -se*cp,
              ce*sp, se,  ce*cp;
               
        return PM * ST_transp * NPB;
    }
    
    return ST_transp * NPB;
}

inline std::pair<Eigen::Vector3d, Eigen::Vector3d> gcrs2itrsState(
    const Eigen::Vector3d& r_gcrs,
    const Eigen::Vector3d& v_gcrs,
    double unix_time,
    double dUT1 = 0.0,
    double xp = 0.0,
    double yp = 0.0) {
    
    double jdut1, jdut2, jdtt1, jdtt2;
    unixTimeToJD(unix_time, dUT1, jdut1, jdut2, jdtt1, jdtt2);
    
    // Get transformation matrix
    Eigen::Matrix3d R_gcrs2itrs = computeGCRStoITRSMatrix(jdut1, jdut2, jdtt1, jdtt2, xp, yp);
    
    // Transform position and velocity to match teme2itrs approach
    Eigen::Vector3d r_itrs = R_gcrs2itrs * r_gcrs;
    Eigen::Vector3d v_itrs = R_gcrs2itrs * v_gcrs + EARTH_ROTATION_MATRIX * r_itrs;
    
    return {r_itrs, v_itrs};
}

inline std::pair<Eigen::Vector3d, Eigen::Vector3d> itrs2gcrsState(
    const Eigen::Vector3d& r_itrs,
    const Eigen::Vector3d& v_itrs,
    double unix_time,
    double dUT1 = 0.0,
    double xp = 0.0,
    double yp = 0.0) {
    
    double jdut1, jdut2, jdtt1, jdtt2;
    unixTimeToJD(unix_time, dUT1, jdut1, jdut2, jdtt1, jdtt2);
    
    // Get transformation matrix and its transpose
    Eigen::Matrix3d R_gcrs2itrs = computeGCRStoITRSMatrix(jdut1, jdut2, jdtt1, jdtt2, xp, yp);
    Eigen::Matrix3d R_itrs2gcrs = R_gcrs2itrs.transpose();
    
    // First remove Earth rotation effect from velocity
    Eigen::Vector3d v_norot = v_itrs - EARTH_ROTATION_MATRIX * r_itrs;
    
    // Transform position and velocity
    Eigen::Vector3d r_gcrs = R_itrs2gcrs * r_itrs;
    Eigen::Vector3d v_gcrs = R_itrs2gcrs * v_norot;
    
    return {r_gcrs, v_gcrs};
}

inline Eigen::Vector3d ecef2lla(const Eigen::Vector3d& r_ecef) {
    const double x = r_ecef.x();
    const double y = r_ecef.y();
    const double z = r_ecef.z();
    
    const double p = std::sqrt(x*x + y*y);
    
    // Special case for poles
    if (p < 1e-10) {
        const double lat = z >= 0 ? constants::math::HALF_PI : -constants::math::HALF_PI;
        return Eigen::Vector3d(lat, 0.0, std::abs(z) - constants::earth::RADIUS_POLAR);
    }
    
    // Initial guess using geometric height
    double lat = std::atan2(z, p);
    double N = constants::earth::RADIUS;
    double h = 0.0;
    
    // Iterative solution (usually converges in 2-3 iterations)
    for (int i = 0; i < 5; i++) {
        double sin_lat = std::sin(lat);
        N = constants::earth::RADIUS / 
            std::sqrt(1.0 - constants::earth::E2 * sin_lat * sin_lat);
        h = p / std::cos(lat) - N;
        
        // Update latitude
        lat = std::atan2(z, p * (1.0 - constants::earth::E2 * N/(N + h)));
    }
    
    const double lon = std::atan2(y, x);
    
    return Eigen::Vector3d(lat, lon, h);
}

inline Eigen::Vector3d lla2ecef(const Eigen::Vector3d& lla) {
    const double lat = lla[0];
    const double lon = lla[1];
    const double alt = lla[2];
    
    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double sin_lon = std::sin(lon);
    const double cos_lon = std::cos(lon);
    
    // Calculate radius of curvature in prime vertical
    const double N = constants::earth::RADIUS / 
                    std::sqrt(1.0 - constants::earth::E2 * sin_lat * sin_lat);
    
    // Calculate ECEF coordinates
    const double x = (N + alt) * cos_lat * cos_lon;
    const double y = (N + alt) * cos_lat * sin_lon;
    const double z = (N * (1.0 - constants::earth::E2) + alt) * sin_lat;
    
    return Eigen::Vector3d(x, y, z);
}

inline Eigen::Matrix3d computeRSWFrame(
    const Eigen::Vector3d& r_gcrs,
    const Eigen::Vector3d& v_gcrs
) {
    // Compute RSW frame vectors
    Eigen::Vector3d R = r_gcrs.normalized();  // Radial (outward)
    Eigen::Vector3d S = v_gcrs.normalized();  // First approximation of S
    S = (S - S.dot(R)*R).normalized();       // Make S perpendicular to R
    Eigen::Vector3d W = R.cross(S);          // Cross-track (orbit normal)

    // Create rotation matrix from GCRS to RSW
    Eigen::Matrix3d R_gcrs2rsw;
    R_gcrs2rsw.row(0) = R;
    R_gcrs2rsw.row(1) = S;
    R_gcrs2rsw.row(2) = W;

    return R_gcrs2rsw;
}

} // namespace Frames
} // namespace astro
#endif
