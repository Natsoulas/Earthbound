#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

namespace constants {

// Earth Physical Constants
namespace earth {
    constexpr double MU = 3.986004418e14;     // Gravitational parameter [m^3/s^2]
    constexpr double RADIUS = 6378137.0;       // Equatorial radius [m]
    constexpr double J2 = 1.08262668e-3;      // J2 coefficient
    constexpr double ROTATION_RATE = 7.2921150e-5; // Earth rotation rate [rad/s]
    constexpr double FLATTENING = 1.0/298.257223563; // WGS-84 flattening
    constexpr double ECCENTRICITY = 0.0818191908426215; // WGS-84 eccentricity
    constexpr double RADIUS_POLAR = RADIUS * (1.0 - FLATTENING);  // Polar radius
    constexpr double E2 = FLATTENING * (2.0 - FLATTENING);  // First eccentricity squared
    constexpr double E2_PRIME = E2 / (1.0 - E2);  // Second eccentricity squared
}

// Mathematical Constants
namespace math {
    constexpr double PI = 3.14159265358979323846;
    constexpr double TWO_PI = 2.0 * PI;
    constexpr double HALF_PI = PI / 2.0;
    constexpr double DEG_TO_RAD = PI / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / PI;
    constexpr double ARCSEC2RAD = PI / (180.0 * 3600.0);  // Convert arcseconds to radians
}

// Physical Constants
namespace physics {
    constexpr double G = 6.67430e-11;         // Gravitational constant [m^3/kg/s^2]
    constexpr double C = 299792458.0;         // Speed of light [m/s]
    constexpr double SOLAR_FLUX = 1361.0;     // Solar flux at 1 AU [W/m^2]
}

// Time Constants
namespace time {
    constexpr double SECONDS_PER_DAY = 86400.0;
    constexpr double J2000_UNIX = 946728000.0;  // Unix timestamp for J2000
}

} // namespace constants

#endif // CONSTANTS_HPP 