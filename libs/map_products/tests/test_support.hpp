#ifndef ORTHOSTREAM_MAP_PRODUCTS_TEST_SUPPORT_HPP
#define ORTHOSTREAM_MAP_PRODUCTS_TEST_SUPPORT_HPP

#include <cmath>

#include "orthostream/geo/crs.hpp"
#include "orthostream/geo/frames.hpp"

namespace orthostream::map_products::test
{

constexpr double kPi = 3.14159265358979323846;

// Near the Dorfberg survey (Davos, Switzerland).
constexpr double kLatDeg = 46.825;
constexpr double kLonDeg = 9.835;

inline geo::Geodetic dorfberg(double height)
{
    return { kLatDeg * kPi / 180.0, kLonDeg * kPi / 180.0, height };
}

// Nadir camera at a geodetic position: x east, y south, z down.
inline Eigen::Isometry3d nadirCamera(const geo::GeodeticConverter &conv, const geo::Geodetic &g)
{
    Eigen::Matrix3d R_L_C;
    R_L_C << 1, 0, 0, 0, -1, 0, 0, 0, -1;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear()          = geo::rotationEcefFromEnu(g.latitude_rad, g.longitude_rad) * R_L_C;
    T.translation()     = conv.geodeticToEcef(g);
    return T;
}

}  // namespace orthostream::map_products::test

#endif  // ORTHOSTREAM_MAP_PRODUCTS_TEST_SUPPORT_HPP
