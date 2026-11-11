#ifndef ORTHOSTREAM_MAP_PRODUCTS_TEST_SUPPORT_HPP
#define ORTHOSTREAM_MAP_PRODUCTS_TEST_SUPPORT_HPP

#include <cmath>
#include <vector>

#include "orthostream/geo/camera.hpp"
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

// Depth (z) of the ellipsoidal-height surface h along a camera ray (z = 1).
inline double depthToHeight(const geo::GeodeticConverter &conv, const Eigen::Isometry3d &T_E_C,
                            const Eigen::Vector3d &ray_C, double h)
{
    const Eigen::Vector3d C   = T_E_C.translation();
    const Eigen::Vector3d ray = T_E_C.linear() * ray_C;
    double                t   = conv.ecefToGeodetic(C).height_m - h;
    for (int i = 0; i < 6; ++i)
    {
        const double f  = conv.ecefToGeodetic(C + t * ray).height_m - h;
        const double df = (conv.ecefToGeodetic(C + (t + 1.0) * ray).height_m - h) - f;
        t -= f / df;
    }
    return t;
}

struct Scene
{
    geo::CameraModel   camera;
    Eigen::Isometry3d  T_E_C;
    std::vector<float> depth, confidence;
};

// Nadir pinhole camera 250 m above a flat surface at 2000 m (ellipsoidal),
// optionally shifted east and north in metres.
inline Scene flatScene(const geo::GeodeticConverter &conv, int width = 80, int height = 60, double f = 60.0,
                       double east = 0.0, double north = 0.0)
{
    Scene s;
    s.camera.width  = width;
    s.camera.height = height;
    s.camera.fx = s.camera.fy = f;
    s.camera.cx               = (width - 1) / 2.0;
    s.camera.cy               = (height - 1) / 2.0;
    s.T_E_C                   = nadirCamera(conv, dorfberg(2250.0));
    const geo::Geodetic g     = dorfberg(2250.0);
    s.T_E_C.translation() += geo::rotationEcefFromEnu(g.latitude_rad, g.longitude_rad) * Eigen::Vector3d(east, north, 0.0);
    for (int v = 0; v < height; ++v)
    {
        for (int u = 0; u < width; ++u)
        {
            const Eigen::Vector3d ray((u - s.camera.cx) / f, (v - s.camera.cy) / f, 1.0);
            s.depth.push_back(static_cast<float>(depthToHeight(conv, s.T_E_C, ray, 2000.0)));
            s.confidence.push_back(0.75f);
        }
    }
    return s;
}

}  // namespace orthostream::map_products::test

#endif  // ORTHOSTREAM_MAP_PRODUCTS_TEST_SUPPORT_HPP
