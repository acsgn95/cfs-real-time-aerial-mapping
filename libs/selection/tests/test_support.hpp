#ifndef ORTHOSTREAM_SELECTION_TEST_SUPPORT_HPP
#define ORTHOSTREAM_SELECTION_TEST_SUPPORT_HPP

#include <cmath>

#include "orthostream/selection/footprint.hpp"

namespace orthostream::selection::test
{

// 200 x 100 px, f = 100 px: a nadir footprint is 2 H wide and 1 H high.
inline geo::CameraModel camera()
{
    geo::CameraModel c;
    c.width  = 200;
    c.height = 100;
    c.fx     = 100.0;
    c.fy     = 100.0;
    c.cx     = 99.5;
    c.cy     = 49.5;
    return c;
}

// Nadir camera: x = east, y = south, z = down; optionally rotated about the
// vertical (yaw) and tilted about the camera x axis.
inline Eigen::Isometry3d nadir(const Eigen::Vector3d &centre, double yaw_rad = 0.0, double tilt_rad = 0.0)
{
    Eigen::Matrix3d R;
    R << 1, 0, 0, 0, -1, 0, 0, 0, -1;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear()          = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix() * R
                 * Eigen::AngleAxisd(tilt_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();
    T.translation() = centre;
    return T;
}

inline TerrainHeight flat(double h)
{
    return [h](double, double) { return std::optional<double>(h); };
}

}  // namespace orthostream::selection::test

#endif  // ORTHOSTREAM_SELECTION_TEST_SUPPORT_HPP
