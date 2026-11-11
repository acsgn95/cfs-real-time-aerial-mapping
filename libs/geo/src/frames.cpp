#include "orthostream/geo/frames.hpp"

#include <cmath>

namespace orthostream::geo
{

Eigen::Matrix3d rotationEcefFromEnu(double latitude_rad, double longitude_rad)
{
    const double sin_lat = std::sin(latitude_rad);
    const double cos_lat = std::cos(latitude_rad);
    const double sin_lon = std::sin(longitude_rad);
    const double cos_lon = std::cos(longitude_rad);

    Eigen::Matrix3d R_E_L;
    // Columns: east, north, up.
    R_E_L << -sin_lon, -sin_lat * cos_lon, cos_lat * cos_lon,  //
        cos_lon, -sin_lat * sin_lon, cos_lat * sin_lon,         //
        0.0, cos_lat, sin_lat;
    return R_E_L;
}

Eigen::Quaterniond canonicalQuaternion(const Eigen::Quaterniond &q)
{
    Eigen::Quaterniond n = q.normalized();
    if (n.w() < 0.0)
    {
        n.coeffs() = -n.coeffs();
    }
    return n;
}

LocalFrame::LocalFrame(const Eigen::Vector3d &origin_ecef, double latitude_rad, double longitude_rad)
{
    T_E_L_.setIdentity();
    T_E_L_.linear()      = rotationEcefFromEnu(latitude_rad, longitude_rad);
    T_E_L_.translation() = origin_ecef;
    T_L_E_               = T_E_L_.inverse(Eigen::Isometry);
}

Eigen::Vector3d LocalFrame::toLocal(const Eigen::Vector3d &p_E) const
{
    return T_L_E_ * p_E;
}

Eigen::Vector3d LocalFrame::toEcef(const Eigen::Vector3d &p_L) const
{
    return T_E_L_ * p_L;
}

Eigen::Isometry3d LocalFrame::poseInLocal(const Eigen::Isometry3d &T_E_C) const
{
    return T_L_E_ * T_E_C;
}

}  // namespace orthostream::geo
