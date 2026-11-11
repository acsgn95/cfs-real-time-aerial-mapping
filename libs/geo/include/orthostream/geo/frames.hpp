// Frames and rotations (ADR-004).
//
// Conventions:
// * T_A_B transforms points from frame B into frame A: p_A = T_A_B * p_B.
// * Quaternions are Hamilton unit quaternions, normalised to w >= 0.
// * Lengths in metres, angles in radians. Geodetic latitude and longitude
//   refer to the WGS 84 ellipsoid.

#ifndef ORTHOSTREAM_GEO_FRAMES_HPP
#define ORTHOSTREAM_GEO_FRAMES_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace orthostream::geo
{

/// Rotation R_E_L from a local ENU frame at the given geodetic latitude and
/// longitude to ECEF. Its columns are the east, north and up axes in ECEF.
Eigen::Matrix3d rotationEcefFromEnu(double latitude_rad, double longitude_rad);

/// Returns the quaternion normalised to unit length with w >= 0, the unique
/// representation used on OrthoStream interfaces.
Eigen::Quaterniond canonicalQuaternion(const Eigen::Quaterniond &q);

/// Local ENU working frame L (ADR-004 section 4): origin at a point given in
/// ECEF, axes east, north and up at that point.
class LocalFrame
{
public:
    /// The geodetic latitude and longitude must be those of the origin; use
    /// CrsTransformer::ecefToGeodetic to obtain them.
    LocalFrame(const Eigen::Vector3d &origin_ecef, double latitude_rad, double longitude_rad);

    /// Transform T_E_L from the local frame to ECEF.
    const Eigen::Isometry3d &ecefFromLocal() const { return T_E_L_; }

    Eigen::Vector3d toLocal(const Eigen::Vector3d &p_E) const;
    Eigen::Vector3d toEcef(const Eigen::Vector3d &p_L) const;

    /// Pose of a camera (T_E_C) expressed in the local frame (T_L_C).
    Eigen::Isometry3d poseInLocal(const Eigen::Isometry3d &T_E_C) const;

private:
    Eigen::Isometry3d T_E_L_;
    Eigen::Isometry3d T_L_E_;
};

}  // namespace orthostream::geo

#endif  // ORTHOSTREAM_GEO_FRAMES_HPP
