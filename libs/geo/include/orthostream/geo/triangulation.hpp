// Triangulation of a point from several viewing rays.

#ifndef ORTHOSTREAM_GEO_TRIANGULATION_HPP
#define ORTHOSTREAM_GEO_TRIANGULATION_HPP

#include <optional>
#include <vector>

#include <Eigen/Core>

namespace orthostream::geo
{

struct Ray
{
    Eigen::Vector3d origin;
    Eigen::Vector3d direction;  ///< need not be unit length
};

/// Point with the least sum of squared distances to all rays. Returns
/// nothing for fewer than two rays or (nearly) parallel rays.
std::optional<Eigen::Vector3d> triangulateRays(const std::vector<Ray> &rays);

}  // namespace orthostream::geo

#endif  // ORTHOSTREAM_GEO_TRIANGULATION_HPP
