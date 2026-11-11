// Ground footprints of images and their overlap (ADR-006 sections 1 and 3).
//
// All geometry is in a local Cartesian frame with z pointing up (for example
// the ENU working frame of ADR-004). The terrain is given as a function of
// the horizontal position, so the caller decides where heights come from
// (SRTM in the mission, a reference DSM in tools) and converts them to the
// frame.

#ifndef ORTHOSTREAM_SELECTION_FOOTPRINT_HPP
#define ORTHOSTREAM_SELECTION_FOOTPRINT_HPP

#include <array>
#include <functional>
#include <optional>
#include <stdexcept>

#include <Eigen/Geometry>

#include "orthostream/geo/camera.hpp"

namespace orthostream::selection
{

class SelectionError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

/// Terrain height (z in the local frame, metres) at a horizontal position,
/// or nothing where the elevation model has no value.
using TerrainHeight = std::function<std::optional<double>(double x, double y)>;

struct FootprintConfig
{
    double nominal_height_agl = 250.0;  ///< starting plane below the camera, metres
    int    iterations         = 3;      ///< fixed-point iterations of the plane height
};

struct Footprint
{
    /// Ground points of the image corners (outer pixel edges), starting at
    /// the top-left corner and ordered counter-clockwise seen from above;
    /// z is the terrain height there.
    std::array<Eigen::Vector3d, 4> corners;
    Eigen::Vector3d                centre;  ///< ground point of the principal point
    double plane_height  = 0.0;             ///< height of the intersection plane
    double height_spread = 0.0;             ///< standard deviation of the five terrain heights
    double width_m       = 0.0;             ///< mean length of the sides along the image x axis
    double height_m      = 0.0;             ///< mean length of the sides along the image y axis
};

/// Footprint of a camera with pose T_L_C (camera to local frame). Returns
/// nothing if a ray does not point downwards, if a corner cannot be
/// undistorted, or if the terrain has no height at one of the five points.
std::optional<Footprint> computeFootprint(const geo::CameraModel  &camera,
                                          const Eigen::Isometry3d &T_L_C,
                                          const TerrainHeight     &terrain,
                                          const FootprintConfig   &config = {});

/// Horizontal area of a footprint in square metres.
double footprintArea(const Footprint &footprint);

/// Horizontal area of the intersection of two footprints divided by the area
/// of the reference footprint, in [0, 1].
double overlapFraction(const Footprint &reference, const Footprint &other);

/// Angle between the optical axis and the downward vertical, in radians.
double tiltFromNadir(const Eigen::Isometry3d &T_L_C);

}  // namespace orthostream::selection

#endif  // ORTHOSTREAM_SELECTION_FOOTPRINT_HPP
