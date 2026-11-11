// Product frame P (ADR-004 sections 5 and 6): easting and northing of the
// mission's projected CRS, with ellipsoidal heights. Products are converted
// to their vertical datum only when written.

#ifndef ORTHOSTREAM_MAP_PRODUCTS_PRODUCT_FRAME_HPP
#define ORTHOSTREAM_MAP_PRODUCTS_PRODUCT_FRAME_HPP

#include <array>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "orthostream/geo/crs.hpp"

namespace orthostream::map_products
{

class MapProductsError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

/// Exact conversion between ECEF and P through PROJ. Not thread-safe.
class ProductFrame
{
public:
    /// projected_crs: for example "EPSG:32632" (UTM 32N) or "EPSG:2056" (LV95).
    ProductFrame(const std::string              &projected_crs,
                 const std::string              &proj_data_dir,
                 const std::vector<std::string> &grid_dirs = {});

    /// Easting, northing, ellipsoidal height (WGS 84).
    Eigen::Vector3d fromEcef(const Eigen::Vector3d &p_E) const;
    Eigen::Vector3d toEcef(const Eigen::Vector3d &p_P) const;

private:
    geo::CrsTransformer    to_projected_;
    geo::GeodeticConverter geodetic_;
};

/// Fast approximation of a smooth transformation over a bounded region:
/// the exact transformation is evaluated at the nodes of a regular grid over
/// an axis-aligned box and interpolated trilinearly in between. Over a region
/// of a few hundred metres with nodes about 50 m apart, the error of ECEF to
/// P is far below a millimetre (map projections and ellipsoidal heights curve
/// on the scale of the Earth radius). Points outside the box are
/// extrapolated from the nearest grid cell.
class InterpolatedTransform
{
public:
    using Function = std::function<Eigen::Vector3d(const Eigen::Vector3d &)>;

    /// nodes: grid nodes per axis, at least 2. A box axis shorter than 1 m
    /// is widened to 1 m.
    InterpolatedTransform(const Function &exact, const Eigen::AlignedBox3d &box, int nodes = 9);

    Eigen::Vector3d operator()(const Eigen::Vector3d &p) const;

private:
    Eigen::Vector3d              origin_;
    Eigen::Vector3d              step_;
    int                          nodes_;
    std::vector<Eigen::Vector3d> values_;  // x fastest, then y, then z
};

}  // namespace orthostream::map_products

#endif  // ORTHOSTREAM_MAP_PRODUCTS_PRODUCT_FRAME_HPP
