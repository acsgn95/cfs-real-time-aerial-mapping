// Internal: conversion of ellipsoidal heights to the vertical datum of the
// written products (ADR-004 section 5), shared by the DSM and point cloud
// writers.

#ifndef ORTHOSTREAM_MAP_PRODUCTS_VERTICAL_DATUM_HPP
#define ORTHOSTREAM_MAP_PRODUCTS_VERTICAL_DATUM_HPP

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "orthostream/map_products/product_frame.hpp"

namespace orthostream::map_products::detail
{

/// Offset (output height minus ellipsoidal height) over a square area,
/// sampled on a 9 x 9 grid and interpolated bilinearly. Geoid models vary
/// over kilometres, so a tile-sized grid is far below a millimetre off.
class OffsetGrid
{
public:
    static constexpr int kNodes = 9;

    /// west, north: corner of the area; size: side length in metres.
    OffsetGrid(double west, double north, double size);

    double &node(int i, int j) { return values_[static_cast<std::size_t>(j * kNodes + i)]; }
    double  at(double easting, double northing) const;
    double  west() const { return west_; }
    double  north() const { return north_; }
    double  step() const { return step_; }

private:
    double                                west_, north_, step_;
    std::array<double, kNodes * kNodes>   values_ {};
};

class VerticalDatum
{
public:
    /// vertical_crs: "EPSG:<code>", or empty for ellipsoidal heights.
    VerticalDatum(const ProductFrame &frame, const std::string &projected_crs, const std::string &vertical_crs,
                  const std::string &proj_data_dir, const std::vector<std::string> &grid_dirs);

    bool               ellipsoidal() const { return !to_output_; }
    /// The CRS of the written products: projected, or compound.
    const std::string &crs() const { return crs_; }
    int                projectedEpsg() const { return projected_epsg_; }
    int                verticalEpsg() const { return vertical_epsg_; }  ///< 0 if ellipsoidal

    /// Offsets over an area, evaluated at the given ellipsoidal height.
    OffsetGrid offsets(double west, double north, double size, double height) const;

private:
    const ProductFrame                  &frame_;
    std::string                          crs_;
    int                                  projected_epsg_ = 0;
    int                                  vertical_epsg_  = 0;
    std::unique_ptr<geo::CrsTransformer> to_output_;
};

}  // namespace orthostream::map_products::detail

#endif  // ORTHOSTREAM_MAP_PRODUCTS_VERTICAL_DATUM_HPP
