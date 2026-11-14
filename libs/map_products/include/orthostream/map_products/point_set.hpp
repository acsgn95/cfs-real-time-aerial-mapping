// Point sets (ADR-007 section 2, ADR-012 section 1): the points of one
// keyframe's depth map in the product frame P, sorted by tile. GEOREF_APP
// builds them, writes the point cloud from them and hands them to
// MAPPROD_APP for DSM fusion through the data store (ADR-013).

#ifndef ORTHOSTREAM_MAP_PRODUCTS_POINT_SET_HPP
#define ORTHOSTREAM_MAP_PRODUCTS_POINT_SET_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "orthostream/geo/camera.hpp"
#include "orthostream/map_products/dsm.hpp"
#include "orthostream/map_products/product_frame.hpp"

namespace orthostream::map_products
{

/// One point, 20 bytes. Coordinates are relative to the origin of the set,
/// so that single precision holds them to well below a millimetre over a
/// keyframe's footprint (float spacing at 1 km: 0.06 mm).
struct PointRecord
{
    float         east   = 0.0f;  ///< metres from the origin
    float         north  = 0.0f;
    float         height = 0.0f;  ///< ellipsoidal, metres from the origin height
    float         weight = 0.0f;  ///< depth confidence (ADR-007 section 2)
    std::uint8_t  rgb[3] = { 0, 0, 0 };
    std::uint8_t  spare  = 0;
};
static_assert(sizeof(PointRecord) == 20, "PointRecord is a data store layout");

/// The points of one tile: records [first, first + count) of the set.
struct TileSpan
{
    TileId        id;
    std::uint32_t first = 0;
    std::uint32_t count = 0;
};

/// A point set in caller-owned memory.
struct PointSetView
{
    const PointRecord *points = nullptr;
    std::size_t        count  = 0;
    Eigen::Vector3d    origin = Eigen::Vector3d::Zero();  ///< easting, northing, ellipsoidal height
    const TileSpan    *tiles  = nullptr;
    std::size_t        tile_count = 0;
};

/// Keyframe image for the point colours: the working image of the depth
/// map, so pixel i of the depth map is pixel i of the image. 1 (grey) or 3
/// (RGB) bytes per pixel.
struct WorkImageView
{
    const std::uint8_t *data     = nullptr;
    std::size_t         stride   = 0;  ///< bytes per row
    int                 channels = 0;
};

struct PointSetStats
{
    std::size_t     points = 0;
    std::size_t     tiles  = 0;
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    double          height_min = 0.0;  ///< ellipsoidal, absolute
    double          height_max = 0.0;
};

/// Converts depth maps to point sets: pixel to the camera frame, to ECEF
/// with the keyframe pose, to P (interpolated over the extent of the map,
/// as in DSM fusion), then sorted by tile. Not thread-safe; holds its
/// working buffers between calls.
class PointSetBuilder
{
public:
    /// grid: the tile grid of the DSM. min_confidence: pixels below it are
    /// left out (and pixels without confidence always).
    PointSetBuilder(const ProductFrame &frame, const TileGridConfig &grid, double min_confidence);

    /// Number of points a depth map gives.
    std::size_t count(const DepthMapView &map, const geo::CameraModel &camera) const;

    /// Writes the points into points[0, capacity) and the tile spans into
    /// tiles[0, max_tiles), ordered by tile (column, then row) and within a
    /// tile by pixel. The origin is the camera centre in P, rounded to whole
    /// metres. image may have data == nullptr: the points are black then.
    /// Throws MapProductsError if a buffer is too small.
    PointSetStats build(const DepthMapView &map, const geo::CameraModel &camera, const WorkImageView &image,
                        const Eigen::Isometry3d &T_E_C, PointRecord *points, std::size_t capacity, TileSpan *tiles,
                        std::size_t max_tiles);

    const TileGridConfig &grid() const { return grid_; }

private:
    const ProductFrame          &frame_;
    TileGridConfig               grid_;
    double                       min_confidence_;
    std::vector<PointRecord>     unsorted_;
    std::vector<std::uint32_t>   tile_of_;
};

}  // namespace orthostream::map_products

#endif  // ORTHOSTREAM_MAP_PRODUCTS_POINT_SET_HPP
