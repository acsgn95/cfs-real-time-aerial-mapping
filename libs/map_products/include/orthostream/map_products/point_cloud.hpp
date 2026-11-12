// Dense point cloud (ADR-007 section 4, ADR-011 section 6): the points of a
// keyframe's depth map are written at once to one LAZ file per tile, since
// LAZ files cannot be appended to. Thinning and duplicate removal happen
// offline.

#ifndef ORTHOSTREAM_MAP_PRODUCTS_POINT_CLOUD_HPP
#define ORTHOSTREAM_MAP_PRODUCTS_POINT_CLOUD_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "orthostream/map_products/dsm.hpp"
#include "orthostream/map_products/ortho.hpp"

namespace orthostream::map_products
{

struct PointCloudWriterConfig
{
    std::string              directory;      ///< files go to <directory>/points/
    std::string              projected_crs;  ///< CRS of P, "EPSG:<code>"
    std::string              vertical_crs;   ///< "EPSG:<code>", or empty for ellipsoidal heights
    std::string              proj_data_dir;
    std::vector<std::string> grid_dirs;
    TileGridConfig           grid;                  ///< the tile grid of the DSM
    float                    min_confidence = 0.0f; ///< pixels below it are not written
};

struct PointCloudStats
{
    std::size_t              points = 0;
    std::vector<std::string> files;  ///< one per tile touched
};

/// LAS 1.2, point format 2, compressed (LAZ); coordinates in millimetres
/// relative to the tile corner. Per point: colour from the keyframe image,
/// intensity = depth confidence x 65535, point source ID = keyframe number
/// (modulo 65536). The CRS is stored as GeoTIFF keys. Not thread-safe.
class PointCloudWriter
{
public:
    PointCloudWriter(const PointCloudWriterConfig &config, const ProductFrame &frame);
    ~PointCloudWriter();
    PointCloudWriter(const PointCloudWriter &)            = delete;
    PointCloudWriter &operator=(const PointCloudWriter &) = delete;

    /// depth_camera: pinhole camera of the depth map; image and
    /// image_camera: the keyframe in colour at any scale, with its full
    /// camera model; points outside it are written black. T_E_C: camera to
    /// ECEF, shared by both cameras.
    PointCloudStats write(std::uint32_t keyframe, const DepthMapView &map, const geo::CameraModel &depth_camera,
                          const RgbImageView &image, const geo::CameraModel &image_camera,
                          const Eigen::Isometry3d &T_E_C);

    static std::string fileName(const TileId &id, std::uint32_t keyframe);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace orthostream::map_products

#endif  // ORTHOSTREAM_MAP_PRODUCTS_POINT_CLOUD_HPP
