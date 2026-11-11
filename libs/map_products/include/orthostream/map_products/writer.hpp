// Writing of map product tiles and their mosaic index (ADR-007 section 7).

#ifndef ORTHOSTREAM_MAP_PRODUCTS_WRITER_HPP
#define ORTHOSTREAM_MAP_PRODUCTS_WRITER_HPP

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "orthostream/map_products/dsm.hpp"
#include "orthostream/map_products/product_frame.hpp"

namespace orthostream::map_products
{

struct DsmWriterConfig
{
    std::string              directory;      ///< tiles go to <directory>/dsm/
    std::string              projected_crs;  ///< CRS of P, for example "EPSG:2056"
    /// Vertical CRS of the written heights, for example "EPSG:5729" (LHN95)
    /// or "EPSG:3855" (EGM2008). Empty: ellipsoidal heights (WGS 84).
    std::string              vertical_crs;
    std::string              proj_data_dir;
    std::vector<std::string> grid_dirs;
};

/// Writes DSM tiles as GeoTIFF: band 1 height, band 2 weighted variance,
/// float32, no-data -32767, deflate-compressed. Files are written under a
/// temporary name and renamed, so readers never see a partial tile. Not
/// thread-safe.
class DsmWriter
{
public:
    static constexpr float kNoData = -32767.0f;

    DsmWriter(const DsmWriterConfig &config, const ProductFrame &frame);
    ~DsmWriter();
    DsmWriter(const DsmWriter &)            = delete;
    DsmWriter &operator=(const DsmWriter &) = delete;

    /// Writes or overwrites the tile; returns the file path.
    std::string write(const DsmTile &tile);

    /// Writes <directory>/dsm.vrt over all tiles written so far.
    std::string writeIndex();

    /// File name of a tile, relative to <directory>/dsm/.
    static std::string tileFileName(const TileId &id);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace orthostream::map_products

#endif  // ORTHOSTREAM_MAP_PRODUCTS_WRITER_HPP
