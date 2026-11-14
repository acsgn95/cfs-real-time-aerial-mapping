// Terrain prior for image footprints (ADR-006 section 2).
//
// A digital elevation model, by default the Copernicus DEM GLO-30 mosaic of
// tools/dataset/fetch_dem.py, read through GDAL. Its heights are converted
// to WGS 84 ellipsoidal heights through PROJ (EPSG:4326+3855 to ECEF needs
// the EGM2008 grid us_nga_egm08_25.tif; without it the transformation fails
// instead of silently skipping the geoid, ADR-004).
//
// Only a window around the flight is held in memory: load() reads the part
// of the model within a radius of a point. All positions are ECEF.

#ifndef ORTHOSTREAM_TERRAIN_TERRAIN_HPP
#define ORTHOSTREAM_TERRAIN_TERRAIN_HPP

#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "orthostream/geo/crs.hpp"

namespace orthostream::terrain
{

class TerrainError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

struct TerrainConfig
{
    std::string              dem_path;                     ///< raster readable by GDAL (GeoTIFF, VRT)
    std::string              dem_crs = "EPSG:4326+3855";   ///< its CRS with heights (Copernicus: EGM2008)
    std::string              proj_data_dir;                ///< directory of proj.db
    std::vector<std::string> grid_dirs;                    ///< directories of the transformation grids
};

class TerrainPrior
{
public:
    /// Opens the raster and the transformations. Throws TerrainError if the
    /// raster cannot be opened or is not north-up, or PROJ cannot create the
    /// transformations.
    explicit TerrainPrior(const TerrainConfig &config);
    ~TerrainPrior();

    TerrainPrior(const TerrainPrior &)            = delete;
    TerrainPrior &operator=(const TerrainPrior &) = delete;

    /// Reads the part of the model within radius_m (horizontally) of the
    /// point into memory, replacing the previous window. Throws TerrainError
    /// if the area does not intersect the raster or the transformation
    /// fails there.
    void load(const Eigen::Vector3d &centre_E, double radius_m);

    bool loaded() const;

    /// Horizontal distance of a point from the centre of the loaded window;
    /// infinite if nothing is loaded.
    double distanceFromWindowCentre(const Eigen::Vector3d &p_E) const;

    /// The terrain point at the horizontal position of p_E (in the model's
    /// CRS), with the model height, in ECEF. Nothing outside the window,
    /// where the model has no data, or where the transformation fails.
    std::optional<Eigen::Vector3d> surface(const Eigen::Vector3d &p_E) const;

    /// WGS 84 geodetic coordinates of an ECEF point.
    geo::Geodetic geodetic(const Eigen::Vector3d &p_E) const;

    /// Pixels of the loaded window.
    std::size_t windowPixels() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace orthostream::terrain

#endif  // ORTHOSTREAM_TERRAIN_TERRAIN_HPP
