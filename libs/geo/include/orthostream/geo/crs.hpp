// Coordinate reference system transformations through PROJ (ADR-004
// section 7).
//
// Coordinates are passed in PROJ's "visualisation" axis order: longitude or
// easting first. Geographic coordinates are in degrees at this boundary, as
// PROJ and external formats use them; ecefToGeodetic and geodeticToEcef use
// radians, the internal unit.

#ifndef ORTHOSTREAM_GEO_CRS_HPP
#define ORTHOSTREAM_GEO_CRS_HPP

#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

namespace orthostream::geo
{

class GeoError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

struct Geodetic
{
    double latitude_rad  = 0.0;
    double longitude_rad = 0.0;
    double height_m      = 0.0;  ///< ellipsoidal height
};

/// Transformation between two coordinate reference systems. Network access is
/// disabled; grids must be present in the data directory. An instance is not
/// thread-safe: each task uses its own.
class CrsTransformer
{
public:
    /// source and target are any CRS definitions PROJ accepts, for example
    /// "EPSG:4978". proj_data_dir is the directory holding proj.db; PROJ's
    /// built-in default directory is never used. Throws GeoError if the
    /// directory has no proj.db or PROJ cannot create the transformation.
    CrsTransformer(const std::string &source, const std::string &target, const std::string &proj_data_dir);
    ~CrsTransformer();

    CrsTransformer(const CrsTransformer &)            = delete;
    CrsTransformer &operator=(const CrsTransformer &) = delete;
    CrsTransformer(CrsTransformer &&) noexcept;
    CrsTransformer &operator=(CrsTransformer &&) noexcept;

    /// Throws GeoError if PROJ reports an error for this point.
    Eigen::Vector3d forward(const Eigen::Vector3d &p) const;
    Eigen::Vector3d inverse(const Eigen::Vector3d &p) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// WGS 84 ECEF (EPSG:4978) to and from geodetic coordinates (EPSG:4979).
class GeodeticConverter
{
public:
    explicit GeodeticConverter(const std::string &proj_data_dir);

    Geodetic        ecefToGeodetic(const Eigen::Vector3d &p_E) const;
    Eigen::Vector3d geodeticToEcef(const Geodetic &g) const;

private:
    CrsTransformer transformer_;
};

}  // namespace orthostream::geo

#endif  // ORTHOSTREAM_GEO_CRS_HPP
