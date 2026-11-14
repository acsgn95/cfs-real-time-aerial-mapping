// Terrain prior (terrain.hpp).

#include "orthostream/terrain/terrain.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <cpl_error.h>
#include <gdal.h>

#include "orthostream/geo/frames.hpp"

namespace orthostream::terrain
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

/// A window of the raster in memory; heights NaN where the raster has no data.
struct Window
{
    double             origin_x = 0.0;  ///< outer corner of the top-left pixel
    double             origin_y = 0.0;
    double             pixel_x  = 0.0;
    double             pixel_y  = 0.0;  ///< negative for north-up rasters
    int                width    = 0;
    int                height   = 0;
    std::vector<float> values;

    float at(int c, int r) const { return values[static_cast<std::size_t>(r) * width + c]; }

    /// Bilinear between pixel centres; nothing outside or next to no data.
    std::optional<double> sample(double x, double y) const
    {
        const double fc = (x - origin_x) / pixel_x - 0.5;
        const double fr = (y - origin_y) / pixel_y - 0.5;
        if (width < 1 || height < 1 || !(fc >= 0.0) || !(fr >= 0.0) || fc > width - 1 || fr > height - 1)
        {
            return std::nullopt;
        }
        const int    c  = std::min(static_cast<int>(fc), std::max(0, width - 2));
        const int    r  = std::min(static_cast<int>(fr), std::max(0, height - 2));
        const int    c1 = std::min(c + 1, width - 1);
        const int    r1 = std::min(r + 1, height - 1);
        const double tx = fc - c, ty = fr - r;
        const double w[4] = {(1 - tx) * (1 - ty), tx * (1 - ty), (1 - tx) * ty, tx * ty};
        const double v[4] = {at(c, r), at(c1, r), at(c, r1), at(c1, r1)};
        double       sum  = 0.0;
        for (int i = 0; i < 4; ++i)
        {
            if (w[i] > 0.0)
            {
                if (std::isnan(v[i]))
                {
                    return std::nullopt;
                }
                sum += w[i] * v[i];
            }
        }
        return sum;
    }
};

}  // namespace

struct TerrainPrior::Impl
{
    GDALDatasetH                         dataset = nullptr;
    double                               gt[6] {};
    int                                  width      = 0;
    int                                  height     = 0;
    bool                                 has_nodata = false;
    double                               nodata     = 0.0;
    std::unique_ptr<geo::CrsTransformer> to_dem;  ///< ECEF -> DEM CRS (x, y, height)
    std::unique_ptr<geo::GeodeticConverter> geodetic;
    Window                               window;
    bool                                 loaded = false;
    Eigen::Vector3d                      centre = Eigen::Vector3d::Zero();
    Eigen::Matrix3d                      R_L_E  = Eigen::Matrix3d::Identity();  ///< ENU at the window centre
};

TerrainPrior::TerrainPrior(const TerrainConfig &config) : impl_(std::make_unique<Impl>())
{
    Impl &r = *impl_;
    try
    {
        r.to_dem   = std::make_unique<geo::CrsTransformer>("EPSG:4978", config.dem_crs, config.proj_data_dir,
                                                         config.grid_dirs);
        r.geodetic = std::make_unique<geo::GeodeticConverter>(config.proj_data_dir);
    }
    catch (const geo::GeoError &e)
    {
        throw TerrainError(std::string("terrain: ") + e.what());
    }

    GDALAllRegister();
    CPLPushErrorHandler(CPLQuietErrorHandler);  // CRS warnings of the file are irrelevant here
    r.dataset = GDALOpen(config.dem_path.c_str(), GA_ReadOnly);
    CPLPopErrorHandler();
    if (r.dataset == nullptr)
    {
        throw TerrainError("terrain: cannot open " + config.dem_path);
    }
    if (GDALGetGeoTransform(r.dataset, r.gt) != CE_None || r.gt[2] != 0.0 || r.gt[4] != 0.0 || r.gt[1] <= 0.0 ||
        r.gt[5] >= 0.0)
    {
        GDALClose(r.dataset);
        r.dataset = nullptr;
        throw TerrainError("terrain: raster is not north-up: " + config.dem_path);
    }
    r.width      = GDALGetRasterXSize(r.dataset);
    r.height     = GDALGetRasterYSize(r.dataset);
    int has      = 0;
    r.nodata     = GDALGetRasterNoDataValue(GDALGetRasterBand(r.dataset, 1), &has);
    r.has_nodata = has != 0;
}

TerrainPrior::~TerrainPrior()
{
    if (impl_ && impl_->dataset != nullptr)
    {
        GDALClose(impl_->dataset);
    }
}

void TerrainPrior::load(const Eigen::Vector3d &centre_E, double radius_m)
{
    Impl &r = *impl_;
    if (!(radius_m > 0.0) || !centre_E.allFinite())
    {
        throw TerrainError("terrain: invalid window");
    }
    // Bounds in the DEM CRS of a ring of points around the centre, in the
    // local ENU frame at the centre.
    geo::Geodetic g;
    try
    {
        g = r.geodetic->ecefToGeodetic(centre_E);
    }
    catch (const geo::GeoError &e)
    {
        throw TerrainError(std::string("terrain: ") + e.what());
    }
    const Eigen::Matrix3d R_E_L = geo::rotationEcefFromEnu(g.latitude_rad, g.longitude_rad);
    double                x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
    for (int k = 0; k < 16; ++k)
    {
        const double a = 2.0 * kPi * k / 16.0;
        // The ring circumscribes the circle, so the box covers it.
        const double          s   = radius_m / std::cos(kPi / 16.0);
        const Eigen::Vector3d p_E = centre_E + R_E_L * Eigen::Vector3d(s * std::cos(a), s * std::sin(a), 0.0);
        try
        {
            const Eigen::Vector3d p = r.to_dem->forward(p_E);
            x0 = std::min(x0, p.x()), y0 = std::min(y0, p.y());
            x1 = std::max(x1, p.x()), y1 = std::max(y1, p.y());
        }
        catch (const geo::GeoError &e)
        {
            throw TerrainError(std::string("terrain: ") + e.what());
        }
    }

    const int col0 = std::max(0, static_cast<int>(std::floor((x0 - r.gt[0]) / r.gt[1])) - 1);
    const int col1 = std::min(r.width, static_cast<int>(std::ceil((x1 - r.gt[0]) / r.gt[1])) + 1);
    const int row0 = std::max(0, static_cast<int>(std::floor((y1 - r.gt[3]) / r.gt[5])) - 1);
    const int row1 = std::min(r.height, static_cast<int>(std::ceil((y0 - r.gt[3]) / r.gt[5])) + 1);
    if (col1 <= col0 || row1 <= row0)
    {
        throw TerrainError("terrain: the area is outside the elevation model");
    }
    Window w;
    w.width    = col1 - col0;
    w.height   = row1 - row0;
    w.pixel_x  = r.gt[1];
    w.pixel_y  = r.gt[5];
    w.origin_x = r.gt[0] + col0 * r.gt[1];
    w.origin_y = r.gt[3] + row0 * r.gt[5];
    w.values.resize(static_cast<std::size_t>(w.width) * w.height);
    if (GDALRasterIO(GDALGetRasterBand(r.dataset, 1), GF_Read, col0, row0, w.width, w.height, w.values.data(),
                     w.width, w.height, GDT_Float32, 0, 0) != CE_None)
    {
        throw TerrainError("terrain: raster read failed");
    }
    if (r.has_nodata)
    {
        const float nodata = static_cast<float>(r.nodata);
        for (float &v : w.values)
        {
            if (v == nodata)
            {
                v = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
    r.window = std::move(w);
    r.loaded = true;
    r.centre = centre_E;
    r.R_L_E  = R_E_L.transpose();
}

bool TerrainPrior::loaded() const
{
    return impl_->loaded;
}

double TerrainPrior::distanceFromWindowCentre(const Eigen::Vector3d &p_E) const
{
    if (!impl_->loaded)
    {
        return std::numeric_limits<double>::infinity();
    }
    return (impl_->R_L_E * (p_E - impl_->centre)).head<2>().norm();
}

std::optional<Eigen::Vector3d> TerrainPrior::surface(const Eigen::Vector3d &p_E) const
{
    const Impl &r = *impl_;
    if (!r.loaded)
    {
        return std::nullopt;
    }
    try
    {
        const Eigen::Vector3d       p = r.to_dem->forward(p_E);
        const std::optional<double> h = r.window.sample(p.x(), p.y());
        if (!h)
        {
            return std::nullopt;
        }
        return r.to_dem->inverse({p.x(), p.y(), *h});
    }
    catch (const geo::GeoError &)
    {
        return std::nullopt;
    }
}

geo::Geodetic TerrainPrior::geodetic(const Eigen::Vector3d &p_E) const
{
    try
    {
        return impl_->geodetic->ecefToGeodetic(p_E);
    }
    catch (const geo::GeoError &e)
    {
        throw TerrainError(std::string("terrain: ") + e.what());
    }
}

std::size_t TerrainPrior::windowPixels() const
{
    return impl_->window.values.size();
}

}  // namespace orthostream::terrain
