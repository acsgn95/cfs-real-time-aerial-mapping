#include "raster.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <cpl_error.h>
#include <gdal.h>

namespace orthostream::bench
{

struct Raster::Impl
{
    GDALDatasetH    dataset = nullptr;
    GDALRasterBandH band    = nullptr;
    double          gt[6] {};
    int             width  = 0;
    int             height = 0;
    bool            has_nodata = false;
    double          nodata     = 0.0;
};

Raster::Raster(const std::string &path) : impl_(std::make_unique<Impl>())
{
    GDALAllRegister();
    CPLPushErrorHandler(CPLQuietErrorHandler);  // CRS warnings of the file are irrelevant here
    impl_->dataset = GDALOpen(path.c_str(), GA_ReadOnly);
    CPLPopErrorHandler();
    if (impl_->dataset == nullptr)
    {
        throw std::runtime_error("cannot open raster " + path);
    }
    if (GDALGetGeoTransform(impl_->dataset, impl_->gt) != CE_None || impl_->gt[2] != 0.0 || impl_->gt[4] != 0.0)
    {
        GDALClose(impl_->dataset);
        throw std::runtime_error("raster is not north-up: " + path);
    }
    impl_->band   = GDALGetRasterBand(impl_->dataset, 1);
    impl_->width  = GDALGetRasterXSize(impl_->dataset);
    impl_->height = GDALGetRasterYSize(impl_->dataset);
    int has       = 0;
    impl_->nodata = GDALGetRasterNoDataValue(impl_->band, &has);
    impl_->has_nodata = has != 0;
}

Raster::~Raster()
{
    if (impl_ && impl_->dataset != nullptr)
    {
        GDALClose(impl_->dataset);
    }
}

RasterWindow Raster::read(double x_min, double y_min, double x_max, double y_max, int decimation) const
{
    const Impl &r = *impl_;
    if (decimation < 1)
    {
        throw std::runtime_error("invalid decimation");
    }
    const int col0 = std::max(0, static_cast<int>(std::floor((x_min - r.gt[0]) / r.gt[1])));
    const int col1 = std::min(r.width, static_cast<int>(std::ceil((x_max - r.gt[0]) / r.gt[1])));
    const int row0 = std::max(0, static_cast<int>(std::floor((y_max - r.gt[3]) / r.gt[5])));
    const int row1 = std::min(r.height, static_cast<int>(std::ceil((y_min - r.gt[3]) / r.gt[5])));
    if (col1 <= col0 || row1 <= row0)
    {
        throw std::runtime_error("window outside the raster");
    }
    RasterWindow w;
    w.width    = std::max(1, (col1 - col0) / decimation);
    w.height   = std::max(1, (row1 - row0) / decimation);
    w.pixel_x  = r.gt[1] * (col1 - col0) / w.width;
    w.pixel_y  = r.gt[5] * (row1 - row0) / w.height;
    w.origin_x = r.gt[0] + col0 * r.gt[1];
    w.origin_y = r.gt[3] + row0 * r.gt[5];
    w.values.resize(static_cast<std::size_t>(w.width) * w.height);
    if (GDALRasterIO(r.band, GF_Read, col0, row0, col1 - col0, row1 - row0, w.values.data(), w.width, w.height,
                     GDT_Float32, 0, 0)
        != CE_None)
    {
        throw std::runtime_error("raster read failed");
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
    return w;
}

RasterWindow Raster::readAll(int decimation) const
{
    const Impl &r = *impl_;
    return read(r.gt[0], r.gt[3] + r.height * r.gt[5], r.gt[0] + r.width * r.gt[1], r.gt[3], decimation);
}

std::optional<double> RasterWindow::sample(double x, double y) const
{
    const double fc = (x - origin_x) / pixel_x - 0.5;
    const double fr = (y - origin_y) / pixel_y - 0.5;
    if (!(fc >= 0.0) || !(fr >= 0.0) || fc > width - 1 || fr > height - 1)
    {
        return std::nullopt;
    }
    const int    c  = std::min(static_cast<int>(fc), width - 2 < 0 ? 0 : width - 2);
    const int    r  = std::min(static_cast<int>(fr), height - 2 < 0 ? 0 : height - 2);
    const int    c1 = std::min(c + 1, width - 1);
    const int    r1 = std::min(r + 1, height - 1);
    const double tx = fc - c, ty = fr - r;
    // Neighbours with zero weight (on the last row or column) may be missing.
    const double w[4] = { (1 - tx) * (1 - ty), tx * (1 - ty), (1 - tx) * ty, tx * ty };
    const double v[4] = { at(c, r), at(c1, r), at(c, r1), at(c1, r1) };
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

}  // namespace orthostream::bench
