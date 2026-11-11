// Windows of a single-band, north-up GeoTIFF read through GDAL.

#ifndef OST_BENCH_RASTER_HPP
#define OST_BENCH_RASTER_HPP

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace orthostream::bench
{

struct RasterWindow
{
    double             origin_x = 0.0;  ///< outer corner of the top-left pixel
    double             origin_y = 0.0;
    double             pixel_x  = 0.0;  ///< pixel size; pixel_y < 0 for north-up rasters
    double             pixel_y  = 0.0;
    int                width    = 0;
    int                height   = 0;
    std::vector<float> values;  ///< row-major; NaN where the raster has no data

    /// Bilinear interpolation between pixel centres; nothing outside the
    /// window or where one of the four neighbours has no data.
    std::optional<double> sample(double x, double y) const;

    float at(int col, int row) const { return values[static_cast<std::size_t>(row) * width + col]; }
    double centreX(int col) const { return origin_x + (col + 0.5) * pixel_x; }
    double centreY(int row) const { return origin_y + (row + 0.5) * pixel_y; }
};

class Raster
{
public:
    explicit Raster(const std::string &path);
    ~Raster();
    Raster(const Raster &)            = delete;
    Raster &operator=(const Raster &) = delete;

    /// Reads the part of the raster that covers the given bounds, every
    /// decimation-th pixel (GDAL uses overviews where they exist). Throws
    /// std::runtime_error if the bounds do not intersect the raster.
    RasterWindow read(double x_min, double y_min, double x_max, double y_max, int decimation = 1) const;

    /// The whole raster, decimated.
    RasterWindow readAll(int decimation) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace orthostream::bench

#endif  // OST_BENCH_RASTER_HPP
