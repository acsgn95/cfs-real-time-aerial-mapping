#include "orthostream/map_products/writer.hpp"

#include <array>
#include <cstdio>
#include <filesystem>

#include <cpl_conv.h>
#include <cpl_string.h>
#include <gdal.h>
#include <gdal_utils.h>
#include <ogr_srs_api.h>

namespace orthostream::map_products
{
namespace
{

namespace fs = std::filesystem;

constexpr int kOffsetNodes = 9;  // per tile side, for the vertical datum offset

std::string compoundCrs(const std::string &projected, const std::string &vertical)
{
    if (vertical.empty())
    {
        return projected;
    }
    if (vertical.rfind("EPSG:", 0) != 0 || projected.rfind("EPSG:", 0) != 0)
    {
        throw MapProductsError("projected and vertical CRS must be given as EPSG:<code>");
    }
    return projected + "+" + vertical.substr(5);
}

}  // namespace

struct DsmWriter::Impl
{
    DsmWriterConfig                      config;
    const ProductFrame                  &frame;
    std::unique_ptr<geo::CrsTransformer> to_output;  // only with a vertical CRS
    std::string                          wkt;
    std::set<std::string>                written;

    Impl(const DsmWriterConfig &c, const ProductFrame &f) : config(c), frame(f) {}

    // Output height minus ellipsoidal height at a point of P.
    double verticalOffset(double e, double n, double h) const
    {
        if (!to_output)
        {
            return 0.0;
        }
        return to_output->forward(frame.toEcef({ e, n, h })).z() - h;
    }
};

DsmWriter::DsmWriter(const DsmWriterConfig &config, const ProductFrame &frame)
    : impl_(std::make_unique<Impl>(config, frame))
{
    if (config.directory.empty() || config.projected_crs.empty())
    {
        throw MapProductsError("DSM writer needs a directory and a CRS");
    }
    GDALAllRegister();
    // GDAL uses its own PROJ context: point it at the same proj.db.
    const char *paths[] = { config.proj_data_dir.c_str(), nullptr };
    OSRSetPROJSearchPaths(paths);

    const std::string         crs = compoundCrs(config.projected_crs, config.vertical_crs);
    OGRSpatialReferenceH      srs = OSRNewSpatialReference(nullptr);
    if (OSRSetFromUserInput(srs, crs.c_str()) != OGRERR_NONE)
    {
        OSRDestroySpatialReference(srs);
        throw MapProductsError("unknown output CRS " + crs);
    }
    char *wkt = nullptr;
    OSRExportToWkt(srs, &wkt);
    impl_->wkt = wkt;
    CPLFree(wkt);
    OSRDestroySpatialReference(srs);

    if (!config.vertical_crs.empty())
    {
        impl_->to_output =
            std::make_unique<geo::CrsTransformer>("EPSG:4978", crs, config.proj_data_dir, config.grid_dirs);
    }
    fs::create_directories(fs::path(config.directory) / "dsm");
}

DsmWriter::~DsmWriter() = default;

std::string DsmWriter::tileFileName(const TileId &id)
{
    char name[64];
    std::snprintf(name, sizeof(name), "dsm_%lld_%lld.tif", static_cast<long long>(id.ix),
                  static_cast<long long>(id.iy));
    return name;
}

std::string DsmWriter::write(const DsmTile &tile)
{
    const int    n  = tile.cells();
    const double cs = tile.cellSize();
    const double e0 = tile.originEasting();
    const double n0 = tile.originNorthing();
    const double h0 = tile.referenceHeight();

    // Vertical offset on a coarse grid of the tile; the geoid is smooth.
    std::array<double, kOffsetNodes * kOffsetNodes> offset {};
    const double                                    step = n * cs / (kOffsetNodes - 1);
    for (int j = 0; j < kOffsetNodes; ++j)
    {
        for (int i = 0; i < kOffsetNodes; ++i)
        {
            offset[static_cast<std::size_t>(j * kOffsetNodes + i)] = impl_->verticalOffset(e0 + i * step, n0 - j * step, h0);
        }
    }
    auto offset_at = [&](double fi, double fj)
    {
        const int    i  = std::min(static_cast<int>(fi), kOffsetNodes - 2);
        const int    j  = std::min(static_cast<int>(fj), kOffsetNodes - 2);
        const double ti = fi - i, tj = fj - j;
        auto         o  = [&](int a, int b) { return offset[static_cast<std::size_t>(b * kOffsetNodes + a)]; };
        return (1 - tj) * ((1 - ti) * o(i, j) + ti * o(i + 1, j)) + tj * ((1 - ti) * o(i, j + 1) + ti * o(i + 1, j + 1));
    };

    const std::size_t  cells = static_cast<std::size_t>(n) * n;
    std::vector<float> height(cells, kNoData), variance(cells, kNoData);
    for (int r = 0; r < n; ++r)
    {
        for (int c = 0; c < n; ++c)
        {
            if (const std::optional<double> h = tile.height(c, r))
            {
                const std::size_t k = tile.index(c, r);
                height[k]   = static_cast<float>(*h + offset_at((c + 0.5) * cs / step, (r + 0.5) * cs / step));
                variance[k] = static_cast<float>(*tile.variance(c, r));
            }
        }
    }

    const fs::path path = fs::path(impl_->config.directory) / "dsm" / tileFileName(tile.id());
    const fs::path tmp  = fs::path(path).concat(".tmp");
    GDALDriverH    drv  = GDALGetDriverByName("GTiff");
    char         **opts = nullptr;
    opts                = CSLSetNameValue(opts, "TILED", "YES");
    opts                = CSLSetNameValue(opts, "COMPRESS", "DEFLATE");
    opts                = CSLSetNameValue(opts, "PREDICTOR", "3");
    GDALDatasetH ds     = GDALCreate(drv, tmp.c_str(), n, n, 2, GDT_Float32, opts);
    CSLDestroy(opts);
    if (ds == nullptr)
    {
        throw MapProductsError("cannot create " + tmp.string());
    }
    double gt[6] = { e0, cs, 0.0, n0, 0.0, -cs };
    GDALSetGeoTransform(ds, gt);
    GDALSetProjection(ds, impl_->wkt.c_str());
    GDALSetMetadataItem(ds, "HEIGHTS",
                        impl_->config.vertical_crs.empty() ? "ellipsoidal (WGS 84)" : impl_->config.vertical_crs.c_str(),
                        nullptr);
    bool ok = true;
    for (int b = 1; b <= 2; ++b)
    {
        GDALRasterBandH band = GDALGetRasterBand(ds, b);
        GDALSetRasterNoDataValue(band, kNoData);
        GDALSetDescription(band, b == 1 ? "height" : "variance");
        float *data = b == 1 ? height.data() : variance.data();
        ok = ok && GDALRasterIO(band, GF_Write, 0, 0, n, n, data, n, n, GDT_Float32, 0, 0) == CE_None;
    }
    ok = GDALClose(ds) == CE_None && ok;
    if (!ok)
    {
        fs::remove(tmp);
        throw MapProductsError("writing " + path.string() + " failed");
    }
    fs::rename(tmp, path);
    impl_->written.insert(path.string());
    return path.string();
}

std::string DsmWriter::writeIndex()
{
    const fs::path           path = fs::path(impl_->config.directory) / "dsm.vrt";
    std::vector<const char *> sources;
    for (const std::string &s : impl_->written)
    {
        sources.push_back(s.c_str());
    }
    if (sources.empty())
    {
        throw MapProductsError("no DSM tiles written");
    }
    const fs::path tmp = fs::path(path).concat(".tmp");
    int            usage_error = 0;
    GDALDatasetH   vrt = GDALBuildVRT(tmp.c_str(), static_cast<int>(sources.size()), nullptr, sources.data(), nullptr,
                                      &usage_error);
    if (vrt == nullptr || usage_error != 0)
    {
        throw MapProductsError("building the DSM index failed");
    }
    GDALClose(vrt);
    fs::rename(tmp, path);
    return path.string();
}

}  // namespace orthostream::map_products
