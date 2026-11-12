#include "orthostream/map_products/writer.hpp"

#include <array>
#include <cstdio>
#include <filesystem>
#include <utility>

#include <cpl_conv.h>
#include <cpl_string.h>
#include <gdal.h>
#include <gdal_utils.h>
#include <ogr_srs_api.h>

#include "vertical_datum.hpp"

namespace orthostream::map_products
{
namespace
{

namespace fs = std::filesystem;

// Registers GDAL, points its PROJ context at proj.db and returns the WKT
// of a CRS.
std::string prepareGdal(const std::string &crs, const std::string &proj_data_dir)
{
    GDALAllRegister();
    const char *paths[] = { proj_data_dir.c_str(), nullptr };
    OSRSetPROJSearchPaths(paths);
    OGRSpatialReferenceH srs = OSRNewSpatialReference(nullptr);
    if (OSRSetFromUserInput(srs, crs.c_str()) != OGRERR_NONE)
    {
        OSRDestroySpatialReference(srs);
        throw MapProductsError("unknown output CRS " + crs);
    }
    char *wkt = nullptr;
    OSRExportToWkt(srs, &wkt);
    std::string result = wkt;
    CPLFree(wkt);
    OSRDestroySpatialReference(srs);
    return result;
}

struct TiffBand
{
    const void *data;
    const char *description;
};

struct TiffSpec
{
    int                                              size = 0;  // square
    GDALDataType                                     type = GDT_Float32;
    std::array<double, 6>                            geotransform {};
    std::vector<std::pair<const char *, const char *>> options;
    std::vector<std::pair<const char *, const char *>> metadata;
    bool                                             has_nodata = false;
    double                                           nodata     = 0.0;
};

// Writes a GeoTIFF under a temporary name and renames it.
void writeTiff(const fs::path &path, const std::string &wkt, const TiffSpec &spec, const std::vector<TiffBand> &bands)
{
    const fs::path tmp  = fs::path(path).concat(".tmp");
    char         **opts = nullptr;
    for (const auto &[k, v] : spec.options)
    {
        opts = CSLSetNameValue(opts, k, v);
    }
    GDALDatasetH ds = GDALCreate(GDALGetDriverByName("GTiff"), tmp.c_str(), spec.size, spec.size,
                                 static_cast<int>(bands.size()), spec.type, opts);
    CSLDestroy(opts);
    if (ds == nullptr)
    {
        throw MapProductsError("cannot create " + tmp.string());
    }
    std::array<double, 6> gt = spec.geotransform;
    GDALSetGeoTransform(ds, gt.data());
    GDALSetProjection(ds, wkt.c_str());
    for (const auto &[k, v] : spec.metadata)
    {
        GDALSetMetadataItem(ds, k, v, nullptr);
    }
    bool ok = true;
    for (std::size_t b = 0; b < bands.size(); ++b)
    {
        GDALRasterBandH band = GDALGetRasterBand(ds, static_cast<int>(b + 1));
        if (spec.has_nodata)
        {
            GDALSetRasterNoDataValue(band, spec.nodata);
        }
        GDALSetDescription(band, bands[b].description);
        ok = ok
             && GDALRasterIO(band, GF_Write, 0, 0, spec.size, spec.size, const_cast<void *>(bands[b].data), spec.size,
                             spec.size, spec.type, 0, 0)
                    == CE_None;
    }
    ok = GDALClose(ds) == CE_None && ok;
    if (!ok)
    {
        fs::remove(tmp);
        throw MapProductsError("writing " + path.string() + " failed");
    }
    fs::rename(tmp, path);
}

std::string buildIndex(const fs::path &path, const std::set<std::string> &files)
{
    std::vector<const char *> sources;
    for (const std::string &s : files)
    {
        sources.push_back(s.c_str());
    }
    if (sources.empty())
    {
        throw MapProductsError("no tiles written for " + path.string());
    }
    const fs::path tmp         = fs::path(path).concat(".tmp");
    int            usage_error = 0;
    GDALDatasetH   vrt =
        GDALBuildVRT(tmp.c_str(), static_cast<int>(sources.size()), nullptr, sources.data(), nullptr, &usage_error);
    if (vrt == nullptr || usage_error != 0)
    {
        throw MapProductsError("building " + path.string() + " failed");
    }
    GDALClose(vrt);
    fs::rename(tmp, path);
    return path.string();
}

std::string tileName(const char *prefix, const TileId &id)
{
    char name[64];
    std::snprintf(name, sizeof(name), "%s_%lld_%lld.tif", prefix, static_cast<long long>(id.ix),
                  static_cast<long long>(id.iy));
    return name;
}

}  // namespace

// ---------------------------------------------------------------- DSM

struct DsmWriter::Impl
{
    DsmWriterConfig       config;
    detail::VerticalDatum datum;
    std::string           wkt;
    std::set<std::string> written;

    Impl(const DsmWriterConfig &c, const ProductFrame &f)
        : config(c), datum(f, c.projected_crs, c.vertical_crs, c.proj_data_dir, c.grid_dirs)
    {
    }
};

DsmWriter::DsmWriter(const DsmWriterConfig &config, const ProductFrame &frame)
    : impl_(std::make_unique<Impl>(config, frame))
{
    if (config.directory.empty() || config.projected_crs.empty())
    {
        throw MapProductsError("DSM writer needs a directory and a CRS");
    }
    impl_->wkt = prepareGdal(impl_->datum.crs(), config.proj_data_dir);
    fs::create_directories(fs::path(config.directory) / "dsm");
}

DsmWriter::~DsmWriter() = default;

std::string DsmWriter::tileFileName(const TileId &id)
{
    return tileName("dsm", id);
}

std::string DsmWriter::write(const DsmTile &tile)
{
    const int    n  = tile.cells();
    const double cs = tile.cellSize();
    const double e0 = tile.originEasting();
    const double n0 = tile.originNorthing();
    const double h0 = tile.referenceHeight();

    const detail::OffsetGrid offset = impl_->datum.offsets(e0, n0, n * cs, h0);

    const std::size_t  cells = static_cast<std::size_t>(n) * n;
    std::vector<float> height(cells, kNoData), variance(cells, kNoData);
    for (int r = 0; r < n; ++r)
    {
        for (int c = 0; c < n; ++c)
        {
            if (const std::optional<double> h = tile.height(c, r))
            {
                const std::size_t k = tile.index(c, r);
                height[k]   = static_cast<float>(*h + offset.at(e0 + (c + 0.5) * cs, n0 - (r + 0.5) * cs));
                variance[k] = static_cast<float>(*tile.variance(c, r));
            }
        }
    }

    TiffSpec spec;
    spec.size         = n;
    spec.type         = GDT_Float32;
    spec.geotransform = { e0, cs, 0.0, n0, 0.0, -cs };
    spec.options      = { { "TILED", "YES" }, { "COMPRESS", "DEFLATE" }, { "PREDICTOR", "3" } };
    spec.metadata     = { { "HEIGHTS", impl_->config.vertical_crs.empty() ? "ellipsoidal (WGS 84)"
                                                                          : impl_->config.vertical_crs.c_str() } };
    spec.has_nodata   = true;
    spec.nodata       = kNoData;
    const fs::path path = fs::path(impl_->config.directory) / "dsm" / tileFileName(tile.id());
    writeTiff(path, impl_->wkt, spec, { { height.data(), "height" }, { variance.data(), "variance" } });
    impl_->written.insert(path.string());
    return path.string();
}

std::string DsmWriter::writeIndex()
{
    return buildIndex(fs::path(impl_->config.directory) / "dsm.vrt", impl_->written);
}

// ---------------------------------------------------------------- orthomosaic

struct OrthoWriter::Impl
{
    OrthoWriterConfig     config;
    std::string           wkt;
    std::set<std::string> written;
};

OrthoWriter::OrthoWriter(const OrthoWriterConfig &config) : impl_(std::make_unique<Impl>())
{
    if (config.directory.empty() || config.projected_crs.empty())
    {
        throw MapProductsError("orthomosaic writer needs a directory and a CRS");
    }
    impl_->config = config;
    impl_->wkt    = prepareGdal(config.projected_crs, config.proj_data_dir);
    fs::create_directories(fs::path(config.directory) / "ortho");
}

OrthoWriter::~OrthoWriter() = default;

std::string OrthoWriter::tileFileName(const TileId &id)
{
    return tileName("ortho", id);
}

std::string OrthoWriter::write(const OrthoTile &tile)
{
    const int                 n = tile.pixels();
    const std::size_t         count = static_cast<std::size_t>(n) * n;
    std::vector<std::uint8_t> planes(4 * count);  // band-sequential R, G, B, alpha
    for (std::size_t i = 0; i < count; ++i)
    {
        const bool filled     = tile.angle(static_cast<int>(i % n), static_cast<int>(i / n)) != OrthoTile::kEmpty;
        planes[i]             = tile.rgb()[3 * i];
        planes[count + i]     = tile.rgb()[3 * i + 1];
        planes[2 * count + i] = tile.rgb()[3 * i + 2];
        planes[3 * count + i] = filled ? 255 : 0;
    }
    TiffSpec spec;
    spec.size         = n;
    spec.type         = GDT_Byte;
    spec.geotransform = { tile.originEasting(), tile.pixelSize(), 0.0, tile.originNorthing(), 0.0, -tile.pixelSize() };
    spec.options      = { { "TILED", "YES" },      { "COMPRESS", "DEFLATE" }, { "PREDICTOR", "2" },
                          { "PHOTOMETRIC", "RGB" }, { "ALPHA", "YES" } };
    const fs::path path = fs::path(impl_->config.directory) / "ortho" / tileFileName(tile.id());
    writeTiff(path, impl_->wkt, spec,
              { { planes.data(), "red" },
                { planes.data() + count, "green" },
                { planes.data() + 2 * count, "blue" },
                { planes.data() + 3 * count, "alpha" } });
    impl_->written.insert(path.string());
    return path.string();
}

std::string OrthoWriter::writeIndex()
{
    return buildIndex(fs::path(impl_->config.directory) / "ortho.vrt", impl_->written);
}

}  // namespace orthostream::map_products
