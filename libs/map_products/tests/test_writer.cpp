#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include <gdal.h>
#include <gtest/gtest.h>
#include <ogr_srs_api.h>

#include "orthostream/map_products/writer.hpp"
#include "test_support.hpp"

namespace orthostream::map_products
{
namespace
{

namespace fs = std::filesystem;
using test::dorfberg;

struct Raster
{
    int                 width = 0, height = 0, bands = 0;
    double              gt[6] {};
    std::string         wkt;
    double              nodata = 0.0;
    std::vector<float>  band1;
};

Raster readRaster(const std::string &path)
{
    Raster       r;
    GDALDatasetH ds = GDALOpen(path.c_str(), GA_ReadOnly);
    EXPECT_NE(ds, nullptr) << path;
    if (ds == nullptr)
    {
        return r;
    }
    r.width  = GDALGetRasterXSize(ds);
    r.height = GDALGetRasterYSize(ds);
    r.bands  = GDALGetRasterCount(ds);
    GDALGetGeoTransform(ds, r.gt);
    r.wkt                = GDALGetProjectionRef(ds);
    GDALRasterBandH band = GDALGetRasterBand(ds, 1);
    r.nodata             = GDALGetRasterNoDataValue(band, nullptr);
    r.band1.resize(static_cast<std::size_t>(r.width) * r.height);
    EXPECT_EQ(GDALRasterIO(band, GF_Read, 0, 0, r.width, r.height, r.band1.data(), r.width, r.height, GDT_Float32, 0, 0),
              CE_None);
    GDALClose(ds);
    return r;
}

std::string authorityCode(const std::string &wkt, const char *node)
{
    OGRSpatialReferenceH srs = OSRNewSpatialReference(wkt.c_str());
    const char          *c   = OSRGetAuthorityCode(srs, node);
    std::string          code = c ? c : "";
    OSRDestroySpatialReference(srs);
    return code;
}

fs::path outputDir(const std::string &name)
{
    const fs::path dir = fs::path(::testing::TempDir()) / ("ost_map_products_" + name);
    fs::remove_all(dir);
    return dir;
}

// A tile around the Dorfberg site with a sloped surface.
DsmTile slopedTile(const ProductFrame &frame, const geo::GeodeticConverter &conv, const TileId &offset,
                   const DsmFusionConfig &config)
{
    const Eigen::Vector3d p = frame.fromEcef(conv.geodeticToEcef(dorfberg(2000.0)));
    const double          T = config.grid.cell_size * config.grid.tile_cells;
    const TileId          id { static_cast<std::int64_t>(std::floor(p.x() / T)) + offset.ix,
                      static_cast<std::int64_t>(std::floor(p.y() / T)) + offset.iy };
    DsmTile tile(id, config.grid, 2000.0);
    for (int r = 0; r < tile.cells(); ++r)
    {
        for (int c = 0; c < tile.cells(); c += 2)  // every other column stays empty
        {
            tile.add(c, r, 2000.0 + 0.5 * c - 0.25 * r, 1.0f, config);
        }
    }
    return tile;
}

TEST(DsmWriter, EllipsoidalTileReadsBack)
{
    const geo::GeodeticConverter conv(OST_MP_TEST_PROJ_DATA_DIR);
    const ProductFrame           frame("EPSG:32632", OST_MP_TEST_PROJ_DATA_DIR);
    DsmFusionConfig              config;
    config.grid.cell_size  = 0.5;
    config.grid.tile_cells = 64;
    const DsmTile tile     = slopedTile(frame, conv, { 0, 0 }, config);

    const fs::path dir = outputDir("ellipsoidal");
    DsmWriter      writer({ dir.string(), "EPSG:32632", "", OST_MP_TEST_PROJ_DATA_DIR, {} }, frame);
    const std::string path = writer.write(tile);
    EXPECT_EQ(fs::path(path).filename(), DsmWriter::tileFileName(tile.id()));
    EXPECT_FALSE(fs::exists(path + ".tmp"));

    const Raster r = readRaster(path);
    EXPECT_EQ(r.width, 64);
    EXPECT_EQ(r.height, 64);
    EXPECT_EQ(r.bands, 2);
    EXPECT_DOUBLE_EQ(r.gt[0], tile.originEasting());
    EXPECT_DOUBLE_EQ(r.gt[3], tile.originNorthing());
    EXPECT_DOUBLE_EQ(r.gt[1], 0.5);
    EXPECT_DOUBLE_EQ(r.gt[5], -0.5);
    EXPECT_EQ(authorityCode(r.wkt, "PROJCS"), "32632");
    EXPECT_EQ(r.nodata, DsmWriter::kNoData);
    EXPECT_NEAR(r.band1[static_cast<std::size_t>(10 * 64 + 4)], 2000.0 + 2.0 - 2.5, 1e-3);
    EXPECT_EQ(r.band1[static_cast<std::size_t>(10 * 64 + 5)], DsmWriter::kNoData);
}

TEST(DsmWriter, OrthometricHeightsAndIndex)
{
#ifndef OST_MP_TEST_GRID_DIR
    GTEST_SKIP() << "needs the LHN95 geoid grid (scripts/fetch_proj_grids.sh)";
#else
    const geo::GeodeticConverter conv(OST_MP_TEST_PROJ_DATA_DIR);
    const ProductFrame           frame("EPSG:2056", OST_MP_TEST_PROJ_DATA_DIR, { OST_MP_TEST_GRID_DIR });
    DsmFusionConfig              config;
    config.grid.cell_size  = 2.0;
    config.grid.tile_cells = 64;  // 128 m tiles
    const DsmTile a        = slopedTile(frame, conv, { 0, 0 }, config);
    const DsmTile b        = slopedTile(frame, conv, { 1, 0 }, config);

    const fs::path dir = outputDir("lhn95");
    DsmWriter      writer({ dir.string(), "EPSG:2056", "EPSG:5729", OST_MP_TEST_PROJ_DATA_DIR, { OST_MP_TEST_GRID_DIR } },
                          frame);
    const Raster   r = readRaster(writer.write(a));
    writer.write(b);
    EXPECT_EQ(authorityCode(r.wkt, "PROJCS"), "2056");
    EXPECT_EQ(authorityCode(r.wkt, "VERT_CS"), "5729");

    // Every written height equals the exact LHN95 height of the cell.
    const geo::CrsTransformer to_lhn95("EPSG:4978", "EPSG:2056+5729", OST_MP_TEST_PROJ_DATA_DIR, { OST_MP_TEST_GRID_DIR });
    double                    max_err = 0.0, offset = 0.0;
    for (int row = 0; row < 64; row += 7)
    {
        for (int col = 0; col < 64; col += 6)
        {
            const double e     = a.originEasting() + (col + 0.5) * 2.0;
            const double n     = a.originNorthing() - (row + 0.5) * 2.0;
            const double h     = *a.height(col, row);
            const double exact = to_lhn95.forward(frame.toEcef({ e, n, h })).z();
            max_err            = std::max(max_err, std::abs(r.band1[static_cast<std::size_t>(row * 64 + col)] - exact));
            offset             = exact - h;
        }
    }
    EXPECT_LT(max_err, 0.002);
    EXPECT_NEAR(offset, -50.0, 5.0);  // geoid height at the site, about 50 m

    const Raster index = readRaster(writer.writeIndex());
    EXPECT_EQ(index.width, 128);
    EXPECT_EQ(index.height, 64);
    EXPECT_DOUBLE_EQ(index.gt[0], a.originEasting());
#endif
}

TEST(DsmWriter, RejectsAnUnknownCrs)
{
    const ProductFrame frame("EPSG:32632", OST_MP_TEST_PROJ_DATA_DIR);
    const fs::path     dir = outputDir("bad");
    EXPECT_THROW(DsmWriter({ dir.string(), "EPSG:32632", "LHN95", OST_MP_TEST_PROJ_DATA_DIR, {} }, frame),
                 MapProductsError);
    EXPECT_THROW(DsmWriter({ dir.string(), "EPSG:999999", "", OST_MP_TEST_PROJ_DATA_DIR, {} }, frame), MapProductsError);
}

}  // namespace
}  // namespace orthostream::map_products
