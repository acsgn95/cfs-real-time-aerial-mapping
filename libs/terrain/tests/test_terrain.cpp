// Tests of the terrain prior (ADR-006 section 2).

#include <cmath>
#include <filesystem>
#include <string>

#include <gdal.h>
#include <gtest/gtest.h>

#include "orthostream/terrain/terrain.hpp"

namespace fs = std::filesystem;
namespace tr = orthostream::terrain;
namespace geo = orthostream::geo;

namespace
{

constexpr double kDeg = 3.14159265358979323846 / 180.0;

/// A 0.1 x 0.1 degree DEM at 0.001 degree pixels around (46.8 N, 9.8 E):
/// height = 1500 + 1000 * (lon - 9.75) + 500 * (lat - 46.75) metres, with
/// a no-data hole near (46.82, 9.82).
std::string writeDem(const fs::path &dir)
{
    GDALAllRegister();
    const std::string path = (dir / "dem.tif").string();
    GDALDatasetH      ds   = GDALCreate(GDALGetDriverByName("GTiff"), path.c_str(), 100, 100, 1, GDT_Float32, nullptr);
    double            gt[6] = {9.75, 0.001, 0.0, 46.85, 0.0, -0.001};
    GDALSetGeoTransform(ds, gt);
    GDALRasterBandH band = GDALGetRasterBand(ds, 1);
    GDALSetRasterNoDataValue(band, -32767.0);
    std::vector<float> v(100 * 100);
    for (int r = 0; r < 100; ++r)
    {
        for (int c = 0; c < 100; ++c)
        {
            const double lon = 9.75 + (c + 0.5) * 0.001;
            const double lat = 46.85 - (r + 0.5) * 0.001;
            v[static_cast<std::size_t>(r) * 100 + c] =
                static_cast<float>(1500.0 + 1000.0 * (lon - 9.75) + 500.0 * (lat - 46.75));
        }
    }
    for (int r = 28; r < 32; ++r)
    {
        for (int c = 68; c < 72; ++c)
        {
            v[static_cast<std::size_t>(r) * 100 + c] = -32767.0f;
        }
    }
    EXPECT_EQ(GDALRasterIO(band, GF_Write, 0, 0, 100, 100, v.data(), 100, 100, GDT_Float32, 0, 0), CE_None);
    GDALClose(ds);
    return path;
}

class TerrainTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        dir_ = fs::temp_directory_path() / ("ost_terrain_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
        fs::create_directories(dir_);
        config_.dem_path      = writeDem(dir_);
        config_.dem_crs       = "EPSG:4979";  // ellipsoidal heights: no geoid grid needed
        config_.proj_data_dir = OST_TERRAIN_TEST_PROJ_DATA_DIR;
    }
    void TearDown() override { fs::remove_all(dir_); }

    Eigen::Vector3d ecef(double lat, double lon, double h) const
    {
        const geo::GeodeticConverter g(OST_TERRAIN_TEST_PROJ_DATA_DIR);
        return g.geodeticToEcef({lat * kDeg, lon * kDeg, h});
    }

    fs::path          dir_;
    tr::TerrainConfig config_;
};

}  // namespace

TEST_F(TerrainTest, SamplesTheModelBelowAPoint)
{
    tr::TerrainPrior t(config_);
    EXPECT_FALSE(t.loaded());
    EXPECT_FALSE(t.surface(ecef(46.8, 9.8, 2500.0)).has_value());

    t.load(ecef(46.8, 9.8, 2500.0), 2000.0);
    ASSERT_TRUE(t.loaded());
    EXPECT_GT(t.windowPixels(), 0u);
    const auto s = t.surface(ecef(46.8, 9.8, 2500.0));
    ASSERT_TRUE(s.has_value());
    const geo::Geodetic g = t.geodetic(*s);
    EXPECT_NEAR(g.latitude_rad / kDeg, 46.8, 1e-9);
    EXPECT_NEAR(g.longitude_rad / kDeg, 9.8, 1e-9);
    EXPECT_NEAR(g.height_m, 1500.0 + 50.0 + 25.0, 1e-3);  // linear: bilinear is exact
    EXPECT_NEAR(t.distanceFromWindowCentre(ecef(46.8, 9.8, 100.0)), 0.0, 1e-6);
}

TEST_F(TerrainTest, NothingOutsideTheWindowOrWithoutData)
{
    tr::TerrainPrior t(config_);
    t.load(ecef(46.8, 9.8, 2500.0), 1000.0);
    EXPECT_FALSE(t.surface(ecef(46.82, 9.82, 2500.0)).has_value());  // no-data hole, also outside 1 km
    t.load(ecef(46.82, 9.82, 2500.0), 1000.0);
    EXPECT_FALSE(t.surface(ecef(46.8201, 9.8201, 2500.0)).has_value());  // in the hole
    EXPECT_TRUE(t.surface(ecef(46.81, 9.81, 2500.0)).has_value());
    EXPECT_FALSE(t.surface(ecef(46.79, 9.79, 2500.0)).has_value());      // outside the window
    EXPECT_NEAR(t.distanceFromWindowCentre(ecef(46.82, 9.82 + 1000.0 / (111320.0 * std::cos(46.82 * kDeg)), 0.0)),
                1000.0, 2.0);
}

TEST_F(TerrainTest, RejectsAnAreaOutsideTheModel)
{
    tr::TerrainPrior t(config_);
    EXPECT_THROW(t.load(ecef(40.0, 9.8, 2500.0), 2000.0), tr::TerrainError);
    config_.dem_path = (dir_ / "missing.tif").string();
    EXPECT_THROW(tr::TerrainPrior bad(config_), tr::TerrainError);
}

// The Copernicus mosaic of the nominal dataset with the EGM2008 grid, when
// both are present (scripts/fetch_proj_grids.sh, tools/dataset/fetch_dem.py).
TEST(TerrainCopernicus, GeoidIsApplied)
{
    const std::string dem  = OST_TERRAIN_TEST_REPO "/data/raw/dem/cop30/dorfberg.vrt";
    const std::string grid = OST_TERRAIN_TEST_REPO "/build-deps/proj-grids";
    if (!fs::exists(dem) || !fs::exists(grid + "/us_nga_egm08_25.tif"))
    {
        GTEST_SKIP() << "Copernicus mosaic or EGM2008 grid not present";
    }
    tr::TerrainConfig c;
    c.dem_path      = dem;
    c.proj_data_dir = OST_TERRAIN_TEST_PROJ_DATA_DIR;
    c.grid_dirs     = {grid};
    tr::TerrainPrior        t(c);
    const geo::GeodeticConverter g(OST_TERRAIN_TEST_PROJ_DATA_DIR);
    const Eigen::Vector3d   p = g.geodeticToEcef({46.82 * kDeg, 9.84 * kDeg, 2500.0});
    t.load(p, 5000.0);
    const auto s = t.surface(p);
    ASSERT_TRUE(s.has_value());
    // Orthometric height from the raster directly.
    GDALAllRegister();
    GDALDatasetH ds = GDALOpen(dem.c_str(), GA_ReadOnly);
    ASSERT_NE(ds, nullptr);
    double gt[6];
    GDALGetGeoTransform(ds, gt);
    float     H   = 0.0f;
    const int col = static_cast<int>((9.84 - gt[0]) / gt[1] - 0.5);
    const int row = static_cast<int>((46.82 - gt[3]) / gt[5] - 0.5);
    ASSERT_EQ(GDALRasterIO(GDALGetRasterBand(ds, 1), GF_Read, col, row, 1, 1, &H, 1, 1, GDT_Float32, 0, 0), CE_None);
    GDALClose(ds);
    // EGM2008 undulation at Davos is about 49 m (ADR-006: -50 m without it).
    const double N = t.geodetic(*s).height_m - H;
    EXPECT_GT(N, 45.0);
    EXPECT_LT(N, 53.0);
}
